use embassy_time::Duration;
use esp_hal::Async;
use esp_hal::gpio::{AnyPin, Output};
use esp_hal::time::Rate;
use esp_hal::uart::{
    Config as UartConfig, DataBits, Instance as UartInstance, Parity, RxConfig, StopBits, Uart,
};

use crate::tmc2209_helper::{
    constrain, default_map, send_and_await_read, send_write, send_write_request,
};

use tmc2209::data::{MicroStepResolution, StandstillMode};
use tmc2209::reg::{self, Map, WritableRegister};
use tmc2209::{ReadableRegister, WriteRequest};

use crate::environment::Environment;
use crate::utils::yield_for;

const SEMIN_MIN: u8 = 1;
const SEMIN_MAX: u8 = 15;
const SEMAX_MIN: u8 = 0;
const SEMAX_MAX: u8 = 15;

/// This type configures and manages the TMC2209 stepper driver.
pub struct StepperDriver<'d> {
    is_enabled: bool,
    is_setup: bool,
    uart: Uart<'d, Async>,
    en_pin: Output<'d>,
    map: Map,
    serial_address: u8,
}

impl<'d> StepperDriver<'d> {
    /// Constructs a communication channel over UART with the stepper driver.
    pub fn new(
        baudrate: Rate,
        uart: impl UartInstance + 'static,
        tx: AnyPin<'static>,
        rx: AnyPin<'static>,
        mut en_pin: Output<'d>,
    ) -> anyhow::Result<Self> {
        let config = UartConfig::default()
            .with_data_bits(DataBits::_8)
            .with_parity(Parity::None)
            .with_stop_bits(StopBits::_1)
            .with_baudrate(baudrate.as_hz())
            // TODO: For some reason the FIFO overflows...
            // TODO: The 7 bytes won't save this, seems like we have to buffer more on our side or include an interrupt
            .with_rx(RxConfig::default().with_fifo_full_threshold(120));

        // TODO: Old code had a timeout of 400ms, this got lost, because the old wrapper is no longer used
        //let uart = BufferedUart::new(Uart::new(uart, config)?.with_tx(tx).with_rx(rx));

        let uart = Uart::new(uart, config)?
            .with_tx(tx)
            .with_rx(rx)
            .into_async();

        en_pin.set_high(); // Disable driver initially

        Ok(Self {
            is_enabled: false,
            is_setup: false,
            uart,
            en_pin,
            map: default_map(),
            serial_address: 0,
        })
    }

    async fn update_register<R: WritableRegister + Copy + 'static>(
        &mut self,
        update: impl FnOnce(&mut R),
    ) -> anyhow::Result<()> {
        let reg = self.map.reg_mut::<R>();
        update(reg);

        // After the update, it has to write back the register to the driver.
        send_write(self.serial_address, *reg, &mut self.uart).await?;

        Ok(())
    }

    pub async fn disable(&mut self) -> anyhow::Result<()> {
        if !self.is_enabled {
            return Ok(());
        }

        self.en_pin.set_high();
        self.is_enabled = false;
        self.map.chopconf_mut().set_toff(0);
        send_write(self.serial_address, *self.map.chopconf(), &mut self.uart).await?;

        Ok(())
    }

    pub fn is_setup(&self) -> bool {
        self.is_setup
    }

    /// Checks if the driver is communicating.
    async fn is_communicating(&mut self) -> anyhow::Result<bool> {
        const VERSION: u8 = 0x21;

        Ok(self.version().await? == VERSION)
    }

    async fn version(&mut self) -> anyhow::Result<u8> {
        Ok(self.read_register::<reg::IOIN>().await?.version())
    }

    pub async fn read_register<R: ReadableRegister + Copy + 'static>(
        &mut self,
    ) -> anyhow::Result<R> {
        let result = send_and_await_read::<R>(self.serial_address, &mut self.uart).await?;

        self.map.set_state(result.into());

        Ok(result)
    }

    async fn setup(&mut self) -> anyhow::Result<()> {
        if self.is_setup() {
            return Ok(());
        }

        let movement_config = Environment::resolve_movement_config();

        // TODO: It seems suboptimal to pass on the error to the caller here
        log::info!("[driver] waiting for communication to be established...");
        while !self.is_communicating().await? {
            log::info!("[driver] waiting for communication to be established...");
            yield_for(Duration::from_millis(500)).await;
        }

        log::info!("[driver] driver is communicating, applying configuration...");

        // The driver refuses to work if the drv_err flag is set, this clears it:
        self.update_register::<reg::GSTAT>(|reg| reg.set_drv_err(true))
            .await?;

        // TODO: Might set motor currents to 0 here and disable both automatic current scaling and gradient adaption

        // Write all writable registers from the map to the driver:
        for state in self.map.iter() {
            // skip read-only registers:
            if !state.addr().writable() {
                continue;
            }

            send_write_request(
                WriteRequest::try_from_state(self.serial_address, *state).unwrap(),
                &mut self.uart,
            )
            .await?;
        }

        loop {
            self.update_register(|reg: &mut reg::IHOLD_IRUN| {
                reg.set_irun((Environment::run_current() * 31.0) as u8);
                reg.set_ihold(0); // To allow for freewheeling when not moving
            })
            .await?;

            self.update_register(|reg: &mut reg::PWMCONF| {
                reg.set_pwm_autoscale(true);
                reg.set_pwm_autograd(true);
                reg.set_freewheel(StandstillMode::Freewheeling);
            })
            .await?;

            self.update_register(|reg: &mut reg::CHOPCONF| {
                reg.set_mres(MicroStepResolution::from_microsteps(
                    movement_config.microsteps_per_step,
                ));
            })
            .await?;

            // TODO: correctly set cool step/stall guard thresholds?

            self.update_register(|reg: &mut reg::COOLCONF| {
                reg.set_semin(constrain(1, SEMIN_MIN, SEMIN_MAX) as u16);
                reg.set_semax(constrain(0, SEMAX_MIN, SEMAX_MAX) as u16);
            })
            .await?;

            if Environment::enable_stealth_chop() {
                self.update_register(|reg: &mut reg::GCONF| {
                    reg.set_en_spread_cycle(false);
                })
                .await?;
            }

            log::info!(
                "[driver] is_hardware_disabled: {}",
                self.read_register::<reg::IOIN>().await?.enn()
            );

            self.actual_set_enable(true)?;

            log::info!("[driver] version: {}", self.version().await?);

            log::info!(
                "[driver] expecting microsteps_per_step: {} = {:?}",
                movement_config.microsteps_per_step,
                MicroStepResolution::from_microsteps(movement_config.microsteps_per_step)
            );
            let set_microsteps_per_step = self.read_register::<reg::CHOPCONF>().await?.mres();
            log::info!(
                "[driver] microsteps_per_step: 2^{} = {}",
                set_microsteps_per_step.exponent(),
                set_microsteps_per_step.number_of_microsteps()
            );

            if set_microsteps_per_step
                == MicroStepResolution::from_microsteps(movement_config.microsteps_per_step)
            {
                break;
            } else {
                log::warn!("[driver] microsteps_per_step not set correctly, retrying...");
                self.actual_set_enable(false)?;
                yield_for(Duration::from_millis(500)).await;
            }
        }

        log::info!(
            "[driver] status: {:?}",
            self.read_register::<reg::DRV_STATUS>().await?
        );
        log::info!("[driver] driver setup is complete");
        self.is_setup = true;

        Ok(())
    }

    fn actual_set_enable(&mut self, is_enabled: bool) -> anyhow::Result<()> {
        if is_enabled {
            self.en_pin.set_low();
        } else {
            self.en_pin.set_high();
        }

        self.is_enabled = is_enabled;

        Ok(())
    }

    /// Enables the stepper driver.
    ///
    /// If the driver is already enabled, this function does nothing.
    /// If the setup has not been performed yet, this function will do the initial setup first.
    pub async fn enable(&mut self) -> anyhow::Result<()> {
        if !self.is_setup() {
            self.setup().await?;
        }

        if self.is_enabled {
            return Ok(());
        }

        self.actual_set_enable(true)
    }
}
