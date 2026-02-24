use core::cell::RefCell;
use core::fmt;
use core::sync::atomic::{AtomicUsize, Ordering};

use bincode::{Decode, Encode};
use critical_section::Mutex;

use esp_hal::clock::Clocks;
use esp_hal::gpio::interconnect::{InputSignal, PeripheralInput};
use esp_hal::gpio::{AnyPin, Flex, Level, Output, OutputConfig};
use esp_hal::interrupt::Priority;
use esp_hal::pcnt::Pcnt;
use esp_hal::pcnt::channel::{CtrlMode, EdgeMode};
use esp_hal::pcnt::unit::{self, Unit};
use esp_hal::peripherals::{PCNT, RMT};
use esp_hal::ram;
use esp_hal::rmt::{PulseCode, TxChannelConfig};
use esp_hal::time::Rate;
use esp_hal::uart::Instance as UartInstance;

use crate::environment::Environment;
use crate::matter::Position;
use crate::movement::rmt::TxDriver;
use crate::movement::stepper_driver::StepperDriver;
use crate::storage::Storage;

#[derive(Debug, Default)]
pub struct UnitPosition {
    value: AtomicUsize,
}

impl Encode for UnitPosition {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.value.load(Ordering::SeqCst).encode(encoder)
    }
}

impl<C> Decode<C> for UnitPosition {
    fn decode<D: bincode::de::Decoder>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let value = usize::decode(decoder)?;
        Ok(Self {
            value: AtomicUsize::new(value),
        })
    }
}

impl UnitPosition {
    pub const fn new(value: usize) -> Self {
        Self {
            value: AtomicUsize::new(value),
        }
    }

    pub fn max() -> Self {
        Self::new(usize::MAX)
    }

    pub fn from_position(position: Position, max: &UnitPosition) -> Self {
        Self::new(position.map_to(max.to_value() as f64) as usize)
    }

    pub fn add_assign(&self, value: isize) {
        if value >= 0 {
            self.value.fetch_add(value as usize, Ordering::SeqCst);
        } else {
            self.value.fetch_sub((-value) as usize, Ordering::SeqCst);
        }
    }

    pub fn set(&self, position: &UnitPosition) {
        self.value.store(position.to_value(), Ordering::SeqCst);
    }

    pub fn to_value(&self) -> usize {
        self.value.load(Ordering::SeqCst)
    }

    #[must_use]
    pub fn to_position(&self, max: &UnitPosition) -> Position {
        Position::new(self.to_value() as f64 / max.to_value() as f64)
    }
}

impl Clone for UnitPosition {
    fn clone(&self) -> Self {
        Self::new(self.to_value())
    }
}

impl PartialEq for UnitPosition {
    fn eq(&self, other: &Self) -> bool {
        self.to_value() == other.to_value()
    }
}

impl fmt::Display for UnitPosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.to_value())
    }
}

const PCNT_LOW_LIMIT: i16 = -i16::MAX;
const PCNT_HIGH_LIMIT: i16 = -PCNT_LOW_LIMIT;

/// Type to control a stepper motor.
pub struct StepperController<'d> {
    current_position: &'d UnitPosition,
    max_position: UnitPosition,
    target: Option<UnitPosition>,
    stepper_driver: StepperDriver<'d>,
    tx_driver: TxDriver<'d>,
    has_position: bool,
    dir_pin: Output<'d>,
}

const PCNT_UNIT_NUM: usize = 1;
static UNIT0: Mutex<RefCell<Option<unit::Unit<'static, PCNT_UNIT_NUM>>>> =
    Mutex::new(RefCell::new(None));
static CURRENT_POSITION: UnitPosition = UnitPosition::new(0);

#[ram]
#[esp_hal::handler(priority = Priority::Priority2)]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut u0 = UNIT0.borrow_ref_mut(cs);
        if let Some(u0) = u0.as_mut() {
            if !u0.interrupt_is_set() {
                return;
            }

            let events = u0.events();
            if events.high_limit {
                CURRENT_POSITION.add_assign(PCNT_HIGH_LIMIT as isize);
            } else if events.low_limit {
                CURRENT_POSITION.add_assign(PCNT_LOW_LIMIT as isize);
            }

            u0.reset_interrupt();
        }
    });
}

fn access_pcnt_unit<T>(f: impl FnOnce(&mut unit::Unit<'_, PCNT_UNIT_NUM>) -> T) -> T {
    critical_section::with(|cs| f((*UNIT0.borrow_ref_mut(cs)).as_mut().unwrap()))
}

fn build_pcnt<'d, const U: usize>(
    unit: &Unit<'d, U>,
    step_pin: Option<impl PeripheralInput<'d>>,
    dir_pin: Option<impl PeripheralInput<'d>>,
) -> anyhow::Result<()> {
    unit.set_low_limit(Some(PCNT_LOW_LIMIT)).unwrap();
    unit.set_high_limit(Some(PCNT_HIGH_LIMIT)).unwrap();
    let max_glitch_ns = 2000;
    // TODO: upstream this to esp-hal
    unit.set_filter(Some(
        (Clocks::get().apb_clock.as_hz() / 1_000_000 * max_glitch_ns / 1_000) as u16,
    ))
    .unwrap();
    unit.clear();

    let channel = &unit.channel0;

    // When the dir_pin is high, reverse the counting direction, and when low, keep it
    channel.set_ctrl_mode(CtrlMode::Keep, CtrlMode::Reverse);
    if let Some(dir_pin) = dir_pin {
        channel.set_ctrl_signal(dir_pin);
    }

    // when signal goes from low to high, increase counter, but do nothing when it goes back to low
    channel.set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
    if let Some(step_pin) = step_pin {
        channel.set_edge_signal(step_pin);
    }

    unit.clear();
    unit.listen();
    unit.resume();

    Ok(())
}

impl<'d> StepperController<'d> {
    pub async fn new(
        rmt: RMT<'d>,
        pcnt: PCNT<'static>,
        uart: impl UartInstance + 'static,
        mut step_pin: Flex<'d>,
        mut dir_pin: Flex<'d>,
        storage: &Storage<'_>,
        tx: AnyPin<'static>,
        rx: AnyPin<'static>,
        enable_pin: Output<'d>,
    ) -> anyhow::Result<Self> {
        let storage_position = storage.current_position().await?;
        let has_position = storage_position.is_some();
        CURRENT_POSITION.set(&storage_position.unwrap_or(UnitPosition::new(usize::MAX / 2)));
        let current_position = &CURRENT_POSITION;

        let max_position = storage.max_position().await?.unwrap_or(UnitPosition::max());

        // We need to split the pins into input and output pins,
        // the inputs are used by the PCNT to track the position
        // and the step output pin is used by the RMT peripheral
        // to drive the stepper motor.

        step_pin.set_output_enable(true);
        step_pin.apply_output_config(&OutputConfig::default().with_pull(esp_hal::gpio::Pull::Down));
        step_pin.set_level(Level::Low);

        let (step_in, step_out) = unsafe { step_pin.split_into_drivers() };

        dir_pin.set_output_enable(true);
        dir_pin.apply_output_config(&Default::default());

        let (dir_in, dir_out) = unsafe { dir_pin.split_into_drivers() };

        let mut dir_in: InputSignal = dir_in.into();
        if Environment::should_invert_direction() {
            dir_in = dir_in.with_input_inverter(true);
        }

        let tx_driver = TxDriver::new(
            rmt,
            step_out,
            Rate::from_mhz(16),
            TxChannelConfig::default()
                .with_clk_divider(1)
                .with_idle_output_level(Level::Low)
                .with_idle_output(true),
        )?;

        let mut pcnt = Pcnt::new(pcnt);
        pcnt.set_interrupt_handler(interrupt_handler);

        build_pcnt(&pcnt.unit1, Some(step_in), Some(dir_in))?;

        critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(pcnt.unit1));

        Ok(Self {
            current_position,
            max_position,
            stepper_driver: StepperDriver::new(
                Environment::uart_baud_rate(),
                uart,
                tx,
                rx,
                enable_pin,
            )?,
            target: None,
            tx_driver,
            dir_pin: dir_out,
            has_position,
        })
    }

    pub fn has_max_position(&self) -> bool {
        self.max_position != UnitPosition::max()
    }

    pub fn has_position(&self) -> bool {
        self.has_position
    }

    pub async fn target_position(&self) -> Option<Position> {
        self.target_unit_position()
            .await
            .as_ref()
            .map(|pos| pos.to_position(&self.max_position))
    }

    async fn target_unit_position(&self) -> Option<UnitPosition> {
        self.target.clone()
    }

    async fn internal_update_target_position(
        &mut self,
        new_target: Option<UnitPosition>,
    ) -> anyhow::Result<()> {
        // If the target is None, just set it to None, there is nothing more to do.
        let Some(target) = new_target else {
            self.target = None;
            return Ok(());
        };

        assert!(!self.is_moving());

        // Based on the current position, it will calculate the number of steps it has to move:
        let current_position = self.current_unit_position();
        let steps_to_move = target.to_value() as i64 - current_position.to_value() as i64;
        log::info!(
            "Moving from {current_position} to {target} (pos -> target), steps: {steps_to_move}"
        );

        // Sanity check, shouldn't happen, but just in case:
        let new_position = current_position.to_value() as i64 + steps_to_move;
        if new_position > self.max_position.to_value() as i64 || new_position < 0 {
            log::warn!(
                "Target position {target} will be out of bounds (max: {}, min: 0).",
                self.max_position
            );
            return Ok(());
        }

        // If there is nothing to do, just return:
        if steps_to_move == 0 {
            self.target = None;
            return Ok(());
        }

        // Make sure the stepper driver is enabled otherwise it won't move:
        self.stepper_driver.enable().await?;

        let should_invert_direction =
            (steps_to_move.signum() == -1) ^ Environment::should_invert_direction();

        if should_invert_direction {
            self.dir_pin.set_high();
        } else {
            self.dir_pin.set_low();
        }

        let steps_to_move = steps_to_move.unsigned_abs();
        log::info!("Moving {steps_to_move:?} steps");

        let movement_config = Environment::resolve_movement_config();

        let rpm = movement_config.max_rounds_per_second * 60.0;
        let microsteps_per_step = movement_config.microsteps_per_step as f64;
        let steps_per_revolution = movement_config.full_steps_per_rotation as f64;

        // Calculate period in nanoseconds for higher precision with RMT ticks
        // 1s = 1_000_000_000ns
        let period_ns = (60.0 * 1_000_000_000.0
            / (rpm * steps_per_revolution * microsteps_per_step))
            .max(1000.0); // Min period 1000ns = 1us

        // Define pulse width (e.g., 2-5 us) in nanoseconds
        let pulse_width_ns: u32 = 2_000; // 2 microseconds = 2000 nanoseconds

        // Calculate delay between pulses in nanoseconds
        let delay_ns = if period_ns > pulse_width_ns as f64 {
            (period_ns - pulse_width_ns as f64).max(2000.0) as u32 // Ensure minimum delay (e.g., 2us)
        } else {
            2_000 // Minimum delay if period is very short
        };

        let ticks_hz = self.tx_driver.get_resolution(); // Get the actual RMT counter clock frequency
        let ns_per_tick = 1_000_000_000u64 / ticks_hz.as_hz() as u64;

        let signal = &[PulseCode::new(
            Level::High,
            (pulse_width_ns as u64 / ns_per_tick) as u16,
            Level::Low,
            (delay_ns as u64 / ns_per_tick) as u16,
        )];

        log::info!(
            "Moving with high_length={} ticks, low_length={} ticks",
            (pulse_width_ns as u64 / ns_per_tick) as u16,
            (delay_ns as u64 / ns_per_tick) as u16,
        );

        assert!(
            steps_to_move <= u32::MAX as u64,
            "Requested to move too many steps at once."
        );

        // First ensure that there is no pending movement:
        self.tx_driver.stop_wait().await?;

        self.tx_driver.send(signal, Some(steps_to_move as usize))?;

        let duplicate_current_target = target.clone();
        let duplicate_current_position = self.current_position;
        self.tx_driver.on_finish(move || {
            // current_position only contains the overflow values,
            // the pcnt contains the remaining steps:
            let result = duplicate_current_position.clone();
            let pcnt_count = access_pcnt_unit(|unit| unit.value());
            result.add_assign(pcnt_count as isize);

            log::info!(
                "Finished movement to {duplicate_current_target}, current position is {result}"
            );
        });

        self.target = Some(target);

        Ok(())
    }

    pub async fn update_target_position(
        &mut self,
        new_target: Option<Position>,
    ) -> anyhow::Result<()> {
        // If the target did not change, there is nothing to do:
        if self.target_position().await == new_target {
            log::info!("Target position is already set to the requested value, doing nothing.");
            return Ok(());
        }

        log::info!(
            "Updating target position to {new_target:?}, currently at {}",
            self.current_unit_position()
        );

        let target =
            new_target.map(|target| UnitPosition::from_position(target, &self.max_position));
        log::info!("Converted percentage target to: {target:?}");

        // Make sure to stop the stepper motor if it is currently moving?:
        if self.is_moving() {
            log::warn!(
                "Stepper is currently moving from {} to {:?}, sending stop signal first.",
                self.current_unit_position(),
                &self.target
            );
            self.stop().await?;
        }

        self.internal_update_target_position(target).await
    }

    /// Returns true if the stepper motor is currently moving.
    pub fn is_moving(&self) -> bool {
        self.target.is_some() && self.target != Some(self.current_unit_position())
    }

    pub fn driver(&mut self) -> &mut StepperDriver<'d> {
        &mut self.stepper_driver
    }

    fn current_unit_position(&self) -> UnitPosition {
        // current_position only contains the overflow values,
        // the pcnt contains the remaining steps:
        let result = self.current_position.clone();
        let pcnt_count = access_pcnt_unit(|unit| unit.value());
        result.add_assign(pcnt_count as isize);

        result
    }

    /// Returns the current position of the stepper motor.
    ///
    /// It is a value between (inclusive range) 0.0 (min) and 1.0 (max).
    pub fn current_position(&self) -> Position {
        self.current_unit_position().to_position(&self.max_position)
    }

    /// Removes the target, stopping the stepper motor.
    ///
    /// It will wait for the stepper motor to actually stop moving.
    pub async fn stop(&mut self) -> anyhow::Result<()> {
        if !self.is_moving() {
            log::info!(
                "Stepper is not moving: current_position: {}",
                self.current_unit_position()
            );
            return Ok(());
        }

        log::info!(
            "Stopping stepper motor, currently at {}, moving to {:?}",
            self.current_unit_position(),
            self.target_unit_position().await
        );

        // This will cancel all pending step movements:
        self.tx_driver.stop_wait().await?;

        log::info!("Updating internal position after stop");
        // Update the target position to None to indicate that we are not moving anymore:
        self.internal_update_target_position(None).await?;

        log::info!(
            "Stopped stepper motor at {:?}, target: {:?}",
            self.current_unit_position(),
            self.target_unit_position().await
        );

        Ok(())
    }

    /// Resets the current position to 0.
    pub async fn reset_start_position(&mut self, storage: &Storage<'_>) -> anyhow::Result<()> {
        self.stop().await?;

        // The current position will be set to 0:
        self.current_position.set(&UnitPosition::new(0));
        access_pcnt_unit(|unit| unit.clear());
        self.has_position = true;
        self.store(storage).await?;

        log::info!("Updated position to {}", self.current_unit_position());

        Ok(())
    }

    pub async fn reset_max_position(&mut self, storage: &Storage<'_>) -> anyhow::Result<()> {
        self.stop().await?;

        // The max will be set to the current position:
        self.max_position.set(&self.current_unit_position());
        self.store(storage).await?;
        log::info!(
            "Set max position to {}, currently at {}",
            self.max_position,
            self.current_unit_position()
        );

        Ok(())
    }

    pub async fn store(&self, storage: &Storage<'_>) -> anyhow::Result<()> {
        if self.has_max_position() {
            storage.set_max_position(self.max_position.clone()).await?;
        }

        if self.has_position() {
            storage
                .set_current_position(self.current_unit_position())
                .await?;
        }

        Ok(())
    }
}
