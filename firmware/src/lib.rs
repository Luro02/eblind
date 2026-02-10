#![no_std]
#![no_main]

extern crate alloc;

mod button;
mod environment;
mod led;
mod matter;
mod movement;
mod storage;
mod tmc2209_helper;
mod utils;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};
use esp_hal::gpio::{AnyPin, Flex, Level, Output, OutputConfig};
use esp_hal::peripherals::Peripherals;

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::{Watch, WatchBehavior};
use embassy_time::{Duration, Timer};
use smart_leds::RGB8;
use tmc2209::reg::SG_RESULT;

use crate::button::spawn_button_poll_tasks;
use crate::environment::Environment;
use crate::led::{LED_TARGET_WATCH, LedController, led_task};
use crate::movement::StepperController;
use crate::storage::Storage;
use crate::utils::yield_for;

pub type EmbassyMutex<T> = Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T>;
pub type EmbassyWatch<T, const N: usize> =
    Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T, N>;

pub static START_CALIBRATION_DONE_WATCH: EmbassyWatch<(), 2> = Watch::new();
pub static MAX_CALIBRATION_DONE_WATCH: EmbassyWatch<(), 2> = Watch::new();

#[embassy_executor::task]
async fn periodically_store_position(
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
) {
    START_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await; // Wait for start calibration to finish before storing position
    MAX_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await; // Wait for max calibration to finish before storing position

    loop {
        yield_for(Duration::from_secs(1)).await;
        if let Err(err) = controller.lock().await.store(&storage).await {
            log::error!("Failed to store current position in NVS: {err}");
        }
    }
}

#[embassy_executor::task]
async fn led_indicator_task(is_ready: Arc<AtomicBool>) {
    if !START_CALIBRATION_DONE_WATCH.contains_value() {
        log::info!("The start position is not calibrated, starting calibration.");
        LED_TARGET_WATCH.sender().send(led::LedTarget::Blink {
            color: RGB8::new(0x00, 0x25, 0x25),
            duration: Duration::from_millis(500),
        });
    }

    // Wait for the start calibration to be done:
    START_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await;
    if !MAX_CALIBRATION_DONE_WATCH.contains_value() {
        log::info!("The end position is not calibrated, starting calibration.");
        LED_TARGET_WATCH.sender().send(led::LedTarget::Blink {
            color: RGB8::new(0x25, 0x25, 0x00),
            duration: Duration::from_millis(500),
        });
    }
    MAX_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await;

    // Indicate that the system is running normally by turning the led off (less annoying for users)
    LED_TARGET_WATCH.sender().send(led::LedTarget::off());
    is_ready.store(true, Ordering::Relaxed);
}

#[embassy_executor::task]
async fn show_stallguard(controller: Arc<EmbassyMutex<StepperController<'static>>>) {
    loop {
        yield_for(Duration::from_millis(500)).await;

        let stallguard = controller
            .lock()
            .await
            .driver()
            .read_register::<SG_RESULT>()
            .await;

        // 390 while it is "off"? or 418??? I dont know
        log::info!("StallGuard value: {}", stallguard.unwrap().get());
    }
}

pub struct ProgramState {
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
    led_controller: LedController<'static>,
    buttons: [AnyPin<'static>; 3],
}

impl ProgramState {
    pub fn new(
        controller: Arc<EmbassyMutex<StepperController<'static>>>,
        storage: Storage<'static>,
        led_controller: LedController<'static>,
        buttons: [AnyPin<'static>; 3],
    ) -> Self {
        Self {
            controller,
            storage,
            led_controller,
            buttons,
        }
    }

    pub async fn spawn(self, spawner: Spawner, is_ready: Arc<AtomicBool>) -> anyhow::Result<()> {
        log::info!("Spawning LED task...");

        // TODO: This one is not yet ready
        //spawner.spawn(show_stallguard(self.controller.clone()))?;
        // The led task will dynamically react to the targets too.
        spawner.spawn(led_task(self.led_controller))?;
        spawner.spawn(led_indicator_task(is_ready.clone()))?;

        log::info!("Spawning task to periodically store position...");
        // Makes sure that the current position is stored persistently.
        spawner.spawn(periodically_store_position(
            self.controller.clone(),
            self.storage.clone(),
        ))?;

        log::info!("Spawning button poll tasks...");
        spawn_button_poll_tasks(
            spawner,
            self.buttons,
            self.controller.clone(),
            self.storage.clone(),
        )?;

        log::info!("Setting initial calibration state...");

        let needs_start_calibration = !self.controller.lock().await.has_position();
        let needs_max_calibration = !self.controller.lock().await.has_max_position();

        if !needs_start_calibration {
            log::info!("Current position is already calibrated.");
            START_CALIBRATION_DONE_WATCH.sender().send(());
        }

        if !needs_max_calibration {
            log::info!("No calibration required.");
            MAX_CALIBRATION_DONE_WATCH.sender().send(());
        }

        Ok(())
    }
}

pub struct PinTracker {
    used_pins: Vec<u8>,
}

impl PinTracker {
    /// Creates a new PinTracker.
    ///
    /// # Safety
    ///
    /// You must ensure that this is the only instance of PinTracker in the program.
    pub const unsafe fn new() -> Self {
        Self {
            used_pins: Vec::new(),
        }
    }

    pub fn take_pin(&mut self, pin: u8) -> anyhow::Result<AnyPin<'static>> {
        if self.used_pins.contains(&pin) {
            return Err(anyhow::anyhow!("Pin {pin} is already taken"));
        }

        self.used_pins.push(pin);
        Ok(unsafe { AnyPin::steal(pin) })
    }
}

pub async fn run(spawner: Spawner, peripherals: Peripherals) -> anyhow::Result<()> {
    // SAFETY: This is only instantiated here
    let mut pins = unsafe { PinTracker::new() };

    let flash_access = storage::FlashAccess::new(peripherals.FLASH);

    let storage = Storage::new(flash_access, storage::StoragePartition::Nvs).await?;

    if Environment::should_clear_stored_data() {
        storage
            .reset_partitions(&storage::StoragePartition::all())
            .await?;
    }

    log::info!("Ensuring LEDs are off...");
    // Ensure that the secondary LEDs are always turned off
    if let Some(led_pin) = Environment::led2_pin() {
        let mut controller = LedController::new(peripherals.SPI3, pins.take_pin(led_pin)?);

        controller
            .write([RGB8::new(0, 0, 0); 20])
            .map_err(|err| anyhow::anyhow!("failed to turn leds off: {err:?}"))?;

        // Wait a bit to ensure the leds are off
        Timer::after(Duration::from_millis(400)).await;
    }

    log::info!("Starting LED controller...");

    let led_controller =
        LedController::new(peripherals.SPI2, pins.take_pin(Environment::led_pin())?);

    log::info!("Waiting for StepperController to be available...");
    let controller = Arc::new(EmbassyMutex::new(
        StepperController::new(
            peripherals.RMT,
            peripherals.PCNT,
            peripherals.UART1,
            Flex::new(pins.take_pin(Environment::step_pin())?),
            Flex::new(pins.take_pin(Environment::dir_pin())?),
            &storage,
            pins.take_pin(Environment::uart_tx_pin())?,
            pins.take_pin(Environment::uart_rx_pin())?,
            Output::new(
                pins.take_pin(Environment::enable_pin())?,
                Level::Low,
                OutputConfig::default(),
            ),
        )
        .await?,
    ));

    let is_ready = Arc::new(AtomicBool::new(false));

    // Start the matter task
    matter::spawn_matter_task(
        spawner,
        is_ready.clone(),
        peripherals.WIFI,
        peripherals.BT,
        controller.clone(),
        storage.clone(),
    )?;

    let program_state = ProgramState::new(
        controller.clone(),
        storage,
        led_controller,
        [
            pins.take_pin(Environment::up_button_pin())?,
            pins.take_pin(Environment::pause_button_pin())?,
            pins.take_pin(Environment::down_button_pin())?,
        ],
    );

    log::info!("Starting executor...");
    program_state.spawn(spawner, is_ready).await?;

    Ok(())
}
