#![recursion_limit = "256"]

mod button;
mod environment;
pub(crate) mod executor;
pub(crate) mod led;
mod matter;
mod movement;
pub(crate) mod storage;
mod tmc2209_helper;
pub(crate) mod utils;

use core::time::Duration;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::{Watch, WatchBehavior};
use esp_idf_hal::interrupt::IsrCriticalSection;
use smart_leds::RGB8;
use static_cell::StaticCell;

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use tmc2209::reg::SG_RESULT;

use crate::button::spawn_button_poll_tasks;
use crate::environment::Environment;
use crate::executor::Executor;
use crate::led::{led_task, LedController, LedTarget, LED_TARGET_WATCH};
use crate::movement::StepperController;
use crate::storage::Storage;
use crate::utils::{yield_for, TakePin};

pub type EmbassyMutex<T> = Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T>;
pub type EmbassyWatch<T, const N: usize> = Watch<IsrCriticalSectionRawMutex, T, N>;

/// A mutex that allows borrowing data across executors and interrupts.
///
/// # Safety
///
/// This mutex is safe to share between different executors and interrupts.
pub struct IsrCriticalSectionRawMutex {
    cs: IsrCriticalSection,
}
unsafe impl Send for IsrCriticalSectionRawMutex {}
unsafe impl Sync for IsrCriticalSectionRawMutex {}

impl IsrCriticalSectionRawMutex {
    /// Create a new `IsrCriticalSectionRawMutex`.
    pub const fn new() -> Self {
        Self {
            cs: IsrCriticalSection::new(),
        }
    }
}

unsafe impl RawMutex for IsrCriticalSectionRawMutex {
    const INIT: Self = Self::new();

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        let guard = self.cs.enter();
        let result = f();
        drop(guard);
        result
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
pub static START_CALIBRATION_DONE_WATCH: EmbassyWatch<(), 2> = Watch::new();
pub static MAX_CALIBRATION_DONE_WATCH: EmbassyWatch<(), 2> = Watch::new();

#[embassy_executor::task]
async fn periodically_store_position(
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    nvs: Arc<Storage>,
) {
    START_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await; // Wait for start calibration to finish before storing position
    MAX_CALIBRATION_DONE_WATCH.receiver().unwrap().get().await; // Wait for max calibration to finish before storing position

    loop {
        yield_for(Duration::from_secs(1)).await;
        if let Err(err) = controller.lock().await.store(&*nvs) {
            log::error!("Failed to store current position in NVS: {}", err);
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
    storage: Arc<Storage>,
    led_controller: LedController<'static>,
    buttons: [AnyIOPin<'static>; 3],
}

impl ProgramState {
    pub fn new(
        controller: Arc<EmbassyMutex<StepperController<'static>>>,
        storage: Arc<Storage>,
        led_controller: LedController<'static>,
        buttons: [AnyIOPin<'static>; 3],
    ) -> Self {
        Self {
            controller,
            storage,
            led_controller,
            buttons,
        }
    }

    pub fn spawn(self, spawner: Spawner, is_ready: Arc<AtomicBool>) -> anyhow::Result<()> {
        log::info!("Spawning LED task...");

        spawner.spawn(show_stallguard(self.controller.clone()))?;
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

        let needs_start_calibration =
            !crate::utils::block_on(self.controller.lock()).has_position();
        let needs_max_calibration =
            !crate::utils::block_on(self.controller.lock()).has_max_position();

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

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    //esp_idf_svc::log::set_target_level();

    log::info!("Registering eventfd vfs...");
    let config = esp_idf_sys::esp_vfs_eventfd_config_t {
        max_fds: 10,
        ..Default::default()
    };
    esp_idf_sys::esp! { unsafe { esp_idf_sys::esp_vfs_eventfd_register(&config) } }?;

    // log::info!("Initializing async-io");
    // let mounted_event_fs = Arc::new(esp_idf_svc::io::vfs::MountedEventfs::mount(10)?);
    // // Eagerly initialize `async-io` to minimize the risk of stack blowups later on
    // esp_idf_matter::init_async_io(mounted_event_fs)?;

    let peripherals = Peripherals::take()?;
    log::info!("Took peripherals.");
    let sysloop = EspSystemEventLoop::take()?;
    log::info!("Took sysloop.");
    let timer = EspTaskTimerService::new()?;
    log::info!("Took timer.");
    let nvs = EspDefaultNvsPartition::take()?;
    log::info!("Took nvs.");

    let mut pins = [
        Some(peripherals.pins.gpio0.degrade_input_output()),
        Some(peripherals.pins.gpio1.degrade_input_output()),
        Some(peripherals.pins.gpio2.degrade_input_output()),
        Some(peripherals.pins.gpio3.degrade_input_output()),
        Some(peripherals.pins.gpio4.degrade_input_output()),
        Some(peripherals.pins.gpio5.degrade_input_output()),
        Some(peripherals.pins.gpio6.degrade_input_output()),
        Some(peripherals.pins.gpio7.degrade_input_output()),
        Some(peripherals.pins.gpio8.degrade_input_output()),
        Some(peripherals.pins.gpio9.degrade_input_output()),
        Some(peripherals.pins.gpio10.degrade_input_output()),
        Some(peripherals.pins.gpio11.degrade_input_output()),
        Some(peripherals.pins.gpio12.degrade_input_output()),
        Some(peripherals.pins.gpio13.degrade_input_output()),
        None, // GPIO14
        None, // GPIO15
        None, // GPIO16
        None, // GPIO17
        None, // GPIO18
        None, // GPIO19
        None, // GPIO20
        Some(peripherals.pins.gpio21.degrade_input_output()),
    ];

    let storage = Arc::new(Storage::new(nvs.clone())?);

    // Ensure that the secondary LEDs are always turned off
    if let Some(led_pin) = Environment::led2_pin() {
        let mut controller = LedController::new(peripherals.spi3, pins.take_pin(led_pin)?)?;

        controller.write([LedTarget::off().color(); 20])?;
    }

    let led_controller =
        LedController::new(peripherals.spi2, pins.take_pin(Environment::led_pin())?)?;

    log::info!("Waiting for StepperController to be available...");
    let controller = Arc::new(EmbassyMutex::new(StepperController::new(
        peripherals.uart1,
        pins.take_pin(Environment::step_pin())?,
        pins.take_pin(Environment::dir_pin())?,
        &*storage,
        pins.take_pin(Environment::uart_tx_pin())?,
        pins.take_pin(Environment::uart_rx_pin())?,
        pins.take_pin(Environment::enable_pin())?,
    )?));

    let is_ready = Arc::new(AtomicBool::new(false));

    // "Eagerly" spawn the matter thread so it can initialize itself
    let _matter_thread = matter::spawn_matter_task(
        is_ready.clone(),
        controller.clone(),
        peripherals.modem,
        sysloop,
        timer,
        nvs,
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
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        program_state.spawn(spawner, is_ready).unwrap();
    });
}
