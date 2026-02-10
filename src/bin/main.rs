#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::mem::ManuallyDrop;
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::timer::timg::TimerGroup;
use log::{error, info};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

use esp_backtrace as _;

use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_metadata_generated::memory_range;

const HEAP_SIZE: usize = 100 * 1024;
const RECLAIMED_RAM: usize =
    memory_range!("DRAM2_UNINIT").end - memory_range!("DRAM2_UNINIT").start;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Create the crypto provider, using the `esp-hal` TRNG/ADC1 as the source of randomness for a reseeding CSPRNG.
    //
    // On drop the `TrngSource` will be deactivated, but it should be active until the program exits
    // -> wrapped in a `ManuallyDrop` to prevent it from being dropped
    let _trng_source = ManuallyDrop::new(esp_hal::rng::TrngSource::new(
        peripherals.RNG,
        peripherals.ADC1,
    ));

    log::info!("TRNG is_enabled={}", esp_hal::rng::TrngSource::is_enabled());

    esp_alloc::heap_allocator!(size: HEAP_SIZE - RECLAIMED_RAM);
    // A part of the ram is used by the bootloader, this is reclaimed here
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: RECLAIMED_RAM);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    info!("Embassy initialized!");

    if let Err(err) = eblind::run(spawner, unsafe { Peripherals::steal() }).await {
        error!("Error: {err}");
    }
}
