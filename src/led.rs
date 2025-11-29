use core::time::Duration;

use embassy_sync::watch::Watch;
use esp_idf_hal::gpio::{AnyIOPin, OutputPin};
use esp_idf_hal::spi::config::{Config, DriverConfig};
use esp_idf_hal::spi::{SpiAnyPins, SpiBusDriver, SpiDriver, SpiError};
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::sys::EspError;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

use crate::utils::{yield_for, yield_now};
use crate::EmbassyWatch;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedTarget {
    Blink { color: RGB8, duration: Duration },
    Solid { color: RGB8 },
}

impl LedTarget {
    pub const fn off() -> Self {
        Self::Solid {
            color: RGB8::new(0, 0, 0),
        }
    }

    pub const fn is_repeating(&self) -> bool {
        matches!(self, Self::Blink { .. })
    }

    pub fn color(&self) -> RGB8 {
        match self {
            Self::Blink { color, .. } => *color,
            Self::Solid { color } => *color,
        }
    }
}

const MAX_TARGET_WATCH: usize = 2;
pub static LED_TARGET_WATCH: EmbassyWatch<LedTarget, MAX_TARGET_WATCH> =
    Watch::new_with(LedTarget::off());

#[embassy_executor::task]
pub async fn led_task(mut led_controller: LedController<'static>) {
    let mut target_receiver = LED_TARGET_WATCH
        .receiver()
        .expect("There should be at least one receiver for the led task");

    let mut last_target = LedTarget::off();
    loop {
        yield_now().await;
        let target = target_receiver.get().await;

        // TODO: improve the handling of blinking LEDs? Maybe through a channel that is emptied by this code?
        // If the target did not change and is not repeating e.g. blinking, just wait until the next change.
        if target == last_target && !target.is_repeating() {
            yield_for(Duration::from_millis(100)).await;
            continue;
        }

        match target {
            LedTarget::Blink { color, duration } => {
                led_controller.write([color]).unwrap();
                yield_for(duration).await;
                led_controller.write([RGB8::new(0, 0, 0)]).unwrap();
                yield_for(duration).await;
            }
            LedTarget::Solid { color } => {
                led_controller.write([color]).unwrap();
                yield_for(Duration::from_secs(2)).await; // Keep solid color for at least 2 seconds
            }
        }

        last_target = target;
    }
}

pub struct LedController<'d> {
    led: Ws2812<SpiBusDriver<'d, SpiDriver<'d>>>,
}

impl<'d> LedController<'d> {
    /// Creates a new LED controller, controlling the LEDs connected to the given pin.
    ///
    /// The SPI bus will be used to generate the pwm signal for the LEDs.
    pub fn new<SPI>(spi: SPI, pin: impl OutputPin + 'd) -> Result<Self, EspError>
    where
        SPI: SpiAnyPins + 'static,
    {
        let bus_config = DriverConfig::default();
        let config = Config::default().baudrate(3.MHz().into());

        let spi_driver = SpiDriver::new_without_sclk(spi, pin, AnyIOPin::none(), &bus_config)?;
        let spi_bus_driver = SpiBusDriver::new(spi_driver, &config)?;

        Ok(Self {
            led: Ws2812::new(spi_bus_driver),
        })
    }

    /// Writes the given colors to the LED strip.
    pub fn write(&mut self, colors: impl IntoIterator<Item = RGB8>) -> Result<(), SpiError> {
        self.led.write(colors)
    }
}
