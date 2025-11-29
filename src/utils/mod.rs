mod async_uart_driver;
pub use async_uart_driver::*;
mod async_queue;
pub use async_queue::*;

use core::future::Future;
use core::pin::pin;
use std::task::Poll;
use std::time::Duration;

use embassy_time::{Duration as EmbassyDuration, Instant, Timer};
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::gpio::{AnyInputPin, AnyOutputPin, InputPin, OutputPin, Pull};
use esp_idf_sys::EspError;

/// Yields to the scheduler, allowing other tasks to run, and does a yield to the OS to allow other threads to run.
pub async fn yield_now() {
    esp_idf_hal::task::yield_now().await;
    esp_idf_hal::task::do_yield();
}

/// Yields for at least the given duration, allowing other tasks and threads to run.
pub async fn yield_for(duration: Duration) {
    // Keep the current instant to calculate when we want to wake up again.
    let instant = Instant::now();
    // converting to u64 is okay here, it will only overflow with durations longer than 21000 days.
    let expires_at = instant + EmbassyDuration::from_nanos(duration.as_nanos() as u64);

    // Yield to the OS scheduler to allow other threads to run.
    esp_idf_hal::task::do_yield();

    // Finish waiting until the desired time has passed:
    Timer::at(expires_at).await
}

pub fn block_on<F: Future>(fut: F) -> F::Output {
    let mut future = pin!(fut);

    loop {
        if let Poll::Ready(value) = embassy_futures::poll_once(future.as_mut()) {
            return value;
        }

        esp_idf_hal::task::do_yield();
    }
}

pub trait TakePin<P> {
    fn take_pin(&mut self, number: usize) -> anyhow::Result<P>;
}

impl<'a, P> TakePin<P> for &mut [Option<P>] {
    fn take_pin(&mut self, number: usize) -> anyhow::Result<P> {
        self[number]
            .take()
            .ok_or_else(|| anyhow::anyhow!("The pin {} is not available or already in use", number))
    }
}

impl<P, const N: usize> TakePin<P> for [Option<P>; N] {
    fn take_pin(&mut self, number: usize) -> anyhow::Result<P> {
        self.as_mut().take_pin(number)
    }
}

/// A helper to split a pin that can be used as both input and output into separate input and output pins.
///
/// This configures the pin to be used as both input and output.
pub fn split_pin<'d>(
    gpio: impl InputPin + OutputPin + 'd,
    pull: Pull,
) -> Result<(AnyInputPin<'d>, AnyOutputPin<'d>), EspError> {
    let pin = PinDriver::input_output(gpio, pull)?;

    let number = pin.pin();

    // The drop would reset the pin, which we don't want:
    core::mem::forget(pin);

    Ok(unsafe { (AnyInputPin::steal(number), AnyOutputPin::steal(number)) })
}
