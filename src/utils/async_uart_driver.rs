//! Rust doesn't like the `AsyncUartTxDriver<'d, T> where T: BorrowMut<UartTxDriver<'d>>`, therefore refusing
//! to make any futures using it `Send`.
//!
//! I could not think of a better solution other than reimplementing the `AsyncUartDriver` without the `BorrowMut` trait bound.

use embassy_time::{Duration, Instant};
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use esp_idf_svc::hal::io::EspIOError;
use esp_idf_svc::hal::uart::{config::Config, Uart, UartDriver};
use esp_idf_svc::sys::EspError;
use esp_idf_svc::sys::ESP_ERR_TIMEOUT;

use crate::utils::yield_now;

// TODO: make an upstream issue regarding the use of the `Borrow`
pub struct AsyncUartDriver<'d> {
    driver: UartDriver<'d>,
    timeout: Duration,
}

impl<'d> AsyncUartDriver<'d> {
    pub fn new<UART: Uart + 'd>(
        uart: UART,
        tx: impl OutputPin + 'd,
        rx: impl InputPin + 'd,
        cts: Option<impl InputPin + 'd>,
        rts: Option<impl OutputPin + 'd>,
        timeout: Duration,
        config: &Config,
    ) -> Result<Self, EspError> {
        Self::wrap(UartDriver::new(uart, tx, rx, cts, rts, config)?, timeout)
    }

    pub fn wrap(driver: UartDriver<'d>, timeout: Duration) -> Result<Self, EspError> {
        Ok(Self { driver, timeout })
    }

    pub async fn read(&self, buf: &mut [u8]) -> Result<usize, EspError> {
        if buf.is_empty() {
            return Ok(0);
        }

        let instant = Instant::now();
        loop {
            let res = self.driver.read(buf, delay::NON_BLOCK);

            match res {
                Ok(len) if len > 0 => return Ok(len),
                Err(e) if e.code() != ESP_ERR_TIMEOUT => return Err(e),
                _ => (),
            }

            if instant.elapsed() > self.timeout {
                return Err(EspError::from(ESP_ERR_TIMEOUT).unwrap());
            }

            yield_now().await;
        }
    }

    pub async fn write(&self, bytes: &[u8]) -> Result<usize, EspError> {
        if bytes.is_empty() {
            return Ok(0);
        }

        loop {
            let res = self.driver.write_nb(bytes);

            match res {
                Ok(len) if len > 0 => return Ok(len),
                Err(e) => return Err(e),
                _ => (),
            }

            // We cannot properly wait for the TX FIFO queue to become non-full
            // because the ESP IDF UART ISR does not notify us on that
            //
            // Instead, spin a busy loop, however still allowing other futures to be polled too.
            yield_now().await;
        }
    }

    pub async fn wait_tx_done(&self) -> Result<(), EspError> {
        loop {
            let res = self.driver.wait_tx_done(delay::NON_BLOCK);

            match res {
                Ok(()) => return Ok(()),
                Err(e) if e.code() != ESP_ERR_TIMEOUT => return Err(e),
                _ => (),
            }

            // We cannot properly wait for the TX FIFO queue to become empty
            // because the ESP IDF UART ISR does not notify us on that
            //
            // Instead, spin a busy loop, however still allowing other futures to be polled too.
            yield_now().await;
        }
    }
}

impl<'d> embedded_io::ErrorType for AsyncUartDriver<'d> {
    type Error = EspIOError;
}

impl<'d> embedded_io_async::Read for AsyncUartDriver<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        AsyncUartDriver::read(self, buf).await.map_err(EspIOError)
    }
}

impl<'d> embedded_io_async::Write for AsyncUartDriver<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        AsyncUartDriver::write(self, buf).await.map_err(EspIOError)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        AsyncUartDriver::wait_tx_done(self)
            .await
            .map_err(EspIOError)
    }
}
