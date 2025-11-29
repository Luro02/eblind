use core::sync::atomic::{AtomicUsize, Ordering};
use std::collections::VecDeque;
use std::sync::Arc;

use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::rmt::config::{TransmitConfig, TxChannelConfig};
use esp_idf_hal::rmt::encoder::CopyEncoder;
use esp_idf_hal::rmt::{RmtChannel, Symbol, TxChannelDriver};
use esp_idf_hal::sys::*;
use esp_idf_hal::units::{FromValueType, Hertz};

pub struct CustomTxChannelDriver<'d> {
    driver: TxChannelDriver<'d>,
    pending: VecDeque<(CopyEncoder, Vec<Symbol>)>,
    pending_count: Arc<AtomicUsize>,
    resolution: Hertz,
}

impl<'d> CustomTxChannelDriver<'d> {
    pub fn new(pin: impl OutputPin + 'd) -> Result<Self, EspError> {
        let resolution = 16_u32.MHz().into();
        Ok(Self {
            driver: TxChannelDriver::new(
                pin,
                &TxChannelConfig {
                    resolution,
                    transaction_queue_depth: 8,
                    ..Default::default()
                },
            )?,
            pending: VecDeque::new(),
            pending_count: Arc::new(AtomicUsize::new(0)),
            resolution,
        })
    }

    pub fn disable(&mut self) -> Result<(), EspError> {
        if self.driver.is_enabled() {
            self.driver.disable()?;
        }

        self.pending_count.store(0, Ordering::SeqCst);
        self.pending.clear();

        Ok(())
    }

    pub fn resolution(&self) -> Hertz {
        self.resolution
    }

    /// Starts sending the given signal asynchronously.
    ///
    /// # Cancel Safety
    ///
    /// The future can safely be cancelled. If the future is cancelled before it is started,
    /// nothing happens. If the future is cancelled while it is in progress, the transmission will
    /// still complete.
    pub async fn start_send(
        &mut self,
        signal: &[Symbol],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        let mut raw_encoder = CopyEncoder::new()?;
        let signal = signal.to_vec();

        // SAFETY: The signal must live until the transmission is complete (ensured through 'd, which must outlive self)
        //         and on drop it will disable the driver anyway
        // NOTE: This is only safe if nobody calls mem::forget, but I am not going to do this anyway.
        // Asynchronously wait until we can start sending
        loop {
            match unsafe {
                self.driver.start_send(
                    &mut raw_encoder,
                    signal.as_slice(),
                    &TransmitConfig {
                        queue_non_blocking: true,
                        ..config.clone()
                    },
                )
            } {
                Ok(()) => break,
                Err(err) if err.code() == ESP_ERR_TIMEOUT => {
                    self.driver.wait_for_progress().await;
                }
                Err(err) => return Err(err),
            }
        }

        // Remove old pending signals to keep the queue size short
        while self.pending.len() >= self.driver.queue_size() {
            self.pending.pop_front();
        }

        self.pending.push_back((raw_encoder, signal));
        self.pending_count.fetch_add(1, Ordering::SeqCst);

        Ok(())
    }
}

unsafe impl Sync for CustomTxChannelDriver<'_> {}
unsafe impl Send for CustomTxChannelDriver<'_> {}
