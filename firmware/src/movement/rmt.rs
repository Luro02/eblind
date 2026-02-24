use core::cell::RefCell;
use core::mem;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::boxed::Box;

use critical_section::Mutex;
use esp_hal::gpio::Level;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::interrupt::DEFAULT_INTERRUPT_HANDLER;
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{
    self, CHANNEL_RAM_SIZE, Channel, ContinuousTxTransaction, LoopMode, MAX_TX_LOOPCOUNT,
    PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator,
};
use esp_hal::time::Rate;
use esp_hal::{Blocking, ram};

use crate::movement::rmt_utils::*;

const CHANNEL_NUMBER: u8 = 0;
#[ram]
static REMAINING_LOOPS: AtomicUsize = AtomicUsize::new(0);

#[ram]
static ON_FINISH_CALLBACK: Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>> =
    Mutex::new(RefCell::new(None));

macro_rules! min {
    ($a:expr, $b:expr) => {
        {
            if $a < $b {
                $a
            } else {
                $b
            }
        }
    };
    ($a:expr, $b:expr, $($rest:expr),+ ) => {
        min!($a, min!($b, $($rest),+))
    }
}

#[esp_hal::handler]
#[ram]
fn interrupt_handler() {
    let mut channel: RawChannel<Tx> = unsafe { RawChannel::conjure(CHANNEL_NUMBER) };

    let status = channel.interrupts();
    channel.clear_interrupts(status);

    // This interrupt is called every time the loop count is reached.
    // To prevent sending out too many symbols, the transmission is stopped
    // after it reaches the desired loop count.
    //
    // That is why this function has to restart the transmission after updating
    // the remaining loop count.
    if status.contains(Event::Loop) {
        let remaining_loops = REMAINING_LOOPS.load(Ordering::SeqCst);
        if remaining_loops == 0 {
            if let Some(callback) =
                critical_section::with(|cs| ON_FINISH_CALLBACK.borrow(cs).take())
            {
                callback();
            }
            return;
        }

        let mut state_machine = channel.state_machine();

        let this_loop_count = min!(MAX_TX_LOOPCOUNT as usize, remaining_loops);
        REMAINING_LOOPS.fetch_sub(this_loop_count, Ordering::SeqCst);

        state_machine.set_target_loop_count(this_loop_count as u16);
        state_machine.reset_ptr();
        state_machine.reset_loop_count();

        // Restart the transmission
        state_machine.start();
    }
}

#[inline]
unsafe fn copy_unchecked<T>(value: &T) -> T {
    unsafe { ptr::read(value) }
}

pub struct TxDriver<'d> {
    rmt: Rmt<'d, Blocking>,
    channel_number: u8,
    pending: Option<ContinuousTxTransaction<'d>>,
    channel: Option<Channel<'d, Blocking, Tx>>,
}

impl<'d> TxDriver<'d> {
    pub fn new(
        peripheral: RMT<'d>,
        pin: impl PeripheralOutput<'d>,
        resolution: Rate,
        config: TxChannelConfig,
    ) -> Result<Self, rmt::ConfigError> {
        // This configures the clock with the desired resolution:
        let mut rmt = Rmt::new(peripheral, resolution)?;
        rmt.set_interrupt_handler(interrupt_handler);

        let rmt_copy = unsafe { copy_unchecked(&rmt) };

        Ok(Self {
            rmt: rmt_copy,
            channel_number: CHANNEL_NUMBER,
            channel: Some(
                {
                    match CHANNEL_NUMBER {
                        0 => rmt.channel0.configure_tx(&config)?,
                        1 => rmt.channel1.configure_tx(&config)?,
                        2 => rmt.channel2.configure_tx(&config)?,
                        3 => rmt.channel3.configure_tx(&config)?,
                        _ => unimplemented!("The channel is currently unimplemented"),
                    }
                }
                .with_pin(pin),
            ),
            pending: None,
        })
    }

    pub fn get_resolution(&self) -> Rate {
        Rate::from_hz(channel_resolution(self.channel_number) as u32)
    }

    /// Registers a callback that is called when the transmission finishes.
    pub fn on_finish(&mut self, callback: impl FnOnce() + Send + 'd) {
        // SAFETY: This relies on the drop handler of this type to ensure that the callback
        //         is removed when the driver is dropped. This should ensure that the callback is
        //         never called after 'd ends.
        let boxed = unsafe {
            mem::transmute::<Box<dyn FnOnce() + Send + 'd>, Box<dyn FnOnce() + Send + 'static>>(
                Box::new(callback),
            )
        };

        critical_section::with(|cs| {
            ON_FINISH_CALLBACK.borrow(cs).replace(Some(boxed));
        });
    }

    /// The given `data` should be repeated for `total_repetitions` times.
    ///
    /// If for example `data` has a length of 1 and it should be repeated 10 times,
    /// then one could fill the buffer with 10 copies of the same element instead of
    /// looping 10 times over a single element.
    fn fill_buffer(
        &self,
        data: &[PulseCode],
        total_repetitions: usize,
        buffer: &mut [PulseCode],
    ) -> usize {
        // The maximum number of times the data can be repeated in the buffer:
        let mut max_repeat = buffer.len() / data.len();

        // Adjust the max_repeat to be a divisor of total_repetitions, so
        // that after max_repeat repetitions, the total number of repetitions
        // is exactly total_repetitions and there isn't a remainder.
        while total_repetitions % max_repeat != 0 {
            max_repeat -= 1;
        }

        let mut i = 0;
        for _ in 0..max_repeat {
            buffer[i..i + data.len()].copy_from_slice(data);
            i += data.len();
        }

        max_repeat
    }

    pub fn send(&mut self, data: &[PulseCode], count: Option<usize>) -> Result<(), rmt::Error> {
        // Ensure there is no ongoing transmission (.stop should have been called before to ensure this)
        let Some(channel) = self.channel.take() else {
            log::error!("Can not send while another transmission is ongoing");
            return Err(rmt::Error::TransmissionError);
        };

        const CUSTOM_END_MARKER: PulseCode = PulseCode::new(Level::Low, 0, Level::Low, 0);

        // Technically could be increased to more than CHANNEL_RAM_SIZE if multiple blocks are used,
        // but then it wouldn't be a const size anymore.
        let mut payload_buffer = [{ CUSTOM_END_MARKER }; CHANNEL_RAM_SIZE];

        // The data must fit into the payload buffer, and it should not be empty
        if payload_buffer.len() < data.len() || data.is_empty() {
            return Err(rmt::Error::InvalidDataLength);
        }

        // If the data exactly fills the payload buffer, there is no space to add an end marker, which is required for
        // loop counting to work correctly.
        //
        // In those cases, the provided data buffer should include an end marker at the end of the data
        if payload_buffer.len() == data.len() && !data[data.len() - 1].is_end_marker() {
            return Err(rmt::Error::EndMarkerMissing);
        }

        // Treat a count of None as a single transmission:
        let mut total_loops = count.unwrap_or(1);

        // Copy the data into the payload buffer:
        // The last element is reserved for the end-marker:
        let buffer_len = payload_buffer.len();
        let repeats = self.fill_buffer(data, total_loops, &mut payload_buffer[..buffer_len - 1]);

        log::info!(
            "Unrolled RMT data with len {} into buffer with len {} by repeating {} times, old loops: {}, new loops: {}",
            data.len(),
            repeats * data.len(),
            repeats,
            total_loops,
            total_loops / repeats
        );

        total_loops /= repeats;
        let payload_end = repeats * data.len();

        // If the loops exceed the maximum loop count, they have to be sent in batches.
        let loop_batch_size = min!(MAX_TX_LOOPCOUNT as usize, total_loops) as u16;
        total_loops -= loop_batch_size as usize;

        // This will be used by the interrupt handler to restart the transmission after each batch is finished.
        REMAINING_LOOPS.store(total_loops, Ordering::SeqCst);

        log::info!(
            "Sending an initial batch of {loop_batch_size} loops, remaining loops after this batch: {total_loops}",
        );

        let channel_number = self.channel_number;
        // Start the transmission in continuous tx mode:
        //
        // In this mode the transmitter sends the pulse codes from RAM in loops:
        // - If an end-marker is encountered, the transmitter starts transmitting from the first data of the
        //   channel's RAM again.
        // - If no end-marker is encountered, the transmitter starts transmitting from the first data again
        //   after the last data is transmitted.
        //
        // The loop counter is only incremented when an end-marker is encountered
        // -> it is necessary to include an end-marker in the data for the loop counting to work correctly.
        let mut raw_channel = unsafe { RawChannel::<Tx>::conjure(channel_number) };

        // The interrupt handler is triggered when the loop count is reached, so it can restart the
        // transmission for the next batch.
        raw_channel.listen(Event::Loop);

        self.pending = Some({
            match channel.transmit_continuously(
                &payload_buffer[..=payload_end],
                LoopMode::Finite(loop_batch_size),
            ) {
                Ok(tx) => tx,
                Err((err, channel)) => {
                    self.channel = Some(channel);
                    return Err(err);
                }
            }
        });

        // Inspect the actual buffer:
        log::info!(
            "raw_memory: {:?}",
            unsafe { raw_channel.memory() }.as_slice()
        );

        log::info!("Transmission start should be finished");

        Ok(())
    }

    pub async fn stop_wait(&mut self) -> Result<(), rmt::Error> {
        self.stop()
    }

    fn stop(&mut self) -> Result<(), rmt::Error> {
        log::warn!("Stopping transmission");
        let Some(pending) = self.pending.take() else {
            return Ok(());
        };

        // TODO: This will currently block without yielding to other tasks.
        // TODO: make a PR that implements a non-blocking try_stop() -> Option<Result<...>> to accomplish this?

        // By setting this to zero, the interrupt handler will not restart the transmission again
        // after the loop count was reached.
        REMAINING_LOOPS.store(0, Ordering::SeqCst);

        // TODO: stop_next doesn't work, because we have a custom interrupt handler that clears the interrupt status?

        match pending.stop() {
            Ok(channel) => {
                self.channel = Some(channel);
            }
            Err((err, channel)) => {
                self.channel = Some(channel);
                return Err(err);
            }
        }

        // Clear the interrupt status
        //rmt_ll_tx_clear_interrupt_status(
        //    self.channel_number,
        //    rmt_ll_tx_get_interrupt_status(self.channel_number),
        //);
        Ok(())
    }
}

impl<'d> Drop for TxDriver<'d> {
    fn drop(&mut self) {
        let _ = self.stop(); // Force stop the transmission
        self.rmt.set_interrupt_handler(DEFAULT_INTERRUPT_HANDLER);
        // Ensure that the callback is removed when the driver is dropped
        critical_section::with(|cs| {
            ON_FINISH_CALLBACK.borrow(cs).take();
        });
    }
}
