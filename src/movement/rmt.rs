use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

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

#[ram]
static REMAINING_LOOPS: AtomicUsize = AtomicUsize::new(0);

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
    let mut channel: RawChannel<Tx> = unsafe { RawChannel::conjure(0) };

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
            return;
        }

        let mut state_machine = channel.state_machine();

        let this_loop_count = min!(MAX_TX_LOOPCOUNT as usize, remaining_loops);
        state_machine.set_target_loop_count(this_loop_count);
        REMAINING_LOOPS.fetch_sub(this_loop_count, Ordering::SeqCst);
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
    channel: Option<Channel<'d, Blocking, Tx>>,
    pending: Option<ContinuousTxTransaction<'d>>,
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

        let channel_number = 0;
        Ok(Self {
            rmt: rmt_copy,
            channel_number,
            channel: Some(
                {
                    match channel_number {
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
        let sysconf = RMT::regs().sys_conf().read();

        let selected_clock = sysconf.sclk_sel().bits();

        if selected_clock != 1 {
            panic!("Only APB clock is supported currently");
        }

        let base_clock_rate = Rate::from_mhz(80);

        // The working clock is obtained by dividing the selected source clock with the divider:
        let mut base_divider = sysconf.sclk_div_num().bits() as f32 + 1.0;

        let sclk_div_a = sysconf.sclk_div_a().bits();
        let sclk_div_b = sysconf.sclk_div_b().bits();

        if sclk_div_b != 0 {
            base_divider += (sclk_div_a as f32) / (sclk_div_b as f32);
        }

        // In addition to that, each channel can configure its own divider:
        let mut channel_divider = RMT::regs()
            .ch_tx_conf0(self.channel_number as usize)
            .read()
            .div_cnt()
            .bits() as usize;

        // A value of 0 represents a divider of 256:
        if channel_divider == 0 {
            channel_divider = 256;
        }

        Rate::from_hz(
            ((base_clock_rate.as_hz() as f32 / base_divider) / channel_divider as f32) as u32,
        )
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
        let mut max_repeat = buffer.len() / data.len();

        // Find the maximum number of repetitions that when repeated will be exactly total_repetitions
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
        let Some(channel) = self.channel.take() else {
            log::error!("Can not send while another transmission is ongoing");
            return Err(rmt::Error::TransmissionError);
        };

        // Technically could be increased to more than CHANNEL_RAM_SIZE if multiple blocks are used,
        // but then it wouldn't be a const size anymore.

        let mut payload_buffer = [{ PulseCode::end_marker() }; CHANNEL_RAM_SIZE];

        if payload_buffer.len() < data.len() || data.is_empty() {
            return Err(rmt::Error::InvalidDataLength);
        }

        // It is necessary that there is an end-marker in the data for the loop counting to work correctly.
        if payload_buffer.len() == data.len() && !data[data.len() - 1].is_end_marker() {
            return Err(rmt::Error::EndMarkerMissing);
        }

        // Treat a count of None as a single transmission:
        let mut total_loops = count.unwrap_or(1);

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

        // The payload end is either the length of the data + 1 (for the end marker) or the length of the payload buffer
        // whichever is smaller.
        // let payload_end = min!(data.len(), payload_buffer.len());
        // debug_assert!(payload_buffer[payload_end].is_end_marker());

        // If the loops exceed the maximum loop count, they have to be sent in batches.
        let loop_batch_size = min!(MAX_TX_LOOPCOUNT as usize, total_loops) as u16;
        total_loops -= loop_batch_size as usize;

        REMAINING_LOOPS.store(total_loops, Ordering::SeqCst);

        let channel_number = self.channel_number;
        // Start the transmission in continuious tx mode:
        //
        // In this mode the transmitter sends the pulse codes from RAM in loops:
        // - If an end-marker is encountered, the transmitter starts transmitting from the first data of the
        //   channel's RAM again.
        // - If no end-marker is encountered, the transmitter starts transmitting from the first data again
        //   after the last data is transmitted.
        //
        // The loop counter is only incremented when an end-marker is encountered
        // -> it is necessary to include an end-marker in the data for the loop counting to work correctly.
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

        let mut channel = unsafe { RawChannel::<Tx>::conjure(channel_number) };

        //Event::End | Event::Error | Event::Threshold | Event::Loop,
        channel.listen(Event::Error | Event::Loop);

        Ok(())
    }

    pub async fn stop_wait(&mut self) -> Result<(), rmt::Error> {
        self.stop()
    }

    fn stop(&mut self) -> Result<(), rmt::Error> {
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
    }
}
