//! At the time of writing, the rmt module esp-hal does not expose all of the necessary
//! functionality for this project.
//!
//! This module contains low-level RMT functions that are current inaccessible.

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{Direction, PulseCode, Tx};

use esp_metadata_generated::property;

#[derive(EnumSetType, Debug)]
pub enum Event {
    /// Triggered when channel n/m does not read or write data correctly.
    ///
    /// For example, the receiver still tries to write data into RAM when the RAM is full.
    /// Or the transmitter still tries to read data from RAM when the RAM is empty.
    Error, // = RMT_LL_EVENT_TX_ERROR
    /// Triggered when the amount of data the transmitter has sent matches the value of
    /// `RMT_CHn_TX_LIM_REG`.
    Threshold, // = RMT_LL_EVENT_TX_THRESH
    /// Triggered when the transmitter has finished transmitting signals.
    End, // = RMT_LL_EVENT_TX_DONE
    /// Triggered when the loop counting reaches the value set by `RMT_TX_LOOP_NUM_CHn` in **continous** TX mode.
    Loop, // = RMT_LL_EVENT_TX_LOOP_END
}

/// This type provides dynamic low-level access to RMT channels.
pub struct RawChannel<Dir: Direction> {
    channel: u8,
    _marker: PhantomData<Dir>,
}

impl<Dir: Direction> RawChannel<Dir> {
    pub unsafe fn conjure(channel: u8) -> Self {
        // TODO: Would have to validate that the channel number is valid for the given direction.

        Self {
            channel,
            _marker: PhantomData,
        }
    }

    pub fn state_machine(&mut self) -> StateMachine<'_, Dir> {
        StateMachine {
            channel: self.channel,
            _marker: PhantomData,
        }
    }

    // Items of interest: - RMT_STATE_CHn in RMT_STATUS_REG
}

impl RawChannel<Tx> {
    #[inline]
    fn listen_for(&mut self, interrupts: impl Into<EnumSet<Event>>, enable: bool) {
        let rmt = RMT::regs();
        let events = interrupts.into();

        rmt.int_ena().modify(|_, w| {
            if events.contains(Event::Error) {
                w.ch_tx_err(self.channel).bit(enable);
            }
            if events.contains(Event::End) {
                w.ch_tx_end(self.channel).bit(enable);
            }
            if events.contains(Event::Threshold) {
                w.ch_tx_thr_event(self.channel).bit(enable);
            }
            if events.contains(Event::Loop) {
                w.ch_tx_loop(self.channel).bit(enable);
            }
            w
        });
    }

    /// Listen for the given interrupts.
    #[inline]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.listen_for(interrupts, true)
    }

    /// Unlisten the given interrupts.
    #[inline]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.listen_for(interrupts, false)
    }

    /// Resets asserted interrupts.
    ///
    /// ### Note
    ///
    /// After an interrupt handler has been invoked, the interrupt handler must clear
    /// the interrupt status (through this function) to avoid being invoked again for
    /// the same event.
    #[inline]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        let rmt = RMT::regs();
        let events = interrupts.into();

        rmt.int_clr().write(|w| {
            if events.contains(Event::Error) {
                w.ch_tx_err(self.channel).set_bit();
            }
            if events.contains(Event::End) {
                w.ch_tx_end(self.channel).set_bit();
            }
            if events.contains(Event::Threshold) {
                w.ch_tx_thr_event(self.channel).set_bit();
            }
            if events.contains(Event::Loop) {
                w.ch_tx_loop(self.channel).set_bit();
            }
            w
        });
    }

    /// Gets asserted interrupts.
    #[inline]
    pub fn interrupts(&mut self) -> EnumSet<Event> {
        let st = RMT::regs().int_st().read();
        let mut events = EnumSet::new();

        if st.ch_tx_err(self.channel).bit() {
            events.insert(Event::Error);
        }

        if st.ch_tx_end(self.channel).bit() {
            events.insert(Event::End);
        }

        if st.ch_tx_thr_event(self.channel).bit() {
            events.insert(Event::Threshold);
        }

        if st.ch_tx_loop(self.channel).bit() {
            events.insert(Event::Loop);
        }

        events
    }

    fn channel_ram_start_offset(&self) -> usize {
        usize::from(self.channel) * property!("rmt.channel_ram_size")
    }

    fn channel_ram_start(&self) -> *mut PulseCode {
        unsafe {
            (property!("rmt.ram_start") as *mut PulseCode).add(self.channel_ram_start_offset())
        }
    }

    /// Provides mutable access to the RMT channel's memory buffer.
    ///
    /// # Safety
    ///
    /// Given that a [`RawChannel`] can be constructed out of thin air using [`RawChannel::conjure`],
    /// it is not guaranteed that this has exclusive access to the memory buffer.
    ///
    /// The caller must ensure that no other code is accessing the memory buffer
    /// while this mutable reference is alive.
    #[inline]
    pub unsafe fn memory(&mut self) -> Memory<'_> {
        let ptr = self.channel_ram_start();
        let len = self.memory_len();

        Memory::from_slice(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
    }

    /// This returns the maximum number of memory blocks allocated to the channel.
    #[inline]
    pub fn memory_blocks(&self) -> u8 {
        RMT::regs()
            .ch_tx_conf0(self.channel as usize)
            .read()
            .mem_size()
            .bits()
    }

    fn memory_len(&self) -> usize {
        self.memory_blocks() as usize * property!("rmt.channel_ram_size")
    }
}

pub struct StateMachine<'a, Dir: Direction> {
    channel: u8,
    _marker: PhantomData<(&'a (), Dir)>,
}

#[derive(Debug)]
pub struct Memory<'a> {
    buffer: &'a mut [PulseCode],
    len: usize,
}

impl<'a> Memory<'a> {
    #[must_use]
    fn from_slice(buffer: &'a mut [PulseCode]) -> Self {
        for i in 0..buffer.len() {
            if buffer[i].is_end_marker() {
                return Self { buffer, len: i };
            }
        }

        unreachable!("rmt buffer does not contain an end marker")
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub fn capacity(&self) -> usize {
        // The last entry is reserved for the end marker
        self.buffer.len() - 1
    }

    /// Clears the memory buffer.
    pub fn clear(&mut self) {
        self.buffer[..self.len].fill(PulseCode::end_marker());
        self.len = 0;
    }
}

// TODO: Some of these flags only apply in certain modes like continuous mode
impl<'a> StateMachine<'a, Tx> {
    //#[inline]
    //pub fn ptr(&self) -> *mut u32 {
    //    let rmt = RMT::regs();
    //    let base = rmt.ch_mem_addr(self.channel as usize).read().mem_start_addr().bits();
    //
    //    base as *mut u32
    //}

    /// This will reset the read address accessed by the transmitter of the channel to the start
    /// of the data.
    #[inline]
    pub fn reset_ptr(&mut self) {
        let rmt = RMT::regs();
        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.mem_rd_rst().set_bit();
            w
        });
        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.mem_rd_rst().clear_bit();
            w
        });
        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.apb_mem_rst().set_bit();
            w
        });
        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.apb_mem_rst().clear_bit();
            w
        });
    }

    /// Reset the current loop count of the state machine to zero.
    #[inline]
    pub fn reset_loop_count(&mut self) {
        let rmt = RMT::regs();
        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.loop_count_reset().set_bit());

        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.loop_count_reset().clear_bit());
    }

    // TODO: Force continuous mode?

    /// Set the number of times the state machine should loop in continuous mode
    /// over the data in memory, before triggering a [`Event::Loop`] interrupt.
    #[inline]
    pub fn set_target_loop_count(&mut self, loop_count: usize) {
        let rmt = RMT::regs();
        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| unsafe { w.tx_loop_num().bits(loop_count as u16) });
    }

    // TODO: The esp-hal version has a dedicated update method that has to be manually called, instead of having the
    //       setters call it automatically. Might be better for performance?
    pub fn update(self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize)
            .modify(|_, w| w.conf_update().set_bit());
    }

    pub fn stop(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.conf_update().bit(true);
            w.tx_stop().bit(true)
        });
    }

    /// Start the state machine.
    #[inline]
    pub fn start(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.conf_update().bit(true);
            w.tx_start().bit(true);
            w
        });
    }
}
