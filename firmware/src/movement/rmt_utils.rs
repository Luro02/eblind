//! At the time of writing, the rmt module esp-hal does not expose all of the necessary
//! functionality for this project.
//!
//! This module contains low-level RMT functions that are current inaccessible.

use core::marker::PhantomData;
use core::ptr::NonNull;

use enumset::{EnumSet, EnumSetType};
use esp_hal::clock::Clocks;
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{Direction, MAX_TX_LOOPCOUNT, PulseCode, Tx};

use esp_metadata_generated::property;
/// The events that can be listened for on an RMT channel.
///
/// If an event is listened for, an interrupt will be triggered when the event occurs,
/// which can be handled in an interrupt handler.
#[derive(EnumSetType, Debug, Hash)]
pub enum Event {
    /// Triggered when a channel does not read or write data correctly.
    ///
    /// For example, the receiver still tries to write data into RAM when the RAM is full.
    /// Or the transmitter still tries to read data from RAM when the RAM is empty.
    Error,
    /// Triggered when the amount of data the transmitter has sent matches the value of
    /// [`StateMachine::set_threshold`]
    Threshold,
    /// Triggered when the transmitter has finished transmitting signals.
    End,
    /// Triggered when the loop counting reaches the value set by [`StateMachine::set_target_loop_count`]
    /// in **loop** TX mode.
    Loop,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum State {
    Idle,
    Send,
    ReadMemory,
    Receive,
    Wait,
    Unknown(u8),
}

impl From<u8> for State {
    // Information is from https://github.com/espressif/esp-idf/issues/1175#issuecomment-428055913
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Idle,
            1 => Self::Send,
            2 => Self::ReadMemory,
            3 => Self::Receive,
            4 => Self::Wait,
            other => Self::Unknown(other),
        }
    }
}

/// Represents an RMT channel, which can be used for transmitting or receiving data.
///
/// This type provides dynamic low-level access to an RMT channel.
pub struct RawChannel<Dir: Direction> {
    channel: u8,
    _marker: PhantomData<Dir>,
}

impl<Dir: Direction> RawChannel<Dir> {
    /// Constructs a `RawChannel` for the given channel number.
    ///
    /// # Safety
    ///
    /// It is not checked that the channel number is valid for the set [`Dir`],
    /// that the channel number is valid for the given hardware,
    /// or that the channel number is not already in use.
    pub unsafe fn conjure(channel: u8) -> Self {
        // TODO: Should validate that the channel number is valid for the given direction.
        // TODO: It must guarantee that the channel number exists, otherwise it might access invalid memory

        Self {
            channel,
            _marker: PhantomData,
        }
    }

    /// Provides access to the state machine of the channel.
    ///
    /// The state machine can be used to configure the behavior of the channel,
    /// such as starting and stopping, and setting the loop count in loop mode.
    #[inline]
    pub fn state_machine(&mut self) -> StateMachine<'_, Dir> {
        unsafe { StateMachine::new(self.channel) }
    }

    // Items of interest: - RMT_STATE_CHn in RMT_STATUS_REG
}

/// Returns the frequency of the RMT working clock in Hz.
pub fn rmt_sclk_frequency() -> f32 {
    let apb_clock_frequency = Clocks::get().apb_clock.as_hz() as f32;
    let sysconf = RMT::regs().sys_conf().read();

    let selected_clock = sysconf.sclk_sel().bits();

    if selected_clock != 1 {
        panic!("Only APB clock is supported currently");
    }

    // The working clock is obtained by dividing the selected source clock with the divider:
    let mut base_divider = sysconf.sclk_div_num().bits() as f32 + 1.0;

    let sclk_div_a = sysconf.sclk_div_a().bits();
    let sclk_div_b = sysconf.sclk_div_b().bits();

    if sclk_div_b != 0 {
        base_divider += (sclk_div_a as f32) / (sclk_div_b as f32);
    }

    apb_clock_frequency / base_divider
}

/// Returns the resolution of the channel in Hz.
pub fn channel_resolution(channel_number: u8) -> f32 {
    // In addition to that, each channel can configure its own divider:
    let mut channel_divider = RMT::regs()
        .ch_tx_conf0(channel_number as usize)
        .read()
        .div_cnt()
        .bits() as usize;

    // A value of 0 represents a divider of 256:
    if channel_divider == 0 {
        channel_divider = 256;
    }

    rmt_sclk_frequency() / channel_divider as f32
}

pub fn assert_timing_requirements(
    channel_number: u8,
    pulse_codes: &[PulseCode],
    should_autostop: bool,
) {
    let apb_clock_frequency = Clocks::get().apb_clock.as_hz() as f32;

    // The timing requirements for any pulse that is not an end marker:
    for pulse in pulse_codes.iter().filter(|pulse| !pulse.is_end_marker()) {
        assert_timing_requirements_any(channel_number, pulse.length1());
        assert_timing_requirements_any(channel_number, pulse.length2());
    }

    // For autostop there is an extra requirement:
    if should_autostop {
        let mut period = 0;

        for pulse in pulse_codes
            .iter()
            // TODO: This ignores the other half period if only half of the pulse code is an end marker
            .take_while(|pulse| !pulse.is_end_marker())
        {
            period += pulse.length1() as u32 + pulse.length2() as u32;
        }

        // TODO: This is kind of questionable? Maybe a newer technical reference manual like the P4 one will clarify this?
        //       or maybe an older one like the original for the ESP32?
        let minimum = 6.0 * 1.0 / apb_clock_frequency + 12.0 * 1.0 / rmt_sclk_frequency();

        let resolution = channel_resolution(channel_number);
        log::info!(
            "Minimum period: {}ns < {}ns, resolution: {resolution}, period: {period} ticks",
            minimum * 1_000_000_000.0,
            (period as f32 * 1.0 / resolution) * 1_000_000_000.0,
        );
        //assert!(minimum < period as f32 * 1.0 / get_resolution(channel_number));
    }
}

pub fn assert_timing_requirements_any(channel_number: u8, period: u16) {
    let apb_clock_frequency = Clocks::get().apb_clock.as_hz() as f32;
    let minimum = 3.0 * 1.0 / apb_clock_frequency + 5.0 * 1.0 / rmt_sclk_frequency();
    log::info!("Minimum period: {}ns", minimum * 1_000_000_000.0);
    assert!(minimum < period as f32 * 1.0 / channel_resolution(channel_number))
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
    /// # Note
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

    /// The APB bus is used to access the channel RAM.
    ///
    /// The RAM can be accessed through direct addressing (fifo mode disabled)
    /// or through a FIFO (fifo mode enabled).
    ///
    /// In FIFO mode the channel RAM is accessed via a fixed address obtained from
    /// [`RawChannel::fifo_address`].
    pub fn set_use_fifo(&mut self, use_fifo: bool) {
        let rmt = RMT::regs();

        rmt.sys_conf().modify(|_, w| {
            w.apb_fifo_mask().bit(!use_fifo);
            w
        });
    }

    pub fn fifo_address(&self) -> u32 {
        let rmt = RMT::regs();

        rmt.chdata(self.channel as usize).read().bits()
    }

    // TODO: This is only for non-fifo mode
    /// Provides mutable access to the RMT channel's memory buffer.
    #[inline]
    pub unsafe fn memory(&mut self) -> Memory<'_> {
        // The APB bus is used to access the channel RAM. There are the following modes to access the channel RAM:
        // 1. use NONFIFO mode (Direct Address)
        // 2. use FIFO mode
        //
        // This is configured by the RMT_APB_FIFO_MASK
        unsafe { Memory::from_ptr(self.channel_ram_start(), self.capacity()) }
    }

    const fn channel_ram_start_offset(&self) -> usize {
        self.channel as usize * property!("rmt.channel_ram_size")
    }

    const fn channel_ram_start(&self) -> NonNull<PulseCode> {
        // SAFETY: The pointer is guaranteed to be non-null, because it refers to a hardware address that should
        //         always be present on the device.
        let ptr = unsafe { NonNull::new_unchecked(property!("rmt.ram_start") as *mut PulseCode) };

        // SAFETY: The address is calculated like described in the technical reference manual.
        //         For the calculation, only the channel number can be varied, which is guaranteed
        //         to be valid by the construction of this type.
        unsafe { ptr.add(self.channel_ram_start_offset()) }
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

    /// This returns the maximum number of pulse codes that can be stored in the channel RAM.
    #[inline]
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.memory_blocks() as usize * property!("rmt.channel_ram_size")
    }
}

// TODO: Look at the read write offset fields, described in the above linked issue comment.
//       I think those are the current pointer position of the state machine?

/// Represents the memory buffer of an RMT channel, which is used to hold the
/// data to be transmitted or received.
///
/// By default the RAM is divided into eight 48 x 32-bit blocks. Exact numbers
/// might differ between chips.
/// One block is allocated to each channel, e.g. channel0 gets block 0, channel1 gets block 1, etc.
///
/// If the data to be transmitted or received exceeds memory size, users can configure the channel
/// - to enable wrap mode through [`StateMachine::set_wrap_mode`]
/// - or to use more blocks by configuring the channel's memory blocks by setting ???
///
/// TODO: currently no way to change allocated memory blocks for a channel
///
/// Using more blocks allows a channel to use the memory of subsequent channels, so those
/// channels can not be used while their blocks are occupied.
///
/// For example if channel0 is configured to use block 0 and block 1, then channel1 can not be used
/// due to its block being occupied by channel0, but channel2 can be used because its block is not used.
#[derive(Debug)]
pub struct Memory<'a> {
    // The following things have to be considered when using
    // NonNull instead of a raw pointer:
    // - The pointer must never be null, even if never dereferenced.
    // - NonNull<T> is covariant over T
    //
    // The former is checked at construction, and **should** never be the case anyway,
    // otherwise we would have a big problem when the hardware address is null.
    //
    // The latter is not a problem, because `T` is always `PulseCode`.
    ptr: NonNull<PulseCode>,
    len: usize,
    _marker: PhantomData<&'a mut ()>,
    // TODO: For the future:
    //
    // According to the technical reference manual,
    // access to the modules/peripherals (excluding RTC FAST and SLOW memory) must be
    // word aligned.
    //
    // This should imply that the pointer is aligned as well.
    //
    // If this is the case, we could convert the pointer to a slice of `PulseCode`.
    //
    // TODO: Technically the memory is MaybeUninit, but the applicability is questionable
}

impl<'a> Memory<'a> {
    pub(crate) unsafe fn from_ptr(ptr: NonNull<PulseCode>, len: usize) -> Self {
        Self {
            ptr,
            len,
            _marker: PhantomData,
        }
    }

    pub fn as_slice(&self) -> &[PulseCode] {
        // TODO maybe safe
        unsafe { core::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }

    /// Returns a mutable pointer to the underlying memory buffer.
    ///
    /// The pointer is guaranteed to be non-null.
    #[must_use]
    pub fn as_mut_ptr(&self) -> NonNull<PulseCode> {
        self.ptr
    }

    // TODO: I am not happy with this API
    pub fn write_all(&mut self, index: usize, values: &[PulseCode]) {
        if values.len() + index > self.capacity() {
            panic!(
                "The memory buffer can not hold more than {} pulse codes, but tried to write {} at index {}",
                self.capacity(),
                values.len(),
                index
            );
        }

        for (i, value) in values.iter().enumerate() {
            unsafe {
                self.as_mut_ptr().add(index + i).write_volatile(*value);
            }
        }
    }

    #[must_use]
    pub fn capacity(&self) -> usize {
        self.len
    }
}

/// Each channel has its own state machine, which processes the pulse codes in the channel RAM,
/// and controls the transmission and reception of signals.
///
/// The state machine can be configured to operate in different modes:
/// - Regular mode (wrap mode and loop mode disabled)
/// - Wrap mode
/// - Loop mode
///
/// In regular TX mode, the transmitter of channel n starts reading and sending pulse codes from the
/// channel memory when the start bit is set. When the transmitter encounters a [`PulseCode::end_marker()`]
/// it will stop transmitting and set the end interrupt status bit.
///
/// The tx stop bit can be set as well to immediately stop the transmission.
///
/// After the transmission is stopped or ends, the transmitter will return to idle state and wait for the
/// next start signal.
///
/// Wrap mode can be activated through [`StateMachine::set_wrap_mode`],
/// and loop mode can be activated through [`StateMachine::set_loop_mode`].
pub struct StateMachine<'a, Dir: Direction> {
    channel: u8,
    _marker: PhantomData<(&'a mut (), Dir)>,
}

impl<'a, Dir: Direction> StateMachine<'a, Dir> {
    pub(crate) unsafe fn new(channel: u8) -> Self {
        Self {
            channel,
            _marker: PhantomData,
        }
    }
}

// copy-paste from esp-hal rmt code with minor adjustments
macro_rules! max_from_register_spec {
    ($typ:ty, $reg:ident, $spec:ident, $w:ident) => {{
        use esp32s3::rmt::$reg as reg;
        type Spec = reg::$spec;
        const WIDTH: u8 = reg::$w::<Spec>::WIDTH;

        const _: () = if WIDTH as u32 > <$typ>::BITS {
            ::core::panic!("Unexpectedly large register WIDTH");
        };

        // Fits into $ty according to the assertion above
        ((1u32 << WIDTH) - 1) as $typ
    }};
}

#[allow(missing_docs)]
pub const MAX_TX_THRESHOLD: u16 = max_from_register_spec!(u16, ch_tx_lim, CH_TX_LIM_SPEC, TX_LIM_W);

impl<'a> StateMachine<'a, Tx> {
    /// This will reset the read address accessed by the transmitter of the channel to the start
    /// of the data.
    #[inline]
    pub fn reset_ptr(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.mem_rd_rst().set_bit();
            w.apb_mem_rst().set_bit();
            w
        });

        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.mem_rd_rst().clear_bit();
            w.apb_mem_rst().clear_bit();
            w
        });

        self.update();
    }

    /// Enables or disables wrapping around the channel RAM when the end of the RAM is reached
    /// without encountering a [`PulseCode::end_marker`].
    ///
    /// When wrap mode is enabled, the transmitter will send the data from RAM, if it encounters
    /// a [`PulseCode::end_marker`] it will stop the transmission, and assert a [`Event::End`]
    /// interrupt. (Here it behaves like when wrap mode is disabled).
    ///
    /// If it does not encounter an end marker, it will wrap around to the beginning of the RAM and continue
    /// transmitting.
    ///
    /// To be notified when data has been transmitted, the [`Event::Threshold`] interrupt can be listened for,
    /// which is triggered when the amount of data transmitted is equal to the value set in [`Self::set_tx_threshold`].
    ///
    /// For example, assuming the channel RAM can hold 48 pulse codes and one wants to transmit 100 pulse codes:
    ///
    /// 1. The RAM is filled with the first 48 pulse codes of the data to be transmitted,
    ///    the wrap mode is enabled, and the limit is set to 24 for the threshold interrupt.
    /// 2. The transmitter is started, and it starts transmitting the data in RAM.
    /// 3. When the transmitter has sent 24 pulse codes, it triggers the threshold interrupt,
    /// 4. The interrupt handler then fills the RAM with the next 24 pulse codes, given that the transmitter has already
    ///    sent the first 24 pulse codes, it keeps track of where it currently is in the RAM.
    /// 5. After it reaches the end of the RAM, it encounters no end marker, therefore starts again from the beginning
    ///    of the RAM.
    /// 6. At the same time, it reaches the threshold again after reaching the end, because the threshold is set to 24, and
    ///    the ram fits 48 pulse codes -> the interrupt handler triggers again, and fills the RAM with the next 24 pulse codes
    ///    **Important**: It can only overwrite the last 24 pulse codes, because the first 24 pulse codes are still being transmitted
    /// 7. The next time it transmitted 24 pulse codes, and the interrupt handler is triggered again, the last 4 pulse codes
    ///    would be transmitted and then the end marker, which would stop the transmission.
    ///
    /// Note that according to the technical reference manual, when the RAM is accessed through DMA, the wrap mode
    /// is not necessary.
    ///
    /// # What is the difference between wrap mode and loop mode?
    ///
    /// In loop mode, the transmitter wraps when an end marker is encountered, in wrap mode it will stop.
    /// The loop mode triggers an interrupt after a number of loops while the wrap mode triggers an interrupt
    /// after a number of transmitted pulse codes.
    #[inline]
    pub fn set_wrap_mode(&mut self, should_wrap: bool) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize)
            .modify(|_, w| w.mem_tx_wrap_en().bit(should_wrap));

        self.update();
    }

    /// Sets the number of [`PulseCode`]s that have to be transmitted after which the
    /// [`Event::Threshold`] interrupt is triggered.
    ///
    /// After the interrupt is triggered, the counter is reset, therefore the next interrupt
    /// will be triggered when number of transmited pulse codes equals the threshold.
    ///
    /// The threshold interrupt is independent of the modes, it will trigger in wrap mode,
    /// loop mode, and normal mode.
    ///
    /// # Panics
    ///
    /// If the threshold is larger than the maximum value that can be set in the register,
    /// this function will panic.
    ///
    /// The maximum value should be `2.pow(9) - 1 = 511`, but it might differ depending on the chip.
    /// See [`MAX_TX_THRESHOLD`] for the actual maximum value.
    ///
    /// A value of 0 is not sensible, because the interrupt will be triggered repeatedly without the transmitter
    /// making progress, therefore this function will panic (in debug mode) if the threshold is set to 0 as well.
    #[inline]
    pub fn set_tx_threshold(&mut self, threshold: u16) {
        let rmt = RMT::regs();
        // This does not cause undefined behavior, but the microcontroller will always interrupt
        // and never finish with transmitting, which is not desirable.
        debug_assert!(
            threshold > 0,
            "Threshold {threshold} should be greater than 0"
        );

        rmt.ch_tx_lim(self.channel as usize).modify(|_, w| {
            let max_value = 2u16.pow(w.tx_lim().width() as u32) - 1;
            assert!(
                threshold <= max_value,
                "Threshold {threshold} must be less than or equal to {max_value}"
            );

            unsafe { w.tx_lim().bits(threshold) }
        });

        self.update();
    }

    /// Reset the current loop count of the state machine to zero.
    ///
    /// This counter is incremented when [`Self::set_loop_mode`] is enabled,
    /// and the transmitter encounters an end marker.
    #[inline]
    pub fn reset_loop_count(&mut self) {
        let rmt = RMT::regs();
        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.loop_count_reset().set_bit());
        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.loop_count_reset().clear_bit());
    }

    /// Enables or disables loop mode for the transmitter of the channel.
    ///
    /// In this mode, the transmitter sends the pulse codes from RAM in loops.
    /// When it reaches the end of the RAM **or** an end marker is encountered, it
    /// will restart from the **beginning** of the RAM.
    ///
    /// The end marker is not necessary for wrapping around, but the loop counter
    /// is only incremented when an end marker is encountered.
    ///
    /// If an end marker is set in the middle of the RAM, it will not read past it,
    /// instead wrapping around to the beginning of the RAM and incrementing the loop
    /// count.
    ///
    /// After the loop count reaches the target loop count set by [`StateMachine::set_target_loop_count`],
    /// it will assert the [`Event::Loop`] interrupt and restart the loop count from zero.
    ///
    /// If [`Self::set_loop_stop`] is set, it will stop after the next loop interrupt is triggered.
    ///
    /// [`Self::stop`] can be used to stop the transmission immediately.
    ///
    /// TODO: I do not think this condition is enforced by esp-hal code, but it should really be:
    ///
    /// In an end-marker if its period[14:0] is 0, then the period of the previous data must satisfy:
    ///
    /// 6 * T_apb_clk + 12 * T_rmt_sclk < period * T_clk_div
    ///
    /// For the other data the period condition is
    ///
    /// 3 * T_apb_clk + 5 * T_rmt_sclk < period * T_clk_div
    #[inline]
    pub fn set_loop_mode(&mut self, should_enable: bool) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize)
            .modify(|_, w| w.tx_conti_mode().bit(should_enable));

        self.update();
    }

    /// Set the number of times the state machine should loop over the data in memory,
    /// before triggering a [`Event::Loop`] interrupt.
    ///
    /// A `loop_count` of `0` will disable the loop counting, resulting in an infinite loop
    /// without triggering the loop interrupt, even if the loop mode is enabled.
    ///
    /// For this to work, the loop mode must be enabled through [`Self::set_loop_mode`].
    #[inline]
    pub fn set_target_loop_count(&mut self, loop_count: u16) {
        let rmt = RMT::regs();

        debug_assert!(
            loop_count <= MAX_TX_LOOPCOUNT,
            "Loop count {loop_count} must be less than or equal to {MAX_TX_LOOPCOUNT}"
        );
        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| unsafe { w.tx_loop_num().bits(loop_count) });

        self.enable_loop_count(loop_count > 0);

        self.update();
    }

    #[inline]
    pub fn enable_loop_count(&mut self, should_enable: bool) {
        let rmt = RMT::regs();

        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.tx_loop_cnt_en().bit(should_enable));
    }

    pub fn state(&mut self) -> State {
        let rmt = RMT::regs();

        State::from(
            rmt.ch_tx_status(self.channel as usize)
                .read()
                .state()
                .bits(),
        )
    }

    /// Set whether the state machine should stop after the next loop interrupt is triggered.
    ///
    /// This will only have an effect if the loop mode is enabled, and the target loop count is
    /// greater than zero.
    pub fn set_loop_auto_stop(&mut self, should_stop: bool) {
        let rmt = RMT::regs();

        rmt.ch_tx_lim(self.channel as usize)
            .modify(|_, w| w.loop_stop_en().bit(should_stop));
    }

    fn update(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize)
            .modify(|_, w| w.conf_update().set_bit());
    }

    /// Immediately stop the transmission of the channel.
    pub fn stop(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize)
            .modify(|_, w| w.tx_stop().bit(true));

        self.update();
    }

    /// Start the state machine.
    #[inline]
    pub fn start(&mut self) {
        let rmt = RMT::regs();

        rmt.ch_tx_conf0(self.channel as usize).modify(|_, w| {
            w.tx_start().bit(true);
            w
        });

        self.update();
    }
}
