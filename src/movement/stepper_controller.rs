use std::fmt;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use esp_idf_hal::gpio::{AnyInputPin, InputOutput, InputPin, OutputPin, PinDriver, Pull};
use esp_idf_hal::pcnt::config::{
    ChannelConfig, ChannelEdgeAction, ChannelLevelAction, GlitchFilterConfig, UnitConfig,
};
use esp_idf_hal::pcnt::PcntUnitDriver;
use esp_idf_hal::rmt::config::{Loop, TransmitConfig};
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, Symbol};

use bincode::{Decode, Encode};

use crate::environment::Environment;
use crate::matter::Position;
use crate::movement::custom_tx_driver::CustomTxChannelDriver;
use crate::movement::stepper_driver::StepperDriver;
use crate::storage::Storage;
use crate::utils::split_pin;

#[derive(Debug, Default)]
pub struct UnitPosition {
    value: AtomicUsize,
}

impl Encode for UnitPosition {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.value.load(Ordering::SeqCst).encode(encoder)
    }
}

impl<C> Decode<C> for UnitPosition {
    fn decode<D: bincode::de::Decoder>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let value = usize::decode(decoder)?;
        Ok(Self {
            value: AtomicUsize::new(value),
        })
    }
}

impl UnitPosition {
    pub fn new(value: usize) -> Self {
        Self {
            value: AtomicUsize::new(value),
        }
    }

    pub fn max() -> Self {
        Self::new(usize::MAX)
    }

    pub fn from_position(position: Position, max: &UnitPosition) -> Self {
        Self::new(position.map_to(max.to_value() as f64) as usize)
    }

    pub fn add_assign(&self, value: isize) {
        if value >= 0 {
            self.value.fetch_add(value as usize, Ordering::SeqCst);
        } else {
            self.value.fetch_sub((-value) as usize, Ordering::SeqCst);
        }
    }

    pub fn set(&self, position: &UnitPosition) {
        self.value.store(position.to_value(), Ordering::SeqCst);
    }

    pub fn to_value(&self) -> usize {
        self.value.load(Ordering::SeqCst)
    }

    #[must_use]
    pub fn to_position(&self, max: &UnitPosition) -> Position {
        Position::new(self.to_value() as f64 / max.to_value() as f64)
    }
}

impl Clone for UnitPosition {
    fn clone(&self) -> Self {
        Self::new(self.to_value())
    }
}

impl PartialEq for UnitPosition {
    fn eq(&self, other: &Self) -> bool {
        self.to_value() == other.to_value()
    }
}

impl fmt::Display for UnitPosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.to_value())
    }
}

const PCNT_LOW_LIMIT: i32 = -(i16::MAX as i32);
const PCNT_HIGH_LIMIT: i32 = -PCNT_LOW_LIMIT;

/// Type to control a stepper motor.
pub struct StepperController<'d> {
    current_position: Arc<UnitPosition>,
    max_position: UnitPosition,
    target: Option<UnitPosition>,
    stepper_driver: StepperDriver<'d>,
    pcnt: PcntUnitDriver<'d>,
    tx_driver: CustomTxChannelDriver<'d>,
    has_position: bool,
    dir_pin: PinDriver<'d, InputOutput>,
}

fn build_pcnt<'d>(
    step_pin: Option<impl InputPin + 'd>,
    dir_pin: Option<impl InputPin + 'd>,
    current_position: Arc<UnitPosition>,
) -> anyhow::Result<PcntUnitDriver<'d>> {
    let should_invert = Environment::should_invert_direction();

    let mut pcnt = PcntUnitDriver::new(&UnitConfig {
        low_limit: PCNT_LOW_LIMIT,
        high_limit: PCNT_HIGH_LIMIT,
        intr_priority: 0,
        accum_count: false,
        ..Default::default()
    })?;

    pcnt.add_channel(
        step_pin,
        dir_pin,
        &ChannelConfig {
            invert_edge_input: false,
            invert_level_input: should_invert,
            virt_edge_io_level: false,
            virt_level_io_level: false,
            ..Default::default()
        },
    )?
    // when signal goes from low to high, increase counter, but do nothing when it goes back to low
    .set_edge_action(ChannelEdgeAction::Increase, ChannelEdgeAction::Hold)?
    .set_level_action(ChannelLevelAction::Inverse, ChannelLevelAction::Keep)?;

    pcnt.set_glitch_filter(Some(&GlitchFilterConfig {
        max_glitch: Duration::from_nanos(2000), // 2us is the minimum pulse width generated
        ..Default::default()
    }))?;

    // TODO: It could use accum_count here, should be faster
    pcnt.subscribe(move |event| {
        let watch_point = event.watch_point_value;
        if watch_point == PCNT_HIGH_LIMIT || watch_point == PCNT_LOW_LIMIT {
            // reached limit -> will reset to 0 afterwards
            current_position.add_assign(watch_point as isize);
        }
    })?;

    pcnt.enable()?;

    for point in [PCNT_LOW_LIMIT, PCNT_HIGH_LIMIT] {
        pcnt.add_watch_point(point)?;
    }
    pcnt.clear_count()?;
    pcnt.start()?;

    Ok(pcnt)
}

impl<'d> StepperController<'d> {
    pub fn new<UART: esp_idf_hal::uart::Uart + 'd>(
        uart: UART,
        step_pin: impl InputPin + OutputPin + 'd,
        dir_pin: impl InputPin + OutputPin + 'd,
        storage: &Storage,
        tx: impl OutputPin + 'd,
        rx: impl InputPin + 'd,
        enable_pin: impl InputPin + OutputPin + 'd,
    ) -> anyhow::Result<Self> {
        let storage_position = storage.current_position()?;
        let has_position = storage_position.is_some();
        let current_position = storage_position.unwrap_or(UnitPosition::new(usize::MAX / 2));

        let max_position = storage.max_position()?.unwrap_or(UnitPosition::max());

        // We need to split the pins into input and output pins,
        // the inputs are used by the PCNT to track the position
        // and the step output pin is used by the RMT peripheral
        // to drive the stepper motor.

        let (step_in, step_out) = split_pin(step_pin, Pull::Down)?;

        // The dir_out pin will be driven manually through the PinDriver -> it has to be used like this
        let dir_out = PinDriver::input_output(dir_pin, Pull::Down)?;
        let dir_in = unsafe { AnyInputPin::steal(dir_out.pin()) };

        let current_position = Arc::new(current_position);
        Ok(Self {
            current_position: current_position.clone(),
            max_position,
            stepper_driver: StepperDriver::new(
                Environment::uart_baud_rate(),
                uart,
                tx,
                rx,
                enable_pin,
            )?,
            target: None,
            pcnt: build_pcnt(Some(step_in), Some(dir_in), current_position)?,
            tx_driver: CustomTxChannelDriver::new(step_out)?,
            dir_pin: dir_out,
            has_position,
        })
    }

    pub fn has_max_position(&self) -> bool {
        self.max_position != UnitPosition::max()
    }

    pub fn has_position(&self) -> bool {
        self.has_position
    }

    pub async fn target_position(&self) -> Option<Position> {
        self.target_unit_position()
            .await
            .as_ref()
            .map(|pos| pos.to_position(&self.max_position))
    }

    async fn target_unit_position(&self) -> Option<UnitPosition> {
        self.target.clone()
    }

    async fn internal_update_target_position(
        &mut self,
        new_target: Option<UnitPosition>,
    ) -> anyhow::Result<()> {
        // If the target is None, just set it to None, there is nothing more to do.
        let Some(target) = new_target else {
            self.target = None;
            return Ok(());
        };

        assert!(self.is_moving() == false);

        // Based on the current position, it will calculate the number of steps it has to move:
        let current_position = self.current_unit_position();
        let steps_to_move = target.to_value() as i64 - current_position.to_value() as i64;
        log::info!(
            "Moving from {current_position} to {target} (pos -> target), steps: {steps_to_move}"
        );

        // Sanity check, shouldn't happen, but just in case:
        let new_position = current_position.to_value() as i64 + steps_to_move;
        if new_position > self.max_position.to_value() as i64 || new_position < 0 {
            log::warn!(
                "Target position {target} will be out of bounds (max: {}, min: 0).",
                self.max_position
            );
            return Ok(());
        }

        // If there is nothing to do, just return:
        if steps_to_move == 0 {
            self.target = None;
            return Ok(());
        }

        // Make sure the stepper driver is enabled otherwise it won't move:
        self.stepper_driver.enable().await?;

        let should_invert_direction =
            (steps_to_move.signum() == -1) ^ Environment::should_invert_direction();

        if should_invert_direction {
            self.dir_pin.set_high()?;
        } else {
            self.dir_pin.set_low()?;
        }

        let steps_to_move = steps_to_move.unsigned_abs();
        log::info!("Moving {steps_to_move:?} steps");

        let movement_config = Environment::resolve_movement_config();

        let rpm = movement_config.max_rounds_per_second as f64 * 60.0;
        let microsteps_per_step = movement_config.microsteps_per_step as f64;
        let steps_per_revolution = movement_config.full_steps_per_rotation as f64;

        // TODO: Why not use PulseTicks here?
        // Calculate period in nanoseconds for higher precision with RMT ticks
        // 1s = 1_000_000_000ns
        let period_ns = (60.0 * 1_000_000_000.0
            / (rpm * steps_per_revolution * microsteps_per_step))
            .max(1000.0); // Min period 1000ns = 1us

        // Define pulse width (e.g., 2-5 us) in nanoseconds
        let pulse_width_ns: u32 = 2_000; // 2 microseconds = 2000 nanoseconds

        // Calculate delay between pulses in nanoseconds
        let delay_ns = if period_ns > pulse_width_ns as f64 {
            (period_ns - pulse_width_ns as f64).max(2000.0) as u32 // Ensure minimum delay (e.g., 2us)
        } else {
            2_000 // Minimum delay if period is very short
        };

        let ticks_hz = self.tx_driver.resolution(); // Get the actual RMT counter clock frequency
        let ns_per_tick = 1_000_000_000u64 / ticks_hz.0 as u64;

        let high_ticks = PulseTicks::new((pulse_width_ns as u64 / ns_per_tick) as u16)?;
        let low_ticks = PulseTicks::new((delay_ns as u64 / ns_per_tick) as u16)?;

        let signal = &[Symbol::new(
            Pulse::new(PinState::High, high_ticks),
            Pulse::new(PinState::Low, low_ticks),
        )];

        assert!(
            steps_to_move <= u32::MAX as u64,
            "Requested to move too many steps at once."
        );

        // First ensure that there is no pending movement:
        self.tx_driver.disable()?;

        self.tx_driver
            .start_send(
                signal,
                &TransmitConfig {
                    loop_count: Loop::Count(steps_to_move as u32),
                    queue_non_blocking: true,
                    ..Default::default()
                },
            )
            .await?;

        self.target = Some(target);

        Ok(())
    }

    pub async fn update_target_position(
        &mut self,
        new_target: Option<Position>,
    ) -> anyhow::Result<()> {
        // If the target did not change, there is nothing to do:
        if self.target_position().await == new_target {
            log::info!("Target position is already set to the requested value, doing nothing.");
            return Ok(());
        }

        log::info!(
            "Updating target position to {new_target:?}, currently at {}",
            self.current_unit_position()
        );

        let target =
            new_target.map(|target| UnitPosition::from_position(target, &self.max_position));
        log::info!("Converted percentage target to: {target:?}");

        // Make sure to stop the stepper motor if it is currently moving?:
        if self.is_moving() {
            log::info!(
                "Stepper is currently moving from {} to {:?}, sending stop signal first.",
                self.current_unit_position(),
                &self.target
            );
            self.stop().await?;
        }

        self.internal_update_target_position(target).await
    }

    /// Returns true if the stepper motor is currently moving.
    pub fn is_moving(&self) -> bool {
        self.target.is_some() && &self.target != &Some(self.current_unit_position())
    }

    pub fn driver(&mut self) -> &mut StepperDriver<'d> {
        &mut self.stepper_driver
    }

    fn current_unit_position(&self) -> UnitPosition {
        // current_position only contains the overflow values,
        // the pcnt contains the remaining steps:
        let result = (*self.current_position).clone();
        let pcnt_count = self.pcnt.get_count().unwrap();
        result.add_assign(pcnt_count as isize);

        result
    }

    /// Returns the current position of the stepper motor.
    ///
    /// It is a value between (inclusive range) 0.0 (min) and 1.0 (max).
    pub fn current_position(&self) -> Position {
        self.current_unit_position().to_position(&self.max_position)
    }

    /// Removes the target, stopping the stepper motor.
    ///
    /// It will wait for the stepper motor to actually stop moving.
    pub async fn stop(&mut self) -> anyhow::Result<()> {
        if !self.is_moving() {
            log::info!(
                "Stepper is not moving: current_position: {}",
                self.current_unit_position()
            );
            return Ok(());
        }

        log::info!(
            "Stopping stepper motor, currently at {}, moving to {:?}",
            self.current_unit_position(),
            self.target_unit_position().await
        );

        // This will cancel all pending step movements:
        self.tx_driver
            .disable()
            .expect("Failed to stop RMT transmission");

        // Update the target position to None to indicate that we are not moving anymore:
        self.internal_update_target_position(None).await?;

        log::info!(
            "Stopped stepper motor at {:?}, target: {:?}",
            self.current_unit_position(),
            self.target_unit_position().await
        );

        Ok(())
    }

    /// Resets the current position to 0.
    pub async fn reset_start_position(&mut self, storage: &Storage) -> anyhow::Result<()> {
        self.stop().await?;

        // The current position will be set to 0:
        self.current_position.set(&UnitPosition::new(0));
        self.pcnt.clear_count()?;
        self.has_position = true;
        self.store(storage)?;

        Ok(())
    }

    pub async fn reset_max_position(&mut self, storage: &Storage) -> anyhow::Result<()> {
        self.stop().await?;

        // The max will be set to the current position:
        self.max_position.set(&self.current_unit_position());
        self.store(storage)?;
        log::info!(
            "Set max position to {}, currently at {}",
            self.max_position,
            self.current_unit_position()
        );

        Ok(())
    }

    pub fn store(&self, storage: &Storage) -> anyhow::Result<()> {
        if self.has_max_position() {
            storage.set_max_position(self.max_position.clone())?;
        }

        if self.has_position() {
            storage.set_current_position(self.current_unit_position())?;
        }

        Ok(())
    }
}
