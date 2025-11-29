use std::cmp;
use std::collections::HashMap;
use std::sync::Arc;

use button_driver::{Button, PinWrapper};
use embassy_executor::Spawner;
use embassy_sync::watch::WatchBehavior;
use embassy_time::{Duration, Instant};

use esp_idf_hal::gpio::{self, AnyIOPin, PinDriver, PinId};
use esp_idf_hal::task::do_yield;
use esp_idf_hal::task::queue::Queue;
use esp_idf_hal::timer::config::{AlarmConfig, TimerConfig};
use esp_idf_hal::timer::TimerDriver;
use esp_idf_hal::units::FromValueType;
use esp_idf_sys::EspError;

use crate::environment::Environment;
use crate::matter::Position;
use crate::movement::StepperController;
use crate::storage::Storage;
use crate::utils::AsyncQueue;
use crate::{EmbassyMutex, MAX_CALIBRATION_DONE_WATCH, START_CALIBRATION_DONE_WATCH};

#[derive(Debug, Clone, Copy)]
pub struct ButtonEvent {
    /// The pin number that generated the event.
    pub pin: usize,
    /// Number of clicks detected.
    pub clicks: usize,
    /// How long the button was held before release (if any).
    pub held_time: Option<Duration>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Direction(i32),
}

pub trait ToPinId {
    #[must_use]
    fn pin_id(&self) -> PinId;
}

impl<'d, M> ToPinId for PinDriver<'d, M> {
    fn pin_id(&self) -> PinId {
        self.pin()
    }
}

pub fn start_poll_timer<'d, P>(
    duration: Duration,
    mut buttons: impl AsMut<[Button<P, Instant, Duration>]> + Send + 'd,
    queue: AsyncQueue<ButtonEvent>,
) -> Result<TimerDriver<'d>, EspError>
where
    P: PinWrapper + ToPinId + Send + 'd,
{
    let mut timer = TimerDriver::new(&TimerConfig {
        // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
        resolution: 1.MHz().into(),
        ..Default::default()
    })?;

    timer.set_alarm_action(Some(&AlarmConfig {
        reload_count: 0,
        // Set the actual alarm period, since the resolution is 1us, 1_000_000 represents 1s
        alarm_count: duration.as_micros(),
        auto_reload_on_alarm: true,
        ..Default::default()
    }))?;

    // SAFETY: The timer will not be forgotten
    unsafe {
        // Every time the timer fires, we poll all buttons
        timer.subscribe_nonstatic(move |_event_data| {
            let mut has_woken = false;
            for button in buttons.as_mut().iter_mut() {
                button.tick();

                if button.held_time().is_some() || button.clicks() > 0 {
                    has_woken = queue
                        .try_send_back(ButtonEvent {
                            pin: button.pin.pin_id() as usize,
                            clicks: button.clicks(),
                            held_time: button.held_time(),
                        })
                        .expect("Failed to send because queue is full");
                }

                button.reset();
            }

            if has_woken {
                // TODO: questionable if this is necessary
                // Yield to allow the receiver to run its task
                do_yield();
            }
        })?;
    }

    timer.enable()?;
    timer.start()?;

    Ok(timer)
}

const BUTTON_EVENT_QUEUE_SIZE: usize = 10;

pub enum ButtonKind {
    Pause,
    Direction(i32),
}

#[embassy_executor::task]
async fn process_button_events_task(
    queue: AsyncQueue<ButtonEvent>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Arc<Storage>,
    _timer: TimerDriver<'static>,
    button_mapping: HashMap<PinId, ButtonKind>,
) {
    let mut state = State::Idle;

    loop {
        let button_event = queue.recv_front().await;
        match button_mapping[&(button_event.pin as u8)] {
            ButtonKind::Pause => {
                if let Some(held_time) = button_event.held_time {
                    if held_time.as_millis() > Environment::reset_positions_duration() {
                        // Initiate a reset of the stepper positions
                        //log::info!("Resetting stepper positions");
                        storage.reset().unwrap();
                    }

                    if held_time.as_millis() > Environment::reset_matter_duration() {
                        //log::info!("Resetting Matter NVS");
                        storage.clear_matter_store().unwrap();
                    }

                    if held_time.as_millis()
                        > cmp::min(
                            Environment::reset_positions_duration(),
                            Environment::reset_matter_duration(),
                        )
                    {
                        // Trigger a system reset:
                        esp_idf_svc::hal::reset::restart();
                    }
                }

                if button_event.clicks == 0 {
                    continue;
                }

                if state != State::Idle {
                    // Not in calibration mode -> pause the stepper motor
                    log::info!("Pausing stepper motor");
                    // TODO: it does not have to wait for it to fully stop, unnecessary delay
                    controller.lock().await.stop().await.unwrap();
                    state = State::Idle;
                    continue;
                }

                // Check if we are in calibration mode:
                if !START_CALIBRATION_DONE_WATCH.contains_value() {
                    log::info!("Found start position, finishing calibration.");
                    // Indicate that the current position is equal to the bottom position,
                    // if the motor is currently moving towards a target, it will be stopped.
                    if let Err(err) = controller
                        .lock()
                        .await
                        .reset_start_position(&*storage)
                        .await
                    {
                        log::error!("Failed to store current position in NVS: {}", err);
                    }
                    START_CALIBRATION_DONE_WATCH.sender().send(());
                } else if !MAX_CALIBRATION_DONE_WATCH.contains_value() {
                    log::info!("Found max position, finishing calibration.");

                    // Indicate that the current position is equal to the bottom position,
                    // if the motor is currently moving towards a target, it will be stopped.
                    if let Err(err) = controller.lock().await.reset_max_position(&*storage).await {
                        log::error!("Failed to store max position in NVS: {}", err);
                    }
                    MAX_CALIBRATION_DONE_WATCH.sender().send(());
                }
            }
            ButtonKind::Direction(direction) => {
                if button_event.clicks == 0 {
                    continue;
                }

                state = match state {
                    State::Idle => State::Direction(direction),
                    // if it is already moving in that direction, stop it
                    State::Direction(d) if d == direction => State::Idle,
                    // if it is moving in the opposite direction, change direction
                    State::Direction(_) => State::Direction(direction),
                };

                if state == State::Idle {
                    log::info!("Stopping stepper motor");
                    controller.lock().await.stop().await.unwrap();
                    continue;
                }

                controller
                    .lock()
                    .await
                    .update_target_position(Some({
                        if direction < 0 {
                            Position::open()
                        } else {
                            Position::closed()
                        }
                    }))
                    .await
                    .unwrap();
            }
        }
    }
}

pub fn spawn_button_poll_tasks(
    spawner: Spawner,
    pins: [AnyIOPin<'static>; 3],
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Arc<Storage>,
) -> anyhow::Result<()> {
    let queue = AsyncQueue::new(Queue::new(BUTTON_EVENT_QUEUE_SIZE));
    let mut buttons = Vec::new();

    let mut button_mapping = HashMap::new();

    for (i, pin) in pins.into_iter().enumerate() {
        // This is important, because there is no external pull-up resistor for the button.
        let Ok(driver) = PinDriver::input(pin, gpio::Pull::Up) else {
            log::error!("Failed to setup button: {}", i);
            continue;
        };

        button_mapping.insert(
            driver.pin_id(),
            match i {
                0 => ButtonKind::Direction(-1),
                1 => ButtonKind::Pause,
                2 => ButtonKind::Direction(1),
                _ => unimplemented!("Unexpected button index"),
            },
        );

        buttons.push(Button::new(driver, Default::default()));
    }

    let timer = start_poll_timer(Duration::from_millis(50), buttons, unsafe {
        queue.borrow()
    })?;

    spawner.spawn(process_button_events_task(
        queue,
        controller,
        storage,
        timer,
        button_mapping,
    ))?;

    Ok(())
}
