use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cmp;

use esp_hal::gpio::{Input, InputConfig, InputPin, Pull};

use button_driver::Button;
use embassy_executor::Spawner;
use embassy_sync::watch::WatchBehavior;
use embassy_time::{Duration, Instant, Timer};

use crate::environment::Environment;
use crate::matter::Position;
use crate::movement::StepperController;
use crate::storage::{Storage, StoragePartition};
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

pub enum ButtonKind {
    Pause,
    Direction(i32),
}

async fn process_button_event(
    button_event: ButtonEvent,
    button_mapping: &BTreeMap<usize, ButtonKind>,
    state: &mut State,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: &Storage<'static>,
) {
    match button_mapping[&button_event.pin] {
        ButtonKind::Pause => {
            if let Some(held_time) = button_event.held_time {
                let mut storages_to_reset = Vec::new();

                let will_reset = held_time.as_millis()
                    > cmp::min(
                        Environment::reset_positions_duration(),
                        Environment::reset_matter_duration(),
                    );

                if will_reset {
                    storage.lock().await;
                }

                if held_time.as_millis() > Environment::reset_positions_duration() {
                    storages_to_reset.push(StoragePartition::Nvs);
                }

                if held_time.as_millis() > Environment::reset_matter_duration() {
                    storages_to_reset.push(StoragePartition::Matter);
                }

                if !storages_to_reset.is_empty() {
                    log::info!(
                        "Resetting storages: {:?} due to held button for {:?} ms",
                        storages_to_reset,
                        held_time.as_millis()
                    );

                    storage.reset_partitions(&storages_to_reset).await.unwrap();
                }

                if will_reset {
                    // Trigger a system reset:
                    esp_hal::system::software_reset();
                }
            }

            if button_event.clicks == 0 {
                return;
            }

            if *state != State::Idle {
                // Not in calibration mode -> pause the stepper motor
                log::info!("Pausing stepper motor");
                // TODO: it does not have to wait for it to fully stop, unnecessary delay
                controller.lock().await.stop().await.unwrap();
                *state = State::Idle;
                return;
            }

            // Check if we are in calibration mode:
            if !START_CALIBRATION_DONE_WATCH.contains_value() {
                log::info!("Found start position, finishing calibration.");
                // Indicate that the current position is equal to the bottom position,
                // if the motor is currently moving towards a target, it will be stopped.
                if let Err(err) = controller.lock().await.reset_start_position(storage).await {
                    log::error!("Failed to store current position in NVS: {err}");
                }
                START_CALIBRATION_DONE_WATCH.sender().send(());
            } else if !MAX_CALIBRATION_DONE_WATCH.contains_value() {
                log::info!("Found max position, finishing calibration.");

                // Indicate that the current position is equal to the bottom position,
                // if the motor is currently moving towards a target, it will be stopped.
                if let Err(err) = controller.lock().await.reset_max_position(storage).await {
                    log::error!("Failed to store max position in NVS: {err}");
                }
                MAX_CALIBRATION_DONE_WATCH.sender().send(());
            }
        }
        ButtonKind::Direction(direction) => {
            if button_event.clicks == 0 {
                return;
            }

            *state = match state {
                State::Idle => State::Direction(direction),
                // if it is already moving in that direction, stop it
                State::Direction(d) if *d == direction => State::Idle,
                // if it is moving in the opposite direction, change direction
                State::Direction(_) => State::Direction(direction),
            };

            if *state == State::Idle {
                log::info!("Stopping stepper motor");
                controller.lock().await.stop().await.unwrap();
                return;
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

async fn update_state_from_controller(
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    state: &mut State,
) {
    let controller = controller.lock().await;

    if !controller.is_moving() {
        *state = State::Idle;
        return;
    }

    let current_position = controller.current_position();
    let target_position = controller.target_position().await;

    *state = match target_position {
        None => State::Idle,
        Some(target) => {
            if current_position.is_opening(target) {
                State::Direction(-1)
            } else {
                State::Direction(1)
            }
        }
    };
}

#[embassy_executor::task]
async fn process_button_events_task(
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
    mut buttons: Vec<Button<Input<'static>, Instant, Duration>>,
    button_mapping: BTreeMap<usize, ButtonKind>,
) {
    let mut state = State::Idle;

    loop {
        Timer::after(Duration::from_millis(50)).await;

        // Sync state with actual movement (e.g. motor moved via matter or finished moving)
        update_state_from_controller(controller.clone(), &mut state).await;

        for button in buttons.iter_mut() {
            button.tick();

            if button.held_time().is_some() || button.clicks() > 0 {
                process_button_event(
                    ButtonEvent {
                        pin: button.pin.peripheral_input().gpio_number().unwrap() as usize,
                        clicks: button.clicks(),
                        held_time: button.held_time(),
                    },
                    &button_mapping,
                    &mut state,
                    controller.clone(),
                    &storage,
                )
                .await;
            }

            button.reset();
        }
    }
}

pub fn spawn_button_poll_tasks(
    spawner: Spawner,
    pins: [impl InputPin + 'static; 3],
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
) -> anyhow::Result<()> {
    let mut buttons = Vec::new();

    let mut button_mapping = BTreeMap::new();

    for (i, pin) in pins.into_iter().enumerate() {
        button_mapping.insert(
            pin.number() as usize,
            match i {
                0 => ButtonKind::Direction(-1),
                1 => ButtonKind::Pause,
                2 => ButtonKind::Direction(1),
                _ => unimplemented!("Unexpected button index"),
            },
        );

        // This is important, because there is no external pull-up resistor for the button.
        buttons.push(Button::new(
            Input::new(pin, InputConfig::default().with_pull(Pull::Up)),
            Default::default(),
        ));
    }

    spawner.spawn(process_button_events_task(
        controller,
        storage,
        buttons,
        button_mapping,
    ))?;

    Ok(())
}
