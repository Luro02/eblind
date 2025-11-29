use const_dotenvy::dotenvy;
use esp_idf_hal::units::Hertz;

#[derive(Debug, Clone, Copy)]
pub struct MovementConfig {
    pub full_steps_per_rotation: u32,
    pub microsteps_per_step: u32,
    pub max_rounds_per_second: f64,
}

pub struct Environment {}

impl Environment {
    pub const fn enable_stealth_chop() -> bool {
        dotenvy!(ENABLE_STEALTH_CHOP: bool = false)
    }

    pub const fn run_current() -> f64 {
        dotenvy!(RUN_CURRENT: f64 = 0.3)
    }

    pub const fn should_invert_direction() -> bool {
        dotenvy!(INVERT_DIRECTION: bool = false)
    }

    pub const fn uart_tx_pin() -> usize {
        dotenvy!(TX_PIN: usize)
    }

    pub const fn uart_rx_pin() -> usize {
        dotenvy!(RX_PIN: usize)
    }

    pub const fn enable_pin() -> usize {
        dotenvy!(EN_PIN: usize)
    }

    pub const fn step_pin() -> usize {
        dotenvy!(STEP_PIN: usize)
    }

    pub const fn dir_pin() -> usize {
        dotenvy!(DIR_PIN: usize)
    }

    pub const fn uart_baud_rate() -> Hertz {
        Hertz(dotenvy!(UART_BAUD_RATE: u32 = 115_200))
    }

    pub const fn reset_matter_duration() -> u64 {
        dotenvy!(RESET_MATTER_DURATION: u64 = u64::MAX)
    }

    pub const fn reset_positions_duration() -> u64 {
        dotenvy!(RESET_POSITIONS_DURATION: u64 = u64::MAX)
    }

    pub const fn led_pin() -> usize {
        dotenvy!(LED_PIN: usize)
    }

    pub const fn led2_pin() -> Option<usize> {
        let pin = dotenvy!(LED2_PIN: usize = 0);
        if pin == 0 {
            None
        } else {
            Some(pin)
        }
    }

    pub const fn up_button_pin() -> usize {
        dotenvy!(UP_BUTTON_PIN: usize)
    }

    pub const fn down_button_pin() -> usize {
        dotenvy!(DOWN_BUTTON_PIN: usize)
    }

    pub const fn pause_button_pin() -> usize {
        dotenvy!(PAUSE_BUTTON_PIN: usize)
    }

    pub const fn resolve_movement_config() -> MovementConfig {
        MovementConfig {
            full_steps_per_rotation: dotenvy!(FULL_STEPS_PER_ROTATION: u32),
            microsteps_per_step: dotenvy!(MICROSTEPS_PER_STEP: u32),
            max_rounds_per_second: dotenvy!(MAX_ROUNDS_PER_SECOND: f64),
        }
    }

    pub const fn should_clear_stored_data() -> bool {
        dotenvy!(SHOULD_CLEAR_STORED_DATA: bool = false)
    }
}
