use const_dotenvy::dotenvy;
use esp_hal::time::Rate;

#[derive(Debug, Clone, Copy)]
pub struct MovementConfig {
    pub full_steps_per_rotation: u32,
    pub microsteps_per_step: u32,
    pub max_rounds_per_second: f64,
}

pub struct Environment {}

impl Environment {
    pub const fn matter_seed() -> u32 {
        dotenvy!(MATTER_RANDOM_NUMBER_SEED: u32)
    }

    pub const fn matter_serial_number() -> &'static str {
        dotenvy!(MATTER_SERIAL_NUMBER: &str)
    }

    pub const fn enable_stealth_chop() -> bool {
        dotenvy!(ENABLE_STEALTH_CHOP: bool = false)
    }

    pub const fn run_current() -> f64 {
        dotenvy!(RUN_CURRENT: f64 = 0.3)
    }

    pub const fn should_invert_direction() -> bool {
        dotenvy!(INVERT_DIRECTION: bool = false)
    }

    pub const fn uart_tx_pin() -> u8 {
        dotenvy!(TX_PIN: u8)
    }

    pub const fn uart_rx_pin() -> u8 {
        dotenvy!(RX_PIN: u8)
    }

    pub const fn enable_pin() -> u8 {
        dotenvy!(EN_PIN: u8)
    }

    pub const fn step_pin() -> u8 {
        dotenvy!(STEP_PIN: u8)
    }

    pub const fn dir_pin() -> u8 {
        dotenvy!(DIR_PIN: u8)
    }

    pub const fn uart_baud_rate() -> Rate {
        Rate::from_hz(dotenvy!(UART_BAUD_RATE: u32 = 115_200))
    }

    pub const fn reset_matter_duration() -> u64 {
        dotenvy!(RESET_MATTER_DURATION: u64 = u64::MAX)
    }

    pub const fn reset_positions_duration() -> u64 {
        dotenvy!(RESET_POSITIONS_DURATION: u64 = u64::MAX)
    }

    pub const fn led_pin() -> u8 {
        dotenvy!(LED_PIN: u8)
    }

    pub const fn led2_pin() -> Option<u8> {
        let pin = dotenvy!(LED2_PIN: u8 = 0);
        if pin == 0 { None } else { Some(pin) }
    }

    pub const fn up_button_pin() -> u8 {
        dotenvy!(UP_BUTTON_PIN: u8)
    }

    pub const fn down_button_pin() -> u8 {
        dotenvy!(DOWN_BUTTON_PIN: u8)
    }

    pub const fn pause_button_pin() -> u8 {
        dotenvy!(PAUSE_BUTTON_PIN: u8)
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
