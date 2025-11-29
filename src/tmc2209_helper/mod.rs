mod driver_utils;
pub use driver_utils::*;

use tmc2209::data::StandstillMode;
use tmc2209::reg::{self, Map};

pub fn default_map() -> Map {
    let mut result = Map::default();

    *result.chopconf_mut() = default_chopconf();
    *result.ihold_irun_mut() = default_ihold_irun();
    *result.gconf_mut() = default_global_config();
    *result.pwmconf_mut() = default_pwmconf();

    result
}

fn default_chopconf() -> reg::CHOPCONF {
    const CHOPPER_CONFIG_DEFAULT: u32 = 0x10000053;

    const TBL_DEFAULT: u32 = 0b10;
    const HEND_DEFAULT: u32 = 0;
    const HSTART_DEFAULT: u32 = 5;

    let mut register = reg::CHOPCONF(CHOPPER_CONFIG_DEFAULT);

    register.set_tbl(TBL_DEFAULT);
    register.set_hend(HEND_DEFAULT);
    register.set_hstrt(HSTART_DEFAULT);
    register.set_toff(3);

    register
}

fn default_ihold_irun() -> reg::IHOLD_IRUN {
    //const HOLD_DELAY_MIN: u8 = 0;
    //const HOLD_DELAY_MAX: u8 = 15;
    const IHOLD_DEFAULT: u8 = 16;
    const IRUN_DEFAULT: u8 = 31;
    const IHOLD_DELAY_DEFAULT: u8 = 1;
    //const CURRENT_SETTING_MIN: u8 = 0;
    //const CURRENT_SETTING_MAX: u8 = 31;

    let mut register = reg::IHOLD_IRUN(0);

    register.set_ihold(IHOLD_DEFAULT);
    register.set_irun(IRUN_DEFAULT);
    register.set_ihold_delay(IHOLD_DELAY_DEFAULT);

    register
}

fn default_global_config() -> reg::GCONF {
    let mut register = reg::GCONF(0);

    register.set_i_scale_analog(false);
    register.set_pdn_disable(true); // according to the datasheet, this should be set to true when communicating via UART
    register.set_mstep_reg_select(true);
    register.set_multistep_filt(true);

    register
}

fn default_pwmconf() -> reg::PWMCONF {
    let mut register = reg::PWMCONF(0);

    register.set_pwm_ofs(36);
    register.set_pwm_freq(1);
    register.set_pwm_autoscale(true);
    register.set_pwm_autograd(true);
    register.set_freewheel(StandstillMode::Normal);
    register.set_pwm_reg(1);
    register.set_pwm_lim(12);

    register
}

use thiserror::Error;
use tmc2209::ReadResponse;

#[derive(Debug, Clone, Error)]
pub enum TmcError {
    #[error("Failed to read/write from/to the driver: {0:?}")]
    IOError(embedded_io::ErrorKind),
    #[error("Invalid crc in response: {0:?}")]
    InvalidCrc([u8; ReadResponse::LEN_BYTES]),
    #[error("Expected address {expected} but received {received}")]
    UnexpectedAddress { expected: u8, received: u8 },
}

impl<E: embedded_io::Error> From<E> for TmcError {
    fn from(err: E) -> Self {
        Self::IOError(err.kind())
    }
}
