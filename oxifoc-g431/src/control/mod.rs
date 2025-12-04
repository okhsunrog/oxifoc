//! Motor control systems (FOC, fault handling)

pub mod fault_handler;
pub mod foc;

pub use foc::{duty_to_iq, get_adc_snapshot, init as init_foc, send_command};
