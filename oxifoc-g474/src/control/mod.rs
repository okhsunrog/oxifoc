//! Motor control systems (FOC, fault handling)

pub mod fault_handler;
pub mod foc;

pub use foc::init as init_foc;
