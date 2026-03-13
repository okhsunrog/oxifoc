//! Motor control systems (FOC, fault handling)

pub mod foc;

pub use foc::init as init_foc;
