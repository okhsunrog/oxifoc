#![no_std]
#![no_main]

use panic_rtt_target as _;

#[path = "../remote.rs"]
mod remote;

#[cortex_m_rt::entry]
fn main() -> ! {
    remote::start(None);
}
