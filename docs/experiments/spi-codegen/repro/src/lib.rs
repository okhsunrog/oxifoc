#![no_std]
use embassy_stm32::{
    gpio::Output,
    mode::Blocking,
    spi::{Spi, mode::Master},
};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice, NoDelay};

type Device<'a, 'd> =
    ExclusiveDevice<&'a mut Spi<'d, Blocking, Master>, &'a mut Output<'d>, NoDelay>;
#[unsafe(no_mangle)]
pub fn spi_transfer(
    device: &mut Device<'_, '_>,
    read: &mut [u8; 2],
    write: &[u8; 2],
) -> Result<(), DeviceError<embassy_stm32::spi::Error, core::convert::Infallible>> {
    device.transfer(read, write)
}
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
