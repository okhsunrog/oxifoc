//! Bench-only USB endpoint timing. Measurements include IRQ preemption.
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_usb::driver::{
    Driver, Endpoint, EndpointAddress, EndpointAllocError, EndpointError, EndpointIn, EndpointInfo,
    EndpointType,
};

pub static WRITES: AtomicU32 = AtomicU32::new(0);
pub static MAX_WRITE_TICKS: AtomicU32 = AtomicU32::new(0);
pub static LONG_WRITES: AtomicU32 = AtomicU32::new(0);

pub struct MeasuredIn<E>(pub E);
impl<E: Endpoint> Endpoint for MeasuredIn<E> {
    fn info(&self) -> &EndpointInfo {
        self.0.info()
    }
    async fn wait_enabled(&mut self) {
        self.0.wait_enabled().await;
    }
}
impl<E: EndpointIn> EndpointIn for MeasuredIn<E> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let streaming =
            oxifoc_core::runtime::streaming::FAST_TELEM_PERIOD.load(Ordering::Relaxed) != 0;
        // TIM5 is the free-running APB1 bench reference. Unlike DWT, it
        // keeps running when probe-rs detaches and clears DEMCR.TRCENA.
        let start = embassy_stm32::pac::TIM5.cnt().read();
        let result = self.0.write(data).await;
        let ticks = embassy_stm32::pac::TIM5.cnt().read().wrapping_sub(start);
        if !streaming {
            // A keepalive waiting for the next host connection is not a
            // telemetry stall. In particular, exclude the first such write.
            return result;
        }
        WRITES.fetch_add(1, Ordering::Relaxed);
        MAX_WRITE_TICKS.fetch_max(ticks, Ordering::Relaxed);
        // About 6 ms at the nominal 84 MHz APB1 timer clock.
        if ticks > 500_000 {
            LONG_WRITES.fetch_add(1, Ordering::Relaxed);
            defmt::info!(
                "usbwait: end_us={} ticks={} len={}",
                embassy_time::Instant::now().as_micros(),
                ticks,
                data.len()
            );
        }
        result
    }
}

// Adapts only the EndpointIn associated type for ergot's generic tx_worker.
pub struct MeasuredDriver<D>(D);
impl<'a, D: Driver<'a>> Driver<'a> for MeasuredDriver<D> {
    type EndpointOut = D::EndpointOut;
    type EndpointIn = MeasuredIn<D::EndpointIn>;
    type ControlPipe = D::ControlPipe;
    type Bus = D::Bus;
    fn alloc_endpoint_out(
        &mut self,
        kind: EndpointType,
        addr: Option<EndpointAddress>,
        size: u16,
        interval: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.0.alloc_endpoint_out(kind, addr, size, interval)
    }
    fn alloc_endpoint_in(
        &mut self,
        kind: EndpointType,
        addr: Option<EndpointAddress>,
        size: u16,
        interval: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.0
            .alloc_endpoint_in(kind, addr, size, interval)
            .map(MeasuredIn)
    }
    fn start(self, size: u16) -> (Self::Bus, Self::ControlPipe) {
        self.0.start(size)
    }
}
