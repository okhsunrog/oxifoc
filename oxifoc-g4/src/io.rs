//! I/O wrappers implementing embedded-io-async traits for transport layers

use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx, Error as UartError};
use embedded_io_async::{ErrorType, Read, Write};

/// Reader half of UART
pub struct UartReader {
    inner: BufferedUartRx<'static>,
}

impl UartReader {
    pub fn new(inner: BufferedUartRx<'static>) -> Self {
        Self { inner }
    }
}

impl ErrorType for UartReader {
    type Error = UartError;
}

impl Read for UartReader {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.inner.read(buf).await
    }
}

/// Writer half of UART
pub struct UartWriter {
    inner: BufferedUartTx<'static>,
}

impl UartWriter {
    /// Max bytes per write — must not exceed the BufferedUart TX ring buffer
    /// capacity, otherwise embassy returns BufferTooLong.
    pub const MAX_WRITE: usize = 512;

    pub fn new(inner: BufferedUartTx<'static>) -> Self {
        Self { inner }
    }
}

impl ErrorType for UartWriter {
    type Error = UartError;
}

impl Write for UartWriter {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Embassy's BufferedUartTx rejects writes where buf.len() exceeds the
        // ring buffer capacity (BufferTooLong). The ergot TX worker can pass
        // large chunks from the OUTQ stream consumer, so cap to a safe size.
        let chunk = &buf[..buf.len().min(Self::MAX_WRITE)];
        self.inner.write(chunk).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush().await
    }
}
