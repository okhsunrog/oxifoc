//! I/O wrappers implementing embedded-io-async traits for transport layers

use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx, Error as UartError};
use embedded_io_async::{ErrorType, Read, Write};

/// Reader half of USART
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

/// Writer half of USART
pub struct UartWriter {
    inner: BufferedUartTx<'static>,
}

impl UartWriter {
    pub fn new(inner: BufferedUartTx<'static>) -> Self {
        Self { inner }
    }
}

impl ErrorType for UartWriter {
    type Error = UartError;
}

impl Write for UartWriter {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush().await
    }
}
