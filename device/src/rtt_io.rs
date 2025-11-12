//! RTT channel wrapper implementing embedded-io-async traits for ergot

use embedded_io_async::{ErrorType, Read, Write};
use rtt_target::{UpChannel, DownChannel};

/// Error type for RTT I/O operations
#[derive(Debug, Clone, Copy)]
pub struct RttError;

impl embedded_io_async::Error for RttError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// RTT channel wrapper for reading (UpChannel - device to host)
pub struct RttReader {
    down: &'static mut DownChannel,
}

impl RttReader {
    pub fn new(down: &'static mut DownChannel) -> Self { Self { down } }
}

impl ErrorType for RttReader {
    type Error = RttError;
}

impl Read for RttReader {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Non-blocking read from RTT down channel
        let n = self.down.read(buf);
        Ok(n)
    }
}

/// RTT channel wrapper for writing (UpChannel - device to host)
pub struct RttWriter {
    channel: &'static mut UpChannel,
}

impl RttWriter {
    pub fn new(channel: &'static mut UpChannel) -> Self {
        Self { channel }
    }
}

impl ErrorType for RttWriter {
    type Error = RttError;
}

impl Write for RttWriter {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // RTT write is blocking, but typically very fast
        let written = self.channel.write(buf);
        Ok(written)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // RTT doesn't need explicit flushing
        Ok(())
    }
}

/// Combined RTT I/O for ergot
pub struct RttIo {
    reader: RttReader,
    writer: RttWriter,
}

impl RttIo {
    pub fn new(up_channel: &'static mut UpChannel, down_channel: &'static mut DownChannel) -> Self {
        Self {
            reader: RttReader::new(down_channel),
            writer: RttWriter::new(up_channel),
        }
    }

    pub fn split(self) -> (RttReader, RttWriter) {
        (self.reader, self.writer)
    }
}
