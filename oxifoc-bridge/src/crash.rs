//! Panic record that survives the reset a panic ends in, reported over
//! `req/bridge_diag`.
//!
//! The record sits in RTC fast RAM that the startup code zeroes only after
//! power-on, so a software reset keeps it. Magic and checksum reject anything
//! else found there. It stays until a client clears it: a panic followed by
//! more reboots is not lost. CPU exceptions are recorded too, since esp-hal
//! turns them into panics whose message carries the cause, `mepc` and `mtval`.

use core::fmt::Write;

use oxifoc_core::types::diag::{CRASH_FILE_LEN, CRASH_MESSAGE_LEN, CrashKind, CrashRecord};

const MAGIC: u32 = 0x4352_4153;

#[repr(C)]
#[derive(Clone, Copy)]
struct Raw {
    magic: u32,
    boots_since: u32,
    uptime_ms: u32,
    line: u32,
    file_len: u32,
    message_len: u32,
    file: [u8; CRASH_FILE_LEN],
    message: [u8; CRASH_MESSAGE_LEN],
    check: u32,
}

const WORDS: usize = size_of::<Raw>() / 4;
const _: () = assert!(size_of::<Raw>() == WORDS * 4 && align_of::<Raw>() == 4);

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut SLOT: [u32; WORDS] = [0; WORDS];

fn slot() -> *mut Raw {
    (&raw mut SLOT).cast()
}

fn load() -> Raw {
    // SAFETY: the slot has the size and alignment of `Raw`, any bit pattern
    // is a valid `Raw`, and only this module touches it.
    unsafe { core::ptr::read_volatile(slot()) }
}

impl Raw {
    fn checksum(&self) -> u32 {
        // FNV-1a over every field but `check`.
        let words = [
            self.magic,
            self.boots_since,
            self.uptime_ms,
            self.line,
            self.file_len,
            self.message_len,
        ];
        words
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .chain(self.file)
            .chain(self.message)
            .fold(0x811c_9dc5_u32, |hash, byte| {
                (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
            })
    }

    fn store(mut self) {
        self.check = self.checksum();
        // SAFETY: see `load`.
        unsafe { core::ptr::write_volatile(slot(), self) }
    }

    fn valid(&self) -> bool {
        self.magic == MAGIC
            && self.file_len as usize <= CRASH_FILE_LEN
            && self.message_len as usize <= CRASH_MESSAGE_LEN
            && self.check == self.checksum()
    }
}

/// Truncating writer into a fixed byte buffer, cut at a char boundary.
struct Truncate<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Write for Truncate<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            let mut utf8 = [0; 4];
            let encoded = c.encode_utf8(&mut utf8).as_bytes();
            if self.len + encoded.len() > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.len..self.len + encoded.len()].copy_from_slice(encoded);
            self.len += encoded.len();
        }
        Ok(())
    }
}

/// Record a panic. Called from the panic handler, just before the reset.
pub fn record_panic(info: &core::panic::PanicInfo) {
    let mut raw = Raw {
        magic: MAGIC,
        boots_since: 0,
        uptime_ms: embassy_time::Instant::now().as_millis() as u32,
        line: 0,
        file_len: 0,
        message_len: 0,
        file: [0; CRASH_FILE_LEN],
        message: [0; CRASH_MESSAGE_LEN],
        check: 0,
    };
    if let Some(location) = info.location() {
        raw.line = location.line();
        let mut file = Truncate {
            buf: &mut raw.file,
            len: 0,
        };
        // Keep the tail of a long path: the file name matters most.
        let path = location.file();
        let mut start = path.len().saturating_sub(CRASH_FILE_LEN);
        while !path.is_char_boundary(start) {
            start += 1;
        }
        let _ = file.write_str(path.get(start..).unwrap_or_default());
        raw.file_len = file.len as u32;
    }
    let mut message = Truncate {
        buf: &mut raw.message,
        len: 0,
    };
    let _ = write!(message, "{}", info.message());
    raw.message_len = message.len as u32;
    raw.store();
}

/// At boot: count this boot against a surviving record.
pub fn count_boot() {
    let mut raw = load();
    if raw.valid() {
        raw.boots_since = raw.boots_since.saturating_add(1);
        raw.store();
    }
}

fn text<const N: usize>(bytes: &[u8]) -> heapless::String<N> {
    let mut s = heapless::String::new();
    for c in core::str::from_utf8(bytes).unwrap_or("?").chars() {
        if s.push(c).is_err() {
            break;
        }
    }
    s
}

/// The surviving record, if any.
pub fn read() -> Option<CrashRecord> {
    let raw = load();
    raw.valid().then(|| CrashRecord {
        kind: CrashKind::Panic,
        boots_since: raw.boots_since.min(u32::from(u16::MAX)) as u16,
        uptime_ms: raw.uptime_ms,
        pc: 0,
        lr: 0,
        cfsr: 0,
        hfsr: 0,
        file: text(&raw.file[..raw.file_len as usize]),
        line: raw.line,
        message: text(&raw.message[..raw.message_len as usize]),
    })
}

/// Forget the record (a client cleared it).
pub fn clear() {
    // SAFETY: see `load`; `magic` is the first field of the repr(C) record.
    unsafe { core::ptr::write_volatile(slot().cast::<u32>(), 0) }
}
