//! Panic and HardFault record that survives a reset, reported over
//! `req/diag` (`DiagRequest::Crash`).
//!
//! The record sits in the 256 CCM bytes the linker reserves between
//! `__scrash` and `__ecrash` (ccm.x): `__pre_init` neither loads nor zeroes
//! them. A reset keeps the contents; a power cycle leaves noise, rejected by
//! the magic and checksum. The record stays until a client clears it, so a
//! crash followed by more reboots is not lost.

use core::fmt::Write;

use oxifoc_core::types::diag::{CRASH_FILE_LEN, CRASH_MESSAGE_LEN, CrashKind, CrashRecord};

const MAGIC: u32 = 0x4352_4153;

#[repr(C)]
#[derive(Clone, Copy)]
struct Raw {
    magic: u32,
    kind: u32,
    boots_since: u32,
    uptime_ms: u32,
    pc: u32,
    lr: u32,
    cfsr: u32,
    hfsr: u32,
    line: u32,
    file_len: u32,
    message_len: u32,
    file: [u8; CRASH_FILE_LEN],
    message: [u8; CRASH_MESSAGE_LEN],
    check: u32,
}

const _: () = assert!(size_of::<Raw>() <= 256);

unsafe extern "C" {
    // Linker-reserved CCM bytes (ccm.x); not a Rust allocation, so reading
    // whatever a previous run left there is defined, like a register.
    static mut __scrash: u32;
}

fn slot() -> *mut Raw {
    (&raw mut __scrash).cast()
}

impl Raw {
    fn checksum(&self) -> u32 {
        // FNV-1a over every field but `check`.
        let words = [
            self.magic,
            self.kind,
            self.boots_since,
            self.uptime_ms,
            self.pc,
            self.lr,
            self.cfsr,
            self.hfsr,
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
        // SAFETY: the reserved slot is 256 bytes, 4-aligned (ccm.x), and
        // only this module touches it.
        unsafe { core::ptr::write_volatile(slot(), self) }
    }

    fn valid(&self) -> bool {
        self.magic == MAGIC
            && self.kind <= 1
            && self.file_len as usize <= CRASH_FILE_LEN
            && self.message_len as usize <= CRASH_MESSAGE_LEN
            && self.check == self.checksum()
    }
}

fn blank(kind: CrashKind) -> Raw {
    Raw {
        magic: MAGIC,
        kind: kind as u32,
        boots_since: 0,
        uptime_ms: embassy_time::Instant::now().as_millis() as u32,
        pc: 0,
        lr: 0,
        cfsr: 0,
        hfsr: 0,
        line: 0,
        file_len: 0,
        message_len: 0,
        file: [0; CRASH_FILE_LEN],
        message: [0; CRASH_MESSAGE_LEN],
        check: 0,
    }
}

fn fault_status() -> (u32, u32) {
    // SAFETY: read-only accesses to the SCB fault status registers.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        (scb.cfsr.read(), scb.hfsr.read())
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

/// Record a panic. Called from the panic handler with interrupts off.
pub fn record_panic(info: &core::panic::PanicInfo) {
    let mut raw = blank(CrashKind::Panic);
    (raw.cfsr, raw.hfsr) = fault_status();
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

/// Record a HardFault.
pub fn record_hard_fault(frame: &cortex_m_rt::ExceptionFrame) {
    let mut raw = blank(CrashKind::HardFault);
    raw.pc = frame.pc();
    raw.lr = frame.lr();
    (raw.cfsr, raw.hfsr) = fault_status();
    raw.store();
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

/// At boot: return the surviving record, counting this boot, or `None`.
pub fn take_at_boot() -> Option<CrashRecord> {
    // SAFETY: see `store`; any bit pattern is a valid `Raw`.
    let mut raw = unsafe { core::ptr::read_volatile(slot()) };
    if !raw.valid() {
        return None;
    }
    raw.boots_since = raw.boots_since.saturating_add(1);
    raw.store();
    Some(CrashRecord {
        kind: if raw.kind == 0 {
            CrashKind::Panic
        } else {
            CrashKind::HardFault
        },
        boots_since: raw.boots_since.min(u32::from(u16::MAX)) as u16,
        uptime_ms: raw.uptime_ms,
        pc: raw.pc,
        lr: raw.lr,
        cfsr: raw.cfsr,
        hfsr: raw.hfsr,
        file: text(&raw.file[..raw.file_len as usize]),
        line: raw.line,
        message: text(&raw.message[..raw.message_len as usize]),
    })
}

/// Forget the record (a client cleared it).
pub fn clear() {
    // SAFETY: see `store`; `magic` is the first field of the repr(C) record.
    unsafe { core::ptr::write_volatile(slot().cast::<u32>(), 0) }
}
