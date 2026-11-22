//! Validate network requests before calling dap-rs, whose USB parser assumes
//! padded input and enough room for the response.

use dap_rs::dap::{Command, DapInfoID};

use super::framing::PACKET_SIZE;

const SWD_READ: u8 = 1 << 1;
const DAP_OK: u8 = 0;
const DAP_ERROR: u8 = 0xff;
const MAX_WAIT_ATTEMPTS: u16 = 5;
const MAX_PIN_WAIT_US: u32 = 1_000;

pub const DEFAULT_CLOCK_HZ: u32 = 1_000_000;
const MIN_CLOCK_HZ: u32 = 500_000;

pub enum Action {
    Process(Command),
    Delay(u16),
    Clock(u32),
    Reply(u8),
    Unsupported,
}

pub fn prepare(packet: &mut [u8]) -> Option<Action> {
    let (id, args) = packet.split_first_mut()?;
    let id = *id;
    let cmd = Command::try_from(id).unwrap_or(Command::Unimplemented);
    let minimum = match cmd {
        Command::DAP_Disconnect | Command::DAP_ResetTarget | Command::DAP_TransferAbort => 0,
        Command::DAP_Info | Command::DAP_Connect | Command::DAP_SWD_Configure => 1,
        Command::DAP_HostStatus | Command::DAP_Delay | Command::DAP_Transfer => 2,
        Command::DAP_SWJ_Sequence | Command::DAP_SWD_Sequence => 1,
        Command::DAP_SWJ_Clock | Command::DAP_TransferBlock => 4,
        Command::DAP_WriteABORT | Command::DAP_TransferConfigure => 5,
        Command::DAP_SWJ_Pins => 6,
        _ => return Some(Action::Unsupported),
    };
    if args.len() < minimum {
        return None;
    }

    match cmd {
        // The shared bit-banger also implements JTAG. This board wires SWD only.
        Command::DAP_Connect if args[0] > 1 => return Some(Action::Reply(0)),
        Command::DAP_SWJ_Clock => {
            let hz = u32::from_le_bytes(args[..4].try_into().ok()?);
            return Some(if hz < MIN_CLOCK_HZ {
                Action::Reply(DAP_ERROR)
            } else {
                Action::Clock(hz.min(DEFAULT_CLOCK_HZ))
            });
        }
        Command::DAP_Delay => {
            return Some(Action::Delay(u16::from_le_bytes(
                args[..2].try_into().ok()?,
            )));
        }
        Command::DAP_SWJ_Pins => {
            let wait = u32::from_le_bytes(args[2..6].try_into().ok()?);
            args[2..6].copy_from_slice(&wait.min(MAX_PIN_WAIT_US).to_le_bytes());
        }
        Command::DAP_TransferConfigure => {
            let wait = u16::from_le_bytes(args[1..3].try_into().ok()?);
            args[1..3].copy_from_slice(&wait.clamp(1, MAX_WAIT_ATTEMPTS).to_le_bytes());
        }
        Command::DAP_Transfer => validate_transfer(args)?,
        Command::DAP_TransferBlock => validate_block(args)?,
        Command::DAP_SWD_Sequence => validate_sequences(args)?,
        _ => {}
    }

    Some(Action::Process(cmd))
}

pub fn short_reply(response: &mut [u8], command: u8, status: u8) -> usize {
    response[..2].copy_from_slice(&[command, status]);
    2
}

pub fn ok(response: &mut [u8], command: u8) -> usize {
    short_reply(response, command, DAP_OK)
}

pub fn mask_capabilities(command: Command, request: &[u8], response: &mut [u8]) {
    if command == Command::DAP_Info && request[1] == DapInfoID::Capabilities as u8 {
        response[2] &= 0x01;
    }
}

fn validate_transfer(args: &[u8]) -> Option<()> {
    let count = args[1];
    let mut transfers = &args[2..];
    let mut response_len = 3;
    for _ in 0..count {
        let (&flags, rest) = transfers.split_first()?;
        // Value matching/timestamps are unused by the host clients. In
        // particular, value-match retries could multiply the polling budget.
        if flags & 0xf0 != 0 {
            return None;
        }
        transfers = if flags & SWD_READ != 0 {
            response_len += 4;
            rest
        } else {
            rest.get(4..)?
        };
    }
    (response_len <= usize::from(PACKET_SIZE)).then_some(())
}

fn validate_block(args: &[u8]) -> Option<()> {
    let count = usize::from(u16::from_le_bytes(args[1..3].try_into().ok()?));
    let bytes = 4 * count;
    let flags = args[3];
    let valid_len = if flags & SWD_READ != 0 {
        4 + bytes <= usize::from(PACKET_SIZE)
    } else {
        4 + bytes <= args.len()
    };
    (count > 0 && flags & 0xf0 == 0 && valid_len).then_some(())
}

fn validate_sequences(args: &[u8]) -> Option<()> {
    let (&count, mut sequences) = args.split_first()?;
    let mut response_len = 2;
    for _ in 0..count {
        let (&flags, rest) = sequences.split_first()?;
        let bits = usize::from(flags & 0x3f);
        let bytes = if bits == 0 { 8 } else { bits.div_ceil(8) };
        sequences = if flags & 0x80 != 0 {
            response_len += bytes;
            rest
        } else {
            rest.get(bytes..)?
        };
    }
    (response_len <= usize::from(PACKET_SIZE)).then_some(())
}
