use std::{
    fmt, io,
    time::{Duration, Instant},
};

use serialport::SerialPort;

const FRAME_MAGIC: [u8; 2] = [0xa5, 0x5a];
// magic(2) + cmd(1) + len(2) + crc(2)
const FRAME_OVERHEAD: usize = 7;

const CMD_ERASE: u8 = 0x00;
const CMD_PROGRAM: u8 = 0x01;
const CMD_JUMP_TO_APP: u8 = 0x02;
const CMD_PING: u8 = 0x03;

const PING_VALUE: u32 = 0x1234_5678;
// BOOTLOADER_COM_MAX_PAYLOAD
const MAX_PAYLOAD_LENGTH: usize = 4096;

// Klipper's serial bootloader-request escape, recognised by the running
// application in serial_irq.c. The leading '~' closes off any partial message
// block; the 32 bytes after it are what the firmware memcmp's against.
const REQUEST_BOOTLOADER: &[u8] = b"~ \x1c Request Serial Bootloader!! ~";

// Firmware bytes per PROGRAM command. The MCU's receive ringbuffer is 1024
// bytes and we wait for each ack before sending more, so this stays well
// inside it even while a flash write stalls the parse task.
const CHUNK_SIZE: usize = 512;

// A ping only defers the MCU's pending auto-jump by 50ms, so we have to ping
// faster than that to hold it in the bootloader.
const PING_INTERVAL: Duration = Duration::from_millis(20);
const READ_POLL: Duration = Duration::from_millis(5);
const ERASE_TIMEOUT: Duration = Duration::from_secs(5);
const PROGRAM_TIMEOUT: Duration = Duration::from_secs(2);

// The MCU arms its independent watchdog with a ~410ms timeout (IWDG RLR=0x0fff
// in stm32/watchdog.c, which is built into the bootloader too) and only feeds
// it from a scheduler task. The stock ERASE handler erases every page in one
// unbroken loop without yielding, so erasing a whole programmed app region
// trips the watchdog and resets the MCU before it can ack.
//
// ERASE always starts at the application base and takes only a length, so we
// cannot erase a sub-range. Instead walk the length up one page at a time:
// each request re-erases the pages already blanked, which is fast, plus at
// most one programmed page. Every step then stays well inside the window.
const ERASE_STEP: u32 = 2048;

#[derive(Debug, PartialEq, Eq)]
pub enum BootResult {
    JumpedToApp,
    AlreadyInApp,
}

#[derive(Debug)]
pub enum Error {
    Serial(serialport::Error),
    Io(io::Error),
    NotInBootloader,
    NoResponse { command: u8 },
    EraseRejected { requested: u32, erased: u32 },
    ProgramRejected { offset: u32, written: u32 },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serial(error) => write!(f, "serial error: {error}"),
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::NotInBootloader => {
                write!(f, "no ping response: board is not in the bootloader")
            }
            Self::NoResponse { command } => {
                write!(f, "no response to command {command:#04x}")
            }
            Self::EraseRejected { requested, erased } => write!(
                f,
                "erase rejected: asked for {requested} bytes, MCU erased {erased}"
            ),
            Self::ProgramRejected { offset, written } => write!(
                f,
                "program rejected at offset {offset:#x}: MCU wrote {written} bytes"
            ),
        }
    }
}

impl std::error::Error for Error {}

impl From<serialport::Error> for Error {
    fn from(error: serialport::Error) -> Self {
        Self::Serial(error)
    }
}

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

/// Catch the board in its bootloader and write `image` to the application
/// partition, then start it.
///
/// Call `request_bootloader` first, or reset the board by hand. Without the
/// backup-domain flag the MCU only stays in the bootloader for 200ms before
/// auto-jumping, so `window` then has to span the reset; pings hold it open in
/// 50ms increments until ERASE cancels the jump for good.
pub fn deploy(port: &mut dyn SerialPort, image: &[u8], window: Duration) -> Result<(), Error> {
    port.set_timeout(READ_POLL)?;
    if !enter_bootloader(port, window)? {
        return Err(Error::NotInBootloader);
    }
    // Must reach ERASE within 50ms of the last pong or the MCU jumps anyway.
    flash(port, image)?;
    jump_to_app(port)
}

/// Ask the running Klipper application to reboot into the bootloader.
///
/// The application sets the backup-domain flag (0x454c in BKP->DR1) and
/// resets, so the bootloader then waits indefinitely instead of auto-jumping
/// after 200ms. This is the reliable way in; catching a cold reset is not.
///
/// Silently does nothing if the board is already in the bootloader - the
/// bootloader does not speak the Klipper protocol. Follow with a ping.
pub fn request_bootloader(port: &mut dyn SerialPort) -> Result<(), Error> {
    std::io::Write::write_all(port, REQUEST_BOOTLOADER)?;
    std::io::Write::flush(port)?;
    Ok(())
}

/// Ping until the bootloader answers, or `window` elapses.
pub fn enter_bootloader(port: &mut dyn SerialPort, window: Duration) -> Result<bool, Error> {
    port.set_timeout(READ_POLL)?;
    let deadline = Instant::now() + window;

    while Instant::now() < deadline {
        let payload = transact(port, CMD_PING, &PING_VALUE.to_le_bytes(), PING_INTERVAL)?;
        if let Some(payload) = payload {
            if read_u32(&payload, 0) == Some(PING_VALUE) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Erase and program the application partition. Assumes the bootloader is
/// already listening (see `enter_bootloader`).
pub fn flash(port: &mut dyn SerialPort, image: &[u8]) -> Result<(), Error> {
    // The MCU programs whole 32-bit words; a non-multiple-of-4 tail hits a
    // broken path in bootloader.c, so pad to a word with the erased value.
    let mut image = image.to_vec();
    while image.len() % 4 != 0 {
        image.push(0xff);
    }
    let size = image.len() as u32;

    // The first ERASE also clears the pending auto-jump, pinning us here.
    let mut erased = 0;
    let mut target = 0;
    while target < size {
        target = (target + ERASE_STEP).min(size);

        print!("\r  erasing {target}/{size}");
        let _ = std::io::Write::flush(&mut std::io::stdout());

        let ack = require(
            transact(port, CMD_ERASE, &target.to_le_bytes(), ERASE_TIMEOUT)?,
            CMD_ERASE,
        )?;
        erased = read_u32(&ack, 0).unwrap_or(0);
        if erased < target {
            println!();
            return Err(Error::EraseRejected {
                requested: target,
                erased,
            });
        }
    }
    println!("\r  erased {erased} bytes; programming {size}...");

    let total = (image.len() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    for (index, chunk) in image.chunks(CHUNK_SIZE).enumerate() {
        let offset = (index * CHUNK_SIZE) as u32;
        let length = chunk.len() as u32;

        print!("\r  block {}/{} at {:#08x}", index + 1, total, offset);
        let _ = std::io::Write::flush(&mut std::io::stdout());

        // bootloader_cmd_program_req_t { offset, length, size } + data
        let mut payload = Vec::with_capacity(12 + chunk.len());
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&length.to_le_bytes());
        payload.extend_from_slice(&size.to_le_bytes());
        payload.extend_from_slice(chunk);

        let ack = require(
            transact(port, CMD_PROGRAM, &payload, PROGRAM_TIMEOUT)?,
            CMD_PROGRAM,
        )?;
        // bootloader_cmd_program_ack_t { length, offset }
        let written = read_u32(&ack, 0).unwrap_or(0);
        let next = read_u32(&ack, 4).unwrap_or(0);
        if written != length || next != offset + length {
            println!();
            return Err(Error::ProgramRejected { offset, written });
        }
    }
    println!();

    Ok(())
}

/// Kick a board that is sitting in the bootloader into its application.
pub fn boot(port: &mut dyn SerialPort, timeout_seconds: u32) -> Result<BootResult, Error> {
    let window = Duration::from_secs(timeout_seconds as u64);
    if enter_bootloader(port, window)? {
        jump_to_app(port)?;
        return Ok(BootResult::JumpedToApp);
    }
    Ok(BootResult::AlreadyInApp)
}

/// JUMP is not acknowledged - the MCU jumps ~1ms later.
pub fn jump_to_app(port: &mut dyn SerialPort) -> Result<(), Error> {
    write_packet(port, CMD_JUMP_TO_APP, &[])
}

fn require(payload: Option<Vec<u8>>, command: u8) -> Result<Vec<u8>, Error> {
    payload.ok_or(Error::NoResponse { command })
}

fn read_u32(payload: &[u8], offset: usize) -> Option<u32> {
    let bytes = payload.get(offset..offset + 4)?;
    Some(u32::from_le_bytes(bytes.try_into().unwrap()))
}

/// Send a command and wait for the matching response. The MCU echoes the
/// request's command id in its ack; there is no NACK, so a rejected command
/// is indistinguishable from silence.
fn transact(
    port: &mut dyn SerialPort,
    command: u8,
    payload: &[u8],
    timeout: Duration,
) -> Result<Option<Vec<u8>>, Error> {
    // Anything already buffered predates this request, so it cannot be the
    // response. Most of it is noise the application produced while we probed
    // it at the bootloader's baud - drop it rather than parse it.
    let _ = port.clear(serialport::ClearBuffer::Input);
    write_packet(port, command, payload)?;

    let deadline = Instant::now() + timeout;
    let mut received = Vec::new();
    let mut chunk = [0u8; 256];

    loop {
        while let Some((cmd, payload, end)) = find_frame(&received) {
            received.drain(..end);
            if cmd == command {
                return Ok(Some(payload));
            }
        }
        if Instant::now() >= deadline {
            return Ok(None);
        }
        match port.read(&mut chunk) {
            Ok(count) => received.extend_from_slice(&chunk[..count]),
            Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(error) => return Err(error.into()),
        }
    }
}

fn write_packet(port: &mut dyn SerialPort, command: u8, payload: &[u8]) -> Result<(), Error> {
    let packet = packet(command, payload);
    std::io::Write::write_all(port, &packet)?;
    std::io::Write::flush(port)?;
    Ok(())
}

fn packet(command: u8, payload: &[u8]) -> Vec<u8> {
    let length = u16::try_from(payload.len()).expect("CC2 payload is too large");
    let mut packet = Vec::with_capacity(FRAME_OVERHEAD + payload.len());
    packet.extend_from_slice(&FRAME_MAGIC);
    packet.push(command);
    packet.extend_from_slice(&length.to_be_bytes());
    packet.extend_from_slice(payload);
    packet.extend_from_slice(&crc16_ccitt(payload).to_be_bytes());
    packet
}

/// Find the first complete, CRC-valid frame in `data`.
///
/// Returns the command, its payload, and how many bytes to drain. Skips past
/// anything that looks like a header but fails to check out - the magic can
/// legitimately appear inside a payload.
fn find_frame(data: &[u8]) -> Option<(u8, Vec<u8>, usize)> {
    let mut start = 0;

    while start + FRAME_OVERHEAD <= data.len() {
        if data[start..start + 2] != FRAME_MAGIC {
            start += 1;
            continue;
        }

        let command = data[start + 2];
        let length = u16::from_be_bytes([data[start + 3], data[start + 4]]) as usize;
        if length > MAX_PAYLOAD_LENGTH {
            start += 1;
            continue;
        }

        let end = start + 5 + length + 2;
        if end > data.len() {
            // Cannot validate this one yet. It may be a real frame still
            // arriving, or a false header whose length field is noise - and we
            // cannot tell which. Keep scanning instead of blocking on it: a
            // real frame is found on a later call once the buffer has filled,
            // whereas waiting here would stall until the deadline.
            start += 1;
            continue;
        }

        let payload = &data[start + 5..start + 5 + length];
        let crc = u16::from_be_bytes([data[end - 2], data[end - 1]]);
        if crc16_ccitt(payload) != crc {
            start += 1;
            continue;
        }

        return Some((command, payload.to_vec(), end));
    }

    None
}

/// CRC-16/MCRF4XX - a transcription of bootloader_crc16_ccitt() in
/// bootloader_com.c, which is Klipper's crc16_ccitt() verbatim.
///
/// Note this is the *reflected* CCITT variant (poly 0x8408, init 0xffff).
/// The MSB-first "CCITT-FALSE" variant does not interoperate.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &byte in data {
        let mut d = byte ^ (crc as u8);
        d ^= d << 4;
        crc = (((d as u16) << 8) | (crc >> 8)) ^ ((d >> 4) as u16) ^ ((d as u16) << 3);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_firmware_crc() {
        // Catalogue "check" value for CRC-16/MCRF4XX.
        assert_eq!(crc16_ccitt(b"123456789"), 0x6f91);
    }

    #[test]
    fn builds_ping_packet_with_little_endian_payload() {
        assert_eq!(
            packet(CMD_PING, &PING_VALUE.to_le_bytes()),
            vec![0xa5, 0x5a, 0x03, 0x00, 0x04, 0x78, 0x56, 0x34, 0x12, 0x0b, 0xd7]
        );
    }

    #[test]
    fn builds_jump_to_app_packet() {
        assert_eq!(
            packet(CMD_JUMP_TO_APP, &[]),
            vec![0xa5, 0x5a, 0x02, 0x00, 0x00, 0xff, 0xff]
        );
    }

    #[test]
    fn finds_valid_frame() {
        let frame = packet(CMD_PING, &PING_VALUE.to_le_bytes());
        let (command, payload, end) = find_frame(&frame).unwrap();
        assert_eq!(command, CMD_PING);
        assert_eq!(read_u32(&payload, 0), Some(PING_VALUE));
        assert_eq!(end, frame.len());
    }

    #[test]
    fn rejects_bad_crc() {
        let mut frame = packet(CMD_PING, &PING_VALUE.to_le_bytes());
        let last = frame.len() - 1;
        frame[last] ^= 1;
        assert!(find_frame(&frame).is_none());
    }

    #[test]
    fn resyncs_past_false_magic() {
        // A bare magic pair with garbage behind it must not wedge the parser.
        let mut stream = vec![0xa5, 0x5a, 0xff, 0x00, 0x00, 0x00, 0x00];
        stream.extend_from_slice(&packet(CMD_PING, &PING_VALUE.to_le_bytes()));
        let (command, payload, _) = find_frame(&stream).unwrap();
        assert_eq!(command, CMD_PING);
        assert_eq!(read_u32(&payload, 0), Some(PING_VALUE));
    }

    #[test]
    fn does_not_stall_on_false_header_with_huge_length() {
        // Noise containing the magic followed by a large length field must not
        // block a real frame sitting behind it in the buffer.
        let mut stream = vec![0xa5, 0x5a, 0x00, 0x0f, 0xff, 0x00, 0x00];
        stream.extend_from_slice(&packet(CMD_ERASE, &1024u32.to_le_bytes()));
        let (command, payload, _) = find_frame(&stream).unwrap();
        assert_eq!(command, CMD_ERASE);
        assert_eq!(read_u32(&payload, 0), Some(1024));
    }

    #[test]
    fn waits_for_incomplete_frame() {
        let frame = packet(CMD_PING, &PING_VALUE.to_le_bytes());
        assert!(find_frame(&frame[..frame.len() - 1]).is_none());
    }
}
