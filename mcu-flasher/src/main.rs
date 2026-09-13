use std::{io::Cursor, path::PathBuf, process::ExitCode};
mod cc2_bootloader;
mod ymodem;
use clap::Parser;
use md5::{Digest, Md5};
use ymodem::Ymodem;

#[derive(Parser, Debug)]
#[command(
    name = "mcu-flasher",
    about = "Flash a new firmware over the Elegoo bootloader",
    version = "0.1"
)]
struct Args {
    /// Don't pad with 0x4000 bytes. The program auto-detects padded firmware by the magic (0x1418011A) at the start of the file, and adds it if the firmware isn't padded. This option force disables this functionality.
    #[arg(long, default_value_t = false)]
    pub no_pad_firmware: bool,

    /// Don't flash firmware and just boot the existing CC1 firmware.
    #[arg(long, default_value_t = false)]
    pub skip: bool,

    /// Probe the CC2 serial bootloader and ask it to jump to the application.
    /// This mode never reads or modifies a firmware file.
    #[arg(long, default_value_t = false)]
    pub cc2_boot: bool,

    /// Flash --firmware over the CC2 serial bootloader rather than YMODEM.
    /// The image is sent raw; the CC1 0x4000 header is never added.
    /// Reset the board after starting, so the ping lands in its 200ms window.
    #[arg(long, default_value_t = false)]
    pub cc2: bool,

    /// Ask a running Klipper application to reboot into the bootloader before
    /// probing, instead of waiting for a manual reset. Harmless if the board
    /// is already in the bootloader, which ignores the request as noise.
    #[arg(long, default_value_t = false)]
    pub cc2_request: bool,

    /// Baud the application runs at, when it differs from the bootloader's.
    /// Only used by --cc2-request, which must be sent at the app's rate.
    #[arg(long)]
    pub app_baud: Option<u32>,

    // Don't wait until the serial port is available.
    #[arg(long, default_value_t = false)]
    pub no_wait: bool,

    // Version of the firmware to flash
    #[arg(long, default_value = "1.2.3")]
    pub firmware_version: String,

    // Path to the firmware file
    #[arg(long, default_value = "")]
    pub firmware: String,

    #[arg(long)]
    pub baud: Option<u32>,

    // Serial port timeout in seconds (minimum 1)
    #[arg(long, default_value_t = 2, value_parser = clap::value_parser!(u32).range(1..))]
    pub timeout: u32,

    // Path to the device
    #[arg(required = true)]
    pub device: String,

    /// Use the Elegoo Canvas bootloader dialect (X0303 / E400 Lite):
    /// skip the initial wait for 'C', send SOH+128 block 0 with a plain
    /// `name\0size ` body, and retransmit on NAK.  Required for Canvas
    /// devices; stock Ymodem receivers without this flag.
    #[arg(long, default_value_t = false)]
    pub canvas: bool,
}

/// Drive cc2_bootloader::request_bootloader() from the CLI: the escape has to
/// go out at the *application's* baud, which is not always the bootloader's
/// (toolhead app is 250000, bed app is 500000), so switch across the write and
/// switch back before probing.
///
/// Safe to send blind. None of the escape's bytes is 0xa5, so a board already
/// in the bootloader stays in its resync state and discards all of them.
fn reboot_into_bootloader(
    port: &mut dyn serialport::SerialPort,
    boot_baud: u32,
    app_baud: Option<u32>,
) -> Result<(), cc2_bootloader::Error> {
    let app_baud = app_baud.unwrap_or(boot_baud);

    if app_baud != boot_baud {
        port.set_baud_rate(app_baud)?;
    }
    println!("Requesting bootloader from the application at {} baud", app_baud);
    cc2_bootloader::request_bootloader(port)?;

    // Just enough for the reset to get underway. Kept short on purpose: if the
    // board was already in the bootloader it did not reset, so its 200ms
    // auto-jump is still counting down and pinging needs to start promptly.
    std::thread::sleep(std::time::Duration::from_millis(50));

    if app_baud != boot_baud {
        port.set_baud_rate(boot_baud)?;
    }
    let _ = port.clear(serialport::ClearBuffer::Input);
    Ok(())
}

fn main() -> ExitCode {
    let mut args = Args::parse();

    let cc2_mode = args.cc2_boot || args.cc2;

    let baud = args.baud.unwrap_or(if cc2_mode { 250000 } else { 115200 });

    let split_version = args.firmware_version.split('.').collect::<Vec<&str>>();
    if split_version.len() != 3 {
        eprintln!("Version must be in the format X.Y.Z (e.g., 1.2.3).");
        return ExitCode::SUCCESS;
    }

    let major_version = match split_version[0].parse::<u8>() {
        Ok(v) => v,
        _ => {
            eprintln!("Invalid major version. Must be a number between 0 and 255.");
            return ExitCode::SUCCESS;
        }
    };

    let minor_version = match split_version[1].parse::<u8>() {
        Ok(v) => v,
        _ => {
            eprintln!("Invalid minor version. Must be a number between 0 and 255.");
            return ExitCode::SUCCESS;
        }
    };

    let patch_version = match split_version[2].parse::<u8>() {
        Ok(v) => v,
        _ => {
            eprintln!("Invalid patch version. Must be a number between 0 and 255.");
            return ExitCode::SUCCESS;
        }
    };

    let mut found = args.no_wait;

    while !found {
        let ports = serialport::available_ports().expect("No ports found!");
        for p in ports {
            if p.port_name == args.device {
                found = true;
            }
        }

        if !found {
            println!("Waiting for device at {}...", args.device);
            std::thread::sleep(std::time::Duration::from_secs(2));
        }
    }

    let port_builder = serialport::new(&args.device, baud)
        .timeout(std::time::Duration::from_secs(args.timeout as u64))
        // CC2 power sequencing is handled by the caller; avoid asserting DTR
        // while opening the temporary bootloader connection.
        .dtr_on_open(!cc2_mode);

    let mut port = match port_builder.open() {
        Ok(port) => port,
        Err(error) => {
            eprintln!("Failed to open {} at {} baud: {}", args.device, baud, error);
            return ExitCode::from(1);
        }
    };

    if args.cc2_request {
        if let Err(error) = reboot_into_bootloader(&mut *port, baud, args.app_baud) {
            eprintln!("Failed to request the bootloader: {}", error);
            return ExitCode::from(1);
        }
    }

    if args.cc2_boot {
        return match cc2_bootloader::boot(&mut *port, args.timeout) {
            Ok(cc2_bootloader::BootResult::JumpedToApp) => {
                println!("CC2 bootloader detected; requested application start");
                ExitCode::SUCCESS
            }
            Ok(cc2_bootloader::BootResult::AlreadyInApp) => {
                println!("CC2 bootloader not detected; leaving application untouched");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("CC2 bootloader initialization failed: {}", error);
                ExitCode::from(1)
            }
        };
    }

    if args.cc2 {
        if args.firmware.is_empty() || !PathBuf::from(&args.firmware).exists() {
            eprintln!("--cc2 requires --firmware <file>");
            return ExitCode::from(1);
        }

        let image = match std::fs::read(&args.firmware) {
            Ok(image) => image,
            Err(error) => {
                eprintln!("Failed to read {}: {}", args.firmware, error);
                return ExitCode::from(1);
            }
        };

        println!(
            "Waiting up to {}s for the CC2 bootloader on {} - reset the board now",
            args.timeout, args.device
        );

        let window = std::time::Duration::from_secs(args.timeout as u64);
        return match cc2_bootloader::deploy(&mut *port, &image, window) {
            Ok(()) => {
                println!("Flashed {} bytes; started application", image.len());
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("CC2 flash failed: {}", error);
                ExitCode::from(1)
            }
        };
    }

    if args.skip {
        println!(
            "Skipping flash. Booting existing firmware on device: {}",
            args.device
        );

        for _ in 0..16 {
            port.write(b"a").expect("Failed to write to port");
            port.flush().unwrap();
        }

        return ExitCode::SUCCESS;
    }

    if args.firmware.is_empty() || !PathBuf::from(&args.firmware).exists() {
        println!("No firmware file provided or file does not exist. Exiting.");
        return ExitCode::SUCCESS;
    }

    let file_name = PathBuf::from(&args.firmware)
        .file_name()
        .expect("Failed to get file name")
        .to_string_lossy()
        .to_string();

    let mut file_bytes = std::fs::read(&args.firmware).expect("Failed to read firmware file");

    let mut file_size_in_bytes = file_bytes.len() as u64;

    if file_bytes.starts_with(&vec![0x14, 0x18, 0x01, 0x1A]) {
        println!("Firmware file already has a header. No need to pad.");
        args.no_pad_firmware = true;
    }

    if !args.no_pad_firmware {
        let file_size = file_size_in_bytes as u32;

        let mut header = [0u8; 0x10];
        header[0x0..0x4].copy_from_slice(&vec![0x14, 0x18, 0x01, 0x1A]); // Magic
        header[0x4] = major_version; // Major version
        header[0x5] = minor_version; // Minor version
        header[0x6] = patch_version; // Patch version
        header[0x7] = 0xFF; // Board Type
        header[0x8] = 0x01; // Unknown
        header[0xC..0x10].copy_from_slice(&file_size.to_le_bytes());

        let mut hasher = Md5::new();
        hasher.update(&file_bytes);
        let checksum = hasher.finalize();

        println!("MD5 Checksum: {:x}", checksum);

        let padding = [0xFFu8; 0x4000 - 0x20];

        file_bytes = [&header[..], &checksum[..], &padding[..], &file_bytes[..]].concat();
        file_size_in_bytes = file_bytes.len() as u64;

        //fs::write("Z:/com.bin", &file_bytes).expect("Failed to write padded firmware file");
    }

    let mut cursor = Cursor::new(&mut file_bytes);

    let mut ymodem = Ymodem::new();
    ymodem.canvas = args.canvas;
    ymodem
        .send(&mut port, &mut cursor, file_name, file_size_in_bytes)
        .unwrap();

    ExitCode::SUCCESS
}
