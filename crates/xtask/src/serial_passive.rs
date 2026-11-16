//! Serial port passivity controller.
//!
//! HW-06 (H2-BUS-001): Ensure host CH340 serial port passivity.
//! Disables HUPCL and clears DTR/RTS without pulsing EN to avoid unwanted board resets.
//! Verifies camera HTTP status endpoint before and after to ensure device uptime continuity.

use std::env;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
#[cfg(target_os = "linux")]
use std::path::Path;
use std::time::Duration;

pub const DEFAULT_SERIAL_PORT: &str = "/dev/ttyUSB-esp32";

/// Parsed serial passivity options.
#[derive(Debug, Clone)]
pub struct SerialPassiveOptions {
    pub port: String,
    pub host: String,
    pub status_url: String,
    pub check_http: bool,
    pub timeout: Duration,
}

impl Default for SerialPassiveOptions {
    fn default() -> Self {
        let port = env::var("CAM_SERIAL_PORT").unwrap_or_else(|_| DEFAULT_SERIAL_PORT.to_string());
        let host = env::var("CAM_HOST").unwrap_or_default();
        let status_url = if !host.is_empty() {
            format!("http://{host}/status")
        } else {
            String::new()
        };

        Self {
            port,
            host,
            status_url,
            check_http: true,
            timeout: Duration::from_secs(3),
        }
    }
}

/// Runs the `ensure-serial-passive` subcommand.
///
/// # Errors
/// Returns an error message if argument parsing or serial configuration fails.
pub fn run(args: &[String]) -> Result<(), String> {
    if args.iter().any(|a| a == "--help" || a == "-h") {
        print_help();
        return Ok(());
    }

    let mut opts = SerialPassiveOptions::default();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--port" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --port".to_string());
                }
                opts.port = args[i].clone();
            }
            "--host" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --host".to_string());
                }
                opts.host = args[i].clone();
            }
            "--status-url" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --status-url".to_string());
                }
                opts.status_url = args[i].clone();
            }
            "--no-http-check" => {
                opts.check_http = false;
            }
            other => return Err(format!("Unknown argument '{other}'")),
        }
        i += 1;
    }

    if opts.status_url.is_empty() && !opts.host.is_empty() {
        opts.status_url = format!("http://{}/status", opts.host);
    }

    if opts.check_http && opts.status_url.is_empty() {
        println!(
            "[WARN] No CAM_HOST or --status-url specified (<camera-ip>); skipping HTTP checks."
        );
        opts.check_http = false;
    }

    ensure_passive(&opts)
}

/// Helper to extract string values from simple flat JSON strings.
#[must_use]
pub fn extract_json_str<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let pattern = format!("\"{key}\"");
    let key_pos = json.find(&pattern)?;
    let rest = &json[key_pos + pattern.len()..];
    let colon_pos = rest.find(':')?;
    let after_colon = rest[colon_pos + 1..].trim_start();
    if !after_colon.starts_with('"') {
        return None;
    }
    let str_content = &after_colon[1..];
    let end_quote = str_content.find('"')?;
    Some(&str_content[..end_quote])
}

/// Helper to extract integer values from simple flat JSON strings.
#[must_use]
pub fn extract_json_u64(json: &str, key: &str) -> Option<u64> {
    let pattern = format!("\"{key}\"");
    let key_pos = json.find(&pattern)?;
    let rest = &json[key_pos + pattern.len()..];
    let colon_pos = rest.find(':')?;
    let after_colon = rest[colon_pos + 1..].trim_start();
    let end_idx = after_colon
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(after_colon.len());
    if end_idx == 0 {
        return None;
    }
    after_colon[..end_idx].parse::<u64>().ok()
}

/// Camera status snapshot parsed from GET `/status`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraStatusSnapshot {
    pub uptime_ms: u64,
    pub active_partition: String,
}

/// Parses simple JSON payload from `/status`.
#[must_use]
pub fn parse_camera_status(raw: &str) -> Option<CameraStatusSnapshot> {
    let uptime_ms = extract_json_u64(raw, "uptime_ms")?;
    let active_partition = extract_json_str(raw, "active_partition")
        .unwrap_or("")
        .to_string();
    Some(CameraStatusSnapshot {
        uptime_ms,
        active_partition,
    })
}

/// Queries GET `/status` over raw TCP.
pub fn get_camera_status(status_url: &str, timeout: Duration) -> Option<CameraStatusSnapshot> {
    if status_url.is_empty() {
        return None;
    }

    let url_without_proto = status_url.strip_prefix("http://").unwrap_or(status_url);
    let (host_port, path) = match url_without_proto.find('/') {
        Some(slash) => (&url_without_proto[..slash], &url_without_proto[slash..]),
        None => (url_without_proto, "/"),
    };

    let (host, port) = match host_port.find(':') {
        Some(colon) => {
            let p = host_port[colon + 1..].parse::<u16>().unwrap_or(80);
            (&host_port[..colon], p)
        }
        None => (host_port, 80),
    };

    let addrs = format!("{host}:{port}").to_socket_addrs().ok()?;
    let mut last_err = None;
    for addr in addrs {
        match TcpStream::connect_timeout(&addr, timeout) {
            Ok(mut stream) => {
                let _ = stream.set_read_timeout(Some(timeout));
                let _ = stream.set_write_timeout(Some(timeout));
                let req = format!(
                    "GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: close\r\n\r\n"
                );
                if stream.write_all(req.as_bytes()).is_err() {
                    continue;
                }
                let mut resp_buf = Vec::new();
                if stream.read_to_end(&mut resp_buf).is_err() {
                    continue;
                }
                let resp_str = String::from_utf8_lossy(&resp_buf);
                if let Some(body_start) = resp_str.find("\r\n\r\n") {
                    let body = &resp_str[body_start + 4..];
                    if let Some(status) = parse_camera_status(body) {
                        return Some(status);
                    }
                }
            }
            Err(e) => last_err = Some(e),
        }
    }

    if let Some(e) = last_err {
        println!("[WARN] Failed to read camera status from {status_url}: {e}");
    }
    None
}

#[cfg(target_os = "linux")]
/// Configures serial port passivity on Linux systems.
pub fn ensure_passive(opts: &SerialPassiveOptions) -> Result<(), String> {
    if !Path::new(&opts.port).exists() {
        return Err(format!("[ERROR] Serial port {} does not exist", opts.port));
    }

    let pre_status = if opts.check_http && !opts.status_url.is_empty() {
        let status = get_camera_status(&opts.status_url, opts.timeout);
        if let Some(ref st) = status {
            println!(
                "[STATUS-PRE] Camera online, uptime_ms={}, active_partition={}",
                st.uptime_ms, st.active_partition
            );
        }
        status
    } else {
        None
    };

    let c_port = std::ffi::CString::new(opts.port.as_str())
        .map_err(|e| format!("Invalid port string: {e}"))?;

    // JUSTIFICATION: Opening character device with O_RDWR | O_NONBLOCK | O_NOCTTY to avoid blocking or acquiring a controlling terminal.
    // SAFETY: The path is a valid null-terminated CString; we check fd >= 0 immediately.
    let fd = unsafe {
        libc::open(
            c_port.as_ptr(),
            libc::O_RDWR | libc::O_NONBLOCK | libc::O_NOCTTY,
        )
    };
    if fd < 0 {
        return Err(format!(
            "Failed to open {}: {}",
            opts.port,
            std::io::Error::last_os_error()
        ));
    }

    // JUSTIFICATION: Querying and modifying terminal control flags to disable HUPCL.
    // SAFETY: fd is a valid open file descriptor; term is initialized and valid for tcgetattr/tcsetattr.
    let termios_res = unsafe {
        let mut term: libc::termios = std::mem::zeroed();
        let get_ret = libc::tcgetattr(fd, &mut term);
        if get_ret == 0 {
            #[allow(clippy::unnecessary_cast)]
            let mask = !(libc::HUPCL as libc::tcflag_t);
            term.c_cflag &= mask;
            libc::tcsetattr(fd, libc::TCSANOW, &term)
        } else {
            -1
        }
    };
    if termios_res != 0 {
        // JUSTIFICATION: Closing fd after failure.
        // SAFETY: fd is valid.
        unsafe { libc::close(fd) };
        return Err(format!(
            "Failed to configure termios HUPCL on {}: {}",
            opts.port,
            std::io::Error::last_os_error()
        ));
    }
    println!("[PASSIVITY] Disabled HUPCL via termios on {}", opts.port);

    // 2. Clear DTR and RTS via TIOCMBIC
    let clear_bits: libc::c_int = libc::TIOCM_DTR | libc::TIOCM_RTS;
    // JUSTIFICATION: Clearing DTR and RTS modem control lines to prevent board reset pulses.
    // SAFETY: fd is valid; &clear_bits is a valid pointer to c_int.
    let bic_res = unsafe { libc::ioctl(fd, libc::TIOCMBIC, &clear_bits) };
    if bic_res != 0 {
        // JUSTIFICATION: Closing fd after failure.
        // SAFETY: fd is valid.
        unsafe { libc::close(fd) };
        return Err(format!(
            "Failed to clear DTR/RTS on {}: {}",
            opts.port,
            std::io::Error::last_os_error()
        ));
    }

    // 3. Read back modem bits via TIOCMGET
    let mut modem_bits: libc::c_int = 0;
    // JUSTIFICATION: Reading back modem line state to verify DTR and RTS are deasserted.
    // SAFETY: fd is valid; &mut modem_bits is a valid pointer to c_int.
    let get_modem_res = unsafe { libc::ioctl(fd, libc::TIOCMGET, &mut modem_bits) };
    if get_modem_res != 0 {
        // JUSTIFICATION: Closing fd after failure.
        // SAFETY: fd is valid.
        unsafe { libc::close(fd) };
        return Err(format!(
            "Failed to read modem status on {}: {}",
            opts.port,
            std::io::Error::last_os_error()
        ));
    }

    let dtr_active = (modem_bits & libc::TIOCM_DTR) != 0;
    let rts_active = (modem_bits & libc::TIOCM_RTS) != 0;
    println!(
        "[PASSIVITY] TIOCMGET modem status: 0x{:04X} (DTR={}, RTS={})",
        modem_bits,
        if dtr_active { "1" } else { "0" },
        if rts_active { "1" } else { "0" }
    );

    // JUSTIFICATION: Closing serial device descriptor after configuration.
    // SAFETY: fd is valid.
    unsafe { libc::close(fd) };

    if dtr_active || rts_active {
        eprintln!("[WARN] DTR or RTS is still asserted!");
        return Err("DTR or RTS is still asserted after TIOCMBIC".to_string());
    }

    // 4. Verify camera uptime continuity
    if opts.check_http && !opts.status_url.is_empty() {
        let post_status = get_camera_status(&opts.status_url, opts.timeout);
        if let Some(post) = post_status {
            println!(
                "[STATUS-POST] Camera online, uptime_ms={}, active_partition={}",
                post.uptime_ms, post.active_partition
            );
            if let Some(pre) = pre_status {
                if post.uptime_ms < pre.uptime_ms {
                    eprintln!("[ERROR] Camera uptime reset during serial configuration!");
                    return Err("Camera uptime reset during serial configuration!".to_string());
                }
            }
            println!("[PASSIVITY] Serial port passive and camera uptime preserved successfully.");
        } else {
            eprintln!("[WARN] Could not reach camera after serial configuration");
            return Err("Could not reach camera after serial configuration".to_string());
        }
    } else {
        println!("[PASSIVITY] Serial port passive (HTTP check skipped).");
    }

    Ok(())
}

#[cfg(not(target_os = "linux"))]
/// Fallback for non-Linux platforms where serial passivity control is not supported.
pub fn ensure_passive(_opts: &SerialPassiveOptions) -> Result<(), String> {
    Err("Serial passivity control is only supported on Linux".to_string())
}

fn print_help() {
    println!("Usage: xtask ensure-serial-passive [OPTIONS]");
    println!();
    println!("Ensure host CH340 serial port passivity (disable HUPCL and clear DTR/RTS).");
    println!();
    println!("Options:");
    println!(
        "  --port <PORT>        Serial port path (default: $CAM_SERIAL_PORT or /dev/ttyUSB-esp32)"
    );
    println!("  --host <HOST>        Camera IP address (default: $CAM_HOST)");
    println!("  --status-url <URL>   Camera HTTP status URL (default: http://<camera-ip>/status)");
    println!(
        "  --no-http-check      Skip camera HTTP status checks (verify serial passivity only)"
    );
    println!("  -h, --help           Print help information");
}
