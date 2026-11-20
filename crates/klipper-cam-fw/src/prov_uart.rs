//! Setup-Mode-only UART0 provisioning CLI fallback (`#![no_std]`, zero heap).
//!
//! Provides a line-oriented serial command processor for configuring Wi-Fi
//! credentials, the private OTA authentication token, and IPv4 addressing mode
//! when the SoftAP captive portal is unavailable.
//!
//! Security properties:
//! - Never echoes staged or submitted passphrases or OTA tokens in replies,
//!   errors, or [`fmt::Debug`] output.
//! - Scrubs the command line buffer after every executed command, on
//!   [`UartProvSession::clear`], on successful `save`/`reboot`, and on [`Drop`]
//!   via volatile byte writes and a `SeqCst` compiler fence.

use core::fmt;
use core::fmt::Write;
use core::ptr;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

use crate::captive_net::AP_IP;
use crate::config::IpMode;
use crate::config::Ipv4Addr;
use crate::config::StaticIpConfig;
use crate::prov_store::ProvisionedConfig;
use crate::prov_store::MAX_OTA_TOKEN_LEN;
use crate::prov_store::MAX_PASS_LEN;
use crate::prov_store::MAX_SSID_LEN;
use crate::prov_store::MAX_STATIC_PREFIX_LEN;
use crate::prov_store::MIN_STATIC_PREFIX_LEN;

/// Fixed byte capacity of [`UartReply`].
pub const UART_REPLY_BUF_LEN: usize = 256;

/// Fixed byte capacity of the [`UartProvSession`] CR/LF command line buffer.
pub const UART_LINE_BUF_LEN: usize = 192;

/// Fixed-capacity `#![no_std]` UTF-8 reply buffer returned by
/// [`UartProvSession::feed_byte`].
#[derive(Clone, Eq)]
pub struct UartReply {
    buf: [u8; UART_REPLY_BUF_LEN],
    len: usize,
}

impl UartReply {
    /// Creates an empty [`UartReply`].
    #[inline]
    pub const fn new() -> Self {
        Self {
            buf: [0u8; UART_REPLY_BUF_LEN],
            len: 0,
        }
    }

    /// Creates a [`UartReply`] initialized with `text` (truncating cleanly at a
    /// UTF-8 boundary if `text` exceeds [`UART_REPLY_BUF_LEN`]).
    fn from_text(text: &str) -> Self {
        let mut reply = Self::new();
        let _ = reply.write_str(text);
        reply
    }

    /// Returns the formatted reply as a UTF-8 string slice.
    #[inline]
    pub fn as_str(&self) -> &str {
        let len = self.len.min(self.buf.len());
        core::str::from_utf8(&self.buf[..len]).unwrap_or("")
    }

    /// Returns the reply length in bytes.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if the reply contains zero bytes.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for UartReply {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for UartReply {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.as_str() == other.as_str()
    }
}

impl fmt::Debug for UartReply {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("UartReply").field(&self.as_str()).finish()
    }
}

impl fmt::Write for UartReply {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let remaining = self.buf.len().saturating_sub(self.len);
        if remaining == 0 {
            return Ok(());
        }
        let bytes = s.as_bytes();
        if bytes.len() <= remaining {
            self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
            self.len += bytes.len();
        } else {
            let mut cut = remaining;
            while cut > 0 && !s.is_char_boundary(cut) {
                cut -= 1;
            }
            self.buf[self.len..self.len + cut].copy_from_slice(&bytes[..cut]);
            self.len += cut;
        }
        Ok(())
    }
}

/// Action returned by [`UartProvSession::feed_byte`] for the UART driver loop.
#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(clippy::large_enum_variant)]
pub enum UartProvAction {
    /// No output or state transition yet (still accumulating a line or empty
    /// line ignored).
    None,
    /// Transmit the formatted reply back over UART0.
    Reply(UartReply),
    /// Persist the validated [`ProvisionedConfig`] to flash and reboot into
    /// station mode.
    SaveAndReboot(ProvisionedConfig),
    /// Reboot immediately without saving staged settings.
    Reboot,
}

/// Stateful, zero-heap Setup-Mode UART0 provisioning session.
pub struct UartProvSession {
    line: [u8; UART_LINE_BUF_LEN],
    line_len: usize,
    line_overflow: bool,
    ssid: [u8; MAX_SSID_LEN],
    ssid_len: u8,
    pass: [u8; MAX_PASS_LEN],
    pass_len: u8,
    pass_set: bool,
    token: [u8; MAX_OTA_TOKEN_LEN],
    token_len: u8,
    token_set: bool,
    ip_mode: IpMode,
}

impl UartProvSession {
    /// Creates a new empty [`UartProvSession`] defaulting to DHCP mode.
    #[inline]
    pub const fn new() -> Self {
        Self {
            line: [0u8; UART_LINE_BUF_LEN],
            line_len: 0,
            line_overflow: false,
            ssid: [0u8; MAX_SSID_LEN],
            ssid_len: 0,
            pass: [0u8; MAX_PASS_LEN],
            pass_len: 0,
            pass_set: false,
            token: [0u8; MAX_OTA_TOKEN_LEN],
            token_len: 0,
            token_set: false,
            ip_mode: IpMode::Dhcp,
        }
    }

    /// Feeds one raw byte from UART0 and returns the resulting
    /// [`UartProvAction`].
    ///
    /// Handles CR (`\r`), LF (`\n`), CRLF (`\r\n`), and backspace (`0x08` /
    /// `0x7F`). Oversized lines exceeding [`UART_LINE_BUF_LEN`] bytes are
    /// cleanly discarded and produce an error reply on the terminating newline.
    pub fn feed_byte(&mut self, byte: u8, mac: &[u8; 6], ap_ssid: &str) -> UartProvAction {
        match byte {
            b'\r' | b'\n' => {
                if self.line_overflow {
                    self.scrub_line();
                    return UartProvAction::Reply(UartReply::from_text(
                        "ERR: line too long (max 192 bytes)\r\n",
                    ));
                }
                if self.line_len == 0 {
                    return UartProvAction::None;
                }
                let action = self.process_line(mac, ap_ssid);
                self.scrub_line();
                action
            }
            0x08 | 0x7F => {
                if !self.line_overflow && self.line_len > 0 {
                    self.line_len -= 1;
                    volatile_zeroize(&mut self.line[self.line_len..=self.line_len]);
                }
                UartProvAction::None
            }
            _ => {
                if self.line_overflow {
                    return UartProvAction::None;
                }
                if self.line_len < self.line.len() {
                    self.line[self.line_len] = byte;
                    self.line_len += 1;
                } else {
                    self.scrub_line();
                    self.line_overflow = true;
                }
                UartProvAction::None
            }
        }
    }

    /// Zeroizes all staged credentials and line buffers and resets the session
    /// back to its initial DHCP state.
    pub fn clear(&mut self) {
        self.scrub_line();
        volatile_zeroize(&mut self.ssid);
        volatile_zeroize(&mut self.pass);
        volatile_zeroize(&mut self.token);
        self.ssid_len = 0;
        self.pass_len = 0;
        self.pass_set = false;
        self.token_len = 0;
        self.token_set = false;
        self.ip_mode = IpMode::Dhcp;
    }

    fn scrub_line(&mut self) {
        volatile_zeroize(&mut self.line);
        self.line_len = 0;
        self.line_overflow = false;
    }

    fn process_line(&mut self, mac: &[u8; 6], ap_ssid: &str) -> UartProvAction {
        let len = self.line_len.min(self.line.len());
        let mut line_copy = [0u8; UART_LINE_BUF_LEN];
        line_copy[..len].copy_from_slice(&self.line[..len]);

        let action = match core::str::from_utf8(&line_copy[..len]) {
            Ok(raw) => self.dispatch_command(raw, mac, ap_ssid),
            Err(_) => UartProvAction::Reply(UartReply::from_text("ERR: invalid UTF-8 input\r\n")),
        };

        volatile_zeroize(&mut line_copy);
        action
    }

    fn dispatch_command(&mut self, raw: &str, mac: &[u8; 6], ap_ssid: &str) -> UartProvAction {
        let trimmed = trim_leading_ascii_ws(raw);
        if trimmed.is_empty() {
            return UartProvAction::None;
        }
        let (cmd, rest) = split_first_token(trimmed);
        match cmd {
            "help" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text("ERR: usage: help\r\n"));
                }
                UartProvAction::Reply(format_help_reply())
            }
            "status" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text("ERR: usage: status\r\n"));
                }
                UartProvAction::Reply(self.format_status_reply(mac, ap_ssid))
            }
            "set" => self.handle_set(rest),
            "save" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text("ERR: usage: save\r\n"));
                }
                self.handle_save()
            }
            "clear" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text("ERR: usage: clear\r\n"));
                }
                self.clear();
                UartProvAction::Reply(UartReply::from_text("OK: staged config cleared\r\n"))
            }
            "reboot" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text("ERR: usage: reboot\r\n"));
                }
                self.clear();
                UartProvAction::Reboot
            }
            _ => UartProvAction::Reply(UartReply::from_text(
                "ERR: unknown command (type 'help')\r\n",
            )),
        }
    }

    fn handle_set(&mut self, args: &str) -> UartProvAction {
        let trimmed = trim_leading_ascii_ws(args);
        if trimmed.is_empty() {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: usage: set <ssid|pass|token|ip> <value>\r\n",
            ));
        }
        let (field, value) = split_first_token(trimmed);
        match field {
            "ssid" => self.handle_set_ssid(value),
            "pass" => self.handle_set_pass(value),
            "token" => self.handle_set_token(value),
            "ip" => self.handle_set_ip(value),
            _ => UartProvAction::Reply(UartReply::from_text(
                "ERR: unknown field (use ssid, pass, token, or ip)\r\n",
            )),
        }
    }

    fn handle_set_ssid(&mut self, value: &str) -> UartProvAction {
        let bytes = value.as_bytes();
        if bytes.is_empty() || bytes.len() > MAX_SSID_LEN {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: ssid must be 1..=32 bytes\r\n",
            ));
        }
        volatile_zeroize(&mut self.ssid);
        self.ssid[..bytes.len()].copy_from_slice(bytes);
        self.ssid_len = bytes.len() as u8;
        let mut reply = UartReply::new();
        let _ = write!(reply, "OK: ssid staged ({} bytes)\r\n", bytes.len());
        UartProvAction::Reply(reply)
    }

    fn handle_set_pass(&mut self, value: &str) -> UartProvAction {
        let bytes = value.as_bytes();
        if bytes.is_empty() || bytes.len() > MAX_PASS_LEN {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: pass must be 1..=63 bytes\r\n",
            ));
        }
        volatile_zeroize(&mut self.pass);
        self.pass[..bytes.len()].copy_from_slice(bytes);
        self.pass_len = bytes.len() as u8;
        self.pass_set = true;
        let mut reply = UartReply::new();
        let _ = write!(reply, "OK: pass staged ([set, {} bytes])\r\n", bytes.len());
        UartProvAction::Reply(reply)
    }

    fn handle_set_token(&mut self, value: &str) -> UartProvAction {
        let bytes = value.as_bytes();
        if bytes.is_empty() || bytes.len() > MAX_OTA_TOKEN_LEN {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: token must be 1..=64 bytes\r\n",
            ));
        }
        volatile_zeroize(&mut self.token);
        self.token[..bytes.len()].copy_from_slice(bytes);
        self.token_len = bytes.len() as u8;
        self.token_set = true;
        let mut reply = UartReply::new();
        let _ = write!(reply, "OK: token staged ([set, {} bytes])\r\n", bytes.len());
        UartProvAction::Reply(reply)
    }

    fn handle_set_ip(&mut self, value: &str) -> UartProvAction {
        let trimmed = trim_leading_ascii_ws(value);
        let (mode, rest) = split_first_token(trimmed);
        match mode {
            "dhcp" => {
                if !rest.trim().is_empty() {
                    return UartProvAction::Reply(UartReply::from_text(
                        "ERR: usage: set ip dhcp\r\n",
                    ));
                }
                self.ip_mode = IpMode::Dhcp;
                UartProvAction::Reply(UartReply::from_text("OK: ip mode set to dhcp\r\n"))
            }
            "static" => {
                let mut parts = rest.split_ascii_whitespace();
                let (Some(ip_s), Some(gw_s), Some(prefix_s), None) =
                    (parts.next(), parts.next(), parts.next(), parts.next())
                else {
                    return UartProvAction::Reply(UartReply::from_text(
                        "ERR: usage: set ip static <ip> <gw> <prefix>\r\n",
                    ));
                };
                let (Some(ip), Some(gateway), Some(prefix_len)) = (
                    Ipv4Addr::parse_ascii(ip_s),
                    Ipv4Addr::parse_ascii(gw_s),
                    parse_prefix_len(prefix_s),
                ) else {
                    return UartProvAction::Reply(UartReply::from_text(
                        "ERR: invalid static IPv4 address, gateway, or prefix (8..=30)\r\n",
                    ));
                };
                let netmask = Ipv4Addr((!0u32 << (32 - prefix_len)).to_be_bytes());
                self.ip_mode = IpMode::Static(StaticIpConfig {
                    ip,
                    netmask,
                    gateway,
                    dns: gateway,
                    prefix_len,
                });
                let mut reply = UartReply::new();
                let _ = write!(
                    reply,
                    "OK: ip mode set to static {ip}/{prefix_len} gw {gateway}\r\n"
                );
                UartProvAction::Reply(reply)
            }
            _ => UartProvAction::Reply(UartReply::from_text(
                "ERR: usage: set ip <dhcp | static <ip> <gw> <prefix>>\r\n",
            )),
        }
    }

    fn handle_save(&mut self) -> UartProvAction {
        let ssid_len = usize::from(self.ssid_len).min(self.ssid.len());
        let pass_len = usize::from(self.pass_len).min(self.pass.len());
        let token_len = usize::from(self.token_len).min(self.token.len());

        let Ok(ssid) = core::str::from_utf8(&self.ssid[..ssid_len]) else {
            return UartProvAction::Reply(UartReply::from_text("ERR: invalid SSID encoding\r\n"));
        };
        let Ok(pass) = core::str::from_utf8(&self.pass[..pass_len]) else {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: invalid passphrase encoding\r\n",
            ));
        };
        let Ok(token) = core::str::from_utf8(&self.token[..token_len]) else {
            return UartProvAction::Reply(UartReply::from_text(
                "ERR: invalid OTA token encoding\r\n",
            ));
        };

        match ProvisionedConfig::new(ssid, pass, token, self.ip_mode) {
            Ok(cfg) => {
                self.clear();
                UartProvAction::SaveAndReboot(cfg)
            }
            Err(err) => {
                let mut reply = UartReply::new();
                let _ = write!(reply, "ERR: {err}\r\n");
                UartProvAction::Reply(reply)
            }
        }
    }

    fn format_status_reply(&self, mac: &[u8; 6], ap_ssid: &str) -> UartReply {
        let mut reply = UartReply::new();
        let _ = write!(
            reply,
            "mac={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} ap={} ip={}.{}.{}.{}\r\n",
            mac[0],
            mac[1],
            mac[2],
            mac[3],
            mac[4],
            mac[5],
            ap_ssid,
            AP_IP[0],
            AP_IP[1],
            AP_IP[2],
            AP_IP[3],
        );

        let ssid_len = usize::from(self.ssid_len).min(self.ssid.len());
        if ssid_len == 0 {
            let _ = reply.write_str("ssid=[unset]\r\n");
        } else {
            let ssid = core::str::from_utf8(&self.ssid[..ssid_len]).unwrap_or("[invalid]");
            let _ = write!(reply, "ssid={ssid}\r\n");
        }

        if self.pass_set {
            let _ = write!(reply, "pass=[set, {} bytes] ", self.pass_len);
        } else {
            let _ = reply.write_str("pass=[unset] ");
        }

        if self.token_set {
            let _ = write!(reply, "token=[set, {} bytes]\r\n", self.token_len);
        } else {
            let _ = reply.write_str("token=[unset]\r\n");
        }

        match self.ip_mode {
            IpMode::Dhcp => {
                let _ = reply.write_str("mode=dhcp\r\n");
            }
            IpMode::Static(cfg) => {
                let _ = write!(
                    reply,
                    "mode=static {}/{} gw {}\r\n",
                    cfg.ip, cfg.prefix_len, cfg.gateway
                );
            }
        }
        reply
    }
}

impl Default for UartProvSession {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Debug for UartProvSession {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UartProvSession")
            .field("line_len", &self.line_len)
            .field("line_overflow", &self.line_overflow)
            .field("ssid_len", &self.ssid_len)
            .field("pass", &"[REDACTED]")
            .field("pass_set", &self.pass_set)
            .field("pass_len", &self.pass_len)
            .field("token", &"[REDACTED]")
            .field("token_set", &self.token_set)
            .field("token_len", &self.token_len)
            .field("ip_mode", &self.ip_mode)
            .finish()
    }
}

impl Drop for UartProvSession {
    fn drop(&mut self) {
        self.clear();
    }
}

/// Formats the concise Setup-Mode UART CLI help menu.
fn format_help_reply() -> UartReply {
    UartReply::from_text(
        "Commands:\r\n\
         help | status | clear | save | reboot\r\n\
         set ssid <val>\r\n\
         set pass <val>\r\n\
         set token <val>\r\n\
         set ip dhcp\r\n\
         set ip static <ip> <gw> <prefix>\r\n",
    )
}

/// Strips leading ASCII spaces and horizontal tabs.
#[inline]
fn trim_leading_ascii_ws(s: &str) -> &str {
    s.trim_start_matches([' ', '\t'])
}

/// Splits `s` into its first ASCII-whitespace-delimited token and the remainder
/// after any separating ASCII spaces/tabs.
fn split_first_token(s: &str) -> (&str, &str) {
    match s.find([' ', '\t']) {
        Some(idx) => (&s[..idx], trim_leading_ascii_ws(&s[idx..])),
        None => (s, ""),
    }
}

/// Parses a decimal CIDR prefix length strictly within
/// `MIN_STATIC_PREFIX_LEN..=MAX_STATIC_PREFIX_LEN` (`8..=30`).
fn parse_prefix_len(s: &str) -> Option<u8> {
    let bytes = s.as_bytes();
    if bytes.is_empty() || bytes.len() > 2 {
        return None;
    }
    let mut val = 0u8;
    for &b in bytes {
        if !b.is_ascii_digit() {
            return None;
        }
        val = val.checked_mul(10)?.checked_add(b - b'0')?;
    }
    if (MIN_STATIC_PREFIX_LEN..=MAX_STATIC_PREFIX_LEN).contains(&val) {
        Some(val)
    } else {
        None
    }
}

/// Scrubs `buf` with volatile writes so dead-store elimination cannot retain
/// command lines or secret bytes in RAM after `clear` or `Drop`.
fn volatile_zeroize(buf: &mut [u8]) {
    let ptr = buf.as_mut_ptr();
    let len = buf.len();
    for i in 0..len {
        // JUSTIFICATION: Compilers may eliminate ordinary zero assignments to
        // secret or line buffers during `clear` or `Drop` (dead-store
        // elimination). A volatile byte write is required in `#![no_std]` to
        // scrub Wi-Fi passphrases, OTA tokens, and UART command history from
        // RAM before release.
        // SAFETY:
        // 1. Preconditions: `i < len`, where `len == buf.len()`.
        // 2. Alignment: `u8` has alignment 1, so `ptr.add(i)` is always aligned.
        // 3. Provenance & Lifetime: `ptr.add(i)` stays within the live
        //    exclusively borrowed `buf` allocation for the entire loop.
        // 4. Aliasing: `&mut [u8]` guarantees exclusive access to every byte.
        // 5. Volatile Semantics: Writes `0u8` without reordering or elision.
        // 6. Panic Safety: Pointer offset within bounds and `write_volatile`
        //    cannot unwind.
        unsafe {
            ptr::write_volatile(ptr.add(i), 0u8);
        }
    }
    compiler_fence(Ordering::SeqCst);
}

#[cfg(test)]
mod tests {
    use alloc::format;

    use super::*;
    use crate::config::DEFAULT_OTA_TOKEN;

    const TEST_MAC: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    const TEST_AP_SSID: &str = "KlipperCam-EEFF";
    const TEST_SSID: &str = "Workshop-2.4G";
    const TEST_PASS: &str = "SuperSecretPass42!";
    const TEST_TOKEN: &str = "priv-ota-token-2026-x9k2";

    fn send_line(session: &mut UartProvSession, line: &str) -> UartProvAction {
        let mut last = UartProvAction::None;
        for &b in line.as_bytes() {
            let action = session.feed_byte(b, &TEST_MAC, TEST_AP_SSID);
            if action != UartProvAction::None {
                last = action;
            }
        }
        last
    }

    #[test]
    fn help_command_lists_supported_commands() {
        let mut session = UartProvSession::new();
        let action = send_line(&mut session, "help\r\n");
        let UartProvAction::Reply(reply) = action else {
            panic!("expected reply for help");
        };
        let text = reply.as_str();
        assert!(text.contains("help"));
        assert!(text.contains("status"));
        assert!(text.contains("set ssid"));
        assert!(text.contains("set pass"));
        assert!(text.contains("set token"));
        assert!(text.contains("set ip dhcp"));
        assert!(text.contains("set ip static"));
        assert!(text.contains("save"));
        assert!(text.contains("clear"));
        assert!(text.contains("reboot"));
    }

    #[test]
    fn status_shows_metadata_and_redacts_secrets() {
        let mut session = UartProvSession::new();

        let UartProvAction::Reply(initial) = send_line(&mut session, "status\n") else {
            panic!("expected status reply");
        };
        assert!(initial.as_str().contains("AA:BB:CC:DD:EE:FF"));
        assert!(initial.as_str().contains(TEST_AP_SSID));
        assert!(initial.as_str().contains("192.168.4.1"));
        assert!(initial.as_str().contains("ssid=[unset]"));
        assert!(initial.as_str().contains("pass=[unset]"));
        assert!(initial.as_str().contains("token=[unset]"));
        assert!(initial.as_str().contains("mode=dhcp"));

        let UartProvAction::Reply(pass_reply) =
            send_line(&mut session, &format!("set pass {TEST_PASS}\r\n"))
        else {
            panic!("expected reply for set pass");
        };
        assert!(!pass_reply.as_str().contains(TEST_PASS));
        assert!(pass_reply
            .as_str()
            .contains(&format!("[set, {} bytes]", TEST_PASS.len())));

        let UartProvAction::Reply(token_reply) =
            send_line(&mut session, &format!("set token {TEST_TOKEN}\r\n"))
        else {
            panic!("expected reply for set token");
        };
        assert!(!token_reply.as_str().contains(TEST_TOKEN));
        assert!(token_reply
            .as_str()
            .contains(&format!("[set, {} bytes]", TEST_TOKEN.len())));

        let _ = send_line(&mut session, &format!("set ssid {TEST_SSID}\r\n"));
        let _ = send_line(&mut session, "set ip static 172.22.1.52 172.22.1.1 24\r\n");

        let UartProvAction::Reply(staged) = send_line(&mut session, "status\r\n") else {
            panic!("expected staged status reply");
        };
        let text = staged.as_str();
        assert!(text.contains(TEST_SSID));
        assert!(text.contains(&format!("pass=[set, {} bytes]", TEST_PASS.len())));
        assert!(text.contains(&format!("token=[set, {} bytes]", TEST_TOKEN.len())));
        assert!(text.contains("mode=static 172.22.1.52/24 gw 172.22.1.1"));
        assert!(!text.contains(TEST_PASS));
        assert!(!text.contains(TEST_TOKEN));

        let dbg = format!("{session:?}");
        assert!(!dbg.contains(TEST_PASS));
        assert!(!dbg.contains(TEST_TOKEN));
    }

    #[test]
    fn save_succeeds_for_dhcp_and_static_and_zeroizes_staged_buffers() {
        let mut session = UartProvSession::new();
        let _ = send_line(&mut session, &format!("set ssid {TEST_SSID}\n"));
        let _ = send_line(&mut session, &format!("set pass {TEST_PASS}\n"));
        let _ = send_line(&mut session, &format!("set token {TEST_TOKEN}\n"));
        let _ = send_line(&mut session, "set ip dhcp\n");

        let action = send_line(&mut session, "save\r\n");
        let UartProvAction::SaveAndReboot(cfg) = action else {
            panic!("expected SaveAndReboot");
        };
        assert_eq!(cfg.ssid_str(), TEST_SSID);
        assert_eq!(cfg.pass_str(), TEST_PASS);
        assert_eq!(cfg.ota_token_str(), TEST_TOKEN);
        assert_eq!(cfg.ip_mode(), IpMode::Dhcp);

        // Staged buffers and line buffer must be zeroized after save.
        assert!(session.pass.iter().all(|&b| b == 0));
        assert!(session.token.iter().all(|&b| b == 0));
        assert!(session.line.iter().all(|&b| b == 0));
        assert!(!session.pass_set);
        assert!(!session.token_set);

        // Static IP round-trip.
        let _ = send_line(&mut session, "set ssid My Lab WiFi\r\n");
        let _ = send_line(&mut session, "set pass my secret pass 99\r\n");
        let _ = send_line(&mut session, &format!("set token {TEST_TOKEN}\r\n"));
        let _ = send_line(&mut session, "set ip static 172.22.1.52 172.22.1.1 24\r\n");

        let UartProvAction::SaveAndReboot(static_cfg) = send_line(&mut session, "save\n") else {
            panic!("expected SaveAndReboot for static config");
        };
        assert_eq!(static_cfg.ssid_str(), "My Lab WiFi");
        assert_eq!(static_cfg.pass_str(), "my secret pass 99");
        assert_eq!(
            static_cfg.ip_mode(),
            IpMode::Static(StaticIpConfig {
                ip: Ipv4Addr::new(172, 22, 1, 52),
                netmask: Ipv4Addr::new(255, 255, 255, 0),
                gateway: Ipv4Addr::new(172, 22, 1, 1),
                dns: Ipv4Addr::new(172, 22, 1, 1),
                prefix_len: 24,
            })
        );
    }

    #[test]
    fn save_validation_errors_never_echo_secrets() {
        let mut session = UartProvSession::new();

        // Missing SSID.
        let UartProvAction::Reply(err_ssid) = send_line(&mut session, "save\n") else {
            panic!("expected SSID error reply");
        };
        assert!(err_ssid.as_str().starts_with("ERR:"));

        // Short passphrase (not echoed).
        let _ = send_line(&mut session, &format!("set ssid {TEST_SSID}\n"));
        let _ = send_line(&mut session, "set pass tiny777\n");
        let _ = send_line(&mut session, &format!("set token {TEST_TOKEN}\n"));
        let UartProvAction::Reply(err_pass) = send_line(&mut session, "save\n") else {
            panic!("expected pass error reply");
        };
        assert!(err_pass.as_str().starts_with("ERR:"));
        assert!(!err_pass.as_str().contains("tiny777"));
        assert!(!err_pass.as_str().contains(TEST_TOKEN));

        // Factory-default OTA token rejected without echoing token.
        let _ = send_line(&mut session, &format!("set pass {TEST_PASS}\n"));
        let _ = send_line(&mut session, &format!("set token {DEFAULT_OTA_TOKEN}\n"));
        let UartProvAction::Reply(err_tok) = send_line(&mut session, "save\n") else {
            panic!("expected token error reply");
        };
        assert!(err_tok.as_str().starts_with("ERR:"));
        assert!(!err_tok.as_str().contains(DEFAULT_OTA_TOKEN));
        assert!(!err_tok.as_str().contains(TEST_PASS));

        // Mismatched static IP subnet rejected on save without echoing secrets.
        let _ = send_line(&mut session, &format!("set token {TEST_TOKEN}\n"));
        let _ = send_line(&mut session, "set ip static 172.22.1.52 10.0.0.1 24\n");
        let UartProvAction::Reply(err_ip) = send_line(&mut session, "save\n") else {
            panic!("expected static IP error reply");
        };
        assert!(err_ip.as_str().starts_with("ERR:"));
        assert!(!err_ip.as_str().contains(TEST_PASS));
        assert!(!err_ip.as_str().contains(TEST_TOKEN));

        // Malformed unknown command also never echoes secret payload.
        let UartProvAction::Reply(err_cmd) =
            send_line(&mut session, &format!("setpas {TEST_PASS}\n"))
        else {
            panic!("expected unknown command reply");
        };
        assert!(err_cmd.as_str().starts_with("ERR:"));
        assert!(!err_cmd.as_str().contains(TEST_PASS));
    }

    #[test]
    fn clear_and_reboot_commands_reset_state() {
        let mut session = UartProvSession::new();
        let _ = send_line(&mut session, &format!("set ssid {TEST_SSID}\n"));
        let _ = send_line(&mut session, &format!("set pass {TEST_PASS}\n"));
        let _ = send_line(&mut session, &format!("set token {TEST_TOKEN}\n"));
        let _ = send_line(&mut session, "set ip static 172.22.1.52 172.22.1.1 24\n");

        let UartProvAction::Reply(cleared) = send_line(&mut session, "clear\r\n") else {
            panic!("expected clear reply");
        };
        assert!(cleared.as_str().starts_with("OK:"));
        assert!(session.pass.iter().all(|&b| b == 0));
        assert!(session.token.iter().all(|&b| b == 0));
        assert_eq!(session.ssid_len, 0);
        assert_eq!(session.ip_mode, IpMode::Dhcp);

        let _ = send_line(&mut session, &format!("set pass {TEST_PASS}\n"));
        assert_eq!(
            send_line(&mut session, "reboot\r\n"),
            UartProvAction::Reboot
        );
        assert!(session.pass.iter().all(|&b| b == 0));
    }

    #[test]
    fn backspace_and_delete_edit_line_buffer() {
        let mut session = UartProvSession::new();
        // Type "set ssid FooXX<BS><DEL>Bar\r\n" -> "set ssid FooBar"
        for &b in b"set ssid FooXX" {
            assert_eq!(
                session.feed_byte(b, &TEST_MAC, TEST_AP_SSID),
                UartProvAction::None
            );
        }
        assert_eq!(
            session.feed_byte(0x08, &TEST_MAC, TEST_AP_SSID),
            UartProvAction::None
        );
        assert_eq!(
            session.feed_byte(0x7F, &TEST_MAC, TEST_AP_SSID),
            UartProvAction::None
        );
        let UartProvAction::Reply(reply) = send_line(&mut session, "Bar\r\n") else {
            panic!("expected reply after edited line");
        };
        assert!(reply.as_str().starts_with("OK:"));

        let UartProvAction::Reply(status) = send_line(&mut session, "status\n") else {
            panic!("expected status reply");
        };
        assert!(status.as_str().contains("ssid=FooBar\r\n"));
    }

    #[test]
    fn oversized_line_is_dropped_cleanly_with_error_on_newline() {
        let mut session = UartProvSession::new();
        // Feed 220 bytes (> 192-byte line buffer).
        for _ in 0..220 {
            assert_eq!(
                session.feed_byte(b'A', &TEST_MAC, TEST_AP_SSID),
                UartProvAction::None
            );
        }
        // Backspace during overflow does not recover truncated garbage.
        assert_eq!(
            session.feed_byte(0x08, &TEST_MAC, TEST_AP_SSID),
            UartProvAction::None
        );
        let action = session.feed_byte(b'\r', &TEST_MAC, TEST_AP_SSID);
        let UartProvAction::Reply(err) = action else {
            panic!("expected overflow error reply on CR");
        };
        assert!(err.as_str().starts_with("ERR:"));
        assert!(err.as_str().contains("192"));

        // Trailing LF of CRLF is ignored cleanly.
        assert_eq!(
            session.feed_byte(b'\n', &TEST_MAC, TEST_AP_SSID),
            UartProvAction::None
        );

        // Session remains usable for subsequent commands.
        let UartProvAction::Reply(help) = send_line(&mut session, "help\n") else {
            panic!("expected help reply after overflow recovery");
        };
        assert!(help.as_str().contains("Commands:"));
    }

    #[test]
    fn set_ip_static_rejects_bad_arguments_and_set_bounds_reject_oversized_values() {
        let mut session = UartProvSession::new();

        for bad_ip in [
            "set ip static 172.22.1.52 172.22.1.1\n",
            "set ip static 999.22.1.52 172.22.1.1 24\n",
            "set ip static 172.22.1.52 172.22.1.1 7\n",
            "set ip static 172.22.1.52 172.22.1.1 31\n",
            "set ip static 172.22.1.52 172.22.1.1 24 extra\n",
            "set ip unknown\n",
        ] {
            let UartProvAction::Reply(reply) = send_line(&mut session, bad_ip) else {
                panic!("expected error reply for {bad_ip:?}");
            };
            assert!(reply.as_str().starts_with("ERR:"));
        }

        let long_pass = "P".repeat(MAX_PASS_LEN + 1);
        let UartProvAction::Reply(pass_err) =
            send_line(&mut session, &format!("set pass {long_pass}\n"))
        else {
            panic!("expected oversized pass error");
        };
        assert!(pass_err.as_str().starts_with("ERR:"));
        assert!(!pass_err.as_str().contains(&long_pass));

        let long_token = "T".repeat(MAX_OTA_TOKEN_LEN + 1);
        let UartProvAction::Reply(tok_err) =
            send_line(&mut session, &format!("set token {long_token}\n"))
        else {
            panic!("expected oversized token error");
        };
        assert!(tok_err.as_str().starts_with("ERR:"));
        assert!(!tok_err.as_str().contains(&long_token));
    }
}
