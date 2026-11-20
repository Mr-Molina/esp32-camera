//! Dual-sector power-fail-safe flash provisioning store (`nvs` partition).
//!
//! Persists Wi-Fi station credentials, private OTA authentication token, and
//! IPv4 addressing mode (`Dhcp` or `Static`) across two 4 KiB SPI NOR flash
//! sectors (`PROV_SECTOR_A_OFFSET = 0x11000`, `PROV_SECTOR_B_OFFSET = 0x12000`)
//! using 256-byte IEEE 802.3 CRC32 records and monotonic sequence numbers.
//!
//! Writes always erase and program only the inactive sector, then read back
//! all 256 bytes to verify exact byte equality and CRC32 integrity before
//! returning. A brownout or torn write in one sector leaves the previously
//! committed sector intact and recoverable.

use core::fmt;
use core::ptr;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

use crate::config::{
    is_valid_ota_token, IpMode, Ipv4Addr, KlipperCamConfig, StaticIpConfig, ALLOW_OPEN_WIFI,
};
use crate::ota::{crc32_ieee, OtaError, OtaFlashStorage, FLASH_SECTOR_SIZE, NVS_OFFSET, NVS_SIZE};

/// Flash offset of provisioning sector A (`0x11000`, start of `nvs`).
pub const PROV_SECTOR_A_OFFSET: u32 = 0x0001_1000;

/// Flash offset of provisioning sector B (`0x12000`, second 4 KiB sector of
/// `nvs`).
pub const PROV_SECTOR_B_OFFSET: u32 = 0x0001_2000;

/// Fixed serialized record size in bytes (`256`, 4-byte aligned).
pub const PROV_RECORD_LEN: usize = 256;

/// 4-byte record magic header (`"KCFG"`).
pub const PROV_MAGIC: [u8; 4] = *b"KCFG";

/// Current binary record schema version (`1`).
pub const PROV_VERSION: u16 = 1;

/// Record flag indicating the sector holds a provisioned configuration.
pub const FLAG_PROVISIONED: u16 = 1 << 0;

/// Record flag indicating static IPv4 addressing (cleared for DHCP).
pub const FLAG_STATIC_IP: u16 = 1 << 1;

/// Maximum Wi-Fi SSID length in bytes (`32`).
pub const MAX_SSID_LEN: usize = 32;

/// Maximum WPA2-PSK passphrase length in bytes (`63`).
pub const MAX_PASS_LEN: usize = 63;

/// Maximum OTA authentication token length in bytes (`64`).
pub const MAX_OTA_TOKEN_LEN: usize = 64;

/// Minimum allowed CIDR prefix length for static IPv4 (`8`).
pub const MIN_STATIC_PREFIX_LEN: u8 = 8;

/// Maximum allowed CIDR prefix length for static IPv4 (`30`).
pub const MAX_STATIC_PREFIX_LEN: u8 = 30;

const KNOWN_FLAGS_MASK: u16 = FLAG_PROVISIONED | FLAG_STATIC_IP;
const OFF_MAGIC: usize = 0;
const OFF_VERSION: usize = 4;
const OFF_FLAGS: usize = 6;
const OFF_SEQ: usize = 8;
const OFF_SSID_LEN: usize = 12;
const OFF_SSID: usize = 13;
const OFF_PASS_LEN: usize = 45;
const OFF_PASS: usize = 46;
const OFF_TOKEN_LEN: usize = 109;
const OFF_TOKEN: usize = 110;
const OFF_STATIC_IP: usize = 174;
const OFF_GATEWAY: usize = 178;
const OFF_DNS: usize = 182;
const OFF_PREFIX_LEN: usize = 186;
const OFF_RESERVED: usize = 187;
const OFF_CRC: usize = 252;

const _: () = assert!(PROV_SECTOR_A_OFFSET >= NVS_OFFSET);
const _: () = assert!(PROV_SECTOR_B_OFFSET == PROV_SECTOR_A_OFFSET + FLASH_SECTOR_SIZE);
const _: () = assert!(PROV_SECTOR_B_OFFSET + FLASH_SECTOR_SIZE <= NVS_OFFSET + NVS_SIZE);
const _: () = assert!((PROV_RECORD_LEN & 3) == 0);
const _: () = assert!(OFF_SSID + MAX_SSID_LEN == OFF_PASS_LEN);
const _: () = assert!(OFF_PASS + MAX_PASS_LEN == OFF_TOKEN_LEN);
const _: () = assert!(OFF_TOKEN + MAX_OTA_TOKEN_LEN == OFF_STATIC_IP);
const _: () = assert!(OFF_CRC - OFF_RESERVED == 65);
const _: () = assert!(OFF_CRC + 4 == PROV_RECORD_LEN);

/// Errors returned by [`ProvisionedConfig`] validation and [`ProvStore`]
/// persistence operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProvStoreError {
    /// SSID is empty, longer than 32 bytes, invalid UTF-8, or contains
    /// control bytes.
    InvalidSsid,
    /// Passphrase is not `8..=63` printable ASCII bytes (or empty without
    /// `allow-open-wifi`).
    InvalidPassphrase,
    /// OTA token is shorter than 16 bytes, longer than 64 bytes, non-graphic
    /// ASCII, or factory default.
    InvalidOtaToken,
    /// Static IPv4 address, gateway, DNS, netmask, or prefix length (`8..=30`)
    /// is invalid.
    InvalidStaticIp,
    /// Monotonic sequence counter reached `u32::MAX - 1` and cannot wrap safely.
    SequenceExhausted,
    /// Underlying SPI NOR flash read, erase, or write failed.
    FlashError(OtaError),
    /// Post-write 256-byte readback or CRC32 check did not match the written
    /// record.
    ReadbackMismatch,
}

impl From<OtaError> for ProvStoreError {
    #[inline]
    fn from(err: OtaError) -> Self {
        Self::FlashError(err)
    }
}

impl fmt::Display for ProvStoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSsid => {
                f.write_str("SSID must be 1..=32 bytes of UTF-8 without control characters")
            }
            Self::InvalidPassphrase => {
                f.write_str("Wi-Fi passphrase must be 8..=63 printable ASCII bytes")
            }
            Self::InvalidOtaToken => f.write_str(
                "OTA token must be 16..=64 graphic ASCII bytes and not the factory default",
            ),
            Self::InvalidStaticIp => {
                f.write_str("static IPv4 config requires valid unicast IP/gateway/DNS and /8../30")
            }
            Self::SequenceExhausted => f.write_str("provisioning sequence counter exhausted"),
            Self::FlashError(err) => write!(f, "flash storage error: {err}"),
            Self::ReadbackMismatch => {
                f.write_str("provisioning flash readback verification failed")
            }
        }
    }
}

/// Computes the IPv4 subnet mask for a CIDR `prefix_len` in `8..=30`.
#[inline]
const fn netmask_from_prefix(prefix_len: u8) -> Option<Ipv4Addr> {
    if prefix_len < MIN_STATIC_PREFIX_LEN || prefix_len > MAX_STATIC_PREFIX_LEN {
        return None;
    }
    let mask = !0u32 << (32 - prefix_len);
    Some(Ipv4Addr(mask.to_be_bytes()))
}

/// Returns `true` if `addr` is a non-zero, non-loopback, non-multicast,
/// non-broadcast unicast IPv4 address.
#[inline]
const fn is_valid_unicast_ipv4(addr: Ipv4Addr) -> bool {
    let octets = addr.octets();
    let first = octets[0];
    !(first == 0 || first == 127 || first >= 224)
}

/// Validates a static IPv4 configuration for flash provisioning.
#[inline]
const fn validate_static_ip(cfg: &StaticIpConfig) -> Result<(), ProvStoreError> {
    let Some(expected_mask) = netmask_from_prefix(cfg.prefix_len) else {
        return Err(ProvStoreError::InvalidStaticIp);
    };
    if cfg.netmask.to_u32_be() != expected_mask.to_u32_be() {
        return Err(ProvStoreError::InvalidStaticIp);
    }
    if !is_valid_unicast_ipv4(cfg.ip)
        || !is_valid_unicast_ipv4(cfg.gateway)
        || !is_valid_unicast_ipv4(cfg.dns)
    {
        return Err(ProvStoreError::InvalidStaticIp);
    }
    let mask = expected_mask.to_u32_be();
    let host_mask = !mask;
    let ip_u32 = cfg.ip.to_u32_be();
    let gw_u32 = cfg.gateway.to_u32_be();
    let ip_host = ip_u32 & host_mask;
    let gw_host = gw_u32 & host_mask;
    if ip_host == 0 || ip_host == host_mask || gw_host == 0 || gw_host == host_mask {
        return Err(ProvStoreError::InvalidStaticIp);
    }
    if !cfg.is_valid() {
        return Err(ProvStoreError::InvalidStaticIp);
    }
    Ok(())
}

/// Validates SSID bytes (`1..=32` valid UTF-8, no ASCII control bytes).
#[inline]
const fn validate_ssid_bytes(bytes: &[u8]) -> Result<(), ProvStoreError> {
    if bytes.is_empty() || bytes.len() > MAX_SSID_LEN {
        return Err(ProvStoreError::InvalidSsid);
    }
    if core::str::from_utf8(bytes).is_err() {
        return Err(ProvStoreError::InvalidSsid);
    }
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        if b < 0x20 || b == 0x7F {
            return Err(ProvStoreError::InvalidSsid);
        }
        i += 1;
    }
    Ok(())
}

/// Validates passphrase bytes (`8..=63` printable ASCII `0x20..=0x7E`, or empty
/// only when compiled with `allow-open-wifi`).
#[inline]
const fn validate_pass_bytes(bytes: &[u8]) -> Result<(), ProvStoreError> {
    if bytes.is_empty() {
        if ALLOW_OPEN_WIFI {
            return Ok(());
        }
        return Err(ProvStoreError::InvalidPassphrase);
    }
    if bytes.len() < 8 || bytes.len() > MAX_PASS_LEN {
        return Err(ProvStoreError::InvalidPassphrase);
    }
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        if b < 0x20 || b > 0x7E {
            return Err(ProvStoreError::InvalidPassphrase);
        }
        i += 1;
    }
    Ok(())
}

/// Validates OTA token bytes (`16..=64` graphic ASCII and non-default).
#[inline]
const fn validate_token_bytes(bytes: &[u8]) -> Result<(), ProvStoreError> {
    if bytes.len() > MAX_OTA_TOKEN_LEN || !is_valid_ota_token(bytes) {
        return Err(ProvStoreError::InvalidOtaToken);
    }
    Ok(())
}

/// Scrubs `buf` with volatile writes so dead-store elimination cannot retain
/// secret bytes in RAM after `Drop`.
fn volatile_zeroize(buf: &mut [u8]) {
    let ptr = buf.as_mut_ptr();
    let len = buf.len();
    for i in 0..len {
        // JUSTIFICATION: Compilers may eliminate ordinary zero assignments to
        // secret buffers at the end of `Drop` (dead-store elimination). A
        // volatile byte write is required in `#![no_std]` to scrub Wi-Fi
        // passphrases and OTA tokens from RAM before release.
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

/// Copies `src` into `dst` (up to `src.len()`) in a `const`-compatible loop.
#[inline]
const fn copy_bytes_const<const N: usize>(src: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    let mut i = 0;
    while i < src.len() && i < N {
        out[i] = src[i];
        i += 1;
    }
    out
}

/// Fixed-size, `#![no_std]` provisioned device configuration.
///
/// Secrets (`pass` and `token`) are redacted in [`fmt::Debug`] and scrubbed
/// with volatile writes on [`Drop`].
#[derive(Clone, PartialEq, Eq)]
pub struct ProvisionedConfig {
    ssid: [u8; MAX_SSID_LEN],
    ssid_len: u8,
    pass: [u8; MAX_PASS_LEN],
    pass_len: u8,
    token: [u8; MAX_OTA_TOKEN_LEN],
    token_len: u8,
    ip_mode: IpMode,
}

impl ProvisionedConfig {
    /// Constructs and validates a [`ProvisionedConfig`] from borrowed strings
    /// and an [`IpMode`].
    pub const fn new(
        ssid: &str,
        pass: &str,
        ota_token: &str,
        ip_mode: IpMode,
    ) -> Result<Self, ProvStoreError> {
        let ssid_bytes = ssid.as_bytes();
        let pass_bytes = pass.as_bytes();
        let token_bytes = ota_token.as_bytes();

        if let Err(err) = validate_ssid_bytes(ssid_bytes) {
            return Err(err);
        }
        if let Err(err) = validate_pass_bytes(pass_bytes) {
            return Err(err);
        }
        if let Err(err) = validate_token_bytes(token_bytes) {
            return Err(err);
        }
        if let IpMode::Static(static_cfg) = ip_mode {
            if let Err(err) = validate_static_ip(&static_cfg) {
                return Err(err);
            }
        }

        Ok(Self {
            ssid: copy_bytes_const::<MAX_SSID_LEN>(ssid_bytes),
            ssid_len: ssid_bytes.len() as u8,
            pass: copy_bytes_const::<MAX_PASS_LEN>(pass_bytes),
            pass_len: pass_bytes.len() as u8,
            token: copy_bytes_const::<MAX_OTA_TOKEN_LEN>(token_bytes),
            token_len: token_bytes.len() as u8,
            ip_mode,
        })
    }

    /// Constructs and validates a DHCP-mode [`ProvisionedConfig`].
    #[inline]
    pub const fn new_dhcp(ssid: &str, pass: &str, ota_token: &str) -> Result<Self, ProvStoreError> {
        Self::new(ssid, pass, ota_token, IpMode::Dhcp)
    }

    /// Constructs and validates a static-IPv4 [`ProvisionedConfig`], deriving
    /// `netmask` from `prefix_len` (`8..=30`).
    #[inline]
    pub const fn new_static(
        ssid: &str,
        pass: &str,
        ota_token: &str,
        ip: Ipv4Addr,
        gateway: Ipv4Addr,
        dns: Ipv4Addr,
        prefix_len: u8,
    ) -> Result<Self, ProvStoreError> {
        let Some(netmask) = netmask_from_prefix(prefix_len) else {
            return Err(ProvStoreError::InvalidStaticIp);
        };
        let static_cfg = StaticIpConfig {
            ip,
            netmask,
            gateway,
            dns,
            prefix_len,
        };
        Self::new(ssid, pass, ota_token, IpMode::Static(static_cfg))
    }

    /// Returns the provisioned Wi-Fi SSID as a UTF-8 string slice.
    #[inline]
    pub fn ssid_str(&self) -> &str {
        let len = usize::from(self.ssid_len).min(self.ssid.len());
        core::str::from_utf8(&self.ssid[..len]).unwrap_or("")
    }

    /// Returns the provisioned Wi-Fi passphrase as a string slice.
    #[inline]
    pub fn pass_str(&self) -> &str {
        let len = usize::from(self.pass_len).min(self.pass.len());
        core::str::from_utf8(&self.pass[..len]).unwrap_or("")
    }

    /// Returns the provisioned OTA authentication token as a string slice.
    #[inline]
    pub fn ota_token_str(&self) -> &str {
        let len = usize::from(self.token_len).min(self.token.len());
        core::str::from_utf8(&self.token[..len]).unwrap_or("")
    }

    /// Returns the provisioned IPv4 assignment mode (`Dhcp` or `Static`).
    #[inline]
    pub const fn ip_mode(&self) -> IpMode {
        self.ip_mode
    }

    /// Re-validates all fields of this provisioning record.
    pub fn validate(&self) -> Result<(), ProvStoreError> {
        let ssid_len = usize::from(self.ssid_len);
        if ssid_len > self.ssid.len() {
            return Err(ProvStoreError::InvalidSsid);
        }
        validate_ssid_bytes(&self.ssid[..ssid_len])?;

        let pass_len = usize::from(self.pass_len);
        if pass_len > self.pass.len() {
            return Err(ProvStoreError::InvalidPassphrase);
        }
        validate_pass_bytes(&self.pass[..pass_len])?;

        let token_len = usize::from(self.token_len);
        if token_len > self.token.len() {
            return Err(ProvStoreError::InvalidOtaToken);
        }
        validate_token_bytes(&self.token[..token_len])?;

        if let IpMode::Static(static_cfg) = self.ip_mode {
            validate_static_ip(&static_cfg)?;
        }
        Ok(())
    }

    /// Overlays this validated provisioning configuration onto `cfg`.
    ///
    /// Requires `&'static self` (for example, initialized once in a
    /// `StaticCell` at boot) because [`KlipperCamConfig`] stores `&'static str`
    /// credential slices in `#![no_std]`.
    pub fn apply_to(&'static self, cfg: &mut KlipperCamConfig) -> Result<(), ProvStoreError> {
        self.validate()?;
        cfg.network.wifi_ssid = self.ssid_str();
        cfg.network.wifi_password = self.pass_str();
        cfg.ota_token = self.ota_token_str();
        cfg.network.ip_mode = self.ip_mode;
        if let IpMode::Static(static_cfg) = self.ip_mode {
            cfg.network.static_ip = static_cfg;
        }
        Ok(())
    }
}

impl fmt::Debug for ProvisionedConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ProvisionedConfig")
            .field("ssid", &self.ssid_str())
            .field("pass", &"[REDACTED]")
            .field("pass_len", &self.pass_len)
            .field("ota_token", &"[REDACTED]")
            .field("ota_token_len", &self.token_len)
            .field("ip_mode", &self.ip_mode)
            .finish()
    }
}

impl Drop for ProvisionedConfig {
    fn drop(&mut self) {
        volatile_zeroize(&mut self.pass);
        volatile_zeroize(&mut self.token);
        self.pass_len = 0;
        self.token_len = 0;
    }
}

/// 4-byte-aligned 256-byte scratch buffer for flash reads/writes, zeroized on
/// drop so secret bytes do not linger on the stack.
#[repr(C, align(4))]
struct AlignedProvBuf([u8; PROV_RECORD_LEN]);

impl AlignedProvBuf {
    #[inline]
    const fn zeroed() -> Self {
        Self([0u8; PROV_RECORD_LEN])
    }
}

impl Drop for AlignedProvBuf {
    fn drop(&mut self) {
        volatile_zeroize(&mut self.0);
    }
}

/// Active flash sector identifier for A/B ping-pong persistence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActiveSector {
    SectorA,
    SectorB,
}

/// Stateless brownout-safe dual-sector provisioning store backed by
/// [`OtaFlashStorage`].
#[derive(Debug, Clone, Copy, Default)]
pub struct ProvStore;

impl ProvStore {
    /// Loads the newest valid provisioning record from sectors A and B.
    ///
    /// Returns `Ok(None)` when both sectors are blank (`0xFF`) or invalid.
    pub fn load<F: OtaFlashStorage>(
        flash: &mut F,
    ) -> Result<Option<ProvisionedConfig>, ProvStoreError> {
        Ok(Self::load_active(flash)?.map(|(_, _, cfg)| cfg))
    }

    /// Persists `cfg` into the inactive 4 KiB sector with `seq = active_seq + 1`,
    /// verifies exact 256-byte readback equality and CRC32, and returns the
    /// newly committed sequence number.
    pub fn save<F: OtaFlashStorage>(
        flash: &mut F,
        cfg: &ProvisionedConfig,
    ) -> Result<u32, ProvStoreError> {
        cfg.validate()?;

        let (target_offset, next_seq) = match Self::load_active(flash)? {
            Some((ActiveSector::SectorA, active_seq, _)) => {
                (PROV_SECTOR_B_OFFSET, next_valid_seq(active_seq)?)
            }
            Some((ActiveSector::SectorB, active_seq, _)) => {
                (PROV_SECTOR_A_OFFSET, next_valid_seq(active_seq)?)
            }
            None => (PROV_SECTOR_A_OFFSET, 1),
        };

        let mut encoded = AlignedProvBuf::zeroed();
        encode_record(cfg, next_seq, &mut encoded.0);

        flash.erase_range(target_offset, target_offset + FLASH_SECTOR_SIZE)?;
        flash.write_aligned(target_offset, &encoded.0)?;

        let mut readback = AlignedProvBuf::zeroed();
        flash.read_aligned(target_offset, &mut readback.0)?;
        if readback.0 != encoded.0 {
            return Err(ProvStoreError::ReadbackMismatch);
        }
        let stored_crc = u32::from_le_bytes([
            readback.0[OFF_CRC],
            readback.0[OFF_CRC + 1],
            readback.0[OFF_CRC + 2],
            readback.0[OFF_CRC + 3],
        ]);
        if crc32_ieee(&readback.0[..OFF_CRC]) != stored_crc {
            return Err(ProvStoreError::ReadbackMismatch);
        }

        Ok(next_seq)
    }

    /// Erases both provisioning sectors (`0x11000` and `0x12000`).
    pub fn erase_all<F: OtaFlashStorage>(flash: &mut F) -> Result<(), ProvStoreError> {
        flash.erase_range(
            PROV_SECTOR_A_OFFSET,
            PROV_SECTOR_A_OFFSET + FLASH_SECTOR_SIZE,
        )?;
        flash.erase_range(
            PROV_SECTOR_B_OFFSET,
            PROV_SECTOR_B_OFFSET + FLASH_SECTOR_SIZE,
        )?;
        Ok(())
    }

    fn load_active<F: OtaFlashStorage>(
        flash: &mut F,
    ) -> Result<Option<(ActiveSector, u32, ProvisionedConfig)>, ProvStoreError> {
        let mut buf_a = AlignedProvBuf::zeroed();
        flash.read_aligned(PROV_SECTOR_A_OFFSET, &mut buf_a.0)?;
        let rec_a = decode_record(&buf_a.0);

        let mut buf_b = AlignedProvBuf::zeroed();
        flash.read_aligned(PROV_SECTOR_B_OFFSET, &mut buf_b.0)?;
        let rec_b = decode_record(&buf_b.0);

        let active = match (rec_a, rec_b) {
            (Some((seq_a, cfg_a)), Some((seq_b, cfg_b))) => {
                if seq_b > seq_a {
                    Some((ActiveSector::SectorB, seq_b, cfg_b))
                } else {
                    Some((ActiveSector::SectorA, seq_a, cfg_a))
                }
            }
            (Some((seq_a, cfg_a)), None) => Some((ActiveSector::SectorA, seq_a, cfg_a)),
            (None, Some((seq_b, cfg_b))) => Some((ActiveSector::SectorB, seq_b, cfg_b)),
            (None, None) => None,
        };
        Ok(active)
    }
}

/// Computes `active_seq + 1` while rejecting wrap into `u32::MAX`.
#[inline]
const fn next_valid_seq(active_seq: u32) -> Result<u32, ProvStoreError> {
    match active_seq.checked_add(1) {
        Some(next) if next < u32::MAX => Ok(next),
        _ => Err(ProvStoreError::SequenceExhausted),
    }
}

/// Encodes a validated [`ProvisionedConfig`] and `seq` into a 256-byte record.
fn encode_record(cfg: &ProvisionedConfig, seq: u32, out: &mut [u8; PROV_RECORD_LEN]) {
    out.fill(0);
    out[OFF_MAGIC..OFF_MAGIC + 4].copy_from_slice(&PROV_MAGIC);
    out[OFF_VERSION..OFF_VERSION + 2].copy_from_slice(&PROV_VERSION.to_le_bytes());

    let mut flags = FLAG_PROVISIONED;
    if let IpMode::Static(static_cfg) = cfg.ip_mode {
        flags |= FLAG_STATIC_IP;
        out[OFF_STATIC_IP..OFF_STATIC_IP + 4].copy_from_slice(&static_cfg.ip.octets());
        out[OFF_GATEWAY..OFF_GATEWAY + 4].copy_from_slice(&static_cfg.gateway.octets());
        out[OFF_DNS..OFF_DNS + 4].copy_from_slice(&static_cfg.dns.octets());
        out[OFF_PREFIX_LEN] = static_cfg.prefix_len;
    }
    out[OFF_FLAGS..OFF_FLAGS + 2].copy_from_slice(&flags.to_le_bytes());
    out[OFF_SEQ..OFF_SEQ + 4].copy_from_slice(&seq.to_le_bytes());

    let ssid_len = usize::from(cfg.ssid_len).min(MAX_SSID_LEN);
    out[OFF_SSID_LEN] = ssid_len as u8;
    out[OFF_SSID..OFF_SSID + ssid_len].copy_from_slice(&cfg.ssid[..ssid_len]);

    let pass_len = usize::from(cfg.pass_len).min(MAX_PASS_LEN);
    out[OFF_PASS_LEN] = pass_len as u8;
    out[OFF_PASS..OFF_PASS + pass_len].copy_from_slice(&cfg.pass[..pass_len]);

    let token_len = usize::from(cfg.token_len).min(MAX_OTA_TOKEN_LEN);
    out[OFF_TOKEN_LEN] = token_len as u8;
    out[OFF_TOKEN..OFF_TOKEN + token_len].copy_from_slice(&cfg.token[..token_len]);

    let crc = crc32_ieee(&out[..OFF_CRC]);
    out[OFF_CRC..OFF_CRC + 4].copy_from_slice(&crc.to_le_bytes());
}

/// Decodes and validates a 256-byte flash record, returning `None` on any
/// structural, reserved-field, CRC32, or semantic validation mismatch.
fn decode_record(raw: &[u8; PROV_RECORD_LEN]) -> Option<(u32, ProvisionedConfig)> {
    if raw[OFF_MAGIC..OFF_MAGIC + 4] != PROV_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([raw[OFF_VERSION], raw[OFF_VERSION + 1]]);
    if version != PROV_VERSION {
        return None;
    }
    let flags = u16::from_le_bytes([raw[OFF_FLAGS], raw[OFF_FLAGS + 1]]);
    if (flags & FLAG_PROVISIONED) == 0 || (flags & !KNOWN_FLAGS_MASK) != 0 {
        return None;
    }
    let seq = u32::from_le_bytes([
        raw[OFF_SEQ],
        raw[OFF_SEQ + 1],
        raw[OFF_SEQ + 2],
        raw[OFF_SEQ + 3],
    ]);
    if seq == 0 || seq == u32::MAX {
        return None;
    }
    if raw[OFF_RESERVED..OFF_CRC].iter().any(|&b| b != 0) {
        return None;
    }
    let stored_crc = u32::from_le_bytes([
        raw[OFF_CRC],
        raw[OFF_CRC + 1],
        raw[OFF_CRC + 2],
        raw[OFF_CRC + 3],
    ]);
    if crc32_ieee(&raw[..OFF_CRC]) != stored_crc {
        return None;
    }

    let ssid_len = usize::from(raw[OFF_SSID_LEN]);
    let pass_len = usize::from(raw[OFF_PASS_LEN]);
    let token_len = usize::from(raw[OFF_TOKEN_LEN]);
    if ssid_len == 0
        || ssid_len > MAX_SSID_LEN
        || pass_len > MAX_PASS_LEN
        || token_len > MAX_OTA_TOKEN_LEN
    {
        return None;
    }

    let mut ssid = [0u8; MAX_SSID_LEN];
    ssid[..ssid_len].copy_from_slice(&raw[OFF_SSID..OFF_SSID + ssid_len]);

    let mut pass = [0u8; MAX_PASS_LEN];
    pass[..pass_len].copy_from_slice(&raw[OFF_PASS..OFF_PASS + pass_len]);

    let mut token = [0u8; MAX_OTA_TOKEN_LEN];
    token[..token_len].copy_from_slice(&raw[OFF_TOKEN..OFF_TOKEN + token_len]);

    let ip_mode = if (flags & FLAG_STATIC_IP) != 0 {
        let prefix_len = raw[OFF_PREFIX_LEN];
        let netmask = netmask_from_prefix(prefix_len)?;
        let ip = Ipv4Addr([
            raw[OFF_STATIC_IP],
            raw[OFF_STATIC_IP + 1],
            raw[OFF_STATIC_IP + 2],
            raw[OFF_STATIC_IP + 3],
        ]);
        let gateway = Ipv4Addr([
            raw[OFF_GATEWAY],
            raw[OFF_GATEWAY + 1],
            raw[OFF_GATEWAY + 2],
            raw[OFF_GATEWAY + 3],
        ]);
        let dns = Ipv4Addr([
            raw[OFF_DNS],
            raw[OFF_DNS + 1],
            raw[OFF_DNS + 2],
            raw[OFF_DNS + 3],
        ]);
        IpMode::Static(StaticIpConfig {
            ip,
            netmask,
            gateway,
            dns,
            prefix_len,
        })
    } else {
        IpMode::Dhcp
    };

    let cfg = ProvisionedConfig {
        ssid,
        ssid_len: ssid_len as u8,
        pass,
        pass_len: pass_len as u8,
        token,
        token_len: token_len as u8,
        ip_mode,
    };
    cfg.validate().ok()?;
    Some((seq, cfg))
}

#[cfg(test)]
mod tests {
    use alloc::format;

    use super::*;
    use crate::config::DEFAULT_OTA_TOKEN;

    const VALID_SSID: &str = "Workshop-2.4G";
    const VALID_PASS: &str = "SuperSecretPass42!";
    const VALID_TOKEN: &str = "priv-ota-token-2026-x9k2";

    struct MockProvFlash {
        sector_a: [u8; FLASH_SECTOR_SIZE as usize],
        sector_b: [u8; FLASH_SECTOR_SIZE as usize],
        erase_count: u32,
        write_count: u32,
        last_erased: Option<u32>,
        last_written: Option<u32>,
        corrupt_readback_after_write: bool,
        corrupt_next_read: bool,
        fail_writes: bool,
    }

    impl MockProvFlash {
        fn new() -> Self {
            Self {
                sector_a: [0xFF; FLASH_SECTOR_SIZE as usize],
                sector_b: [0xFF; FLASH_SECTOR_SIZE as usize],
                erase_count: 0,
                write_count: 0,
                last_erased: None,
                last_written: None,
                corrupt_readback_after_write: false,
                corrupt_next_read: false,
                fail_writes: false,
            }
        }

        fn region_mut(&mut self, offset: u32, len: usize) -> Result<&mut [u8], OtaError> {
            let len_u32 = u32::try_from(len).map_err(|_| OtaError::FlashWriteError)?;
            let end = offset
                .checked_add(len_u32)
                .ok_or(OtaError::FlashWriteError)?;
            if offset >= PROV_SECTOR_A_OFFSET && end <= PROV_SECTOR_A_OFFSET + FLASH_SECTOR_SIZE {
                let start = (offset - PROV_SECTOR_A_OFFSET) as usize;
                Ok(&mut self.sector_a[start..start + len])
            } else if offset >= PROV_SECTOR_B_OFFSET
                && end <= PROV_SECTOR_B_OFFSET + FLASH_SECTOR_SIZE
            {
                let start = (offset - PROV_SECTOR_B_OFFSET) as usize;
                Ok(&mut self.sector_b[start..start + len])
            } else {
                Err(OtaError::FlashWriteError)
            }
        }
    }

    impl OtaFlashStorage for MockProvFlash {
        fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
            if (offset & 3) != 0 || (buf.len() & 3) != 0 {
                return Err(OtaError::FlashWriteError);
            }
            let corrupt = self.corrupt_next_read;
            if corrupt {
                self.corrupt_next_read = false;
            }
            let slice = self.region_mut(offset, buf.len())?;
            buf.copy_from_slice(slice);
            if corrupt && !buf.is_empty() {
                buf[0] ^= 0x01;
            }
            Ok(())
        }

        fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
            if self.fail_writes
                || (from & (FLASH_SECTOR_SIZE - 1)) != 0
                || (to & (FLASH_SECTOR_SIZE - 1)) != 0
                || from >= to
            {
                return Err(OtaError::FlashWriteError);
            }
            let len = (to - from) as usize;
            let slice = self.region_mut(from, len)?;
            slice.fill(0xFF);
            self.erase_count += 1;
            self.last_erased = Some(from);
            Ok(())
        }

        fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
            if self.fail_writes || (offset & 3) != 0 || (data.len() & 3) != 0 {
                return Err(OtaError::FlashWriteError);
            }
            let slice = self.region_mut(offset, data.len())?;
            for (dst, &src) in slice.iter_mut().zip(data.iter()) {
                *dst &= src;
            }
            self.write_count += 1;
            self.last_written = Some(offset);
            if self.corrupt_readback_after_write {
                self.corrupt_readback_after_write = false;
                self.corrupt_next_read = true;
            }
            Ok(())
        }
    }

    #[test]
    fn blank_flash_loads_none() {
        let mut flash = MockProvFlash::new();
        let loaded = ProvStore::load(&mut flash);
        assert_eq!(loaded, Ok(None));
    }

    #[test]
    fn dhcp_save_and_load_roundtrip() {
        let mut flash = MockProvFlash::new();
        let cfg = ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, VALID_TOKEN)
            .expect("valid DHCP config");

        let seq = ProvStore::save(&mut flash, &cfg).expect("save succeeds");
        assert_eq!(seq, 1);
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_A_OFFSET));
        assert_eq!(flash.last_written, Some(PROV_SECTOR_A_OFFSET));

        let loaded = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("config present");
        assert_eq!(loaded, cfg);
        assert_eq!(loaded.ssid_str(), VALID_SSID);
        assert_eq!(loaded.pass_str(), VALID_PASS);
        assert_eq!(loaded.ota_token_str(), VALID_TOKEN);
        assert_eq!(loaded.ip_mode(), IpMode::Dhcp);
    }

    #[test]
    fn static_ip_save_and_load_roundtrip() {
        let mut flash = MockProvFlash::new();
        let cfg = ProvisionedConfig::new_static(
            VALID_SSID,
            VALID_PASS,
            VALID_TOKEN,
            Ipv4Addr::new(172, 22, 1, 52),
            Ipv4Addr::new(172, 22, 1, 1),
            Ipv4Addr::new(1, 1, 1, 1),
            24,
        )
        .expect("valid static config");

        let seq = ProvStore::save(&mut flash, &cfg).expect("save succeeds");
        assert_eq!(seq, 1);

        let loaded = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("config present");
        assert_eq!(loaded, cfg);
        assert_eq!(
            loaded.ip_mode(),
            IpMode::Static(StaticIpConfig {
                ip: Ipv4Addr::new(172, 22, 1, 52),
                netmask: Ipv4Addr::new(255, 255, 255, 0),
                gateway: Ipv4Addr::new(172, 22, 1, 1),
                dns: Ipv4Addr::new(1, 1, 1, 1),
                prefix_len: 24,
            })
        );
    }

    #[test]
    fn alternating_ab_sectors_and_monotonic_seq() {
        let mut flash = MockProvFlash::new();

        let cfg1 = ProvisionedConfig::new_dhcp("Net-One", VALID_PASS, VALID_TOKEN)
            .expect("valid config 1");
        let cfg2 = ProvisionedConfig::new_dhcp("Net-Two", VALID_PASS, VALID_TOKEN)
            .expect("valid config 2");
        let cfg3 = ProvisionedConfig::new_dhcp("Net-Three", VALID_PASS, VALID_TOKEN)
            .expect("valid config 3");
        let cfg4 = ProvisionedConfig::new_dhcp("Net-Four", VALID_PASS, VALID_TOKEN)
            .expect("valid config 4");

        assert_eq!(ProvStore::save(&mut flash, &cfg1), Ok(1));
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_A_OFFSET));
        assert_eq!(flash.erase_count, 1);

        assert_eq!(ProvStore::save(&mut flash, &cfg2), Ok(2));
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_B_OFFSET));
        assert_eq!(flash.erase_count, 2);

        assert_eq!(ProvStore::save(&mut flash, &cfg3), Ok(3));
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_A_OFFSET));
        assert_eq!(flash.erase_count, 3);

        assert_eq!(ProvStore::save(&mut flash, &cfg4), Ok(4));
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_B_OFFSET));
        assert_eq!(flash.erase_count, 4);

        let loaded = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("config present");
        assert_eq!(loaded.ssid_str(), "Net-Four");
    }

    #[test]
    fn brownout_corrupt_crc_in_sector_b_recovers_sector_a_seq3_and_writes_seq4() {
        let mut flash = MockProvFlash::new();
        let cfg1 = ProvisionedConfig::new_dhcp("Boot-1", VALID_PASS, VALID_TOKEN).expect("cfg1");
        let cfg2 = ProvisionedConfig::new_dhcp("Boot-2", VALID_PASS, VALID_TOKEN).expect("cfg2");
        let cfg3 =
            ProvisionedConfig::new_dhcp("Stable-Seq3", VALID_PASS, VALID_TOKEN).expect("cfg3");
        let cfg4 =
            ProvisionedConfig::new_dhcp("Recovered-Seq4", VALID_PASS, VALID_TOKEN).expect("cfg4");

        assert_eq!(ProvStore::save(&mut flash, &cfg1), Ok(1)); // Sector A
        assert_eq!(ProvStore::save(&mut flash, &cfg2), Ok(2)); // Sector B
        assert_eq!(ProvStore::save(&mut flash, &cfg3), Ok(3)); // Sector A

        // Simulate a brownout that wrote seq=4 into Sector B with a corrupted byte.
        let mut corrupt_rec = [0u8; PROV_RECORD_LEN];
        encode_record(&cfg4, 4, &mut corrupt_rec);
        corrupt_rec[OFF_SSID] ^= 0xFF;
        flash.sector_b[..PROV_RECORD_LEN].copy_from_slice(&corrupt_rec);

        let recovered = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("sector A recovered");
        assert_eq!(recovered.ssid_str(), "Stable-Seq3");

        // Next save must erase ONLY Sector B and commit seq = 4 there.
        let sector_a_before = flash.sector_a;
        let committed_seq = ProvStore::save(&mut flash, &cfg4).expect("save seq 4");
        assert_eq!(committed_seq, 4);
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_B_OFFSET));
        assert_eq!(flash.last_written, Some(PROV_SECTOR_B_OFFSET));
        assert_eq!(flash.sector_a, sector_a_before);

        let loaded = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("seq 4 active");
        assert_eq!(loaded.ssid_str(), "Recovered-Seq4");
    }

    #[test]
    fn brownout_torn_ff_write_in_sector_b_recovers_sector_a_seq3_and_writes_seq4() {
        let mut flash = MockProvFlash::new();
        let cfg1 = ProvisionedConfig::new_dhcp("Boot-1", VALID_PASS, VALID_TOKEN).expect("cfg1");
        let cfg2 = ProvisionedConfig::new_dhcp("Boot-2", VALID_PASS, VALID_TOKEN).expect("cfg2");
        let cfg3 =
            ProvisionedConfig::new_dhcp("Stable-Seq3", VALID_PASS, VALID_TOKEN).expect("cfg3");
        let cfg4 =
            ProvisionedConfig::new_dhcp("After-Torn-Write", VALID_PASS, VALID_TOKEN).expect("cfg4");

        assert_eq!(ProvStore::save(&mut flash, &cfg1), Ok(1)); // Sector A
        assert_eq!(ProvStore::save(&mut flash, &cfg2), Ok(2)); // Sector B
        assert_eq!(ProvStore::save(&mut flash, &cfg3), Ok(3)); // Sector A

        // Simulate a torn write: Sector B erased to 0xFF, only first 40 bytes
        // written before power loss.
        flash.sector_b.fill(0xFF);
        let mut partial = [0u8; PROV_RECORD_LEN];
        encode_record(&cfg4, 4, &mut partial);
        flash.sector_b[..40].copy_from_slice(&partial[..40]);

        let recovered = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("sector A recovered");
        assert_eq!(recovered.ssid_str(), "Stable-Seq3");

        let committed_seq = ProvStore::save(&mut flash, &cfg4).expect("save seq 4");
        assert_eq!(committed_seq, 4);
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_B_OFFSET));

        let loaded = ProvStore::load(&mut flash)
            .expect("load succeeds")
            .expect("seq 4 active");
        assert_eq!(loaded.ssid_str(), "After-Torn-Write");
    }

    #[test]
    fn erase_all_clears_both_sectors() {
        let mut flash = MockProvFlash::new();
        let cfg = ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, VALID_TOKEN).expect("cfg");
        assert_eq!(ProvStore::save(&mut flash, &cfg), Ok(1));
        assert_eq!(ProvStore::save(&mut flash, &cfg), Ok(2));

        assert_eq!(ProvStore::erase_all(&mut flash), Ok(()));
        assert!(flash.sector_a.iter().all(|&b| b == 0xFF));
        assert!(flash.sector_b.iter().all(|&b| b == 0xFF));
        assert_eq!(ProvStore::load(&mut flash), Ok(None));
    }

    #[test]
    fn validation_rejects_invalid_ssid_pass_token_and_static_ip() {
        // SSID rejections
        assert_eq!(
            ProvisionedConfig::new_dhcp("", VALID_PASS, VALID_TOKEN),
            Err(ProvStoreError::InvalidSsid)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp(
                "123456789012345678901234567890123",
                VALID_PASS,
                VALID_TOKEN
            ),
            Err(ProvStoreError::InvalidSsid)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp("Bad\nSSID", VALID_PASS, VALID_TOKEN),
            Err(ProvStoreError::InvalidSsid)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp("Bad\x7FSSID", VALID_PASS, VALID_TOKEN),
            Err(ProvStoreError::InvalidSsid)
        );

        // Passphrase rejections
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, "short7!", VALID_TOKEN),
            Err(ProvStoreError::InvalidPassphrase)
        );
        let too_long_pass = "a".repeat(64);
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, &too_long_pass, VALID_TOKEN),
            Err(ProvStoreError::InvalidPassphrase)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, "bad\tpass123", VALID_TOKEN),
            Err(ProvStoreError::InvalidPassphrase)
        );
        if !ALLOW_OPEN_WIFI {
            assert_eq!(
                ProvisionedConfig::new_dhcp(VALID_SSID, "", VALID_TOKEN),
                Err(ProvStoreError::InvalidPassphrase)
            );
        }

        // Token rejections
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, "too-short-15byt"),
            Err(ProvStoreError::InvalidOtaToken)
        );
        let too_long_token = "t".repeat(65);
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, &too_long_token),
            Err(ProvStoreError::InvalidOtaToken)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, "has space in token 1234"),
            Err(ProvStoreError::InvalidOtaToken)
        );
        assert_eq!(
            ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, DEFAULT_OTA_TOKEN),
            Err(ProvStoreError::InvalidOtaToken)
        );

        // Static IP rejections
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(0, 0, 0, 0),
                Ipv4Addr::new(172, 22, 1, 1),
                Ipv4Addr::new(172, 22, 1, 1),
                24,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(224, 0, 0, 10),
                Ipv4Addr::new(224, 0, 0, 1),
                Ipv4Addr::new(1, 1, 1, 1),
                24,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(172, 22, 1, 52),
                Ipv4Addr::new(172, 22, 1, 1),
                Ipv4Addr::new(172, 22, 1, 1),
                7,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(172, 22, 1, 52),
                Ipv4Addr::new(172, 22, 1, 1),
                Ipv4Addr::new(172, 22, 1, 1),
                31,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(172, 22, 1, 0),
                Ipv4Addr::new(172, 22, 1, 1),
                Ipv4Addr::new(172, 22, 1, 1),
                24,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(172, 22, 1, 255),
                Ipv4Addr::new(172, 22, 1, 1),
                Ipv4Addr::new(172, 22, 1, 1),
                24,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
        assert_eq!(
            ProvisionedConfig::new_static(
                VALID_SSID,
                VALID_PASS,
                VALID_TOKEN,
                Ipv4Addr::new(172, 22, 1, 52),
                Ipv4Addr::new(10, 0, 0, 1),
                Ipv4Addr::new(1, 1, 1, 1),
                24,
            ),
            Err(ProvStoreError::InvalidStaticIp)
        );
    }

    #[test]
    fn debug_redacts_secrets() {
        let cfg =
            ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, VALID_TOKEN).expect("valid config");
        let rendered = format!("{cfg:?}");
        assert!(rendered.contains(VALID_SSID));
        assert!(rendered.contains("[REDACTED]"));
        assert!(!rendered.contains(VALID_PASS));
        assert!(!rendered.contains(VALID_TOKEN));
    }

    #[test]
    fn apply_to_updates_klipper_cam_config() {
        static STATIC_PROV: ProvisionedConfig = ProvisionedConfig {
            ssid: copy_bytes_const::<MAX_SSID_LEN>(VALID_SSID.as_bytes()),
            ssid_len: VALID_SSID.len() as u8,
            pass: copy_bytes_const::<MAX_PASS_LEN>(VALID_PASS.as_bytes()),
            pass_len: VALID_PASS.len() as u8,
            token: copy_bytes_const::<MAX_OTA_TOKEN_LEN>(VALID_TOKEN.as_bytes()),
            token_len: VALID_TOKEN.len() as u8,
            ip_mode: IpMode::Static(StaticIpConfig {
                ip: Ipv4Addr::new(192, 168, 10, 44),
                netmask: Ipv4Addr::new(255, 255, 255, 0),
                gateway: Ipv4Addr::new(192, 168, 10, 1),
                dns: Ipv4Addr::new(192, 168, 10, 1),
                prefix_len: 24,
            }),
        };
        static DHCP_PROV: ProvisionedConfig = ProvisionedConfig {
            ssid: copy_bytes_const::<MAX_SSID_LEN>(VALID_SSID.as_bytes()),
            ssid_len: VALID_SSID.len() as u8,
            pass: copy_bytes_const::<MAX_PASS_LEN>(VALID_PASS.as_bytes()),
            pass_len: VALID_PASS.len() as u8,
            token: copy_bytes_const::<MAX_OTA_TOKEN_LEN>(VALID_TOKEN.as_bytes()),
            token_len: VALID_TOKEN.len() as u8,
            ip_mode: IpMode::Dhcp,
        };

        let mut fw_cfg = KlipperCamConfig::from_build_env();
        assert_eq!(STATIC_PROV.apply_to(&mut fw_cfg), Ok(()));
        assert_eq!(fw_cfg.network.wifi_ssid, VALID_SSID);
        assert_eq!(fw_cfg.network.wifi_password, VALID_PASS);
        assert_eq!(fw_cfg.ota_token, VALID_TOKEN);
        assert_eq!(fw_cfg.network.static_ip.ip, Ipv4Addr::new(192, 168, 10, 44));
        assert!(matches!(fw_cfg.network.ip_mode, IpMode::Static(_)));
        assert!(fw_cfg.has_sta_credentials());
        assert!(fw_cfg.has_valid_ota_token());

        let mut dhcp_cfg = KlipperCamConfig::from_build_env();
        assert_eq!(DHCP_PROV.apply_to(&mut dhcp_cfg), Ok(()));
        assert_eq!(dhcp_cfg.network.ip_mode, IpMode::Dhcp);
        assert!(dhcp_cfg.has_sta_credentials());
        assert!(dhcp_cfg.has_valid_ota_token());
    }

    #[test]
    fn readback_mismatch_returns_error() {
        let mut flash = MockProvFlash::new();
        let cfg = ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, VALID_TOKEN).expect("cfg");
        // Succeed on first save to Sector A.
        assert_eq!(ProvStore::save(&mut flash, &cfg), Ok(1));

        // Fail readback on second save to Sector B; Sector A remains intact.
        let cfg2 =
            ProvisionedConfig::new_dhcp("Second-SSID", VALID_PASS, VALID_TOKEN).expect("cfg2");
        let sector_a_before = flash.sector_a;
        flash.corrupt_readback_after_write = true;
        assert_eq!(
            ProvStore::save(&mut flash, &cfg2),
            Err(ProvStoreError::ReadbackMismatch)
        );
        assert_eq!(flash.sector_a, sector_a_before);
        assert_eq!(flash.last_erased, Some(PROV_SECTOR_B_OFFSET));
    }

    #[test]
    fn reserved_nonzero_and_invalid_seq_rejected() {
        let mut flash = MockProvFlash::new();
        let cfg = ProvisionedConfig::new_dhcp(VALID_SSID, VALID_PASS, VALID_TOKEN).expect("cfg");

        // Non-zero reserved byte with valid recomputed CRC32 must be rejected.
        let mut rec = [0u8; PROV_RECORD_LEN];
        encode_record(&cfg, 1, &mut rec);
        rec[OFF_RESERVED] = 1;
        let crc = crc32_ieee(&rec[..OFF_CRC]);
        rec[OFF_CRC..OFF_CRC + 4].copy_from_slice(&crc.to_le_bytes());
        flash.sector_a[..PROV_RECORD_LEN].copy_from_slice(&rec);
        assert_eq!(ProvStore::load(&mut flash), Ok(None));

        // seq == u32::MAX with valid CRC32 must be rejected.
        encode_record(&cfg, u32::MAX, &mut rec);
        flash.sector_a[..PROV_RECORD_LEN].copy_from_slice(&rec);
        assert_eq!(ProvStore::load(&mut flash), Ok(None));

        // seq == 0 with valid CRC32 must be rejected.
        encode_record(&cfg, 0, &mut rec);
        flash.sector_a[..PROV_RECORD_LEN].copy_from_slice(&rec);
        assert_eq!(ProvStore::load(&mut flash), Ok(None));
    }
}
