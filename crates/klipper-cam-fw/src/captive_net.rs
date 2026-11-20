//! Pure-Rust `#![no_std]` captive-portal UDP services for SoftAP Setup Mode.
//!
//! Provides a minimal RFC 2131 / RFC 8910 DHCPv4 server (4-slot lease pool
//! `192.168.4.2..=5`) and a bounded RFC 1035 captive-portal DNS responder that
//! answers `A`/`ANY` queries with `AP_IP` (`192.168.4.1`) and returns clean
//! `NOERROR` (`ANCOUNT = 0`) for `AAAA` and other record types.

#![forbid(unsafe_code)]

/// Default IPv4 address of the SoftAP captive portal (`192.168.4.1`).
pub const AP_IP: [u8; 4] = [192, 168, 4, 1];

/// Subnet mask advertised by the SoftAP DHCPv4 server (`255.255.255.0`).
pub const AP_SUBNET_MASK: [u8; 4] = [255, 255, 255, 0];

/// First host octet in the SoftAP DHCPv4 pool (`192.168.4.2`).
pub const DHCP_POOL_START_HOST: u8 = 2;

/// Fixed number of client lease slots (`192.168.4.2..=5`).
pub const DHCP_POOL_SIZE: usize = 4;

/// Default DHCPv4 lease duration in seconds (`3600`, 1 hour).
pub const DHCP_LEASE_SECS: u32 = 3600;

/// RFC 8910 DHCP Option 114 captive-portal API URI.
pub const CAPTIVE_PORTAL_URI: &str = "http://192.168.4.1/capport";

/// Standard UDP port for DHCPv4 servers (`67`).
pub const DHCP_SERVER_PORT: u16 = 67;

/// Standard UDP port for DHCPv4 clients (`68`).
pub const DHCP_CLIENT_PORT: u16 = 68;

/// Standard UDP port for DNS responders (`53`).
pub const DNS_PORT: u16 = 53;

/// TTL in seconds for synthetic captive-portal DNS `A` answers (`30`).
pub const DNS_TTL_SECS: u32 = 30;

const DHCP_HEADER_LEN: usize = 236;
const DHCP_MIN_PACKET_LEN: usize = 240;
const DHCP_MAGIC_COOKIE: [u8; 4] = [99, 130, 83, 99];

const BOOTREQUEST: u8 = 1;
const BOOTREPLY: u8 = 2;
const HTYPE_ETHERNET: u8 = 1;
const HLEN_ETHERNET: u8 = 6;
const DHCP_FLAG_BROADCAST: [u8; 2] = [0x80, 0x00];

const DHCP_MSG_DISCOVER: u8 = 1;
const DHCP_MSG_OFFER: u8 = 2;
const DHCP_MSG_REQUEST: u8 = 3;
const DHCP_MSG_DECLINE: u8 = 4;
const DHCP_MSG_ACK: u8 = 5;
const DHCP_MSG_NAK: u8 = 6;
const DHCP_MSG_RELEASE: u8 = 7;

const OPT_PAD: u8 = 0;
const OPT_SUBNET_MASK: u8 = 1;
const OPT_ROUTER: u8 = 3;
const OPT_DNS_SERVER: u8 = 6;
const OPT_REQUESTED_IP: u8 = 50;
const OPT_LEASE_TIME: u8 = 51;
const OPT_MSG_TYPE: u8 = 53;
const OPT_SERVER_ID: u8 = 54;
const OPT_CAPTIVE_PORTAL: u8 = 114;
const OPT_END: u8 = 255;

const DNS_HEADER_LEN: usize = 12;
const DNS_MAX_QNAME_LEN: usize = 255;
const DNS_LABEL_PTR_MASK: u8 = 0xC0;
const DNS_FLAG_QR: u16 = 0x8000;
const DNS_FLAG_OPCODE_MASK: u16 = 0x7800;
const DNS_FLAG_RD: u16 = 0x0100;
const DNS_RESP_FLAGS_BASE: u16 = 0x8480; // QR=1, AA=1, RA=1, RCODE=0
const DNS_TYPE_A: u16 = 1;
const DNS_TYPE_ANY: u16 = 255;
const DNS_CLASS_IN: u16 = 1;
const DNS_A_ANSWER_LEN: usize = 16;

/// Single fixed lease slot in [`DhcpServer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LeaseSlot {
    mac: [u8; 6],
    active: bool,
    last_used_seq: u32,
}

impl LeaseSlot {
    const fn empty() -> Self {
        Self {
            mac: [0; 6],
            active: false,
            last_used_seq: 0,
        }
    }
}

/// Parsed subset of incoming DHCPv4 options required for Setup Mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ParsedDhcpOptions {
    msg_type: u8,
    requested_ip: Option<[u8; 4]>,
    server_id: Option<[u8; 4]>,
}

/// Minimal fixed-pool DHCPv4 server for SoftAP Setup Mode.
///
/// Serves up to 4 concurrent clients (`192.168.4.2..=5`) with a 1-hour lease
/// and RFC 8910 Option 114 (`"http://192.168.4.1/capport"`). Evicts the least
/// recently used slot when a 5th distinct MAC arrives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DhcpServer {
    server_ip: [u8; 4],
    lease_secs: u32,
    leases: [LeaseSlot; DHCP_POOL_SIZE],
    seq: u32,
}

impl Default for DhcpServer {
    fn default() -> Self {
        Self::new()
    }
}

impl DhcpServer {
    /// Creates a new SoftAP DHCPv4 server bound to [`AP_IP`] (`192.168.4.1`)
    /// with a 4-slot pool (`192.168.4.2..=5`) and [`DHCP_LEASE_SECS`] leases.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            server_ip: AP_IP,
            lease_secs: DHCP_LEASE_SECS,
            leases: [LeaseSlot::empty(); DHCP_POOL_SIZE],
            seq: 0,
        }
    }

    /// Returns the DHCP server / gateway / DNS IPv4 address.
    #[must_use]
    pub const fn server_ip(&self) -> [u8; 4] {
        self.server_ip
    }

    /// Returns the configured lease duration in seconds.
    #[must_use]
    pub const fn lease_secs(&self) -> u32 {
        self.lease_secs
    }

    /// Returns the number of currently active MAC-to-IP leases.
    #[must_use]
    pub fn active_lease_count(&self) -> usize {
        let mut count = 0;
        for slot in self.leases {
            if slot.active {
                count += 1;
            }
        }
        count
    }

    /// Looks up the active IPv4 address currently assigned to `mac`, if any.
    #[must_use]
    pub fn lease_for_mac(&self, mac: &[u8; 6]) -> Option<[u8; 4]> {
        let mut i = 0;
        while i < DHCP_POOL_SIZE {
            if self.leases[i].active && self.leases[i].mac == *mac {
                return Some(self.ip_for_slot(i));
            }
            i += 1;
        }
        None
    }

    /// Processes a raw UDP payload received on port 67 and writes the DHCPv4
    /// reply payload into `out`, returning `Some(len)` when a reply should be
    /// transmitted to UDP port 68.
    ///
    /// Malformed packets (truncated `< 240 B`, invalid BOOTP opcode/htype/hlen,
    /// wrong magic cookie, truncated TLV options, or unsupported message types)
    /// are ignored and return `None`.
    pub fn handle_packet(&mut self, req: &[u8], out: &mut [u8]) -> Option<usize> {
        if req.len() < DHCP_MIN_PACKET_LEN {
            return None;
        }
        if req[0] != BOOTREQUEST || req[1] != HTYPE_ETHERNET || req[2] != HLEN_ETHERNET {
            return None;
        }
        if req[DHCP_HEADER_LEN..DHCP_MIN_PACKET_LEN] != DHCP_MAGIC_COOKIE {
            return None;
        }

        let opts = parse_dhcp_options(&req[DHCP_MIN_PACKET_LEN..])?;
        let xid = [req[4], req[5], req[6], req[7]];
        let flags = [req[10], req[11]];
        let ciaddr = [req[12], req[13], req[14], req[15]];
        let chaddr = [req[28], req[29], req[30], req[31], req[32], req[33]];

        match opts.msg_type {
            DHCP_MSG_DISCOVER => {
                let offered_ip = self.allocate_for_discover(&chaddr, opts.requested_ip);
                self.write_offer_or_ack(out, DHCP_MSG_OFFER, xid, flags, chaddr, offered_ip)
            }
            DHCP_MSG_REQUEST => {
                if let Some(sid) = opts.server_id {
                    if sid != self.server_ip {
                        return None;
                    }
                }
                let target_ip =
                    opts.requested_ip
                        .or_else(|| if ciaddr != [0; 4] { Some(ciaddr) } else { None });
                let Some(req_ip) = target_ip else {
                    if let Some(existing) = self.lease_for_mac(&chaddr) {
                        return self.write_offer_or_ack(
                            out,
                            DHCP_MSG_ACK,
                            xid,
                            flags,
                            chaddr,
                            existing,
                        );
                    }
                    return self.write_nak(out, xid, chaddr);
                };

                match self.claim_requested_ip(&chaddr, req_ip) {
                    Some(assigned) => {
                        self.write_offer_or_ack(out, DHCP_MSG_ACK, xid, flags, chaddr, assigned)
                    }
                    None => self.write_nak(out, xid, chaddr),
                }
            }
            DHCP_MSG_RELEASE | DHCP_MSG_DECLINE => {
                self.release_mac(&chaddr);
                None
            }
            _ => None,
        }
    }

    #[inline]
    const fn ip_for_slot(&self, slot: usize) -> [u8; 4] {
        [
            self.server_ip[0],
            self.server_ip[1],
            self.server_ip[2],
            DHCP_POOL_START_HOST + (slot as u8),
        ]
    }

    #[inline]
    fn slot_for_ip(&self, ip: [u8; 4]) -> Option<usize> {
        if ip[0] != self.server_ip[0] || ip[1] != self.server_ip[1] || ip[2] != self.server_ip[2] {
            return None;
        }
        let host = ip[3];
        let end_host = DHCP_POOL_START_HOST + (DHCP_POOL_SIZE as u8) - 1;
        if host < DHCP_POOL_START_HOST || host > end_host {
            return None;
        }
        Some(usize::from(host - DHCP_POOL_START_HOST))
    }

    fn next_seq(&mut self) -> u32 {
        self.seq = self.seq.wrapping_add(1);
        self.seq
    }

    fn find_mac_slot(&self, mac: &[u8; 6]) -> Option<usize> {
        let mut i = 0;
        while i < DHCP_POOL_SIZE {
            if self.leases[i].active && self.leases[i].mac == *mac {
                return Some(i);
            }
            i += 1;
        }
        None
    }

    fn allocate_for_discover(&mut self, mac: &[u8; 6], requested_ip: Option<[u8; 4]>) -> [u8; 4] {
        let seq = self.next_seq();
        if let Some(idx) = self.find_mac_slot(mac) {
            self.leases[idx].last_used_seq = seq;
            return self.ip_for_slot(idx);
        }
        if let Some(req_ip) = requested_ip {
            if let Some(idx) = self.slot_for_ip(req_ip) {
                if !self.leases[idx].active {
                    self.leases[idx] = LeaseSlot {
                        mac: *mac,
                        active: true,
                        last_used_seq: seq,
                    };
                    return self.ip_for_slot(idx);
                }
            }
        }
        let mut i = 0;
        while i < DHCP_POOL_SIZE {
            if !self.leases[i].active {
                self.leases[i] = LeaseSlot {
                    mac: *mac,
                    active: true,
                    last_used_seq: seq,
                };
                return self.ip_for_slot(i);
            }
            i += 1;
        }

        let mut oldest_idx = 0;
        let mut oldest_seq = self.leases[0].last_used_seq;
        let mut j = 1;
        while j < DHCP_POOL_SIZE {
            if self.leases[j].last_used_seq < oldest_seq {
                oldest_seq = self.leases[j].last_used_seq;
                oldest_idx = j;
            }
            j += 1;
        }
        self.leases[oldest_idx] = LeaseSlot {
            mac: *mac,
            active: true,
            last_used_seq: seq,
        };
        self.ip_for_slot(oldest_idx)
    }

    fn claim_requested_ip(&mut self, mac: &[u8; 6], req_ip: [u8; 4]) -> Option<[u8; 4]> {
        let idx = self.slot_for_ip(req_ip)?;
        if self.leases[idx].active && self.leases[idx].mac != *mac {
            return None;
        }
        let seq = self.next_seq();
        self.release_mac(mac);
        self.leases[idx] = LeaseSlot {
            mac: *mac,
            active: true,
            last_used_seq: seq,
        };
        Some(self.ip_for_slot(idx))
    }

    fn release_mac(&mut self, mac: &[u8; 6]) {
        for slot in &mut self.leases {
            if slot.active && slot.mac == *mac {
                *slot = LeaseSlot::empty();
            }
        }
    }

    fn write_base_header(
        &self,
        out: &mut [u8],
        xid: [u8; 4],
        flags: [u8; 2],
        yiaddr: [u8; 4],
        chaddr: [u8; 6],
    ) -> Option<usize> {
        if out.len() < DHCP_MIN_PACKET_LEN {
            return None;
        }
        out[..DHCP_MIN_PACKET_LEN].fill(0);
        out[0] = BOOTREPLY;
        out[1] = HTYPE_ETHERNET;
        out[2] = HLEN_ETHERNET;
        out[4..8].copy_from_slice(&xid);
        out[10..12].copy_from_slice(&flags);
        out[16..20].copy_from_slice(&yiaddr);
        out[20..24].copy_from_slice(&self.server_ip);
        out[28..34].copy_from_slice(&chaddr);
        out[DHCP_HEADER_LEN..DHCP_MIN_PACKET_LEN].copy_from_slice(&DHCP_MAGIC_COOKIE);
        Some(DHCP_MIN_PACKET_LEN)
    }

    fn write_offer_or_ack(
        &self,
        out: &mut [u8],
        msg_type: u8,
        xid: [u8; 4],
        flags: [u8; 2],
        chaddr: [u8; 6],
        yiaddr: [u8; 4],
    ) -> Option<usize> {
        let uri = CAPTIVE_PORTAL_URI.as_bytes();
        // Options: 53(3) + 54(6) + 51(6) + 1(6) + 3(6) + 6(6) + 114(2+uri) + 255(1)
        let needed = DHCP_MIN_PACKET_LEN + 3 + 6 + 6 + 6 + 6 + 6 + 2 + uri.len() + 1;
        if out.len() < needed {
            return None;
        }

        let mut pos = self.write_base_header(out, xid, flags, yiaddr, chaddr)?;
        pos = append_opt(out, pos, OPT_MSG_TYPE, &[msg_type])?;
        pos = append_opt(out, pos, OPT_SERVER_ID, &self.server_ip)?;
        pos = append_opt(out, pos, OPT_LEASE_TIME, &self.lease_secs.to_be_bytes())?;
        pos = append_opt(out, pos, OPT_SUBNET_MASK, &AP_SUBNET_MASK)?;
        pos = append_opt(out, pos, OPT_ROUTER, &self.server_ip)?;
        pos = append_opt(out, pos, OPT_DNS_SERVER, &self.server_ip)?;
        pos = append_opt(out, pos, OPT_CAPTIVE_PORTAL, uri)?;
        out[pos] = OPT_END;
        Some(pos + 1)
    }

    fn write_nak(&self, out: &mut [u8], xid: [u8; 4], chaddr: [u8; 6]) -> Option<usize> {
        // Options: 53(3) + 54(6) + 255(1)
        let needed = DHCP_MIN_PACKET_LEN + 3 + 6 + 1;
        if out.len() < needed {
            return None;
        }
        let mut pos = self.write_base_header(out, xid, DHCP_FLAG_BROADCAST, [0; 4], chaddr)?;
        pos = append_opt(out, pos, OPT_MSG_TYPE, &[DHCP_MSG_NAK])?;
        pos = append_opt(out, pos, OPT_SERVER_ID, &self.server_ip)?;
        out[pos] = OPT_END;
        Some(pos + 1)
    }
}

fn append_opt(out: &mut [u8], pos: usize, code: u8, val: &[u8]) -> Option<usize> {
    let val_len = u8::try_from(val.len()).ok()?;
    let end = pos.checked_add(2)?.checked_add(val.len())?;
    if end > out.len() {
        return None;
    }
    out[pos] = code;
    out[pos + 1] = val_len;
    out[pos + 2..end].copy_from_slice(val);
    Some(end)
}

fn parse_dhcp_options(opts: &[u8]) -> Option<ParsedDhcpOptions> {
    let mut pos = 0usize;
    let mut msg_type: Option<u8> = None;
    let mut requested_ip: Option<[u8; 4]> = None;
    let mut server_id: Option<[u8; 4]> = None;

    while pos < opts.len() {
        let code = opts[pos];
        if code == OPT_PAD {
            pos += 1;
            continue;
        }
        if code == OPT_END {
            break;
        }
        if pos + 1 >= opts.len() {
            return None;
        }
        let len = usize::from(opts[pos + 1]);
        let val_start = pos + 2;
        let val_end = val_start.checked_add(len)?;
        if val_end > opts.len() {
            return None;
        }
        let val = &opts[val_start..val_end];
        match code {
            OPT_MSG_TYPE => {
                if len != 1 {
                    return None;
                }
                msg_type = Some(val[0]);
            }
            OPT_REQUESTED_IP if len == 4 => {
                requested_ip = Some([val[0], val[1], val[2], val[3]]);
            }
            OPT_SERVER_ID if len == 4 => {
                server_id = Some([val[0], val[1], val[2], val[3]]);
            }
            _ => {}
        }
        pos = val_end;
    }

    let msg_type = msg_type?;
    Some(ParsedDhcpOptions {
        msg_type,
        requested_ip,
        server_id,
    })
}

/// Handles a single UDP DNS query for the SoftAP captive portal.
///
/// Validates the 12-byte DNS header (`QR == 0`, standard opcode `0`,
/// `QDCOUNT >= 1`) and parses the first question's QNAME with strict bounds
/// (`<= 255` wire bytes, rejecting compression pointers or reserved label flags
/// in queries so pointer loops like `0xC0 0x0C` can never loop or panic).
///
/// - For `QTYPE == A (1)` or `ANY (255)` with `QCLASS == IN (1)`: echoes the
///   question and writes a 16-byte `A` answer pointing to `ap_ip` with TTL
///   [`DNS_TTL_SECS`] (`30s`) and flags `AA = 1, RA = 1`.
/// - For `QTYPE == AAAA (28)` or any other type/class: returns a clean
///   `NOERROR` response with `ANCOUNT = 0` so OS captive-portal probes never
///   stall waiting for IPv6 timeouts.
#[must_use]
pub fn handle_captive_dns_query(req: &[u8], ap_ip: [u8; 4], out: &mut [u8]) -> Option<usize> {
    if req.len() < DNS_HEADER_LEN {
        return None;
    }

    let flags = u16::from_be_bytes([req[2], req[3]]);
    if (flags & DNS_FLAG_QR) != 0 || (flags & DNS_FLAG_OPCODE_MASK) != 0 {
        return None;
    }
    let qdcount = u16::from_be_bytes([req[4], req[5]]);
    if qdcount == 0 {
        return None;
    }

    let qname_end = parse_uncompressed_qname(req, DNS_HEADER_LEN)?;
    let question_end = qname_end.checked_add(4)?;
    if question_end > req.len() {
        return None;
    }

    let qtype = u16::from_be_bytes([req[qname_end], req[qname_end + 1]]);
    let qclass = u16::from_be_bytes([req[qname_end + 2], req[qname_end + 3]]);
    let question = &req[DNS_HEADER_LEN..question_end];

    let send_a_record = qclass == DNS_CLASS_IN && (qtype == DNS_TYPE_A || qtype == DNS_TYPE_ANY);
    let total_len = DNS_HEADER_LEN
        .checked_add(question.len())?
        .checked_add(if send_a_record { DNS_A_ANSWER_LEN } else { 0 })?;
    if out.len() < total_len {
        return None;
    }

    let resp_flags = DNS_RESP_FLAGS_BASE | (flags & DNS_FLAG_RD);
    let ancount: u16 = if send_a_record { 1 } else { 0 };

    out[0] = req[0];
    out[1] = req[1];
    out[2..4].copy_from_slice(&resp_flags.to_be_bytes());
    out[4..6].copy_from_slice(&1u16.to_be_bytes());
    out[6..8].copy_from_slice(&ancount.to_be_bytes());
    out[8..10].copy_from_slice(&0u16.to_be_bytes());
    out[10..12].copy_from_slice(&0u16.to_be_bytes());

    let q_out_end = DNS_HEADER_LEN + question.len();
    out[DNS_HEADER_LEN..q_out_end].copy_from_slice(question);

    if send_a_record {
        // Pointer to QNAME at offset 12 (0xC00C), TYPE A, CLASS IN, TTL 30s,
        // RDLENGTH 4, RDATA ap_ip.
        out[q_out_end] = 0xC0;
        out[q_out_end + 1] = DNS_HEADER_LEN as u8;
        out[q_out_end + 2..q_out_end + 4].copy_from_slice(&DNS_TYPE_A.to_be_bytes());
        out[q_out_end + 4..q_out_end + 6].copy_from_slice(&DNS_CLASS_IN.to_be_bytes());
        out[q_out_end + 6..q_out_end + 10].copy_from_slice(&DNS_TTL_SECS.to_be_bytes());
        out[q_out_end + 10..q_out_end + 12].copy_from_slice(&4u16.to_be_bytes());
        out[q_out_end + 12..q_out_end + 16].copy_from_slice(&ap_ip);
    }

    Some(total_len)
}

/// Validates and advances past an uncompressed RFC 1035 QNAME starting at
/// `start`, rejecting any compression pointer (`0xC0`), reserved label bits
/// (`0x40`/`0x80`), truncated label, or total wire length `> 255` bytes.
fn parse_uncompressed_qname(pkt: &[u8], start: usize) -> Option<usize> {
    let mut pos = start;
    loop {
        if pos >= pkt.len() {
            return None;
        }
        let len_byte = pkt[pos];
        if (len_byte & DNS_LABEL_PTR_MASK) != 0 {
            return None;
        }
        if len_byte == 0 {
            let end = pos.checked_add(1)?;
            if end - start > DNS_MAX_QNAME_LEN {
                return None;
            }
            return Some(end);
        }
        let label_len = usize::from(len_byte);
        let next = pos.checked_add(1)?.checked_add(label_len)?;
        if next > pkt.len() {
            return None;
        }
        // Leave room for the terminating NUL root label within 255 wire bytes.
        if next - start >= DNS_MAX_QNAME_LEN {
            return None;
        }
        pos = next;
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;

    fn build_dhcp_request_pkt(
        msg_type: u8,
        xid: [u8; 4],
        mac: [u8; 6],
        ciaddr: [u8; 4],
        requested_ip: Option<[u8; 4]>,
        server_id: Option<[u8; 4]>,
    ) -> Vec<u8> {
        let mut pkt = vec![0u8; DHCP_MIN_PACKET_LEN];
        pkt[0] = BOOTREQUEST;
        pkt[1] = HTYPE_ETHERNET;
        pkt[2] = HLEN_ETHERNET;
        pkt[4..8].copy_from_slice(&xid);
        pkt[10..12].copy_from_slice(&DHCP_FLAG_BROADCAST);
        pkt[12..16].copy_from_slice(&ciaddr);
        pkt[28..34].copy_from_slice(&mac);
        pkt[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE);

        pkt.extend_from_slice(&[OPT_MSG_TYPE, 1, msg_type]);
        if let Some(ip) = requested_ip {
            pkt.extend_from_slice(&[OPT_REQUESTED_IP, 4]);
            pkt.extend_from_slice(&ip);
        }
        if let Some(sid) = server_id {
            pkt.extend_from_slice(&[OPT_SERVER_ID, 4]);
            pkt.extend_from_slice(&sid);
        }
        pkt.push(OPT_END);
        pkt
    }

    fn find_dhcp_option(pkt: &[u8], target: u8) -> Option<&[u8]> {
        let opts = pkt.get(DHCP_MIN_PACKET_LEN..)?;
        let mut pos = 0;
        while pos < opts.len() {
            let code = opts[pos];
            if code == OPT_PAD {
                pos += 1;
                continue;
            }
            if code == OPT_END {
                break;
            }
            let len = usize::from(*opts.get(pos + 1)?);
            let start = pos + 2;
            let end = start + len;
            let val = opts.get(start..end)?;
            if code == target {
                return Some(val);
            }
            pos = end;
        }
        None
    }

    fn build_dns_query(id: u16, labels: &[&[u8]], qtype: u16, qclass: u16) -> Vec<u8> {
        let mut pkt = Vec::new();
        pkt.extend_from_slice(&id.to_be_bytes());
        pkt.extend_from_slice(&0x0100u16.to_be_bytes()); // RD=1, QR=0
        pkt.extend_from_slice(&1u16.to_be_bytes()); // QDCOUNT=1
        pkt.extend_from_slice(&0u16.to_be_bytes());
        pkt.extend_from_slice(&0u16.to_be_bytes());
        pkt.extend_from_slice(&0u16.to_be_bytes());
        for label in labels {
            pkt.push(label.len() as u8);
            pkt.extend_from_slice(label);
        }
        pkt.push(0);
        pkt.extend_from_slice(&qtype.to_be_bytes());
        pkt.extend_from_slice(&qclass.to_be_bytes());
        pkt
    }

    #[test]
    fn dhcp_discover_offer_and_request_ack_emit_required_options() {
        let mut srv = DhcpServer::new();
        let mac = [0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33];
        let xid = [0x12, 0x34, 0x56, 0x78];
        let mut out = [0u8; 576];

        let disc = build_dhcp_request_pkt(DHCP_MSG_DISCOVER, xid, mac, [0; 4], None, None);
        let offer_len = srv
            .handle_packet(&disc, &mut out)
            .expect("DHCPDISCOVER must yield DHCPOFFER");
        let offer = &out[..offer_len];

        assert_eq!(offer[0], BOOTREPLY, "reply opcode must be BOOTREPLY");
        assert_eq!(&offer[4..8], &xid, "transaction ID must be echoed");
        assert_eq!(&offer[16..20], &[192, 168, 4, 2], "first offered IP");
        assert_eq!(&offer[20..24], &AP_IP, "siaddr must be AP_IP");
        assert_eq!(&offer[28..34], &mac, "chaddr must be echoed");
        assert_eq!(
            find_dhcp_option(offer, OPT_MSG_TYPE),
            Some(&[DHCP_MSG_OFFER][..])
        );
        assert_eq!(find_dhcp_option(offer, OPT_SERVER_ID), Some(&AP_IP[..]));
        assert_eq!(
            find_dhcp_option(offer, OPT_LEASE_TIME),
            Some(&DHCP_LEASE_SECS.to_be_bytes()[..])
        );
        assert_eq!(
            find_dhcp_option(offer, OPT_SUBNET_MASK),
            Some(&AP_SUBNET_MASK[..])
        );
        assert_eq!(find_dhcp_option(offer, OPT_ROUTER), Some(&AP_IP[..]));
        assert_eq!(find_dhcp_option(offer, OPT_DNS_SERVER), Some(&AP_IP[..]));
        assert_eq!(
            CAPTIVE_PORTAL_URI, "http://192.168.4.1/capport",
            "RFC 8910 Option 114 URI must point to /capport"
        );
        assert_eq!(
            find_dhcp_option(offer, OPT_CAPTIVE_PORTAL),
            Some(b"http://192.168.4.1/capport".as_slice())
        );

        let req = build_dhcp_request_pkt(
            DHCP_MSG_REQUEST,
            xid,
            mac,
            [0; 4],
            Some([192, 168, 4, 2]),
            Some(AP_IP),
        );
        let ack_len = srv
            .handle_packet(&req, &mut out)
            .expect("DHCPREQUEST for offered pool IP must yield DHCPACK");
        let ack = &out[..ack_len];
        assert_eq!(
            find_dhcp_option(ack, OPT_MSG_TYPE),
            Some(&[DHCP_MSG_ACK][..])
        );
        assert_eq!(&ack[16..20], &[192, 168, 4, 2]);
        assert_eq!(
            find_dhcp_option(ack, OPT_CAPTIVE_PORTAL),
            Some(b"http://192.168.4.1/capport".as_slice())
        );
        assert_eq!(srv.lease_for_mac(&mac), Some([192, 168, 4, 2]));
        assert_eq!(srv.active_lease_count(), 1);
    }

    #[test]
    fn dhcp_request_out_of_pool_returns_nak() {
        let mut srv = DhcpServer::new();
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x09];
        let xid = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut out = [0u8; 576];

        let req = build_dhcp_request_pkt(
            DHCP_MSG_REQUEST,
            xid,
            mac,
            [0; 4],
            Some([172, 22, 1, 52]),
            None,
        );
        let nak_len = srv
            .handle_packet(&req, &mut out)
            .expect("out-of-pool DHCPREQUEST must yield DHCPNAK");
        let nak = &out[..nak_len];
        assert_eq!(
            find_dhcp_option(nak, OPT_MSG_TYPE),
            Some(&[DHCP_MSG_NAK][..])
        );
        assert_eq!(find_dhcp_option(nak, OPT_SERVER_ID), Some(&AP_IP[..]));
        assert_eq!(&nak[16..20], &[0, 0, 0, 0], "NAK yiaddr must be zero");
        assert_eq!(&nak[10..12], &DHCP_FLAG_BROADCAST, "NAK must set broadcast");
        assert_eq!(srv.active_lease_count(), 0);
    }

    #[test]
    fn dhcp_four_client_pool_reuse_eviction_and_release_decline() {
        let mut srv = DhcpServer::new();
        let mut out = [0u8; 576];

        for host in 2u8..=5u8 {
            let mac = [0x02, 0x11, 0x22, 0x33, 0x44, host];
            let disc =
                build_dhcp_request_pkt(DHCP_MSG_DISCOVER, [0, 0, 0, host], mac, [0; 4], None, None);
            let len = srv
                .handle_packet(&disc, &mut out)
                .expect("client within pool must receive offer");
            assert_eq!(&out[16..20], &[192, 168, 4, host]);
            assert!(len > DHCP_MIN_PACKET_LEN);
        }
        assert_eq!(srv.active_lease_count(), 4);

        // Existing client 1 re-discovers and reuses its original IP (192.168.4.2),
        // making client 2 (192.168.4.3) the LRU slot.
        let mac1 = [0x02, 0x11, 0x22, 0x33, 0x44, 2];
        let disc1 =
            build_dhcp_request_pkt(DHCP_MSG_DISCOVER, [1, 2, 3, 4], mac1, [0; 4], None, None);
        let _ = srv
            .handle_packet(&disc1, &mut out)
            .expect("existing client must reuse its lease");
        assert_eq!(&out[16..20], &[192, 168, 4, 2]);

        // 5th distinct client evicts LRU slot (192.168.4.3).
        let mac5 = [0x02, 0x11, 0x22, 0x33, 0x44, 99];
        let disc5 =
            build_dhcp_request_pkt(DHCP_MSG_DISCOVER, [5, 6, 7, 8], mac5, [0; 4], None, None);
        let _ = srv
            .handle_packet(&disc5, &mut out)
            .expect("5th client must evict LRU slot");
        assert_eq!(&out[16..20], &[192, 168, 4, 3]);
        assert_eq!(srv.lease_for_mac(&mac5), Some([192, 168, 4, 3]));

        // Release frees mac5's slot with no reply packet.
        let rel = build_dhcp_request_pkt(
            DHCP_MSG_RELEASE,
            [9, 9, 9, 9],
            mac5,
            [192, 168, 4, 3],
            None,
            None,
        );
        assert!(srv.handle_packet(&rel, &mut out).is_none());
        assert_eq!(srv.lease_for_mac(&mac5), None);
        assert_eq!(srv.active_lease_count(), 3);

        // Decline frees mac1's slot with no reply packet.
        let dec = build_dhcp_request_pkt(
            DHCP_MSG_DECLINE,
            [8, 8, 8, 8],
            mac1,
            [0; 4],
            Some([192, 168, 4, 2]),
            None,
        );
        assert!(srv.handle_packet(&dec, &mut out).is_none());
        assert_eq!(srv.lease_for_mac(&mac1), None);
        assert_eq!(srv.active_lease_count(), 2);
    }

    #[test]
    fn dhcp_rejects_truncated_bad_cookie_and_malformed_tlv() {
        let mut srv = DhcpServer::new();
        let mut out = [0u8; 576];
        let mac = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE];

        // Review Focus #5: truncated < 240 B packet.
        let short_pkt = [0u8; 239];
        assert!(srv.handle_packet(&short_pkt, &mut out).is_none());

        // Review Focus #5: wrong magic cookie.
        let mut bad_cookie =
            build_dhcp_request_pkt(DHCP_MSG_DISCOVER, [1, 1, 1, 1], mac, [0; 4], None, None);
        bad_cookie[236] ^= 0xFF;
        assert!(srv.handle_packet(&bad_cookie, &mut out).is_none());

        // Truncated option TLV length past end of packet.
        let mut bad_tlv = vec![0u8; DHCP_MIN_PACKET_LEN];
        bad_tlv[0] = BOOTREQUEST;
        bad_tlv[1] = HTYPE_ETHERNET;
        bad_tlv[2] = HLEN_ETHERNET;
        bad_tlv[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE);
        bad_tlv.extend_from_slice(&[
            OPT_MSG_TYPE,
            1,
            DHCP_MSG_DISCOVER,
            OPT_REQUESTED_IP,
            4,
            1,
            2,
        ]);
        assert!(srv.handle_packet(&bad_tlv, &mut out).is_none());

        // Short output buffer never panics.
        let valid =
            build_dhcp_request_pkt(DHCP_MSG_DISCOVER, [2, 2, 2, 2], mac, [0; 4], None, None);
        let mut tiny_out = [0u8; 64];
        assert!(srv.handle_packet(&valid, &mut tiny_out).is_none());
    }

    #[test]
    fn dns_a_and_any_queries_return_ap_ip_with_ttl_30_aa_ra() {
        let mut out = [0u8; 512];
        for qtype in [DNS_TYPE_A, DNS_TYPE_ANY] {
            let query = build_dns_query(
                0x4242,
                &[b"connectivitycheck", b"gstatic", b"com"],
                qtype,
                DNS_CLASS_IN,
            );
            let len = handle_captive_dns_query(&query, AP_IP, &mut out)
                .expect("A/ANY IN query must return synthetic answer");
            let resp = &out[..len];

            assert_eq!(&resp[0..2], &0x4242u16.to_be_bytes());
            let resp_flags = u16::from_be_bytes([resp[2], resp[3]]);
            assert_ne!(resp_flags & 0x8000, 0, "QR must be 1");
            assert_ne!(resp_flags & 0x0400, 0, "AA must be 1");
            assert_ne!(resp_flags & 0x0080, 0, "RA must be 1");
            assert_eq!(resp_flags & 0x000F, 0, "RCODE must be NOERROR");
            assert_eq!(u16::from_be_bytes([resp[4], resp[5]]), 1, "QDCOUNT=1");
            assert_eq!(u16::from_be_bytes([resp[6], resp[7]]), 1, "ANCOUNT=1");

            let and = &resp[query.len()..];
            assert_eq!(and.len(), DNS_A_ANSWER_LEN);
            assert_eq!(&and[0..2], &[0xC0, 0x0C], "compressed pointer to QNAME");
            assert_eq!(&and[2..4], &DNS_TYPE_A.to_be_bytes());
            assert_eq!(&and[4..6], &DNS_CLASS_IN.to_be_bytes());
            assert_eq!(&and[6..10], &DNS_TTL_SECS.to_be_bytes());
            assert_eq!(&and[10..12], &4u16.to_be_bytes());
            assert_eq!(&and[12..16], &AP_IP);
        }
    }

    #[test]
    fn dns_aaaa_returns_noerror_with_zero_answers() {
        let mut out = [0u8; 512];
        let query = build_dns_query(0x1337, &[b"captive", b"apple", b"com"], 28, DNS_CLASS_IN);
        let len = handle_captive_dns_query(&query, AP_IP, &mut out)
            .expect("AAAA query must return clean NOERROR empty answer");
        let resp = &out[..len];

        assert_eq!(len, query.len(), "no answer RR appended for AAAA");
        let resp_flags = u16::from_be_bytes([resp[2], resp[3]]);
        assert_eq!(resp_flags & 0x000F, 0, "RCODE must be NOERROR");
        assert_eq!(u16::from_be_bytes([resp[6], resp[7]]), 0, "ANCOUNT=0");
    }

    #[test]
    fn dns_rejects_compression_pointer_loop_oversized_qname_and_truncated() {
        let mut out = [0u8; 512];

        // Review Focus #5: DNS compression-pointer self-loop 0xC0 0x0C in query.
        let mut loop_pkt = Vec::new();
        loop_pkt.extend_from_slice(&0xABCDu16.to_be_bytes());
        loop_pkt.extend_from_slice(&0x0100u16.to_be_bytes());
        loop_pkt.extend_from_slice(&1u16.to_be_bytes());
        loop_pkt.extend_from_slice(&[0u8; 6]);
        loop_pkt.extend_from_slice(&[0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01]);
        assert!(handle_captive_dns_query(&loop_pkt, AP_IP, &mut out).is_none());

        // Review Focus #5: oversized QNAME (> 255 wire bytes: 5 labels * 64 = 320 B).
        let big_label = [b'a'; 63];
        let oversized = build_dns_query(
            0x2222,
            &[&big_label, &big_label, &big_label, &big_label, &big_label],
            DNS_TYPE_A,
            DNS_CLASS_IN,
        );
        assert!(handle_captive_dns_query(&oversized, AP_IP, &mut out).is_none());

        // Boundary check: exact 255-byte QNAME (63+1)*3 + (61+1) + 1 = 255 B succeeds.
        let tail_label = [b'b'; 61];
        let exact_max = build_dns_query(
            0x3333,
            &[&big_label, &big_label, &big_label, &tail_label],
            DNS_TYPE_A,
            DNS_CLASS_IN,
        );
        assert!(handle_captive_dns_query(&exact_max, AP_IP, &mut out).is_some());

        // Truncated header (< 12 B) and response packet (QR=1).
        assert!(handle_captive_dns_query(&[0u8; 11], AP_IP, &mut out).is_none());
        let mut resp_as_req = build_dns_query(0x4444, &[b"a"], DNS_TYPE_A, DNS_CLASS_IN);
        resp_as_req[2] |= 0x80;
        assert!(handle_captive_dns_query(&resp_as_req, AP_IP, &mut out).is_none());

        // Short output buffer never panics.
        let valid = build_dns_query(0x5555, &[b"example", b"com"], DNS_TYPE_A, DNS_CLASS_IN);
        let mut tiny_out = [0u8; 16];
        assert!(handle_captive_dns_query(&valid, AP_IP, &mut tiny_out).is_none());
    }
}
