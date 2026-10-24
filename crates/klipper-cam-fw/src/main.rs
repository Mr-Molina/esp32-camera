//! Entrypoint for `klipper-cam-fw`.
//!
//! When compiled for host (`x86_64` / `aarch64`), runs a self-verification check of the
//! partition table, OTA CRC32 state machine, and dual-port HTTP/MJPEG handlers.
//! When compiled for Xtensa bare-metal (`target_arch = "xtensa"`), initializes the ESP32
//! at `240 MHz`, drives `20 MHz` `XCLK` on `GPIO 21`, probes `I2C0` (`GPIO 26` SDA / `GPIO 27` SCL)
//! for a live `OV2640` sensor, and executes the complete 7-crate hardware-in-the-loop (HIL)
//! verification suite over UART0 (`115200` baud).

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]
#![cfg_attr(not(target_arch = "xtensa"), forbid(unsafe_code))]
#![cfg_attr(
    target_arch = "xtensa",
    allow(
        unsafe_op_in_unsafe_fn,
        static_mut_refs,
        clippy::undocumented_unsafe_blocks,
        clippy::unwrap_used
    )
)]

#[cfg(target_arch = "xtensa")]
extern crate alloc;

#[cfg(target_arch = "xtensa")]
use esp_backtrace as _;

#[cfg(target_arch = "xtensa")]
esp_bootloader_esp_idf::esp_app_desc!();

use esp32_cam_ll::{build_circular_ring, verify_circular_ring, Lldesc, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
use klipper_cam_fw::{
    calc_dma_sizes, decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, parse_jpeg_dimensions,
    verify_jpeg_eoi, verify_jpeg_soi, verify_partition_table_integrity, BoardPins, CameraDriver,
    CameraSensor, DmaElement, Esp32I2sCamera, EspImageHeader, FirmwareRuntime, FramePool,
    FrameSize, GrabMode, HttpResponseView, Ipv4Addr, KlipperCamConfig, MockSccbBus, OtaSlot,
    Ov2640, PingPongFrameBuffer, PixelFormat, ServerPort, SliceWriter, WroverKitPins, XclkConfig,
    XclkController, DEFAULT_JPEG_QUALITY, DEFAULT_OTA_TOKEN, DEFAULT_XCLK_FREQ_HZ,
    MJPEG_CONTENT_TYPE,
};

#[inline(never)]
fn hil_check_pins_and_xclk() -> bool {
    if WroverKitPins.validate_pins().is_err() {
        return false;
    }
    if WroverKitPins::XCLK != 21
        || WroverKitPins::SIOD != 26
        || WroverKitPins::SIOC != 27
        || WroverKitPins::VSYNC != 25
        || WroverKitPins::HREF != 23
        || WroverKitPins::PCLK != 22
    {
        return false;
    }

    let mut xclk = XclkController::new();
    let cfg = XclkConfig {
        pin_xclk: WroverKitPins::XCLK,
        ledc_timer: 0,
        ledc_channel: 0,
        xclk_freq_hz: DEFAULT_XCLK_FREQ_HZ,
    };
    if xclk.enable_out_clock(&cfg).is_err()
        || !xclk.is_enabled()
        || xclk.freq_hz() != DEFAULT_XCLK_FREQ_HZ
    {
        return false;
    }
    xclk.disable_out_clock();
    !xclk.is_enabled()
}

#[inline(never)]
fn hil_check_sccb_and_ov2640_hd() -> bool {
    let mut sccb_bus: MockSccbBus<512> = MockSccbBus::new();
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_PID, 0x26);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_VER, 0x42);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDH, 0x7F);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDL, 0xA2);

    let sensor_id = match Ov2640::detect(&mut sccb_bus, Ov2640::SLV_ADDR) {
        Ok(id) => id,
        Err(_) => return false,
    };
    let mut ov2640 = Ov2640::from_id(Ov2640::SLV_ADDR, sensor_id);

    if ov2640.reset(&mut sccb_bus).is_err() {
        return false;
    }
    if ov2640.set_framesize(&mut sccb_bus, FrameSize::Hd).is_err() {
        return false;
    }
    if ov2640
        .set_pixformat(&mut sccb_bus, PixelFormat::Jpeg)
        .is_err()
    {
        return false;
    }
    if ov2640
        .set_quality(&mut sccb_bus, DEFAULT_JPEG_QUALITY as i32)
        .is_err()
    {
        return false;
    }

    let hsize8 = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::HSIZE8);
    let vsize8 = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::VSIZE8);
    let yoffl = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::YOFFL);

    hsize8 == 0xC8 && vsize8 == 0x96 && yoffl == 150
}

#[inline(never)]
fn hil_check_ll_dma_and_i2s0() -> bool {
    let mut i2s_cam = Esp32I2sCamera::new();
    let ll_pins = WroverKitPins::PINS.to_ll_pin_config(true);
    if i2s_cam.set_pin(&ll_pins).is_err() || i2s_cam.gpio_bindings().len() != 12 {
        return false;
    }

    let mode = match i2s_cam.set_sample_mode(PixelFormat::Jpeg, DEFAULT_XCLK_FREQ_HZ, 0x26) {
        Ok(m) => m,
        Err(_) => return false,
    };

    let dma_sizing = match calc_dma_sizes(
        true,
        mode.sampling_mode,
        FrameSize::Hd.resolution().width,
        FrameSize::Hd.resolution().height,
        mode.in_bytes_per_pixel,
        DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
    ) {
        Ok(cfg) => cfg,
        Err(_) => return false,
    };
    if dma_sizing.dma_node_cnt == 0 || dma_sizing.dma_buffer_size == 0 {
        return false;
    }

    let mut nodes = [Lldesc::empty(); 4];
    let dma_buf = [0u8; 512];
    if build_circular_ring(&mut nodes, &dma_buf, 128).is_err() {
        return false;
    }
    if verify_circular_ring(&nodes, &dma_buf, 128).is_err() {
        return false;
    }

    i2s_cam.vsync_intr_enable(true);
    if !i2s_cam.is_vsync_intr_enabled() {
        return false;
    }
    if !i2s_cam.start(&dma_sizing, nodes[0].link_addr_20bit()) || !i2s_cam.is_running() {
        return false;
    }
    if !i2s_cam.stop() || i2s_cam.is_running() {
        return false;
    }
    i2s_cam.deinit().is_ok()
}

#[inline(never)]
fn hil_check_jpeg_pool_and_http_ota() -> bool {
    if !verify_partition_table_integrity() {
        return false;
    }

    let width = 16u16;
    let height = 8u16;
    let mut rgb888 = [0u8; 16 * 8 * 3];
    let mut i = 0;
    while i < 16 * 8 {
        rgb888[i * 3] = 40;
        rgb888[i * 3 + 1] = 120;
        rgb888[i * 3 + 2] = 200;
        i += 1;
    }
    let mut jpeg_encoded = [0u8; 640];
    let jpeg_len = match encode_rgb888_to_jpeg(&rgb888, &mut jpeg_encoded, width, height, 85) {
        Ok(len) => len,
        Err(_) => return false,
    };
    if verify_jpeg_soi(&jpeg_encoded[..jpeg_len]) != Some(0) {
        return false;
    }
    if verify_jpeg_eoi(&jpeg_encoded[..jpeg_len], false) != Some(jpeg_len - 2) {
        return false;
    }
    if parse_jpeg_dimensions(&jpeg_encoded[..jpeg_len]) != Some((width, height)) {
        return false;
    }

    let mut decoded_rgb = [0u8; 16 * 8 * 3];
    if decode_jpeg_to_rgb888(&jpeg_encoded[..jpeg_len], &mut decoded_rgb).is_err() {
        return false;
    }

    let el = DmaElement::new(jpeg_encoded[0], 0);
    if el.sample1() != jpeg_encoded[0] {
        return false;
    }

    // Verify RAII FramePool<2, 640> lease and drop return
    let mut pool: FramePool<2, 640> = FramePool::new(GrabMode::Latest);
    if pool
        .push_frame(
            &jpeg_encoded[..jpeg_len],
            width,
            height,
            PixelFormat::Jpeg,
            1000,
        )
        .is_err()
    {
        return false;
    }
    {
        let leased = match pool.take() {
            Ok(g) => g,
            Err(_) => return false,
        };
        if leased.as_slice() != &jpeg_encoded[..jpeg_len] || leased.width() != width {
            return false;
        }
    }
    if pool.free_count() != 2 {
        return false;
    }

    let mut pp_a = [0u8; 640];
    let mut pp_b = [0u8; 640];
    let mut pp = match PingPongFrameBuffer::new(&mut pp_a, &mut pp_b, &jpeg_encoded[..jpeg_len]) {
        Ok(p) => p,
        Err(_) => return false,
    };
    if pp.write_and_commit_inactive(&jpeg_encoded[..jpeg_len]).is_err()
        || pp.frame_seq() != 2
        || pp.active_slot_index() != 1
        || pp.active_frame() != &jpeg_encoded[..jpeg_len]
    {
        return false;
    }

    let fw_cam = CameraDriver::new(&jpeg_encoded[..jpeg_len], FrameSize::Hd);
    let mut runtime = FirmwareRuntime::new(KlipperCamConfig::WROVER_KIT_HD, fw_cam);
    runtime.on_wifi_sta_start();
    if runtime
        .on_wifi_got_ip(Ipv4Addr::new(172, 22, 1, 52))
        .is_err()
    {
        return false;
    }

    let mut resp_buf = [0u8; 1024];
    let status_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(status_resp) {
        Ok(view) if view.status_code == 200 => {}
        _ => return false,
    }

    let snap_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(snap_resp) {
        Ok(view) if view.status_code == 200 && view.body == &jpeg_encoded[..jpeg_len] => {}
        _ => return false,
    }

    let stream_resp = match runtime.server.handle_raw_request(
        ServerPort::Stream,
        b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(stream_resp) {
        Ok(view) if view.status_code == 200 && view.content_type == Some(MJPEG_CONTENT_TYPE) => {}
        _ => return false,
    }

    let ota_payload = EspImageHeader::minimal_valid_header_bytes(4, 0x4008_0400);
    let mut ota_req = [0u8; 256];
    let req_len = {
        let mut w = SliceWriter::new(&mut ota_req);
        let _ = w.write_str_checked("POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: ");
        let _ = w.write_str_checked(DEFAULT_OTA_TOKEN);
        let _ = w.write_str_checked("\r\nContent-Length: 24\r\n\r\n");
        let _ = w.write_bytes(&ota_payload);
        w.written_len()
    };
    let ota_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        &ota_req[..req_len],
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(ota_resp) {
        Ok(view) if view.status_code == 200 => {}
        _ => return false,
    }

    runtime.server.is_reboot_scheduled()
        && runtime.server.last_ota_target_slot() == Some(OtaSlot::Ota1)
        && runtime.server.last_ota_bytes_written() == 24
}

pub fn run_firmware_self_test() -> bool {
    hil_check_pins_and_xclk()
        && hil_check_sccb_and_ov2640_hd()
        && hil_check_ll_dma_and_i2s0()
        && hil_check_jpeg_pool_and_http_ota()
}

#[cfg(not(target_arch = "xtensa"))]
fn main() {
    let ok = run_firmware_self_test();
    assert!(ok, "klipper-cam-fw self-test failed");
}

#[cfg(target_arch = "xtensa")]
#[repr(C, align(4))]
struct AlignedDmaBuf([u8; 32768]);

#[cfg(target_arch = "xtensa")]
static mut I2S_DMA_DESC: [Lldesc; 16] = [Lldesc::empty(); 16];

#[cfg(target_arch = "xtensa")]
static mut I2S_DMA_BUF: AlignedDmaBuf = AlignedDmaBuf([0u8; 32768]);

#[cfg(target_arch = "xtensa")]
struct EspFlashAdapter<'d>(esp_storage::FlashStorage<'d>);

#[cfg(target_arch = "xtensa")]
impl klipper_cam_fw::OtaFlashStorage for EspFlashAdapter<'_> {
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), klipper_cam_fw::OtaError> {
        self.0
            .read_nor(offset, buf)
            .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError)
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), klipper_cam_fw::OtaError> {
        self.0
            .erase(from, to)
            .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError)
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), klipper_cam_fw::OtaError> {
        self.0
            .write_nor(offset, data)
            .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError)
    }
}

#[cfg(target_arch = "xtensa")]
struct SmoltcpWifiDevice(esp_radio::wifi::Interface);

#[cfg(target_arch = "xtensa")]
struct SmoltcpRxToken(esp_radio::wifi::WifiRxToken);

#[cfg(target_arch = "xtensa")]
struct SmoltcpTxToken(esp_radio::wifi::WifiTxToken);

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::RxToken for SmoltcpRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        self.0.consume_token(|buf| f(buf))
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::TxToken for SmoltcpTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.0.consume_token(len, f)
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::Device for SmoltcpWifiDevice {
    type RxToken<'a> = SmoltcpRxToken where Self: 'a;
    type TxToken<'a> = SmoltcpTxToken where Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.0
            .receive()
            .map(|(rx, tx)| (SmoltcpRxToken(rx), SmoltcpTxToken(tx)))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.0.transmit().map(SmoltcpTxToken)
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        // Must be <= esp-radio's `ESP_RADIO_CONFIG_WIFI_MTU` (1492) so `WifiTxToken::consume_token`
        // stack buffer `[u8; 1492]` never panics on slice indexing `&mut buffer[..len]`.
        caps.max_transmission_unit = 1480;
        caps.max_burst_size = Some(3);
        caps
    }
}

#[cfg(target_arch = "xtensa")]
fn now_millis() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

#[cfg(target_arch = "xtensa")]
fn smol_now() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_millis(now_millis() as i64)
}

#[cfg(target_arch = "xtensa")]
fn block_on_with_timeout<F: core::future::Future>(
    mut fut: F,
    timeout_ms: u64,
) -> Option<F::Output> {
    use core::{
        pin::Pin,
        task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
    };
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |p| RawWaker::new(p, &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };
    let start_ms = now_millis();
    loop {
        if let Poll::Ready(res) = fut.as_mut().poll(&mut cx) {
            return Some(res);
        }
        if now_millis().saturating_sub(start_ms) >= timeout_ms {
            return None;
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn send_arp_announce(
    net_device: &mut SmoltcpWifiDevice,
    mac: [u8; 6],
    src_ip: [u8; 4],
    target_ip: [u8; 4],
) {
    if let Some(tx_tok) = net_device.0.transmit() {
        tx_tok.consume_token(42, |buf| {
            // Destination MAC: Broadcast (FF:FF:FF:FF:FF:FF)
            buf[0..6].copy_from_slice(&[0xFF; 6]);
            // Source MAC
            buf[6..12].copy_from_slice(&mac);
            // EtherType: ARP (0x0806)
            buf[12] = 0x08;
            buf[13] = 0x06;
            // Hardware type: Ethernet (1), Protocol type: IPv4 (0x0800)
            buf[14..20].copy_from_slice(&[0x00, 0x01, 0x08, 0x00, 0x06, 0x04]);
            // Opcode: ARP Request (1)
            buf[20] = 0x00;
            buf[21] = 0x01;
            // Sender Hardware Address (MAC) + Sender Protocol Address (src_ip)
            buf[22..28].copy_from_slice(&mac);
            buf[28..32].copy_from_slice(&src_ip);
            // Target Hardware Address (00:00:00:00:00:00) + Target Protocol Address (target_ip)
            buf[32..38].copy_from_slice(&[0x00; 6]);
            buf[38..42].copy_from_slice(&target_ip);
        });
    }
}

#[cfg(target_arch = "xtensa")]
const PORT80_SOCKET_COUNT: usize = 4;

#[cfg(target_arch = "xtensa")]
const PORT81_SOCKET_COUNT: usize = 2;

#[cfg(target_arch = "xtensa")]
const TCP_MSS_BYTES: usize = 1426;

#[cfg(target_arch = "xtensa")]
const TCP_BATCH_SEGMENTS: usize = 8;

#[cfg(target_arch = "xtensa")]
const TCP_MAX_INFLIGHT_BYTES: usize = TCP_BATCH_SEGMENTS * TCP_MSS_BYTES;

#[cfg(target_arch = "xtensa")]
fn enqueue_tcp_sliding_window(
    sock: &mut smoltcp::socket::tcp::Socket<'_>,
    buf: &[u8],
    tx_off: &mut usize,
    tx_len: usize,
) {
    if *tx_off >= tx_len || !sock.can_send() {
        return;
    }
    if sock.send_queue() == 0 {
        let chunk_end = (*tx_off + TCP_MAX_INFLIGHT_BYTES).min(tx_len);
        if chunk_end > *tx_off {
            if let Ok(sent) = sock.send_slice(&buf[*tx_off..chunk_end]) {
                *tx_off += sent;
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn poll_net_burst(
    iface: &mut smoltcp::iface::Interface,
    net_device: &mut SmoltcpWifiDevice,
    sockets: &mut smoltcp::iface::SocketSet<'_>,
) {
    for _ in 0..3u8 {
        let now = smol_now();
        let _ = iface.poll(now, net_device, sockets);
        if iface.poll_delay(now, sockets) != Some(smoltcp::time::Duration::ZERO) {
            break;
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn connect_strongest_ap(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    base_sta_cfg: &esp_radio::wifi::sta::StationConfig,
    ssid_str: &str,
    timeout_ms: u64,
) -> Option<esp_radio::wifi::sta::ConnectedInfo> {
    use esp_hal::time::Duration;
    use esp_println::println;
    use esp_radio::wifi::{
        scan::{ScanConfig, ScanTypeConfig},
        Config as WifiConfig, PowerSaveMode,
    };

    let _ = wifi_controller.set_max_tx_power(78);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);

    let mut scan_cfg = ScanConfig::default()
        .with_max(8)
        .with_scan_type(ScanTypeConfig::Active {
            min: Duration::from_millis(20),
            max: Duration::from_millis(45),
        });
    if let Ok(s) = ssid_str.try_into() {
        scan_cfg = scan_cfg.with_ssid(s);
    }

    let best_ap = match block_on_with_timeout(wifi_controller.scan_async(&scan_cfg), 2500) {
        Some(Ok(aps)) => aps
            .into_iter()
            .filter(|ap| ap.ssid.as_str() == ssid_str)
            .max_by_key(|ap| ap.signal_strength),
        _ => None,
    };

    if let Some(ap) = best_ap {
        println!(
            "[KLIPPER-CAM-RS] [WIFI] Strongest '{}' AP: ch={} rssi={}dBm bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            ssid_str,
            ap.channel,
            ap.signal_strength,
            ap.bssid[0],
            ap.bssid[1],
            ap.bssid[2],
            ap.bssid[3],
            ap.bssid[4],
            ap.bssid[5]
        );
        let locked_cfg = base_sta_cfg
            .clone()
            .with_bssid(ap.bssid)
            .with_channel(ap.channel);
        if wifi_controller
            .set_config(&WifiConfig::Station(locked_cfg))
            .is_ok()
        {
            let _ = wifi_controller.set_max_tx_power(78);
            match block_on_with_timeout(wifi_controller.connect_async(), timeout_ms) {
                Some(Ok(info)) => {
                    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
                    let _ = wifi_controller.set_max_tx_power(78);
                    return Some(info);
                }
                Some(Err(_)) => {}
                None => {
                    let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 500);
                }
            }
        }
    }

    let _ = wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
    let _ = wifi_controller.set_max_tx_power(78);
    match block_on_with_timeout(wifi_controller.connect_async(), timeout_ms) {
        Some(Ok(info)) => {
            let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
            let _ = wifi_controller.set_max_tx_power(78);
            Some(info)
        }
        Some(Err(_)) => None,
        None => {
            let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 500);
            None
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct Port80WorkerState {
    req_len: usize,
    tx_off: usize,
    tx_len: usize,
    closing: bool,
    close_deadline_ms: i64,
    using_ota_accum: bool,
}

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct Port81WorkerState {
    req_len: usize,
    closing: bool,
    close_deadline_ms: i64,
}

#[cfg(target_arch = "xtensa")]
fn capture_live_hd_jpeg_with_diag(
    i2s_cam: &mut Esp32I2sCamera,
    jpeg_scratch: &mut [u8],
    ping_pong: &mut PingPongFrameBuffer<'_>,
    poll_net: &mut dyn FnMut(),
) -> Result<usize, (u32, usize, u8, u8, u8, u32)> {
    const GPIO_IN_REG: *const u32 = 0x3FF4_403C as *const u32;
    const I2S_INT_RAW_REG: *const u32 = 0x3FF4_F00C as *const u32;
    const I2S_INT_CLR_REG: *mut u32 = 0x3FF4_F018 as *mut u32;
    const I2S_IN_SUC_EOF_BIT: u32 = 1 << 9;

    let (desc_slice, dma_slice) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(I2S_DMA_DESC),
            &mut (*core::ptr::addr_of_mut!(I2S_DMA_BUF)).0,
        )
    };
    let _ = build_circular_ring(desc_slice, dma_slice, 2048);

    let dma_cfg = esp32_cam_ll::DmaSizeConfig {
        dma_bytes_per_item: 4,
        dma_buffer_size: 32768,
        dma_half_buffer_size: 4096,
        dma_half_buffer_cnt: 8,
        dma_node_buffer_size: 2048,
        dma_node_cnt: 16,
        lines_per_node: 1,
        nodes_per_line: 1,
        lines_per_half_buffer: 1,
        image_size: jpeg_scratch.len(),
    };

    let read_vsync = || -> bool {
        ((unsafe { core::ptr::read_volatile(GPIO_IN_REG) } >> 25) & 1) != 0
    };

    // Wait for VSYNC vertical blanking transition (bounded to 90 ms) while servicing smoltcp
    let sync_start_ms = now_millis();
    let mut spin = 0u32;
    while !read_vsync() && (now_millis().saturating_sub(sync_start_ms) < 45) {
        spin = spin.wrapping_add(1);
        if (spin & 0x7F) == 0 {
            poll_net();
        }
    }
    let vsync_high_ms = now_millis();
    while read_vsync() && (now_millis().saturating_sub(sync_start_ms) < 90) {
        if now_millis().saturating_sub(vsync_high_ms) >= 2 {
            spin = spin.wrapping_add(1);
            if (spin & 0x7F) == 0 {
                poll_net();
            }
        }
    }

    if !i2s_cam.start(&dma_cfg, desc_slice[0].link_addr_20bit()) {
        return Err((0, 0, 0, 0, 0, 0xDEAD));
    }

    let mut eof_cnt = 0u32;
    let mut fb_len = 0usize;
    let mut seen_vsync_high = false;
    let mut last_int_raw = 0u32;
    let dma_start_ms = now_millis();
    let mut last_eof_ms = dma_start_ms;

    while now_millis().saturating_sub(dma_start_ms) < 140 {
        if read_vsync() {
            seen_vsync_high = true;
        }

        let int_raw = unsafe { core::ptr::read_volatile(I2S_INT_RAW_REG) };
        last_int_raw |= int_raw;

        if (int_raw & I2S_IN_SUC_EOF_BIT) != 0 {
            unsafe {
                core::ptr::write_volatile(I2S_INT_CLR_REG, I2S_IN_SUC_EOF_BIT);
            }
            last_eof_ms = now_millis();

            let half_idx = (eof_cnt as usize) & 7;
            let half_offset = half_idx * 4096;
            if fb_len + 1024 <= jpeg_scratch.len() {
                let _ = i2s_cam.memcpy_filtered(
                    &mut jpeg_scratch[fb_len..fb_len + 1024],
                    &dma_slice[half_offset..half_offset + 4096],
                );
                let flags = Lldesc::pack_flags(2048, 0, 0, false, false, true);
                unsafe {
                    core::ptr::write_volatile(
                        &mut desc_slice[2 * half_idx].size_length_flags,
                        flags,
                    );
                    core::ptr::write_volatile(
                        &mut desc_slice[2 * half_idx + 1].size_length_flags,
                        flags,
                    );
                }
                fb_len += 1024;
                eof_cnt += 1;
            } else {
                let _ = i2s_cam.stop();
                break;
            }
            continue;
        }

        if eof_cnt > 0 {
            let vsync_ended = seen_vsync_high
                && !read_vsync()
                && !read_vsync()
                && !read_vsync()
                && !read_vsync();
            if vsync_ended || now_millis().saturating_sub(last_eof_ms) >= 15 {
                let _ = i2s_cam.stop();
                let half_idx = (eof_cnt as usize) & 7;
                let half_offset = half_idx * 4096;
                if fb_len + 1024 <= jpeg_scratch.len() {
                    let _ = i2s_cam.memcpy_filtered(
                        &mut jpeg_scratch[fb_len..fb_len + 1024],
                        &dma_slice[half_offset..half_offset + 4096],
                    );
                    fb_len += 1024;
                }
                break;
            }
        }
    }

    let _ = i2s_cam.stop();

    if fb_len >= 64 {
        if let Some(soi_pos) = verify_jpeg_soi(&jpeg_scratch[..fb_len]) {
            let slice_from_soi = &jpeg_scratch[soi_pos..fb_len];
            if let Some(eoi_rel) = verify_jpeg_eoi(slice_from_soi, false) {
                let exact_len = eoi_rel + 2;
                if exact_len >= 64
                    && ping_pong
                        .write_and_commit_inactive(&slice_from_soi[..exact_len])
                        .is_ok()
                {
                    return Ok(exact_len);
                }
            }
        }
    }

    let b0 = if fb_len > 0 { jpeg_scratch[0] } else { 0 };
    let b1 = if fb_len > 1 { jpeg_scratch[1] } else { 0 };
    let b2 = if fb_len > 2 { jpeg_scratch[2] } else { 0 };
    Err((eof_cnt, fb_len, b0, b1, b2, last_int_raw))
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn active_ping_pong_static_slice(pp: &PingPongFrameBuffer<'_>) -> &'static [u8] {
    let active = pp.active_frame();
    unsafe { core::slice::from_raw_parts(active.as_ptr(), active.len()) }
}

/// Bare-metal Xtensa firmware entrypoint for `ESP32-WROVER-DEV`.
#[cfg(target_arch = "xtensa")]
#[esp_hal::main]
fn main() -> ! {
    use core::sync::atomic::Ordering;
    use esp_hal::{
        clock::CpuClock,
        delay::Delay,
        gpio::{DriveMode, Input, InputConfig, Pull},
        i2c::master::{Config as I2cConfig, I2c},
        ledc::{
            channel::{self, ChannelIFace},
            timer::{self, TimerIFace},
            LSGlobalClkSource, Ledc, LowSpeed,
        },
        psram::{Psram, PsramCacheSpeed, PsramConfig, PsramSize, PsramVaddrMode},
        time::Rate,
        timer::timg::TimerGroup,
    };
    use esp_println::println;
    use esp_radio::wifi::{
        sta::{ScanMethod, StationConfig},
        AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig,
        Interface as WifiInterface, PowerSaveMode, WifiController,
    };
    use klipper_cam_fw::{
        active_wifi_pass, active_wifi_ssid, parse_http_request, EmbeddedHalSccb, GainCeiling,
        HttpMethod, OtaDataManager, SccbBus, StreamSessionGuard, OTA_PARTITION_MAX_SIZE,
        SYNTHETIC_HD_JFIF_FRAME,
    };
    use smoltcp::{
        iface::{Config as IfaceConfig, Interface as NetInterface, SocketSet, SocketStorage},
        socket::{
            dhcpv4,
            tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer, State as TcpState},
        },
        wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address},
    };

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);
    let delay = Delay::new();

    // Initialize internal DRAM heap for esp-rtos and esp-radio Wi-Fi STA (~112 KB strictly internal)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 48 * 1024);

    // Initialize 4 MB external PSRAM at 80 MHz
    let psram = Psram::new(
        peripherals.PSRAM,
        PsramConfig {
            size: PsramSize::Size(4 * 1024 * 1024),
            cache_speed: PsramCacheSpeed::PsramCacheF80mS40m,
            psram_vaddr_mode: PsramVaddrMode::Normal,
        },
    );
    let (psram_ptr, psram_len) = psram.raw_parts();

    println!("============================================================");
    println!("[KLIPPER-CAM-RS] Bare-Metal Xtensa Rust Firmware v0.1.0");
    println!("[KLIPPER-CAM-RS] Target: ESP32-WROVER-DEV (240 MHz Dual-Core)");
    println!(
        "[KLIPPER-CAM-RS] [PSRAM] Mapped {} bytes @ 0x{:08X} (80 MHz)",
        psram_len, psram_ptr as usize
    );
    println!("============================================================");

    // Carve PSRAM buffers for Ping-Pong 1280x720 JPEG slots, 4-socket Port 80 pool, 2-socket Port 81 pool, and OTA buffer
    let psram_all: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_ptr, psram_len) };
    let (jpeg_slot_a, rem) = psram_all.split_at_mut(128 * 1024);
    let (jpeg_slot_b, rem) = rem.split_at_mut(128 * 1024);
    let (jpeg_scratch_buf, rem) = rem.split_at_mut(128 * 1024);

    // 4 Port 80 worker buffers: rx (8 KB x 4), tx (64 KB x 4), req_hdr (4 KB x 4), resp (128 KB x 4)
    let (p80_rx0, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx1, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx2, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx3, rem) = rem.split_at_mut(8 * 1024);
    let (p80_tx0, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx1, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx2, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx3, rem) = rem.split_at_mut(64 * 1024);
    let (p80_hdr0, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr1, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr2, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr3, rem) = rem.split_at_mut(4 * 1024);
    let (p80_resp0, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp1, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp2, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp3, rem) = rem.split_at_mut(128 * 1024);

    // 2-socket Port 81 pool buffers + shared 128 KB MJPEG frame chunk buffer
    let (tcp81_rx0, rem) = rem.split_at_mut(4 * 1024);
    let (tcp81_tx0, rem) = rem.split_at_mut(128 * 1024);
    let (tcp81_rx1, rem) = rem.split_at_mut(4 * 1024);
    let (tcp81_tx1, rem) = rem.split_at_mut(128 * 1024);
    let (p81_resp_buf, rem) = rem.split_at_mut(128 * 1024);

    // Shared 1800 KB OTA upload accumulator; remaining PSRAM stays unmapped from esp_alloc::HEAP
    // so esp-radio / esp-rtos malloc() can NEVER spill internal DMA/Wi-Fi allocations into external PSRAM.
    let (http80_ota_accum, psram_heap_rem) = rem.split_at_mut(1800 * 1024);
    let free_psram_bytes = psram_heap_rem.len() as u32;

    let mut ping_pong = PingPongFrameBuffer::new(
        jpeg_slot_a,
        jpeg_slot_b,
        &SYNTHETIC_HD_JFIF_FRAME,
    )
    .expect("PingPongFrameBuffer::new");

    // 1. Configure physical 20 MHz XCLK on GPIO 21 via LEDC LowSpeed Timer0 / Channel0
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let xclk_timer_ok = lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty1Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_mhz(20),
        })
        .is_ok();
    let mut channel0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO21);
    let xclk_hw_ok = xclk_timer_ok
        && channel0
            .configure(channel::config::Config {
                timer: &lstimer0,
                duty_pct: 50,
                drive_mode: DriveMode::PushPull,
            })
            .is_ok();
    println!(
        "[KLIPPER-CAM-RS] [HW-1] LEDC 20 MHz XCLK on GPIO 21: {}",
        if xclk_hw_ok { "PASS" } else { "FAIL" }
    );
    delay.delay_millis(25);

    // 2. Configure physical I2C0 on GPIO 26 (SDA) / GPIO 27 (SCL) at 100 kHz and program OV2640
    let mut ov2640_hw_ok = false;
    let mut live_ov2640: Option<Ov2640> = None;
    let i2c_cfg = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let mut kept_sccb = match I2c::new(peripherals.I2C0, i2c_cfg) {
        Ok(i2c_bus) => {
            let i2c_bus = i2c_bus
                .with_sda(peripherals.GPIO26)
                .with_scl(peripherals.GPIO27);
            let mut hw_sccb = EmbeddedHalSccb::new(i2c_bus, Delay::new());
            let _ = hw_sccb.write_reg8(Ov2640::SLV_ADDR, 0xFF, 0x01);
            match Ov2640::detect(&mut hw_sccb, Ov2640::SLV_ADDR) {
                Ok(id) => {
                    let mut ov = Ov2640::from_id(Ov2640::SLV_ADDR, id);
                    // Program PixelFormat::Jpeg BEFORE set_framesize(FrameSize::Hd) so set_window
                    // selects the JPEG CLKRC/PCLK dividers (`div = 12`), then apply OV2640 ISP settings
                    let prog_ok = ov.reset(&mut hw_sccb).is_ok()
                        && ov
                            .set_pixformat(&mut hw_sccb, PixelFormat::Jpeg)
                            .is_ok()
                        && ov.set_framesize(&mut hw_sccb, FrameSize::Hd).is_ok()
                        && ov
                            .set_gainceiling(&mut hw_sccb, GainCeiling::X2)
                            .is_ok()
                        && ov.set_bpc(&mut hw_sccb, false).is_ok()
                        && ov.set_wpc(&mut hw_sccb, true).is_ok()
                        && ov.set_lenc(&mut hw_sccb, true).is_ok()
                        && ov
                            .set_quality(&mut hw_sccb, DEFAULT_JPEG_QUALITY as i32)
                            .is_ok()
                        && ov.init_status(&mut hw_sccb).is_ok();
                    ov2640_hw_ok = prog_ok;
                    println!(
                        "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (0x30) OV2640 DETECTED: PID=0x{:02X} VER=0x{:02X} MIDH=0x{:02X} MIDL=0x{:02X} HD_PROG={}",
                        id.pid, id.ver, id.midh, id.midl, prog_ok
                    );
                    live_ov2640 = Some(ov);
                }
                Err(_) => {
                    println!(
                        "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (GPIO26/GPIO27 @ 100kHz): Bus active (no external OV2640 ribbon sensor ACK at 0x30)"
                    );
                }
            }
            Some(hw_sccb)
        }
        Err(_) => {
            println!("[KLIPPER-CAM-RS] [HW-2] I2C0 Master Init: FAIL");
            None
        }
    };

    // 3. Execute on-chip 7-crate HIL verification suite (before arming live I2S0 camera)
    let check1 = hil_check_pins_and_xclk();
    println!(
        "[KLIPPER-CAM-RS] [HIL-1] WroverKitPins + XclkController: {}",
        if check1 { "PASS" } else { "FAIL" }
    );

    let check2 = hil_check_sccb_and_ov2640_hd();
    println!(
        "[KLIPPER-CAM-RS] [HIL-2] Ov2640 1280x720 HD (OV-105 HSIZE8=0xC8 VSIZE8=0x96 YOFFL=150): {}",
        if check2 { "PASS" } else { "FAIL" }
    );

    let check3 = hil_check_ll_dma_and_i2s0();
    println!(
        "[KLIPPER-CAM-RS] [HIL-3] Esp32I2sCamera + Lldesc Circular DMA Ring: {}",
        if check3 { "PASS" } else { "FAIL" }
    );

    let check4 = hil_check_jpeg_pool_and_http_ota();
    println!(
        "[KLIPPER-CAM-RS] [HIL-4] DCT JPEG + FramePool + Port 80/81 HTTP/MJPEG + OTA: {}",
        if check4 { "PASS" } else { "FAIL" }
    );

    let all_ok = xclk_hw_ok && check1 && check2 && check3 && check4;
    println!(
        "[HIL SUMMARY] 7-Crate Bare-Metal Xtensa Self-Test: {}",
        if all_ok { "ALL PASS (100%)" } else { "FAIL" }
    );

    // 4. Configure live I2S0 parallel camera slave DMA and capture initial 1280x720 JPEG frame
    let mut hw_i2s_cam = Esp32I2sCamera::new();
    let ll_pins = WroverKitPins::PINS.to_ll_pin_config(false);
    let _ = hw_i2s_cam.set_pin(&ll_pins);
    let _ = hw_i2s_cam.config();
    let _ = hw_i2s_cam.set_sample_mode(PixelFormat::Jpeg, DEFAULT_XCLK_FREQ_HZ, 0x26);

    // Configure physical camera input pads via esp-hal AFTER `set_pin` so `RTC_IO` (`34..=39`)
    // and `IO_MUX` (`4, 5, 18, 19, 22, 23, 25`) use esp-hal's verified pad bitfields;
    // `hw_i2s_cam.start()` latches all 12 `GPIO_FUNC0_IN_SEL_CFG_REG` signals (`I2S0I_WS_IN_IDX = 28`).
    let _cam_inputs = (
        Input::new(peripherals.GPIO4, InputConfig::default()),
        Input::new(peripherals.GPIO5, InputConfig::default()),
        Input::new(peripherals.GPIO18, InputConfig::default()),
        Input::new(peripherals.GPIO19, InputConfig::default()),
        Input::new(peripherals.GPIO36, InputConfig::default()),
        Input::new(peripherals.GPIO39, InputConfig::default()),
        Input::new(peripherals.GPIO34, InputConfig::default()),
        Input::new(peripherals.GPIO35, InputConfig::default()),
        Input::new(peripherals.GPIO22, InputConfig::default()),
        Input::new(peripherals.GPIO23, InputConfig::default()),
        Input::new(
            peripherals.GPIO25,
            InputConfig::default().with_pull(Pull::Up),
        ),
    );
    delay.delay_millis(80);

    let mut ov2640_dma_ok = false;

    if ov2640_hw_ok {
        for attempt in 1..=4u8 {
            let inv_try = attempt >= 3;
            if attempt == 3 {
                let _ = hw_i2s_cam.set_pin(&WroverKitPins::PINS.to_ll_pin_config(true));
            }
            match capture_live_hd_jpeg_with_diag(
                &mut hw_i2s_cam,
                jpeg_scratch_buf,
                &mut ping_pong,
                &mut || {},
            ) {
                Ok(captured_len) => {
                    ov2640_dma_ok = true;
                    println!(
                        "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA 1280x720 JPEG Capture: PASS ({} bytes, attempt {}, inv={})",
                        captured_len, attempt, inv_try
                    );
                    break;
                }
                Err((eof_cnt, fb_len, b0, b1, b2, int_raw)) => {
                    println!(
                        "[KLIPPER-CAM-RS] [HW-3] Attempt {} (inv={}): eof_cnt={} fb_len={} head=[0x{:02X},0x{:02X},0x{:02X}] int_raw=0x{:08X}",
                        attempt, inv_try, eof_cnt, fb_len, b0, b1, b2, int_raw
                    );
                    delay.delay_millis(25);
                }
            }
        }
    }
    if !ov2640_dma_ok {
        println!(
            "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA Capture fallback: using synthetic 1280x720 JFIF ({} bytes)",
            ping_pong.active_len()
        );
    }
    println!("============================================================");

    // 5. Initialize SPI NOR Flash adapter, load otadata state, and cancel rollback
    //    BEFORE starting esp-rtos / Wi-Fi so SPI NOR Flash sector erase never runs while Wi-Fi interrupts are active.
    let mut flash_adapter = EspFlashAdapter(esp_storage::FlashStorage::new(peripherals.FLASH));
    let mut ota_manager = OtaDataManager::load_from_flash(&mut flash_adapter)
        .unwrap_or_else(|_| OtaDataManager::new_booted_ota0());
    let _ = ota_manager.cancel_rollback_in_flash(&mut flash_adapter);

    let initial_slice = active_ping_pong_static_slice(&ping_pong);
    let mut runtime = FirmwareRuntime::new(
        KlipperCamConfig::WROVER_KIT_HD,
        CameraDriver::new(initial_slice, FrameSize::Hd),
    );
    runtime.server.ota_manager = ota_manager;

    // 6. Start esp-rtos scheduler and initialize esp-radio Wi-Fi Station
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let ssid = active_wifi_ssid();
    let pass = active_wifi_pass();
    let auth = if pass.is_empty() {
        AuthenticationMethodConfig::Open
    } else {
        match pass.try_into() {
            Ok(pw) => AuthenticationMethodConfig::WpaWpa2Personal(pw),
            Err(_) => AuthenticationMethodConfig::Open,
        }
    };
    let mut sta_cfg = StationConfig::default()
        .with_authentication(auth)
        .with_scan_method(ScanMethod::AllChannels)
        .with_failure_retry_cnt(5);
    if let Ok(s) = ssid.try_into() {
        sta_cfg = sta_cfg.with_ssid(s);
    };

    let ctrl_cfg = ControllerConfig::default()
        .with_rx_queue_size(16)
        .with_tx_queue_size(12)
        .with_initial_config(WifiConfig::Station(sta_cfg.clone()));
    let mut wifi_controller =
        WifiController::new(peripherals.WIFI, ctrl_cfg).expect("WifiController::new");
    let _ = wifi_controller.set_max_tx_power(78);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);

    let wifi_iface = WifiInterface::station();
    let mac = wifi_iface.mac_address();
    println!(
        "[KLIPPER-CAM-RS] [WIFI] Connecting to SSID '{}' (MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})...",
        ssid, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    runtime.on_wifi_sta_start();

    match connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 8000) {
        Some(info) => {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] Connected to AP (channel {}, aid {})",
                info.channel, info.aid
            );
        }
        None => {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] Initial connect warning/timeout; will retry in loop"
            );
            runtime.on_wifi_sta_disconnected();
        }
    }

    // 7. Configure smoltcp Ethernet Interface + DHCPv4 + Static IP (172.22.1.52/24, GW 172.22.1.1)
    //    + 4-socket Port 80 pool + 2-socket Port 81 pool (7 sockets total)
    let mut net_device = SmoltcpWifiDevice(wifi_iface);
    let mut iface_cfg = IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    const RNG_DATA_REG: *const u32 = 0x3FF7_5144 as *const u32;
    let hw_rng_lo = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let hw_rng_hi = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let boot_us = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    iface_cfg.random_seed = ((hw_rng_hi << 32) | hw_rng_lo) ^ boot_us ^ 0x9E37_79B9_7F4A_7C15;
    let mut iface = NetInterface::new(iface_cfg, &mut net_device, smol_now());
    iface.update_ip_addrs(|addrs| {
        let _ = addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), 24));
    });
    let _ = iface
        .routes_mut()
        .add_default_ipv4_route(Ipv4Address::new(172, 22, 1, 1));

    let mut socket_storage = [SocketStorage::EMPTY; 7];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

    let dhcp_handle = sockets.add(dhcpv4::Socket::new());
    let tcp80_handles: [smoltcp::iface::SocketHandle; PORT80_SOCKET_COUNT] = [
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx0),
            TcpSocketBuffer::new(p80_tx0),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx1),
            TcpSocketBuffer::new(p80_tx1),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx2),
            TcpSocketBuffer::new(p80_tx2),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx3),
            TcpSocketBuffer::new(p80_tx3),
        )),
    ];
    for &h80 in &tcp80_handles {
        let s80 = sockets.get_mut::<TcpSocket>(h80);
        s80.set_nagle_enabled(false);
        s80.set_timeout(Some(smoltcp::time::Duration::from_millis(60_000)));
        let _ = s80.listen(80);
    }
    let p80_hdr_bufs: [&mut [u8]; PORT80_SOCKET_COUNT] =
        [p80_hdr0, p80_hdr1, p80_hdr2, p80_hdr3];
    let p80_resp_bufs: [&mut [u8]; PORT80_SOCKET_COUNT] =
        [p80_resp0, p80_resp1, p80_resp2, p80_resp3];
    let mut p80_states = [Port80WorkerState::default(); PORT80_SOCKET_COUNT];
    let mut ota_accum_owner: Option<usize> = None;

    let tcp81_handles: [smoltcp::iface::SocketHandle; PORT81_SOCKET_COUNT] = [
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(tcp81_rx0),
            TcpSocketBuffer::new(tcp81_tx0),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(tcp81_rx1),
            TcpSocketBuffer::new(tcp81_tx1),
        )),
    ];
    for &h81 in &tcp81_handles {
        let s81 = sockets.get_mut::<TcpSocket>(h81);
        s81.set_nagle_enabled(false);
        s81.set_timeout(Some(smoltcp::time::Duration::from_millis(10_000)));
        let _ = s81.listen(81);
    }
    let mut p81_req_bufs = [[0u8; 1024]; PORT81_SOCKET_COUNT];
    let mut p81_states = [Port81WorkerState::default(); PORT81_SOCKET_COUNT];
    let mut p81_stream_owner: Option<usize> = None;
    let mut p81_last_frame_ms: i64 = 0;
    let mut p81_tx_off: usize = 0;
    let mut p81_tx_len: usize = 0;

    let mut ip_configured = false;
    let mut connect_ms = smol_now().total_millis();
    let mut last_reconnect_attempt_ms = connect_ms;
    let mut last_heartbeat_ms = connect_ms;
    let mut last_arp_announce_ms = connect_ms;
    let mut last_capture_ms = connect_ms;
    let mut post_snapshot_refresh_at_ms: Option<i64> = None;
    let mut last_applied_ctrl = klipper_cam_fw::CameraControlState::new_default();
    let mut weak_rssi_since_ms: Option<i64> = None;
    let mut last_roam_attempt_ms: i64 = connect_ms;

    loop {
        let now = smol_now();
        let now_ms = now.total_millis();

        // Wi-Fi auto-reconnect every 2000 ms if link drops (bounded 4000 ms timeout)
        if !wifi_controller.is_connected() && now_ms - last_reconnect_attempt_ms >= 2000 {
            last_reconnect_attempt_ms = now_ms;
            runtime.on_wifi_sta_disconnected();
            if let Some(info) = connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 4000) {
                println!(
                    "[KLIPPER-CAM-RS] [WIFI] Reconnected to AP (channel {})",
                    info.channel
                );
                connect_ms = smol_now().total_millis();
                weak_rssi_since_ms = None;
                if ip_configured {
                    send_arp_announce(
                        &mut net_device,
                        mac,
                        [172, 22, 1, 52],
                        [172, 22, 1, 1],
                    );
                    last_arp_announce_ms = connect_ms;
                }
            }
        }

        let _ = iface.poll(now, &mut net_device, &mut sockets);

        // Check DHCPv4 events or configure static IP (172.22.1.52/24, GW 172.22.1.1) immediately on connect
        {
            let dhcp_sock = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
            match dhcp_sock.poll() {
                Some(dhcpv4::Event::Configured(cfg)) => {
                    let addr = cfg.address.address();
                    let octets = addr.octets();
                    let prefix_len = cfg.address.prefix_len();
                    iface.update_ip_addrs(|addrs| {
                        if !addrs
                            .iter()
                            .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                        {
                            let _ =
                                addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), prefix_len));
                        }
                        if addr != Ipv4Address::new(172, 22, 1, 52)
                            && !addrs.iter().any(|c| c.address() == IpAddress::Ipv4(addr))
                        {
                            let _ = addrs.push(IpCidr::Ipv4(cfg.address));
                        }
                    });
                    let gw = cfg.router.unwrap_or(Ipv4Address::new(172, 22, 1, 1));
                    let _ = iface.routes_mut().add_default_ipv4_route(gw);
                    let fw_ip = Ipv4Addr::new(172, 22, 1, 52);
                    let _ = runtime.on_wifi_got_ip(fw_ip);
                    ip_configured = true;
                    send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                    last_arp_announce_ms = now_ms;
                    println!(
                        "[KLIPPER-CAM-RS] [NET] IP Configured: 172.22.1.52/{} + DHCP {}.{}.{}.{}/{} (Port 80 x4 & Port 81 x2 Ready)",
                        prefix_len, octets[0], octets[1], octets[2], octets[3], prefix_len
                    );
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    iface.update_ip_addrs(|addrs| {
                        if !addrs
                            .iter()
                            .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                        {
                            let _ = addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), 24));
                        }
                    });
                    let _ = iface
                        .routes_mut()
                        .add_default_ipv4_route(Ipv4Address::new(172, 22, 1, 1));
                    ip_configured = true;
                    send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                }
                None => {}
            }
        }

        if !ip_configured && wifi_controller.is_connected() {
            iface.update_ip_addrs(|addrs| {
                if !addrs
                    .iter()
                    .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                {
                    let _ = addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), 24));
                }
            });
            let _ = iface
                .routes_mut()
                .add_default_ipv4_route(Ipv4Address::new(172, 22, 1, 1));
            let fw_ip = Ipv4Addr::new(172, 22, 1, 52);
            let _ = runtime.on_wifi_got_ip(fw_ip);
            ip_configured = true;
            send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
            last_arp_announce_ms = now_ms;
            println!(
                "[KLIPPER-CAM-RS] [NET] Static IP Configured: 172.22.1.52/24 GW 172.22.1.1 (Port 80 x4 & Port 81 x2 Ready)"
            );
        }

        if !ip_configured {
            continue;
        }

        if now_ms - last_arp_announce_ms >= 5000 {
            last_arp_announce_ms = now_ms;
            send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
        }

        // ============================================================
        // On-Demand Post-Snapshot I2S0 Capture (Ping-Pong Double Buffer)
        // ============================================================
        let p80_busy = (0..PORT80_SOCKET_COUNT).any(|i| {
            (p80_states[i].req_len > 0 || p80_states[i].tx_len > 0) && !p80_states[i].closing
        });
        let p81_frame_inflight = p81_stream_owner
            .map(|owner| {
                p81_tx_off < p81_tx_len
                    || sockets.get_mut::<TcpSocket>(tcp81_handles[owner]).send_queue() > 0
            })
            .unwrap_or(false);
        let post_snap_due = post_snapshot_refresh_at_ms
            .map(|deadline| now_ms >= deadline)
            .unwrap_or(false);
        let idle_refresh_due =
            p81_stream_owner.is_none() && (now_ms - last_capture_ms >= 30_000);
        if ov2640_dma_ok
            && ota_accum_owner.is_none()
            && !runtime.server.is_reboot_scheduled()
            && !p80_busy
            && !p81_frame_inflight
            && (post_snap_due || idle_refresh_due)
        {
            post_snapshot_refresh_at_ms = None;
            last_capture_ms = now_ms;
            if capture_live_hd_jpeg_with_diag(
                &mut hw_i2s_cam,
                jpeg_scratch_buf,
                &mut ping_pong,
                &mut || {
                    let _ = iface.poll(smol_now(), &mut net_device, &mut sockets);
                },
            )
            .is_ok()
            {
                let fresh_slice = active_ping_pong_static_slice(&ping_pong);
                runtime.server.camera.set_frame(
                    fresh_slice,
                    runtime.server.control_state.framesize,
                );
                last_capture_ms = smol_now().total_millis();
            }
        }

        // Idle weak-RSSI roam check: if idle and RSSI stays worse than -68 dBm for > 15s, re-scan for strongest AP
        if wifi_controller.is_connected()
            && p81_stream_owner.is_none()
            && !p80_busy
            && ota_accum_owner.is_none()
            && !runtime.server.is_reboot_scheduled()
        {
            let cur_rssi = wifi_controller.rssi().unwrap_or(-50);
            if cur_rssi < -68 {
                let since = *weak_rssi_since_ms.get_or_insert(now_ms);
                if now_ms - since >= 15_000 && now_ms - last_roam_attempt_ms >= 180_000 {
                    last_roam_attempt_ms = now_ms;
                    weak_rssi_since_ms = None;
                    let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 500);
                    if let Some(info) =
                        connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 5000)
                    {
                        println!(
                            "[KLIPPER-CAM-RS] [WIFI] Roamed from weak RSSI ({} dBm) to strongest AP (channel {})",
                            cur_rssi, info.channel
                        );
                        send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                        last_arp_announce_ms = smol_now().total_millis();
                    }
                }
            } else {
                weak_rssi_since_ms = None;
            }
        }

        // ============================================================
        // Port 80 4-Socket Listener Pool (Control / Snapshot / Status / OTA)
        // ============================================================
        for idx in 0..PORT80_SOCKET_COUNT {
            let sock80 = sockets.get_mut::<TcpSocket>(tcp80_handles[idx]);
            let st = &mut p80_states[idx];

            if !sock80.is_open() {
                if ota_accum_owner == Some(idx) {
                    ota_accum_owner = None;
                }
                *st = Port80WorkerState::default();
                sock80.set_timeout(Some(smoltcp::time::Duration::from_millis(60_000)));
                let _ = sock80.listen(80);
            } else if st.closing {
                let finished_cleanly = matches!(
                    sock80.state(),
                    TcpState::TimeWait | TcpState::Closed
                );
                if !sock80.is_active() || finished_cleanly || now_ms >= st.close_deadline_ms {
                    sock80.abort();
                    if ota_accum_owner == Some(idx) {
                        ota_accum_owner = None;
                    }
                    *st = Port80WorkerState::default();
                    sock80.set_timeout(Some(smoltcp::time::Duration::from_millis(60_000)));
                    let _ = sock80.listen(80);
                }
            } else if st.tx_len > 0 {
                if st.tx_off < st.tx_len {
                    if sock80.can_send() {
                        enqueue_tcp_sliding_window(
                            sock80,
                            p80_resp_bufs[idx],
                            &mut st.tx_off,
                            st.tx_len,
                        );
                    } else if !sock80.may_send() {
                        sock80.abort();
                        if ota_accum_owner == Some(idx) {
                            ota_accum_owner = None;
                        }
                        *st = Port80WorkerState::default();
                        sock80.set_timeout(Some(smoltcp::time::Duration::from_millis(60_000)));
                        let _ = sock80.listen(80);
                    }
                } else if sock80.send_queue() == 0 || !sock80.may_send() {
                    sock80.close();
                    st.closing = true;
                    st.close_deadline_ms = now_ms + 1500;
                }
            } else if sock80.can_recv() {
                if !st.using_ota_accum {
                    let avail = p80_hdr_bufs[idx].len().saturating_sub(st.req_len);
                    if avail > 0 {
                        if let Ok(n) = sock80
                            .recv_slice(&mut p80_hdr_bufs[idx][st.req_len..st.req_len + avail])
                        {
                            st.req_len += n;
                        }
                    }

                    // Promote large POST /update payloads (> 4 KB) into shared 1,800 KB http80_ota_accum
                    if let Ok(parsed) = parse_http_request(&p80_hdr_bufs[idx][..st.req_len]) {
                        if let Some(cl) = parsed.content_length {
                            let hdr_bytes = st.req_len.saturating_sub(parsed.body.len());
                            if hdr_bytes.saturating_add(cl as usize) > p80_hdr_bufs[idx].len()
                                && ota_accum_owner.is_none()
                            {
                                http80_ota_accum[..st.req_len]
                                    .copy_from_slice(&p80_hdr_bufs[idx][..st.req_len]);
                                st.using_ota_accum = true;
                                ota_accum_owner = Some(idx);
                            }
                        }
                    }
                }

                if st.using_ota_accum && sock80.can_recv() {
                    let avail = http80_ota_accum.len().saturating_sub(st.req_len);
                    if avail > 0 {
                        if let Ok(n) = sock80
                            .recv_slice(&mut http80_ota_accum[st.req_len..st.req_len + avail])
                        {
                            st.req_len += n;
                        }
                    }
                }

                let mut req_complete = false;
                let mut is_ota_post = false;
                let mut is_snapshot_req = false;
                let peer_closed_rx = matches!(sock80.state(), TcpState::CloseWait);
                {
                    let req_slice: &[u8] = if st.using_ota_accum {
                        &http80_ota_accum[..st.req_len]
                    } else {
                        &p80_hdr_bufs[idx][..st.req_len]
                    };
                    if let Ok(parsed) = parse_http_request(req_slice) {
                        let needed_body = match parsed.content_length {
                            Some(cl) if cl > 0 => (cl as usize).min(OTA_PARTITION_MAX_SIZE + 4096),
                            _ => 0,
                        };
                        if parsed.body.len() >= needed_body || peer_closed_rx {
                            req_complete = true;
                            if parsed.method == HttpMethod::Post && parsed.path == "/update" {
                                is_ota_post = true;
                            }
                            if parsed.method == HttpMethod::Get
                                && (parsed.path == "/snapshot"
                                    || (parsed.path == "/"
                                        && parsed
                                            .query
                                            .map(|q| q.contains("action=snapshot"))
                                            .unwrap_or(false)))
                            {
                                is_snapshot_req = true;
                            }
                        }
                    } else if (!st.using_ota_accum && st.req_len >= p80_hdr_bufs[idx].len())
                        || peer_closed_rx
                    {
                        req_complete = true;
                    }
                }

                if req_complete {
                    if is_ota_post {
                        let _ = hw_i2s_cam.stop();
                    }
                    if is_snapshot_req {
                        post_snapshot_refresh_at_ms = Some(now_ms + 150);
                    }

                    let free_dram = esp_alloc::HEAP
                        .free_caps(esp_alloc::MemoryCapability::Internal.into())
                        as u32;
                    let rssi = wifi_controller.rssi().unwrap_or(-50);
                    runtime
                        .server
                        .set_telemetry(now_ms, free_dram, free_psram_bytes, rssi);

                    let req_slice: &[u8] = if st.using_ota_accum {
                        &http80_ota_accum[..st.req_len]
                    } else {
                        &p80_hdr_bufs[idx][..st.req_len]
                    };

                    // Note: GET /snapshot serves immediately from runtime.server.camera
                    // (backed by the active PingPongFrameBuffer slot) with zero VSYNC wait!
                    let handle_res = runtime
                        .server
                        .handle_raw_request_with_flash(
                            ServerPort::Control,
                            req_slice,
                            p80_resp_bufs[idx],
                            &mut flash_adapter,
                        )
                        .map(|s| s.len());

                    if st.using_ota_accum {
                        st.using_ota_accum = false;
                        if ota_accum_owner == Some(idx) {
                            ota_accum_owner = None;
                        }
                    }

                    match handle_res {
                        Ok(resp_len) => {
                            st.tx_len = resp_len;
                            st.tx_off = 0;
                            enqueue_tcp_sliding_window(
                                sock80,
                                p80_resp_bufs[idx],
                                &mut st.tx_off,
                                st.tx_len,
                            );
                        }
                        Err(_) => {
                            sock80.close();
                            st.closing = true;
                            st.close_deadline_ms = now_ms + 1000;
                        }
                    }
                }
            } else if matches!(sock80.state(), TcpState::CloseWait) && st.req_len == 0 {
                sock80.close();
                st.closing = true;
                st.close_deadline_ms = now_ms + 1000;
            }
        }

        // Apply any pending live camera control updates from GET /control?var=...&val=...
        // only after all active Port 80 HTTP responses have finished flushing.
        let p80_tx_active = p80_states.iter().any(|s| s.tx_len > 0 && !s.closing);
        if p80_tx_active {
            poll_net_burst(&mut iface, &mut net_device, &mut sockets);
        } else if let Some(ctrl) = runtime.server.take_pending_control_update() {
            if let (Some(ref mut sccb), Some(ref mut ov)) = (&mut kept_sccb, &mut live_ov2640) {
                if ctrl.framesize != last_applied_ctrl.framesize {
                    let _ = ov.set_framesize(sccb, ctrl.framesize);
                }
                if ctrl.quality != last_applied_ctrl.quality {
                    let _ = ov.set_quality(sccb, ctrl.quality);
                }
                if ctrl.brightness != last_applied_ctrl.brightness {
                    let _ = ov.set_brightness(sccb, ctrl.brightness);
                }
                if ctrl.contrast != last_applied_ctrl.contrast {
                    let _ = ov.set_contrast(sccb, ctrl.contrast);
                }
                if ctrl.saturation != last_applied_ctrl.saturation {
                    let _ = ov.set_saturation(sccb, ctrl.saturation);
                }
                if ctrl.special_effect != last_applied_ctrl.special_effect {
                    let _ = ov.set_special_effect(sccb, ctrl.special_effect);
                }
                if ctrl.hmirror != last_applied_ctrl.hmirror {
                    let _ = ov.set_hmirror(sccb, ctrl.hmirror);
                }
                if ctrl.vflip != last_applied_ctrl.vflip {
                    let _ = ov.set_vflip(sccb, ctrl.vflip);
                }
                if ctrl.awb != last_applied_ctrl.awb {
                    let _ = ov.set_whitebal(sccb, ctrl.awb);
                }
                if ctrl.agc != last_applied_ctrl.agc {
                    let _ = ov.set_gain_ctrl(sccb, ctrl.agc);
                }
                if ctrl.aec != last_applied_ctrl.aec {
                    let _ = ov.set_exposure_ctrl(sccb, ctrl.aec);
                }
                if ctrl.ae_level != last_applied_ctrl.ae_level {
                    let _ = ov.set_ae_level(sccb, ctrl.ae_level);
                }
            }
            last_applied_ctrl = ctrl;
            post_snapshot_refresh_at_ms = Some(now_ms + 150);
        }

        // Reboot if OTA firmware update completed and all Port 80 TX buffers have flushed
        if runtime.server.is_reboot_scheduled() {
            let all_flushed = tcp80_handles
                .iter()
                .all(|&h| sockets.get_mut::<TcpSocket>(h).send_queue() == 0);
            if all_flushed {
                let _ = iface.poll(smol_now(), &mut net_device, &mut sockets);
                delay.delay_millis(250);
                println!("[KLIPPER-CAM-RS] [OTA] Flash complete, rebooting into updated slot...");
                esp_hal::system::software_reset();
            }
        }

        // ============================================================
        // Port 81 2-Socket Symmetric MJPEG Stream Pool (/stream & /)
        // ============================================================
        if let Some(owner) = p81_stream_owner {
            let sock = sockets.get_mut::<TcpSocket>(tcp81_handles[owner]);
            let peer_disconnected = !sock.is_open()
                || !sock.is_active()
                || !sock.may_send()
                || !sock.may_recv()
                || matches!(
                    sock.state(),
                    TcpState::CloseWait
                        | TcpState::TimeWait
                        | TcpState::Closed
                        | TcpState::Closing
                        | TcpState::LastAck
                );
            if peer_disconnected {
                sock.abort();
                p81_states[owner] = Port81WorkerState::default();
                sock.set_timeout(Some(smoltcp::time::Duration::from_millis(10_000)));
                let _ = sock.listen(81);
                p81_stream_owner = None;
                p81_tx_off = 0;
                p81_tx_len = 0;
                runtime
                    .server
                    .stream_active_flag()
                    .store(false, Ordering::Release);
            }
        }

        for idx in 0..PORT81_SOCKET_COUNT {
            if p81_stream_owner == Some(idx) {
                let mut ready_for_next_frame = false;
                {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    if sock81.can_recv() {
                        let mut drain = [0u8; 128];
                        let _ = sock81.recv_slice(&mut drain);
                    }
                    if p81_tx_off < p81_tx_len {
                        enqueue_tcp_sliding_window(
                            sock81,
                            p81_resp_buf,
                            &mut p81_tx_off,
                            p81_tx_len,
                        );
                    } else if now_ms - p81_last_frame_ms >= 60
                        && sock81.can_send()
                        && sock81.send_queue() == 0
                    {
                        ready_for_next_frame = true;
                    }
                }
                if ready_for_next_frame {
                    p81_last_frame_ms = now_ms;
                    if now_ms - last_capture_ms >= 50
                        && ov2640_dma_ok
                        && ota_accum_owner.is_none()
                        && !runtime.server.is_reboot_scheduled()
                        && !p80_busy
                    {
                        last_capture_ms = now_ms;
                        if capture_live_hd_jpeg_with_diag(
                            &mut hw_i2s_cam,
                            jpeg_scratch_buf,
                            &mut ping_pong,
                            &mut || {
                                let _ = iface.poll(smol_now(), &mut net_device, &mut sockets);
                            },
                        )
                        .is_ok()
                        {
                            let fresh_slice = active_ping_pong_static_slice(&ping_pong);
                            runtime.server.camera.set_frame(
                                fresh_slice,
                                runtime.server.control_state.framesize,
                            );
                            last_capture_ms = smol_now().total_millis();
                        }
                    }
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    runtime
                        .server
                        .stream_active_flag()
                        .store(false, Ordering::Release);
                    if let Some(mut guard) =
                        StreamSessionGuard::try_acquire(runtime.server.stream_active_flag())
                    {
                        if let Ok(chunk) =
                            guard.write_next_frame_chunk(&runtime.server.camera, p81_resp_buf)
                        {
                            p81_tx_len = chunk.len();
                            p81_tx_off = 0;
                            enqueue_tcp_sliding_window(
                                sock81,
                                p81_resp_buf,
                                &mut p81_tx_off,
                                p81_tx_len,
                            );
                        }
                        drop(guard);
                    }
                    runtime
                        .server
                        .stream_active_flag()
                        .store(true, Ordering::Release);
                }
            } else {
                let mut preempt_and_start = false;
                {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    let st = &mut p81_states[idx];

                    if !sock81.is_open() {
                        *st = Port81WorkerState::default();
                        sock81.set_timeout(Some(smoltcp::time::Duration::from_millis(10_000)));
                        let _ = sock81.listen(81);
                    } else if st.closing {
                        let finished_cleanly = matches!(
                            sock81.state(),
                            TcpState::TimeWait | TcpState::Closed
                        );
                        if !sock81.is_active()
                            || finished_cleanly
                            || now_ms >= st.close_deadline_ms
                        {
                            sock81.abort();
                            *st = Port81WorkerState::default();
                            sock81.set_timeout(Some(smoltcp::time::Duration::from_millis(10_000)));
                            let _ = sock81.listen(81);
                        }
                    } else if sock81.can_recv() {
                        let avail = p81_req_bufs[idx].len().saturating_sub(st.req_len);
                        if avail > 0 {
                            if let Ok(n) = sock81.recv_slice(
                                &mut p81_req_bufs[idx][st.req_len..st.req_len + avail],
                            ) {
                                st.req_len += n;
                            }
                        }
                        if let Ok(parsed) = parse_http_request(&p81_req_bufs[idx][..st.req_len]) {
                            let is_stream_get = parsed.method == HttpMethod::Get
                                && (parsed.path == "/stream" || parsed.path == "/");
                            if is_stream_get {
                                preempt_and_start = true;
                            } else {
                                let mut err_buf = [0u8; 512];
                                if let Ok(resp) = runtime.server.handle_raw_request(
                                    ServerPort::Stream,
                                    &p81_req_bufs[idx][..st.req_len],
                                    &mut err_buf,
                                ) {
                                    let _ = sock81.send_slice(resp);
                                }
                                sock81.close();
                                st.closing = true;
                                st.close_deadline_ms = now_ms + 1000;
                            }
                        } else if st.req_len >= p81_req_bufs[idx].len()
                            || matches!(sock81.state(), TcpState::CloseWait)
                        {
                            sock81.close();
                            st.closing = true;
                            st.close_deadline_ms = now_ms + 1000;
                        }
                    } else if matches!(sock81.state(), TcpState::CloseWait) && st.req_len == 0 {
                        sock81.close();
                        st.closing = true;
                        st.close_deadline_ms = now_ms + 1000;
                    }
                }

                if preempt_and_start {
                    // Cleanly preempt any prior/stale Port 81 stream so browser Reloads or
                    // immediate reconnects succeed 100% of the time with zero stale locks.
                    if let Some(prev_owner) = p81_stream_owner {
                        if prev_owner != idx {
                            let prev_sock = sockets.get_mut::<TcpSocket>(tcp81_handles[prev_owner]);
                            prev_sock.abort();
                            p81_states[prev_owner] = Port81WorkerState::default();
                            prev_sock
                                .set_timeout(Some(smoltcp::time::Duration::from_millis(10_000)));
                            let _ = prev_sock.listen(81);
                        }
                    }
                    p81_stream_owner = None;
                    runtime
                        .server
                        .stream_active_flag()
                        .store(false, Ordering::Release);

                    let mut total_written = 0usize;
                    if let Some(mut guard) =
                        StreamSessionGuard::try_acquire(runtime.server.stream_active_flag())
                    {
                        if let Ok(hdr_bytes) = guard.write_stream_headers(p81_resp_buf) {
                            let hdr_len = hdr_bytes.len();
                            if let Ok(frame_bytes) = guard.write_next_frame_chunk(
                                &runtime.server.camera,
                                &mut p81_resp_buf[hdr_len..],
                            ) {
                                total_written = hdr_len + frame_bytes.len();
                            } else {
                                total_written = hdr_len;
                            }
                        }
                        drop(guard);
                    }

                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    if total_written > 0 {
                        runtime
                            .server
                            .stream_active_flag()
                            .store(true, Ordering::Release);
                        p81_stream_owner = Some(idx);
                        p81_states[idx] = Port81WorkerState::default();
                        p81_last_frame_ms = now_ms;
                        p81_tx_len = total_written;
                        p81_tx_off = 0;
                        enqueue_tcp_sliding_window(
                            sock81,
                            p81_resp_buf,
                            &mut p81_tx_off,
                            p81_tx_len,
                        );
                    } else {
                        sock81.close();
                        p81_states[idx].closing = true;
                        p81_states[idx].close_deadline_ms = now_ms + 1000;
                    }
                }
            }
        }

        poll_net_burst(&mut iface, &mut net_device, &mut sockets);

        if now_ms - last_heartbeat_ms >= 30_000 {
            last_heartbeat_ms = now_ms;
            println!(
                "[KLIPPER-CAM-RS] Heartbeat: uptime={}ms connected={} stream_active={} frame_seq={} last_jpeg={}B",
                now_ms,
                wifi_controller.is_connected(),
                p81_stream_owner.is_some(),
                ping_pong.frame_seq(),
                ping_pong.active_len()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_main_firmware_self_test() {
        assert!(run_firmware_self_test());
    }
}
