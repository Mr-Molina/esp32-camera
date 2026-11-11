//! Golden SCCB burst, bank-isolation, and transaction-budget tests (`WP-22`).

use esp32_cam_sccb::{MockSccbBus, SccbBus, SccbTransaction};
use esp32_cam_sensors::ov2640::Ov2640;
use esp32_cam_sensors::ov5640::Ov5640Af;
use esp32_cam_sensors::{write_regs16, write_regs8, CameraSensor, Reg16, Reg8};
use esp32_cam_types::{CameraError, CameraSccbAddr, FrameSize, PixelFormat};

const AF_FW_LEN: usize = 4077;
const AF_FW_BASE: u16 = 0x8000;
const OV5640_CMD_FW_STATUS: u16 = 0x3029;
const OV5640_FW_STATUS_IDLE: u8 = 0x70;

struct AfReadyBus {
    inner: MockSccbBus<4200>,
    streamed: [u8; AF_FW_LEN],
    streamed_len: usize,
}

impl AfReadyBus {
    const fn new() -> Self {
        Self {
            inner: MockSccbBus::new(),
            streamed: [0u8; AF_FW_LEN],
            streamed_len: 0,
        }
    }
}

impl SccbBus for AfReadyBus {
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError> {
        self.inner.read_reg8(slv_addr, reg)
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        self.inner.write_reg8(slv_addr, reg, data)
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        if reg == OV5640_CMD_FW_STATUS {
            return Ok(OV5640_FW_STATUS_IDLE);
        }
        self.inner.read_reg16(slv_addr, reg)
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        self.inner.write_reg16(slv_addr, reg, data)
    }

    fn write_reg16_seq(
        &mut self,
        slv_addr: u8,
        start_reg: u16,
        data: &[u8],
    ) -> Result<(), CameraError> {
        if start_reg >= AF_FW_BASE {
            let base_off = usize::from(start_reg - AF_FW_BASE);
            if base_off + data.len() <= AF_FW_LEN {
                self.streamed[base_off..base_off + data.len()].copy_from_slice(data);
                self.streamed_len += data.len();
            }
        }
        self.inner.write_reg16_seq(slv_addr, start_reg, data)
    }

    fn delay_ms(&mut self, ms: u32) {
        self.inner.delay_ms(ms);
    }
}

#[test]
fn test_sens16_ov5640_af_firmware_uploads_in_256_byte_bursts() {
    let mut bus = AfReadyBus::new();
    let af = Ov5640Af::new(CameraSccbAddr::OV5640);

    assert_eq!(af.init(&mut bus, 50), Ok(()));
    assert_eq!(
        bus.streamed_len, AF_FW_LEN,
        "all 4077 AF firmware bytes must upload"
    );
    assert!(
        bus.streamed.iter().any(|&b| b != 0),
        "uploaded AF firmware payload must be non-zero"
    );

    let mut burst_lens = [0u16; 16];
    let mut burst_addrs = [0u16; 16];
    let mut burst_count = 0usize;
    for tx in bus.inner.transactions() {
        if let SccbTransaction::Write16Seq {
            slv_addr,
            start_reg,
            len,
        } = *tx
        {
            assert_eq!(slv_addr, CameraSccbAddr::OV5640);
            assert!(burst_count < 16, "expected at most 16 AF burst writes");
            burst_addrs[burst_count] = start_reg;
            burst_lens[burst_count] = len;
            burst_count += 1;
        }
    }
    assert_eq!(burst_count, 16, "4077 bytes in 256B chunks = 16 bursts");
    for idx in 0..15usize {
        assert_eq!(burst_addrs[idx], AF_FW_BASE + (idx as u16) * 256);
        assert_eq!(burst_lens[idx], 256);
    }
    assert_eq!(burst_addrs[15], AF_FW_BASE + 15 * 256);
    assert_eq!(burst_lens[15], 237);

    for (idx, &expected) in bus.streamed.iter().enumerate() {
        let reg = AF_FW_BASE + (idx as u16);
        assert_eq!(
            bus.inner.peek_reg16(CameraSccbAddr::OV5640, reg),
            Some(expected),
            "AF SRAM mismatch at 0x{reg:04X}"
        );
    }
}

#[test]
fn test_sccb06_and_sens17_ov2640_banks_and_golden_budgets() {
    let mut bus: MockSccbBus<512> =
        MockSccbBus::new().with_bank_select(CameraSccbAddr::OV2640, 0xFF);

    // SCCB-06: register 0x44 holds distinct values in DSP (0) vs Sensor (1).
    assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x00), Ok(()));
    assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0x44, 0x0C), Ok(()));
    assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x01), Ok(()));
    assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0x44, 0x3A), Ok(()));
    assert_eq!(bus.reg8_banked(CameraSccbAddr::OV2640, 0, 0x44), 0x0C);
    assert_eq!(bus.reg8_banked(CameraSccbAddr::OV2640, 1, 0x44), 0x3A);

    // SENS-17: fresh Ov2640 reset = 159 writes and 10 ms delay.
    let mut reset_bus: MockSccbBus<512> =
        MockSccbBus::new().with_bank_select(CameraSccbAddr::OV2640, 0xFF);
    let mut sensor = Ov2640::new();
    assert_eq!(sensor.reset(&mut reset_bus), Ok(()));
    assert_eq!(
        reset_bus.register_write_count(),
        159,
        "OV2640 reset must issue 159 register writes"
    );
    assert_eq!(
        reset_bus.total_delay_ms(),
        10,
        "OV2640 reset delay must be 10 ms"
    );
    assert_eq!(reset_bus.dropped_count(), 0);

    // Pin JPEG + HD set_framesize write budget.
    assert_eq!(
        sensor.set_pixformat(&mut reset_bus, PixelFormat::Jpeg),
        Ok(())
    );
    reset_bus.clear_transactions();
    let delay_before_fs = reset_bus.total_delay_ms();
    assert_eq!(sensor.set_framesize(&mut reset_bus, FrameSize::Hd), Ok(()));
    let hd_writes = reset_bus.register_write_count();
    let hd_delay = reset_bus.total_delay_ms() - delay_before_fs;
    assert!(
        (40..=75).contains(&hd_writes),
        "OV2640 HD set_framesize write budget out of range: {hd_writes}"
    );
    assert!(
        hd_delay <= 20,
        "OV2640 HD set_framesize delay budget exceeded: {hd_delay} ms"
    );
    assert_eq!(reset_bus.reg8_banked(CameraSccbAddr::OV2640, 1, 0x11), 0x00);
    assert_eq!(reset_bus.reg8_banked(CameraSccbAddr::OV2640, 0, 0xD3), 0x0C);

    // SENS-17: set_quality(12) in DSP bank is a single ordered Write8 to 0x44.
    reset_bus.clear_transactions();
    assert_eq!(sensor.set_quality(&mut reset_bus, 12), Ok(()));
    assert_eq!(reset_bus.register_write_count(), 1);
    reset_bus.assert_transactions(&[SccbTransaction::Write8 {
        slv_addr: CameraSccbAddr::OV2640,
        reg: 0x44,
        val: 12,
    }]);
    assert_eq!(reset_bus.reg8_banked(CameraSccbAddr::OV2640, 0, 0x44), 12);
}

#[test]
fn test_sccb02_table_writers_coalesce_with_identical_register_state() {
    let table16 = [
        Reg16::new(0x3000, 0x11),
        Reg16::new(0x3001, 0x22),
        Reg16::new(0x3002, 0x33),
        Reg16::new(0x3010, 0x44),
        Reg16::new(0x3800, 0x55),
        Reg16::new(0x3801, 0x66),
    ];

    let mut coalesced16: MockSccbBus<32> = MockSccbBus::new();
    let mut scalar16: MockSccbBus<32> = MockSccbBus::new();
    assert_eq!(
        write_regs16(&mut coalesced16, CameraSccbAddr::OV5640, &table16),
        Ok(())
    );
    for entry in table16 {
        assert_eq!(
            scalar16.write_reg16(CameraSccbAddr::OV5640, entry.reg, entry.val),
            Ok(())
        );
    }
    assert_eq!(coalesced16.register_write_count(), table16.len());
    coalesced16.assert_transactions(&[
        SccbTransaction::Write16Seq {
            slv_addr: CameraSccbAddr::OV5640,
            start_reg: 0x3000,
            len: 3,
        },
        SccbTransaction::Write16 {
            slv_addr: CameraSccbAddr::OV5640,
            reg: 0x3010,
            val: 0x44,
        },
        SccbTransaction::Write16Seq {
            slv_addr: CameraSccbAddr::OV5640,
            start_reg: 0x3800,
            len: 2,
        },
    ]);
    for entry in table16 {
        assert_eq!(
            coalesced16.peek_reg16(CameraSccbAddr::OV5640, entry.reg),
            scalar16.peek_reg16(CameraSccbAddr::OV5640, entry.reg)
        );
    }

    // Verify 0xFF bank-select is never absorbed into a coalesced 8-bit burst.
    let table8 = [
        Reg8::new(0xFE, 0xAA),
        Reg8::new(0xFF, 0x01),
        Reg8::new(0x11, 0x00),
        Reg8::new(0x12, 0x80),
    ];
    let mut coalesced8: MockSccbBus<32> =
        MockSccbBus::new().with_bank_select(CameraSccbAddr::OV2640, 0xFF);
    assert_eq!(
        write_regs8(&mut coalesced8, CameraSccbAddr::OV2640, &table8),
        Ok(())
    );
    coalesced8.assert_transactions(&[
        SccbTransaction::Write8 {
            slv_addr: CameraSccbAddr::OV2640,
            reg: 0xFE,
            val: 0xAA,
        },
        SccbTransaction::Write8 {
            slv_addr: CameraSccbAddr::OV2640,
            reg: 0xFF,
            val: 0x01,
        },
        SccbTransaction::Write8Seq {
            slv_addr: CameraSccbAddr::OV2640,
            start_reg: 0x11,
            len: 2,
        },
    ]);
    assert_eq!(
        coalesced8.reg8_banked(CameraSccbAddr::OV2640, 0, 0xFE),
        0xAA
    );
    assert_eq!(
        coalesced8.reg8_banked(CameraSccbAddr::OV2640, 1, 0x11),
        0x00
    );
    assert_eq!(
        coalesced8.reg8_banked(CameraSccbAddr::OV2640, 1, 0x12),
        0x80
    );
}
