//! SC031GS VGA Global-Shutter Grayscale CMOS image sensor driver.
//!
//! Ported from `sensors/sc031gs.c` and `sensors/private_include/sc031gs_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const SC031GS_PID_HIGH_REG: u16 = 0x3107;
pub const SC031GS_PID_LOW_REG: u16 = 0x3108;
pub const SC031GS_MAX_FRAME_WIDTH: u16 = 640;
pub const SC031GS_MAX_FRAME_HEIGHT: u16 = 480;
pub const SC031GS_GAIN_CTRL_COARSE_REG: u16 = 0x3E08;
pub const SC031GS_GAIN_CTRL_FINE_REG: u16 = 0x3E09;

pub const SC031GS_OUTPUT_WINDOW_START_X_H_REG: u16 = 0x3212;
pub const SC031GS_OUTPUT_WINDOW_START_X_L_REG: u16 = 0x3213;
pub const SC031GS_OUTPUT_WINDOW_START_Y_H_REG: u16 = 0x3210;
pub const SC031GS_OUTPUT_WINDOW_START_Y_L_REG: u16 = 0x3211;
pub const SC031GS_OUTPUT_WINDOW_WIDTH_H_REG: u16 = 0x3208;
pub const SC031GS_OUTPUT_WINDOW_WIDTH_L_REG: u16 = 0x3209;
pub const SC031GS_OUTPUT_WINDOW_HIGH_H_REG: u16 = 0x320A;
pub const SC031GS_OUTPUT_WINDOW_HIGH_L_REG: u16 = 0x320B;
pub const SC031GS_LED_STROBE_ENABLE_REG: u16 = 0x3361;

pub const REG_NULL: u16 = 0xFFFF;
pub const REG_DELAY: u16 = 0x0000;

pub const SC031GS_RESET_REGS: &[Reg16] = &[
    Reg16::new(0x0103, 0x01),
    Reg16::new(REG_DELAY, 10),
    Reg16::new(REG_NULL, 0x00),
];

pub const SC031GS_200X200_INIT_REGS: &[Reg16] = &[
    Reg16::new(0x0100, 0x00),
    Reg16::new(0x36E9, 0x80),
    Reg16::new(0x36F9, 0x80),
    Reg16::new(0x300F, 0x0F),
    Reg16::new(0x3018, 0x1F),
    Reg16::new(0x3019, 0xFF),
    Reg16::new(0x301C, 0xB4),
    Reg16::new(0x301F, 0x7B),
    Reg16::new(0x3028, 0x82),
    Reg16::new(0x3200, 0x00),
    Reg16::new(0x3201, 0xDC),
    Reg16::new(0x3202, 0x00),
    Reg16::new(0x3203, 0x98),
    Reg16::new(0x3204, 0x01),
    Reg16::new(0x3205, 0xB3),
    Reg16::new(0x3206, 0x01),
    Reg16::new(0x3207, 0x67),
    Reg16::new(SC031GS_OUTPUT_WINDOW_WIDTH_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_WIDTH_L_REG, 0xC8),
    Reg16::new(SC031GS_OUTPUT_WINDOW_HIGH_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_HIGH_L_REG, 0xC8),
    Reg16::new(0x320C, 0x03),
    Reg16::new(0x320D, 0x6B),
    Reg16::new(0x320E, 0x01),
    Reg16::new(0x320F, 0x40),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_Y_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_Y_L_REG, 0x08),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_X_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_X_L_REG, 0x04),
    Reg16::new(0x3220, 0x10),
    Reg16::new(0x3223, 0x50),
    Reg16::new(0x3250, 0xF0),
    Reg16::new(0x3251, 0x02),
    Reg16::new(0x3252, 0x01),
    Reg16::new(0x3253, 0x3B),
    Reg16::new(0x3254, 0x02),
    Reg16::new(0x3255, 0x07),
    Reg16::new(0x3304, 0x48),
    Reg16::new(0x3306, 0x38),
    Reg16::new(0x3309, 0x50),
    Reg16::new(0x330B, 0xE0),
    Reg16::new(0x330C, 0x18),
    Reg16::new(0x330F, 0x20),
    Reg16::new(0x3310, 0x10),
    Reg16::new(0x3314, 0x70),
    Reg16::new(0x3315, 0x38),
    Reg16::new(0x3316, 0x68),
    Reg16::new(0x3317, 0x0D),
    Reg16::new(0x3329, 0x5C),
    Reg16::new(0x332D, 0x5C),
    Reg16::new(0x332F, 0x60),
    Reg16::new(0x3335, 0x64),
    Reg16::new(0x3344, 0x64),
    Reg16::new(0x335B, 0x80),
    Reg16::new(0x335F, 0x80),
    Reg16::new(0x3366, 0x06),
    Reg16::new(0x3385, 0x41),
    Reg16::new(0x3387, 0x49),
    Reg16::new(0x3389, 0x01),
    Reg16::new(0x33B1, 0x03),
    Reg16::new(0x33B2, 0x06),
    Reg16::new(0x3621, 0xA4),
    Reg16::new(0x3622, 0x05),
    Reg16::new(0x3624, 0x47),
    Reg16::new(0x3631, 0x48),
    Reg16::new(0x3633, 0x52),
    Reg16::new(0x3635, 0x18),
    Reg16::new(0x3636, 0x25),
    Reg16::new(0x3637, 0x89),
    Reg16::new(0x3638, 0x0F),
    Reg16::new(0x3639, 0x08),
    Reg16::new(0x363A, 0x00),
    Reg16::new(0x363B, 0x48),
    Reg16::new(0x363C, 0x06),
    Reg16::new(0x363E, 0xF8),
    Reg16::new(0x3640, 0x00),
    Reg16::new(0x3641, 0x01),
    Reg16::new(0x36EA, 0x39),
    Reg16::new(0x36EB, 0x1E),
    Reg16::new(0x36EC, 0x0E),
    Reg16::new(0x36ED, 0x23),
    Reg16::new(0x36FA, 0x39),
    Reg16::new(0x36FB, 0x10),
    Reg16::new(0x36FC, 0x01),
    Reg16::new(0x36FD, 0x03),
    Reg16::new(0x3908, 0x91),
    Reg16::new(0x3D08, 0x01),
    Reg16::new(0x3D04, 0x04),
    Reg16::new(0x3E01, 0x13),
    Reg16::new(0x3E02, 0xA0),
    Reg16::new(0x3E06, 0x0C),
    Reg16::new(0x3F04, 0x03),
    Reg16::new(0x3F05, 0x4B),
    Reg16::new(0x4500, 0x59),
    Reg16::new(0x4501, 0xC4),
    Reg16::new(0x4809, 0x01),
    Reg16::new(0x4837, 0x39),
    Reg16::new(0x5011, 0x00),
    Reg16::new(0x36E9, 0x04),
    Reg16::new(0x36F9, 0x04),
    Reg16::new(0x0100, 0x01),
    Reg16::new(REG_DELAY, 0x0A),
    Reg16::new(0x4418, 0x08),
    Reg16::new(0x4419, 0x80),
    Reg16::new(0x363D, 0x10),
    Reg16::new(0x3630, 0x48),
    Reg16::new(0x3317, 0x0D),
    Reg16::new(0x3314, 0x70),
    Reg16::new(0x3314, 0x68),
    Reg16::new(0x3317, 0x0E),
    Reg16::new(REG_NULL, 0x00),
];

pub const SC031GS_640X480_50FPS_INIT_REGS: &[Reg16] = &[
    Reg16::new(0x0100, 0x00),
    Reg16::new(0x36E9, 0x80),
    Reg16::new(0x36F9, 0x80),
    Reg16::new(0x300F, 0x0F),
    Reg16::new(0x3018, 0x1F),
    Reg16::new(0x3019, 0xFF),
    Reg16::new(0x301C, 0xB4),
    Reg16::new(0x301F, 0x6C),
    Reg16::new(0x3028, 0x82),
    Reg16::new(0x3200, 0x00),
    Reg16::new(0x3201, 0x00),
    Reg16::new(0x3202, 0x00),
    Reg16::new(0x3203, 0x08),
    Reg16::new(0x3204, 0x02),
    Reg16::new(0x3205, 0x8F),
    Reg16::new(0x3206, 0x01),
    Reg16::new(0x3207, 0xF7),
    Reg16::new(SC031GS_OUTPUT_WINDOW_WIDTH_H_REG, 0x02),
    Reg16::new(SC031GS_OUTPUT_WINDOW_WIDTH_L_REG, 0x80),
    Reg16::new(SC031GS_OUTPUT_WINDOW_HIGH_H_REG, 0x01),
    Reg16::new(SC031GS_OUTPUT_WINDOW_HIGH_L_REG, 0xE0),
    Reg16::new(0x320C, 0x03),
    Reg16::new(0x320D, 0x6E),
    Reg16::new(0x320E, 0x04),
    Reg16::new(0x320F, 0x72),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_Y_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_Y_L_REG, 0x08),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_X_H_REG, 0x00),
    Reg16::new(SC031GS_OUTPUT_WINDOW_START_X_L_REG, 0x08),
    Reg16::new(0x3220, 0x10),
    Reg16::new(0x3223, 0x50),
    Reg16::new(0x3250, 0xF0),
    Reg16::new(0x3251, 0x02),
    Reg16::new(0x3252, 0x03),
    Reg16::new(0x3253, 0xB0),
    Reg16::new(0x3254, 0x02),
    Reg16::new(0x3255, 0x07),
    Reg16::new(0x3304, 0x48),
    Reg16::new(0x3306, 0x38),
    Reg16::new(0x3309, 0x68),
    Reg16::new(0x330B, 0xE0),
    Reg16::new(0x330C, 0x18),
    Reg16::new(0x330F, 0x20),
    Reg16::new(0x3310, 0x10),
    Reg16::new(0x3314, 0x6D),
    Reg16::new(0x3315, 0x38),
    Reg16::new(0x3316, 0x68),
    Reg16::new(0x3317, 0x0F),
    Reg16::new(0x3329, 0x5C),
    Reg16::new(0x332D, 0x5C),
    Reg16::new(0x332F, 0x60),
    Reg16::new(0x3335, 0x64),
    Reg16::new(0x3344, 0x64),
    Reg16::new(0x335B, 0x80),
    Reg16::new(0x335F, 0x80),
    Reg16::new(0x3366, 0x06),
    Reg16::new(0x3385, 0x31),
    Reg16::new(0x3387, 0x51),
    Reg16::new(0x3389, 0x01),
    Reg16::new(0x33B1, 0x03),
    Reg16::new(0x33B2, 0x06),
    Reg16::new(0x3621, 0xA4),
    Reg16::new(0x3622, 0x05),
    Reg16::new(0x3624, 0x47),
    Reg16::new(0x3631, 0x48),
    Reg16::new(0x3633, 0x52),
    Reg16::new(0x3635, 0x18),
    Reg16::new(0x3636, 0x25),
    Reg16::new(0x3637, 0x89),
    Reg16::new(0x3638, 0x0F),
    Reg16::new(0x3639, 0x08),
    Reg16::new(0x363A, 0x00),
    Reg16::new(0x363B, 0x48),
    Reg16::new(0x363C, 0x06),
    Reg16::new(0x363E, 0xF8),
    Reg16::new(0x3640, 0x00),
    Reg16::new(0x3641, 0x01),
    Reg16::new(0x36EA, 0x36),
    Reg16::new(0x36EB, 0x1A),
    Reg16::new(0x36EC, 0x0A),
    Reg16::new(0x36ED, 0x23),
    Reg16::new(0x36FA, 0x36),
    Reg16::new(0x36FB, 0x10),
    Reg16::new(0x36FC, 0x01),
    Reg16::new(0x36FD, 0x03),
    Reg16::new(0x3908, 0x91),
    Reg16::new(0x3D08, 0x01),
    Reg16::new(0x3E01, 0x14),
    Reg16::new(0x3E02, 0x80),
    Reg16::new(0x3E06, 0x0C),
    Reg16::new(0x3F04, 0x03),
    Reg16::new(0x3F05, 0x4E),
    Reg16::new(0x4500, 0x59),
    Reg16::new(0x4501, 0xC4),
    Reg16::new(0x4809, 0x01),
    Reg16::new(0x4837, 0x1B),
    Reg16::new(0x5011, 0x00),
    Reg16::new(0x36E9, 0x20),
    Reg16::new(0x36F9, 0x24),
    Reg16::new(0x0100, 0x01),
    Reg16::new(REG_DELAY, 0x0A),
    Reg16::new(0x4418, 0x08),
    Reg16::new(0x4419, 0x80),
    Reg16::new(0x363D, 0x10),
    Reg16::new(0x3630, 0x48),
    Reg16::new(REG_NULL, 0x00),
];

#[inline]
fn set_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u16,
    offset: u8,
    length: u8,
    value: u8,
) -> Result<(), CameraError> {
    let current = bus.read_reg16(slv_addr, reg)?;
    let mask = (((1u16 << length) - 1) as u8) << offset;
    let next = (current & !mask) | ((value << offset) & mask);
    bus.write_reg16(slv_addr, reg, next)
}

fn write_reg_list<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    regs: &[Reg16],
) -> Result<(), CameraError> {
    for entry in regs {
        if entry.reg == REG_NULL {
            break;
        }
        if entry.reg == REG_DELAY {
            bus.delay_ms(entry.val as u32);
        } else {
            bus.write_reg16(slv_addr, entry.reg, entry.val)?;
        }
    }
    Ok(())
}

/// Detects an SC031GS sensor at `slv_addr` (`0x30`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Sc031Gs::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the SmartSens SC031GS global-shutter VGA camera sensor.
#[derive(Debug, Clone)]
pub struct Sc031Gs {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type SC031GS = Sc031Gs;
pub type Sc031gs = Sc031Gs;

impl Default for Sc031Gs {
    fn default() -> Self {
        Self::new()
    }
}

impl Sc031Gs {
    /// Creates a new [`Sc031Gs`] instance with default SCCB address (`0x30`) and PID (`0x0031`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::SC031GS.as_u16(),
                ver: 0,
            },
            status: CameraStatus {
                framesize: FrameSize::Vga,
                scale: false,
                binning: false,
                quality: 12,
                brightness: 0,
                contrast: 0,
                saturation: 0,
                sharpness: 0,
                denoise: 0,
                special_effect: 0,
                wb_mode: 0,
                awb: 1,
                awb_gain: 1,
                aec: 1,
                aec2: 0,
                ae_level: 0,
                aec_value: 300,
                agc: 1,
                agc_gain: 0,
                gainceiling: 0,
                bpc: 0,
                wpc: 1,
                raw_gma: 1,
                lenc: 1,
                hmirror: 0,
                vflip: 0,
                dcw: 1,
                colorbar: 0,
            },
            slv_addr: CameraSccbAddr::SC031GS,
            pixformat: PixelFormat::Grayscale,
            xclk_freq_hz: 20_000_000,
        }
    }

    /// Creates a new [`Sc031Gs`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x30`) and verifies `SC031GS_PID` (`0x0031`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::SC031GS {
            return Err(CameraError::NotDetected);
        }
        let midh = bus.read_reg16(slv_addr, SC031GS_PID_HIGH_REG)? as u16;
        let midl = bus.read_reg16(slv_addr, SC031GS_PID_LOW_REG)? as u16;
        let pid = (midh << 8) | midl;
        if pid == CameraPid::SC031GS.as_u16() {
            Ok(SensorId {
                midh: 0,
                midl: 0,
                pid,
                ver: 0,
            })
        } else {
            Err(CameraError::NotDetected)
        }
    }

    /// Returns the 7-bit SCCB slave address (`0x30`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the configured external clock frequency in Hz.
    pub const fn xclk_freq_hz(&self) -> u32 {
        self.xclk_freq_hz
    }

    /// Returns the [`CameraModel`] (`CameraModel::Sc031Gs`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Sc031Gs
    }

    /// Returns the detected [`SensorId`].
    pub const fn id(&self) -> &SensorId {
        &self.id
    }

    /// Returns the current [`CameraStatus`].
    pub const fn status(&self) -> &CameraStatus {
        &self.status
    }

    /// Returns a mutable reference to [`CameraStatus`].
    pub fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    fn set_output_window<B: SccbBus>(
        &mut self,
        bus: &mut B,
        offset_x: u16,
        offset_y: u16,
        w: u16,
        h: u16,
    ) -> Result<(), CameraError> {
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_START_X_H_REG as i32,
            0xFF,
            ((offset_x >> 8) & 0x03) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_START_X_L_REG as i32,
            0xFF,
            (offset_x & 0xFF) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_WIDTH_H_REG as i32,
            0xFF,
            ((w >> 8) & 0x03) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_WIDTH_L_REG as i32,
            0xFF,
            (w & 0xFF) as i32,
        )?;

        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_START_Y_H_REG as i32,
            0xFF,
            ((offset_y >> 8) & 0x03) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_START_Y_L_REG as i32,
            0xFF,
            (offset_y & 0xFF) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_HIGH_H_REG as i32,
            0xFF,
            ((h >> 8) & 0x03) as i32,
        )?;
        self.set_reg(
            bus,
            SC031GS_OUTPUT_WINDOW_HIGH_L_REG as i32,
            0xFF,
            (h & 0xFF) as i32,
        )?;

        bus.delay_ms(10);
        Ok(())
    }
}

impl<B: SccbBus> CameraSensor<B> for Sc031Gs {
    fn model(&self) -> CameraModel {
        CameraModel::Sc031Gs
    }

    fn id(&self) -> &SensorId {
        &self.id
    }

    fn status(&self) -> &CameraStatus {
        &self.status
    }

    fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError> {
        write_reg_list(bus, self.slv_addr, SC031GS_RESET_REGS)
    }

    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Ok(())
    }

    fn set_pixformat(&mut self, _bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Grayscale => {
                self.pixformat = pixformat;
                Ok(())
            }
            _ => Err(CameraError::NotSupported),
        }
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Vga {
            return Err(CameraError::FailedToSetFrameSize);
        }
        let w = framesize.width();
        let h = framesize.height();
        let (framesize_regs, offset_x, offset_y) = if w <= 200 && h <= 200 {
            (
                SC031GS_200X200_INIT_REGS,
                (200 - w) / 2 + 4,
                (200 - h) / 2 + 8,
            )
        } else {
            (
                SC031GS_640X480_50FPS_INIT_REGS,
                (SC031GS_MAX_FRAME_WIDTH - w) / 2 + 4,
                (SC031GS_MAX_FRAME_HEIGHT - h) / 2 + 4,
            )
        };

        write_reg_list(bus, self.slv_addr, framesize_regs)?;
        self.set_output_window(bus, offset_x, offset_y, w, h)?;
        self.status.framesize = framesize;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x4501, 3, 1, val)?;
        set_reg_bits(bus, self.slv_addr, 0x3902, 6, 1, val)?;
        set_reg_bits(bus, self.slv_addr, 0x3E06, 0, 2, if enable { 3 } else { 0 })?;
        self.status.colorbar = val;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bits = if enable { 0x03 } else { 0x00 };
        set_reg_bits(bus, self.slv_addr, 0x3221, 1, 2, bits)?;
        self.status.hmirror = u8::from(enable);
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bits = if enable { 0x03 } else { 0x00 };
        set_reg_bits(bus, self.slv_addr, 0x3221, 5, 2, bits)?;
        self.status.vflip = u8::from(enable);
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        let clamped_gain = gain.max(16) as u32;
        if clamped_gain < 0x20 {
            self.set_reg(bus, 0x3314, 0xFF, 0x3A)?;
            self.set_reg(bus, 0x3317, 0xFF, 0x20)?;
        } else {
            self.set_reg(bus, 0x3314, 0xFF, 0x44)?;
            self.set_reg(bus, 0x3317, 0xFF, 0x0F)?;
        }

        let (coarse_gain, fine_gain) = if clamped_gain < 0x20 {
            (0x03u32, clamped_gain - 16)
        } else if clamped_gain < 0x40 {
            (0x07u32, (clamped_gain >> 1) - 16)
        } else if clamped_gain < 0x80 {
            (0x0Fu32, (clamped_gain >> 2) - 16)
        } else {
            (0x1Fu32, (clamped_gain >> 3) - 16)
        };

        let fine_again_reg_v = (0x10 | (fine_gain & 0x0F)) as i32;
        let coarse_gain_reg_v = (coarse_gain & 0x1F) as i32;

        self.set_reg(
            bus,
            SC031GS_GAIN_CTRL_COARSE_REG as i32,
            0xFF,
            coarse_gain_reg_v,
        )?;
        self.set_reg(
            bus,
            SC031GS_GAIN_CTRL_FINE_REG as i32,
            0xFF,
            fine_again_reg_v,
        )?;
        self.status.agc_gain = (clamped_gain.min(0xFF)) as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        self.set_reg(bus, 0x3E01, 0xFF, (gain >> 8) & 0xFF)?;
        self.set_reg(bus, 0x3E02, 0xFF, gain & 0xFF)?;
        self.status.aec_value = (gain & 0xFFFF) as u16;
        Ok(())
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        let sleep_disable = u8::from((effect & 0x01) == 0);
        set_reg_bits(bus, self.slv_addr, 0x0100, 0, 1, sleep_disable)?;
        self.status.special_effect = (effect & 0xFF) as u8;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let val = bus.read_reg16(self.slv_addr, reg as u16)? as i32;
        Ok(val & mask)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let reg16 = reg as u16;
        let current = bus.read_reg16(self.slv_addr, reg16)? as i32;
        let next = ((current & !mask) | (value & mask)) as u8;
        bus.write_reg16(self.slv_addr, reg16, next)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
