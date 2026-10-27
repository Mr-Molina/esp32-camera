//! Safe `#![no_std]` OV7725 (VGA RGB565/YUV422/Grayscale) camera sensor driver.
//!
//! Ported from `sensors/ov7725.c` and `sensors/private_include/ov7725_regs.h`.

use crate::{
    write_regs8, CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus,
    FrameSize, GainCeiling, PixelFormat, Reg8, SccbBus, SensorId,
};

pub const GAIN: u8 = 0x00;
pub const BLUE: u8 = 0x01;
pub const RED: u8 = 0x02;
pub const GREEN: u8 = 0x03;
pub const BAVG: u8 = 0x05;
pub const GAVG: u8 = 0x06;
pub const RAVG: u8 = 0x07;
pub const AECH: u8 = 0x08;
pub const COM2: u8 = 0x09;
pub const REG_PID: u8 = 0x0A;
pub const REG_VER: u8 = 0x0B;
pub const COM3: u8 = 0x0C;
pub const COM4: u8 = 0x0D;
pub const COM5: u8 = 0x0E;
pub const COM6: u8 = 0x0F;
pub const AEC: u8 = 0x10;
pub const CLKRC: u8 = 0x11;
pub const COM7: u8 = 0x12;
pub const COM8: u8 = 0x13;
pub const COM9: u8 = 0x14;
pub const COM10: u8 = 0x15;
pub const REG16: u8 = 0x16;
pub const HSTART: u8 = 0x17;
pub const HSIZE: u8 = 0x18;
pub const VSTART: u8 = 0x19;
pub const VSIZE: u8 = 0x1A;
pub const PSHFT: u8 = 0x1B;
pub const REG_MIDH: u8 = 0x1C;
pub const REG_MIDL: u8 = 0x1D;
pub const LAEC: u8 = 0x1F;
pub const COM11: u8 = 0x20;
pub const BDBASE: u8 = 0x22;
pub const DBSTEP: u8 = 0x23;
pub const AEW: u8 = 0x24;
pub const AEB: u8 = 0x25;
pub const VPT: u8 = 0x26;
pub const REG28: u8 = 0x28;
pub const HOUTSIZE: u8 = 0x29;
pub const EXHCH: u8 = 0x2A;
pub const EXHCL: u8 = 0x2B;
pub const VOUTSIZE: u8 = 0x2C;
pub const ADVFL: u8 = 0x2D;
pub const ADVFH: u8 = 0x2E;
pub const YAVE: u8 = 0x2F;
pub const LUMHTH: u8 = 0x30;
pub const LUMLTH: u8 = 0x31;
pub const HREF: u8 = 0x32;
pub const DM_LNL: u8 = 0x33;
pub const DM_LNH: u8 = 0x34;
pub const ADOFF_B: u8 = 0x35;
pub const ADOFF_R: u8 = 0x36;
pub const ADOFF_GB: u8 = 0x37;
pub const ADOFF_GR: u8 = 0x38;
pub const OFF_B: u8 = 0x39;
pub const OFF_R: u8 = 0x3A;
pub const OFF_GB: u8 = 0x3B;
pub const OFF_GR: u8 = 0x3C;
pub const COM12: u8 = 0x3D;
pub const COM13: u8 = 0x3E;
pub const COM14: u8 = 0x3F;
pub const COM15: u8 = 0x40;
pub const COM16: u8 = 0x41;
pub const TGT_B: u8 = 0x42;
pub const TGT_R: u8 = 0x43;
pub const TGT_GB: u8 = 0x44;
pub const TGT_GR: u8 = 0x45;
pub const LC_CTR: u8 = 0x46;
pub const LC_XC: u8 = 0x47;
pub const LC_YC: u8 = 0x48;
pub const LC_COEF: u8 = 0x49;
pub const LC_RADI: u8 = 0x4A;
pub const LC_COEFB: u8 = 0x4B;
pub const LC_COEFR: u8 = 0x4C;
pub const FIXGAIN: u8 = 0x4D;
pub const AREF0: u8 = 0x4E;
pub const AREF1: u8 = 0x4F;
pub const AREF2: u8 = 0x50;
pub const AREF3: u8 = 0x51;
pub const AREF4: u8 = 0x52;
pub const AREF5: u8 = 0x53;
pub const AREF6: u8 = 0x54;
pub const AREF7: u8 = 0x55;
pub const UFIX: u8 = 0x60;
pub const VFIX: u8 = 0x61;
pub const AWBB_BLK: u8 = 0x62;
pub const AWB_CTRL0: u8 = 0x63;
pub const DSP_CTRL1: u8 = 0x64;
pub const DSP_CTRL2: u8 = 0x65;
pub const DSP_CTRL3: u8 = 0x66;
pub const DSP_CTRL4: u8 = 0x67;
pub const AWB_BIAS: u8 = 0x68;
pub const AWB_CTRL1: u8 = 0x69;
pub const AWB_CTRL2: u8 = 0x6A;
pub const AWB_CTRL3: u8 = 0x6B;
pub const GAM1: u8 = 0x7E;
pub const GAM2: u8 = 0x7F;
pub const GAM3: u8 = 0x80;
pub const GAM4: u8 = 0x81;
pub const GAM5: u8 = 0x82;
pub const GAM6: u8 = 0x83;
pub const GAM7: u8 = 0x84;
pub const GAM8: u8 = 0x85;
pub const GAM9: u8 = 0x86;
pub const GAM10: u8 = 0x87;
pub const GAM11: u8 = 0x88;
pub const GAM12: u8 = 0x89;
pub const GAM13: u8 = 0x8A;
pub const GAM14: u8 = 0x8B;
pub const GAM15: u8 = 0x8C;
pub const SLOP: u8 = 0x8D;
pub const DNSTH: u8 = 0x8E;
pub const EDGE0: u8 = 0x8F;
pub const EDGE1: u8 = 0x90;
pub const DNSOFF: u8 = 0x91;
pub const EDGE2: u8 = 0x92;
pub const EDGE3: u8 = 0x93;
pub const MTX1: u8 = 0x94;
pub const MTX2: u8 = 0x95;
pub const MTX3: u8 = 0x96;
pub const MTX4: u8 = 0x97;
pub const MTX5: u8 = 0x98;
pub const MTX6: u8 = 0x99;
pub const MTX_CTRL: u8 = 0x9A;
pub const BRIGHTNESS: u8 = 0x9B;
pub const CONTRAST: u8 = 0x9C;
pub const UVADJ0: u8 = 0x9E;
pub const UVADJ1: u8 = 0x9F;
pub const SCAL0: u8 = 0xA0;
pub const SCAL1: u8 = 0xA1;
pub const SCAL2: u8 = 0xA2;
pub const FIFODLYM: u8 = 0xA3;
pub const FIFODLYA: u8 = 0xA4;
pub const SDE: u8 = 0xA6;
pub const USAT: u8 = 0xA7;
pub const VSAT: u8 = 0xA8;
pub const HUECOS: u8 = 0xA9;
pub const HUESIN: u8 = 0xAA;
pub const SIGN_BIT: u8 = 0xAB;
pub const DSPAUTO: u8 = 0xAC;

pub const COM3_VFLIP: u8 = 0x80;
pub const COM3_MIRROR: u8 = 0x40;
pub const COM3_SWAP_BR: u8 = 0x20;
pub const COM3_SWAP_YUV: u8 = 0x10;
pub const COM3_SWAP_MSB: u8 = 0x08;
pub const COM3_COLOR_BAR: u8 = 0x01;

pub const COM7_RESET: u8 = 0x80;
pub const COM7_RES_VGA: u8 = 0x00;
pub const COM7_RES_QVGA: u8 = 0x40;
pub const COM7_FMT_RGB565: u8 = 0x04;
pub const COM7_FMT_YUV: u8 = 0x00;
pub const COM7_FMT_RGB: u8 = 0x02;

pub const COM10_PCLK_FREE: u8 = 0x00;
pub const COM10_VSYNC_NEG: u8 = 0x02;

pub const DSP_CTRL2_VDCW_EN: u8 = 0x08;
pub const DSP_CTRL2_HDCW_EN: u8 = 0x04;
pub const DSP_CTRL2_VZOOM_EN: u8 = 0x02;
pub const DSP_CTRL2_HZOOM_EN: u8 = 0x01;

pub const DSP_CTRL3_CBAR_EN: u8 = 0x20;

pub const SDE_CONT_BRIGHT_EN: u8 = 0x04;
pub const SDE_SATURATION_EN: u8 = 0x02;

pub const DEFAULT_REGS: &[Reg8] = &[
    Reg8::new(COM3, COM3_SWAP_YUV),
    Reg8::new(COM7, COM7_RES_QVGA | COM7_FMT_YUV),
    Reg8::new(COM4, 0x01),
    Reg8::new(CLKRC, 0x80 | 0x03),
    Reg8::new(HSTART, 0x3F),
    Reg8::new(HSIZE, 0x50),
    Reg8::new(VSTART, 0x03),
    Reg8::new(VSIZE, 0x78),
    Reg8::new(HREF, 0x00),
    Reg8::new(HOUTSIZE, 0x50),
    Reg8::new(VOUTSIZE, 0x78),
    Reg8::new(EXHCH, 0x00),
    Reg8::new(COM12, 0x03),
    Reg8::new(TGT_B, 0x7F),
    Reg8::new(FIXGAIN, 0x09),
    Reg8::new(AWB_CTRL0, 0xE0),
    Reg8::new(DSP_CTRL1, 0xFF),
    Reg8::new(
        DSP_CTRL2,
        DSP_CTRL2_VDCW_EN | DSP_CTRL2_HDCW_EN | DSP_CTRL2_HZOOM_EN | DSP_CTRL2_VZOOM_EN,
    ),
    Reg8::new(DSP_CTRL3, 0x00),
    Reg8::new(DSP_CTRL4, 0x00),
    Reg8::new(DSPAUTO, 0xFF),
    Reg8::new(COM8, 0xF0),
    Reg8::new(COM6, 0xC5),
    Reg8::new(COM9, 0x11),
    Reg8::new(COM10, COM10_VSYNC_NEG | COM10_PCLK_FREE),
    Reg8::new(BDBASE, 0x7F),
    Reg8::new(DBSTEP, 0x03),
    Reg8::new(AEW, 0x75),
    Reg8::new(AEB, 0x64),
    Reg8::new(VPT, 0xA1),
    Reg8::new(EXHCL, 0x00),
    Reg8::new(AWB_CTRL3, 0xAA),
    Reg8::new(COM8, 0xFF),
    Reg8::new(GAM1, 0x0C),
    Reg8::new(GAM2, 0x16),
    Reg8::new(GAM3, 0x2A),
    Reg8::new(GAM4, 0x4E),
    Reg8::new(GAM5, 0x61),
    Reg8::new(GAM6, 0x6F),
    Reg8::new(GAM7, 0x7B),
    Reg8::new(GAM8, 0x86),
    Reg8::new(GAM9, 0x8E),
    Reg8::new(GAM10, 0x97),
    Reg8::new(GAM11, 0xA4),
    Reg8::new(GAM12, 0xAF),
    Reg8::new(GAM13, 0xC5),
    Reg8::new(GAM14, 0xD7),
    Reg8::new(GAM15, 0xE8),
    Reg8::new(SLOP, 0x20),
    Reg8::new(EDGE1, 0x05),
    Reg8::new(EDGE2, 0x03),
    Reg8::new(EDGE3, 0x00),
    Reg8::new(DNSOFF, 0x01),
    Reg8::new(MTX1, 0xB0),
    Reg8::new(MTX2, 0x9D),
    Reg8::new(MTX3, 0x13),
    Reg8::new(MTX4, 0x16),
    Reg8::new(MTX5, 0x7B),
    Reg8::new(MTX6, 0x91),
    Reg8::new(MTX_CTRL, 0x1E),
    Reg8::new(BRIGHTNESS, 0x08),
    Reg8::new(CONTRAST, 0x30),
    Reg8::new(UVADJ0, 0x81),
    Reg8::new(SDE, SDE_CONT_BRIGHT_EN | SDE_SATURATION_EN),
    Reg8::new(DM_LNL, 0x00),
    Reg8::new(DM_LNH, 0x00),
    Reg8::new(BDBASE, 0x7F),
    Reg8::new(DBSTEP, 0x03),
    Reg8::new(LC_RADI, 0x10),
    Reg8::new(LC_COEF, 0x10),
    Reg8::new(LC_COEFB, 0x14),
    Reg8::new(LC_COEFR, 0x17),
    Reg8::new(LC_CTR, 0x05),
    Reg8::new(COM5, 0xF5),
];

/// Safe `#![no_std]` OV7725 sensor driver instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ov7725 {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

impl Default for Ov7725 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov7725 {
    /// Creates a new [`Ov7725`] instance with default SCCB address `0x21` and PID `0x77`.
    pub const fn new() -> Self {
        Self {
            slv_addr: CameraSccbAddr::OV7725,
            id: SensorId {
                midh: 0x7F,
                midl: 0xA2,
                pid: CameraPid::OV7725.as_u16(),
                ver: 0x21,
            },
            status: CameraStatus {
                framesize: FrameSize::Qvga,
                scale: false,
                binning: false,
                quality: 0,
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
                aec_value: 0,
                agc: 1,
                agc_gain: 0,
                gainceiling: 0,
                bpc: 0,
                wpc: 0,
                raw_gma: 0,
                lenc: 0,
                hmirror: 0,
                vflip: 0,
                dcw: 0,
                colorbar: 0,
            },
            pixformat: PixelFormat::Yuv422,
            xclk_freq_hz: 20_000_000,
        }
    }

    /// Returns the [`CameraModel`] enum discriminant for this sensor.
    #[inline]
    pub const fn model(&self) -> CameraModel {
        CameraModel::Ov7725
    }

    /// Returns a shared reference to the sensor's [`SensorId`].
    #[inline]
    pub const fn id(&self) -> &SensorId {
        &self.id
    }

    /// Returns a shared reference to the sensor's current [`CameraStatus`].
    #[inline]
    pub const fn status(&self) -> &CameraStatus {
        &self.status
    }

    /// Returns a mutable reference to the sensor's current [`CameraStatus`].
    #[inline]
    pub fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    /// Probes the SCCB bus at `slv_addr` (`0x21`) and verifies `PID == 0x77`.
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::OV7725 {
            return Err(CameraError::NotSupported);
        }
        bus.write_reg8(slv_addr, 0xFF, 0x01)?;
        let pid = bus.read_reg8(slv_addr, REG_PID)? as u16;
        if pid != CameraPid::OV7725.as_u16() {
            return Err(CameraError::NotDetected);
        }
        let ver = bus.read_reg8(slv_addr, REG_VER)?;
        let midl = bus.read_reg8(slv_addr, REG_MIDL)?;
        let midh = bus.read_reg8(slv_addr, REG_MIDH)?;
        Ok(SensorId {
            midh,
            midl,
            pid,
            ver,
        })
    }

    fn set_reg_bits<B: SccbBus>(
        &self,
        bus: &mut B,
        reg: u8,
        offset: u8,
        length: u8,
        value: u8,
    ) -> Result<(), CameraError> {
        let ret = bus.read_reg8(self.slv_addr, reg)?;
        let mask = (((1u16 << length) - 1) as u8) << offset;
        let next = (ret & !mask) | ((value << offset) & mask);
        bus.write_reg8(self.slv_addr, reg, next)
    }

    fn get_reg_bits<B: SccbBus>(
        &self,
        bus: &mut B,
        reg: u8,
        offset: u8,
        length: u8,
    ) -> Result<u8, CameraError> {
        let ret = bus.read_reg8(self.slv_addr, reg)?;
        let mask = (((1u16 << length) - 1) as u8) << offset;
        Ok((ret & mask) >> offset)
    }
}

impl<B: SccbBus> CameraSensor<B> for Ov7725 {
    fn model(&self) -> CameraModel {
        CameraModel::Ov7725
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
        bus.write_reg8(self.slv_addr, COM7, COM7_RESET)?;
        bus.delay_ms(10);
        write_regs8(bus, self.slv_addr, DEFAULT_REGS)?;
        bus.delay_ms(30);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = bus.read_reg8(self.slv_addr, BRIGHTNESS)? as i8;
        self.status.contrast = bus.read_reg8(self.slv_addr, CONTRAST)? as i8;
        self.status.saturation = 0;
        self.status.ae_level = 0;
        self.status.special_effect = self.get_reg_bits(bus, DSP_CTRL1, 5, 1)?;
        self.status.wb_mode = self.get_reg_bits(bus, AWB_CTRL3, 7, 1)?;
        self.status.agc_gain = bus.read_reg8(self.slv_addr, GAIN)?;
        let aec_l = bus.read_reg8(self.slv_addr, AEC)? as u16;
        let aec_h = bus.read_reg8(self.slv_addr, AECH)? as u16;
        self.status.aec_value = aec_l | (aec_h << 8);
        self.status.gainceiling = self.get_reg_bits(bus, COM9, 4, 3)?;
        self.status.awb = self.get_reg_bits(bus, COM8, 1, 1)?;
        self.status.awb_gain = self.get_reg_bits(bus, AWB_CTRL0, 7, 1)?;
        self.status.aec = self.get_reg_bits(bus, COM8, 0, 1)?;
        self.status.aec2 = self.get_reg_bits(bus, COM8, 7, 1)?;
        self.status.agc = self.get_reg_bits(bus, COM8, 2, 1)?;
        self.status.bpc = self.get_reg_bits(bus, DSP_CTRL1, 1, 1)?;
        self.status.wpc = self.get_reg_bits(bus, DSP_CTRL1, 0, 1)?;
        self.status.raw_gma = self.get_reg_bits(bus, DSP_CTRL1, 2, 1)?;
        self.status.lenc = self.get_reg_bits(bus, LC_CTR, 0, 1)?;
        self.status.hmirror = self.get_reg_bits(bus, COM3, 6, 1)?;
        self.status.vflip = self.get_reg_bits(bus, COM3, 7, 1)?;
        self.status.dcw = self.get_reg_bits(bus, DSP_CTRL2, 2, 1)?;
        self.status.colorbar = self.get_reg_bits(bus, COM3, 0, 1)?;
        self.status.sharpness = self.get_reg_bits(bus, EDGE0, 0, 5)? as i8;
        self.status.denoise = bus.read_reg8(self.slv_addr, DNSTH)?;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let mut reg = bus.read_reg8(self.slv_addr, COM7)?;
        match pixformat {
            PixelFormat::Rgb565 => {
                reg = (reg & 0xF0) | (COM7_FMT_RGB565 & 0x0C) | COM7_FMT_RGB;
            }
            PixelFormat::Yuv422 | PixelFormat::Grayscale => {
                reg = (reg & 0xF0) | (COM7_FMT_YUV & 0x03);
            }
            _ => return Err(CameraError::NotSupported),
        }
        bus.write_reg8(self.slv_addr, COM7, reg)?;
        self.pixformat = pixformat;
        bus.delay_ms(30);
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if (framesize as u8) > (FrameSize::Vga as u8) {
            return Err(CameraError::InvalidArg);
        }
        let res = framesize.resolution();
        let w = res.width;
        let h = res.height;
        let reg = bus.read_reg8(self.slv_addr, COM7)?;

        self.status.framesize = framesize;

        bus.write_reg8(self.slv_addr, HOUTSIZE, (w >> 2) as u8)?;
        bus.write_reg8(self.slv_addr, VOUTSIZE, (h >> 1) as u8)?;
        bus.write_reg8(self.slv_addr, HSIZE, (w >> 2) as u8)?;
        bus.write_reg8(self.slv_addr, VSIZE, (h >> 1) as u8)?;
        bus.write_reg8(
            self.slv_addr,
            EXHCH,
            ((w & 0x03) as u8) | (((h & 0x01) as u8) << 2),
        )?;

        if !(w > 320 || h > 240) {
            bus.write_reg8(self.slv_addr, DSPAUTO, 0xFF)?;
            bus.write_reg8(self.slv_addr, HSTART, 0x3F)?;
            bus.write_reg8(self.slv_addr, VSTART, 0x03)?;
            bus.write_reg8(self.slv_addr, COM7, reg | COM7_RES_QVGA)?;
            bus.write_reg8(self.slv_addr, CLKRC, 0x80 | 0x01)?;
        } else {
            bus.write_reg8(self.slv_addr, DSPAUTO, 0xF3)?;
            bus.write_reg8(self.slv_addr, SCAL0, 0x00)?;
            bus.write_reg8(self.slv_addr, SCAL1, 0x00)?;
            bus.write_reg8(self.slv_addr, SCAL2, 0x00)?;
            bus.write_reg8(self.slv_addr, HSTART, 0x23)?;
            bus.write_reg8(self.slv_addr, VSTART, 0x07)?;
            bus.write_reg8(self.slv_addr, COM7, reg & !COM7_RES_QVGA)?;
            bus.write_reg8(self.slv_addr, CLKRC, 0x80 | 0x03)?;
        }

        bus.delay_ms(30);
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, CONTRAST, (level & 0xFF) as u8)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, BRIGHTNESS, (level & 0xFF) as u8)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_gainceiling(
        &mut self,
        bus: &mut B,
        gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        match gainceiling {
            GainCeiling::Gain64x | GainCeiling::Gain128x => return Err(CameraError::NotSupported),
            _ => {}
        }
        let level = (gainceiling as u8) % 5;
        self.set_reg_bits(bus, COM9, 4, 3, level)?;
        self.status.gainceiling = gainceiling as u8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let com3 = (bus.read_reg8(self.slv_addr, COM3)? & 0xFE) | bit;
        bus.write_reg8(self.slv_addr, COM3, com3)?;

        let dsp3 = (bus.read_reg8(self.slv_addr, DSP_CTRL3)? & 0xDF) | (bit << 5);
        bus.write_reg8(self.slv_addr, DSP_CTRL3, dsp3)?;

        self.status.colorbar = bit;
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM8, 1, 1, bit)?;
        self.status.awb = bit;
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM8, 2, 1, bit)?;
        self.status.agc = bit;
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM8, 0, 1, bit)?;
        self.status.aec = bit;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM3, 6, 1, bit)?;
        self.status.hmirror = bit;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM3, 7, 1, bit)?;
        self.status.vflip = bit;
        Ok(())
    }

    fn set_aec2(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, COM8, 7, 1, bit)?;
        self.status.aec2 = bit;
        Ok(())
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, AWB_CTRL0, 7, 1, bit)?;
        self.status.awb_gain = bit;
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        if !(0..=0xFF).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        let g = (gain & 0xFF) as u8;
        bus.write_reg8(self.slv_addr, GAIN, g)?;
        self.status.agc_gain = g;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, value: i32) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&value) {
            return Err(CameraError::InvalidArg);
        }
        let val = value as u16;
        bus.write_reg8(self.slv_addr, AEC, (val & 0xFF) as u8)?;
        bus.write_reg8(self.slv_addr, AECH, (val >> 8) as u8)?;
        self.status.aec_value = val;
        Ok(())
    }

    fn set_dcw(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.set_reg_bits(bus, DSP_CTRL2, 2, 1, u8::from(!enable))?;
        self.status.dcw = u8::from(enable);
        Ok(())
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, DSP_CTRL1, 1, 1, bit)?;
        self.status.bpc = bit;
        Ok(())
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, DSP_CTRL1, 0, 1, bit)?;
        self.status.wpc = bit;
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, DSP_CTRL1, 2, 1, bit)?;
        self.status.raw_gma = bit;
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        self.set_reg_bits(bus, LC_CTR, 0, 1, bit)?;
        self.status.lenc = bit;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let mut ret = bus.read_reg8(self.slv_addr, (reg & 0xFF) as u8)? as i32;
        if ret > 0 {
            ret &= mask;
        }
        Ok(ret)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0xFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let addr = (reg & 0xFF) as u8;
        let current = bus.read_reg8(self.slv_addr, addr)? as i32;
        let next = ((current & !mask) | (value & mask)) as u8;
        bus.write_reg8(self.slv_addr, addr, next)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
