//! Safe `#![no_std]` OV7670 (VGA/QVGA/QQVGA RGB565/YUV422) camera sensor driver.
//!
//! Ported from `sensors/ov7670.c` and `sensors/private_include/ov7670_regs.h`.

use crate::{
    write_regs8, CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus,
    FrameSize, PixelFormat, Reg8, SccbBus, SensorId,
};

pub const GAIN: u8 = 0x00;
pub const BLUE: u8 = 0x01;
pub const RED: u8 = 0x02;
pub const VREF: u8 = 0x03;
pub const COM1: u8 = 0x04;
pub const BAVG: u8 = 0x05;
pub const GAVG: u8 = 0x06;
pub const AECH: u8 = 0x07;
pub const RAVG: u8 = 0x08;
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
pub const RSVD_16: u8 = 0x16;
pub const HSTART: u8 = 0x17;
pub const HSTOP: u8 = 0x18;
pub const VSTART: u8 = 0x19;
pub const VSTOP: u8 = 0x1A;
pub const PSHFT: u8 = 0x1B;
pub const REG_MIDH: u8 = 0x1C;
pub const REG_MIDL: u8 = 0x1D;
pub const MVFP: u8 = 0x1E;
pub const LAEC: u8 = 0x1F;
pub const ADCCTR0: u8 = 0x20;
pub const ADCCTR1: u8 = 0x21;
pub const ADCCTR2: u8 = 0x22;
pub const ADCCTR3: u8 = 0x23;
pub const AEW: u8 = 0x24;
pub const AEB: u8 = 0x25;
pub const VPT: u8 = 0x26;
pub const BBIAS: u8 = 0x27;
pub const GBBIAS: u8 = 0x28;
pub const RSVD_29: u8 = 0x29;
pub const EXHCH: u8 = 0x2A;
pub const EXHCL: u8 = 0x2B;
pub const RBIAS: u8 = 0x2C;
pub const ADVFL: u8 = 0x2D;
pub const ADVFH: u8 = 0x2E;
pub const YAVE: u8 = 0x2F;
pub const HSYST: u8 = 0x30;
pub const HSYEN: u8 = 0x31;
pub const HREF: u8 = 0x32;
pub const CHLF: u8 = 0x33;
pub const ARBLM: u8 = 0x34;
pub const ADC: u8 = 0x37;
pub const ACOM: u8 = 0x38;
pub const OFON: u8 = 0x39;
pub const TSLB: u8 = 0x3A;
pub const COM11: u8 = 0x3B;
pub const COM12: u8 = 0x3C;
pub const COM13: u8 = 0x3D;
pub const COM14: u8 = 0x3E;
pub const EDGE: u8 = 0x3F;
pub const COM15: u8 = 0x40;
pub const COM16: u8 = 0x41;
pub const COM17: u8 = 0x42;
pub const AWBC1: u8 = 0x43;
pub const AWBC2: u8 = 0x44;
pub const AWBC3: u8 = 0x45;
pub const AWBC4: u8 = 0x46;
pub const AWBC5: u8 = 0x47;
pub const AWBC6: u8 = 0x48;
pub const REG4B: u8 = 0x4B;
pub const DNSTH: u8 = 0x4C;
pub const MTX1: u8 = 0x4F;
pub const MTX2: u8 = 0x50;
pub const MTX3: u8 = 0x51;
pub const MTX4: u8 = 0x52;
pub const MTX5: u8 = 0x53;
pub const MTX6: u8 = 0x54;
pub const BRIGHTNESS: u8 = 0x55;
pub const CONTRAST: u8 = 0x56;
pub const CONTRASCENTER: u8 = 0x57;
pub const MTXS: u8 = 0x58;
pub const LCC1: u8 = 0x62;
pub const LCC2: u8 = 0x63;
pub const LCC3: u8 = 0x64;
pub const LCC4: u8 = 0x65;
pub const LCC5: u8 = 0x66;
pub const MANU: u8 = 0x67;
pub const MANV: u8 = 0x68;
pub const GFIX: u8 = 0x69;
pub const GGAIN: u8 = 0x6A;
pub const DBLV: u8 = 0x6B;
pub const AWBCTR3: u8 = 0x6C;
pub const AWBCTR2: u8 = 0x6D;
pub const AWBCTR1: u8 = 0x6E;
pub const AWBCTR0: u8 = 0x6F;
pub const SCALING_XSC: u8 = 0x70;
pub const SCALING_YSC: u8 = 0x71;
pub const SCALING_DCWCTR: u8 = 0x72;
pub const SCALING_PCLK_DIV: u8 = 0x73;
pub const REG74: u8 = 0x74;
pub const REG75: u8 = 0x75;
pub const REG76: u8 = 0x76;
pub const REG77: u8 = 0x77;
pub const SLOP: u8 = 0x7A;
pub const GAM1: u8 = 0x7B;
pub const GAM2: u8 = 0x7C;
pub const GAM3: u8 = 0x7D;
pub const GAM4: u8 = 0x7E;
pub const GAM5: u8 = 0x7F;
pub const GAM6: u8 = 0x80;
pub const GAM7: u8 = 0x81;
pub const GAM8: u8 = 0x82;
pub const GAM9: u8 = 0x83;
pub const GAM10: u8 = 0x84;
pub const GAM11: u8 = 0x85;
pub const GAM12: u8 = 0x86;
pub const GAM13: u8 = 0x87;
pub const GAM14: u8 = 0x88;
pub const GAM15: u8 = 0x89;
pub const RGB444: u8 = 0x8C;
pub const DM_LNL: u8 = 0x92;
pub const DM_LNH: u8 = 0x93;
pub const LCC6: u8 = 0x94;
pub const LCC7: u8 = 0x95;
pub const BD50ST: u8 = 0x9D;
pub const BD60ST: u8 = 0x9E;
pub const HAECC1: u8 = 0x9F;
pub const HAECC2: u8 = 0xA0;
pub const SCALING_PCLK_DELAY: u8 = 0xA2;
pub const NT_CNTRL: u8 = 0xA4;
pub const BD50MAX: u8 = 0xA5;
pub const HAECC3: u8 = 0xA6;
pub const HAECC4: u8 = 0xA7;
pub const HAECC5: u8 = 0xA8;
pub const HAECC6: u8 = 0xA9;
pub const HAECC7: u8 = 0xAA;
pub const BD60MAX: u8 = 0xAB;
pub const STR_OPT: u8 = 0xAC;
pub const STR_R: u8 = 0xAD;
pub const STR_G: u8 = 0xAE;
pub const STR_B: u8 = 0xAF;
pub const RSVD_B0: u8 = 0xB0;
pub const ABLC1: u8 = 0xB1;
pub const THL_ST: u8 = 0xB3;
pub const THL_DLT: u8 = 0xB5;
pub const AD_CHB: u8 = 0xBE;
pub const AD_CHR: u8 = 0xBF;
pub const AD_CHGB: u8 = 0xC0;
pub const AD_CHGR: u8 = 0xC1;
pub const SATCTR: u8 = 0xC9;

pub const COM7_RESET: u8 = 0x80;
pub const COM7_FMT_RGB565: u8 = 0x04;

pub const COM8_FAST_AUTO: u8 = 0x80;
pub const COM8_STEP_UNLIMIT: u8 = 0x40;
pub const COM8_AGC_EN: u8 = 0x04;
pub const COM8_AWB_EN: u8 = 0x02;
pub const COM8_AEC_EN: u8 = 0x01;

pub const COM10_PCLK_FREE: u8 = 0x00;
pub const COM10_VSYNC_NEG: u8 = 0x02;

pub const MVFP_MIRROR: u8 = 0x20;
pub const MVFP_FLIP: u8 = 0x10;
pub const MVFP_SUN: u8 = 0x02;

pub const COM11_EXP: u8 = 0x02;
pub const COM11_HZAUTO: u8 = 0x10;

pub const COM13_GAMMA: u8 = 0x80;
pub const COM13_UVSAT: u8 = 0x40;

pub const COM15_RGB565: u8 = 0x10;
pub const COM15_R00FF: u8 = 0xC0;

pub const OV7670_DEFAULT_REGS: &[Reg8] = &[
    Reg8::new(TSLB, 0x04),
    Reg8::new(CLKRC, 0x00),
    Reg8::new(DBLV, 0x4A),
    Reg8::new(COM10, COM10_VSYNC_NEG | COM10_PCLK_FREE),
    Reg8::new(COM4, 0x40),
    Reg8::new(RSVD_B0, 0x84),
    Reg8::new(COM11, COM11_EXP | COM11_HZAUTO),
    Reg8::new(HSYST, 0x00),
    Reg8::new(HSYEN, 0x00),
    Reg8::new(MVFP, MVFP_SUN),
    Reg8::new(AWBC1, 0x0A),
    Reg8::new(AWBC2, 0xF0),
    Reg8::new(AWBC3, 0x34),
    Reg8::new(AWBC4, 0x58),
    Reg8::new(AWBC5, 0x28),
    Reg8::new(AWBC6, 0x3A),
    Reg8::new(AWBCTR3, 0x0A),
    Reg8::new(AWBCTR2, 0x55),
    Reg8::new(AWBCTR1, 0x11),
    Reg8::new(AWBCTR0, 0x9E),
    Reg8::new(
        COM8,
        COM8_FAST_AUTO | COM8_STEP_UNLIMIT | COM8_AGC_EN | COM8_AEC_EN | COM8_AWB_EN,
    ),
];

pub const OV7670_FMT_YUV422: &[Reg8] = &[
    Reg8::new(COM7, 0x00),
    Reg8::new(RGB444, 0x00),
    Reg8::new(COM1, 0x00),
    Reg8::new(COM15, COM15_R00FF),
    Reg8::new(MVFP, MVFP_SUN),
    Reg8::new(COM9, 0x6A),
    Reg8::new(MTX1, 0x80),
    Reg8::new(MTX2, 0x80),
    Reg8::new(MTX3, 0x00),
    Reg8::new(MTX4, 0x22),
    Reg8::new(MTX5, 0x5E),
    Reg8::new(MTX6, 0x80),
    Reg8::new(COM13, COM13_UVSAT),
];

pub const OV7670_FMT_RGB565: &[Reg8] = &[
    Reg8::new(COM7, COM7_FMT_RGB565),
    Reg8::new(RGB444, 0x00),
    Reg8::new(COM1, 0x00),
    Reg8::new(COM15, COM15_RGB565 | COM15_R00FF),
    Reg8::new(MVFP, MVFP_SUN),
    Reg8::new(COM9, 0x6A),
    Reg8::new(MTX1, 0xB3),
    Reg8::new(MTX2, 0xB3),
    Reg8::new(MTX3, 0x00),
    Reg8::new(MTX4, 0x3D),
    Reg8::new(MTX5, 0xA7),
    Reg8::new(MTX6, 0xE4),
    Reg8::new(COM13, COM13_UVSAT),
];

pub const OV7670_VGA: &[Reg8] = &[
    Reg8::new(COM3, 0x00),
    Reg8::new(COM14, 0x00),
    Reg8::new(SCALING_XSC, 0x3A),
    Reg8::new(SCALING_YSC, 0x35),
    Reg8::new(SCALING_DCWCTR, 0x11),
    Reg8::new(SCALING_PCLK_DIV, 0xF0),
    Reg8::new(SCALING_PCLK_DELAY, 0x02),
];

pub const OV7670_QVGA: &[Reg8] = &[
    Reg8::new(COM3, 0x04),
    Reg8::new(COM14, 0x19),
    Reg8::new(SCALING_XSC, 0x3A),
    Reg8::new(SCALING_YSC, 0x35),
    Reg8::new(SCALING_DCWCTR, 0x11),
    Reg8::new(SCALING_PCLK_DIV, 0xF1),
    Reg8::new(SCALING_PCLK_DELAY, 0x02),
];

pub const OV7670_QQVGA: &[Reg8] = &[
    Reg8::new(COM3, 0x04),
    Reg8::new(COM14, 0x1A),
    Reg8::new(SCALING_XSC, 0x3A),
    Reg8::new(SCALING_YSC, 0x35),
    Reg8::new(SCALING_DCWCTR, 0x22),
    Reg8::new(SCALING_PCLK_DIV, 0xF2),
    Reg8::new(SCALING_PCLK_DELAY, 0x02),
];

/// Safe `#![no_std]` OV7670 sensor driver instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ov7670 {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
    clkrc: u8,
}

impl Default for Ov7670 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov7670 {
    /// Creates a new [`Ov7670`] instance with default SCCB address `0x21` and PID `0x76`.
    pub const fn new() -> Self {
        Self {
            slv_addr: CameraSccbAddr::OV7670,
            id: SensorId {
                midh: 0x7F,
                midl: 0xA2,
                pid: CameraPid::OV7670.as_u16(),
                ver: 0x73,
            },
            status: CameraStatus {
                framesize: FrameSize::Vga,
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
                awb: 0,
                awb_gain: 0,
                aec: 0,
                aec2: 0,
                ae_level: 0,
                aec_value: 0,
                agc: 0,
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
            clkrc: 0x01,
        }
    }

    /// Returns the [`CameraModel`] enum discriminant for this sensor.
    #[inline]
    pub const fn model(&self) -> CameraModel {
        CameraModel::Ov7670
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

    /// Probes the SCCB bus at `slv_addr` (`0x21`) and verifies `PID == 0x76`.
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::OV7670 {
            return Err(CameraError::NotSupported);
        }
        bus.write_reg8(slv_addr, 0xFF, 0x01)?;
        let pid = bus.read_reg8(slv_addr, REG_PID)? as u16;
        if pid != CameraPid::OV7670.as_u16() {
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

    /// Returns the cached `CLKRC` register value retained across resolution/format transitions.
    #[inline]
    pub const fn clkrc(&self) -> u8 {
        self.clkrc
    }

    fn frame_control<B: SccbBus>(
        &self,
        bus: &mut B,
        hstart: u16,
        hstop: u16,
        vstart: u16,
        vstop: u16,
    ) -> Result<(), CameraError> {
        let frame = [
            Reg8::new(HSTART, (hstart >> 3) as u8),
            Reg8::new(HSTOP, (hstop >> 3) as u8),
            Reg8::new(HREF, (((hstop & 0x07) << 3) | (hstart & 0x07)) as u8),
            Reg8::new(VSTART, (vstart >> 2) as u8),
            Reg8::new(VSTOP, (vstop >> 2) as u8),
            Reg8::new(VREF, (((vstop & 0x03) << 2) | (vstart & 0x03)) as u8),
        ];
        write_regs8(bus, self.slv_addr, &frame)
    }
}

impl<B: SccbBus> CameraSensor<B> for Ov7670 {
    fn model(&self) -> CameraModel {
        CameraModel::Ov7670
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
        write_regs8(bus, self.slv_addr, OV7670_DEFAULT_REGS)?;
        bus.delay_ms(30);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        let com8 = bus.read_reg8(self.slv_addr, COM8)?;
        self.status.aec = com8 & 0x01;
        self.status.awb = (com8 >> 1) & 0x01;
        self.status.agc = (com8 >> 2) & 0x01;

        let mvfp = bus.read_reg8(self.slv_addr, MVFP)?;
        self.status.vflip = (mvfp >> 4) & 0x01;
        self.status.hmirror = (mvfp >> 5) & 0x01;

        let ysc = bus.read_reg8(self.slv_addr, SCALING_YSC)?;
        self.status.colorbar = (ysc >> 7) & 0x01;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let mvfp_old = bus.read_reg8(self.slv_addr, MVFP)?;
        match pixformat {
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => {
                write_regs8(bus, self.slv_addr, OV7670_FMT_RGB565)?;
            }
            PixelFormat::Yuv422 | PixelFormat::Grayscale => {
                write_regs8(bus, self.slv_addr, OV7670_FMT_YUV422)?;
            }
            _ => return Err(CameraError::NotSupported),
        }
        let mvfp_new = (bus.read_reg8(self.slv_addr, MVFP)? & !0x30) | (mvfp_old & 0x30);
        bus.write_reg8(self.slv_addr, MVFP, mvfp_new)?;

        bus.delay_ms(30);
        if pixformat == PixelFormat::Rgb565 {
            bus.write_reg8(self.slv_addr, CLKRC, self.clkrc)?;
        }
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Vga {
            return Err(CameraError::FailedToSetFrameSize);
        }
        self.clkrc = bus.read_reg8(self.slv_addr, CLKRC)?;

        match framesize {
            FrameSize::Vga => {
                write_regs8(bus, self.slv_addr, OV7670_VGA)?;
                self.frame_control(bus, 158, 14, 10, 490)?;
            }
            FrameSize::Qvga => {
                write_regs8(bus, self.slv_addr, OV7670_QVGA)?;
                self.frame_control(bus, 158, 14, 10, 490)?;
            }
            FrameSize::Qqvga => {
                write_regs8(bus, self.slv_addr, OV7670_QQVGA)?;
                self.frame_control(bus, 158, 14, 12, 490)?;
            }
            _ => return Err(CameraError::InvalidArg),
        }

        bus.delay_ms(30);
        self.status.framesize = framesize;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let xsc = bus.read_reg8(self.slv_addr, SCALING_XSC)? & 0x7F;
        bus.write_reg8(self.slv_addr, SCALING_XSC, xsc)?;

        let bit = u8::from(enable);
        let ysc = (bus.read_reg8(self.slv_addr, SCALING_YSC)? & 0x7F) | (bit << 7);
        bus.write_reg8(self.slv_addr, SCALING_YSC, ysc)?;

        self.status.colorbar = bit;
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let reg = (bus.read_reg8(self.slv_addr, COM8)? & 0xFD) | (bit << 1);
        bus.write_reg8(self.slv_addr, COM8, reg)?;
        self.status.awb = bit;
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let reg = (bus.read_reg8(self.slv_addr, COM8)? & 0xFB) | (bit << 2);
        bus.write_reg8(self.slv_addr, COM8, reg)?;
        self.status.agc = bit;
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let reg = (bus.read_reg8(self.slv_addr, COM8)? & 0xFE) | bit;
        bus.write_reg8(self.slv_addr, COM8, reg)?;
        self.status.aec = bit;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let reg = (bus.read_reg8(self.slv_addr, MVFP)? & 0xDF) | (bit << 5);
        bus.write_reg8(self.slv_addr, MVFP, reg)?;
        self.status.hmirror = bit;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bit = u8::from(enable);
        let reg = (bus.read_reg8(self.slv_addr, MVFP)? & 0xEF) | (bit << 4);
        bus.write_reg8(self.slv_addr, MVFP, reg)?;
        self.status.vflip = bit;
        Ok(())
    }
}
