//! Control implementations shared by the 16-bit-addressed OmniVision drivers
//! (OV3640, OV3660, OV5640).
//!
//! SENS-23: these bodies were copied verbatim between the drivers, down to
//! the register constants — every name they share holds the same value in
//! each module, which is what makes one implementation correct for all of
//! them. Each driver's trait method now delegates here.
//!
//! Only the self-contained controls live here. `init_status`, `set_res_raw`
//! and `set_pll` are still per-driver: they call helpers (`set_image_options`,
//! `configure_pll`) whose bodies genuinely differ.

use crate::{CameraError, CameraStatus, GainCeiling, SccbBus};

/// Y_TOTAL_SIZE_H, the frame-length register `set_aec_value` clamps against.
const Y_TOTAL_SIZE_H: u16 = 0x380E;

/// Sets the automatic-gain ceiling.
///
/// # Errors
///
/// Any error reported by the bus.
pub fn set_gainceiling<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    gainceiling: GainCeiling,
) -> Result<(), CameraError> {
    let level = gainceiling as u8;
    let mut reg_val = 16u16 << (level + 1);
    if reg_val > 0x03FF {
        reg_val = 0x03FF;
    }
    bus.write_reg16(slv_addr, 0x3A18, ((reg_val >> 8) & 0x03) as u8)?;
    bus.write_reg16(slv_addr, 0x3A19, (reg_val & 0xFF) as u8)?;
    status.gainceiling = level;
    Ok(())
}

/// Sets the manual exposure value, clamped to the current frame length.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `0..=0xFFFF`, else any bus error.
pub fn set_aec_value<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    value: i32,
) -> Result<(), CameraError> {
    if !(0..=0xFFFF).contains(&value) {
        return Err(CameraError::InvalidArg);
    }
    let max_val = bus.read_reg16_val16(slv_addr, Y_TOTAL_SIZE_H)?;
    let mut val = value as u16;
    if max_val > 0 && val > max_val {
        val = max_val;
    }
    bus.write_reg16(slv_addr, 0x3500, ((val >> 12) & 0x0F) as u8)?;
    bus.write_reg16(slv_addr, 0x3501, ((val >> 4) & 0xFF) as u8)?;
    bus.write_reg16(slv_addr, 0x3502, ((val << 4) & 0xF0) as u8)?;
    status.aec_value = val;
    Ok(())
}

/// Sets the manual analogue gain.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `0..=64`, else any bus error.
pub fn set_agc_gain<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    gain: i32,
) -> Result<(), CameraError> {
    if !(0..=64).contains(&gain) {
        return Err(CameraError::InvalidArg);
    }
    let gainv = ((gain as u16) << 4).saturating_sub(1);
    bus.write_reg16(slv_addr, 0x350A, (gainv >> 8) as u8)?;
    bus.write_reg16(slv_addr, 0x350B, (gainv & 0xFF) as u8)?;
    status.agc_gain = gain as u8;
    Ok(())
}

/// Sets the denoise strength, disabling the block at level 0.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `0..=8`, else any bus error.
pub fn set_denoise<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    level: i32,
) -> Result<(), CameraError> {
    if !(0..=8).contains(&level) {
        return Err(CameraError::InvalidArg);
    }
    crate::write_reg_bits16(bus, slv_addr, 0x5308, 0x10, level > 0)?;
    if level > 0 {
        bus.write_reg16(slv_addr, 0x5306, ((level - 1) as u8) * 4)?;
    }
    status.denoise = level as u8;
    Ok(())
}

/// Sets brightness as a magnitude plus a sign bit.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `-3..=3`, else any bus error.
pub fn set_brightness<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    level: i32,
) -> Result<(), CameraError> {
    if !(-3..=3).contains(&level) {
        return Err(CameraError::InvalidArg);
    }
    let (value, negative) = match level {
        3 => (0x30u8, false),
        2 => (0x20u8, false),
        1 => (0x10u8, false),
        -1 => (0x10u8, true),
        -2 => (0x20u8, true),
        -3 => (0x30u8, true),
        _ => (0x00u8, false),
    };
    bus.write_reg16(slv_addr, 0x5587, value)?;
    crate::write_reg_bits16(bus, slv_addr, 0x5588, 0x08, negative)?;
    status.brightness = level as i8;
    Ok(())
}

/// Sets sharpness by programming the ISP edge-enhancement block.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `-3..=3`, else any bus error.
pub fn set_sharpness<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    level: i32,
) -> Result<(), CameraError> {
    if !(-3..=3).contains(&level) {
        return Err(CameraError::InvalidArg);
    }
    let mt_offset_2 = ((level + 3) as u8) * 8;
    let mt_offset_1 = mt_offset_2 + 1;

    crate::write_reg_bits16(bus, slv_addr, 0x5308, 0x40, false)?;
    bus.write_reg16(slv_addr, 0x5300, 0x10)?;
    bus.write_reg16(slv_addr, 0x5301, 0x10)?;
    bus.write_reg16(slv_addr, 0x5302, mt_offset_1)?;
    bus.write_reg16(slv_addr, 0x5303, mt_offset_2)?;
    bus.write_reg16(slv_addr, 0x5309, 0x10)?;
    bus.write_reg16(slv_addr, 0x530A, 0x10)?;
    bus.write_reg16(slv_addr, 0x530B, 0x04)?;
    bus.write_reg16(slv_addr, 0x530C, 0x06)?;

    status.sharpness = level as i8;
    Ok(())
}

/// Sets the auto-exposure target level.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `-5..=5`, else any bus error.
pub fn set_ae_level<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    level: i32,
) -> Result<(), CameraError> {
    if !(-5..=5).contains(&level) {
        return Err(CameraError::InvalidArg);
    }
    let target_level = ((level + 5) * 10) + 5;
    let level_low = (target_level * 23) / 25;
    let level_high = (target_level * 27) / 25;
    let fast_low = level_low >> 1;
    let mut fast_high = level_high << 1;
    if fast_high > 255 {
        fast_high = 255;
    }

    bus.write_reg16(slv_addr, 0x3A0F, level_high as u8)?;
    bus.write_reg16(slv_addr, 0x3A10, level_low as u8)?;
    bus.write_reg16(slv_addr, 0x3A1B, level_high as u8)?;
    bus.write_reg16(slv_addr, 0x3A1E, level_low as u8)?;
    bus.write_reg16(slv_addr, 0x3A11, fast_high as u8)?;
    bus.write_reg16(slv_addr, 0x3A1F, fast_low as u8)?;

    status.ae_level = level as i8;
    Ok(())
}

/// Selects a white-balance preset, or returns to auto at mode 0.
///
/// # Errors
///
/// [`CameraError::InvalidArg`] outside `0..=4`, else any bus error.
pub fn set_wb_mode<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    status: &mut CameraStatus,
    mode: i32,
) -> Result<(), CameraError> {
    if !(0..=4).contains(&mode) {
        return Err(CameraError::InvalidArg);
    }
    bus.write_reg16(slv_addr, 0x3406, u8::from(mode != 0))?;
    match mode {
        1 => {
            bus.write_reg16_val16(slv_addr, 0x3400, 0x05E0)?;
            bus.write_reg16_val16(slv_addr, 0x3402, 0x0410)?;
            bus.write_reg16_val16(slv_addr, 0x3404, 0x0540)?;
        }
        2 => {
            bus.write_reg16_val16(slv_addr, 0x3400, 0x0650)?;
            bus.write_reg16_val16(slv_addr, 0x3402, 0x0410)?;
            bus.write_reg16_val16(slv_addr, 0x3404, 0x04F0)?;
        }
        3 => {
            bus.write_reg16_val16(slv_addr, 0x3400, 0x0520)?;
            bus.write_reg16_val16(slv_addr, 0x3402, 0x0410)?;
            bus.write_reg16_val16(slv_addr, 0x3404, 0x0660)?;
        }
        4 => {
            bus.write_reg16_val16(slv_addr, 0x3400, 0x0420)?;
            bus.write_reg16_val16(slv_addr, 0x3402, 0x03F0)?;
            bus.write_reg16_val16(slv_addr, 0x3404, 0x0710)?;
        }
        _ => {}
    }
    status.wb_mode = mode as u8;
    Ok(())
}
