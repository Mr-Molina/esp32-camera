//! Safe `#![no_std]` ITU-R BT.601 YUV/RGB color-space conversions.
//!
//! Replaces `conversions/yuv.c` and software YUV422/RGB565/RGB888/YUV420
//! conversion paths across `conversions/to_bmp.c` and `conversions/to_jpg.cpp`.

use esp32_cam_types::CameraError;

/// Luma and chroma excursion range for ITU-R BT.601 YUV conversions.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum YuvRange {
    /// Full-range JFIF YCbCr (`Y` in `0..=255`, `U`/`V` in `0..=255`).
    Full,
    /// Studio-range ITU-R BT.601 (`Y` in `16..=235`, `U`/`V` in `16..=240`).
    #[default]
    Bt601Limited,
}

#[derive(Clone, Copy)]
#[repr(C)]
struct YuvTableRow {
    v_y: i16,
    v_vr: i16,
    v_vg: i16,
    v_ug: i16,
    v_ub: i16,
}

const _: () = assert!(core::mem::size_of::<YuvTableRow>() == 10);
const _: () = assert!(core::mem::size_of::<[YuvTableRow; 256]>() == 2560);

/// Exact 256-entry studio-range lookup table from `conversions/yuv.c:25-283`.
static YUV_TABLE: [YuvTableRow; 256] = [
    YuvTableRow {
        v_y: -18,
        v_vr: -204,
        v_vg: 50,
        v_ug: 104,
        v_ub: -258,
    }, // 0
    YuvTableRow {
        v_y: -17,
        v_vr: -202,
        v_vg: 49,
        v_ug: 103,
        v_ub: -256,
    }, // 1
    YuvTableRow {
        v_y: -16,
        v_vr: -201,
        v_vg: 49,
        v_ug: 102,
        v_ub: -254,
    }, // 2
    YuvTableRow {
        v_y: -15,
        v_vr: -199,
        v_vg: 48,
        v_ug: 101,
        v_ub: -252,
    }, // 3
    YuvTableRow {
        v_y: -13,
        v_vr: -197,
        v_vg: 48,
        v_ug: 100,
        v_ub: -250,
    }, // 4
    YuvTableRow {
        v_y: -12,
        v_vr: -196,
        v_vg: 48,
        v_ug: 99,
        v_ub: -248,
    }, // 5
    YuvTableRow {
        v_y: -11,
        v_vr: -194,
        v_vg: 47,
        v_ug: 99,
        v_ub: -246,
    }, // 6
    YuvTableRow {
        v_y: -10,
        v_vr: -193,
        v_vg: 47,
        v_ug: 98,
        v_ub: -244,
    }, // 7
    YuvTableRow {
        v_y: -9,
        v_vr: -191,
        v_vg: 46,
        v_ug: 97,
        v_ub: -242,
    }, // 8
    YuvTableRow {
        v_y: -8,
        v_vr: -189,
        v_vg: 46,
        v_ug: 96,
        v_ub: -240,
    }, // 9
    YuvTableRow {
        v_y: -6,
        v_vr: -188,
        v_vg: 46,
        v_ug: 95,
        v_ub: -238,
    }, // 10
    YuvTableRow {
        v_y: -5,
        v_vr: -186,
        v_vg: 45,
        v_ug: 95,
        v_ub: -236,
    }, // 11
    YuvTableRow {
        v_y: -4,
        v_vr: -185,
        v_vg: 45,
        v_ug: 94,
        v_ub: -234,
    }, // 12
    YuvTableRow {
        v_y: -3,
        v_vr: -183,
        v_vg: 44,
        v_ug: 93,
        v_ub: -232,
    }, // 13
    YuvTableRow {
        v_y: -2,
        v_vr: -181,
        v_vg: 44,
        v_ug: 92,
        v_ub: -230,
    }, // 14
    YuvTableRow {
        v_y: -1,
        v_vr: -180,
        v_vg: 44,
        v_ug: 91,
        v_ub: -228,
    }, // 15
    YuvTableRow {
        v_y: 0,
        v_vr: -178,
        v_vg: 43,
        v_ug: 91,
        v_ub: -226,
    }, // 16
    YuvTableRow {
        v_y: 1,
        v_vr: -177,
        v_vg: 43,
        v_ug: 90,
        v_ub: -223,
    }, // 17
    YuvTableRow {
        v_y: 2,
        v_vr: -175,
        v_vg: 43,
        v_ug: 89,
        v_ub: -221,
    }, // 18
    YuvTableRow {
        v_y: 3,
        v_vr: -173,
        v_vg: 42,
        v_ug: 88,
        v_ub: -219,
    }, // 19
    YuvTableRow {
        v_y: 4,
        v_vr: -172,
        v_vg: 42,
        v_ug: 87,
        v_ub: -217,
    }, // 20
    YuvTableRow {
        v_y: 5,
        v_vr: -170,
        v_vg: 41,
        v_ug: 86,
        v_ub: -215,
    }, // 21
    YuvTableRow {
        v_y: 6,
        v_vr: -169,
        v_vg: 41,
        v_ug: 86,
        v_ub: -213,
    }, // 22
    YuvTableRow {
        v_y: 8,
        v_vr: -167,
        v_vg: 41,
        v_ug: 85,
        v_ub: -211,
    }, // 23
    YuvTableRow {
        v_y: 9,
        v_vr: -165,
        v_vg: 40,
        v_ug: 84,
        v_ub: -209,
    }, // 24
    YuvTableRow {
        v_y: 10,
        v_vr: -164,
        v_vg: 40,
        v_ug: 83,
        v_ub: -207,
    }, // 25
    YuvTableRow {
        v_y: 11,
        v_vr: -162,
        v_vg: 39,
        v_ug: 82,
        v_ub: -205,
    }, // 26
    YuvTableRow {
        v_y: 12,
        v_vr: -161,
        v_vg: 39,
        v_ug: 82,
        v_ub: -203,
    }, // 27
    YuvTableRow {
        v_y: 13,
        v_vr: -159,
        v_vg: 39,
        v_ug: 81,
        v_ub: -201,
    }, // 28
    YuvTableRow {
        v_y: 15,
        v_vr: -158,
        v_vg: 38,
        v_ug: 80,
        v_ub: -199,
    }, // 29
    YuvTableRow {
        v_y: 16,
        v_vr: -156,
        v_vg: 38,
        v_ug: 79,
        v_ub: -197,
    }, // 30
    YuvTableRow {
        v_y: 17,
        v_vr: -154,
        v_vg: 37,
        v_ug: 78,
        v_ub: -195,
    }, // 31
    YuvTableRow {
        v_y: 18,
        v_vr: -153,
        v_vg: 37,
        v_ug: 78,
        v_ub: -193,
    }, // 32
    YuvTableRow {
        v_y: 19,
        v_vr: -151,
        v_vg: 37,
        v_ug: 77,
        v_ub: -191,
    }, // 33
    YuvTableRow {
        v_y: 20,
        v_vr: -150,
        v_vg: 36,
        v_ug: 76,
        v_ub: -189,
    }, // 34
    YuvTableRow {
        v_y: 22,
        v_vr: -148,
        v_vg: 36,
        v_ug: 75,
        v_ub: -187,
    }, // 35
    YuvTableRow {
        v_y: 23,
        v_vr: -146,
        v_vg: 35,
        v_ug: 74,
        v_ub: -185,
    }, // 36
    YuvTableRow {
        v_y: 24,
        v_vr: -145,
        v_vg: 35,
        v_ug: 73,
        v_ub: -183,
    }, // 37
    YuvTableRow {
        v_y: 25,
        v_vr: -143,
        v_vg: 35,
        v_ug: 73,
        v_ub: -181,
    }, // 38
    YuvTableRow {
        v_y: 26,
        v_vr: -142,
        v_vg: 34,
        v_ug: 72,
        v_ub: -179,
    }, // 39
    YuvTableRow {
        v_y: 27,
        v_vr: -140,
        v_vg: 34,
        v_ug: 71,
        v_ub: -177,
    }, // 40
    YuvTableRow {
        v_y: 29,
        v_vr: -138,
        v_vg: 34,
        v_ug: 70,
        v_ub: -175,
    }, // 41
    YuvTableRow {
        v_y: 30,
        v_vr: -137,
        v_vg: 33,
        v_ug: 69,
        v_ub: -173,
    }, // 42
    YuvTableRow {
        v_y: 31,
        v_vr: -135,
        v_vg: 33,
        v_ug: 69,
        v_ub: -171,
    }, // 43
    YuvTableRow {
        v_y: 32,
        v_vr: -134,
        v_vg: 32,
        v_ug: 68,
        v_ub: -169,
    }, // 44
    YuvTableRow {
        v_y: 33,
        v_vr: -132,
        v_vg: 32,
        v_ug: 67,
        v_ub: -167,
    }, // 45
    YuvTableRow {
        v_y: 34,
        v_vr: -130,
        v_vg: 32,
        v_ug: 66,
        v_ub: -165,
    }, // 46
    YuvTableRow {
        v_y: 36,
        v_vr: -129,
        v_vg: 31,
        v_ug: 65,
        v_ub: -163,
    }, // 47
    YuvTableRow {
        v_y: 37,
        v_vr: -127,
        v_vg: 31,
        v_ug: 65,
        v_ub: -161,
    }, // 48
    YuvTableRow {
        v_y: 38,
        v_vr: -126,
        v_vg: 30,
        v_ug: 64,
        v_ub: -159,
    }, // 49
    YuvTableRow {
        v_y: 39,
        v_vr: -124,
        v_vg: 30,
        v_ug: 63,
        v_ub: -157,
    }, // 50
    YuvTableRow {
        v_y: 40,
        v_vr: -122,
        v_vg: 30,
        v_ug: 62,
        v_ub: -155,
    }, // 51
    YuvTableRow {
        v_y: 41,
        v_vr: -121,
        v_vg: 29,
        v_ug: 61,
        v_ub: -153,
    }, // 52
    YuvTableRow {
        v_y: 43,
        v_vr: -119,
        v_vg: 29,
        v_ug: 60,
        v_ub: -151,
    }, // 53
    YuvTableRow {
        v_y: 44,
        v_vr: -118,
        v_vg: 28,
        v_ug: 60,
        v_ub: -149,
    }, // 54
    YuvTableRow {
        v_y: 45,
        v_vr: -116,
        v_vg: 28,
        v_ug: 59,
        v_ub: -147,
    }, // 55
    YuvTableRow {
        v_y: 46,
        v_vr: -114,
        v_vg: 28,
        v_ug: 58,
        v_ub: -145,
    }, // 56
    YuvTableRow {
        v_y: 47,
        v_vr: -113,
        v_vg: 27,
        v_ug: 57,
        v_ub: -143,
    }, // 57
    YuvTableRow {
        v_y: 48,
        v_vr: -111,
        v_vg: 27,
        v_ug: 56,
        v_ub: -141,
    }, // 58
    YuvTableRow {
        v_y: 50,
        v_vr: -110,
        v_vg: 26,
        v_ug: 56,
        v_ub: -139,
    }, // 59
    YuvTableRow {
        v_y: 51,
        v_vr: -108,
        v_vg: 26,
        v_ug: 55,
        v_ub: -137,
    }, // 60
    YuvTableRow {
        v_y: 52,
        v_vr: -106,
        v_vg: 26,
        v_ug: 54,
        v_ub: -135,
    }, // 61
    YuvTableRow {
        v_y: 53,
        v_vr: -105,
        v_vg: 25,
        v_ug: 53,
        v_ub: -133,
    }, // 62
    YuvTableRow {
        v_y: 54,
        v_vr: -103,
        v_vg: 25,
        v_ug: 52,
        v_ub: -131,
    }, // 63
    YuvTableRow {
        v_y: 55,
        v_vr: -102,
        v_vg: 25,
        v_ug: 52,
        v_ub: -129,
    }, // 64
    YuvTableRow {
        v_y: 57,
        v_vr: -100,
        v_vg: 24,
        v_ug: 51,
        v_ub: -127,
    }, // 65
    YuvTableRow {
        v_y: 58,
        v_vr: -98,
        v_vg: 24,
        v_ug: 50,
        v_ub: -125,
    }, // 66
    YuvTableRow {
        v_y: 59,
        v_vr: -97,
        v_vg: 23,
        v_ug: 49,
        v_ub: -123,
    }, // 67
    YuvTableRow {
        v_y: 60,
        v_vr: -95,
        v_vg: 23,
        v_ug: 48,
        v_ub: -121,
    }, // 68
    YuvTableRow {
        v_y: 61,
        v_vr: -94,
        v_vg: 23,
        v_ug: 47,
        v_ub: -119,
    }, // 69
    YuvTableRow {
        v_y: 62,
        v_vr: -92,
        v_vg: 22,
        v_ug: 47,
        v_ub: -117,
    }, // 70
    YuvTableRow {
        v_y: 64,
        v_vr: -90,
        v_vg: 22,
        v_ug: 46,
        v_ub: -115,
    }, // 71
    YuvTableRow {
        v_y: 65,
        v_vr: -89,
        v_vg: 21,
        v_ug: 45,
        v_ub: -113,
    }, // 72
    YuvTableRow {
        v_y: 66,
        v_vr: -87,
        v_vg: 21,
        v_ug: 44,
        v_ub: -110,
    }, // 73
    YuvTableRow {
        v_y: 67,
        v_vr: -86,
        v_vg: 21,
        v_ug: 43,
        v_ub: -108,
    }, // 74
    YuvTableRow {
        v_y: 68,
        v_vr: -84,
        v_vg: 20,
        v_ug: 43,
        v_ub: -106,
    }, // 75
    YuvTableRow {
        v_y: 69,
        v_vr: -82,
        v_vg: 20,
        v_ug: 42,
        v_ub: -104,
    }, // 76
    YuvTableRow {
        v_y: 71,
        v_vr: -81,
        v_vg: 19,
        v_ug: 41,
        v_ub: -102,
    }, // 77
    YuvTableRow {
        v_y: 72,
        v_vr: -79,
        v_vg: 19,
        v_ug: 40,
        v_ub: -100,
    }, // 78
    YuvTableRow {
        v_y: 73,
        v_vr: -78,
        v_vg: 19,
        v_ug: 39,
        v_ub: -98,
    }, // 79
    YuvTableRow {
        v_y: 74,
        v_vr: -76,
        v_vg: 18,
        v_ug: 39,
        v_ub: -96,
    }, // 80
    YuvTableRow {
        v_y: 75,
        v_vr: -75,
        v_vg: 18,
        v_ug: 38,
        v_ub: -94,
    }, // 81
    YuvTableRow {
        v_y: 76,
        v_vr: -73,
        v_vg: 17,
        v_ug: 37,
        v_ub: -92,
    }, // 82
    YuvTableRow {
        v_y: 77,
        v_vr: -71,
        v_vg: 17,
        v_ug: 36,
        v_ub: -90,
    }, // 83
    YuvTableRow {
        v_y: 79,
        v_vr: -70,
        v_vg: 17,
        v_ug: 35,
        v_ub: -88,
    }, // 84
    YuvTableRow {
        v_y: 80,
        v_vr: -68,
        v_vg: 16,
        v_ug: 34,
        v_ub: -86,
    }, // 85
    YuvTableRow {
        v_y: 81,
        v_vr: -67,
        v_vg: 16,
        v_ug: 34,
        v_ub: -84,
    }, // 86
    YuvTableRow {
        v_y: 82,
        v_vr: -65,
        v_vg: 16,
        v_ug: 33,
        v_ub: -82,
    }, // 87
    YuvTableRow {
        v_y: 83,
        v_vr: -63,
        v_vg: 15,
        v_ug: 32,
        v_ub: -80,
    }, // 88
    YuvTableRow {
        v_y: 84,
        v_vr: -62,
        v_vg: 15,
        v_ug: 31,
        v_ub: -78,
    }, // 89
    YuvTableRow {
        v_y: 86,
        v_vr: -60,
        v_vg: 14,
        v_ug: 30,
        v_ub: -76,
    }, // 90
    YuvTableRow {
        v_y: 87,
        v_vr: -59,
        v_vg: 14,
        v_ug: 30,
        v_ub: -74,
    }, // 91
    YuvTableRow {
        v_y: 88,
        v_vr: -57,
        v_vg: 14,
        v_ug: 29,
        v_ub: -72,
    }, // 92
    YuvTableRow {
        v_y: 89,
        v_vr: -55,
        v_vg: 13,
        v_ug: 28,
        v_ub: -70,
    }, // 93
    YuvTableRow {
        v_y: 90,
        v_vr: -54,
        v_vg: 13,
        v_ug: 27,
        v_ub: -68,
    }, // 94
    YuvTableRow {
        v_y: 91,
        v_vr: -52,
        v_vg: 12,
        v_ug: 26,
        v_ub: -66,
    }, // 95
    YuvTableRow {
        v_y: 93,
        v_vr: -51,
        v_vg: 12,
        v_ug: 26,
        v_ub: -64,
    }, // 96
    YuvTableRow {
        v_y: 94,
        v_vr: -49,
        v_vg: 12,
        v_ug: 25,
        v_ub: -62,
    }, // 97
    YuvTableRow {
        v_y: 95,
        v_vr: -47,
        v_vg: 11,
        v_ug: 24,
        v_ub: -60,
    }, // 98
    YuvTableRow {
        v_y: 96,
        v_vr: -46,
        v_vg: 11,
        v_ug: 23,
        v_ub: -58,
    }, // 99
    YuvTableRow {
        v_y: 97,
        v_vr: -44,
        v_vg: 10,
        v_ug: 22,
        v_ub: -56,
    }, // 100
    YuvTableRow {
        v_y: 98,
        v_vr: -43,
        v_vg: 10,
        v_ug: 21,
        v_ub: -54,
    }, // 101
    YuvTableRow {
        v_y: 100,
        v_vr: -41,
        v_vg: 10,
        v_ug: 21,
        v_ub: -52,
    }, // 102
    YuvTableRow {
        v_y: 101,
        v_vr: -39,
        v_vg: 9,
        v_ug: 20,
        v_ub: -50,
    }, // 103
    YuvTableRow {
        v_y: 102,
        v_vr: -38,
        v_vg: 9,
        v_ug: 19,
        v_ub: -48,
    }, // 104
    YuvTableRow {
        v_y: 103,
        v_vr: -36,
        v_vg: 8,
        v_ug: 18,
        v_ub: -46,
    }, // 105
    YuvTableRow {
        v_y: 104,
        v_vr: -35,
        v_vg: 8,
        v_ug: 17,
        v_ub: -44,
    }, // 106
    YuvTableRow {
        v_y: 105,
        v_vr: -33,
        v_vg: 8,
        v_ug: 17,
        v_ub: -42,
    }, // 107
    YuvTableRow {
        v_y: 107,
        v_vr: -31,
        v_vg: 7,
        v_ug: 16,
        v_ub: -40,
    }, // 108
    YuvTableRow {
        v_y: 108,
        v_vr: -30,
        v_vg: 7,
        v_ug: 15,
        v_ub: -38,
    }, // 109
    YuvTableRow {
        v_y: 109,
        v_vr: -28,
        v_vg: 7,
        v_ug: 14,
        v_ub: -36,
    }, // 110
    YuvTableRow {
        v_y: 110,
        v_vr: -27,
        v_vg: 6,
        v_ug: 13,
        v_ub: -34,
    }, // 111
    YuvTableRow {
        v_y: 111,
        v_vr: -25,
        v_vg: 6,
        v_ug: 13,
        v_ub: -32,
    }, // 112
    YuvTableRow {
        v_y: 112,
        v_vr: -23,
        v_vg: 5,
        v_ug: 12,
        v_ub: -30,
    }, // 113
    YuvTableRow {
        v_y: 114,
        v_vr: -22,
        v_vg: 5,
        v_ug: 11,
        v_ub: -28,
    }, // 114
    YuvTableRow {
        v_y: 115,
        v_vr: -20,
        v_vg: 5,
        v_ug: 10,
        v_ub: -26,
    }, // 115
    YuvTableRow {
        v_y: 116,
        v_vr: -19,
        v_vg: 4,
        v_ug: 9,
        v_ub: -24,
    }, // 116
    YuvTableRow {
        v_y: 117,
        v_vr: -17,
        v_vg: 4,
        v_ug: 8,
        v_ub: -22,
    }, // 117
    YuvTableRow {
        v_y: 118,
        v_vr: -15,
        v_vg: 3,
        v_ug: 8,
        v_ub: -20,
    }, // 118
    YuvTableRow {
        v_y: 119,
        v_vr: -14,
        v_vg: 3,
        v_ug: 7,
        v_ub: -18,
    }, // 119
    YuvTableRow {
        v_y: 121,
        v_vr: -12,
        v_vg: 3,
        v_ug: 6,
        v_ub: -16,
    }, // 120
    YuvTableRow {
        v_y: 122,
        v_vr: -11,
        v_vg: 2,
        v_ug: 5,
        v_ub: -14,
    }, // 121
    YuvTableRow {
        v_y: 123,
        v_vr: -9,
        v_vg: 2,
        v_ug: 4,
        v_ub: -12,
    }, // 122
    YuvTableRow {
        v_y: 124,
        v_vr: -7,
        v_vg: 1,
        v_ug: 4,
        v_ub: -10,
    }, // 123
    YuvTableRow {
        v_y: 125,
        v_vr: -6,
        v_vg: 1,
        v_ug: 3,
        v_ub: -8,
    }, // 124
    YuvTableRow {
        v_y: 126,
        v_vr: -4,
        v_vg: 1,
        v_ug: 2,
        v_ub: -6,
    }, // 125
    YuvTableRow {
        v_y: 128,
        v_vr: -3,
        v_vg: 0,
        v_ug: 1,
        v_ub: -4,
    }, // 126
    YuvTableRow {
        v_y: 129,
        v_vr: -1,
        v_vg: 0,
        v_ug: 0,
        v_ub: -2,
    }, // 127
    YuvTableRow {
        v_y: 130,
        v_vr: 0,
        v_vg: 0,
        v_ug: 0,
        v_ub: 0,
    }, // 128
    YuvTableRow {
        v_y: 131,
        v_vr: 1,
        v_vg: 0,
        v_ug: 0,
        v_ub: 2,
    }, // 129
    YuvTableRow {
        v_y: 132,
        v_vr: 3,
        v_vg: 0,
        v_ug: -1,
        v_ub: 4,
    }, // 130
    YuvTableRow {
        v_y: 133,
        v_vr: 4,
        v_vg: -1,
        v_ug: -2,
        v_ub: 6,
    }, // 131
    YuvTableRow {
        v_y: 135,
        v_vr: 6,
        v_vg: -1,
        v_ug: -3,
        v_ub: 8,
    }, // 132
    YuvTableRow {
        v_y: 136,
        v_vr: 7,
        v_vg: -1,
        v_ug: -4,
        v_ub: 10,
    }, // 133
    YuvTableRow {
        v_y: 137,
        v_vr: 9,
        v_vg: -2,
        v_ug: -4,
        v_ub: 12,
    }, // 134
    YuvTableRow {
        v_y: 138,
        v_vr: 11,
        v_vg: -2,
        v_ug: -5,
        v_ub: 14,
    }, // 135
    YuvTableRow {
        v_y: 139,
        v_vr: 12,
        v_vg: -3,
        v_ug: -6,
        v_ub: 16,
    }, // 136
    YuvTableRow {
        v_y: 140,
        v_vr: 14,
        v_vg: -3,
        v_ug: -7,
        v_ub: 18,
    }, // 137
    YuvTableRow {
        v_y: 142,
        v_vr: 15,
        v_vg: -3,
        v_ug: -8,
        v_ub: 20,
    }, // 138
    YuvTableRow {
        v_y: 143,
        v_vr: 17,
        v_vg: -4,
        v_ug: -8,
        v_ub: 22,
    }, // 139
    YuvTableRow {
        v_y: 144,
        v_vr: 19,
        v_vg: -4,
        v_ug: -9,
        v_ub: 24,
    }, // 140
    YuvTableRow {
        v_y: 145,
        v_vr: 20,
        v_vg: -5,
        v_ug: -10,
        v_ub: 26,
    }, // 141
    YuvTableRow {
        v_y: 146,
        v_vr: 22,
        v_vg: -5,
        v_ug: -11,
        v_ub: 28,
    }, // 142
    YuvTableRow {
        v_y: 147,
        v_vr: 23,
        v_vg: -5,
        v_ug: -12,
        v_ub: 30,
    }, // 143
    YuvTableRow {
        v_y: 148,
        v_vr: 25,
        v_vg: -6,
        v_ug: -13,
        v_ub: 32,
    }, // 144
    YuvTableRow {
        v_y: 150,
        v_vr: 27,
        v_vg: -6,
        v_ug: -13,
        v_ub: 34,
    }, // 145
    YuvTableRow {
        v_y: 151,
        v_vr: 28,
        v_vg: -7,
        v_ug: -14,
        v_ub: 36,
    }, // 146
    YuvTableRow {
        v_y: 152,
        v_vr: 30,
        v_vg: -7,
        v_ug: -15,
        v_ub: 38,
    }, // 147
    YuvTableRow {
        v_y: 153,
        v_vr: 31,
        v_vg: -7,
        v_ug: -16,
        v_ub: 40,
    }, // 148
    YuvTableRow {
        v_y: 154,
        v_vr: 33,
        v_vg: -8,
        v_ug: -17,
        v_ub: 42,
    }, // 149
    YuvTableRow {
        v_y: 155,
        v_vr: 35,
        v_vg: -8,
        v_ug: -17,
        v_ub: 44,
    }, // 150
    YuvTableRow {
        v_y: 157,
        v_vr: 36,
        v_vg: -8,
        v_ug: -18,
        v_ub: 46,
    }, // 151
    YuvTableRow {
        v_y: 158,
        v_vr: 38,
        v_vg: -9,
        v_ug: -19,
        v_ub: 48,
    }, // 152
    YuvTableRow {
        v_y: 159,
        v_vr: 39,
        v_vg: -9,
        v_ug: -20,
        v_ub: 50,
    }, // 153
    YuvTableRow {
        v_y: 160,
        v_vr: 41,
        v_vg: -10,
        v_ug: -21,
        v_ub: 52,
    }, // 154
    YuvTableRow {
        v_y: 161,
        v_vr: 43,
        v_vg: -10,
        v_ug: -21,
        v_ub: 54,
    }, // 155
    YuvTableRow {
        v_y: 162,
        v_vr: 44,
        v_vg: -10,
        v_ug: -22,
        v_ub: 56,
    }, // 156
    YuvTableRow {
        v_y: 164,
        v_vr: 46,
        v_vg: -11,
        v_ug: -23,
        v_ub: 58,
    }, // 157
    YuvTableRow {
        v_y: 165,
        v_vr: 47,
        v_vg: -11,
        v_ug: -24,
        v_ub: 60,
    }, // 158
    YuvTableRow {
        v_y: 166,
        v_vr: 49,
        v_vg: -12,
        v_ug: -25,
        v_ub: 62,
    }, // 159
    YuvTableRow {
        v_y: 167,
        v_vr: 51,
        v_vg: -12,
        v_ug: -26,
        v_ub: 64,
    }, // 160
    YuvTableRow {
        v_y: 168,
        v_vr: 52,
        v_vg: -12,
        v_ug: -26,
        v_ub: 66,
    }, // 161
    YuvTableRow {
        v_y: 169,
        v_vr: 54,
        v_vg: -13,
        v_ug: -27,
        v_ub: 68,
    }, // 162
    YuvTableRow {
        v_y: 171,
        v_vr: 55,
        v_vg: -13,
        v_ug: -28,
        v_ub: 70,
    }, // 163
    YuvTableRow {
        v_y: 172,
        v_vr: 57,
        v_vg: -14,
        v_ug: -29,
        v_ub: 72,
    }, // 164
    YuvTableRow {
        v_y: 173,
        v_vr: 59,
        v_vg: -14,
        v_ug: -30,
        v_ub: 74,
    }, // 165
    YuvTableRow {
        v_y: 174,
        v_vr: 60,
        v_vg: -14,
        v_ug: -30,
        v_ub: 76,
    }, // 166
    YuvTableRow {
        v_y: 175,
        v_vr: 62,
        v_vg: -15,
        v_ug: -31,
        v_ub: 78,
    }, // 167
    YuvTableRow {
        v_y: 176,
        v_vr: 63,
        v_vg: -15,
        v_ug: -32,
        v_ub: 80,
    }, // 168
    YuvTableRow {
        v_y: 178,
        v_vr: 65,
        v_vg: -16,
        v_ug: -33,
        v_ub: 82,
    }, // 169
    YuvTableRow {
        v_y: 179,
        v_vr: 67,
        v_vg: -16,
        v_ug: -34,
        v_ub: 84,
    }, // 170
    YuvTableRow {
        v_y: 180,
        v_vr: 68,
        v_vg: -16,
        v_ug: -34,
        v_ub: 86,
    }, // 171
    YuvTableRow {
        v_y: 181,
        v_vr: 70,
        v_vg: -17,
        v_ug: -35,
        v_ub: 88,
    }, // 172
    YuvTableRow {
        v_y: 182,
        v_vr: 71,
        v_vg: -17,
        v_ug: -36,
        v_ub: 90,
    }, // 173
    YuvTableRow {
        v_y: 183,
        v_vr: 73,
        v_vg: -17,
        v_ug: -37,
        v_ub: 92,
    }, // 174
    YuvTableRow {
        v_y: 185,
        v_vr: 75,
        v_vg: -18,
        v_ug: -38,
        v_ub: 94,
    }, // 175
    YuvTableRow {
        v_y: 186,
        v_vr: 76,
        v_vg: -18,
        v_ug: -39,
        v_ub: 96,
    }, // 176
    YuvTableRow {
        v_y: 187,
        v_vr: 78,
        v_vg: -19,
        v_ug: -39,
        v_ub: 98,
    }, // 177
    YuvTableRow {
        v_y: 188,
        v_vr: 79,
        v_vg: -19,
        v_ug: -40,
        v_ub: 100,
    }, // 178
    YuvTableRow {
        v_y: 189,
        v_vr: 81,
        v_vg: -19,
        v_ug: -41,
        v_ub: 102,
    }, // 179
    YuvTableRow {
        v_y: 190,
        v_vr: 82,
        v_vg: -20,
        v_ug: -42,
        v_ub: 104,
    }, // 180
    YuvTableRow {
        v_y: 192,
        v_vr: 84,
        v_vg: -20,
        v_ug: -43,
        v_ub: 106,
    }, // 181
    YuvTableRow {
        v_y: 193,
        v_vr: 86,
        v_vg: -21,
        v_ug: -43,
        v_ub: 108,
    }, // 182
    YuvTableRow {
        v_y: 194,
        v_vr: 87,
        v_vg: -21,
        v_ug: -44,
        v_ub: 110,
    }, // 183
    YuvTableRow {
        v_y: 195,
        v_vr: 89,
        v_vg: -21,
        v_ug: -45,
        v_ub: 113,
    }, // 184
    YuvTableRow {
        v_y: 196,
        v_vr: 90,
        v_vg: -22,
        v_ug: -46,
        v_ub: 115,
    }, // 185
    YuvTableRow {
        v_y: 197,
        v_vr: 92,
        v_vg: -22,
        v_ug: -47,
        v_ub: 117,
    }, // 186
    YuvTableRow {
        v_y: 199,
        v_vr: 94,
        v_vg: -23,
        v_ug: -47,
        v_ub: 119,
    }, // 187
    YuvTableRow {
        v_y: 200,
        v_vr: 95,
        v_vg: -23,
        v_ug: -48,
        v_ub: 121,
    }, // 188
    YuvTableRow {
        v_y: 201,
        v_vr: 97,
        v_vg: -23,
        v_ug: -49,
        v_ub: 123,
    }, // 189
    YuvTableRow {
        v_y: 202,
        v_vr: 98,
        v_vg: -24,
        v_ug: -50,
        v_ub: 125,
    }, // 190
    YuvTableRow {
        v_y: 203,
        v_vr: 100,
        v_vg: -24,
        v_ug: -51,
        v_ub: 127,
    }, // 191
    YuvTableRow {
        v_y: 204,
        v_vr: 102,
        v_vg: -25,
        v_ug: -52,
        v_ub: 129,
    }, // 192
    YuvTableRow {
        v_y: 206,
        v_vr: 103,
        v_vg: -25,
        v_ug: -52,
        v_ub: 131,
    }, // 193
    YuvTableRow {
        v_y: 207,
        v_vr: 105,
        v_vg: -25,
        v_ug: -53,
        v_ub: 133,
    }, // 194
    YuvTableRow {
        v_y: 208,
        v_vr: 106,
        v_vg: -26,
        v_ug: -54,
        v_ub: 135,
    }, // 195
    YuvTableRow {
        v_y: 209,
        v_vr: 108,
        v_vg: -26,
        v_ug: -55,
        v_ub: 137,
    }, // 196
    YuvTableRow {
        v_y: 210,
        v_vr: 110,
        v_vg: -26,
        v_ug: -56,
        v_ub: 139,
    }, // 197
    YuvTableRow {
        v_y: 211,
        v_vr: 111,
        v_vg: -27,
        v_ug: -56,
        v_ub: 141,
    }, // 198
    YuvTableRow {
        v_y: 213,
        v_vr: 113,
        v_vg: -27,
        v_ug: -57,
        v_ub: 143,
    }, // 199
    YuvTableRow {
        v_y: 214,
        v_vr: 114,
        v_vg: -28,
        v_ug: -58,
        v_ub: 145,
    }, // 200
    YuvTableRow {
        v_y: 215,
        v_vr: 116,
        v_vg: -28,
        v_ug: -59,
        v_ub: 147,
    }, // 201
    YuvTableRow {
        v_y: 216,
        v_vr: 118,
        v_vg: -28,
        v_ug: -60,
        v_ub: 149,
    }, // 202
    YuvTableRow {
        v_y: 217,
        v_vr: 119,
        v_vg: -29,
        v_ug: -60,
        v_ub: 151,
    }, // 203
    YuvTableRow {
        v_y: 218,
        v_vr: 121,
        v_vg: -29,
        v_ug: -61,
        v_ub: 153,
    }, // 204
    YuvTableRow {
        v_y: 219,
        v_vr: 122,
        v_vg: -30,
        v_ug: -62,
        v_ub: 155,
    }, // 205
    YuvTableRow {
        v_y: 221,
        v_vr: 124,
        v_vg: -30,
        v_ug: -63,
        v_ub: 157,
    }, // 206
    YuvTableRow {
        v_y: 222,
        v_vr: 126,
        v_vg: -30,
        v_ug: -64,
        v_ub: 159,
    }, // 207
    YuvTableRow {
        v_y: 223,
        v_vr: 127,
        v_vg: -31,
        v_ug: -65,
        v_ub: 161,
    }, // 208
    YuvTableRow {
        v_y: 224,
        v_vr: 129,
        v_vg: -31,
        v_ug: -65,
        v_ub: 163,
    }, // 209
    YuvTableRow {
        v_y: 225,
        v_vr: 130,
        v_vg: -32,
        v_ug: -66,
        v_ub: 165,
    }, // 210
    YuvTableRow {
        v_y: 226,
        v_vr: 132,
        v_vg: -32,
        v_ug: -67,
        v_ub: 167,
    }, // 211
    YuvTableRow {
        v_y: 228,
        v_vr: 134,
        v_vg: -32,
        v_ug: -68,
        v_ub: 169,
    }, // 212
    YuvTableRow {
        v_y: 229,
        v_vr: 135,
        v_vg: -33,
        v_ug: -69,
        v_ub: 171,
    }, // 213
    YuvTableRow {
        v_y: 230,
        v_vr: 137,
        v_vg: -33,
        v_ug: -69,
        v_ub: 173,
    }, // 214
    YuvTableRow {
        v_y: 231,
        v_vr: 138,
        v_vg: -34,
        v_ug: -70,
        v_ub: 175,
    }, // 215
    YuvTableRow {
        v_y: 232,
        v_vr: 140,
        v_vg: -34,
        v_ug: -71,
        v_ub: 177,
    }, // 216
    YuvTableRow {
        v_y: 233,
        v_vr: 142,
        v_vg: -34,
        v_ug: -72,
        v_ub: 179,
    }, // 217
    YuvTableRow {
        v_y: 235,
        v_vr: 143,
        v_vg: -35,
        v_ug: -73,
        v_ub: 181,
    }, // 218
    YuvTableRow {
        v_y: 236,
        v_vr: 145,
        v_vg: -35,
        v_ug: -73,
        v_ub: 183,
    }, // 219
    YuvTableRow {
        v_y: 237,
        v_vr: 146,
        v_vg: -35,
        v_ug: -74,
        v_ub: 185,
    }, // 220
    YuvTableRow {
        v_y: 238,
        v_vr: 148,
        v_vg: -36,
        v_ug: -75,
        v_ub: 187,
    }, // 221
    YuvTableRow {
        v_y: 239,
        v_vr: 150,
        v_vg: -36,
        v_ug: -76,
        v_ub: 189,
    }, // 222
    YuvTableRow {
        v_y: 240,
        v_vr: 151,
        v_vg: -37,
        v_ug: -77,
        v_ub: 191,
    }, // 223
    YuvTableRow {
        v_y: 242,
        v_vr: 153,
        v_vg: -37,
        v_ug: -78,
        v_ub: 193,
    }, // 224
    YuvTableRow {
        v_y: 243,
        v_vr: 154,
        v_vg: -37,
        v_ug: -78,
        v_ub: 195,
    }, // 225
    YuvTableRow {
        v_y: 244,
        v_vr: 156,
        v_vg: -38,
        v_ug: -79,
        v_ub: 197,
    }, // 226
    YuvTableRow {
        v_y: 245,
        v_vr: 158,
        v_vg: -38,
        v_ug: -80,
        v_ub: 199,
    }, // 227
    YuvTableRow {
        v_y: 246,
        v_vr: 159,
        v_vg: -39,
        v_ug: -81,
        v_ub: 201,
    }, // 228
    YuvTableRow {
        v_y: 247,
        v_vr: 161,
        v_vg: -39,
        v_ug: -82,
        v_ub: 203,
    }, // 229
    YuvTableRow {
        v_y: 249,
        v_vr: 162,
        v_vg: -39,
        v_ug: -82,
        v_ub: 205,
    }, // 230
    YuvTableRow {
        v_y: 250,
        v_vr: 164,
        v_vg: -40,
        v_ug: -83,
        v_ub: 207,
    }, // 231
    YuvTableRow {
        v_y: 251,
        v_vr: 165,
        v_vg: -40,
        v_ug: -84,
        v_ub: 209,
    }, // 232
    YuvTableRow {
        v_y: 252,
        v_vr: 167,
        v_vg: -41,
        v_ug: -85,
        v_ub: 211,
    }, // 233
    YuvTableRow {
        v_y: 253,
        v_vr: 169,
        v_vg: -41,
        v_ug: -86,
        v_ub: 213,
    }, // 234
    YuvTableRow {
        v_y: 254,
        v_vr: 170,
        v_vg: -41,
        v_ug: -86,
        v_ub: 215,
    }, // 235
    YuvTableRow {
        v_y: 256,
        v_vr: 172,
        v_vg: -42,
        v_ug: -87,
        v_ub: 217,
    }, // 236
    YuvTableRow {
        v_y: 257,
        v_vr: 173,
        v_vg: -42,
        v_ug: -88,
        v_ub: 219,
    }, // 237
    YuvTableRow {
        v_y: 258,
        v_vr: 175,
        v_vg: -43,
        v_ug: -89,
        v_ub: 221,
    }, // 238
    YuvTableRow {
        v_y: 259,
        v_vr: 177,
        v_vg: -43,
        v_ug: -90,
        v_ub: 223,
    }, // 239
    YuvTableRow {
        v_y: 260,
        v_vr: 178,
        v_vg: -43,
        v_ug: -91,
        v_ub: 226,
    }, // 240
    YuvTableRow {
        v_y: 261,
        v_vr: 180,
        v_vg: -44,
        v_ug: -91,
        v_ub: 228,
    }, // 241
    YuvTableRow {
        v_y: 263,
        v_vr: 181,
        v_vg: -44,
        v_ug: -92,
        v_ub: 230,
    }, // 242
    YuvTableRow {
        v_y: 264,
        v_vr: 183,
        v_vg: -44,
        v_ug: -93,
        v_ub: 232,
    }, // 243
    YuvTableRow {
        v_y: 265,
        v_vr: 185,
        v_vg: -45,
        v_ug: -94,
        v_ub: 234,
    }, // 244
    YuvTableRow {
        v_y: 266,
        v_vr: 186,
        v_vg: -45,
        v_ug: -95,
        v_ub: 236,
    }, // 245
    YuvTableRow {
        v_y: 267,
        v_vr: 188,
        v_vg: -46,
        v_ug: -95,
        v_ub: 238,
    }, // 246
    YuvTableRow {
        v_y: 268,
        v_vr: 189,
        v_vg: -46,
        v_ug: -96,
        v_ub: 240,
    }, // 247
    YuvTableRow {
        v_y: 270,
        v_vr: 191,
        v_vg: -46,
        v_ug: -97,
        v_ub: 242,
    }, // 248
    YuvTableRow {
        v_y: 271,
        v_vr: 193,
        v_vg: -47,
        v_ug: -98,
        v_ub: 244,
    }, // 249
    YuvTableRow {
        v_y: 272,
        v_vr: 194,
        v_vg: -47,
        v_ug: -99,
        v_ub: 246,
    }, // 250
    YuvTableRow {
        v_y: 273,
        v_vr: 196,
        v_vg: -48,
        v_ug: -99,
        v_ub: 248,
    }, // 251
    YuvTableRow {
        v_y: 274,
        v_vr: 197,
        v_vg: -48,
        v_ug: -100,
        v_ub: 250,
    }, // 252
    YuvTableRow {
        v_y: 275,
        v_vr: 199,
        v_vg: -48,
        v_ug: -101,
        v_ub: 252,
    }, // 253
    YuvTableRow {
        v_y: 277,
        v_vr: 201,
        v_vg: -49,
        v_ug: -102,
        v_ub: 254,
    }, // 254
    YuvTableRow {
        v_y: 278,
        v_vr: 202,
        v_vg: -49,
        v_ug: -103,
        v_ub: 256,
    }, // 255
];

/// Converts a single YUV pixel `(y, u, v)` to RGB `(r, g, b)` using the exact
/// studio-range (`YuvRange::Bt601Limited`) table from `conversions/yuv.c`.
#[must_use]
#[inline]
pub fn yuv2rgb(y: u8, u: u8, v: u8) -> (u8, u8, u8) {
    yuv2rgb_with_range(y, u, v, YuvRange::Bt601Limited)
}

/// Converts a single YUV pixel `(y, u, v)` to RGB `(r, g, b)` using the
/// specified [`YuvRange`].
#[must_use]
#[inline]
pub fn yuv2rgb_with_range(y: u8, u: u8, v: u8, range: YuvRange) -> (u8, u8, u8) {
    match range {
        YuvRange::Bt601Limited => {
            let y_row = YUV_TABLE[y as usize];
            let u_row = YUV_TABLE[u as usize];
            let v_row = YUV_TABLE[v as usize];

            let ri = y_row.v_y + v_row.v_vr;
            let gi = y_row.v_y + u_row.v_vg + v_row.v_ug;
            let bi = y_row.v_y + u_row.v_ub;

            (
                ri.clamp(0, 255) as u8,
                gi.clamp(0, 255) as u8,
                bi.clamp(0, 255) as u8,
            )
        }
        YuvRange::Full => {
            let yi = i32::from(y);
            let cb = i32::from(u) - 128;
            let cr = i32::from(v) - 128;

            let r = clamp_u8(yi + ((91881 * cr + 32768) >> 16));
            let g = clamp_u8(yi - ((22554 * cb + 46802 * cr + 32768) >> 16));
            let b = clamp_u8(yi + ((116130 * cb + 32768) >> 16));
            (r, g, b)
        }
    }
}

/// Converts a macropixel pair `(y0, u, y1, v)` to two RGB `(r, g, b)` pixels
/// using [`YuvRange::Bt601Limited`], fetching table rows for `u`/`v` once.
#[must_use]
#[inline]
pub fn yuv422_pair_to_rgb(y0: u8, u: u8, y1: u8, v: u8) -> ((u8, u8, u8), (u8, u8, u8)) {
    yuv422_pair_to_rgb_with_range(y0, u, y1, v, YuvRange::Bt601Limited)
}

/// Converts a macropixel pair `(y0, u, y1, v)` to two RGB `(r, g, b)` pixels
/// sharing chroma `u` and `v` under the specified [`YuvRange`].
#[must_use]
#[inline]
pub fn yuv422_pair_to_rgb_with_range(
    y0: u8,
    u: u8,
    y1: u8,
    v: u8,
    range: YuvRange,
) -> ((u8, u8, u8), (u8, u8, u8)) {
    match range {
        YuvRange::Bt601Limited => {
            let u_row = YUV_TABLE[u as usize];
            let v_row = YUV_TABLE[v as usize];
            let uv_g = u_row.v_vg + v_row.v_ug;

            let y0_row = YUV_TABLE[y0 as usize];
            let r0 = (y0_row.v_y + v_row.v_vr).clamp(0, 255) as u8;
            let g0 = (y0_row.v_y + uv_g).clamp(0, 255) as u8;
            let b0 = (y0_row.v_y + u_row.v_ub).clamp(0, 255) as u8;

            let y1_row = YUV_TABLE[y1 as usize];
            let r1 = (y1_row.v_y + v_row.v_vr).clamp(0, 255) as u8;
            let g1 = (y1_row.v_y + uv_g).clamp(0, 255) as u8;
            let b1 = (y1_row.v_y + u_row.v_ub).clamp(0, 255) as u8;

            ((r0, g0, b0), (r1, g1, b1))
        }
        YuvRange::Full => {
            let cb = i32::from(u) - 128;
            let cr = i32::from(v) - 128;
            let r_off = (91881 * cr + 32768) >> 16;
            let g_off = (22554 * cb + 46802 * cr + 32768) >> 16;
            let b_off = (116130 * cb + 32768) >> 16;

            let yi0 = i32::from(y0);
            let r0 = clamp_u8(yi0 + r_off);
            let g0 = clamp_u8(yi0 - g_off);
            let b0 = clamp_u8(yi0 + b_off);

            let yi1 = i32::from(y1);
            let r1 = clamp_u8(yi1 + r_off);
            let g1 = clamp_u8(yi1 - g_off);
            let b1 = clamp_u8(yi1 + b_off);

            ((r0, g0, b0), (r1, g1, b1))
        }
    }
}

#[inline]
const fn clamp_u8(v: i32) -> u8 {
    if v < 0 {
        0
    } else if v > 255 {
        255
    } else {
        v as u8
    }
}

/// Converts a single RGB888 pixel `(r, g, b)` to ITU-R BT.601 `(y, u, v)`
/// using the specified [`YuvRange`].
#[must_use]
#[inline]
pub const fn rgb2yuv_with_range(r: u8, g: u8, b: u8, range: YuvRange) -> (u8, u8, u8) {
    let r = r as i32;
    let g = g as i32;
    let b = b as i32;

    match range {
        YuvRange::Full => {
            let y = clamp_u8((19595 * r + 38470 * g + 7471 * b + 32768) >> 16);
            let u = clamp_u8(128 + ((-11059 * r - 21709 * g + 32768 * b + 32768) >> 16));
            let v = clamp_u8(128 + ((32768 * r - 27439 * g - 5329 * b + 32768) >> 16));
            (y, u, v)
        }
        YuvRange::Bt601Limited => {
            let y = clamp_u8(16 + ((16829 * r + 33039 * g + 6416 * b + 32768) >> 16));
            let u = clamp_u8(128 + ((-9714 * r - 19070 * g + 28784 * b + 32768) >> 16));
            let v = clamp_u8(128 + ((28784 * r - 24103 * g - 4681 * b + 32768) >> 16));
            (y, u, v)
        }
    }
}

/// Converts a single RGB888 pixel `(r, g, b)` to full-range JFIF `(y, cb, cr)`.
#[inline]
pub(crate) const fn rgb2yuv(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
    rgb2yuv_with_range(r, g, b, YuvRange::Full)
}

/// Unpacks a big-endian RGB565 pixel (`[b0, b1]`) into full-range 8-bit RGB888
/// `(r, g, b)` using bit-replication (`LOGIC-063`, `CONV-15`).
///
/// Maps 5-bit channels `[0..31]` to `[0..255]` via `(v << 3) | (v >> 2)` and
/// 6-bit green `[0..63]` to `[0..255]` via `(v << 2) | (v >> 4)`.
#[must_use]
#[inline]
pub const fn unpack_rgb565_be_full(b0: u8, b1: u8) -> (u8, u8, u8) {
    let r5 = b0 >> 3;
    let r = (r5 << 3) | (r5 >> 2);

    let g6 = ((b0 & 0x07) << 3) | (b1 >> 5);
    let g = (g6 << 2) | (g6 >> 4);

    let b5 = b1 & 0x1F;
    let b = (b5 << 3) | (b5 >> 2);

    (r, g, b)
}

#[inline]
const fn div255_u16(n: u16) -> u8 {
    ((n + 1 + (n >> 8)) >> 8) as u8
}

#[inline]
const fn pack_rgb565_raw(r: u8, g: u8, b: u8) -> (u8, u8) {
    let r5 = div255_u16(r as u16 * 31 + 127);
    let g6 = div255_u16(g as u16 * 63 + 127);
    let b5 = div255_u16(b as u16 * 31 + 127);
    let hb = (r5 << 3) | (g6 >> 3);
    let lb = ((g6 & 0x07) << 5) | b5;
    (hb, lb)
}

fn checked_pixel_count(width: u16, height: u16, max_bpp: u32) -> Result<usize, CameraError> {
    if width == 0 || height == 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels32 = (width as u32)
        .checked_mul(height as u32)
        .ok_or(CameraError::Overflow)?;
    let bytes32 = pixels32.checked_mul(max_bpp).ok_or(CameraError::Overflow)?;
    if bytes32 > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }
    usize::try_from(pixels32).map_err(|_| CameraError::Overflow)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into
/// 8-bit grayscale by extracting `Y0` and `Y1` without inner bounds
/// checks (`LOGIC-061`).
///
/// Returns the number of bytes written to `dst` (`width * height`).
pub fn yuv422_to_grayscale(
    src: &[u8],
    width: u16,
    height: u16,
    dst: &mut [u8],
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let dst_len = pixels;
    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..src_len].as_chunks::<4>().0;
    let dst_chunks = dst[..dst_len].as_chunks_mut::<2>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        *out = [chunk[0], chunk[2]];
    }

    Ok(dst_len)
}

/// Converts a big-endian RGB565 buffer into packed studio-range YUV422
/// (YUYV: `[Y0, U, Y1, V]`) using [`YuvRange::Bt601Limited`].
///
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn rgb565_to_yuv422(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    rgb565_to_yuv422_with_range(src, dst, width, height, YuvRange::Bt601Limited)
}

/// Converts a big-endian RGB565 buffer into packed YUV422
/// (YUYV: `[Y0, U, Y1, V]`) using the specified [`YuvRange`].
///
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn rgb565_to_yuv422_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..byte_len].as_chunks::<4>().0;
    let dst_chunks = dst[..byte_len].as_chunks_mut::<4>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        let (r0, g0, b0) = unpack_rgb565_be_full(chunk[0], chunk[1]);
        let (r1, g1, b1) = unpack_rgb565_be_full(chunk[2], chunk[3]);

        let (y0, u0, v0) = rgb2yuv_with_range(r0, g0, b0, range);
        let (y1, u1, v1) = rgb2yuv_with_range(r1, g1, b1, range);

        *out = [
            y0,
            ((u16::from(u0) + u16::from(u1)) / 2) as u8,
            y1,
            ((u16::from(v0) + u16::from(v1)) / 2) as u8,
        ];
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into RGB565 using
/// [`YuvRange::Bt601Limited`].
///
/// When `byte_swap` is `false`, writes big-endian RGB565 (`[hb, lb]`);
/// when `true`, swaps the byte order (`[lb, hb]`).
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn yuv422_to_rgb565(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    byte_swap: bool,
) -> Result<usize, CameraError> {
    yuv422_to_rgb565_with_range(src, dst, width, height, byte_swap, YuvRange::Bt601Limited)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into RGB565 using
/// the specified [`YuvRange`].
///
/// When `byte_swap` is `false`, writes big-endian RGB565 (`[hb, lb]`);
/// when `true`, swaps the byte order (`[lb, hb]`).
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn yuv422_to_rgb565_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    byte_swap: bool,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..byte_len].as_chunks::<4>().0;
    let dst_chunks = dst[..byte_len].as_chunks_mut::<4>().0;

    if byte_swap {
        for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
            let ((r0, g0, b0), (r1, g1, b1)) =
                yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
            let (hb0, lb0) = pack_rgb565_raw(r0, g0, b0);
            let (hb1, lb1) = pack_rgb565_raw(r1, g1, b1);
            *out = [lb0, hb0, lb1, hb1];
        }
    } else {
        for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
            let ((r0, g0, b0), (r1, g1, b1)) =
                yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
            let (hb0, lb0) = pack_rgb565_raw(r0, g0, b0);
            let (hb1, lb1) = pack_rgb565_raw(r1, g1, b1);
            *out = [hb0, lb0, hb1, lb1];
        }
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into 24-bit
/// RGB888 (`[R, G, B]`) using [`YuvRange::Bt601Limited`].
///
/// Returns the number of bytes written to `dst` (`width * height * 3`).
pub fn yuv422_to_rgb888(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    yuv422_to_rgb888_with_range(src, dst, width, height, YuvRange::Bt601Limited)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into 24-bit
/// RGB888 (`[R, G, B]`) using the specified [`YuvRange`].
///
/// Returns the number of bytes written to `dst` (`width * height * 3`).
pub fn yuv422_to_rgb888_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 3)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let dst_len = pixels.checked_mul(3).ok_or(CameraError::Overflow)?;
    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..src_len].as_chunks::<4>().0;
    let dst_chunks = dst[..dst_len].as_chunks_mut::<6>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        let ((r0, g0, b0), (r1, g1, b1)) =
            yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
        *out = [r0, g0, b0, r1, g1, b1];
    }

    Ok(dst_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into planar
/// YUV420 (I420).
///
/// Requires `width` and `height` to be non-zero and even
/// (`width % 2 == 0`, `height % 2 == 0`).
/// Returns the number of bytes written to `dst` (`(width * height * 3) / 2`).
pub fn yuv422_to_yuv420(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    if width == 0 || height == 0 || (width & 1) != 0 || (height & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels = checked_pixel_count(width, height, 2)?;
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let chroma_plane_len = pixels / 4;
    let dst_len = pixels
        .checked_add(chroma_plane_len * 2)
        .ok_or(CameraError::Overflow)?;

    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let w = width as usize;
    let half_w = w / 2;
    let src_row_bytes = w * 2;

    let (y_plane, uv_planes) = dst[..dst_len].split_at_mut(pixels);
    let (u_plane, v_plane) = uv_planes.split_at_mut(chroma_plane_len);

    let row_iter = src[..src_len]
        .chunks_exact(src_row_bytes * 2)
        .zip(y_plane.chunks_exact_mut(w * 2))
        .zip(u_plane.chunks_exact_mut(half_w))
        .zip(v_plane.chunks_exact_mut(half_w));

    for (((src_pair, y_pair), u_row), v_row) in row_iter {
        let (src_r0, src_r1) = src_pair.split_at(src_row_bytes);
        let (y_r0, y_r1) = y_pair.split_at_mut(w);

        let block_iter = src_r0
            .as_chunks::<4>()
            .0
            .iter()
            .zip(src_r1.as_chunks::<4>().0.iter())
            .zip(y_r0.as_chunks_mut::<2>().0.iter_mut())
            .zip(y_r1.as_chunks_mut::<2>().0.iter_mut())
            .zip(u_row.iter_mut())
            .zip(v_row.iter_mut());

        for (((((&[y00, u0, y01, v0], &[y10, u1, y11, v1]), y0_out), y1_out), u_out), v_out) in
            block_iter
        {
            *y0_out = [y00, y01];
            *y1_out = [y10, y11];
            *u_out = ((u16::from(u0) + u16::from(u1)) / 2) as u8;
            *v_out = ((u16::from(v0) + u16::from(v1)) / 2) as u8;
        }
    }

    Ok(dst_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yuv422_to_yuv420_indexed_ref(src: &[u8], dst: &mut [u8], width: u16, height: u16) {
        let w = width as usize;
        let h = height as usize;
        let pixels = w * h;
        let u_offset = pixels;
        let v_offset = pixels + pixels / 4;

        for by in 0..(h / 2) {
            let row0 = by * 2;
            let row1 = row0 + 1;
            for bx in 0..(w / 2) {
                let x0 = bx * 2;
                let s0 = (row0 * w + x0) * 2;
                let s1 = (row1 * w + x0) * 2;

                dst[row0 * w + x0] = src[s0];
                dst[row0 * w + x0 + 1] = src[s0 + 2];
                dst[row1 * w + x0] = src[s1];
                dst[row1 * w + x0 + 1] = src[s1 + 2];

                let uv_idx = by * (w / 2) + bx;
                dst[u_offset + uv_idx] = ((src[s0 + 1] as u16 + src[s1 + 1] as u16) / 2) as u8;
                dst[v_offset + uv_idx] = ((src[s0 + 3] as u16 + src[s1 + 3] as u16) / 2) as u8;
            }
        }
    }

    #[test]
    fn test_yuv2rgb_bt601_table_values() {
        // Neutral black in BT.601 (Y=16, U=128, V=128) -> (0, 0, 0)
        assert_eq!(yuv2rgb(16, 128, 128), (0, 0, 0));
        // Neutral mid-gray (Y=128, U=128, V=128) -> (130, 130, 130) per
        // yuv_table[128].vY = 130
        assert_eq!(yuv2rgb(128, 128, 128), (130, 130, 130));
        // Clamping at extremes
        assert_eq!(yuv2rgb(0, 0, 0), (0, 136, 0));
        assert_eq!(yuv2rgb(255, 255, 255), (255, 126, 255));
    }

    #[test]
    fn test_conv12_yuv_table_size_and_all_24bit_inputs() {
        assert_eq!(
            core::mem::size_of::<YuvTableRow>(),
            10,
            "YuvTableRow should be 10 bytes without align(16) padding"
        );
        assert_eq!(
            core::mem::size_of_val(&YUV_TABLE),
            2560,
            "YUV_TABLE should occupy 2560 bytes instead of 4096"
        );

        let mut vy = [0i16; 256];
        let mut vvr = [0i16; 256];
        let mut vvg = [0i16; 256];
        let mut vug = [0i16; 256];
        let mut vub = [0i16; 256];
        for i in 0..256 {
            vy[i] = YUV_TABLE[i].v_y;
            vvr[i] = YUV_TABLE[i].v_vr;
            vvg[i] = YUV_TABLE[i].v_vg;
            vug[i] = YUV_TABLE[i].v_ug;
            vub[i] = YUV_TABLE[i].v_ub;
        }

        // Exhaustively verify all 2^24 (y, u, v) combinations against split
        // channel tables.
        for u in 0..=255u8 {
            let ub = vub[u as usize];
            let ug_part = vvg[u as usize];
            for v in 0..=255u8 {
                let vr = vvr[v as usize];
                let g_chroma = ug_part + vug[v as usize];
                for y in 0..=255u8 {
                    let yv = vy[y as usize];
                    let expected = (
                        (yv + vr).clamp(0, 255) as u8,
                        (yv + g_chroma).clamp(0, 255) as u8,
                        (yv + ub).clamp(0, 255) as u8,
                    );
                    assert_eq!(yuv2rgb(y, u, v), expected);
                }
            }
        }
    }

    #[test]
    fn test_conv09_yuv_range_and_all_65536_rgb565_roundtrip() {
        assert_eq!(YuvRange::default(), YuvRange::Bt601Limited);

        // Full-range endpoints: black -> Y=0, white -> Y=255
        assert_eq!(rgb2yuv_with_range(0, 0, 0, YuvRange::Full), (0, 128, 128));
        assert_eq!(
            rgb2yuv_with_range(255, 255, 255, YuvRange::Full),
            (255, 128, 128)
        );
        assert_eq!(yuv2rgb_with_range(0, 128, 128, YuvRange::Full), (0, 0, 0));
        assert_eq!(
            yuv2rgb_with_range(255, 128, 128, YuvRange::Full),
            (255, 255, 255)
        );

        // Studio-range endpoints: black -> Y=16, white -> Y=235 (v_y=254 in yuv.c)
        assert_eq!(
            rgb2yuv_with_range(0, 0, 0, YuvRange::Bt601Limited),
            (16, 128, 128)
        );
        assert_eq!(
            rgb2yuv_with_range(255, 255, 255, YuvRange::Bt601Limited),
            (235, 128, 128)
        );
        assert_eq!(
            yuv2rgb_with_range(16, 128, 128, YuvRange::Bt601Limited),
            (0, 0, 0)
        );
        assert_eq!(
            yuv2rgb_with_range(235, 128, 128, YuvRange::Bt601Limited),
            (254, 254, 254)
        );

        // Exhaustive RGB565 -> YUV422 -> RGB565 roundtrip within +/-8 across
        // all 65,536 colors for both Bt601Limited and Full ranges.
        for range in [YuvRange::Bt601Limited, YuvRange::Full] {
            let mut yuv_buf = [0u8; 4];
            let mut out_buf = [0u8; 4];
            for code in 0..=u16::MAX {
                let [hb, lb] = code.to_be_bytes();
                let rgb565_pair = [hb, lb, hb, lb];
                let (r_in, g_in, b_in) = unpack_rgb565_be_full(hb, lb);

                rgb565_to_yuv422_with_range(&rgb565_pair, &mut yuv_buf, 2, 1, range)
                    .expect("rgb565_to_yuv422_with_range should succeed");
                yuv422_to_rgb565_with_range(&yuv_buf, &mut out_buf, 2, 1, false, range)
                    .expect("yuv422_to_rgb565_with_range should succeed");

                let (r_out, g_out, b_out) = unpack_rgb565_be_full(out_buf[0], out_buf[1]);
                let dr = (i32::from(r_out) - i32::from(r_in)).abs();
                let dg = (i32::from(g_out) - i32::from(g_in)).abs();
                let db = (i32::from(b_out) - i32::from(b_in)).abs();

                assert!(
                    dr <= 8 && dg <= 8 && db <= 8,
                    "range={range:?} code={code:#06x} in=({r_in},{g_in},{b_in}) \
                     out=({r_out},{g_out},{b_out})"
                );
                if range == YuvRange::Full {
                    assert_eq!((out_buf[0], out_buf[1]), (hb, lb));
                }
            }
        }
    }

    #[test]
    fn test_conv15_unpack_rgb565_be_full_bit_exactness() {
        assert_eq!(unpack_rgb565_be_full(0x00, 0x00), (0, 0, 0));
        assert_eq!(unpack_rgb565_be_full(0xFF, 0xFF), (255, 255, 255));
        assert_eq!(unpack_rgb565_be_full(0xF8, 0x00), (255, 0, 0));
        assert_eq!(unpack_rgb565_be_full(0x07, 0xE0), (0, 255, 0));
        assert_eq!(unpack_rgb565_be_full(0x00, 0x1F), (0, 0, 255));

        for code in 0..=u16::MAX {
            let [hb, lb] = code.to_be_bytes();
            let (r, g, b) = unpack_rgb565_be_full(hb, lb);
            assert_eq!(pack_rgb565_raw(r, g, b), (hb, lb));
        }

        // Verify rgb565_to_yuv422 uses full bit-replicated white (255,255,255)
        let white_pair = [0xFFu8; 4];
        let mut yuv_lim = [0u8; 4];
        let mut yuv_full = [0u8; 4];
        rgb565_to_yuv422(&white_pair, &mut yuv_lim, 2, 1)
            .expect("studio rgb565_to_yuv422 should succeed");
        rgb565_to_yuv422_with_range(&white_pair, &mut yuv_full, 2, 1, YuvRange::Full)
            .expect("full rgb565_to_yuv422_with_range should succeed");
        assert_eq!(yuv_lim, [235, 128, 235, 128]);
        assert_eq!(yuv_full, [255, 128, 255, 128]);
    }

    #[test]
    fn test_conv19_yuv422_to_yuv420_differential_64x32() {
        let width = 64u16;
        let height = 32u16;
        let mut src = [0u8; 64 * 32 * 2];
        let mut state = 0x9E37_79B9u32;
        for byte in &mut src {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            *byte = (state & 0xFF) as u8;
        }

        let mut dst_actual = [0u8; (64 * 32 * 3) / 2];
        let mut dst_expected = [0u8; (64 * 32 * 3) / 2];

        let written = yuv422_to_yuv420(&src, &mut dst_actual, width, height)
            .expect("64x32 yuv422_to_yuv420 should succeed");
        yuv422_to_yuv420_indexed_ref(&src, &mut dst_expected, width, height);

        assert_eq!(written, dst_expected.len());
        assert_eq!(dst_actual, dst_expected);

        // Reject odd height and undersized destination
        assert_eq!(
            yuv422_to_yuv420(&src[..24], &mut dst_actual, 4, 3),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_yuv420(&src[..16], &mut dst_actual[..5], 2, 2),
            Err(CameraError::BufferTooSmall)
        );
    }

    #[test]
    fn test_rgb565_yuv422_roundtrip_and_byte_swap() {
        let width = 4u16;
        let height = 2u16;
        // 8 pixels of solid white in RGB565 (0xFFFF)
        let rgb565_in = [0xFFu8; 16];
        let mut yuv422 = [0u8; 16];
        let mut rgb565_be = [0u8; 16];
        let mut rgb565_le = [0u8; 16];

        let n_yuv = rgb565_to_yuv422(&rgb565_in, &mut yuv422, width, height)
            .expect("rgb565_to_yuv422 should succeed");
        assert_eq!(n_yuv, 16);

        let n_be = yuv422_to_rgb565(&yuv422, &mut rgb565_be, width, height, false)
            .expect("yuv422_to_rgb565 (BE) should succeed");
        let n_le = yuv422_to_rgb565(&yuv422, &mut rgb565_le, width, height, true)
            .expect("yuv422_to_rgb565 (LE) should succeed");
        assert_eq!(n_be, 16);
        assert_eq!(n_le, 16);
        assert_eq!(rgb565_be, [0xFFu8; 16]);

        for i in 0..8 {
            assert_eq!(rgb565_be[i * 2], rgb565_le[i * 2 + 1]);
            assert_eq!(rgb565_be[i * 2 + 1], rgb565_le[i * 2]);
        }
    }

    #[test]
    fn test_yuv422_to_rgb888_and_yuv420() {
        let width = 2u16;
        let height = 2u16;
        // 4 pixels (2x2), Y=128, U=128, V=128
        let yuv422 = [128, 128, 128, 128, 128, 128, 128, 128];
        let mut rgb888 = [0u8; 12];
        let written_rgb = yuv422_to_rgb888(&yuv422, &mut rgb888, width, height)
            .expect("yuv422_to_rgb888 should succeed");
        assert_eq!(written_rgb, 12);
        assert_eq!(rgb888, [130u8; 12]);

        let written_rgb_full =
            yuv422_to_rgb888_with_range(&yuv422, &mut rgb888, width, height, YuvRange::Full)
                .expect("yuv422_to_rgb888_with_range(Full) should succeed");
        assert_eq!(written_rgb_full, 12);
        assert_eq!(rgb888, [128u8; 12]);

        let mut yuv420 = [0u8; 6];
        let written_420 = yuv422_to_yuv420(&yuv422, &mut yuv420, width, height)
            .expect("yuv422_to_yuv420 should succeed");
        assert_eq!(written_420, 6);
        assert_eq!(yuv420, [128u8; 6]);
    }

    #[test]
    fn test_yuv_bounds_and_overflow_rejection() {
        let mut dst = [0u8; 16];
        assert_eq!(
            rgb565_to_yuv422(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            yuv422_to_rgb888(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            yuv422_to_yuv420(&[0u8; 8], &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb888(&[0u8; 8], &mut dst[..4], 2, 2),
            Err(CameraError::BufferTooSmall)
        );
    }

    #[test]
    fn test_audit_07_yuv_001_odd_width_rejected() {
        let mut dst = [0u8; 32];
        let src = [0u8; 32];
        assert_eq!(
            rgb565_to_yuv422(&src, &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb565(&src, &mut dst, 3, 2, false),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb888(&src, &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
    }
}
