//! Safe `#![no_std]` ITU-R BT.601 YUV/RGB color-space conversions.
//!
//! Replaces `conversions/yuv.c` and software YUV422/RGB565/RGB888/YUV420
//! conversion paths across `conversions/to_bmp.c` and `conversions/to_jpg.cpp`.

use esp32_cam_types::CameraError;

#[derive(Clone, Copy)]
struct YuvTableRow {
    v_y: i16,
    v_vr: i16,
    v_vg: i16,
    v_ug: i16,
    v_ub: i16,
}

/// Exact 256-entry lookup table from `conversions/yuv.c:25-283`.
static YUV_TABLE: [YuvTableRow; 256] = [
    YuvTableRow { v_y: -18, v_vr: -204, v_vg: 50, v_ug: 104, v_ub: -258 }, // 0
    YuvTableRow { v_y: -17, v_vr: -202, v_vg: 49, v_ug: 103, v_ub: -256 }, // 1
    YuvTableRow { v_y: -16, v_vr: -201, v_vg: 49, v_ug: 102, v_ub: -254 }, // 2
    YuvTableRow { v_y: -15, v_vr: -199, v_vg: 48, v_ug: 101, v_ub: -252 }, // 3
    YuvTableRow { v_y: -13, v_vr: -197, v_vg: 48, v_ug: 100, v_ub: -250 }, // 4
    YuvTableRow { v_y: -12, v_vr: -196, v_vg: 48, v_ug: 99, v_ub: -248 },  // 5
    YuvTableRow { v_y: -11, v_vr: -194, v_vg: 47, v_ug: 99, v_ub: -246 },  // 6
    YuvTableRow { v_y: -10, v_vr: -193, v_vg: 47, v_ug: 98, v_ub: -244 },  // 7
    YuvTableRow { v_y: -9, v_vr: -191, v_vg: 46, v_ug: 97, v_ub: -242 },   // 8
    YuvTableRow { v_y: -8, v_vr: -189, v_vg: 46, v_ug: 96, v_ub: -240 },   // 9
    YuvTableRow { v_y: -6, v_vr: -188, v_vg: 46, v_ug: 95, v_ub: -238 },   // 10
    YuvTableRow { v_y: -5, v_vr: -186, v_vg: 45, v_ug: 95, v_ub: -236 },   // 11
    YuvTableRow { v_y: -4, v_vr: -185, v_vg: 45, v_ug: 94, v_ub: -234 },   // 12
    YuvTableRow { v_y: -3, v_vr: -183, v_vg: 44, v_ug: 93, v_ub: -232 },   // 13
    YuvTableRow { v_y: -2, v_vr: -181, v_vg: 44, v_ug: 92, v_ub: -230 },   // 14
    YuvTableRow { v_y: -1, v_vr: -180, v_vg: 44, v_ug: 91, v_ub: -228 },   // 15
    YuvTableRow { v_y: 0, v_vr: -178, v_vg: 43, v_ug: 91, v_ub: -226 },    // 16
    YuvTableRow { v_y: 1, v_vr: -177, v_vg: 43, v_ug: 90, v_ub: -223 },    // 17
    YuvTableRow { v_y: 2, v_vr: -175, v_vg: 43, v_ug: 89, v_ub: -221 },    // 18
    YuvTableRow { v_y: 3, v_vr: -173, v_vg: 42, v_ug: 88, v_ub: -219 },    // 19
    YuvTableRow { v_y: 4, v_vr: -172, v_vg: 42, v_ug: 87, v_ub: -217 },    // 20
    YuvTableRow { v_y: 5, v_vr: -170, v_vg: 41, v_ug: 86, v_ub: -215 },    // 21
    YuvTableRow { v_y: 6, v_vr: -169, v_vg: 41, v_ug: 86, v_ub: -213 },    // 22
    YuvTableRow { v_y: 8, v_vr: -167, v_vg: 41, v_ug: 85, v_ub: -211 },    // 23
    YuvTableRow { v_y: 9, v_vr: -165, v_vg: 40, v_ug: 84, v_ub: -209 },    // 24
    YuvTableRow { v_y: 10, v_vr: -164, v_vg: 40, v_ug: 83, v_ub: -207 },   // 25
    YuvTableRow { v_y: 11, v_vr: -162, v_vg: 39, v_ug: 82, v_ub: -205 },   // 26
    YuvTableRow { v_y: 12, v_vr: -161, v_vg: 39, v_ug: 82, v_ub: -203 },   // 27
    YuvTableRow { v_y: 13, v_vr: -159, v_vg: 39, v_ug: 81, v_ub: -201 },   // 28
    YuvTableRow { v_y: 15, v_vr: -158, v_vg: 38, v_ug: 80, v_ub: -199 },   // 29
    YuvTableRow { v_y: 16, v_vr: -156, v_vg: 38, v_ug: 79, v_ub: -197 },   // 30
    YuvTableRow { v_y: 17, v_vr: -154, v_vg: 37, v_ug: 78, v_ub: -195 },   // 31
    YuvTableRow { v_y: 18, v_vr: -153, v_vg: 37, v_ug: 78, v_ub: -193 },   // 32
    YuvTableRow { v_y: 19, v_vr: -151, v_vg: 37, v_ug: 77, v_ub: -191 },   // 33
    YuvTableRow { v_y: 20, v_vr: -150, v_vg: 36, v_ug: 76, v_ub: -189 },   // 34
    YuvTableRow { v_y: 22, v_vr: -148, v_vg: 36, v_ug: 75, v_ub: -187 },   // 35
    YuvTableRow { v_y: 23, v_vr: -146, v_vg: 35, v_ug: 74, v_ub: -185 },   // 36
    YuvTableRow { v_y: 24, v_vr: -145, v_vg: 35, v_ug: 73, v_ub: -183 },   // 37
    YuvTableRow { v_y: 25, v_vr: -143, v_vg: 35, v_ug: 73, v_ub: -181 },   // 38
    YuvTableRow { v_y: 26, v_vr: -142, v_vg: 34, v_ug: 72, v_ub: -179 },   // 39
    YuvTableRow { v_y: 27, v_vr: -140, v_vg: 34, v_ug: 71, v_ub: -177 },   // 40
    YuvTableRow { v_y: 29, v_vr: -138, v_vg: 34, v_ug: 70, v_ub: -175 },   // 41
    YuvTableRow { v_y: 30, v_vr: -137, v_vg: 33, v_ug: 69, v_ub: -173 },   // 42
    YuvTableRow { v_y: 31, v_vr: -135, v_vg: 33, v_ug: 69, v_ub: -171 },   // 43
    YuvTableRow { v_y: 32, v_vr: -134, v_vg: 32, v_ug: 68, v_ub: -169 },   // 44
    YuvTableRow { v_y: 33, v_vr: -132, v_vg: 32, v_ug: 67, v_ub: -167 },   // 45
    YuvTableRow { v_y: 34, v_vr: -130, v_vg: 32, v_ug: 66, v_ub: -165 },   // 46
    YuvTableRow { v_y: 36, v_vr: -129, v_vg: 31, v_ug: 65, v_ub: -163 },   // 47
    YuvTableRow { v_y: 37, v_vr: -127, v_vg: 31, v_ug: 65, v_ub: -161 },   // 48
    YuvTableRow { v_y: 38, v_vr: -126, v_vg: 30, v_ug: 64, v_ub: -159 },   // 49
    YuvTableRow { v_y: 39, v_vr: -124, v_vg: 30, v_ug: 63, v_ub: -157 },   // 50
    YuvTableRow { v_y: 40, v_vr: -122, v_vg: 30, v_ug: 62, v_ub: -155 },   // 51
    YuvTableRow { v_y: 41, v_vr: -121, v_vg: 29, v_ug: 61, v_ub: -153 },   // 52
    YuvTableRow { v_y: 43, v_vr: -119, v_vg: 29, v_ug: 60, v_ub: -151 },   // 53
    YuvTableRow { v_y: 44, v_vr: -118, v_vg: 28, v_ug: 60, v_ub: -149 },   // 54
    YuvTableRow { v_y: 45, v_vr: -116, v_vg: 28, v_ug: 59, v_ub: -147 },   // 55
    YuvTableRow { v_y: 46, v_vr: -114, v_vg: 28, v_ug: 58, v_ub: -145 },   // 56
    YuvTableRow { v_y: 47, v_vr: -113, v_vg: 27, v_ug: 57, v_ub: -143 },   // 57
    YuvTableRow { v_y: 48, v_vr: -111, v_vg: 27, v_ug: 56, v_ub: -141 },   // 58
    YuvTableRow { v_y: 50, v_vr: -110, v_vg: 26, v_ug: 56, v_ub: -139 },   // 59
    YuvTableRow { v_y: 51, v_vr: -108, v_vg: 26, v_ug: 55, v_ub: -137 },   // 60
    YuvTableRow { v_y: 52, v_vr: -106, v_vg: 26, v_ug: 54, v_ub: -135 },   // 61
    YuvTableRow { v_y: 53, v_vr: -105, v_vg: 25, v_ug: 53, v_ub: -133 },   // 62
    YuvTableRow { v_y: 54, v_vr: -103, v_vg: 25, v_ug: 52, v_ub: -131 },   // 63
    YuvTableRow { v_y: 55, v_vr: -102, v_vg: 25, v_ug: 52, v_ub: -129 },   // 64
    YuvTableRow { v_y: 57, v_vr: -100, v_vg: 24, v_ug: 51, v_ub: -127 },   // 65
    YuvTableRow { v_y: 58, v_vr: -98, v_vg: 24, v_ug: 50, v_ub: -125 },    // 66
    YuvTableRow { v_y: 59, v_vr: -97, v_vg: 23, v_ug: 49, v_ub: -123 },    // 67
    YuvTableRow { v_y: 60, v_vr: -95, v_vg: 23, v_ug: 48, v_ub: -121 },    // 68
    YuvTableRow { v_y: 61, v_vr: -94, v_vg: 23, v_ug: 47, v_ub: -119 },    // 69
    YuvTableRow { v_y: 62, v_vr: -92, v_vg: 22, v_ug: 47, v_ub: -117 },    // 70
    YuvTableRow { v_y: 64, v_vr: -90, v_vg: 22, v_ug: 46, v_ub: -115 },    // 71
    YuvTableRow { v_y: 65, v_vr: -89, v_vg: 21, v_ug: 45, v_ub: -113 },    // 72
    YuvTableRow { v_y: 66, v_vr: -87, v_vg: 21, v_ug: 44, v_ub: -110 },    // 73
    YuvTableRow { v_y: 67, v_vr: -86, v_vg: 21, v_ug: 43, v_ub: -108 },    // 74
    YuvTableRow { v_y: 68, v_vr: -84, v_vg: 20, v_ug: 43, v_ub: -106 },    // 75
    YuvTableRow { v_y: 69, v_vr: -82, v_vg: 20, v_ug: 42, v_ub: -104 },    // 76
    YuvTableRow { v_y: 71, v_vr: -81, v_vg: 19, v_ug: 41, v_ub: -102 },    // 77
    YuvTableRow { v_y: 72, v_vr: -79, v_vg: 19, v_ug: 40, v_ub: -100 },    // 78
    YuvTableRow { v_y: 73, v_vr: -78, v_vg: 19, v_ug: 39, v_ub: -98 },     // 79
    YuvTableRow { v_y: 74, v_vr: -76, v_vg: 18, v_ug: 39, v_ub: -96 },     // 80
    YuvTableRow { v_y: 75, v_vr: -75, v_vg: 18, v_ug: 38, v_ub: -94 },     // 81
    YuvTableRow { v_y: 76, v_vr: -73, v_vg: 17, v_ug: 37, v_ub: -92 },     // 82
    YuvTableRow { v_y: 77, v_vr: -71, v_vg: 17, v_ug: 36, v_ub: -90 },     // 83
    YuvTableRow { v_y: 79, v_vr: -70, v_vg: 17, v_ug: 35, v_ub: -88 },     // 84
    YuvTableRow { v_y: 80, v_vr: -68, v_vg: 16, v_ug: 34, v_ub: -86 },     // 85
    YuvTableRow { v_y: 81, v_vr: -67, v_vg: 16, v_ug: 34, v_ub: -84 },     // 86
    YuvTableRow { v_y: 82, v_vr: -65, v_vg: 16, v_ug: 33, v_ub: -82 },     // 87
    YuvTableRow { v_y: 83, v_vr: -63, v_vg: 15, v_ug: 32, v_ub: -80 },     // 88
    YuvTableRow { v_y: 84, v_vr: -62, v_vg: 15, v_ug: 31, v_ub: -78 },     // 89
    YuvTableRow { v_y: 86, v_vr: -60, v_vg: 14, v_ug: 30, v_ub: -76 },     // 90
    YuvTableRow { v_y: 87, v_vr: -59, v_vg: 14, v_ug: 30, v_ub: -74 },     // 91
    YuvTableRow { v_y: 88, v_vr: -57, v_vg: 14, v_ug: 29, v_ub: -72 },     // 92
    YuvTableRow { v_y: 89, v_vr: -55, v_vg: 13, v_ug: 28, v_ub: -70 },     // 93
    YuvTableRow { v_y: 90, v_vr: -54, v_vg: 13, v_ug: 27, v_ub: -68 },     // 94
    YuvTableRow { v_y: 91, v_vr: -52, v_vg: 12, v_ug: 26, v_ub: -66 },     // 95
    YuvTableRow { v_y: 93, v_vr: -51, v_vg: 12, v_ug: 26, v_ub: -64 },     // 96
    YuvTableRow { v_y: 94, v_vr: -49, v_vg: 12, v_ug: 25, v_ub: -62 },     // 97
    YuvTableRow { v_y: 95, v_vr: -47, v_vg: 11, v_ug: 24, v_ub: -60 },     // 98
    YuvTableRow { v_y: 96, v_vr: -46, v_vg: 11, v_ug: 23, v_ub: -58 },     // 99
    YuvTableRow { v_y: 97, v_vr: -44, v_vg: 10, v_ug: 22, v_ub: -56 },     // 100
    YuvTableRow { v_y: 98, v_vr: -43, v_vg: 10, v_ug: 21, v_ub: -54 },     // 101
    YuvTableRow { v_y: 100, v_vr: -41, v_vg: 10, v_ug: 21, v_ub: -52 },    // 102
    YuvTableRow { v_y: 101, v_vr: -39, v_vg: 9, v_ug: 20, v_ub: -50 },     // 103
    YuvTableRow { v_y: 102, v_vr: -38, v_vg: 9, v_ug: 19, v_ub: -48 },     // 104
    YuvTableRow { v_y: 103, v_vr: -36, v_vg: 8, v_ug: 18, v_ub: -46 },     // 105
    YuvTableRow { v_y: 104, v_vr: -35, v_vg: 8, v_ug: 17, v_ub: -44 },     // 106
    YuvTableRow { v_y: 105, v_vr: -33, v_vg: 8, v_ug: 17, v_ub: -42 },     // 107
    YuvTableRow { v_y: 107, v_vr: -31, v_vg: 7, v_ug: 16, v_ub: -40 },     // 108
    YuvTableRow { v_y: 108, v_vr: -30, v_vg: 7, v_ug: 15, v_ub: -38 },     // 109
    YuvTableRow { v_y: 109, v_vr: -28, v_vg: 7, v_ug: 14, v_ub: -36 },     // 110
    YuvTableRow { v_y: 110, v_vr: -27, v_vg: 6, v_ug: 13, v_ub: -34 },     // 111
    YuvTableRow { v_y: 111, v_vr: -25, v_vg: 6, v_ug: 13, v_ub: -32 },     // 112
    YuvTableRow { v_y: 112, v_vr: -23, v_vg: 5, v_ug: 12, v_ub: -30 },     // 113
    YuvTableRow { v_y: 114, v_vr: -22, v_vg: 5, v_ug: 11, v_ub: -28 },     // 114
    YuvTableRow { v_y: 115, v_vr: -20, v_vg: 5, v_ug: 10, v_ub: -26 },     // 115
    YuvTableRow { v_y: 116, v_vr: -19, v_vg: 4, v_ug: 9, v_ub: -24 },      // 116
    YuvTableRow { v_y: 117, v_vr: -17, v_vg: 4, v_ug: 8, v_ub: -22 },      // 117
    YuvTableRow { v_y: 118, v_vr: -15, v_vg: 3, v_ug: 8, v_ub: -20 },      // 118
    YuvTableRow { v_y: 119, v_vr: -14, v_vg: 3, v_ug: 7, v_ub: -18 },      // 119
    YuvTableRow { v_y: 121, v_vr: -12, v_vg: 3, v_ug: 6, v_ub: -16 },      // 120
    YuvTableRow { v_y: 122, v_vr: -11, v_vg: 2, v_ug: 5, v_ub: -14 },      // 121
    YuvTableRow { v_y: 123, v_vr: -9, v_vg: 2, v_ug: 4, v_ub: -12 },       // 122
    YuvTableRow { v_y: 124, v_vr: -7, v_vg: 1, v_ug: 4, v_ub: -10 },       // 123
    YuvTableRow { v_y: 125, v_vr: -6, v_vg: 1, v_ug: 3, v_ub: -8 },        // 124
    YuvTableRow { v_y: 126, v_vr: -4, v_vg: 1, v_ug: 2, v_ub: -6 },        // 125
    YuvTableRow { v_y: 128, v_vr: -3, v_vg: 0, v_ug: 1, v_ub: -4 },        // 126
    YuvTableRow { v_y: 129, v_vr: -1, v_vg: 0, v_ug: 0, v_ub: -2 },        // 127
    YuvTableRow { v_y: 130, v_vr: 0, v_vg: 0, v_ug: 0, v_ub: 0 },          // 128
    YuvTableRow { v_y: 131, v_vr: 1, v_vg: 0, v_ug: 0, v_ub: 2 },          // 129
    YuvTableRow { v_y: 132, v_vr: 3, v_vg: 0, v_ug: -1, v_ub: 4 },         // 130
    YuvTableRow { v_y: 133, v_vr: 4, v_vg: -1, v_ug: -2, v_ub: 6 },        // 131
    YuvTableRow { v_y: 135, v_vr: 6, v_vg: -1, v_ug: -3, v_ub: 8 },        // 132
    YuvTableRow { v_y: 136, v_vr: 7, v_vg: -1, v_ug: -4, v_ub: 10 },       // 133
    YuvTableRow { v_y: 137, v_vr: 9, v_vg: -2, v_ug: -4, v_ub: 12 },       // 134
    YuvTableRow { v_y: 138, v_vr: 11, v_vg: -2, v_ug: -5, v_ub: 14 },      // 135
    YuvTableRow { v_y: 139, v_vr: 12, v_vg: -3, v_ug: -6, v_ub: 16 },      // 136
    YuvTableRow { v_y: 140, v_vr: 14, v_vg: -3, v_ug: -7, v_ub: 18 },      // 137
    YuvTableRow { v_y: 142, v_vr: 15, v_vg: -3, v_ug: -8, v_ub: 20 },      // 138
    YuvTableRow { v_y: 143, v_vr: 17, v_vg: -4, v_ug: -8, v_ub: 22 },      // 139
    YuvTableRow { v_y: 144, v_vr: 19, v_vg: -4, v_ug: -9, v_ub: 24 },      // 140
    YuvTableRow { v_y: 145, v_vr: 20, v_vg: -5, v_ug: -10, v_ub: 26 },     // 141
    YuvTableRow { v_y: 146, v_vr: 22, v_vg: -5, v_ug: -11, v_ub: 28 },     // 142
    YuvTableRow { v_y: 147, v_vr: 23, v_vg: -5, v_ug: -12, v_ub: 30 },     // 143
    YuvTableRow { v_y: 148, v_vr: 25, v_vg: -6, v_ug: -13, v_ub: 32 },     // 144
    YuvTableRow { v_y: 150, v_vr: 27, v_vg: -6, v_ug: -13, v_ub: 34 },     // 145
    YuvTableRow { v_y: 151, v_vr: 28, v_vg: -7, v_ug: -14, v_ub: 36 },     // 146
    YuvTableRow { v_y: 152, v_vr: 30, v_vg: -7, v_ug: -15, v_ub: 38 },     // 147
    YuvTableRow { v_y: 153, v_vr: 31, v_vg: -7, v_ug: -16, v_ub: 40 },     // 148
    YuvTableRow { v_y: 154, v_vr: 33, v_vg: -8, v_ug: -17, v_ub: 42 },     // 149
    YuvTableRow { v_y: 155, v_vr: 35, v_vg: -8, v_ug: -17, v_ub: 44 },     // 150
    YuvTableRow { v_y: 157, v_vr: 36, v_vg: -8, v_ug: -18, v_ub: 46 },     // 151
    YuvTableRow { v_y: 158, v_vr: 38, v_vg: -9, v_ug: -19, v_ub: 48 },     // 152
    YuvTableRow { v_y: 159, v_vr: 39, v_vg: -9, v_ug: -20, v_ub: 50 },     // 153
    YuvTableRow { v_y: 160, v_vr: 41, v_vg: -10, v_ug: -21, v_ub: 52 },    // 154
    YuvTableRow { v_y: 161, v_vr: 43, v_vg: -10, v_ug: -21, v_ub: 54 },    // 155
    YuvTableRow { v_y: 162, v_vr: 44, v_vg: -10, v_ug: -22, v_ub: 56 },    // 156
    YuvTableRow { v_y: 164, v_vr: 46, v_vg: -11, v_ug: -23, v_ub: 58 },    // 157
    YuvTableRow { v_y: 165, v_vr: 47, v_vg: -11, v_ug: -24, v_ub: 60 },    // 158
    YuvTableRow { v_y: 166, v_vr: 49, v_vg: -12, v_ug: -25, v_ub: 62 },    // 159
    YuvTableRow { v_y: 167, v_vr: 51, v_vg: -12, v_ug: -26, v_ub: 64 },    // 160
    YuvTableRow { v_y: 168, v_vr: 52, v_vg: -12, v_ug: -26, v_ub: 66 },    // 161
    YuvTableRow { v_y: 169, v_vr: 54, v_vg: -13, v_ug: -27, v_ub: 68 },    // 162
    YuvTableRow { v_y: 171, v_vr: 55, v_vg: -13, v_ug: -28, v_ub: 70 },    // 163
    YuvTableRow { v_y: 172, v_vr: 57, v_vg: -14, v_ug: -29, v_ub: 72 },    // 164
    YuvTableRow { v_y: 173, v_vr: 59, v_vg: -14, v_ug: -30, v_ub: 74 },    // 165
    YuvTableRow { v_y: 174, v_vr: 60, v_vg: -14, v_ug: -30, v_ub: 76 },    // 166
    YuvTableRow { v_y: 175, v_vr: 62, v_vg: -15, v_ug: -31, v_ub: 78 },    // 167
    YuvTableRow { v_y: 176, v_vr: 63, v_vg: -15, v_ug: -32, v_ub: 80 },    // 168
    YuvTableRow { v_y: 178, v_vr: 65, v_vg: -16, v_ug: -33, v_ub: 82 },    // 169
    YuvTableRow { v_y: 179, v_vr: 67, v_vg: -16, v_ug: -34, v_ub: 84 },    // 170
    YuvTableRow { v_y: 180, v_vr: 68, v_vg: -16, v_ug: -34, v_ub: 86 },    // 171
    YuvTableRow { v_y: 181, v_vr: 70, v_vg: -17, v_ug: -35, v_ub: 88 },    // 172
    YuvTableRow { v_y: 182, v_vr: 71, v_vg: -17, v_ug: -36, v_ub: 90 },    // 173
    YuvTableRow { v_y: 183, v_vr: 73, v_vg: -17, v_ug: -37, v_ub: 92 },    // 174
    YuvTableRow { v_y: 185, v_vr: 75, v_vg: -18, v_ug: -38, v_ub: 94 },    // 175
    YuvTableRow { v_y: 186, v_vr: 76, v_vg: -18, v_ug: -39, v_ub: 96 },    // 176
    YuvTableRow { v_y: 187, v_vr: 78, v_vg: -19, v_ug: -39, v_ub: 98 },    // 177
    YuvTableRow { v_y: 188, v_vr: 79, v_vg: -19, v_ug: -40, v_ub: 100 },   // 178
    YuvTableRow { v_y: 189, v_vr: 81, v_vg: -19, v_ug: -41, v_ub: 102 },   // 179
    YuvTableRow { v_y: 190, v_vr: 82, v_vg: -20, v_ug: -42, v_ub: 104 },   // 180
    YuvTableRow { v_y: 192, v_vr: 84, v_vg: -20, v_ug: -43, v_ub: 106 },   // 181
    YuvTableRow { v_y: 193, v_vr: 86, v_vg: -21, v_ug: -43, v_ub: 108 },   // 182
    YuvTableRow { v_y: 194, v_vr: 87, v_vg: -21, v_ug: -44, v_ub: 110 },   // 183
    YuvTableRow { v_y: 195, v_vr: 89, v_vg: -21, v_ug: -45, v_ub: 113 },   // 184
    YuvTableRow { v_y: 196, v_vr: 90, v_vg: -22, v_ug: -46, v_ub: 115 },   // 185
    YuvTableRow { v_y: 197, v_vr: 92, v_vg: -22, v_ug: -47, v_ub: 117 },   // 186
    YuvTableRow { v_y: 199, v_vr: 94, v_vg: -23, v_ug: -47, v_ub: 119 },   // 187
    YuvTableRow { v_y: 200, v_vr: 95, v_vg: -23, v_ug: -48, v_ub: 121 },   // 188
    YuvTableRow { v_y: 201, v_vr: 97, v_vg: -23, v_ug: -49, v_ub: 123 },   // 189
    YuvTableRow { v_y: 202, v_vr: 98, v_vg: -24, v_ug: -50, v_ub: 125 },   // 190
    YuvTableRow { v_y: 203, v_vr: 100, v_vg: -24, v_ug: -51, v_ub: 127 },  // 191
    YuvTableRow { v_y: 204, v_vr: 102, v_vg: -25, v_ug: -52, v_ub: 129 },  // 192
    YuvTableRow { v_y: 206, v_vr: 103, v_vg: -25, v_ug: -52, v_ub: 131 },  // 193
    YuvTableRow { v_y: 207, v_vr: 105, v_vg: -25, v_ug: -53, v_ub: 133 },  // 194
    YuvTableRow { v_y: 208, v_vr: 106, v_vg: -26, v_ug: -54, v_ub: 135 },  // 195
    YuvTableRow { v_y: 209, v_vr: 108, v_vg: -26, v_ug: -55, v_ub: 137 },  // 196
    YuvTableRow { v_y: 210, v_vr: 110, v_vg: -26, v_ug: -56, v_ub: 139 },  // 197
    YuvTableRow { v_y: 211, v_vr: 111, v_vg: -27, v_ug: -56, v_ub: 141 },  // 198
    YuvTableRow { v_y: 213, v_vr: 113, v_vg: -27, v_ug: -57, v_ub: 143 },  // 199
    YuvTableRow { v_y: 214, v_vr: 114, v_vg: -28, v_ug: -58, v_ub: 145 },  // 200
    YuvTableRow { v_y: 215, v_vr: 116, v_vg: -28, v_ug: -59, v_ub: 147 },  // 201
    YuvTableRow { v_y: 216, v_vr: 118, v_vg: -28, v_ug: -60, v_ub: 149 },  // 202
    YuvTableRow { v_y: 217, v_vr: 119, v_vg: -29, v_ug: -60, v_ub: 151 },  // 203
    YuvTableRow { v_y: 218, v_vr: 121, v_vg: -29, v_ug: -61, v_ub: 153 },  // 204
    YuvTableRow { v_y: 219, v_vr: 122, v_vg: -30, v_ug: -62, v_ub: 155 },  // 205
    YuvTableRow { v_y: 221, v_vr: 124, v_vg: -30, v_ug: -63, v_ub: 157 },  // 206
    YuvTableRow { v_y: 222, v_vr: 126, v_vg: -30, v_ug: -64, v_ub: 159 },  // 207
    YuvTableRow { v_y: 223, v_vr: 127, v_vg: -31, v_ug: -65, v_ub: 161 },  // 208
    YuvTableRow { v_y: 224, v_vr: 129, v_vg: -31, v_ug: -65, v_ub: 163 },  // 209
    YuvTableRow { v_y: 225, v_vr: 130, v_vg: -32, v_ug: -66, v_ub: 165 },  // 210
    YuvTableRow { v_y: 226, v_vr: 132, v_vg: -32, v_ug: -67, v_ub: 167 },  // 211
    YuvTableRow { v_y: 228, v_vr: 134, v_vg: -32, v_ug: -68, v_ub: 169 },  // 212
    YuvTableRow { v_y: 229, v_vr: 135, v_vg: -33, v_ug: -69, v_ub: 171 },  // 213
    YuvTableRow { v_y: 230, v_vr: 137, v_vg: -33, v_ug: -69, v_ub: 173 },  // 214
    YuvTableRow { v_y: 231, v_vr: 138, v_vg: -34, v_ug: -70, v_ub: 175 },  // 215
    YuvTableRow { v_y: 232, v_vr: 140, v_vg: -34, v_ug: -71, v_ub: 177 },  // 216
    YuvTableRow { v_y: 233, v_vr: 142, v_vg: -34, v_ug: -72, v_ub: 179 },  // 217
    YuvTableRow { v_y: 235, v_vr: 143, v_vg: -35, v_ug: -73, v_ub: 181 },  // 218
    YuvTableRow { v_y: 236, v_vr: 145, v_vg: -35, v_ug: -73, v_ub: 183 },  // 219
    YuvTableRow { v_y: 237, v_vr: 146, v_vg: -35, v_ug: -74, v_ub: 185 },  // 220
    YuvTableRow { v_y: 238, v_vr: 148, v_vg: -36, v_ug: -75, v_ub: 187 },  // 221
    YuvTableRow { v_y: 239, v_vr: 150, v_vg: -36, v_ug: -76, v_ub: 189 },  // 222
    YuvTableRow { v_y: 240, v_vr: 151, v_vg: -37, v_ug: -77, v_ub: 191 },  // 223
    YuvTableRow { v_y: 242, v_vr: 153, v_vg: -37, v_ug: -78, v_ub: 193 },  // 224
    YuvTableRow { v_y: 243, v_vr: 154, v_vg: -37, v_ug: -78, v_ub: 195 },  // 225
    YuvTableRow { v_y: 244, v_vr: 156, v_vg: -38, v_ug: -79, v_ub: 197 },  // 226
    YuvTableRow { v_y: 245, v_vr: 158, v_vg: -38, v_ug: -80, v_ub: 199 },  // 227
    YuvTableRow { v_y: 246, v_vr: 159, v_vg: -39, v_ug: -81, v_ub: 201 },  // 228
    YuvTableRow { v_y: 247, v_vr: 161, v_vg: -39, v_ug: -82, v_ub: 203 },  // 229
    YuvTableRow { v_y: 249, v_vr: 162, v_vg: -39, v_ug: -82, v_ub: 205 },  // 230
    YuvTableRow { v_y: 250, v_vr: 164, v_vg: -40, v_ug: -83, v_ub: 207 },  // 231
    YuvTableRow { v_y: 251, v_vr: 165, v_vg: -40, v_ug: -84, v_ub: 209 },  // 232
    YuvTableRow { v_y: 252, v_vr: 167, v_vg: -41, v_ug: -85, v_ub: 211 },  // 233
    YuvTableRow { v_y: 253, v_vr: 169, v_vg: -41, v_ug: -86, v_ub: 213 },  // 234
    YuvTableRow { v_y: 254, v_vr: 170, v_vg: -41, v_ug: -86, v_ub: 215 },  // 235
    YuvTableRow { v_y: 256, v_vr: 172, v_vg: -42, v_ug: -87, v_ub: 217 },  // 236
    YuvTableRow { v_y: 257, v_vr: 173, v_vg: -42, v_ug: -88, v_ub: 219 },  // 237
    YuvTableRow { v_y: 258, v_vr: 175, v_vg: -43, v_ug: -89, v_ub: 221 },  // 238
    YuvTableRow { v_y: 259, v_vr: 177, v_vg: -43, v_ug: -90, v_ub: 223 },  // 239
    YuvTableRow { v_y: 260, v_vr: 178, v_vg: -43, v_ug: -91, v_ub: 226 },  // 240
    YuvTableRow { v_y: 261, v_vr: 180, v_vg: -44, v_ug: -91, v_ub: 228 },  // 241
    YuvTableRow { v_y: 263, v_vr: 181, v_vg: -44, v_ug: -92, v_ub: 230 },  // 242
    YuvTableRow { v_y: 264, v_vr: 183, v_vg: -44, v_ug: -93, v_ub: 232 },  // 243
    YuvTableRow { v_y: 265, v_vr: 185, v_vg: -45, v_ug: -94, v_ub: 234 },  // 244
    YuvTableRow { v_y: 266, v_vr: 186, v_vg: -45, v_ug: -95, v_ub: 236 },  // 245
    YuvTableRow { v_y: 267, v_vr: 188, v_vg: -46, v_ug: -95, v_ub: 238 },  // 246
    YuvTableRow { v_y: 268, v_vr: 189, v_vg: -46, v_ug: -96, v_ub: 240 },  // 247
    YuvTableRow { v_y: 270, v_vr: 191, v_vg: -46, v_ug: -97, v_ub: 242 },  // 248
    YuvTableRow { v_y: 271, v_vr: 193, v_vg: -47, v_ug: -98, v_ub: 244 },  // 249
    YuvTableRow { v_y: 272, v_vr: 194, v_vg: -47, v_ug: -99, v_ub: 246 },  // 250
    YuvTableRow { v_y: 273, v_vr: 196, v_vg: -48, v_ug: -99, v_ub: 248 },  // 251
    YuvTableRow { v_y: 274, v_vr: 197, v_vg: -48, v_ug: -100, v_ub: 250 }, // 252
    YuvTableRow { v_y: 275, v_vr: 199, v_vg: -48, v_ug: -101, v_ub: 252 }, // 253
    YuvTableRow { v_y: 277, v_vr: 201, v_vg: -49, v_ug: -102, v_ub: 254 }, // 254
    YuvTableRow { v_y: 278, v_vr: 202, v_vg: -49, v_ug: -103, v_ub: 256 }, // 255
];

/// Converts a single YUV pixel `(y, u, v)` to RGB `(r, g, b)` using the exact
/// integer ITU-R BT.601 lookup table and clamping from `conversions/yuv.c`.
#[inline]
pub fn yuv2rgb(y: u8, u: u8, v: u8) -> (u8, u8, u8) {
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

/// Converts a single RGB888 pixel `(r, g, b)` to ITU-R BT.601 `(y, u, v)`.
#[inline]
pub(crate) const fn rgb2yuv(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
    let r = r as i32;
    let g = g as i32;
    let b = b as i32;

    let y = clamp_u8((19595 * r + 38470 * g + 7471 * b + 32768) >> 16);
    let u = clamp_u8(128 + ((-11059 * r - 21709 * g + 32768 * b + 32768) >> 16));
    let v = clamp_u8(128 + ((32768 * r - 27439 * g - 5329 * b + 32768) >> 16));
    (y, u, v)
}

#[inline]
const fn unpack_rgb565_be(hb: u8, lb: u8) -> (u8, u8, u8) {
    let r = hb & 0xF8;
    let g = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
    let b = (lb & 0x1F) << 3;
    (r, g, b)
}

#[inline]
const fn pack_rgb565(r: u8, g: u8, b: u8, byte_swap: bool) -> [u8; 2] {
    let hb = (r & 0xF8) | (g >> 5);
    let lb = ((g & 0x1C) << 3) | (b >> 3);
    if byte_swap {
        [lb, hb]
    } else {
        [hb, lb]
    }
}

fn checked_pixel_count(width: u16, height: u16, max_bpp: u32) -> Result<usize, CameraError> {
    if width == 0 || height == 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels32 = (width as u32)
        .checked_mul(height as u32)
        .ok_or(CameraError::Overflow)?;
    let bytes32 = pixels32
        .checked_mul(max_bpp)
        .ok_or(CameraError::Overflow)?;
    if bytes32 > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }
    usize::try_from(pixels32).map_err(|_| CameraError::Overflow)
}

/// Converts a big-endian RGB565 buffer into packed YUV422 (YUYV: `[Y0, U, Y1, V]`).
///
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn rgb565_to_yuv422(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let pairs = pixels / 2;
    for i in 0..pairs {
        let base = i * 4;
        let (r0, g0, b0) = unpack_rgb565_be(src[base], src[base + 1]);
        let (r1, g1, b1) = unpack_rgb565_be(src[base + 2], src[base + 3]);

        let (y0, u0, v0) = rgb2yuv(r0, g0, b0);
        let (y1, u1, v1) = rgb2yuv(r1, g1, b1);

        dst[base] = y0;
        dst[base + 1] = ((u0 as u16 + u1 as u16) / 2) as u8;
        dst[base + 2] = y1;
        dst[base + 3] = ((v0 as u16 + v1 as u16) / 2) as u8;
    }

    if pixels & 1 != 0 {
        let base = pairs * 4;
        let (r0, g0, b0) = unpack_rgb565_be(src[base], src[base + 1]);
        let (y0, u0, _v0) = rgb2yuv(r0, g0, b0);
        dst[base] = y0;
        dst[base + 1] = u0;
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into RGB565.
///
/// When `byte_swap` is `false`, writes big-endian RGB565 (`[hb, lb]`); when `true`,
/// swaps the byte order (`[lb, hb]`).
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn yuv422_to_rgb565(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    byte_swap: bool,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let pairs = pixels / 2;
    for i in 0..pairs {
        let base = i * 4;
        let y0 = src[base];
        let u = src[base + 1];
        let y1 = src[base + 2];
        let v = src[base + 3];

        let (r0, g0, b0) = yuv2rgb(y0, u, v);
        let (r1, g1, b1) = yuv2rgb(y1, u, v);

        let p0 = pack_rgb565(r0, g0, b0, byte_swap);
        let p1 = pack_rgb565(r1, g1, b1, byte_swap);

        dst[base] = p0[0];
        dst[base + 1] = p0[1];
        dst[base + 2] = p1[0];
        dst[base + 3] = p1[1];
    }

    if pixels & 1 != 0 {
        let base = pairs * 4;
        let y0 = src[base];
        let u = src[base + 1];
        let (r0, g0, b0) = yuv2rgb(y0, u, 128);
        let p0 = pack_rgb565(r0, g0, b0, byte_swap);
        dst[base] = p0[0];
        dst[base + 1] = p0[1];
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into 24-bit RGB888 (`[R, G, B]`).
///
/// Returns the number of bytes written to `dst` (`width * height * 3`).
pub fn yuv422_to_rgb888(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 3)?;
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let dst_len = pixels.checked_mul(3).ok_or(CameraError::Overflow)?;
    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let pairs = pixels / 2;
    for i in 0..pairs {
        let s_base = i * 4;
        let d_base = i * 6;
        let y0 = src[s_base];
        let u = src[s_base + 1];
        let y1 = src[s_base + 2];
        let v = src[s_base + 3];

        let (r0, g0, b0) = yuv2rgb(y0, u, v);
        let (r1, g1, b1) = yuv2rgb(y1, u, v);

        dst[d_base] = r0;
        dst[d_base + 1] = g0;
        dst[d_base + 2] = b0;
        dst[d_base + 3] = r1;
        dst[d_base + 4] = g1;
        dst[d_base + 5] = b1;
    }

    if pixels & 1 != 0 {
        let s_base = pairs * 4;
        let d_base = pairs * 6;
        let y0 = src[s_base];
        let u = src[s_base + 1];
        let (r0, g0, b0) = yuv2rgb(y0, u, 128);
        dst[d_base] = r0;
        dst[d_base + 1] = g0;
        dst[d_base + 2] = b0;
    }

    Ok(dst_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into planar YUV420 (I420).
///
/// Requires `width` and `height` to be non-zero and even (`width % 2 == 0`, `height % 2 == 0`).
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
    let h = height as usize;
    let u_offset = pixels;
    let v_offset = pixels + chroma_plane_len;

    for by in 0..(h / 2) {
        let row0 = by * 2;
        let row1 = row0 + 1;
        for bx in 0..(w / 2) {
            let x0 = bx * 2;
            let s0 = (row0 * w + x0) * 2;
            let s1 = (row1 * w + x0) * 2;

            let y00 = src[s0];
            let u0 = src[s0 + 1];
            let y01 = src[s0 + 2];
            let v0 = src[s0 + 3];

            let y10 = src[s1];
            let u1 = src[s1 + 1];
            let y11 = src[s1 + 2];
            let v1 = src[s1 + 3];

            dst[row0 * w + x0] = y00;
            dst[row0 * w + x0 + 1] = y01;
            dst[row1 * w + x0] = y10;
            dst[row1 * w + x0 + 1] = y11;

            let uv_idx = by * (w / 2) + bx;
            dst[u_offset + uv_idx] = ((u0 as u16 + u1 as u16) / 2) as u8;
            dst[v_offset + uv_idx] = ((v0 as u16 + v1 as u16) / 2) as u8;
        }
    }

    Ok(dst_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yuv2rgb_bt601_table_values() {
        // Neutral black in BT.601 (Y=16, U=128, V=128) -> (0, 0, 0)
        assert_eq!(yuv2rgb(16, 128, 128), (0, 0, 0));
        // Neutral mid-gray (Y=128, U=128, V=128) -> (130, 130, 130) per yuv_table[128].vY = 130
        assert_eq!(yuv2rgb(128, 128, 128), (130, 130, 130));
        // Clamping at extremes
        assert_eq!(yuv2rgb(0, 0, 0), (0, 136, 0));
        assert_eq!(yuv2rgb(255, 255, 255), (255, 126, 255));
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
}
