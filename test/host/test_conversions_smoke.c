/*
 * Host smoke test for esp32-camera conversions/ (WO-REM-SETUP-01).
 * Converts a small synthetic RGB565 frame through fmt2bmp() and fmt2jpg() under ASan/UBSan.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "img_converters.h"

#define W 16
#define H 8

static int failures = 0;

#define CHECK(cond, msg) do { \
        if (!(cond)) { fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); failures++; } \
        else { printf("ok: %s\n", msg); } \
    } while (0)

static uint32_t rd32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int main(void)
{
    /* Exact-size heap buffer so ASan catches any over-read of the source frame. */
    const size_t src_len = (size_t)W * H * 2;
    uint8_t *src = malloc(src_len);
    if (!src) {
        fprintf(stderr, "FAIL: malloc\n");
        return 1;
    }
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            uint16_t r = (uint16_t)((x * 31) / (W - 1));
            uint16_t g = (uint16_t)((y * 63) / (H - 1));
            uint16_t b = (uint16_t)(((x + y) * 31) / (W + H - 2));
            uint16_t px = (uint16_t)((r << 11) | (g << 5) | b);
            /* esp32-camera RGB565 frames are big-endian (high byte first) */
            src[(y * W + x) * 2 + 0] = (uint8_t)(px >> 8);
            src[(y * W + x) * 2 + 1] = (uint8_t)(px & 0xFF);
        }
    }

    /* fmt2bmp */
    uint8_t *bmp = NULL;
    size_t bmp_len = 0;
    bool ok = fmt2bmp(src, src_len, W, H, PIXFORMAT_RGB565, &bmp, &bmp_len);
    CHECK(ok, "fmt2bmp returns true");
    if (ok && bmp) {
        CHECK(bmp_len == 54 + (size_t)W * H * 3, "fmt2bmp length == 54 + W*H*3");
        CHECK(bmp_len >= 2 && bmp[0] == 'B' && bmp[1] == 'M', "fmt2bmp 'BM' signature");
        CHECK(bmp_len >= 6 && rd32le(&bmp[2]) == bmp_len, "fmt2bmp header filesize field");
        CHECK(bmp_len >= 22 && rd32le(&bmp[18]) == W, "fmt2bmp header width field");
        free(bmp);
    }

    /* fmt2jpg */
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    ok = fmt2jpg(src, src_len, W, H, PIXFORMAT_RGB565, 80, &jpg, &jpg_len);
    CHECK(ok, "fmt2jpg returns true");
    if (ok && jpg) {
        CHECK(jpg_len > 4, "fmt2jpg non-trivial length");
        CHECK(jpg_len >= 2 && jpg[0] == 0xFF && jpg[1] == 0xD8, "fmt2jpg SOI marker");
        CHECK(jpg_len >= 2 && jpg[jpg_len - 2] == 0xFF && jpg[jpg_len - 1] == 0xD9, "fmt2jpg EOI marker");
        free(jpg);
    }

    free(src);

    if (failures) {
        printf("RESULT: FAIL (%d)\n", failures);
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
