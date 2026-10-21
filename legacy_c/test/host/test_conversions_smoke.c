/*
 * Host smoke test for esp32-camera conversions/ (WO-REM-SETUP-01, QA-102).
 * Converts synthetic RGB565 and GRAYSCALE frames through fmt2bmp() and fmt2jpg()
 * under ASan/UBSan, verifying standard dimensions, odd-dimension BMP row-padding,
 * palette generation, and bounded decode checks.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
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

static uint16_t rd16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void test_rgb565_standard(void)
{
    /* Exact-size heap buffer so ASan catches any over-read of the source frame. */
    const size_t src_len = (size_t)W * H * 2;
    uint8_t *src = malloc(src_len);
    if (!src) {
        fprintf(stderr, "FAIL: malloc\n");
        failures++;
        return;
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
}

static void test_bmp_odd_dimensions(void)
{
    /* Case 1: 15x7 RGB565
     * row_bytes = 15 * 3 = 45; row_stride = (45 + 3) & ~3 = 48 (3 padding bytes/row)
     * image_size = 48 * 7 = 336; total bmp_len = 54 + 336 = 390
     */
    {
        const uint16_t w = 15;
        const uint16_t h = 7;
        const size_t src_len = (size_t)w * h * 2;
        uint8_t *src = malloc(src_len);
        if (!src) {
            fprintf(stderr, "FAIL: malloc 15x7\n");
            failures++;
            return;
        }
        for (size_t i = 0; i < src_len; i++) {
            src[i] = (uint8_t)(i ^ 0x5A);
        }

        uint8_t *bmp = NULL;
        size_t bmp_len = 0;
        bool ok = fmt2bmp(src, src_len, w, h, PIXFORMAT_RGB565, &bmp, &bmp_len);
        CHECK(ok, "fmt2bmp returns true (15x7 odd-dimension RGB565)");
        if (ok && bmp) {
            const size_t row_stride = (((size_t)w * 3) + 3) & ~(size_t)3;
            const size_t expected_len = 54 + row_stride * h;
            CHECK(bmp_len == expected_len, "fmt2bmp length matches row-padded stride (15x7 RGB565)");
            CHECK(bmp[0] == 'B' && bmp[1] == 'M', "fmt2bmp signature 'BM' (15x7 RGB565)");
            CHECK(rd32le(&bmp[2]) == bmp_len, "fmt2bmp filesize field (15x7 RGB565)");
            CHECK(rd32le(&bmp[18]) == w, "fmt2bmp width field == 15");
            CHECK((int32_t)rd32le(&bmp[22]) == -(int32_t)h, "fmt2bmp height field == -7 (top-down)");
            CHECK(rd16le(&bmp[28]) == 24, "fmt2bmp bitsperpixel == 24");
            free(bmp);
        }
        free(src);
    }

    /* Case 2: 17x11 RGB565
     * row_bytes = 17 * 3 = 51; row_stride = (51 + 3) & ~3 = 52 (1 padding byte/row)
     * image_size = 52 * 11 = 572; total bmp_len = 54 + 572 = 626
     */
    {
        const uint16_t w = 17;
        const uint16_t h = 11;
        const size_t src_len = (size_t)w * h * 2;
        uint8_t *src = malloc(src_len);
        if (!src) {
            fprintf(stderr, "FAIL: malloc 17x11\n");
            failures++;
            return;
        }
        for (size_t i = 0; i < src_len; i++) {
            src[i] = (uint8_t)(i ^ 0x3C);
        }

        uint8_t *bmp = NULL;
        size_t bmp_len = 0;
        bool ok = fmt2bmp(src, src_len, w, h, PIXFORMAT_RGB565, &bmp, &bmp_len);
        CHECK(ok, "fmt2bmp returns true (17x11 odd-dimension RGB565)");
        if (ok && bmp) {
            const size_t row_stride = (((size_t)w * 3) + 3) & ~(size_t)3;
            const size_t expected_len = 54 + row_stride * h;
            CHECK(bmp_len == expected_len, "fmt2bmp length matches row-padded stride (17x11 RGB565)");
            CHECK(bmp[0] == 'B' && bmp[1] == 'M', "fmt2bmp signature 'BM' (17x11 RGB565)");
            CHECK(rd32le(&bmp[2]) == bmp_len, "fmt2bmp filesize field (17x11 RGB565)");
            CHECK(rd32le(&bmp[18]) == w, "fmt2bmp width field == 17");
            CHECK((int32_t)rd32le(&bmp[22]) == -(int32_t)h, "fmt2bmp height field == -11 (top-down)");
            CHECK(rd16le(&bmp[28]) == 24, "fmt2bmp bitsperpixel == 24");
            free(bmp);
        }
        free(src);
    }
}

static void test_bmp_grayscale(void)
{
    /* Case 1: 16x8 GRAYSCALE
     * bpp = 1; palette_size = 1024 (256 * 4); row_stride = 16
     * image_size = 16 * 8 = 128; total bmp_len = 54 + 1024 + 128 = 1206
     */
    {
        const uint16_t w = 16;
        const uint16_t h = 8;
        const size_t src_len = (size_t)w * h;
        uint8_t *src = malloc(src_len);
        if (!src) {
            fprintf(stderr, "FAIL: malloc 16x8 grayscale\n");
            failures++;
            return;
        }
        for (size_t i = 0; i < src_len; i++) {
            src[i] = (uint8_t)(i * 2);
        }

        uint8_t *bmp = NULL;
        size_t bmp_len = 0;
        bool ok = fmt2bmp(src, src_len, w, h, PIXFORMAT_GRAYSCALE, &bmp, &bmp_len);
        CHECK(ok, "fmt2bmp returns true (16x8 GRAYSCALE)");
        if (ok && bmp) {
            const size_t row_stride = (((size_t)w * 1) + 3) & ~(size_t)3;
            const size_t expected_len = 54 + 1024 + row_stride * h;
            CHECK(bmp_len == expected_len, "fmt2bmp length == 54 + 1024 + row_stride*H (16x8 GRAYSCALE)");
            CHECK(bmp[0] == 'B' && bmp[1] == 'M', "fmt2bmp signature 'BM' (16x8 GRAYSCALE)");
            CHECK(rd32le(&bmp[2]) == bmp_len, "fmt2bmp filesize field (16x8 GRAYSCALE)");
            CHECK(rd32le(&bmp[10]) == 54 + 1024, "fmt2bmp pixel offset == 54 + 1024 palette (16x8 GRAYSCALE)");
            CHECK(rd32le(&bmp[18]) == w, "fmt2bmp width field == 16");
            CHECK((int32_t)rd32le(&bmp[22]) == -(int32_t)h, "fmt2bmp height field == -8");
            CHECK(rd16le(&bmp[28]) == 8, "fmt2bmp bitsperpixel == 8 (GRAYSCALE)");
            free(bmp);
        }
        free(src);
    }

    /* Case 2: 15x7 GRAYSCALE
     * bpp = 1; palette_size = 1024; row_stride = (15 + 3) & ~3 = 16 (1 padding byte/row)
     * image_size = 16 * 7 = 112; total bmp_len = 54 + 1024 + 112 = 1190
     */
    {
        const uint16_t w = 15;
        const uint16_t h = 7;
        const size_t src_len = (size_t)w * h;
        uint8_t *src = malloc(src_len);
        if (!src) {
            fprintf(stderr, "FAIL: malloc 15x7 grayscale\n");
            failures++;
            return;
        }
        for (size_t i = 0; i < src_len; i++) {
            src[i] = (uint8_t)(i * 3);
        }

        uint8_t *bmp = NULL;
        size_t bmp_len = 0;
        bool ok = fmt2bmp(src, src_len, w, h, PIXFORMAT_GRAYSCALE, &bmp, &bmp_len);
        CHECK(ok, "fmt2bmp returns true (15x7 GRAYSCALE with row padding)");
        if (ok && bmp) {
            const size_t row_stride = (((size_t)w * 1) + 3) & ~(size_t)3;
            const size_t expected_len = 54 + 1024 + row_stride * h;
            CHECK(bmp_len == expected_len, "fmt2bmp length == 54 + 1024 + row_stride*H (15x7 GRAYSCALE)");
            CHECK(bmp[0] == 'B' && bmp[1] == 'M', "fmt2bmp signature 'BM' (15x7 GRAYSCALE)");
            CHECK(rd32le(&bmp[2]) == bmp_len, "fmt2bmp filesize field (15x7 GRAYSCALE)");
            CHECK(rd32le(&bmp[10]) == 54 + 1024, "fmt2bmp pixel offset == 54 + 1024 palette (15x7 GRAYSCALE)");
            CHECK(rd32le(&bmp[18]) == w, "fmt2bmp width field == 15");
            CHECK((int32_t)rd32le(&bmp[22]) == -(int32_t)h, "fmt2bmp height field == -7");
            CHECK(rd16le(&bmp[28]) == 8, "fmt2bmp bitsperpixel == 8");
            free(bmp);
        }
        free(src);
    }
}

static void test_bounded_and_overflow(void)
{
    /* 1. fmt2rgb888_bounded test with insufficient vs sufficient buffer */
    const size_t src_len = (size_t)W * H * 2; /* 256 bytes for 16x8 RGB565 */
    uint8_t *src = malloc(src_len);
    if (!src) {
        fprintf(stderr, "FAIL: malloc for bounded test\n");
        failures++;
        return;
    }
    memset(src, 0x55, src_len);

    const size_t needed_rgb_len = (size_t)W * H * 3; /* 384 bytes */
    uint8_t *out_buf = malloc(needed_rgb_len);
    if (!out_buf) {
        free(src);
        fprintf(stderr, "FAIL: malloc out_buf\n");
        failures++;
        return;
    }

    /* Insufficient buffer: 383 bytes provided */
    bool ok_underflow = fmt2rgb888_bounded(src, src_len, PIXFORMAT_RGB565, out_buf, needed_rgb_len - 1);
    CHECK(!ok_underflow, "fmt2rgb888_bounded rejects insufficient output buffer");

    /* Sufficient buffer: exactly 384 bytes provided */
    bool ok_exact = fmt2rgb888_bounded(src, src_len, PIXFORMAT_RGB565, out_buf, needed_rgb_len);
    CHECK(ok_exact, "fmt2rgb888_bounded succeeds with exact output buffer size");

    /* 2. fmt2bmp buffer bounds rejection test */
    uint8_t *bmp = NULL;
    size_t bmp_len = 0;
    /* Source buffer of only 10 bytes is far smaller than 16*8*2 = 256 bytes */
    bool ok_truncated = fmt2bmp(src, 10, W, H, PIXFORMAT_RGB565, &bmp, &bmp_len);
    CHECK(!ok_truncated, "fmt2bmp rejects truncated source buffer");
    if (bmp) {
        free(bmp);
    }

    free(out_buf);
    free(src);
}

int main(void)
{
    test_rgb565_standard();
    test_bmp_odd_dimensions();
    test_bmp_grayscale();
    test_bounded_and_overflow();

    if (failures) {
        printf("RESULT: FAIL (%d)\n", failures);
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
