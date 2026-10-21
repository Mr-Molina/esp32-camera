/* Host stub for ESP-IDF esp_attr.h (test/host only). */
#pragma once

#define IRAM_ATTR
#define DRAM_ATTR
#define RTC_DATA_ATTR
#define EXT_RAM_BSS_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
