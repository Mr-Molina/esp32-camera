/* Host stub for ESP-IDF esp_heap_caps.h (test/host only). */
#pragma once
#include <stdlib.h>
#include <stdint.h>

#define MALLOC_CAP_EXEC     (1 << 0)
#define MALLOC_CAP_32BIT    (1 << 1)
#define MALLOC_CAP_8BIT     (1 << 2)
#define MALLOC_CAP_DMA      (1 << 3)
#define MALLOC_CAP_SPIRAM   (1 << 10)
#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_DEFAULT  (1 << 12)

static inline void *heap_caps_malloc(size_t size, uint32_t caps)
{
    (void)caps;
    return malloc(size);
}

static inline void *heap_caps_calloc(size_t n, size_t size, uint32_t caps)
{
    (void)caps;
    return calloc(n, size);
}

static inline void heap_caps_free(void *ptr)
{
    free(ptr);
}
