/* Host stub for sdkconfig.h (test/host only).
 * Intentionally empty: no CONFIG_SPIRAM*, so the conversions' _malloc() helpers use libc malloc,
 * which ASan instruments. */
#pragma once
