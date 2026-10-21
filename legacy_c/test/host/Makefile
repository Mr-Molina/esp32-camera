# Host-side ASan/UBSan test harness for esp32-camera conversions/ (audit WO-REM-SETUP-01).
#
# Usage:
#   make -C /home/console-admin/Github/esp32-camera/test/host test         # ASan + UBSan; known alignment finding is reported, not fatal
#   make -C /home/console-admin/Github/esp32-camera/test/host test-strict  # every UBSan finding is fatal (expected to FAIL until the
#                                                                          # misaligned bmp_header_t store in conversions/to_bmp.c is fixed)
#   make -C /home/console-admin/Github/esp32-camera/test/host clean
#
# ROOT defaults to this checkout; override with: make ROOT=/abs/path/to/esp32-camera ...
#
# Sources are compiled unmodified. ESP-IDF headers are replaced by minimal stubs in stubs/.
# The JPEG decoder (espressif/esp_jpeg) is stubbed and always fails, so the decode paths
# (jpg2rgb565, jpg2bmp, fmt2rgb888 with JPEG input) are not exercised here.

ROOT     ?= /home/console-admin/Github/esp32-camera
HOST_DIR := $(ROOT)/test/host
BUILD    := $(HOST_DIR)/build

CC  ?= gcc
CXX ?= g++

# -m32 (ILP32) matches the Xtensa ABI. Following the remediation of INFRA-103,
# callback_stream::get_size() in conversions/to_jpg.cpp now returns jpge::uint
# (matching jpge::output_stream::get_size()), so both -m32 and native 64-bit
# (ARCH_FLAGS=) compile cleanly. -m32 remains default to match Xtensa ILP32.
# Requires 32-bit multilib (gcc-multilib/g++-multilib, lib32asan, lib32ubsan).
ARCH_FLAGS ?= -m32
SAN_FLAGS := $(ARCH_FLAGS) -g -fsanitize=address,undefined -fno-omit-frame-pointer
INCLUDES  := -I$(HOST_DIR)/stubs \
             -I$(ROOT)/conversions/include \
             -I$(ROOT)/conversions/private_include \
             -I$(ROOT)/driver/include
COMMON    := $(SAN_FLAGS) -O1 -Wall -Wextra -Wno-unused-parameter $(INCLUDES)

# Default mode: all UBSan checks abort. Misaligned bmp_header_t stores in conversions/to_bmp.c
# have been remediated (CONV-101), so -fsanitize-recover=alignment is removed (QA-101).
RECOVER_DEFAULT := -fno-sanitize-recover=all
RECOVER_STRICT  := -fno-sanitize-recover=all

CFLAGS_EXTRA   ?=
CXXFLAGS_EXTRA ?=

# Language standards match ESP-IDF v5.5 (taken from the IDF build's compile_commands.json).
# gnu++2b matters: jpge.cpp left-shifts negative ints (e.g. DCT2D, jpge.cpp:139), which is UB in
# C++17 but well-defined since C++20; building as gnu++17 would report UB that does not exist on target.
CSTD   := -std=gnu17
CXXSTD := -std=gnu++2b

SRCS_C   := $(ROOT)/conversions/to_bmp.c $(ROOT)/conversions/yuv.c $(HOST_DIR)/stubs/jpeg_decoder_stub.c
SRCS_CXX := $(ROOT)/conversions/to_jpg.cpp $(ROOT)/conversions/jpge.cpp
TEST_SRC := $(HOST_DIR)/test_conversions_smoke.c

.PHONY: all test test-strict clean

all: $(BUILD)/default/test_conversions_smoke

define build_variant
$(BUILD)/$(1)/test_conversions_smoke: $(SRCS_C) $(SRCS_CXX) $(TEST_SRC) $(wildcard $(HOST_DIR)/stubs/*.h) $(wildcard $(HOST_DIR)/stubs/*/*.h) $(HOST_DIR)/Makefile
	mkdir -p $(BUILD)/$(1)
	$(CC)  $(COMMON) $(2) $(CSTD)   $(CFLAGS_EXTRA)   -c $(ROOT)/conversions/to_bmp.c          -o $(BUILD)/$(1)/to_bmp.o
	$(CC)  $(COMMON) $(2) $(CSTD)   $(CFLAGS_EXTRA)   -c $(ROOT)/conversions/yuv.c             -o $(BUILD)/$(1)/yuv.o
	$(CC)  $(COMMON) $(2) $(CSTD)   $(CFLAGS_EXTRA)   -c $(HOST_DIR)/stubs/jpeg_decoder_stub.c -o $(BUILD)/$(1)/jpeg_decoder_stub.o
	$(CXX) $(COMMON) $(2) $(CXXSTD) $(CXXFLAGS_EXTRA) -c $(ROOT)/conversions/to_jpg.cpp       -o $(BUILD)/$(1)/to_jpg.o
	$(CXX) $(COMMON) $(2) $(CXXSTD) $(CXXFLAGS_EXTRA) -c $(ROOT)/conversions/jpge.cpp         -o $(BUILD)/$(1)/jpge.o
	$(CC)  $(COMMON) $(2) $(CSTD)   $(CFLAGS_EXTRA)   -c $(TEST_SRC)                           -o $(BUILD)/$(1)/test_conversions_smoke.o
	$(CXX) $(SAN_FLAGS) $(BUILD)/$(1)/*.o -o $$@
endef

$(eval $(call build_variant,default,$(RECOVER_DEFAULT)))
$(eval $(call build_variant,strict,$(RECOVER_STRICT)))

test: $(BUILD)/default/test_conversions_smoke
	ASAN_OPTIONS=detect_leaks=1:abort_on_error=0:halt_on_error=1 \
	UBSAN_OPTIONS=print_stacktrace=1 \
	$(BUILD)/default/test_conversions_smoke

test-strict: $(BUILD)/strict/test_conversions_smoke
	ASAN_OPTIONS=detect_leaks=1:abort_on_error=0:halt_on_error=1 \
	UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	$(BUILD)/strict/test_conversions_smoke

clean:
	rm -rf $(BUILD)
