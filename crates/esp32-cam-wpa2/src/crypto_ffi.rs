//! Crypto C-FFI jump table and shims for WPA supplicant.

use core::ffi::c_char;

type HmacVecFn =
    unsafe extern "C" fn(*const u8, i32, i32, *mut *const u8, *const i32, *mut u8) -> i32;
type Pbkdf2Fn = unsafe extern "C" fn(*const c_char, *const c_char, u32, i32, *mut u8, u32) -> i32;
type Aes128Fn = unsafe extern "C" fn(*const u8, *const u8, *mut u8, i32) -> i32;
type Omac1Fn = unsafe extern "C" fn(*const u8, *const u8, usize, *mut u8) -> i32;
type CcmpDecFn =
    unsafe extern "C" fn(*const u8, *const u8, *const u8, usize, *mut usize, bool) -> *mut u8;
type CcmpEncFn =
    unsafe extern "C" fn(*const u8, *mut u8, usize, usize, *mut u8, i32, *mut usize) -> *mut u8;
type AesGmacFn =
    unsafe extern "C" fn(*const u8, usize, *const u8, usize, *const u8, usize, *mut u8) -> i32;
type ShaVecFn = unsafe extern "C" fn(usize, *mut *const u8, *const usize, *mut u8) -> i32;
type AesWrapFn = unsafe extern "C" fn(*const u8, usize, i32, *const u8, *mut u8) -> i32;

#[repr(C)]
pub struct WpaCryptoFuncs {
    pub size: u32,
    pub version: u32,
    pub hmac_sha256_vector: Option<HmacVecFn>,
    pub pbkdf2_sha1: Option<Pbkdf2Fn>,
    pub aes_128_encrypt: Option<Aes128Fn>,
    pub aes_128_decrypt: Option<Aes128Fn>,
    pub omac1_aes_128: Option<Omac1Fn>,
    pub ccmp_decrypt: Option<CcmpDecFn>,
    pub ccmp_encrypt: Option<CcmpEncFn>,
    pub aes_gmac: Option<AesGmacFn>,
    pub sha256_vector: Option<ShaVecFn>,
    pub aes_wrap: Option<AesWrapFn>,
    pub aes_unwrap: Option<AesWrapFn>,
}

#[cfg(target_pointer_width = "32")]
const _: () = assert!(core::mem::size_of::<WpaCryptoFuncs>() == 52);

#[cfg(target_arch = "xtensa")]
extern "C" {
    fn malloc(size: usize) -> *mut u8;
    fn free(ptr: *mut u8);
}

/// Deallocates a buffer previously returned by CCMP encryption or decryption.
///
/// # Safety
///
/// `ptr` must be null or point to a buffer allocated by `ccmp_alloc`.
// JUSTIFICATION: Deallocates CCMP buffers in host unit tests or xtensa targets.
// SAFETY: Caller must pass pointer allocated by ccmp_alloc or null.
pub unsafe fn ccmp_test_free(ptr: *mut u8) {
    #[cfg(all(not(target_arch = "xtensa"), test))]
    if !ptr.is_null() {
        // JUSTIFICATION: Layout recovery from 16-byte prefix.
        // SAFETY: ptr allocated with 16-byte alignment and size prefix.
        unsafe {
            let raw = ptr.sub(16);
            let total = core::ptr::read(raw as *const usize);
            if let Ok(l) = std::alloc::Layout::from_size_align(total, 16) {
                std::alloc::dealloc(raw, l);
            }
        }
    }
    #[cfg(target_arch = "xtensa")]
    // JUSTIFICATION: Free xtensa memory.
    // SAFETY: Calls free on non-null malloc pointer.
    unsafe {
        if !ptr.is_null() {
            free(ptr);
        }
    }
    #[cfg(all(not(target_arch = "xtensa"), not(test)))]
    let _ = ptr;
}

// JUSTIFICATION: Allocates frame buffers for CCMP encryption/decryption.
// SAFETY: Caller must free non-null returns via ccmp_test_free.
unsafe fn ccmp_alloc(size: usize) -> *mut u8 {
    #[cfg(target_arch = "xtensa")]
    // JUSTIFICATION: Allocate buffer via ESP-IDF malloc.
    // SAFETY: Calls C runtime malloc.
    unsafe {
        malloc(size)
    }
    #[cfg(all(not(target_arch = "xtensa"), test))]
    // JUSTIFICATION: Allocate host memory with size prefix.
    // SAFETY: 16-byte aligned allocation.
    unsafe {
        if size == 0 {
            return core::ptr::null_mut();
        }
        let total = size + 16;
        let l = match std::alloc::Layout::from_size_align(total, 16) {
            Ok(layout) => layout,
            Err(_) => return core::ptr::null_mut(),
        };
        let raw = std::alloc::alloc(l);
        if raw.is_null() {
            return core::ptr::null_mut();
        }
        core::ptr::write(raw as *mut usize, total);
        raw.add(16)
    }
    #[cfg(all(not(target_arch = "xtensa"), not(test)))]
    {
        let _ = size;
        core::ptr::null_mut()
    }
}

/// Encrypts an IEEE 802.11 MPDU payload using CCMP (AES-128-CCM).
///
/// # Safety
///
/// - `tk` must point to a 16-byte temporal key.
/// - `frame` must point to at least `len` readable bytes.
/// - `pn` must point to at least 6 readable packet number bytes.
/// - `encrypted_len` must be a valid, writable pointer to `usize`.
// JUSTIFICATION: CCMP encrypt ABI compatible with ccmp.c.obj.
// SAFETY: Validates pointers and bounds before slicing buffers.
pub unsafe extern "C" fn wpa_crypto_ccmp_encrypt(
    tk: *const u8,
    frame: *mut u8,
    len: usize,
    hdrlen: usize,
    pn: *mut u8,
    keyid: i32,
    encrypted_len: *mut usize,
) -> *mut u8 {
    // JUSTIFICATION: Slices frame and derives CCMP ciphertext.
    // SAFETY: Validates non-null pointers, lengths, and copies in bounds.
    unsafe {
        if tk.is_null()
            || frame.is_null()
            || pn.is_null()
            || encrypted_len.is_null()
            || hdrlen < 24
            || len < hdrlen
        {
            return core::ptr::null_mut();
        }
        let plen = len - hdrlen;
        let crypt = ccmp_alloc(len + 32);
        if crypt.is_null() {
            return core::ptr::null_mut();
        }
        core::ptr::copy_nonoverlapping(frame, crypt, hdrlen);
        *crypt.add(1) |= 0x40;
        let p = core::slice::from_raw_parts(pn, 6);
        let kid = ((keyid as u8) & 3) << 6;
        let iv = [p[5], p[4], 0, 0x20 | kid, p[3], p[2], p[1], p[0]];
        core::ptr::copy_nonoverlapping(iv.as_ptr(), crypt.add(hdrlen), 8);
        let hdr_s = core::slice::from_raw_parts(crypt, hdrlen);
        let iv_s = core::slice::from_raw_parts(crypt.add(hdrlen), 8);
        let (aad, aad_len, nonce) = match crate::crypto::ccm::ccmp_aad_nonce(hdr_s, iv_s) {
            Ok(v) => v,
            Err(_) => {
                ccmp_test_free(crypt);
                return core::ptr::null_mut();
            }
        };
        let plain_s = core::slice::from_raw_parts(frame.add(hdrlen + 8), plen);
        let out_crypt = core::slice::from_raw_parts_mut(crypt.add(hdrlen + 8), plen);
        let out_tag = core::slice::from_raw_parts_mut(crypt.add(len + 8), 8);
        let key = &*(tk as *const [u8; 16]);
        if crate::crypto::ccm::aes_ccm_ae(
            key,
            &nonce,
            8,
            plain_s,
            &aad[..aad_len],
            out_crypt,
            out_tag,
        )
        .is_err()
        {
            ccmp_test_free(crypt);
            return core::ptr::null_mut();
        }
        *encrypted_len = len + 16;
        crypt
    }
}

/// Decrypts an IEEE 802.11 MPDU payload using CCMP (AES-128-CCM).
///
/// # Safety
///
/// - `tk` must point to a 16-byte temporal key.
/// - `frame` must point to at least 24 readable header bytes.
/// - `data` must point to at least `data_len` readable bytes.
/// - `decrypted_len` must be a valid, writable pointer to `usize`.
// JUSTIFICATION: CCMP decrypt ABI compatible with ccmp.c.obj.
// SAFETY: Validates pointers and reads frame control header for length.
pub unsafe extern "C" fn wpa_crypto_ccmp_decrypt(
    tk: *const u8,
    frame: *const u8,
    data: *const u8,
    data_len: usize,
    decrypted_len: *mut usize,
    espnow_pkt: bool,
) -> *mut u8 {
    // JUSTIFICATION: Slices 802.11 header and decrypted output buffer.
    // SAFETY: Validates non-null pointers and min data_len for CCMP header.
    unsafe {
        if tk.is_null()
            || frame.is_null()
            || data.is_null()
            || decrypted_len.is_null()
            || data_len < 16
        {
            return core::ptr::null_mut();
        }
        let fc = u16::from_le_bytes([*frame, *frame.add(1)]);
        let to_ds_from_ds = (fc & 0x0300) == 0x0300;
        let is_qos = (fc & 0x0080) != 0 && (fc & 0x000c) == 0x0008;
        let hdrlen = 24 + if to_ds_from_ds { 6 } else { 0 } + if is_qos { 2 } else { 0 };
        let hdr = core::slice::from_raw_parts(frame, hdrlen);
        let in_data = core::slice::from_raw_parts(data, data_len);
        let alloc_len = data_len + 16;
        let out_buf = ccmp_alloc(alloc_len);
        if out_buf.is_null() {
            return core::ptr::null_mut();
        }
        let out_slice = core::slice::from_raw_parts_mut(out_buf, alloc_len);
        let key = &*(tk as *const [u8; 16]);
        match crate::crypto::ccm::ccmp_decrypt_frame(key, hdr, in_data, out_slice, espnow_pkt) {
            Ok(dlen) => {
                *decrypted_len = dlen;
                out_buf
            }
            Err(_) => {
                ccmp_test_free(out_buf);
                core::ptr::null_mut()
            }
        }
    }
}

/// Computes an HMAC-SHA256 digest over an array of vector buffers.
///
/// # Safety
///
/// - `key` must point to `key_len` readable bytes if `key_len > 0`.
/// - `addr` and `len` must point to `num_elem` readable entries.
/// - `mac` must point to at least 32 writable bytes.
// JUSTIFICATION: HMAC-SHA256 vector computation for WPA FFI.
// SAFETY: Validates pointers and bounds stack array to at most 16 slices.
pub unsafe extern "C" fn wpa_crypto_hmac_sha256_vector(
    key: *const u8,
    key_len: i32,
    num_elem: i32,
    addr: *mut *const u8,
    len: *const i32,
    mac: *mut u8,
) -> i32 {
    // JUSTIFICATION: Dereferencing vector pointers for HMAC calculation.
    // SAFETY: Validates non-null pointers, positive lengths, and num_elem.
    unsafe {
        if key.is_null()
            || addr.is_null()
            || len.is_null()
            || mac.is_null()
            || key_len < 0
            || !(0..=16).contains(&num_elem)
        {
            return -1;
        }
        let k = core::slice::from_raw_parts(key, key_len as usize);
        let mut slices: [&[u8]; 16] = [&[]; 16];
        for (i, slice_slot) in slices.iter_mut().enumerate().take(num_elem as usize) {
            let p = *addr.add(i);
            let l = *len.add(i);
            if p.is_null() || l < 0 {
                return -1;
            }
            *slice_slot = core::slice::from_raw_parts(p, l as usize);
        }
        let res = crate::crypto::hash::hmac_sha256_vector(k, &slices[..num_elem as usize]);
        core::ptr::copy_nonoverlapping(res.as_ptr(), mac, 32);
        0
    }
}

/// Computes a SHA256 digest over an array of vector buffers.
///
/// # Safety
///
/// - `addr` and `len` must point to `num_elem` readable entries.
/// - `mac` must point to at least 32 writable bytes.
// JUSTIFICATION: SHA256 vector computation for WPA FFI.
// SAFETY: Validates pointers and bounds stack array to at most 16 slices.
pub unsafe extern "C" fn wpa_crypto_sha256_vector(
    num_elem: usize,
    addr: *mut *const u8,
    len: *const usize,
    mac: *mut u8,
) -> i32 {
    // JUSTIFICATION: Dereferencing vector pointers for SHA256 calculation.
    // SAFETY: Validates non-null pointers and num_elem bounds.
    unsafe {
        if addr.is_null() || len.is_null() || mac.is_null() || num_elem > 16 {
            return -1;
        }
        let mut slices: [&[u8]; 16] = [&[]; 16];
        for (i, slice_slot) in slices.iter_mut().enumerate().take(num_elem) {
            let p = *addr.add(i);
            let l = *len.add(i);
            if p.is_null() {
                return -1;
            }
            *slice_slot = core::slice::from_raw_parts(p, l);
        }
        let res = crate::crypto::hash::sha256_vector(&slices[..num_elem]);
        core::ptr::copy_nonoverlapping(res.as_ptr(), mac, 32);
        0
    }
}

/// Derives a key using PBKDF2-HMAC-SHA1.
///
/// # Safety
///
/// - `passphrase` must point to a valid null-terminated C string.
/// - `ssid` must point to at least `ssid_len` readable bytes.
/// - `buf` must point to at least `buflen` writable bytes.
// JUSTIFICATION: PBKDF2-SHA1 passphrase derivation for WPA FFI.
// SAFETY: Validates pointers; scans passphrase string up to 64 bytes.
pub unsafe extern "C" fn wpa_crypto_pbkdf2_sha1(
    passphrase: *const c_char,
    ssid: *const c_char,
    ssid_len: u32,
    iterations: i32,
    buf: *mut u8,
    buflen: u32,
) -> i32 {
    // JUSTIFICATION: String scanning and buffer mutation for PBKDF2.
    // SAFETY: Bounded pointer scans, validates non-null and valid range.
    unsafe {
        if passphrase.is_null()
            || ssid.is_null()
            || buf.is_null()
            || iterations <= 0
            || buflen == 0
            || buflen > 64
        {
            return -1;
        }
        let mut pass_len = 0;
        while pass_len < 64 && *passphrase.add(pass_len) != 0 {
            pass_len += 1;
        }
        if pass_len == 64 && *passphrase.add(pass_len) != 0 {
            return -1;
        }
        let pass = core::slice::from_raw_parts(passphrase.cast::<u8>(), pass_len);
        let ssid_s = core::slice::from_raw_parts(ssid.cast::<u8>(), ssid_len as usize);
        let out = core::slice::from_raw_parts_mut(buf, buflen as usize);
        if crate::kdf::pbkdf2_hmac_sha1(pass, ssid_s, iterations as u32, out).is_err() {
            return -1;
        }
        0
    }
}

/// Encrypts data in place using AES-128-CBC.
///
/// # Safety
///
/// - `key` and `iv` must point to 16 readable bytes.
/// - `data` must point to at least `data_len` readable and writable bytes.
// JUSTIFICATION: AES-128-CBC encryption for WPA FFI.
// SAFETY: Validates non-null pointers and non-negative data_len.
pub unsafe extern "C" fn wpa_crypto_aes_128_encrypt(
    key: *const u8,
    iv: *const u8,
    data: *mut u8,
    data_len: i32,
) -> i32 {
    // JUSTIFICATION: Mutates data buffer in place with AES-CBC ciphertext.
    // SAFETY: Validates non-null pointers and non-negative length.
    unsafe {
        if key.is_null() || iv.is_null() || data.is_null() || data_len < 0 {
            return -1;
        }
        let k = &*(key as *const [u8; 16]);
        let v = &*(iv as *const [u8; 16]);
        let d = core::slice::from_raw_parts_mut(data, data_len as usize);
        if crate::crypto::aes::aes_128_cbc_encrypt(k, v, d).is_err() {
            -1
        } else {
            0
        }
    }
}

/// Decrypts data in place using AES-128-CBC.
///
/// # Safety
///
/// - `key` and `iv` must point to 16 readable bytes.
/// - `data` must point to at least `data_len` readable and writable bytes.
// JUSTIFICATION: AES-128-CBC decryption for WPA FFI.
// SAFETY: Validates non-null pointers and non-negative data_len.
pub unsafe extern "C" fn wpa_crypto_aes_128_decrypt(
    key: *const u8,
    iv: *const u8,
    data: *mut u8,
    data_len: i32,
) -> i32 {
    // JUSTIFICATION: Mutates data buffer in place with AES-CBC plaintext.
    // SAFETY: Validates non-null pointers and non-negative length.
    unsafe {
        if key.is_null() || iv.is_null() || data.is_null() || data_len < 0 {
            return -1;
        }
        let k = &*(key as *const [u8; 16]);
        let v = &*(iv as *const [u8; 16]);
        let d = core::slice::from_raw_parts_mut(data, data_len as usize);
        if crate::crypto::aes::aes_128_cbc_decrypt(k, v, d).is_err() {
            -1
        } else {
            0
        }
    }
}

/// Computes OMAC1-AES-128 (CMAC) over the input buffer.
///
/// # Safety
///
/// - `key` must point to 16 readable bytes.
/// - `data` must point to at least `data_len` readable bytes.
/// - `mac` must point to at least 16 writable bytes.
// JUSTIFICATION: OMAC1 AES-128 MAC calculation for WPA FFI.
// SAFETY: Validates pointers; writes 16-byte MAC into mac.
pub unsafe extern "C" fn wpa_crypto_omac1_aes_128(
    key: *const u8,
    data: *const u8,
    data_len: usize,
    mac: *mut u8,
) -> i32 {
    // JUSTIFICATION: Copies calculated 16-byte OMAC1 tag into destination.
    // SAFETY: Validates non-null pointers.
    unsafe {
        if key.is_null() || data.is_null() || mac.is_null() {
            return -1;
        }
        let k = &*(key as *const [u8; 16]);
        let d = core::slice::from_raw_parts(data, data_len);
        let tag = crate::crypto::aes::omac1_aes_128(k, d);
        core::ptr::copy_nonoverlapping(tag.as_ptr(), mac, 16);
        0
    }
}

/// Computes AES-GMAC authentication tag over additional authenticated data.
///
/// # Safety
///
/// - `key` must point to 16 readable bytes (`key_len == 16`).
/// - `iv` must point to at least `iv_len` readable bytes.
/// - `aad` must point to at least `aad_len` readable bytes.
/// - `tag` must point to at least 16 writable bytes.
// JUSTIFICATION: AES-GMAC calculation for WPA FFI.
// SAFETY: Validates non-null pointers; requires 16-byte key.
pub unsafe extern "C" fn wpa_crypto_aes_gmac(
    key: *const u8,
    key_len: usize,
    iv: *const u8,
    iv_len: usize,
    aad: *const u8,
    aad_len: usize,
    tag: *mut u8,
) -> i32 {
    // JUSTIFICATION: Computes GMAC tag and stores into 16-byte tag buffer.
    // SAFETY: Validates non-null pointers and key_len == 16.
    unsafe {
        if key.is_null() || iv.is_null() || aad.is_null() || tag.is_null() || key_len != 16 {
            return -1;
        }
        let k = &*(key as *const [u8; 16]);
        let v = core::slice::from_raw_parts(iv, iv_len);
        let a = core::slice::from_raw_parts(aad, aad_len);
        let t = &mut *(tag as *mut [u8; 16]);
        if crate::crypto::aes::aes_gmac(k, v, a, t).is_err() {
            -1
        } else {
            0
        }
    }
}

/// Wraps 64-bit plaintext blocks using RFC 3394 AES Key Wrap.
///
/// # Safety
///
/// - `kek` must point to 16 readable key bytes (`kek_len == 16`).
/// - `plain` must point to at least `n * 8` readable bytes (`1 <= n <= 16`).
/// - `cipher` must point to at least `(n + 1) * 8` writable bytes.
// JUSTIFICATION: RFC 3394 AES Key Wrap for WPA FFI.
// SAFETY: Validates pointers, kek_len == 16, and 1 <= n <= 16.
pub unsafe extern "C" fn wpa_crypto_aes_wrap(
    kek: *const u8,
    kek_len: usize,
    n: i32,
    plain: *const u8,
    cipher: *mut u8,
) -> i32 {
    // JUSTIFICATION: Wraps plaintext key blocks into cipher output.
    // SAFETY: Validates non-null pointers, kek_len == 16, and 1 <= n <= 16.
    unsafe {
        if kek.is_null()
            || plain.is_null()
            || cipher.is_null()
            || kek_len != 16
            || !(1..=16).contains(&n)
        {
            return -1;
        }
        let k = &*(kek as *const [u8; 16]);
        let plen = (n as usize) * 8;
        let clen = (n as usize + 1) * 8;
        let p = core::slice::from_raw_parts(plain, plen);
        let c = core::slice::from_raw_parts_mut(cipher, clen);
        if crate::crypto::aes::aes_wrap(k, p, c).is_err() {
            -1
        } else {
            0
        }
    }
}

/// Unwraps 64-bit ciphertext blocks using RFC 3394 AES Key Wrap.
///
/// # Safety
///
/// - `kek` must point to 16 readable key bytes (`kek_len == 16`).
/// - `cipher` must point to `(n + 1) * 8` readable bytes (`1 <= n <= 16`).
/// - `plain` must point to at least `n * 8` writable bytes.
// JUSTIFICATION: RFC 3394 AES Key Unwrap for WPA FFI.
// SAFETY: Validates pointers, kek_len == 16, and 1 <= n <= 16.
pub unsafe extern "C" fn wpa_crypto_aes_unwrap(
    kek: *const u8,
    kek_len: usize,
    n: i32,
    cipher: *const u8,
    plain: *mut u8,
) -> i32 {
    // JUSTIFICATION: Unwraps ciphertext key blocks into plain output.
    // SAFETY: Validates non-null pointers, kek_len == 16, and 1 <= n <= 16.
    unsafe {
        if kek.is_null()
            || cipher.is_null()
            || plain.is_null()
            || kek_len != 16
            || !(1..=16).contains(&n)
        {
            return -1;
        }
        let k = &*(kek as *const [u8; 16]);
        let plen = (n as usize) * 8;
        let clen = (n as usize + 1) * 8;
        let c = core::slice::from_raw_parts(cipher, clen);
        let p = core::slice::from_raw_parts_mut(plain, plen);
        if crate::crypto::aes::aes_unwrap(k, c, p).is_err() {
            -1
        } else {
            0
        }
    }
}

#[no_mangle]
pub static g_wifi_default_wpa_crypto_funcs: WpaCryptoFuncs = WpaCryptoFuncs {
    size: core::mem::size_of::<WpaCryptoFuncs>() as u32,
    version: 1,
    hmac_sha256_vector: Some(wpa_crypto_hmac_sha256_vector),
    pbkdf2_sha1: Some(wpa_crypto_pbkdf2_sha1),
    aes_128_encrypt: Some(wpa_crypto_aes_128_encrypt),
    aes_128_decrypt: Some(wpa_crypto_aes_128_decrypt),
    omac1_aes_128: Some(wpa_crypto_omac1_aes_128),
    ccmp_decrypt: Some(wpa_crypto_ccmp_decrypt),
    ccmp_encrypt: Some(wpa_crypto_ccmp_encrypt),
    aes_gmac: Some(wpa_crypto_aes_gmac),
    sha256_vector: Some(wpa_crypto_sha256_vector),
    aes_wrap: Some(wpa_crypto_aes_wrap),
    aes_unwrap: Some(wpa_crypto_aes_unwrap),
};
