//! Pure-Rust ESP32 firmware image and ELF32 bootloader utilities.
//!
//! Converts a raw ESP32 `0xE9` image binary into an `EM_XTENSA` ELF32
//! executable with `PT_LOAD` program headers, Progbits section headers,
//! entry symbol `_entry`, and ESP32 hardware MMIO register symbols. Also packs
//! pure-Rust `EM_XTENSA` bootloader ELFs into `0xE9` `.bin` flash images.

use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::sha256::Sha256;
use crate::verify_invariants::AUTHORITATIVE_MMIO_REGISTERS;

/// ESP32 application image header magic byte (`0xE9`).
pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;
/// Length of standard ESP32 image header in bytes.
pub const ESP_IMAGE_HEADER_LEN: usize = 24;
/// Length of standard ESP32 segment header in bytes.
pub const ESP_SEGMENT_HEADER_LEN: usize = 8;

/// ELF machine code for Tensilica Xtensa (`EM_XTENSA`).
pub const EM_XTENSA: u16 = 94;
/// ELF object file type for an executable (`ET_EXEC`).
pub const ET_EXEC: u16 = 2;
/// ELF class for 32-bit architecture (`ELFCLASS32`).
pub const ELFCLASS32: u8 = 1;
/// ELF data encoding for 2's complement little-endian (`ELFDATA2LSB`).
pub const ELFDATA2LSB: u8 = 1;
/// ELF version (`EV_CURRENT`).
pub const EV_CURRENT: u8 = 1;

/// ELF segment type for loadable program segment (`PT_LOAD`).
pub const PT_LOAD: u32 = 1;
/// Segment flag: executable (`PF_X`).
pub const PF_X: u32 = 1;
/// Segment flag: writable (`PF_W`).
pub const PF_W: u32 = 2;
/// Segment flag: readable (`PF_R`).
pub const PF_R: u32 = 4;

/// Section header type for progbits (`SHT_PROGBITS`).
pub const SHT_PROGBITS: u32 = 1;
/// Section header type for symbol table (`SHT_SYMTAB`).
pub const SHT_SYMTAB: u32 = 2;
/// Section header type for string table (`SHT_STRTAB`).
pub const SHT_STRTAB: u32 = 3;
/// Section header type for nobits (`SHT_NOBITS`).
pub const SHT_NOBITS: u32 = 8;
/// Maximum allowed ESP32 2nd-stage bootloader `.bin` size (`0x7000` bytes).
pub const BOOTLOADER_MAX_BYTES: usize = 28_672;

const ELF_EHDR_SIZE: usize = 52;
const ELF_PHDR_SIZE: usize = 32;
const ELF_SHDR_SIZE: usize = 40;
const ESP_FLASH_MODE_DIO: u8 = 0x02;
const ESP_FLASH_SPI_40MHZ_4MB: u8 = 0x20;
const ESP_WP_PIN_DISABLED: u8 = 0xEE;
const ESP_MAX_CHIP_REV_V3_99: u16 = 399;
const ESP_CHECKSUM_SEED: u8 = 0xEF;

/// Section flag: writable (`SHF_WRITE`).
pub const SHF_WRITE: u32 = 1;
/// Section flag: occupies memory during execution (`SHF_ALLOC`).
pub const SHF_ALLOC: u32 = 2;
/// Section flag: executable machine instructions (`SHF_EXECINSTR`).
pub const SHF_EXECINSTR: u32 = 4;

/// Special section index: absolute symbol (`SHN_ABS`).
pub const SHN_ABS: u16 = 0xFFF1;

/// Symbol binding: Global (`STB_GLOBAL`).
pub const STB_GLOBAL: u8 = 1;
/// Symbol type: Object (`STT_OBJECT`).
pub const STT_OBJECT: u8 = 1;
/// Symbol type: Function (`STT_FUNC`).
pub const STT_FUNC: u8 = 2;

/// Segment metadata parsed from an ESP32 firmware image.
#[derive(Debug, Clone)]
struct EspSegment<'a> {
    load_addr: u32,
    data: &'a [u8],
}

/// Wraps a raw ESP32 `0xE9` image into a valid little-endian `EM_XTENSA` ELF32.
///
/// # Errors
/// Returns an error message if:
/// - The image length is less than 24 bytes.
/// - The magic byte is not `0xE9`.
/// - The segment count is not within `1..=16`.
/// - Segment headers or data exceed image boundaries.
/// - The segment XOR checksum does not match.
/// - The appended SHA-256 hash does not match (if enabled).
pub fn wrap_esp_image_to_elf(image: &[u8], entry_override: Option<u32>) -> Result<Vec<u8>, String> {
    if image.len() < ESP_IMAGE_HEADER_LEN {
        return Err(format!(
            "Image length {} is too short (minimum {} bytes)",
            image.len(),
            ESP_IMAGE_HEADER_LEN
        ));
    }

    let magic = image[0];
    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(format!(
            "Invalid image magic {magic:#04x} (expected {ESP_IMAGE_HEADER_MAGIC:#04x})"
        ));
    }

    let segment_count = image[1];
    if !(1..=16).contains(&segment_count) {
        return Err(format!(
            "Invalid segment count {segment_count} (must be 1..=16)"
        ));
    }

    let header_entry = u32::from_le_bytes([image[4], image[5], image[6], image[7]]);
    let entry_addr = entry_override.unwrap_or(header_entry);
    let hash_appended = (image[23] & 1) != 0;

    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut offset = ESP_IMAGE_HEADER_LEN;
    let mut xor_chk = 0xEFu8;

    for seg_idx in 0..segment_count {
        if offset + ESP_SEGMENT_HEADER_LEN > image.len() {
            return Err(format!(
                "Truncated segment {seg_idx} header at offset {offset}"
            ));
        }
        let load_addr = u32::from_le_bytes([
            image[offset],
            image[offset + 1],
            image[offset + 2],
            image[offset + 3],
        ]);
        let data_len = u32::from_le_bytes([
            image[offset + 4],
            image[offset + 5],
            image[offset + 6],
            image[offset + 7],
        ]) as usize;

        offset += ESP_SEGMENT_HEADER_LEN;
        if offset + data_len > image.len() {
            return Err(format!(
                "Truncated segment {seg_idx} payload (len {data_len}) at offset {offset}"
            ));
        }

        let seg_data = &image[offset..offset + data_len];
        for &b in seg_data {
            xor_chk ^= b;
        }

        segments.push(EspSegment {
            load_addr,
            data: seg_data,
        });

        offset += data_len;
    }

    // Checksum byte position: padded to 16-byte boundary ending at checksum byte.
    let pad_len = 16 - (offset % 16);
    let chk_offset = offset + pad_len - 1;

    if chk_offset >= image.len() {
        return Err(format!(
            "Image truncated before checksum byte at offset {chk_offset}"
        ));
    }

    let actual_chk = image[chk_offset];
    if actual_chk != xor_chk {
        return Err(format!(
            "Checksum mismatch: computed {xor_chk:#04x}, image has {actual_chk:#04x}"
        ));
    }

    if hash_appended {
        let hash_start = chk_offset + 1;
        if hash_start + 32 > image.len() {
            return Err(format!(
                "Image truncated before 32-byte SHA-256 digest at offset {hash_start}"
            ));
        }
        let expected_hash = &image[hash_start..hash_start + 32];
        let computed_hash = Sha256::digest(&image[..=chk_offset]);
        if computed_hash != expected_hash {
            return Err("Appended SHA-256 digest mismatch".to_string());
        }
    }

    // Build the ELF32
    // Program headers count: segment_count
    let phnum = segments.len();
    let ehdr_size = 52usize;
    let phdr_size = 32usize;
    let shdr_size = 40usize;
    let sym_size = 16usize;

    // Calculate segment file offsets
    let mut current_offset = ehdr_size + phnum * phdr_size;
    let mut seg_offsets = Vec::with_capacity(phnum);
    for seg in &segments {
        current_offset = (current_offset + 3) & !3; // 4-byte align
        seg_offsets.push(current_offset);
        current_offset += seg.data.len();
    }

    // Build Symbol Table (.symtab) and String Table (.strtab)
    // Sym 0: NULL
    // Sym 1: _entry
    // Sym 2..: MMIO registers (0x3FF*)
    let mut strtab = Vec::new();
    strtab.push(0u8); // index 0 is null string

    let entry_name_off = strtab.len() as u32;
    strtab.extend_from_slice(b"_entry\0");

    let mut mmio_symbols = Vec::new();
    for &(name, addr) in AUTHORITATIVE_MMIO_REGISTERS {
        if (0x3FF0_0000..0x4000_0000).contains(&addr) {
            let name_off = strtab.len() as u32;
            strtab.extend_from_slice(name.as_bytes());
            strtab.push(0u8);
            mmio_symbols.push((name_off, addr));
        }
    }

    // Determine section for _entry
    let mut entry_shndx = SHN_ABS;
    for (i, seg) in segments.iter().enumerate() {
        let start = seg.load_addr;
        let end = start.saturating_add(seg.data.len() as u32);
        if (start..end).contains(&entry_addr) {
            entry_shndx = (i + 1) as u16;
            break;
        }
    }

    let mut symtab_bytes = Vec::new();
    // Symbol 0: NULL
    symtab_bytes.extend_from_slice(&[0u8; 16]);

    // Symbol 1: _entry
    symtab_bytes.extend_from_slice(&entry_name_off.to_le_bytes()); // st_name
    symtab_bytes.extend_from_slice(&entry_addr.to_le_bytes()); // st_value
    symtab_bytes.extend_from_slice(&0u32.to_le_bytes()); // st_size
    let entry_info = (STB_GLOBAL << 4) | (STT_FUNC & 0xF);
    symtab_bytes.push(entry_info); // st_info
    symtab_bytes.push(0); // st_other
    symtab_bytes.extend_from_slice(&entry_shndx.to_le_bytes()); // st_shndx

    // MMIO symbols
    let mmio_info = (STB_GLOBAL << 4) | (STT_OBJECT & 0xF);
    for (name_off, addr) in &mmio_symbols {
        symtab_bytes.extend_from_slice(&name_off.to_le_bytes()); // st_name
        symtab_bytes.extend_from_slice(&addr.to_le_bytes()); // st_value
        symtab_bytes.extend_from_slice(&4u32.to_le_bytes()); // st_size = 4
        symtab_bytes.push(mmio_info); // st_info
        symtab_bytes.push(0); // st_other
        symtab_bytes.extend_from_slice(&SHN_ABS.to_le_bytes()); // st_shndx
    }

    // Build Section Header String Table (.shstrtab)
    let mut shstrtab = Vec::new();
    shstrtab.push(0u8); // null string

    let mut seg_sh_names = Vec::with_capacity(phnum);
    let mut dram_count = 0;
    let mut iram_count = 0;

    for (i, seg) in segments.iter().enumerate() {
        let name_off = shstrtab.len() as u32;
        let sec_name = if (0x3FF8_0000..0x4000_0000).contains(&seg.load_addr) {
            let n = format!(".dram{dram_count}");
            dram_count += 1;
            n
        } else if (0x4000_0000..0x400C_0000).contains(&seg.load_addr) {
            let n = format!(".iram{iram_count}");
            iram_count += 1;
            n
        } else {
            format!(".seg{i}")
        };
        shstrtab.extend_from_slice(sec_name.as_bytes());
        shstrtab.push(0u8);
        seg_sh_names.push(name_off);
    }

    let symtab_name_off = shstrtab.len() as u32;
    shstrtab.extend_from_slice(b".symtab\0");

    let strtab_name_off = shstrtab.len() as u32;
    shstrtab.extend_from_slice(b".strtab\0");

    let shstrtab_name_off = shstrtab.len() as u32;
    shstrtab.extend_from_slice(b".shstrtab\0");

    // Sections order:
    // 0: NULL
    // 1..=phnum: segment sections
    // phnum + 1: .symtab
    // phnum + 2: .strtab
    // phnum + 3: .shstrtab
    let strtab_sec_idx = (phnum + 2) as u32;
    let shstrtab_sec_idx = (phnum + 3) as u16;
    let total_sections = phnum + 4;

    current_offset = (current_offset + 3) & !3;
    let symtab_offset = current_offset;
    current_offset += symtab_bytes.len();

    current_offset = (current_offset + 3) & !3;
    let strtab_offset = current_offset;
    current_offset += strtab.len();

    current_offset = (current_offset + 3) & !3;
    let shstrtab_offset = current_offset;
    current_offset += shstrtab.len();

    current_offset = (current_offset + 3) & !3;
    let shoff = current_offset;

    let mut elf = Vec::with_capacity(shoff + total_sections * shdr_size);

    // Write Elf32_Ehdr
    // e_ident
    elf.extend_from_slice(&[0x7F, b'E', b'L', b'F']);
    elf.push(ELFCLASS32);
    elf.push(ELFDATA2LSB);
    elf.push(EV_CURRENT);
    elf.push(0); // ELFOSABI_NONE
    elf.extend_from_slice(&[0u8; 8]); // padding

    elf.extend_from_slice(&ET_EXEC.to_le_bytes()); // e_type
    elf.extend_from_slice(&EM_XTENSA.to_le_bytes()); // e_machine
    elf.extend_from_slice(&1u32.to_le_bytes()); // e_version
    elf.extend_from_slice(&entry_addr.to_le_bytes()); // e_entry
    elf.extend_from_slice(&(ehdr_size as u32).to_le_bytes()); // e_phoff = 52
    elf.extend_from_slice(&(shoff as u32).to_le_bytes()); // e_shoff
    elf.extend_from_slice(&0x300u32.to_le_bytes()); // e_flags
    elf.extend_from_slice(&(ehdr_size as u16).to_le_bytes()); // e_ehsize = 52
    elf.extend_from_slice(&(phdr_size as u16).to_le_bytes()); // e_phentsize = 32
    elf.extend_from_slice(&(phnum as u16).to_le_bytes()); // e_phnum
    elf.extend_from_slice(&(shdr_size as u16).to_le_bytes()); // e_shentsize = 40
    elf.extend_from_slice(&(total_sections as u16).to_le_bytes()); // e_shnum
    elf.extend_from_slice(&shstrtab_sec_idx.to_le_bytes()); // e_shstrndx

    // Write Elf32_Phdr entries
    for (i, seg) in segments.iter().enumerate() {
        let seg_flags = if (0x4000_0000..0x400C_0000).contains(&seg.load_addr) {
            PF_R | PF_X
        } else if (0x3FF8_0000..0x4000_0000).contains(&seg.load_addr) {
            PF_R | PF_W
        } else {
            PF_R
        };

        elf.extend_from_slice(&PT_LOAD.to_le_bytes()); // p_type
        elf.extend_from_slice(&(seg_offsets[i] as u32).to_le_bytes()); // p_offset
        elf.extend_from_slice(&seg.load_addr.to_le_bytes()); // p_vaddr
        elf.extend_from_slice(&seg.load_addr.to_le_bytes()); // p_paddr
        elf.extend_from_slice(&(seg.data.len() as u32).to_le_bytes()); // p_filesz
        elf.extend_from_slice(&(seg.data.len() as u32).to_le_bytes()); // p_memsz
        elf.extend_from_slice(&seg_flags.to_le_bytes()); // p_flags
        elf.extend_from_slice(&4u32.to_le_bytes()); // p_align
    }

    // Write segments data
    for (i, seg) in segments.iter().enumerate() {
        let target_off = seg_offsets[i];
        while elf.len() < target_off {
            elf.push(0);
        }
        elf.extend_from_slice(seg.data);
    }

    // Write .symtab
    while elf.len() < symtab_offset {
        elf.push(0);
    }
    elf.extend_from_slice(&symtab_bytes);

    // Write .strtab
    while elf.len() < strtab_offset {
        elf.push(0);
    }
    elf.extend_from_slice(&strtab);

    // Write .shstrtab
    while elf.len() < shstrtab_offset {
        elf.push(0);
    }
    elf.extend_from_slice(&shstrtab);

    // Write Section Header Table
    while elf.len() < shoff {
        elf.push(0);
    }

    // Section 0: NULL
    elf.extend_from_slice(&[0u8; 40]);

    // Section 1..=phnum: segment sections
    for (i, seg) in segments.iter().enumerate() {
        let sec_flags = if (0x4000_0000..0x400C_0000).contains(&seg.load_addr) {
            SHF_ALLOC | SHF_EXECINSTR
        } else if (0x3FF8_0000..0x4000_0000).contains(&seg.load_addr) {
            SHF_ALLOC | SHF_WRITE
        } else {
            SHF_ALLOC
        };

        elf.extend_from_slice(&seg_sh_names[i].to_le_bytes()); // sh_name
        elf.extend_from_slice(&SHT_PROGBITS.to_le_bytes()); // sh_type
        elf.extend_from_slice(&sec_flags.to_le_bytes()); // sh_flags
        elf.extend_from_slice(&seg.load_addr.to_le_bytes()); // sh_addr
        elf.extend_from_slice(&(seg_offsets[i] as u32).to_le_bytes()); // sh_offset
        elf.extend_from_slice(&(seg.data.len() as u32).to_le_bytes()); // sh_size
        elf.extend_from_slice(&0u32.to_le_bytes()); // sh_link
        elf.extend_from_slice(&0u32.to_le_bytes()); // sh_info
        elf.extend_from_slice(&4u32.to_le_bytes()); // sh_addralign
        elf.extend_from_slice(&0u32.to_le_bytes()); // sh_entsize
    }

    // Section phnum + 1: .symtab
    elf.extend_from_slice(&symtab_name_off.to_le_bytes()); // sh_name
    elf.extend_from_slice(&SHT_SYMTAB.to_le_bytes()); // sh_type
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_flags
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_addr
    elf.extend_from_slice(&(symtab_offset as u32).to_le_bytes()); // sh_offset
    elf.extend_from_slice(&(symtab_bytes.len() as u32).to_le_bytes()); // sh_size
    elf.extend_from_slice(&strtab_sec_idx.to_le_bytes()); // sh_link
    elf.extend_from_slice(&1u32.to_le_bytes()); // sh_info (first global symbol index = 1)
    elf.extend_from_slice(&4u32.to_le_bytes()); // sh_addralign
    elf.extend_from_slice(&(sym_size as u32).to_le_bytes()); // sh_entsize = 16

    // Section phnum + 2: .strtab
    elf.extend_from_slice(&strtab_name_off.to_le_bytes()); // sh_name
    elf.extend_from_slice(&SHT_STRTAB.to_le_bytes()); // sh_type
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_flags
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_addr
    elf.extend_from_slice(&(strtab_offset as u32).to_le_bytes()); // sh_offset
    elf.extend_from_slice(&(strtab.len() as u32).to_le_bytes()); // sh_size
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_link
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_info
    elf.extend_from_slice(&1u32.to_le_bytes()); // sh_addralign
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_entsize

    // Section phnum + 3: .shstrtab
    elf.extend_from_slice(&shstrtab_name_off.to_le_bytes()); // sh_name
    elf.extend_from_slice(&SHT_STRTAB.to_le_bytes()); // sh_type
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_flags
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_addr
    elf.extend_from_slice(&(shstrtab_offset as u32).to_le_bytes()); // sh_offset
    elf.extend_from_slice(&(shstrtab.len() as u32).to_le_bytes()); // sh_size
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_link
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_info
    elf.extend_from_slice(&1u32.to_le_bytes()); // sh_addralign
    elf.extend_from_slice(&0u32.to_le_bytes()); // sh_entsize

    Ok(elf)
}

/// Runs the `wrap-esp-image-elf` CLI subcommand.
///
/// # Errors
/// Returns an error message if parsing fails, input file cannot be read,
/// ELF conversion fails, or output file cannot be written.
pub fn run(args: &[String]) -> Result<(), String> {
    let mut input_path: Option<String> = None;
    let mut output_path: Option<String> = None;
    let mut entry_override: Option<u32> = None;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--input" | "-i" => {
                i += 1;
                if i < args.len() {
                    input_path = Some(args[i].clone());
                } else {
                    return Err("Missing argument for --input".to_string());
                }
            }
            "--output" | "-o" => {
                i += 1;
                if i < args.len() {
                    output_path = Some(args[i].clone());
                } else {
                    return Err("Missing argument for --output".to_string());
                }
            }
            "--entry" | "-e" => {
                i += 1;
                if i < args.len() {
                    let hex_str = args[i].trim();
                    let clean = hex_str
                        .strip_prefix("0x")
                        .or_else(|| hex_str.strip_prefix("0X"))
                        .unwrap_or(hex_str);
                    let val = u32::from_str_radix(clean, 16)
                        .map_err(|e| format!("Invalid hex entry address '{hex_str}': {e}"))?;
                    entry_override = Some(val);
                } else {
                    return Err("Missing argument for --entry".to_string());
                }
            }
            "--help" | "-h" => {
                println!(
                    "Usage: cargo run -p xtask -- wrap-esp-image-elf \
                     --input <BIN> --output <ELF> [--entry <HEX>]"
                );
                return Ok(());
            }
            other => {
                return Err(format!("Unknown option '{other}' for wrap-esp-image-elf"));
            }
        }
        i += 1;
    }

    let input = input_path.ok_or_else(|| "Missing required --input <BIN> argument".to_string())?;
    let output =
        output_path.ok_or_else(|| "Missing required --output <ELF> argument".to_string())?;

    let image_bytes =
        fs::read(&input).map_err(|e| format!("Failed to read input image '{}': {e}", input))?;

    let elf_bytes = wrap_esp_image_to_elf(&image_bytes, entry_override)?;

    fs::write(&output, &elf_bytes)
        .map_err(|e| format!("Failed to write output ELF '{}': {e}", output))?;

    println!(
        "Successfully wrapped '{}' into '{}' ({} bytes, entry {:#010x})",
        input,
        output,
        elf_bytes.len(),
        entry_override.unwrap_or_else(|| {
            if image_bytes.len() >= 8 {
                u32::from_le_bytes([
                    image_bytes[4],
                    image_bytes[5],
                    image_bytes[6],
                    image_bytes[7],
                ])
            } else {
                0
            }
        })
    );

    Ok(())
}

/// Parsed fields from an ELF32 little-endian file header.
struct Elf32Header {
    e_machine: u16,
    e_entry: u32,
    e_phoff: usize,
    e_shoff: usize,
    e_phentsize: usize,
    e_phnum: usize,
    e_shentsize: usize,
    e_shnum: usize,
    e_shstrndx: usize,
}

fn read_u16_le(bytes: &[u8], off: usize) -> Result<u16, String> {
    let slice = bytes
        .get(off..off + 2)
        .ok_or_else(|| format!("Unexpected EOF reading u16 at offset {off}"))?;
    let arr: [u8; 2] = slice
        .try_into()
        .map_err(|_| format!("Failed to decode u16 at offset {off}"))?;
    Ok(u16::from_le_bytes(arr))
}

fn read_u32_le(bytes: &[u8], off: usize) -> Result<u32, String> {
    let slice = bytes
        .get(off..off + 4)
        .ok_or_else(|| format!("Unexpected EOF reading u32 at offset {off}"))?;
    let arr: [u8; 4] = slice
        .try_into()
        .map_err(|_| format!("Failed to decode u32 at offset {off}"))?;
    Ok(u32::from_le_bytes(arr))
}

fn parse_elf32_header(elf_bytes: &[u8]) -> Result<Elf32Header, String> {
    if elf_bytes.len() < ELF_EHDR_SIZE {
        return Err(format!(
            "ELF length {} is shorter than 52-byte ELF32 header",
            elf_bytes.len()
        ));
    }
    if &elf_bytes[0..4] != b"\x7fELF" {
        return Err("Invalid ELF magic (expected \\x7fELF)".to_string());
    }
    if elf_bytes[4] != ELFCLASS32 {
        return Err(format!(
            "Unsupported ELF class {} (expected ELFCLASS32={ELFCLASS32})",
            elf_bytes[4]
        ));
    }
    if elf_bytes[5] != ELFDATA2LSB {
        return Err(format!(
            "Unsupported ELF endianness {} (expected ELFDATA2LSB={ELFDATA2LSB})",
            elf_bytes[5]
        ));
    }

    Ok(Elf32Header {
        e_machine: read_u16_le(elf_bytes, 18)?,
        e_entry: read_u32_le(elf_bytes, 24)?,
        e_phoff: read_u32_le(elf_bytes, 28)? as usize,
        e_shoff: read_u32_le(elf_bytes, 32)? as usize,
        e_phentsize: read_u16_le(elf_bytes, 42)? as usize,
        e_phnum: read_u16_le(elf_bytes, 44)? as usize,
        e_shentsize: read_u16_le(elf_bytes, 46)? as usize,
        e_shnum: read_u16_le(elf_bytes, 48)? as usize,
        e_shstrndx: read_u16_le(elf_bytes, 50)? as usize,
    })
}

fn validate_phdr_table(hdr: &Elf32Header, file_len: usize) -> Result<(), String> {
    if hdr.e_phnum == 0 {
        return Err("ELF contains no program headers (e_phnum == 0)".to_string());
    }
    if hdr.e_phentsize < ELF_PHDR_SIZE {
        return Err(format!(
            "Invalid e_phentsize {} (expected >= {ELF_PHDR_SIZE})",
            hdr.e_phentsize
        ));
    }
    let table_bytes = hdr
        .e_phnum
        .checked_mul(hdr.e_phentsize)
        .ok_or_else(|| "Program header table size overflow".to_string())?;
    let table_end = hdr
        .e_phoff
        .checked_add(table_bytes)
        .ok_or_else(|| "Program header table offset overflow".to_string())?;
    if table_end > file_len {
        return Err(format!(
            "Program header table [{}..{table_end}] exceeds ELF length {file_len}",
            hdr.e_phoff
        ));
    }
    Ok(())
}

fn validate_shdr_table(hdr: &Elf32Header, file_len: usize) -> Result<(), String> {
    if hdr.e_shnum == 0 {
        if hdr.e_shoff > file_len {
            return Err(format!(
                "e_shoff {} exceeds ELF length {file_len}",
                hdr.e_shoff
            ));
        }
        return Ok(());
    }
    if hdr.e_shentsize < ELF_SHDR_SIZE {
        return Err(format!(
            "Invalid e_shentsize {} (expected >= {ELF_SHDR_SIZE})",
            hdr.e_shentsize
        ));
    }
    let table_bytes = hdr
        .e_shnum
        .checked_mul(hdr.e_shentsize)
        .ok_or_else(|| "Section header table size overflow".to_string())?;
    let table_end = hdr
        .e_shoff
        .checked_add(table_bytes)
        .ok_or_else(|| "Section header table offset overflow".to_string())?;
    if table_end > file_len {
        return Err(format!(
            "Section header table [{}..{table_end}] exceeds ELF length {file_len}",
            hdr.e_shoff
        ));
    }
    Ok(())
}

fn collect_pt_load_segments<'a>(
    elf_bytes: &'a [u8],
    hdr: &Elf32Header,
) -> Result<Vec<EspSegment<'a>>, String> {
    let mut segments = Vec::new();
    for idx in 0..hdr.e_phnum {
        let ph = hdr.e_phoff + idx * hdr.e_phentsize;
        let p_type = read_u32_le(elf_bytes, ph)?;
        let p_offset = read_u32_le(elf_bytes, ph + 4)? as usize;
        let p_vaddr = read_u32_le(elf_bytes, ph + 8)?;
        let p_filesz = read_u32_le(elf_bytes, ph + 16)? as usize;
        let p_memsz = read_u32_le(elf_bytes, ph + 20)? as usize;

        if p_type != PT_LOAD {
            continue;
        }
        if p_filesz > p_memsz {
            return Err(format!(
                "PT_LOAD[{idx}] corrupt: p_filesz ({p_filesz}) > p_memsz ({p_memsz})"
            ));
        }
        if p_filesz == 0 {
            continue;
        }
        let seg_end = p_offset
            .checked_add(p_filesz)
            .ok_or_else(|| format!("PT_LOAD[{idx}] offset+filesz overflow"))?;
        let data = elf_bytes.get(p_offset..seg_end).ok_or_else(|| {
            format!(
                "PT_LOAD[{idx}] slice [{p_offset}..{seg_end}] exceeds ELF len {}",
                elf_bytes.len()
            )
        })?;
        segments.push(EspSegment {
            load_addr: p_vaddr,
            data,
        });
    }
    Ok(segments)
}

fn emit_esp_bootloader_image(e_entry: u32, segments: &[EspSegment<'_>]) -> Result<Vec<u8>, String> {
    if segments.is_empty() || segments.len() > 16 {
        return Err(format!(
            "Invalid PT_LOAD segment count {} (must be 1..=16)",
            segments.len()
        ));
    }
    let seg_count_u8 =
        u8::try_from(segments.len()).map_err(|_| "Segment count overflow".to_string())?;

    let mut out = vec![
        ESP_IMAGE_HEADER_MAGIC,
        seg_count_u8,
        ESP_FLASH_MODE_DIO,
        ESP_FLASH_SPI_40MHZ_4MB,
    ];
    out.extend_from_slice(&e_entry.to_le_bytes());
    out.push(ESP_WP_PIN_DISABLED);
    out.extend_from_slice(&[0u8; 3]); // spi_pin_drv
    out.extend_from_slice(&0u16.to_le_bytes()); // chip_id = 0
    out.push(0); // min_chip_rev = 0
    out.extend_from_slice(&0u16.to_le_bytes()); // min_chip_rev_full = 0
    out.extend_from_slice(&ESP_MAX_CHIP_REV_V3_99.to_le_bytes()); // max_chip_rev_full
    out.extend_from_slice(&[0u8; 4]); // reserved
    out.push(1); // hash_appended = 1

    let mut xor_chk = ESP_CHECKSUM_SEED;
    for seg in segments {
        let pad = (4 - (seg.data.len() % 4)) % 4;
        let padded_len = u32::try_from(seg.data.len() + pad)
            .map_err(|_| "Segment payload exceeds u32 length".to_string())?;
        out.extend_from_slice(&seg.load_addr.to_le_bytes());
        out.extend_from_slice(&padded_len.to_le_bytes());
        for &b in seg.data {
            xor_chk ^= b;
        }
        out.extend_from_slice(seg.data);
        out.resize(out.len() + pad, 0u8);
    }

    let offset = out.len();
    let pad_len = 16 - (offset % 16);
    out.resize(offset + pad_len - 1, 0u8);
    out.push(xor_chk);

    let digest = Sha256::digest(&out);
    out.extend_from_slice(&digest);
    Ok(out)
}

/// Packs an `EM_XTENSA` ELF32 executable into an ESP-IDF v5.5.4 `0xE9` image.
///
/// # Errors
/// Returns an error if the ELF header, program headers, or section headers
/// are malformed, `e_machine != EM_XTENSA`, any `PT_LOAD` segment has corrupt
/// or out-of-bounds slices, or the segment count is not in `1..=16`.
pub fn pack_esp_bootloader_bin(elf_bytes: &[u8]) -> Result<Vec<u8>, String> {
    let hdr = parse_elf32_header(elf_bytes)?;
    if hdr.e_machine != EM_XTENSA {
        return Err(format!(
            "Unsupported ELF machine {} (expected EM_XTENSA={EM_XTENSA})",
            hdr.e_machine
        ));
    }
    validate_phdr_table(&hdr, elf_bytes.len())?;
    validate_shdr_table(&hdr, elf_bytes.len())?;

    let segments = collect_pt_load_segments(elf_bytes, &hdr)?;
    emit_esp_bootloader_image(hdr.e_entry, &segments)
}

fn cstr_at(table: &[u8], off: usize) -> Result<&[u8], String> {
    let tail = table.get(off..).ok_or_else(|| {
        format!(
            "Section name offset {off} out of .shstrtab bounds ({})",
            table.len()
        )
    })?;
    let end = tail.iter().position(|&b| b == 0).unwrap_or(tail.len());
    Ok(&tail[..end])
}

/// Verifies that the ELF `.comment` section (if present) has no `GCC:` tag.
///
/// # Errors
/// Returns an error if the ELF header or section headers are malformed, or if
/// any `.comment` section contains the `b"GCC:"` substring.
pub fn verify_elf_no_gcc_comment(elf_bytes: &[u8]) -> Result<(), String> {
    let hdr = parse_elf32_header(elf_bytes)?;
    validate_shdr_table(&hdr, elf_bytes.len())?;
    if hdr.e_shnum == 0 {
        return Ok(());
    }
    if hdr.e_shstrndx >= hdr.e_shnum {
        return Err(format!(
            "Invalid e_shstrndx {} for {} sections",
            hdr.e_shstrndx, hdr.e_shnum
        ));
    }

    let shstr_hdr = hdr.e_shoff + hdr.e_shstrndx * hdr.e_shentsize;
    let shstr_off = read_u32_le(elf_bytes, shstr_hdr + 16)? as usize;
    let shstr_size = read_u32_le(elf_bytes, shstr_hdr + 20)? as usize;
    let shstr_end = shstr_off
        .checked_add(shstr_size)
        .ok_or_else(|| ".shstrtab offset+size overflow".to_string())?;
    let shstrtab = elf_bytes.get(shstr_off..shstr_end).ok_or_else(|| {
        format!(
            ".shstrtab [{shstr_off}..{shstr_end}] exceeds ELF length {}",
            elf_bytes.len()
        )
    })?;

    for idx in 0..hdr.e_shnum {
        let sh = hdr.e_shoff + idx * hdr.e_shentsize;
        let name_off = read_u32_le(elf_bytes, sh)? as usize;
        let sh_type = read_u32_le(elf_bytes, sh + 4)?;
        let sh_offset = read_u32_le(elf_bytes, sh + 16)? as usize;
        let sh_size = read_u32_le(elf_bytes, sh + 20)? as usize;

        if cstr_at(shstrtab, name_off)? == b".comment" && sh_type != SHT_NOBITS {
            let sec_end = sh_offset
                .checked_add(sh_size)
                .ok_or_else(|| ".comment offset+size overflow".to_string())?;
            let sec = elf_bytes.get(sh_offset..sec_end).ok_or_else(|| {
                format!(
                    ".comment [{sh_offset}..{sec_end}] exceeds ELF length {}",
                    elf_bytes.len()
                )
            })?;
            if sec.windows(4).any(|w| w == b"GCC:") {
                return Err(
                    "ELF .comment section contains forbidden 'GCC:' toolchain entry".to_string(),
                );
            }
        }
    }

    Ok(())
}

fn find_xtensa_gcc_bin_dirs() -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    let Some(home) = std::env::var_os("HOME") else {
        return dirs;
    };
    let root = PathBuf::from(home).join(".rustup/toolchains/esp/xtensa-esp-elf");
    let Ok(entries) = fs::read_dir(&root) else {
        return dirs;
    };
    for entry in entries.flatten() {
        let candidate = entry.path().join("xtensa-esp-elf/bin");
        if candidate.join("xtensa-esp32-elf-gcc").is_file() {
            dirs.push(candidate);
        }
    }
    dirs.sort();
    dirs.reverse();
    dirs
}

fn build_xtensa_child_path() -> Option<OsString> {
    let current = std::env::var_os("PATH").unwrap_or_default();
    let existing: Vec<PathBuf> = std::env::split_paths(&current).collect();
    if existing
        .iter()
        .any(|dir| dir.join("xtensa-esp32-elf-gcc").is_file())
    {
        return None;
    }

    let discovered = find_xtensa_gcc_bin_dirs();
    if discovered.is_empty() {
        return None;
    }
    let mut combined = discovered;
    combined.extend(existing);
    std::env::join_paths(combined).ok()
}

fn workspace_root() -> Result<PathBuf, String> {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    if let Some(root) = manifest_dir.parent().and_then(Path::parent) {
        return Ok(root.to_path_buf());
    }
    std::env::current_dir().map_err(|e| format!("Failed to resolve workspace root: {e}"))
}

struct BuildBootloaderArgs {
    elf_override: Option<PathBuf>,
    out_override: Option<PathBuf>,
}

fn parse_build_bootloader_args(args: &[String]) -> Result<Option<BuildBootloaderArgs>, String> {
    let mut elf_override: Option<PathBuf> = None;
    let mut out_override: Option<PathBuf> = None;
    let mut i = 0;

    while i < args.len() {
        match args[i].as_str() {
            "--elf" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| "Missing argument for --elf".to_string())?;
                elf_override = Some(PathBuf::from(val));
            }
            "--out" | "-o" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| "Missing argument for --out".to_string())?;
                out_override = Some(PathBuf::from(val));
            }
            "--help" | "-h" => {
                println!("Usage: cargo xtask build-bootloader [--elf <PATH>] [--out <PATH>]");
                return Ok(None);
            }
            other => {
                return Err(format!("Unknown option '{other}' for build-bootloader"));
            }
        }
        i += 1;
    }

    Ok(Some(BuildBootloaderArgs {
        elf_override,
        out_override,
    }))
}

/// Builds `esp32-cam-bootloader` and packs it into an ESP32 `0xE9` `.bin`.
///
/// # Errors
/// Returns an error if CLI arguments are invalid, `cargo +esp build` fails,
/// `.comment` contains `GCC:`, packing fails, or the `.bin` exceeds 28,672 B.
pub fn run_build_bootloader(args: &[String]) -> Result<(), String> {
    let Some(parsed) = parse_build_bootloader_args(args)? else {
        return Ok(());
    };

    let root = workspace_root()?;
    let rel_target = Path::new("target/xtensa-esp32-none-elf/release");
    let elf_path = parsed
        .elf_override
        .unwrap_or_else(|| root.join(rel_target).join("esp32-cam-bootloader"));
    let out_path = parsed
        .out_override
        .unwrap_or_else(|| root.join(rel_target).join("esp32-cam-bootloader.bin"));

    let mut cmd = Command::new("cargo");
    cmd.current_dir(&root)
        .args([
            "+esp",
            "build",
            "--target",
            "xtensa-esp32-none-elf",
            "-Zbuild-std=core,alloc",
            "-p",
            "esp32-cam-bootloader",
            "--release",
        ])
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("RUSTC");
    if let Some(child_path) = build_xtensa_child_path() {
        cmd.env("PATH", child_path);
    }

    let status = cmd
        .status()
        .map_err(|e| format!("Failed to invoke cargo +esp build: {e}"))?;
    if !status.success() {
        return Err(format!(
            "cargo +esp build for esp32-cam-bootloader failed with {status}"
        ));
    }

    let elf_bytes = fs::read(&elf_path).map_err(|e| {
        format!(
            "Failed to read bootloader ELF '{}': {e}",
            elf_path.display()
        )
    })?;
    verify_elf_no_gcc_comment(&elf_bytes)?;

    let bin_bytes = pack_esp_bootloader_bin(&elf_bytes)?;
    if bin_bytes.len() > BOOTLOADER_MAX_BYTES {
        return Err(format!(
            "Packed bootloader size {} bytes exceeds {BOOTLOADER_MAX_BYTES} (0x7000) limit",
            bin_bytes.len()
        ));
    }

    if let Some(parent) = out_path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent).map_err(|e| {
                format!(
                    "Failed to create output directory '{}': {e}",
                    parent.display()
                )
            })?;
        }
    }
    fs::write(&out_path, &bin_bytes).map_err(|e| {
        format!(
            "Failed to write bootloader image '{}': {e}",
            out_path.display()
        )
    })?;

    println!(
        "Built pure-Rust bootloader: {} ({} B ELF -> {} B / {BOOTLOADER_MAX_BYTES} B max, \
         0 GCC: comments)",
        out_path.display(),
        elf_bytes.len(),
        bin_bytes.len()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const REF_BOOTLOADER_BIN: &str = "/home/console-admin/.gemini/antigravity/brain/\
        a41b5f83-80dc-4a4b-afc9-4e3e04a36b8a/scratch/backups/phase2/\
        bootloader_v554_gpio16.bin";

    fn build_synthetic_elf_with_comment(comment: Option<&[u8]>) -> Vec<u8> {
        let mut shstrtab = vec![0u8];
        let shstr_name = shstrtab.len() as u32;
        shstrtab.extend_from_slice(b".shstrtab\0");
        let comment_name = shstrtab.len() as u32;
        if comment.is_some() {
            shstrtab.extend_from_slice(b".comment\0");
        }

        let mut payload = vec![0x00, 0x00, 0x00, 0x00];
        let mut comment_off = 0usize;
        let mut comment_len = 0usize;
        let mut elf = vec![0u8; ELF_EHDR_SIZE + ELF_PHDR_SIZE];
        let seg_off = elf.len();
        elf.extend_from_slice(&payload);
        if let Some(c) = comment {
            comment_off = elf.len();
            comment_len = c.len();
            elf.extend_from_slice(c);
        } else {
            let _ = &mut payload;
        }
        let shstr_off = elf.len();
        elf.extend_from_slice(&shstrtab);
        while !elf.len().is_multiple_of(4) {
            elf.push(0);
        }
        let shoff = elf.len();
        let shnum = if comment.is_some() { 3u16 } else { 2u16 };
        let shstrndx = 1u16;

        // Fill ELF32 header
        elf[0..4].copy_from_slice(b"\x7fELF");
        elf[4] = ELFCLASS32;
        elf[5] = ELFDATA2LSB;
        elf[6] = EV_CURRENT;
        elf[16..18].copy_from_slice(&ET_EXEC.to_le_bytes());
        elf[18..20].copy_from_slice(&EM_XTENSA.to_le_bytes());
        elf[20..24].copy_from_slice(&1u32.to_le_bytes());
        elf[24..28].copy_from_slice(&0x4007_8000u32.to_le_bytes());
        elf[28..32].copy_from_slice(&(ELF_EHDR_SIZE as u32).to_le_bytes());
        elf[32..36].copy_from_slice(&(shoff as u32).to_le_bytes());
        elf[40..42].copy_from_slice(&(ELF_EHDR_SIZE as u16).to_le_bytes());
        elf[42..44].copy_from_slice(&(ELF_PHDR_SIZE as u16).to_le_bytes());
        elf[44..46].copy_from_slice(&1u16.to_le_bytes());
        elf[46..48].copy_from_slice(&(ELF_SHDR_SIZE as u16).to_le_bytes());
        elf[48..50].copy_from_slice(&shnum.to_le_bytes());
        elf[50..52].copy_from_slice(&shstrndx.to_le_bytes());

        // Single PT_LOAD program header
        let ph = ELF_EHDR_SIZE;
        elf[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
        elf[ph + 4..ph + 8].copy_from_slice(&(seg_off as u32).to_le_bytes());
        elf[ph + 8..ph + 12].copy_from_slice(&0x4007_8000u32.to_le_bytes());
        elf[ph + 12..ph + 16].copy_from_slice(&0x4007_8000u32.to_le_bytes());
        elf[ph + 16..ph + 20].copy_from_slice(&4u32.to_le_bytes());
        elf[ph + 20..ph + 24].copy_from_slice(&4u32.to_le_bytes());
        elf[ph + 24..ph + 28].copy_from_slice(&(PF_R | PF_X).to_le_bytes());
        elf[ph + 28..ph + 32].copy_from_slice(&4u32.to_le_bytes());

        // Section 0: NULL
        elf.extend_from_slice(&[0u8; ELF_SHDR_SIZE]);
        // Section 1: .shstrtab
        elf.extend_from_slice(&shstr_name.to_le_bytes());
        elf.extend_from_slice(&SHT_STRTAB.to_le_bytes());
        elf.extend_from_slice(&[0u8; 8]);
        elf.extend_from_slice(&(shstr_off as u32).to_le_bytes());
        elf.extend_from_slice(&(shstrtab.len() as u32).to_le_bytes());
        elf.extend_from_slice(&[0u8; 16]);
        // Section 2: .comment (optional)
        if comment.is_some() {
            elf.extend_from_slice(&comment_name.to_le_bytes());
            elf.extend_from_slice(&SHT_PROGBITS.to_le_bytes());
            elf.extend_from_slice(&[0u8; 8]);
            elf.extend_from_slice(&(comment_off as u32).to_le_bytes());
            elf.extend_from_slice(&(comment_len as u32).to_le_bytes());
            elf.extend_from_slice(&[0u8; 16]);
        }
        elf
    }

    fn build_synthetic_multi_segment_elf() -> Vec<u8> {
        let segments: &[(u32, &[u8], u32)] = &[
            (
                0x3FFF_0030,
                &[0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB, 0xCC, 0xDD],
                PF_R | PF_W,
            ),
            (
                0x4007_8000,
                &[
                    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                ],
                PF_R | PF_X,
            ),
            (
                0x4008_0400,
                &[0x36, 0x41, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00],
                PF_R | PF_X,
            ),
        ];
        let shstrtab = b"\0.shstrtab\0";
        let header_end = ELF_EHDR_SIZE + segments.len() * ELF_PHDR_SIZE;
        let mut elf = vec![0u8; header_end];

        let mut seg_offsets = Vec::with_capacity(segments.len());
        for &(_, payload, _) in segments {
            let off = elf.len();
            seg_offsets.push(off);
            elf.extend_from_slice(payload);
            while !elf.len().is_multiple_of(4) {
                elf.push(0);
            }
        }

        let shstr_off = elf.len();
        elf.extend_from_slice(shstrtab);
        while !elf.len().is_multiple_of(4) {
            elf.push(0);
        }
        let shoff = elf.len();

        elf[0..4].copy_from_slice(b"\x7fELF");
        elf[4] = ELFCLASS32;
        elf[5] = ELFDATA2LSB;
        elf[6] = EV_CURRENT;
        elf[16..18].copy_from_slice(&ET_EXEC.to_le_bytes());
        elf[18..20].copy_from_slice(&EM_XTENSA.to_le_bytes());
        elf[20..24].copy_from_slice(&1u32.to_le_bytes());
        elf[24..28].copy_from_slice(&0x4008_0400u32.to_le_bytes());
        elf[28..32].copy_from_slice(&(ELF_EHDR_SIZE as u32).to_le_bytes());
        elf[32..36].copy_from_slice(&(shoff as u32).to_le_bytes());
        elf[40..42].copy_from_slice(&(ELF_EHDR_SIZE as u16).to_le_bytes());
        elf[42..44].copy_from_slice(&(ELF_PHDR_SIZE as u16).to_le_bytes());
        elf[44..46].copy_from_slice(&(segments.len() as u16).to_le_bytes());
        elf[46..48].copy_from_slice(&(ELF_SHDR_SIZE as u16).to_le_bytes());
        elf[48..50].copy_from_slice(&2u16.to_le_bytes());
        elf[50..52].copy_from_slice(&1u16.to_le_bytes());

        for (idx, &(vaddr, payload, flags)) in segments.iter().enumerate() {
            let ph = ELF_EHDR_SIZE + idx * ELF_PHDR_SIZE;
            let filesz = payload.len() as u32;
            elf[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
            elf[ph + 4..ph + 8].copy_from_slice(&(seg_offsets[idx] as u32).to_le_bytes());
            elf[ph + 8..ph + 12].copy_from_slice(&vaddr.to_le_bytes());
            elf[ph + 12..ph + 16].copy_from_slice(&vaddr.to_le_bytes());
            elf[ph + 16..ph + 20].copy_from_slice(&filesz.to_le_bytes());
            elf[ph + 20..ph + 24].copy_from_slice(&filesz.to_le_bytes());
            elf[ph + 24..ph + 28].copy_from_slice(&flags.to_le_bytes());
            elf[ph + 28..ph + 32].copy_from_slice(&4u32.to_le_bytes());
        }

        // Section 0: NULL
        elf.extend_from_slice(&[0u8; ELF_SHDR_SIZE]);
        // Section 1: .shstrtab
        elf.extend_from_slice(&1u32.to_le_bytes());
        elf.extend_from_slice(&SHT_STRTAB.to_le_bytes());
        elf.extend_from_slice(&[0u8; 8]);
        elf.extend_from_slice(&(shstr_off as u32).to_le_bytes());
        elf.extend_from_slice(&(shstrtab.len() as u32).to_le_bytes());
        elf.extend_from_slice(&[0u8; 16]);
        elf
    }

    #[test]
    fn round_trip_matches_bootloader_v554_gpio16_bin_byte_for_byte() {
        if !Path::new(REF_BOOTLOADER_BIN).is_file() {
            return;
        }
        let original = fs::read(REF_BOOTLOADER_BIN).expect("read reference bootloader bin");
        assert_eq!(original.len(), 26_256);

        let wrapped_elf =
            wrap_esp_image_to_elf(&original, None).expect("wrap reference bin to ELF");
        verify_elf_no_gcc_comment(&wrapped_elf).expect("wrapped ELF has no GCC comment");

        let repacked = pack_esp_bootloader_bin(&wrapped_elf).expect("repack ELF to bin");
        assert_eq!(repacked.len(), original.len());
        assert_eq!(repacked, original);
    }

    #[test]
    fn round_trip_synthetic_multi_segment_bootloader_image() {
        let elf = build_synthetic_multi_segment_elf();
        verify_elf_no_gcc_comment(&elf).expect("synthetic ELF has no GCC comment");

        let bin = pack_esp_bootloader_bin(&elf).expect("pack synthetic multi-segment ELF");
        assert_eq!(bin[0], ESP_IMAGE_HEADER_MAGIC);
        assert_eq!(bin[1], 3);
        assert_eq!(
            u32::from_le_bytes([bin[4], bin[5], bin[6], bin[7]]),
            0x4008_0400
        );

        let elf2 = wrap_esp_image_to_elf(&bin, None).expect("wrap multi-segment bin to ELF");
        verify_elf_no_gcc_comment(&elf2).expect("wrapped ELF has no GCC comment");

        let bin2 = pack_esp_bootloader_bin(&elf2).expect("repack wrapped multi-segment ELF");
        assert_eq!(bin2, bin);
    }

    #[test]
    fn verify_elf_no_gcc_comment_rejects_gcc_and_accepts_rustc() {
        let rustc_elf = build_synthetic_elf_with_comment(Some(b"rustc version 1.97.0-nightly\0"));
        assert!(verify_elf_no_gcc_comment(&rustc_elf).is_ok());

        let no_comment_elf = build_synthetic_elf_with_comment(None);
        assert!(verify_elf_no_gcc_comment(&no_comment_elf).is_ok());

        let gcc_elf = build_synthetic_elf_with_comment(Some(
            b"rustc version 1.97.0\0GCC: (crosstool-NG esp-16.2.0) 16.2.0\0",
        ));
        let err = verify_elf_no_gcc_comment(&gcc_elf).expect_err("GCC: must be rejected");
        assert!(err.contains("GCC:"));
    }

    #[test]
    fn pack_esp_bootloader_bin_rejects_corrupt_phdr_and_pads_segments() {
        let mut bad_elf = build_synthetic_elf_with_comment(None);
        // Set p_filesz (8) > p_memsz (4) on PT_LOAD[0]
        let ph = ELF_EHDR_SIZE;
        bad_elf[ph + 16..ph + 20].copy_from_slice(&8u32.to_le_bytes());
        let err = pack_esp_bootloader_bin(&bad_elf).expect_err("p_filesz > p_memsz must fail");
        assert!(err.contains("p_filesz"));

        // Non-4-byte segment payload gets zero-padded to 4-byte boundary.
        let mut unaligned_elf = build_synthetic_elf_with_comment(None);
        unaligned_elf[ph + 16..ph + 20].copy_from_slice(&3u32.to_le_bytes());
        unaligned_elf[ph + 20..ph + 24].copy_from_slice(&3u32.to_le_bytes());
        unaligned_elf[ph + 32..ph + 36].copy_from_slice(&[0x11, 0x22, 0x33, 0x99]);
        let bin = pack_esp_bootloader_bin(&unaligned_elf).expect("pack unaligned segment");
        assert_eq!(u32::from_le_bytes([bin[28], bin[29], bin[30], bin[31]]), 4);
        assert_eq!(&bin[32..36], &[0x11, 0x22, 0x33, 0x00]);
        assert!(wrap_esp_image_to_elf(&bin, None).is_ok());
    }
}
