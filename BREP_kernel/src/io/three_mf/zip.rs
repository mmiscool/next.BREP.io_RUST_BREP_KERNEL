//! A bounded, read-only ZIP (OPC package) reader.
//!
//! Only what an OPC package needs: the end-of-central-directory record, the
//! central directory, and one named entry at a time. Entries are located
//! through the CENTRAL directory, never by walking local headers, so a package
//! whose local headers carry a data descriptor (flag bit 3, sizes written
//! after the data — what most slicers emit) reads correctly.
//!
//! Refused by design, each with the entry named: encryption, ZIP64, multi-disk
//! packages, compression methods other than stored (0) and deflate (8), and any
//! entry whose declared uncompressed size exceeds the caller's cap. Every
//! offset is computed with `checked_add`: the kernel's release profile sets
//! `overflow-checks = false`, so a wrapped offset would otherwise index inside
//! the file instead of failing.

use super::inflate::inflate;

const END_OF_CENTRAL_DIRECTORY: u32 = 0x0605_4b50;
const CENTRAL_FILE_HEADER: u32 = 0x0201_4b50;
const LOCAL_FILE_HEADER: u32 = 0x0403_4b50;
/// The EOCD record is 22 bytes plus a comment of at most 64 KiB.
const MAX_END_RECORD_SEARCH: usize = 22 + 0xffff;
/// A 3MF package is a handful of parts; thousands mean a corpus, not a model.
pub const MAX_ENTRIES: usize = 4096;
/// `0xffff` / `0xffffffff` in a size or count field means "see the ZIP64
/// record", which this reader does not implement.
const ZIP64_MARKER_16: u16 = 0xffff;
const ZIP64_MARKER_32: u32 = 0xffff_ffff;

/// What kind of fault a [`ZipError`] reports, so the caller can map it onto a
/// typed 3MF refusal instead of guessing from the message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZipFault {
    /// The bytes are not a readable package.
    Container,
    /// One entry's bytes are unreadable: a bad header, a short read, a failed
    /// CRC.
    Damaged,
    /// A container feature this reader refuses on purpose.
    Unsupported,
    /// A declared size or count past one of the reader's bounds.
    Bounds,
}

/// Why a package could not be read. `entry` names the archive member when the
/// failure belongs to one rather than to the container.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ZipError {
    pub entry: Option<String>,
    pub message: String,
    pub fault: ZipFault,
}

impl ZipError {
    fn container(message: impl Into<String>) -> Self {
        Self { entry: None, message: message.into(), fault: ZipFault::Container }
    }
    fn damaged(name: &str, message: impl Into<String>) -> Self {
        Self {
            entry: Some(name.to_owned()),
            message: message.into(),
            fault: ZipFault::Damaged,
        }
    }
    fn bounds(name: &str, message: impl Into<String>) -> Self {
        Self {
            entry: Some(name.to_owned()),
            message: message.into(),
            fault: ZipFault::Bounds,
        }
    }
    fn unsupported(entry: Option<&str>, message: impl Into<String>) -> Self {
        Self {
            entry: entry.map(str::to_owned),
            message: message.into(),
            fault: ZipFault::Unsupported,
        }
    }
}

impl core::fmt::Display for ZipError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match &self.entry {
            Some(entry) => write!(f, "{entry}: {}", self.message),
            None => f.write_str(&self.message),
        }
    }
}

/// One central-directory record: what the package says about an entry.
#[derive(Clone, Debug)]
pub struct ZipEntry {
    pub name: String,
    method: u16,
    crc32: u32,
    compressed_size: usize,
    uncompressed_size: usize,
    local_header_offset: usize,
}

/// The parsed central directory of an in-memory package.
pub struct ZipArchive<'a> {
    data: &'a [u8],
    entries: Vec<ZipEntry>,
}

fn u16_at(data: &[u8], at: usize) -> Result<u16, ZipError> {
    let bytes = data
        .get(at..at.checked_add(2).ok_or_else(|| ZipError::container("ZIP offset overflow"))?)
        .ok_or_else(|| ZipError::container("ZIP record ends before its fields"))?;
    Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
}

fn u32_at(data: &[u8], at: usize) -> Result<u32, ZipError> {
    let bytes = data
        .get(at..at.checked_add(4).ok_or_else(|| ZipError::container("ZIP offset overflow"))?)
        .ok_or_else(|| ZipError::container("ZIP record ends before its fields"))?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn add(a: usize, b: usize) -> Result<usize, ZipError> {
    a.checked_add(b)
        .ok_or_else(|| ZipError::container("ZIP offset arithmetic overflowed"))
}

/// CRC-32 (IEEE 802.3), computed bitwise so no static table is needed.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

impl<'a> ZipArchive<'a> {
    /// Parse the central directory of `data`. Nothing is decompressed here.
    pub fn open(data: &'a [u8]) -> Result<Self, ZipError> {
        let end = find_end_record(data)?;
        if u16_at(data, end + 4)? != 0 || u16_at(data, end + 6)? != 0 {
            return Err(ZipError::unsupported(None, "the package spans several disks"));
        }
        let entry_count = u16_at(data, end + 10)?;
        let directory_size = u32_at(data, end + 12)?;
        let directory_offset = u32_at(data, end + 16)?;
        if entry_count == ZIP64_MARKER_16
            || directory_size == ZIP64_MARKER_32
            || directory_offset == ZIP64_MARKER_32
        {
            return Err(ZipError::unsupported(None, "the package is ZIP64"));
        }
        let entry_count = entry_count as usize;
        if entry_count > MAX_ENTRIES {
            return Err(ZipError::container(format!(
                "the package declares {entry_count} entries; at most {MAX_ENTRIES} are read"
            )));
        }
        let directory_offset = directory_offset as usize;
        let directory_end = add(directory_offset, directory_size as usize)?;
        if directory_end > data.len() {
            return Err(ZipError::container(
                "the central directory runs past the end of the package",
            ));
        }
        let mut entries = Vec::with_capacity(entry_count);
        let mut at = directory_offset;
        for _ in 0..entry_count {
            if u32_at(data, at)? != CENTRAL_FILE_HEADER {
                return Err(ZipError::container(
                    "a central-directory record has the wrong signature",
                ));
            }
            let flags = u16_at(data, add(at, 8)?)?;
            let method = u16_at(data, add(at, 10)?)?;
            let crc = u32_at(data, add(at, 16)?)?;
            let compressed_size = u32_at(data, add(at, 20)?)?;
            let uncompressed_size = u32_at(data, add(at, 24)?)?;
            let name_length = u16_at(data, add(at, 28)?)? as usize;
            let extra_length = u16_at(data, add(at, 30)?)? as usize;
            let comment_length = u16_at(data, add(at, 32)?)? as usize;
            let local_header_offset = u32_at(data, add(at, 42)?)?;
            let name_start = add(at, 46)?;
            let name_end = add(name_start, name_length)?;
            let name_bytes = data
                .get(name_start..name_end)
                .ok_or_else(|| ZipError::container("an entry name runs past the package"))?;
            let name = String::from_utf8_lossy(name_bytes).into_owned();
            if flags & 0x1 != 0 {
                return Err(ZipError::unsupported(Some(&name), "the entry is encrypted"));
            }
            if compressed_size == ZIP64_MARKER_32
                || uncompressed_size == ZIP64_MARKER_32
                || local_header_offset == ZIP64_MARKER_32
            {
                return Err(ZipError::unsupported(Some(&name), "the entry is ZIP64"));
            }
            entries.push(ZipEntry {
                name,
                method,
                crc32: crc,
                compressed_size: compressed_size as usize,
                uncompressed_size: uncompressed_size as usize,
                local_header_offset: local_header_offset as usize,
            });
            at = add(add(name_end, extra_length)?, comment_length)?;
        }
        Ok(Self { data, entries })
    }

    /// Find an entry by OPC part name. A leading `/` is dropped (part names are
    /// absolute in the package, relative in the archive) and the ASCII-case
    /// -insensitive spelling is accepted as a fallback, because packages differ
    /// on `3D/3dmodel.model` vs `3d/3dmodel.model`.
    pub fn find(&self, name: &str) -> Option<&ZipEntry> {
        let wanted = name.trim_start_matches('/');
        self.entries
            .iter()
            .find(|entry| entry.name == wanted)
            .or_else(|| {
                self.entries
                    .iter()
                    .find(|entry| entry.name.eq_ignore_ascii_case(wanted))
            })
    }

    /// Decompress one entry, refusing before any work if it declares more than
    /// `cap` bytes. The CRC-32 and the declared length are both verified.
    pub fn read(&self, entry: &ZipEntry, cap: usize) -> Result<Vec<u8>, ZipError> {
        if entry.uncompressed_size > cap {
            return Err(ZipError::bounds(
                &entry.name,
                format!(
                    "the part declares {} bytes; at most {cap} are read",
                    entry.uncompressed_size
                ),
            ));
        }
        if u32_at(self.data, entry.local_header_offset)? != LOCAL_FILE_HEADER {
            return Err(ZipError::damaged(
                &entry.name,
                "the local file header has the wrong signature",
            ));
        }
        // The LOCAL header's name and extra lengths are authoritative for the
        // data offset; they need not match the central directory's.
        let name_length = u16_at(self.data, add(entry.local_header_offset, 26)?)? as usize;
        let extra_length = u16_at(self.data, add(entry.local_header_offset, 28)?)? as usize;
        let start = add(add(add(entry.local_header_offset, 30)?, name_length)?, extra_length)?;
        let end = add(start, entry.compressed_size)?;
        let compressed = self
            .data
            .get(start..end)
            .ok_or_else(|| ZipError::damaged(&entry.name, "the entry data runs past the package"))?;
        let plain = match entry.method {
            0 => {
                if entry.compressed_size != entry.uncompressed_size {
                    return Err(ZipError::damaged(
                        &entry.name,
                        "the stored entry's sizes disagree",
                    ));
                }
                compressed.to_vec()
            }
            8 => inflate(compressed, entry.uncompressed_size)
                .map_err(|error| ZipError::damaged(&entry.name, error.0))?,
            method => {
                return Err(ZipError::unsupported(
                    Some(&entry.name),
                    format!("compression method {method} is not read (only stored and deflate)"),
                ))
            }
        };
        if plain.len() != entry.uncompressed_size {
            return Err(ZipError::damaged(
                &entry.name,
                format!(
                    "the part expanded to {} bytes, not the declared {}",
                    plain.len(),
                    entry.uncompressed_size
                ),
            ));
        }
        if crc32(&plain) != entry.crc32 {
            return Err(ZipError::damaged(&entry.name, "the part fails its CRC-32 check"));
        }
        Ok(plain)
    }
}

/// Scan backwards for the end-of-central-directory signature.
fn find_end_record(data: &[u8]) -> Result<usize, ZipError> {
    if data.len() < 22 {
        return Err(ZipError::container(
            "the file is too short to be a ZIP package",
        ));
    }
    let earliest = data.len().saturating_sub(MAX_END_RECORD_SEARCH);
    let mut at = data.len() - 22;
    loop {
        if u32_at(data, at)? == END_OF_CENTRAL_DIRECTORY {
            return Ok(at);
        }
        if at == earliest {
            return Err(ZipError::container(
                "no ZIP end-of-central-directory record (not a 3MF package)",
            ));
        }
        at -= 1;
    }
}
