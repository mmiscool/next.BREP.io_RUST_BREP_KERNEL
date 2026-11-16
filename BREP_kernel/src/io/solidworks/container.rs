//! The two containers a SolidWorks document comes in: an OLE compound file
//! (before 2015) and SolidWorks' own ZIP-like archive (2015 and later).
//! Only the streams a caller wants are read.

/// The bound on any one stream's size (bytes): a part's partition is a few
/// megabytes; a stream claiming more than this is refused, not allocated.
const MAX_STREAM: usize = 1 << 30;

/// The streams of a SolidWorks document whose path (`Storage/Stream`) `want`
/// accepts, decompressed.
pub fn solidworks_streams(bytes: &[u8], want: &dyn Fn(&str) -> bool) -> Result<Vec<(String, Vec<u8>)>, String> {
    if bytes.starts_with(&[0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1]) {
        return ole_streams(bytes, want);
    }
    if bytes.len() >= 8 && bytes[4..8] == [0, 0, 0, 4] {
        return archive_streams(bytes, want);
    }
    Err("not a SolidWorks document (neither an OLE compound file nor a SolidWorks archive)".into())
}

fn u16_at(data: &[u8], at: usize) -> Result<usize, String> {
    data.get(at..at + 2).map(|b| u16::from_le_bytes([b[0], b[1]]) as usize).ok_or_else(|| "the file is cut short".to_string())
}

fn u32_at(data: &[u8], at: usize) -> Result<usize, String> {
    data.get(at..at + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).ok_or_else(|| "the file is cut short".to_string())
}

/// SolidWorks' archive: an eight-byte prefix whose last byte is the key the
/// stream names are rotated by, then a ZIP-like body whose end record holds
/// the entry count twice, the directory's size and offset. Each directory
/// entry gives the CRC, sizes, the name's length (and the extra and comment
/// lengths), the local header's offset and the rotated name; the data is raw
/// DEFLATE after the local header.
fn archive_streams(bytes: &[u8], want: &dyn Fn(&str) -> bool) -> Result<Vec<(String, Vec<u8>)>, String> {
    let key = u32::from(bytes[7] & 7);
    let body = &bytes[8..];
    let mut directory = None;
    if body.len() >= 22 {
        for i in (0..=body.len() - 22).rev() {
            let (Ok(a), Ok(b), Ok(size), Ok(offset), Ok(zero)) =
                (u16_at(body, i + 8), u16_at(body, i + 10), u32_at(body, i + 12), u32_at(body, i + 16), u16_at(body, i + 20))
            else {
                continue;
            };
            if a == b && a > 0 && offset + size == i && zero == 0 && body[i + 4..i + 8] == [0, 0, 0, 0] {
                directory = Some((a, offset));
                break;
            }
        }
    }
    let (count, offset) = directory.ok_or("the SolidWorks archive has no end record")?;
    let mut out = Vec::new();
    let mut at = offset;
    for _ in 0..count {
        let crc = u32_at(body, at + 16)? as u32;
        let packed = u32_at(body, at + 20)?;
        let full = u32_at(body, at + 24)?;
        let (name_len, extra_len, comment_len) = (u16_at(body, at + 28)?, u16_at(body, at + 30)?, u16_at(body, at + 32)?);
        let local = u32_at(body, at + 42)?;
        let raw_name = body.get(at + 46..at + 46 + name_len).ok_or("an archive entry's name runs past the directory")?;
        let name: String = raw_name.iter().map(|&c| char::from(c.rotate_right(key))).collect();
        at += 46 + name_len + extra_len + comment_len;
        if !want(&name) {
            continue;
        }
        if full > MAX_STREAM {
            return Err(format!("{name} claims {full} bytes"));
        }
        let start = local + 30 + u16_at(body, local + 26)? + u16_at(body, local + 28)?;
        let data = body.get(start..start + packed).ok_or_else(|| format!("{name}'s data runs past the archive"))?;
        let data = if packed == 0 {
            Vec::new()
        } else {
            crate::three_mf::inflate::inflate(data, full.max(1)).map_err(|e| format!("{name}: {e}"))?
        };
        if data.len() != full || crc32(&data) != crc {
            return Err(format!("{name} does not inflate to its stated size and checksum"));
        }
        out.push((name, data));
    }
    Ok(out)
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// An OLE compound file ([MS-CFB]): sectors chained by the FAT (whose own
/// sectors the header's DIFAT lists), a directory of 128-byte entries in a
/// red-black tree, and streams below the cutoff in the mini stream (64-byte
/// sectors chained by the mini FAT).
fn ole_streams(bytes: &[u8], want: &dyn Fn(&str) -> bool) -> Result<Vec<(String, Vec<u8>)>, String> {
    const END: usize = 0xFFFF_FFFE;
    const FREE: usize = 0xFFFF_FFFF;
    let shift = u16_at(bytes, 30)?;
    let mini_shift = u16_at(bytes, 32)?;
    if !(7..=16).contains(&shift) || !(2..=shift).contains(&mini_shift) {
        return Err(format!("an OLE file with sector shift {shift}"));
    }
    let size = 1usize << shift;
    let mini = 1usize << mini_shift;
    let first_dir = u32_at(bytes, 48)?;
    let cutoff = u32_at(bytes, 56)?;
    let first_mini_fat = u32_at(bytes, 60)?;
    let mut difat_next = u32_at(bytes, 68)?;
    let sector = |n: usize| -> Result<&[u8], String> {
        let at = (n + 1).checked_mul(size).ok_or("an OLE sector number overflows")?;
        bytes.get(at..at + size).ok_or_else(|| format!("OLE sector {n} is past the file's end"))
    };
    // The FAT's own sectors: the header's 109, then the DIFAT chain.
    let mut fat_sectors = Vec::new();
    for k in 0..109 {
        let s = u32_at(bytes, 76 + 4 * k)?;
        if s != FREE && s != END {
            fat_sectors.push(s);
        }
    }
    let mut guard = 0;
    while difat_next != END && difat_next != FREE {
        guard += 1;
        if guard > bytes.len() / size + 1 {
            return Err("the OLE DIFAT chain loops".into());
        }
        let s = sector(difat_next)?;
        for k in 0..size / 4 - 1 {
            let n = u32_at(s, 4 * k)?;
            if n != FREE && n != END {
                fat_sectors.push(n);
            }
        }
        difat_next = u32_at(s, size - 4)?;
    }
    let mut fat = Vec::with_capacity(fat_sectors.len() * size / 4);
    for n in fat_sectors {
        let s = sector(n)?;
        for k in 0..size / 4 {
            fat.push(u32_at(s, 4 * k)?);
        }
    }
    let chain = |start: usize, table: &[usize]| -> Result<Vec<usize>, String> {
        let mut out = Vec::new();
        let mut at = start;
        while at != END {
            if at >= table.len() || out.len() > table.len() {
                return Err("an OLE sector chain runs off its table or loops".into());
            }
            out.push(at);
            at = table[at];
        }
        Ok(out)
    };
    let read_chain = |start: usize| -> Result<Vec<u8>, String> {
        let mut out = Vec::new();
        for n in chain(start, &fat)? {
            out.extend_from_slice(sector(n)?);
            if out.len() > MAX_STREAM {
                return Err("an OLE stream is larger than any part".into());
            }
        }
        Ok(out)
    };
    let directory = read_chain(first_dir)?;
    struct Entry {
        name: String,
        kind: u8,
        left: usize,
        right: usize,
        child: usize,
        start: usize,
        size: usize,
    }
    let entries: Vec<Entry> = directory
        .chunks_exact(128)
        .map(|e| {
            let name_len = (u16::from_le_bytes([e[64], e[65]]) as usize).min(64);
            let units: Vec<u16> = e[..name_len.saturating_sub(2)].chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
            Entry {
                name: String::from_utf16_lossy(&units),
                kind: e[66],
                left: u32::from_le_bytes([e[68], e[69], e[70], e[71]]) as usize,
                right: u32::from_le_bytes([e[72], e[73], e[74], e[75]]) as usize,
                child: u32::from_le_bytes([e[76], e[77], e[78], e[79]]) as usize,
                start: u32::from_le_bytes([e[116], e[117], e[118], e[119]]) as usize,
                size: u32::from_le_bytes([e[120], e[121], e[122], e[123]]) as usize,
            }
        })
        .collect();
    let root = entries.first().ok_or("the OLE file has no directory")?;
    // The mini stream (the root's data) and its FAT, read only if needed.
    let mut mini_stream: Option<Vec<u8>> = None;
    let mut mini_fat: Option<Vec<usize>> = None;
    // Walk the tree: each storage's children hang from its `child` as a
    // binary tree through `left` and `right`.
    let mut paths: Vec<(String, usize)> = Vec::new();
    let mut stack: Vec<(usize, String)> = vec![(root.child, String::new())];
    let mut visits = 0;
    while let Some((at, prefix)) = stack.pop() {
        if at >= entries.len() {
            continue;
        }
        visits += 1;
        if visits > 4 * entries.len() {
            return Err("the OLE directory tree loops".into());
        }
        let e = &entries[at];
        stack.push((e.left, prefix.clone()));
        stack.push((e.right, prefix.clone()));
        match e.kind {
            1 => stack.push((e.child, format!("{prefix}{}/", e.name))),
            2 => paths.push((format!("{prefix}{}", e.name), at)),
            _ => {}
        }
    }
    let mut out = Vec::new();
    for (path, at) in paths {
        if !want(&path) {
            continue;
        }
        let e = &entries[at];
        if e.size > MAX_STREAM {
            return Err(format!("{path} claims {} bytes", e.size));
        }
        let mut data = if e.size < cutoff {
            if mini_stream.is_none() {
                mini_stream = Some(read_chain(root.start)?);
                let table = read_chain(first_mini_fat)?;
                mini_fat = Some(table.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]) as usize).collect());
            }
            let (stream, table) = (mini_stream.as_ref().unwrap(), mini_fat.as_ref().unwrap());
            let mut d = Vec::new();
            for n in chain(e.start, table)? {
                d.extend_from_slice(stream.get(n * mini..(n + 1) * mini).ok_or_else(|| format!("{path}'s mini sector {n} is past the mini stream"))?);
            }
            d
        } else {
            read_chain(e.start)?
        };
        if data.len() < e.size {
            return Err(format!("{path} is shorter than its stated {} bytes", e.size));
        }
        data.truncate(e.size);
        out.push((path, data));
    }
    Ok(out)
}
