//! DEFLATE decompression (RFC 1951) with a hard output bound.
//!
//! The kernel hand-rolls this for the same reason it hand-rolls the STEP, IGES
//! and STL codecs: `BREP_kernel` is published to crates.io and compiled to
//! wasm with nine runtime dependencies, and the alternatives (`zip`, `flate2`)
//! are absent from its lock file — see the dependency review in the module doc
//! of [`super`].
//!
//! The decoder is deliberately small: stored, fixed-Huffman and dynamic-Huffman
//! blocks, no preset dictionary, no zlib/gzip wrapper. Every output byte is
//! counted against `limit`, so a declared-size mismatch or a decompression bomb
//! stops at the bound instead of growing the heap.

/// Why a deflate stream could not be decoded. The caller turns this into a
/// [`super::ThreeMfError`] that names the package part it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InflateError(pub String);

impl core::fmt::Display for InflateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

fn error<T>(message: impl Into<String>) -> Result<T, InflateError> {
    Err(InflateError(message.into()))
}

/// LSB-first bit reader over the compressed bytes.
struct BitReader<'a> {
    data: &'a [u8],
    /// Index of the next byte to pull into `bits`.
    position: usize,
    /// Buffered bits, LSB first.
    bits: u32,
    /// Number of valid bits in `bits`.
    count: u32,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, position: 0, bits: 0, count: 0 }
    }

    /// Read `n` bits (n <= 24), refilling from the byte stream as needed.
    fn bits(&mut self, n: u32) -> Result<u32, InflateError> {
        while self.count < n {
            let Some(byte) = self.data.get(self.position) else {
                return error("deflate stream ends inside a block");
            };
            self.position += 1;
            self.bits |= u32::from(*byte) << self.count;
            self.count += 8;
        }
        let value = self.bits & ((1u32 << n) - 1);
        self.bits >>= n;
        self.count -= n;
        Ok(value)
    }

    /// Drop the partial byte and return to byte alignment (stored blocks).
    fn align(&mut self) {
        let whole = self.count % 8;
        self.bits >>= whole;
        self.count -= whole;
    }

    /// Take `n` whole bytes, first from the bit buffer and then from the input.
    fn bytes(&mut self, n: usize) -> Result<Vec<u8>, InflateError> {
        let mut out = Vec::with_capacity(n);
        while out.len() < n {
            if self.count >= 8 {
                out.push((self.bits & 0xff) as u8);
                self.bits >>= 8;
                self.count -= 8;
                continue;
            }
            let Some(byte) = self.data.get(self.position) else {
                return error("deflate stream ends inside a stored block");
            };
            self.position += 1;
            out.push(*byte);
        }
        Ok(out)
    }
}

/// A canonical Huffman decoding table: how many codes share each bit length,
/// and the symbols in canonical order. Decoding walks the lengths one bit at a
/// time, which needs no lookup table and cannot index out of range.
struct Huffman {
    counts: [u16; 16],
    symbols: Vec<u16>,
}

impl Huffman {
    fn new(lengths: &[u8]) -> Result<Self, InflateError> {
        let mut counts = [0u16; 16];
        for &length in lengths {
            if length as usize > 15 {
                return error("deflate Huffman code longer than 15 bits");
            }
            counts[length as usize] += 1;
        }
        counts[0] = 0;
        // An over-subscribed set would decode symbols that were never encoded.
        let mut left = 1i32;
        for length in 1..16 {
            left = left * 2 - i32::from(counts[length]);
            if left < 0 {
                return error("deflate Huffman code set is over-subscribed");
            }
        }
        let mut offsets = [0u16; 16];
        for length in 1..15 {
            offsets[length + 1] = offsets[length] + counts[length];
        }
        let mut symbols = vec![0u16; lengths.len()];
        for (symbol, &length) in lengths.iter().enumerate() {
            if length != 0 {
                symbols[offsets[length as usize] as usize] = symbol as u16;
                offsets[length as usize] += 1;
            }
        }
        Ok(Self { counts, symbols })
    }

    fn decode(&self, reader: &mut BitReader<'_>) -> Result<u16, InflateError> {
        let mut code = 0i32;
        let mut first = 0i32;
        let mut index = 0i32;
        for length in 1..16 {
            code |= reader.bits(1)? as i32;
            let count = i32::from(self.counts[length]);
            if code - first < count {
                let at = index + (code - first);
                return self
                    .symbols
                    .get(at as usize)
                    .copied()
                    .ok_or_else(|| InflateError("deflate symbol outside the code table".into()));
            }
            index += count;
            first = (first + count) << 1;
            code <<= 1;
        }
        error("deflate code longer than 15 bits")
    }
}

/// RFC 1951 §3.2.5 length codes 257..=285: extra bits and base length.
const LENGTH_BASE: [u16; 29] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131,
    163, 195, 227, 258,
];
const LENGTH_EXTRA: [u8; 29] = [
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
];
/// RFC 1951 §3.2.5 distance codes 0..=29: extra bits and base distance.
const DISTANCE_BASE: [u16; 30] = [
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537,
    2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577,
];
const DISTANCE_EXTRA: [u8; 30] = [
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13,
    13,
];
/// RFC 1951 §3.2.7: the order code-length code lengths are written in.
const CODE_LENGTH_ORDER: [usize; 19] = [
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15,
];

fn fixed_tables() -> Result<(Huffman, Huffman), InflateError> {
    let mut literal_lengths = [0u8; 288];
    for (symbol, length) in literal_lengths.iter_mut().enumerate() {
        *length = match symbol {
            0..=143 => 8,
            144..=255 => 9,
            256..=279 => 7,
            _ => 8,
        };
    }
    let distance_lengths = [5u8; 30];
    Ok((Huffman::new(&literal_lengths)?, Huffman::new(&distance_lengths)?))
}

fn dynamic_tables(reader: &mut BitReader<'_>) -> Result<(Huffman, Huffman), InflateError> {
    let literal_count = reader.bits(5)? as usize + 257;
    let distance_count = reader.bits(5)? as usize + 1;
    let code_length_count = reader.bits(4)? as usize + 4;
    if literal_count > 286 || distance_count > 30 {
        return error("deflate dynamic block declares too many codes");
    }
    let mut code_lengths = [0u8; 19];
    for &slot in CODE_LENGTH_ORDER.iter().take(code_length_count) {
        code_lengths[slot] = reader.bits(3)? as u8;
    }
    let code_length_table = Huffman::new(&code_lengths)?;
    let mut lengths = vec![0u8; literal_count + distance_count];
    let mut written = 0usize;
    while written < lengths.len() {
        let symbol = code_length_table.decode(reader)?;
        match symbol {
            0..=15 => {
                lengths[written] = symbol as u8;
                written += 1;
            }
            16 => {
                if written == 0 {
                    return error("deflate code-length repeat with no previous length");
                }
                let previous = lengths[written - 1];
                let repeat = reader.bits(2)? as usize + 3;
                if written + repeat > lengths.len() {
                    return error("deflate code-length repeat runs past the table");
                }
                lengths[written..written + repeat].fill(previous);
                written += repeat;
            }
            17 | 18 => {
                let repeat = if symbol == 17 {
                    reader.bits(3)? as usize + 3
                } else {
                    reader.bits(7)? as usize + 11
                };
                if written + repeat > lengths.len() {
                    return error("deflate zero-length repeat runs past the table");
                }
                written += repeat;
            }
            _ => return error("deflate code-length symbol outside 0..=18"),
        }
    }
    let (literal_lengths, distance_lengths) = lengths.split_at(literal_count);
    if literal_lengths[256] == 0 {
        return error("deflate dynamic block has no end-of-block code");
    }
    Ok((
        Huffman::new(literal_lengths)?,
        Huffman::new(distance_lengths)?,
    ))
}

/// Decompress a raw DEFLATE stream, refusing to emit more than `limit` bytes.
///
/// `limit` is the size the container declared for the entry: a stream that
/// produces more than it promised is a malformed (or hostile) package, not a
/// buffer to grow.
pub fn inflate(data: &[u8], limit: usize) -> Result<Vec<u8>, InflateError> {
    let mut reader = BitReader::new(data);
    let mut out = Vec::with_capacity(limit.min(1 << 20));
    loop {
        let final_block = reader.bits(1)? == 1;
        let block_type = reader.bits(2)?;
        if block_type == 0 {
            reader.align();
            let header = reader.bytes(4)?;
            let length = u16::from_le_bytes([header[0], header[1]]) as usize;
            let complement = u16::from_le_bytes([header[2], header[3]]) as usize;
            if length != complement ^ 0xffff {
                return error("deflate stored block length does not match its complement");
            }
            if out.len() + length > limit {
                return error("deflate output exceeds the declared size");
            }
            out.extend(reader.bytes(length)?);
        } else if block_type == 1 || block_type == 2 {
            let (literals, distances) = if block_type == 1 {
                fixed_tables()?
            } else {
                dynamic_tables(&mut reader)?
            };
            loop {
                let symbol = literals.decode(&mut reader)?;
                if symbol == 256 {
                    break;
                }
                if symbol < 256 {
                    if out.len() + 1 > limit {
                        return error("deflate output exceeds the declared size");
                    }
                    out.push(symbol as u8);
                    continue;
                }
                let length_index = symbol as usize - 257;
                if length_index >= LENGTH_BASE.len() {
                    return error("deflate length symbol outside 257..=285");
                }
                let length = LENGTH_BASE[length_index] as usize
                    + reader.bits(u32::from(LENGTH_EXTRA[length_index]))? as usize;
                let distance_symbol = distances.decode(&mut reader)? as usize;
                if distance_symbol >= DISTANCE_BASE.len() {
                    return error("deflate distance symbol outside 0..=29");
                }
                let distance = DISTANCE_BASE[distance_symbol] as usize
                    + reader.bits(u32::from(DISTANCE_EXTRA[distance_symbol]))? as usize;
                if distance == 0 || distance > out.len() {
                    return error("deflate back-reference points before the output start");
                }
                if out.len() + length > limit {
                    return error("deflate output exceeds the declared size");
                }
                // Byte-at-a-time: runs shorter than the distance overlap by
                // design (RFC 1951 3.2.3), so a slice copy would be wrong.
                let mut source = out.len() - distance;
                for _ in 0..length {
                    let byte = out[source];
                    out.push(byte);
                    source += 1;
                }
            }
        } else {
            return error("deflate reserved block type 3");
        }
        if final_block {
            break;
        }
    }
    Ok(out)
}
