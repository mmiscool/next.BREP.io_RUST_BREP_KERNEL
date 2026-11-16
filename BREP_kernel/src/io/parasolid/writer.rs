//! The text transmit writer: the reader's inverse (Parasolid XT Format
//! Reference §2.1, §3.2).
//!
//! The part data is one token stream, cut into records of at most 80
//! characters. Numbers end with a space; `c` and `l` fields, `?` (unset)
//! and the characters of a string do not. A record never ends with a space
//! (a reader drops trailing spaces), so a cut that would leave one there
//! moves back to the last character that is not a space and the spaces
//! start the next record. Reals are written in the shorter of their
//! decimal form (no leading zero) and an integer mantissa with an exponent
//! (`4826e-8`), the decimal on a tie when the exponent would be negative;
//! with the fewest digits that read back to the same double. All 2000 files
//! of the ABC sample written by Parasolid 30.1 cut and format exactly so.

use super::reader::{Node, NodeLayout, Value, XtFile};
use super::schema::{Count, Kind};

/// `text` with every character a text transmit file cannot hold (beyond
/// 7-bit ASCII) as `?`.
pub(super) fn ascii(text: &str) -> std::borrow::Cow<'_, str> {
    if text.is_ascii() {
        std::borrow::Cow::Borrowed(text)
    } else {
        std::borrow::Cow::Owned(text.chars().map(|c| if c.is_ascii() { c } else { '?' }).collect())
    }
}

/// The record length of a text transmit file.
const RECORD: usize = 80;

/// A real as Parasolid writes it in a text file.
pub(super) fn format_real(x: f64) -> String {
    if x == 0.0 {
        return "0".into();
    }
    // The fewest significant digits that read back to `x`.
    let mut digits = String::new();
    let mut exponent = 0i32;
    for precision in 1..=17usize {
        let s = format!("{:.*e}", precision - 1, x.abs());
        if s.parse::<f64>().ok() == Some(x.abs()) {
            let (mantissa, e) = s.split_once('e').unwrap_or((&s, "0"));
            digits = mantissa.replace('.', "");
            exponent = e.parse().unwrap_or(0);
            break;
        }
    }
    let digits = digits.trim_end_matches('0');
    let digits = if digits.is_empty() { "0" } else { digits };
    let n = digits.len() as i32;
    // x = 0.d1 d2 ... × 10^(exponent + 1) = D × 10^(exponent - n + 1).
    let integer_exponent = exponent - n + 1;
    let scientific = format!("{digits}e{integer_exponent}");
    let point = exponent + 1;
    let fixed = if point <= 0 {
        format!(".{}{digits}", "0".repeat((-point) as usize))
    } else if point >= n {
        format!("{digits}{}", "0".repeat((point - n) as usize))
    } else {
        format!("{}.{}", &digits[..point as usize], &digits[point as usize..])
    };
    let chosen = if fixed.len() < scientific.len() || (fixed.len() == scientific.len() && integer_exponent < 0) {
        fixed
    } else {
        scientific
    };
    if x < 0.0 {
        format!("-{chosen}")
    } else {
        chosen
    }
}

/// Cut the token stream into records.
pub(super) fn records(data: &str) -> String {
    // The token stream is ASCII ([`ascii`]), so every byte is a character.
    debug_assert!(data.is_ascii());
    let bytes = data.as_bytes();
    let mut out = String::with_capacity(data.len() + data.len() / RECORD + 1);
    let mut at = 0;
    while at < bytes.len() {
        let mut end = (at + RECORD).min(bytes.len());
        if end - at == RECORD {
            while end > at + 1 && bytes[end - 1] == b' ' {
                end -= 1;
            }
        }
        out.push_str(&data[at..end]);
        out.push('\n');
        at = end;
    }
    out
}

/// The token stream of a part's data.
#[derive(Default)]
pub(super) struct Tokens {
    pub(super) data: String,
}

impl Tokens {
    pub(super) fn int(&mut self, value: Option<i64>) {
        match value {
            Some(v) => {
                self.data.push_str(&v.to_string());
                self.data.push(' ');
            }
            None => self.data.push('?'),
        }
    }

    pub(super) fn real(&mut self, value: Option<f64>) {
        match value {
            Some(v) => {
                self.data.push_str(&format_real(v));
                self.data.push(' ');
            }
            None => self.data.push('?'),
        }
    }

    /// The characters of a string, escaped (§3.2): null, carriage return,
    /// line feed and backslash. A text file holds 7-bit characters: any
    /// other is written `?` (so the length counts characters written).
    pub(super) fn chars(&mut self, text: &str) {
        for c in ascii(text).bytes() {
            match c {
                0 => self.data.push_str("\\0"),
                b'\n' => self.data.push_str("\\n"),
                b'\r' => self.data.push_str("\\r"),
                b'\\' => self.data.push_str("\\\\"),
                c => self.data.push(char::from(c)),
            }
        }
    }

    /// A short string: its length, a space, its characters.
    pub(super) fn string(&mut self, text: &str) {
        self.int(Some(ascii(text).len() as i64));
        self.chars(text);
    }

    fn value(&mut self, kind: Kind, value: &Value) -> Result<(), String> {
        match (kind, value) {
            (Kind::Int, Value::Int(v)) => self.int(*v),
            (Kind::Real, Value::Real(v)) => self.real(*v),
            (Kind::Vector, Value::Vector(None)) => self.data.push('?'),
            (Kind::Vector, Value::Vector(Some(v))) => v.iter().for_each(|x| self.real(Some(*x))),
            (Kind::Interval, Value::Interval(v)) if v.iter().all(Option::is_none) => self.data.push('?'),
            (Kind::Interval, Value::Interval(v)) => v.iter().for_each(|x| self.real(*x)),
            (Kind::Box, Value::Box(v)) if v.iter().all(Option::is_none) => self.data.push('?'),
            (Kind::Box, Value::Box(v)) => v.iter().for_each(|x| self.real(*x)),
            (Kind::Char, Value::Char(c)) => self.data.push(char::from(*c)),
            (kind, value) => return Err(format!("a {kind:?} field holds {value:?}")),
        }
        Ok(())
    }

    /// One node: its type, its layout's description at the type's first
    /// node, its variable length, its index and its fields.
    pub(super) fn node(&mut self, node: &Node, describe: bool, user_field_size: usize) -> Result<(), String> {
        let layout: &NodeLayout = &node.layout;
        self.int(Some(i64::from(layout.node_type)));
        if describe {
            let delta = layout.delta.as_deref().ok_or_else(|| format!("{} has no layout description to write", layout.name))?;
            self.data.push_str(delta);
        }
        let variable = layout.fields.last().filter(|f| f.count == Count::Variable);
        if let Some(field) = variable {
            let length = match node.values.last() {
                Some(Value::Text(text)) => ascii(text).len(),
                Some(Value::Array(items)) => items.len(),
                other => return Err(format!("{}'s variable field {} holds {other:?}", layout.name, field.name)),
            };
            self.int(Some(length as i64));
        }
        self.int(Some(i64::from(node.index)));
        for (field, value) in layout.fields.iter().zip(&node.values) {
            match (field.count, value) {
                (Count::Scalar, value) => self.value(field.kind, value)?,
                (Count::Variable, Value::Text(text)) => self.chars(text),
                (_, Value::Array(items)) => {
                    for item in items {
                        self.value(field.kind, item)?;
                    }
                }
                (_, value) => return Err(format!("{} field {} holds {value:?}", layout.name, field.name)),
            }
        }
        if layout.node_type < 1000 && super::reader::carries_user_field(layout.node_type) {
            for _ in 0..user_field_size {
                self.int(Some(0));
            }
        }
        Ok(())
    }
}

/// A parsed file's part data written again: the banner, the schema, each
/// node with its layout described at its type's first node, the terminator
/// — cut into records. What [`super::parse_xt_text`] read from a text file
/// with embedded deltas is written back as that file had it.
pub fn write_xt_part_data(file: &XtFile) -> Result<String, String> {
    if file.user_field_size > 0 {
        return Err("the file carries user fields, which the reader does not keep: it is not written back".into());
    }
    let mut tokens = Tokens::default();
    tokens.data.push('T');
    tokens.string(&file.modeller_version);
    tokens.string(&file.schema);
    if file.max_node_type.is_some() {
        tokens.int(file.max_node_type);
    }
    tokens.int(Some(file.user_field_size as i64));
    let mut described = rustc_hash::FxHashSet::default();
    for node in file.nodes() {
        let first = described.insert(node.node_type());
        tokens.node(node, first && file.max_node_type.is_some(), file.user_field_size)?;
    }
    tokens.int(Some(1));
    tokens.int(Some(0));
    Ok(records(&tokens.data))
}

/// The neutral binary token stream (§3.3.3): big-endian; node types and
/// short integers in two bytes, integers and lengths in four, reals in
/// eight; pointers as index + 1 in two bytes (the negated remainder + 1 and
/// the quotient past 32766); nulls as -32764 and -3.14158e13; logicals as 0
/// and 1; short strings with a one-byte length.
#[derive(Default)]
pub(super) struct BinaryTokens {
    pub(super) data: Vec<u8>,
}

const NULL_INT: i64 = -32764;
const NULL_REAL: f64 = -3.14158e13;

impl BinaryTokens {
    fn u8(&mut self, v: u8) {
        self.data.push(v);
    }

    fn i16(&mut self, v: i64) -> Result<(), String> {
        let v = i16::try_from(v).map_err(|_| format!("{v} does not fit two bytes"))?;
        self.data.extend_from_slice(&v.to_be_bytes());
        Ok(())
    }

    fn i32(&mut self, v: i64) -> Result<(), String> {
        let v = i32::try_from(v).map_err(|_| format!("{v} does not fit four bytes"))?;
        self.data.extend_from_slice(&v.to_be_bytes());
        Ok(())
    }

    fn f64(&mut self, v: f64) {
        self.data.extend_from_slice(&v.to_be_bytes());
    }

    /// A node index or pointer.
    pub(super) fn index(&mut self, index: i64) -> Result<(), String> {
        if !(0..=i64::from(i16::MAX) * 32767).contains(&index) {
            return Err(format!("index {index} cannot be written"));
        }
        if index < 32767 {
            self.i16(index + 1)
        } else {
            self.i16(-(index % 32767 + 1))?;
            self.i16(index / 32767)
        }
    }

    fn short_string(&mut self, text: &[u8]) -> Result<(), String> {
        let n = u8::try_from(text.len()).map_err(|_| "a short string is longer than 255 bytes".to_string())?;
        self.u8(n);
        self.data.extend_from_slice(text);
        Ok(())
    }

    fn value(&mut self, code: u8, value: &Value) -> Result<(), String> {
        let real = |s: &mut Self, x: &Option<f64>| s.f64(x.unwrap_or(NULL_REAL));
        match (code, value) {
            (b'p', Value::Int(v)) => self.index(v.unwrap_or(0))?,
            (b'u', Value::Int(v)) => self.u8(u8::try_from(v.unwrap_or(0)).map_err(|_| format!("byte field holds {v:?}"))?),
            (b'n' | b'w', Value::Int(v)) => self.i16(v.unwrap_or(NULL_INT))?,
            (b'd' | b't', Value::Int(v)) => self.i32(v.unwrap_or(NULL_INT))?,
            (b'f', Value::Real(v)) => real(self, v),
            (b'v' | b'h', Value::Vector(v)) => match v {
                Some(p) => p.iter().for_each(|x| self.f64(*x)),
                None => (0..3).for_each(|_| self.f64(NULL_REAL)),
            },
            (b'i', Value::Interval(v)) => v.iter().for_each(|x| real(self, x)),
            (b'b', Value::Box(v)) => v.iter().for_each(|x| real(self, x)),
            (b'l', Value::Char(c)) => self.u8(u8::from(*c == b'T')),
            (b'c', Value::Char(c)) => self.u8(*c),
            (code, value) => return Err(format!("a `{}` field holds {value:?}", char::from(code))),
        }
        Ok(())
    }

    /// One node, its layout described at its type's first node: the raw
    /// binary description a binary file carried, or a text description
    /// converted.
    pub(super) fn node(&mut self, node: &Node, describe: bool, user_field_size: usize) -> Result<(), String> {
        let layout: &NodeLayout = &node.layout;
        self.i16(i64::from(layout.node_type))?;
        if describe {
            match (&layout.delta_binary, &layout.delta) {
                (Some(raw), _) => self.data.extend_from_slice(raw),
                (None, Some(text)) => self.data.extend(delta_to_binary(layout.node_type, text)?),
                (None, None) => return Err(format!("{} has no layout description to write", layout.name)),
            }
        }
        if let Some(field) = layout.fields.last().filter(|f| f.count == Count::Variable) {
            let length = match node.values.last() {
                Some(Value::Text(text)) => text.len(),
                Some(Value::Array(items)) => items.len(),
                other => return Err(format!("{}'s variable field {} holds {other:?}", layout.name, field.name)),
            };
            self.i32(length as i64)?;
        }
        self.index(i64::from(node.index))?;
        for (field, value) in layout.fields.iter().zip(&node.values) {
            match (field.count, value) {
                (Count::Scalar, value) => self.value(field.code, value)?,
                (Count::Variable, Value::Text(text)) => self.data.extend_from_slice(text.as_bytes()),
                (_, Value::Array(items)) => {
                    for item in items {
                        self.value(field.code, item)?;
                    }
                }
                (_, value) => return Err(format!("{} field {} holds {value:?}", layout.name, field.name)),
            }
        }
        if layout.node_type < 1000 && super::reader::carries_user_field(layout.node_type) {
            for _ in 0..user_field_size {
                self.i32(0)?;
            }
        }
        Ok(())
    }

    /// The flag sequence and prologue (§3.3): `PS\0\0`, the modeller version
    /// (two-byte length), the schema (four-byte length), the largest node
    /// type when the layouts are described, the user field size.
    pub(super) fn prologue(&mut self, modeller: &str, schema: &str, max_node_type: Option<i64>, user_field_size: usize) -> Result<(), String> {
        self.data.extend_from_slice(b"PS\0\0");
        self.i16(modeller.len() as i64)?;
        self.data.extend_from_slice(modeller.as_bytes());
        self.i32(schema.len() as i64)?;
        self.data.extend_from_slice(schema.as_bytes());
        if let Some(max) = max_node_type {
            self.i16(max)?;
        }
        self.i32(user_field_size as i64)
    }

    pub(super) fn terminator(&mut self) -> Result<(), String> {
        self.i16(1)?;
        self.index(0)
    }
}

/// A layout description as a text file writes it, in binary: the field
/// count as a byte; for a type the base has, the edit letters, each new
/// field as its name (one-byte length), pointer class (two bytes), element
/// count (as an index), type code (when not a pointer) and transmit flag
/// (when variable); for a new type, its name, description and fields.
pub(super) fn delta_to_binary(node_type: u16, text: &str) -> Result<Vec<u8>, String> {
    let bytes = text.as_bytes();
    let mut at = 0usize;
    let token = |at: &mut usize| -> Result<i64, String> {
        let start = *at;
        while *at < bytes.len() && bytes[*at] != b' ' {
            *at += 1;
        }
        let t = std::str::from_utf8(&bytes[start..*at]).map_err(|e| e.to_string())?;
        *at += 1;
        t.parse::<i64>().map_err(|_| format!("a delta for node type {node_type} has {t:?} where an integer belongs"))
    };
    let string = |at: &mut usize| -> Result<Vec<u8>, String> {
        let n = token(at)? as usize;
        let s = bytes.get(*at..*at + n).ok_or("a delta string runs past its end")?.to_vec();
        *at += n;
        Ok(s)
    };
    let mut out = BinaryTokens::default();
    let field = |at: &mut usize, out: &mut BinaryTokens| -> Result<(), String> {
        let name = string(at)?;
        let class = token(at)?;
        let elements = token(at)?;
        out.short_string(&name)?;
        out.i16(class)?;
        out.index(elements)?;
        if class == 0 {
            let code = string(at)?;
            out.short_string(&code)?;
        }
        if elements == 1 {
            let flag = *bytes.get(*at).ok_or("a delta ends inside a field")?;
            *at += 1;
            out.u8(flag);
        }
        Ok(())
    };
    let count = token(&mut at)?;
    out.u8(u8::try_from(count).map_err(|_| format!("a field count of {count}"))?);
    if count == 255 {
        return Ok(out.data);
    }
    if super::schema::base(node_type).is_none() {
        let name = string(&mut at)?;
        let description = string(&mut at)?;
        out.short_string(&name)?;
        out.short_string(&description)?;
        for _ in 0..count {
            field(&mut at, &mut out)?;
        }
        return Ok(out.data);
    }
    loop {
        let letter = *bytes.get(at).ok_or("a delta ends before Z")?;
        at += 1;
        out.u8(letter);
        match letter {
            b'C' | b'D' => {}
            b'I' | b'A' => field(&mut at, &mut out)?,
            b'Z' => break,
            other => return Err(format!("a delta has edit letter {:?}", char::from(other))),
        }
    }
    Ok(out.data)
}

/// A parsed file's part data written again in neutral binary: what
/// [`super::parse_xt`] read from a neutral binary file comes back as that
/// file had it.
pub fn write_xt_binary_part_data(file: &XtFile) -> Result<Vec<u8>, String> {
    if file.user_field_size > 0 {
        return Err("the file carries user fields, which the reader does not keep: it is not written back".into());
    }
    let mut tokens = BinaryTokens::default();
    tokens.prologue(&file.modeller_version, &file.schema, file.max_node_type, file.user_field_size)?;
    let mut described = rustc_hash::FxHashSet::default();
    for node in file.nodes() {
        let first = described.insert(node.node_type());
        tokens.node(node, first && file.max_node_type.is_some(), file.user_field_size)?;
    }
    tokens.terminator()?;
    Ok(tokens.data)
}
