//! The text transmit reader: header, token stream, embedded-schema deltas and
//! the node graph (Parasolid XT Format Reference, chapters 2 and 3).
//!
//! A text part is a bytestream cut into ~80-character records; the record
//! breaks carry no meaning, so the reader joins the records and walks one
//! string. Numbers are followed by one space; `c` (char) and `l` (logical)
//! fields are single characters with no separator, and the unset value of
//! an integer, real or vector is a lone `?` with no separator either.

use std::rc::Rc;

use rustc_hash::FxHashMap as HashMap;

use super::schema::{self, Count, FieldDef, Kind};

/// The field layout of one node type as this file writes it.
#[derive(Debug)]
pub struct NodeLayout {
    pub node_type: u16,
    pub name: String,
    pub fields: Vec<FieldDef>,
}

impl NodeLayout {
    fn variable(&self) -> bool {
        self.fields.last().is_some_and(|f| f.count == Count::Variable)
    }

    fn position(&self, name: &str) -> Option<usize> {
        self.fields.iter().position(|f| f.name == name)
    }
}

/// One field value. Unset values (`?`) are `None`.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Int(Option<i64>),
    Real(Option<f64>),
    Vector(Option<[f64; 3]>),
    Interval([Option<f64>; 2]),
    Box([Option<f64>; 6]),
    Char(u8),
    Array(Vec<Value>),
    Text(String),
}

/// A node: its type, index and field values in its layout's order.
#[derive(Debug)]
pub struct Node {
    pub index: u32,
    pub layout: Rc<NodeLayout>,
    pub values: Vec<Value>,
}

impl Node {
    pub fn node_type(&self) -> u16 {
        self.layout.node_type
    }

    pub fn type_name(&self) -> &str {
        &self.layout.name
    }

    /// A field by name; `None` when this file's layout has no such field.
    pub fn field(&self, name: &str) -> Option<&Value> {
        self.layout.position(name).map(|i| &self.values[i])
    }

    /// An integer or pointer field; unset and absent read as `None`.
    pub fn int(&self, name: &str) -> Option<i64> {
        match self.field(name) {
            Some(Value::Int(v)) => *v,
            _ => None,
        }
    }

    /// A pointer field's index; null, unset and absent read as 0.
    pub fn ptr(&self, name: &str) -> u32 {
        self.int(name).filter(|&v| v > 0).map_or(0, |v| v as u32)
    }

    pub fn real(&self, name: &str) -> Option<f64> {
        match self.field(name) {
            Some(Value::Real(v)) => *v,
            _ => None,
        }
    }

    pub fn vector(&self, name: &str) -> Option<[f64; 3]> {
        match self.field(name) {
            Some(Value::Vector(v)) => *v,
            _ => None,
        }
    }

    pub fn interval(&self, name: &str) -> Option<[Option<f64>; 2]> {
        match self.field(name) {
            Some(Value::Interval(v)) => Some(*v),
            _ => None,
        }
    }

    pub fn char(&self, name: &str) -> Option<u8> {
        match self.field(name) {
            Some(Value::Char(c)) => Some(*c),
            _ => None,
        }
    }

    /// A logical field; `T` is true.
    pub fn logical(&self, name: &str) -> Option<bool> {
        self.char(name).map(|c| c == b'T')
    }

    pub fn text(&self, name: &str) -> Option<&str> {
        match self.field(name) {
            Some(Value::Text(s)) => Some(s),
            _ => None,
        }
    }

    pub fn array(&self, name: &str) -> &[Value] {
        match self.field(name) {
            Some(Value::Array(v)) => v,
            _ => &[],
        }
    }

    /// An array of reals (unset entries as NaN).
    pub fn reals(&self, name: &str) -> Vec<f64> {
        self.array(name)
            .iter()
            .map(|v| match v {
                Value::Real(Some(x)) => *x,
                _ => f64::NAN,
            })
            .collect()
    }

    /// An array of integers or pointers (unset entries as 0).
    pub fn ints(&self, name: &str) -> Vec<i64> {
        self.array(name)
            .iter()
            .map(|v| match v {
                Value::Int(Some(x)) => *x,
                _ => 0,
            })
            .collect()
    }

    /// An array of vectors (unset entries as `None`).
    pub fn vectors(&self, name: &str) -> Vec<Option<[f64; 3]>> {
        self.array(name)
            .iter()
            .map(|v| match v {
                Value::Vector(x) => *x,
                _ => None,
            })
            .collect()
    }
}

/// A parsed text transmit file.
#[derive(Debug)]
pub struct XtFile {
    /// `TRANSMIT FILE created by modeller version …`.
    pub modeller_version: String,
    /// `SCH_<version>_<schema>[_<base>]`.
    pub schema: String,
    /// The header's `KEY=value` pairs (parts 1 to 3).
    pub header: Vec<(String, String)>,
    /// A partition (`TRANSMIT FILE (partition)`): its root is the WORLD,
    /// whose bodies are the partition's.
    pub partition: bool,
    nodes: Vec<Node>,
    by_index: HashMap<u32, usize>,
}

impl XtFile {
    /// The root node: the first node in the file (§2.1).
    pub fn root(&self) -> Option<&Node> {
        self.nodes.first()
    }

    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    /// The node an index names; 0 and indices with no node are null (§2.1.4.4).
    pub fn node(&self, index: u32) -> Option<&Node> {
        if index == 0 {
            return None;
        }
        self.by_index.get(&index).map(|&i| &self.nodes[i])
    }

    /// The node a pointer field of `node` names.
    pub fn follow(&self, node: &Node, field: &str) -> Option<&Node> {
        self.node(node.ptr(field))
    }

    pub fn header_value(&self, key: &str) -> Option<&str> {
        self.header.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
}

/// Unset integer and real values as written by binary writers; text writes `?`.
const NULL_INT: i64 = -32764;

/// Read a text transmit file (`.x_t`, `.xmt_txt`).
pub fn parse_xt_text(text: &str) -> Result<XtFile, String> {
    let (header, body_start) = split_header(text);
    if text.as_bytes()[body_start..].starts_with(b"PS") {
        return Err("this is a binary Parasolid file (.x_b); read its bytes, not its text".into());
    }
    let data = join_records(&text[body_start..]);
    let mut stream = Stream { data: &data, at: 0, encoding: Encoding::Text };
    stream.skip_blank();
    match stream.next_byte() {
        Some(b'T') => {}
        Some(b'P' | b'B') => {
            return Err("this is a binary Parasolid file (.x_b); read its bytes, not its text".into())
        }
        other => {
            return Err(format!(
                "not a Parasolid text transmit file: the part data starts with {:?}, expected 'T'",
                other.map(char::from)
            ))
        }
    }
    parse_data(header, stream)
}

/// Read a transmit file, text or binary, from its bytes: a text part, a
/// neutral binary (`PS\0\0`, big-endian) or typed binary (`PS\0\1` and the
/// writer's machine description) part or partition, with or without the
/// keyword header (§3.1, §3.3). A SolidWorks body is such a partition.
pub fn parse_xt(bytes: &[u8]) -> Result<XtFile, String> {
    // The keyword header is text in every format; the data follows the line
    // that ends it. Only a file that starts with the header's preamble has
    // one: a headerless binary file may hold that text in its own data.
    let has_header = bytes.trim_ascii_start().starts_with(b"**ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    let body_start = match find(bytes, b"**END_OF_HEADER").filter(|_| has_header) {
        Some(end) => bytes[end..].iter().position(|&b| b == b'\n').map_or(bytes.len(), |n| end + n + 1),
        None => 0,
    };
    let data = &bytes[body_start..];
    let flag = data.iter().position(|b| !b.is_ascii_whitespace()).unwrap_or(data.len());
    if !data[flag..].starts_with(b"PS") && !data[flag..].starts_with(b"B") {
        return parse_xt_text(&String::from_utf8_lossy(bytes));
    }
    let (header, _) = split_header(&String::from_utf8_lossy(&bytes[..body_start]));
    let data = &data[flag..];
    let (little, at) = match data {
        [b'P', b'S', 0, 0, ..] => (false, 4),
        [b'P', b'S', 0, 1, order, double, character, ..] => {
            if *double != 0 || *character != 0 {
                return Err(format!(
                    "this typed binary Parasolid file was written with {} reals and {} characters; only IEEE reals and \
                     ASCII characters are read",
                    if *double == 0 { "IEEE" } else { "VAX D-float" },
                    if *character == 0 { "ASCII" } else { "EBCDIC" }
                ));
            }
            (*order == 1, 7)
        }
        [b'P', b'S', ..] => return Err("this binary Parasolid file has an unknown flag sequence after 'PS'".into()),
        _ => {
            return Err("this is a bare binary Parasolid file, written in its machine's own format with no \
                        description of that machine; only neutral and typed binary are read"
                .into())
        }
    };
    parse_data(header, Stream::binary(&data[at..], little))
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// The part data after its flag sequence: the modeller version, the schema,
/// the user field size, then the nodes to the terminator (§2.1, §3.2, §3.3).
fn parse_data(header: Vec<(String, String)>, mut stream: Stream) -> Result<XtFile, String> {
    // Binary writes the modeller version's length in two bytes and the
    // schema's in four (§3.3).
    let modeller_version = stream.header_string(2)?;
    let partition = modeller_version.contains("(partition)");
    if modeller_version.contains("(deltas)") {
        return Err("this is a Parasolid partition's deltas (its rollback history), which holds no bodies".into());
    }
    let schema_name = stream.header_string(4)?;
    // `SCH_1400068_14000_13006`: the base suffix marks embedded schema deltas,
    // followed by the maximum node type (§2.1.2.2).
    let embedded = schema_name.split('_').count() >= 4;
    if embedded {
        let base = schema_name.rsplit('_').next().unwrap_or_default();
        if base != schema::BASE_SCHEMA {
            return Err(format!(
                "the file's schema {schema_name} is relative to base {base}; only base {} is known",
                schema::BASE_SCHEMA
            ));
        }
        stream.short()?;
    }
    // Without embedded deltas the layouts are the named schema's own: the
    // base's, or a stored schema's.
    let stored = if embedded {
        None
    } else {
        schema::STORED_SCHEMAS.into_iter().find(|s| schema_name.ends_with(&format!("_{s}")))
    };
    if !embedded && stored.is_none() && !schema_name.ends_with(&format!("_{}", schema::BASE_SCHEMA)) {
        return Err(format!(
            "the file's schema {schema_name} carries no description of its layout; only files written with schema {} \
             or {}, or with embedded schema deltas (Parasolid V14 and later), are read",
            schema::BASE_SCHEMA,
            schema::STORED_SCHEMAS.join(", ")
        ));
    }
    let user_field_size = stream.long()?.unwrap_or(0).max(0) as usize;
    let mut layouts: HashMap<u16, Rc<NodeLayout>> = HashMap::default();
    let mut nodes = Vec::new();
    let mut by_index = HashMap::default();
    loop {
        let at = stream.at;
        let node_type = stream.short()?.ok_or("a node type is unset")?;
        if node_type == 1 {
            // Terminator: type 1, index 0.
            if stream.index()? == 0 {
                break;
            }
            return Err(format!("node type 1 at byte {at} is not followed by the terminating 0"));
        }
        let node_type = u16::try_from(node_type).map_err(|_| format!("node type {node_type} at byte {at} is out of range"))?;
        let layout = match layouts.get(&node_type) {
            Some(layout) => layout.clone(),
            None => {
                let layout = Rc::new(if embedded {
                    read_layout(&mut stream, node_type)?
                } else if let Some(stored) = stored {
                    stored_layout(stored, node_type)?
                } else {
                    base_layout(node_type)?
                });
                layouts.insert(node_type, layout.clone());
                layout
            }
        };
        let node = read_node(&mut stream, layout, user_field_size)
            .map_err(|e| format!("node type {node_type} at byte {at}: {e}"))?;
        if by_index.insert(node.index, nodes.len()).is_some() {
            return Err(format!("node index {} appears twice", node.index));
        }
        nodes.push(node);
    }
    if nodes.is_empty() {
        return Err("the file holds no nodes".into());
    }
    Ok(XtFile { modeller_version, schema: schema_name, header, partition, nodes, by_index })
}

fn base_layout(node_type: u16) -> Result<NodeLayout, String> {
    let (name, fields) = schema::base(node_type)
        .ok_or_else(|| format!("node type {node_type} is not in the base schema and the file does not describe it"))?;
    Ok(NodeLayout { node_type, name: name.to_string(), fields })
}

/// A node type's layout in a stored schema; a type no file in that schema
/// has shown is refused by name.
fn stored_layout(stored: &str, node_type: u16) -> Result<NodeLayout, String> {
    let delta = schema::stored_delta(stored, node_type).ok_or_else(|| {
        let name = schema::base(node_type).map_or_else(|| format!("node type {node_type}"), |(n, _)| format!("node type {node_type} ({n})"));
        format!("{name} has no known layout in schema {stored} and the file does not describe it")
    })?;
    read_layout(&mut Stream { data: delta.as_bytes(), at: 0, encoding: Encoding::Text }, node_type)
        .map_err(|e| format!("the stored schema {stored} layout of node type {node_type}: {e}"))
}

/// A node type's layout from its first occurrence (§2.1.2.2): 255 for "as the
/// base", a new type's full description, or an edit script against the base.
fn read_layout(stream: &mut Stream, node_type: u16) -> Result<NodeLayout, String> {
    // A field count is a byte (§2.1.2.2); 255 means "as the base".
    let count = stream.byte_int()?.ok_or("a field count is unset")?;
    if !(0..=255).contains(&count) {
        return Err(format!("node type {node_type} declares {count} fields"));
    }
    let Some((name, base)) = schema::base(node_type) else {
        // A type the base schema does not have: its whole description.
        let name = stream.string()?;
        let _description = stream.string()?;
        let mut fields = Vec::with_capacity(count.max(0) as usize);
        for _ in 0..count {
            fields.push(stream.field_def()?);
        }
        return Ok(NodeLayout { node_type, name, fields });
    };
    if count == 255 {
        return Ok(NodeLayout { node_type, name: name.to_string(), fields: base });
    }
    let mut fields = Vec::with_capacity(count.max(0) as usize);
    let mut base_fields = base.into_iter();
    loop {
        match stream.next_byte() {
            Some(b'C') => fields.push(
                base_fields
                    .next()
                    .ok_or_else(|| format!("{name}'s schema copies past the end of the base layout"))?,
            ),
            Some(b'D') => {
                base_fields
                    .next()
                    .ok_or_else(|| format!("{name}'s schema deletes past the end of the base layout"))?;
            }
            Some(b'I' | b'A') => fields.push(stream.field_def()?),
            Some(b'Z') => break,
            other => {
                return Err(format!(
                    "{name}'s schema edit has instruction {:?}, expected C, D, I, A or Z",
                    other.map(char::from)
                ))
            }
        }
    }
    if fields.len() as i64 != count {
        return Err(format!("{name}'s schema edit gives {} fields, its header says {count}", fields.len()));
    }
    Ok(NodeLayout { node_type, name: name.to_string(), fields })
}

/// Node types visible at the PK interface, which carry the user field (§2.1, §6.1).
fn carries_user_field(node_type: u16) -> bool {
    matches!(
        node_type,
        10..=19 | 29..=32 | 38 | 50..=54 | 56 | 60 | 67 | 68 | 70 | 80 | 81 | 90 | 100 | 120 | 124 | 130 | 133
            | 134 | 137 | 200 | 201 | 222
    )
}

fn read_node(stream: &mut Stream, layout: Rc<NodeLayout>, user_field_size: usize) -> Result<Node, String> {
    let length = if layout.variable() {
        let n = stream.long()?.ok_or("a variable length is unset")?;
        let n = usize::try_from(n).map_err(|_| format!("variable length {n} is negative"))?;
        // Every element takes at least one byte of the data.
        if n > stream.remaining() {
            return Err(format!("variable length {n} runs past the end of the data"));
        }
        n
    } else {
        0
    };
    let index = stream.index()?;
    let index = u32::try_from(index).ok().filter(|&i| i > 0).ok_or_else(|| format!("node index {index} is invalid"))?;
    let mut values = Vec::with_capacity(layout.fields.len());
    for field in &layout.fields {
        let value = match field.count {
            Count::Scalar => stream.value(field),
            Count::Variable if field.code == b'c' => stream.chars(length).map(Value::Text),
            Count::Variable => (0..length).map(|_| stream.value(field)).collect::<Result<_, _>>().map(Value::Array),
            Count::Fixed(n) => (0..n).map(|_| stream.value(field)).collect::<Result<_, _>>().map(Value::Array),
        }
        .map_err(|e| format!("{} field {}: {e}", layout.name, field.name))?;
        values.push(value);
    }
    if layout.node_type < 1000 && carries_user_field(layout.node_type) {
        for _ in 0..user_field_size {
            stream.long()?;
        }
    }
    Ok(Node { index, layout, values })
}

/// The header's `KEY=value;` pairs and where the part data starts (§3.1). A
/// file with no header starts with its part data.
fn split_header(text: &str) -> (Vec<(String, String)>, usize) {
    let Some(end) = text.find("**END_OF_HEADER") else {
        return (Vec::new(), 0);
    };
    let body_start = text[end..].find('\n').map_or(text.len(), |n| end + n + 1);
    let mut joined = String::new();
    for line in text[..end].lines() {
        let line = line.trim_end_matches('\r');
        if line.starts_with("**ABCDEFG") || line.starts_with("**PARASOLID") {
            continue;
        }
        joined.push_str(line);
    }
    let mut header = Vec::new();
    for item in joined.split(';') {
        let item = item.trim_start_matches(|c| c == '*').trim();
        let item = item.strip_prefix("PART1").or_else(|| item.strip_prefix("PART2")).or_else(|| item.strip_prefix("PART3")).unwrap_or(item);
        let item = item.trim_start_matches(|c| c == '*' || c == ';');
        if let Some((key, value)) = item.split_once('=') {
            header.push((key.trim().to_string(), unescape_header(value)));
        }
    }
    (header, body_start)
}

/// §3.1.1 keyword escapes.
fn unescape_header(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    let mut chars = value.chars();
    while let Some(c) = chars.next() {
        if c != '^' {
            out.push(c);
            continue;
        }
        match chars.next() {
            Some('n') => out.push('\n'),
            Some('-') => out.push(' '),
            Some(';') => out.push(';'),
            Some('^') => out.push('^'),
            Some(other) => {
                out.push('^');
                out.push(other);
            }
            None => out.push('^'),
        }
    }
    out
}

/// §3.2: record breaks, carriage returns and the trailing spaces a system may
/// add to a record are not part of the data; leading spaces are.
fn join_records(text: &str) -> Vec<u8> {
    let mut data = Vec::with_capacity(text.len());
    for line in text.split('\n') {
        data.extend_from_slice(line.trim_end_matches(['\r', ' ']).as_bytes());
    }
    data
}

/// How the part data is encoded: text tokens, or binary (§3.3), neutral
/// binary big-endian and typed binary in its writer's byte order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Encoding {
    Text,
    Binary { little: bool },
}

pub(super) struct Stream<'a> {
    data: &'a [u8],
    at: usize,
    encoding: Encoding,
}

impl<'a> Stream<'a> {
    /// A binary stream from its first byte.
    pub(super) fn binary(data: &'a [u8], little: bool) -> Self {
        Stream { data, at: 0, encoding: Encoding::Binary { little } }
    }

    fn skip_blank(&mut self) {
        while self.at < self.data.len() && self.data[self.at].is_ascii_whitespace() {
            self.at += 1;
        }
    }

    fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.at)
    }

    fn peek(&self) -> Option<u8> {
        self.data.get(self.at).copied()
    }

    fn next_byte(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.at += 1;
        Some(b)
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], String> {
        let bytes = self
            .data
            .get(self.at..self.at + N)
            .ok_or("the data ends inside a binary value")?
            .try_into()
            .map_err(|_| "a binary value is cut short".to_string())?;
        self.at += N;
        Ok(bytes)
    }

    fn little(&self) -> bool {
        matches!(self.encoding, Encoding::Binary { little: true })
    }

    fn raw_i16(&mut self) -> Result<i16, String> {
        let b = self.take::<2>()?;
        Ok(if self.little() { i16::from_le_bytes(b) } else { i16::from_be_bytes(b) })
    }

    fn raw_i32(&mut self) -> Result<i32, String> {
        let b = self.take::<4>()?;
        Ok(if self.little() { i32::from_le_bytes(b) } else { i32::from_be_bytes(b) })
    }

    fn raw_f64(&mut self) -> Result<f64, String> {
        let b = self.take::<8>()?;
        Ok(if self.little() { f64::from_le_bytes(b) } else { f64::from_be_bytes(b) })
    }

    /// A one-byte integer (a field count); text writes it as any integer.
    fn byte_int(&mut self) -> Result<Option<i64>, String> {
        match self.encoding {
            Encoding::Text => self.int(),
            Encoding::Binary { .. } => Ok(Some(i64::from(self.next_byte().ok_or("the data ends inside a byte")?))),
        }
    }

    /// A two-byte integer (a node type, a pointer class); null is -32764.
    fn short(&mut self) -> Result<Option<i64>, String> {
        match self.encoding {
            Encoding::Text => self.int(),
            Encoding::Binary { .. } => Ok(Some(i64::from(self.raw_i16()?)).filter(|&v| v != NULL_INT)),
        }
    }

    /// A four-byte integer (a variable length, the user field size, an
    /// integer field); null is -32764.
    fn long(&mut self) -> Result<Option<i64>, String> {
        match self.encoding {
            Encoding::Text => self.int(),
            Encoding::Binary { .. } => Ok(Some(i64::from(self.raw_i32()?)).filter(|&v| v != NULL_INT)),
        }
    }

    /// A node index or pointer (§3.3.3): in binary, index + 1 in two bytes
    /// below 32767; above, the negated remainder + 1 and then the quotient.
    /// Null is 0.
    pub(super) fn index(&mut self) -> Result<i64, String> {
        match self.encoding {
            Encoding::Text => Ok(self.int()?.unwrap_or(0)),
            Encoding::Binary { .. } => {
                let r = i64::from(self.raw_i16()?);
                match r {
                    0 => Err(format!("a pointer at byte {} is 0, which no writer emits", self.at - 2)),
                    r if r > 0 => Ok(r - 1),
                    r => {
                        let q = i64::from(self.raw_i16()?);
                        Ok(q * 32767 - r - 1)
                    }
                }
            }
        }
    }

    /// The modeller version and schema strings: in binary, a length of
    /// `width` bytes then the characters.
    fn header_string(&mut self, width: usize) -> Result<String, String> {
        let n = match (self.encoding, width) {
            (Encoding::Text, _) => return self.string(),
            (_, 2) => i64::from(self.raw_i16()?),
            _ => i64::from(self.raw_i32()?),
        };
        let n = usize::try_from(n).map_err(|_| format!("a header string length {n} is negative"))?;
        let bytes = self.data.get(self.at..self.at + n).ok_or("a header string runs past the end of the data")?;
        self.at += n;
        Ok(String::from_utf8_lossy(bytes).into_owned())
    }

    /// A space-terminated token.
    fn token(&mut self) -> Result<&str, String> {
        let start = self.at;
        while self.at < self.data.len() && self.data[self.at] != b' ' {
            self.at += 1;
        }
        let token = &self.data[start..self.at];
        if self.at < self.data.len() {
            self.at += 1;
        }
        if token.is_empty() {
            return Err(if start >= self.data.len() {
                "the data ends before the terminator".into()
            } else {
                format!("an empty token at byte {start}")
            });
        }
        std::str::from_utf8(token).map_err(|_| format!("a non-text token at byte {start}"))
    }

    /// An integer, or `None` for `?`.
    fn int(&mut self) -> Result<Option<i64>, String> {
        if self.peek() == Some(b'?') {
            self.at += 1;
            return Ok(None);
        }
        let at = self.at;
        let token = self.token()?;
        let value: i64 = token.parse().map_err(|_| format!("expected an integer at byte {at}, read {token:?}"))?;
        Ok((value != NULL_INT).then_some(value))
    }

    /// A real, or `None` for `?`.
    fn real(&mut self) -> Result<Option<f64>, String> {
        if self.peek() == Some(b'?') {
            self.at += 1;
            return Ok(None);
        }
        let at = self.at;
        let token = self.token()?;
        let value: f64 = token.parse().map_err(|_| format!("expected a real at byte {at}, read {token:?}"))?;
        if !value.is_finite() {
            return Err(format!("a non-finite real at byte {at}"));
        }
        // The null real, -3.14158e13, marks an unset value in binary writers.
        Ok((value != -3.14158e13).then_some(value))
    }

    fn vector(&mut self) -> Result<Option<[f64; 3]>, String> {
        // One `?` stands for a whole unset vector (§3.2).
        if self.peek() == Some(b'?') {
            self.at += 1;
            return Ok(None);
        }
        let (x, y, z) = (self.real()?, self.real()?, self.real()?);
        Ok(match (x, y, z) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        })
    }

    fn value(&mut self, field: &FieldDef) -> Result<Value, String> {
        if let Encoding::Binary { .. } = self.encoding {
            return self.binary_value(field);
        }
        let kind = field.kind;
        Ok(match kind {
            Kind::Int => Value::Int(self.int()?),
            Kind::Real => Value::Real(self.real()?),
            Kind::Vector => Value::Vector(self.vector()?),
            // Like a vector, an unset interval or box is one `?`.
            Kind::Interval | Kind::Box if self.peek() == Some(b'?') => {
                self.at += 1;
                match kind {
                    Kind::Interval => Value::Interval([None; 2]),
                    _ => Value::Box([None; 6]),
                }
            }
            Kind::Interval => Value::Interval([self.real()?, self.real()?]),
            Kind::Box => {
                let mut out = [None; 6];
                for v in &mut out {
                    *v = self.real()?;
                }
                Value::Box(out)
            }
            Kind::Char => Value::Char(self.next_byte().ok_or("the data ends inside a character field")?),
            Kind::Unknown => {
                return Err("the base schema layout of this field is not known (it is deleted in every schema seen)".into())
            }
        })
    }

    /// A field in binary (§3.3): by its type code's width. A logical is 0 or
    /// 1 in its byte and is read as text writes it, `F` or `T`.
    fn binary_value(&mut self, field: &FieldDef) -> Result<Value, String> {
        let real = |stream: &mut Self| -> Result<Option<f64>, String> {
            let value = stream.raw_f64()?;
            if !value.is_finite() {
                return Err(format!("a non-finite real at byte {}", stream.at - 8));
            }
            Ok((value != -3.14158e13).then_some(value))
        };
        Ok(match field.code {
            b'p' => Value::Int(Some(self.index()?)),
            b'u' => Value::Int(Some(i64::from(self.next_byte().ok_or("the data ends inside a byte field")?))),
            b'n' | b'w' => Value::Int(self.short()?),
            b'd' | b't' => Value::Int(self.long()?),
            b'f' => Value::Real(real(self)?),
            b'v' | b'h' => {
                let (x, y, z) = (real(self)?, real(self)?, real(self)?);
                Value::Vector(match (x, y, z) {
                    (Some(x), Some(y), Some(z)) => Some([x, y, z]),
                    _ => None,
                })
            }
            b'i' => Value::Interval([real(self)?, real(self)?]),
            b'b' => {
                let mut out = [None; 6];
                for v in &mut out {
                    *v = real(self)?;
                }
                Value::Box(out)
            }
            b'l' => Value::Char(match self.next_byte().ok_or("the data ends inside a logical field")? {
                0 => b'F',
                1 => b'T',
                other => return Err(format!("a logical byte {other}, expected 0 or 1")),
            }),
            b'c' => Value::Char(self.next_byte().ok_or("the data ends inside a character field")?),
            _ => {
                return Err("the base schema layout of this field is not known (it is deleted in every schema seen)".into())
            }
        })
    }

    /// A short string: its length, a space, then the characters (§2.1.2.1);
    /// in binary a one-byte length, then the characters.
    fn string(&mut self) -> Result<String, String> {
        let n = self.byte_int()?.ok_or("a string length is unset")?;
        let n = usize::try_from(n).map_err(|_| format!("string length {n} is negative"))?;
        if n > self.remaining() {
            return Err(format!("string length {n} runs past the end of the data"));
        }
        self.chars(n)
    }

    /// `n` characters of a char field, with the §3.2 and §2.1.3 escapes
    /// (`\0`, `\n`, `\r`, `\\`, and `\<d>` for d spaces) each counting as the
    /// characters they stand for.
    fn chars(&mut self, n: usize) -> Result<String, String> {
        if let Encoding::Binary { .. } = self.encoding {
            let bytes = self.data.get(self.at..self.at + n).ok_or("the data ends inside a string")?;
            self.at += n;
            return Ok(String::from_utf8_lossy(bytes).into_owned());
        }
        let mut out = Vec::with_capacity(n);
        while out.len() < n {
            let b = self.next_byte().ok_or("the data ends inside a string")?;
            if b != b'\\' {
                out.push(b);
                continue;
            }
            match self.next_byte() {
                Some(b'0') => out.push(0),
                Some(b'n') => out.push(b'\n'),
                Some(b'r') => out.push(b'\r'),
                Some(b'\\') => out.push(b'\\'),
                Some(d @ b'1'..=b'9') => out.extend(std::iter::repeat(b' ').take(usize::from(d - b'0'))),
                Some(other) => {
                    out.push(b'\\');
                    out.push(other);
                }
                None => return Err("the data ends inside a string escape".into()),
            }
        }
        out.truncate(n);
        Ok(String::from_utf8_lossy(&out).into_owned())
    }

    /// A field description in a schema delta (§2.1.2.2): name, pointer class,
    /// element count, type (when not a pointer), transmit flag (when variable).
    fn field_def(&mut self) -> Result<FieldDef, String> {
        let name = self.string()?;
        let pointer_class = self.short()?.unwrap_or(0);
        // Binary writes the element count as it writes an index (+1, two
        // bytes): every SolidWorks partition's deltas read to their end so.
        let elements = match self.encoding {
            Encoding::Text => self.int()?.ok_or("a field's element count is unset")?,
            Encoding::Binary { .. } => self.index()?,
        };
        let (kind, code) = if pointer_class != 0 {
            (Kind::Int, b'p')
        } else {
            let code = self.string()?;
            let code = code.bytes().next().ok_or_else(|| format!("field {name} has an empty type"))?;
            (schema::kind_of(code).ok_or_else(|| format!("field {name} has unknown type {:?}", char::from(code)))?, code)
        };
        if elements == 1 {
            self.next_byte().ok_or("the data ends inside a field description")?;
        }
        let count = match elements {
            0 => Count::Scalar,
            1 => Count::Variable,
            n if n > 1 && (n as u64) <= self.remaining() as u64 => Count::Fixed(n as usize),
            n => return Err(format!("field {name} has element count {n}")),
        };
        Ok(FieldDef { name, kind, count, code })
    }
}
