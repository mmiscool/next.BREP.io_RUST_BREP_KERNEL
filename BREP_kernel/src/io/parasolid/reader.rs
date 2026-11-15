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
    let bytes = text.as_bytes();
    if bytes.starts_with(b"PS") {
        return Err("this is a binary Parasolid file (.x_b); only text .x_t files are read".into());
    }
    let (header, body_start) = split_header(text);
    let data = join_records(&text[body_start..]);
    let mut stream = Stream { data: &data, at: 0 };
    stream.skip_blank();
    match stream.next_byte() {
        Some(b'T') => {}
        Some(b'P' | b'B') => {
            return Err("this is a binary Parasolid file (.x_b); only text .x_t files are read".into())
        }
        other => {
            return Err(format!(
                "not a Parasolid text transmit file: the part data starts with {:?}, expected 'T'",
                other.map(char::from)
            ))
        }
    }
    let modeller_version = stream.string()?;
    if modeller_version.contains("(partition)") {
        return Err("this is a Parasolid partition file; only part files are read".into());
    }
    let schema_name = stream.string()?;
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
        stream.int()?;
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
    let user_field_size = stream.int()?.unwrap_or(0).max(0) as usize;
    let mut layouts: HashMap<u16, Rc<NodeLayout>> = HashMap::default();
    let mut nodes = Vec::new();
    let mut by_index = HashMap::default();
    loop {
        let at = stream.at;
        let node_type = stream.int()?.ok_or("a node type is unset")?;
        if node_type == 1 {
            // Terminator: type 1, index 0.
            if stream.int()? == Some(0) {
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
    Ok(XtFile { modeller_version, schema: schema_name, header, nodes, by_index })
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
    read_layout(&mut Stream { data: delta.as_bytes(), at: 0 }, node_type)
        .map_err(|e| format!("the stored schema {stored} layout of node type {node_type}: {e}"))
}

/// A node type's layout from its first occurrence (§2.1.2.2): 255 for "as the
/// base", a new type's full description, or an edit script against the base.
fn read_layout(stream: &mut Stream, node_type: u16) -> Result<NodeLayout, String> {
    let count = stream.int()?.ok_or("a field count is unset")?;
    // A field count is a byte (§2.1.2.2); 255 means "as the base".
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
        let n = stream.int()?.ok_or("a variable length is unset")?;
        let n = usize::try_from(n).map_err(|_| format!("variable length {n} is negative"))?;
        // Every element takes at least one byte of the data.
        if n > stream.remaining() {
            return Err(format!("variable length {n} runs past the end of the data"));
        }
        n
    } else {
        0
    };
    let index = stream.int()?.ok_or("a node index is unset")?;
    let index = u32::try_from(index).ok().filter(|&i| i > 0).ok_or_else(|| format!("node index {index} is invalid"))?;
    let mut values = Vec::with_capacity(layout.fields.len());
    for field in &layout.fields {
        let value = match field.count {
            Count::Scalar => stream.value(field.kind),
            Count::Variable if matches!(field.kind, Kind::Char) => stream.chars(length).map(Value::Text),
            Count::Variable => (0..length).map(|_| stream.value(field.kind)).collect::<Result<_, _>>().map(Value::Array),
            Count::Fixed(n) => (0..n).map(|_| stream.value(field.kind)).collect::<Result<_, _>>().map(Value::Array),
        }
        .map_err(|e| format!("{} field {}: {e}", layout.name, field.name))?;
        values.push(value);
    }
    if layout.node_type < 1000 && carries_user_field(layout.node_type) {
        for _ in 0..user_field_size {
            stream.int()?;
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

struct Stream<'a> {
    data: &'a [u8],
    at: usize,
}

impl Stream<'_> {
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

    fn value(&mut self, kind: Kind) -> Result<Value, String> {
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

    /// A short string: its length, a space, then the characters (§2.1.2.1).
    fn string(&mut self) -> Result<String, String> {
        let n = self.int()?.ok_or("a string length is unset")?;
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
        let pointer_class = self.int()?.unwrap_or(0);
        let elements = self.int()?.ok_or("a field's element count is unset")?;
        let kind = if pointer_class != 0 {
            Kind::Int
        } else {
            let code = self.string()?;
            let code = code.bytes().next().ok_or_else(|| format!("field {name} has an empty type"))?;
            schema::kind_of(code).ok_or_else(|| format!("field {name} has unknown type {:?}", char::from(code)))?
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
        Ok(FieldDef { name, kind, count })
    }
}
