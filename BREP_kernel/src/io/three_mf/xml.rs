//! A bounded, namespace-aware XML pull reader for the 3MF model part.
//!
//! Enough of XML 1.0 for the 3MF core specification and nothing more: elements,
//! attributes, the five predefined entities plus numeric character references,
//! comments, CDATA and processing instructions. Text content is skipped —
//! every value the core model carries is an attribute.
//!
//! Refused by design: a `<!DOCTYPE …>` declaration anywhere (so no entity
//! expansion, no external entities, no billion-laughs), and nesting deeper than
//! [`MAX_DEPTH`]. Elements outside the 3MF core namespace are reported with
//! their resolved namespace so the caller can skip an extension subtree whole.

/// Nesting bound. The core model reaches depth 6 (`model/resources/object/
/// mesh/triangles/triangle`); anything past this is an attack or a generator
/// bug, not a model.
pub const MAX_DEPTH: usize = 64;

/// A parse failure with the byte offset it was found at.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XmlError {
    pub message: String,
    pub offset: usize,
}

impl core::fmt::Display for XmlError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} (at byte {})", self.message, self.offset)
    }
}

/// One attribute, with its prefix split off. Namespace declarations are
/// consumed by the reader and never handed out.
#[derive(Clone, Debug)]
pub struct Attribute {
    pub prefix: String,
    pub name: String,
    pub value: String,
}

/// A start tag: its resolved namespace, local name, attributes, and whether it
/// was written `<tag/>`.
#[derive(Clone, Debug)]
pub struct StartTag {
    pub namespace: String,
    pub name: String,
    pub attributes: Vec<Attribute>,
    pub empty: bool,
}

impl StartTag {
    /// The value of an unprefixed attribute. Prefixed attributes belong to an
    /// extension (`p:UUID`, `s:…`) and are ignored by design.
    pub fn attribute(&self, name: &str) -> Option<&str> {
        self.attributes
            .iter()
            .find(|attribute| attribute.prefix.is_empty() && attribute.name == name)
            .map(|attribute| attribute.value.as_str())
    }
}

#[derive(Clone, Debug)]
pub enum Event {
    Start(StartTag),
    /// An end tag, with the local name it closed. The reader has already
    /// checked it against the open element, prefix included.
    End { name: String },
    Eof,
}

/// The namespace bindings in force, innermost last.
struct Scope {
    /// `(prefix, uri)`; the default namespace uses an empty prefix.
    bindings: Vec<(String, String)>,
    /// How many bindings this element added, so `End` can pop exactly those.
    depth_marks: Vec<usize>,
}

pub struct XmlReader<'a> {
    data: &'a [u8],
    at: usize,
    scope: Scope,
    /// Open elements, for end-tag matching and depth bounds.
    open: Vec<(String, String)>,
}

impl<'a> XmlReader<'a> {
    pub fn new(text: &'a str) -> Self {
        Self {
            data: text.as_bytes(),
            at: 0,
            scope: Scope { bindings: Vec::new(), depth_marks: Vec::new() },
            open: Vec::new(),
        }
    }

    fn error<T>(&self, message: impl Into<String>) -> Result<T, XmlError> {
        Err(XmlError { message: message.into(), offset: self.at })
    }

    fn peek(&self) -> Option<u8> {
        self.data.get(self.at).copied()
    }

    fn starts_with(&self, text: &str) -> bool {
        self.data[self.at..].starts_with(text.as_bytes())
    }

    fn skip_whitespace(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\r' | b'\n')) {
            self.at += 1;
        }
    }

    /// Advance past `text`, or fail naming what was expected.
    fn expect(&mut self, text: &str) -> Result<(), XmlError> {
        if self.starts_with(text) {
            self.at += text.len();
            Ok(())
        } else {
            self.error(format!("expected '{text}'"))
        }
    }

    /// The next event. Text between tags is discarded.
    pub fn next(&mut self) -> Result<Event, XmlError> {
        loop {
            // Skip character data up to the next '<'.
            while let Some(byte) = self.peek() {
                if byte == b'<' {
                    break;
                }
                self.at += 1;
            }
            let Some(_) = self.peek() else {
                if let Some((_, name)) = self.open.last() {
                    return self.error(format!("the document ends inside <{name}>"));
                }
                return Ok(Event::Eof);
            };
            if self.starts_with("<!--") {
                self.at += 4;
                let Some(end) = find(self.data, self.at, b"-->") else {
                    return self.error("an XML comment is never closed");
                };
                self.at = end + 3;
                continue;
            }
            if self.starts_with("<![CDATA[") {
                self.at += 9;
                let Some(end) = find(self.data, self.at, b"]]>") else {
                    return self.error("a CDATA section is never closed");
                };
                self.at = end + 3;
                continue;
            }
            if self.starts_with("<!DOCTYPE") {
                return self.error(
                    "a DOCTYPE declaration is refused: this reader expands no entities",
                );
            }
            if self.starts_with("<!") {
                return self.error("unsupported XML declaration");
            }
            if self.starts_with("<?") {
                self.at += 2;
                let Some(end) = find(self.data, self.at, b"?>") else {
                    return self.error("an XML processing instruction is never closed");
                };
                self.at = end + 2;
                continue;
            }
            if self.starts_with("</") {
                self.at += 2;
                let name = self.read_name()?;
                self.skip_whitespace();
                self.expect(">")?;
                let Some((open_prefix, open_name)) = self.open.pop() else {
                    return self.error(format!("</{name}> closes nothing"));
                };
                let (prefix, local) = split_name(&name);
                if prefix != open_prefix || local != open_name {
                    return self
                        .error(format!("</{name}> does not close the open element"));
                }
                // Resolving the prefix here would add nothing: the tag was
                // matched against the open element before the scope popped.
                self.pop_scope();
                return Ok(Event::End { name: local });
            }
            // A start tag.
            self.at += 1;
            let name = self.read_name()?;
            let (prefix, local) = split_name(&name);
            let mut attributes = Vec::new();
            let mut declared = 0usize;
            loop {
                self.skip_whitespace();
                if self.starts_with("/>") {
                    self.at += 2;
                    self.scope.depth_marks.push(declared);
                    let namespace = self.resolve(&prefix)?;
                    self.pop_scope();
                    return Ok(Event::Start(StartTag {
                        namespace,
                        name: local,
                        attributes,
                        empty: true,
                    }));
                }
                if self.starts_with(">") {
                    self.at += 1;
                    self.scope.depth_marks.push(declared);
                    if self.open.len() + 1 > MAX_DEPTH {
                        return self
                            .error(format!("XML nesting deeper than {MAX_DEPTH} elements"));
                    }
                    let namespace = self.resolve(&prefix)?;
                    self.open.push((prefix, local.clone()));
                    return Ok(Event::Start(StartTag {
                        namespace,
                        name: local,
                        attributes,
                        empty: false,
                    }));
                }
                let attribute_name = self.read_name()?;
                self.skip_whitespace();
                self.expect("=")?;
                self.skip_whitespace();
                let value = self.read_attribute_value()?;
                let (attribute_prefix, attribute_local) = split_name(&attribute_name);
                if attribute_name == "xmlns" {
                    self.scope.bindings.push((String::new(), value));
                    declared += 1;
                } else if attribute_prefix == "xmlns" {
                    self.scope.bindings.push((attribute_local, value));
                    declared += 1;
                } else {
                    attributes.push(Attribute {
                        prefix: attribute_prefix,
                        name: attribute_local,
                        value,
                    });
                }
            }
        }
    }

    /// Skip the subtree of the start tag just returned (a no-op for an empty
    /// element) — how an extension namespace is ignored.
    pub fn skip_subtree(&mut self, tag: &StartTag) -> Result<(), XmlError> {
        if tag.empty {
            return Ok(());
        }
        let mut depth = 1usize;
        loop {
            match self.next()? {
                Event::Start(inner) if !inner.empty => depth += 1,
                Event::Start(_) => {}
                Event::End { .. } => {
                    depth -= 1;
                    if depth == 0 {
                        return Ok(());
                    }
                }
                Event::Eof => return self.error("the document ends inside a skipped element"),
            }
        }
    }

    fn pop_scope(&mut self) {
        if let Some(declared) = self.scope.depth_marks.pop() {
            let keep = self.scope.bindings.len().saturating_sub(declared);
            self.scope.bindings.truncate(keep);
        }
    }

    fn resolve(&self, prefix: &str) -> Result<String, XmlError> {
        if prefix == "xml" {
            return Ok("http://www.w3.org/XML/1998/namespace".into());
        }
        match self
            .scope
            .bindings
            .iter()
            .rev()
            .find(|(bound, _)| bound == prefix)
        {
            Some((_, uri)) => Ok(uri.clone()),
            None if prefix.is_empty() => Ok(String::new()),
            None => Err(XmlError {
                message: format!("namespace prefix '{prefix}' is not declared"),
                offset: self.at,
            }),
        }
    }

    fn read_name(&mut self) -> Result<String, XmlError> {
        let start = self.at;
        while let Some(byte) = self.peek() {
            if byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':') {
                self.at += 1;
            } else {
                break;
            }
        }
        if self.at == start {
            return self.error("expected an element or attribute name");
        }
        core::str::from_utf8(&self.data[start..self.at])
            .map(str::to_owned)
            .map_err(|_| XmlError { message: "a name is not UTF-8".into(), offset: start })
    }

    fn read_attribute_value(&mut self) -> Result<String, XmlError> {
        let quote = match self.peek() {
            Some(byte @ (b'"' | b'\'')) => byte,
            _ => return self.error("an attribute value must be quoted"),
        };
        self.at += 1;
        let start = self.at;
        while let Some(byte) = self.peek() {
            if byte == quote {
                let raw = core::str::from_utf8(&self.data[start..self.at]).map_err(|_| {
                    XmlError { message: "an attribute value is not UTF-8".into(), offset: start }
                })?;
                let value = unescape(raw).ok_or(XmlError {
                    message: "an attribute value has an unknown entity reference".into(),
                    offset: start,
                })?;
                self.at += 1;
                return Ok(value);
            }
            if byte == b'<' {
                return self.error("'<' is not allowed inside an attribute value");
            }
            self.at += 1;
        }
        self.error("an attribute value is never closed")
    }
}

fn find(data: &[u8], from: usize, needle: &[u8]) -> Option<usize> {
    if from >= data.len() || needle.is_empty() {
        return None;
    }
    data[from..]
        .windows(needle.len())
        .position(|window| window == needle)
        .map(|at| at + from)
}

/// Split `prefix:local`; an unprefixed name yields an empty prefix.
fn split_name(name: &str) -> (String, String) {
    match name.split_once(':') {
        Some((prefix, local)) => (prefix.to_owned(), local.to_owned()),
        None => (String::new(), name.to_owned()),
    }
}

/// Expand the five predefined entities and numeric character references. Any
/// other reference is rejected — this reader has no entity table to consult.
fn unescape(text: &str) -> Option<String> {
    if !text.contains('&') {
        return Some(text.to_owned());
    }
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(at) = rest.find('&') {
        out.push_str(&rest[..at]);
        let tail = &rest[at + 1..];
        let end = tail.find(';')?;
        let reference = &tail[..end];
        match reference {
            "amp" => out.push('&'),
            "lt" => out.push('<'),
            "gt" => out.push('>'),
            "quot" => out.push('"'),
            "apos" => out.push('\''),
            _ => {
                let code = if let Some(hex) = reference.strip_prefix("#x") {
                    u32::from_str_radix(hex, 16).ok()?
                } else if let Some(decimal) = reference.strip_prefix('#') {
                    decimal.parse::<u32>().ok()?
                } else {
                    return None;
                };
                out.push(char::from_u32(code)?);
            }
        }
        rest = &tail[end + 1..];
    }
    out.push_str(rest);
    Some(out)
}
