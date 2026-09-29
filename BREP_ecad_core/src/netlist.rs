//! A schematic's netlist as KiCad writes one: the `.net` s-expression file
//! (export version "E") that Pcbnew, and most tools that read KiCad, take in.
//!
//! The nets are [`Document::netlist`]'s, under the names the Connectivity panel shows
//! them by, so what the panel lists is what the file says. A net with no pin (a wire
//! or label joined to nothing) is left out: a netlist joins pins. The file carries no
//! date, so the same schematic writes the same bytes.
//!
//! [`read_kicad_netlist`] reads one back, for the tests and for anyone checking a
//! file this wrote.
use crate::{Document, DocumentKind};
use std::collections::{BTreeMap, BTreeSet};

/// A string as KiCad quotes one.
fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

impl Document {
    /// The KiCad netlist (`.net`) of this schematic. `source` names the design in
    /// the file's header, as KiCad names its schematic file there.
    pub fn kicad_netlist(&self, source: &str) -> Result<String, String> {
        if self.kind == DocumentKind::Wiring {
            return Err("a wiring diagram has connections, not nets; export its connection list instead".into());
        }
        if self.components.is_empty() {
            return Err("the schematic has no parts, so there is no netlist to write".into());
        }
        let mut s = String::from("(export (version \"E\")\n");
        s += &format!(
            "  (design\n    (source {})\n    (tool {}))\n",
            quote(source),
            quote(concat!("BREP eCAD ", env!("CARGO_PKG_VERSION")))
        );
        // KiCad leaves power symbols and flags out, as it leaves out every symbol
        // whose reference starts with `#`: they name a net and are not parts. The
        // net keeps their name and its other pins.
        let virtual_part = |reference: &str| reference.starts_with('#');
        let mut components: Vec<_> = self.components.iter().filter(|c| !virtual_part(&c.reference)).collect();
        components.sort_by(|a, b| natural(&a.reference).cmp(&natural(&b.reference)));
        s += "  (components";
        for c in &components {
            let (lib, part) = lib_part(&c.symbol.library_id);
            let footprint = c
                .symbol
                .properties
                .get("Footprint")
                .filter(|f| !f.trim().is_empty())
                .cloned()
                .or_else(|| c.pads.as_ref().map(|p| p.name.clone()).filter(|n| !n.trim().is_empty()));
            s += &format!("\n    (comp (ref {})\n      (value {})", quote(&c.reference), quote(&c.value));
            if let Some(footprint) = footprint {
                s += &format!("\n      (footprint {})", quote(&footprint));
            }
            s += &format!(
                "\n      (libsource (lib {}) (part {}) (description {}))\n      (tstamps {}))",
                quote(lib),
                quote(part),
                quote(&c.symbol.description),
                quote(&c.id.to_string())
            );
        }
        s += ")\n  (libparts";
        let mut seen = BTreeSet::new();
        for c in &components {
            if !seen.insert(c.symbol.library_id.clone()) {
                continue;
            }
            let (lib, part) = lib_part(&c.symbol.library_id);
            s += &format!(
                "\n    (libpart (lib {}) (part {})\n      (description {})\n      (pins",
                quote(lib),
                quote(part),
                quote(&c.symbol.description)
            );
            for p in &c.symbol.pins {
                s += &format!(
                    "\n        (pin (num {}) (name {}) (type {}))",
                    quote(&p.number),
                    quote(&p.name),
                    quote(&p.electrical_type)
                );
            }
            s += "))";
        }
        s += ")\n  (nets";
        let netlist = self.netlist();
        let mut code = 0;
        for net in netlist.nets.iter().filter(|n| n.pins.iter().any(|p| !virtual_part(&p.reference))) {
            code += 1;
            s += &format!("\n    (net (code {}) (name {})", quote(&code.to_string()), quote(&net.name));
            let mut pins: Vec<_> = net.pins.iter().filter(|p| !virtual_part(&p.reference)).collect();
            pins.sort_by(|a, b| (natural(&a.reference), natural(&a.number)).cmp(&(natural(&b.reference), natural(&b.number))));
            for p in pins {
                let function = self
                    .components
                    .iter()
                    .find(|c| c.id == p.component_id)
                    .and_then(|c| c.symbol.pins.iter().find(|q| q.number == p.number))
                    .map(|q| q.name.clone())
                    .filter(|n| !n.is_empty() && n != "~");
                s += &format!("\n      (node (ref {}) (pin {})", quote(&p.reference), quote(&p.number));
                if let Some(function) = function {
                    s += &format!(" (pinfunction {})", quote(&function));
                }
                s += &format!(" (pintype {}))", quote(&p.electrical_type));
            }
            s += ")";
        }
        s += "))\n";
        Ok(s)
    }
}
/// `Device:R` as KiCad's `(lib "Device") (part "R")`; a name with no library is its
/// own part in an empty library.
fn lib_part(library_id: &str) -> (&str, &str) {
    library_id.split_once(':').unwrap_or(("", library_id))
}
/// Sorts `R2` before `R10` and `2` before `10`.
fn natural(text: &str) -> (String, u64, String) {
    let prefix = text.trim_end_matches(|c: char| c.is_ascii_digit());
    (
        prefix.to_owned(),
        text[prefix.len()..].parse().unwrap_or(0),
        text.to_owned(),
    )
}

/// An s-expression: a list or an atom (a quoted string is an atom without quotes).
#[derive(Clone, Debug, PartialEq)]
enum Sexp {
    Atom(String),
    List(Vec<Sexp>),
}
impl Sexp {
    fn head(&self) -> Option<&str> {
        match self {
            Sexp::List(items) => match items.first() {
                Some(Sexp::Atom(a)) => Some(a),
                _ => None,
            },
            Sexp::Atom(_) => None,
        }
    }
    fn children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Sexp> + 'a {
        let items: &[Sexp] = match self {
            Sexp::List(items) => items,
            Sexp::Atom(_) => &[],
        };
        items.iter().filter(move |c| c.head() == Some(name))
    }
    /// The first atom of the child list `name`: `(ref "R1")` gives `R1`.
    fn value<'a>(&'a self, name: &'a str) -> Option<&'a str> {
        match self.children(name).next()? {
            Sexp::List(items) => match items.get(1)? {
                Sexp::Atom(a) => Some(a),
                Sexp::List(_) => None,
            },
            Sexp::Atom(_) => None,
        }
    }
}
fn parse(text: &str) -> Result<Sexp, String> {
    let chars: Vec<char> = text.chars().collect();
    let mut i = 0;
    fn skip(chars: &[char], i: &mut usize) {
        while *i < chars.len() && chars[*i].is_whitespace() {
            *i += 1;
        }
    }
    fn item(chars: &[char], i: &mut usize) -> Result<Sexp, String> {
        skip(chars, i);
        match chars.get(*i) {
            None => Err("unexpected end of the file".into()),
            Some('(') => {
                *i += 1;
                let mut items = vec![];
                loop {
                    skip(chars, i);
                    match chars.get(*i) {
                        None => return Err("a list is not closed".into()),
                        Some(')') => {
                            *i += 1;
                            return Ok(Sexp::List(items));
                        }
                        _ => items.push(item(chars, i)?),
                    }
                }
            }
            Some(')') => Err("an unexpected )".into()),
            Some('"') => {
                *i += 1;
                let mut out = String::new();
                loop {
                    match chars.get(*i) {
                        None => return Err("a string is not closed".into()),
                        Some('"') => {
                            *i += 1;
                            return Ok(Sexp::Atom(out));
                        }
                        Some('\\') => {
                            *i += 1;
                            match chars.get(*i) {
                                Some('n') => out.push('\n'),
                                Some(c) => out.push(*c),
                                None => return Err("a string is not closed".into()),
                            }
                            *i += 1;
                        }
                        Some(c) => {
                            out.push(*c);
                            *i += 1;
                        }
                    }
                }
            }
            Some(_) => {
                let start = *i;
                while *i < chars.len() && !chars[*i].is_whitespace() && chars[*i] != '(' && chars[*i] != ')' {
                    *i += 1;
                }
                Ok(Sexp::Atom(chars[start..*i].iter().collect()))
            }
        }
    }
    let root = item(&chars, &mut i)?;
    skip(&chars, &mut i);
    if i != chars.len() {
        return Err("text after the netlist's closing )".into());
    }
    Ok(root)
}

/// What a KiCad netlist file says: each component's value and footprint by
/// reference, and each net's pins by name.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NetlistFile {
    pub version: String,
    /// Reference to (value, footprint).
    pub components: BTreeMap<String, (String, Option<String>)>,
    /// Net name to its pins, `(reference, pin number, pin type)`.
    pub nets: BTreeMap<String, BTreeSet<(String, String, String)>>,
}
/// Read a KiCad `.net` file: the components and the nets, nothing else.
pub fn read_kicad_netlist(text: &str) -> Result<NetlistFile, String> {
    let root = parse(text)?;
    if root.head() != Some("export") {
        return Err("not a KiCad netlist: it does not begin with (export".into());
    }
    let mut out = NetlistFile {
        version: root.value("version").unwrap_or_default().to_owned(),
        ..Default::default()
    };
    for list in root.children("components") {
        for comp in list.children("comp") {
            let reference = comp.value("ref").ok_or("a comp has no ref")?.to_owned();
            let value = comp.value("value").unwrap_or_default().to_owned();
            let footprint = comp.value("footprint").map(str::to_owned);
            if out.components.insert(reference.clone(), (value, footprint)).is_some() {
                return Err(format!("{reference} is listed twice"));
            }
        }
    }
    for list in root.children("nets") {
        for net in list.children("net") {
            let name = net.value("name").ok_or("a net has no name")?.to_owned();
            let mut pins = BTreeSet::new();
            for node in net.children("node") {
                pins.insert((
                    node.value("ref").ok_or("a node has no ref")?.to_owned(),
                    node.value("pin").ok_or("a node has no pin")?.to_owned(),
                    node.value("pintype").unwrap_or_default().to_owned(),
                ));
            }
            if out.nets.insert(name.clone(), pins).is_some() {
                return Err(format!("net {name} is listed twice"));
            }
        }
    }
    Ok(out)
}
