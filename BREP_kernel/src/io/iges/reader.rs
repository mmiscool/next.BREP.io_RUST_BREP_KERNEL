//! IGES record framework — the read side.
//!
//! Splits a fixed 80-column IGES document into its five sections, parses the
//! Global section (delimiters, units, model scale), the two-line 20-field
//! Directory Entries, and the free-format Parameter Data (respecting Hollerith
//! strings and the DE↔PD back-pointer), then exposes each entity as a
//! [`ParsedEntity`] with its directory metadata and raw parameter tokens.
//!
//! Tolerant by design: accepts CRLF or LF line endings, short (un-padded)
//! lines, and default `,`/`;` delimiters when the Global section omits them.

use rustc_hash::FxHashMap as HashMap;

/// The subset of directory-entry fields the importer needs.
#[derive(Clone, Debug)]
pub(crate) struct DirectoryEntry {
    pub entity_type: i64,
    /// Field 15: form number (retained for entity-form disambiguation).
    #[allow(dead_code)]
    pub form: i64,
}

/// One resolved entity: its directory record plus the raw parameter tokens that
/// FOLLOW the leading entity-type number.
#[derive(Clone, Debug)]
pub(crate) struct ParsedEntity {
    pub de: DirectoryEntry,
    pub params: Vec<String>,
}

impl ParsedEntity {
    pub fn real(&self, index: usize) -> Result<f64, String> {
        let token = self
            .params
            .get(index)
            .ok_or_else(|| format!("iges: entity {} missing param {index}", self.de.entity_type))?;
        parse_real(token)
    }

    pub fn int(&self, index: usize) -> Result<i64, String> {
        let token = self
            .params
            .get(index)
            .ok_or_else(|| format!("iges: entity {} missing param {index}", self.de.entity_type))?;
        token
            .trim()
            .parse::<i64>()
            .map_err(|_| format!("iges: entity {} param {index} not an integer: {token:?}", self.de.entity_type))
    }
}

/// Global-section values relevant to geometry reconstruction.
#[derive(Clone, Debug)]
pub(crate) struct GlobalSection {
    pub units_flag: i64,
    pub units_name: String,
    pub model_scale: f64,
}

impl GlobalSection {
    /// Multiplier converting stored model coordinates to millimetres.
    pub fn length_scale_mm(&self) -> Result<f64, String> {
        // Prefer the explicit units flag; fall back to the units name.
        let per_unit = match self.units_flag {
            1 => 25.4,     // inch
            2 => 1.0,      // millimetre
            4 => 304.8,    // foot
            5 => 1_609_344.0, // mile
            6 => 1000.0,   // metre
            7 => 1_000_000.0, // kilometre
            8 => 0.0254,   // mil (0.001 inch)
            9 => 0.001,    // micron
            10 => 10.0,    // centimetre
            11 => 25.4e-6, // microinch
            3 | 0 => match self.units_name.trim().to_ascii_uppercase().as_str() {
                "MM" => 1.0,
                "IN" | "INCH" | "INCHES" => 25.4,
                "M" | "METER" | "METERS" | "METRE" => 1000.0,
                "CM" => 10.0,
                "FT" | "FEET" => 304.8,
                other => {
                    return Err(format!(
                        "iges_import: unsupported units name {other:?} (flag {})",
                        self.units_flag
                    ))
                }
            },
            other => return Err(format!("iges_import: unsupported units flag {other}")),
        };
        let scale = if self.model_scale.abs() > 1e-12 {
            per_unit / self.model_scale
        } else {
            per_unit
        };
        Ok(scale)
    }
}

/// A parsed IGES document.
pub(crate) struct IgesFile {
    pub global: GlobalSection,
    pub entities: Vec<ParsedEntity>,
    /// DE pointer → index into `entities`.
    by_pointer: HashMap<i64, usize>,
}

impl IgesFile {
    pub fn entity(&self, de_pointer: i64) -> Result<&ParsedEntity, String> {
        self.by_pointer
            .get(&de_pointer)
            .and_then(|&i| self.entities.get(i))
            .ok_or_else(|| format!("iges: dangling DE pointer {de_pointer}"))
    }

    pub fn entities_of_type(&self, entity_type: i64) -> impl Iterator<Item = &ParsedEntity> {
        self.entities
            .iter()
            .filter(move |e| e.de.entity_type == entity_type)
    }
}

/// Parse a complete IGES document.
pub(crate) fn parse_iges(text: &str) -> Result<IgesFile, String> {
    // Section buffers keyed by the column-73 letter.
    let mut global_raw = String::new();
    let mut de_lines: Vec<String> = Vec::new();
    // DE pointer → concatenated cols 1-64 of its parameter data.
    let mut pd_by_pointer: HashMap<i64, String> = HashMap::default();
    // Preserve first-seen order of DE pointers in the PD section.
    let mut pd_order: Vec<i64> = Vec::new();

    for raw_line in text.split('\n') {
        let line = raw_line.strip_suffix('\r').unwrap_or(raw_line);
        if line.len() < 73 {
            // Too short to carry a section letter in column 73 — skip blanks.
            if line.trim().is_empty() {
                continue;
            }
        }
        let chars: Vec<char> = line.chars().collect();
        if chars.len() < 73 {
            continue;
        }
        let section = chars[72];
        let cols_1_72: String = chars[..72.min(chars.len())].iter().collect();
        match section {
            'S' => { /* start section: human prose, ignored */ }
            'G' => {
                global_raw.push_str(&cols_1_72);
            }
            'D' => {
                de_lines.push(cols_1_72);
            }
            'P' => {
                // Data is columns 1-64; columns 65-72 hold the DE back-pointer.
                let data: String = chars[..64.min(chars.len())].iter().collect();
                let ptr_field: String = if chars.len() >= 72 {
                    chars[64..72].iter().collect()
                } else if chars.len() > 64 {
                    chars[64..].iter().collect()
                } else {
                    String::new()
                };
                let de_pointer: i64 = ptr_field
                    .trim()
                    .parse()
                    .map_err(|_| format!("iges: bad PD back-pointer {ptr_field:?}"))?;
                if !pd_by_pointer.contains_key(&de_pointer) {
                    pd_order.push(de_pointer);
                }
                pd_by_pointer.entry(de_pointer).or_default().push_str(&data);
            }
            'T' => { /* terminate: counts, not needed for reconstruction */ }
            _ => { /* unknown section letter — ignore */ }
        }
    }

    let global = parse_global(&global_raw)?;

    // ---- Directory entries --------------------------------------------------
    if de_lines.len() % 2 != 0 {
        return Err(format!(
            "iges: directory-entry section has an odd line count ({})",
            de_lines.len()
        ));
    }
    let mut directory: HashMap<i64, DirectoryEntry> = HashMap::default();
    for pair_index in 0..de_lines.len() / 2 {
        let line1 = &de_lines[pair_index * 2];
        let line2 = &de_lines[pair_index * 2 + 1];
        let entity_type = de_field(line1, 0)?;
        let form = de_field(line2, 4)?;
        let de_pointer = (2 * pair_index + 1) as i64;
        directory.insert(de_pointer, DirectoryEntry { entity_type, form });
    }

    // ---- Parameter data → tokens per entity --------------------------------
    let mut entities: Vec<ParsedEntity> = Vec::new();
    let mut by_pointer: HashMap<i64, usize> = HashMap::default();
    // Iterate DE pointers in directory order for determinism.
    let mut de_pointers: Vec<i64> = directory.keys().copied().collect();
    de_pointers.sort_unstable();
    for de_pointer in de_pointers {
        let de = directory[&de_pointer].clone();
        let raw = pd_by_pointer
            .get(&de_pointer)
            .ok_or_else(|| format!("iges: entity DE {de_pointer} has no parameter data"))?;
        let tokens = tokenize_parameters(raw, global.param_delim(), global.record_delim())?;
        if tokens.is_empty() {
            return Err(format!("iges: entity DE {de_pointer} has empty parameter data"));
        }
        // The first parameter token is the entity type; it must match the DE.
        let pd_type: i64 = tokens[0]
            .trim()
            .parse()
            .map_err(|_| format!("iges: entity DE {de_pointer} has a non-numeric type token {:?}", tokens[0]))?;
        if pd_type != de.entity_type {
            return Err(format!(
                "iges: entity DE {de_pointer} type {} disagrees with its parameter-data type {pd_type}",
                de.entity_type
            ));
        }
        let params = tokens[1..].to_vec();
        by_pointer.insert(de_pointer, entities.len());
        entities.push(ParsedEntity { de, params });
    }

    Ok(IgesFile {
        global,
        entities,
        by_pointer,
    })
}

impl GlobalSection {
    fn param_delim(&self) -> char {
        ','
    }
    fn record_delim(&self) -> char {
        ';'
    }
}

/// Read an 8-column integer field (0-based `index`) from a 72-column DE line.
fn de_field(line: &str, index: usize) -> Result<i64, String> {
    let chars: Vec<char> = line.chars().collect();
    let start = index * 8;
    let end = (start + 8).min(chars.len());
    if start >= chars.len() {
        return Ok(0);
    }
    let field: String = chars[start..end].iter().collect();
    let trimmed = field.trim();
    if trimmed.is_empty() {
        return Ok(0);
    }
    trimmed
        .parse::<i64>()
        .map_err(|_| format!("iges: directory field {index} not an integer: {field:?}"))
}

/// Parse the Global section into the values the importer needs. Delimiters are
/// assumed to be the defaults `,` and `;` (params 1-2 declare them but they are
/// the defaults for every writer we interoperate with).
fn parse_global(raw: &str) -> Result<GlobalSection, String> {
    let tokens = tokenize_parameters(raw, ',', ';')?;
    // Standard positional layout; tokens[13] = units flag, [14] = units name,
    // [12] = model space scale. Guard against short globals.
    let get = |i: usize| tokens.get(i).map(|s| s.as_str()).unwrap_or("");
    let model_scale = parse_real(get(12)).unwrap_or(1.0);
    let units_flag = get(13).trim().parse::<i64>().unwrap_or(2);
    let units_name = get(14).trim().to_string();
    Ok(GlobalSection {
        units_flag,
        units_name: if units_name.is_empty() {
            "MM".to_string()
        } else {
            units_name
        },
        model_scale,
    })
}

/// Free-format tokenizer for Global and Parameter Data sections. Splits on the
/// parameter delimiter, stops at the record delimiter, and consumes Hollerith
/// strings (`nHxxxx`) literally so embedded delimiters do not split tokens.
fn tokenize_parameters(raw: &str, param_delim: char, record_delim: char) -> Result<Vec<String>, String> {
    let chars: Vec<char> = raw.chars().collect();
    let mut tokens: Vec<String> = Vec::new();
    let mut current = String::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c == param_delim {
            tokens.push(current.trim().to_string());
            current.clear();
            i += 1;
            continue;
        }
        if c == record_delim {
            tokens.push(current.trim().to_string());
            current.clear();
            // Everything after the first record delimiter is padding/next data.
            return Ok(tokens);
        }
        if c == 'H' && !current.trim().is_empty() && current.trim().chars().all(|d| d.is_ascii_digit()) {
            // Hollerith: the accumulated digits give the literal length.
            let length: usize = current.trim().parse().unwrap_or(0);
            current.clear();
            let mut literal = String::new();
            for _ in 0..length {
                i += 1;
                if i < chars.len() {
                    literal.push(chars[i]);
                }
            }
            tokens.push(literal);
            i += 1;
            // Skip the following delimiter if present.
            if i < chars.len() && (chars[i] == param_delim) {
                i += 1;
            } else if i < chars.len() && chars[i] == record_delim {
                return Ok(tokens);
            }
            continue;
        }
        current.push(c);
        i += 1;
    }
    let last = current.trim();
    if !last.is_empty() {
        tokens.push(last.to_string());
    }
    Ok(tokens)
}

/// Parse an IGES real, accepting Fortran `D` exponents (`1.0D+01`).
pub(crate) fn parse_real(token: &str) -> Result<f64, String> {
    let t = token.trim();
    if t.is_empty() {
        return Ok(0.0);
    }
    let normalized = t.replace(['D', 'd'], "E");
    normalized
        .parse::<f64>()
        .map_err(|_| format!("iges: not a real number: {token:?}"))
}
