//! IGES (IGES 5.3 / ASME Y14.26M) record framework — the write side.
//!
//! An IGES file is fixed 80-column ASCII split into five sections marked by a
//! letter in column 73: **S**tart, **G**lobal, **D**irectory Entry,
//! **P**arameter Data, **T**erminate. Columns 74-80 carry a 1-based sequence
//! number that runs independently within each section.
//!
//! * The **Directory Entry** section stores two 80-column lines per entity, 20
//!   fixed 8-column fields; each entity's DE pointer is the odd sequence number
//!   of its first DE line (`2*index + 1`).
//! * The **Parameter Data** section is free-format: the entity's type number
//!   followed by comma-separated parameters, terminated by a semicolon, wrapped
//!   in columns 1-64 with the owning DE's back-pointer right-justified in
//!   columns 65-72.
//!
//! [`IgesWriter`] hides all of that: callers push [`IgesEntity`] values (type +
//! already-stringified parameters), receive the entity's DE pointer for
//! cross-references, and finally call [`IgesWriter::finish`] to emit the whole
//! document with correct sequence numbering and the terminate record.

/// A single entity queued for output. `params` are the parameter-data fields
/// that FOLLOW the leading entity-type number (the writer prepends the type).
pub(crate) struct IgesEntity {
    pub entity_type: i64,
    /// Parameter-data fields after the entity type number, already formatted.
    pub params: Vec<String>,
    /// Directory-entry form number (field 15).
    pub form: i64,
    /// Up to 8 characters, directory-entry label (field 18).
    pub label: String,
}

impl IgesEntity {
    pub fn new(entity_type: i64, params: Vec<String>) -> Self {
        Self {
            entity_type,
            params,
            form: 0,
            label: String::new(),
        }
    }
}

/// Global-section values that vary per export.
pub(crate) struct GlobalParams {
    pub product_id: String,
    pub file_name: String,
    pub units_flag: i64,
    pub units_name: String,
    pub timestamp: String,
    pub min_resolution: f64,
}

/// Accumulates entities and start-section prose, then serializes a complete
/// IGES document.
pub(crate) struct IgesWriter {
    start_lines: Vec<String>,
    entities: Vec<IgesEntity>,
}

impl IgesWriter {
    pub fn new() -> Self {
        Self {
            start_lines: Vec::new(),
            entities: Vec::new(),
        }
    }

    pub fn add_start_line(&mut self, text: &str) {
        self.start_lines.push(text.to_string());
    }

    /// Queue an entity and return its DE pointer (the odd sequence number of
    /// its first directory-entry line), usable immediately as a cross-reference
    /// in later entities' parameters.
    pub fn add_entity(&mut self, entity: IgesEntity) -> i64 {
        let index = self.entities.len();
        self.entities.push(entity);
        (2 * index + 1) as i64
    }

    /// Serialize the accumulated document.
    pub fn finish(self, global: GlobalParams) -> String {
        // ---- Parameter Data layout ------------------------------------------
        // Each entity's fields are: [type, params...]. Wrap free-format into
        // <=64-column data lines broken only at delimiter boundaries.
        let mut pd_lines: Vec<String> = Vec::new();
        // For each entity: (pd_start_seq, pd_line_count).
        let mut pd_extent: Vec<(i64, i64)> = Vec::with_capacity(self.entities.len());
        for (index, entity) in self.entities.iter().enumerate() {
            let de_pointer = (2 * index + 1) as i64;
            let mut fields = Vec::with_capacity(entity.params.len() + 1);
            fields.push(entity.entity_type.to_string());
            fields.extend(entity.params.iter().cloned());
            let data_lines = wrap_free_format(&fields, 64);
            let start_seq = pd_lines.len() as i64 + 1;
            for data in &data_lines {
                let seq = pd_lines.len() as i64 + 1;
                pd_lines.push(format!(
                    "{:<64}{:>8}P{:>7}",
                    data, de_pointer, seq
                ));
            }
            pd_extent.push((start_seq, data_lines.len() as i64));
        }

        // ---- Directory Entry layout -----------------------------------------
        let mut de_lines: Vec<String> = Vec::with_capacity(self.entities.len() * 2);
        for (index, entity) in self.entities.iter().enumerate() {
            let seq1 = (2 * index + 1) as i64;
            let seq2 = seq1 + 1;
            let (pd_start, pd_count) = pd_extent[index];
            // Line 1: fields 1-9 (cols 1-72) then 'D' + seq.
            let line1 = format!(
                "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{}D{:>7}",
                entity.entity_type, // 1 entity type
                pd_start,           // 2 parameter data pointer
                0,                  // 3 structure
                0,                  // 4 line font pattern
                0,                  // 5 level
                0,                  // 6 view
                0,                  // 7 transformation matrix
                0,                  // 8 label display associativity
                "00000000",         // 9 status number
                seq1,
            );
            // Line 2: fields 11-19 (cols 1-72) then 'D' + seq.
            let label = truncate_right(&entity.label, 8);
            let line2 = format!(
                "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}D{:>7}",
                entity.entity_type, // 11 entity type
                0,                  // 12 line weight number
                0,                  // 13 color number
                pd_count,           // 14 parameter line count
                entity.form,        // 15 form number
                "",                 // 16 reserved
                "",                 // 17 reserved
                label,              // 18 entity label
                0,                  // 19 entity subscript
                seq2,
            );
            de_lines.push(line1);
            de_lines.push(line2);
        }

        // ---- Global section -------------------------------------------------
        let global_fields = build_global_fields(&global);
        let global_data_lines = wrap_free_format(&global_fields, 72);
        let mut global_lines: Vec<String> = Vec::with_capacity(global_data_lines.len());
        for (i, data) in global_data_lines.iter().enumerate() {
            global_lines.push(format!("{:<72}G{:>7}", data, i as i64 + 1));
        }

        // ---- Start section --------------------------------------------------
        let start_source: Vec<String> = if self.start_lines.is_empty() {
            vec!["Generated by the BREP kernel IGES exporter.".to_string()]
        } else {
            self.start_lines.clone()
        };
        let mut start_lines: Vec<String> = Vec::with_capacity(start_source.len());
        for (i, text) in start_source.iter().enumerate() {
            start_lines.push(format!("{:<72}S{:>7}", truncate_right_pad(text, 72), i as i64 + 1));
        }

        // ---- Terminate ------------------------------------------------------
        let terminate = format!(
            "{:<72}T{:>7}",
            format!(
                "S{:>7}G{:>7}D{:>7}P{:>7}",
                start_lines.len(),
                global_lines.len(),
                de_lines.len(),
                pd_lines.len()
            ),
            1
        );

        // ---- Assemble -------------------------------------------------------
        let mut out = String::new();
        for line in start_lines
            .iter()
            .chain(global_lines.iter())
            .chain(de_lines.iter())
            .chain(pd_lines.iter())
        {
            out.push_str(line);
            out.push('\n');
        }
        out.push_str(&terminate);
        out.push('\n');
        out
    }
}

/// The 24 standard global parameters, in order, as free-format fields.
fn build_global_fields(g: &GlobalParams) -> Vec<String> {
    vec![
        hollerith(","),                 // 1 parameter delimiter
        hollerith(";"),                 // 2 record delimiter
        hollerith(&g.product_id),       // 3 product id (sender)
        hollerith(&g.file_name),        // 4 file name
        hollerith("BREP-kernel-rs"),    // 5 native system id
        hollerith("BREP-IGES-1.0"),     // 6 preprocessor version
        "32".to_string(),               // 7 integer bits
        "38".to_string(),               // 8 single-precision magnitude
        "6".to_string(),                // 9 single-precision significance
        "308".to_string(),              // 10 double-precision magnitude
        "15".to_string(),               // 11 double-precision significance
        hollerith(&g.product_id),       // 12 product id (receiver)
        real_field(1.0),                // 13 model space scale
        g.units_flag.to_string(),       // 14 units flag
        hollerith(&g.units_name),       // 15 units name
        "1".to_string(),                // 16 max line-weight gradations
        real_field(0.0),                // 17 max line weight
        hollerith(&g.timestamp),        // 18 file generation timestamp
        real_field(g.min_resolution),   // 19 minimum resolution
        real_field(0.0),                // 20 max coordinate value
        hollerith("BREP kernel"),       // 21 author
        hollerith("Autodrop3d"),        // 22 organization
        "11".to_string(),               // 23 IGES version flag (5.3)
        "0".to_string(),                // 24 drafting standard flag
    ]
}

/// Encode a string as an IGES Hollerith constant `nHxxxx`.
pub(crate) fn hollerith(text: &str) -> String {
    format!("{}H{}", text.chars().count(), text)
}

/// Format a real value with shortest round-trip precision, guaranteeing IGES
/// real syntax (a decimal point or exponent is always present).
pub(crate) fn real_field(value: f64) -> String {
    let mut s = format!("{}", value);
    if !s.contains('.') && !s.contains('e') && !s.contains('E') {
        s.push('.');
    }
    s
}

fn truncate_right(text: &str, width: usize) -> String {
    let chars: Vec<char> = text.chars().collect();
    if chars.len() > width {
        chars[chars.len() - width..].iter().collect()
    } else {
        text.to_string()
    }
}

fn truncate_right_pad(text: &str, width: usize) -> String {
    let chars: Vec<char> = text.chars().collect();
    if chars.len() > width {
        chars[..width].iter().collect()
    } else {
        text.to_string()
    }
}

/// Pack free-format fields into data lines no wider than `width`, breaking only
/// at delimiter (comma) boundaries. Every field except the last is followed by
/// a comma; the last is followed by the record terminator `;`.
fn wrap_free_format(fields: &[String], width: usize) -> Vec<String> {
    let mut tokens: Vec<String> = Vec::with_capacity(fields.len());
    let count = fields.len();
    for (i, field) in fields.iter().enumerate() {
        let sep = if i + 1 == count { ';' } else { ',' };
        tokens.push(format!("{}{}", field, sep));
    }
    let mut lines: Vec<String> = Vec::new();
    let mut current = String::new();
    for token in tokens {
        if !current.is_empty() && current.len() + token.len() > width {
            lines.push(std::mem::take(&mut current));
        }
        // A single token longer than the width still occupies its own line;
        // IGES readers tolerate it (numbers/Hollerith here never exceed 64).
        current.push_str(&token);
    }
    if !current.is_empty() {
        lines.push(current);
    }
    if lines.is_empty() {
        lines.push(String::new());
    }
    lines
}
