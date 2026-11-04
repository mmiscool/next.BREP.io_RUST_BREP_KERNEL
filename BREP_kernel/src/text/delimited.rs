//! Delimited text (CSV, and the tab-separated text a spreadsheet puts on the
//! clipboard): ONE reader, so a spreadsheet export pasted into the CAD app and
//! the same file imported into the PLM read the same cells.
//!
//! The grammar is RFC 4180's, read generously where that costs nothing:
//!
//! * a record ends at `\n`, `\r\n` or a lone `\r`; a trailing line break does
//!   not make an empty last record;
//! * a field is quoted when its first character other than spaces is `"`
//!   (the spaces are dropped); inside quotes `""` is a quote, and separators
//!   and line breaks are text;
//! * a `"` anywhere else in an unquoted field is an ordinary character
//!   (`5" bolt`);
//! * a blank line is a record of one empty field. Callers that skip blank
//!   lines test [`Record::is_blank`].
//!
//! [`Strictness`] decides only what malformed quoting does. A [`Strictness::Lenient`]
//! read (a paste: show what the user copied, never refuse it) closes an
//! unterminated quote at the end of the text and keeps text after a closing
//! quote; a [`Strictness::Strict`] read (an import that writes data) refuses
//! both, naming the line.
//!
//! A byte-order mark is not part of the grammar: [`strip_bom`] removes one, and
//! every caller that reads a file or a spreadsheet's clipboard calls it.

/// What malformed quoting does (see the module documentation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Strictness {
    /// Never refuses: an unterminated quote ends at the end of the text, and
    /// text after a closing quote is appended to the field.
    Lenient,
    /// Refuses an unterminated quote and text (other than spaces) after a
    /// closing quote.
    Strict,
}

/// One record, with the 1-based line it starts on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    pub line: usize,
    pub fields: Vec<String>,
}

impl Record {
    /// Whether every field is empty or only whitespace (a blank line, or a
    /// line of bare separators).
    pub fn is_blank(&self) -> bool {
        self.fields.iter().all(|field| field.trim().is_empty())
    }
}

/// `text` without a leading UTF-8 byte-order mark.
pub fn strip_bom(text: &str) -> &str {
    text.strip_prefix('\u{feff}').unwrap_or(text)
}

/// The separator a CSV-or-TSV text uses: a tab when its first non-blank line
/// holds a tab and no comma (a paste from a spreadsheet), else a comma.
pub fn sniff_separator(text: &str) -> char {
    let first_line = text.lines().find(|line| !line.trim().is_empty()).unwrap_or("");
    if first_line.contains('\t') && !first_line.contains(',') {
        '\t'
    } else {
        ','
    }
}

/// The records of `text`, fields split on `separator`. A
/// [`Strictness::Lenient`] read never returns `Err`.
pub fn parse(text: &str, separator: char, strictness: Strictness) -> Result<Vec<Record>, String> {
    let strict = strictness == Strictness::Strict;
    let mut records = Vec::new();
    let mut fields = Vec::new();
    let mut field = String::new();
    // Inside an open quote.
    let mut quoted = false;
    // The current field had a quoted part that has closed.
    let mut closed = false;
    let mut line = 1;
    let mut record_line = 1;
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if quoted {
            match c {
                '"' if chars.peek() == Some(&'"') => {
                    chars.next();
                    field.push('"');
                }
                '"' => {
                    quoted = false;
                    closed = true;
                }
                '\n' => {
                    line += 1;
                    field.push('\n');
                }
                _ => field.push(c),
            }
            continue;
        }
        match c {
            '"' if !closed && field.trim().is_empty() => {
                field.clear();
                quoted = true;
            }
            c if c == separator => {
                fields.push(std::mem::take(&mut field));
                closed = false;
            }
            '\r' | '\n' => {
                if c == '\r' && chars.peek() == Some(&'\n') {
                    chars.next();
                }
                fields.push(std::mem::take(&mut field));
                records.push(Record { line: record_line, fields: std::mem::take(&mut fields) });
                closed = false;
                line += 1;
                record_line = line;
            }
            c if closed && strict => {
                if !c.is_whitespace() {
                    return Err(format!("line {line}: text after a closing quote"));
                }
            }
            _ => field.push(c),
        }
    }
    if quoted && strict {
        return Err(format!("line {record_line}: a quoted field is never closed"));
    }
    if !field.is_empty() || !fields.is_empty() || closed || quoted {
        fields.push(field);
        records.push(Record { line: record_line, fields });
    }
    Ok(records)
}

