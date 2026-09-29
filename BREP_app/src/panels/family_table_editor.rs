//! The **Family table** pane: the spreadsheet a family seed (`.fbrep`) carries.
//! It has one row per member and these columns: the member's PART NUMBER, its
//! REVISION, a description, then one column per model expression the family
//! drives.
//!
//! # What it owns
//!
//! Nothing of the model. The table is the document's `familyTable` block, read
//! and written ONLY through [`crate::family_table::read_table`] /
//! [`crate::family_table::write_table`]. Each committed edit (a cell, a row
//! operation, a paste) is ONE undoable document mutation, so Ctrl+Z, redo and
//! the tab's unsaved dot treat it like any other model edit. The pane holds
//! only transient UI state: the selected cell, the cell being typed into, the
//! column being relabelled, and the last Generate's report.
//!
//! # How it behaves
//!
//! It works like a spreadsheet. A click selects a cell. Arrows, Tab and
//! Shift+Tab move the selection. Enter, F2 or a double click edits the cell, and
//! typing starts an edit that replaces the cell's text. Enter, Tab and the
//! arrows commit and move on; Escape cancels. Delete clears the cell. Ctrl+V
//! pastes a tab-separated block, which is what a spreadsheet puts on the
//! clipboard. The block lands at the selected cell and adds rows as needed. A
//! block whose first line is a header row (Part number, Revision, Description,
//! expression names) is instead mapped by header, and it adds any model
//! expression it names as a column.
//!
//! # What it checks, as you type
//!
//! Each problem is drawn in the cell it belongs to, with the reason on hover:
//! an empty or duplicate part number, a part number that cannot be a file name,
//! a value that does not evaluate, and a column whose expression the model no
//! longer defines. A value is evaluated the way Generate bakes it: the row's
//! text replaces that expression's definition in the model's expressions
//! source, and the whole source is evaluated.
//!
//! # Generate
//!
//! The button calls [`crate::family_table::generate_family`] with the document as
//! it stands. Generate is Lane A's. This pane only shows the returned
//! [`GenerateReport`]: a status column per row (written / skipped / failed,
//! with the reason on hover) and the failures listed in full below the table.

use std::collections::{BTreeMap, HashMap};

use eframe::egui;
use serde_json::{json, Value};

use crate::automation::hit_keys::HitKeyDoc;
use crate::family_table::{self, FamilyColumn, FamilyRow, FamilyTable, GenerateReport};
use crate::store::ModelStore;
use brep_render::brep_kernel::Env;
use brep_render::engine_state::{EngineState, ExpressionPreview};

/// The three columns every family has, left of the expression columns.
pub const FIXED_COLUMNS: usize = 3;
const FIXED_TITLES: [&str; FIXED_COLUMNS] = ["Part number", "Revision", "Description"];

// ============================================================================
// The table model: pure edits on a `FamilyTable`, tested without a UI.
// ============================================================================

/// A cell address: `row` into `rows`, `col` into the DISPLAYED columns
/// (0 part number, 1 revision, 2 description, `FIXED_COLUMNS + i` the i-th
/// expression column).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Cell {
    pub row: usize,
    pub col: usize,
}

/// How many displayed columns `table` has.
pub fn column_count(table: &FamilyTable) -> usize {
    FIXED_COLUMNS + table.columns.len()
}

/// The text of one cell (empty for an address past the table).
pub fn cell_text(table: &FamilyTable, cell: Cell) -> String {
    let Some(row) = table.rows.get(cell.row) else {
        return String::new();
    };
    match cell.col {
        0 => row.part_number.clone(),
        1 => row.revision.clone(),
        2 => row.description.clone(),
        c => table
            .columns
            .get(c - FIXED_COLUMNS)
            .and_then(|column| row.values.get(&column.name))
            .cloned()
            .unwrap_or_default(),
    }
}

/// Write one cell. An expression cell set to blank text is REMOVED from the
/// row's values: a row that sets nothing for an expression leaves the model's
/// own definition in force. Returns whether anything changed.
pub fn set_cell(table: &mut FamilyTable, cell: Cell, text: &str) -> bool {
    let name = cell
        .col
        .checked_sub(FIXED_COLUMNS)
        .and_then(|i| table.columns.get(i))
        .map(|column| column.name.clone());
    let Some(row) = table.rows.get_mut(cell.row) else {
        return false;
    };
    let slot = match cell.col {
        0 => &mut row.part_number,
        1 => &mut row.revision,
        2 => &mut row.description,
        _ => {
            let Some(name) = name else { return false };
            let text = text.trim();
            if text.is_empty() {
                return row.values.remove(&name).is_some();
            }
            if row.values.get(&name).map(String::as_str) == Some(text) {
                return false;
            }
            row.values.insert(name, text.to_string());
            return true;
        }
    };
    // Part numbers are file names, so the edges are trimmed; free text keeps
    // what was typed, bar the edges.
    let text = text.trim();
    if slot == text {
        return false;
    }
    *slot = text.to_string();
    true
}

/// A new part number derived from `from` that no row uses: `M3x8` becomes
/// `M3x8-copy`, then `M3x8-copy2`, …. Empty stays empty (the row then
/// shows "part number is empty", which is the prompt to type one).
pub fn fresh_part_number(table: &FamilyTable, from: &str) -> String {
    if from.trim().is_empty() {
        return String::new();
    }
    let taken = |candidate: &str| {
        table
            .rows
            .iter()
            .any(|row| row.part_number.trim().eq_ignore_ascii_case(candidate))
    };
    let base = format!("{}-copy", from.trim());
    if !taken(&base) {
        return base;
    }
    (2..).map(|n| format!("{base}{n}")).find(|c| !taken(c)).unwrap_or(base)
}

/// Insert a blank row at `at` (clamped), returning its index.
pub fn insert_row(table: &mut FamilyTable, at: usize) -> usize {
    let at = at.min(table.rows.len());
    table.rows.insert(at, FamilyRow::default());
    at
}

/// Copy row `index` to just below itself with a fresh part number, returning
/// the copy's index.
pub fn duplicate_row(table: &mut FamilyTable, index: usize) -> Option<usize> {
    let mut copy = table.rows.get(index)?.clone();
    copy.part_number = fresh_part_number(table, &copy.part_number);
    table.rows.insert(index + 1, copy);
    Some(index + 1)
}

/// Move row `from` one step up (`-1`) or down (`+1`), returning its new index.
pub fn move_row(table: &mut FamilyTable, from: usize, step: isize) -> Option<usize> {
    let to = from.checked_add_signed(step)?;
    if from >= table.rows.len() || to >= table.rows.len() {
        return None;
    }
    table.rows.swap(from, to);
    Some(to)
}

/// Add a column driving expression `name`. A name already a column is left
/// alone. Returns the column's index among the expression columns.
pub fn add_column(table: &mut FamilyTable, name: &str) -> usize {
    if let Some(i) = table.columns.iter().position(|c| c.name == name) {
        return i;
    }
    table.columns.push(FamilyColumn { name: name.to_string(), label: String::new() });
    table.columns.len() - 1
}

/// Remove expression column `index`, and its values from every row. A value
/// left behind for a column that is gone would still be baked by Generate
/// with nothing in the table to show it.
pub fn remove_column(table: &mut FamilyTable, index: usize) -> bool {
    if index >= table.columns.len() {
        return false;
    }
    let column = table.columns.remove(index);
    for row in &mut table.rows {
        row.values.remove(&column.name);
    }
    true
}

/// Move expression column `index` one step left or right.
pub fn move_column(table: &mut FamilyTable, index: usize, step: isize) -> Option<usize> {
    let to = index.checked_add_signed(step)?;
    if index >= table.columns.len() || to >= table.columns.len() {
        return None;
    }
    table.columns.swap(index, to);
    Some(to)
}

/// Relabel expression column `index`. A label equal to the expression name is
/// stored empty, which is what "show the name" means in the contract.
pub fn relabel_column(table: &mut FamilyTable, index: usize, label: &str) -> bool {
    let Some(column) = table.columns.get_mut(index) else {
        return false;
    };
    let label = label.trim();
    let label = if label == column.name { "" } else { label };
    if column.label == label {
        return false;
    }
    column.label = label.to_string();
    true
}

/// A column's header text: its label, or its expression name.
pub fn column_title(column: &FamilyColumn) -> &str {
    if column.label.trim().is_empty() {
        &column.name
    } else {
        &column.label
    }
}

// --- Tab-separated paste ----------------------------------------------------

/// Split clipboard text into rows of cells the way spreadsheets write it:
/// tab between cells, a newline (`\n` or `\r\n`) between rows, and a cell
/// holding a tab, a newline or a quote wrapped in double quotes with inner
/// quotes doubled. A trailing newline does not make an empty last row.
pub fn parse_tsv(text: &str) -> Vec<Vec<String>> {
    let mut rows = Vec::new();
    let mut row = Vec::new();
    let mut cell = String::new();
    let mut chars = text.chars().peekable();
    let mut at_cell_start = true;
    while let Some(ch) = chars.next() {
        match ch {
            '"' if at_cell_start => {
                // A quoted cell: read to the closing quote, `""` is a quote.
                loop {
                    match chars.next() {
                        Some('"') if chars.peek() == Some(&'"') => {
                            chars.next();
                            cell.push('"');
                        }
                        Some('"') | None => break,
                        Some(c) => cell.push(c),
                    }
                }
                at_cell_start = false;
            }
            '\t' => {
                row.push(std::mem::take(&mut cell));
                at_cell_start = true;
            }
            '\r' if chars.peek() == Some(&'\n') => {}
            '\n' | '\r' => {
                row.push(std::mem::take(&mut cell));
                rows.push(std::mem::take(&mut row));
                at_cell_start = true;
            }
            c => {
                cell.push(c);
                at_cell_start = false;
            }
        }
    }
    if !cell.is_empty() || !row.is_empty() {
        row.push(cell);
        rows.push(row);
    }
    rows
}

/// Where a pasted column lands.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Target {
    Fixed(usize),
    Expression(String),
}

/// The target of a header cell, or `None` when it is not a header this table
/// knows: a fixed title (or a common short form), an expression column's label
/// or name, or the name of any expression the model defines.
fn header_target(table: &FamilyTable, expressions: &[String], text: &str) -> Option<Target> {
    let key = text.trim().to_ascii_lowercase();
    let compact: String = key.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
    match compact.as_str() {
        "partnumber" | "partno" | "pn" | "part" | "member" => return Some(Target::Fixed(0)),
        "revision" | "rev" => return Some(Target::Fixed(1)),
        "description" | "desc" => return Some(Target::Fixed(2)),
        _ => {}
    }
    if let Some(column) = table
        .columns
        .iter()
        .find(|c| c.name == text.trim() || (!c.label.is_empty() && c.label.trim().eq_ignore_ascii_case(&key)))
    {
        return Some(Target::Expression(column.name.clone()));
    }
    expressions
        .iter()
        .find(|name| name.as_str() == text.trim())
        .map(|name| Target::Expression(name.clone()))
}

/// What a paste did, for the status line and the automation blob.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PasteOutcome {
    /// Cells whose text changed.
    pub cells: usize,
    /// Rows appended to hold the block.
    pub rows_added: usize,
    /// Expression columns the header row added.
    pub columns_added: Vec<String>,
    /// Whether the first line was read as a header row.
    pub header: bool,
    /// Cells that fell right of the last column and were dropped.
    pub dropped: usize,
}

impl PasteOutcome {
    pub fn summary(&self) -> String {
        let mut text = format!(
            "pasted {} cell{}",
            self.cells,
            if self.cells == 1 { "" } else { "s" }
        );
        if self.header {
            text.push_str(", mapped by the header row");
        }
        if self.rows_added > 0 {
            text.push_str(&format!(", added {} row{}", self.rows_added, if self.rows_added == 1 { "" } else { "s" }));
        }
        if !self.columns_added.is_empty() {
            text.push_str(&format!(", added column{} {}", if self.columns_added.len() == 1 { "" } else { "s" }, self.columns_added.join(", ")));
        }
        if self.dropped > 0 {
            text.push_str(&format!("; {} cell{} right of the last column dropped", self.dropped, if self.dropped == 1 { "" } else { "s" }));
        }
        text
    }
}

/// Paste `block` (parsed clipboard rows) at `at`. `expressions` is the model's
/// expression names, which a header row may name as new columns.
///
/// * A header row: every cell of the first line is a header this table knows
///   ([`header_target`]) and at least one names a part number or an
///   expression. The remaining lines are written column by column under their
///   headers, starting at `at.row`, and a model expression that is not yet a
///   column becomes one.
/// * Otherwise the block is laid out from `at` as it stands: cell `(i, j)`
///   lands at `(at.row + i, at.col + j)`.
///
/// Either way the block appends rows past the end, and drops cells right of
/// the last column (counted).
pub fn paste_block(table: &mut FamilyTable, at: Cell, block: &[Vec<String>], expressions: &[String]) -> PasteOutcome {
    let mut outcome = PasteOutcome::default();
    if block.is_empty() {
        return outcome;
    }
    let header: Option<Vec<Target>> = block[0]
        .iter()
        .map(|text| header_target(table, expressions, text))
        .collect::<Option<Vec<_>>>()
        .filter(|targets| {
            targets
                .iter()
                .any(|t| matches!(t, Target::Fixed(0) | Target::Expression(_)))
        });
    let (targets, body): (Vec<Option<usize>>, &[Vec<String>]) = match header {
        Some(targets) => {
            outcome.header = true;
            let mut columns = Vec::new();
            for target in targets {
                columns.push(Some(match target {
                    Target::Fixed(c) => c,
                    Target::Expression(name) => {
                        if !table.columns.iter().any(|c| c.name == name) {
                            outcome.columns_added.push(name.clone());
                        }
                        FIXED_COLUMNS + add_column(table, &name)
                    }
                }));
            }
            (columns, &block[1..])
        }
        None => {
            let width = column_count(table);
            let columns = (0..block.iter().map(Vec::len).max().unwrap_or(0))
                .map(|j| Some(at.col + j).filter(|c| *c < width))
                .collect();
            (columns, block)
        }
    };
    for (i, line) in body.iter().enumerate() {
        let row = at.row + i;
        while table.rows.len() <= row {
            table.rows.push(FamilyRow::default());
            outcome.rows_added += 1;
        }
        for (j, text) in line.iter().enumerate() {
            match targets.get(j).copied().flatten() {
                Some(col) => {
                    if set_cell(table, Cell { row, col }, text) {
                        outcome.cells += 1;
                    }
                }
                None if !text.trim().is_empty() => outcome.dropped += 1,
                None => {}
            }
        }
    }
    outcome
}

/// The whole table as tab-separated text with a header row — what "Copy
/// table" puts on the clipboard, and what a header-row paste reads back.
pub fn to_tsv(table: &FamilyTable) -> String {
    let quote = |text: &str| {
        if text.contains(['\t', '\n', '\r', '"']) {
            format!("\"{}\"", text.replace('"', "\"\""))
        } else {
            text.to_string()
        }
    };
    let mut out = String::new();
    let mut header: Vec<String> = FIXED_TITLES.iter().map(|t| t.to_string()).collect();
    header.extend(table.columns.iter().map(|c| c.name.clone()));
    out.push_str(&header.join("\t"));
    out.push('\n');
    for r in 0..table.rows.len() {
        let line: Vec<String> = (0..column_count(table))
            .map(|col| quote(&cell_text(table, Cell { row: r, col })))
            .collect();
        out.push_str(&line.join("\t"));
        out.push('\n');
    }
    out
}

// --- Validation -------------------------------------------------------------

/// Why `part_number` cannot name the member's file, or `None` when it can.
///
/// Generate's own rule first ([`family_table::part_number_problem`]: empty,
/// path separators, the characters Windows forbids, control characters, a
/// family or template extension). Then one check that rule does not make: a
/// Windows device name (`CON`, `NUL`, `COM1`, …) is refused as a file name
/// there even with an extension, so `CON.nbrep` cannot be written on Windows.
pub fn part_number_problem(part_number: &str) -> Option<String> {
    if let Some(problem) = family_table::part_number_problem(part_number) {
        return Some(problem);
    }
    let pn = part_number.trim();
    let stem = pn.split('.').next().unwrap_or(pn).trim_end().to_ascii_uppercase();
    let device = matches!(stem.as_str(), "CON" | "PRN" | "AUX" | "NUL")
        || ((stem.starts_with("COM") || stem.starts_with("LPT"))
            && stem.len() == 4
            && matches!(stem.as_bytes()[3], b'1'..=b'9'));
    device.then(|| format!("'{stem}' is a reserved file name on Windows"))
}

/// Every problem the table has, by where it is drawn.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Check {
    /// Per cell: the reason it is wrong.
    pub cells: BTreeMap<Cell, String>,
    /// Per expression column (index among the expression columns): the reason
    /// the whole column is wrong.
    pub columns: BTreeMap<usize, String>,
    /// Per expression cell that evaluated: its value, shown on hover.
    pub values: BTreeMap<Cell, f64>,
}

impl Check {
    pub fn problem_count(&self) -> usize {
        self.cells.len() + self.columns.len()
    }
}

/// Check the whole table against the model: `expressions` is the model's
/// expressions source, `names` the expressions it defines (in source order),
/// `configurator` its configurator object.
pub fn check(table: &FamilyTable, expressions: &str, names: &[String], configurator: &Value) -> Check {
    let mut out = Check::default();

    // Part numbers: each on its own, then duplicates. File names compare
    // without case: two members `A1` and `a1` are one file on Windows and macOS.
    let mut seen: HashMap<String, usize> = HashMap::new();
    for (r, row) in table.rows.iter().enumerate() {
        let cell = Cell { row: r, col: 0 };
        if let Some(problem) = part_number_problem(&row.part_number) {
            out.cells.insert(cell, problem);
            continue;
        }
        let key = row.part_number.trim().to_lowercase();
        if let Some(first) = seen.get(&key) {
            out.cells.insert(cell, format!("duplicate part number (row {} has it too)", first + 1));
            let first_cell = Cell { row: *first, col: 0 };
            out.cells
                .entry(first_cell)
                .or_insert_with(|| format!("duplicate part number (row {} has it too)", r + 1));
        } else {
            seen.insert(key, r);
        }
    }

    // Columns the model no longer defines.
    let live: Vec<(usize, &FamilyColumn)> = table
        .columns
        .iter()
        .enumerate()
        .filter(|(i, column)| {
            let defined = names.iter().any(|n| n == &column.name);
            if !defined {
                out.columns.insert(
                    *i,
                    format!("the model has no expression `{}`, so Generate cannot drive it", column.name),
                );
            } else if table.columns[..*i].iter().any(|c| c.name == column.name) {
                out.columns.insert(*i, format!("`{}` is already a column", column.name));
                return false;
            }
            defined
        })
        .collect();

    // Values: evaluate each row's member source.
    for (r, row) in table.rows.iter().enumerate() {
        // Exactly what Generate bakes: the row's non-empty values of current
        // columns, spliced into the model's source.
        let effective = family_table::effective_row(table, row);
        let set = &effective.values;
        let cell_of = |name: &str| {
            live.iter()
                .find(|(_, c)| c.name == name)
                .map(|(i, _)| Cell { row: r, col: FIXED_COLUMNS + i })
        };
        match Env::build(&family_table::member_expressions(expressions, &effective), configurator) {
            Ok(env) => {
                for name in set.keys() {
                    if let (Some(cell), Some(value)) = (cell_of(name), env.get(name)) {
                        if value.is_finite() {
                            out.values.insert(cell, value);
                        } else {
                            out.cells.insert(cell, format!("evaluates to {value}"));
                        }
                    }
                }
            }
            Err(whole) => {
                // Find the cell to blame: the one that fails on its own
                // against the model's other definitions.
                let mut blamed = false;
                for (name, text) in set {
                    let alone = FamilyRow { values: BTreeMap::from([(name.clone(), text.clone())]), ..FamilyRow::default() };
                    if let Err(error) = Env::build(&family_table::member_expressions(expressions, &alone), configurator) {
                        if let Some(cell) = cell_of(name) {
                            out.cells.insert(cell, format!("does not evaluate: {error}"));
                            blamed = true;
                        }
                    }
                }
                if !blamed {
                    // Every value is fine alone and they fail together (or
                    // the model's own source fails): mark the row's values.
                    for name in set.keys() {
                        if let Some(cell) = cell_of(name) {
                            out.cells.insert(cell, format!("with this row's values: {whole}"));
                        }
                    }
                }
            }
        }
    }
    out
}

// ============================================================================
// The pane.
// ============================================================================

/// The cell being typed into.
#[derive(Debug, Clone)]
struct Edit {
    cell: Cell,
    text: String,
    /// The first frame of the edit: the field takes the keyboard and puts the
    /// cursor at the end.
    fresh: bool,
}

/// One row's line of the last Generate report, keyed by part number so it
/// still finds its row after rows move.
#[derive(Debug, Clone, PartialEq)]
struct RowReport {
    outcome: &'static str,
    reason: String,
}

#[derive(Debug, Clone, Default)]
struct ReportView {
    rows: Vec<(String, RowReport)>,
    written: usize,
    skipped: usize,
    failed: usize,
}

impl ReportView {
    fn of(report: &GenerateReport) -> Self {
        let mut view = ReportView::default();
        for outcome in &report.rows {
            let status = outcome.status();
            match status {
                "written" => view.written += 1,
                "skipped" => view.skipped += 1,
                _ => view.failed += 1,
            }
            view.rows.push((
                outcome.part_number().to_string(),
                RowReport { outcome: status, reason: outcome.reason().unwrap_or("").to_string() },
            ));
        }
        view
    }

    fn for_part(&self, part_number: &str) -> Option<&RowReport> {
        self.rows
            .iter()
            .find(|(pn, _)| pn.trim().eq_ignore_ascii_case(part_number.trim()))
            .map(|(_, line)| line)
    }
}

/// What this frame's table read came from: the history revision, plus the
/// derived check.
struct Cache {
    revision: u64,
    table: FamilyTable,
    names: Vec<String>,
    definitions: Vec<(String, String)>,
    check: Check,
}

/// The family-row PREVIEW: the displayed model rebuilt with one row's values,
/// computed exactly as Generate bakes that row's member
/// (`member_expressions(src, &effective_row(table, row))`). It lives in the
/// engine as a transient expressions override
/// ([`EngineState::set_expression_preview`]), never in the document.
#[derive(Debug, Clone)]
struct RowPreview {
    /// The row previewed. While a preview is on it follows the selected row.
    row: usize,
    /// The row's part number, or `row <n>` when it has none.
    label: String,
    /// Why this row cannot be shown (its values do not evaluate). The engine
    /// keeps what it showed before.
    problem: Option<String>,
    /// The history's edit serial after this pane's own last write. Any other
    /// move is the user editing the model, which ends the preview.
    serial: u64,
    /// The history revision the row was last evaluated against.
    revision: u64,
    /// What the display shows, as the engine reports it: a row's label, or
    /// `None` for the family's own values. Refreshed every frame.
    showing: Option<String>,
    /// The feature errors the shown build reports, keyed on the applied run.
    feature_errors: (u64, usize),
}

/// The pane's transient UI state (the table lives in the document).
pub struct FamilyTableEditor {
    selected: Option<Cell>,
    edit: Option<Edit>,
    /// The expression column whose label is being edited, and the text.
    relabel: Option<(usize, String, bool)>,
    report: Option<ReportView>,
    generate_calls: u64,
    status: String,
    cache: Option<Cache>,
    /// Which rows' members exist in the store, by row: `Some(true)` exists,
    /// `Some(false)` does not, `None` has no file name to look for (a family
    /// never saved, or a part number that cannot be one). Keyed on the store's
    /// mutation count, the history revision and the family's identity, so the
    /// store is read once per change, not per frame.
    members: (Option<(u64, u64, Option<String>)>, Vec<Option<bool>>),
    hits: HashMap<String, egui::Rect>,
    preview: Option<RowPreview>,
    /// A row's preview toggle was clicked this frame.
    preview_toggle: Option<usize>,
    /// The engine's executed-run count, for automation (`runsReplied`).
    runs_replied: u64,
    /// The pane's PLM half (S9): Generate on the server, the member part
    /// type, the category's keys. Inert on a file store.
    plm: crate::panels::plm_family::PlmFamily,
    /// Whether the family has unsaved edits, as the shell said this frame.
    dirty: bool,
}

impl Default for FamilyTableEditor {
    fn default() -> Self {
        Self::new()
    }
}

/// Committed row and column operations, collected while drawing and applied
/// once after, so a frame commits at most one undo step.
enum Change {
    Table(FamilyTable, String),
}

impl FamilyTableEditor {
    pub fn new() -> Self {
        Self {
            selected: None,
            edit: None,
            relabel: None,
            report: None,
            generate_calls: 0,
            status: String::new(),
            cache: None,
            members: (None, Vec::new()),
            hits: HashMap::new(),
            preview: None,
            preview_toggle: None,
            runs_replied: 0,
            plm: Default::default(),
            dirty: false,
        }
    }

    /// Whether the shell should save the family now: the PLM pane's Save and
    /// generate was pressed. Asked once per frame by the shell, after [`Self::show`].
    pub fn take_save_request(&mut self) -> bool {
        self.plm.take_save_request()
    }

    /// The shell's save for Save and generate failed.
    pub fn save_failed(&mut self, error: String) {
        self.plm.save_failed(error);
    }

    /// The row being previewed, if any.
    pub fn previewing(&self) -> Option<usize> {
        self.preview.as_ref().map(|p| p.row)
    }

    /// Preview row `r` (or move the preview to it): evaluate its values the
    /// way Generate does and, when they evaluate, hand the member's
    /// expressions source to the engine. A row that does not evaluate keeps
    /// the display as it was and says why.
    fn preview_row(&mut self, state: &mut EngineState, table: &FamilyTable, r: usize) {
        let Some(raw) = table.rows.get(r) else {
            return self.end_preview(state, None);
        };
        let row = family_table::effective_row(table, raw);
        let source = state.history.expressions();
        let family = json!({ "expressions": source, "configurator": state.history.configurator() });
        let label = match raw.part_number.trim() {
            "" => format!("row {}", r + 1),
            pn => pn.to_string(),
        };
        let problem = family_table::row_evaluation_problem(&family, &row);
        if problem.is_none() {
            let expressions = family_table::member_expressions(&source, &row);
            state.set_expression_preview(Some(ExpressionPreview::new(expressions, label.clone())));
        } else {
            // This pane may have just written the table: the preview the
            // display keeps is still this pane's.
            state.rearm_expression_preview();
        }
        let showing = state.expression_preview().map(|p| p.label.clone());
        // The card says what is previewed; an older "Preview ended" would contradict it.
        if self.status.starts_with("Preview ended") {
            self.status.clear();
        }
        self.preview = Some(RowPreview {
            row: r,
            label,
            problem,
            serial: state.history.edit_serial(),
            revision: state.history.revision(),
            showing,
            feature_errors: (u64::MAX, 0),
        });
    }

    /// End the preview and show the family's own values again.
    fn end_preview(&mut self, state: &mut EngineState, why: Option<&str>) {
        if self.preview.take().is_some() {
            state.set_expression_preview(None);
            if let Some(why) = why {
                self.status = why.to_string();
            }
        }
    }

    /// Escape, from the app's key router: end a preview if one is on.
    /// Returns whether it took the key.
    pub fn escape_preview(&mut self, state: &mut EngineState) -> bool {
        if self.preview.is_none() {
            return false;
        }
        self.end_preview(state, Some("Preview ended"));
        true
    }

    /// Notice the user editing the model under the preview (a feature, an
    /// expression, an undo): the preview ends, and the edit applies to the
    /// family's own values. The app calls this every frame, drawn or not.
    pub fn sync_preview(&mut self, state: &mut EngineState) {
        self.runs_replied = state.runs_replied();
        let Some(preview) = self.preview.as_mut() else {
            return;
        };
        if preview.serial != state.history.edit_serial() {
            self.preview = None;
            state.set_expression_preview(None);
            self.status = "Preview ended: the model was edited, and the edit applies to the family's own values".into();
            return;
        }
        preview.showing = state.expression_preview().map(|p| p.label.clone());
        let generation = state.applied_generation();
        if preview.feature_errors.0 != generation {
            let report: Value = serde_json::from_str(&state.history_report_json()).unwrap_or_default();
            let count = report["featureErrors"].as_array().map_or(0, Vec::len);
            preview.feature_errors = (generation, count);
        }
    }

    /// Keep the preview on the selected row, and re-evaluate it when the
    /// table changed under it (a committed cell of the previewed row updates
    /// the model live).
    fn follow_preview(&mut self, state: &mut EngineState) {
        let Some(preview) = &self.preview else { return };
        let Some(table) = self.cache.as_ref().map(|c| c.table.clone()) else { return };
        if table.rows.is_empty() {
            return self.end_preview(state, Some("Preview ended: the table has no rows"));
        }
        let row = self.selected.map_or(preview.row, |c| c.row).min(table.rows.len() - 1);
        if row != preview.row || preview.revision != state.history.revision() {
            self.preview_row(state, &table, row);
        }
    }

    /// Re-read the table and re-check it when the document moved.
    fn refresh(&mut self, state: &EngineState) {
        let revision = state.history.revision();
        if self.cache.as_ref().is_some_and(|c| c.revision == revision) {
            return;
        }
        let table = family_table::engine_table(state);
        let definitions: Vec<(String, String)> =
            serde_json::from_str::<Vec<Value>>(&state.expression_variables_json())
                .unwrap_or_default()
                .iter()
                .filter_map(|v| {
                    Some((v.get("name")?.as_str()?.to_string(), v.get("expr")?.as_str()?.to_string()))
                })
                .collect();
        let names: Vec<String> = definitions.iter().map(|(n, _)| n.clone()).collect();
        let check = check(&table, &state.history.expressions(), &names, &state.history.configurator());
        // A selection past a table that shrank (undo, a removed row) follows it.
        if let Some(cell) = self.selected {
            if table.rows.is_empty() {
                self.selected = None;
            } else {
                self.selected = Some(Cell {
                    row: cell.row.min(table.rows.len() - 1),
                    col: cell.col.min(column_count(&table) - 1),
                });
            }
        }
        if self.edit.as_ref().is_some_and(|e| e.cell.row >= table.rows.len() || e.cell.col >= column_count(&table)) {
            self.edit = None;
        }
        self.cache = Some(Cache { revision, table, names, definitions, check });
    }

    /// Write `table` into the document as one undo step. Every commit is its
    /// own step (no coalescing): a cell is committed once, when it is left.
    /// A table edit is not a model edit: a preview stays on through it,
    /// because [`Self::follow_preview`] re-evaluates and re-arms it in the
    /// same frame, before anything else can run the model.
    fn commit(&mut self, state: &mut EngineState, table: &FamilyTable) {
        family_table::apply_table(state, table, None);
    }

    /// Draw the pane. `store` and `identity` (the family's store identity,
    /// `None` when never saved) are what Generate writes beside, or, on a PLM
    /// store, the revision the server generates from. `dirty`: the family has
    /// unsaved edits.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, store: &dyn ModelStore, identity: Option<&str>, dirty: bool) {
        self.hits.clear();
        self.dirty = dirty;
        self.plm.sync(store.plm_client(), identity);
        if let Some(report) = self.plm.poll(dirty, store.pending_writes(), || state.history_request_json()) {
            self.status.clear();
            self.report = Some(ReportView::of(&report));
        }
        self.sync_preview(state);
        self.refresh(state);
        let Some(cache) = self.cache.as_ref() else { return };
        let key = (store.mutation_generation(), cache.revision, identity.map(str::to_string));
        if self.members.0.as_ref() != Some(&key) {
            let exists = cache
                .table
                .rows
                .iter()
                .map(|row| {
                    // On a PLM the members are parts, not files beside the
                    // family: no file mark (the report says what Generate did).
                    if self.plm.active() {
                        return None;
                    }
                    family_table::member_identity(identity, &row.part_number)
                        .map(|path| store.read(&path).is_some())
                })
                .collect();
            self.members = (Some(key), exists);
        }
        let table = cache.table.clone();
        let check = cache.check.clone();
        let names = cache.names.clone();
        let definitions = cache.definitions.clone();

        let mut change: Option<Change> = None;

        self.toolbar(ui, &table, &names, &mut change, state, store, identity);
        self.missing_line(ui, &table, &names);
        if let Some(key) = self.plm.show(ui, &table, &mut self.hits) {
            let mut next = table.clone();
            add_column(&mut next, &key);
            change = Some(Change::Table(next, format!("added the category key {key} as a column")));
        }
        ui.separator();
        self.grid(ui, &table, &check, &names, &definitions, &mut change);
        self.footer(ui, &table, &check);

        if let Some(Change::Table(next, what)) = change {
            if next != table {
                self.commit(state, &next);
                self.status = what;
                self.refresh(state);
            }
        }
        if let Some(r) = self.preview_toggle.take() {
            if self.previewing() == Some(r) {
                self.end_preview(state, Some("Preview ended"));
            } else if let Some(table) = self.cache.as_ref().map(|c| c.table.clone()) {
                self.selected = Some(Cell { row: r, col: self.selected.map_or(0, |c| c.col) });
                self.edit = None;
                self.preview_row(state, &table, r);
            }
        }
        self.follow_preview(state);
    }

    #[allow(clippy::too_many_arguments)]
    fn toolbar(
        &mut self,
        ui: &mut egui::Ui,
        table: &FamilyTable,
        names: &[String],
        change: &mut Option<Change>,
        state: &mut EngineState,
        store: &dyn ModelStore,
        identity: Option<&str>,
    ) {
        let row = self.selected.map(|c| c.row).filter(|r| *r < table.rows.len());
        ui.horizontal_wrapped(|ui| {
            let add = ui.button("Add row").on_hover_text("Add a member row below the selected row (or at the end)");
            self.hits.insert("family:add_row".into(), add.rect);
            if add.clicked() {
                let mut next = table.clone();
                let end = next.rows.len();
                let at = insert_row(&mut next, row.map_or(end, |r| r + 1));
                self.selected = Some(Cell { row: at, col: 0 });
                *change = Some(Change::Table(next, format!("added row {}", at + 1)));
            }
            let dup = ui
                .add_enabled(row.is_some(), egui::Button::new("Duplicate"))
                .on_hover_text("Copy the selected row below itself");
            self.hits.insert("family:duplicate_row".into(), dup.rect);
            if let (true, Some(r)) = (dup.clicked(), row) {
                let mut next = table.clone();
                if let Some(at) = duplicate_row(&mut next, r) {
                    self.selected = Some(Cell { row: at, col: 0 });
                    *change = Some(Change::Table(next, format!("duplicated row {}", r + 1)));
                }
            }
            let del = ui
                .add_enabled(row.is_some(), egui::Button::new("Delete row"))
                .on_hover_text("Remove the selected row. A member already generated from it stays where it is.");
            self.hits.insert("family:delete_row".into(), del.rect);
            if let (true, Some(r)) = (del.clicked(), row) {
                let mut next = table.clone();
                next.rows.remove(r);
                *change = Some(Change::Table(next, format!("deleted row {}", r + 1)));
            }
            for (key, label, step, tip) in [
                ("family:row_up", "Move up", -1, "Move the selected row up"),
                ("family:row_down", "Move down", 1, "Move the selected row down"),
            ] {
                let can = row.is_some_and(|r| r.checked_add_signed(step).is_some_and(|to| to < table.rows.len()));
                let button = ui.add_enabled(can, egui::Button::new(label)).on_hover_text(tip);
                self.hits.insert(key.into(), button.rect);
                if let (true, Some(r)) = (button.clicked(), row) {
                    let mut next = table.clone();
                    if let Some(to) = move_row(&mut next, r, step) {
                        if let Some(cell) = self.selected.as_mut() {
                            cell.row = to;
                        }
                        *change = Some(Change::Table(next, format!("moved row {} to {}", r + 1, to + 1)));
                    }
                }
            }
            ui.separator();
            let missing: Vec<&String> = names
                .iter()
                .filter(|n| !table.columns.iter().any(|c| &c.name == *n))
                .collect();
            let menu = ui.menu_button("Add column", |ui| {
                if missing.is_empty() {
                    ui.weak(if names.is_empty() {
                        "The model defines no expressions. Add some in the Expressions pane."
                    } else {
                        "Every expression is already a column."
                    });
                }
                for name in &missing {
                    let item = ui.button(name.as_str());
                    self.hits.insert(format!("family:add_column:{name}"), item.rect);
                    if item.clicked() {
                        let mut next = table.clone();
                        let i = add_column(&mut next, name);
                        self.selected = Some(Cell { row: self.selected.map_or(0, |c| c.row), col: FIXED_COLUMNS + i });
                        *change = Some(Change::Table(next, format!("added column {name}")));
                        ui.close();
                    }
                }
            });
            self.hits.insert("family:add_column".into(), menu.response.rect);
            ui.separator();
            let copy = ui.button("Copy table").on_hover_text("Copy the table as tab-separated text with a header row, for a spreadsheet");
            self.hits.insert("family:copy_table".into(), copy.rect);
            if copy.clicked() {
                ui.ctx().copy_text(to_tsv(table));
                self.status = format!("copied {} rows", table.rows.len());
            }
            ui.separator();
            let plm = self.plm.active();
            let generate = ui
                .add_enabled(
                    !table.rows.is_empty() && !self.plm.generating(),
                    egui::Button::new(egui::RichText::new("Generate").strong()),
                )
                .on_hover_text(if plm {
                    "Build every row and send it to the PLM, which writes each member as a part (the row's revision column is the revision written)"
                } else {
                    "Write every row out as its own part, <part number>.nbrep, beside this family"
                });
            self.hits.insert("family:generate".into(), generate.rect);
            if generate.clicked() {
                // Generate reads the table from the document, so a cell still
                // being typed into is committed first: what the user sees is
                // what is generated.
                let mut committed = false;
                if let Some(edit) = self.edit.take() {
                    let mut next = family_table::engine_table(state);
                    if set_cell(&mut next, edit.cell, &edit.text) {
                        self.commit(state, &next);
                        committed = true;
                    }
                }
                self.generate_calls += 1;
                if plm {
                    // The server answers later; `show` picks the report up. A
                    // cell committed just now is an unsaved edit too.
                    self.plm.request_generate(self.dirty || committed, state.history_request_json());
                    return;
                }
                let report = family_table::generate_family(store, identity, &state.history_request_json());
                // The report's own header says the counts; no second line.
                self.status.clear();
                self.report = Some(ReportView::of(&report));
                // Members were written: look again for which exist.
                self.members.0 = None;
            }
        });
    }

    /// The expressions that are not columns yet, so the author sees what the
    /// family could still drive.
    fn missing_line(&mut self, ui: &mut egui::Ui, table: &FamilyTable, names: &[String]) {
        let missing: Vec<&str> = names
            .iter()
            .filter(|n| !table.columns.iter().any(|c| &c.name == *n))
            .map(String::as_str)
            .collect();
        let line = if names.is_empty() {
            ui.weak("The model defines no expressions yet. The table drives expressions, so add them in the Expressions pane first.")
        } else if missing.is_empty() {
            ui.weak(format!("All {} expressions are columns.", names.len()))
        } else {
            ui.weak(format!("Not in the table: {}", missing.join(", ")))
        };
        self.hits.insert("family:missing".into(), line.rect);
    }

    fn footer(&mut self, ui: &mut egui::Ui, table: &FamilyTable, check: &Check) {
        ui.separator();
        // Wrap at the pane's VISIBLE right edge. The layout can be wider than
        // what the dock shows of it, so a line wrapped at the available width
        // ran under the pane's edge and was cut there.
        let visible = (ui.clip_rect().right() - ui.cursor().left() - 4.0).max(120.0);
        ui.set_max_width(ui.available_width().min(visible));
        let wrapped = |ui: &mut egui::Ui, text: egui::WidgetText| ui.add(egui::Label::new(text).wrap());
        let problems = check.problem_count();
        let summary = format!(
            "{} member{} · {} driven expression{} · {}",
            table.rows.len(),
            if table.rows.len() == 1 { "" } else { "s" },
            table.columns.len(),
            if table.columns.len() == 1 { "" } else { "s" },
            match problems {
                0 => "no problems".to_string(),
                1 => "1 problem".to_string(),
                n => format!("{n} problems"),
            }
        );
        let color = if problems > 0 { ui.visuals().warn_fg_color } else { ui.visuals().weak_text_color() };
        let label = wrapped(ui, egui::RichText::new(summary).color(color).into());
        self.hits.insert("family:summary".into(), label.rect);
        if !self.status.is_empty() {
            let status = egui::RichText::new(&self.status).color(ui.visuals().weak_text_color());
            wrapped(ui, status.into());
        }
        if let Some(report) = &self.report {
            // Failures first: they need acting on, and a long run of
            // skipped rows must not push them out of sight.
            let mut failed: Vec<&(String, RowReport)> =
                report.rows.iter().filter(|(_, r)| r.outcome != "written").collect();
            failed.sort_by_key(|(_, r)| r.outcome != "failed");
            // The counts on a line of their own that wraps (a collapsing
            // header's title does not), then the rows under a short header.
            let counts = format!(
                "Last Generate: {} written, {} skipped (unchanged), {} failed",
                report.written, report.skipped, report.failed
            );
            let counts = wrapped(ui, egui::RichText::new(counts).strong().into());
            self.hits.insert("family:report_counts".into(), counts.rect);
            let title = format!("Rows skipped or failed ({})", failed.len());
            let header = egui::CollapsingHeader::new(title)
                .id_salt("family-report")
                .default_open(report.failed > 0)
                .show(ui, |ui| {
                    if failed.is_empty() {
                        ui.weak("Every row was written.");
                    }
                    let font = egui::TextStyle::Body.resolve(ui.style());
                    let rows = |ui: &mut egui::Ui| for (pn, line) in &failed {
                        // One paragraph per row, so the reason wraps under
                        // the part number instead of running off the pane.
                        let mut job = egui::text::LayoutJob::default();
                        let strong = ui.visuals().strong_text_color();
                        let text = ui.visuals().text_color();
                        let format = |color| egui::TextFormat::simple(font.clone(), color);
                        job.append(if pn.is_empty() { "(no part number)" } else { pn }, 0.0, format(strong));
                        job.append(line.outcome, 6.0, format(outcome_color(ui, line.outcome)));
                        job.append(&line.reason, 6.0, format(text));
                        wrapped(ui, job.into());
                    };
                    // Its own scroll, within what is left of the pane.
                    egui::ScrollArea::vertical()
                        .id_salt("family-report-rows")
                        .max_height(ui.available_height().max(80.0))
                        .auto_shrink([false, true])
                        .show(ui, rows);
                });
            self.hits.insert("family:report".into(), header.header_response.rect);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn grid(
        &mut self,
        ui: &mut egui::Ui,
        table: &FamilyTable,
        check: &Check,
        names: &[String],
        definitions: &[(String, String)],
        change: &mut Option<Change>,
    ) {
        let grid_id = ui.make_persistent_id("family-grid");
        let columns = column_count(table);
        let row_h = ui.spacing().interact_size.y + 4.0;
        let font = egui::TextStyle::Body.resolve(ui.style());
        let mono = egui::TextStyle::Monospace.resolve(ui.style());
        let text_w = |ui: &egui::Ui, text: &str, font: &egui::FontId| {
            ui.fonts_mut(|f| f.layout_no_wrap(text.to_string(), font.clone(), egui::Color32::WHITE).size().x)
        };

        // Column widths from their content, within bounds.
        let mut widths = Vec::with_capacity(columns);
        for col in 0..columns {
            let (title, min, max) = match col {
                0 => (FIXED_TITLES[0].to_string(), 110.0, 220.0),
                1 => (FIXED_TITLES[1].to_string(), 60.0, 90.0),
                2 => (FIXED_TITLES[2].to_string(), 140.0, 220.0),
                c => (column_title(&table.columns[c - FIXED_COLUMNS]).to_string() + "  \u{25BE}", 70.0, 180.0),
            };
            let mut w = text_w(ui, &title, &font) + 16.0;
            for r in 0..table.rows.len() {
                let f = if col == 0 || col >= FIXED_COLUMNS { &mono } else { &font };
                w = w.max(text_w(ui, &cell_text(table, Cell { row: r, col }), f) + 16.0);
            }
            widths.push(w.clamp(min, max));
        }
        // The preview eye, the row number and the member mark.
        let gutter = 76.0;
        // The last Generate's outcome sits right of the row number, not at the
        // far right, where a narrow pane would scroll it out of sight.
        let status_w = if self.report.is_some() { 72.0 } else { 0.0 };
        let total_w = gutter + status_w + widths.iter().sum::<f32>();

        // Keyboard: consumed before any widget draws, so neither the cell's
        // text field nor egui's focus traversal sees the keys it acts on.
        let grid_focused = ui.memory(|m| m.has_focus(grid_id));
        let grab_grid_focus = self.keys(ui, table, names, columns, grid_focused, change);
        // Solid scroll bars: a floating one is drawn OVER the last row, and
        // takes the hover off the cell under it.
        ui.spacing_mut().scroll = egui::style::ScrollStyle::solid();
        let scroll = egui::ScrollArea::both()
            .id_salt("family-scroll")
            .auto_shrink([false, true])
            .max_height(ui.available_height() - 90.0)
            .show(ui, |ui| {
                let height = row_h * (table.rows.len() as f32 + 1.0);
                let (rect, _) = ui.allocate_exact_size(egui::vec2(total_w, height), egui::Sense::hover());
                // The grid's own focus target, under the cells.
                let grid = ui.interact(rect, grid_id, egui::Sense::focusable_noninteractive());
                if grab_grid_focus {
                    grid.request_focus();
                }
                ui.memory_mut(|m| {
                    m.set_focus_lock_filter(
                        grid_id,
                        egui::EventFilter { tab: true, horizontal_arrows: true, vertical_arrows: true, escape: false },
                    )
                });
                self.hits.insert("family:grid".into(), rect);
                let painter = ui.painter_at(rect);
                let visuals = ui.visuals().clone();
                let stroke = visuals.widgets.noninteractive.bg_stroke;

                // Header row.
                let mut x = rect.left() + gutter + status_w;
                let top = rect.top();
                painter.rect_filled(
                    egui::Rect::from_min_size(rect.min, egui::vec2(total_w, row_h)),
                    0.0,
                    visuals.faint_bg_color,
                );
                for col in 0..columns {
                    let cell_rect = egui::Rect::from_min_size(egui::pos2(x, top), egui::vec2(widths[col], row_h));
                    if col < FIXED_COLUMNS {
                        cell_label(&painter, cell_rect, FIXED_TITLES[col], font.clone(), visuals.strong_text_color(), false);
                    } else {
                        self.expression_header(ui, table, check, col - FIXED_COLUMNS, cell_rect, change);
                    }
                    painter.vline(cell_rect.right(), rect.y_range(), stroke);
                    x += widths[col];
                }
                if status_w > 0.0 {
                    let at = egui::Rect::from_min_size(egui::pos2(rect.left() + gutter, top), egui::vec2(status_w, row_h));
                    cell_label(&painter, at, "Result", font.clone(), visuals.strong_text_color(), false);
                    painter.vline(at.right(), rect.y_range(), stroke);
                }
                let _ = x;
                painter.hline(rect.x_range(), top + row_h, stroke);

                // Rows.
                for r in 0..table.rows.len() {
                    let y = top + row_h * (r as f32 + 1.0);
                    let row_rect = egui::Rect::from_min_size(egui::pos2(rect.left(), y), egui::vec2(total_w, row_h));
                    if r % 2 == 1 {
                        painter.rect_filled(row_rect, 0.0, visuals.faint_bg_color);
                    }
                    if self.previewing() == Some(r) {
                        // The previewed row: tinted, with a bar down its left edge.
                        painter.rect_filled(row_rect, 0.0, visuals.selection.bg_fill.gamma_multiply(0.35));
                        painter.rect_filled(
                            egui::Rect::from_min_size(row_rect.min, egui::vec2(3.0, row_h)),
                            0.0,
                            visuals.selection.bg_fill,
                        );
                    }
                    self.row_gutter(ui, table, r, egui::Rect::from_min_size(row_rect.min, egui::vec2(gutter, row_h)), grid_id);
                    if status_w > 0.0 {
                        let at = egui::Rect::from_min_size(egui::pos2(rect.left() + gutter, y), egui::vec2(status_w, row_h));
                        self.status_cell(ui, &table.rows[r], r, at);
                    }
                    let mut x = rect.left() + gutter + status_w;
                    for col in 0..columns {
                        let cell = Cell { row: r, col };
                        let cell_rect = egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(widths[col], row_h));
                        self.cell(ui, table, check, definitions, cell, cell_rect, grid_id, change);
                        x += widths[col];
                    }
                    painter.hline(rect.x_range(), y + row_h, stroke);
                }
                if table.rows.is_empty() {
                    ui.weak("No members yet. Use Add row, or paste rows copied from a spreadsheet.");
                }
            });
        let _ = scroll;
    }

    /// The frame's keyboard events, walked IN ORDER, while the grid or the
    /// cell being edited has the keyboard. In order because a fast typist (or
    /// a script) can deliver "type, Enter, type, Tab" in one frame, and each
    /// key has to see the state the one before it left.
    ///
    /// Grid mode: arrows / Tab / Shift+Tab move, Enter or F2 edits, typing
    /// starts an edit that replaces the cell, Delete clears it, a paste lands
    /// as a block, a copy copies the cell. Edit mode: Enter / Shift+Enter /
    /// Tab / Shift+Tab / Up / Down commit and move, Escape cancels, a pasted
    /// block is a block paste at the cell. Returns whether the grid should
    /// take the keyboard back (an edit ended).
    fn keys(
        &mut self,
        ui: &mut egui::Ui,
        table: &FamilyTable,
        names: &[String],
        columns: usize,
        grid_focused: bool,
        change: &mut Option<Change>,
    ) -> bool {
        use egui::{Event, Key};
        let editor_focused = ui.memory(|m| m.has_focus(egui::Id::new("family-cell-editor")));
        let ours = match &self.edit {
            Some(edit) => edit.fresh || editor_focused,
            None => grid_focused && self.selected.is_some(),
        };
        if !ours {
            return false;
        }
        let rows = table.rows.len();
        let events = ui.input(|i| i.events.clone());
        let mut consumed = vec![false; events.len()];
        let mut working = table.clone();
        let mut what: Option<String> = None;
        let mut grab = false;
        for (k, event) in events.iter().enumerate() {
            match (&mut self.edit, event) {
                // --- editing ---------------------------------------------
                (Some(edit), Event::Paste(text)) if text.contains(['\t', '\n']) => {
                    let at = edit.cell;
                    self.edit = None;
                    let outcome = paste_block(&mut working, at, &parse_tsv(text), names);
                    what = Some(outcome.summary());
                    grab = true;
                    consumed[k] = true;
                }
                // The field has not taken the keyboard yet (the edit began
                // this frame or last): what is typed goes to the buffer.
                (Some(edit), Event::Text(text) | Event::Paste(text)) if edit.fresh && !editor_focused => {
                    edit.text.push_str(text);
                    consumed[k] = true;
                }
                (Some(edit), Event::Key { key, pressed: true, modifiers, .. }) => {
                    let shift = modifiers.shift;
                    let step = match key {
                        Key::Escape => {
                            self.edit = None;
                            grab = true;
                            consumed[k] = true;
                            continue;
                        }
                        Key::Enter if shift => (-1, 0, false),
                        Key::Enter | Key::ArrowDown => (1, 0, false),
                        Key::ArrowUp => (-1, 0, false),
                        Key::Tab if shift => (0, -1, true),
                        Key::Tab => (0, 1, true),
                        _ => continue,
                    };
                    let cell = edit.cell;
                    if set_cell(&mut working, cell, &edit.text) {
                        what = Some(format!("set {}", cell_name(table, cell)));
                    }
                    self.edit = None;
                    self.selected = Some(step_cell(cell, step.0, step.1, working.rows.len(), columns, step.2));
                    grab = true;
                    consumed[k] = true;
                }
                (Some(_), _) => {}
                // --- grid ------------------------------------------------
                (None, Event::Key { key, pressed: true, modifiers, .. }) => {
                    let Some(cell) = self.selected else { continue };
                    if modifiers.command || modifiers.alt {
                        continue;
                    }
                    let shift = modifiers.shift;
                    let moved = match key {
                        Key::ArrowUp => Some((-1, 0, false)),
                        Key::ArrowDown => Some((1, 0, false)),
                        Key::ArrowLeft => Some((0, -1, false)),
                        Key::ArrowRight => Some((0, 1, false)),
                        Key::Tab if shift => Some((0, -1, true)),
                        Key::Tab => Some((0, 1, true)),
                        _ => None,
                    };
                    if let Some((dr, dc, wrap)) = moved {
                        self.selected = Some(step_cell(cell, dr, dc, rows.max(working.rows.len()), columns, wrap));
                        consumed[k] = true;
                    } else if matches!(key, Key::Enter | Key::F2) && !shift {
                        self.edit = Some(Edit { cell, text: cell_text(&working, cell), fresh: true });
                        consumed[k] = true;
                    } else if matches!(key, Key::Delete | Key::Backspace) {
                        if set_cell(&mut working, cell, "") {
                            what = Some(format!("cleared {}", cell_name(table, cell)));
                        }
                        consumed[k] = true;
                    }
                }
                (None, Event::Text(text)) => {
                    if let Some(cell) = self.selected {
                        self.edit = Some(Edit { cell, text: text.clone(), fresh: true });
                        consumed[k] = true;
                    }
                }
                (None, Event::Paste(text)) => {
                    if let Some(cell) = self.selected {
                        let outcome = paste_block(&mut working, cell, &parse_tsv(text), names);
                        what = Some(outcome.summary());
                        consumed[k] = true;
                    }
                }
                (None, Event::Copy) => {
                    if let Some(cell) = self.selected {
                        ui.ctx().copy_text(cell_text(&working, cell));
                        consumed[k] = true;
                    }
                }
                (None, _) => {}
            }
        }
        if consumed.iter().any(|c| *c) {
            ui.input_mut(|i| {
                let mut index = 0;
                i.events.retain(|_| {
                    let keep = !consumed.get(index).copied().unwrap_or(false);
                    index += 1;
                    keep
                });
            });
        }
        if let Some(what) = what {
            if working != *table {
                self.status = what.clone();
                *change = Some(Change::Table(working, what));
            }
        }
        grab
    }
    fn expression_header(
        &mut self,
        ui: &mut egui::Ui,
        table: &FamilyTable,
        check: &Check,
        index: usize,
        rect: egui::Rect,
        change: &mut Option<Change>,
    ) {
        let column = &table.columns[index];
        let problem = check.columns.get(&index);
        if let Some((_, mut text, fresh)) = self.relabel.take_if(|(i, _, _)| *i == index) {
            let id = egui::Id::new(("family-relabel", index));
            let response = ui.put(
                rect.shrink(2.0),
                egui::TextEdit::singleline(&mut text).id(id).hint_text(column.name.as_str()),
            );
            self.hits.insert(format!("family:col:{index}:label"), response.rect);
            if fresh {
                response.request_focus();
            } else if response.lost_focus() || !response.has_focus() {
                // Enter or a click away commits; Escape cancels.
                let cancel = ui.input(|i| i.key_pressed(egui::Key::Escape));
                if !cancel {
                    let mut next = table.clone();
                    if relabel_column(&mut next, index, &text) {
                        *change = Some(Change::Table(next, format!("relabelled column {}", column.name)));
                    }
                }
                return;
            }
            self.relabel = Some((index, text, false));
            return;
        }
        let title = column_title(column);
        let color = if problem.is_some() { ui.visuals().error_fg_color } else { ui.visuals().strong_text_color() };
        let font = egui::TextStyle::Body.resolve(ui.style());
        let response = ui.interact(rect, egui::Id::new(("family-col", index)), egui::Sense::click());
        if response.hovered() {
            ui.painter().rect_filled(rect.shrink(1.0), 2.0, ui.visuals().widgets.hovered.weak_bg_fill);
        }
        let painter = ui.painter_at(rect);
        cell_label(&painter, rect, &format!("{title}  \u{25BE}"), font, color, false);
        let response = response
            .on_hover_text(match problem {
                Some(problem) => problem.clone(),
                None if column.label.is_empty() => format!("Drives expression `{}`. Click for column options.", column.name),
                None => format!("Drives expression `{}` (labelled \"{}\"). Click for column options.", column.name, column.label),
            });
        self.hits.insert(format!("family:col:{index}"), response.rect);
        egui::Popup::menu(&response).show(|ui| {
            let mut item = |ui: &mut egui::Ui, key: &str, label: &str, enabled: bool| {
                let r = ui.add_enabled(enabled, egui::Button::new(label));
                self.hits.insert(format!("family:col:{index}:{key}"), r.rect);
                r.clicked()
            };
            if item(ui, "relabel", "Relabel…", true) {
                self.relabel = Some((index, column_title(column).to_string(), true));
            }
            if item(ui, "left", "Move left", index > 0) {
                let mut next = table.clone();
                move_column(&mut next, index, -1);
                *change = Some(Change::Table(next, format!("moved column {} left", column.name)));
            }
            if item(ui, "right", "Move right", index + 1 < table.columns.len()) {
                let mut next = table.clone();
                move_column(&mut next, index, 1);
                *change = Some(Change::Table(next, format!("moved column {} right", column.name)));
            }
            ui.separator();
            if item(ui, "remove", "Remove column", true) {
                let mut next = table.clone();
                remove_column(&mut next, index);
                *change = Some(Change::Table(next, format!("removed column {}", column.name)));
            }
        });
    }

    fn row_gutter(&mut self, ui: &mut egui::Ui, table: &FamilyTable, r: usize, rect: egui::Rect, grid_id: egui::Id) {
        let row = &table.rows[r];
        // The member mark: filled when `<part number>.nbrep` is in the store
        // beside the family, hollow when it is not, absent when there is no
        // file name to look for.
        let exists = self.members.1.get(r).copied().flatten();
        let response = ui.interact(rect, egui::Id::new(("family-row", r)), egui::Sense::click());
        self.hits.insert(format!("family:row:{r}"), rect);
        if response.clicked() {
            self.selected = Some(Cell { row: r, col: self.selected.map_or(0, |c| c.col) });
            self.edit = None;
        }
        let selected_row = self.selected.is_some_and(|c| c.row == r);
        let visuals = ui.visuals();
        let color = if selected_row { visuals.strong_text_color() } else { visuals.weak_text_color() };
        let painter = ui.painter_at(rect);
        painter.text(
            rect.left_center() + egui::vec2(30.0, 0.0),
            egui::Align2::LEFT_CENTER,
            format!("{}", r + 1),
            egui::TextStyle::Monospace.resolve(ui.style()),
            color,
        );
        if let Some(exists) = exists {
            let centre = rect.right_center() - egui::vec2(12.0, 0.0);
            if exists {
                painter.circle_filled(centre, 4.0, WRITTEN);
            } else {
                painter.circle_stroke(centre, 4.0, egui::Stroke::new(1.0, visuals.weak_text_color()));
            }
        }
        response.on_hover_text(match exists {
            Some(true) => format!("{}.nbrep exists", row.part_number.trim()),
            Some(false) => format!("{}.nbrep has not been generated", row.part_number.trim()),
            None => "No member file to look for: the family is not saved yet, or the part number cannot be a file name".to_string(),
        });

        // The preview toggle, over the gutter's left end (registered after
        // the gutter, so it takes the click there).
        let eye_rect = egui::Rect::from_min_size(rect.min + egui::vec2(4.0, 0.0), egui::vec2(22.0, rect.height()));
        let eye = ui.interact(eye_rect, egui::Id::new(("family-preview", r)), egui::Sense::click());
        self.hits.insert(format!("family:preview:{r}"), eye_rect);
        let on = self.previewing() == Some(r);
        let visuals = ui.visuals();
        let color = if on {
            visuals.selection.stroke.color
        } else if eye.hovered() {
            visuals.strong_text_color()
        } else {
            visuals.weak_text_color().gamma_multiply(0.6)
        };
        if eye.hovered() {
            painter.rect_filled(eye_rect.shrink(2.0), 3.0, visuals.widgets.hovered.weak_bg_fill);
        }
        paint_eye(&painter, eye_rect.center(), color, on);
        if eye.clicked() {
            self.preview_toggle = Some(r);
            ui.memory_mut(|m| m.request_focus(grid_id));
        }
        eye.on_hover_text(if on {
            "Previewing this row on the model. Click (or Esc) to show the family's own values again."
        } else {
            "Preview this row on the model: rebuild the displayed part with this row's values. Nothing is saved."
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn cell(
        &mut self,
        ui: &mut egui::Ui,
        table: &FamilyTable,
        check: &Check,
        definitions: &[(String, String)],
        cell: Cell,
        rect: egui::Rect,
        grid_id: egui::Id,
        change: &mut Option<Change>,
    ) {
        let key = format!("family:cell:{}:{}", cell.row, cell.col);
        self.hits.insert(key, rect);
        let expression = cell.col >= FIXED_COLUMNS;
        let problem = check.cells.get(&cell);
        let column_problem = expression.then(|| check.columns.get(&(cell.col - FIXED_COLUMNS))).flatten();

        // Editing: a text field in the cell.
        if let Some(mut edit) = self.edit.take_if(|e| e.cell == cell) {
            let id = egui::Id::new("family-cell-editor");
            if edit.fresh {
                let mut state = egui::TextEdit::load_state(ui.ctx(), id).unwrap_or_default();
                let end = egui::text::CCursor::new(edit.text.chars().count());
                state.cursor.set_char_range(Some(egui::text::CCursorRange::one(end)));
                state.store(ui.ctx(), id);
            }
            let mut field = egui::TextEdit::singleline(&mut edit.text).id(id).margin(egui::vec2(4.0, 2.0));
            if expression || cell.col == 0 {
                field = field.font(egui::TextStyle::Monospace);
            }
            let response = ui.put(rect.shrink(1.0), field);
            self.hits.insert("family:editor".into(), response.rect);
            if edit.fresh {
                response.request_focus();
                edit.fresh = false;
            } else if response.lost_focus() || !response.has_focus() {
                // Focus went elsewhere (a click on another cell or button):
                // commit what was typed, as a spreadsheet does.
                let mut next = table.clone();
                if change.is_none() && set_cell(&mut next, cell, &edit.text) {
                    *change = Some(Change::Table(next, format!("set {}", cell_name(table, cell))));
                }
                return;
            }
            self.edit = Some(edit);
            return;
        }

        let response = ui.interact(rect, egui::Id::new(("family-cell", cell.row, cell.col)), egui::Sense::click());
        if response.clicked() || response.double_clicked() {
            self.selected = Some(cell);
            ui.memory_mut(|m| m.request_focus(grid_id));
            if response.double_clicked() {
                self.edit = Some(Edit { cell, text: cell_text(table, cell), fresh: true });
            }
        }
        let visuals = ui.visuals().clone();
        let painter = ui.painter_at(rect);
        if problem.is_some() {
            painter.rect_filled(rect.shrink(1.0), 0.0, visuals.error_fg_color.gamma_multiply(0.18));
        } else if column_problem.is_some() {
            painter.rect_filled(rect.shrink(1.0), 0.0, visuals.warn_fg_color.gamma_multiply(0.12));
        }
        if self.selected == Some(cell) {
            painter.rect_stroke(rect.shrink(1.0), 0.0, visuals.selection.stroke, egui::StrokeKind::Inside);
        }
        let text = cell_text(table, cell);
        let font = if expression || cell.col == 0 {
            egui::TextStyle::Monospace.resolve(ui.style())
        } else {
            egui::TextStyle::Body.resolve(ui.style())
        };
        let inherited = expression && text.is_empty();
        let (shown, color) = if inherited {
            // A blank expression cell leaves the model's definition in force:
            // show that definition, dimmed.
            let name = &table.columns[cell.col - FIXED_COLUMNS].name;
            let definition = definitions.iter().find(|(n, _)| n == name).map(|(_, d)| d.as_str()).unwrap_or("");
            (definition.to_string(), visuals.weak_text_color().gamma_multiply(0.7))
        } else if problem.is_some() {
            (text.clone(), visuals.error_fg_color)
        } else {
            (text.clone(), visuals.text_color())
        };
        cell_label(&painter, rect, &shown, font, color, expression);
        if let Some(problem) = problem.or(column_problem) {
            // A corner flag, so a problem reads even in a narrow cell.
            let corner = rect.right_top() + egui::vec2(-1.0, 1.0);
            painter.add(egui::Shape::convex_polygon(
                vec![corner, corner + egui::vec2(-7.0, 0.0), corner + egui::vec2(0.0, 7.0)],
                if check.cells.contains_key(&cell) { visuals.error_fg_color } else { visuals.warn_fg_color },
                egui::Stroke::NONE,
            ));
            response.on_hover_text(problem.as_str());
        } else if let Some(value) = check.values.get(&cell) {
            response.on_hover_text(format!("= {}", format_value(*value)));
        } else if inherited {
            response.on_hover_text("Blank: this member keeps the model's own definition");
        }
        let _ = change;
    }

    fn status_cell(&mut self, ui: &mut egui::Ui, row: &FamilyRow, r: usize, rect: egui::Rect) {
        let Some(line) = self.report.as_ref().and_then(|rep| rep.for_part(&row.part_number)).cloned() else {
            return;
        };
        self.hits.insert(format!("family:status:{r}"), rect);
        let response = ui.interact(rect, egui::Id::new(("family-status", r)), egui::Sense::hover());
        ui.painter_at(rect).text(
            rect.left_center() + egui::vec2(6.0, 0.0),
            egui::Align2::LEFT_CENTER,
            line.outcome,
            egui::TextStyle::Body.resolve(ui.style()),
            outcome_color(ui, line.outcome),
        );
        if !line.reason.is_empty() {
            response.on_hover_text(line.reason);
        }
    }

    /// The card over the 3D view while a row is previewed: which row, that
    /// the family's own values are not shown, a row that does not evaluate,
    /// and Exit preview. Drawn every frame by the app (whether or not the
    /// pane is), in the viewport's top-right overlay column. Escape exits
    /// too: the app's key router calls [`Self::escape_preview`].
    pub fn preview_card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.sync_preview(state);
        let Some(preview) = self.preview.clone() else { return };
        let mut exit = false;
        let visuals = ui.visuals().clone();
        let card = egui::Frame::popup(ui.style()).show(ui, |ui| {
            ui.set_max_width(300.0);
            let heading = match &preview.problem {
                None => format!("Previewing {}", preview.label),
                Some(_) => format!("{} cannot be previewed", preview.label),
            };
            ui.add(egui::Label::new(egui::RichText::new(heading).strong().color(visuals.selection.stroke.color)).wrap());
            match &preview.problem {
                None => {
                    ui.add(egui::Label::new(egui::RichText::new(
                        "The model shows this row's values. The family's own values are not shown, and nothing here is saved. Measurements and exports are of this row.",
                    ).weak()).wrap());
                    // Values that evaluate can still break a feature: say so
                    // here, where the user is looking, not only in History.
                    let errors = preview.feature_errors.1;
                    if errors > 0 {
                        let text = format!(
                            "{errors} feature error{} with this row's values. See the History tab.",
                            if errors == 1 { "" } else { "s" }
                        );
                        ui.add(egui::Label::new(egui::RichText::new(text).color(visuals.warn_fg_color)).wrap());
                    }
                }
                Some(problem) => {
                    ui.add(egui::Label::new(egui::RichText::new(problem).color(visuals.error_fg_color)).wrap());
                    let still = match &preview.showing {
                        Some(label) => format!("Still showing {label}."),
                        None => "Still showing the family's own values.".to_string(),
                    };
                    ui.add(egui::Label::new(egui::RichText::new(still).weak()).wrap());
                }
            }
            ui.horizontal(|ui| {
                let button = ui.button("Exit preview").on_hover_text("Show the family's own values again (Esc)");
                self.hits.insert("family:preview_exit".into(), button.rect);
                if button.clicked() {
                    exit = true;
                }
                ui.weak("Esc");
            });
        });
        self.hits.insert("family:preview_banner".into(), card.response.rect);
        if exit {
            self.end_preview(state, Some("Preview ended"));
        }
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// The `__brepFamily` blob: the table as the pane last read it, where the
    /// selection and the edit are, every problem, and the last Generate's
    /// report.
    pub fn state_json(&self) -> String {
        let Some(cache) = &self.cache else {
            return json!({ "table": null }).to_string();
        };
        let problems: Vec<Value> = cache
            .check
            .cells
            .iter()
            .map(|(cell, message)| json!({ "row": cell.row, "col": cell.col, "message": message }))
            .collect();
        let column_problems: Vec<Value> = cache
            .check
            .columns
            .iter()
            .map(|(column, message)| json!({ "column": column, "message": message }))
            .collect();
        let missing: Vec<&String> = cache
            .names
            .iter()
            .filter(|n| !cache.table.columns.iter().any(|c| &c.name == *n))
            .collect();
        let report = self.report.as_ref().map(|r| {
            json!({
                "written": r.written,
                "skipped": r.skipped,
                "failed": r.failed,
                "rows": r.rows.iter().map(|(pn, line)| json!({ "partNumber": pn, "outcome": line.outcome, "reason": line.reason })).collect::<Vec<_>>(),
            })
        });
        json!({
            "table": cache.table,
            "expressions": cache.names,
            "notColumns": missing,
            "selected": self.selected.map(|c| json!({ "row": c.row, "col": c.col })),
            "editing": self.edit.as_ref().map(|e| json!({ "row": e.cell.row, "col": e.cell.col, "text": e.text })),
            "problems": problems,
            "columnProblems": column_problems,
            "values": cache.check.values.iter().map(|(c, v)| json!({ "row": c.row, "col": c.col, "value": v })).collect::<Vec<_>>(),
            "members": self.members.1,
            "generateCalls": self.generate_calls,
            "report": report,
            "plm": self.plm.state_json(Some(&cache.table)),
            "status": self.status,
            "preview": self.preview.as_ref().map(|p| json!({
                "row": p.row,
                "partNumber": p.label,
                "applied": p.problem.is_none() && p.showing.as_deref() == Some(p.label.as_str()),
                "showing": p.showing,
                "message": p.problem,
                "featureErrors": p.feature_errors.1,
            })),
            "runsReplied": self.runs_replied,
        })
        .to_string()
    }
}

/// The next cell from `cell`, `dr` rows and `dc` columns away, kept inside
/// the table. With `wrap`, stepping off a row's end continues on the next
/// row's start (Tab), and off its start on the previous row's end.
fn step_cell(cell: Cell, dr: isize, dc: isize, rows: usize, columns: usize, wrap: bool) -> Cell {
    let rows = rows.max(1);
    let columns = columns.max(1);
    if wrap && dc != 0 {
        let flat = (cell.row * columns + cell.col) as isize + dc;
        let flat = flat.clamp(0, (rows * columns) as isize - 1) as usize;
        return Cell { row: flat / columns, col: flat % columns };
    }
    Cell {
        row: (cell.row as isize + dr).clamp(0, rows as isize - 1) as usize,
        col: (cell.col as isize + dc).clamp(0, columns as isize - 1) as usize,
    }
}

/// "row 3 Part number" / "row 3 length", for the status line.
fn cell_name(table: &FamilyTable, cell: Cell) -> String {
    let column = match cell.col {
        c if c < FIXED_COLUMNS => FIXED_TITLES[c].to_string(),
        c => table.columns.get(c - FIXED_COLUMNS).map(|c| c.name.clone()).unwrap_or_default(),
    };
    format!("row {} {}", cell.row + 1, column)
}

fn format_value(value: f64) -> String {
    let text = format!("{value:.6}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// Draw `text` inside a cell: padded, on one line, cut with `…` when it does
/// not fit, left-aligned or (a value) right-aligned.
fn cell_label(painter: &egui::Painter, rect: egui::Rect, text: &str, font: egui::FontId, color: egui::Color32, right: bool) {
    let pad = 6.0;
    let mut job = egui::text::LayoutJob::simple_singleline(text.to_string(), font, color);
    job.wrap = egui::text::TextWrapping::truncate_at_width((rect.width() - 2.0 * pad).max(4.0));
    let galley = painter.layout_job(job);
    let y = rect.center().y - galley.size().y / 2.0;
    let x = if right { rect.right() - pad - galley.size().x } else { rect.left() + pad };
    painter.galley(egui::pos2(x, y), galley, color);
}

/// An eye, the preview toggle: a lens outline with a pupil, filled when on.
fn paint_eye(painter: &egui::Painter, centre: egui::Pos2, color: egui::Color32, on: bool) {
    let (w, h) = (7.5, 4.5);
    let steps = 14;
    let mut points = Vec::with_capacity(2 * steps);
    for i in 0..steps {
        let t = std::f32::consts::PI * i as f32 / steps as f32;
        points.push(centre + egui::vec2(-w * t.cos(), -h * t.sin()));
    }
    for i in 0..steps {
        let t = std::f32::consts::PI * i as f32 / steps as f32;
        points.push(centre + egui::vec2(w * t.cos(), h * t.sin()));
    }
    if on {
        painter.add(egui::Shape::convex_polygon(points, color.gamma_multiply(0.35), egui::Stroke::new(1.3, color)));
    } else {
        painter.add(egui::Shape::closed_line(points, egui::Stroke::new(1.2, color)));
    }
    painter.circle_filled(centre, 2.2, color);
}

/// The colour of a member that exists, and of a `written` outcome.
const WRITTEN: egui::Color32 = egui::Color32::from_rgb(0x3a, 0xa8, 0x5a);

fn outcome_color(ui: &egui::Ui, outcome: &str) -> egui::Color32 {
    match outcome {
        "written" => WRITTEN,
        "failed" => ui.visuals().error_fg_color,
        _ => ui.visuals().weak_text_color(),
    }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "family", prefix: "family:add_row", meaning: "add a member row below the selected row", command: None },
    HitKeyDoc { panel: "family", prefix: "family:duplicate_row", meaning: "copy the selected row below itself", command: None },
    HitKeyDoc { panel: "family", prefix: "family:delete_row", meaning: "remove the selected row", command: None },
    HitKeyDoc { panel: "family", prefix: "family:row_up", meaning: "move the selected row up", command: None },
    HitKeyDoc { panel: "family", prefix: "family:row_down", meaning: "move the selected row down", command: None },
    HitKeyDoc { panel: "family", prefix: "family:add_column", meaning: "the Add column menu (the model's expressions not yet columns)", command: None },
    HitKeyDoc { panel: "family", prefix: "family:add_column:", meaning: "add the named expression as a column (inside the Add column menu)", command: None },
    HitKeyDoc { panel: "family", prefix: "family:copy_table", meaning: "copy the table as tab-separated text", command: None },
    HitKeyDoc { panel: "family", prefix: "family:generate", meaning: "generate every row as its own part and show the report", command: None },
    HitKeyDoc { panel: "family", prefix: "family:plm_member_type", meaning: "on a PLM store: the part type new members are created in", command: None },
    HitKeyDoc { panel: "family", prefix: "family:plm_member_type_set", meaning: "on a PLM store: set the member part type to what was typed", command: None },
    HitKeyDoc { panel: "family", prefix: "family:plm_key:", meaning: "on a PLM store: add the named category attribute as a column", command: None },
    HitKeyDoc { panel: "family", prefix: "family:plm_attribute_problem:", meaning: "on a PLM store: a cell the server's catalog type check will refuse", command: None },
    HitKeyDoc { panel: "family", prefix: "family:plm_save_and_generate", meaning: "on a PLM store: save the family with its unsaved edits, then Generate", command: Some("family_generate") },
    HitKeyDoc { panel: "family", prefix: "family:missing", meaning: "the line naming the expressions not yet columns", command: None },
    HitKeyDoc { panel: "family", prefix: "family:grid", meaning: "the whole table (the keyboard target while a cell is selected)", command: None },
    HitKeyDoc { panel: "family", prefix: "family:cell:", meaning: "one cell, `family:cell:<row>:<col>`; col 0 part number, 1 revision, 2 description, 3+ the expression columns", command: None },
    HitKeyDoc { panel: "family", prefix: "family:editor", meaning: "the text field of the cell being edited", command: None },
    HitKeyDoc { panel: "family", prefix: "family:preview:", meaning: "a row's preview toggle (the eye), `family:preview:<row>`: preview the row on the model, or end the preview when it is the row previewed", command: None },
    HitKeyDoc { panel: "family", prefix: "family:preview_banner", meaning: "the card over the 3D view while a row is previewed", command: None },
    HitKeyDoc { panel: "family", prefix: "family:preview_exit", meaning: "Exit preview on that card: show the family's own values again", command: None },
    HitKeyDoc { panel: "family", prefix: "family:row:", meaning: "a row's number and member mark; a click selects the row", command: None },
    HitKeyDoc { panel: "family", prefix: "family:col:", meaning: "an expression column's header (`family:col:<i>`), its menu items (`:relabel`, `:left`, `:right`, `:remove`) and its label field (`:label`)", command: None },
    HitKeyDoc { panel: "family", prefix: "family:status:", meaning: "a row's outcome in the last Generate report", command: None },
    HitKeyDoc { panel: "family", prefix: "family:summary", meaning: "the members / columns / problems count", command: None },
    HitKeyDoc { panel: "family", prefix: "family:report", meaning: "the last Generate report's rows (failures first, then skips, listed in full)", command: None },
    HitKeyDoc { panel: "family", prefix: "family:report_counts", meaning: "the last Generate's written / skipped / failed line", command: None },
];

