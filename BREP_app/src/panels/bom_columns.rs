//! The BOM's COLUMN CONFIGURATION — the catalogue of known fields, the
//! settings-textarea format, and the translation between that text and the
//! generic [`crate::column_tree`] widget's `ColumnSpec` / `ColumnLayout`.
//!
//! This is the BOM-shaped half deliberately kept OUT of the widget. The widget
//! knows "a column of kind K"; this module knows that a BOM column is either a
//! PART field or an OCCURRENCE field, which of them are built in, and how the
//! user writes that down.
//!
//! # The settings text
//!
//! One column per line, in display order. A leading `*` means SHOWN:
//!
//! ```text
//! part.Mass
//! *part.Part_Number
//! *occurrence.Notes
//! *occurrence.Item_Number
//! ```
//!
//! * `part.` — stored on the PART, shared by every occurrence of it.
//! * `occurrence.` — stored on ONE placement.
//! * A name the catalogue does not know is a user-added CUSTOM field, not an
//!   error: it becomes a text column and is stored like any other.
//! * `#` starts a comment. Blank lines are ignored.
//! * A MALFORMED line (no `.`, an unknown prefix, an empty field name) costs
//!   that ONE column and nothing else: it is skipped, the text is left exactly
//!   as typed, and the panel lists `line N: …` under the textarea. A typo must
//!   never cost the user the rest of their configuration.
//! * A duplicate `prefix.Name` keeps the FIRST and reports the rest — two rules
//!   for one column would otherwise be decided by whichever the loop saw last.
//!
//! # The freeze marker
//!
//! A line that is exactly `-` FREEZES everything above it: those columns stay
//! put while the rest scroll horizontally, as in a spreadsheet.
//!
//! ```text
//! *part.Part_Number
//! -
//! *part.Description
//! ```
//!
//! * The STRUCTURAL columns — Item, the visibility toggle, the status badges —
//!   have no lines here and NEVER scroll. They are the row's identity and its
//!   state; scrolling them away leaves a table of values with nothing saying
//!   which part each line is about. So the frozen band is those three plus
//!   whatever the marker names, and a configuration with no marker at all still
//!   holds them still.
//! * A marker below every column would freeze the lot, leaving nothing to
//!   scroll it against; the widget reads that as "frozen: none" so no column
//!   can be stranded off the right edge.
//! * The FIRST marker decides. A second one is reported like any other
//!   unusable line — and, unlike a comment, it is DROPPED rather than
//!   preserved: preserved lines are re-emitted ABOVE the column block, where a
//!   stray `-` would come back as "freeze nothing" on the next write-back.
//!
//! Reordering or hiding a column by dragging its heading WRITES BACK here, so
//! the text and the table can never disagree. Comment and malformed lines
//! survive that round trip (they are re-emitted above the column block), so an
//! automated rewrite can never eat something the user typed.

use crate::column_tree::{CellKind, ColumnLayout, ColumnSpec};
use std::collections::HashSet;

/// The user-facing prefix for a field stored on the PART.
pub const PART_PREFIX: &str = "part";

/// The user-facing prefix for a field stored on ONE occurrence.
pub const OCCURRENCE_PREFIX: &str = "occurrence";

/// The always-present structural column: the tree itself (part name +
/// occurrence id). Not a stored attribute, so it is not written in the
/// settings text — but it IS an ordinary column to the widget, so the user can
/// still resize, reorder, and even hide it (the tree then draws in whatever
/// column comes first).
pub const ITEM_KEY: &str = "item";

/// The other structural column: the per-row ACTION MENU trigger. Also not a
/// stored attribute, so also not written in the settings text.
pub const ACTIONS_KEY: &str = "actions";

/// The per-row visibility toggle — structural, like [`ITEM_KEY`]: it shows the
/// scene's truth for the row's member solids and is never configured away,
/// because a component list you cannot blank out of the viewport is a
/// regression against the Structure panel this replaced.
pub const VISIBLE_KEY: &str = "visible";

/// The per-row status badges (fixed / outdated / worst constraint status).
/// Structural for the same reason, and one column rather than three because
/// they are all "what is true of this instance right now", read at a glance.
pub const FLAGS_KEY: &str = "flags";

/// The freeze marker: a settings line that is exactly this holds every column
/// above it fixed while the rest scroll.
pub const FREEZE_MARKER: &str = "-";

/// `occurrence.Quantity` is DERIVED, never stored: in the packed view it is
/// how many occurrences the row rolls up, in the unpacked view it is 1. It is
/// the one column the brief exempts from "all columns are editable".
pub const QUANTITY_KEY: &str = "occurrence.Quantity";

/// `occurrence.MF_QTY` is DERIVED as well: a harness wire's CUT LENGTH, the
/// quantity an automatic wire-cutting machine reads — the wire's routed length
/// from the current run plus the harness block's cut margin, once per wire
/// (plan decision 6). Blank on a component row; on a wire whose length the
/// run on hand cannot give, the cell says why instead of showing a number
/// (`brep_render::engine_state::WireLengthState`).
pub const MF_QTY_KEY: &str = "occurrence.MF_QTY";

/// `part.PMI` is DERIVED too: how many annotations the PART's own document
/// carries. A STEP assembly import puts a part-scoped dimension or tolerance
/// on the part rather than on each instance
/// (`brep_kernel`'s `io/step_import/assembly_pmi.rs`), so the parts list is
/// where a reader sees that the bolt is toleranced at all — the PMI panel shows
/// the OPEN document's annotations, which are the assembly's own.
pub const PMI_KEY: &str = "part.PMI";

/// Which store a BOM column's value lives in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scope {
    /// On the part's own document (`partAttributes`) — shared by every
    /// occurrence of the part.
    Part,
    /// On the placing ACOMP's `inputParams.bom` — one placement's own.
    Occurrence,
}

impl Scope {
    pub fn prefix(self) -> &'static str {
        match self {
            Scope::Part => PART_PREFIX,
            Scope::Occurrence => OCCURRENCE_PREFIX,
        }
    }
}

/// One configured BOM column.
#[derive(Debug, Clone, PartialEq)]
pub struct BomColumn {
    pub scope: Scope,
    /// The field name as written in the settings text (`Part_Number`) — also
    /// the attribute key in the stored document.
    pub field: String,
    /// Shown in the table (a leading `*` in the settings text).
    pub shown: bool,
}

impl BomColumn {
    /// The widget column key — the settings text's own `prefix.Field`, so a
    /// hit key, a layout entry and a config line all read the same.
    pub fn key(&self) -> String {
        format!("{}.{}", self.scope.prefix(), self.field)
    }

    /// The heading: the field name with underscores opened out.
    pub fn label(&self) -> String {
        self.field.replace('_', " ")
    }

    /// The editor kind — from the catalogue when the field is built in, plain
    /// text for a user-added one.
    pub fn kind(&self) -> CellKind {
        if self.key() == QUANTITY_KEY || self.key() == PMI_KEY || self.key() == MF_QTY_KEY {
            return CellKind::ReadOnly;
        }
        catalogue()
            .iter()
            .find(|entry| entry.scope == self.scope && entry.field == self.field)
            .map(|entry| entry.kind.clone())
            .unwrap_or(CellKind::Text)
    }
}

/// A built-in field: its scope, name and editor kind.
pub struct CatalogueEntry {
    pub scope: Scope,
    pub field: &'static str,
    pub kind: CellKind,
}

/// The built-in BOM fields. Part fields describe the PART, occurrence fields
/// describe ONE PLACEMENT — the owner's split, and the reason they persist in
/// two different documents.
pub fn catalogue() -> Vec<CatalogueEntry> {
    let choice = |options: &[&str]| CellKind::Choice {
        options: options.iter().map(|option| option.to_string()).collect(),
    };
    let part = |field, kind| CatalogueEntry {
        scope: Scope::Part,
        field,
        kind,
    };
    let occurrence = |field, kind| CatalogueEntry {
        scope: Scope::Occurrence,
        field,
        kind,
    };
    vec![
        // --- Part ---------------------------------------------------------
        part("Part_Number", CellKind::Text),
        part("Revision", CellKind::Text),
        part("Description", CellKind::Text),
        part(
            "Part_Type",
            choice(&["Manufactured", "Purchased", "Assembly", "Phantom", "Reference"]),
        ),
        part("Make_Buy", choice(&["Make", "Buy"])),
        part(
            "Unit_of_Measure",
            choice(&["EA", "MM", "M", "IN", "FT", "KG", "G", "LB", "L", "ML"]),
        ),
        part("Material", CellKind::Text),
        part("Finish", CellKind::Text),
        part("Mass", CellKind::Numeric { step: 0.01 }),
        // Derived — see `PMI_KEY`.
        part("PMI", CellKind::ReadOnly),
        part("Manufacturer", CellKind::Text),
        part("Manufacturer_Part_Number", CellKind::Text),
        part("Supplier", CellKind::Text),
        part("Supplier_Part_Number", CellKind::Text),
        part(
            "Lifecycle_State",
            choice(&["In Work", "In Review", "Released", "Obsolete"]),
        ),
        // --- Occurrence ---------------------------------------------------
        // Item / Find numbers are TEXT, not numeric: real BOMs use `1.2.3`
        // and `010`, and a numeric editor would eat both.
        occurrence("Item_Number", CellKind::Text),
        // Derived — see `QUANTITY_KEY`.
        occurrence("Quantity", CellKind::ReadOnly),
        // Derived — see `MF_QTY_KEY`.
        occurrence("MF_QTY", CellKind::ReadOnly),
        occurrence("Reference_Designator", CellKind::Text),
        occurrence("Find_Number", CellKind::Text),
        occurrence("Effectivity", CellKind::Text),
        occurrence("Occurrence_Name", CellKind::Text),
        occurrence("Position", CellKind::Text),
        occurrence("Notes", CellKind::Text),
        occurrence(
            "BOM_Structure",
            choice(&["Normal", "Phantom", "Reference", "Inseparable"]),
        ),
        occurrence("Alternate_Substitute", CellKind::Text),
    ]
}

/// The shipped configuration: every built-in field, in a sensible BOM reading
/// order, with the ones a parts list is actually read for starred. Everything
/// else is one `*` away.
pub fn default_text() -> String {
    let shown: HashSet<&str> = [
        "occurrence.Item_Number",
        "occurrence.Quantity",
        "occurrence.MF_QTY",
        "part.Part_Number",
        "part.Revision",
        "part.Description",
        "part.Material",
        "occurrence.Reference_Designator",
        "occurrence.Notes",
    ]
    .into_iter()
    .collect();
    // Reading order: what identifies the line, then what the part IS, then the
    // sourcing tail.
    let order = [
        "occurrence.Item_Number",
        "occurrence.Quantity",
        "occurrence.MF_QTY",
        "part.Part_Number",
        "part.Revision",
        "part.Description",
        "part.Part_Type",
        "part.Make_Buy",
        "part.Unit_of_Measure",
        "part.Material",
        "part.Finish",
        "part.Mass",
        "part.PMI",
        "part.Lifecycle_State",
        "occurrence.Reference_Designator",
        "occurrence.Find_Number",
        "occurrence.Occurrence_Name",
        "occurrence.Position",
        "occurrence.Effectivity",
        "occurrence.BOM_Structure",
        "occurrence.Alternate_Substitute",
        "occurrence.Notes",
        "part.Manufacturer",
        "part.Manufacturer_Part_Number",
        "part.Supplier",
        "part.Supplier_Part_Number",
    ];
    let mut out = String::from(
        "# One BOM column per line, in display order. A leading * shows it.\n\
         # part.<Field> is stored on the part; occurrence.<Field> on one placement.\n\
         # A name not listed here is a custom text field, not an error.\n\
         # A line that is just - freezes the columns above it; the rest scroll.\n",
    );
    for key in order {
        if shown.contains(key) {
            out.push('*');
        }
        out.push_str(key);
        out.push('\n');
    }
    out
}

/// The configuration text actually in force: the stored setting, or the
/// shipped default when it is empty. EMPTY is the persisted "never configured"
/// state (see `RenderSettings::bom_columns`), so a later change to the shipped
/// default still reaches every user who never overrode it — and a document
/// that was never configured persists byte-for-byte as before.
pub fn effective_text(stored: &str) -> String {
    if stored.trim().is_empty() {
        default_text()
    } else {
        stored.to_string()
    }
}

/// EVERY part field, for the toolbar's Part Properties dialog: the whole
/// `partAttributes` record of one part, not the subset a parts list is read
/// for.
///
/// The BOM table draws the columns the configuration STARS; the dialog draws
/// them all, because hiding a column is a statement about the table's width,
/// never about whether the part has a Supplier. So the `shown` flag is read
/// here only to be ignored — and the returned columns carry it verbatim, so a
/// caller can still say which ones the table happens to show.
///
/// Three sources, in this order, each contributing what the ones before it did
/// not:
///
/// 1. the configured `part.` columns, in the USER's own reading order (their
///    configuration is the order they think about these fields in, and a
///    custom field they added is a real field of the part);
/// 2. the built-in [`catalogue`] fields the configuration leaves out — a
///    deleted column line must not delete a part's Material;
/// 3. any key ALREADY STORED that neither names, alphabetically — a record
///    written by an import, an older configuration or the automation layer is
///    still the part's data, and a dialog that silently dropped it would be a
///    dialog that lies about what saving will keep.
pub fn part_fields(config_text: &str, stored: &serde_json::Value) -> Vec<BomColumn> {
    let mut out: Vec<BomColumn> = Vec::new();
    let mut seen: HashSet<String> = HashSet::new();
    let mut push = |out: &mut Vec<BomColumn>, seen: &mut HashSet<String>, column: BomColumn| {
        if seen.insert(column.field.clone()) {
            out.push(column);
        }
    };
    for column in parse(config_text).columns {
        if column.scope == Scope::Part {
            push(&mut out, &mut seen, column);
        }
    }
    for entry in catalogue() {
        if entry.scope == Scope::Part {
            push(
                &mut out,
                &mut seen,
                BomColumn { scope: Scope::Part, field: entry.field.to_string(), shown: false },
            );
        }
    }
    if let Some(record) = stored.as_object() {
        // BTreeMap-backed only under serde_json's `preserve_order` default off;
        // sort explicitly so the tail is stable whichever map serde_json uses.
        let mut orphans: Vec<&String> = record.keys().collect();
        orphans.sort();
        for key in orphans {
            push(
                &mut out,
                &mut seen,
                BomColumn { scope: Scope::Part, field: key.clone(), shown: false },
            );
        }
    }
    out
}

/// The result of reading the settings text.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ParsedColumns {
    /// The configured columns, in the text's order.
    pub columns: Vec<BomColumn>,
    /// Lines the parser could not use, as `line N: why` — shown under the
    /// textarea. The text itself is never rewritten by parsing.
    pub problems: Vec<String>,
    /// Comment and malformed lines, verbatim, so a write-back can re-emit them
    /// instead of eating them.
    pub preserved: Vec<String>,
    /// How many configured columns stood ABOVE the `-` freeze marker; `None`
    /// when the text has no marker at all (nothing frozen).
    pub frozen: Option<usize>,
}

/// Read the settings text. Never fails: an unusable line costs that one column
/// and is reported.
pub fn parse(text: &str) -> ParsedColumns {
    let mut out = ParsedColumns::default();
    let mut seen: HashSet<String> = HashSet::new();
    for (index, raw) in text.lines().enumerate() {
        let number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('#') {
            out.preserved.push(line.to_string());
            continue;
        }
        if line == FREEZE_MARKER {
            // The FIRST marker decides. A second is reported and dropped — a
            // preserved one would be re-emitted above the column block on the
            // next write-back and would then read as "freeze nothing".
            if out.frozen.is_some() {
                out.problems.push(format!(
                    "line {number}: a second '{FREEZE_MARKER}' freeze marker — the first one decides"
                ));
            } else {
                out.frozen = Some(out.columns.len());
            }
            continue;
        }
        let (shown, rest) = match line.strip_prefix('*') {
            Some(rest) => (true, rest.trim()),
            None => (false, line),
        };
        let Some((prefix, field)) = rest.split_once('.') else {
            out.problems.push(format!(
                "line {number}: '{rest}' has no '.' — write {PART_PREFIX}.Field or {OCCURRENCE_PREFIX}.Field"
            ));
            out.preserved.push(raw.trim_end().to_string());
            continue;
        };
        let scope = match prefix.trim() {
            PART_PREFIX => Scope::Part,
            OCCURRENCE_PREFIX => Scope::Occurrence,
            other => {
                out.problems.push(format!(
                    "line {number}: unknown prefix '{other}' — only '{PART_PREFIX}.' and '{OCCURRENCE_PREFIX}.' exist"
                ));
                out.preserved.push(raw.trim_end().to_string());
                continue;
            }
        };
        let field = field.trim();
        if field.is_empty() {
            out.problems
                .push(format!("line {number}: '{rest}' names no field"));
            out.preserved.push(raw.trim_end().to_string());
            continue;
        }
        let column = BomColumn {
            scope,
            field: field.to_string(),
            shown,
        };
        let key = column.key();
        if !seen.insert(key.clone()) {
            out.problems
                .push(format!("line {number}: '{key}' is already configured above"));
            continue;
        }
        out.columns.push(column);
    }
    out
}

/// Write a column list back out as settings text, re-emitting the preserved
/// (comment / malformed) lines above it so an automated rewrite — a header
/// drag, a hide — can never eat something the user typed. `frozen` re-emits the
/// `-` marker after that many columns, so dragging a column across the freeze
/// boundary writes itself down like any other reorder.
pub fn serialize(columns: &[BomColumn], preserved: &[String], frozen: Option<usize>) -> String {
    let mut out = String::new();
    for line in preserved {
        out.push_str(line);
        out.push('\n');
    }
    for (index, column) in columns.iter().enumerate() {
        if frozen == Some(index) {
            out.push_str(FREEZE_MARKER);
            out.push('\n');
        }
        if column.shown {
            out.push('*');
        }
        out.push_str(&column.key());
        out.push('\n');
    }
    // A marker BELOW every column (everything frozen) is legitimate and must
    // survive the round trip.
    if frozen.is_some_and(|at| at >= columns.len()) {
        out.push_str(FREEZE_MARKER);
        out.push('\n');
    }
    out
}

/// The widget's column SPECS for a parsed configuration: the structural tree
/// column first, then every configured column (hidden ones included — the
/// widget's own hide list decides what is drawn, so a hidden column is still
/// offered in the right-click checklist).
pub fn column_specs(parsed: &ParsedColumns) -> Vec<ColumnSpec> {
    let mut specs = vec![
        ColumnSpec::new(ITEM_KEY, "Item", CellKind::ReadOnly).width(190.0),
        ColumnSpec::new(VISIBLE_KEY, "", CellKind::Toggle).width(26.0),
        ColumnSpec::new(FLAGS_KEY, "", CellKind::Badges).width(52.0),
    ];
    for column in &parsed.columns {
        let width = match column.kind() {
            CellKind::ReadOnly => 60.0,
            CellKind::Numeric { .. } => 70.0,
            _ => 120.0,
        };
        specs.push(ColumnSpec::new(column.key(), column.label(), column.kind()).width(width));
    }
    specs.push(
        ColumnSpec::new(
            ACTIONS_KEY,
            "",
            // The MENU trigger, not one action: the ellipsis says "there is a
            // list behind this", which a pencil did not.
            CellKind::Actions {
                label: "\u{22EF}".to_string(),
            },
        )
        .width(34.0),
    );
    specs
}

/// The widget LAYOUT for a parsed configuration: the text's order, and its
/// unstarred columns hidden. Widths and sort are the caller's — they are
/// session state, not configuration, so they are deliberately not written into
/// the user's text.
pub fn layout_from(parsed: &ParsedColumns, keep: &ColumnLayout) -> ColumnLayout {
    let mut order = vec![
        ITEM_KEY.to_string(),
        VISIBLE_KEY.to_string(),
        FLAGS_KEY.to_string(),
    ];
    let mut hidden = HashSet::new();
    for column in &parsed.columns {
        let key = column.key();
        if !column.shown {
            hidden.insert(key.clone());
        }
        order.push(key);
    }
    order.push(ACTIONS_KEY.to_string());
    ColumnLayout {
        order,
        hidden,
        widths: keep.widths.clone(),
        sort: keep.sort.clone(),
        // +1 for the structural Item column, which leads the order and is
        // always inside the frozen band when there is a marker at all.
        frozen: parsed.frozen.map_or(STRUCTURAL_LEADING, |above| above + STRUCTURAL_LEADING),
    }
}

/// The freeze marker's place in the settings text for a layout the USER changed
/// by dragging: how many CONFIGURED columns fall inside the frozen band (the
/// structural Item / Actions columns have no line of their own). `None` when
/// nothing is frozen, so a document that never froze anything keeps a text with
/// no marker in it.
/// The structural columns that always lead the arrangement: Item, the
/// visibility toggle and the status badges. They have no lines in the settings
/// text, so every conversion between "columns the user typed" and "columns the
/// widget arranges" has to step over exactly this many.
pub const STRUCTURAL_LEADING: usize = 3;

pub fn frozen_from_layout(layout: &ColumnLayout) -> Option<usize> {
    // The structural columns freeze IMPLICITLY — they are not configurable and
    // have no lines — so a band holding only them is spelled by no marker at
    // all. Emitting one here would invent a `-` in the user's text on the first
    // resize of a table nobody had frozen anything in.
    if layout.frozen <= STRUCTURAL_LEADING {
        return None;
    }
    Some(
        layout
            .order
            .iter()
            .take(layout.frozen)
            .filter(|key| {
                !matches!(
                    key.as_str(),
                    ITEM_KEY | VISIBLE_KEY | FLAGS_KEY | ACTIONS_KEY
                )
            })
            .count(),
    )
}

/// Fold a layout the USER changed by dragging back into a column list, so it
/// can be serialized into the settings text. The structural columns have no
/// lines of their own, so they are skipped; a column the layout somehow does
/// not name keeps its configured place at the end.
pub fn columns_from_layout(parsed: &ParsedColumns, layout: &ColumnLayout) -> Vec<BomColumn> {
    let mut out: Vec<BomColumn> = Vec::new();
    for key in &layout.order {
        if matches!(
            key.as_str(),
            ITEM_KEY | VISIBLE_KEY | FLAGS_KEY | ACTIONS_KEY
        ) {
            continue;
        }
        if let Some(column) = parsed.columns.iter().find(|column| &column.key() == key) {
            out.push(BomColumn {
                shown: !layout.hidden.contains(key),
                ..column.clone()
            });
        }
    }
    for column in &parsed.columns {
        if !out.iter().any(|kept| kept.key() == column.key()) {
            out.push(BomColumn {
                shown: !layout.hidden.contains(&column.key()),
                ..column.clone()
            });
        }
    }
    out
}

