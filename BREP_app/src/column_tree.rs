//! A tree table with resizable, reorderable, hideable, sortable columns.
//!
//! Callers supply columns, rows, and layout; [`ColumnTreeOut`] reports edits and
//! actions for the caller to apply after drawing. Consumer-specific behavior stays
//! in the panel. The first visible column uses [`crate::panels::tree::node`].
//!
//! Layout is mutated directly; callers persist it when `layout_changed` is set.
//! [`ColumnLayout::frozen`] pins leading columns, so this widget owns horizontal
//! scrolling. Expansion remains caller-owned. Transient drag, menu, and text-edit
//! state lives in egui memory keyed by [`ColumnTreeSpec::id`]. Text edits commit on
//! focus loss or Enter because applying a cell edit may rebuild geometry.
//!
//! Row actions share one menu for cell clicks and right-clicks. Disabled entries
//! remain visible with tooltips explaining why they are unavailable.

use crate::color::parse_hex_color;
use crate::panels::tree::{self, TreeRow};
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// Minimum column width, in points — narrow enough to park a column out of the
/// way, wide enough that its resize grip is still catchable.
pub const MIN_COLUMN_WIDTH: f32 = 28.0;

/// The default width of a column whose [`ColumnLayout`] carries none.
pub const DEFAULT_COLUMN_WIDTH: f32 = 110.0;

/// Width of the draggable divider strip at a header cell's right edge.
const GRIP: f32 = 5.0;

/// Horizontal padding inside a cell, so text does not touch the divider.
const CELL_PAD: f32 = 3.0;

/// Width of the band between the frozen columns and the scrolling ones.
const FREEZE_GAP: f32 = 4.0;

/// What a column's cells are, and therefore which editor the widget draws.
#[derive(Debug, Clone, PartialEq)]
pub enum CellKind {
    /// Free text. Committed on focus-loss / Enter, NOT per keystroke.
    Text,
    /// A number, edited with a drag-value.
    Numeric { step: f64 },
    /// A fixed set of choices — a combobox. An empty string is always
    /// offered as the "unset" choice, so a user can clear a cell.
    Choice { options: Vec<String> },
    /// An action button in every row of the column. The widget reports the
    /// click ([`ColumnTreeOut::buttons`]); the consumer acts.
    Button { label: String },
    /// The row's ACTION MENU trigger. Every row draws `label`; clicking it
    /// opens that row's [`RowNode::actions`] — the same menu a right-click on
    /// the row opens. A row that declares no action draws the trigger greyed.
    Actions { label: String },
    /// Status glyphs drawn inline, from a cell value shaped
    /// `[{"glyph": "⏚", "color": "#ff9f0a", "tooltip": "…"}, …]` (`color` and
    /// `tooltip` optional). An optional `action` makes a glyph a button that
    /// emits a row action. A plain text cell cannot colour part of its own
    /// content, and these glyphs carry meaning IN their colour — a constraint
    /// status, an out-of-date badge — so they get a kind rather than being
    /// flattened into a string.
    Badges,
    /// A boolean, drawn as a checkbox. The cell's value is read with
    /// [`Value::as_bool`]; a missing value reads false. A row that is not
    /// [`RowNode::editable`] still DRAWS its box (so the column stays
    /// readable) but cannot be clicked — a derived grouping row has no state
    /// of its own to toggle.
    Toggle,
    /// Displayed, never edited (a derived value — a rolled-up quantity, a
    /// computed length).
    ReadOnly,
}

/// One column: its cell key, its heading, its editor, its default width.
#[derive(Debug, Clone, PartialEq)]
pub struct ColumnSpec {
    /// The key this column reads out of [`RowNode::cells`]. Unique per spec.
    pub key: String,
    /// The heading text.
    pub label: String,
    pub kind: CellKind,
    /// Width used when [`ColumnLayout::widths`] carries none for this key.
    pub default_width: f32,
}

impl ColumnSpec {
    /// A column with the default width.
    pub fn new(key: impl Into<String>, label: impl Into<String>, kind: CellKind) -> Self {
        Self {
            key: key.into(),
            label: label.into(),
            kind,
            default_width: DEFAULT_COLUMN_WIDTH,
        }
    }

    pub fn width(mut self, width: f32) -> Self {
        self.default_width = width;
        self
    }
}

/// One entry in a row's action menu: what to call it, whether this ROW allows
/// it, and how to draw it. Everything here is the consumer's vocabulary — the
/// widget only ever compares [`RowAction::id`] for equality when it reports the
/// click back.
#[derive(Debug, Clone, PartialEq)]
pub struct RowAction {
    /// Stable id, handed back in [`RowActionClick::action`].
    pub id: String,
    /// The menu text.
    pub label: String,
    /// Hover text. Set it on a DISABLED entry to say why it is refused — the
    /// entry is greyed, not hidden, so this is where the reason is told.
    pub tooltip: String,
    /// Whether THIS ROW allows it. A disabled entry is drawn greyed and
    /// reports nothing.
    pub enabled: bool,
    /// Draw a separator line above this entry (grouping, e.g. before a
    /// destructive tail).
    pub separator_above: bool,
    /// Draw in the error colour — an entry that destroys something.
    pub destructive: bool,
}

impl RowAction {
    /// An enabled entry.
    pub fn new(id: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            label: label.into(),
            tooltip: String::new(),
            enabled: true,
            separator_above: false,
            destructive: false,
        }
    }

    pub fn tooltip(mut self, text: impl Into<String>) -> Self {
        self.tooltip = text.into();
        self
    }

    /// Refuse this entry ON THIS ROW, saying why (the greyed entry's tooltip).
    pub fn disabled(mut self, why: impl Into<String>) -> Self {
        self.enabled = false;
        self.tooltip = why.into();
        self
    }

    pub fn separator_above(mut self) -> Self {
        self.separator_above = true;
        self
    }

    pub fn destructive(mut self) -> Self {
        self.destructive = true;
        self
    }
}

/// One row. Rows nest — this is a TREE, not a flat table — and each row's
/// cells are looked up by column key, so adding a column never touches the
/// row builder's shape.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RowNode {
    /// Stable, unique across the whole tree — the id every out-value names.
    pub id: String,
    /// Cell values by column key. A missing key draws an empty cell.
    pub cells: HashMap<String, Value>,
    /// Whether this row's cells accept edits. A read-only row still draws its
    /// values (and its buttons), it just cannot be typed into — the BOM's
    /// nested sub-assembly rows, whose data belongs to another document.
    pub editable: bool,
    /// Draw the first column's label emphasized (the selected row).
    pub selected: bool,
    /// Expansion is the CALLER's, as in `panels::tree`.
    pub expanded: bool,
    /// What this row offers, in menu order. EMPTY means the row offers
    /// nothing: its trigger cell draws greyed and a right-click on it opens
    /// nothing (the BOM's nested sub-assembly rows, which own no feature in
    /// this document).
    pub actions: Vec<RowAction>,
    pub children: Vec<RowNode>,
}

impl RowNode {
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            editable: true,
            ..Default::default()
        }
    }

    /// Set one cell.
    pub fn cell(mut self, key: impl Into<String>, value: Value) -> Self {
        self.cells.insert(key.into(), value);
        self
    }

    /// Set this row's action menu.
    pub fn actions(mut self, actions: Vec<RowAction>) -> Self {
        self.actions = actions;
        self
    }
}

/// The user's column arrangement — the widget WRITES this (header drag,
/// divider drag, hide/show, sort click) and the consumer owns and persists it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ColumnLayout {
    /// Column keys in display order. Keys absent from the spec are ignored;
    /// spec columns absent from `order` are appended in spec order, so a
    /// consumer may start with an empty layout and get the spec's order.
    pub order: Vec<String>,
    /// Column keys the user has hidden.
    pub hidden: HashSet<String>,
    /// Per-column width overrides, in points.
    pub widths: HashMap<String, f32>,
    /// `(column key, ascending)`. `None` = the consumer's own row order.
    pub sort: Option<(String, bool)>,
    /// How many LEADING columns are held fixed while the rest scroll
    /// horizontally. Counted over the arranged order INCLUDING hidden columns,
    /// so hiding a frozen column does not silently promote the next one into
    /// the frozen region. `0` = nothing frozen, one plain scrolling table.
    pub frozen: usize,
}

/// One cell was edited.
#[derive(Debug, Clone, PartialEq)]
pub struct CellEdit {
    pub row_id: String,
    pub column: String,
    pub value: Value,
}

/// A [`CellKind::Button`] cell was clicked.
#[derive(Debug, Clone, PartialEq)]
pub struct CellClick {
    pub row_id: String,
    pub column: String,
}

/// A row's action menu or actionable badge fired.
#[derive(Debug, Clone, PartialEq)]
pub struct RowActionClick {
    pub row_id: String,
    /// The [`RowAction::id`] the consumer declared.
    pub action: String,
}

/// Everything the consumer supplies. Borrowed; the widget holds no state of
/// its own beyond egui memory.
pub struct ColumnTreeSpec<'a> {
    /// Scope key for this tree's transient view state and widget ids. Must be
    /// STABLE and UNIQUE per tree — two trees sharing it share their drag
    /// state and their text buffers.
    pub id: &'a str,
    /// The columns, in spec order (the fallback order when `layout.order` does
    /// not name them).
    pub columns: &'a [ColumnSpec],
    /// An optional always-open root row above the tree (`"Assembly"`), drawn
    /// with its own cells taken from `root_cells`.
    pub root_label: Option<&'a str>,
    /// Cells for the root row, when there is one.
    pub root_cells: Option<&'a HashMap<String, Value>>,
    /// Text drawn in place of the body when `rows` is empty.
    pub empty_hint: Option<&'a str>,
    /// Prefix for every published hit key. `""` for a consumer that shows one
    /// tree at a time.
    pub hits_prefix: &'a str,
}

/// What the user did in one drawn frame.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ColumnTreeOut {
    /// Cell edits, in draw order. A frame can carry at most one (egui gives
    /// one widget the focus), but the vec keeps the caller from having to care.
    pub edits: Vec<CellEdit>,
    /// Button cells clicked this frame.
    pub buttons: Vec<CellClick>,
    /// Row action-menu entries chosen this frame.
    pub actions: Vec<RowActionClick>,
    /// A row's `[+]`/`[-]` box was clicked — the caller flips its own
    /// expansion state (expansion is the caller's, as in `panels::tree`).
    pub toggled: Option<String>,
    /// A row was clicked (select) — on its first column's label, or anywhere
    /// else along its band that no cell of its own claimed.
    pub clicked: Option<String>,
    /// A row was double-clicked (open its dialog), over the same area as
    /// [`Self::clicked`]. egui reports the second press as a click too, so on
    /// this frame `clicked` names the same row: a consumer checks this first.
    pub double_clicked: Option<String>,
    /// A row (any of its cells, in either pane) is under the pointer this
    /// frame — the consumer's hover-to-highlight hook. `None` when the pointer
    /// is over the header, the root row, or off the table.
    pub hovered: Option<String>,
    /// The layout was changed by the user (reorder / resize / hide / sort) —
    /// the caller persists it.
    pub layout_changed: bool,
}

/// Draw ONE complete column tree and return what the user did.
///
/// `hits`, when supplied, receives widget screen rects for the headed
/// verifier, all prefixed with [`ColumnTreeSpec::hits_prefix`]:
///
/// | key | what |
/// |---|---|
/// | `col:{key}` | a column heading |
/// | `grip:{key}` | a heading's resize divider |
/// | `row:{row id}` | the row's click band (of the pane holding column 0) |
/// | `box:{row id}` | the first column's collapse box |
/// | `cell:{row id}:{col key}` | one cell's editor |
/// | `menu:{row id}` | the row's action-menu trigger cell |
/// | `menuitem:{row id}:{action id}` | one entry of the OPEN action menu |
/// | `freeze:divider` | the frozen / scrolling boundary (only when frozen) |
pub fn column_tree(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    layout: &mut ColumnLayout,
    rows: &[RowNode],
    mut hits: Option<&mut HashMap<String, egui::Rect>>,
) -> ColumnTreeOut {
    let mut out = ColumnTreeOut::default();
    // The arranged order INCLUDING the hidden columns: the freeze boundary is
    // counted over THIS, so hiding a frozen column cannot silently promote the
    // next one into the frozen band.
    let arranged = arranged_columns(spec, layout);
    let visible: Vec<&ColumnSpec> = arranged
        .iter()
        .copied()
        .filter(|column| !layout.hidden.contains(&column.key))
        .collect();
    if visible.is_empty() {
        ui.label(egui::RichText::new("(every column is hidden)").weak());
        return out;
    }

    // Column widths, in the drawn order.
    let widths: Vec<f32> = visible
        .iter()
        .map(|column| column_width(layout, column))
        .collect();

    // How many DRAWN columns are frozen. Freezing every column is meaningless
    // — there is nothing left to scroll it against — and would strand any
    // column past the pane's right edge with no way to reach it, so it reads
    // as "frozen: none".
    let mut frozen = arranged
        .iter()
        .take(layout.frozen)
        .filter(|column| !layout.hidden.contains(&column.key))
        .count();
    if frozen >= visible.len() {
        frozen = 0;
    }

    ui.spacing_mut().item_spacing.y = 2.0;
    // The width to lay out in. `available_rect_before_wrap` can be WIDER than
    // what is on screen — an outer scroll area hands its inner `Ui` a max_rect
    // that ignores its own viewport — and the horizontal scroll area below
    // sizes its viewport from this rect. A `full` wider than the clip
    // overstates that viewport by the difference, and egui clamps scrolling at
    // `content - viewport`: the last columns then have no scroll position that
    // brings them inside the pane. In the shipped BOM that stranded the
    // rightmost column, which is the ACTIONS one — its trigger cell was never
    // drawn at any scroll offset, so `menu:<row>` was a key the pane could not
    // publish. Found by the bom-menu verifier migration (2026-09-13); the
    // right-click trigger on the row was carrying the whole feature.
    let full = ui.available_rect_before_wrap();
    let full = egui::Rect::from_min_max(
        full.min,
        egui::pos2(full.max.x.min(ui.clip_rect().max.x), full.max.y),
    );
    // The frozen band never eats the whole width — something has to be left to
    // scroll in.
    let frozen_width: f32 = widths[..frozen]
        .iter()
        .sum::<f32>()
        .min((full.width() - MIN_COLUMN_WIDTH).max(0.0));

    // Which row's action menu either trigger asked for, this frame.
    let mut pending: Option<MenuOpen> = None;
    // Heading bounds from BOTH panes, merged: a reorder drag that crosses the
    // freeze boundary must find its drop target, because dropping a column on
    // the other side of the boundary is how a column is frozen or unfrozen
    // with the mouse.
    let mut bounds: Vec<(String, f32, f32)> = Vec::new();
    let mut bottom = full.top();

    if frozen > 0 {
        let rect = egui::Rect::from_min_max(
            full.min,
            egui::pos2(full.min.x + frozen_width, full.max.y),
        );
        let mut pane = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect)
                .layout(egui::Layout::top_down(egui::Align::Min))
                .id_salt((spec.id, "column-tree-frozen")),
        );
        // Clip the frozen band HORIZONTALLY only: its height belongs to the
        // caller (an outer vertical scroll area owns that axis).
        pane.set_clip_rect(pane.clip_rect().intersect(egui::Rect::from_x_y_ranges(
            rect.x_range(),
            ui.clip_rect().y_range(),
        )));
        pane.spacing_mut().item_spacing.y = 2.0;
        draw_pane(
            &mut pane,
            spec,
            layout,
            &visible[..frozen],
            &widths[..frozen],
            0,
            frozen_width,
            rows,
            &mut hits,
            &mut out,
            &mut pending,
            &mut bounds,
        );
        bottom = bottom.max(pane.min_rect().bottom());
    }

    // The scrolling remainder. The widget owns this scroll area rather than the
    // caller, because a scroll area OUTSIDE the widget would carry the frozen
    // columns away with everything else.
    let scroll_left = full.min.x + if frozen > 0 { frozen_width + FREEZE_GAP } else { 0.0 };
    let scroll_rect = egui::Rect::from_min_max(egui::pos2(scroll_left, full.min.y), full.max);
    let viewport = scroll_rect.width();
    let mut pane = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(scroll_rect)
            .layout(egui::Layout::top_down(egui::Align::Min))
            .id_salt((spec.id, "column-tree-scrolling")),
    );
    let rest: f32 = widths[frozen..].iter().sum();
    // (egui's default `ScrollSource` drags to scroll on TOUCH only, which is
    // what we want: a mouse drag-to-scroll would fight the header's reorder
    // drag and a text cell's selection drag.)
    egui::ScrollArea::horizontal()
        .id_salt((spec.id, "column-tree-hscroll"))
        .show(&mut pane, |ui: &mut egui::Ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            draw_pane(
                ui,
                spec,
                layout,
                &visible[frozen..],
                &widths[frozen..],
                frozen,
                rest.max(viewport),
                rows,
                &mut hits,
                &mut out,
                &mut pending,
                &mut bounds,
            );
            if rest > viewport {
                // A GUTTER for the horizontal scrollbar. egui's bars float over
                // the content, and this one lands on the LAST ROW — where it
                // silently eats every click on the bottom half of that row's
                // cells (found by the headed verifier: the action trigger opened
                // from its top edge and not from its centre).
                let scroll = ui.spacing().scroll;
                ui.add_space(scroll.bar_width + scroll.bar_inner_margin + scroll.bar_outer_margin);
            }
        });
    bottom = bottom.max(pane.min_rect().bottom());

    let used = egui::Rect::from_min_max(full.min, egui::pos2(full.max.x, bottom));
    ui.advance_cursor_after_rect(used);

    // The boundary, so the user can SEE which columns are pinned.
    if frozen > 0 {
        let x = full.min.x + frozen_width + FREEZE_GAP * 0.5;
        let divider = egui::Rect::from_min_max(
            egui::pos2(x - 1.0, used.top()),
            egui::pos2(x + 1.0, used.bottom()),
        );
        ui.painter()
            .rect_filled(divider, 0.0, ui.visuals().widgets.active.bg_fill);
        publish(&mut hits, spec, "freeze:divider", divider);
    }

    finish_reorder(ui, spec, layout, &arranged, &bounds, &mut out);
    // ONE menu, drawn from whatever record a trigger left. `pending` is applied
    // AFTER it — apply it first and the popup's own close-on-click would see
    // the very click that opened it and shut immediately.
    row_action_menu(ui, spec, rows, &mut hits, &mut out, &mut pending);
    out
}

/// Draw ONE pane — a contiguous run of columns, with the header and every row.
/// The frozen band and the scrolling remainder are the same code walking the
/// same rows in the same order, which is what keeps them aligned.
///
/// `offset` is the drawn index of `columns[0]`, so the pane holding column 0
/// (and only it) draws the TREE cell. `band` is the width every row spans here.
#[allow(clippy::too_many_arguments)]
fn draw_pane(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    layout: &mut ColumnLayout,
    columns: &[&ColumnSpec],
    widths: &[f32],
    offset: usize,
    band: f32,
    rows: &[RowNode],
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut ColumnTreeOut,
    pending: &mut Option<MenuOpen>,
    bounds: &mut Vec<(String, f32, f32)>,
) {
    header(ui, spec, layout, columns, widths, band, hits, out, bounds);
    ui.separator();

    if let Some(label) = spec.root_label {
        let mut cells = spec.root_cells.cloned().unwrap_or_default();
        // The root's label belongs to whichever column is drawn FIRST — the
        // tree column follows the user's order, so it is not a fixed key.
        if offset == 0 {
            cells.insert(columns[0].key.clone(), Value::String(label.to_string()));
        }
        let root = RowNode {
            id: format!("{}__root", spec.id),
            cells,
            editable: false,
            selected: false,
            expanded: true,
            actions: Vec::new(),
            children: Vec::new(),
        };
        draw_row(
            ui, spec, columns, widths, offset, band, &root, &[], true, true, hits, out, pending,
        );
    }

    if rows.is_empty() {
        if let Some(hint) = spec.empty_hint {
            if offset == 0 {
                let guides = tree::child_guides(&[], true);
                tree::node(ui, TreeRow::leaf(&guides, true, hint), |_| {});
            } else {
                // The other pane still spends the same row, so the two panes
                // keep the same height.
                ui.allocate_exact_size(
                    egui::vec2(band, ui.spacing().interact_size.y),
                    egui::Sense::hover(),
                );
            }
        }
        return;
    }

    let ordered = sorted_siblings(rows, layout);
    let last = ordered.len();
    for (index, row) in ordered.iter().enumerate() {
        draw_subtree(
            ui,
            spec,
            layout,
            columns,
            widths,
            offset,
            band,
            row,
            &[],
            index + 1 == last,
            hits,
            out,
            pending,
        );
    }
}

/// Every spec column in the user's order — `layout.order` first (spec columns
/// only), then any spec column the layout has never heard of. Hidden columns
/// KEEP their slot here; the drawn set filters them out afterwards, and the
/// freeze boundary counts over this list.
fn arranged_columns<'a>(
    spec: &'a ColumnTreeSpec<'_>,
    layout: &ColumnLayout,
) -> Vec<&'a ColumnSpec> {
    let mut out: Vec<&ColumnSpec> = Vec::new();
    for key in &layout.order {
        if let Some(column) = spec.columns.iter().find(|c| &c.key == key) {
            if !out.iter().any(|c| c.key == column.key) {
                out.push(column);
            }
        }
    }
    for column in spec.columns {
        if !out.iter().any(|c| c.key == column.key) {
            out.push(column);
        }
    }
    out
}

fn column_width(layout: &ColumnLayout, column: &ColumnSpec) -> f32 {
    layout
        .widths
        .get(&column.key)
        .copied()
        .unwrap_or(column.default_width)
        .max(MIN_COLUMN_WIDTH)
}

/// One pane's header: a heading per column (click = sort, drag = reorder,
/// right-click = the show/hide checklist) with a resize grip at each right
/// edge. The drop of a reorder drag is resolved by [`finish_reorder`] once
/// BOTH panes have contributed their bounds.
#[allow(clippy::too_many_arguments)]
fn header(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    layout: &mut ColumnLayout,
    visible: &[&ColumnSpec],
    widths: &[f32],
    width: f32,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut ColumnTreeOut,
    bounds: &mut Vec<(String, f32, f32)>,
) {
    let height = ui.spacing().interact_size.y;
    let (band, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let drag_key = egui::Id::new((spec.id, "column-tree-drag"));
    let dragging: Option<String> = ui.data(|d| d.get_temp(drag_key));

    let mut x = band.left();
    // The reorder DROP target is decided from the pointer's x at drag end, so
    // the boundaries are collected while the headings are laid out.
    let first = bounds.len();
    for (column, width) in visible.iter().zip(widths) {
        let cell = egui::Rect::from_min_size(egui::pos2(x, band.top()), egui::vec2(*width, height));
        bounds.push((column.key.clone(), cell.left(), cell.right()));

        let resp = ui.interact(
            cell,
            ui.id().with(("column-tree-head", spec.id, &column.key)),
            egui::Sense::click_and_drag(),
        );
        let held = dragging.as_deref() == Some(column.key.as_str());
        let fill = if held {
            ui.visuals().selection.bg_fill.gamma_multiply(0.45)
        } else if resp.hovered() {
            ui.visuals().widgets.hovered.bg_fill
        } else {
            ui.visuals().widgets.noninteractive.bg_fill
        };
        ui.painter().rect_filled(cell, 0.0, fill);
        // The sort marker rides the heading text, so the sorted column is
        // obvious without a second row of chrome.
        let marker = match &layout.sort {
            Some((key, true)) if key == &column.key => " \u{25B2}",
            Some((key, false)) if key == &column.key => " \u{25BC}",
            _ => "",
        };
        let text = format!("{}{marker}", column.label);
        ui.painter().text(
            egui::pos2(cell.left() + CELL_PAD, cell.center().y),
            egui::Align2::LEFT_CENTER,
            elide(ui, &text, *width - 2.0 * CELL_PAD),
            egui::TextStyle::Body.resolve(ui.style()),
            ui.visuals().strong_text_color(),
        );
        publish(hits, spec, &format!("col:{}", column.key), cell);

        // Right-click ANY heading → the show/hide checklist. Hiding lives here
        // rather than on a toolbar because the column is the thing being
        // hidden and this is where the user's hand already is.
        resp.context_menu(|ui| {
            ui.label(egui::RichText::new("Columns").strong());
            for candidate in spec.columns {
                let mut shown = !layout.hidden.contains(&candidate.key);
                if ui.checkbox(&mut shown, &candidate.label).changed() {
                    if shown {
                        layout.hidden.remove(&candidate.key);
                    } else {
                        layout.hidden.insert(candidate.key.clone());
                    }
                    out.layout_changed = true;
                }
            }
        });

        // A CLICK sorts. egui reports `clicked()` false once a press turns
        // into a drag, so the click and the reorder drag share the heading
        // without a mode.
        if resp.clicked() {
            layout.sort = match &layout.sort {
                Some((key, true)) if key == &column.key => Some((column.key.clone(), false)),
                Some((key, false)) if key == &column.key => None,
                _ => Some((column.key.clone(), true)),
            };
            out.layout_changed = true;
        }
        if resp.drag_started() {
            ui.data_mut(|d| d.insert_temp(drag_key, column.key.clone()));
        }

        ui.painter().line_segment(
            [
                egui::pos2(cell.right(), band.top()),
                egui::pos2(cell.right(), band.bottom()),
            ],
            ui.visuals().widgets.noninteractive.bg_stroke,
        );
        x = cell.right();
    }

    // The resize grips, in a SECOND pass. A grip straddles the divider, so it
    // overlaps the heading to its right — and egui hands an overlapped point to
    // the LAST widget registered there. Interleaved with the headings, every
    // grip would therefore lose its own hit to the next heading and no column
    // would ever resize.
    for (column, (_, _, right)) in visible.iter().zip(&bounds[first..]) {
        let grip_rect = egui::Rect::from_min_max(
            egui::pos2(right - GRIP * 0.5, band.top()),
            egui::pos2(right + GRIP * 0.5, band.bottom()),
        );
        let grip = ui.interact(
            grip_rect,
            ui.id().with(("column-tree-grip", spec.id, &column.key)),
            egui::Sense::drag(),
        );
        if grip.hovered() || grip.dragged() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        }
        if grip.dragged() {
            let next = (column_width(layout, column) + grip.drag_delta().x).max(MIN_COLUMN_WIDTH);
            layout.widths.insert(column.key.clone(), next);
            out.layout_changed = true;
        }
        publish(hits, spec, &format!("grip:{}", column.key), grip_rect);
    }
}

/// Finish a reorder: on release, the dragged column moves to whichever heading
/// the pointer is over — in EITHER pane, so dragging a column across the freeze
/// boundary is what freezes or unfreezes it.
fn finish_reorder(
    ui: &egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    layout: &mut ColumnLayout,
    arranged: &[&ColumnSpec],
    bounds: &[(String, f32, f32)],
    out: &mut ColumnTreeOut,
) {
    let drag_key = egui::Id::new((spec.id, "column-tree-drag"));
    let Some(held) = ui.data(|d| d.get_temp::<String>(drag_key)) else {
        return;
    };
    if ui.input(|i| i.pointer.any_down()) {
        return;
    }
    ui.data_mut(|d| d.remove::<String>(drag_key));
    let Some(pos) = ui.input(|i| i.pointer.latest_pos()) else {
        return;
    };
    if let Some((target, _, _)) = bounds
        .iter()
        .find(|(_, left, right)| pos.x >= *left && pos.x < *right)
    {
        if *target != held && move_column(layout, arranged, &held, target) {
            out.layout_changed = true;
        }
    }
}

/// Move `held` to `target`'s slot in the layout order, materializing the
/// current arranged order first so a layout that never named its columns still
/// reorders correctly. Returns whether anything moved.
fn move_column(
    layout: &mut ColumnLayout,
    arranged: &[&ColumnSpec],
    held: &str,
    target: &str,
) -> bool {
    let mut order: Vec<String> = layout.order.clone();
    // Seed from the drawn order so the first drag on a fresh layout is not a
    // no-op against an empty `order`.
    for column in arranged {
        if !order.iter().any(|key| key == &column.key) {
            order.push(column.key.clone());
        }
    }
    let Some(from) = order.iter().position(|key| key == held) else {
        return false;
    };
    let key = order.remove(from);
    let Some(to) = order.iter().position(|k| k == target) else {
        order.insert(from.min(order.len()), key);
        return false;
    };
    order.insert(to, key);
    layout.order = order;
    true
}

/// Draw one row and, when it is open, its children — the recursion that keeps
/// the rows a TREE.
#[allow(clippy::too_many_arguments)]
fn draw_subtree(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    layout: &ColumnLayout,
    visible: &[&ColumnSpec],
    widths: &[f32],
    offset: usize,
    band: f32,
    row: &RowNode,
    guides: &[bool],
    is_last: bool,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut ColumnTreeOut,
    pending: &mut Option<MenuOpen>,
) {
    draw_row(
        ui, spec, visible, widths, offset, band, row, guides, is_last, false, hits, out, pending,
    );
    if !row.expanded || row.children.is_empty() {
        return;
    }
    let child_guides = tree::child_guides(guides, is_last);
    let ordered = sorted_siblings(&row.children, layout);
    let last = ordered.len();
    for (index, child) in ordered.iter().enumerate() {
        draw_subtree(
            ui,
            spec,
            layout,
            visible,
            widths,
            offset,
            band,
            child,
            &child_guides,
            index + 1 == last,
            hits,
            out,
            pending,
        );
    }
}

/// Sort ONE level of siblings by the layout's sort column. Sorting is
/// per-level so the nesting survives it — a child never overtakes its parent.
/// A stable sort, so an unsorted-equal run keeps the consumer's order.
fn sorted_siblings<'a>(rows: &'a [RowNode], layout: &ColumnLayout) -> Vec<&'a RowNode> {
    let mut out: Vec<&RowNode> = rows.iter().collect();
    if let Some((key, ascending)) = &layout.sort {
        out.sort_by(|a, b| {
            let ordering = compare_cells(a.cells.get(key), b.cells.get(key));
            if *ascending {
                ordering
            } else {
                ordering.reverse()
            }
        });
    }
    out
}

/// Order two cell values: numbers numerically, everything else by its display
/// text case-insensitively, and an ABSENT/null cell last in ascending order
/// (an unfilled cell is not a small value, it is a missing one).
fn compare_cells(a: Option<&Value>, b: Option<&Value>) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    let empty = |value: Option<&Value>| match value {
        None | Some(Value::Null) => true,
        Some(Value::String(text)) => text.is_empty(),
        _ => false,
    };
    match (empty(a), empty(b)) {
        (true, true) => return Ordering::Equal,
        (true, false) => return Ordering::Greater,
        (false, true) => return Ordering::Less,
        (false, false) => {}
    }
    if let (Some(Value::Number(x)), Some(Value::Number(y))) = (a, b) {
        if let (Some(x), Some(y)) = (x.as_f64(), y.as_f64()) {
            return x.partial_cmp(&y).unwrap_or(Ordering::Equal);
        }
    }
    display_text(a).to_lowercase().cmp(&display_text(b).to_lowercase())
}

/// A cell value as the text a cell shows: a string bare (not JSON-quoted), a
/// number in its shortest form, anything else as its JSON.
fn display_text(value: Option<&Value>) -> String {
    match value {
        None | Some(Value::Null) => String::new(),
        Some(Value::String(text)) => text.clone(),
        Some(other) => other.to_string(),
    }
}

/// Draw ONE row: the tree cell in column 0, an editor in each other column.
#[allow(clippy::too_many_arguments)]
fn draw_row(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    visible: &[&ColumnSpec],
    widths: &[f32],
    offset: usize,
    band_width: f32,
    row: &RowNode,
    guides: &[bool],
    is_last: bool,
    root: bool,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut ColumnTreeOut,
    pending: &mut Option<MenuOpen>,
) {
    let height = ui.spacing().interact_size.y;
    // THE ROW BAND — the row's click target, allocated BEFORE the cells. A row
    // is picked by clicking anywhere along it, not only on the text in its
    // first column: in the shipped BOM that text was 40 pt of a 900 pt row, so
    // 95 % of every row was dead. egui hands an overlapped point to the LAST
    // widget registered there, and every cell editor is registered after this,
    // so a button, a checkbox, a text cell and the collapse box all still take
    // their own clicks — the band receives only what none of them wanted.
    //
    // It does NOT take the keyboard focus: the row's Tab stop is the band
    // inside `tree::node` (drawn once, in whichever pane holds column 0), so a
    // frozen table does not give one row two stops.
    let (band, band_resp) =
        ui.allocate_exact_size(egui::vec2(band_width, height), egui::Sense::CLICK);
    let clip = ui.clip_rect();

    // TRIGGER 2 — a right-click ANYWHERE on the row. Read off the raw pointer
    // rather than from a `Response`, because the cell editors are registered
    // AFTER this band and egui hands an overlapped point to the LAST widget
    // registered there: a `context_menu` on the band would silently never fire
    // over a text cell. Reading the position also leaves the band's own primary
    // click alone, so the select path above and this one cannot fight.
    let over_row = band.intersect(clip);
    if !root && over_row.is_positive() && ui.rect_contains_pointer(over_row) {
        out.hovered = Some(row.id.clone());
    }

    // The row's published rect IS its band — what a script clicks has to be
    // what a user can click, and that is the strip, not the name in it.
    //
    // A FROZEN table draws the row once per pane, and only the pane holding
    // column 0 publishes: `click_widget` aims at a rect's CENTRE, and the
    // centre of the two bands unioned lands in whichever scrolling column
    // happens to be halfway across — a text editor, a drag-value — where a
    // click focuses that editor instead of picking the row. The frozen band
    // leads with the tree column, so its centre is the row. The scrolling
    // columns are still clickable; they are just not what the key aims at.
    if !root && offset == 0 && over_row.is_positive() {
        publish(hits, spec, &format!("row:{}", row.id), over_row);
    }
    // A SELECT click anywhere on the band that no cell claimed. The tree cell
    // reports its own (see below); this is every other column.
    if !root && band_resp.clicked() {
        out.clicked = Some(row.id.clone());
    }
    if !root && tree::is_double_click(&band_resp) {
        out.double_clicked = Some(row.id.clone());
    }
    if !row.actions.is_empty()
        && over_row.is_positive()
        && ui.input(|i| i.pointer.secondary_clicked())
    {
        if let Some(pos) = ui.ctx().input(|i| i.pointer.interact_pos()) {
            // ...and nothing floating (an open menu, the header's own column
            // checklist) is above that point.
            let above = ui.ctx().layer_id_at(pos);
            let ours = above.is_none() || above == Some(ui.layer_id());
            if over_row.contains(pos) && ours {
                *pending = Some(MenuOpen {
                    row: row.id.clone(),
                    pos,
                });
            }
        }
    }

    // The SELECTION band. A selected row is emphasised across its whole width,
    // not just by bolding the tree cell's text: this table is as wide as its
    // configured columns, and a picked component has to be findable at a
    // glance from anywhere in it. Painted BEFORE the cells so every editor
    // draws over it, and drawn in both panes (each calls this with its own
    // band) so the highlight does not stop at the frozen boundary.
    if row.selected && !root {
        let visible_band = band.intersect(clip);
        if visible_band.is_positive() {
            ui.painter().rect_filled(
                visible_band,
                0.0,
                ui.visuals().selection.bg_fill.gamma_multiply(0.35),
            );
        }
    }

    let mut x = band.left();
    for (index, (column, width)) in visible.iter().zip(widths).enumerate() {
        let cell = egui::Rect::from_min_size(egui::pos2(x, band.top()), egui::vec2(*width, height));
        x = cell.right();
        let Some(cell_clip) = cell.intersect(clip).is_positive().then_some(cell.intersect(clip))
        else {
            continue;
        };
        let mut child = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(cell.shrink2(egui::vec2(CELL_PAD, 0.0)))
                .layout(egui::Layout::left_to_right(egui::Align::Center))
                .id_salt(("column-tree-cell", spec.id, &row.id, &column.key)),
        );
        child.set_clip_rect(cell_clip);

        if offset + index == 0 {
            // The TREE cell — drawn by `panels::tree::node` itself, so the
            // connector rules and collapse boxes are the shared ones.
            let label = display_text(row.cells.get(&column.key));
            let expandable = !row.children.is_empty();
            let mut tree_row = TreeRow {
                guides,
                is_last,
                expandable,
                expanded: row.expanded,
                root,
                glyph: None,
                label: &label,
                selected: row.selected,
                highlighted: false,
                draggable: false,
                tint: None,
            };
            if root {
                tree_row.expandable = true;
                tree_row.expanded = true;
            }
            let resp = tree::node(&mut child, tree_row, |_| {});
            publish(hits, spec, &format!("box:{}", row.id), resp.box_rect);
            if resp.toggled {
                out.toggled = Some(row.id.clone());
            }
            // The tree cell has a band of its OWN (`tree::node` draws one), and
            // it is registered after this row's, so inside column 0 it is the
            // one that hears a blank-space click. Both are the same intent.
            if !root && resp.clicked() {
                out.clicked = Some(row.id.clone());
            }
            if !root && resp.double_clicked() {
                out.double_clicked = Some(row.id.clone());
            }
        } else {
            let rect = cell_editor(&mut child, spec, row, column, out, pending, hits);
            publish(
                hits,
                spec,
                &format!("cell:{}:{}", row.id, column.key),
                rect,
            );
            if matches!(column.kind, CellKind::Actions { .. }) {
                publish(hits, spec, &format!("menu:{}", row.id), rect);
            }
        }
    }
}

/// Draw ONE non-tree cell's editor and return its rect. A non-editable row
/// still SHOWS its value and still offers its buttons and its action menu — it
/// just cannot be typed into.
fn cell_editor(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    row: &RowNode,
    column: &ColumnSpec,
    out: &mut ColumnTreeOut,
    pending: &mut Option<MenuOpen>,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
) -> egui::Rect {
    let value = row.cells.get(&column.key);
    let width = ui.available_width();
    let mut emit = |new_value: Value| {
        out.edits.push(CellEdit {
            row_id: row.id.clone(),
            column: column.key.clone(),
            value: new_value,
        });
    };

    match &column.kind {
        CellKind::Button { label } => {
            let button = ui.add_sized([width, ui.available_height()], egui::Button::new(label));
            if button.clicked() {
                out.buttons.push(CellClick {
                    row_id: row.id.clone(),
                    column: column.key.clone(),
                });
            }
            button.rect
        }
        CellKind::Actions { label } => {
            // TRIGGER 1 — the actions cell. Like the right-click it only
            // RECORDS which row was asked for; the menu is drawn once, after
            // every row, from that record. A row that offers nothing draws the
            // trigger greyed rather than dropping it, so the column stays a
            // column.
            let offered = !row.actions.is_empty();
            let button = ui
                .add_enabled_ui(offered, |ui| {
                    ui.add_sized([width, ui.available_height()], egui::Button::new(label))
                })
                .inner;
            if button.clicked() {
                *pending = Some(MenuOpen {
                    row: row.id.clone(),
                    pos: button.rect.left_bottom(),
                });
            }
            button.rect
        }
        // A DISPLAY cell: not selectable, so the click reaches the row band
        // behind it. egui labels are drag-to-select by default, which senses
        // click and drag and would make a derived value the one piece of the
        // row you cannot pick the row by.
        CellKind::ReadOnly => ui
            .add(
                egui::Label::new(display_text(value))
                    .selectable(false)
                    .truncate(),
            )
            .rect,
        CellKind::Badges => {
            let badges = value.and_then(Value::as_array).cloned().unwrap_or_default();
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 3.0;
                for badge in &badges {
                    let glyph = badge.get("glyph").and_then(Value::as_str).unwrap_or("");
                    if glyph.is_empty() {
                        continue;
                    }
                    let color = badge
                        .get("color")
                        .and_then(Value::as_str)
                        .and_then(parse_hex_color)
                        .unwrap_or_else(|| ui.visuals().text_color());
                    // A badge glyph carries meaning IN its colour, so it is
                    // drawn from the icon catalog and tinted — there is no icon
                    // font to render the character with.
                    // A badge may name an action; status-only badges remain
                    // passive and continue to let the row receive the click.
                    if let Some(action) = badge.get("action").and_then(Value::as_str) {
                        let button = ui.add(crate::icon_text::icon_button_colored(ui, glyph, Some(color)).small().frame(false));
                        publish(hits, spec, &format!("badge:{}:{action}", row.id), button.rect);
                        if button.clicked() {
                            out.actions.push(RowActionClick { row_id: row.id.clone(), action: action.into() });
                        }
                        if let Some(tip) = badge.get("tooltip").and_then(Value::as_str) {
                            button.on_hover_text(tip);
                        }
                        continue;
                    }
                    let label = match crate::icon_text::glyph(ui, glyph, color) {
                        Some(art) => ui.add(art),
                        // …and the text fallback is not selectable either, for
                        // the same reason the read-only cell is not.
                        None => ui.add(
                            egui::Label::new(egui::RichText::new(glyph).color(color))
                                .selectable(false),
                        ),
                    };
                    if let Some(tip) = badge.get("tooltip").and_then(Value::as_str) {
                        label.on_hover_text(tip);
                    }
                }
            })
            .response
            .rect
        }
        CellKind::Toggle => {
            // Drawn for every row, clickable only on an editable one: the
            // weak-label fallback below would render a bool as the text
            // "false", which is not a checkbox.
            let mut on = value.and_then(Value::as_bool).unwrap_or(false);
            let box_ = ui
                .add_enabled_ui(row.editable, |ui| ui.checkbox(&mut on, ""))
                .inner;
            if box_.changed() {
                emit(Value::Bool(on));
            }
            box_.rect
        }
        _ if !row.editable => ui
            .add(
                egui::Label::new(egui::RichText::new(display_text(value)).weak())
                    .selectable(false)
                    .truncate(),
            )
            .rect,
        CellKind::Text | CellKind::Numeric { .. } | CellKind::Choice { .. } => {
            // The VALUE editors are shared (see [`value_editor`]) — the table
            // and the part-properties dialog must agree on what editing a
            // `Text` / `Numeric` / `Choice` attribute feels like, because they
            // edit the very same stored records.
            let salt = egui::Id::new(("column-tree-cell", spec.id, &row.id, &column.key));
            let size = egui::vec2(width, ui.available_height());
            let (edited, rect) = value_editor(ui, salt, &column.kind, value, size);
            if let Some(new_value) = edited {
                emit(new_value);
            }
            rect
        }
    }
}

/// Draw the editor for ONE value of a [`CellKind`] and report the committed
/// edit, if any. `salt` seeds the per-widget egui ids (an in-progress text
/// buffer, a combo's open state), so it must be stable for a given value and
/// distinct between values; `size` is the space to fill.
///
/// Split out of [`cell_editor`] so a consumer that is NOT a table can use the
/// same editors: the part-properties dialog edits the same BOM attribute
/// records the BOM table's cells do, and a `Choice` that could be cleared in
/// one place but not the other, or a text field that committed per keystroke
/// in one and on focus-loss in the other, would be one attribute with two
/// behaviours. The kinds that only make sense inside a table row (`Button`,
/// `Actions`, `Badges`, `Toggle`) stay in [`cell_editor`]; asking for one here
/// draws the value read-only.
pub fn value_editor(
    ui: &mut egui::Ui,
    salt: egui::Id,
    kind: &CellKind,
    value: Option<&Value>,
    size: egui::Vec2,
) -> (Option<Value>, egui::Rect) {
    match kind {
        CellKind::Text => {
            // An in-progress edit lives in egui memory and commits on
            // focus-loss / Enter. Per-keystroke commits are wrong here: a
            // consumer's commit can be arbitrarily expensive (the BOM's
            // part-level one re-signs a part document and re-heals every
            // instance of it), and a half-typed value is not a value.
            let buffer_id = salt.with("text");
            let stored = display_text(value);
            let mut buffer: String =
                ui.data(|d| d.get_temp(buffer_id)).unwrap_or_else(|| stored.clone());
            let edit = ui.add_sized(size, egui::TextEdit::singleline(&mut buffer));
            let entered = edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            let mut committed = None;
            if edit.has_focus() || edit.changed() {
                ui.data_mut(|d| d.insert_temp(buffer_id, buffer.clone()));
            }
            if edit.lost_focus() || entered {
                ui.data_mut(|d| d.remove::<String>(buffer_id));
                if buffer != stored {
                    committed = Some(Value::String(buffer));
                }
            } else if !edit.has_focus() {
                // Unfocused: track the model, so an edit made elsewhere (a
                // packed fan-out landing on a sibling row) shows immediately.
                ui.data_mut(|d| d.remove::<String>(buffer_id));
            }
            (committed, edit.rect)
        }
        CellKind::Numeric { step } => {
            let mut number = value.and_then(Value::as_f64).unwrap_or(0.0);
            let drag = ui.add_sized(size, egui::DragValue::new(&mut number).speed(*step));
            let committed = drag.changed().then(|| serde_json::json!(number));
            (committed, drag.rect)
        }
        CellKind::Choice { options } => {
            let current = display_text(value);
            let mut chosen = current.clone();
            let combo = egui::ComboBox::from_id_salt(salt.with("combo"))
                .width(size.x)
                .selected_text(if current.is_empty() { "—" } else { &current })
                .show_ui(ui, |ui| {
                    // The blank choice is always offered: without it a dropdown
                    // cell can be set but never cleared.
                    ui.selectable_value(&mut chosen, String::new(), "—");
                    for option in options {
                        ui.selectable_value(&mut chosen, option.clone(), option);
                    }
                });
            let committed = (chosen != current).then(|| Value::String(chosen));
            (committed, combo.response.rect)
        }
        _ => {
            let label = ui.add(egui::Label::new(display_text(value)).truncate());
            (None, label.rect)
        }
    }
}

/// Which row's action menu is open, and the point it was opened at. Lives in
/// egui memory keyed by [`ColumnTreeSpec::id`] — one record, so there can only
/// ever be one open menu per tree.
#[derive(Clone)]
struct MenuOpen {
    row: String,
    pos: egui::Pos2,
}

/// THE action menu — one definition, drawn from whichever trigger recorded a
/// [`MenuOpen`]. Neither trigger renders anything itself, so the cell click and
/// the right-click cannot drift apart: there is only one menu to drift.
fn row_action_menu(
    ui: &mut egui::Ui,
    spec: &ColumnTreeSpec<'_>,
    rows: &[RowNode],
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut ColumnTreeOut,
    pending: &mut Option<MenuOpen>,
) {
    let key = egui::Id::new((spec.id, "column-tree-menu"));
    let mut open: Option<MenuOpen> = ui.data(|d| d.get_temp(key));
    let was_open = open.as_ref().map(|state| state.row.clone());

    if let Some(state) = open.clone() {
        // A row that has gone (or lost every action) takes its menu with it.
        match find_row(rows, &state.row) {
            Some(row) if !row.actions.is_empty() => {
                let (chosen, still_open) = action_menu(
                    ui.ctx(),
                    ui.layer_id(),
                    key.with("popup"),
                    state.pos,
                    &row.actions,
                    |action, rect| {
                        publish(hits, spec, &format!("menuitem:{}:{}", row.id, action.id), rect);
                    },
                );
                if let Some(action) = chosen {
                    out.actions.push(RowActionClick { row_id: row.id.clone(), action });
                }
                if !still_open {
                    open = None;
                }
            }
            _ => open = None,
        }
    }

    // Applied AFTER the draw (see the call site): applying it first would let
    // the popup's own close-on-click see the very click that opened it. A
    // trigger fired on the row whose menu just closed itself is a TOGGLE — the
    // second click on the same trigger shuts it.
    if let Some(next) = pending.take() {
        let toggled_off = open.is_none() && was_open.as_deref() == Some(next.row.as_str());
        open = (!toggled_off).then_some(next);
    }
    match &open {
        Some(state) => ui.data_mut(|d| {
            d.insert_temp(key, state.clone());
        }),
        None => ui.data_mut(|d| d.remove::<MenuOpen>(key)),
    }
}

/// Draw ONE action menu as a popup at `pos` — the body of a row's menu, and
/// the menu an object drawn somewhere else opens for that same row: a placed
/// view or a sheet dimension right-clicked on the paper, a PMI label chip
/// right-clicked in the 3D view. Every one of them draws its entries HERE, so
/// the object's menu and its tree row's cannot drift apart.
///
/// `on_entry` receives each entry's screen rect as it is drawn (the caller's
/// `menuitem:` hit key). Returns the id of the entry chosen this frame and
/// whether the popup is still open — it closes on any click, on an entry or
/// elsewhere, and on Escape.
pub fn action_menu(
    ctx: &egui::Context,
    layer: egui::LayerId,
    id: egui::Id,
    pos: egui::Pos2,
    actions: &[RowAction],
    mut on_entry: impl FnMut(&RowAction, egui::Rect),
) -> (Option<String>, bool) {
    let mut still_open = true;
    let mut chosen: Option<String> = None;
    egui::Popup::new(id, ctx.clone(), egui::PopupAnchor::Position(pos), layer)
        .open_bool(&mut still_open)
        .kind(egui::PopupKind::Menu)
        .layout(egui::Layout::top_down_justified(egui::Align::Min))
        .width(160.0)
        .show(|ui| {
            for action in actions {
                if action.separator_above {
                    ui.separator();
                }
                // The caption may lead with a catalogued glyph (🔒 Fix,
                // ✖ Delete); it is drawn as artwork, not as a character. A
                // destructive entry keeps its red — on the artwork as well as
                // the text.
                let color = action.destructive.then(|| ui.visuals().error_fg_color);
                // An entry never wraps. The popup's `Area` keeps the width it
                // measured the last time it was up, so without this a menu
                // opened after a narrower one broke its longer entries over
                // two lines ("Remove from / sheet" after a dimension's menu).
                let button = crate::icon_text::icon_button_colored(ui, &action.label, color)
                    .wrap_mode(egui::TextWrapMode::Extend);
                let entry = ui.add_enabled(action.enabled, button);
                on_entry(action, entry.rect);
                if !action.tooltip.is_empty() {
                    // A greyed entry's tooltip is where its refusal is
                    // explained, so it has to be the DISABLED hover.
                    if action.enabled {
                        entry.clone().on_hover_text(&action.tooltip);
                    } else {
                        entry.clone().on_disabled_hover_text(&action.tooltip);
                    }
                }
                if entry.clicked() {
                    chosen = Some(action.id.clone());
                }
            }
        });
    (chosen, still_open)
}

/// The row with `id`, anywhere in the tree.
fn find_row<'a>(rows: &'a [RowNode], id: &str) -> Option<&'a RowNode> {
    for row in rows {
        if row.id == id {
            return Some(row);
        }
        if let Some(found) = find_row(&row.children, id) {
            return Some(found);
        }
    }
    None
}

/// Truncate `text` with an ellipsis so it fits `width`.
fn elide(ui: &egui::Ui, text: &str, width: f32) -> String {
    let font = egui::TextStyle::Body.resolve(ui.style());
    let measure = |candidate: &str| {
        ui.painter()
            .layout_no_wrap(candidate.to_string(), font.clone(), egui::Color32::WHITE)
            .rect
            .width()
    };
    if width <= 0.0 || measure(text) <= width {
        return text.to_string();
    }
    let mut cut: Vec<char> = text.chars().collect();
    while !cut.is_empty() {
        cut.pop();
        let candidate: String = cut.iter().collect::<String>() + "\u{2026}";
        if measure(&candidate) <= width {
            return candidate;
        }
    }
    String::new()
}

fn publish(
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    spec: &ColumnTreeSpec<'_>,
    key: &str,
    rect: egui::Rect,
) {
    if let Some(map) = hits.as_deref_mut() {
        map.insert(format!("{}{key}", spec.hits_prefix), rect);
    }
}


