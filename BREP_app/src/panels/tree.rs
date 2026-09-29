//! A reusable, custom-painted TREE-NODE widget — the shared building block for
//! the engine-native side panels (the history feature tree here; the Scene tree
//! and others reuse it verbatim). egui's default `CollapsingHeader` draws a
//! disclosure TRIANGLE and no connector rules; the design reference is a classic
//! file-tree with **`[+]`/`[-]` collapse boxes + connector lines**, so this
//! module paints those itself.
//!
//! # The node model (immutable-mode friendly)
//!
//! A node is ONE row: `⟨connectors⟩ [+/-] ⟨glyph⟩ ⟨label⟩ … ⟨right content⟩`.
//! Expansion state is OWNED BY THE CALLER (the panels need exclusive-expand +
//! roll-to side effects), so [`node`] just draws a row and returns what was
//! clicked; the caller decides whether to recurse into children.
//!
//! ## Connector geometry
//!
//! Indentation is one [`INDENT`] column per tree level. A node at connector
//! column `d = guides.len()` draws its `├`/`└` connector at column `d` and its
//! box slot at column `d + 1`. `guides[i]` (`i < d`) says whether an ANCESTOR's
//! sibling line passes vertically through this row at column `i`. Descending into
//! a node's children extends the guide stack by [`child_guides`] with
//! `!this_node_is_last` — the node's own connector line continues down past its
//! children only if it has following siblings. This is the standard file-tree
//! rule and yields the reference's exact rules.
//!
//! ## Select and open
//!
//! Every tree reads a row the same way: a single click SELECTS the row's object
//! ([`NodeResponse::clicked`]) and a double click OPENS its dialog
//! ([`NodeResponse::double_clicked`]). A selected row is drawn with the
//! selection band behind its whole width ([`TreeRow::highlighted`]), the look
//! the column trees give theirs.
//!
//! This module has NO engine dependency — it is pure egui + geometry, so a later
//! `brep-ui` crate (and the theme pass, #49) can reuse it unchanged.

use crate::icon_text::IconText;
use eframe::egui;

/// One tree indentation level, in egui points.
pub const INDENT: f32 = 16.0;
/// The collapse box's side length.
const BOX: f32 = 13.0;
/// Gap between the box slot, an optional glyph, and the label.
const GAP: f32 = 4.0;
/// Right-side gutter reserved before the right-aligned row content (timing,
/// delete X, field inputs, a Select button) so those controls clear the sidebar
/// scroll bar instead of underlapping it — an underlapping X eats the click as a
/// scroll drag rather than firing.
const RIGHT_PAD: f32 = 10.0;

/// The spec for one tree row. Borrows its strings; holds no state (expansion is
/// the caller's — see the module docs).
pub struct TreeRow<'a> {
    /// Ancestor vertical-guide flags, one per level strictly above this node
    /// (`true` = a sibling line passes through this row at that level). The
    /// node's own connector column index is `guides.len()`.
    pub guides: &'a [bool],
    /// This node is the LAST among its siblings (`└` vs `├`). Ignored when `root`.
    pub is_last: bool,
    /// Draw a `[+]`/`[-]` collapse box (an expandable node) instead of an empty
    /// box slot (a leaf). A leaf keeps the same label indent so labels align.
    pub expandable: bool,
    /// Current expand state — only meaningful when `expandable`.
    pub expanded: bool,
    /// The far-left ROOT row (e.g. `Features`): no connector, box at column 0.
    pub root: bool,
    /// An optional per-type glyph drawn between the box slot and the label.
    pub glyph: Option<&'a str>,
    /// The row's text label.
    pub label: &'a str,
    /// Emphasize the label (the selected node).
    pub selected: bool,
    /// Paint the selection band behind the whole row — the selected LINE.
    /// [`Self::selected`] sets it with the emphasis; a column tree, which paints
    /// its own band across every column, draws its tree cell without it.
    pub highlighted: bool,
    /// Make the label a drag handle (`Sense::click_and_drag`) — drag-reorder.
    pub draggable: bool,
    /// Override the label's text color (the sketch panel paints a CONFLICTING
    /// constraint's row red). `None` keeps the theme's text color, including its
    /// hover/selection behavior.
    pub tint: Option<egui::Color32>,
}

impl<'a> TreeRow<'a> {
    /// A plain expandable branch node.
    pub fn branch(guides: &'a [bool], is_last: bool, expanded: bool, label: &'a str) -> Self {
        Self {
            guides,
            is_last,
            expandable: true,
            expanded,
            root: false,
            glyph: None,
            label,
            selected: false,
            highlighted: false,
            draggable: false,
            tint: None,
        }
    }

    /// A plain leaf node (no collapse box).
    pub fn leaf(guides: &'a [bool], is_last: bool, label: &'a str) -> Self {
        Self {
            guides,
            is_last,
            expandable: false,
            expanded: false,
            root: false,
            glyph: None,
            label,
            selected: false,
            highlighted: false,
            draggable: false,
            tint: None,
        }
    }

    pub fn glyph(mut self, glyph: Option<&'a str>) -> Self {
        self.glyph = glyph;
        self
    }
    /// Mark the row selected: its label emphasized and its line highlighted.
    pub fn selected(mut self, selected: bool) -> Self {
        self.selected = selected;
        self.highlighted = selected;
        self
    }
    pub fn draggable(mut self, draggable: bool) -> Self {
        self.draggable = draggable;
        self
    }
    pub fn tint(mut self, tint: Option<egui::Color32>) -> Self {
        self.tint = tint;
        self
    }
}

/// What happened to a drawn tree row.
pub struct NodeResponse {
    /// The whole row rect (for hit publishing / drag-target hit-testing).
    pub row_rect: egui::Rect,
    /// The collapse-box rect (for hit publishing).
    pub box_rect: egui::Rect,
    /// The `[+]`/`[-]` collapse box was clicked (a pure toggle intent).
    pub toggled: bool,
    /// The label response — the DRAG handle and the hover source. Use
    /// `.drag_started()` / `.drag_stopped()` to reorder and `.hovered()` for a
    /// text-keyed highlight. For "the row was clicked" use [`Self::clicked`],
    /// not this: the label's rect is only its own text.
    pub label: egui::Response,
    /// The whole ROW BAND, sensing clicks BELOW everything drawn in the row.
    /// egui hands an overlapped point to the last widget registered there and
    /// this one is registered first (`UiBuilder::sense`), so the collapse box,
    /// the label and every control the caller put in `add_right` still take
    /// their own clicks — the band only ever receives what none of them wanted,
    /// which is exactly the blank space between them.
    ///
    /// It is also the row's KEYBOARD stop: the label is deliberately not
    /// focusable, so a row is one Tab stop and Enter/Space on it reports
    /// `clicked()` here.
    ///
    /// It senses DRAG as well as click, and it has to: egui withdraws the click
    /// from a press that outlives `max_click_duration` (0.8 s) or drifts past
    /// `max_click_dist` (6 pt) whether or not the widget senses drag, so a
    /// click-only band would go dead under exactly the slow, wobbly press an
    /// ordinary hand makes. A caller resolves that press through
    /// [`Self::drag_started`], as the history and constraint trees do.
    pub band: egui::Response,
    /// The BAND's drag started, and its press did not begin on one of the row's
    /// own controls (the collapse box, anything `add_right` drew). Read through
    /// [`Self::drag_started`].
    pub band_drag_started: bool,
}

impl NodeResponse {
    /// The ROW was clicked — on its text or anywhere else in its band that no
    /// control of its own claimed. This, not `label.clicked()`, is what
    /// "the user picked this row" means: a row is a click target across its
    /// whole width, and a label's rect is only as wide as its text. A click
    /// SELECTS the row's object.
    pub fn clicked(&self) -> bool {
        self.label.clicked() || self.band.clicked()
    }

    /// The ROW was double-clicked, over the same area as [`Self::clicked`]. A
    /// double click OPENS the row's dialog. egui reports the second press as a
    /// click as well, so on this frame [`Self::clicked`] is true too: a
    /// consumer checks this first.
    ///
    /// egui's TRIPLE click counts. egui numbers a click 3 when the click two
    /// before it is under 0.6 s old, and a 3 is not a 2 — so a row clicked once
    /// and then double-clicked a moment later would otherwise never open.
    pub fn double_clicked(&self) -> bool {
        is_double_click(&self.label) || is_double_click(&self.band)
    }
}

/// A response's double click, egui's triple included — see
/// [`NodeResponse::double_clicked`]. The column trees read their row bands
/// through this too.
pub fn is_double_click(response: &egui::Response) -> bool {
    response.double_clicked() || response.triple_clicked()
}

impl NodeResponse {

    /// A press on the row turned into a DRAG — on the label or anywhere else
    /// along the band. A pane that reorders by drag arms itself from this; a
    /// pane that does not still wants it, because a press egui reclassified as
    /// a drag is the click it refused to report, and resolving it is the only
    /// way a slow click on a row still opens that row.
    pub fn drag_started(&self) -> bool {
        self.label.drag_started() || self.band_drag_started
    }
}

/// Extend a guide stack for a node's children: the node's own connector column
/// keeps drawing a vertical line past the children ONLY if the node has more
/// siblings below it (`!is_last`).
pub fn child_guides(guides: &[bool], is_last: bool) -> Vec<bool> {
    let mut next = guides.to_vec();
    next.push(!is_last);
    next
}

/// Draw ONE tree row and return what was interacted with. `add_right` fills the
/// right-aligned content area (timing + delete, field inputs, a Select button…).
///
/// The whole row is a click target: read [`NodeResponse::clicked`], which is the
/// band and the label together. [`NodeResponse::label`] remains the reorder
/// drag handle and the source of a text-keyed hover.
pub fn node(
    ui: &mut egui::Ui,
    row: TreeRow,
    add_right: impl FnOnce(&mut egui::Ui),
) -> NodeResponse {
    let connector_col = if row.root { 0 } else { row.guides.len() };
    // The box slot (and thus the label) begins one column right of the connector.
    let indent = if row.root {
        0.0
    } else {
        (connector_col as f32 + 1.0) * INDENT
    };

    let mut box_rect = egui::Rect::NOTHING;
    let mut toggled = false;
    let mut label_resp: Option<egui::Response> = None;
    // The selection band's slot, reserved BEFORE the row so the row's contents
    // paint over it; filled once the row's rect is known.
    let highlight = ui.painter().add(egui::Shape::Noop);
    // The row's OWN controls — the collapse box and whatever `add_right` drew.
    // A press that starts on one of those belongs to it, never to the band.
    let mut controls = egui::Rect::NOTHING;

    // THE ROW BAND. A row is a click target across its whole width, not only
    // across its text — so the row is drawn inside a `Ui` that senses clicks
    // itself. `UiBuilder::sense` registers that rect BEFORE the row's contents
    // and updates it in place when the scope closes, and egui's hit test hands
    // an overlapped point to the LAST widget registered there, so every control
    // inside the row (the collapse box, the label, whatever `add_right` draws)
    // still takes its own clicks and the band receives only what none of them
    // wanted. Allocating it AFTER the row instead would invert that and the
    // band would eat the delete button.
    let outer = ui.scope_builder(
        egui::UiBuilder::new().sense(egui::Sense::click_and_drag()),
        |ui| {
            ui.horizontal(|ui| {
                // Zero the inter-item spacing so the geometry math is exact; gaps are
                // added explicitly below.
                ui.spacing_mut().item_spacing.x = 0.0;
                if indent > 0.0 {
                    ui.add_space(indent);
                }

                // --- collapse box (expandable) or an equally-wide empty slot (leaf) ----
                let (brect, bresp) = ui.allocate_exact_size(
                    egui::vec2(BOX, BOX),
                    if row.expandable {
                        egui::Sense::click()
                    } else {
                        egui::Sense::hover()
                    },
                );
                box_rect = brect;
                if row.expandable {
                    paint_box(ui, brect, row.expanded);
                    if bresp.clicked() {
                        toggled = true;
                    }
                }
                ui.add_space(GAP);

                // --- optional per-type glyph ------------------------------------------
                // Drawn as catalogued artwork, sized to the row's text. Monochrome
                // artwork is tinted to the row's text colour, so it reads exactly as the
                // old font glyph did; colour artwork is left alone. Anything
                // uncatalogued falls back to text.
                if let Some(glyph) = row.glyph {
                    match crate::icons::artwork(glyph) {
                        Some(icon) => {
                            // A tree row can be the first thing on screen to draw an
                            // SVG; `install_image_loaders` is idempotent.
                            egui_extras::install_image_loaders(ui.ctx());
                            let height = ui.text_style_height(&egui::TextStyle::Body);
                            let mut art = crate::icon_text::image(icon, height);
                            if icon.mono {
                                art = art.tint(ui.visuals().text_color());
                            }
                            ui.add(art);
                        }
                        None => {
                            ui.label(egui::RichText::new(glyph).color(ui.visuals().text_color()));
                        }
                    }
                    ui.add_space(GAP);
                }

                // --- label (the click / drag handle) ----------------------------------
                let mut text = egui::RichText::new(row.label);
                if row.selected {
                    text = text.strong();
                }
                if let Some(tint) = row.tint {
                    text = text.color(tint);
                }
                // Click (and, on a reorder handle, drag) but NOT focus: the row's Tab
                // stop is the band, which covers the whole row rather than just this
                // text. Two focusable widgets per row would make Tab visit every row
                // twice and put the keyboard's idea of "the row" on its label.
                let sense = if row.draggable {
                    egui::Sense::CLICK | egui::Sense::DRAG
                } else {
                    egui::Sense::CLICK
                };
                // A DRAGGABLE row is a reorder handle, so its label must NOT be
                // text-selectable: egui labels are selectable by default, and a press on a
                // selectable label anchors a text selection that the ensuing reorder drag
                // then SWEEPS across every row the cursor passes over (the passed-over rows
                // "light up"). Making only the drag handle non-selectable means the reorder
                // press never anchors a selection, so no sweep — while every non-draggable
                // row (fields, groups, scene/settings leaves) keeps default selectable text.
                // Hover coloring is unaffected: it is computed from the response independent
                // of `selectable` (see egui `Label`), so ordinary non-drag hover is unchanged.
                // A label can carry catalogued glyphs of its own — the sketch panel
                // appends ⛓ / ◐ / ⏚ marks to entity rows — and with no icon font left,
                // drawing those as text would render a box. `icon_label` draws them from
                // the same SVG catalog the glyph column uses.
                //
                // A DRAGGABLE row keeps the plain `Label`: it is the reorder handle, and
                // it must stay ONE widget whose response is the drag. Nothing needs
                // both — a draggable row's glyph goes in the glyph column above, not
                // inline — and `has_icon` is false for those labels, so this only ever
                // takes the icon path for the rows that need it.
                let has_icon = row.label.chars().any(crate::icons::has);

                // --- right-aligned content, THEN the label ----------------------------
                // The right content is allocated FIRST, from the panel's right edge, and
                // the label truncates into whatever it leaves — the order the form's
                // reference lines and the form view's title row already use, for the
                // same reason. A row's label is not a UI string the app chose: a scene
                // row is labelled with a SOLID's name, as long as the modelling history
                // made it. Laid out label-first, at its natural width, a 76-character
                // name pushed the visibility checkbox 202 pt past the edge of a 320 pt
                // panel — off the panel, where an ancestor clip eats the click too.
                //
                // Both label flavours take a wrap mode, so neither can overflow: the
                // plain `Label` is the drag handle and keeps its `sense`, and `IconText`
                // truncates the same way (a row whose label carries ⛓ / ◐ / ⏚ marks).
                // A label SHORTER than the band still measures its own text, which is
                // exactly why the row's click target and published rect are the BAND
                // and not this: a two-word name left four fifths of its row dead.
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add_space(RIGHT_PAD);
                    add_right(ui);
                    // What the CALLER put at the row's right edge, as one rect.
                    // A right-to-left `Ui` grows leftwards from the panel edge,
                    // so its min_rect here — after the gutter and the caller's
                    // controls, before the label — is exactly that strip.
                    controls = ui.min_rect();
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        label_resp = Some(if has_icon && !row.draggable {
                            IconText::new(text)
                                .sense(sense)
                                .selectable(false)
                                .truncate()
                                .show(ui)
                        } else {
                            ui.add(
                                egui::Label::new(text)
                                    .sense(sense)
                                    .selectable(!row.draggable)
                                    .truncate(),
                            )
                        });
                    });
                });
            })
        },
    );
    let band = outer.response;
    if row.highlighted && !row.root {
        ui.painter().set(
            highlight,
            egui::epaint::RectShape::filled(
                band.rect,
                0.0,
                ui.visuals().selection.bg_fill.gamma_multiply(0.35),
            ),
        );
    }

    let row_rect = outer.inner.response.rect;
    if !row.root {
        paint_connectors(ui, row_rect, row.guides, connector_col, row.is_last);
    }
    // The row is the keyboard stop, so say so when it holds the focus: nothing
    // else in the row paints one now that the label is not focusable, and a
    // focus nobody can see is a focus nobody can use.
    if band.has_focus() {
        ui.painter().rect_stroke(
            row_rect,
            2.0,
            ui.visuals().selection.stroke,
            egui::StrokeKind::Inside,
        );
    }

    // A band drag whose press STARTED on one of the row's own controls is that
    // control's press, not a row drag. egui reports both — a click-sensing
    // button on top and the drag-sensing band beneath are "interested in
    // different things", so `hit_test` hands back both hits — and without this
    // the edit button became a reorder handle the moment the hand moved.
    let press = ui.input(|i| i.pointer.press_origin());
    let from_a_control = press
        .is_some_and(|at| box_rect.contains(at) || controls.contains(at));
    let band_drag_started = band.drag_started() && !from_a_control;

    NodeResponse {
        row_rect,
        box_rect,
        toggled,
        label: label_resp.expect("label always drawn"),
        band,
        band_drag_started,
    }
}

/// Draw a LEAF whose content is a multi-line, WRAPPING colored message — e.g. a
/// feature's error text shown under its header. Unlike [`node`] (a single
/// non-wrapping row), the label WRAPS to the available width and the row grows as
/// tall as it needs; the `└`/`├` connector anchors to the FIRST line so it still
/// reads as a normal child leaf under its parent. Returns the full row rect.
pub fn message_leaf(
    ui: &mut egui::Ui,
    guides: &[bool],
    is_last: bool,
    text: &str,
    color: egui::Color32,
) -> egui::Rect {
    let connector_col = guides.len();
    // Align the wrapped text with where a LEAF's label starts: indent to the box
    // slot column, then clear the (empty) box slot + gap.
    let indent = (connector_col as f32 + 1.0) * INDENT + BOX + GAP;
    let line_h = ui.text_style_height(&egui::TextStyle::Body);
    let inner = ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.add_space(indent);
        // A vertical child claims the remaining row; the wrapping label wraps to its
        // width (minus the right gutter so it clears the sidebar scroll bar).
        ui.vertical(|ui| {
            ui.set_max_width((ui.available_width() - RIGHT_PAD).max(40.0));
            ui.add(
                egui::Label::new(egui::RichText::new(text).color(color))
                    .wrap_mode(egui::TextWrapMode::Wrap),
            );
        });
    });
    let row_rect = inner.response.rect;
    let tick_y = row_rect.top() + line_h * 0.5;
    paint_connectors_at(ui, row_rect, guides, connector_col, is_last, tick_y);
    row_rect
}

/// Paint a `[+]`/`[-]` collapse box: a rounded stroked square with a minus bar
/// (expanded) plus a vertical bar (collapsed → a plus). Theme-aware.
fn paint_box(ui: &egui::Ui, rect: egui::Rect, expanded: bool) {
    let painter = ui.painter();
    let stroke = egui::Stroke::new(1.0, line_color(ui));
    painter.rect(
        rect,
        egui::CornerRadius::same(2),
        egui::Color32::TRANSPARENT,
        stroke,
        egui::StrokeKind::Inside,
    );
    let c = rect.center();
    let arm = rect.width() * 0.28;
    let sign = egui::Stroke::new(1.4, ui.visuals().text_color());
    // horizontal bar (always → the minus of a `[-]`)
    painter.line_segment(
        [egui::pos2(c.x - arm, c.y), egui::pos2(c.x + arm, c.y)],
        sign,
    );
    // vertical bar (only when collapsed → completes the plus of a `[+]`)
    if !expanded {
        painter.line_segment(
            [egui::pos2(c.x, c.y - arm), egui::pos2(c.x, c.y + arm)],
            sign,
        );
    }
}

/// Paint the ancestor guide lines + this node's `├`/`└` connector into the row's
/// left gutter (over-drawn by the inter-row spacing so verticals join seamlessly
/// across rows). The `├`/`└` tick lands at the row's vertical center.
fn paint_connectors(
    ui: &egui::Ui,
    row: egui::Rect,
    guides: &[bool],
    connector_col: usize,
    is_last: bool,
) {
    paint_connectors_at(ui, row, guides, connector_col, is_last, row.center().y);
}

/// Like [`paint_connectors`] but with an explicit `tick_y` for the `├`/`└` join —
/// so a MULTI-LINE row (a wrapping message leaf) can anchor its connector to its
/// FIRST line instead of the block's vertical center.
fn paint_connectors_at(
    ui: &egui::Ui,
    row: egui::Rect,
    guides: &[bool],
    connector_col: usize,
    is_last: bool,
    mid: f32,
) {
    let painter = ui.painter();
    let stroke = egui::Stroke::new(1.0, line_color(ui));
    let sp = ui.spacing().item_spacing.y + 1.0;
    let left = row.left();
    let top = row.top() - sp;
    let bottom = row.bottom() + sp;
    let col_x = |c: usize| left + c as f32 * INDENT + INDENT * 0.5;

    // Ancestor sibling lines passing through this row.
    for (i, on) in guides.iter().enumerate() {
        if *on {
            let x = col_x(i);
            painter.line_segment([egui::pos2(x, top), egui::pos2(x, bottom)], stroke);
        }
    }
    // This node's own connector: vertical down to mid (└) or through (├), plus a
    // horizontal tick reaching the box slot.
    let x = col_x(connector_col);
    let v_bottom = if is_last { mid } else { bottom };
    painter.line_segment([egui::pos2(x, top), egui::pos2(x, v_bottom)], stroke);
    painter.line_segment(
        [egui::pos2(x, mid), egui::pos2(x + INDENT * 0.5, mid)],
        stroke,
    );
}

/// The subtle connector / box stroke color — the theme's non-interactive
/// foreground, dimmed. Theme-aware (light + dark) and the seam the later theme
/// pass tunes in one place.
fn line_color(ui: &egui::Ui) -> egui::Color32 {
    ui.visuals()
        .widgets
        .noninteractive
        .fg_stroke
        .color
        .gamma_multiply(0.7)
}

