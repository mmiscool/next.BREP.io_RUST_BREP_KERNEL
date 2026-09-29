//! Drawing sheets — the document's `sheets` block, the projection of a placed
//! PMI view into paper millimetres, and the SVG writer.
//!
//! # The `sheets` block
//!
//! `{ sheets: [{ id, name, size, widthMm, heightMm, views: [{ id, view,
//! position, scale, projection }] }], idCounter }`, a top-level key of the
//! saved document beside `pmi` and `partAttributes`. It is ABSENT on every
//! document without a sheet (and omitted on re-serialize), so a part that
//! never had one saves byte-identically. The kernel never sees it: a sheet
//! places what the PMI tail already resolved, so the block rides the
//! document's unknown-key pass-through ([`crate::history::History::request_json`]
//! serializes the raw document) exactly as `partAttributes` does — no
//! `HistoryRequest` field, no migration, because it is new.
//!
//! A **sheet** is a paper size in millimetres (a preset or a custom pair), the
//! paper's own FURNITURE — a drawing border inset from the page edge and a
//! title block in its bottom-right corner ([`frame`]) — and an ordered list of
//! **placed views**. A placed view names a saved PMI view
//! by id and adds where it sits on the paper (`position`, mm from the paper's
//! top-left corner), how big it draws (`scale`, paper mm per model unit) and
//! its projection kind. The saved view's own camera is what the placement
//! projects through; its zoom (`halfHeight`) is NOT the sheet scale — a sheet
//! says how large the part draws on paper, the camera only says from where.
//!
//! # This slice
//!
//! Orthographic only: a placed view whose saved camera is perspective is
//! REFUSED at placement with a message rather than silently flattened
//! (`EngineState::sheet_place_view`, and the placement form that re-points a
//! placement). Hidden lines are removed exactly, on the B-rep — see
//! [`hlr`] for the pass and its floors, [`project`] for the placements, and
//! `docs/panels/sheets.md` for the user-facing contract.

pub mod dimension;
pub mod frame;
pub mod hlr;
pub mod ordinate;
pub mod pdf;
pub mod pdf3d;
pub mod project;
pub mod svg;
pub mod u3d;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

/// The document key the block is stored under.
pub const SHEETS: &str = "sheets";

/// The only projection kind this slice draws.
pub const ORTHOGRAPHIC: &str = "orthographic";

/// The paper-size presets, `(key, width mm, height mm)` — ISO A landscape plus
/// the two ANSI sizes in millimetres (the block stores millimetres only, so a
/// Letter sheet is 279.4 × 215.9 and not a second unit system).
pub const PAPER_SIZES: &[(&str, f64, f64)] = &[
    ("A4", 297.0, 210.0),
    ("A3", 420.0, 297.0),
    ("A2", 594.0, 420.0),
    ("A1", 841.0, 594.0),
    ("A0", 1189.0, 841.0),
    ("Letter", 279.4, 215.9),
    ("Tabloid", 431.8, 279.4),
];

/// The `size` value of a sheet whose `widthMm` / `heightMm` are the user's own.
pub const CUSTOM: &str = "Custom";

/// The default paper size of a new sheet.
pub const DEFAULT_SIZE: &str = "A3";

/// The default scale of a newly placed view (1 mm of paper per model unit).
pub const DEFAULT_SCALE: f64 = 1.0;

/// The default inset of the drawing BORDER from the paper's edge, millimetres.
pub const DEFAULT_BORDER_INSET_MM: f64 = 10.0;

/// A preset's millimetres, or `None` for [`CUSTOM`] and anything unknown.
pub fn paper_size(key: &str) -> Option<(f64, f64)> {
    PAPER_SIZES
        .iter()
        .find(|(name, _, _)| name.eq_ignore_ascii_case(key))
        .map(|(_, w, h)| (*w, *h))
}

/// The document's `sheets` block.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SheetState {
    #[serde(default)]
    pub sheets: Vec<Sheet>,
    /// Mints every sheet AND placed-view id (`SHEET{n}`, `SV{n}`); monotonic,
    /// never reused, re-seeded from the largest numeric suffix on load — the
    /// `pmi` block's counter rule, so a merged or edited document never
    /// collides.
    #[serde(default, rename = "idCounter")]
    pub id_counter: u64,
}

/// One sheet of paper.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sheet {
    pub id: String,
    #[serde(default)]
    pub name: String,
    /// A [`PAPER_SIZES`] key, or [`CUSTOM`].
    #[serde(default = "default_size")]
    pub size: String,
    #[serde(default, rename = "widthMm")]
    pub width_mm: f64,
    #[serde(default, rename = "heightMm")]
    pub height_mm: f64,
    /// Draw the BORDER: the rectangle inset from the paper's edge that the
    /// drawing lives inside. On by default; a sheet may switch it off.
    #[serde(default = "default_true")]
    pub border: bool,
    /// How far the border sits from the paper's edge, millimetres. It is also
    /// the margin the TITLE BLOCK is placed against, so a sheet with no border
    /// still has a margin.
    #[serde(default = "default_border_inset", rename = "borderInsetMm")]
    pub border_inset_mm: f64,
    /// Draw the TITLE BLOCK in the paper's bottom-right corner, inside the
    /// border. On by default.
    #[serde(default = "default_true", rename = "titleBlock")]
    pub title_block: bool,
    /// The title block's free text field — whatever the drawing has to say.
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub views: Vec<PlacedView>,
    /// The sheet's OWN dimensions — drawn on the paper, anchored to the
    /// projected geometry of the placements above ([`dimension`]). A sheet
    /// saved before they existed simply has none.
    #[serde(default)]
    pub dimensions: Vec<dimension::SheetDimension>,
    /// The sheet's ORDINATE SETS ([`ordinate`]) — a datum and a run of
    /// members, each reading its signed distance from the datum. Beside the
    /// dimensions rather than among them: a set measures any number of points
    /// against one origin and draws no dimension line, which is a different
    /// object and not a fifth kind.
    #[serde(default)]
    pub ordinates: Vec<ordinate::SheetOrdinate>,
    /// The sheet's REVISIONS, oldest first as the user keeps them — drawn as
    /// the revision table beside the title block ([`frame`]). A sheet saved
    /// before they existed has none, and a sheet with none draws no table.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub revisions: Vec<Revision>,
}

/// One row of a sheet's revision table: the revision's letter, its date and
/// what changed. All three are the user's own text — a drawing office letters,
/// dates and words its revisions its own way — so none is validated beyond
/// being a string, and a blank one leaves its cell ruled and empty.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Revision {
    #[serde(default)]
    pub rev: String,
    #[serde(default)]
    pub date: String,
    #[serde(default)]
    pub description: String,
}

impl Revision {
    /// The three fields, by the keys the block, the form and the commands use.
    pub const FIELDS: &'static [&'static str] = &["rev", "date", "description"];

    /// Patch this row from `params`; absent keys are unchanged, and a key that
    /// is not a string is refused by name.
    pub fn apply(&mut self, params: &Value) -> Result<(), String> {
        let Some(object) = params.as_object() else {
            return Err("a revision is an object of rev, date and description".into());
        };
        for (key, value) in object {
            let slot = match key.as_str() {
                "rev" => &mut self.rev,
                "date" => &mut self.date,
                "description" => &mut self.description,
                other => return Err(format!("a revision has no field '{other}'")),
            };
            *slot = value
                .as_str()
                .ok_or_else(|| format!("revision {key}: {value} is not text"))?
                .to_string();
        }
        Ok(())
    }
}

fn default_size() -> String {
    DEFAULT_SIZE.to_string()
}

fn default_true() -> bool {
    true
}

fn is_false(value: &bool) -> bool {
    !*value
}

fn default_border_inset() -> f64 {
    DEFAULT_BORDER_INSET_MM
}

/// A saved PMI view placed on a sheet.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacedView {
    pub id: String,
    /// The PMI view's id (`VIEW3`).
    #[serde(default)]
    pub view: String,
    /// Millimetres from the paper's TOP-LEFT corner (SVG's own origin), of the
    /// point the saved camera targets — the view's centre, not its corner, so
    /// changing the scale grows the drawing about where it was placed.
    #[serde(default)]
    pub position: [f64; 2],
    /// Paper millimetres per model unit.
    #[serde(default = "default_scale")]
    pub scale: f64,
    /// `orthographic` — the only kind this slice draws.
    #[serde(default = "default_projection")]
    pub projection: String,
    /// Rotate every text run — and every framed symbol, frame and cells
    /// together — FLAT to the sheet, about its own anchor, at its sheet-scale
    /// size. ON by default, because that is the drawing convention: a
    /// drawing's text reads left to right whatever plane the annotation was
    /// authored in. Off, each run is drawn as the projection gives it:
    /// foreshortened, rotated, and on an edge-on plane collapsed onto a line.
    #[serde(default = "default_true", rename = "flattenText")]
    pub flatten_text: bool,
    /// Draw this placement LIVE in the Sheets + 3D PDF as well: a 3D box over the
    /// drawing that Acrobat and Foxit turn and zoom, opening on this
    /// placement's camera and scale ([`pdf3d`]). Every other reader, and the
    /// SVG, show the drawing alone. Off by default, and absent from a saved
    /// placement that is off, so a document that never asks for 3D saves
    /// exactly as before.
    #[serde(default, rename = "threeD", skip_serializing_if = "is_false")]
    pub three_d: bool,
    /// This placement is a SECTION VIEW: its camera is derived from a cutting
    /// plane picked on ANOTHER placement, and it draws the model clipped at
    /// that plane with the cut faces hatched. Absent on a plain placement,
    /// which is every one saved before sections existed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub section: Option<SectionCut>,
    /// This placement is a DETAIL VIEW: a circled region of ANOTHER placement
    /// redrawn at this placement's own scale, through the source's camera and
    /// clipped to the circle. Absent on a plain placement and on a section; a
    /// placement is at most one of the two.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<DetailCircle>,
}

/// The cutting plane of a section view, as it was PICKED: two anchors of an
/// existing placement's projection.
///
/// The line is stored as two ANCHOR REFERENCES rather than two paper points,
/// for the reason every other sheet reference is: a paper point is not a
/// property of the model, and a placement that moved or a scale that changed
/// would leave the cut somewhere else. Two anchors follow their geometry
/// through a camera, scale, placement or model change — and a section whose
/// anchor is gone goes UNRESOLVED with the reason, exactly as a dimension
/// does, rather than quietly cutting the wrong place.
///
/// It is also what makes the cut reachable without a pointer: an anchor has a
/// keyed hit rect (`sheet/anchor:<reference>`) and a bare paper point does
/// not, so a section picked from free points could not be driven by a script
/// or by anything but a mouse.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SectionCut {
    /// The placement the line is drawn ON — the view the cut was picked in.
    /// Its camera's viewing direction lies IN the cutting plane, which is what
    /// makes the section camera's frame orthonormal by construction.
    #[serde(default)]
    pub source: String,
    /// The two anchors the cutting line runs through.
    #[serde(default)]
    pub from: String,
    #[serde(default)]
    pub to: String,
    /// The section's letter — drawn at both arrows on the source view and in
    /// this view's own `SECTION A-A` caption.
    #[serde(default = "default_label")]
    pub label: String,
    /// Look the other way along the plane's normal. The unflipped direction is
    /// the drawing convention read off the pick order: 90\u{00B0} clockwise from
    /// `from` \u{2192} `to` on the source's paper, which is the side a reader walking
    /// from the first arrow to the second has on their right.
    #[serde(default)]
    pub flip: bool,
}

fn default_label() -> String {
    "A".to_string()
}

/// The circle of a detail view, as it was PICKED: two anchors of an existing
/// placement's projection — the circle's centre and a point on its rim.
///
/// Anchors rather than a paper centre and a radius in millimetres, for the
/// section's reason: a paper point is not a property of the model, and the
/// two anchors are what keeps the circle on the feature it was drawn round
/// through a camera, scale, placement or model change. They also give the
/// circle keyed hit rects (`sheet/anchor:<reference>`), so a script picks one
/// exactly as a user does.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DetailCircle {
    /// The placement the circle is drawn ON. The detail looks through that
    /// placement's camera; only the scale and the clip are its own.
    #[serde(default)]
    pub source: String,
    /// The anchor at the circle's centre.
    #[serde(default)]
    pub centre: String,
    /// An anchor ON the circle: its distance from the centre, measured on the
    /// source's paper and divided by the source's scale, is the radius in
    /// model units.
    #[serde(default)]
    pub rim: String,
    /// The detail's letter — beside the circle on the source and in this
    /// view's own `DETAIL B` caption. Handed out from the SAME run of letters
    /// as a section's ([`Sheet::next_view_letter`]).
    #[serde(default = "default_label")]
    pub label: String,
}

/// A new detail view draws at this multiple of its source's scale — the
/// usual reason for a detail is that the source is too small to read there.
pub const DETAIL_SCALE_FACTOR: f64 = 2.0;

impl SectionCut {
    /// The letters a sheet hands out, in order, skipping the three a drawing
    /// office does not use for a section (I, O and Q read as 1, 0 and O).
    pub const LETTERS: &'static [char] = &[
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P', 'R', 'S', 'T', 'U',
        'V', 'W', 'X', 'Y', 'Z',
    ];
}

fn default_scale() -> f64 {
    DEFAULT_SCALE
}

fn default_projection() -> String {
    ORTHOGRAPHIC.to_string()
}

impl Sheet {
    /// The paper's millimetres: the preset's when `size` names one (so a
    /// preset sheet cannot drift), else the stored custom pair.
    pub fn millimetres(&self) -> (f64, f64) {
        match paper_size(&self.size) {
            Some((w, h)) => (w, h),
            None => (self.width_mm.max(1.0), self.height_mm.max(1.0)),
        }
    }

    /// The border's inset, clamped to something the paper can carry: a sheet
    /// whose inset ate the page would have no drawing area at all.
    pub fn border_inset(&self) -> f64 {
        let (w, h) = self.millimetres();
        self.border_inset_mm.max(0.0).min(w.min(h) * 0.25)
    }

    pub fn find_view(&self, id: &str) -> Option<&PlacedView> {
        self.views.iter().find(|view| view.id == id)
    }

    pub fn find_view_mut(&mut self, id: &str) -> Option<&mut PlacedView> {
        self.views.iter_mut().find(|view| view.id == id)
    }

    pub fn find_dimension(&self, id: &str) -> Option<&dimension::SheetDimension> {
        self.dimensions.iter().find(|d| d.id == id)
    }

    pub fn find_ordinate(&self, id: &str) -> Option<&ordinate::SheetOrdinate> {
        self.ordinates.iter().find(|o| o.id == id)
    }

    /// The next view LETTER this sheet has not handed out, for a section or a
    /// detail alike: the two draw from ONE run of letters, because a drawing
    /// with a `SECTION A—A` and a `DETAIL A` names two different views with
    /// one letter. A sheet past `Z` reuses the last one rather than refusing
    /// the view: two views sharing a letter is a drawing-office mistake, not a
    /// broken document, and the form is where it is corrected.
    pub fn next_view_letter(&self) -> String {
        let taken: Vec<&str> = self
            .views
            .iter()
            .filter_map(|view| {
                view.section
                    .as_ref()
                    .map(|cut| cut.label.as_str())
                    .or_else(|| view.detail.as_ref().map(|circle| circle.label.as_str()))
            })
            .collect();
        SectionCut::LETTERS
            .iter()
            .map(|letter| letter.to_string())
            .find(|letter| !taken.iter().any(|used| used == letter))
            .unwrap_or_else(|| SectionCut::LETTERS[SectionCut::LETTERS.len() - 1].to_string())
    }
}

impl SheetState {
    /// Mint the next id with `prefix` (`SHEET` → `SHEET3`), re-seeding the
    /// counter from the largest numeric suffix already in the block.
    pub fn next_id(&mut self, prefix: &str) -> String {
        let seen = self.max_numeric_suffix();
        if seen > self.id_counter {
            self.id_counter = seen;
        }
        loop {
            self.id_counter += 1;
            let candidate = format!("{prefix}{}", self.id_counter);
            if self.find_sheet(&candidate).is_none()
                && self.locate_view(&candidate).is_none()
                && self.locate_dimension(&candidate).is_none()
                && self.locate_ordinate(&candidate).is_none()
            {
                return candidate;
            }
        }
    }

    fn max_numeric_suffix(&self) -> u64 {
        let mut best = 0u64;
        let mut consider = |id: &str| {
            let digits = id.bytes().rev().take_while(u8::is_ascii_digit).count();
            if digits > 0 {
                if let Ok(value) = id[id.len() - digits..].parse::<u64>() {
                    best = best.max(value);
                }
            }
        };
        for sheet in &self.sheets {
            consider(&sheet.id);
            for view in &sheet.views {
                consider(&view.id);
            }
            for dimension in &sheet.dimensions {
                consider(&dimension.id);
            }
            for set in &sheet.ordinates {
                consider(&set.id);
            }
        }
        best
    }

    pub fn find_sheet(&self, id: &str) -> Option<&Sheet> {
        self.sheets.iter().find(|sheet| sheet.id == id)
    }

    pub fn find_sheet_mut(&mut self, id: &str) -> Option<&mut Sheet> {
        self.sheets.iter_mut().find(|sheet| sheet.id == id)
    }

    /// The sheet id and index of placed view `id`.
    pub fn locate_view(&self, id: &str) -> Option<(String, usize)> {
        self.sheets.iter().find_map(|sheet| {
            sheet
                .views
                .iter()
                .position(|view| view.id == id)
                .map(|index| (sheet.id.clone(), index))
        })
    }

    pub fn find_view_mut(&mut self, id: &str) -> Option<&mut PlacedView> {
        self.sheets
            .iter_mut()
            .find_map(|sheet| sheet.views.iter_mut().find(|view| view.id == id))
    }

    /// The sheet id and index of dimension `id`.
    pub fn locate_dimension(&self, id: &str) -> Option<(String, usize)> {
        self.sheets.iter().find_map(|sheet| {
            sheet
                .dimensions
                .iter()
                .position(|dimension| dimension.id == id)
                .map(|index| (sheet.id.clone(), index))
        })
    }

    pub fn find_dimension(&self, id: &str) -> Option<&dimension::SheetDimension> {
        self.sheets.iter().find_map(|sheet| sheet.find_dimension(id))
    }

    pub fn find_dimension_mut(&mut self, id: &str) -> Option<&mut dimension::SheetDimension> {
        self.sheets
            .iter_mut()
            .find_map(|sheet| sheet.dimensions.iter_mut().find(|d| d.id == id))
    }

    /// The sheet id and index of ordinate set `id`.
    pub fn locate_ordinate(&self, id: &str) -> Option<(String, usize)> {
        self.sheets.iter().find_map(|sheet| {
            sheet
                .ordinates
                .iter()
                .position(|set| set.id == id)
                .map(|index| (sheet.id.clone(), index))
        })
    }

    pub fn find_ordinate(&self, id: &str) -> Option<&ordinate::SheetOrdinate> {
        self.sheets.iter().find_map(|sheet| sheet.find_ordinate(id))
    }

    pub fn find_ordinate_mut(&mut self, id: &str) -> Option<&mut ordinate::SheetOrdinate> {
        self.sheets
            .iter_mut()
            .find_map(|sheet| sheet.ordinates.iter_mut().find(|o| o.id == id))
    }

    /// Whether the block carries anything worth persisting.
    pub fn is_empty(&self) -> bool {
        self.sheets.is_empty() && self.id_counter == 0
    }
}

// ===========================================================================
// Schemas — the catalogue the dialogs and the MCP contract are drawn from
// ===========================================================================

/// The sheet object schemas, in the `inputParamsSchema` vocabulary the shared
/// form engine already renders ([`crate::features::form_fields_from_schema`]),
/// so neither dialog is hand-written. `view_ids` are the document's saved PMI
/// views — the options of a placed view's `view` field, which is why this
/// catalogue is built per document rather than being a static.
pub fn schema_catalogue(view_ids: &[String]) -> Value {
    json!({
        "objects": [
            sheet_schema(),
            sheet_view_schema(view_ids),
            dimension::sheet_dimension_schema(),
            ordinate::sheet_ordinate_schema(),
            section_view_schema(),
            detail_view_schema(),
        ]
    })
}

/// The **Section view** dialog's schema: where the view sits and at what
/// scale, and the section itself — its CUTTING LINE, two anchors picked on a
/// placed view with the reference picker, its letter and which way it looks.
/// Its saved view is its source's, so it has no `view` field of its own.
pub fn section_view_schema() -> Value {
    json!({
        "type": "sectionView",
        "shortName": "SV",
        "longName": "Section view",
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null },
            "cut": { "type": "reference_selection", "label": "Cutting line", "selectionFilter": ["sheetAnchor"], "multiple": true, "hint": "Two anchors on one placed view" },
            "sectionLabel": { "type": "string", "label": "Section letter", "default_value": "A" },
            "sectionFlip": { "type": "boolean", "label": "Section looks the other way", "default_value": false },
            "positionXMm": { "type": "number", "label": "X (mm)", "default_value": 0.0 },
            "positionYMm": { "type": "number", "label": "Y (mm)", "default_value": 0.0 },
            "scale": { "type": "number", "label": "Scale (mm per model unit)", "default_value": DEFAULT_SCALE },
            "flattenText": { "type": "boolean", "label": "Flatten text to sheet", "default_value": true },
        }
    })
}

/// The **Detail view** dialog's schema: the circle — its CENTRE and a point on
/// its RIM, each an anchor picked on one placed view — its letter, and where
/// the enlarged view sits and at what scale.
pub fn detail_view_schema() -> Value {
    json!({
        "type": "detailView",
        "shortName": "SV",
        "longName": "Detail view",
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null },
            "centre": { "type": "reference_selection", "label": "Centre", "selectionFilter": ["sheetAnchor"], "multiple": false, "hint": "The circle's centre, an anchor on a placed view" },
            "rim": { "type": "reference_selection", "label": "Point on the rim", "selectionFilter": ["sheetAnchor"], "multiple": false, "hint": "An anchor on the same view, on the circle" },
            "detailLabel": { "type": "string", "label": "Detail letter", "default_value": "A" },
            "positionXMm": { "type": "number", "label": "X (mm)", "default_value": 0.0 },
            "positionYMm": { "type": "number", "label": "Y (mm)", "default_value": 0.0 },
            "scale": { "type": "number", "label": "Scale (mm per model unit)", "default_value": DEFAULT_SCALE },
            "flattenText": { "type": "boolean", "label": "Flatten text to sheet", "default_value": true },
        }
    })
}

/// The **Sheet** dialog's schema.
pub fn sheet_schema() -> Value {
    let sizes: Vec<&str> = PAPER_SIZES
        .iter()
        .map(|(key, _, _)| *key)
        .chain(std::iter::once(CUSTOM))
        .collect();
    json!({
        "type": "sheet",
        "shortName": "SHT",
        "longName": "Sheet",
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null },
            "name": { "type": "string", "default_value": "Sheet 1" },
            "size": { "type": "options", "label": "Paper size", "options": sizes, "default_value": DEFAULT_SIZE },
            "widthMm": { "type": "number", "label": "Width (mm)", "default_value": 420.0 },
            "heightMm": { "type": "number", "label": "Height (mm)", "default_value": 297.0 },
            "border": { "type": "boolean", "label": "Border", "default_value": true },
            "borderInsetMm": { "type": "number", "label": "Border inset (mm)", "default_value": DEFAULT_BORDER_INSET_MM },
            "titleBlock": { "type": "boolean", "label": "Title block", "default_value": true },
            "notes": { "type": "string", "label": "Notes", "default_value": "" },
            // A LIST of rows, which the schema vocabulary has no field for: the
            // Sheets pane draws it as the dialog's own Revisions section, and
            // `sheet_update` takes the whole list in this shape.
            "revisions": { "type": "revision_table", "label": "Revisions", "columns": Revision::FIELDS, "default_value": [] },
        }
    })
}

/// The **Placed view** dialog's schema. `view` offers the document's saved PMI
/// views; `projection` offers only [`ORTHOGRAPHIC`] in this slice.
pub fn sheet_view_schema(view_ids: &[String]) -> Value {
    json!({
        "type": "sheetView",
        "shortName": "SV",
        "longName": "Placed view",
        "inputParamsSchema": {
            "id": { "type": "string", "default_value": null },
            "view": { "type": "options", "label": "PMI view", "options": view_ids, "default_value": view_ids.first().cloned().unwrap_or_default() },
            "positionXMm": { "type": "number", "label": "X (mm)", "default_value": 0.0 },
            "positionYMm": { "type": "number", "label": "Y (mm)", "default_value": 0.0 },
            "scale": { "type": "number", "label": "Scale (mm per model unit)", "default_value": DEFAULT_SCALE },
            "projection": { "type": "options", "label": "Projection", "options": [ORTHOGRAPHIC], "default_value": ORTHOGRAPHIC },
            "flattenText": { "type": "boolean", "label": "Flatten text to sheet", "default_value": true },
            "threeD": { "type": "boolean", "label": "Live 3D in PDF", "default_value": false, "hint": "In the Sheets + 3D (PDF) export, Acrobat and Foxit turn and zoom this view; other readers show the drawing" },
        }
    })
}

impl Sheet {
    /// The params the **Sheet** form edits, as the schema shapes them.
    pub fn params(&self) -> Value {
        let (w, h) = self.millimetres();
        json!({
            "id": self.id,
            "name": self.name,
            "size": self.size,
            "widthMm": w,
            "heightMm": h,
            "border": self.border,
            "borderInsetMm": self.border_inset_mm,
            "titleBlock": self.title_block,
            "notes": self.notes,
            "revisions": self.revisions,
        })
    }

    /// Apply an edited **Sheet** form. `id` is identity and is never taken
    /// from the form.
    pub fn apply(&mut self, params: &Value) -> Result<(), String> {
        if let Some(name) = params.get("name").and_then(Value::as_str) {
            let name = name.trim();
            if name.is_empty() {
                return Err("a sheet needs a name".into());
            }
            self.name = name.to_string();
        }
        if let Some(size) = params.get("size").and_then(Value::as_str) {
            let size = size.trim();
            if !size.eq_ignore_ascii_case(CUSTOM) && paper_size(size).is_none() {
                return Err(format!("no paper size '{size}'"));
            }
            self.size = if size.eq_ignore_ascii_case(CUSTOM) {
                CUSTOM.to_string()
            } else {
                size.to_string()
            };
        }
        let dimension = |key: &str, current: f64| -> Result<f64, String> {
            match params.get(key).and_then(Value::as_f64) {
                None => Ok(current),
                Some(value) if value.is_finite() && value > 0.0 => Ok(value),
                Some(value) => Err(format!("{key}: {value} is not a positive length")),
            }
        };
        self.width_mm = dimension("widthMm", self.width_mm)?;
        self.height_mm = dimension("heightMm", self.height_mm)?;
        // A preset's dimensions are the preset's; storing them keeps the block
        // readable and keeps a switch to Custom from starting at zero.
        if let Some((w, h)) = paper_size(&self.size) {
            self.width_mm = w;
            self.height_mm = h;
        }
        // The paper's furniture. The inset is refused rather than clamped when
        // it would eat the page — a form that silently kept a quarter of what
        // was typed would be lying about what the sheet is.
        if let Some(border) = params.get("border").and_then(Value::as_bool) {
            self.border = border;
        }
        if let Some(inset) = params.get("borderInsetMm").and_then(Value::as_f64) {
            let (w, h) = self.millimetres();
            if !inset.is_finite() || inset < 0.0 {
                return Err(format!("borderInsetMm: {inset} is not a length"));
            }
            if inset > w.min(h) * 0.25 {
                return Err(format!(
                    "borderInsetMm: {inset} leaves no drawing area on a {w:.0} × {h:.0} mm sheet"
                ));
            }
            self.border_inset_mm = inset;
        }
        if let Some(title_block) = params.get("titleBlock").and_then(Value::as_bool) {
            self.title_block = title_block;
        }
        // Notes are the user's own text, empty included — the one field on a
        // sheet whose blank value is a value.
        if let Some(notes) = params.get("notes").and_then(Value::as_str) {
            self.notes = notes.to_string();
        }
        // The revision list is replaced WHOLE and in the order given — the
        // order IS the table's — and a row the list cannot mean refuses the
        // edit before anything is kept.
        if let Some(rows) = params.get("revisions") {
            let rows = rows.as_array().ok_or("revisions: not a list of rows")?;
            let mut revisions = Vec::with_capacity(rows.len());
            for (index, row) in rows.iter().enumerate() {
                let mut revision = Revision::default();
                revision.apply(row).map_err(|error| format!("revisions[{index}]: {error}"))?;
                revisions.push(revision);
            }
            self.revisions = revisions;
        }
        Ok(())
    }
}

impl PlacedView {
    /// The params the placement's form edits — the **Placed view**, **Section
    /// view** or **Detail view** schema's, whichever this placement is. A
    /// section's cutting line (`cut`) and a detail's `centre` and `rim` are
    /// the picked anchors its reference rows show.
    pub fn params(&self) -> Value {
        let mut params = json!({
            "id": self.id,
            "view": self.view,
            "positionXMm": self.position[0],
            "positionYMm": self.position[1],
            "scale": self.scale,
            "projection": self.projection,
            "flattenText": self.flatten_text,
        });
        if self.schema_type() == "sheetView" {
            params.as_object_mut().expect("an object").insert("threeD".into(), json!(self.three_d));
        }
        let object = params.as_object_mut().expect("an object");
        if let Some(cut) = &self.section {
            object.insert("cut".into(), json!(self.picked("cut")));
            object.insert("sectionLabel".into(), json!(cut.label));
            object.insert("sectionFlip".into(), json!(cut.flip));
        }
        if let Some(circle) = &self.detail {
            object.insert("centre".into(), json!(circle.centre));
            object.insert("rim".into(), json!(circle.rim));
            object.insert("detailLabel".into(), json!(circle.label));
        }
        params
    }

    /// The anchors picked for a derived placement's reference `field` (`cut`,
    /// `centre`, `rim`), blanks left out — empty for a field this placement
    /// does not have.
    pub fn picked(&self, field: &str) -> Vec<String> {
        let list: Vec<&String> = match (field, &self.section, &self.detail) {
            ("cut", Some(cut), _) => vec![&cut.from, &cut.to],
            ("centre", _, Some(circle)) => vec![&circle.centre],
            ("rim", _, Some(circle)) => vec![&circle.rim],
            _ => Vec::new(),
        };
        list.into_iter().filter(|anchor| !anchor.is_empty()).cloned().collect()
    }

    /// Which catalogue object this placement's form is: `sheetView`,
    /// `sectionView` or `detailView`.
    pub fn schema_type(&self) -> &'static str {
        match (&self.section, &self.detail) {
            (Some(_), _) => "sectionView",
            (_, Some(_)) => "detailView",
            _ => "sheetView",
        }
    }

    /// Apply an edited **Placed view** form.
    pub fn apply(&mut self, params: &Value, view_ids: &[String]) -> Result<(), String> {
        if let Some(view) = params.get("view").and_then(Value::as_str) {
            let view = view.trim();
            if !view.is_empty() && !view_ids.iter().any(|id| id == view) {
                return Err(format!("no PMI view '{view}'"));
            }
            self.view = view.to_string();
        }
        for (key, slot) in [("positionXMm", 0usize), ("positionYMm", 1usize)] {
            if let Some(value) = params.get(key).and_then(Value::as_f64) {
                if !value.is_finite() {
                    return Err(format!("{key}: not a finite length"));
                }
                self.position[slot] = value;
            }
        }
        if let Some(scale) = params.get("scale").and_then(Value::as_f64) {
            if !scale.is_finite() || scale <= 0.0 {
                return Err(format!("scale: {scale} is not a positive ratio"));
            }
            self.scale = scale;
        }
        if let Some(projection) = params.get("projection").and_then(Value::as_str) {
            if !projection.eq_ignore_ascii_case(ORTHOGRAPHIC) {
                return Err(format!(
                    "projection '{projection}' is not drawn on a sheet yet — this slice is orthographic only"
                ));
            }
            self.projection = ORTHOGRAPHIC.to_string();
        }
        if let Some(flatten) = params.get("flattenText").and_then(Value::as_bool) {
            self.flatten_text = flatten;
        }
        // Live 3D is a PLAIN placement's: a section's clipped model and a
        // detail's circle are not what the 3D box would show, so asking for it
        // there is refused rather than drawn wrong.
        if let Some(three_d) = params.get("threeD").and_then(Value::as_bool) {
            if three_d && self.schema_type() != "sheetView" {
                return Err(format!("'{}' is a {}; live 3D is for a plain placed view", self.id, self.schema_type()));
            }
            self.three_d = three_d;
        }
        // The section's two editable fields are refused on a PLAIN placement
        // rather than ignored: a form that silently swallowed "look the other
        // way" on a view with no cutting plane would be lying about what it
        // did. A blank label from the plain placement's own form is not an
        // edit, so it passes.
        let label = params.get("sectionLabel").and_then(Value::as_str).map(str::trim);
        let flip = params.get("sectionFlip").and_then(Value::as_bool);
        match self.section.as_mut() {
            Some(cut) => {
                if let Some(label) = label.filter(|l| !l.is_empty()) {
                    cut.label = label.to_string();
                }
                if let Some(flip) = flip {
                    cut.flip = flip;
                }
            }
            None => {
                if label.is_some_and(|l| !l.is_empty()) || flip == Some(true) {
                    return Err(format!(
                        "'{}' is not a section view; it has no cutting plane to letter or turn round",
                        self.id
                    ));
                }
            }
        }
        // The detail's letter, by the same rule: refused on a placement that is
        // not a detail, and a blank one from any other placement's form passes.
        let detail_label = params.get("detailLabel").and_then(Value::as_str).map(str::trim);
        match self.detail.as_mut() {
            Some(circle) => {
                if let Some(label) = detail_label.filter(|l| !l.is_empty()) {
                    circle.label = label.to_string();
                }
            }
            None => {
                if detail_label.is_some_and(|l| !l.is_empty()) {
                    return Err(format!(
                        "'{}' is not a detail view; it has no circle to letter",
                        self.id
                    ));
                }
            }
        }
        Ok(())
    }
}

