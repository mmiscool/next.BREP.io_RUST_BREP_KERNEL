//! Render settings (R14 — the `CADmaterials` semantics: per-kind material
//! variants, hover color, flat-shading toggle, user-persisted overrides) and
//! the selection/hover emphasis state (R17/R24 — name-keyed, fed by the host's
//! `SelectionFilter`). Plain data, shared by the engine core and the wgpu
//! renderer; JSON in/out at the R3 boundary.

use std::collections::HashSet;

/// sRGB color in 0..1 + alpha.
pub type Rgba = [f32; 4];

fn hex(hex: u32, alpha: f32) -> Rgba {
    let [r, g, b] = crate::color::hex_to_srgb_f32(hex);
    [r, g, b, alpha]
}

/// Parse `#rrggbb` / `#rgb` / `0xrrggbb` (returns None on anything else).
pub fn parse_css_hex(value: &str) -> Option<[f32; 3]> {
    let v = value.trim();
    let digits = v
        .strip_prefix('#')
        .or_else(|| v.strip_prefix("0x"))
        .or_else(|| v.strip_prefix("0X"))?;
    let expand = |c: char| c.to_digit(16).map(|d| (d * 17) as f32 / 255.0);
    match digits.len() {
        3 => {
            let mut chars = digits.chars();
            Some([
                expand(chars.next()?)?,
                expand(chars.next()?)?,
                expand(chars.next()?)?,
            ])
        }
        6 => {
            let n = u32::from_str_radix(digits, 16).ok()?;
            Some(crate::color::hex_to_srgb_f32(n))
        }
        _ => None,
    }
}

/// The sketcher's overlay palette as plain `0xRRGGBB` hex — the ONE place the
/// default color literals live. [`RenderSettings`] stores each of these as an
/// editable [`Rgba`] field (defaults derived from here via [`hex`]) and hands the
/// sketch tessellation/overlay builders a live `SketchColors` view via
/// [`RenderSettings::sketch_colors`], so the sketch renderer reads its colors from
/// the display settings just like faces/edges/vertices do — no scattered constants.
///
/// The `constraint` green is a deliberate user directive (2026-08-22): CONSTRAINT
/// annotations (geometric-constraint glyphs + dimension leaders/labels) read in
/// green so they stand apart from the blue/white sketch GEOMETRY.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SketchColors {
    /// Movable geometry / point (blue).
    pub movable: u32,
    /// Locked geometry / point (near-white).
    pub locked: u32,
    /// No-mobility geometry fallback (yellow).
    pub geometry: u32,
    /// No-mobility point fallback.
    pub point: u32,
    /// Construction point (orange).
    pub construction_point: u32,
    /// Heuristic under-constrained point.
    pub under_constrained_point: u32,
    /// Selected entity — amber (the transform-gizmo accent); beats hover.
    pub selected: u32,
    /// Hovered entity — light blue (brighter than movable).
    pub hovered: u32,
    /// Draw-tool rubber-band preview (dim, so it reads as tentative).
    pub preview: u32,
    /// Constraint annotations — glyphs + dimension leaders/labels (green).
    pub constraint: u32,
    /// A constraint the solver named as part of a CONFLICT — the red of the
    /// status bar's conflict dot, so the two readouts agree at a glance.
    pub conflict: u32,
}

impl Default for SketchColors {
    fn default() -> Self {
        // The previous sketcher's theme defaults — the single source of these literals.
        Self {
            movable: 0x4aa3ff,
            locked: 0xe6ebf2,
            geometry: 0xffff88,
            point: 0x9ec9ff,
            construction_point: 0xffa86a,
            under_constrained_point: 0xffb347,
            selected: 0xffa500,
            hovered: 0x7fd0ff,
            preview: 0x8fa0b8,
            constraint: 0x4ade80,
            conflict: 0xff5c5c,
        }
    }
}

/// The GUI chrome theme (panels, windows, toolbar, text) — controls the egui
/// look, NOT the 3D viewport background (that is the separate `background`
/// setting). `Auto` follows the OS/system theme (`prefers-color-scheme` on web).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThemeMode {
    /// Follow the system's theme preference (falls back to dark when no OS signal).
    Auto,
    Light,
    Dark,
}

/// How a plain viewport click builds a MULTI-selection (the Settings "Multi-select"
/// dropdown). Read by the app's viewport click routing — the engine's selection
/// primitives (`select_candidate` replace / `toggle_candidate` toggle) are
/// mode-agnostic; this only chooses which one a plain click drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MultiSelectMode {
    /// A plain click REPLACES the selection; Ctrl/Cmd+click adds/toggles (the
    /// classic CAD behavior).
    CtrlClick,
    /// A plain click TOGGLES the item in the selection (click again to unselect)
    /// so a multi-selection needs no modifier key; a click on empty space clears.
    ClickToggles,
}

impl MultiSelectMode {
    /// Every mode, in the order the Settings dropdown lists them.
    pub const ALL: [MultiSelectMode; 2] = [MultiSelectMode::CtrlClick, MultiSelectMode::ClickToggles];

    /// The human label — ALSO the serialized `multiSelect` value (the
    /// renderQuality pattern: the dropdown and the stored JSON speak the label;
    /// `apply_json` parses it back tolerantly).
    pub fn label(&self) -> &'static str {
        match self {
            MultiSelectMode::CtrlClick => "Ctrl+Click",
            MultiSelectMode::ClickToggles => "Click toggles",
        }
    }
}

/// How base face color is chosen per solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceColorMode {
    /// The app look: every face gets `face_color` (unless the solid carries a
    /// metadata override).
    Uniform,
    /// The artifact look: stable name-hashed color per solid.
    HashedBySolid,
}

/// The viewer's material palette + display toggles. Defaults are the
/// `CADmaterials` values.
/// Global toolbar appearance. Missing saved values migrate to Ribbon.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ToolbarStyle {
    #[default]
    Ribbon,
    Classic,
}

impl ToolbarStyle {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Ribbon => "Ribbon",
            Self::Classic => "Classic",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RenderSettings {
    /// GUI chrome theme (egui panels/windows/toolbar/text). Defaults to `Auto`,
    /// following the OS light/dark preference.
    pub theme: ThemeMode,
    /// Global UI size scale applied to the whole egui chrome via
    /// [`egui::Context::set_zoom_factor`]. 1.0 = native size; composes with the
    /// device pixel ratio. Clamped to `[0.5, 3.0]`.
    pub ui_scale: f32,
    /// Skip the startup recovery offer without restoring documents automatically.
    /// Autosaving remains enabled for the current session.
    pub disable_recovery_prompt: bool,
    /// Size multiplier for the floating TEXT LABELS the app overlays on the 3D
    /// model — the dimension-gizmo value chips (sketch + feature dimensions), the
    /// assembly-constraint chips, and the transform gizmo's axis letters. 1.0 = the
    /// base monospace size; the label's glyphs, its measured edit-box width, and its
    /// chip padding all scale together (see `brep-app`'s `viewport::labels`). A
    /// MULTIPLIER, not a point size, so labels stay consistent with each other and
    /// the setting survives a restyle. Clamped to `[0.25, 3.0]`: 0.25 is the
    /// smallest label that is still a LABEL rather than a smudge — a quarter of the
    /// 12pt base monospace is a ~3.8pt glyph row inside a ~4.5pt-tall chip, which is
    /// the point at which the double-click/drag target stops being something a
    /// pointer can reliably land on (and it composes with `uiScale` + the device
    /// pixel ratio, so it is not an absolute 3pt on screen). The 3.0 ceiling keeps a
    /// user from filling the viewport with one label. Independent of
    /// [`RenderSettings::ui_scale`], which sizes the egui CHROME (panels/toolbar)
    /// and not the model overlay.
    pub label_scale: f32,
    /// Debug overlay: draw the 1px red outline of each gizmo grab handle's hit
    /// region (arrows/leaders as capsules, balls as circles). Off by default;
    /// toggled from Settings to inspect exactly where a drag will grab.
    pub debug_grab_handles: bool,
    pub background: [f32; 3],
    pub face_color_mode: FaceColorMode,
    /// IGNORE the model's own colours when shading (RENDER-ONLY).
    ///
    /// Bodies and faces carry a durable `color` metadata attribute — stamped by
    /// a STEP import, editable in the Info window — and by default it IS the
    /// shaded colour. Ticking this box makes the viewport fall back to
    /// [`Self::face_color_mode`] (the uniform or name-hashed colour) as though
    /// the model carried no colours at all.
    ///
    /// It is a DISPLAY switch, not an edit: the metadata store is never touched,
    /// so unticking it brings every model colour straight back, and a document
    /// saved with the box ticked still carries all of its colours. Defaults to
    /// false — a coloured model shows its colours.
    pub override_model_colors: bool,
    /// Show the WORKBENCH ACTIONS toolbar — a second strip under the primary
    /// toolbar with one button per feature the active workbench offers (and, where
    /// the Constraints panel is shown, one per assembly-constraint type). Chrome
    /// only: it never changes what the palette or context bar offer. The app
    /// hides the strip while a sketch is being edited regardless of this flag.
    pub show_workbench_toolbar: bool,
    pub toolbar_style: ToolbarStyle,
    pub face_color: Rgba,
    pub face_selected_color: Rgba,
    pub hover_color: Rgba,
    pub edge_color: Rgba,
    pub edge_selected_color: Rgba,
    pub edge_width_px: f32,
    /// Draw the OCCLUDED portions of edges at all — the second, depth-failing
    /// edge pass that [`Self::hidden_edge_alpha`] dims.
    ///
    /// It is a whole extra pass over every edge buffer in the scene, so turning
    /// it off is the single largest display saving the viewer has: measured
    /// 2026-09-11 on a heavy model, frame `draw` fell 33.61 ms -> 24.76 ms
    /// (-26%). The viewport looks materially different with it off — edges
    /// behind a face simply are not there — which is why this is a user's
    /// choice and not a performance default. Defaults to true, the look the
    /// viewer has always had.
    ///
    /// Distinct from setting [`Self::hidden_edge_alpha`] to 0, which also skips
    /// the pass: the alpha is how DIM the occluded edges are, this is WHETHER
    /// they are drawn, so a user can A/B the cost without losing their dimness.
    pub show_occluded_edges: bool,
    /// Occluded edges render dimmed, not dropped (R17): alpha of the
    /// depth-failing edge pass. 0 disables the pass, as does
    /// [`Self::show_occluded_edges`] being false.
    pub hidden_edge_alpha: f32,
    pub vertex_color: Rgba,
    pub vertex_selected_color: Rgba,
    pub vertex_size_px: f32,
    pub flat_shading: bool,
    /// Wireframe display (R14): when true the shaded face-fill pass is skipped so
    /// only edges draw (the CAD wireframe look; back edges show through). Does
    /// NOT affect picking (CPU ray-based) or the overlay pass.
    pub wireframe: bool,
    /// Draw the shaded FACES (R14 companion): false leaves the model as its
    /// edges and vertices alone. Independent of [`Self::wireframe`], which
    /// swaps the shaded fill for the triangle wireframe — with faces off there
    /// is nothing to swap, so both face passes are skipped.
    ///
    /// A DISPLAY switch only: picking is CPU ray-based and unaffected, so a
    /// hidden face still selects. This exists so a presentation capture (and
    /// the toolbar's three visibility toggles) can drop a whole class of
    /// geometry without editing the size settings that describe how the class
    /// is DRAWN — a zeroed `edgeWidthPx` is not "no edges", it is a
    /// zero-width edge the user has to restore by remembering the old number.
    pub show_faces: bool,
    /// Draw the EDGES (both the visible pass and the dimmed occluded one).
    /// See [`Self::show_faces`] for why this is not `edgeWidthPx = 0`.
    pub show_edges: bool,
    /// Draw the VERTEX points. See [`Self::show_faces`].
    pub show_vertices: bool,
    /// World-axis helper (R20): screen length in CSS px; 0 disables.
    pub axis_length_px: f32,
    /// On-screen edge length of the always-on corner ViewCube, in CSS px. Drives
    /// BOTH the rendered mini-camera viewport AND the hit-test corner rect (they
    /// read the same value), so the cube and its clickable region scale together.
    /// Defaults to [`ViewCube::DEFAULT_SIZE_PX`], so the cube is unchanged until edited.
    pub viewcube_size_px: f32,
    pub pick_double_sided: bool,
    /// How a plain viewport click builds a multi-selection (see [`MultiSelectMode`]).
    pub multi_select: MultiSelectMode,
    /// Tessellation LOD factor (1.0 = the app's "Normal" preset).
    pub lod_factor: f64,
    // --- Sketcher overlay palette (managed here like every other display color) ---
    // Defaults come from `SketchColors::default()` (the single source of the
    // literals); `sketch_colors()` re-derives a `SketchColors` view for the
    // tessellation/overlay builders.
    pub sketch_movable_color: Rgba,
    pub sketch_locked_color: Rgba,
    pub sketch_geometry_color: Rgba,
    pub sketch_point_color: Rgba,
    pub sketch_construction_point_color: Rgba,
    pub sketch_under_constrained_point_color: Rgba,
    pub sketch_selected_color: Rgba,
    pub sketch_hovered_color: Rgba,
    pub sketch_preview_color: Rgba,
    pub sketch_constraint_color: Rgba,
    pub sketch_conflict_color: Rgba,
    /// The active UI WORKBENCH id (`"all"` / `"modeling"` / `"sheetMetal"`), a
    /// plain string (NOT an enum) so the app-side per-file workbench registry
    /// stays the sole owner of the valid-id set — adding a workbench never touches
    /// this crate. This is purely a UI FILTER over feature-CREATION: it does not
    /// affect what the history executes or renders. Default `"modeling"`. An
    /// unknown stored id is tolerated here and validated app-side (the registry's
    /// resolver falls back to the default).
    pub workbench: String,
    /// Assembly AUTO-SOLVE: when true (the default) every constraint mutation
    /// re-solves + re-runs immediately; when false the mutation paths only
    /// update state and the user drives the manual Solve button. Consulted by
    /// the app's constraint-mutation path.
    pub assembly_auto_solve: bool,
    /// Show Constraint Graphics: the render toggle the viewport overlay lane
    /// consumes — per-constraint leader/label graphics draw only while this is
    /// on. Owned here so the panel toggle, persistence, and the overlay
    /// renderer all read ONE flag.
    pub show_constraint_graphics: bool,
    /// The BOM's COLUMN CONFIGURATION — the raw text of the Settings panel's
    /// Assemblies textarea (one `[*]prefix.Field` per line). Stored VERBATIM,
    /// exactly as the user typed it: parsing, the known-field catalogue and the
    /// `part.` / `occurrence.` vocabulary all live app-side
    /// (`panels::bom_columns`), so adding a BOM field never touches this crate
    /// — the same division of labour that keeps `workbench` a plain string here.
    ///
    /// Default EMPTY, which means "the app's shipped default configuration".
    /// A document that was never configured therefore persists byte-for-byte as
    /// before, and a later change to the shipped default still reaches every
    /// user who never overrode it.
    pub bom_columns: String,
}

impl Default for RenderSettings {
    fn default() -> Self {
        let sk = SketchColors::default();
        Self {
            // Default to Auto so the chrome follows the OS light/dark preference
            // (egui's `ThemePreference::System`; falls back to dark when there is
            // no OS signal).
            theme: ThemeMode::Auto,
            // 1.0 = native UI size (no zoom); scales the whole egui chrome.
            ui_scale: 1.0,
            disable_recovery_prompt: false,
            // 1.0 = the labels' native monospace size (no scaling).
            label_scale: 1.0,
            // Debug grab-handle outlines are off by default (a diagnostic aid).
            debug_grab_handles: false,
            background: [
                ((0x0b) as f32) / 255.0,
                ((0x0d) as f32) / 255.0,
                ((0x10) as f32) / 255.0,
            ],
            face_color_mode: FaceColorMode::Uniform,
            face_color: hex(0x00009e, 1.0),
            face_selected_color: hex(0xffc400, 1.0),
            hover_color: hex(0xfbff00, 1.0),
            edge_color: hex(0x009dff, 1.0),
            edge_selected_color: hex(0xff00ff, 1.0),
            edge_width_px: 2.0,
            show_occluded_edges: true,
            hidden_edge_alpha: 0.22,
            vertex_color: hex(0x4aff03, 1.0),
            vertex_selected_color: hex(0x00ffff, 1.0),
            vertex_size_px: 6.0,
            flat_shading: false,
            wireframe: false,
            // Everything visible: the three toggles are a subtractive control.
            show_faces: true,
            show_edges: true,
            show_vertices: true,
            override_model_colors: false,
            show_workbench_toolbar: true,
            toolbar_style: ToolbarStyle::Ribbon,
            axis_length_px: 46.0,
            // The corner ViewCube's current on-screen size — the single source of
            // the literal is `ViewCube::DEFAULT_SIZE_PX`, so the settings default and
            // the widget default can never drift.
            viewcube_size_px: brep_gizmos::view_cube::ViewCube::DEFAULT_SIZE_PX,
            pick_double_sided: true,
            multi_select: MultiSelectMode::ClickToggles,
            lod_factor: 1.0,
            // Sketch palette: derive each Rgba from the ONE source of the literals
            // (`SketchColors::default`) so nothing changes visually until edited.
            sketch_movable_color: hex(sk.movable, 1.0),
            sketch_locked_color: hex(sk.locked, 1.0),
            sketch_geometry_color: hex(sk.geometry, 1.0),
            sketch_point_color: hex(sk.point, 1.0),
            sketch_construction_point_color: hex(sk.construction_point, 1.0),
            sketch_under_constrained_point_color: hex(sk.under_constrained_point, 1.0),
            sketch_selected_color: hex(sk.selected, 1.0),
            sketch_hovered_color: hex(sk.hovered, 1.0),
            sketch_preview_color: hex(sk.preview, 1.0),
            sketch_constraint_color: hex(sk.constraint, 1.0),
            sketch_conflict_color: hex(sk.conflict, 1.0),
            // Default workbench: general Modeling.
            workbench: "modeling".to_string(),
            // Assembly: auto-solve every constraint mutation; constraint
            // graphics shown while a document has constraints.
            assembly_auto_solve: true,
            show_constraint_graphics: true,
            // Empty = the app's shipped BOM column configuration.
            bom_columns: String::new(),
        }
    }
}

impl RenderSettings {
    /// The artifact-corpus preset: byte-faithful to the slice-1 artifact look
    /// (per-solid hashed colors, 0x101418 background, 1.6px edges in
    /// 0x0d1030, no hidden-edge pass, no vertices, no axes).
    pub fn artifact() -> Self {
        Self {
            background: [
                ((0x10) as f32) / 255.0,
                ((0x14) as f32) / 255.0,
                ((0x18) as f32) / 255.0,
            ],
            face_color_mode: FaceColorMode::HashedBySolid,
            edge_color: hex(0x0d1030, 1.0),
            edge_width_px: 1.6,
            hidden_edge_alpha: 0.0,
            vertex_size_px: 0.0,
            axis_length_px: 0.0,
            ..Self::default()
        }
    }

    /// Apply a partial JSON override (R3 settings entrypoint). Unknown keys
    /// are ignored; colors are CSS hex strings.
    pub fn apply_json(&mut self, json: &str) -> Result<(), String> {
        let value: serde_json::Value =
            serde_json::from_str(json).map_err(|error| format!("settings parse: {error}"))?;
        let color = |key: &str, target: &mut Rgba| {
            if let Some(v) = value.get(key).and_then(|v| v.as_str()) {
                if let Some(rgb) = parse_css_hex(v) {
                    target[0] = rgb[0];
                    target[1] = rgb[1];
                    target[2] = rgb[2];
                }
            }
        };
        color("faceColor", &mut self.face_color);
        color("faceSelectedColor", &mut self.face_selected_color);
        color("hoverColor", &mut self.hover_color);
        color("edgeColor", &mut self.edge_color);
        color("edgeSelectedColor", &mut self.edge_selected_color);
        color("vertexColor", &mut self.vertex_color);
        color("vertexSelectedColor", &mut self.vertex_selected_color);
        // Sketcher overlay palette (same CSS-hex shape as the face/edge/vertex colors).
        color("sketchMovableColor", &mut self.sketch_movable_color);
        color("sketchLockedColor", &mut self.sketch_locked_color);
        color("sketchGeometryColor", &mut self.sketch_geometry_color);
        color("sketchPointColor", &mut self.sketch_point_color);
        color("sketchConstructionPointColor", &mut self.sketch_construction_point_color);
        color("sketchUnderConstrainedPointColor", &mut self.sketch_under_constrained_point_color);
        color("sketchSelectedColor", &mut self.sketch_selected_color);
        color("sketchHoveredColor", &mut self.sketch_hovered_color);
        color("sketchPreviewColor", &mut self.sketch_preview_color);
        color("sketchConstraintColor", &mut self.sketch_constraint_color);
        color("sketchConflictColor", &mut self.sketch_conflict_color);
        if let Some(v) = value.get("background").and_then(|v| v.as_str()) {
            if let Some(rgb) = parse_css_hex(v) {
                self.background = rgb;
            }
        }
        if let Some(v) = value.get("edgeWidthPx").and_then(|v| v.as_f64()) {
            self.edge_width_px = (v as f32).clamp(0.0, 32.0);
        }
        if let Some(v) = value.get("vertexSizePx").and_then(|v| v.as_f64()) {
            self.vertex_size_px = (v as f32).clamp(0.0, 64.0);
        }
        if let Some(v) = value.get("showOccludedEdges").and_then(|v| v.as_bool()) {
            self.show_occluded_edges = v;
        }
        if let Some(v) = value.get("hiddenEdgeAlpha").and_then(|v| v.as_f64()) {
            self.hidden_edge_alpha = (v as f32).clamp(0.0, 1.0);
        }
        if let Some(v) = value.get("faceColorMode").and_then(|v| v.as_str()) {
            match v.trim().to_ascii_lowercase().as_str() {
                "hashedbysolid" | "hashed" => self.face_color_mode = FaceColorMode::HashedBySolid,
                "uniform" => self.face_color_mode = FaceColorMode::Uniform,
                _ => {}
            }
        }
        if let Some(v) = value.get("theme").and_then(|v| v.as_str()) {
            match v.trim().to_ascii_lowercase().as_str() {
                "auto" => self.theme = ThemeMode::Auto,
                "light" => self.theme = ThemeMode::Light,
                "dark" => self.theme = ThemeMode::Dark,
                _ => {}
            }
        }
        if let Some(v) = value.get("uiScale").and_then(|v| v.as_f64()) {
            self.ui_scale = (v as f32).clamp(0.5, 3.0);
        }
        if let Some(v) = value.get("disableRecoveryPrompt").and_then(|v| v.as_bool()) {
            self.disable_recovery_prompt = v;
        }
        // Model-overlay label size. Clamped to the SAME [0.25, 3.0] domain the
        // settings slider offers, so a persisted value never silently re-clamps on
        // reload (the `viewcubeSizePx` rule).
        if let Some(v) = value.get("labelScale").and_then(|v| v.as_f64()) {
            self.label_scale = (v as f32).clamp(0.25, 3.0);
        }
        if let Some(v) = value.get("debugGrabHandles").and_then(|v| v.as_bool()) {
            self.debug_grab_handles = v;
        }
        if let Some(v) = value.get("flatShading").and_then(|v| v.as_bool()) {
            self.flat_shading = v;
        }
        if let Some(v) = value.get("wireframe").and_then(|v| v.as_bool()) {
            self.wireframe = v;
        }
        if let Some(v) = value.get("showFaces").and_then(|v| v.as_bool()) {
            self.show_faces = v;
        }
        if let Some(v) = value.get("showEdges").and_then(|v| v.as_bool()) {
            self.show_edges = v;
        }
        if let Some(v) = value.get("showVertices").and_then(|v| v.as_bool()) {
            self.show_vertices = v;
        }
        if let Some(v) = value.get("overrideModelColors").and_then(|v| v.as_bool()) {
            self.override_model_colors = v;
        }
        if let Some(v) = value.get("toolbarStyle").and_then(|v| v.as_str()) {
            self.toolbar_style = match v {
                "Ribbon" => ToolbarStyle::Ribbon,
                "Classic" => ToolbarStyle::Classic,
                _ => return Err(format!("unknown toolbar style: {v}")),
            };
        }
        if let Some(v) = value.get("showWorkbenchToolbar").and_then(|v| v.as_bool()) {
            self.show_workbench_toolbar = v;
        }
        if let Some(v) = value.get("axisLengthPx").and_then(|v| v.as_f64()) {
            self.axis_length_px = (v as f32).clamp(0.0, 512.0);
        }
        // ViewCube corner size — clamp to the SAME [40, 230] domain the settings
        // slider offers, so a persisted value never silently re-clamps on reload.
        if let Some(v) = value.get("viewcubeSizePx").and_then(|v| v.as_f64()) {
            self.viewcube_size_px = (v as f32).clamp(40.0, 230.0);
        }
        if let Some(v) = value.get("pickDoubleSided").and_then(|v| v.as_bool()) {
            self.pick_double_sided = v;
        }
        // The multi-select mode dropdown serializes its human label (the
        // renderQuality pattern); match on the alphanumeric skeleton so
        // "Ctrl+Click" / "ctrlClick" / "CTRL CLICK" all parse.
        if let Some(v) = value.get("multiSelect").and_then(|v| v.as_str()) {
            let skeleton: String = v
                .chars()
                .filter(|c| c.is_ascii_alphanumeric())
                .collect::<String>()
                .to_ascii_lowercase();
            match skeleton.as_str() {
                "ctrlclick" => self.multi_select = MultiSelectMode::CtrlClick,
                "clicktoggles" => self.multi_select = MultiSelectMode::ClickToggles,
                _ => {}
            }
        }
        // The active UI workbench id (a plain string; the app validates it against
        // its registry). Stored verbatim — an unknown id is tolerated here.
        if let Some(v) = value.get("workbench").and_then(|v| v.as_str()) {
            self.workbench = v.to_string();
        }
        if let Some(v) = value.get("assemblyAutoSolve").and_then(|v| v.as_bool()) {
            self.assembly_auto_solve = v;
        }
        // The BOM column configuration, verbatim (see the field docs) — never
        // normalized here, so a malformed line survives a save/reload and the
        // panel can still point at the line the user has to fix.
        if let Some(v) = value.get("bomColumns").and_then(|v| v.as_str()) {
            self.bom_columns = v.to_string();
        }
        if let Some(v) = value.get("showConstraintGraphics").and_then(|v| v.as_bool()) {
            self.show_constraint_graphics = v;
        }
        // "Render Quality" is a named dropdown (Draft…Ultra) mapping to the display
        // LOD factor (higher quality = finer mesh = smaller factor). We store the
        // resolved f64 so the tessellation path is unchanged.
        if let Some(label) = value.get("renderQuality").and_then(|v| v.as_str()) {
            if let Some(lod) = lod_from_quality(label) {
                self.lod_factor = lod;
            }
        }
        Ok(())
    }

    /// Serialize EVERY setting to the SAME camelCase / CSS-hex shape
    /// [`apply_json`] reads, so `s.apply_json(&s.to_json())` is the identity.
    /// This is the counterpart the settings schema serializes/persists through
    /// (there was previously no serializer, only the partial-override reader).
    pub fn to_json(&self) -> String {
        serde_json::json!({
            "theme": match self.theme {
                ThemeMode::Auto => "auto",
                ThemeMode::Light => "light",
                ThemeMode::Dark => "dark",
            },
            "uiScale": self.ui_scale as f64,
            "disableRecoveryPrompt": self.disable_recovery_prompt,
            "labelScale": self.label_scale as f64,
            "debugGrabHandles": self.debug_grab_handles,
            "background": rgb_to_css_hex(self.background),
            "faceColorMode": match self.face_color_mode {
                FaceColorMode::Uniform => "uniform",
                FaceColorMode::HashedBySolid => "hashedBySolid",
            },
            "faceColor": rgba_to_css_hex(self.face_color),
            "faceSelectedColor": rgba_to_css_hex(self.face_selected_color),
            "hoverColor": rgba_to_css_hex(self.hover_color),
            "edgeColor": rgba_to_css_hex(self.edge_color),
            "edgeSelectedColor": rgba_to_css_hex(self.edge_selected_color),
            "edgeWidthPx": self.edge_width_px as f64,
            "showOccludedEdges": self.show_occluded_edges,
            "hiddenEdgeAlpha": self.hidden_edge_alpha as f64,
            "vertexColor": rgba_to_css_hex(self.vertex_color),
            "vertexSelectedColor": rgba_to_css_hex(self.vertex_selected_color),
            "vertexSizePx": self.vertex_size_px as f64,
            "flatShading": self.flat_shading,
            "wireframe": self.wireframe,
            "showFaces": self.show_faces,
            "showEdges": self.show_edges,
            "showVertices": self.show_vertices,
            "overrideModelColors": self.override_model_colors,
            "showWorkbenchToolbar": self.show_workbench_toolbar,
            "toolbarStyle": self.toolbar_style.as_str(),
            "axisLengthPx": self.axis_length_px as f64,
            "viewcubeSizePx": self.viewcube_size_px as f64,
            "pickDoubleSided": self.pick_double_sided,
            "multiSelect": self.multi_select.label(),
            "renderQuality": quality_from_lod(self.lod_factor),
            "sketchMovableColor": rgba_to_css_hex(self.sketch_movable_color),
            "sketchLockedColor": rgba_to_css_hex(self.sketch_locked_color),
            "sketchGeometryColor": rgba_to_css_hex(self.sketch_geometry_color),
            "sketchPointColor": rgba_to_css_hex(self.sketch_point_color),
            "sketchConstructionPointColor": rgba_to_css_hex(self.sketch_construction_point_color),
            "sketchUnderConstrainedPointColor": rgba_to_css_hex(self.sketch_under_constrained_point_color),
            "sketchSelectedColor": rgba_to_css_hex(self.sketch_selected_color),
            "sketchHoveredColor": rgba_to_css_hex(self.sketch_hovered_color),
            "sketchPreviewColor": rgba_to_css_hex(self.sketch_preview_color),
            "sketchConstraintColor": rgba_to_css_hex(self.sketch_constraint_color),
            "sketchConflictColor": rgba_to_css_hex(self.sketch_conflict_color),
            "workbench": self.workbench,
            "assemblyAutoSolve": self.assembly_auto_solve,
            "showConstraintGraphics": self.show_constraint_graphics,
            "bomColumns": self.bom_columns,
        })
        .to_string()
    }

    /// The live [`SketchColors`] view of the sketcher palette — the tessellation /
    /// overlay builders read their colors from THIS (via
    /// [`crate::sketch::SketchSession::colors`]) so the display settings are the one
    /// source of truth. Each `Rgba` is quantized back to `0xRRGGBB` the SAME way
    /// [`rgb_to_css_hex`] serializes it, so a default settings value round-trips to
    /// the default `SketchColors` byte-exact.
    pub fn sketch_colors(&self) -> SketchColors {
        SketchColors {
            movable: rgba_to_u32(self.sketch_movable_color),
            locked: rgba_to_u32(self.sketch_locked_color),
            geometry: rgba_to_u32(self.sketch_geometry_color),
            point: rgba_to_u32(self.sketch_point_color),
            construction_point: rgba_to_u32(self.sketch_construction_point_color),
            under_constrained_point: rgba_to_u32(self.sketch_under_constrained_point_color),
            selected: rgba_to_u32(self.sketch_selected_color),
            hovered: rgba_to_u32(self.sketch_hovered_color),
            preview: rgba_to_u32(self.sketch_preview_color),
            constraint: rgba_to_u32(self.sketch_constraint_color),
            conflict: rgba_to_u32(self.sketch_conflict_color),
        }
    }

    /// The settings schema WITH the current value baked into each field, as JSON
    /// — mirrors the kernel's `feature_schemas_json` export so a UI shell (egui
    /// here, a later `brep-ui` crate) can generate the whole form from data. The
    /// `value` of each field is pulled live from [`to_json`], so the export
    /// always reflects the current settings.
    pub fn settings_schema_json(&self) -> String {
        let current: serde_json::Value =
            serde_json::from_str(&self.to_json()).unwrap_or(serde_json::Value::Null);
        let fields: Vec<serde_json::Value> = settings_schema()
            .iter()
            .map(|field| {
                let kind = match &field.kind {
                    FieldKind::Color => serde_json::json!({ "type": "color" }),
                    FieldKind::Bool => serde_json::json!({ "type": "bool" }),
                    FieldKind::Enum { variants } => {
                        serde_json::json!({ "type": "enum", "variants": variants })
                    }
                    FieldKind::Number { min, max, step } => serde_json::json!({
                        "type": "number", "min": min, "max": max, "step": step
                    }),
                    FieldKind::Range { min, max, step } => serde_json::json!({
                        "type": "range", "min": min, "max": max, "step": step
                    }),
                    // The feature-dialog kinds never appear in the settings
                    // schema, but the match must stay exhaustive.
                    FieldKind::Scalar { step } => {
                        serde_json::json!({ "type": "scalar", "step": step })
                    }
                    FieldKind::BoundedScalar { min, max, step } => serde_json::json!({
                        "type": "scalar", "min": min, "max": max, "step": step
                    }),
                    FieldKind::Text { read_only } => {
                        serde_json::json!({ "type": "text", "readOnly": read_only })
                    }
                    FieldKind::Vec3 { step } => serde_json::json!({ "type": "vec3", "step": step }),
                    FieldKind::Reference { filter, multiple } => serde_json::json!({
                        "type": "reference", "filter": filter, "multiple": multiple
                    }),
                    FieldKind::Button { label } => {
                        serde_json::json!({ "type": "button", "label": label })
                    }
                };
                serde_json::json!({
                    "key": field.key,
                    "label": field.label,
                    "group": field.group,
                    "kind": kind,
                    "value": current.get(field.key),
                })
            })
            .collect();
        serde_json::json!({ "fields": fields }).to_string()
    }
}

/// Quantize an sRGB channel triple to a `#rrggbb` CSS hex string (the shape
/// [`RenderSettings::apply_json`] parses back).
fn rgb_to_css_hex(rgb: [f32; 3]) -> String {
    format!("#{:06x}", rgba_to_u32([rgb[0], rgb[1], rgb[2], 1.0]))
}

/// Like [`rgb_to_css_hex`] but for an `Rgba` (the alpha is intentionally dropped
/// — `apply_json` only overrides the rgb channels, preserving existing alpha).
fn rgba_to_css_hex(rgba: Rgba) -> String {
    rgb_to_css_hex([rgba[0], rgba[1], rgba[2]])
}

/// Clamp and round sRGB channels to packed `0xRRGGBB`, dropping alpha.
/// Shared by CSS serialization and the sketch palette.
fn rgba_to_u32(rgba: Rgba) -> u32 {
    let q = |c: f32| (c.clamp(0.0, 1.0) * 255.0).round() as u32;
    (q(rgba[0]) << 16) | (q(rgba[1]) << 8) | q(rgba[2])
}

/// The kind of one settings field — the closed set of widget shapes the generic
/// form renderer knows how to emit. Plain data, NO egui dependency: the schema
/// lives in the engine, the renderer (brep-app / a later brep-ui) walks it.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldKind {
    /// An sRGB color (`#rrggbb`) → a color picker button.
    Color,
    /// A boolean toggle → a checkbox.
    Bool,
    /// A closed choice → a combo box. `variants` are the JSON string values
    /// (exactly what `apply_json` accepts / `to_json` emits).
    Enum { variants: Vec<String> },
    /// A bounded number → a slider / drag-value honoring `min`/`max`/`step`.
    Number { min: f64, max: f64, step: f64 },
    /// A 0..1 (or otherwise fractional) number → a slider honoring the bounds.
    Range { min: f64, max: f64, step: f64 },
    // --- extra kinds the FEATURE dialogs need (settings never use these) -------
    /// An UNBOUNDED number (feature params carry no min/max) → a drag value.
    Scalar { step: f64 },
    /// A scalar text field whose numeric edits are bounded on commit. Drawing
    /// the field never clamps or quantizes its existing value.
    BoundedScalar { min: f64, max: f64, step: f64 },
    /// A single-line text edit. `read_only` protects identity fields whose
    /// renaming must also update references.
    Text { read_only: bool },
    /// A 3-vector (position / rotationEuler / scale) → three drag values.
    Vec3 { step: f64 },
    /// A reference-selection field for viewport picking. `filter` limits entity
    /// kinds (e.g. `["SOLID"]`); `multiple` allows a list of selections.
    Reference { filter: Vec<String>, multiple: bool },
    /// An ACTION button (schema `"type":"button"`) → a clickable button. It binds
    /// to no value; a click is surfaced to the caller by the field `key` (e.g.
    /// `editSketch`), which the host acts on. `label` is the button caption.
    Button { label: String },
}

/// One field of the settings form: the camelCase `apply_json`/`to_json` key, a
/// human label, a UI group, and the widget `kind`. The ordered list of these
/// (see [`settings_schema`]) fully describes the form — a UI shell generates a
/// widget per field with no per-field code.
#[derive(Debug, Clone)]
pub struct SettingsField {
    /// The camelCase key — the SAME one `apply_json`/`to_json` use.
    pub key: &'static str,
    pub label: &'static str,
    pub group: &'static str,
    pub kind: FieldKind,
}

/// The GENERAL form field — the schema element ONE form engine renders for BOTH
/// the display-settings dialog AND the schema-driven feature dialogs. Unlike
/// [`SettingsField`] (which uses `&'static str` because the settings schema is
/// compile-time), this owns its strings so it can carry a per-feature schema
/// pulled from the kernel catalogue at run time, and it carries a `path` (a
/// chain of JSON object keys) so a field can bind to a NESTED value —
/// `["transform","position"]`, `["boolean","operation"]` — not just a top-level
/// key. The settings form is `path == [key]`.
#[derive(Debug, Clone)]
pub struct FormField {
    /// JSON object-key chain into the document the form edits (≥ 1 segment).
    pub path: Vec<String>,
    pub label: String,
    pub group: String,
    pub kind: FieldKind,
}

impl FormField {
    /// The last path segment — a stable per-field id for egui salting / probing.
    pub fn key(&self) -> &str {
        self.path.last().map(String::as_str).unwrap_or("")
    }
}

/// The "Render Quality" dropdown levels shown in Settings, each mapped to a
/// display-tessellation LOD factor. Higher quality = finer mesh = SMALLER factor
/// (chord tolerance = extent · 1.5e-3 · factor). Order is coarse→fine, the order
/// the ComboBox lists them. `lod_factor` stays the internal representation the
/// tessellation path reads; only the SETTINGS UI/serialization speaks in levels.
pub const RENDER_QUALITY: &[(&str, f64)] = &[
    ("Draft", 4.0),
    ("Low", 2.0),
    ("Medium", 1.0),
    ("High", 0.5),
    ("Ultra", 0.25),
];

/// The LOD factor for a quality label (`None` if not a known level).
fn lod_from_quality(label: &str) -> Option<f64> {
    RENDER_QUALITY
        .iter()
        .find(|(name, _)| *name == label)
        .map(|(_, factor)| *factor)
}

/// The quality label NEAREST a LOD factor — the serialization inverse of
/// [`lod_from_quality`]. A stored factor is always one of the level values in
/// normal use; nearest-match keeps a hand-set/legacy value mapping to a sane label.
fn quality_from_lod(lod: f64) -> &'static str {
    RENDER_QUALITY
        .iter()
        .min_by(|(_, a), (_, b)| {
            (a - lod).abs().total_cmp(&(b - lod).abs())
        })
        .map(|(name, _)| *name)
        .unwrap_or("Medium")
}

/// Lift the compile-time display-settings schema into general [`FormField`]s so
/// the ONE `field_input` engine (which the feature dialogs also use) renders the
/// settings panel too — a single schema-driven dialog engine, not two.
pub fn settings_form_fields() -> Vec<FormField> {
    settings_schema()
        .into_iter()
        .map(|field| FormField {
            path: vec![field.key.to_string()],
            label: field.label.to_string(),
            group: field.group.to_string(),
            kind: field.kind,
        })
        .collect()
}

/// The Rust-owned settings schema: the ordered list of display-settings fields,
/// grouped, analogous to the kernel feature schemas. Adding a field here (plus
/// its `apply_json`/`to_json` handling) makes the whole UI grow a widget for it
/// with ZERO renderer changes.
pub fn settings_schema() -> Vec<SettingsField> {
    let f = |key, label, group, kind| SettingsField { key, label, group, kind };
    vec![
        // --- Appearance ----------------------------------------------------
        // FIRST so the "Appearance" group renders at the TOP of the settings tree
        // (groups are ordered by first field appearance). The GUI-chrome theme —
        // NOT the 3D viewport background (that lives under Scene).
        f(
            "theme",
            "Theme",
            "Appearance",
            FieldKind::Enum { variants: ["auto", "light", "dark"].iter().map(|s| s.to_string()).collect() },
        ),
        // A SLIDER (both `Range` and `Number` render an `egui::Slider` over the
        // given domain) scaling the whole egui UI via `Context::set_zoom_factor`.
        f(
            "uiScale",
            "UI scale",
            "Appearance",
            FieldKind::Range { min: 0.5, max: 3.0, step: 0.05 },
        ),
        // The size of the floating text labels overlaid on the MODEL (dimension
        // value chips, constraint chips, gizmo axis letters) — a multiplier over
        // their base monospace size, NOT a point size. Bounds MATCH the
        // `apply_json` clamp so the slider can't set a value that re-clamps on
        // reload. The 0.25 floor (a quarter of the base, on the 0.05 step) is the
        // smallest chip whose click/drag target a pointer can still land on — see
        // `RenderSettings::label_scale`. Separate from `uiScale`, which sizes the
        // egui chrome.
        f(
            "labelScale",
            "Label scale",
            "Appearance",
            FieldKind::Range { min: 0.25, max: 3.0, step: 0.05 },
        ),
        // The workbench actions toolbar (the feature-button strip under the
        // primary toolbar). Chrome, so it sits with the other chrome settings.
        f(
            "showWorkbenchToolbar",
            "Show workbench actions toolbar",
            "Appearance",
            FieldKind::Bool,
        ),
        f("toolbarStyle", "Toolbar style", "Appearance",
          FieldKind::Enum { variants: ["Ribbon", "Classic"].iter().map(|s| s.to_string()).collect() }),
        // --- Startup -------------------------------------------------------
        f(
            "disableRecoveryPrompt",
            "Disable recovery dialog on startup",
            "Startup",
            FieldKind::Bool,
        ),
        // --- Scene ---------------------------------------------------------
        f("background", "Background", "Scene", FieldKind::Color),
        f(
            "axisLengthPx",
            "Axis length (px)",
            "Scene",
            FieldKind::Number { min: 0.0, max: 512.0, step: 1.0 },
        ),
        // The corner ViewCube's on-screen size. Bounds MATCH the `apply_json` clamp
        // ([40, 230], centered on the 135px default) so the slider can't set a value
        // that re-clamps on reload. Renders as an `egui::Slider`.
        f(
            "viewcubeSizePx",
            "ViewCube size (px)",
            "Scene",
            FieldKind::Number { min: 40.0, max: 230.0, step: 1.0 },
        ),
        f(
            "renderQuality",
            "Render Quality",
            "Scene",
            FieldKind::Enum { variants: RENDER_QUALITY.iter().map(|(label, _)| label.to_string()).collect() },
        ),
        // --- Faces ---------------------------------------------------------
        f(
            "faceColorMode",
            "Face color mode",
            "Faces",
            FieldKind::Enum { variants: ["uniform", "hashedBySolid"].iter().map(|s| s.to_string()).collect() },
        ),
        f("faceColor", "Face color", "Faces", FieldKind::Color),
        f("faceSelectedColor", "Selected face", "Faces", FieldKind::Color),
        f("hoverColor", "Hover", "Faces", FieldKind::Color),
        f("flatShading", "Flat shading", "Faces", FieldKind::Bool),
        f("wireframe", "Wireframe", "Faces", FieldKind::Bool),
        f("showFaces", "Show faces", "Faces", FieldKind::Bool),
        f(
            "overrideModelColors",
            "Override model colors",
            "Faces",
            FieldKind::Bool,
        ),
        // --- Edges ---------------------------------------------------------
        f("showEdges", "Show edges", "Edges", FieldKind::Bool),
        f("edgeColor", "Edge color", "Edges", FieldKind::Color),
        f("edgeSelectedColor", "Selected edge", "Edges", FieldKind::Color),
        f(
            "edgeWidthPx",
            "Edge width (px)",
            "Edges",
            FieldKind::Number { min: 0.0, max: 32.0, step: 0.1 },
        ),
        f("showOccludedEdges", "Show occluded edges", "Edges", FieldKind::Bool),
        f(
            "hiddenEdgeAlpha",
            "Hidden-edge alpha",
            "Edges",
            FieldKind::Range { min: 0.0, max: 1.0, step: 0.01 },
        ),
        // --- Vertices ------------------------------------------------------
        f("showVertices", "Show vertices", "Vertices", FieldKind::Bool),
        f("vertexColor", "Vertex color", "Vertices", FieldKind::Color),
        f("vertexSelectedColor", "Selected vertex", "Vertices", FieldKind::Color),
        f(
            "vertexSizePx",
            "Vertex size (px)",
            "Vertices",
            FieldKind::Number { min: 0.0, max: 64.0, step: 0.5 },
        ),
        // --- Picking -------------------------------------------------------
        f("pickDoubleSided", "Pick double-sided", "Picking", FieldKind::Bool),
        // How a plain viewport click builds a multi-selection: the classic
        // Ctrl+Click add, or modifier-free click-toggles (click an item to add
        // it, click it again to remove it).
        f(
            "multiSelect",
            "Multi-select",
            "Picking",
            FieldKind::Enum {
                variants: MultiSelectMode::ALL.iter().map(|m| m.label().to_string()).collect(),
            },
        ),
        // --- Sketch --------------------------------------------------------
        // The sketcher overlay palette, editable live like every other display
        // color. `sketch_colors()` feeds these to the tessellation/overlay builders.
        f("sketchMovableColor", "Movable geometry", "Sketch", FieldKind::Color),
        f("sketchLockedColor", "Locked geometry", "Sketch", FieldKind::Color),
        f("sketchGeometryColor", "Geometry (no mobility)", "Sketch", FieldKind::Color),
        f("sketchPointColor", "Point", "Sketch", FieldKind::Color),
        f("sketchConstructionPointColor", "Construction point", "Sketch", FieldKind::Color),
        f("sketchUnderConstrainedPointColor", "Under-constrained point", "Sketch", FieldKind::Color),
        f("sketchSelectedColor", "Selected", "Sketch", FieldKind::Color),
        f("sketchHoveredColor", "Hovered", "Sketch", FieldKind::Color),
        f("sketchPreviewColor", "Draw preview", "Sketch", FieldKind::Color),
        f("sketchConstraintColor", "Constraint / dimension", "Sketch", FieldKind::Color),
        f("sketchConflictColor", "Conflicting constraint", "Sketch", FieldKind::Color),
        // --- Debug ---------------------------------------------------------
        // LAST so the "Debug" group renders at the BOTTOM of the settings tree.
        f("debugGrabHandles", "Debug grab handles", "Debug", FieldKind::Bool),
    ]
}

/// A selected/hovered vertex reference: vertices have no kernel names, so they
/// resolve by owning solid + position.
#[derive(Debug, Clone)]
pub struct VertexRef {
    pub solid: String,
    pub position: [f64; 3],
}

/// A set of entity NAMES that remembers the order they were ADDED in — the
/// selection's pick order.
///
/// The selection used to be a plain `HashSet`, so which of two picked faces
/// came out first was the hash seed's choice and differed between processes.
/// That is not cosmetic: an assembly Distance takes element 0 as its BASE face,
/// whose outward normal gives the value its sign and the arrow its direction,
/// so the same two picks could seed opposite constraints in two runs. Every
/// reader of the selection — [`EngineState::selection_json`], the context bar's
/// feature offers, the constraint seeder — now sees the picks in the order they
/// were made.
///
/// [`insert`](Self::insert) appends, and a name already in the set keeps its
/// place; [`remove`](Self::remove) keeps the rest in order. Toggling a name off
/// and on therefore moves it to the END, which is where a re-pick belongs.
/// Membership rides in a `HashSet` beside the order, because `contains` is
/// called for every face and every edge of every solid, every frame, by the
/// emphasis colour feed.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct OrderedNames {
    order: Vec<String>,
    index: HashSet<String>,
}

impl OrderedNames {
    /// Add `name` at the END. Returns whether it was not already there (the
    /// `HashSet::insert` contract), and an already-present name does NOT move.
    pub fn insert(&mut self, name: String) -> bool {
        if !self.index.insert(name.clone()) {
            return false;
        }
        self.order.push(name);
        true
    }

    /// Drop `name`, keeping the order of everything else. Returns whether it
    /// was there (the `HashSet::remove` contract).
    pub fn remove(&mut self, name: &str) -> bool {
        if !self.index.remove(name) {
            return false;
        }
        self.order.retain(|n| n != name);
        true
    }

    pub fn contains(&self, name: &str) -> bool {
        self.index.contains(name)
    }

    pub fn clear(&mut self) {
        self.order.clear();
        self.index.clear();
    }

    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    /// The names in PICK ORDER, oldest first.
    pub fn iter(&self) -> std::slice::Iter<'_, String> {
        self.order.iter()
    }
}

impl FromIterator<String> for OrderedNames {
    fn from_iter<I: IntoIterator<Item = String>>(iter: I) -> Self {
        let mut out = Self::default();
        for name in iter {
            out.insert(name);
        }
        out
    }
}

impl Extend<String> for OrderedNames {
    fn extend<I: IntoIterator<Item = String>>(&mut self, iter: I) {
        for name in iter {
            self.insert(name);
        }
    }
}

impl<'a> IntoIterator for &'a OrderedNames {
    type Item = &'a String;
    type IntoIter = std::slice::Iter<'a, String>;
    fn into_iter(self) -> Self::IntoIter {
        self.order.iter()
    }
}

/// The emphasis state (selection + hover), name-keyed like `SelectionFilter`.
/// Solid-level emphasis cascades to that solid's faces/edges (the
/// `SelectionState._applyToSolid` behavior).
#[derive(Debug, Default, Clone)]
pub struct Emphasis {
    pub selected_solids: OrderedNames,
    pub selected_faces: OrderedNames,
    pub selected_edges: OrderedNames,
    pub selected_vertices: Vec<VertexRef>,
    /// Selected construction datum/plane FRAMES, keyed by frame NAME (`{id}:XY`
    /// for a DATUM base plane, `{id}` for a PLANE feature). Datums carry no
    /// resident geometry, so — like the render-color feed — they emphasize purely
    /// by name; a selected datum is re-colored with the selection accent when the
    /// engine re-feeds the datum planes.
    pub selected_datums: OrderedNames,
    pub hovered_solids: HashSet<String>,
    pub hovered_faces: HashSet<String>,
    pub hovered_edges: HashSet<String>,
    pub hovered_vertices: Vec<VertexRef>,
    /// HOVERED construction datum/plane FRAMES — the hover twin of
    /// [`selected_datums`](Self::selected_datums), keyed the same way. Construction
    /// planes are ordinary pick candidates, so the pointer (and a pick-list row)
    /// pre-highlights one exactly like a face; the accent is applied when the engine
    /// re-feeds the datum planes.
    pub hovered_datums: HashSet<String>,
    /// Bumped on every change — cache key for derived GPU state.
    pub generation: u64,
}

impl Emphasis {
    pub fn is_empty(&self) -> bool {
        self.selected_solids.is_empty()
            && self.selected_faces.is_empty()
            && self.selected_edges.is_empty()
            && self.selected_vertices.is_empty()
            && self.selected_datums.is_empty()
            && self.hovered_solids.is_empty()
            && self.hovered_faces.is_empty()
            && self.hovered_edges.is_empty()
            && self.hovered_vertices.is_empty()
            && self.hovered_datums.is_empty()
    }

    /// Replace the whole emphasis state from the R3 JSON shape:
    /// `{selected: {solids, faces, edges, vertices:[{solid,position}]}, hovered: {...}}`.
    pub fn apply_json(&mut self, json: &str) -> Result<(), String> {
        let value: serde_json::Value =
            serde_json::from_str(json).map_err(|error| format!("emphasis parse: {error}"))?;
        let names = |group: &serde_json::Value, key: &str| -> HashSet<String> {
            crate::json_support::string_values(group.get(key))
                .map(str::to_string)
                .collect()
        };
        // The SELECTED buckets keep the order they are given: a saved selection
        // comes back in the order it was picked in.
        let ordered = |group: &serde_json::Value, key: &str| -> OrderedNames {
            crate::json_support::string_values(group.get(key))
                .map(str::to_string)
                .collect()
        };
        let vertices = |group: &serde_json::Value| -> Vec<VertexRef> {
            group
                .get("vertices")
                .and_then(|v| v.as_array())
                .map(|list| {
                    list.iter()
                        .filter_map(|v| {
                            let solid = v.get("solid")?.as_str()?.to_string();
                            let p = v.get("position")?.as_array()?;
                            Some(VertexRef {
                                solid,
                                position: [
                                    p.first()?.as_f64()?,
                                    p.get(1)?.as_f64()?,
                                    p.get(2)?.as_f64()?,
                                ],
                            })
                        })
                        .collect()
                })
                .unwrap_or_default()
        };
        let empty = serde_json::json!({});
        let selected = value.get("selected").unwrap_or(&empty);
        let hovered = value.get("hovered").unwrap_or(&empty);
        self.selected_solids = ordered(selected, "solids");
        self.selected_faces = ordered(selected, "faces");
        self.selected_edges = ordered(selected, "edges");
        self.selected_datums = ordered(selected, "datums");
        self.selected_vertices = vertices(selected);
        self.hovered_solids = names(hovered, "solids");
        self.hovered_faces = names(hovered, "faces");
        self.hovered_edges = names(hovered, "edges");
        self.hovered_datums = names(hovered, "datums");
        self.hovered_vertices = vertices(hovered);
        self.generation = self.generation.wrapping_add(1);
        Ok(())
    }
}

/// The visual state of one displayed face/edge (hover wins over selected, the
/// `SelectionState` order).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmphasisState {
    Base,
    Selected,
    Hovered,
}

impl Emphasis {
    pub fn face_state(&self, solid: &str, face: &str) -> EmphasisState {
        if self.hovered_solids.contains(solid) || (!face.is_empty() && self.hovered_faces.contains(face)) {
            EmphasisState::Hovered
        } else if self.selected_solids.contains(solid)
            || (!face.is_empty() && self.selected_faces.contains(face))
        {
            EmphasisState::Selected
        } else {
            EmphasisState::Base
        }
    }

    pub fn edge_state(&self, solid: &str, edge: &str) -> EmphasisState {
        if self.hovered_solids.contains(solid) || (!edge.is_empty() && self.hovered_edges.contains(edge)) {
            EmphasisState::Hovered
        } else if self.selected_solids.contains(solid)
            || (!edge.is_empty() && self.selected_edges.contains(edge))
        {
            EmphasisState::Selected
        } else {
            EmphasisState::Base
        }
    }

    pub fn vertex_state(&self, solid: &str, position: [f64; 3], tol: f64) -> EmphasisState {
        let matches = |refs: &[VertexRef]| {
            refs.iter().any(|r| {
                r.solid == solid
                    && (r.position[0] - position[0]).abs() <= tol
                    && (r.position[1] - position[1]).abs() <= tol
                    && (r.position[2] - position[2]).abs() <= tol
            })
        };
        if matches(&self.hovered_vertices) {
            EmphasisState::Hovered
        } else if matches(&self.selected_vertices) {
            EmphasisState::Selected
        } else {
            EmphasisState::Base
        }
    }
}


