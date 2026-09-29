//! Auto Constraints window — the Assembly workbench's `⚿` toolbar button.
//!
//! An imported STEP assembly is fully posed and completely unconstrained: every
//! part sits where the file put it, and nothing holds it there. This window
//! reads that placement back as constraint intent — the kernel's inference lane
//! ([`brep_kernel::feature_pipeline::assembly::infer`], reached through
//! [`EngineState::assembly_infer_constraints`]) — so an import becomes an
//! editable assembly without re-picking every mate by hand.
//!
//! The window is the pinned Info/Interference idiom: a movable, resizable
//! [`egui::Window`] drawn at ctx level, owning only its last scan. It decides
//! nothing about geometry:
//!
//! * the LIST of constraint types it offers is the kernel's rule table
//!   ([`EngineState::assembly_inferable_types`]) — label, icon and the
//!   "what it detects" line all come from there, so a rule added in the kernel
//!   appears here as a row with no change to this file;
//! * ticking a type re-runs the SCAN, because which types are on changes what
//!   the kernel accepts (a Concentric can make a second Touch Align redundant);
//! * Generate hands the same ticked list to the apply lane, which creates and
//!   solves the whole batch as ONE undo step.
//!
//! Nothing here writes a constraint itself, and a scan never mutates: pressing
//! the button to look is free, and pressing Generate twice is a no-op because
//! the kernel leaves an already-constrained pair alone.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::assembly_status;
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::Value;
use std::collections::{BTreeMap, HashMap};

/// One offered constraint type, as the kernel describes it.
#[derive(Clone)]
struct OfferedType {
    /// Canonical type id (`"touch_align"`) — the scan's `byType` key.
    id: String,
    /// `"⪥ Touch Align"` — icon + label, drawn as artwork by `icon_text`.
    long_name: String,
    /// What placement makes this constraint.
    detects: String,
}

/// The shell-owned Auto Constraints window.
#[derive(Default)]
pub struct AutoConstraintsWindow {
    open: bool,
    /// The kernel's rule table, read once on first open.
    types: Vec<OfferedType>,
    /// Which rules are ticked (seeded from each rule's `defaultOn`).
    enabled: BTreeMap<String, bool>,
    /// Contact tolerance in mm — how far apart two faces may sit and still
    /// read as touching. Exposed because an imported assembly's real gaps are a
    /// property of the file, not of this tool: when the scan reports a nearest
    /// miss just above this, raising it is the fix.
    tolerance: f64,
    /// The last scan report, or `None` before the first one.
    scan: Option<Value>,
    /// The last Generate reply — kept so its outcome line survives the rescan
    /// that follows it.
    generated: Option<Value>,
    /// Per-frame interactive-widget screen rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

/// An [`egui::Color32`] off the shared assembly-status palette — the same map
/// the constraints panel and the interference window read.
fn status_color(status: &str) -> egui::Color32 {
    let [r, g, b] = assembly_status::status_color_rgb(status);
    egui::Color32::from_rgb(r, g, b)
}

/// The kernel's own default contact tolerance (mm).
const DEFAULT_TOLERANCE: f64 = 0.001;

impl AutoConstraintsWindow {
    pub fn new() -> Self {
        Self {
            tolerance: DEFAULT_TOLERANCE,
            ..Self::default()
        }
    }

    /// The toolbar entry point: show the window and scan NOW, so it opens with
    /// real counts rather than an empty form waiting to be pressed.
    pub fn open_and_scan(&mut self, state: &mut EngineState) {
        self.load_types(state);
        self.generated = None;
        self.open = true;
        self.rescan(state);
    }

    /// Read the kernel's rule table (once) and seed the tick boxes from it.
    fn load_types(&mut self, state: &EngineState) {
        if !self.types.is_empty() {
            return;
        }
        let offered = state.assembly_inferable_types();
        for row in offered.as_array().into_iter().flatten() {
            let Some(id) = row.get("type").and_then(Value::as_str) else {
                continue;
            };
            self.enabled.insert(
                id.to_string(),
                row.get("defaultOn").and_then(Value::as_bool).unwrap_or(true),
            );
            self.types.push(OfferedType {
                id: id.to_string(),
                long_name: row
                    .get("longName")
                    .and_then(Value::as_str)
                    .unwrap_or(id)
                    .to_string(),
                detects: row
                    .get("detects")
                    .and_then(Value::as_str)
                    .unwrap_or_default()
                    .to_string(),
            });
        }
    }

    /// The ticked type ids, in the kernel's order.
    fn ticked(&self) -> Vec<String> {
        self.types
            .iter()
            .filter(|offered| self.enabled.get(&offered.id).copied().unwrap_or(false))
            .map(|offered| offered.id.clone())
            .collect()
    }

    /// The kernel `InferOptions` body for the current ticks and tolerance.
    fn options(&self) -> String {
        serde_json::json!({
            "types": self.ticked(),
            "tolerance": self.tolerance,
        })
        .to_string()
    }

    /// Re-run the (read-only) scan for the current ticks.
    fn rescan(&mut self, state: &mut EngineState) {
        self.scan = Some(state.assembly_infer_constraints(&self.options()));
    }

    /// Create every candidate the current ticks accept, then rescan so the
    /// counts show what is left (nothing, in a healthy pass).
    fn generate(&mut self, state: &mut EngineState) {
        let reply = state.assembly_apply_inferred_constraints(&self.options());
        self.generated = Some(reply);
        self.rescan(state);
    }

    /// How many candidates the last scan holds in total.
    fn candidate_total(&self) -> usize {
        self.scan
            .as_ref()
            .and_then(|scan| scan.get("candidates"))
            .and_then(Value::as_array)
            .map(Vec::len)
            .unwrap_or(0)
    }

    /// The per-type count from the last scan.
    fn found(&self, type_id: &str) -> usize {
        self.scan
            .as_ref()
            .and_then(|scan| scan.get("byType"))
            .and_then(|counts| counts.get(type_id))
            .and_then(Value::as_u64)
            .unwrap_or(0) as usize
    }

    /// Draw the window (if open) at ctx level, like the Info windows.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();
        if !self.open {
            return;
        }
        let mut open = true;
        egui::Window::new("Auto Constraints")
            .id(egui::Id::new("brep-auto-constraints-window"))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            .default_size([380.0, 400.0])
            .default_pos([820.0, 120.0])
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui, state));
            });
        self.open = open;
    }

    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.load_types(state);
        crate::icon_text::IconText::new(
            "Infers the mates the components' current placement already implies \
             \u{2014} the quickest way to make an imported assembly editable.",
        )
        .wrap()
        .show(ui);
        ui.add_space(4.0);

        if state.assembly_components().len() < 2 {
            ui.weak("Needs at least two components.");
            return;
        }
        if let Some(error) = self
            .scan
            .as_ref()
            .and_then(|scan| scan.get("error"))
            .and_then(Value::as_str)
        {
            ui.colored_label(status_color("error"), error);
            return;
        }

        // --- the offered rules, each with what the last scan found ----------
        let mut toggled = false;
        for offered in self.types.clone() {
            let mut on = self.enabled.get(&offered.id).copied().unwrap_or(false);
            ui.horizontal(|ui| {
                let box_ = ui.checkbox(&mut on, "");
                self.hits
                    .insert(format!("autoconstraints:type:{}", offered.id), box_.rect);
                if box_.changed() {
                    toggled = true;
                }
                crate::icon_text::IconText::new(offered.long_name.clone()).show(ui);
                let found = self.found(&offered.id);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !on {
                        ui.weak("off");
                    } else if found == 0 {
                        ui.weak("none found");
                    } else {
                        ui.colored_label(
                            status_color("satisfied"),
                            format!("{found} found"),
                        );
                    }
                });
            });
            ui.indent(&offered.id, |ui| {
                ui.weak(egui::RichText::new(offered.detects.clone()).small());
            });
            self.enabled.insert(offered.id.clone(), on);
            ui.add_space(2.0);
        }
        ui.separator();

        // --- the contact tolerance -------------------------------------------
        ui.horizontal(|ui| {
            ui.label("Contact tolerance");
            let field = ui.add(
                egui::DragValue::new(&mut self.tolerance)
                    .speed(0.001)
                    .range(1e-6..=10.0)
                    .suffix(" mm"),
            );
            self.hits
                .insert("autoconstraints:tolerance".into(), field.rect);
            if field.changed() {
                toggled = true;
            }
        });

        if toggled {
            // Which rules are on and how close counts as touching both change
            // what the kernel accepts, so the counts are only true for the
            // settings that produced them.
            self.rescan(state);
        }

        ui.separator();

        // --- what the scan looked at ----------------------------------------
        if let Some(scan) = self.scan.clone() {
            let number = |key: &str| scan.get(key).and_then(Value::as_u64).unwrap_or(0);
            ui.weak(format!(
                "{} components \u{00b7} {} pair{} examined \u{00b7} {} already constrained \u{00b7} {} not touching",
                number("componentCount"),
                number("pairsConsidered"),
                if number("pairsConsidered") == 1 { "" } else { "s" },
                number("pairsAlreadyConstrained"),
                number("pairsApart"),
            ));
            for warning in scan
                .get("warnings")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
            {
                ui.colored_label(status_color("unsupported-selection"), warning);
            }
            self.candidate_list(ui, &scan);
            self.near_miss_line(ui, &scan);
            self.suppressed_list(ui, &scan);
        }

        ui.add_space(4.0);

        // --- generate --------------------------------------------------------
        let total = self.candidate_total();
        let label = match total {
            0 => "Nothing to create".to_string(),
            1 => "Create 1 constraint".to_string(),
            many => format!("Create {many} constraints"),
        };
        let button = ui.add_enabled(total > 0, egui::Button::new(label));
        self.hits
            .insert("autoconstraints:generate".into(), button.rect);
        if button.clicked() {
            self.generate(state);
        }

        if let Some(generated) = self.generated.clone() {
            self.outcome_line(ui, &generated);
        }
    }

    /// The candidate rows — what WOULD be created, with the measurement each
    /// was read from, so the button is never a blind commitment.
    fn candidate_list(&mut self, ui: &mut egui::Ui, scan: &Value) {
        let candidates = scan
            .get("candidates")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        if candidates.is_empty() {
            return;
        }
        let icon_of = |type_id: &str| {
            self.types
                .iter()
                .find(|offered| offered.id == type_id)
                .map(|offered| offered.long_name.clone())
                .unwrap_or_else(|| type_id.to_string())
        };
        egui::CollapsingHeader::new(format!("What it found ({})", candidates.len()))
            .id_salt("brep-auto-constraints-candidates")
            .show(ui, |ui| {
                for candidate in &candidates {
                    let type_id = candidate
                        .get("type")
                        .and_then(Value::as_str)
                        .unwrap_or_default();
                    let components = candidate
                        .get("components")
                        .and_then(Value::as_array)
                        .map(|pair| {
                            pair.iter()
                                .filter_map(Value::as_str)
                                .collect::<Vec<_>>()
                                .join(" \u{2194} ")
                        })
                        .unwrap_or_default();
                    let detail = candidate
                        .get("detail")
                        .and_then(Value::as_str)
                        .unwrap_or_default();
                    crate::icon_text::IconText::new(format!(
                        "{}  {components}  \u{00b7}  {detail}",
                        icon_of(type_id)
                    ))
                    .truncate()
                    .show(ui);
                }
            });
    }

    /// The near-miss line — the single most useful thing to show a user who
    /// expected more contacts than the scan found. The kernel measures the
    /// closest pair of parallel planes that failed the coincidence test; if that
    /// distance is just above the tolerance, the geometry is fine and the
    /// tolerance is the answer, and this says so with the number in hand.
    fn near_miss_line(&mut self, ui: &mut egui::Ui, scan: &Value) {
        let gates = &scan["gates"];
        let count = |key: &str| gates[key].as_u64().unwrap_or(0);
        let mut notes: Vec<String> = Vec::new();
        // Leads, because it is the usual answer to "you missed faces that are
        // touching": they were found, they ARE in contact, and they lie in a
        // plane (or on an axis) a created constraint already holds. One mate
        // per plane is the whole point — a second would over-constrain it.
        if count("alsoOnCarrier") > 0 {
            notes.push(format!(
                "{} more face pair{} qualified on a plane or centreline a constraint above already holds \u{2014} one mate per carrier is deliberate",
                count("alsoOnCarrier"),
                if count("alsoOnCarrier") == 1 { " is" } else { "s are" }
            ));
        }
        if let Some(gap) = gates["nearestPlaneGap"].as_f64() {
            if gap > self.tolerance {
                notes.push(format!(
                    "closest untouched face pair is {} mm apart \u{2014} raise the tolerance above that to include it",
                    super::info_windows::num(gap)
                ));
            }
        }
        if count("sameFacing") > 0 {
            notes.push(format!(
                "{} coplanar face pair{} point the same way (flush, not in contact)",
                count("sameFacing"),
                if count("sameFacing") == 1 { "" } else { "s" }
            ));
        }
        if count("noOverlap") > 0 {
            notes.push(format!(
                "{} coplanar facing pair{} do not overlap",
                count("noOverlap"),
                if count("noOverlap") == 1 { "" } else { "s" }
            ));
        }
        if count("noExtent") > 0 {
            notes.push(format!("{} face(s) had no measurable extent", count("noExtent")));
        }
        if notes.is_empty() {
            return;
        }
        egui::CollapsingHeader::new("About the faces with no constraint of their own")
            .id_salt("brep-auto-constraints-gates")
            .show(ui, |ui| {
                for note in notes {
                    ui.weak(egui::RichText::new(note).small());
                }
            });
    }

    /// The candidates that WERE found and are deliberately not being created.
    /// A user who counts touching faces and finds fewer constraints is owed
    /// this list: each row says which pair it would have held and why the
    /// already-accepted set makes it unnecessary.
    fn suppressed_list(&mut self, ui: &mut egui::Ui, scan: &Value) {
        let suppressed = scan
            .get("suppressed")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        if suppressed.is_empty() {
            return;
        }
        egui::CollapsingHeader::new(format!("Found but not needed ({})", suppressed.len()))
            .id_salt("brep-auto-constraints-suppressed")
            .show(ui, |ui| {
                ui.weak(
                    egui::RichText::new(
                        "These would not remove any freedom the constraints above do not \
                         already remove. Creating them would over-constrain the solve.",
                    )
                    .small(),
                );
                for row in &suppressed {
                    let components = row
                        .get("components")
                        .and_then(Value::as_array)
                        .map(|pair| {
                            pair.iter()
                                .filter_map(Value::as_str)
                                .collect::<Vec<_>>()
                                .join(" \u{2194} ")
                        })
                        .unwrap_or_default();
                    crate::icon_text::IconText::new(format!(
                        "{}  {components}  \u{00b7}  {}",
                        row.get("type").and_then(Value::as_str).unwrap_or_default(),
                        row.get("why").and_then(Value::as_str).unwrap_or_default(),
                    ))
                    .truncate()
                    .show(ui);
                }
            });
    }

    /// The outcome of the last Generate: what was created, and what the solve
    /// made of it (an inferred batch is read off the current pose, so a healthy
    /// assembly solves without moving).
    fn outcome_line(&mut self, ui: &mut egui::Ui, generated: &Value) {
        ui.separator();
        if let Some(error) = generated.get("error").and_then(Value::as_str) {
            ui.colored_label(status_color("error"), error);
            return;
        }
        let created = generated
            .get("created")
            .and_then(Value::as_array)
            .map(Vec::len)
            .unwrap_or(0);
        if created == 0 {
            ui.weak("Nothing was created \u{2014} every pair was already constrained.");
            return;
        }
        ui.colored_label(
            status_color("satisfied"),
            format!(
                "\u{2713} Created {created} constraint{}",
                if created == 1 { "" } else { "s" }
            ),
        );
        let solve = &generated["solve"];
        if solve.get("ok").and_then(Value::as_bool) == Some(false) {
            let message = solve
                .get("error")
                .and_then(Value::as_str)
                .unwrap_or("the solve refused");
            ui.colored_label(status_color("error"), message);
            return;
        }
        if let Some(dof) = solve.get("dof").and_then(Value::as_u64) {
            ui.weak(format!(
                "solved \u{00b7} {dof} degree{} of freedom left",
                if dof == 1 { "" } else { "s" }
            ));
        }
        // An inferred set is read off ONE pose, so several constraints can hold
        // the same freedom and the solver reports the set as over-determined
        // even though it satisfied every one of them. Say so plainly — it is a
        // property of inferring everything, not a failure, and hiding it would
        // leave the constraints panel's own status word unexplained.
        if solve.get("status").and_then(Value::as_str) == Some("over") {
            let redundant = solve.get("redundant").and_then(Value::as_u64).unwrap_or(0);
            ui.colored_label(
                status_color("unsupported-selection"),
                format!(
                    "over-determined: {redundant} redundant row{} \u{2014} consistent (the parts did not \
                     move), but trimming constraints will make later edits easier",
                    if redundant == 1 { "" } else { "s" }
                ),
            );
        }
    }

    /// The window's logical state for the headed verifier
    /// (`__brepAutoConstraints`): `{open, types, enabled, scan, generated}`.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn state_json(&self) -> String {
        serde_json::json!({
            "open": self.open,
            "types": self.types.iter().map(|offered| offered.id.clone()).collect::<Vec<_>>(),
            "enabled": self.ticked(),
            "scan": self.scan.clone().unwrap_or(Value::Null),
            "generated": self.generated.clone().unwrap_or(Value::Null),
        })
        .to_string()
    }

    /// Per-frame widget rects (`autoconstraints:type:<id>`,
    /// `autoconstraints:generate`) for the headed verifier.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "autoConstraints", prefix: "autoconstraints:type:", meaning: "tick one inferable constraint type (rescans)", command: Some("assembly_infer_constraints") },
    HitKeyDoc { panel: "autoConstraints", prefix: "autoconstraints:tolerance", meaning: "how close two faces must sit to read as touching, in mm (rescans)", command: Some("assembly_infer_constraints") },
    HitKeyDoc { panel: "autoConstraints", prefix: "autoconstraints:generate", meaning: "create every inferred constraint", command: Some("assembly_apply_inferred_constraints") },
];
