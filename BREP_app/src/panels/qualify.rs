//! The **Qualify** panel — a part's declared CONNECTION POINTS, on whichever
//! surface the user is looking at.
//!
//! A part document carries `ports` beside `features`, `symbol` and `pads`: a
//! list of named GROUPS, each with a `purpose` (`pcb`, `wiring`, `piping`, …)
//! and a list of points whose NAME is their identity. The kernel resolves that
//! block at the tail of every history run (`brep_kernel`'s `ports`), binds each
//! point to the symbol pin of the same name within its unit's group
//! (`part_pins`), and reports what does not hold. Until now nothing drew any of
//! it: the block was written by the KiCad import, by editing a pin, or by hand.
//!
//! This panel is that UI. It is ONE panel on THREE surfaces — the schematic
//! symbol, the pads, and the 3D model — because a connection point is one thing
//! seen from three places, and the job it exists for is crossing between them:
//! pick a point here and jump to its pin, its pad or its geometry with the
//! point still selected on the other side.
//!
//! # What it draws
//!
//! * the port GROUPS, each with its name, its purpose and the symbol unit whose
//!   pins bind into it, and the points in it;
//! * the SELECTED point's fields, expanded in place: its placement (position
//!   and rotation, each component a number or an expression string, exactly as
//!   the block carries it), the geometry that seats it (`pointRef` /
//!   `directionRef`), its straight run, and — for an ASSEMBLY — the descendant
//!   point it maps to;
//! * the three JUMP buttons, which switch surface AND carry the selection;
//! * the two CONSISTENCY REPORTS the model already produces: `History::pin_point_report`
//!   (every pin paired with its point, and everything that does not pair) and
//!   the ports tail's own report (`EngineState::ports_report` — whether this
//!   document is an encapsulation boundary, every point it resolved, and every
//!   name it refused).
//!
//! # When it is on screen
//!
//! Exactly while the part declares a connection point. That is not a workbench
//! claim — it is a DOCUMENT condition ([`crate::workbench::PANEL_CONDITIONS`]),
//! so the pane appears on every surface at once and a part with no connection
//! points is not offered an empty panel. The first group therefore comes from
//! somewhere else: the KiCad import, the pin binding, or the Wire harness
//! workbench's **Declare connection point** button, which exists for exactly
//! this reason.
//!
//! # Edits
//!
//! Every edit is a write of the WHOLE `ports` block through
//! `History::set_ports_block`, which carries the pin/point follow inside it: a
//! renamed point renames its pin (and the pad matching that pin) in the same
//! undo step, a new point gets a pin, and a removed point loses one. The block
//! is edited as raw JSON rather than round-tripped through
//! [`brep_kernel::PortDeclaration`], so a field this panel does not know about
//! survives an edit to the field beside it.
//!
//! Each field carries its own COALESCE KEY, so typing a name is one undo entry
//! and not one per keystroke — the same key the eCAD editors hand the history.

use std::collections::HashMap;

use eframe::egui;
use serde_json::Value;

use crate::automation::hit_keys::HitKeyDoc;
use crate::workbench::ecad::{Editors, Target};
use brep_render::brep_kernel::{self, PinPointReport, PortDeclaration};
use brep_render::engine_state::{EngineState, PortRefField};

/// The row glyphs, every one catalogued artwork (`assets/glyphs`) drawn by
/// [`crate::icon_text`] — this app ships no icon font, so an uncatalogued
/// character paints as tofu.
const GLYPH_SELECT: &str = "\u{25CE}"; // ◎ select this connection point
const GLYPH_REMOVE: &str = "\u{2715}"; // ✕ remove
const GLYPH_ADD: &str = "\u{FF0B}"; // ＋ add
const GLYPH_PICK: &str = "\u{2316}"; // ⌖ pick this reference in the 3D view
const GLYPH_PRESENT: &str = "\u{25CF}"; // ● this surface carries the point
const GLYPH_ABSENT: &str = "\u{25CB}"; // ○ it does not

/// The purposes offered in the dropdown. The kernel never enumerates
/// `purpose` — it carries the string — so a value the document already holds is
/// offered beside these rather than silently rewritten.
const PURPOSES: [&str; 3] = ["pcb", "wiring", "piping"];

/// A problem's amber and a refusal's red, as the harness panel uses them.
const WARN_COLOR: egui::Color32 = egui::Color32::from_rgb(0xd2, 0x99, 0x22);
const ERROR_COLOR: egui::Color32 = egui::Color32::from_rgb(0xf8, 0x51, 0x49);
const OK_COLOR: egui::Color32 = egui::Color32::from_rgb(0x3f, 0xb9, 0x50);
/// The destructive red the other panels use for a remove affordance.
const REMOVE_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// The 3D workbench a jump from an eCAD editor lands in. A part's connection
/// points are the harness's wire ends, so this is where looking at one in 3D
/// means something; a jump from a workbench that ALREADY draws the model does
/// not switch at all (see [`QualifyPanel::jump`]).
const MODEL_WORKBENCH: &str = "wireHarness";

/// The key the 3D surface's adopted selection is remembered under, beside the
/// two eCAD editors'.
const MODEL_SURFACE: &str = "model";

/// One edit the panel's draw asked for, applied after it.
enum Edit {
    GroupName(usize, String),
    GroupPurpose(usize, String),
    GroupUnit(usize, Option<u32>),
    AddGroup,
    RemoveGroup(usize),
    PointName(usize, usize, String),
    /// One of the point's optional STRING fields, by JSON key; an empty string
    /// removes the key.
    PointText(usize, usize, &'static str, String),
    /// One component of `transform.position` / `transform.rotationEuler`: a
    /// number when it parses as one, else an expression string.
    PointTransform(usize, usize, &'static str, usize, String),
    PointReverse(usize, usize, bool),
    AddPoint(usize),
    RemovePoint(usize, usize),
}

/// Where a jump goes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Jump {
    Symbol,
    Pads,
    Model,
}

/// The panel's transient UI state. Everything it draws comes from the engine.
pub struct QualifyPanel {
    hits: HashMap<String, egui::Rect>,
    /// The selected connection point, by part-local ADDRESS (`J1.VCC`). Held
    /// by address rather than by index so an edit that reorders the block does
    /// not move the selection to another point.
    selected: Option<String>,
    /// The editor selection last adopted, so an eCAD pick is adopted ONCE: a
    /// user who picks a point here and then a different pin there gets the pin,
    /// and one who picks here and leaves the editor alone keeps their pick.
    /// The 3D surface goes through the same field under [`MODEL_SURFACE`].
    adopted: Option<(&'static str, String)>,
    /// The selection this panel last acted on for the GIZMO. The gizmo is
    /// shared — the History panel arms it on a feature — so this panel acts on
    /// a CHANGE of selection and not every frame: otherwise it would take the
    /// handles back off a feature dialog on the frame after it armed them, and
    /// it would re-ask (and re-notice) a refusal once a frame.
    gizmo_armed_for: Option<String>,
}

impl Default for QualifyPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl QualifyPanel {
    pub fn new() -> Self {
        Self { hits: HashMap::new(), selected: None, adopted: None, gizmo_armed_for: None }
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// The panel's verifier global: the selected address and the surface it is
    /// drawn on, beside the block and report the engine publishes.
    pub fn state_json(&self) -> String {
        serde_json::json!({
            "selected": self.selected,
            "gizmoArmedFor": self.gizmo_armed_for,
        })
        .to_string()
    }

    /// Draw the panel. `target` is the eCAD editor on screen, or `None` where
    /// the central tile draws the 3D model.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        engine: &mut EngineState,
        ecad: &mut Editors,
        target: Option<Target>,
    ) {
        self.hits.clear();
        self.hits.insert("qualify:panel:clip".into(), ui.clip_rect());

        let groups: Vec<PortDeclaration> = engine
            .history
            .ports_block()
            .cloned()
            .and_then(|block| serde_json::from_value(block).ok())
            .unwrap_or_default();

        // An eCAD pick is a pick of the same thing: adopt it, once.
        self.adopt_editor_selection(engine, ecad, target);
        // A selection whose point is gone (removed, or renamed out from under
        // it) is dropped rather than kept as a stale address.
        if let Some(address) = &self.selected {
            if !groups.iter().any(|group| {
                group.points.iter().any(|point| &brep_kernel::port_address(&group.name, &point.name) == address)
            }) {
                self.selected = None;
            }
        }

        let mut edit: Option<(Edit, Option<String>)> = None;
        let mut jump: Option<Jump> = None;
        let mut pick: Option<Option<String>> = None;
        // `(address, field, enter the picker)` — clearing a reference is the
        // same act said with `false`, so both go through one commit.
        let mut refs: Option<(String, PortRefField, bool)> = None;

        // --- header -----------------------------------------------------------
        let points: usize = groups.iter().map(|group| group.points.len()).sum();
        ui.horizontal(|ui| {
            let add = ui
                .add(crate::icon_text::icon_button(ui, &format!("{GLYPH_ADD} Add group")).small())
                .on_hover_text("Declare another port group on this part — a second connector, or a port with a different purpose");
            self.hits.insert("qualify:add_group".into(), add.rect);
            if add.clicked() {
                edit = Some((Edit::AddGroup, None));
            }
            ui.label(
                egui::RichText::new(format!(
                    "{} group{} | {points} point{}",
                    groups.len(),
                    if groups.len() == 1 { "" } else { "s" },
                    if points == 1 { "" } else { "s" },
                ))
                .weak(),
            );
        });
        if self.selected.is_none() {
            // A pane whose whole lower half is blank until something is
            // selected should say what to select and what it will get.
            let hint = ui.label(
                egui::RichText::new(
                    "Select a point to place it, seat it on geometry and cross to its pin and pad",
                )
                .weak()
                .small(),
            );
            self.hits.insert("qualify:hint".into(), hint.rect);
        }
        ui.add_space(2.0);

        // --- the groups -------------------------------------------------------
        for (gi, group) in groups.iter().enumerate() {
            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(4))
                .show(ui, |ui| {
                    // The frame must not lay out wider than what is on screen,
                    // or its right-aligned ✕ is published past the pane's edge
                    // and a click aims into the viewport (the same clamp the
                    // spline-anchor rows carry, for the same reason).
                    let room = (ui.clip_rect().right() - ui.max_rect().left()).max(0.0);
                    ui.set_width(ui.available_width().min(room));
                    if let Some(asked) = self.group_header(ui, gi, group) {
                        edit = Some(asked);
                    }
                    for (pi, point) in group.points.iter().enumerate() {
                        let address = brep_kernel::port_address(&group.name, &point.name);
                        let selected = self.selected.as_deref() == Some(address.as_str());
                        if let Some(asked) = self
                            .point_row(ui, engine, gi, pi, point, &address, selected, &mut pick)
                        {
                            edit = Some(asked);
                        }
                        if selected {
                            if let Some(asked) = self
                                .point_fields(ui, engine, gi, pi, point, &address, target, &mut refs)
                            {
                                edit = Some(asked);
                            }
                            if let Some(asked) = self.jump_row(ui, &address, target) {
                                jump = Some(asked);
                            }
                        }
                    }
                    let add = ui
                        .add(crate::icon_text::icon_button(ui, &format!("{GLYPH_ADD} Add point")).small())
                        .on_hover_text("Add a connection point to this group. It arrives at the part origin, facing +X; place it below.");
                    self.hits.insert(format!("qualify:group:{gi}:add_point"), add.rect);
                    if add.clicked() {
                        edit = Some((Edit::AddPoint(gi), None));
                    }
                });
        }

        // --- the reports ------------------------------------------------------
        ui.add_space(4.0);
        self.reports(ui, engine);

        // --- act on what the draw asked for (at most one of each) --------------
        if let Some(pick) = pick {
            self.pick(engine, ecad, target, pick);
        }
        if let Some((address, field, enter)) = refs {
            match enter {
                true => engine.begin_ref_select_for_port_point(&address, field),
                false => engine.clear_port_point_ref(&address, field),
            }
        }
        if let Some((edit, coalesce)) = edit {
            self.apply(engine, edit, coalesce.as_deref());
        }
        if let Some(jump) = jump {
            self.jump(engine, ecad, jump);
        }
        // Last, so it follows whatever the frame just did to the selection.
        self.follow_gizmo(engine, target);
    }

    /// Put the move/rotate handles on the selected point, or take them off.
    ///
    /// Selecting a row IS arming the gizmo, which is the convention the spline
    /// anchors panel already set: one affordance, not a select and then a
    /// separate arm. Only on the 3D surface, because an eCAD editor has no
    /// viewport to drag handles in, and never while the reference picker is
    /// up, because that picker owns the clicks.
    fn follow_gizmo(&mut self, engine: &mut EngineState, target: Option<Target>) {
        let wanted = match target.is_none() && !engine.ref_select_active() {
            true => self.selected.clone(),
            false => None,
        };
        // The gizmo is ONE shared widget, so this is deliberately not "arm what
        // is selected, every frame". It is SETTLED — and left alone — when the
        // selection has not changed AND one of three things is true:
        //
        // * nothing is wanted (and the disarm below has already run);
        // * SOMETHING is armed. Whose it is does not matter: if a feature
        //   dialog has taken the handles, re-arming would take them straight
        //   back on the next frame, and this pane is drawn beside every
        //   feature dialog in the product;
        // * the point REFUSES the handles, which would otherwise re-ask —
        //   and re-push its notice — sixty times a second.
        //
        // What that leaves is the case the plain change check got wrong: the
        // reference picker takes the handles off at its entry, and the pane is
        // not drawn while it is up, so on the frame it closes the selection has
        // NOT changed and nothing is armed. That is exactly when they go back.
        let refusal = wanted
            .as_deref()
            .and_then(|address| engine.port_point_gizmo_refusal(address));
        let settled = wanted == self.gizmo_armed_for
            && (wanted.is_none() || engine.transform_armed() || refusal.is_some());
        if settled {
            return;
        }
        self.gizmo_armed_for = wanted.clone();
        match wanted {
            // A refusal is asked ONCE — the arm pushes the notice — and the
            // reason the gizmo line shows is read from the engine, not from
            // anything kept here.
            Some(address) => {
                engine.arm_transform_for_port_point(&address);
            }
            // Take away only what THIS panel put there. A feature's gizmo,
            // armed from the History panel, is not ours — and neither is one
            // on another connection point that something else armed.
            None => {
                if engine.transform_armed_port_point().is_some() {
                    engine.disarm_transform();
                }
            }
        }
    }

    /// Take a pick made in the panel: select `address` (or nothing) and LIGHT
    /// IT UP on the surface the user is on.
    ///
    /// Lighting it up is not decoration — it is the other half of the same
    /// sentence the jump buttons speak, and without it a pick here means
    /// nothing until the user leaves. It also has to drive the editor through
    /// the SAME deferred path a jump uses: the editor's own selection is what
    /// [`Self::adopt_editor_selection`] reads back, so a panel pick that left
    /// the editor sitting on another pin would be overwritten by that pin on
    /// the very next frame.
    fn pick(
        &mut self,
        engine: &mut EngineState,
        ecad: &mut Editors,
        target: Option<Target>,
        address: Option<String>,
    ) {
        self.selected = address.clone();
        match target {
            Some(target) => {
                // Agreeing with the editor NOW as well as after its next draw:
                // `apply_focus` is what survives a pull, and this is what makes
                // the pin light up on the frame of the click.
                if let Some((_, name)) =
                    address.as_deref().and_then(brep_kernel::split_address)
                {
                    ecad.focus(target, name.to_string());
                    ecad.apply_focus(target);
                }
                // What is remembered is what the editor ACTUALLY holds now, not
                // what was asked of it. They differ for a point with no pin —
                // the focus finds nothing and the editor stays where it was —
                // and remembering the ask would make the next frame read that
                // stale pin as a NEW editor pick and overwrite the user's.
                self.adopted = editor_selection(ecad, target)
                    .map(|held| (surface_of(target), held));
            }
            // The 3D surface: the point's own scene entities are published
            // under its address, so this is the selection a click in the
            // viewport makes — the Scene tree's row click, by name.
            None => {
                if let Some(address) = address.as_deref() {
                    if resolved(engine, address) {
                        engine.select_by_name("solid", address);
                    }
                }
                // The same rule, on this surface: what is remembered is what
                // the VIEWPORT holds afterwards. Deselecting here leaves the
                // geometry lit, and remembering the ask instead would make the
                // next frame read that as a fresh viewport pick and undo the
                // deselection.
                self.adopted = engine
                    .selected_port_point()
                    .map(|held| (MODEL_SURFACE, held));
            }
        }
    }

    /// One group's header row: name, purpose, symbol unit, remove.
    fn group_header(
        &mut self,
        ui: &mut egui::Ui,
        gi: usize,
        group: &PortDeclaration,
    ) -> Option<(Edit, Option<String>)> {
        let mut asked = None;
        ui.horizontal(|ui| {
            let mut name = group.name.clone();
            let field = ui
                .add(egui::TextEdit::singleline(&mut name).desired_width(70.0))
                .on_hover_text("The group's name — the first half of every address in it (`J1` in `J1.VCC`). `:`, `.` and `/` are refused.");
            self.hits.insert(format!("qualify:group:{gi}:name"), field.rect);
            if field.changed() {
                asked = Some((Edit::GroupName(gi, name), Some(format!("group:{gi}:name"))));
            }

            let mut purpose = group.purpose.clone();
            let combo = egui::ComboBox::from_id_salt(("qualify-purpose", gi))
                .selected_text(if purpose.is_empty() { "purpose" } else { purpose.as_str() })
                .width(76.0)
                .show_ui(ui, |ui| {
                    // The document's own value is offered beside the three when
                    // it is none of them: the kernel carries `purpose` as a
                    // string and never enumerates it, so a purpose a later
                    // slice invents must not vanish because this list has not
                    // heard of it.
                    let mut offered: Vec<String> =
                        PURPOSES.iter().map(|p| (*p).to_string()).collect();
                    if !group.purpose.is_empty() && !PURPOSES.contains(&group.purpose.as_str()) {
                        offered.push(group.purpose.clone());
                    }
                    for candidate in offered {
                        let item = ui.selectable_label(purpose == candidate, &candidate);
                        self.hits
                            .insert(format!("qualify:group:{gi}:purpose:{candidate}"), item.rect);
                        if item.clicked() {
                            purpose = candidate;
                        }
                    }
                });
            self.hits.insert(format!("qualify:group:{gi}:purpose"), combo.response.rect);
            if purpose != group.purpose {
                asked = Some((Edit::GroupPurpose(gi, purpose), None));
            }

            ui.label(egui::RichText::new("unit").weak());
            let mut unit = group.symbol_unit.map(|unit| unit.to_string()).unwrap_or_default();
            let field = ui
                .add(egui::TextEdit::singleline(&mut unit).desired_width(24.0))
                .on_hover_text("The SYMBOL UNIT whose pins bind into this group, 1-based. Empty binds no pins — which is right for a piping port — unless this is the part's only group, when it takes unit 1.");
            self.hits.insert(format!("qualify:group:{gi}:unit"), field.rect);
            if field.changed() {
                let parsed = unit.trim().parse::<u32>().ok();
                // A field being cleared reads as "no unit"; a field holding
                // something that is not a number keeps what was there, so a
                // half-typed value never silently unbinds a group.
                if parsed.is_some() || unit.trim().is_empty() {
                    asked = Some((Edit::GroupUnit(gi, parsed), Some(format!("group:{gi}:unit"))));
                }
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let remove = ui
                    .add(crate::icon_text::icon_button_colored(ui, GLYPH_REMOVE, Some(REMOVE_RED)).small())
                    .on_hover_text("Remove this group and every point in it — and the pins bound to them");
                self.hits.insert(format!("qualify:group:{gi}:remove"), remove.rect);
                if remove.clicked() {
                    asked = Some((Edit::RemoveGroup(gi), None));
                }
            });
        });
        asked
    }

    /// One point's row: select, name, remove. A click on ◎ is reported as a
    /// PICK rather than applied here, because a pick is not only a selection:
    /// it lights the point on whatever surface is on screen, which needs the
    /// editors and the engine.
    #[allow(clippy::too_many_arguments)]
    fn point_row(
        &mut self,
        ui: &mut egui::Ui,
        engine: &EngineState,
        gi: usize,
        pi: usize,
        point: &brep_kernel::PortPoint,
        address: &str,
        selected: bool,
        pick: &mut Option<Option<String>>,
    ) -> Option<(Edit, Option<String>)> {
        let mut asked = None;
        ui.horizontal(|ui| {
            let label = crate::icon_text::selectable_icon_label(ui, selected, GLYPH_SELECT)
                .on_hover_text(format!("Select {address} — it lights up on the surface you are on, its fields open below, and the jump buttons carry it to the others"));
            self.hits.insert(format!("qualify:point:{gi}:{pi}:select"), label.rect);
            if label.clicked() {
                *pick = Some((!selected).then(|| address.to_string()));
            }
            let mut name = point.name.clone();
            let field = ui
                .add(egui::TextEdit::singleline(&mut name).desired_width(70.0))
                .on_hover_text("The point's name, which is its identity: the symbol pin of this name in this group's unit binds to it, and the pad of that number follows.");
            self.hits.insert(format!("qualify:point:{gi}:{pi}:name"), field.rect);
            if field.changed() {
                asked = Some((Edit::PointName(gi, pi, name), Some(format!("point:{gi}:{pi}:name"))));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let remove = ui
                    .add(crate::icon_text::icon_button_colored(ui, GLYPH_REMOVE, Some(REMOVE_RED)).small())
                    .on_hover_text("Remove this connection point — and the pin bound to it");
                self.hits.insert(format!("qualify:point:{gi}:{pi}:remove"), remove.rect);
                if remove.clicked() {
                    asked = Some((Edit::RemovePoint(gi, pi), None));
                }
                ui.add_space(4.0);
                self.presence(ui, engine, gi, pi, address, &point.name);
            });
        });
        asked
    }

    /// The selected point's fields, indented under its row, in four sections:
    /// where it is SEATED, the PLACEMENT read in that seat, what a WIRE leaving
    /// it does, and — for an assembly — the descendant it maps to.
    ///
    /// The sections are the point's own sentence in order: seat it, nudge it,
    /// say how the wire leaves, and (an assembly only) say whose point it
    /// really is. Before this they were one flat list of eight fields in which
    /// `pointRef` and `extension` looked like the same kind of thing.
    fn point_fields(
        &mut self,
        ui: &mut egui::Ui,
        engine: &EngineState,
        gi: usize,
        pi: usize,
        point: &brep_kernel::PortPoint,
        address: &str,
        target: Option<Target>,
        refs: &mut Option<(String, PortRefField, bool)>,
    ) -> Option<(Edit, Option<String>)> {
        let mut asked = None;
        // A MAPPED point has no placement of its own: the kernel's
        // `resolve_point` returns on `mapsTo` before it reads the transform or
        // either reference. Offering a seat and an offset it will ignore is
        // three sections of lie, so a mapped point shows only the one field
        // that decides where it is.
        let mapped = point
            .maps_to
            .as_deref()
            .map(str::trim)
            .is_some_and(|target| !target.is_empty());
        ui.indent(("qualify-point", gi, pi), |ui| {
            if mapped {
                if let Some(mapped_asked) = self.maps_to_field(ui, gi, pi, point) {
                    asked = Some(mapped_asked);
                }
                self.seat_line(ui, engine, address, point);
                return;
            }
            // --- seat ---------------------------------------------------------
            self.section(ui, "Seated on");
            for (field, caption, hover) in [
                (PortRefField::Point, "Point", "The geometry this point SITS on: a face, an edge, a circle's centre, a vertex, a datum frame. Pick it in the 3D view with \u{2316}. Empty places the point by the offset below alone."),
                (PortRefField::Direction, "Direction", "The geometry it FACES along: a face's normal, a bore's axis, an edge, a datum frame. Empty takes the direction the point reference implies, or +X. Seat the POINT first where you can: a direction on its own still re-reads the offset in its own axes, so a placement you typed is zeroed rather than turned."),
            ] {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(caption).weak());
                    let key = field.key();
                    let mut text = optional_text(point, key);
                    let field_ui = ui
                        .add(egui::TextEdit::singleline(&mut text).desired_width(120.0))
                        .on_hover_text(hover);
                    self.hits.insert(format!("qualify:point:{gi}:{pi}:{key}"), field_ui.rect);
                    if field_ui.changed() {
                        asked = Some((
                            Edit::PointText(gi, pi, key, text),
                            Some(format!("point:{gi}:{pi}:{key}")),
                        ));
                    }
                    // Picking is offered only where there is a model to pick
                    // IN. From an eCAD editor the button would open a picker
                    // over a schematic, which has no faces.
                    let pickable = target.is_none();
                    let pick = ui
                        .add_enabled(
                            pickable,
                            crate::icon_text::icon_button(ui, GLYPH_PICK).small(),
                        )
                        .on_hover_text("Pick it in the 3D view")
                        .on_disabled_hover_text("Go to the 3D model to pick geometry");
                    self.hits.insert(format!("qualify:point:{gi}:{pi}:{key}:pick"), pick.rect);
                    if pick.clicked() {
                        *refs = Some((address.to_string(), field, true));
                    }
                    let has = !optional_text(point, key).trim().is_empty();
                    let clear = ui
                        .add_enabled(
                            has,
                            crate::icon_text::icon_button_colored(ui, GLYPH_REMOVE, Some(REMOVE_RED)).small(),
                        )
                        .on_hover_text("Unseat it — the offset below stays as it is, read from the part origin");
                    self.hits.insert(format!("qualify:point:{gi}:{pi}:{key}:clear"), clear.rect);
                    if clear.clicked() {
                        *refs = Some((address.to_string(), field, false));
                    }
                });
            }
            self.seat_line(ui, engine, address, point);

            // --- placement ----------------------------------------------------
            self.section(ui, "Placement");
            let seated = point.point_ref.is_some() || point.direction_ref.is_some();
            for (key, caption, hover) in [
                ("position", "Offset", if seated {
                    "Where the point sits RELATIVE TO ITS SEAT, in millimetres: x runs along the direction the reference gives, y and z across it. A component that is not a number is kept as an EXPRESSION."
                } else {
                    "Where the point sits, in millimetres from the part origin. Seat it on geometry above and these become an offset from that. A component that is not a number is kept as an EXPRESSION."
                }),
                ("rotationEuler", "Angle", if seated {
                    "The rotation applied to the seat's own axes to get the direction a wire leaves along, in degrees. All zero means \"straight out along the reference\"."
                } else {
                    "The rotation applied to +X to get the direction a wire leaves along, in degrees."
                }),
            ] {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(caption).weak());
                    for axis in 0..3 {
                        let mut text = transform_component(point, key, axis);
                        let field = ui
                            .add(egui::TextEdit::singleline(&mut text).desired_width(46.0))
                            .on_hover_text(hover);
                        self.hits
                            .insert(format!("qualify:point:{gi}:{pi}:{key}{axis}"), field.rect);
                        if field.changed() {
                            asked = Some((
                                Edit::PointTransform(gi, pi, key, axis, text),
                                Some(format!("point:{gi}:{pi}:{key}{axis}")),
                            ));
                        }
                    }
                });
            }
            self.gizmo_line(ui, engine, address, target);

            // --- the wire -----------------------------------------------------
            self.section(ui, "Wire");
            ui.horizontal(|ui| {
                for (key, caption, hover) in [
                    ("extension", "Extension", "The straight run a wire keeps before it may bend. Empty takes the default."),
                    ("displayLength", "Drawn", "How long the point's line is drawn. Empty takes the default."),
                ] {
                    ui.label(egui::RichText::new(caption).weak());
                    let mut text = optional_text(point, key);
                    let field = ui
                        .add(egui::TextEdit::singleline(&mut text).desired_width(46.0))
                        .on_hover_text(hover);
                    self.hits.insert(format!("qualify:point:{gi}:{pi}:{key}"), field.rect);
                    if field.changed() {
                        asked = Some((
                            Edit::PointText(gi, pi, key, text),
                            Some(format!("point:{gi}:{pi}:{key}")),
                        ));
                    }
                }
            });
            let mut reverse = point.reverse_direction;
            let toggle = ui
                .checkbox(&mut reverse, "Leaves the other way")
                .on_hover_text("Flip the direction a wire leaves along, without disturbing the seat");
            self.hits.insert(format!("qualify:point:{gi}:{pi}:reverse"), toggle.rect);
            if toggle.changed() {
                asked = Some((Edit::PointReverse(gi, pi, reverse), None));
            }

            // --- the assembly declaration -------------------------------------
            // Shown only where it can mean something: `mapsTo` says "this point
            // IS a descendant's point", so on a part with no components it is a
            // field that can only ever be wrong.
            if !engine.assembly_components().is_empty() {
                if let Some(mapped_asked) = self.maps_to_field(ui, gi, pi, point) {
                    asked = Some(mapped_asked);
                }
            }
        });
        asked
    }

    /// The `mapsTo` field and its heading — the only field a MAPPED point has,
    /// and the last section of an unmapped one in a document with components.
    fn maps_to_field(
        &mut self,
        ui: &mut egui::Ui,
        gi: usize,
        pi: usize,
        point: &brep_kernel::PortPoint,
    ) -> Option<(Edit, Option<String>)> {
        let mut asked = None;
        self.section(ui, "Assembly");
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Maps to").weak());
            let mut text = optional_text(point, "mapsTo");
            let field = ui
                .add(egui::TextEdit::singleline(&mut text).desired_width(130.0))
                .on_hover_text("The descendant point this one IS, by address (`ACOMP2:J5.VCC`). It takes the target's resolved placement, so the interface follows the child — and this point has no seat, no offset and no handles of its own. Clear it to place this point here instead.");
            self.hits.insert(format!("qualify:point:{gi}:{pi}:mapsTo"), field.rect);
            if field.changed() {
                asked = Some((
                    Edit::PointText(gi, pi, "mapsTo", text),
                    Some(format!("point:{gi}:{pi}:mapsTo")),
                ));
            }
        });
        asked
    }

    /// A section heading inside the selected point's fields.
    fn section(&mut self, ui: &mut egui::Ui, caption: &str) {
        ui.add_space(3.0);
        ui.label(egui::RichText::new(caption).small().strong());
    }

    /// What the last run made of the point's seat, in words: where it sits and
    /// which way it faces. This is the answer to "did my pick take?", and
    /// without it a reference field is a string the user must trust.
    fn seat_line(
        &mut self,
        ui: &mut egui::Ui,
        engine: &EngineState,
        address: &str,
        point: &brep_kernel::PortPoint,
    ) {
        let seated = !optional_text(point, "pointRef").trim().is_empty()
            || !optional_text(point, "directionRef").trim().is_empty();
        let text = match engine.port_point_row(address) {
            Some(row) if seated => format!(
                "seated at ({}), facing ({})",
                triple(row.seat.origin),
                triple(row.seat.x),
            ),
            Some(row) => format!("at ({}), facing ({})", triple(row.position), triple(row.direction)),
            // The tail resolves at the END of the run, so a point just added
            // has nothing yet — which is a state, not a fault.
            None => "not resolved by the last run".to_string(),
        };
        let label = ui.label(egui::RichText::new(text).weak().small());
        self.hits.insert("qualify:seat".into(), label.rect);
    }

    /// Whether the handles are on this point, and why not when they are not.
    fn gizmo_line(
        &mut self,
        ui: &mut egui::Ui,
        engine: &EngineState,
        address: &str,
        target: Option<Target>,
    ) {
        // Asked of the ENGINE rather than inferred from the gizmo not being
        // armed: this line is drawn a moment before `follow_gizmo` runs, so on
        // the frame a point is selected an inference would show the refusal
        // reason for a point that is about to arm perfectly well.
        let text = match (target, engine.port_point_gizmo_refusal(address)) {
            (Some(_), _) => "the move/rotate handles arm on the 3D surface".to_string(),
            (None, Some(expression)) => format!(
                "no handles: '{expression}' places this point, and a drag writes numbers"
            ),
            (None, None) => {
                "the move/rotate handles are on it — a drag writes the offset above".to_string()
            }
        };
        let label = ui.label(egui::RichText::new(text).weak().small());
        self.hits.insert("qualify:gizmo".into(), label.rect);
    }

    /// Three lamps per point: does a symbol PIN carry this name, does a PAD,
    /// and did the last run resolve it in 3D. A connection point is ONE thing
    /// on three surfaces, and this row is where that claim is either true in
    /// front of the user or visibly not.
    fn presence(
        &mut self,
        ui: &mut egui::Ui,
        engine: &EngineState,
        gi: usize,
        pi: usize,
        address: &str,
        name: &str,
    ) {
        let pinned = engine
            .history
            .pin_point_report()
            .map(|report| report.pairs.iter().any(|pair| pair.point == address));
        let padded = pad_numbers(engine).iter().any(|number| number == name);
        for (key, state, present, absent) in [
            (
                "symbol",
                pinned,
                "a symbol pin carries this name",
                "no symbol pin carries this name, so nothing on the schematic is this point",
            ),
            (
                "pads",
                Some(padded),
                "a pad carries this number",
                "no pad carries this number",
            ),
            (
                "model",
                Some(resolved(engine, address)),
                "the last run resolved it in 3D",
                "the last run did not resolve it, so there is no geometry to show",
            ),
        ] {
            let (glyph, color, hover) = match state {
                Some(true) => (GLYPH_PRESENT, OK_COLOR, present),
                Some(false) => (GLYPH_ABSENT, WARN_COLOR, absent),
                // No symbol at all: the question does not arise, and an amber
                // lamp on a 3D-only part would be a fault where there is none.
                None => (
                    GLYPH_ABSENT,
                    ui.visuals().weak_text_color(),
                    "this part has no symbol",
                ),
            };
            let lamp = lamp(ui, glyph, color).on_hover_text(format!("{key}: {hover}"));
            self.hits
                .insert(format!("qualify:point:{gi}:{pi}:lamp:{key}"), lamp.rect);
        }
    }

    /// The three JUMP buttons for the selected point. The one for the surface
    /// already on screen is drawn pressed and does nothing.
    fn jump_row(&mut self, ui: &mut egui::Ui, address: &str, target: Option<Target>) -> Option<Jump> {
        let mut asked = None;
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Look at it in").weak());
            for (jump, key, label, hover) in [
                (Jump::Symbol, "symbol", "Symbol", "Open the schematic symbol with this point's PIN selected"),
                (Jump::Pads, "pads", "Pads", "Open the footprint with this point's PAD selected"),
                (Jump::Model, "model", "3D", "Show the model with this point's geometry selected"),
            ] {
                let here = match jump {
                    Jump::Symbol => target == Some(Target::Symbol),
                    Jump::Pads => target == Some(Target::Pads),
                    Jump::Model => target.is_none(),
                };
                let button = ui
                    .add_enabled(!here, egui::Button::new(label).small())
                    .on_hover_text(format!("{hover} ({address})"))
                    .on_disabled_hover_text("You are looking at it");
                self.hits.insert(format!("qualify:jump:{key}"), button.rect);
                if button.clicked() {
                    asked = Some(jump);
                }
            }
        });
        asked
    }

    /// The two consistency reports, which exist to be shown and were shown
    /// nowhere: the pin/point pairing and the ports tail's own resolution.
    fn reports(&mut self, ui: &mut egui::Ui, engine: &EngineState) {
        let mut problem = 0usize;
        let mut line = |ui: &mut egui::Ui,
                        hits: &mut HashMap<String, egui::Rect>,
                        text: String,
                        color: egui::Color32| {
            let label = ui.label(egui::RichText::new(text).weak().color(color));
            hits.insert(format!("qualify:problem:{problem}"), label.rect);
            problem += 1;
        };

        ui.label(egui::RichText::new("Pins and points").strong());
        match engine.history.pin_point_report() {
            None => {
                ui.label(
                    egui::RichText::new("this part has no symbol, so its points bind to no pins")
                        .weak(),
                );
            }
            Some(PinPointReport { pairs, problems }) => {
                ui.label(
                    egui::RichText::new(format!(
                        "{} pin{} paired",
                        pairs.len(),
                        if pairs.len() == 1 { "" } else { "s" }
                    ))
                    .weak()
                    .color(if problems.is_empty() { OK_COLOR } else { WARN_COLOR }),
                );
                for entry in &problems {
                    line(ui, &mut self.hits, entry.message(), WARN_COLOR);
                }
            }
        }
        // A HOLD is the live state of the edit in flight, not a property of the
        // document: the last keystroke could not be carried to the other side,
        // and the next one will be diffed from the same base again.
        if let Some(hold) = engine.history.pin_port_hold() {
            line(ui, &mut self.hits, format!("not carried to the pins: {hold}"), WARN_COLOR);
        }

        ui.add_space(2.0);
        ui.label(egui::RichText::new("Ports").strong());
        match engine.ports_report() {
            None => {
                ui.label(egui::RichText::new("the model has not run yet").weak());
            }
            Some(report) => {
                ui.label(
                    egui::RichText::new(format!(
                        "{} point{} resolved | {}",
                        report.points.len(),
                        if report.points.len() == 1 { "" } else { "s" },
                        if report.boundary {
                            "a boundary: a parent sees only what this document declares"
                        } else {
                            "transparent: a parent sees this document's children too"
                        },
                    ))
                    .weak()
                    .color(if report.problems.is_empty() { OK_COLOR } else { ERROR_COLOR }),
                )
                .on_hover_text(if report.boundary {
                    "A board hides its components behind the ports it declares itself (`portBoundary`, or a `pcb` block)."
                } else {
                    "An enclosure does not hide the parts in it: their points are exported namespaced."
                });
                for entry in &report.problems {
                    line(ui, &mut self.hits, entry.clone(), ERROR_COLOR);
                }
            }
        }
        // A point that maps DOWN says where to, since `mapsTo` is the one field
        // whose meaning is another document's.
        if let Some(report) = engine.ports_report() {
            for row in report.points.iter().filter(|row| row.maps_to.is_some()) {
                let target = row.maps_to.clone().unwrap_or_default();
                line(
                    ui,
                    &mut self.hits,
                    format!("{} \u{2192} {target}", row.address),
                    ui.visuals().weak_text_color(),
                );
            }
        }
    }

    // --- acting -------------------------------------------------------------

    /// Apply one edit to the raw block and write it back. The write carries the
    /// pin/point follow, and sets the flag that re-runs the history next frame.
    fn apply(&mut self, engine: &mut EngineState, edit: Edit, coalesce: Option<&str>) {
        let mut block = engine
            .history
            .ports_block()
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        match edit {
            Edit::AddGroup => {
                block.push(serde_json::json!({
                    "name": unique_group_name(&block),
                    "purpose": PURPOSES[0],
                    "points": [],
                }));
            }
            Edit::RemoveGroup(gi) => {
                if gi < block.len() {
                    block.remove(gi);
                }
            }
            Edit::GroupName(gi, name) => {
                // Renaming a group moves EVERY address in it, and the selection
                // is held by address — so carry it, exactly as a point rename
                // does, or the point being edited deselects itself the moment
                // its group's name is touched.
                let was = block.get(gi).and_then(|group| group.get("name")).and_then(Value::as_str);
                if let (Some(was), Some(address)) = (was, self.selected.clone()) {
                    if let Some((port, point)) = brep_kernel::split_address(&address) {
                        if port == was {
                            self.selected = Some(brep_kernel::port_address(&name, point));
                        }
                    }
                }
                set_group(&mut block, gi, "name", Value::String(name));
            }
            Edit::GroupPurpose(gi, purpose) => {
                set_group(&mut block, gi, "purpose", Value::String(purpose));
            }
            Edit::GroupUnit(gi, unit) => match unit {
                Some(unit) => set_group(&mut block, gi, "symbolUnit", Value::from(unit)),
                None => remove_group_key(&mut block, gi, "symbolUnit"),
            },
            Edit::AddPoint(gi) => {
                if let Some(points) = points_mut(&mut block, gi) {
                    let name = unique_point_name(points);
                    points.push(serde_json::json!({
                        "name": name,
                        "transform": { "position": [0.0, 0.0, 0.0], "rotationEuler": [0.0, 0.0, 0.0] },
                    }));
                }
            }
            Edit::RemovePoint(gi, pi) => {
                if let Some(points) = points_mut(&mut block, gi) {
                    if pi < points.len() {
                        points.remove(pi);
                    }
                }
            }
            Edit::PointName(gi, pi, name) => {
                // The selection is held by ADDRESS, so a rename has to move it
                // or the point the user is editing deselects itself mid-word.
                if let Some(group) = block.get(gi).and_then(|group| group.get("name")).and_then(Value::as_str) {
                    self.selected = Some(brep_kernel::port_address(group, &name));
                }
                set_point(&mut block, gi, pi, "name", Value::String(name));
            }
            Edit::PointText(gi, pi, key, text) => match text.trim().is_empty() {
                true => remove_point_key(&mut block, gi, pi, key),
                false => set_point(&mut block, gi, pi, key, scalar(&text)),
            },
            Edit::PointTransform(gi, pi, key, axis, text) => {
                set_transform_component(&mut block, gi, pi, key, axis, scalar(&text));
            }
            Edit::PointReverse(gi, pi, reverse) => match reverse {
                true => set_point(&mut block, gi, pi, "reverseDirection", Value::Bool(true)),
                false => remove_point_key(&mut block, gi, pi, "reverseDirection"),
            },
        }
        // An EMPTY block is not a block: the kernel omits it on re-serialize,
        // and leaving `[]` behind would keep this pane up over a part with
        // nothing to qualify. Removing the last group therefore closes the pane,
        // which is worth saying out loud.
        if block.is_empty() {
            engine.history.set_ports_block(None, None);
            engine.push_notice("the last connection-point group is gone; the Qualify pane closes with it");
            return;
        }
        engine.history.set_ports_block(Some(Value::Array(block)), coalesce);
    }

    /// Carry the selected point to another surface: switch the workbench and
    /// hand the editor the selection to make when it next draws.
    ///
    /// The editor selection is DEFERRED rather than set here: the editor is
    /// re-handed its block on the frame it is drawn (`viewport::ecad`'s pull),
    /// which clears whatever it had selected, so a selection set now would be
    /// wiped by the switch itself.
    fn jump(&mut self, engine: &mut EngineState, ecad: &mut Editors, jump: Jump) {
        let Some(address) = self.selected.clone() else {
            return;
        };
        let Some((_, name)) = brep_kernel::split_address(&address) else {
            return;
        };
        let (target, workbench) = match jump {
            Jump::Symbol => (Some(Target::Symbol), "symbol"),
            Jump::Pads => (Some(Target::Pads), "pads"),
            Jump::Model => (None, MODEL_WORKBENCH),
        };
        match target {
            Some(target) => ecad.focus(target, name.to_string()),
            None => {
                // The part's OWN points resolve at the TAIL of the run, so one
                // the last run did not resolve has nothing in the scene to
                // select. Selecting its name anyway would light nothing and
                // leave a phantom selection behind, so say so instead.
                if resolved(engine, &address) {
                    engine.select_by_name("solid", &address);
                } else {
                    engine.push_notice(format!(
                        "{address} has no geometry yet \u{2014} the last run did not resolve it"
                    ));
                }
            }
        }
        // A workbench that ALREADY draws what was asked for is left alone: a
        // user in Modeling who asks to see the point in 3D wants the selection,
        // not to be moved to another workbench.
        let already = match jump {
            Jump::Model => Target::of_workbench(&engine.settings.workbench).is_none(),
            _ => false,
        };
        if !already && engine.settings.workbench != workbench {
            // The SAME seam the workbench dropdown writes through, but NOT
            // persisted to the settings blob: a jump is navigation within a
            // document, like the assembly auto-switch, and the blob stays the
            // user's boot preference.
            let _ = engine
                .apply_settings_json(&serde_json::json!({ "workbench": workbench }).to_string());
        }
    }

    /// Adopt the eCAD editor's own selection when it names a connection point:
    /// picking pin `VCC` in the symbol selects `J1.VCC` here, which is the half
    /// of "select it there, look at it here" the jump buttons do not cover.
    fn adopt_editor_selection(&mut self, engine: &EngineState, ecad: &Editors, target: Option<Target>) {
        // The 3D surface has a selection of its own, and it already names the
        // point: a declared point draws as a sheet keyed by its address. This
        // is the direction the panel could not reach before — clicking a point
        // in the viewport now opens it here.
        let (surface, label) = match target {
            None => (MODEL_SURFACE, engine.selected_port_point()),
            Some(target @ (Target::Symbol | Target::Pads)) => {
                (surface_of(target), editor_selection(ecad, target))
            }
            // A schematic sheet or a board: neither selects a pin or a pad, so
            // there is nothing of this part's to adopt.
            Some(_) => return,
        };
        let Some(label) = label else {
            // Nothing selected over there clears what was adopted FROM there,
            // so re-picking the same pin is a new pick.
            if self.adopted.as_ref().is_some_and(|(from, _)| *from == surface) {
                self.adopted = None;
            }
            return;
        };
        if self.adopted.as_ref().is_some_and(|(from, what)| *from == surface && *what == label) {
            return;
        }
        self.adopted = Some((surface, label.clone()));
        // The 3D surface's selection IS the address — the join the two eCAD
        // surfaces need is already made there.
        if surface == MODEL_SURFACE {
            self.selected = Some(label);
            return;
        }
        // A pin's group is its UNIT's group: a pad matches its pin by number
        // symbol-wide, so both surfaces route through the same join.
        let unit = engine
            .history
            .pins()
            .unwrap_or_default()
            .into_iter()
            .find(|pin| pin.label == label)
            .map(|pin| pin.unit)
            .unwrap_or(1);
        if let Some(group) = engine.history.port_group_for_unit(unit) {
            self.selected = Some(brep_kernel::port_address(&group, &label));
        }
    }
}

/// Whether the LAST RUN resolved `address` — whether, in other words, there is
/// anything in the scene under that name to select. A part's own points are
/// resolved at the tail of the run, so a point just typed in has none until the
/// next one finishes.
fn resolved(engine: &EngineState, address: &str) -> bool {
    engine
        .ports_report()
        .is_some_and(|report| report.points.iter().any(|row| row.address == address))
}

/// What `target`'s editor currently has selected, by NUMBER — the one reading
/// of it, so a pick and the adopt that follows it cannot disagree about what
/// the editor holds. `None` for a surface that selects no pin or pad.
fn editor_selection(ecad: &Editors, target: Target) -> Option<String> {
    match target {
        Target::Symbol => ecad.symbol.selected_pin().map(str::to_string),
        Target::Pads => ecad.pads.selected_pad().map(str::to_string),
        Target::Diagram | Target::Pcb => None,
    }
}

/// The key an adopted editor selection is remembered under. A `&'static str`
/// rather than the `Target` itself, because what is remembered is "the last
/// thing I saw selected over there" and the two eCAD editors are the only
/// surfaces that have one.
fn surface_of(target: Target) -> &'static str {
    match target {
        Target::Symbol => "symbol",
        Target::Pads => "pads",
        Target::Diagram => "diagram",
        Target::Pcb => "pcb",
    }
}

// ---------------------------------------------------------------------------
// Raw-block helpers: every edit touches ONE key, so a field this panel does not
// know about survives an edit to the field beside it.
// ---------------------------------------------------------------------------

fn set_group(block: &mut [Value], gi: usize, key: &str, value: Value) {
    if let Some(object) = block.get_mut(gi).and_then(Value::as_object_mut) {
        object.insert(key.into(), value);
    }
}

fn remove_group_key(block: &mut [Value], gi: usize, key: &str) {
    if let Some(object) = block.get_mut(gi).and_then(Value::as_object_mut) {
        object.remove(key);
    }
}

fn points_mut(block: &mut [Value], gi: usize) -> Option<&mut Vec<Value>> {
    let group = block.get_mut(gi)?.as_object_mut()?;
    group.entry("points").or_insert_with(|| Value::Array(Vec::new())).as_array_mut()
}

fn point_mut<'a>(block: &'a mut [Value], gi: usize, pi: usize) -> Option<&'a mut serde_json::Map<String, Value>> {
    points_mut(block, gi)?.get_mut(pi)?.as_object_mut()
}

fn set_point(block: &mut [Value], gi: usize, pi: usize, key: &str, value: Value) {
    if let Some(point) = point_mut(block, gi, pi) {
        point.insert(key.into(), value);
    }
}

fn remove_point_key(block: &mut [Value], gi: usize, pi: usize, key: &str) {
    if let Some(point) = point_mut(block, gi, pi) {
        point.remove(key);
    }
}

/// Write one component of `transform.position` / `transform.rotationEuler`,
/// filling in the array (and the transform) when the point has none.
fn set_transform_component(
    block: &mut [Value],
    gi: usize,
    pi: usize,
    key: &str,
    axis: usize,
    value: Value,
) {
    let Some(point) = point_mut(block, gi, pi) else {
        return;
    };
    let transform = point
        .entry("transform")
        .or_insert_with(|| Value::Object(Default::default()));
    if !transform.is_object() {
        *transform = Value::Object(Default::default());
    }
    let Some(transform) = transform.as_object_mut() else {
        return;
    };
    let axes = transform
        .entry(key.to_string())
        .or_insert_with(|| Value::Array(vec![Value::from(0.0), Value::from(0.0), Value::from(0.0)]));
    if !axes.is_array() {
        *axes = Value::Array(vec![Value::from(0.0), Value::from(0.0), Value::from(0.0)]);
    }
    if let Some(axes) = axes.as_array_mut() {
        while axes.len() < 3 {
            axes.push(Value::from(0.0));
        }
        axes[axis] = value;
    }
}

/// A typed field's value: a NUMBER when it parses as one, else the string,
/// which the transform lane reads as an expression. An empty field is `0`
/// rather than an empty expression, because a blank component is not a
/// placement.
fn scalar(text: &str) -> Value {
    let text = text.trim();
    if text.is_empty() {
        return Value::from(0.0);
    }
    match text.parse::<f64>() {
        Ok(number) => Value::from(number),
        Err(_) => Value::String(text.to_string()),
    }
}

/// One component of a point's transform as the field shows it: a number
/// formatted plainly, an expression verbatim, and `0` for a point with no
/// transform at all.
fn transform_component(point: &brep_kernel::PortPoint, key: &str, axis: usize) -> String {
    match point.transform.get(key).and_then(Value::as_array).and_then(|axes| axes.get(axis)) {
        // COMPACT, not `to_string`: the block carries a component the panel
        // wrote as a JSON float, so an untouched zero read back as `0.0` and a
        // whole millimetre as `2.0`. Six places is past anything a placement
        // field is typed to and short of f64 noise.
        Some(Value::Number(number)) => match number.as_f64() {
            Some(value) => brep_render::formatting::compact_decimal(value, 6),
            None => number.to_string(),
        },
        Some(Value::String(expression)) => expression.clone(),
        _ => "0".to_string(),
    }
}

/// One of the point's optional scalar fields as its field shows it — empty
/// where the point does not carry it.
fn optional_text(point: &brep_kernel::PortPoint, key: &str) -> String {
    match key {
        "pointRef" => point.point_ref.clone().unwrap_or_default(),
        "directionRef" => point.direction_ref.clone().unwrap_or_default(),
        "mapsTo" => point.maps_to.clone().unwrap_or_default(),
        "extension" => point.extension.map(|v| v.to_string()).unwrap_or_default(),
        "displayLength" => point.display_length.map(|v| v.to_string()).unwrap_or_default(),
        _ => String::new(),
    }
}

/// One status lamp: catalogued artwork where the glyph has some, a coloured
/// character where it does not.
fn lamp(ui: &mut egui::Ui, glyph: &str, color: egui::Color32) -> egui::Response {
    match crate::icon_text::glyph(ui, glyph, color) {
        Some(art) => ui.add(art),
        None => ui.label(egui::RichText::new(glyph).color(color)),
    }
}

/// A vector as the seat readout prints it: three numbers, short.
fn triple(values: [f64; 3]) -> String {
    values
        .iter()
        .map(|value| {
            let rounded = (value * 1000.0).round() / 1000.0;
            // `-0` is the same place as `0` and reads as a mistake.
            if rounded == 0.0 { "0".to_string() } else { format!("{rounded}") }
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The footprint's pad numbers. A pad matches a pin by number symbol-wide, and
/// a pin carries its point's name, so a pad is this point's when its number is
/// the point's name — which is exactly the join `part_pins` makes.
fn pad_numbers(engine: &EngineState) -> Vec<String> {
    engine
        .history
        .pads_block()
        .and_then(|block| block.get("pads"))
        .and_then(Value::as_array)
        .map(|pads| {
            pads.iter()
                .filter_map(|pad| pad.get("number").and_then(Value::as_str))
                .map(str::to_string)
                .collect()
        })
        .unwrap_or_default()
}

/// `J1`, `J2`, … — the first that no group in the block has taken.
fn unique_group_name(block: &[Value]) -> String {
    let taken: Vec<&str> = block
        .iter()
        .filter_map(|group| group.get("name").and_then(Value::as_str))
        .collect();
    (1..).map(|n| format!("J{n}")).find(|name| !taken.contains(&name.as_str())).unwrap_or_default()
}

/// `1`, `2`, … — the first that no point in the group has taken. A number, not
/// a word, because a pin's label is what this becomes and pins are numbered.
fn unique_point_name(points: &[Value]) -> String {
    let taken: Vec<&str> = points
        .iter()
        .filter_map(|point| point.get("name").and_then(Value::as_str))
        .collect();
    (1..).map(|n| n.to_string()).find(|name| !taken.contains(&name.as_str())).unwrap_or_default()
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "qualify", prefix: "qualify:panel:clip", meaning: "the visible region of the pane", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:add_group", meaning: "declare another port group", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:group:", meaning: "a port group's name, purpose, symbol unit, add-point or remove", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:point:", meaning: "a connection point's select, name, remove, or one of the selected point's fields", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:jump:", meaning: "carry the selected point to the symbol, the pads or the 3D model", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:hint", meaning: "what selecting a point is for, shown while none is", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:seat", meaning: "what the last run made of the selected point's seat", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:gizmo", meaning: "whether the move/rotate handles are on the selected point", command: None },
    HitKeyDoc { panel: "qualify", prefix: "qualify:problem:", meaning: "one line of the pin/point or ports report", command: None },
];

