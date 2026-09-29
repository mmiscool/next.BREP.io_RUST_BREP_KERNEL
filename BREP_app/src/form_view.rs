//! Schema-driven form layout built on [`form::field_input`].
//!
//! The view edits parameter data and returns [`FormViewOut`] actions for the caller
//! to apply after drawing. It takes no engine or document, so the app and capture
//! tools can use the same layout without overlapping mutable engine borrows.
//!
//! Sections run from references and parameters through schema-ordered groups,
//! followed by Transform and read-only sections. Transform and Outputs start
//! collapsed. Reference selections stay visible below their activation button.
//!
//! Accordion and arrival-scroll state lives in egui memory keyed by the form's
//! hit prefix, title, and section.

use crate::form;
use brep_render::style::{FieldKind, FormField};
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// Groups that are drawn as a COLLAPSED accordion by default. `Transform` is the
/// owner's explicit call (R6); the read-only trailing sections join it because
/// they report, they do not edit. Everything else opens by default — a section
/// the user must click to see is a section they will miss.
const COLLAPSED_BY_DEFAULT: &[&str] = &["Transform", "Outputs"];

/// Everything ABOUT the form that the consumer supplies. Borrowed; the form view
/// holds no state of its own beyond egui memory.
pub struct FormViewSpec<'a> {
    /// The form's heading — e.g. `"E3  ⟠ Extrude"`. Also the scope key for this
    /// form's transient view state and per-field widget ids, so it must be
    /// STABLE and UNIQUE per subject (a feature id / constraint id qualifies).
    pub title: &'a str,
    /// An optional second line under the title (a status, a type name…).
    pub subtitle: Option<&'a str>,
    /// The schema fields to render, in schema order.
    pub fields: &'a [FormField],
    /// Field keys ([`FormField::key`]) to HIDE this frame — the schema-driven
    /// field-visibility hook. A field whose key is in this set is dropped BEFORE
    /// the fields are partitioned into sections, so a group left with nothing
    /// visible draws no header. `None` (the common case) hides nothing. A feature
    /// consumer fills this from [`brep_render::features::feature_hidden_params`]
    /// against the LIVE params each frame, so it reacts to both first display and
    /// every field change with no extra plumbing.
    pub hidden: Option<&'a HashSet<String>>,
    /// A message banner drawn under the title — e.g. the feature's run error.
    pub banner: Option<(&'a str, egui::Color32)>,
    /// Read-only sections appended after the fields — e.g. `[("Outputs", …)]`.
    pub trailing: Option<&'a [(&'a str, Vec<String>)]>,
    /// ONE consumer-drawn section `(title, drawer)` placed after the schema
    /// groups and before the trailing sections — for a subject whose editing
    /// surface the schema cannot express (a spline's anchor list). The drawer
    /// is `Fn`, so anything it decides goes through interior mutability the
    /// consumer owns; the form view stays intent-out like everything else here.
    pub extra: Option<(&'a str, &'a dyn Fn(&mut egui::Ui))>,
    /// The exit button's label — the button at the RIGHT end of the title row.
    /// Both of today's consumers say "Return to tree" — the history feature list
    /// and the constraint list are BOTH drawn by `panels::tree`, so the word is
    /// literal, not a metaphor (Q12). A consumer whose list is NOT a tree
    /// supplies its own wording here.
    pub exit_label: &'a str,
    /// Whether this consumer's subject lives in a ROLLED history — i.e. whether
    /// LEAVING the form also means "roll the model back to the tip".
    ///
    /// The form view holds no engine and therefore cannot roll anything itself
    /// (that is [`FormViewOut::roll_to_tip`], which the caller acts on). What
    /// this flag does is decide, in ONE place, whether an exit *carries* that
    /// intent — so no consumer has to hard-code "am I the history panel?" at its
    /// exit arm. The history panel passes `true`; assembly constraints have no
    /// rollback at all (the owner's call) and pass `false`, which is why their
    /// panel has no roll branch to get wrong.
    pub rollback: bool,
    /// Prefix for every published hit key. `""` for a consumer that shows ONE
    /// form at a time (history); a consumer that can show several at once MUST
    /// pass a prefix carrying the subject id or the rects collide silently.
    pub hits_prefix: &'a str,
    /// Why the subject may not change (a PLM revision not checked out, or
    /// released), or `None`. The fields then draw DISABLED under a banner
    /// saying why, so nothing is typed that the engine would refuse and the
    /// field's buffer would keep; the exit button stays live.
    pub read_only: Option<&'a str>,
}

/// A reference field's `Select` was pressed — the caller dispatches this to its
/// own picker flavour (`begin_ref_select` / `begin_ref_select_for_constraint`).
#[derive(Debug, Clone, PartialEq)]
pub struct RefActivate {
    /// The JSON key chain the picked selection writes back into.
    pub path: Vec<String>,
    /// The field's label, for the picker's prompt.
    pub label: String,
    /// The schema's entity-kind filter (`["solid"]`, `["face"]`…).
    pub filter: Vec<String>,
    /// Whether the field accepts more than one entity.
    pub multiple: bool,
    /// The names already chosen — the picker lights them up as the seed.
    pub seed: Vec<String>,
}

/// What the user did in one drawn frame of the form.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct FormViewOut {
    /// A field wrote into `params` — the caller commits the whole buffer.
    pub changed: bool,
    /// A [`FieldKind::Button`] field was clicked, by its key (`"editSketch"`).
    pub button_clicked: Option<String>,
    /// A reference field's `Select` was pressed.
    pub ref_activate: Option<RefActivate>,
    /// The title row's exit button was pressed — the caller returns to its list.
    pub exit_clicked: bool,
    /// The exit ALSO means "roll the model to the tip" — set only when the exit
    /// button was pressed AND the consumer declared [`FormViewSpec::rollback`].
    /// Always `false` for a consumer with no rollback, so its panel never needs
    /// the branch.
    pub roll_to_tip: bool,
    /// The pointer is over a line that NAMES a scene entity — a reference line or
    /// a read-only trailing line (`Outputs`). The caller feeds it to
    /// `EngineState::hover_entity_by_name` so the entity lights in the 3D view
    /// exactly as mousing over it there would, and calls
    /// `EngineState::dialog_hover_end` when it is `None`.
    pub hovered_entity: Option<String>,
}

/// Draw ONE complete schema-driven form into `ui`, editing `params` live, and
/// return what the user did. `hits`, when supplied, receives the widget screen
/// rects the headed verifier drives:
///
/// | key | what |
/// |---|---|
/// | `form:feature` | the title label |
/// | `form:section:{Group}` | an accordion header |
/// | `form:return` | the exit button, right end of the title row |
/// | `field:{path}` | a field's input widget |
/// | `field:{path}#{variant}` | an OPEN enum's items |
/// | `field:{path}#activate` / `#x{i}` | a reference's Select / remove buttons |
/// | `field:{path}#line{i}` | a reference's i-th chosen-entity line |
/// | `form:trailing:{Section}:{i}` | a read-only section's i-th line (`Outputs`) |
///
/// all with `spec.hits_prefix` prepended.
/// Padding INSIDE the form's container — the gap between the frame edge and the
/// first/last widget on every side.
const FORM_MARGIN: i8 = 10;

/// Gap OUTSIDE the container, between it and the pane edge, so the frame's own
/// stroke is not flush against the dock border.
const FORM_OUTER_MARGIN: i8 = 6;

pub fn form_view(
    ui: &mut egui::Ui,
    spec: &FormViewSpec<'_>,
    params: &mut Value,
    mut hits: Option<&mut HashMap<String, egui::Rect>>,
) -> FormViewOut {
    let mut out = FormViewOut::default();

    // A form ARRIVES IN VIEW. Every consumer's pane is wrapped in the dock's
    // `ScrollArea`, whose offset egui remembers PER PANE — so a form opened from
    // a tree the user had scrolled down inherits that offset and starts part-way
    // in, with the title row (and the exit button on it) above the viewport. The
    // enclosing scroll area is asked to bring the CURSOR — which is the form's
    // TOP EDGE here, before the container's own margins push it down — into
    // view, ONCE per arrival: keyed on the subject AND on having been drawn on
    // the previous pass, so re-opening the same subject after a trip back to the
    // tree resets too, while scrolling INSIDE an open form sticks.
    //
    // WHERE this sits matters: inside the container it would align the first
    // WIDGET, not the form, and leave the pane a margin's worth (measured: 14
    // pt) past the form's own top edge and border. Out here the cursor IS that
    // edge. The align is `None` — "only if it is not already visible" — rather
    // than `Align::TOP`, so a pane already at the top computes a zero delta and
    // asks for nothing at all, rather than a target that only happens to clamp
    // back to where it was.
    //
    // (`form_view` cannot reach the scroll area itself — it is the consumer's,
    // two levels out — and asking through the cursor is what keeps the required
    // inputs at `(ui, spec, &mut params)` for the headless caller.)
    let arrival = egui::Id::new(("form-view-arrival", spec.hits_prefix));
    let pass = ui.ctx().cumulative_pass_nr();
    let arrived = ui.ctx().memory_mut(|m| {
        let previous: Option<(String, u64)> = m.data.get_temp(arrival);
        let continuing = previous
            .is_some_and(|(title, drawn)| title == spec.title && pass.saturating_sub(drawn) <= 1);
        m.data.insert_temp(arrival, (spec.title.to_string(), pass));
        !continuing
    });
    if arrived {
        // INSTANT, not animated: the reset must land before the frame the user
        // sees, and a dialog gliding into place on every open is noise.
        ui.scroll_to_cursor_animation(None, egui::style::ScrollAnimation::none());
    }

    // The whole form sits inside ONE padded container, so the fields never run
    // flush against the pane edge the way the inline tree rendering did. The
    // margin is the frame's, not per-widget spacing: a single container keeps
    // the label-above-full-width rhythm intact when the pane is resized, and
    // gives the scroll region a consistent inset on every side.
    egui::Frame::group(ui.style())
        .inner_margin(egui::Margin::same(FORM_MARGIN))
        .outer_margin(egui::Margin::same(FORM_OUTER_MARGIN))
        .show(ui, |ui| {
            form_body(ui, spec, params, &mut hits, &mut out);
        });

    out
}

/// The form's contents, drawn INSIDE the padded container opened by
/// [`form_view`]. Split out so the container owns the margin in one place
/// rather than every section adding its own edge spacing.
fn form_body(
    ui: &mut egui::Ui,
    spec: &FormViewSpec<'_>,
    params: &mut Value,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut FormViewOut,
) {
    // Fill the container's width so full-width inputs stay full width.
    ui.set_width(ui.available_width());

    // --- heading + the ONE exit button, on one row -------------------------
    // No OK/Cancel pair and no buffer: editing is LIVE (every change commits and
    // re-runs) and UNDO is the revert mechanism, so there is nothing for a
    // Cancel to roll back that undo does not already cover.
    //
    // The exit is allocated FIRST, from the RIGHT edge, and the title takes what
    // is left over: a right-to-left row measures the button before the heading
    // exists, so no title — however long — can push the button off the pane, and
    // the heading truncates into the remainder instead of running under it. (The
    // reverse order, title-then-button, is exactly the layout that loses the
    // button on a long feature name.)
    ui.horizontal(|ui| {
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let exit = ui.button(spec.exit_label);
            publish(spec, hits, "form:return", exit.rect);
            out.exit_clicked = exit.clicked();
            // The exit's rollback half, gated by the consumer's declaration —
            // see [`FormViewSpec::rollback`].
            out.roll_to_tip = out.exit_clicked && spec.rollback;
            // Back to reading order for the title, inside whatever width the
            // button left behind.
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                let head = ui.add(
                    egui::Label::new(egui::RichText::new(spec.title).heading()).truncate(),
                );
                publish(spec, hits, "form:feature", head.rect);
            });
        });
    });
    if let Some(subtitle) = spec.subtitle {
        ui.label(egui::RichText::new(subtitle).weak());
    }
    ui.separator();

    // --- banner: the subject's error, ABOVE the fields ---------------------
    // (The history tree ALSO keeps this on the feature's row, so a failure is
    // still visible while scanning the tree — see the panel.)
    if let Some((text, color)) = spec.banner {
        egui::Frame::group(ui.style())
            .stroke(egui::Stroke::new(1.0, color))
            .fill(color.gamma_multiply(0.12))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.add(
                    egui::Label::new(egui::RichText::new(text).color(color))
                        .wrap_mode(egui::TextWrapMode::Wrap),
                );
            });
        ui.add_space(4.0);
    }

    // --- read-only: say why, and take no edit below this line ------------
    if let Some(reason) = spec.read_only {
        egui::Frame::group(ui.style())
            .stroke(egui::Stroke::new(1.0, ui.visuals().weak_text_color()))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                let said = ui.add(egui::Label::new(format!("Read-only: {reason}")).wrap_mode(egui::TextWrapMode::Wrap));
                publish(spec, hits, "form:read-only", said.rect);
            });
        ui.add_space(4.0);
        ui.disable();
    }

    // --- partition the schema into direct params + grouped sections --------
    let mut references: Vec<&FormField> = Vec::new();
    let mut param_leaves: Vec<&FormField> = Vec::new();
    let mut groups: Vec<(&str, Vec<&FormField>)> = Vec::new();
    for f in spec.fields {
        // The field-visibility hook: a hidden field is dropped here, before the
        // partition, so it never reaches a section — and an emptied group never
        // draws a bare header. Evaluated fresh each frame against the live params
        // by the consumer, this is what makes fields show/hide as the field that
        // governs them (e.g. a hole's type) changes.
        if spec.hidden.is_some_and(|h| h.contains(f.key())) {
            continue;
        }
        match f.group.as_str() {
            "References" => references.push(f),
            "Parameters" => {
                if matches!(f.kind, FieldKind::Text { read_only: true }) {
                    continue; // the subject id — already the form's title
                }
                param_leaves.push(f);
            }
            group => match groups.iter_mut().find(|(name, _)| *name == group) {
                Some(existing) => existing.1.push(f),
                None => groups.push((group, vec![f])),
            },
        }
    }
    // Transform trails the other groups: it is the one the owner named as the
    // collapsed accordion, so it belongs at the bottom, not between two open
    // sections.
    groups.sort_by_key(|(name, _)| usize::from(*name == "Transform"));

    // (1) references, then (2) plain parameters — both un-sectioned, because a
    // section over the fields the user came here to edit is a click in the way.
    for f in references.iter().chain(param_leaves.iter()) {
        draw_field(ui, spec, f, params, hits, out);
    }

    // (3) the remaining groups, each an accordion.
    for (name, fields) in &groups {
        section(ui, spec, name, hits, |ui, hits| {
            for f in fields {
                draw_field(ui, spec, f, params, hits, out);
            }
        });
    }

    // (4) the consumer's own section, when it has one.
    if let Some((name, draw)) = spec.extra {
        section(ui, spec, name, hits, |ui, _hits| draw(ui));
    }

    // (5) read-only trailing sections (Outputs…).
    //
    // These lines NAME scene entities too (a feature's output solids), so they
    // report a hover exactly as a reference line does — the section is read-only
    // in the sense that it edits nothing, not that it is inert.
    let mut trailing_hover: Option<String> = None;
    for (name, values) in spec.trailing.unwrap_or(&[]) {
        section(ui, spec, name, hits, |ui, hits| {
            if values.is_empty() {
                ui.label(egui::RichText::new("(none)").weak());
            }
            for (i, value) in values.iter().enumerate() {
                let line = ui.label(format!("• {value}"));
                if line.hovered() {
                    trailing_hover = Some(value.clone());
                }
                publish(spec, hits, &format!("form:trailing:{name}:{i}"), line.rect);
            }
        });
    }
    if trailing_hover.is_some() {
        out.hovered_entity = trailing_hover;
    }

    // Breathing room UNDER the last thing in the form, so the final section's
    // widgets are not flush against (and, once egui rounds their rects to
    // physical pixels, a hair past) the pane's clip edge — which is exactly
    // where a click stops landing.
    ui.add_space(8.0);
}

/// Record one widget rect under `spec.hits_prefix`.
fn publish(
    spec: &FormViewSpec<'_>,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    key: &str,
    rect: egui::Rect,
) {
    if let Some(map) = hits.as_deref_mut() {
        map.insert(format!("{}{key}", spec.hits_prefix), rect);
    }
}

/// One collapsible section: a header row that publishes `form:section:{name}`
/// and, when open, `body`. Open/collapsed lives in egui memory, keyed to this
/// form's subject, so two features' Transform sections remember separately and
/// the caller carries no state.
fn section(
    ui: &mut egui::Ui,
    spec: &FormViewSpec<'_>,
    name: &str,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    body: impl FnOnce(&mut egui::Ui, &mut Option<&mut HashMap<String, egui::Rect>>),
) {
    ui.add_space(6.0);
    let open = !COLLAPSED_BY_DEFAULT.contains(&name);
    let response = egui::CollapsingHeader::new(egui::RichText::new(name).strong())
        .id_salt(("form-view-section", spec.hits_prefix, spec.title, name))
        .default_open(open)
        .show(ui, |ui| body(ui, hits));
    publish(
        spec,
        hits,
        &format!("form:section:{name}"),
        response.header_response.rect,
    );
}

/// Draw ONE schema field: its label, then the input at FULL WIDTH beneath it.
/// A `Reference` is the same shape — [`form::field_input`] draws its activation
/// button and the chosen entities under this same label.
fn draw_field(
    ui: &mut egui::Ui,
    spec: &FormViewSpec<'_>,
    field: &FormField,
    params: &mut Value,
    hits: &mut Option<&mut HashMap<String, egui::Rect>>,
    out: &mut FormViewOut,
) {
    let mut probe: HashMap<String, egui::Rect> = HashMap::new();
    let mut rect = egui::Rect::NOTHING;
    let mut actions = form::FieldActions::default();
    ui.add_space(4.0);
    // A `Button` field IS its own label (the schema gives it one), so a label
    // above it would say the same thing twice.
    if !matches!(field.kind, FieldKind::Button { .. }) {
        ui.label(&field.label);
    }
    // Scope the widget id-stack to THIS (subject, field) so a Scalar's
    // per-location egui-memory edit buffer (and its TextEdit focus id) can't
    // collide when two subjects share a param name — switching between two
    // extrudes mid-edit must not hand one's half-typed `distance` to the other.
    // `make_persistent_id` folds in the ui id-stack only.
    ui.push_id((spec.title, field.key()), |ui| {
        let (changed, r) = form::field_input(ui, field, params, Some(&mut probe), &mut actions);
        out.changed |= changed;
        rect = r;
    });
    if let Some(map) = hits.as_deref_mut() {
        let prefix = spec.hits_prefix;
        map.insert(format!("{prefix}field:{}", field.path.join(".")), rect);
        for (key, r) in probe {
            map.insert(format!("{prefix}field:{key}"), r);
        }
    }
    // A hovered reference LINE names an entity; the form's consumer owns lighting
    // it (that needs the engine). One line is under the pointer at a time, so the
    // last field to report wins — and only one ever reports.
    if actions.hovered_entity.is_some() {
        out.hovered_entity = actions.hovered_entity.take();
    }
    // `field_input` reports the two ACTION kinds through the same out-param, by
    // field key; which intent it is comes from the field's own kind.
    if actions.clicked.is_some() {
        match &field.kind {
            FieldKind::Reference { filter, multiple } => {
                out.ref_activate = Some(RefActivate {
                    path: field.path.clone(),
                    label: field.label.clone(),
                    filter: filter.clone(),
                    multiple: *multiple,
                    seed: form::reference_names(form::value_at(params, &field.path)),
                });
            }
            _ => out.button_clicked = actions.clicked.take(),
        }
    }
}

