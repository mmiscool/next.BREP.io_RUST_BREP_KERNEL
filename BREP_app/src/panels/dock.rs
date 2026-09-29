//! The dockable / tabbed side-panel layout — an `egui_tiles` tree that hosts
//! every side-panel section AND the 3D viewport as tiles the user can split,
//! tab, resize, and drag-rearrange (IDE-style), with the layout persisted.
//!
//! This is **workbench-agnostic**, and strictly so: EVERY pane lives in the
//! tree permanently, at whatever position the user dragged it to, and selecting
//! a workbench changes nothing but VISIBILITY. A pane the active workbench does
//! not claim ([`workbench::panel_visible`]) is hidden *in place* — never removed
//! and re-inserted — so a workbench switch can no longer re-arrange the layout.
//! Positions / tabs / splits come from one shared, persisted layout. New
//! workbench panes plug in with one [`PaneKind`] arm.
//!
//! Structure:
//! * [`PaneKind`] — the serde discriminant of a tile; carries NO state.
//! * [`DockState`] — owns the `Tree<PaneKind>`, load/save/reconcile, and the
//!   per-frame workbench visibility pass
//!   ([`DockState::apply_workbench_visibility`]). One field on `BrepApp`.
//! * [`DockBehavior`] — a transient, per-frame `egui_tiles::Behavior` built from
//!   disjoint `&mut` borrows of the app's panels + engine (see [`DockContext`]);
//!   its `pane_ui` just delegates to each panel's existing `show(...)`.
//!
//! The shell draws the tree ONLY in normal modeling mode. In sketch / ref-select
//! mode it bypasses the tree and draws the viewport directly (see `app.rs`), so
//! the side panes simply don't appear — no reliance on container-visibility
//! edge cases.

use eframe::egui;
use egui_tiles::{
    Behavior, Container, EditAction, TabState, Tile, TileId, Tiles, Tree, UiResponse,
};
use serde::{Deserialize, Serialize};

use crate::document::Documents;
use crate::panels::assembly_constraints::AssemblyConstraintsPanel;
use crate::panels::component_actions::ComponentActionRequest;
use crate::panels::bom::BomPanel;
use crate::panels::document_tabs::TabsOutcome;
use crate::panels::expressions::ExpressionsPanel;
use crate::panels::history::HistoryPanel;
use crate::panels::scene::ScenePanel;
use crate::panels::update_components::UpdateComponents;
use crate::panels::wire_harness::WireHarnessPanel;
use crate::panels::pmi::PmiPanel;
use crate::panels::qualify::QualifyPanel;
use crate::panels::family_table_editor::FamilyTableEditor;
use crate::panels::sheets::SheetsPanel;
use crate::store::{ModelStore, DOCK_LAYOUT_KEY};
use brep_render::engine_state::EngineState;
use crate::viewport::Viewport;
use crate::workbench;

/// One tile in the dock tree. A pure discriminant — every panel's real state
/// lives on its own struct (a field of `BrepApp`), reached in `pane_ui`.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Serialize, Deserialize)]
pub enum PaneKind {
    /// ONE OPEN MODEL's 3D view, keyed by [`crate::document::Document::id`].
    ///
    /// These are the document tabs: they all live in a single `Tabs` container
    /// — the DOCUMENT GROUP — whose tab bar IS the model switcher, so the pane
    /// showing 3D views is tabbed by egui_tiles itself rather than carrying a
    /// second hand-drawn strip inside it.
    ///
    /// The id is process-unique and therefore meaningless in a RELOADED layout;
    /// [`DockState::sync_document_panes`] renumbers whatever it finds onto the
    /// live documents, which is what makes the persisted layout survive.
    Document(u64),
    History,
    AssemblyConstraints,
    Bom,
    /// The wire-harness connection list (claimed by the Wire harness
    /// workbench).
    WireHarness,
    /// The PMI view tree (claimed by the PMI workbench).
    Pmi,
    /// The drawing SHEETS the PMI views are placed on (claimed by the PMI
    /// workbench beside the view tree).
    Sheets,
    Scene,
    Expressions,
    /// The parts library of the active eCAD editor (claimed by Diagram and
    /// PCB): `Editor::library_panel`.
    EcadLibrary,
    /// The active eCAD editor's inspector (claimed by all four eCAD
    /// workbenches): `Editor::inspector` for Diagram and PCB, the symbol or
    /// pads editor's `properties` for Symbol and Pads.
    EcadInspector,
    /// The part's declared connection points: its port groups, their purpose,
    /// and the pin/point and ports-tail consistency reports. Claimed by NO
    /// workbench and gated on the DOCUMENT instead — it is on screen exactly
    /// while the part declares connection points
    /// ([`workbench::QUALIFY_PANEL_ID`]).
    Qualify,
    /// A family seed's member table. Claimed by no workbench and gated on the
    /// DOCUMENT, like Qualify: on screen exactly while a `.fbrep` is edited
    /// ([`workbench::FAMILY_TABLE_PANEL_ID`]).
    FamilyTable,
    /// Every PLM panel, as sections of one pane ([`crate::panels::plm_host`]).
    /// Claimed by no workbench and gated on the SESSION: on screen exactly
    /// while the store is a PLM ([`workbench::PLM_PANEL_ID`]).
    Plm,
}

impl PaneKind {
    /// The side panes in default TAB-STRIP order (excludes the viewport): the
    /// three unclaimed panes first, then the workbench-claimed ones. Every one
    /// of them is in the tree at all times — the workbench only hides them — so
    /// this is the order of the default left-hand tab strip, left to right.
    const SIDE: [PaneKind; 13] = [
        PaneKind::History,
        PaneKind::Scene,
        PaneKind::Expressions,
        PaneKind::AssemblyConstraints,
        PaneKind::Bom,
        PaneKind::WireHarness,
        PaneKind::Pmi,
        PaneKind::Sheets,
        PaneKind::EcadLibrary,
        PaneKind::EcadInspector,
        PaneKind::Qualify,
        PaneKind::FamilyTable,
        PaneKind::Plm,
    ];

    /// The side panes an automation caller can name — [`Self::SIDE`] under a
    /// public name, so `describe_panes` / `show_pane` enumerate the dock rather
    /// than keeping a second list of it.
    pub const ALL: [PaneKind; 13] = Self::SIDE;

    /// Human tab title.
    pub fn title(self) -> &'static str {
        match self {
            // Only a fallback: the real per-document title (file name + dirty
            // marker) comes from `DockBehavior::tab_title_for_pane`, which can
            // reach the open documents.
            PaneKind::Document(_) => "3D View",
            PaneKind::History => "History",
            // The component TREE. It was titled "BOM" before the BOM
            // existed; now that there is a real columned parts list next to
            // it, the honest name is what it draws.
            PaneKind::Bom => "BOM",
            PaneKind::AssemblyConstraints => "Constraints",
            PaneKind::WireHarness => "Wire Harness",
            PaneKind::Pmi => "PMI",
            PaneKind::Sheets => "Sheets",
            PaneKind::Scene => "Scene",
            PaneKind::Expressions => "Expressions",
            PaneKind::EcadLibrary => "Assembly parts",
            PaneKind::EcadInspector => "Inspector",
            PaneKind::Qualify => "Qualify",
            PaneKind::FamilyTable => "Family table",
            PaneKind::Plm => "PLM",
        }
    }

    /// The workbench-registry panel id this pane is claimed under, or `None` for
    /// the viewport (which is not a workbench-filterable panel). Must match the
    /// ids used in `app.rs`'s old `panel_visible` gates + `workbench::assembly`.
    fn panel_id(self) -> Option<&'static str> {
        match self {
            PaneKind::Document(_) => None,
            PaneKind::History => Some("history"),
            PaneKind::Bom => Some(workbench::assembly::BOM_PANEL_ID),
            PaneKind::AssemblyConstraints => Some(workbench::assembly::CONSTRAINTS_PANEL_ID),
            PaneKind::WireHarness => Some(workbench::wire_harness::PANEL_ID),
            PaneKind::Pmi => Some(workbench::pmi::PANEL_ID),
            PaneKind::Sheets => Some(workbench::drawing::SHEETS_PANEL_ID),
            PaneKind::Scene => Some("scene"),
            PaneKind::Expressions => Some("expressions"),
            PaneKind::EcadLibrary => Some(workbench::ecad::LIBRARY_PANEL_ID),
            PaneKind::EcadInspector => Some(workbench::ecad::INSPECTOR_PANEL_ID),
            PaneKind::Qualify => Some(workbench::QUALIFY_PANEL_ID),
            PaneKind::FamilyTable => Some(workbench::FAMILY_TABLE_PANEL_ID),
            PaneKind::Plm => Some(workbench::PLM_PANEL_ID),
        }
    }

    /// [`Self::visible_in`] under a public name, so `show_pane` can refuse a
    /// pane the active workbench does not carry instead of no-op'ing.
    pub fn visible_under_workbench(self, wb: &str, panels: &workbench::ButtonState) -> bool {
        self.visible_in(wb, panels)
    }

    /// Whether this pane is visible under workbench `wb`, for this document.
    /// The viewport is always visible; side panes defer to
    /// [`workbench::panel_visible`], which answers both halves — the workbench
    /// claim AND, for a conditional pane like Qualify, the document.
    fn visible_in(self, wb: &str, panels: &workbench::ButtonState) -> bool {
        match self.panel_id() {
            None => true,
            Some(id) => workbench::panel_visible(wb, id, panels),
        }
    }
}

/// **The dialog door.** Which dynamically generated dialog is open, per HOSTING
/// PANE, as of one frame — read by [`DialogTargets::read`], diffed by
/// [`DockState::surface_opened_dialogs`], which brings the host pane's tab
/// forward for every dialog that JUST opened.
///
/// # Why a watcher and not a setter
///
/// The obvious door is "one function that sets the form target and calls
/// `show_pane`". It cannot be written: the openers do not share a place that
/// can reach the dock.
///
/// * Three of the four targets are set INSIDE the engine, as a side effect of
///   the add: `EngineState::pmi_add_annotation`, `sheet_place_view`,
///   `sheet_add_dimension`, `sheet_add_ordinate`, `sheet_new_section` / `_detail`, and the kernel's own
///   `open: true` on a minted constraint. `brep_render` knows nothing of a
///   dock and must not.
/// * Two are set from VIEWPORT draw code — a PMI label double click
///   (`viewport::labels` → `pmi_label_double_clicked`) and a constraint label click
///   (`constraint_label_clicked`) — which runs inside the dock's own draw, with
///   the tree already borrowed.
/// * The rest are set inside a PANEL's `show(ui, state)`, which is handed an
///   engine and a `Ui` and deliberately nothing else.
/// * The automation commands (`pmi_add_annotation`, `assembly_add_constraint`,
///   `sheet_add_dimension`) open a form with no UI frame involved at all.
///
/// So the door watches the TARGET instead of the caller: one read per frame,
/// one comparison, and every present and future opener is covered without a
/// line of its own.
///
/// # Why every target carries a COUNT
///
/// The one thing a pure state diff cannot see is a form RE-opened on the same
/// subject: the id does not move, so the open reads as "nothing happened" and
/// the pane stays behind its tab — the one case where the user is CERTAIN to be
/// looking somewhere else, because re-opening a form you can already see is not
/// a gesture anyone makes. So every target is `(subject, open count)`, and each
/// of the four has a monotonic counter its openers go through:
/// `HistoryPanel::open_form` (the context bar's "Edit owning feature" on the
/// feature whose form is already up), `EngineState::pmi_dialog_opens` and
/// `sheet_object_opens` (a viewport label double click, a double click on the same
/// dimension on the paper) and `assembly_constraint_opens` (a constraint label click).
/// A close never counts, so the rule below still reads "opened", not "changed".
///
/// The one opener that reaches no counter is the kernel minting a constraint
/// `open: true`, which never passes `assembly_set_constraint_open`. It needs
/// none: a new constraint brings a new id, and the subject half sees that.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct DialogTargets {
    /// The document these targets were read from. A document SWITCH re-baselines
    /// rather than surfacing: doc A's tree and doc B's open annotation are not a
    /// dialog the user just opened.
    document: u64,
    /// The History pane's open feature form: its subject and
    /// `HistoryPanel::form_opens`.
    feature: (Option<String>, u64),
    /// The PMI pane's open annotation or view form, and `pmi_dialog_opens`.
    pmi: (Option<String>, u64),
    /// The Constraints pane's open constraint form, and
    /// `assembly_constraint_opens`.
    constraint: (Option<String>, u64),
    /// The Sheets pane's open sheet / placed-view / dimension form, and
    /// `sheet_object_opens`.
    sheet: (Option<String>, u64),
}

impl DialogTargets {
    /// Read this frame's targets. `document` is the ACTIVE document's id and
    /// `engine` its engine — the two must agree, or a switch reads as an open.
    pub fn read(history: &HistoryPanel, engine: &EngineState, document: u64) -> Self {
        let (feature, opens) = history.form_open();
        Self {
            document,
            feature: (feature.map(str::to_string), opens),
            pmi: (
                engine
                    .pmi_open_annotation()
                    .or(engine.pmi_open_view())
                    .map(str::to_string),
                engine.pmi_dialog_opens(),
            ),
            constraint: (engine.assembly_open_constraint(), engine.assembly_constraint_opens()),
            sheet: (
                engine.sheet_open_object().map(str::to_string),
                engine.sheet_object_opens(),
            ),
        }
    }
}

/// The dock layout: the tile tree + a dirty flag so a user layout edit persists.
/// One field on `BrepApp`.
pub struct DockState {
    tree: Tree<PaneKind>,
    /// Set by [`DockBehavior::on_edit`] when the user drags / resizes a tile;
    /// drained in [`DockState::ui`] to persist the new layout. A workbench
    /// switch never sets it: visibility is derived per frame, not stored state.
    /// The eCAD door does, on the frame one of its notes flips.
    dirty: bool,
    /// Per-pane `(kind, visible, rendered-this-frame)` from the last `ui()`, in
    /// TAB-STRIP order (a depth-first walk of the tree, so the published list's
    /// indices are stable between runs) — the source for the `__brepDock`
    /// verifier global. EVERY pane is listed, always:
    /// `visible` is false for one the active workbench does not claim (it keeps
    /// its place in the tree), and `rendered` is false for a pane sitting behind
    /// an inactive tab (egui_tiles skips its `pane_ui`), so an e2e script knows
    /// to switch workbench and/or activate that tab before asserting on its
    /// widgets.
    snapshot: Vec<(PaneKind, bool, bool)>,
    /// Last frame's [`DialogTargets`], or `None` before the first read — the
    /// baseline [`Self::surface_opened_dialogs`] diffs against.
    dialogs: Option<DialogTargets>,
    /// `(document, whether it declares connection points)` as of last frame —
    /// the baseline for the QUALIFY pane's door ([`Self::surface_declared_ports`]).
    ports: Option<(u64, bool)>,
    /// `(document, workbench id, whether the door notes)` as of last frame —
    /// the baseline for the eCAD workbenches' door ([`Self::surface_ecad_pane`]).
    ecad_entry: Option<(u64, String, Noting)>,
    /// The eCAD panes the user put another tab in front of, the last frame
    /// they were their workbench's own pane on screen. The door leaves these
    /// behind on the next entry: the user's choice, not the door's. Saved with
    /// the layout, so a restart remembers it too.
    ecad_declined: Vec<PaneKind>,
}

/// Whether the eCAD door takes notes this frame ([`DockState::surface_ecad_pane`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Noting {
    /// No eCAD pane is its workbench's own on screen.
    Off,
    /// The pane came on screen without an entry — the first frame, or a
    /// document switch — with this front state. Nothing the user chose, so no
    /// note, until the front state changes under the user's hand.
    Baseline(bool),
    /// After a real entry, or a front-tab change since the baseline.
    Live,
}

/// The field of the saved layout that carries [`DockState`]'s eCAD door notes.
const ECAD_DECLINED_KEY: &str = "brepEcadDeclined";

/// The disjoint `&mut` borrows the dock needs to draw a frame — assembled by the
/// shell from `BrepApp`'s fields (all distinct, so the borrow checker allows it).
pub struct DockContext<'a> {
    /// The open documents. The dock reaches the ACTIVE engine through
    /// `docs.engine_mut()` — a borrow of one FIELD, so it still composes with
    /// the disjoint panel borrows beside it (see `crate::document`).
    pub docs: &'a mut Documents,
    pub viewport: &'a mut Viewport,
    pub history: &'a mut HistoryPanel,
    pub bom: &'a mut BomPanel,
    pub assembly_constraints: &'a mut AssemblyConstraintsPanel,
    pub wire_harness: &'a mut WireHarnessPanel,
    pub pmi: &'a mut PmiPanel,
    pub sheets: &'a mut SheetsPanel,
    pub scene: &'a mut ScenePanel,
    pub expressions: &'a mut ExpressionsPanel,
    pub qualify: &'a mut QualifyPanel,
    pub family_table: &'a mut FamilyTableEditor,
    pub update_components: &'a mut UpdateComponents,
    pub model_store: &'a dyn ModelStore,
    /// Every PLM panel ([`crate::panels::plm_host`]); drawn only in a PLM session.
    pub plm: &'a mut crate::panels::plm_host::PlmHost,
}

/// What a dock frame hands back to the shell — the SAME cross-panel requests the
/// old left-panel closure bubbled out (borrows inside prevent acting there).
#[derive(Default)]
pub struct DockOutcome {
    /// The ACOMP palette pick asked to open the component-selector modal.
    pub insert_component_requested: bool,
    /// A structure-tree Edit asked to expand this feature in the history tree.
    pub feature_focus: Option<String>,
    /// Structure-tree row interactions (Move / Edit-in-place / Open Part).
    /// A document-level component flow a pane's row menu asked for (the BOM's;
    /// the engine-mutating half already ran in the shared dispatcher).
    pub component_request: Option<ComponentActionRequest>,
    /// What the DOCUMENT TAB STRIP inside the viewport tile was clicked for —
    /// acted on by the shell, which owns the unsaved-changes prompt (close) and
    /// the shared-panel reset (activate).
    pub document_tabs: TabsOutcome,
}

impl DockState {
    /// Load the persisted layout (reconciled against the current pane set), or
    /// fall back to the default layout.
    pub fn new(store: &dyn ModelStore) -> Self {
        let saved = store
            .read(DOCK_LAYOUT_KEY)
            .and_then(|json| serde_json::from_str::<serde_json::Value>(&json).ok());
        // The eCAD door's notes ride in the layout file, beside the tree's own
        // fields ([`Self::save`]); a file from before them has none.
        let ecad_declined = saved
            .as_ref()
            .and_then(|json| json.get(ECAD_DECLINED_KEY))
            .and_then(|notes| serde_json::from_value::<Vec<PaneKind>>(notes.clone()).ok())
            .unwrap_or_default();
        let tree = saved
            .and_then(|json| serde_json::from_value::<Tree<PaneKind>>(json).ok())
            .and_then(salvage)
            .unwrap_or_else(default_tree);
        Self {
            tree,
            dirty: false,
            snapshot: Vec::new(),
            dialogs: None,
            ports: None,
            ecad_entry: None,
            ecad_declined,
        }
    }

    /// Draw the dock tree, delegating each visible pane to its panel's `show`.
    /// Applies workbench visibility first, then persists if the user re-laid it.
    pub fn ui(&mut self, ui: &mut egui::Ui, ctx: DockContext<'_>) -> DockOutcome {
        let wb = ctx.docs.engine().settings.workbench.clone();
        // Visibility is asked of the workbench AND the document, so it is
        // computed here, before the behavior takes `&mut docs`.
        self.apply_workbench_visibility(&wb, &workbench::ButtonState::of(ctx.docs.engine()));
        // The QUALIFY pane's door, run right after the visibility pass and for
        // the same reason the dialog door runs where it does: a pane that was
        // hidden a moment ago cannot be surfaced until the pass has let it back
        // in, and `show_pane` would rightly no-op. This is the watcher shape,
        // not a setter — every way a part comes to declare its first connection
        // point is covered (the Wire harness button, the KiCad import, a symbol
        // pin, an undo that brings the block back) without a line in any of them.
        self.surface_declared_ports(
            ctx.docs.active_id(),
            workbench::declares_connection_points(&workbench::ButtonState::of(ctx.docs.engine())),
            &wb,
        );
        self.surface_ecad_pane(ctx.docs.active_id(), &wb);
        // One tab per open model, active tab = active model. Must run BEFORE the
        // draw so a document opened or closed last frame is already reflected in
        // the bar the user is about to see.
        if self.sync_document_panes(ctx.docs) {
            self.dirty = true;
        }
        // Restore the group's exclusivity, so a pane the user dropped in there
        // last frame never gets a second frame among the document tabs.
        if self.evict_foreign_panes_from_document_group() {
            self.dirty = true;
        }

        // Snapshot the document identity before the behavior takes `&mut docs`,
        // so the post-draw tab-click read has something to compare against.
        let active_id = ctx.docs.active_id();
        let document_ids: Vec<u64> = ctx.docs.iter().map(|d| d.id()).collect();

        let store = ctx.model_store;
        let mut behavior = DockBehavior::new(ctx);
        behavior.tab_title_spacing = behavior.tab_title_spacing(ui.visuals());
        self.tree.ui(&mut behavior, ui);

        // A tab CLICK shows up as egui_tiles' own active tab disagreeing with
        // `Documents`; the shell resolves it by activating that document.
        behavior.document_tabs.activate = self.tab_bar_selection(active_id, &document_ids);

        let outcome = DockOutcome {
            insert_component_requested: behavior.insert_component_requested,
            feature_focus: behavior.feature_focus.take(),
            component_request: behavior.component_request.take(),
            document_tabs: std::mem::take(&mut behavior.document_tabs),
        };
        if behavior.layout_changed {
            self.dirty = true;
        }
        let rendered = std::mem::take(&mut behavior.rendered);
        // A pane egui_tiles skipped — behind another tab, or hidden by the
        // workbench — ran no `show`, so nothing cleared its rects and last
        // draw's would be published again: live keys over whatever pane is on
        // screen there now.
        behavior.retract_undrawn(&rendered);
        drop(behavior);

        // Snapshot for the `__brepDock` verifier global, in TAB-STRIP order: a
        // depth-first walk from the root, not `Tiles::iter`, which walks a
        // HashMap and would hand the verifier a list whose indices move
        // between runs — an unassertable published blob.
        self.snapshot = self
            .panes_in_layout_order()
            .into_iter()
            .map(|(id, kind)| (kind, self.tree.tiles.is_visible(id), rendered.contains(&kind)))
            .collect();

        // Persist the layout, but DEBOUNCED: `on_edit` fires every frame while a
        // tile is being drag-resized (the shares change per mouse-move), so only
        // serialize once the pointer is released — mirrors how the shell defers
        // `applied_ui_scale`. A dirty flag set mid-drag simply waits for release.
        if self.dirty && !ui.ctx().input(|i| i.pointer.any_down()) {
            self.save(store);
            self.dirty = false;
        }
        outcome
    }

    /// For a frame the dock is NOT drawn (sketch mode, reference selection):
    /// no side pane is on screen, so none may keep publishing rects.
    pub fn retract_all(ctx: DockContext<'_>) {
        DockBehavior::new(ctx).retract_undrawn(&[]);
    }

    /// The `__brepDock` verifier global:
    /// `{active, panes:[{kind,visible,activeTab,rendered}]}`. `active` is whether
    /// the dock owns the layout right now (false in sketch / ref-select mode,
    /// where the shell draws the viewport directly). When inactive, no side pane
    /// is rendered — the caller passes `active=false`.
    ///
    /// `activeTab` is read from the TREE as it stands now, not from the draw
    /// snapshot, so a tab the dialog door brought forward this frame is already
    /// true here — that is the difference between a script that can assert on
    /// the door and one that has to pump a frame and hope. `rendered` stays the
    /// snapshot's: it answers "were this pane's rects published", which is
    /// necessarily last draw's answer.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn state_json(&self, active: bool) -> String {
        let panes: Vec<serde_json::Value> = self
            .snapshot
            .iter()
            .map(|(kind, visible, rendered)| {
                serde_json::json!({
                    "kind": format!("{kind:?}"),
                    "visible": visible,
                    "activeTab": self.find_pane(*kind).is_some_and(|id| self.tab_active(id)),
                    "rendered": active && *rendered,
                })
            })
            .collect();
        serde_json::json!({ "active": active, "panes": panes }).to_string()
    }

    /// Whether tile `id` is the tab its tab group would draw — for EVERY tab
    /// container between it and the root, so a pane tabbed inside a tabbed
    /// container answers honestly.
    ///
    /// Asks `Tabs::next_active` rather than reading `Tabs::active` directly:
    /// that is the same function egui_tiles' layout calls, so an `active` that
    /// is unset or points at a hidden sibling resolves to the child the layout
    /// will really draw instead of to a stale id.
    fn tab_active(&self, id: TileId) -> bool {
        let mut child = id;
        while let Some(parent) = self.tree.tiles.parent_of(child) {
            if let Some(Tile::Container(Container::Tabs(tabs))) = self.tree.tiles.get(parent) {
                if tabs.next_active(&self.tree.tiles) != Some(child) {
                    return false;
                }
            }
            child = parent;
        }
        true
    }

    /// Surface the pane of `kind` — make it the ACTIVE tab in its tab group so a
    /// pane sitting behind another tab becomes visible. No-op if it is already
    /// active. Used to bring the History tab forward when a feature is added (a
    /// context-bar create can happen while another side tab is showing), so the
    /// new row is actually seen (`app.rs`, paired with `history.focus_feature`).
    ///
    /// A pane the active WORKBENCH hides is left alone. Activating its tab would
    /// not show it — `Tabs::ensure_active` moves `active` back to a VISIBLE tab
    /// during layout — but it would drag the user off whatever tab they were on
    /// and onto the first visible one, which is worse than doing nothing. Every
    /// caller today is already gated on the workbench that claims the pane, and
    /// the automation entry point refuses the call outright rather than no-op'ing
    /// (see `automation::cmd_shell::show_pane`); this keeps a future one honest.
    pub fn show_pane(&mut self, kind: PaneKind) {
        match self.find_pane(kind) {
            Some(id) if self.tree.tiles.is_visible(id) => {}
            _ => return,
        }
        self.tree
            .make_active(|_id, tile| matches!(tile, Tile::Pane(k) if *k == kind));
    }

    /// **The dialog door's dock half.** Bring the host pane's tab forward for
    /// every dynamically generated dialog that opened since the last call.
    ///
    /// Called once per frame by the shell with this frame's
    /// [`DialogTargets`]. A target that went from closed to open, from one
    /// subject to another, or that was RE-opened on the subject it already had
    /// (its open count moved), is a dialog the user just opened — its pane gets
    /// [`Self::show_pane`], which is a no-op when that tab is already in front.
    /// A target that CLOSED, or that never moved, surfaces nothing: the user's
    /// tab is theirs.
    ///
    /// A DOCUMENT SWITCH re-baselines and surfaces nothing. Each document has
    /// its own engine, so switching from a document with nothing open to one
    /// with an annotation form open would otherwise read as an open and yank
    /// the user to the PMI tab for a form they opened in another model.
    ///
    /// One consequence worth knowing: the CONSTRAINT open flag is part of the
    /// SAVED DOCUMENT (the kernel persists it per constraint), so opening a
    /// document that was saved with a constraint's dialog open surfaces the
    /// Constraints tab — a load into the same document is not a switch, so it
    /// reads as an open, which is the honest answer for a document that really
    /// does have a dialog open. The other three targets are engine state and
    /// never come back from a file.
    ///
    /// A pane the active WORKBENCH hides is left alone by `show_pane` (see its
    /// doc comment). The openers that can reach that case are the ungated ones
    /// — a viewport PMI label double click or constraint label click, a sheet-canvas click, and the
    /// automation adds — and what the user sees then is nothing: the form is
    /// open in state, its pane is not in this workbench, and dragging them onto
    /// some other visible tab instead would be worse than doing nothing. The
    /// panel-, context-bar- and toolbar-driven openers are all gated on the
    /// workbench that claims the pane, so they cannot reach it.
    pub fn surface_opened_dialogs(&mut self, current: DialogTargets) {
        let Some(previous) = self.dialogs.replace(current.clone()) else {
            return;
        };
        if previous.document != current.document {
            return;
        }
        for (current, previous, host) in [
            (&current.feature, &previous.feature, PaneKind::History),
            (&current.pmi, &previous.pmi, PaneKind::Pmi),
            (&current.constraint, &previous.constraint, PaneKind::AssemblyConstraints),
            (&current.sheet, &previous.sheet, PaneKind::Sheets),
        ] {
            // `is_some()` is what makes this "opened" and not merely "changed":
            // a form CLOSING moves the target too, and must move no tab. The
            // pair moves on a new subject OR on a re-open of the same one.
            if current.0.is_some() && current != previous {
                self.show_pane(host);
            }
        }
    }

    /// Bring the Qualify pane forward for a document that has just BEGUN
    /// declaring connection points — the moment its pane exists at all.
    ///
    /// Diffed like the dialog door: only a false→true step surfaces, so a part
    /// that has declared points all along never drags the user off their tab,
    /// and losing the last point (which closes the pane) surfaces nothing. A
    /// DOCUMENT SWITCH re-baselines rather than surfacing, for the same reason
    /// it does there: another model's ports are not something this user just did.
    ///
    /// **Not in Symbol or Pads.** There the first connection point is the first
    /// PIN or PAD the user places, and they are placing it with the Inspector in
    /// front — the eCAD door put it there, and it is the pane that shows the pin
    /// just placed. Jumping to Qualify on that one click (and on no later one)
    /// took the user off the pane they were working in; the Qualify tab still
    /// appears in the strip beside it. The baseline moves all the same, so
    /// leaving for 3D afterwards surfaces nothing either: the points were not
    /// declared there.
    fn surface_declared_ports(&mut self, document: u64, declares: bool, wb: &str) {
        let previous = self.ports.replace((document, declares));
        let authoring = matches!(
            workbench::ecad::Target::of_workbench(wb),
            Some(workbench::ecad::Target::Symbol | workbench::ecad::Target::Pads)
        );
        if previous == Some((document, false)) && declares && !authoring {
            self.show_pane(PaneKind::Qualify);
        }
    }

    /// **The eCAD workbenches' door.** Entering Diagram or PCB brings
    /// **Assembly parts** to the front of its tab group, and entering Symbol or
    /// Pads brings the **Inspector** (the only pane either claims) — so a first
    /// run lands on the pane the canvas tells the user to use, not on History.
    ///
    /// # The rule for a user who picks another tab on purpose
    ///
    /// **The pane comes back the way you left it.** While a workbench's own
    /// pane is on screen, the door notes every frame whether it is the tab in
    /// front. If the user put another tab (History, Scene…) in front of it and
    /// left the workbench like that, the next entry leaves their tab alone; if
    /// they left with the pane in front, or never entered before, it is brought
    /// forward. So a user who wants History beside the board picks it once, not
    /// on every return, and a user who never touches the tabs always lands on
    /// the parts. Bringing the pane back by hand clears the note.
    ///
    /// **Nor over Qualify.** Its jump row (Symbol, Pads, 3D) switches
    /// workbench from inside the pane, so an entry made with Qualify in front
    /// is made by a user working in Qualify, who is left there.
    ///
    /// Only an ENTRY moves a tab: a frame whose workbench did not change does
    /// nothing, and neither does a switch between two workbenches that share the
    /// pane (Diagram ↔ PCB, Symbol ↔ Pads), where the pane never left the
    /// screen. Like the other doors, the first frame and a DOCUMENT switch only
    /// re-baseline: the saved layout's front tab, and another model's workbench,
    /// are not something the user just did. The notes are saved with the
    /// layout: a user who left PCB on History and quit comes back to History.
    ///
    /// So a baseline takes no NOTE either. An app that starts in PCB with
    /// History in front has made no choice; noting "declined" there would keep
    /// the parts behind History on every later entry with nobody having picked
    /// that. Notes start at the first real entry, or at the first frame the
    /// front tab changes after a baseline — a click in the tab strip, which is
    /// a choice.
    fn surface_ecad_pane(&mut self, document: u64, wb: &str) {
        let own = |wb: &str| match workbench::ecad::Target::of_workbench(wb) {
            Some(workbench::ecad::Target::Diagram | workbench::ecad::Target::Pcb) => Some(PaneKind::EcadLibrary),
            Some(workbench::ecad::Target::Symbol | workbench::ecad::Target::Pads) => Some(PaneKind::EcadInspector),
            None => None,
        };
        let previous = self.ecad_entry.take();
        let Some(pane) = own(wb) else {
            self.ecad_entry = Some((document, wb.to_string(), Noting::Off));
            return;
        };
        let same_document = matches!(&previous, Some((d, ..)) if *d == document);
        let entered = same_document && matches!(&previous, Some((_, before, _)) if own(before) != Some(pane));
        if entered && !self.ecad_declined.contains(&pane) && self.front_beside(pane) != Some(PaneKind::Qualify) {
            self.show_pane(pane);
        }
        // Noted after the door acts, so the entry frame reads its own result.
        let in_front = self.find_pane(pane).is_some_and(|id| self.tree.tiles.is_visible(id) && self.tab_active(id));
        let noting = match previous {
            _ if entered => Noting::Live,
            Some((_, _, Noting::Baseline(was))) if same_document => match was == in_front {
                true => Noting::Baseline(was),
                false => Noting::Live,
            },
            Some((_, _, noting)) if same_document => noting,
            _ => Noting::Baseline(in_front),
        };
        self.ecad_entry = Some((document, wb.to_string(), noting));
        if noting != Noting::Live {
            return;
        }
        if self.ecad_declined.contains(&pane) == in_front {
            // The note flips: the layout file carries it, so write it out.
            self.dirty = true;
        }
        self.ecad_declined.retain(|kind| *kind != pane);
        if !in_front {
            self.ecad_declined.push(pane);
        }
    }

    /// The pane in front of the tab group `kind` sits in (itself when it is in
    /// front), as the layout would resolve it; `None` for a pane not in a tab
    /// group.
    fn front_beside(&self, kind: PaneKind) -> Option<PaneKind> {
        let id = self.find_pane(kind)?;
        let parent = self.tree.tiles.parent_of(id)?;
        let Some(Tile::Container(Container::Tabs(tabs))) = self.tree.tiles.get(parent) else {
            return None;
        };
        match self.tree.tiles.get(tabs.next_active(&self.tree.tiles)?) {
            Some(Tile::Pane(front)) => Some(*front),
            _ => None,
        }
    }

    /// Point the tree's VISIBILITY at the active workbench — the only thing
    /// selecting a workbench is allowed to change about the dock.
    ///
    /// Every pane stays exactly where the user put it; one the active workbench
    /// does not claim is merely hidden. Hiding is PROPAGATED up: a container
    /// whose every child ends up hidden is hidden too, so a claimed pane parked
    /// in its own split leaves behind neither an empty tab bar nor a dead strip
    /// of layout — which is what an earlier revision removed panes structurally
    /// to avoid, at the cost of forgetting their position on every switch.
    ///
    /// Runs EVERY frame rather than on workbench change, because a drag can
    /// create the very container that now needs hiding (drop the Constraints
    /// pane into its own split, then switch to Modeling). It is a walk of a
    /// tree with a dozen nodes, and it sets no dirty flag: visibility is
    /// derived from the workbench, never persisted state.
    fn apply_workbench_visibility(&mut self, wb: &str, panels: &workbench::ButtonState) {
        let Some(root) = self.tree.root() else {
            return;
        };
        self.refresh_visibility(root, wb, panels);
        // The root is what the dock draws into: it shows even in the degenerate
        // case where the workbench claims none of what is in it.
        self.tree.tiles.set_visible(root, true);
    }

    /// The post-order half of [`Self::apply_workbench_visibility`]: set `id`'s
    /// visibility and answer whether its subtree has anything left to show.
    fn refresh_visibility(&mut self, id: TileId, wb: &str, panels: &workbench::ButtonState) -> bool {
        let visible = match self.tree.tiles.get(id) {
            Some(Tile::Pane(kind)) => kind.visible_in(wb, panels),
            // `children_vec` first: the recursion needs `&mut tiles` back.
            // `|=` on `bool` does not short-circuit, so every child is visited
            // (each one has its OWN visibility to set, not just a vote here).
            Some(Tile::Container(container)) => {
                let mut any = false;
                for child in container.children_vec() {
                    any |= self.refresh_visibility(child, wb, panels);
                }
                any
            }
            None => return false,
        };
        self.tree.tiles.set_visible(id, visible);
        visible
    }

    /// Every pane in the tree, in LAYOUT order: the order a depth-first walk
    /// from the root meets them, which for a tab group is its tab-strip order
    /// left to right. Deterministic, unlike `Tiles::iter`.
    fn panes_in_layout_order(&self) -> Vec<(TileId, PaneKind)> {
        let mut out = Vec::new();
        if let Some(root) = self.tree.root() {
            self.collect_panes(root, &mut out);
        }
        out
    }

    /// The recursive half of [`Self::panes_in_layout_order`].
    fn collect_panes(&self, id: TileId, out: &mut Vec<(TileId, PaneKind)>) {
        match self.tree.tiles.get(id) {
            Some(Tile::Pane(kind)) => out.push((id, *kind)),
            Some(Tile::Container(container)) => {
                for child in container.children() {
                    self.collect_panes(*child, out);
                }
            }
            None => {}
        }
    }

    /// The tile id of the pane of `kind`, if present.
    fn find_pane(&self, kind: PaneKind) -> Option<TileId> {
        self.tree.tiles.iter().find_map(|(id, tile)| match tile {
            Tile::Pane(k) if *k == kind => Some(*id),
            _ => None,
        })
    }

    /// The DOCUMENT GROUP: the `Tabs` container whose tab bar is the model
    /// switcher. Identified by content — the container holding document panes —
    /// so it survives the user re-docking it anywhere in the tree.
    fn document_group(&self) -> Option<TileId> {
        let pane = self.tree.tiles.iter().find_map(|(id, tile)| {
            matches!(tile, Tile::Pane(PaneKind::Document(_))).then_some(*id)
        })?;
        let parent = self.tree.tiles.parent_of(pane)?;
        matches!(self.tree.tiles.get(parent), Some(Tile::Container(Container::Tabs(_))))
            .then_some(parent)
    }

    /// Match the document panes to the OPEN DOCUMENTS: one pane per document, in
    /// the documents' own order, with the active document's pane as the active
    /// tab.
    ///
    /// This is what lets the tab bar be egui_tiles' own rather than a strip drawn
    /// inside a pane. It also absorbs the id problem: `Document` ids are
    /// process-unique, so the panes in a RELOADED layout carry ids from a dead
    /// session. Rather than special-casing that, the pass simply rewrites
    /// whatever it finds onto the live documents — a restored layout keeps its
    /// shape (where the group sits, how wide it is) and gets this session's
    /// documents in it.
    ///
    /// Returns whether the tree changed, so the caller can persist.
    fn sync_document_panes(&mut self, docs: &Documents) -> bool {
        let Some(group) = self.document_group() else {
            return false;
        };
        let wanted: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        let present: Vec<(TileId, u64)> = match self.tree.tiles.get(group) {
            Some(Tile::Container(Container::Tabs(tabs))) => tabs
                .children
                .iter()
                .filter_map(|id| match self.tree.tiles.get(*id) {
                    Some(Tile::Pane(PaneKind::Document(doc))) => Some((*id, *doc)),
                    _ => None,
                })
                .collect(),
            _ => return false,
        };

        let mut changed = false;

        // Re-key the panes we already have onto the wanted documents, in order.
        // A reloaded layout hits this path for every pane; a steady-state frame
        // hits it for none.
        for ((tile, current), want) in present.iter().zip(wanted.iter()) {
            if current != want {
                if let Some(Tile::Pane(kind)) = self.tree.tiles.get_mut(*tile) {
                    *kind = PaneKind::Document(*want);
                    changed = true;
                }
            }
        }

        // Too few panes: a document was opened. Too many: one was closed.
        for want in wanted.iter().skip(present.len()) {
            let tile = self.tree.tiles.insert_pane(PaneKind::Document(*want));
            if let Some(Tile::Container(container)) = self.tree.tiles.get_mut(group) {
                container.add_child(tile);
                changed = true;
            }
        }
        for (tile, _) in present.iter().skip(wanted.len()) {
            self.tree.remove_recursively(*tile);
            changed = true;
        }

        // Point the tab bar at the active document. Done every frame (not only
        // on change) because egui_tiles also moves `active` itself — when a tab
        // is closed, say — and the two must not drift apart.
        let active_id = docs.active_id();
        let active_tile = self.tree.tiles.iter().find_map(|(id, tile)| {
            matches!(tile, Tile::Pane(PaneKind::Document(d)) if *d == active_id).then_some(*id)
        });
        if let (Some(active_tile), Some(Tile::Container(Container::Tabs(tabs)))) =
            (active_tile, self.tree.tiles.get_mut(group))
        {
            if tabs.active != Some(active_tile) {
                tabs.set_active(active_tile);
            }
        }
        changed
    }

    /// Which document the tab bar is currently showing, if it disagrees with
    /// `Documents`. That disagreement is exactly how a TAB CLICK reaches us:
    /// egui_tiles moves its own `active` when the user clicks, and the shell
    /// then activates that document (which `sync_document_panes` will agree with
    /// on the next frame).
    /// Takes an id SNAPSHOT rather than `&Documents` because the live
    /// `Documents` is mutably borrowed by the behavior while the tree draws.
    fn tab_bar_selection(&self, active_id: u64, ids: &[u64]) -> Option<usize> {
        let group = self.document_group()?;
        let Some(Tile::Container(Container::Tabs(tabs))) = self.tree.tiles.get(group) else {
            return None;
        };
        let Some(Tile::Pane(PaneKind::Document(id))) = self.tree.tiles.get(tabs.active?) else {
            return None;
        };
        (*id != active_id).then(|| ids.iter().position(|d| d == id))?
    }

    /// Turf any pane that is not a document out of the DOCUMENT GROUP.
    ///
    /// The group's tab bar must list open models and nothing else. egui_tiles
    /// offers no hook to refuse a drop into a container — `Behavior` can say a
    /// tile is not draggable, which stops a document tab being torn OUT, but
    /// nothing stops a side pane being dropped IN. So the invariant is restored
    /// after the fact instead: a pane dropped in there is moved back to the side
    /// column on the very same frame, before anything is drawn or persisted.
    ///
    /// Returns whether it moved anything, so the caller can persist the layout —
    /// otherwise the eviction would silently repeat on every reload.
    fn evict_foreign_panes_from_document_group(&mut self) -> bool {
        let Some(group) = self.document_group() else {
            return false;
        };
        let intruders: Vec<TileId> = match self.tree.tiles.get(group) {
            Some(Tile::Container(Container::Tabs(tabs))) => tabs
                .children
                .iter()
                .copied()
                .filter(|id| {
                    !matches!(self.tree.tiles.get(*id), Some(Tile::Pane(PaneKind::Document(_))))
                })
                .collect(),
            _ => return false,
        };
        if intruders.is_empty() {
            return false;
        }

        // Detach first, then re-home. The side home is looked up AFTER
        // detaching so it cannot be skewed by the intruders it is re-homing.
        if let Some(Tile::Container(container)) = self.tree.tiles.get_mut(group) {
            for id in &intruders {
                container.remove_child(*id);
            }
        }
        let target = self.side_home().or_else(|| self.tree.root());
        match target {
            Some(target) if target != group => {
                if let Some(Tile::Container(container)) = self.tree.tiles.get_mut(target) {
                    for id in &intruders {
                        container.add_child(*id);
                    }
                    return true;
                }
                self.put_back(group, &intruders);
                false
            }
            _ => {
                self.put_back(group, &intruders);
                false
            }
        }
    }

    /// Return detached panes to `group`. Losing a pane the user can no longer
    /// reach would be a worse outcome than the layout violation being fixed.
    fn put_back(&mut self, group: TileId, panes: &[TileId]) {
        if let Some(Tile::Container(container)) = self.tree.tiles.get_mut(group) {
            for id in panes {
                container.add_child(*id);
            }
        }
    }

    /// Where side panes belong: the container holding the MOST of them — the
    /// left tab strip in the default layout, or wherever the user has gathered
    /// them — preferring a `Tabs` when two containers tie. Never the document
    /// group (3D views own that one), so a pane re-homed here cannot land back
    /// among the model tabs.
    ///
    /// Identified by CONTENT rather than by shape, so it follows the user's
    /// arrangement instead of assuming the default one.
    fn side_home(&self) -> Option<TileId> {
        let group = self.document_group();
        // `Tiles::iter` walks a HashMap, so the ranking key has to break every
        // tie by itself or the answer wanders between runs: most side panes,
        // then a `Tabs` over a split, then the OLDEST tile (ids count up, so
        // reversing the id prefers the container that was there first).
        let mut best: Option<((usize, bool, std::cmp::Reverse<u64>), TileId)> = None;
        for (id, tile) in self.tree.tiles.iter() {
            let Tile::Container(container) = tile else {
                continue;
            };
            if Some(*id) == group {
                continue;
            }
            let side_panes = container
                .children()
                .filter(|child| {
                    matches!(
                        self.tree.tiles.get(**child),
                        Some(Tile::Pane(kind)) if !matches!(kind, PaneKind::Document(_))
                    )
                })
                .count();
            if side_panes == 0 {
                continue;
            }
            let rank = (
                side_panes,
                matches!(container, Container::Tabs(_)),
                std::cmp::Reverse(id.0),
            );
            if best.is_none_or(|(current, _)| rank > current) {
                best = Some((rank, *id));
            }
        }
        best.map(|(_, id)| id)
    }

    /// Serialize the layout through the unified persistence seam (best-effort).
    /// The eCAD door's notes ([`Self::surface_ecad_pane`]) are written INTO the
    /// tree's object under [`ECAD_DECLINED_KEY`], not around it: `Tree`
    /// ignores a field it does not know, so a build from before the notes still
    /// reads this file as its layout, and this build still reads that build's.
    fn save(&self, store: &dyn ModelStore) {
        let Ok(mut json) = serde_json::to_value(&self.tree) else {
            return;
        };
        if let (Some(fields), Ok(notes)) = (json.as_object_mut(), serde_json::to_value(&self.ecad_declined)) {
            fields.insert(ECAD_DECLINED_KEY.into(), notes);
        }
        let _ = store.write(DOCK_LAYOUT_KEY, &json.to_string());
    }
}

/// The default layout: a horizontal split of `[ tabs(side panes) | viewport ]`.
///
/// Every side pane is a TAB in one full-height strip down the left, so a fresh
/// install opens on one titled pane using the whole height rather than seven
/// slivers stacked in a column. Splitting them apart is a drag away; gathering
/// them back is not obvious, which is why the tabbed arrangement is the default
/// and not the other way round.
fn default_tree() -> Tree<PaneKind> {
    let mut tiles = Tiles::default();
    let side: Vec<TileId> = PaneKind::SIDE
        .into_iter()
        .map(|k| tiles.insert_pane(k))
        .collect();
    let side_container = tiles.insert_tab_tile(side);
    // A placeholder document: ids are process-unique, so the real one is put in
    // by `sync_document_panes` on the first frame. It lives in a `Tabs`
    // container from the start — that container IS the document tab bar.
    let placeholder = tiles.insert_pane(PaneKind::Document(0));
    let viewport = tiles.insert_tab_tile(vec![placeholder]);
    let root = tiles.insert_horizontal_tile(vec![side_container, viewport]);
    // Bias the root split so the side column starts narrow (relative shares).
    if let Some(Tile::Container(Container::Linear(linear))) = tiles.get_mut(root) {
        linear.shares.set_share(side_container, 0.30);
        linear.shares.set_share(viewport, 0.70);
    }
    Tree::new("brep-dock", root, tiles)
}

/// The side panes that existed at 874912fbf (2026-09-10), when a workbench
/// stopped DELETING the panes it did not claim and began hiding them. Every
/// layout written since holds all of these; a layout from the scheme before it
/// lacks the ones its workbench did not claim.
const PANES_WHEN_HIDING_BEGAN: [PaneKind; 7] = [
    PaneKind::History,
    PaneKind::Scene,
    PaneKind::Expressions,
    PaneKind::AssemblyConstraints,
    PaneKind::Bom,
    PaneKind::WireHarness,
    PaneKind::Pmi,
];

/// A deserialized tree made to describe the CURRENT pane set, or `None` when it
/// cannot be, in which case the caller uses [`default_tree`].
///
/// It needs a root and at least one document pane: a layout with no document
/// pane has nowhere to put the 3D views and no record of where the group
/// belonged. (The document COUNT is not checked: a saved layout legitimately
/// holds as many as were open, and `sync_document_panes` renumbers them onto
/// this session's documents.)
///
/// A side pane the file lacks decides the rest, and there are two reasons a
/// file can lack one:
///
/// * **It was written before the pane existed.** Side tabs don't close, an
///   evicted pane is re-homed, and neither simplify nor GC drops a hidden tile,
///   so a layout written by the current scheme lacks exactly the panes added
///   since it was written. It holds the user's arrangement of everything else,
///   so it is KEPT, and each missing pane is added as a tab to the side panes'
///   home ([`DockState::side_home`]), where the default layout keeps it too.
///   Discarding it instead replaced a customised layout with the default the
///   first time a new pane shipped — and wrote the default back over the file
///   on the first frame, so the arrangement was gone for good. Measured with
///   the Sheets pane: a side column narrowed to 0.2 with Scene torn out into
///   its own tile came back at 0.3 with Scene re-tabbed.
/// * **It was written by the scheme before hiding** (the workbench deleted the
///   panes it did not claim), recognisable by a missing pane from
///   [`PANES_WHEN_HIDING_BEGAN`]. Such a file holds no opinion about where its
///   missing panes belong, and guessing one produces a layout nobody chose, so
///   it is DISCARDED for the documented default.
fn salvage(tree: Tree<PaneKind>) -> Option<Tree<PaneKind>> {
    tree.root()?;
    let mut documents = false;
    let mut side = std::collections::HashSet::new();
    for tile in tree.tiles.tiles() {
        match tile {
            Tile::Pane(PaneKind::Document(_)) => documents = true,
            Tile::Pane(kind) => {
                side.insert(*kind);
            }
            Tile::Container(_) => {}
        }
    }
    if !documents || PANES_WHEN_HIDING_BEGAN.iter().any(|kind| !side.contains(kind)) {
        return None;
    }
    let missing: Vec<PaneKind> = PaneKind::SIDE
        .into_iter()
        .filter(|kind| !side.contains(kind))
        .collect();
    if missing.is_empty() {
        return Some(tree);
    }
    let mut dock = DockState { tree, dirty: false, snapshot: Vec::new(), dialogs: None, ports: None, ecad_entry: None, ecad_declined: Vec::new() };
    let home = dock.side_home()?;
    for kind in missing {
        let pane = dock.tree.tiles.insert_pane(kind);
        match dock.tree.tiles.get_mut(home) {
            Some(Tile::Container(container)) => container.add_child(pane),
            _ => return None,
        }
    }
    Some(dock.tree)
}

/// The per-frame `egui_tiles::Behavior`: draws each pane by delegating to the
/// owning panel's existing `show(...)`, and collects the cross-panel requests +
/// a layout-edit flag for the shell to act on after `Tree::ui`.
struct DockBehavior<'a> {
    docs: &'a mut Documents,
    viewport: &'a mut Viewport,
    history: &'a mut HistoryPanel,
    bom: &'a mut BomPanel,
    assembly_constraints: &'a mut AssemblyConstraintsPanel,
    wire_harness: &'a mut WireHarnessPanel,
    pmi: &'a mut PmiPanel,
    sheets: &'a mut SheetsPanel,
    scene: &'a mut ScenePanel,
    expressions: &'a mut ExpressionsPanel,
    qualify: &'a mut QualifyPanel,
    family_table: &'a mut FamilyTableEditor,
    update_components: &'a mut UpdateComponents,
    model_store: &'a dyn ModelStore,
    plm: &'a mut crate::panels::plm_host::PlmHost,
    // --- outputs, drained after Tree::ui -----------------------------------
    insert_component_requested: bool,
    feature_focus: Option<String>,
    component_request: Option<ComponentActionRequest>,
    document_tabs: TabsOutcome,
    /// The tab bar's own title spacing, captured at construction from the live
    /// visuals. `on_tab_button` gets no `Ui`, and it needs this to reproduce
    /// egui_tiles' close-button geometry for the verifier's hit rect.
    tab_title_spacing: f32,
    /// This frame's ELIDED side-pane titles, by tab tile: the shortened galley
    /// the tab draws and the full title its tooltip shows. Filled per tab bar
    /// by [`fit_tab_titles`] before that bar's tabs draw; a tab absent here
    /// draws its whole title.
    tab_fit: std::collections::HashMap<TileId, (std::sync::Arc<egui::Galley>, String)>,
    layout_changed: bool,
    /// Panes whose `pane_ui` ran this frame (drawn = visible AND, if tabbed, the
    /// active tab) — feeds the `__brepDock` snapshot.
    rendered: Vec<PaneKind>,
}

/// Whether `tile_id` is a DOCUMENT tab. That single predicate is the whole rule
/// for the document group: such a tab closes (its `✕` shuts the model) and
/// cannot be dragged (tearing it out would put a 3D view outside the group).
fn is_document_tile(tiles: &Tiles<PaneKind>, tile_id: TileId) -> bool {
    matches!(tiles.get(tile_id), Some(Tile::Pane(PaneKind::Document(_))))
}

impl<'a> DockBehavior<'a> {
    fn new(ctx: DockContext<'a>) -> Self {
        DockBehavior {
            docs: ctx.docs,
            viewport: ctx.viewport,
            history: ctx.history,
            bom: ctx.bom,
            assembly_constraints: ctx.assembly_constraints,
            wire_harness: ctx.wire_harness,
            pmi: ctx.pmi,
            sheets: ctx.sheets,
            scene: ctx.scene,
            expressions: ctx.expressions,
            qualify: ctx.qualify,
            family_table: ctx.family_table,
            update_components: ctx.update_components,
            model_store: ctx.model_store,
            plm: ctx.plm,
            insert_component_requested: false,
            feature_focus: None,
            component_request: None,
            document_tabs: TabsOutcome::default(),
            tab_title_spacing: 0.0,
            tab_fit: std::collections::HashMap::new(),
            layout_changed: false,
            rendered: Vec::new(),
        }
    }

    /// Clear the hit rects of every side pane not in `rendered`. The match is
    /// exhaustive on purpose: a new pane that publishes rects must say here
    /// whether it has any to retract.
    fn retract_undrawn(&mut self, rendered: &[PaneKind]) {
        for kind in PaneKind::SIDE {
            if rendered.contains(&kind) {
                continue;
            }
            match kind {
                PaneKind::History => self.history.clear_hits(),
                PaneKind::Bom => self.bom.clear_hits(),
                PaneKind::AssemblyConstraints => self.assembly_constraints.clear_hits(),
                PaneKind::WireHarness => self.wire_harness.clear_hits(),
                PaneKind::Pmi => self.pmi.clear_hits(),
                PaneKind::Sheets => self.sheets.clear_hits(),
                PaneKind::Scene => self.scene.clear_hits(),
                PaneKind::Expressions => self.expressions.clear_hits(),
                PaneKind::Qualify => self.qualify.clear_hits(),
                PaneKind::FamilyTable => self.family_table.clear_hits(),
                PaneKind::Plm => {}
                // The eCAD parts pane's rects (`parts:`) are cleared every frame
                // by `ecad_parts::take_add_part_request`, so an undrawn one
                // already publishes none; the inspector publishes none, and the
                // editor's (`ecad/`) belong to the document pane.
                PaneKind::EcadLibrary | PaneKind::EcadInspector | PaneKind::Document(_) => {}
            }
        }
    }
}

impl<'a> Behavior<PaneKind> for DockBehavior<'a> {
    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: TileId,
        pane: &mut PaneKind,
    ) -> UiResponse {
        self.rendered.push(*pane);
        // Paint the side-pane background with the SAME fill the old
        // `Panel::left("brep-controls")` used (`visuals().panel_fill`), so the
        // docked panels read exactly like the previous side panel — not the
        // egui_tiles default (which leaves the pane transparent over the darker
        // central fill). The viewport paints its own 3D, so skip it.
        if !matches!(*pane, PaneKind::Document(_)) {
            let visuals = ui.visuals();
            ui.painter()
                .rect_filled(ui.max_rect(), 0.0, visuals.panel_fill);
        }
        match *pane {
            // Only the ACTIVE document's pane is ever drawn — egui_tiles shows
            // one tab at a time, and the tab bar was pointed at the active
            // document by `sync_document_panes` before this ran. The 3D body
            // keeps its own click/drag (camera orbit + picking), so we NEVER
            // report a pane drag here.
            PaneKind::Document(_) => {
                // An eCAD workbench draws its editor here instead of the 3D
                // view; the editor lives on the document, beside its engine.
                let doc = self.docs.active_mut();
                match workbench::ecad::Target::of_workbench(&doc.engine.settings.workbench) {
                    Some(target) => self.viewport.show_ecad(ui, doc, target),
                    None => self.viewport.show(ui, &mut doc.engine),
                }
            }
            PaneKind::EcadLibrary => {
                let doc = self.docs.active_mut();
                match workbench::ecad::Target::of_workbench(&doc.engine.settings.workbench) {
                    Some(workbench::ecad::Target::Diagram) => doc.ecad.diagram.library_panel(ui),
                    Some(workbench::ecad::Target::Pcb) => doc.ecad.pcb.library_panel(ui),
                    // Claimed by Diagram and PCB only, so never drawn elsewhere.
                    _ => {}
                }
            }
            PaneKind::EcadInspector => {
                let doc = self.docs.active_mut();
                match workbench::ecad::Target::of_workbench(&doc.engine.settings.workbench) {
                    Some(workbench::ecad::Target::Diagram) => doc.ecad.diagram.inspector(ui),
                    Some(workbench::ecad::Target::Pcb) => doc.ecad.pcb.inspector(ui),
                    Some(workbench::ecad::Target::Symbol) => doc.ecad.symbol.properties(ui),
                    Some(workbench::ecad::Target::Pads) => doc.ecad.pads.properties(ui),
                    None => {}
                }
            }
            PaneKind::History => scroll(ui, "dock-history", |ui| {
                self.history.show(ui, self.docs.engine_mut());
                // The ACOMP palette pick opens the COMPONENT SELECTOR, not a bare
                // feature dialog — bubbled to the shell (the file dialog is shell-owned).
                self.insert_component_requested |= self.history.take_insert_component_request();
            }),
            PaneKind::Bom => {
                // VERTICAL only, even though a BOM is as wide as its configured
                // columns: the column tree owns its own HORIZONTAL scrolling,
                // because a scroll area out here would carry the frozen columns
                // away with everything else. (The shared `scroll` helper is
                // vertical-only too, but the BOM wants `auto_shrink` off.)
                egui::ScrollArea::vertical()
                    .id_salt("dock-bom")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        let document = self.docs.active().name().map(str::to_string);
                        let outcome = self.bom.show(
                            ui,
                            self.docs.engine_mut(),
                            self.model_store,
                            self.update_components,
                            document.as_deref(),
                        );
                        if outcome.focus.is_some() {
                            // The BOM's Edit action: roll to the owning feature
                            // and open it in the history panel.
                            self.feature_focus = outcome.focus;
                        }
                        if outcome.component.is_some() {
                            self.component_request = outcome.component;
                        }
                        if outcome.update_components {
                            if let Err(error) = self.update_components.run(self.docs.engine_mut(), self.model_store) {
                                self.docs.engine_mut().push_notice(format!("Update components: {error}"));
                            }
                        }
                    });
            }
            PaneKind::AssemblyConstraints => scroll(ui, "dock-constraints", |ui| {
                self.assembly_constraints.show(
                    ui,
                    self.docs.engine_mut(),
                    self.model_store,
                    self.update_components,
                );
            }),
            PaneKind::WireHarness => {
                // Vertical only, like the BOM: the column tree owns its own
                // horizontal scrolling (a frozen column must not scroll away).
                egui::ScrollArea::vertical()
                    .id_salt("dock-wire-harness")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        self.wire_harness.show(ui, self.docs.engine_mut());
                    });
            }
            PaneKind::Pmi => scroll(ui, "dock-pmi", |ui| {
                self.pmi.show(ui, self.docs.engine_mut());
            }),
            PaneKind::Sheets => {
                // Vertical only, like the BOM and the harness: the column tree
                // owns its own horizontal scrolling.
                egui::ScrollArea::vertical()
                    .id_salt("dock-sheets")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        self.sheets.show(ui, self.docs.engine_mut());
                    });
            }
            PaneKind::Scene => scroll(ui, "dock-scene", |ui| {
                self.scene.show(ui, self.docs.engine_mut());
            }),
            // The panel has no ScrollArea of its own: a long variable sheet or
            // variable list overflowed the pane with no way to reach the rest.
            PaneKind::Expressions => scroll(ui, "dock-expressions", |ui| {
                self.expressions.show(ui, self.docs.engine_mut());
            }),
            // ONE panel on THREE surfaces: it draws the same connection points
            // whatever the central tile shows, and only its jump buttons care
            // which that is — so, unlike the eCAD panes, the target is passed
            // IN rather than dispatched on.
            PaneKind::Qualify => scroll(ui, "dock-qualify", |ui| {
                let doc = self.docs.active_mut();
                let target = workbench::ecad::Target::of_workbench(&doc.engine.settings.workbench);
                self.qualify.show(ui, &mut doc.engine, &mut doc.ecad, target);
            }),
            // The table scrolls itself (both ways, a wide family is wide), so
            // no outer wrap. Generate writes beside the family's own file.
            PaneKind::FamilyTable => {
                let doc = self.docs.active_mut();
                let identity = doc.name().map(str::to_string);
                let dirty = doc.dirty_marker();
                self.family_table.show(ui, &mut doc.engine, self.model_store, identity.as_deref(), dirty);
                // Save and generate (a PLM family with unsaved edits): save it
                // here, where the document is; Generate starts once it landed.
                if self.family_table.take_save_request() {
                    let document = doc.engine.history_request_json();
                    if let Some(name) = identity.as_deref() {
                        crate::plm::thumbnail::stage_save(self.model_store, name, &document, &doc.engine, doc.id());
                    }
                    match identity.as_deref().map(|name| self.model_store.write(name, &document)) {
                        Some(Ok(())) => doc.mark_clean(),
                        Some(Err(error)) => self.family_table.save_failed(error),
                        None => self.family_table.save_failed("the family has never been saved".into()),
                    }
                }
            }
            PaneKind::Plm => scroll(ui, "dock-plm", |ui| self.plm.ui(ui, self.docs)),
        }
        UiResponse::None
    }

    /// A document tab is titled by its FILE, with a bullet while it has unsaved
    /// changes — the tab bar is the only place that state is visible now that
    /// several models are open at once. Every other pane keeps its fixed title.
    fn tab_title_for_pane(&mut self, pane: &PaneKind) -> egui::WidgetText {
        match pane {
            PaneKind::Document(id) => match self.docs.iter().find(|d| d.id() == *id) {
                Some(doc) => {
                    let title = doc.title();
                    // U+2022, not U+25CF: the icon font draws the latter as a
                    // hollow ring, which reads as a status light rather than
                    // "unsaved".
                    if doc.dirty_marker() {
                        format!("{title} \u{2022}").into()
                    } else {
                        title.into()
                    }
                }
                // A pane whose document is gone is about to be removed by
                // `sync_document_panes`; it must not panic in the meantime.
                None => pane.title().into(),
            },
            _ => pane.title().into(),
        }
    }

    /// A tab ELIDED to fit its bar draws the shortened galley
    /// ([`Behavior::top_bar_right_ui`] decided it); every other tab its title.
    fn tab_title_for_tile(&mut self, tiles: &Tiles<PaneKind>, tile_id: TileId) -> egui::WidgetText {
        if let Some((galley, _)) = self.tab_fit.get(&tile_id) {
            return egui::WidgetText::Galley(galley.clone());
        }
        match tiles.get(tile_id) {
            Some(Tile::Pane(pane)) => self.tab_title_for_pane(pane),
            Some(Tile::Container(container)) => format!("{:?}", container.kind()).into(),
            None => "MISSING TILE".into(),
        }
    }

    /// Called for each tab bar BEFORE its tabs draw, with the bar's whole
    /// width available — the one moment the width and the titles are both in
    /// hand, so this is where a crowded bar's titles are fitted to it.
    fn top_bar_right_ui(
        &mut self,
        tiles: &Tiles<PaneKind>,
        ui: &mut egui::Ui,
        _tile_id: TileId,
        tabs: &egui_tiles::Tabs,
        _scroll_offset: &mut f32,
    ) {
        let spacing = self.tab_title_spacing;
        let titles: Vec<(TileId, String, bool)> = tabs
            .children
            .iter()
            .filter(|id| tiles.is_visible(**id))
            .filter_map(|id| match tiles.get(*id) {
                Some(Tile::Pane(pane)) => {
                    let text = self.tab_title_for_pane(pane).text().to_string();
                    Some((*id, text, is_document_tile(tiles, *id)))
                }
                _ => None,
            })
            .collect();
        for (id, full, galley) in fit_tab_titles(ui, &titles, ui.available_width(), spacing, self.close_button_outer_size()) {
            self.tab_fit.insert(id, (galley, full));
        }
    }

    /// Only a DOCUMENT tab closes — that is the model's `✕`. Side panels have no
    /// re-open affordance, so they are shown/hidden by workbench and rearranged
    /// by drag, never destroyed.
    fn is_tab_closable(&self, tiles: &Tiles<PaneKind>, tile_id: TileId) -> bool {
        is_document_tile(tiles, tile_id)
    }

    /// A `✕` click. We REFUSE the removal (`false`) and hand the request to the
    /// shell instead: closing a model has to run the unsaved-changes prompt and
    /// drop the document, and only then does its pane go — removed by
    /// `sync_document_panes`. Letting egui_tiles delete the tile here would
    /// close the tab while leaving the document open.
    fn on_tab_close(&mut self, tiles: &mut Tiles<PaneKind>, tile_id: TileId) -> bool {
        if let Some(Tile::Pane(PaneKind::Document(id))) = tiles.get(tile_id) {
            if let Some(index) = self.docs.iter().position(|d| d.id() == *id) {
                self.document_tabs.close = Some(index);
            }
        }
        false
    }

    /// Publish each document tab's screen rect for the headed verifier, keyed
    /// `doctab:<index>` exactly as the old hand-drawn strip did, so the existing
    /// browser checks drive the real tab bar unchanged. This is the hook the
    /// default tab renderer offers for precisely this — it hands back the tab
    /// button's own `Response`, so we keep egui_tiles' native tab drawing.
    fn on_tab_button(
        &mut self,
        tiles: &mut Tiles<PaneKind>,
        tile_id: TileId,
        button_response: egui::Response,
    ) -> egui::Response {
        if let Some(Tile::Pane(PaneKind::Document(id))) = tiles.get(tile_id) {
            if let Some(index) = self.docs.iter().position(|d| d.id() == *id) {
                let tab = button_response.rect;
                // The `✕` is not routed through this hook (egui_tiles calls it
                // once per tab, with the whole tab), so its rect is DERIVED the
                // same way the default tab renderer lays it out: a
                // `close_button_outer_size` square, right-centered in the tab
                // inset by the title spacing. Both inputs come from this same
                // `Behavior`, so an override moves the published rect with it.
                let close = egui::Align2::RIGHT_CENTER.align_size_within_rect(
                    egui::Vec2::splat(self.close_button_outer_size()),
                    tab.shrink(self.tab_title_spacing),
                );
                self.document_tabs.hits.push((format!("doctab:{index}"), tab));
                self.document_tabs
                    .hits
                    .push((format!("doctab:{index}:close"), close));
            }
        }
        // A side pane's tab, keyed by its title (`pane:assembly-parts`), so a
        // script can see whether a crowded strip still shows it whole.
        if let Some(Tile::Pane(pane)) = tiles.get(tile_id) {
            if !matches!(pane, PaneKind::Document(_)) {
                let key = format!("pane:{}", pane.title().to_lowercase().replace(' ', "-"));
                self.document_tabs.hits.push((key, button_response.rect));
            }
        }
        match self.tab_fit.get(&tile_id) {
            Some((_, full)) => button_response.on_hover_text(full.as_str()),
            None => button_response,
        }
    }

    /// A document tab can't be picked up: dragging one out would tear a 3D view
    /// into its own container somewhere else in the tree, which is the mirror
    /// image of the violation `evict_foreign_panes_from_document_group` guards —
    /// the group must be the ONLY home for 3D views, as well as holding nothing
    /// but them. Side panes drag freely to re-dock.
    fn is_tile_draggable(&self, tiles: &Tiles<PaneKind>, tile_id: TileId) -> bool {
        !is_document_tile(tiles, tile_id)
    }

    fn on_edit(&mut self, _edit_action: EditAction) {
        self.layout_changed = true;
    }

    /// Every pane gets its own tab bar — that tab is the label AND the drag
    /// handle, so a default vertical stack still reads as titled, re-dockable
    /// panes (egui_tiles gives bare linear panes neither). Other simplifications
    /// stay at their defaults so the tree tidies itself after a drag / a
    /// workbench-membership removal.
    fn simplification_options(&self) -> egui_tiles::SimplificationOptions {
        egui_tiles::SimplificationOptions {
            all_panes_must_have_tabs: true,
            ..Default::default()
        }
    }

    /// Match the tab strip to the old side panel's fill (`panel_fill`) so a docked
    /// panel reads as one continuous surface (tab bar + body) in the SAME colour
    /// the previous `Panel::left` used — not the egui_tiles default strip colour.
    fn tab_bar_color(&self, visuals: &egui::Visuals) -> egui::Color32 {
        visuals.panel_fill
    }

    /// Give EVERY tab a visible chip so it reads as a tab — egui_tiles' default
    /// leaves inactive tabs fully transparent (they vanish into the strip). Reuse
    /// egui's standard widget fills: the active tab uses the "active" fill (it
    /// stands out as selected), inactive tabs the "inactive" resting fill (a muted
    /// but clearly-there chip). No hand-picked colours — same DRY rule as the rest.
    fn tab_bg_color(
        &self,
        visuals: &egui::Visuals,
        _tiles: &Tiles<PaneKind>,
        _tile_id: TileId,
        state: &TabState,
    ) -> egui::Color32 {
        if state.active {
            visuals.widgets.active.bg_fill
        } else {
            visuals.widgets.inactive.bg_fill
        }
    }
}

/// The narrowest a side-pane title is elided to, in points: about four
/// characters and the ellipsis, so an elided tab still says which pane it is.
/// A bar too narrow even for that scrolls, as it always did.
const MIN_TAB_TITLE: f32 = 40.0;

/// Fit a tab bar's titles to `width`. When every title fits whole, nothing
/// changes. Otherwise the LONGEST side-pane titles are elided (`…`) to one
/// shared cap, the widest that makes the bar fit — so short titles stay whole
/// and a crowded bar gives up characters from its longest names first, never
/// clipping the last tab mid-word behind a scroll arrow (the PCB dock's five
/// tabs at 420 pt read `Insp`, round five's audit, item 15). Document tabs
/// (their `✕`, their file names) are left alone. Returns each elided tab's
/// id, full title and the galley to draw.
fn fit_tab_titles(
    ui: &egui::Ui,
    titles: &[(TileId, String, bool)],
    width: f32,
    spacing: f32,
    close: f32,
) -> Vec<(TileId, String, std::sync::Arc<egui::Galley>)> {
    let natural = |text: &str| {
        egui::WidgetText::from(text)
            .into_galley(ui, Some(egui::TextWrapMode::Extend), f32::INFINITY, egui::TextStyle::Button)
            .size()
            .x
    };
    // What the bar spends besides the elidable titles: every tab's padding,
    // and the whole of each document tab.
    let mut fixed = 0.0;
    let mut elidable: Vec<(TileId, &str, f32)> = Vec::new();
    for (id, text, document) in titles {
        fixed += 2.0 * spacing;
        let w = natural(text);
        if *document {
            fixed += w + 4.0 + close;
        } else {
            elidable.push((*id, text, w));
        }
    }
    let budget = width - fixed - 2.0;
    let Some(cap) = shared_cap(&elidable.iter().map(|e| e.2).collect::<Vec<_>>(), budget) else {
        return Vec::new();
    };
    let cap = cap.max(MIN_TAB_TITLE);
    elidable
        .into_iter()
        .filter(|(_, _, w)| *w > cap + 0.5)
        .map(|(id, text, _)| {
            let galley = egui::WidgetText::from(text).into_galley(
                ui,
                Some(egui::TextWrapMode::Truncate),
                cap,
                egui::TextStyle::Button,
            );
            (id, text.to_string(), galley)
        })
        .collect()
}

/// The largest `cap` with `Σ min(width, cap) ≤ budget` — water-filling — or
/// `None` when the widths already fit whole.
fn shared_cap(widths: &[f32], budget: f32) -> Option<f32> {
    if widths.iter().sum::<f32>() <= budget {
        return None;
    }
    let mut sorted = widths.to_vec();
    sorted.sort_by(f32::total_cmp);
    let mut spent = 0.0;
    for (i, w) in sorted.iter().enumerate() {
        let left = sorted.len() - i;
        let cap = (budget - spent) / left as f32;
        if cap < *w {
            return Some(cap.max(0.0));
        }
        spent += w;
    }
    None
}

/// Wrap a pane body in its own vertical scroll area (unique id per pane so egui
/// never conflates their scroll state). Panels that self-scroll skip this.
fn scroll(ui: &mut egui::Ui, salt: &str, add: impl FnOnce(&mut egui::Ui)) {
    egui::ScrollArea::vertical()
        .id_salt(salt)
        .auto_shrink([false, false])
        .show(ui, add);
}
