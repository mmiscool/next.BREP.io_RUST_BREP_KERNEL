//! The action registry: a published widget key acted on by its egui id, not
//! by a simulated pointer.
//!
//! Every widget that senses a click gets an AccessKit node, whose bounds are
//! the widget's rect and whose id is the widget's egui id. While a host is
//! attached (`registry::enabled`), the app keeps egui's AccessKit tree on and
//! [`ActionTree`] (an egui plugin) records, at the end of every pass, each
//! clickable node's bounds and id. A panel publishes the same rect under its
//! key (`hit_rects`), so `activate_widget` resolves key → rect → node and asks
//! egui to click that node (`Event::AccessKitActionRequest`, the path a screen
//! reader takes). The widget then reports `clicked()` in the next pass, as for
//! a pointer click, with no pointer moved: nothing covering the widget, no
//! scroll position and no multi-click history can send the click elsewhere.
use std::sync::Mutex;

/// One clickable AccessKit node of the last pass: bounds (egui points,
/// `[x, y, w, h]`) and id.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    pub rect: [f32; 4],
    pub id: egui::accesskit::NodeId,
}

/// The last pass's clickable nodes, newest pass only.
static NODES: Mutex<Vec<Node>> = Mutex::new(Vec::new());

fn lock() -> std::sync::MutexGuard<'static, Vec<Node>> {
    NODES.lock().unwrap_or_else(|p| p.into_inner())
}

/// The egui plugin that records the clickable nodes of each pass's AccessKit
/// update. Installed once per context ([`install`]); it reads the update and
/// leaves it in place for anyone else (egui_kittest reads it too).
pub struct ActionTree;

impl egui::Plugin for ActionTree {
    fn debug_name(&self) -> &'static str {
        "brep-action-tree"
    }

    fn output_hook(&mut self, _ctx: &egui::Context, output: &mut egui::FullOutput) {
        // A pass that ran no `ui` built no widgets: keep the last pass's.
        if !DRAWS.load(std::sync::atomic::Ordering::Relaxed) {
            return;
        }
        let Some(update) = &output.platform_output.accesskit_update else { return };
        let nodes = update
            .nodes
            .iter()
            .filter(|(_, node)| node.supports_action(egui::accesskit::Action::Click) && !node.is_disabled())
            .filter_map(|(id, node)| {
                let b = node.bounds()?;
                Some(Node { rect: [b.x0 as f32, b.y0 as f32, (b.x1 - b.x0) as f32, (b.y1 - b.y0) as f32], id: *id })
            })
            .collect();
        *lock() = nodes;
    }
}

/// Install [`ActionTree`] on `ctx`, once.
pub fn install(ctx: &egui::Context) {
    ctx.add_plugin(ActionTree);
}

/// Whether the pass now running draws its `ui` ([`note_pass`]).
static DRAWS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// At the start of every pass (`raw_input_hook`): keep egui building its
/// AccessKit tree while a host is attached (with no host the tree costs
/// nothing), and say whether this pass draws its `ui` (`draws`). eframe runs
/// a pass without `ui` when the window is not visible; its few nodes are not
/// the widgets, so the map keeps the last pass's that drew. Returns whether
/// the PREVIOUS pass drew: egui hit-tests a pass's pointer input against the
/// previous pass's widgets, so after one that did not, input waits a pass.
pub fn note_pass(ctx: &egui::Context, draws: bool) -> bool {
    let drew = DRAWS.swap(draws, std::sync::atomic::Ordering::Relaxed);
    if crate::automation::registry::enabled() {
        ctx.enable_accesskit();
    }
    drew
}

/// How a key's rect was matched to a node.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Matched {
    /// A node's bounds are the key's rect.
    Exact,
    /// The smallest clickable node that contains the rect's centre (a key
    /// published with a padded or partial rect).
    Contains,
}

/// The node `rect` (`[x, y, w, h]`) belongs to, among `nodes`.
pub fn resolve(nodes: &[Node], rect: [f32; 4]) -> Option<(Node, Matched)> {
    const EPS: f32 = 0.01;
    if let Some(node) = nodes.iter().find(|n| n.rect.iter().zip(rect).all(|(a, b)| (a - b).abs() <= EPS)) {
        return Some((*node, Matched::Exact));
    }
    let (cx, cy) = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0);
    nodes
        .iter()
        .filter(|n| cx >= n.rect[0] && cy >= n.rect[1] && cx <= n.rect[0] + n.rect[2] && cy <= n.rect[1] + n.rect[3])
        .min_by(|a, b| (a.rect[2] * a.rect[3]).total_cmp(&(b.rect[2] * b.rect[3])))
        .map(|n| (*n, Matched::Contains))
}

/// The last pass's clickable nodes.
pub fn nodes() -> Vec<Node> {
    lock().clone()
}

