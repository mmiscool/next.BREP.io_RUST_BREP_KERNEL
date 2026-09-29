use super::*;

// ===========================================================================
// Construction PLANES as ordinary pick candidates.
//
// A construction plane / DATUM base plane is drawn by the datum WIDGET, not by
// the scene, so `pick::pick` — which walks the render scene's solids — can never
// see one. Before this module a plane was reachable only through the independent
// `datum_pick` widget path, and ONLY when the geometry pick missed: any face,
// edge or vertex under the pointer won outright and the plane was unreachable
// (the reported "I can't select a plane if there is any other geometry under the
// pointer"). Here the plane cards are turned into ordinary
// [`pick::PickCandidate`]s and merged into the SAME ranked list every selection
// path consumes, so a plane competes for a pick like any other entity and shows
// up in the pick-list popup like any face.
//
// HOW A PLANE COMPETES — by KIND, not by depth. The candidate list is sorted
// category-major in the pick-list order
//
//     VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT
//
// (the `PickKind` discriminant order), nearest-first within a category. So a
// plane ranks straight after faces: any face under the cursor still out-priorities
// it, and the plane cannot swallow a click meant for the model — while it is
// nonetheless IN the list, one row down, reachable through anything. Pure depth
// ordering was rejected precisely because it trades one complaint for its mirror
// image: a plane card in front of the model would win every click near it, and a
// plane behind the model would still lose at a grazing angle.
//
// WHAT COUNTS AS "ON" A PLANE — the drawn rectangle. A construction plane is
// mathematically infinite, but only the card the renderer draws is pickable
// ([`crate::widgets::WidgetRegistry::datum_plane_hits`] → `DatumPlane::hit_point`,
// the same live-camera `half()` extent `build_main_overlay` draws with — never a
// bake, so the pickable region tracks the drawn one across a zoom). Either face
// of the card picks: a plane seen from behind is still a legitimate selection.
//
// DATUMS RIDE ALONG. `refresh_construction_datums` feeds D (datum) and P (plane)
// features through the one `{planes:[…]}` channel — a DATUM contributes its three
// base planes (`{id}:XY|XZ|YZ`), a PLANE its single frame (`{id}`) — so both are
// `PickKind::Plane` candidates with no extra machinery. Datum AXES are not fed by
// that path and stay out of the pick list.
// ===========================================================================

impl EngineState {
    /// EVERY pick candidate under CSS-pixel `(x, y)`: the scene's
    /// vertices/edges/faces/solids ([`pick::pick`]) PLUS the construction PLANE
    /// cards the pointer ray crosses, ranked together category-major
    /// (VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT), nearest first within a
    /// category.
    ///
    /// This is the RAW list (no selection filter); the filter-honoring callers are
    /// [`Self::candidates_filtered_at`] (the pick-list popup) and
    /// [`Self::pick_top_at`] (every single-hit pick).
    pub fn pick_candidates_at(&self, x: f64, y: f64) -> Vec<pick::PickCandidate> {
        let mut out = pick::pick(&self.scene, &self.camera, x, y, &self.pick_options());
        // Appended AFTER `pick`'s own `MAX_CANDIDATES` truncation, so a plane is
        // never dropped by a crowd of faces under the same pixel.
        out.extend(self.plane_candidates_at(x, y));
        sort_pick_candidates(&mut out);
        out
    }

    /// The construction PLANE candidates under CSS-pixel `(x, y)` — one per DRAWN
    /// datum-plane card the pointer ray crosses, carrying the datum FRAME name
    /// (`Pl`, `Datum:XY`) and the world hit point. `solid` is empty (a plane has no
    /// owning solid) and `screen_dist` is 0 (a card hit is exact, like a face hit).
    ///
    /// Hidden planes are absent by construction: the widget feed
    /// ([`Self::refresh_construction_datums`]) only carries the planes that are
    /// actually drawn, so a Scene-tree-hidden or rolled-back plane cannot be picked.
    pub(super) fn plane_candidates_at(&self, x: f64, y: f64) -> Vec<pick::PickCandidate> {
        let cam = gizmo_camera(&self.camera);
        let (_, _, forward) = self.camera.basis();
        self.widgets
            .datum_plane_hits(&cam, x as f32, y as f32)
            .into_iter()
            .map(|(name, point)| {
                let position = [point[0] as f64, point[1] as f64, point[2] as f64];
                pick::PickCandidate {
                    kind: pick::PickKind::Plane,
                    name,
                    solid: String::new(),
                    depth: crate::view::dot3(
                        crate::view::sub3(position, self.camera.eye),
                        forward,
                    ),
                    screen_dist: 0.0,
                    position,
                }
            })
            .collect()
    }

    /// The TOP-ranked candidate under `(x, y)` whose kind `kinds` admits — the
    /// planes-aware replacement for [`pick::pick_filtered`] (which only sees the
    /// render scene). An EMPTY `kinds` means any kind, matching `pick_filtered`.
    ///
    /// `DATUM` is accepted as an alias of `PLANE` so a reference field whose schema
    /// spells its construction kind `["DATUM"]` picks the same plane cards as
    /// `["PLANE"]` (`ref_select_click` passes the field's RAW strings here).
    pub(super) fn pick_top_at(
        &self,
        x: f64,
        y: f64,
        kinds: &[String],
    ) -> Option<pick::PickCandidate> {
        self.pick_candidates_at(x, y)
            .into_iter()
            // EMPTY = any kind, [`pick::pick_filtered`]'s rule (the callers that
            // mean "select nothing" guard on an empty list BEFORE calling).
            .find(|c| kinds.is_empty() || self.candidate_admitted(kinds, c))
    }

    /// Does `kinds` admit this CANDIDATE? [`kind_admitted`] plus the one
    /// distinction the picker cannot make on its own: a committed SKETCH is drawn
    /// as a synthesized sheet solid, so it and a real body arrive with the same
    /// [`pick::PickKind::Solid`]. `SolidDisplay::is_sketch` splits them, and the
    /// candidate then answers to `"SKETCH"` or to `"SOLID"` — never to both.
    ///
    /// Only the whole-object candidate is reclassified. A sketch's planar FACE and
    /// its drawn EDGES keep their own kinds, which is what the profile fields
    /// (`["SKETCH","FACE"]`, picking a sketch through its sheet face) and the path
    /// fields (`["EDGE"]` on path sweep, picking a sketch curve) rely on.
    pub(super) fn candidate_admitted(
        &self,
        kinds: &[String],
        candidate: &pick::PickCandidate,
    ) -> bool {
        if candidate.kind == pick::PickKind::Solid && self.candidate_is_sketch(candidate) {
            return kinds.iter().any(|k| k.eq_ignore_ascii_case("SKETCH"));
        }
        kind_admitted(kinds, candidate.kind)
    }

    /// The kind LABEL a candidate should present under — `PickKind::as_str`,
    /// except that a committed-sketch sheet reads `"SKETCH"` rather than the
    /// `"SOLID"` its `PickKind` carries. The pick-list popup and the published
    /// candidate JSON both use it, so what a row calls itself matches the filter
    /// lane that admitted it.
    pub fn candidate_kind_label(&self, candidate: &pick::PickCandidate) -> &'static str {
        if candidate.kind == pick::PickKind::Solid && self.candidate_is_sketch(candidate) {
            return "SKETCH";
        }
        candidate.kind.as_str()
    }

    /// Whether a candidate's owning scene solid is a committed-sketch SHEET.
    pub(super) fn candidate_is_sketch(&self, candidate: &pick::PickCandidate) -> bool {
        let name = if candidate.solid.is_empty() {
            candidate.name.as_str()
        } else {
            candidate.solid.as_str()
        };
        self.scene
            .solid(name)
            .is_some_and(|solid| solid.is_sketch)
    }
}

/// Does `kinds` LIST `kind`? Case-insensitive, with `DATUM` as an alias of
/// `PLANE`. Kind-only, so it cannot tell a committed-sketch sheet from a real
/// body — [`EngineState::candidate_admitted`] is the candidate-aware wrapper every
/// picking path actually calls. STRICT: an empty list admits nothing (the "empty = any kind"
/// convenience belongs to the single-hit [`EngineState::pick_top_at`] alone — the
/// pick-LIST builder passes an empty list to mean "no sub-entity kind is
/// admitted", e.g. a COMPONENT-only filter).
pub(super) fn kind_admitted(kinds: &[String], kind: pick::PickKind) -> bool {
    let name = kind.as_str();
    let datum_alias = kind == pick::PickKind::Plane;
    kinds
        .iter()
        .any(|k| k.eq_ignore_ascii_case(name) || (datum_alias && k.eq_ignore_ascii_case("DATUM")))
}

/// The ONE candidate ordering every pick path shares: category-major in the
/// pick-list order (`PickKind`'s discriminant order IS that order —
/// VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT), nearest (smallest depth)
/// first within a category, then by screen distance. Stable, so equal-depth
/// entries keep the ray ranker's order.
pub(super) fn sort_pick_candidates(candidates: &mut [pick::PickCandidate]) {
    candidates.sort_by(|a, b| {
        (a.kind as u8)
            .cmp(&(b.kind as u8))
            .then(a.depth.total_cmp(&b.depth))
            .then(a.screen_dist.total_cmp(&b.screen_dist))
    });
}

/// The ONE candidate a hover highlight lights and a plain click takes: the
/// filter-admitted entry FIRST IN THE RAYCAST — nearest the eye — with the
/// shared kind order ([`sort_pick_candidates`]) breaking ties *inside the
/// nearest depth layer*. `candidates` must already be in that shared order
/// (every builder sorts before returning); `band` is the depth window, in world
/// units, that counts as "the same layer".
///
/// Why a LAYER and not the raw minimum depth: an EDGE lies ON the face it bounds
/// and a VERTEX on the edge, so a strict depth comparison between them is a coin
/// toss decided by tessellation noise — edges and vertices would stop being
/// clickable wherever a face happened to round nearer. Inside one layer the
/// established kind order still holds, so small targets keep their priority on
/// the surface the user is actually looking at. What the rule DOES demote is
/// everything BEHIND that surface — the far face of a box, a back edge whose
/// screen projection falls inside the 6 px threshold, a plane card under the
/// model — none of which may outrank the visible surface by kind alone any more.
/// Reaching those is what the pick LIST is for, and the list is unchanged: it
/// still ranks category-major over every candidate.
pub(super) fn front_layer_winner(
    candidates: &[pick::PickCandidate],
    band: f64,
) -> Option<&pick::PickCandidate> {
    let nearest = candidates
        .iter()
        .map(|c| c.depth)
        .fold(f64::INFINITY, f64::min);
    if !nearest.is_finite() {
        // No usable depth (empty list, or all-NaN): fall back to the shared
        // order's own winner rather than inventing one.
        return candidates.first();
    }
    let limit = nearest + band.max(0.0);
    candidates
        .iter()
        .find(|c| c.depth <= limit)
        .or_else(|| candidates.first())
}

