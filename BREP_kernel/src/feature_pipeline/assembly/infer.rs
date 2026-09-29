//! AUTOMATIC constraint inference — "the parts are already where they belong,
//! write down why".
//!
//! An imported STEP assembly arrives fully posed and completely unconstrained:
//! every component sits at its authored place, and nothing holds it there. This
//! module reads that placement back as constraint intent — a shaft coaxial with
//! a bore is a Concentric, two faces touching face-to-face are a Touch Align,
//! two coincident corners are a Coincident — so the assembly becomes editable
//! without the user re-picking every mate by hand.
//!
//! # The rule table is the surface
//!
//! [`INFERENCE_RULES`] is the ONE list of what can be inferred. The dialog, the
//! MCP command's documentation and the report's per-type counts all render from
//! it joined to the constraint catalogue ([`super::constraints`]) — a type's
//! icon and label are never re-spelled here, and adding a rule row is what adds
//! a checkbox. A `type_id` with no catalogue entry is a compile-time-invisible
//! but test-caught mistake ([`tests`]).
//!
//! # What is inferred, and from what
//!
//! Everything is measured on the WORLD geometry of the live scene — the same
//! resident, posed solids the constraints themselves resolve against
//! ([`super::mapping`]), so an inferred constraint is satisfied at the pose it
//! was inferred from and the solve that follows moves nothing.
//!
//! * CONCENTRIC — two axis-bearing faces (cylinder / cone / torus / general
//!   revolution) whose carrier axes are the SAME INFINITE LINE. Neither radii
//!   nor axial separation are compared: a shaft in a clearance hole and a bolt
//!   passing through two plates a hand's width apart are equally concentric,
//!   and a shared centreline is the whole of the evidence. Because a coaxial
//!   pair need not be anywhere near each other, the box prefilter below gates
//!   only the CONTACT rules — this one is asked about every pair.
//! * TOUCH ALIGN — two planar faces that are coplanar, FACE EACH OTHER (outward
//!   normals opposed — two flush-but-same-facing faces are not a contact), and
//!   whose boundaries overlap in the shared plane.
//! * COINCIDENT — two spherical faces sharing a centre (a ball joint), or two
//!   coincident vertices on a pair that already touches. The vertex lane is
//!   gated on contact deliberately: corners land on the same world point by
//!   accident often enough that ungated it would mate parts that never meet.
//!
//! # Why the result is not a heap of redundant mates
//!
//! Candidates are accepted GREEDILY per component pair against an analytic
//! model of the relative rigid DOF they remove: aligning one direction removes
//! two rotational DOF and a second, non-parallel one removes the third; each
//! independent constrained translation direction removes one more. A candidate
//! that raises neither count is geometry the accepted set already explains, and
//! it is dropped rather than handed to the solver as a redundant row. A pair
//! stops at 6. So a bolt through a plate gets its Concentric and the ONE Touch
//! Align under its head (leaving the spin free, which is correct), not one mate
//! per coplanar facet.
//!
//! Pairs that already carry a user constraint are left alone entirely, which is
//! also what makes pressing the button twice a no-op.
//!
//! The model is per PAIR, so it cannot see redundancy that closes through a
//! third component (A-B, B-C and A-C each holding the same freedom). Inferring
//! everything therefore tends to produce a set the solver calls over-determined
//! but consistent — every constraint satisfied at the pose it was read from —
//! and the app reports that rather than hiding it.
//!
//! # Nothing is dropped silently
//!
//! A user who counts touching faces and finds fewer constraints is owed the
//! difference, so every stage keeps its books ([`Gates`], [`InferScan`]):
//! candidates found and deliberately not created carry a [`Suppressed`] reason,
//! and each gate counts what it turned away — including
//! [`Gates::also_on_carrier`], the pairs that qualified in full but share a
//! plane or a centreline with an accepted constraint, which is the usual answer
//! to "it missed faces that are touching". The near-miss distances
//! ([`Gates::nearest_plane_gap`], [`Gates::nearest_axis_gap`]) say whether the
//! tolerance is what stands in the way, with the number needed to change it.

use std::collections::BTreeMap;

use serde::Deserialize;

use super::constraints;
use super::{AssemblyState, ConstraintEntry};
use crate::feature_pipeline::features::common::{bounds_of, face_boundary_points};
use crate::feature_pipeline::SceneMap;
use crate::{resolve_face_selection, SelectionGeometry, Vec3};

// ===========================================================================
// The rule table — what this lane can infer
// ===========================================================================

/// One inferable constraint type: the catalogue type it creates plus the
/// plain-language statement of what placement makes it.
pub struct InferenceRule {
    /// Joins [`constraints::constraint_type`] for the label + icon.
    pub type_id: &'static str,
    /// What the geometry has to look like — the dialog's line under the
    /// checkbox, and the MCP command's documentation.
    pub detects: &'static str,
    /// Whether the dialog starts with this rule ticked.
    pub default_on: bool,
}

/// The inferable set, in the order the dialog lists it (strongest placement
/// evidence first — also the order candidates are accepted in).
pub const INFERENCE_RULES: [InferenceRule; 3] = [
    InferenceRule {
        type_id: "concentric",
        detects: "Cylindrical, conical or toroidal faces on two parts that share \
                  one centreline — a shaft in a bore, a bolt through a stack. \
                  Neither the radii nor the distance along the axis need match.",
        default_on: true,
    },
    InferenceRule {
        type_id: "touch_align",
        detects: "Planar faces on two parts that lie in one plane, face each \
                  other, and overlap — parts resting or bolted flat together.",
        default_on: true,
    },
    InferenceRule {
        type_id: "coincident",
        detects: "Spherical faces sharing a centre, and corners of two already \
                  touching parts that sit on the same point.",
        default_on: true,
    },
];

/// The rule table joined to the constraint catalogue: `[{type, label, icon,
/// longName, detects, defaultOn}]`. The dialog renders THIS — it holds no list
/// of its own.
pub fn inferable_types_json() -> serde_json::Value {
    serde_json::Value::Array(
        INFERENCE_RULES
            .iter()
            .filter_map(|rule| {
                let def = constraints::constraint_type(rule.type_id)?;
                Some(serde_json::json!({
                    "type": def.type_id,
                    "label": def.label,
                    "icon": def.icon,
                    "longName": def.long_name,
                    "detects": rule.detects,
                    "defaultOn": rule.default_on,
                }))
            })
            .collect(),
    )
}

// ===========================================================================
// Options
// ===========================================================================

fn default_tolerance() -> f64 {
    1e-3
}
fn default_angle_tolerance_deg() -> f64 {
    0.1
}
fn default_max_pairs() -> usize {
    4096
}

/// Inference knobs. Every field has a default, so `{}` is a valid request; the
/// dialog only ever sends `types`, and the tolerances exist for the automation
/// surface (a coarsely translated import may want a looser gap).
#[derive(Debug, Clone, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct InferOptions {
    /// Which rules to run. `None` = every rule in [`INFERENCE_RULES`].
    pub types: Option<Vec<String>>,
    /// Linear gap tolerance (mm) for "coplanar", "collinear", "same point".
    pub tolerance: f64,
    /// Angular tolerance (degrees) for "parallel".
    pub angle_tolerance_deg: f64,
    /// Component-pair budget; pairs beyond it are reported as skipped.
    pub max_pairs: usize,
}

impl Default for InferOptions {
    fn default() -> Self {
        Self {
            types: None,
            tolerance: default_tolerance(),
            angle_tolerance_deg: default_angle_tolerance_deg(),
            max_pairs: default_max_pairs(),
        }
    }
}

impl InferOptions {
    /// Parse an options object; an empty / absent body is the defaults.
    pub fn parse(json: &str) -> Result<Self, String> {
        let trimmed = json.trim();
        if trimmed.is_empty() || trimmed == "null" {
            return Ok(Self::default());
        }
        let mut options: Self =
            serde_json::from_str(trimmed).map_err(|error| format!("infer options: {error}"))?;
        if !(options.tolerance > 0.0) {
            options.tolerance = default_tolerance();
        }
        if !(options.angle_tolerance_deg > 0.0) {
            options.angle_tolerance_deg = default_angle_tolerance_deg();
        }
        Ok(options)
    }

    fn enabled(&self, type_id: &str) -> bool {
        match &self.types {
            Some(list) => list.iter().any(|entry| entry == type_id),
            None => INFERENCE_RULES.iter().any(|rule| rule.type_id == type_id),
        }
    }

    /// `1 - cos(angle tolerance)`, the direction-parallelism slack.
    fn dot_slack(&self) -> f64 {
        1.0 - self.angle_tolerance_deg.to_radians().cos()
    }
}

// ===========================================================================
// Candidates + the report
// ===========================================================================

/// One inferred constraint, ready to be created.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub type_id: &'static str,
    pub elements: [String; 2],
    pub components: [String; 2],
    /// Measured evidence, for the report row ("Ø8 · 12 mm engaged").
    pub detail: String,
    /// Accept-order key within a rule (bigger = stronger evidence).
    score: f64,
}

impl Candidate {
    /// The `inputParams` an [`super::ConstraintEntry`] of this candidate carries
    /// (minus the id, which the caller mints).
    pub fn params(&self) -> serde_json::Value {
        serde_json::json!({
            "elements": [self.elements[0].clone(), self.elements[1].clone()],
        })
    }

    fn row(&self) -> serde_json::Value {
        serde_json::json!({
            "type": self.type_id,
            "elements": [self.elements[0].clone(), self.elements[1].clone()],
            "components": [self.components[0].clone(), self.components[1].clone()],
            "detail": self.detail,
        })
    }
}

/// Why a candidate that WAS found is not being created. Reported, never
/// silent: "it missed a face that is touching" is nearly always one of these,
/// and the user is owed the number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Suppressed {
    /// The accepted set already removes every degree of freedom this would.
    Redundant,
    /// The pair was already fully constrained before this one was reached.
    PairLocked,
}

impl Suppressed {
    fn word(self) -> &'static str {
        match self {
            Suppressed::Redundant => "redundant",
            Suppressed::PairLocked => "pair-fully-constrained",
        }
    }

    fn explanation(self) -> &'static str {
        match self {
            Suppressed::Redundant => {
                "the constraints already accepted for this pair hold it the same way"
            }
            Suppressed::PairLocked => "this pair already has no freedom left to remove",
        }
    }
}

/// The face pairs each gate turned away, with the closest miss it saw. This is
/// what turns "it missed some touching faces" into a number one can act on: a
/// non-zero `nearest_plane_gap` just above the tolerance says to loosen it,
/// while a large `same_facing` count says the geometry is flush rather than in
/// contact.
#[derive(Debug, Default, Clone)]
pub struct Gates {
    /// Parallel plane carriers that are not the SAME plane.
    pub planes_apart: usize,
    /// The smallest gap between two parallel plane carriers that missed.
    pub nearest_plane_gap: Option<f64>,
    /// Coplanar faces whose outward normals point the same way (flush, not a
    /// contact).
    pub same_facing: usize,
    /// Coplanar, facing faces whose boundaries do not overlap.
    pub no_overlap: usize,
    /// Parallel axis carriers that are not the SAME line.
    pub axes_apart: usize,
    /// The smallest gap between two parallel axis carriers that missed.
    pub nearest_axis_gap: Option<f64>,
    /// Coaxial faces that do NOT overlap along the axis — still concentric (a
    /// shared centreline is the constraint), recorded because it is worth
    /// knowing how much of the inferred set is a distant alignment rather than
    /// a seated fit.
    pub coaxial_but_apart: usize,
    /// Faces with no usable extent at all (no boundary, no loop vertices).
    pub no_extent: usize,
    /// Face pairs that qualified in full but share their carrier — the same
    /// plane, or the same centreline — with the pair that represents it. This
    /// is the count behind "it missed faces that are touching": they are not
    /// missed, they are held by the constraint that WAS created for that
    /// carrier, and a second mate on it would only over-constrain the solve.
    pub also_on_carrier: usize,
}

impl Gates {
    fn note_plane_gap(&mut self, gap: f64) {
        self.planes_apart += 1;
        self.nearest_plane_gap = Some(match self.nearest_plane_gap {
            Some(best) => best.min(gap),
            None => gap,
        });
    }

    fn note_axis_gap(&mut self, gap: f64) {
        self.axes_apart += 1;
        self.nearest_axis_gap = Some(match self.nearest_axis_gap {
            Some(best) => best.min(gap),
            None => gap,
        });
    }

    fn json(&self) -> serde_json::Value {
        serde_json::json!({
            "planesApart": self.planes_apart,
            "nearestPlaneGap": self.nearest_plane_gap,
            "sameFacing": self.same_facing,
            "noOverlap": self.no_overlap,
            "axesApart": self.axes_apart,
            "nearestAxisGap": self.nearest_axis_gap,
            "coaxialButApart": self.coaxial_but_apart,
            "noExtent": self.no_extent,
            "alsoOnCarrier": self.also_on_carrier,
        })
    }
}

/// What one scan found, what it turned away, and what it did not look at.
#[derive(Debug, Default, Clone)]
pub struct InferScan {
    pub candidates: Vec<Candidate>,
    /// Candidates that were found and deliberately not created, each with why.
    pub suppressed: Vec<(Candidate, Suppressed)>,
    pub component_count: usize,
    pub pairs_considered: usize,
    /// Pairs whose bounding boxes do not touch — nothing can be inferred.
    pub pairs_apart: usize,
    /// Pairs left alone because they already carry a constraint.
    pub pairs_constrained: usize,
    /// Pairs past the budget.
    pub pairs_skipped: usize,
    pub faces_scanned: usize,
    /// Per-gate rejection counts across every pair examined.
    pub gates: Gates,
    /// Components whose geometry could not be read (freed handle, no members).
    pub warnings: Vec<String>,
}

impl InferScan {
    /// Per-type candidate counts, every rule present (a zero is information).
    pub fn by_type(&self) -> serde_json::Value {
        let mut counts = serde_json::Map::new();
        for rule in INFERENCE_RULES.iter() {
            let found = self
                .candidates
                .iter()
                .filter(|candidate| candidate.type_id == rule.type_id)
                .count();
            counts.insert(rule.type_id.to_string(), serde_json::json!(found));
        }
        serde_json::Value::Object(counts)
    }

    /// The scan report the app and MCP render.
    pub fn report(&self) -> serde_json::Value {
        serde_json::json!({
            "ok": true,
            "componentCount": self.component_count,
            "pairsConsidered": self.pairs_considered,
            "pairsApart": self.pairs_apart,
            "pairsAlreadyConstrained": self.pairs_constrained,
            "pairsOverBudget": self.pairs_skipped,
            "facesScanned": self.faces_scanned,
            "byType": self.by_type(),
            "candidates": self
                .candidates
                .iter()
                .map(Candidate::row)
                .collect::<Vec<_>>(),
            "suppressed": self
                .suppressed
                .iter()
                .map(|(candidate, why)| {
                    let mut row = candidate.row();
                    let object = row.as_object_mut().expect("row is an object");
                    object.insert("reason".into(), serde_json::json!(why.word()));
                    object.insert("why".into(), serde_json::json!(why.explanation()));
                    row
                })
                .collect::<Vec<_>>(),
            "gates": self.gates.json(),
            "warnings": self.warnings.clone(),
        })
    }
}

// ===========================================================================
// Component geometry (resolved ONCE per component, reused for every pair)
// ===========================================================================

/// One named, analytically resolved face in WORLD space.
struct FaceGeom {
    name: String,
    geometry: SelectionGeometry,
    points: Vec<Vec3>,
}

/// A carrier plane or axis shared by one component's faces — the matching unit,
/// so a pair costs `groups(A) × groups(B)` and not `faces(A) × faces(B)`.
struct Carrier {
    /// Canonical direction (plane normal / axis direction), unit.
    direction: Vec3,
    /// The point of the carrier closest to the world origin — the foot of the
    /// plane along its normal, or of the axis line. With the directions matched,
    /// the distance between two feet IS the gap between the carriers, so ONE
    /// comparison settles "same plane" and "same axis" alike.
    foot: Vec3,
    faces: Vec<usize>,
}

struct VertexGeom {
    /// `{solid}@x,y,z` — the vertex selection ref (component-LOCAL position).
    reference: String,
    world: Vec3,
}

struct ComponentGeom {
    id: String,
    faces: Vec<FaceGeom>,
    planes: Vec<Carrier>,
    axes: Vec<Carrier>,
    spheres: Vec<usize>,
    vertices: Vec<VertexGeom>,
    min: Vec3,
    max: Vec3,
}

/// The positions of the topology vertices a face's loops reference — the
/// extent fallback for a face whose boundary curves would not evaluate.
fn loop_vertex_points(solid: &crate::BrepSolid, record: &crate::FaceRecord) -> Vec<Vec3> {
    let mut points = Vec::new();
    for loop_record in &record.loops {
        for coedge in &loop_record.coedges {
            let Some(edge) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                continue;
            };
            for vertex_id in [edge.start_vertex_id, edge.end_vertex_id] {
                if let Some(vertex) = solid.vertices.iter().find(|v| v.id == vertex_id) {
                    points.push(vertex.point);
                }
            }
        }
    }
    points
}

/// Canonical sign for a direction, so two ANTI-parallel normals (which is what
/// a contact looks like) land in ONE carrier group.
///
/// The sign is taken from the component of LARGEST magnitude, never the first
/// non-zero one. Imported geometry carries sub-micron noise in the components
/// that ought to be exactly zero, and picking the first "significant" one makes
/// the sign flip on that noise: `(1e-8, 0, 1)` and `(-1e-8, 0, 1)` are the same
/// face direction to any tolerance that matters, but a first-component rule
/// sends them to opposite carriers and the contact between them is never seen.
/// The largest component is stable under exactly that perturbation.
fn canonical(direction: Vec3) -> Vec3 {
    let components = [direction.x, direction.y, direction.z];
    let mut dominant = 0;
    for (index, value) in components.iter().enumerate() {
        if value.abs() > components[dominant].abs() {
            dominant = index;
        }
    }
    if components[dominant] < 0.0 {
        direction.scale(-1.0)
    } else {
        direction
    }
}

/// Add a face to the carrier group it belongs to, or open a new one.
fn group_into(
    groups: &mut Vec<Carrier>,
    direction: Vec3,
    foot: Vec3,
    face: usize,
    linear_tol: f64,
    dot_slack: f64,
) {
    for group in groups.iter_mut() {
        if group.direction.dot(direction) > 1.0 - dot_slack
            && group.foot.sub(foot).length() <= linear_tol
        {
            group.faces.push(face);
            return;
        }
    }
    groups.push(Carrier {
        direction,
        foot,
        faces: vec![face],
    });
}

/// Resolve every named face (and every vertex) of one component into world
/// geometry, grouped by carrier. `None` when the component has no readable
/// geometry at all.
fn component_geometry(
    scene: &SceneMap,
    id: &str,
    options: &InferOptions,
    scan: &mut InferScan,
) -> Option<ComponentGeom> {
    let record = scene.components.get(id)?;
    let inverse = match record.transform.rigid_inverse() {
        Ok(inverse) => inverse,
        Err(error) => {
            scan.warnings
                .push(format!("{id}: non-rigid pose, skipped ({error})"));
            return None;
        }
    };
    let mut faces: Vec<FaceGeom> = Vec::new();
    let mut vertices: Vec<VertexGeom> = Vec::new();
    for (solid_name, handle) in scene.component_solids(id) {
        let read = crate::with_registered_solid_str(handle, |solid| {
            let mut out = Vec::new();
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                let Some(name) = face.name.clone() else {
                    continue; // an unnamed face cannot be a constraint element
                };
                let Ok(geometry) = resolve_face_selection(solid, face.id) else {
                    continue; // freeform / degenerate — no analytic frame
                };
                // A face with no usable boundary samples has no EXTENT, and a
                // contact is decided by extent — so fall back to the topology
                // vertices its loops name before giving up on it. (An edge whose
                // curve refuses to evaluate is rare but real in imported
                // geometry, and it must not cost the whole face.)
                let mut points = face_boundary_points(solid, face).unwrap_or_default();
                if points.is_empty() {
                    points = loop_vertex_points(solid, face);
                }
                out.push(FaceGeom {
                    name,
                    geometry,
                    points,
                });
            }
            let corners: Vec<Vec3> = solid.vertices.iter().map(|vertex| vertex.point).collect();
            Ok((out, corners))
        });
        match read {
            Ok((solid_faces, corners)) => {
                faces.extend(solid_faces);
                for world in corners {
                    let local = inverse.point(world);
                    vertices.push(VertexGeom {
                        reference: format!("{solid_name}@{},{},{}", local.x, local.y, local.z),
                        world,
                    });
                }
            }
            Err(error) => scan
                .warnings
                .push(format!("{id}: member '{solid_name}' unreadable ({error})")),
        }
    }
    if faces.is_empty() {
        return None;
    }
    scan.faces_scanned += faces.len();

    let mut planes = Vec::new();
    let mut axes = Vec::new();
    let mut spheres = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        match face.geometry {
            SelectionGeometry::Plane { origin, normal } => {
                let Ok(unit) = normal.normalized() else { continue };
                let direction = canonical(unit);
                // The foot of the plane along its own normal.
                let foot = direction.scale(origin.dot(direction));
                group_into(
                    &mut planes,
                    direction,
                    foot,
                    index,
                    options.tolerance,
                    options.dot_slack(),
                );
            }
            SelectionGeometry::Axis {
                origin, direction, ..
            } => {
                let Ok(unit) = direction.normalized() else { continue };
                let axis = canonical(unit);
                // The point on the carrier line closest to the world origin —
                // one canonical representative per line, whatever face made it.
                let foot = origin.sub(axis.scale(origin.dot(axis)));
                group_into(
                    &mut axes,
                    axis,
                    foot,
                    index,
                    options.tolerance,
                    options.dot_slack(),
                );
            }
            SelectionGeometry::Sphere { .. } => spheres.push(index),
            _ => {}
        }
    }

    let cloud: Vec<Vec3> = faces
        .iter()
        .flat_map(|face| face.points.iter().copied())
        .chain(faces.iter().map(|face| face.geometry.representative_point()))
        .collect();
    let (min, max) = bounds_of(&cloud)?;
    Some(ComponentGeom {
        id: id.to_string(),
        faces,
        planes,
        axes,
        spheres,
        vertices,
        min,
        max,
    })
}

// ===========================================================================
// Relative-DOF bookkeeping (why the accepted set stays small)
// ===========================================================================

/// The directions a pair's accepted constraints have already pinned. Aligning
/// one direction removes two rotational DOF; a second, independent alignment
/// removes the third. Each independent constrained translation direction
/// removes one.
#[derive(Default)]
struct PairDof {
    rotation: Vec<Vec3>,
    translation: Vec<Vec3>,
    points: Vec<Vec3>,
}

/// Push `candidate` onto an orthonormal span, returning whether it grew it.
fn span_push(span: &mut Vec<Vec3>, candidate: Vec3) -> bool {
    let mut residual = candidate;
    for basis in span.iter() {
        residual = residual.sub(basis.scale(residual.dot(*basis)));
    }
    if residual.length() <= 1e-6 {
        return false;
    }
    match residual.normalized() {
        Ok(unit) => {
            span.push(unit);
            true
        }
        Err(_) => false,
    }
}

impl PairDof {
    fn removed(&self) -> usize {
        let rotational = match self.rotation.len() {
            0 => 0,
            1 => 2,
            _ => 3,
        };
        rotational + self.translation.len()
    }

    fn locked(&self) -> bool {
        self.removed() >= 6
    }

    /// Fold a candidate in if it removes a DOF the pair still has; `false`
    /// means the accepted set already explains this geometry, and the caller
    /// drops the candidate rather than hand the solver a redundant row.
    fn accept(&mut self, geometry: &CandidateDof) -> bool {
        let before = self.removed();
        let mut folded = PairDof {
            rotation: self.rotation.clone(),
            translation: self.translation.clone(),
            points: self.points.clone(),
        };
        for direction in &geometry.rotation {
            span_push(&mut folded.rotation, *direction);
        }
        for direction in &geometry.translation {
            span_push(&mut folded.translation, *direction);
        }
        if let Some(point) = geometry.point {
            // Two points held together fix the chord between them: the pair
            // can no longer spin about anything but that chord.
            for existing in &folded.points {
                let chord = point.sub(*existing);
                if chord.length() > 1e-6 {
                    span_push(&mut folded.rotation, chord);
                }
            }
            folded.points.push(point);
        }
        if folded.removed() <= before {
            return false;
        }
        *self = folded;
        true
    }
}

/// The rigid DOF one candidate would remove.
struct CandidateDof {
    rotation: Vec<Vec3>,
    translation: Vec<Vec3>,
    point: Option<Vec3>,
}

// ===========================================================================
// The scan
// ===========================================================================

/// Which components an existing constraint ties together (for the pair skip).
fn constrained_pairs(state: &AssemblyState, scene: &SceneMap) -> Vec<(String, String)> {
    let mut pairs = Vec::new();
    for entry in &state.constraints {
        let mut owners: Vec<String> = entry
            .elements()
            .iter()
            .filter_map(|element| scene.owning_component(element).map(|record| record.id.clone()))
            .collect();
        owners.sort();
        owners.dedup();
        if owners.len() == 2 {
            pairs.push((owners[0].clone(), owners[1].clone()));
        }
    }
    pairs
}

/// Do two AABBs touch, within `slack`?
fn boxes_touch(a: &ComponentGeom, b: &ComponentGeom, slack: f64) -> bool {
    a.min.x - slack <= b.max.x
        && b.min.x - slack <= a.max.x
        && a.min.y - slack <= b.max.y
        && b.min.y - slack <= a.max.y
        && a.min.z - slack <= b.max.z
        && b.min.z - slack <= a.max.z
}

/// The box-prefilter slack for one pair — deliberately LOOSER than the contact
/// tolerance.
///
/// The prefilter's job is to skip work, not to decide anything, and a pair
/// rejected here is never measured: with the slack set to the tolerance itself,
/// two parts modelled 0.05 mm apart were reported as simply "not touching" with
/// no number attached, and the user had nothing to act on. Admitting anything
/// within 1% of the smaller part's own size costs one carrier match on a pair
/// that will usually fail its gates — and buys the near-miss distance that says
/// whether the tolerance is the problem.
fn prefilter_slack(a: &ComponentGeom, b: &ComponentGeom, tolerance: f64) -> f64 {
    let diagonal = |component: &ComponentGeom| component.max.sub(component.min).length();
    tolerance.max(0.01 * diagonal(a).min(diagonal(b)))
}

/// The interval a point cloud projects onto `direction`.
fn extent_along(points: &[Vec3], direction: Vec3) -> Option<(f64, f64)> {
    let mut iter = points.iter().map(|point| point.dot(direction));
    let first = iter.next()?;
    let (mut low, mut high) = (first, first);
    for value in iter {
        low = low.min(value);
        high = high.max(value);
    }
    Some((low, high))
}

fn overlap(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.1.min(b.1) - a.0.max(b.0)
}

/// Round a measurement for a report line (three decimals, no trailing noise).
/// `+ 0.0` folds a negative zero — which a flipped sign produces at exactly
/// zero separation — onto plain `0`.
fn measure(value: f64) -> String {
    let rounded = (value * 1000.0).round() / 1000.0 + 0.0;
    format!("{rounded}")
}

/// Scan the scene for inferable constraints. READ-ONLY: it neither solves nor
/// touches the constraint list.
pub fn scan(state: &AssemblyState, scene: &SceneMap, options: &InferOptions) -> InferScan {
    let mut result = InferScan::default();
    let ids: Vec<String> = scene.components.keys().cloned().collect();
    result.component_count = ids.len();
    if ids.len() < 2 {
        return result;
    }
    if !scene.components.values().any(|record| record.fixed) {
        result.warnings.push(
            "no component is grounded — inferred constraints will position parts relative to one \
             another, but the assembly as a whole is still free to move"
                .to_string(),
        );
    }

    let mut geometry: BTreeMap<String, ComponentGeom> = BTreeMap::new();
    for id in &ids {
        if let Some(component) = component_geometry(scene, id, options, &mut result) {
            geometry.insert(id.clone(), component);
        }
    }
    let existing = constrained_pairs(state, scene);
    let present: Vec<String> = geometry.keys().cloned().collect();

    for (index, first) in present.iter().enumerate() {
        for second in present.iter().skip(index + 1) {
            if existing
                .iter()
                .any(|(a, b)| a == first && b == second)
            {
                result.pairs_constrained += 1;
                continue;
            }
            let (Some(a), Some(b)) = (geometry.get(first), geometry.get(second)) else {
                continue;
            };
            // The box prefilter decides whether the CONTACT rules are worth
            // running — it cannot decide the pair. Two parts on one centreline
            // are concentric at any distance, so a pair whose boxes are far
            // apart is still asked that question; it is only spared the
            // face-against-face work.
            let touching = boxes_touch(a, b, prefilter_slack(a, b, options.tolerance));
            if !touching {
                result.pairs_apart += 1;
                if !options.enabled("concentric") {
                    continue;
                }
            }
            if result.pairs_considered >= options.max_pairs {
                result.pairs_skipped += 1;
                continue;
            }
            result.pairs_considered += 1;
            let (accepted, suppressed) =
                pair_candidates(a, b, options, &mut result.gates, touching);
            result.candidates.extend(accepted);
            result.suppressed.extend(suppressed);
        }
    }
    result
}

/// Every candidate one component pair yields: the accepted ones, and the ones
/// found but deliberately not created (with why).
fn pair_candidates(
    a: &ComponentGeom,
    b: &ComponentGeom,
    options: &InferOptions,
    gates: &mut Gates,
    touching: bool,
) -> (Vec<Candidate>, Vec<(Candidate, Suppressed)>) {
    let mut proposals: Vec<(Candidate, CandidateDof)> = Vec::new();
    if options.enabled("concentric") {
        proposals.extend(concentric_candidates(a, b, options, gates));
    }
    // The contact rules need the two parts to be near each other at all; the
    // prefilter has already answered that.
    if touching && options.enabled("touch_align") {
        proposals.extend(touch_align_candidates(a, b, options, gates));
    }
    let touches = touching && !proposals.is_empty();
    if touching && options.enabled("coincident") {
        proposals.extend(sphere_candidates(a, b, options));
        if touches {
            proposals.extend(vertex_candidates(a, b, options));
        }
    }

    // Accept in rule order, strongest evidence first inside each rule; the
    // element names break ties so two runs of the same scene agree.
    let rule_order = |type_id: &str| {
        INFERENCE_RULES
            .iter()
            .position(|rule| rule.type_id == type_id)
            .unwrap_or(usize::MAX)
    };
    proposals.sort_by(|(left, _), (right, _)| {
        rule_order(left.type_id)
            .cmp(&rule_order(right.type_id))
            .then(
                right
                    .score
                    .partial_cmp(&left.score)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
            .then(left.elements.cmp(&right.elements))
    });

    let mut dof = PairDof::default();
    let mut accepted = Vec::new();
    let mut suppressed = Vec::new();
    for (candidate, geometry) in proposals {
        if dof.locked() {
            suppressed.push((candidate, Suppressed::PairLocked));
        } else if dof.accept(&geometry) {
            accepted.push(candidate);
        } else {
            suppressed.push((candidate, Suppressed::Redundant));
        }
    }
    (accepted, suppressed)
}

/// Coaxial axis-bearing faces with overlapping axial extents.
fn concentric_candidates(
    a: &ComponentGeom,
    b: &ComponentGeom,
    options: &InferOptions,
    gates: &mut Gates,
) -> Vec<(Candidate, CandidateDof)> {
    let mut out = Vec::new();
    for group_a in &a.axes {
        for group_b in &b.axes {
            if group_a.direction.dot(group_b.direction) <= 1.0 - options.dot_slack() {
                continue;
            }
            let gap = group_a.foot.sub(group_b.foot).length();
            if gap > options.tolerance {
                gates.note_axis_gap(gap);
                continue;
            }
            let axis = group_a.direction;
            let mut best: Option<(f64, usize, usize)> = None;
            for &face_a in &group_a.faces {
                let Some(span_a) = extent_along(&a.faces[face_a].points, axis) else {
                    gates.no_extent += 1;
                    continue;
                };
                for &face_b in &group_b.faces {
                    let Some(span_b) = extent_along(&b.faces[face_b].points, axis) else {
                        gates.no_extent += 1;
                        continue;
                    };
                    // A shared centreline IS the constraint: two coaxial faces
                    // are concentric however far apart they sit along it (a bolt
                    // through a stack of plates is the everyday case). The
                    // overlap is kept only to RANK — an engaged pair is the more
                    // telling representative of the axis than a distant one.
                    let engaged = overlap(span_a, span_b);
                    if engaged <= options.tolerance {
                        gates.coaxial_but_apart += 1;
                    }
                    gates.also_on_carrier += 1;
                    if best.map(|(score, _, _)| engaged > score).unwrap_or(true) {
                        best = Some((engaged, face_a, face_b));
                    }
                }
            }
            let Some((engaged, face_a, face_b)) = best else {
                continue;
            };
            gates.also_on_carrier -= 1; // the representative is not an "also"
            let radius = match a.faces[face_a].geometry {
                SelectionGeometry::Axis { radius, .. } => radius,
                _ => None,
            };
            // Say which it is: an engaged length when the faces overlap along
            // the axis, the gap along it when they are simply coaxial.
            let along = if engaged > options.tolerance {
                format!("{} mm engaged", measure(engaged))
            } else {
                format!("{} mm apart on the axis", measure(-engaged))
            };
            let detail = match radius {
                Some(radius) => {
                    format!("\u{2300}{} \u{00b7} {along}", measure(radius * 2.0))
                }
                None => along,
            };
            let (u, v) = basis_of(axis);
            out.push((
                Candidate {
                    type_id: "concentric",
                    elements: [
                        a.faces[face_a].name.clone(),
                        b.faces[face_b].name.clone(),
                    ],
                    components: [a.id.clone(), b.id.clone()],
                    detail,
                    score: engaged,
                },
                CandidateDof {
                    rotation: vec![axis],
                    translation: vec![u, v],
                    point: None,
                },
            ));
        }
    }
    out
}

/// Coplanar, mutually facing, overlapping planar faces.
///
/// Each of the three conditions is a GATE with a counter: a face pair that is
/// coplanar but flush rather than facing, or facing but not overlapping, is
/// counted rather than dropped, so a scan that finds less than the user expects
/// says which test turned the geometry away.
fn touch_align_candidates(
    a: &ComponentGeom,
    b: &ComponentGeom,
    options: &InferOptions,
    gates: &mut Gates,
) -> Vec<(Candidate, CandidateDof)> {
    let mut out = Vec::new();
    for group_a in &a.planes {
        for group_b in &b.planes {
            if group_a.direction.dot(group_b.direction) <= 1.0 - options.dot_slack() {
                continue;
            }
            let gap = group_a.foot.sub(group_b.foot).length();
            if gap > options.tolerance {
                gates.note_plane_gap(gap);
                continue;
            }
            let normal = group_a.direction;
            let (u, v) = basis_of(normal);
            let mut best: Option<(f64, usize, usize)> = None;
            for &face_a in &group_a.faces {
                let SelectionGeometry::Plane { normal: na, .. } = a.faces[face_a].geometry else {
                    continue;
                };
                let (Some(span_au), Some(span_av)) = (
                    extent_along(&a.faces[face_a].points, u),
                    extent_along(&a.faces[face_a].points, v),
                ) else {
                    gates.no_extent += 1;
                    continue;
                };
                for &face_b in &group_b.faces {
                    let SelectionGeometry::Plane { normal: nb, .. } = b.faces[face_b].geometry
                    else {
                        continue;
                    };
                    // Contact means the outward normals OPPOSE: two flush faces
                    // pointing the same way are not touching, they are aligned.
                    if na.dot(nb) >= 0.0 {
                        gates.same_facing += 1;
                        continue;
                    }
                    let (Some(span_bu), Some(span_bv)) = (
                        extent_along(&b.faces[face_b].points, u),
                        extent_along(&b.faces[face_b].points, v),
                    ) else {
                        gates.no_extent += 1;
                        continue;
                    };
                    let across = overlap(span_au, span_bu);
                    let along = overlap(span_av, span_bv);
                    if across <= options.tolerance || along <= options.tolerance {
                        gates.no_overlap += 1;
                        continue;
                    }
                    let area = across * along;
                    // Every pair that reaches here IS a contact; the carrier
                    // keeps the largest as its representative and counts the
                    // rest, which are held by the same mate.
                    gates.also_on_carrier += 1;
                    if best.map(|(score, _, _)| area > score).unwrap_or(true) {
                        best = Some((area, face_a, face_b));
                    }
                }
            }
            let Some((area, face_a, face_b)) = best else {
                continue;
            };
            gates.also_on_carrier -= 1; // the representative is not an "also"
            out.push((
                Candidate {
                    type_id: "touch_align",
                    elements: [
                        a.faces[face_a].name.clone(),
                        b.faces[face_b].name.clone(),
                    ],
                    components: [a.id.clone(), b.id.clone()],
                    detail: format!("{} mm\u{00b2} of contact", measure(area)),
                    score: area,
                },
                CandidateDof {
                    rotation: vec![normal],
                    translation: vec![normal],
                    point: None,
                },
            ));
        }
    }
    out
}

/// Spherical faces about one centre — a ball joint.
fn sphere_candidates(
    a: &ComponentGeom,
    b: &ComponentGeom,
    options: &InferOptions,
) -> Vec<(Candidate, CandidateDof)> {
    let mut out = Vec::new();
    for &face_a in &a.spheres {
        let SelectionGeometry::Sphere { center: ca, .. } = a.faces[face_a].geometry else {
            continue;
        };
        for &face_b in &b.spheres {
            let SelectionGeometry::Sphere { center: cb, .. } = b.faces[face_b].geometry else {
                continue;
            };
            let gap = ca.sub(cb).length();
            if gap > options.tolerance {
                continue;
            }
            out.push((
                Candidate {
                    type_id: "coincident",
                    elements: [
                        a.faces[face_a].name.clone(),
                        b.faces[face_b].name.clone(),
                    ],
                    components: [a.id.clone(), b.id.clone()],
                    detail: "shared sphere centre".to_string(),
                    score: 1.0 / (1.0 + gap),
                },
                CandidateDof {
                    rotation: Vec::new(),
                    translation: unit_basis(),
                    point: Some(ca),
                },
            ));
        }
    }
    out
}

/// Corners of two touching parts that land on the same world point.
fn vertex_candidates(
    a: &ComponentGeom,
    b: &ComponentGeom,
    options: &InferOptions,
) -> Vec<(Candidate, CandidateDof)> {
    let mut out = Vec::new();
    for vertex_a in &a.vertices {
        for vertex_b in &b.vertices {
            let gap = vertex_a.world.sub(vertex_b.world).length();
            if gap > options.tolerance {
                continue;
            }
            out.push((
                Candidate {
                    type_id: "coincident",
                    elements: [vertex_a.reference.clone(), vertex_b.reference.clone()],
                    components: [a.id.clone(), b.id.clone()],
                    detail: "corners meet".to_string(),
                    score: 1.0 / (1.0 + gap),
                },
                CandidateDof {
                    rotation: Vec::new(),
                    translation: unit_basis(),
                    point: Some(vertex_a.world),
                },
            ));
        }
    }
    out
}

/// Two unit vectors spanning the plane perpendicular to `direction`.
fn basis_of(direction: Vec3) -> (Vec3, Vec3) {
    let u = direction
        .perpendicular()
        .and_then(|vector| vector.normalized())
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    (u, direction.cross(u))
}

fn unit_basis() -> Vec<Vec3> {
    vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ]
}

// ===========================================================================
// Applying a scan
// ===========================================================================

/// Turn accepted candidates into constraint entries on `state`, minting ids
/// from each type's short name and the persistent counter (the same rule
/// [`super::exports`]'s add takes). Entries are created CLOSED — an inferred
/// batch is dozens of rows, and the panel's one-open-dialog accordion is a
/// user's place in the list, not something a batch may claim.
pub fn apply(state: &mut AssemblyState, candidates: &[Candidate]) -> Vec<String> {
    let mut created = Vec::new();
    for candidate in candidates {
        let Some(def) = constraints::constraint_type(candidate.type_id) else {
            continue;
        };
        state.id_counter += 1;
        let id = format!("{}{}", def.short_name, state.id_counter);
        let mut params = candidate.params();
        params
            .as_object_mut()
            .expect("candidate params is an object")
            .insert("id".into(), serde_json::Value::String(id.clone()));
        state.constraints.push(ConstraintEntry {
            constraint_type: candidate.type_id.to_string(),
            input_params: params,
            persistent_data: serde_json::Value::Object(serde_json::Map::new()),
            enabled: true,
            open: false,
        });
        created.push(id);
    }
    created
}


