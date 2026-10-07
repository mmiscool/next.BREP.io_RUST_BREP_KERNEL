//! TU — Tube. A BALL-AND-STICK pipe network: exactly ONE solid segment per input
//! edge, joined by a SPHERE at every junction (a vertex where ≥2 segments meet).
//!
//! # Model (the spec)
//!
//! - Each selected edge → its OWN segment body, following that edge exactly
//!   (start vertex → end vertex, radius = tube radius): a CYLINDER for a straight
//!   edge, a swept circle for a curved one (see "Curved edges" below).
//! - Edges are grouped into CONNECTED COMPONENTS by shared endpoints. Each
//!   component becomes ONE solid; DISCONNECTED components become SEPARATE solids
//!   (selecting several disconnected paths just yields several tubes).
//! - At every junction node (degree ≥ 2, i.e. where two or more segments meet —
//!   a simple corner, a 3-way/4-way branch, anything) a SPHERE is unioned in,
//!   filling and rounding the joint. This is what makes 3-way junctions work: the
//!   sphere bridges arbitrarily many arms. Its radius is the tube radius at a
//!   STRAIGHT-THROUGH node with cylindrical arms and a CLEARANCE factor × the
//!   tube radius wherever two arms form a crotch or either arm is curved. That
//!   factor is not fixed: [`assemble_component`] walks
//!   [`JOINT_CLEARANCE_LADDER`] smallest first and keeps the first rung the
//!   EXACT boolean builds. Rung 0 is 1.0 — an INSCRIBED ball, exactly the tube
//!   radius, standing proud of nothing — and it is what a crotch normally gets
//!   now; the wider rungs survive as fallbacks for a model that still refuses.
//!   See "Joint booleans" below for why the bead was ever there.
//! - Free ends (degree 1) keep the segment's flat cap.
//!
//! # Bends (`bendRadius > 0`)
//!
//! A corner where exactly TWO STRAIGHT edges meet can be ROUNDED instead of
//! filled: the path is pulled back from the node by `bendRadius / tan(γ/2)` on
//! each side, a tangent arc is laid across the corner, and the whole run of
//! segments joined by such corners is swept as ONE body — no joint ball, no
//! boolean, no bead, and a wall that is tangent-continuous through the turn.
//! This is the only construction here with NO clearance in it, because a bend
//! has no crotch: the tangency the ladder's wider rungs exist to bury is a
//! property of two equal-radius arms meeting at a point, and a bend never lets
//! them meet.
//!
//! The lane is chosen PER NODE. A branch (degree ≥ 3) has no single corner to
//! round and keeps its ball; a curved arm arrives with its own tangent and the
//! arc that would meet it is a different solve, so it keeps its ball too. Both
//! coexist with bends in one path.
//!
//! A bend is built from EXACT pieces and SEWN, not lofted: an extruded cylinder
//! per straight run, a revolved torus sector per corner, interior caps dropped,
//! and the coincident rims paired by [`crate::sew_solid`]. The pieces meet
//! tangentially — that is what G1 means — and a tangential contact is the class
//! the imprint declines, so they cannot be UNIONED; but they do not need
//! intersecting, because they already abut. Measured on a 90° elbow: −2.0e-11
//! against `π·r²·L`, every carrier analytic. The lofted chain sweep remains as
//! the fallback for a run the sew cannot close, and reads −5.7e-6.
//!
//! What a bend COSTS is face count: two half-faces per piece, because
//! `revolve_profile_brep_named` will not take a single closed profile curve and
//! the legs must match its rims edge for edge or nothing pairs. A chain of ONE
//! segment with no corner is still dispatched back to the cylinder lane, so a
//! tube that never asked for a bend keeps its single-face walls.
//!
//! Hollow bends bore CONCENTRICALLY — outer and inner sweep the same path — so
//! the wall through a turn is the same thickness as along the straight, which
//! the ball lane cannot do (its two assemblies use clearance balls of different
//! radii, so a joint's wall varies).
//!
//! # Joint booleans (robustness)
//!
//! Two equal-radius arms leaving a node in different directions are TANGENT to
//! each other at the crotch — the two points `node ± radius·(â×b̂)/|â×b̂|`, at
//! distance exactly `radius` from the node. A joint ball of exactly `radius`
//! leaves those tangent points ON its own surface, so the arm-vs-arm face pair
//! stays a SINGULAR (tangent-node) intersection whatever order the bodies are
//! unioned in: sphere-first buries the arms' MIDDLES, not their crotch.
//!
//! That contact is an ISOLATED NODE — two transverse branches crossing at a
//! single point — and the imprint ASSEMBLES it:
//! `csg/imprint/tangent_contact.rs` separates an isolated node from an
//! EXTENDED contact by the rank of `II_a − II_b`, the 2D arrangement carves the
//! pinch on both faces, and the union is exact. An inscribed ball is therefore
//! the normal answer, and rung 0 of the ladder is 1.0.
//!
//! It was not always. The imprint used to refuse every tangential contact
//! alike, the SoS perturbation lane rescued the union by shifting an operand
//! bodily, and that is where the joint's microns-off geometry and its shredded
//! end-cap crescents came from (reported 2026-09-09). The clearance existed to
//! dodge that refusal: a ball a hair wider trims each arm back past its own
//! crotch, so every pair the boolean sees is a clean transversal crossing. Once
//! the refusal was narrowed to extended contacts, the aspect dependence the
//! ladder had been calibrated against turned out to be the GATE rather than the
//! geometry — an inscribed ball builds at every aspect measured from 5 to 200.
//! The wider rungs are still tried in order, accepting only an exact build.
//! A straight-through node of cylindrical arms has no crotch and keeps
//! the plain `radius` ball, which is invisible inside the run.
//!
//! The unions are assembled in a CONNECTIVITY order (each body overlaps the
//! running solid) so the boolean is never asked to union two DISJOINT bodies
//! (untested multi-shell assembly). BINARY folds only.
//!
//! # Where the joint ball's seam runs
//!
//! A sphere is ONE face with a pole-to-pole seam edge and a degenerate edge at
//! each pole — the only way a closed sphere fits a rectangular parametric
//! domain — and the union keeps whichever of them land in the region that
//! survives. A surviving seam borders the SAME face on both sides: a line drawn
//! across the joint that bounds nothing, plus a stray vertex at the pole
//! (reported 2026-09-09). The ball is the kernel's own scaffolding, so
//! [`joint_ball_frame`] chooses a parametrization that parks the seam and both
//! poles INSIDE the arms that are about to remove them, and the joint face comes
//! out bounded by its rim arcs alone. Nothing downstream reads a joint's `u = 0`.
//!
//! # Hollow (`innerRadius > 0`)
//!
//! Build the whole assembly TWICE — once at the outer radius, once at the inner —
//! and subtract the inner solid from the outer, per component. For a STRAIGHT
//! tube the outer and inner are exact and share coincident end caps, so the
//! bore opens cleanly. Curved segments share concentric sweep frames and end caps.
//!
//! Junctions use the same ball-and-stick construction at both radii, keeping
//! the bore connected through corners and branches. The radii specify the arm
//! cross sections; joint walls follow the rounded unions rather than a constant
//! thickness offset. Each crotch ball is widened by the same clearance factor at
//! its respective radius, also at tangent joins involving curved arms: outer and
//! inner clear the SAME rung, so a joint's wall stays `k·(radius − innerRadius)`
//! and the slenderer inner assembly is usually the one that sets the rung.
//!
//! # Curved edges
//!
//! An ARC / SPLINE / HELIX edge builds its segment by SWEEPING a circle of the
//! tube radius along the edge — `sweep_profile_along_path`, the very builder
//! Path Sweep (SWP) drives, at its own 32-station default — instead of the
//! straight lane's cylinder. Nothing else about the model changes: a curved
//! segment is still ONE body per edge, still meets its neighbours at a joint
//! sphere, and still caps flat at a free end.
//!
//! The two lanes agree at their joints because the swept profile is a CIRCLE:
//! the sweep places it perpendicular to the path tangent and it is rotationally
//! symmetric, so a segment's end cap is the disc of radius `radius` centred on
//! the edge's end vertex, exactly the disc the cylinder lane caps with. Straight
//! and curved segments therefore mix freely in one path.
//!
//! Each edge is classified by SAMPLING it against its own chord (16 stations,
//! `1e-6` relative) rather than by a midpoint alone: a symmetric S-spline has
//! its midpoint ON the chord, and the two lanes emit different face counts, so a
//! misread would be a silent topology change rather than a near miss.
//!
//! FOLD-OVER is refused. A tube of radius `r` swept along a curve whose
//! curvature radius drops to `r` turns itself inside out — the sweep builder
//! documents that as the caller's problem, so the check lives here: the segment
//! curvature is sampled and a radius that reaches it is a loud error, naming the
//! two numbers (`sweep_profile_helix` refuses its own coils the same way).
//!
//! # Curved + hollow
//!
//! A hollow CURVED segment bores exactly, for the same reason the straight one
//! does: outer and inner sweep the same path through the same
//! rotation-minimizing frame at the same stations, so the bore is concentric and
//! the end caps are coincident annuli, including in assemblies with junctions.
//!
//! # Closed edges (rings)
//!
//! A CLOSED edge — a cylinder or cone rim, a full circle in a sketch — has no
//! two ends to hang a segment between: its chord is zero, and the ball-and-stick
//! graph would read it as a self-loop at one node of degree 2 and seed a joint
//! ball there. It is a different construction, so it takes a different lane: the
//! circle is swept once round the edge by `sweep_profile_along_chain`, the
//! closed-path builder Path Sweep's `pathAlign` drives, which closes the loft on
//! itself and caps nothing. A ring is ONE body with its two side halves and no
//! caps, and a hollow ring is the outer ring minus the concentric inner one.
//!
//! Each ring is its OWN body: it never joins the ball-and-stick assembly, so a
//! ring selected together with open edges that touch it comes back as a
//! separate solid alongside theirs. Selecting a rim that the tube cannot tell
//! from a zero-length edge is what the report of 2026-10-05 was: *"tube should
//! generate even with a closed loop path"* — the rim of an offset-shelled cone,
//! refused as "path has a zero-length edge".

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{
    extrude_profile_brep, make_arc, make_cylinder_brep, make_line, make_sphere_brep_framed,
    revolve_profile_brep_named, sew_solid, sweep_profile_along_chain_with_stations_reported,
    sweep_profile_along_path, AnalyticSurface, BooleanOperation, BooleanOptions, BrepSolid,
    NurbsCurve, Vec3,
};
use std::collections::{HashMap, HashSet};

/// Segment / dedup tolerance.
const EPS: f64 = 1e-8;

/// One path edge, resolved: its end vertices, and — when it is CURVED — the
/// curve the sweep lane drives the circle along. A straight segment carries no
/// curve, because the cylinder lane needs only the two ends; which lane a segment
/// takes is therefore the PRESENCE of the curve rather than a separate flag that
/// could disagree with it.
struct Segment {
    start: Vec3,
    end: Vec3,
    /// `None` → straight (cylinder lane); `Some(curve)` → curved (sweep lane).
    curve: Option<NurbsCurve>,
}

/// One path edge, classified: an OPEN edge takes its place in the ball-and-stick
/// graph as a [`Segment`]; a CLOSED one (see "Closed edges" in the module doc) is
/// swept on its own as a capless ring.
enum PathEdge {
    Open(Segment),
    Ring(NurbsCurve),
}

impl Segment {
    fn ends(&self) -> (Vec3, Vec3) {
        (self.start, self.end)
    }

    /// Unit direction the segment LEAVES its `from_start` end in: the curve's
    /// tangent there for a swept arm, the chord for a cylinder one. `None` when
    /// the tangent degenerates (a zero-speed parameter), which callers read as
    /// "assume the worst".
    fn direction_at(&self, from_start: bool) -> Option<Vec3> {
        let raw = match &self.curve {
            None => {
                if from_start {
                    self.end.sub(self.start)
                } else {
                    self.start.sub(self.end)
                }
            }
            Some(curve) => {
                let [t0, t1] = curve.domain().ok()?;
                let t = if from_start { t0 } else { t1 };
                let tangent = curve.derivatives(t, 1).ok()?[1];
                if from_start {
                    tangent
                } else {
                    tangent.scale(-1.0)
                }
            }
        };
        raw.normalized().ok()
    }
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

thread_local! {
    /// The station lane's measured bounds (`sweep.stations`) for the swept
    /// bends of the component being assembled: the sweep runs several calls
    /// deep under the joint-clearance ladder, so each attempt clears this and
    /// the feature takes what the accepted attempt left. A side channel like
    /// `take_blend_notes`, scoped to this thread.
    static SWEEP_REPORTS: std::cell::RefCell<Vec<crate::Approximation>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn take_sweep_reports() -> Vec<crate::Approximation> {
    SWEEP_REPORTS.with(|reports| std::mem::take(&mut *reports.borrow_mut()))
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let radius = ctx.number("radius")?;
    if !(radius > 0.0) {
        return Err("tube: requires a positive radius".into());
    }
    let bend = common::number_or_default(ctx, "bendRadius", 0.0);
    if bend < 0.0 {
        return Err("tube: bend radius cannot be negative".into());
    }
    if bend > 0.0 && bend <= radius {
        return Err(format!(
            "tube: a bend radius of {bend} is not larger than the tube radius {radius}; the \
             section would sweep back through itself at the corner"
        ).into());
    }
    let inner = common::number_or_default(ctx, "innerRadius", 0.0);
    if inner < 0.0 {
        return Err("tube: inside radius cannot be negative".into());
    }
    if inner > 0.0 && inner >= radius {
        return Err("tube: inside radius must be smaller than the outer radius".into());
    }

    // Resolve every input reference to INDIVIDUAL edge curves (one segment per
    // edge), then classify each: STRAIGHT ones become cylinders, CURVED ones
    // sweep the circle along themselves.
    // A CLOSED edge is a ring of its own (see "Closed edges"): it is kept out of
    // the graph, where its one node would read as a degree-2 junction.
    let curves = resolve_path_edges(ctx)?;
    let mut segments: Vec<Segment> = Vec::with_capacity(curves.len());
    let mut rings: Vec<NurbsCurve> = Vec::new();
    for curve in &curves {
        match classify_edge(curve, radius)? {
            PathEdge::Open(segment) => segments.push(segment),
            PathEdge::Ring(curve) => rings.push(curve),
        }
    }

    // Junction graph: merge shared endpoints into nodes; group edges into
    // connected components.
    let (nodes, edge_nodes) = build_graph(&segments);
    let components = connected_components(&edge_nodes, nodes.len());

    let name = if ctx.id.is_empty() { "Tube" } else { ctx.id.as_str() };
    let multi = components.len() + rings.len() > 1;
    let body_name = |index: usize| {
        if multi {
            format!("{name}[{index}]")
        } else {
            name.to_string()
        }
    };

    // One solid per connected component, then one per ring.
    let mut bodies: Vec<(String, BrepSolid)> =
        Vec::with_capacity(components.len() + rings.len());
    let mut approximations: Vec<crate::Approximation> = Vec::new();
    for (index, component) in components.iter().enumerate() {
        let body_name = body_name(index);
        let solid = assemble_component(
            &segments,
            component,
            &nodes,
            &edge_nodes,
            radius,
            inner,
            bend,
            &body_name,
        )?;
        approximations.extend(take_sweep_reports());
        bodies.push((body_name, solid));
    }
    for (offset, ring) in rings.iter().enumerate() {
        let body_name = body_name(components.len() + offset);
        let solid = assemble_ring(ring, radius, inner, &body_name)?;
        approximations.extend(take_sweep_reports());
        bodies.push((body_name, solid));
    }

    // NONE → each component as a separate added solid; a boolean set folds every
    // component sequentially into the targets.
    let mut result = common::finalize_solids(ctx, bodies);
    if result.error.is_none() {
        result.approximations.extend(approximations);
    }
    Ok(result)
}

/// One connected component, at the SMALLEST joint clearance that builds it
/// EXACTLY — see [`JOINT_CLEARANCE_LADDER`] for why the clearance is chosen per
/// model rather than fixed.
///
/// Every rung but the last is attempted on the exact boolean alone
/// (`boolean_operation_with_diagnostics`, no SoS perturbation): a rung that
/// needs the perturbation rescue has not earned its bead, because the rescue is
/// what shifts an operand bodily and sheds the cap crescents this clearance
/// exists to prevent. The LAST rung keeps the ordinary
/// [`crate::boolean_operation`] entry, rescue included, so nothing that builds
/// today stops building.
///
/// A hollow tube has to clear the SAME rung at both radii: the wall at a joint
/// is `k·(radius − innerRadius)`, so outer and inner sharing one `k` is what
/// keeps it uniform, and the inner assembly — the slenderer of the two — is
/// usually the one that sets the rung.
#[allow(clippy::too_many_arguments)]
fn assemble_component(
    segments: &[Segment],
    component: &[usize],
    nodes: &[Vec3],
    edge_nodes: &[(usize, usize)],
    radius: f64,
    inner: f64,
    bend: f64,
    name: &str,
) -> Result<BrepSolid, FeatureRefusal> {
    let mut last: Option<FeatureRefusal> = None;
    for (rung, &clearance) in JOINT_CLEARANCE_LADDER.iter().enumerate() {
        let exact_only = rung + 1 < JOINT_CLEARANCE_LADDER.len();
        let attempt = (|| {
            let _ = take_sweep_reports();
            let outer = build_assembly(
                segments, component, nodes, edge_nodes, radius, name, "", clearance, exact_only,
                bend,
            )?;
            if inner <= 0.0 {
                return Ok(outer);
            }
            let cutter = build_assembly(
                segments, component, nodes, edge_nodes, inner, name, "_Inner", clearance,
                exact_only, bend,
            )?;
            common::subtract_solid(outer, cutter)
                .map_err(|error| {
                    FeatureRefusal::from(error.with_message(|error| {
                        format!("tube: hollow subtract (outer − inner) failed: {error}")
                    }))
                })
        })();
        match attempt {
            Ok(solid) => return Ok(solid),
            Err(error) => last = Some(error),
        }
    }
    Err(last.unwrap_or_else(|| "tube: internal error — empty clearance ladder".into()))
}

/// One tube assembly along one connected component: ONE body per edge (a cylinder
/// for a straight edge, a swept tube for a curved one) + a sphere at every
/// junction node (degree ≥ 2).
///
/// Assembly ORDER is load-bearing for robustness. It is SPHERE-FIRST and
/// interleaved by a BFS: start from a junction's sphere, then union each incident
/// segment body (it pokes OUT of that sphere — a clean transversal crossing), and
/// as a body reaches a new junction, union that junction's sphere before its other
/// bodies arrive. Every union therefore overlaps the running solid, so the boolean
/// is never handed two DISJOINT bodies (untested multi-shell assembly).
///
/// Order alone does NOT keep two arms apart — the second arm still meets the
/// first's side face — so what makes that meeting well behaved is
/// the joint CLEARANCE: the joint ball is unioned first and trims both arms back
/// past their crotch tangency before they ever see each other.
#[allow(clippy::too_many_arguments)]
fn build_assembly(
    segments: &[Segment],
    component: &[usize],
    nodes: &[Vec3],
    edge_nodes: &[(usize, usize)],
    radius: f64,
    name: &str,
    suffix: &str,
    clearance: f64,
    exact_only: bool,
    bend: f64,
) -> Result<BrepSolid, FeatureRefusal> {
    if component.is_empty() {
        return Err("tube: internal error — empty component".into());
    }

    // Degrees within the component → the junction nodes (degree ≥ 2).
    let mut degree: HashMap<usize, usize> = HashMap::new();
    for &edge in component {
        let (a, b) = edge_nodes[edge];
        *degree.entry(a).or_default() += 1;
        *degree.entry(b).or_default() += 1;
    }
    // A node a chain sweeps THROUGH is not a joint: its ball is replaced by the
    // bend (see [`build_chains`]).
    let chains = build_chains(segments, component, edge_nodes, bend);
    let swept_through: HashSet<usize> = chains
        .iter()
        .flat_map(|chain| chain.nodes.iter().copied())
        .collect();
    let is_junction =
        |node: usize| degree.get(&node).copied().unwrap_or(0) >= 2 && !swept_through.contains(&node);

    // One chain and no joint left: the whole component is one swept body.
    if chains.len() == 1 {
        return Ok(named_chain_or_segment(
            &chains[0], segments, nodes, edge_nodes, radius, bend, name, suffix, 0,
        )?);
    }

    // Stable joint numbering per junction node.
    let mut junction_nodes: Vec<usize> = degree
        .iter()
        .filter(|(&node, &d)| d >= 2 && !swept_through.contains(&node))
        .map(|(&node, _)| node)
        .collect();
    junction_nodes.sort_unstable();
    let joint_index: HashMap<usize, usize> = junction_nodes
        .iter()
        .enumerate()
        .map(|(index, &node)| (node, index))
        .collect();

    // Each junction's ball radius: the tube radius at a straight cylindrical
    // join, `radius * clearance` for crotches and curved arms (see
    // [`JOINT_CLEARANCE_LADDER`] and [`node_needs_clearance`]).
    let joint_radius = |node: usize| {
        if node_needs_clearance(segments, component, edge_nodes, node) {
            radius * clearance
        } else {
            radius
        }
    };

    // Where each ball's seam and poles are parked, so neither survives into the
    // joint face (see [`joint_ball_frame`]).
    let joint_frame = |node: usize| {
        joint_ball_frame(
            segments,
            component,
            edge_nodes,
            nodes,
            node,
            radius,
            joint_radius(node),
        )
    };

    // A connected multi-edge component always has a junction. Seed with its sphere.
    let start = junction_nodes[0];
    let mut solid = named_sphere(
        nodes[start],
        joint_radius(start),
        name,
        suffix,
        joint_index[&start],
        joint_frame(start),
    )?;
    let mut sphered: HashSet<usize> = [start].into_iter().collect();
    let mut covered: HashSet<usize> = [start].into_iter().collect();
    let mut added: HashSet<usize> = HashSet::new();
    let mut segment_local = 0usize;

    loop {
        // Next chain that touches the covered frontier (guarantees overlap). A
        // chain's ENDS are where it meets the rest of the assembly; the nodes it
        // sweeps through are interior to its own body.
        let next = (0..chains.len()).find(|index| {
            !added.contains(index) && {
                let (a, b) = chains[*index].ends(edge_nodes);
                covered.contains(&a) || covered.contains(&b)
            }
        });
        let Some(index) = next else {
            break; // component fully assembled
        };
        added.insert(index);
        let (a, b) = chains[index].ends(edge_nodes);
        // Its body pokes out of the already-present junction sphere → clean.
        let body = named_chain_or_segment(
            &chains[index], segments, nodes, edge_nodes, radius, bend, name, suffix,
            segment_local,
        )?;
        segment_local += 1;
        solid = union_solid(solid, body, exact_only)?;
        covered.insert(a);
        covered.insert(b);
        // Any newly-reached JUNCTION gets its sphere now, while the cylinder that
        // reached it is the freshest body (the sphere absorbs its end cap).
        for node in [a, b] {
            if is_junction(node) && sphered.insert(node) {
                let sphere = named_sphere(
                    nodes[node],
                    joint_radius(node),
                    name,
                    suffix,
                    joint_index[&node],
                    joint_frame(node),
                )?;
                solid = union_solid(solid, sphere, exact_only)?;
            }
        }
    }
    Ok(solid)
}

/// One body for one [`Chain`]: the BEND lane when the chain actually turns
/// somewhere, and otherwise the lane the segment had before bends existed.
///
/// A chain of ONE segment with no corner is dispatched straight to
/// [`named_segment`], so a straight run is still an exact `make_cylinder_brep`
/// cylinder and a lone curved edge is still its own sweep. That matters beyond
/// byte-identity: the sweep lane emits a lofted NURBS wall, which no longer
/// RECOGNIZES as a cylinder, and everything downstream that keys on the analytic
/// carrier (closed-form intersections, STEP export, face queries) would silently
/// drop to the general path for a tube that never asked for a bend.
#[allow(clippy::too_many_arguments)]
fn named_chain_or_segment(
    chain: &Chain,
    segments: &[Segment],
    nodes: &[Vec3],
    edge_nodes: &[(usize, usize)],
    radius: f64,
    bend: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Result<BrepSolid, String> {
    if chain.segments.len() == 1 {
        return named_segment(&segments[chain.segments[0]], radius, name, suffix, local);
    }
    let (path, length) = chain_path(chain, segments, nodes, edge_nodes, bend)?;
    named_chain(&path, length, radius, bend, name, suffix, local)
}

/// A RUN of consecutive segments swept as ONE body, with a tangent arc at every
/// corner between them — the bend lane (see [`chain_path`]).
///
/// `segments` are edge indices in traversal order and `nodes[i]` is the node
/// between `segments[i]` and `segments[i + 1]`, so a chain of one segment has no
/// nodes and is just that segment.
struct Chain {
    segments: Vec<usize>,
    nodes: Vec<usize>,
}

impl Chain {
    /// The terminal nodes — where the chain meets a joint ball or a free end.
    fn ends(&self, edge_nodes: &[(usize, usize)]) -> (usize, usize) {
        let first = edge_nodes[self.segments[0]];
        let last = edge_nodes[*self.segments.last().expect("non-empty chain")];
        let start = match self.nodes.first() {
            Some(&next) if first.0 == next => first.1,
            _ => first.0,
        };
        let end = match self.nodes.last() {
            Some(&previous) if last.1 == previous => last.0,
            _ => last.1,
        };
        (start, end)
    }
}

/// Is this node a BEND — a corner the tube rounds off with a tangent arc and
/// sweeps through, instead of filling with a joint ball?
///
/// Only a node where exactly TWO STRAIGHT arms meet qualifies. Three arms have
/// no single corner to round; a curved arm arrives with its own tangent and the
/// arc that would meet it is not this construction (a bend between two general
/// curves is a different solve). Everything that is not a bend keeps the ball,
/// so the two lanes coexist inside one path.
fn node_is_bend(
    segments: &[Segment],
    component: &[usize],
    edge_nodes: &[(usize, usize)],
    node: usize,
    bend: f64,
) -> bool {
    if !(bend > 0.0) {
        return false;
    }
    let incident: Vec<usize> = component
        .iter()
        .copied()
        .filter(|&edge| {
            let (a, b) = edge_nodes[edge];
            a == node || b == node
        })
        .collect();
    incident.len() == 2 && incident.iter().all(|&edge| segments[edge].curve.is_none())
}

/// The component's segments grouped into [`Chain`]s: a maximal run of segments
/// joined at BEND nodes.
///
/// A CLOSED run — every node in the component a bend, so the walk comes back to
/// where it started — is not swept HERE. The builder does now sweep a closed
/// PLANAR path, as a capless ring, but a tube's closed run is a different
/// question: its cycle is a network of joints, the ring lane needs the whole loop
/// tangent-continuous and planar, and a tube's corners are neither in general. So
/// the cycle is opened the way it always was, and the two beads below are the
/// price. Making a planar, all-bends cycle one ring instead is open work.
///
/// It takes TWO breaks to open a cycle, not one. Demoting a single node still
/// leaves a chain that starts and ends at that same node — the path is closed
/// even though the walk stopped there — so two nodes are demoted back to joint
/// balls: the lowest index, and the one nearest half way round the cycle from
/// it, which splits the loop into two chains of roughly equal length. Those two
/// balls are the beads a closed loop keeps; every other corner is a bend.
fn build_chains(
    segments: &[Segment],
    component: &[usize],
    edge_nodes: &[(usize, usize)],
    bend: f64,
) -> Vec<Chain> {
    let mut bends: HashSet<usize> = HashSet::new();
    for &edge in component {
        let (a, b) = edge_nodes[edge];
        for node in [a, b] {
            if node_is_bend(segments, component, edge_nodes, node, bend) {
                bends.insert(node);
            }
        }
    }
    // A run with no non-bend node anywhere is closed; open it at one node.
    let touched: HashSet<usize> = component
        .iter()
        .flat_map(|&edge| {
            let (a, b) = edge_nodes[edge];
            [a, b]
        })
        .collect();
    if !touched.is_empty() && touched.iter().all(|node| bends.contains(node)) {
        let first = touched.iter().copied().min().expect("non-empty component");
        // Walk the cycle from `first` to order its nodes, then break at `first`
        // and at the node half way round.
        let mut order: Vec<usize> = vec![first];
        let mut visited: HashSet<usize> = [first].into_iter().collect();
        let mut walked: HashSet<usize> = HashSet::new();
        let mut node = first;
        while let Some(edge) = component.iter().copied().find(|&edge| {
            if walked.contains(&edge) {
                return false;
            }
            let (a, b) = edge_nodes[edge];
            a == node || b == node
        }) {
            walked.insert(edge);
            let (a, b) = edge_nodes[edge];
            node = if a == node { b } else { a };
            if !visited.insert(node) {
                break;
            }
            order.push(node);
        }
        bends.remove(&first);
        if let Some(&opposite) = order.get(order.len() / 2) {
            bends.remove(&opposite);
        }
    }

    let next_at = |node: usize, from: usize| -> Option<usize> {
        component.iter().copied().find(|&edge| {
            if edge == from {
                return false;
            }
            let (a, b) = edge_nodes[edge];
            a == node || b == node
        })
    };
    let mut used: HashSet<usize> = HashSet::new();
    let mut chains: Vec<Chain> = Vec::new();
    for &seed in component {
        if used.contains(&seed) {
            continue;
        }
        let mut chain_segments = vec![seed];
        let mut chain_nodes: Vec<usize> = Vec::new();
        used.insert(seed);
        // Walk forward from each end of the seed through bend nodes.
        for forward in [true, false] {
            let (a, b) = edge_nodes[seed];
            let mut node = if forward { b } else { a };
            let mut from = seed;
            while bends.contains(&node) {
                let Some(next) = next_at(node, from) else { break };
                if used.contains(&next) {
                    break;
                }
                used.insert(next);
                if forward {
                    chain_segments.push(next);
                    chain_nodes.push(node);
                } else {
                    chain_segments.insert(0, next);
                    chain_nodes.insert(0, node);
                }
                let (na, nb) = edge_nodes[next];
                node = if na == node { nb } else { na };
                from = next;
            }
        }
        chains.push(Chain {
            segments: chain_segments,
            nodes: chain_nodes,
        });
    }
    chains
}

/// The chain's G1 PATH: a straight piece per segment, shortened at each bend by
/// that corner's setback, and a tangent arc across every bend.
///
/// For two arms leaving a node at angle `γ` with a bend radius `R`, the arc
/// tangent to both lines touches them at `t = R / tan(γ/2)` from the node and is
/// centred on the bisector at `R / sin(γ/2)`; the path turns by `π − γ`. Both
/// tangent points are exact, so the joints are tangent-continuous to rounding
/// and the sweep's own joint check passes on construction rather than on luck.
///
/// A corner is REFUSED, naming its numbers, when the two setbacks do not fit the
/// segment between them — the arcs would cross and the swept solid would fold
/// through itself. `sweep_profile_along_path` documents a self-intersecting
/// result as the caller's problem, so the caller checks.
fn chain_path(
    chain: &Chain,
    segments: &[Segment],
    nodes: &[Vec3],
    edge_nodes: &[(usize, usize)],
    bend: f64,
) -> Result<(Vec<NurbsCurve>, f64), String> {
    // Per segment, how far each end is pulled back by its corner.
    let mut setback: Vec<(f64, f64)> = vec![(0.0, 0.0); chain.segments.len()];
    let mut geometry: Vec<Option<(Vec3, Vec3, Vec3, f64)>> = vec![None; chain.nodes.len()];
    for (index, &node) in chain.nodes.iter().enumerate() {
        let before = chain.segments[index];
        let after = chain.segments[index + 1];
        let arm = |edge: usize| -> Result<Vec3, String> {
            let (a, b) = edge_nodes[edge];
            let from_start = if a == node {
                true
            } else if b == node {
                false
            } else {
                return Err("tube: internal error — chain node is not on its segment".into());
            };
            segments[edge]
                .direction_at(from_start)
                .ok_or_else(|| "tube: a path edge has no usable direction at a corner".to_string())
        };
        let (first, second) = (arm(before)?, arm(after)?);
        let angle = first.cross(second).length().atan2(first.dot(second));
        let turn = std::f64::consts::PI - angle;
        if turn.abs() <= 1e-9 {
            continue; // collinear: no corner to round
        }
        if angle <= 1e-9 {
            return Err(format!(
                "tube: the path doubles back on itself at ({:.4}, {:.4}, {:.4}); a bend needs                  two distinct directions",
                nodes[node].x, nodes[node].y, nodes[node].z
            ).into());
        }
        let half = angle / 2.0;
        let distance = bend / half.tan();
        let Ok(bisector) = first.add(second).normalized() else {
            continue;
        };
        let centre = nodes[node].add(bisector.scale(bend / half.sin()));
        geometry[index] = Some((
            nodes[node].add(first.scale(distance)),
            nodes[node].add(second.scale(distance)),
            centre,
            turn,
        ));
        setback[index].1 = distance;
        setback[index + 1].0 = distance;
    }

    let mut path: Vec<NurbsCurve> = Vec::new();
    let mut length = 0.0;
    for (index, &edge) in chain.segments.iter().enumerate() {
        // The segment, oriented the way the chain walks it.
        let (a, b) = edge_nodes[edge];
        let entering = if index == 0 {
            match chain.nodes.first() {
                Some(&next) if a == next => b,
                _ => a,
            }
        } else {
            chain.nodes[index - 1]
        };
        let (from, to) = if entering == a {
            (nodes[a], nodes[b])
        } else {
            (nodes[b], nodes[a])
        };
        let along = to.sub(from);
        let span = along.length();
        let (head, tail) = setback[index];
        if head + tail >= span - EPS {
            return Err(format!(
                "tube: a bend radius of {bend} needs {:.6} of the {:.6} long edge at its \
                 corners; use a smaller bend radius, or 0 to join with a ball instead",
                head + tail,
                span
            ).into());
        }
        let direction = along.normalized().map_err(|_| {
            "tube: degenerate (zero-length) path edge".to_string()
        })?;
        let start = from.add(direction.scale(head));
        let end = to.sub(direction.scale(tail));
        path.push(make_line(start, end)?);
        length += end.sub(start).length();
        if let Some((_, _, centre, turn)) = geometry.get(index).copied().flatten() {
            let entry = path
                .last()
                .expect("a line was just pushed")
                .evaluate(1.0)
                .map_err(|error| format!("tube: bend entry: {error}"))?;
            let leaving = geometry[index]
                .map(|(_, exit, _, _)| exit)
                .expect("bend geometry present");
            let ex = entry.sub(centre).normalized().map_err(|_| {
                "tube: a bend's tangent point coincides with its centre".to_string()
            })?;
            let radial = leaving.sub(centre);
            let ey = radial
                .sub(ex.scale(radial.dot(ex)))
                .normalized()
                .map_err(|_| "tube: a bend's two tangent points are collinear".to_string())?;
            path.push(make_arc(centre, ex, ey, bend, 0.0, turn)?);
            length += bend * turn;
        }
    }
    Ok((path, length))
}

/// The chain's PIECES as exact analytic solids, ready to be sewn: an extruded
/// cylinder per straight run and a revolved torus sector per bend.
///
/// Both builders take the SAME two-arc section, and that is load-bearing rather
/// than tidy. `revolve_profile_brep_named` refuses a single closed curve
/// ("profile needs at least 2 curves"), so a bend always has two side faces and
/// its rims are two half-circle edges meeting at two vertices — while
/// `make_cylinder_brep`'s rim is ONE closed edge with a single seam vertex.
/// Those two rims cannot pair, and `sew_solid` reports `edges_sewn: 0` and
/// leaves the run open. Extruding the legs from the same section gives them the
/// bend's topology exactly, and every rim pairs.
fn chain_pieces(
    path: &[NurbsCurve],
    radius: f64,
) -> Result<Vec<BrepSolid>, String> {
    use std::f64::consts::{PI, TAU};
    let mut pieces = Vec::with_capacity(path.len());
    for (index, curve) in path.iter().enumerate() {
        let [t0, t1] = curve.domain()?;
        let start = curve.evaluate(t0)?;
        let end = curve.evaluate(t1)?;
        // Read as the one-sided limit where the derivative vanishes: a
        // stationary END is still a direction of travel, and the straight-or-
        // bent test below compares directions, not derivatives.
        let tangent = curve
            .unit_tangent(t0, t0, t1)
            .map_err(|error| format!("tube: path piece {index}: {error}"))?;
        // The section normal to the path at its start: any radial direction
        // perpendicular to the tangent will do, and the arc's own plane fixes
        // the rest.
        let radial = tangent.perpendicular()?;
        let binormal = tangent.cross(radial).normalized()?;
        let section = vec![
            make_arc(start, radial, binormal, radius, 0.0, PI)?,
            make_arc(start, radial, binormal, radius, PI, TAU)?,
        ];
        // Straight or curved? A line's end tangent equals its start tangent.
        let end_tangent = curve
            .unit_tangent(t1, t0, t1)
            .map_err(|error| format!("tube: path piece {index}: {error}"))?;
        if end_tangent.sub(tangent).length() <= 1e-9 {
            pieces.push(extrude_profile_brep(&section, tangent, end.sub(start).length())?);
            continue;
        }
        // A bend: revolve the section about the arc's own axis, through the
        // angle its tangents turn by. Both come from the curve rather than from
        // the corner that produced it, so this reads the geometry it is given.
        let turn = tangent.cross(end_tangent).length().atan2(tangent.dot(end_tangent));
        let axis = tangent.cross(end_tangent).normalized()?;
        let chord = end.sub(start);
        // The centre is where the two end normals meet: along the in-plane
        // normal of the start tangent, at the arc's radius.
        let inward = axis.cross(tangent).normalized()?;
        let bend_radius = chord.length() / (2.0 * (turn / 2.0).sin());
        let centre = start.add(inward.scale(bend_radius));
        pieces.push(revolve_profile_brep_named(&section, centre, axis, turn, &[], &[])?);
    }
    Ok(pieces)
}

/// Concatenate the pieces into ONE shell with the interior caps removed, so the
/// rims that abut become one-use edges for [`sew_solid`] to pair.
///
/// A cap is INTERIOR when every vertex of its rim sits at the tube radius from a
/// point where two pieces meet. That is a geometric test rather than an index
/// into the builder's face order, because the extrude and the revolve do not
/// emit their caps in the same position and an index would silently pick the
/// wrong face when one of them changes.
fn stitch_pieces(pieces: &[BrepSolid], joints: &[Vec3], radius: f64) -> BrepSolid {
    let interior = |solid: &BrepSolid, face: &crate::FaceRecord| -> bool {
        if !matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
            return false;
        }
        joints.iter().any(|point| {
            face.loops.iter().flat_map(|entry| &entry.coedges).all(|coedge| {
                solid
                    .edges
                    .iter()
                    .find(|edge| edge.id == coedge.edge_id)
                    .and_then(|edge| solid.vertices.iter().find(|v| v.id == edge.start_vertex_id))
                    .is_some_and(|vertex| (vertex.point.sub(*point).length() - radius).abs() < 1e-6)
            })
        })
    };
    let mut out = pieces[0].clone();
    out.vertices.clear();
    out.edges.clear();
    out.shells.truncate(1);
    out.shells[0].faces.clear();
    out.genus = 0;
    let (mut vertex_base, mut edge_base, mut face_base) = (1u64, 1u64, 1u64);
    for piece in pieces {
        let vertex_max = piece.vertices.iter().map(|v| v.id).max().unwrap_or(0);
        let edge_max = piece.edges.iter().map(|e| e.id).max().unwrap_or(0);
        let mut face_max = 0u64;
        for vertex in &piece.vertices {
            let mut copy = vertex.clone();
            copy.id += vertex_base;
            out.vertices.push(copy);
        }
        for edge in &piece.edges {
            let mut copy = edge.clone();
            copy.id += edge_base;
            copy.start_vertex_id += vertex_base;
            copy.end_vertex_id += vertex_base;
            out.edges.push(copy);
        }
        for face in piece.shells.iter().flat_map(|shell| &shell.faces) {
            face_max = face_max.max(face.id);
            if interior(piece, face) {
                continue;
            }
            let mut copy = face.clone();
            copy.id += face_base;
            for entry in &mut copy.loops {
                entry.id += face_base;
                for coedge in &mut entry.coedges {
                    coedge.id += face_base;
                    coedge.edge_id += edge_base;
                }
            }
            out.shells[0].faces.push(copy);
        }
        vertex_base += vertex_max + 1;
        edge_base += edge_max + 1;
        face_base += face_max + 1;
    }
    out
}

/// One named body for a whole [`Chain`], built EXACTLY: each piece an analytic
/// solid, the interior caps dropped, and the rims SEWN rather than unioned.
///
/// Sewing is what makes the exact carriers reachable at all. The pieces meet
/// TANGENTIALLY — that is what G1 means — and a tangential contact is the class
/// `csg/imprint` declines by design, so a union of a cylinder and its own elbow
/// refuses ("unsupported singular/tangent-node surface intersection"). But the
/// pieces do not need intersecting: they already abut on coincident rims, so the
/// join is topological, and `sew_solid` pairs those rims and merges the shells.
///
/// Measured on a 90° elbow, r = 2, bend 6, arms 20: volume `470.293630006`
/// against the exact `470.293630015` — **-2.0e-11 relative**, five orders better
/// than the lofted lane's -5.7e-6 — with every carrier analytic
/// (`Revolution`, `Plane`) where the loft gives NURBS.
///
/// `None` means the sew did not close the run; the caller falls back to the
/// loft rather than shipping a solid with open edges.
fn sewn_chain(
    path: &[NurbsCurve],
    radius: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Option<BrepSolid> {
    let pieces = chain_pieces(path, radius).ok()?;
    if pieces.len() < 2 {
        return None;
    }
    // Where consecutive pieces meet.
    let mut joints = Vec::with_capacity(path.len().saturating_sub(1));
    for curve in path.iter().take(path.len() - 1) {
        let [_, t1] = curve.domain().ok()?;
        joints.push(curve.evaluate(t1).ok()?);
    }
    let stitched = stitch_pieces(&pieces, &joints, radius);
    let (mut sewn, report) = sew_solid(&stitched, 1e-7).ok()?;
    if report.open_edges_after != 0 || !report.oriented_outward || !sewn.validate().is_empty() {
        return None;
    }
    let side = format!("{name}{suffix}_Seg{local}_S");
    let names: Vec<String> = sewn
        .shells
        .first()?
        .faces
        .iter()
        .map(|face| {
            if matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
                String::new()
            } else {
                side.clone()
            }
        })
        .collect();
    // The two free-end caps, in the order they appear.
    let mut cap = 0usize;
    let names: Vec<String> = names
        .into_iter()
        .map(|entry| {
            if !entry.is_empty() {
                return entry;
            }
            cap += 1;
            if cap == 1 {
                format!("{name}{suffix}_Seg{local}_B")
            } else {
                format!("{name}{suffix}_Seg{local}_T")
            }
        })
        .collect();
    name_faces(&mut sewn, &names);
    Some(sewn)
}

/// One named body for a whole [`Chain`]: the circle of the tube radius swept
/// along the chain's G1 path in ONE piece, so a corner is a real bend — no joint
/// ball, no boolean, no bead, and the wall is tangent-continuous through the
/// turn.
///
/// STATIONS are budgeted from the geometry rather than left at the sweep's
/// 32-station default. The budget is spread along the chain by ARC LENGTH, so a
/// long straight run would otherwise starve the arcs that actually need
/// sampling. Asking for 16 stations per quarter turn on a bend — the same
/// density the twisted builder uses — needs
/// `S·(R·θ)/L ≥ 16·θ/(π/2)`, i.e. `S ≥ 32·L/(π·R)`, which is independent of the
/// turn angle and so holds for every bend in the chain at once.
fn named_chain(
    path: &[NurbsCurve],
    length: f64,
    radius: f64,
    bend: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Result<BrepSolid, String> {
    use std::f64::consts::{PI, TAU};
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let origin = Vec3::default();
    let profile = [
        make_arc(origin, x, y, radius, 0.0, PI)?,
        make_arc(origin, x, y, radius, PI, TAU)?,
    ];
    // EXACT first: sewn analytic pieces. The loft below is the fallback for a
    // run the sew cannot close.
    if let Some(sewn) = sewn_chain(path, radius, name, suffix, local) {
        return Ok(sewn);
    }
    let stations = if bend > 0.0 {
        ((32.0 * length / (PI * bend)).ceil() as usize).clamp(32, 1024)
    } else {
        32
    };
    let piece_names: Vec<String> = (0..path.len())
        .map(|piece| format!("{name}{suffix}_Seg{local} piece {piece}"))
        .collect();
    // Classified where the chain is assembled. A bend run is built to be open and
    // tangent-continuous (the straight pieces meet their torus sectors at exact
    // tangent points); the classification is what the corner refusal reads when a
    // too-small bend radius leaves one that is not.
    let classified = crate::SweepPath::from_curves(path, &piece_names)?;
    let crate::SweptChain { solid: mut swept, report } = sweep_profile_along_chain_with_stations_reported(
        &profile,
        &classified,
        stations,
        "raise the tube's bend radius so the corner is rounded, or set it to 0 to \
         join the segments with a ball instead",
        None,
    )
    .map_err(|error| format!("tube: sweeping the section along a bent path failed: {error}"))?;
    // The station lane's measured bound, for the body this chain ends up in
    // (`take_sweep_reports` at the feature's exit).
    if let Some(report) = report {
        SWEEP_REPORTS.with(|reports| {
            reports.borrow_mut().push(report.approximation(format!("{name}{suffix} segment {local}")))
        });
    }
    let faces = face_count(&swept);
    if faces != profile.len() + 2 {
        return Err(format!(
            "tube: swept chain produced {faces} faces, expected {} ({} sides + 2 caps)",
            profile.len() + 2,
            profile.len()
        ).into());
    }
    let side = format!("{name}{suffix}_Seg{local}_S");
    name_faces(
        &mut swept,
        &[
            side.clone(),
            side,
            format!("{name}{suffix}_Seg{local}_B"),
            format!("{name}{suffix}_Seg{local}_T"),
        ],
    );
    Ok(swept)
}

/// How much wider than the tube the JOINT BALL is built where arms form a
/// crotch, tried SMALLEST FIRST — [`assemble_component`] keeps the first rung
/// whose assembly the EXACT boolean builds, so the bead is as small as the
/// kernel can actually carry for that model rather than a blanket constant.
///
/// Two equal-radius arms leaving a node in different directions are TANGENT to
/// each other at the two points `node ± radius·(â×b̂)/|â×b̂|` — the crotch, at
/// distance exactly `radius` from the node. A ball of exactly `radius` puts
/// those tangent points ON its own surface, so they survive into the assembly
/// and the arm-vs-arm face pair is a SINGULAR (tangent-node) intersection: the
/// exact boolean refuses it, the SoS perturbation lane rescues it by rigidly
/// shifting an arm by ~1e-3 of the model, and what the user sees is a joint
/// whose geometry is microns off and whose end-cap disc survives as a crescent
/// sliver — shredded into a dozen tiny faces and edges when the perturbation is
/// large next to the radius.
///
/// A ball a hair WIDER buries the crotch: the union trims each arm back to
/// `radius·√(k²−1)` from the node, so the tangent points are not on the arm
/// faces at all and every pair the boolean sees is a clean transversal crossing.
///
/// # Why a LADDER and not a number
///
/// The bead is `(k−1)·radius`, so a smaller `k` is a smaller bead — but the
/// clearance a joint NEEDS is not a constant of the shape class. It rises with
/// the arms' SLENDERNESS: the intersection features live at the scale of the
/// radius while the carrier's parameter domain spans the whole segment, so a
/// long thin arm resolves the same crossing more coarsely. Measured on a closed
/// square loop (four 90° corners, every arm junction-to-junction at both ends),
/// `BREP_NO_PERTURB=1` so only an exact build counts:
///
/// | radius | side | aspect | smallest k that builds exactly |
/// |---|---|---|---|
/// | 4 | 20 | 5 | ≤ 1.0005 |
/// | 2 | 20 | 10 | ≤ 1.0005 |
/// | 1 | 10 | 10 | ≤ 1.0005 |
/// | 1 | 20 | 20 | 1.007 |
/// | 0.5 | 20 | 40 | 1.007 |
/// | 1 | 50 | 50 | 1.007 |
/// | 1 | 100 | 100 | **1.02** |
/// | 1 | 200 | 200 | **1.1** |
///
/// So the 1% this used to spend everywhere was BOTH too much and too little: a
/// compact joint builds exactly at 0.05%, twenty times smaller, while an `r = 1`
/// loop on 100mm sides does not build exactly at 1% at all — it was reaching the
/// perturbation rescue, which is where the reported cap shards come from (that
/// row reads a spurious 17th vertex on a shape with 16).
///
/// The rungs bracket the measured floors with one step of margin each. The last
/// is the fallback: it is attempted with the perturbation rescue allowed, so a
/// shape that no rung builds exactly still behaves exactly as it did before.
///
/// The 90° elbow is exact against its closed form —
/// `2πr²L + 4πR³/3 − 4r³/3 − 2·(2π/3)(R³ − (R²−r²)^{3/2}) + |A∩B∩S|` — reading
/// **500.392402930 vs 500.392411971 at k = 1.001, −1.8e-8 relative**; at `k = 1`
/// the rescue's answer is `500.415251910` against the exact `500.365738317`,
/// **+9.9e-5 relative**, which is the perturbed geometry rather than the shape.
///
/// # `1.0` is the first rung, and usually the one taken
///
/// The table above was measured against the imprint's tangent-node refusal, and
/// that refusal was a FALSE POSITIVE for this class: the arm-vs-arm crotch is an
/// isolated Morse node the imprint can already carve, and the gate fired merely
/// because a clipped run's ENDPOINT sits on the trim rim with near-parallel
/// normals. With that fixed (`csg/imprint/tangent_contact.rs`), a ball of
/// EXACTLY the tube radius builds at every aspect from 5 to 200 — every corner
/// of the closed-loop series takes the first rung, `validate()` clean, genus 1.
/// **So the whole aspect dependence in that table was the gate, and a tube's
/// joint now carries no bead at all wherever k = 1 builds.** The higher rungs
/// stay as the fallback for anything that still refuses.
///
/// The one number that held this back, and where it went: those k = 1 loops read
/// 1.2e-6 BELOW the closed form `4πr²s − 16r³/3 + 4πr³/3`, five times this
/// feature's own accuracy floor. It is a QUADRATURE artifact on the joint patch,
/// not a shape error, and it is localized per face:
///
/// | face | closed form | measured | rel |
/// |---|---|---|---|
/// | arm wall `2πrs − 4r²` | 235.327412287 | 235.327420177 | +3.4e-8 |
/// | joint lune `2r²·(π/2)` | 12.566370614 | 12.566310213 | **−4.81e-6** |
///
/// The arm walls — the bulk of the solid — are right to 1e-7; every joint lune
/// is low by the same −4.81e-6, and the SAME deficit appears on an open path's
/// joint, which reads exact overall. The loop's total inherits it because a
/// face's volume contribution is `⅓∫p·n dA`, weighted by distance from the
/// origin, and a closed loop has no free end caps whose opposite-signed
/// contributions offset it.
///
/// The shape itself is sound where it matters: `solid_connectivity` reports one
/// shell, one component, no pinch vertices, and `solid_self_intersections`
/// returns **flagged = false with 0 confirmed out of 13,636 candidate pairs**.
///
/// Ruled out along the way, each the obvious first guess: a tangential ball trim
/// costs nothing on its own ([`collinear_joint_keeps_the_plain_radius_ball`]
/// measures `π·1²·40` to 1.6e-11 with an inscribed ball); it is not per-corner
/// (an open 5-segment path has the SAME four corners as the loop and reads
/// +3.7e-9); and it is not the arms being trimmed at both ends (an open path
/// with two such arms reads −2.8e-8).
///
/// A bead-free joint IS the first rung, 1.0. Two equal-radius arms are tangent to
/// each other at the crotch whatever ball is used, so an inscribed ball carries a
/// genuine tangent NODE there — and the imprint assembles that contact rather than
/// declining it: `csg/imprint/tangent_contact.rs` separates an ISOLATED node (two
/// transverse branches crossing, rank-2 `II_a − II_b`) from an EXTENDED contact
/// (rank-deficient), and only the latter is refused. The wider rungs stay for a
/// model that still refuses; a swept tangent-arc bend, which has no crotch at all,
/// is the other way out.
const JOINT_CLEARANCE_LADDER: [f64; 6] = [1.0, 1.0005, 1.002, 1.007, 1.02, 1.1];

/// Does this junction need clearance for a crotch or a curved arm?
///
/// A straight-through node (two collinear cylindrical arms) has none: the arms are coaxial,
/// their side surfaces are the SAME cylinder, and the exact boolean already
/// joins them without help. Widening the ball there would do nothing but raise a
/// bead on a straight pipe, so the clearance is spent only where the tangency it
/// buries actually exists.
fn node_needs_clearance(
    segments: &[Segment],
    component: &[usize],
    edge_nodes: &[(usize, usize)],
    node: usize,
) -> bool {
    // A tangent that cannot be evaluated is reported as a crotch — the clearance
    // is the safe answer.
    let Some(arms) = node_arms(segments, component, edge_nodes, node) else {
        return true;
    };
    // Opposite endpoint tangents do not make a curved arm cylindrical. A ball
    // of exactly the tube radius still touches the swept skin at the joint,
    // forcing a perturbation that can misalign outer and inner assemblies.
    if arms.iter().any(|&(edge, _)| segments[edge].curve.is_some()) {
        return true;
    }
    for (index, &(_, a)) in arms.iter().enumerate() {
        for &(_, b) in &arms[index + 1..] {
            // Exactly opposite (a straight run) is the only crotch-free pair.
            if a.dot(b) > -1.0 + 1e-9 {
                return true;
            }
        }
    }
    false
}

/// Every arm at `node` as `(segment index, outgoing unit direction)` — the
/// curve's tangent there for a swept arm, the chord for a cylinder one. `None`
/// when any arm's tangent degenerates (a zero-speed parameter), which callers
/// read as "assume the worst".
fn node_arms(
    segments: &[Segment],
    component: &[usize],
    edge_nodes: &[(usize, usize)],
    node: usize,
) -> Option<Vec<(usize, Vec3)>> {
    let mut arms: Vec<(usize, Vec3)> = Vec::new();
    for &edge in component {
        let (a, b) = edge_nodes[edge];
        let from_start = if a == node {
            true
        } else if b == node {
            false
        } else {
            continue;
        };
        arms.push((edge, segments[edge].direction_at(from_start)?));
    }
    Some(arms)
}

/// The joint ball's PARAMETRIC FRAME — `(polar axis, seam direction)` — chosen so
/// that both poles and the whole `u = 0` seam meridian lie INSIDE the arms that
/// are about to trim the ball away.
///
/// A sphere carries one seam edge pole to pole and one degenerate edge at each
/// pole, and the union keeps whichever of them fall in the surviving region. A
/// surviving seam is an edge with the SAME face on both sides — a line drawn
/// across the joint ball that borders nothing, and a stray vertex at the pole
/// (reported 2026-09-09, "spherical face gets internal edge bordering a single
/// face"). The ball is the kernel's own scaffolding, so the kernel gets to pick
/// where its seam runs; nothing downstream depends on the joint's `u = 0`.
///
/// The construction is a PAIR of arms. For two arms `a`, `b` with angle `γ`
/// between them, `axis = â − b̂` and `seam = â + b̂` are orthogonal, and the seam
/// meridian is the half great circle `−axis → b̂ → seam → â → +axis` — it runs
/// THROUGH both arms, so it is buried for its whole length whenever the widest
/// gap it leaves, `max(γ/2, (π−γ)/2)`, stays inside the arm. The pair whose
/// angle is nearest a right angle leaves the smallest gap, so it is tried first;
/// the acceptance itself is [`seam_is_buried`], measured against the segment
/// geometry rather than predicted from the angle.
///
/// `None` — no pair hides it — keeps the default `+Z` frame and the seam that
/// comes with it. That is the honest answer for a shallow bend, where the two
/// arms leave an ANNULUS of ball standing (a band around the crotch plane) and
/// no spherical parametrization of an annulus is seam-free, and for a
/// straight-through node, where `â + b̂` vanishes and the ball is buried whole.
fn joint_ball_frame(
    segments: &[Segment],
    component: &[usize],
    edge_nodes: &[(usize, usize)],
    nodes: &[Vec3],
    node: usize,
    radius: f64,
    ball_radius: f64,
) -> Option<(Vec3, Vec3)> {
    let arms = node_arms(segments, component, edge_nodes, node)?;
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (index, &(_, a)) in arms.iter().enumerate() {
        for (offset, &(_, b)) in arms[index + 1..].iter().enumerate() {
            let angle = a.cross(b).length().atan2(a.dot(b));
            pairs.push(((angle - std::f64::consts::FRAC_PI_2).abs(), index, index + 1 + offset));
        }
    }
    // Nearest to a right angle first; the index tie-break keeps the choice
    // deterministic when two pairs are equally square.
    pairs.sort_by(|left, right| {
        left.0
            .partial_cmp(&right.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(left.1.cmp(&right.1))
            .then(left.2.cmp(&right.2))
    });
    for (_, left, right) in pairs {
        let (_, a) = arms[left];
        let (_, b) = arms[right];
        let (Ok(axis), Ok(seam)) = (a.sub(b).normalized(), a.add(b).normalized()) else {
            continue; // exactly opposite arms name no meridian
        };
        if seam_is_buried(segments, &arms, nodes[node], ball_radius, radius, axis, seam) {
            return Some((axis, seam));
        }
    }
    None
}

/// Is every point of the seam meridian — poles included — inside one of the arms
/// that will trim this ball, with room to spare?
///
/// Sampled rather than reasoned: a swept (curved) arm bends away from its own
/// node tangent, so the angle a straight-arm argument would allow is not the
/// angle this ball actually gets. [`SEAM_BURIAL_MARGIN`] is what "with room to
/// spare" means — a seam point that sat exactly on an arm's surface would put a
/// pole ON the sphere/cylinder intersection curve, which is the singular
/// configuration the joint clearance exists to avoid.
fn seam_is_buried(
    segments: &[Segment],
    arms: &[(usize, Vec3)],
    center: Vec3,
    ball_radius: f64,
    radius: f64,
    axis: Vec3,
    seam: Vec3,
) -> bool {
    /// Stations along the half meridian, endpoints (the two poles) included.
    const STATIONS: usize = 64;
    let limit = radius * SEAM_BURIAL_MARGIN;
    (0..=STATIONS).all(|station| {
        let angle = std::f64::consts::PI * (station as f64 / STATIONS as f64 - 0.5);
        let point = center.add(
            seam.scale(ball_radius * angle.cos())
                .add(axis.scale(ball_radius * angle.sin())),
        );
        arms.iter()
            .any(|&(edge, _)| distance_to_arm_axis(&segments[edge], point) < limit)
    })
}

/// How far inside an arm a seam point has to sit, as a fraction of the tube
/// radius. The gap the geometry leaves is large where a pair is anywhere near
/// square (0.71 of the radius at 90°), so this band costs nothing there; what it
/// buys is the refusal of the SHALLOW pairs, whose seam grazes the arm surface —
/// against a straight arm 0.95 accepts `sin(gap) < 0.95/k` — between 37° and
/// 143° apart at the smallest rung — and a shallower pair is where the two caps
/// stop overlapping
/// enough to hide a meridian and the surviving ball turns into a band.
const SEAM_BURIAL_MARGIN: f64 = 0.95;

/// Distance from `point` to an arm's AXIS — the segment's chord for a cylinder
/// arm, its path curve for a swept one. The curved lane samples the curve and
/// takes the nearest sample, which can only OVERSTATE the distance, so it errs
/// towards keeping the default frame.
fn distance_to_arm_axis(segment: &Segment, point: Vec3) -> f64 {
    /// Samples along a curved arm's path.
    const CURVE_SAMPLES: usize = 128;
    match &segment.curve {
        None => {
            let (start, end) = segment.ends();
            let along = end.sub(start);
            let length_squared = along.dot(along);
            let t = if length_squared > EPS {
                (point.sub(start).dot(along) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            point.sub(start.add(along.scale(t))).length()
        }
        Some(curve) => {
            let Ok([t0, t1]) = curve.domain() else {
                return f64::INFINITY;
            };
            (0..=CURVE_SAMPLES)
                .filter_map(|station| {
                    let t = t0 + (t1 - t0) * station as f64 / CURVE_SAMPLES as f64;
                    curve.evaluate(t).ok().map(|p| point.sub(p).length())
                })
                .fold(f64::INFINITY, f64::min)
        }
    }
}

/// One named body for one path edge: the CYLINDER lane for a straight edge, the
/// SWEEP lane for a curved one. Both emit the same name vocabulary — sides `_S`,
/// start cap `_B`, end cap `_T` — so a segment's faces are referred to the same
/// way whichever lane built it.
fn named_segment(
    segment: &Segment,
    radius: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Result<BrepSolid, String> {
    match &segment.curve {
        None => named_cylinder(segment.ends(), radius, name, suffix, local),
        Some(curve) => named_swept(curve, radius, name, suffix, local),
    }
}

/// One named cylinder segment for edge endpoints `(a, b)`.
fn named_cylinder(
    (a, b): (Vec3, Vec3),
    radius: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Result<BrepSolid, String> {
    let axis = b.sub(a);
    let length = axis.length();
    if !(length > EPS) {
        return Err("tube: degenerate (zero-length) path edge".into());
    }
    let mut cylinder = make_cylinder_brep(a, axis, radius, length)?;
    name_faces(
        &mut cylinder,
        &[
            format!("{name}{suffix}_Seg{local}_S"),
            format!("{name}{suffix}_Seg{local}_B"),
            format!("{name}{suffix}_Seg{local}_T"),
        ],
    );
    Ok(cylinder)
}

/// One named CURVED segment: a circle of the tube radius swept along the edge by
/// the path sweep's own builder, at its own station budget (32, raised along a
/// path that turns further than a full turn, such as a helix) — so a tube and a
/// Path Sweep of the same circle along the same edge are the same solid.
///
/// The circle is authored as two half arcs about the ORIGIN in XY. Where it sits
/// is irrelevant: `sweep_profile_along_path` TRANSPLANTS the loop onto the path's
/// rotation-minimizing frame, re-centring it on the path at every station.
///
/// Face order out of the loft is `[side per profile curve…, START cap, END cap]`,
/// i.e. four faces for a two-arc circle. That count is ASSERTED rather than
/// zipped away: `name_faces` silently tolerates a mismatch, and a segment whose
/// sides went unnamed would be a reference that vanished, not a visible fault.
fn named_swept(
    curve: &NurbsCurve,
    radius: f64,
    name: &str,
    suffix: &str,
    local: usize,
) -> Result<BrepSolid, String> {
    use std::f64::consts::{PI, TAU};
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let origin = Vec3::default();
    let profile = [
        make_arc(origin, x, y, radius, 0.0, PI)?,
        make_arc(origin, x, y, radius, PI, TAU)?,
    ];
    let mut swept = sweep_profile_along_path(&profile, curve, None)
        .map_err(|error| format!("tube: sweeping the section along a curved edge failed: {error}"))?;
    let faces = face_count(&swept);
    if faces != profile.len() + 2 {
        return Err(format!(
            "tube: swept curved segment produced {faces} faces, expected {} ({} sides + 2 caps)",
            profile.len() + 2,
            profile.len()
        ).into());
    }
    let side = format!("{name}{suffix}_Seg{local}_S");
    name_faces(
        &mut swept,
        &[
            side.clone(),
            side,
            format!("{name}{suffix}_Seg{local}_B"),
            format!("{name}{suffix}_Seg{local}_T"),
        ],
    );
    Ok(swept)
}

/// One named junction sphere (all its faces share the `Joint{k}` name).
///
/// `frame` is `(polar axis, seam direction)` from [`joint_ball_frame`] — the
/// parametrization that hides the ball's seam and poles inside the arms. `None`
/// keeps the default `+Z` axis and its canonical seam.
fn named_sphere(
    center: Vec3,
    radius: f64,
    name: &str,
    suffix: &str,
    joint: usize,
    frame: Option<(Vec3, Vec3)>,
) -> Result<BrepSolid, String> {
    let (axis, seam) = match frame {
        Some((axis, seam)) => (axis, Some(seam)),
        None => (Vec3::new(0.0, 0.0, 1.0), None),
    };
    let mut sphere = make_sphere_brep_framed(center, radius, axis, seam)?;
    let joint_name = format!("{name}{suffix}_Joint{joint}");
    let names: Vec<String> = (0..face_count(&sphere)).map(|_| joint_name.clone()).collect();
    name_faces(&mut sphere, &names);
    Ok(sphere)
}

/// One CLOSED edge as a ring body, hollowed when `inner > 0` — the closed-edge
/// counterpart of [`assemble_component`]. No clearance ladder: a ring has no
/// joint, so there is no ball to size.
///
/// The hollow subtract is a body entirely inside another with no shared face —
/// the result is the outer wall plus an inner void shell. The boolean owns that
/// case; a failure there is reported by name rather than as a raw boolean error.
///
/// Every refusal the lane's builders raise is TYPED ([`crate::KernelRefusal`])
/// and is passed on as such: the tube prefixes the text and keeps the class and
/// stage, so a sweep refusal reaches `FeatureResult.refusal` with the class it
/// was raised with rather than flattened to text.
fn assemble_ring(
    curve: &NurbsCurve,
    radius: f64,
    inner: f64,
    name: &str,
) -> Result<BrepSolid, FeatureRefusal> {
    let _ = take_sweep_reports();
    let outer = named_ring(curve, radius, name, "")?;
    if inner <= 0.0 {
        return Ok(outer);
    }
    let cutter = named_ring(curve, inner, name, "_Inner")?;
    common::subtract_solid(outer, cutter).map_err(|error| {
        FeatureRefusal::from(error.with_message(|error| {
            format!("tube: hollow subtract (outer − inner) of a closed-path ring failed: {error}")
        }))
    })
}

/// One named RING: the tube circle swept once round a CLOSED edge by the
/// closed-chain sweep, which closes the loft on itself instead of capping it.
///
/// The same two-half-arc profile about the origin as [`named_swept`], under the
/// same `Transplant` placement, so a ring's wall is the wall an open arc of the
/// same edge would have. Face order out of the closed loft is `[side per profile
/// curve…]` and nothing else — a ring has no caps — which is ASSERTED for the
/// same reason [`named_swept`] asserts its own count. Both sides carry the
/// `_Seg0_S` name an open segment's sides do.
///
/// The path classification and the sweep refuse with a typed
/// [`crate::KernelRefusal`]; both are passed on with the tube's prefix on the
/// text and the class and stage untouched (`with_message`), never through a
/// `String`. The two consistency checks of this function's own are text.
fn named_ring(
    curve: &NurbsCurve,
    radius: f64,
    name: &str,
    suffix: &str,
) -> Result<BrepSolid, FeatureRefusal> {
    use std::f64::consts::{PI, TAU};
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let origin = Vec3::default();
    let profile = [
        make_arc(origin, x, y, radius, 0.0, PI)?,
        make_arc(origin, x, y, radius, PI, TAU)?,
    ];
    let segment_name = format!("{name}{suffix}_Seg0");
    let classified = crate::SweepPath::from_curves(std::slice::from_ref(curve), &[segment_name])
        .map_err(|refusal| {
            refusal.with_message(|message| {
                format!("tube: classifying the closed edge as a sweep path failed: {message}")
            })
        })?;
    if !classified.closed {
        return Err(
            "tube: internal error — an edge classified as closed is not a closed sweep path".into(),
        );
    }
    let crate::SweptChain { solid: mut swept, report } = crate::sweep_profile_along_chain_reported(
        &profile,
        &classified,
        0.0,
        None,
        None,
        crate::SectionPlacement::Transplant,
        "",
        None,
    )
    .map_err(|refusal| {
        refusal.with_message(|message| {
            format!("tube: sweeping the section around a closed edge failed: {message}")
        })
    })?;
    if let Some(report) = report {
        SWEEP_REPORTS.with(|reports| {
            reports.borrow_mut().push(report.approximation(format!("{name}{suffix} ring")))
        });
    }
    let faces = face_count(&swept);
    if faces != profile.len() {
        return Err(format!(
            "tube: swept ring produced {faces} faces, expected {} (sides only, no caps)",
            profile.len()
        )
        .into());
    }
    let side = format!("{name}{suffix}_Seg0_S");
    name_faces(&mut swept, &[side.clone(), side]);
    Ok(swept)
}

/// Binary UNION of two owned solids, coplanar-merged. BINARY only — the n-ary
/// path has no perturbation fallback and would die unrescued on the same tangent
/// node.
///
/// `exact_only` picks the EXACT arrangement alone
/// ([`crate::boolean_operation_with_diagnostics`]) over the ordinary entry that
/// falls back to the SoS perturbation rescue. [`assemble_component`] probes its
/// clearance rungs with it, because a joint that only stands up under the rescue
/// is the joint whose geometry is microns off and whose end cap survives as a
/// crescent — the defect the clearance exists to prevent, not a success.
fn union_solid(a: BrepSolid, b: BrepSolid, exact_only: bool) -> Result<BrepSolid, String> {
    let options = BooleanOptions {
        merge_coplanar_faces: true,
        ..BooleanOptions::default()
    };
    let result = if exact_only {
        crate::boolean_operation_with_diagnostics(&a, &b, BooleanOperation::Union, &options)
            .map(|outcome| outcome.value)
    } else {
        crate::boolean_operation(&a, &b, BooleanOperation::Union, &options)
    };
    result.map_err(|error| format!("tube: joining segments/joints failed: {error}"))
}

fn face_count(solid: &BrepSolid) -> usize {
    solid.shells.first().map_or(0, |shell| shell.faces.len())
}

/// Assign `names` to the solid's first-shell faces in order (extra faces beyond
/// `names` are left unnamed; a mismatch is not fatal — the geometry is what
/// matters, names are a downstream convenience).
fn name_faces(solid: &mut BrepSolid, names: &[String]) {
    if let Some(shell) = solid.shells.first_mut() {
        for (face, face_name) in shell.faces.iter_mut().zip(names) {
            face.name = Some(face_name.clone());
        }
    }
}

/// Resolve the `path` `reference_selection` to a set of INDIVIDUAL edge curves,
/// deduped by name. Each name resolves to:
/// - a published PATH (a per-geometry `{id}:G{gid}` single curve, OR a whole
///   sketch `{id}` chain whose curves are each an edge) → its curves;
/// - a resident solid EDGE → that edge's curve TRIMMED to the edge
///   ([`common::edge_curve`]). The trim is load-bearing: an edge whose end was
///   eaten by a fillet or a boolean keeps the WHOLE original curve on its record
///   and records the subrange in `t0`/`t1`, so a tube built from the untrimmed
///   curve runs past the edge the user picked.
fn resolve_path_edges(ctx: &FeatureContext) -> Result<Vec<NurbsCurve>, String> {
    let names = common::reference_names(ctx.param("path"));
    if names.is_empty() {
        return Err("Tube requires at least one EDGE selection for the path.".into());
    }
    let mut curves: Vec<NurbsCurve> = Vec::new();
    let mut seen: HashSet<String> = HashSet::new();
    for name in &names {
        if !seen.insert(name.clone()) {
            continue;
        }
        if let Some(path) = ctx.scene.resolve_path(name) {
            curves.extend(path.iter().cloned());
            continue;
        }
        if let Some(edge) = ctx.scene.resolve_edge(name) {
            curves.push(common::edge_curve(edge).map_err(|error| format!("tube: {error}"))?);
            continue;
        }
        return Err(format!(
            "tube: path '{name}' not found (no sketch path or resident edge)"
        ).into());
    }
    Ok(curves)
}

/// Classify one path edge into its lane and pick up its end vertices.
///
/// STRAIGHTNESS is decided by sampling the curve against its own chord at
/// [`STRAIGHTNESS_SAMPLES`] stations, not by its midpoint alone: a symmetric
/// S-spline has its midpoint exactly ON the chord, and answering "straight" for
/// one would build its chord as a cylinder — a silent geometry drop, and a
/// different face count than the sweep lane emits. Every station must lie within
/// `1e-6` of the chord, relative to the chord length.
///
/// A curved edge is additionally checked for FOLD-OVER: see
/// [`refuse_fold_over`]. A CLOSED edge — no chord, but a sampled length — is a
/// [`PathEdge::Ring`], fold-checked the same way; a zero-length edge errors.
fn classify_edge(curve: &NurbsCurve, radius: f64) -> Result<PathEdge, String> {
    let [t0, t1] = curve.domain()?;
    let start = curve.evaluate(t0)?;
    let end = curve.evaluate(t1)?;
    let chord = end.sub(start);
    let chord_length = chord.length();
    if chord_length <= EPS {
        // No chord to measure against and no two ends to hang a segment between:
        // either the edge went all the way ROUND (a rim circle), or it went
        // nowhere. The sampled travel tells them apart, at the band `SweepPath`
        // reads closure at for a path whose ends coincide (`1e-5`, floored at
        // the unit scale), so the ring lane and its builder agree.
        let mut travel = 0.0;
        let mut previous = start;
        for station in 1..=CLOSURE_SAMPLES {
            let t = t0 + (t1 - t0) * station as f64 / CLOSURE_SAMPLES as f64;
            let point = curve.evaluate(t)?;
            travel += point.sub(previous).length();
            previous = point;
        }
        if travel > CLOSURE_TRAVEL {
            refuse_fold_over(curve, radius)?;
            return Ok(PathEdge::Ring(curve.clone()));
        }
        return Err("tube: path has a zero-length edge".into());
    }
    let tolerance = 1e-6 * chord_length.max(1.0);
    let mut straight = true;
    for station in 1..STRAIGHTNESS_SAMPLES {
        let t = t0 + (t1 - t0) * station as f64 / STRAIGHTNESS_SAMPLES as f64;
        let point = curve.evaluate(t)?;
        // Distance from the chord LINE, not from the chord's midpoint: an edge
        // that doubles back along its own chord is straight in shape but is a
        // fold, and the perpendicular distance is what the cylinder lane drops.
        let along = point.sub(start).dot(chord) / (chord_length * chord_length);
        let projected = start.add(chord.scale(along.clamp(0.0, 1.0)));
        if point.sub(projected).length() > tolerance {
            straight = false;
            break;
        }
    }
    if straight {
        return Ok(PathEdge::Open(Segment { start, end, curve: None }));
    }
    refuse_fold_over(curve, radius)?;
    Ok(PathEdge::Open(Segment {
        start,
        end,
        curve: Some(curve.clone()),
    }))
}

/// Stations the closed-vs-zero-length test sums chords over: `SweepPath`'s own
/// 16 per curve.
const CLOSURE_SAMPLES: usize = 16;

/// The travel below which an edge whose ends coincide is zero-length rather
/// than closed: `SweepPath`'s `1e-5 * scale` with `scale` at its floor of 1,
/// which is the scale a path whose endpoints all coincide reads.
const CLOSURE_TRAVEL: f64 = 1e-5;

/// Stations the straightness test measures against the chord (the interior ones
/// — the ends are the chord). 16 is `profile_anchor`'s own per-curve sampling
/// density, which is what the sweep lane will validate the section with.
const STRAIGHTNESS_SAMPLES: usize = 16;

/// Refuse a curved segment whose tube would turn itself inside out.
///
/// A tube of radius `r` about a curve is a real solid only while `r` stays under
/// the curve's CENTRE OF CURVATURE — at `r = ρ` the inner wall collapses onto the
/// centre line and past it the section sweeps back through material it already
/// swept. `sweep_profile_along_path` says in its own doc that a self-intersecting
/// result is the CALLER's responsibility, so the caller checks it here, the way
/// `sweep_profile_helix` refuses its own coils.
///
/// κ = |r′ × r″| / |r′|³ at [`CURVATURE_SAMPLES`] stations. Sampling can only
/// UNDER-estimate the curvature between stations, so this is a guard against the
/// fold a user can see, not a proof of soundness — which is why it reports the
/// two numbers rather than claiming the tube is safe.
fn refuse_fold_over(curve: &NurbsCurve, radius: f64) -> Result<(), String> {
    let [t0, t1] = curve.domain()?;
    let mut tightest = f64::INFINITY;
    for station in 0..=CURVATURE_SAMPLES {
        let t = t0 + (t1 - t0) * station as f64 / CURVATURE_SAMPLES as f64;
        let derivatives = curve.derivatives(t, 2)?;
        let speed = derivatives[1].length();
        if speed <= EPS {
            continue;
        }
        let curvature = derivatives[1].cross(derivatives[2]).length() / speed.powi(3);
        if curvature > 0.0 {
            tightest = tightest.min(1.0 / curvature);
        }
    }
    if radius >= tightest {
        return Err(format!(
            "tube: radius {radius} reaches the path's curvature radius {tightest:.6} — the tube \
             would fold through itself; use a smaller radius or a gentler path"
        ).into());
    }
    Ok(())
}

/// Stations the fold-over guard measures curvature at. Denser than the sweep's
/// own 32 sections, because a fold between two sections is still a fold.
const CURVATURE_SAMPLES: usize = 128;

/// Merge segment endpoints within a scale-relative tolerance into shared NODES,
/// returning the node positions and each edge's `(node_a, node_b)`.
fn build_graph(segments: &[Segment]) -> (Vec<Vec3>, Vec<(usize, usize)>) {
    let scale = segments
        .iter()
        .flat_map(|segment| [segment.start, segment.end])
        .map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs()))
        .fold(0.0_f64, f64::max)
        .max(1.0);
    let tolerance = scale * 1e-6;
    let mut nodes: Vec<Vec3> = Vec::new();
    let mut node_index = |point: Vec3, nodes: &mut Vec<Vec3>| -> usize {
        for (index, &node) in nodes.iter().enumerate() {
            if node.sub(point).length() <= tolerance {
                return index;
            }
        }
        nodes.push(point);
        nodes.len() - 1
    };
    let mut edge_nodes = Vec::with_capacity(segments.len());
    for segment in segments {
        let ia = node_index(segment.start, &mut nodes);
        let ib = node_index(segment.end, &mut nodes);
        edge_nodes.push((ia, ib));
    }
    (nodes, edge_nodes)
}

/// Connected components of the edge graph (union-find over nodes), each returned
/// as its edge-index list. Components are ordered by their smallest edge index,
/// so the produced solids number `[0], [1], …` deterministically in selection
/// order.
fn connected_components(edge_nodes: &[(usize, usize)], node_count: usize) -> Vec<Vec<usize>> {
    let mut parent: Vec<usize> = (0..node_count).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    for &(a, b) in edge_nodes {
        let ra = find(&mut parent, a);
        let rb = find(&mut parent, b);
        if ra != rb {
            parent[ra] = rb;
        }
    }
    // Group edges by their root; keep first-appearance order.
    let mut order: Vec<usize> = Vec::new();
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for (edge, &(a, _)) in edge_nodes.iter().enumerate() {
        let root = find(&mut parent, a);
        groups.entry(root).or_insert_with(|| {
            order.push(root);
            Vec::new()
        });
        groups.get_mut(&root).unwrap().push(edge);
    }
    order.into_iter().map(|root| groups.remove(&root).unwrap()).collect()
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected edges drive `path`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "TU",
    "shortName": "TU",
    "longName": "Tube",
    "ribbonPath": "Home/Sweep/Tube",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the tube feature"
        },
        "path": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "timestampDependency": "selection",
            "multiple": true,
            "default_value": null,
            "hint": "Select one or more edges (straight or curved — an arc, spline or helix edge is swept). Each edge becomes one tube segment; connected edges are joined with a sphere at every junction; disconnected groups become separate tubes."
        },
        "radius": {
            "type": "number",
            "default_value": 5,
            "hint": "Outer radius of the tube"
        },
        "innerRadius": {
            "type": "number",
            "default_value": 0,
            "hint": "Optional inner radius for hollow tubes (0 for solid)"
        },
        "bendRadius": {
            "type": "number",
            "default_value": 0,
            "hint": "Round every corner where two straight edges meet into a bend of this radius, swept as one tangent-continuous body (0 joins them with a sphere instead). Must exceed the tube radius."
        },
        "boolean": {
            "type": "boolean_operation",
            "default_value": {
                "targets": [],
                "operation": "NONE",
                "mergeCoplanarFaces": true
            },
            "hint": "Optional boolean operation with target solids"
        }
    }
})
}

