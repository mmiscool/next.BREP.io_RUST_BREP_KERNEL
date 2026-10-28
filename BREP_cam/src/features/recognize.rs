//! The recognition rules (see the module documentation of [`super`]).

use super::graph::{Adjacency, FaceGraph};
use super::{Carrier, FaceCarrier};
use crate::holes::{Hole, Recognition};
use crate::mesh::MeshSnapshot;
use crate::model::{Point2, Point3};
use crate::numeric::Tolerance;
use crate::planar::extract::chain_undirected;
use crate::planar::machining::PlanarRegion;
use crate::planar::{simplified, Chain, Loop, Region, Segment, Vec2};
use crate::CamError;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// What recognition reads. Regions are keyed by face name and come from
/// `kernel_adapter::planar::horizontal_faces` (planes perpendicular to the
/// tool axis); carriers from `kernel_adapter::features::face_carriers`.
pub struct RecognitionInput<'a> {
    pub mesh: &'a MeshSnapshot,
    pub carriers: &'a [FaceCarrier],
    pub regions: &'a [(String, PlanarRegion)],
    pub holes: &'a Recognition,
    pub tolerance: Tolerance,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum FeatureKind {
    /// Outside silhouette, excluding holes, measured in machine coordinates.
    OuterProfile { outline: Region, top_z: f64, bottom_z: f64, walls: ProfileWalls },
    /// A top face to face off.
    Face { region: Region, z: f64 },
    /// `open` when part of the outer boundary drops away; `wall_chains`
    /// counts the connected runs of walls. `open_edges` are the outer
    /// boundary parts that drop away (where the tool may run out), `walls`
    /// the planar walls as XY lines with their inward normal.
    Pocket {
        floor: Region,
        floor_z: f64,
        top_z: f64,
        open: bool,
        wall_chains: u32,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        corner_radius: Option<f64>,
        islands: u32,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        open_edges: Vec<Chain>,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        walls: Vec<WallLine>,
    },
    /// An open pocket with one wall chain.
    Step {
        floor: Region,
        floor_z: f64,
        top_z: f64,
        wall_length: f64,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        open_edges: Vec<Chain>,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        walls: Vec<WallLine>,
    },
    /// A closed pocket whose outline is a stadium.
    Slot { centre: Chain, width: f64, floor_z: f64, top_z: f64 },
    /// Material standing on a floor (`base_z`) up to `top_z`.
    Boss {
        outline: Region,
        top_z: f64,
        base_z: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        corner_radius: Option<f64>,
    },
    Hole(Hole),
    /// `width` is the horizontal leg; `chain` is the edge the chamfer
    /// replaces: the wall line at the top height `z`, directed with the part
    /// interior on its left (what `chamfer2d` expects for an open chain).
    Chamfer { chain: Chain, width: f64, z: f64, length: f64 },
    /// A convex rounded edge; `chain` is its tangent edge with the face above.
    Fillet { chain: Chain, radius: f64, z: f64, length: f64 },
}

/// Classification of the outside walls reached from the bottom boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all="camelCase")]
pub enum ProfileWalls { Vertical, Drafted, Undercut, Uncertified }

/// A planar wall of a floor as a line in machine XY: a point on it and the
/// unit normal pointing into the floor (away from the wall's material).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WallLine {
    pub point: Point2,
    pub inward: Point2,
}

impl FeatureKind {
    pub fn name(&self) -> &'static str {
        match self {
            FeatureKind::OuterProfile { .. } => "outer-profile",
            FeatureKind::Face { .. } => "face",
            FeatureKind::Pocket { open: false, .. } => "pocket",
            FeatureKind::Pocket { open: true, .. } => "open-pocket",
            FeatureKind::Step { .. } => "step",
            FeatureKind::Slot { .. } => "slot",
            FeatureKind::Boss { .. } => "boss",
            FeatureKind::Hole(_) => "hole",
            FeatureKind::Chamfer { .. } => "chamfer",
            FeatureKind::Fillet { .. } => "fillet",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Feature {
    pub id: String,
    /// Persistent body name.
    pub solid: String,
    pub kind: FeatureKind,
    /// Persistent face names the feature is made of (floor and walls, the
    /// chamfer face, the hole walls, ...).
    pub faces: Vec<String>,
    /// Direction the tool approaches along (machine space, unit).
    pub direction: Point3,
    pub machinable: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Recognized {
    pub features: Vec<Feature>,
    /// Faces that belong to no feature, with why.
    pub unassigned: Vec<(String, String)>,
}

/// Classification of a face's carrier against the tool axis.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Class {
    /// Plane with its outward unit normal.
    Plane(Point3),
    VerticalCylinder(f64),
    HorizontalCylinder(f64),
    Other,
}

struct Ctx<'a> {
    input: &'a RecognitionInput<'a>,
    graph: FaceGraph,
    class: Vec<Class>,
    carrier: Vec<Option<&'a Carrier>>,
    region: Vec<Option<&'a PlanarRegion>>,
    /// Hole index per face.
    hole_of: Vec<Option<usize>>,
}

fn name_of(mesh: &MeshSnapshot, f: u32) -> String {
    mesh.faces[f as usize].face.clone()
}

/// Recognise the 2.5D features of the snapshot's target bodies.
pub fn recognize(input: &RecognitionInput<'_>) -> Result<Recognized, CamError> {
    let tol = input.tolerance;
    tol.validate()?;
    let mesh = input.mesh;
    if mesh.is_empty() {
        return Err(CamError::InvalidInput("feature recognition needs a mesh".into()));
    }
    let graph = FaceGraph::build(mesh, &tol);
    let n = mesh.faces.len();
    let by_name: BTreeMap<(&str, &str), &Carrier> = input.carriers.iter().map(|c| ((c.solid.as_str(), c.face.as_str()), &c.carrier)).collect();
    let regions: BTreeMap<&str, &PlanarRegion> = input.regions.iter().map(|(n, r)| (n.as_str(), r)).collect();
    let mut carrier = Vec::with_capacity(n);
    let mut region = Vec::with_capacity(n);
    let mut class = Vec::with_capacity(n);
    for f in &mesh.faces {
        let c = by_name.get(&(f.solid.as_str(), f.face.as_str())).copied();
        carrier.push(c);
        region.push(regions.get(f.face.as_str()).copied());
        class.push(match c {
            Some(Carrier::Plane { normal, .. }) => normal.normalized().map_or(Class::Other, Class::Plane),
            Some(Carrier::Cylinder { axis, radius, .. }) => {
                if tol_vertical(*axis, &tol) {
                    Class::VerticalCylinder(*radius)
                } else if axis.z.abs() <= 1e-6 {
                    Class::HorizontalCylinder(*radius)
                } else {
                    Class::Other
                }
            }
            _ => Class::Other,
        });
    }
    let mut hole_of = vec![None; n];
    for (hi, h) in input.holes.holes.iter().enumerate() {
        for (fi, f) in mesh.faces.iter().enumerate() {
            if f.solid == h.solid && h.faces.contains(&f.face) {
                hole_of[fi] = Some(hi);
            }
        }
    }
    let ctx = Ctx { input, graph, class, carrier, region, hole_of };
    let mut out = Recognized::default();
    let mut assigned = vec![false; n];
    // Holes first: their walls are never pocket walls.
    for (hi, h) in input.holes.holes.iter().enumerate() {
        let _ = hi;
        let reason = if h.machinable { None } else { Some("the hole axis is not the tool axis; it needs another setup".to_string()) };
        out.features.push(Feature {
            id: String::new(),
            solid: h.solid.clone(),
            kind: FeatureKind::Hole(h.clone()),
            faces: h.faces.clone(),
            direction: h.axis,
            machinable: h.machinable,
            reason,
        });
        for (fi, f) in mesh.faces.iter().enumerate() {
            if f.solid == h.solid && h.faces.contains(&f.face) {
                assigned[fi] = true;
            }
        }
    }
    let top_z = ctx
        .graph
        .faces
        .iter()
        .filter(|s| matches!(ctx.class[s.face as usize], Class::Plane(nn) if nn.z > 1.0 - 1e-6))
        .map(|s| s.z_max)
        .fold(f64::NEG_INFINITY, f64::max);
    // Floors: the upward planar faces first (their walls are then taken), then
    // every other planar face, which can only report a feature of another
    // setup; each pass in face index order.
    for upward_pass in [true, false] {
        for f in 0..n as u32 {
            if mesh.faces[f as usize].protected_only || assigned[f as usize] {
                continue;
            }
            let Class::Plane(normal) = ctx.class[f as usize] else { continue };
            if (normal.z > 1.0 - 1e-6) != upward_pass {
                continue;
            }
            if let Some(feats) = floor_features(&ctx, f, normal, top_z, &mut assigned)? {
                out.features.extend(feats);
            }
        }
    }
    // Chamfers and fillets on top edges.
    for f in 0..n as u32 {
        if mesh.faces[f as usize].protected_only || assigned[f as usize] {
            continue;
        }
        match ctx.class[f as usize] {
            Class::Plane(normal) if normal.z > 1e-6 && normal.z < 1.0 - 1e-6 && normal.x.hypot(normal.y) > 1e-6 => {
                if let Some(feat) = chamfer_feature(&ctx, f, normal) {
                    assigned[f as usize] = true;
                    out.features.push(feat);
                }
            }
            Class::HorizontalCylinder(radius) => {
                if let Some(feat) = fillet_feature(&ctx, f, radius) {
                    assigned[f as usize] = true;
                    out.features.push(feat);
                }
            }
            _ => {}
        }
    }
    let profiles=outer_profiles(&ctx)?;
    for profile in &profiles {for (i,face) in mesh.faces.iter().enumerate() {if face.solid==profile.solid && profile.faces.contains(&face.face) {assigned[i]=true;}}}
    out.features.extend(profiles);
    for (i, feat) in out.features.iter_mut().enumerate() {
        feat.id = format!("feature-{}", i + 1);
    }
    for f in 0..n {
        if !assigned[f] && !mesh.faces[f].protected_only {
            let why = match ctx.class[f] {
                Class::Plane(nn) if nn.z < -1.0 + 1e-6 => "a horizontal face facing away from the tool",
                Class::Plane(nn) if nn.z.abs() <= 1e-6 => "a vertical wall",
                Class::Plane(_) => "an inclined face that is not a chamfer",
                Class::VerticalCylinder(_) => "a vertical cylinder that is neither a hole nor a pocket corner",
                Class::HorizontalCylinder(_) => "a horizontal cylinder that is not a top-edge fillet",
                Class::Other => "a surface with no 2.5D carrier",
            };
            out.unassigned.push((mesh.faces[f].face.clone(), why.to_string()));
        }
    }
    Ok(out)
}

fn tol_vertical(axis: Point3, tol: &Tolerance) -> bool {
    axis.normalized().map_or(false, |a| a.z.abs() >= 1.0 - tol.angle_eps.max(1e-9))
}

/// Which loop of a region a point lies on (by nearest distance): `None`
/// for the outer loop, `Some(k)` for the k-th hole loop.
fn loop_of(region: &Region, p: Point2, tol: &Tolerance) -> Option<usize> {
    let mut best: Option<(f64, usize)> = None;
    for (i, l) in region.loops.iter().enumerate() {
        let d = l.segments.iter().map(|s| s.distance(p)).fold(f64::INFINITY, f64::min);
        if best.map_or(true, |(bd, _)| d < bd) {
            best = Some((d, i));
        }
    }
    let (d, i) = best?;
    let _ = d;
    let _ = tol;
    if region.loops[i].is_ccw() {
        None
    } else {
        Some(i)
    }
}

/// How far a neighbour rises along the floor's normal.
fn rise(ctx: &Ctx<'_>, floor: u32, normal: Point3, neighbour: u32) -> f64 {
    let fs = &ctx.graph.faces[floor as usize];
    let ns = &ctx.graph.faces[neighbour as usize];
    if normal.z > 0.9 {
        ns.z_max - fs.z_max
    } else if normal.z < -0.9 {
        fs.z_min - ns.z_min
    } else {
        ns.centroid.sub(fs.centroid).dot(normal)
    }
}

struct Neighbour<'a> {
    face: u32,
    adj: &'a Adjacency,
    /// Hole loop index of the floor's region, `None` on the outer loop.
    island: Option<usize>,
}

/// Projection of the triangle portion strictly above a plane. Clipping close
/// to a mesh vertex can introduce a sub-epsilon edge; merge equal points using
/// the same policy that the planar engine uses to validate its segments.
pub(crate) fn projected_above(triangle: &crate::mesh::Triangle, level: f64, tol: &Tolerance) -> Option<Loop> {
    if triangle.max_z() <= level { return None; }
    let vertices = triangle.vertices();
    let mut points = Vec::new();
    for i in 0..3 {
        let (a, b) = (vertices[i], vertices[(i + 1) % 3]);
        if (a.z > level) != (b.z > level) {
            let p = a.lerp(b, (level - a.z) / (b.z - a.z));
            points.push(Point2::new(p.x, p.y));
        }
        if b.z > level { points.push(Point2::new(b.x, b.y)); }
    }
    points.dedup_by(|a, b| a.distance(*b) <= tol.point_eps);
    while points.len() > 1 && points[0].distance(*points.last().unwrap()) <= tol.point_eps { points.pop(); }
    if points.len() < 3 { return None; }
    let mut boundary = Loop::polygon(&points);
    if boundary.signed_area().abs() <= tol.area_eps { return None; }
    if !boundary.is_ccw() { boundary = boundary.reversed(); }
    Some(boundary)
}

/// Certify the whole floor interior, not only its boundary, against overhead material.
pub(crate) fn floor_access_refusal(floor: &Region, z: f64, mesh: &MeshSnapshot, regions: &[(String, PlanarRegion)], tol: &Tolerance) -> Option<String> {
        let (lo, hi) = floor.bounds();
        let level = z + tol.distance_eps;
        // A mesh chord across a circular opening lies inside the exact hole.
        // Use the exact trimmed planar face where supplied, so that sliver
        // is not mistaken for a roof. Nonplanar faces still use the mesh.
        let mut exact_faces = std::collections::BTreeSet::new();
        for (fi, face) in mesh.faces.iter().enumerate() {
            let mut matching = regions.iter().filter(|(name, _)| name == &face.face);
            let Some((_, region)) = matching.next() else { continue; };
            if matching.next().is_some() { continue; }
            exact_faces.insert(fi as u32);
            if region.z <= level { continue; }
            let blocked = match floor.intersection(&region.region, tol) {
                Ok(overlap) => overlap.area() > tol.area_eps,
                Err(_) => true,
            };
            if blocked { return Some(format!("undercut floor approach blocked by {}/{}; no certified vertical access in this setup", face.solid, face.face)); }
        }
        for triangle in &mesh.triangles {
            if exact_faces.contains(&triangle.face) { continue; }
            if triangle.max_z() <= level || triangle.bounds.max.x <= lo.x || triangle.bounds.min.x >= hi.x
                || triangle.bounds.max.y <= lo.y || triangle.bounds.min.y >= hi.y { continue; }
            let Some(boundary) = projected_above(triangle, level, tol) else { continue; };
            let blocked=match floor.intersection(&Region{loops:vec![boundary]},tol) {
                Ok(overlap)=>overlap.area()>tol.area_eps,
                Err(_)=>true,
            };
            if blocked {
                let obstruction=&mesh.faces[triangle.face as usize];
                return Some(format!("undercut floor approach blocked by {}/{}; no certified vertical access in this setup",obstruction.solid,obstruction.face));
            }
        }
    None
}

fn floor_features(ctx: &Ctx<'_>, f: u32, normal: Point3, top_z: f64, assigned: &mut [bool]) -> Result<Option<Vec<Feature>>, CamError> {
    let tol = &ctx.input.tolerance;
    let mesh = ctx.input.mesh;
    let solid = mesh.faces[f as usize].solid.clone();
    let stats = &ctx.graph.faces[f as usize];
    let upward = normal.z > 1.0 - 1e-6;
    let region = ctx.region[f as usize];
    let neighbours: Vec<Neighbour<'_>> = ctx
        .graph
        .neighbours(f)
        .into_iter()
        .map(|(nf, adj)| {
            let island = region.and_then(|r| {
                let (a, b) = adj.segments[0];
                let m = a.lerp(b, 0.5);
                loop_of(&r.region, Point2::new(m.x, m.y), tol)
            });
            Neighbour { face: nf, adj, island }
        })
        .collect();
    let mut wall_len = 0.0;
    let mut drop_len = 0.0;
    let mut walls: Vec<&Neighbour<'_>> = Vec::new();
    let mut drops: Vec<&Neighbour<'_>> = Vec::new();
    for nb in neighbours.iter().filter(|nb| nb.island.is_none()) {
        if nb.adj.concave() && rise(ctx, f, normal, nb.face) > tol.distance_eps {
            wall_len += nb.adj.length();
            walls.push(nb);
        } else if nb.adj.convex_length > 0.0 {
            drop_len += nb.adj.length();
            drops.push(nb);
        }
    }
    let faces_of = |list: &[&Neighbour<'_>]| -> Vec<String> {
        let mut v = vec![name_of(mesh, f)];
        v.extend(list.iter().map(|nb| name_of(mesh, nb.face)));
        v
    };
    // A floor whose walls all belong to one recognised hole is its bottom.
    if !walls.is_empty() {
        let holes: Vec<Option<usize>> = walls.iter().map(|w| ctx.hole_of[w.face as usize]).collect();
        if holes.iter().all(|h| h.is_some()) && holes.windows(2).all(|w| w[0] == w[1]) {
            assigned[f as usize] = true;
            return Ok(None);
        }
    }
    if walls.is_empty() {
        if !upward {
            return Ok(None);
        }
        let Some(r) = region else { return Ok(None) };
        assigned[f as usize] = true;
        let z = stats.z_max;
        if (z - top_z).abs() <= tol.distance_eps || drops.is_empty() {
            return Ok(Some(vec![Feature { id: String::new(), solid: solid.clone(), kind: FeatureKind::Face { region: r.region.clone(), z }, faces: vec![name_of(mesh, f)], direction: Point3::Z, machinable: true, reason: None }]));
        }
        // A raised face below the top: a boss standing on its drops' bottoms.
        let base_z = drops.iter().map(|d| ctx.graph.faces[d.face as usize].z_min).fold(f64::INFINITY, f64::min);
        let corner = drops.iter().filter_map(|d| if let Class::VerticalCylinder(rr) = ctx.class[d.face as usize] { Some(rr) } else { None }).fold(f64::INFINITY, f64::min);
        let reason = boss_access_refusal(ctx, &r.region, z).or_else(|| boss_wall_refusal(ctx, &drops, base_z));
        for d in &drops {
            assigned[d.face as usize] = true;
        }
        return Ok(Some(vec![Feature {
            id: String::new(),
            solid: solid.clone(),
            kind: FeatureKind::Boss { outline: r.region.clone(), top_z: z, base_z, corner_radius: (corner.is_finite()).then_some(corner) },
            faces: faces_of(&drops),
            direction: Point3::Z,
            machinable: reason.is_none(),
            reason,
        }]));
    }
    // A floor with walls: pocket, step or slot, machinable only along +Z.
    let top = walls.iter().map(|w| ctx.graph.faces[w.face as usize].z_max).fold(f64::NEG_INFINITY, f64::max);
    let floor_z = stats.z_max;
    let open = drop_len > tol.distance_eps;
    let chains = wall_chains(ctx, &walls);
    let open_edges: Vec<Chain> = drops.iter().filter_map(|d| shared_chain(d.adj, tol)).collect();
    let wall_lines: Vec<WallLine> = walls
        .iter()
        .filter_map(|w| match ctx.carrier[w.face as usize] {
            Some(Carrier::Plane { point, normal }) if normal.x.hypot(normal.y) > 1e-9 => {
                let inward = Point2::new(normal.x, normal.y).normalized()?;
                Some(WallLine { point: Point2::new(point.x, point.y), inward })
            }
            _ => None,
        })
        .collect();
    let corner = walls.iter().filter_map(|w| if let Class::VerticalCylinder(rr) = ctx.class[w.face as usize] { Some(rr) } else { None }).fold(f64::INFINITY, f64::min);
    let corner_radius = corner.is_finite().then_some(corner);
    let mut faces = faces_of(&walls);
    assigned[f as usize] = true;
    // A floor inaccessible in this setup must not consume adjacent faces:
    // one of those faces can be the accessible floor of a side setup.
    if !upward {
        let reason = if normal.z < -1.0 + 1e-6 {
            "the floor faces away from the tool".to_string()
        } else {
            format!("the feature is accessed along ({:.3}, {:.3}, {:.3}); it needs a setup with the tool axis along it", normal.x, normal.y, normal.z)
        };
        let kind = if open && chains == 1 {
            FeatureKind::Step { floor: Region::empty(), floor_z, top_z: top, wall_length: wall_len, open_edges: open_edges.clone(), walls: wall_lines.clone() }
        } else {
            FeatureKind::Pocket { floor: Region::empty(), floor_z, top_z: top, open, wall_chains: chains, corner_radius, islands: 0, open_edges: open_edges.clone(), walls: wall_lines.clone() }
        };
        return Ok(Some(vec![Feature { id: String::new(), solid: solid.clone(), kind, faces, direction: normal, machinable: false, reason: Some(reason) }]));
    }
    let Some(r) = region else {
        return Ok(Some(vec![Feature {
            id: String::new(),
            solid: solid.clone(),
            kind: FeatureKind::Pocket { floor: Region::empty(), floor_z, top_z: top, open, wall_chains: chains, corner_radius, islands: 0, open_edges, walls: wall_lines },
            faces,
            direction: Point3::Z,
            machinable: false,
            reason: Some("the floor has no exact planar region".to_string()),
        }]));
    };
    let mut feats = Vec::new();
    // An inaccessible floor is refused, but the islands standing on it are
    // still bosses: each one is judged by its own access check below.
    if let Some(reason) = floor_access_refusal(&r.region, floor_z, mesh, ctx.input.regions, tol) {
        let kind = if open && chains == 1 {
            FeatureKind::Step { floor: r.region.clone(), floor_z, top_z: top, wall_length: wall_len, open_edges, walls: wall_lines }
        } else {
            FeatureKind::Pocket { floor: r.region.clone(), floor_z, top_z: top, open, wall_chains: chains, corner_radius, islands: 0, open_edges, walls: wall_lines }
        };
        feats.push(Feature { id: String::new(), solid: solid.clone(), kind, faces: faces.clone(), direction: normal, machinable: false, reason: Some(reason) });
    } else {
        for w in &walls { assigned[w.face as usize] = true; }
        let shells = r.region.shells(tol);
        let islands = shells.iter().map(|s| s.holes.len()).sum::<usize>() as u32;
        if open {
            let kind = if chains == 1 {
                FeatureKind::Step { floor: r.region.clone(), floor_z, top_z: top, wall_length: wall_len, open_edges: open_edges.clone(), walls: wall_lines.clone() }
            } else {
                FeatureKind::Pocket { floor: r.region.clone(), floor_z, top_z: top, open: true, wall_chains: chains, corner_radius, islands, open_edges: open_edges.clone(), walls: wall_lines.clone() }
            };
            feats.push(Feature { id: String::new(), solid: solid.clone(), kind, faces: faces.clone(), direction: Point3::Z, machinable: true, reason: None });
        } else if islands == 0 && shells.len() == 1 {
            if let Some((centre, width)) = stadium_of(&shells[0].outer, tol) {
                feats.push(Feature { id: String::new(), solid: solid.clone(), kind: FeatureKind::Slot { centre, width, floor_z, top_z: top }, faces: faces.clone(), direction: Point3::Z, machinable: true, reason: None });
            } else {
                feats.push(Feature { id: String::new(), solid: solid.clone(), kind: FeatureKind::Pocket { floor: r.region.clone(), floor_z, top_z: top, open: false, wall_chains: chains, corner_radius, islands, open_edges: Vec::new(), walls: wall_lines.clone() }, faces: faces.clone(), direction: Point3::Z, machinable: true, reason: None });
            }
        } else {
            feats.push(Feature { id: String::new(), solid: solid.clone(), kind: FeatureKind::Pocket { floor: r.region.clone(), floor_z, top_z: top, open: false, wall_chains: chains, corner_radius, islands, open_edges: Vec::new(), walls: wall_lines.clone() }, faces: faces.clone(), direction: Point3::Z, machinable: true, reason: None });
        }
    }
    // Islands: bosses standing on this floor.
    for (li, l) in r.region.loops.iter().enumerate() {
        if l.is_ccw() {
            continue;
        }
        // A clockwise floor loop can bound an opening down through the floor,
        // not just material standing above it. Only rising neighbours establish
        // an island; downward walls must remain available to their own floors.
        let island_walls: Vec<&Neighbour<'_>> = neighbours.iter().filter(|nb| nb.island == Some(li) && rise(ctx, f, normal, nb.face) > tol.distance_eps).collect();
        if island_walls.is_empty() {
            continue;
        }
        let top_of_island = island_walls.iter().map(|w| ctx.graph.faces[w.face as usize].z_max).fold(f64::NEG_INFINITY, f64::max);
        let corner = island_walls.iter().filter_map(|w| if let Class::VerticalCylinder(rr) = ctx.class[w.face as usize] { Some(rr) } else { None }).fold(f64::INFINITY, f64::min);
        let mut island_faces: Vec<String> = island_walls.iter().map(|w| name_of(mesh, w.face)).collect();
        for w in &island_walls {
            assigned[w.face as usize] = true;
            faces.push(name_of(mesh, w.face));
        }
        // The island's top face: an upward plane whose centroid lies inside the island loop.
        let outline = Region { loops: vec![l.reversed()] };
        let mut top_z_island = top_of_island;
        for (fi, s) in ctx.graph.faces.iter().enumerate() {
            if assigned[fi] || mesh.faces[fi].protected_only {
                continue;
            }
            if let Class::Plane(nn) = ctx.class[fi] {
                if nn.z > 1.0 - 1e-6 && s.z_max > floor_z + tol.distance_eps && outline.contains(Point2::new(s.centroid.x, s.centroid.y), tol) {
                    assigned[fi] = true;
                    island_faces.push(name_of(mesh, fi as u32));
                    top_z_island = top_z_island.max(s.z_max);
                }
            }
        }
        let reason = boss_access_refusal(ctx, &outline, top_z_island).or_else(|| boss_wall_refusal(ctx, &island_walls, floor_z));
        feats.push(Feature {
            id: String::new(),
            solid: solid.clone(),
            kind: FeatureKind::Boss { outline, top_z: top_z_island, base_z: floor_z, corner_radius: corner.is_finite().then_some(corner) },
            faces: island_faces,
            direction: Point3::Z,
            machinable: reason.is_none(),
            reason,
        });
    }
    Ok(Some(feats))
}

/// Check the actual contour boundary, not the whole enclosed face: a taller
/// island inside a plate does not obstruct the plate's outside contour.
/// Clip overhead triangles at the boss top before projecting them, then use
/// exact line/arc intersections rather than a fixed set of boundary samples.
fn boss_access_refusal(ctx: &Ctx<'_>, outline: &Region, top: f64) -> Option<String> {
    let tol = &ctx.input.tolerance;
    let level = top + tol.distance_eps;
    let (lo, hi) = outline.bounds();
    for triangle in &ctx.input.mesh.triangles {
        if triangle.max_z() <= level || triangle.bounds.max.x < lo.x || triangle.bounds.min.x > hi.x
            || triangle.bounds.max.y < lo.y || triangle.bounds.min.y > hi.y {
            continue;
        }
        let vertices = triangle.vertices();
        let mut clipped = Vec::new();
        for i in 0..3 {
            let (a, b) = (vertices[i], vertices[(i + 1) % 3]);
            if (a.z > level) != (b.z > level) {
                clipped.push(a.lerp(b, (level - a.z) / (b.z - a.z)));
            }
            if b.z > level {
                clipped.push(b);
            }
        }
        let points: Vec<Point2> = clipped.iter().map(|p| Point2::new(p.x, p.y)).collect();
        if points.len() < 3 {
            continue;
        }
        let mut projected = Loop::polygon(&points);
        if projected.signed_area().abs() <= tol.area_eps {
            continue;
        }
        if !projected.is_ccw() { projected = projected.reversed(); }
        let footprint = Region { loops: vec![projected] };
        let blocked = outline.loops.iter().flat_map(|l| &l.segments).any(|s| {
            footprint.contains(s.start, tol) || footprint.contains(s.end, tol)
                || footprint.loops[0].segments.iter().any(|edge| !s.intersect(edge, tol).is_empty())
        });
        if blocked {
            let face = &ctx.input.mesh.faces[triangle.face as usize];
            return Some(format!("undercut wall contour is blocked from +Z by {}/{}; needs another setup", face.solid, face.face));
        }
    }
    None
}

/// A constant-depth boss contour needs vertical walls with a common flat
/// bottom. A vertical carrier alone does not prove that: its lower trim may
/// follow a sculpted floor, which a contour at the lowest vertex would gouge.
fn boss_wall_refusal(ctx: &Ctx<'_>, walls: &[&Neighbour<'_>], base_z: f64) -> Option<String> {
    let eps = ctx.input.tolerance.distance_eps;
    for wall in walls {
        let name = name_of(ctx.input.mesh, wall.face);
        match ctx.class[wall.face as usize] {
            Class::Plane(n) if n.z.abs() <= 1e-6 => {}
            Class::VerticalCylinder(_) => {}
            _ => return Some(format!("wall {name} is drafted or curved; needs 3D finishing or another setup")),
        }
        type Key = (u64, u64, u64);
        let key = |p: Point3| -> Key { (p.x.to_bits(), p.y.to_bits(), p.z.to_bits()) };
        let mut edges: BTreeMap<(Key, Key), (Point3, Point3, u32)> = BTreeMap::new();
        for triangle in ctx.input.mesh.triangles.iter().filter(|t| t.face == wall.face) {
            let v = triangle.vertices();
            for i in 0..3 {
                let (a, b) = (v[i], v[(i + 1) % 3]);
                let (ka, kb) = (key(a), key(b));
                let entry = edges.entry((ka.min(kb), ka.max(kb))).or_insert((a, b, 0));
                entry.2 += 1;
            }
        }
        let top = ctx.graph.faces[wall.face as usize].z_max;
        let mut bottom_found = false;
        for (a, b, count) in edges.values() {
            if *count != 1 || a.distance_xy(*b) <= eps {
                continue; // internal triangulation or vertical side seam
            }
            if (a.z - top).abs() <= eps && (b.z - top).abs() <= eps {
                continue;
            }
            if (a.z - base_z).abs() > eps || (b.z - base_z).abs() > eps {
                return Some(format!("wall {name} has a varying bottom; needs 3D finishing"));
            }
            bottom_found = true;
        }
        if !bottom_found {
            return Some(format!("wall {name} has no certified flat bottom; needs 3D finishing"));
        }
    }
    None
}

/// Connected runs of walls (walls adjacent to each other).
fn wall_chains(ctx: &Ctx<'_>, walls: &[&Neighbour<'_>]) -> u32 {
    let n = walls.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for i in 0..n {
        for j in i + 1..n {
            if ctx.graph.adjacency(walls[i].face, walls[j].face).is_some() {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                if a != b {
                    parent[a.max(b)] = a.min(b);
                }
            }
        }
    }
    (0..n).filter(|&i| find(&mut parent, i) == i).count() as u32
}

/// A stadium outline: two parallel lines joined by two half circles of the
/// same radius. Returns the centre line and the width.
fn stadium_of(outer: &Loop, tol: &Tolerance) -> Option<(Chain, f64)> {
    let l = simplified(outer, tol);
    if l.segments.len() != 4 {
        return None;
    }
    let arcs: Vec<&Segment> = l.segments.iter().filter(|s| s.is_arc()).collect();
    let lines: Vec<&Segment> = l.segments.iter().filter(|s| !s.is_arc()).collect();
    if arcs.len() != 2 || lines.len() != 2 {
        return None;
    }
    let r = arcs[0].radius();
    if (arcs[1].radius() - r).abs() > tol.distance_eps {
        return None;
    }
    for a in &arcs {
        if (a.angles().1.abs() - std::f64::consts::PI).abs() > 1e-6 {
            return None;
        }
    }
    let c0 = arcs[0].arc.unwrap().center;
    let c1 = arcs[1].arc.unwrap().center;
    if c0.distance(c1) <= tol.point_eps {
        return None;
    }
    Some((Chain::open(vec![Segment::line(c0, c1)]), 2.0 * r))
}

/// The shared edge of two faces as a planar chain at the upper face's height.
fn shared_chain(adj: &Adjacency, tol: &Tolerance) -> Option<Chain> {
    let segs: Vec<Segment> = adj
        .segments
        .iter()
        .filter_map(|(a, b)| {
            let s = Segment::line(Point2::new(a.x, a.y), Point2::new(b.x, b.y));
            (s.start.distance(s.end) > tol.point_eps).then_some(s)
        })
        .collect();
    let mut chains = chain_undirected(segs, tol);
    chains.sort_by(|a, b| b.length().total_cmp(&a.length()));
    let mut c = chains.into_iter().next()?;
    c.segments = simplified(&Loop::new(c.segments.clone()), tol).segments;
    Some(c)
}

fn upper_neighbour<'a>(ctx: &'a Ctx<'_>, f: u32) -> Option<(u32, &'a Adjacency)> {
    ctx.graph
        .neighbours(f)
        .into_iter()
        .filter(|(nf, adj)| adj.convex_length > adj.concave_length && matches!(ctx.class[*nf as usize], Class::Plane(nn) if nn.z > 1.0 - 1e-6))
        .max_by(|a, b| a.1.length().total_cmp(&b.1.length()))
}

fn chamfer_feature(ctx: &Ctx<'_>, f: u32, normal: Point3) -> Option<Feature> {
    let tol = &ctx.input.tolerance;
    let solid = ctx.input.mesh.faces[f as usize].solid.clone();
    let (upper, adj) = upper_neighbour(ctx, f)?;
    let stats = &ctx.graph.faces[f as usize];
    let upper_chain = shared_chain(adj, tol)?;
    // The lower edge (with the steeper face below) is the wall line the
    // chamfer replaces; its direction keeps the part interior (the upper
    // edge's side) on the left.
    let lower = ctx
        .graph
        .neighbours(f)
        .into_iter()
        .filter(|(nf, a)| *nf != upper && a.convex_length > a.concave_length && ctx.graph.faces[*nf as usize].z_min < stats.z_min + tol.distance_eps)
        .max_by(|a, b| a.1.length().total_cmp(&b.1.length()));
    let mut chain = match lower {
        Some((_, a)) => shared_chain(a, tol)?,
        None => upper_chain.clone(),
    };
    let interior = upper_chain.segments[0].midpoint().sub(chain.segments[0].nearest(upper_chain.segments[0].midpoint()).0);
    if !chain.closed && chain.segments[0].tangent_at(0.5).cross(interior) < 0.0 {
        chain = chain.reversed();
    }
    let horizontal = normal.x.hypot(normal.y);
    let width = (stats.z_max - stats.z_min) * normal.z / horizontal;
    Some(Feature {
        id: String::new(),
        solid,
        kind: FeatureKind::Chamfer { chain, width, z: ctx.graph.faces[upper as usize].z_max, length: adj.length() },
        faces: vec![name_of(ctx.input.mesh, f)],
        direction: Point3::Z,
        machinable: true,
        reason: None,
    })
}

fn fillet_feature(ctx: &Ctx<'_>, f: u32, radius: f64) -> Option<Feature> {
    let tol = &ctx.input.tolerance;
    let solid = ctx.input.mesh.faces[f as usize].solid.clone();
    let (upper, adj) = upper_neighbour(ctx, f)?;
    let chain = shared_chain(adj, tol)?;
    Some(Feature {
        id: String::new(),
        solid,
        kind: FeatureKind::Fillet { chain, radius, z: ctx.graph.faces[upper as usize].z_max, length: adj.length() },
        faces: vec![name_of(ctx.input.mesh, f)],
        direction: Point3::Z,
        machinable: true,
        reason: None,
    })
}

/// A flat bottom gives an exact outer loop. All body vertices must project
/// inside it; an upper overhang cannot be silently classified as vertical.
fn outer_profiles(ctx:&Ctx<'_>)->Result<Vec<Feature>,CamError> {
    let mesh=ctx.input.mesh;let tol=&ctx.input.tolerance;let mut out=Vec::new();
    let solids:std::collections::BTreeSet<_>=mesh.faces.iter().filter(|f|!f.protected_only).map(|f|f.solid.clone()).collect();
    for solid in solids {
        let ids:Vec<usize>=mesh.faces.iter().enumerate().filter(|(_,f)|f.solid==solid && !f.protected_only).map(|(i,_)|i).collect();
        let bottom=ids.iter().map(|&i|ctx.graph.faces[i].z_min).fold(f64::INFINITY,f64::min);
        let top=ids.iter().map(|&i|ctx.graph.faces[i].z_max).fold(f64::NEG_INFINITY,f64::max);
        if !bottom.is_finite() || !top.is_finite() || top-bottom<=tol.distance_eps {continue;}
        let bottoms:Vec<_>=ids.iter().copied().filter(|&i|matches!(ctx.class[i],Class::Plane(n) if n.z< -1.+1e-6) && (ctx.graph.faces[i].z_max-bottom).abs()<=tol.distance_eps).collect();
        let mut outline=Region::empty();
        for &i in &bottoms {
            let region=if let Some(r)=ctx.region[i].filter(|_|mesh.faces.iter().filter(|f|f.face==mesh.faces[i].face).count()==1) {r.region.clone()}else{crate::planar::extract::face_outline(mesh,|f|f as usize==i,tol)?.0};
            let outer=Region::new(region.loops.into_iter().filter(|l|l.is_ccw()).collect(),tol)?;
            outline=outline.union(&outer,tol)?;
        }
        let mut walls=ProfileWalls::Vertical;let mut faces=Vec::new();
        if outline.loops.is_empty() {
            let projected=crate::planar::extract::silhouette(mesh,|f|ids.contains(&(f as usize)),tol)?;
            outline=Region::new(projected.loops.into_iter().filter(|l|l.is_ccw()).collect(),tol)?;
            walls=ProfileWalls::Uncertified;
        } else {
            for (&(a,b),adj) in &ctx.graph.adjacency {
                let side=if bottoms.contains(&(a as usize)){b as usize}else if bottoms.contains(&(b as usize)){a as usize}else{continue};
                let on_outer=adj.segments.iter().any(|(p,_)|outline.loops.iter().any(|l|l.segments.iter().any(|e|e.distance(Point2::new(p.x,p.y))<=tol.distance_eps*10.)));
                if !on_outer {continue;}
                faces.push(mesh.faces[side].face.clone());
                let class=match ctx.class[side] {
                    Class::Plane(n) if n.z.abs()<1e-6=>ProfileWalls::Vertical,
                    Class::VerticalCylinder(_)=>ProfileWalls::Vertical,
                    Class::Plane(n) if n.z<0.=>ProfileWalls::Undercut,
                    Class::Plane(_)=>ProfileWalls::Drafted,
                    _=>match ctx.carrier[side] {Some(Carrier::Cone{radius0,radius1,..}) if radius1>radius0=>ProfileWalls::Undercut,Some(Carrier::Cone{..})=>ProfileWalls::Drafted,_=>ProfileWalls::Uncertified},
                };
                if class==ProfileWalls::Undercut || walls==ProfileWalls::Vertical {walls=class;}
            }
            if faces.is_empty(){walls=ProfileWalls::Uncertified;}
            if mesh.triangles.iter().filter(|t|ids.contains(&(t.face as usize))).flat_map(|t|t.vertices()).any(|p|outline.signed_distance(Point2::new(p.x,p.y),tol)< -tol.distance_eps*10.) {
                walls=ProfileWalls::Undercut;
                let projected=crate::planar::extract::silhouette(mesh,|f|ids.contains(&(f as usize)),tol)?;
                outline=Region::new(projected.loops.into_iter().filter(|l|l.is_ccw()).collect(),tol)?;
            }
        }
        faces.extend(bottoms.iter().map(|&i|mesh.faces[i].face.clone()));faces.sort();faces.dedup();
        let reason=match walls {ProfileWalls::Vertical=>None,ProfileWalls::Drafted=>Some("outer profile has drafted walls; requires 3D finishing".into()),ProfileWalls::Undercut=>Some("outer profile has undercut or overhanging walls; requires another setup or 3D finishing".into()),ProfileWalls::Uncertified=>Some("outer profile wall/depth classification is uncertified; requires a flat bottom and analytic side walls".into())};
        out.push(Feature{id:String::new(),solid,kind:FeatureKind::OuterProfile{outline,top_z:top,bottom_z:bottom,walls},faces,direction:Point3::Z,machinable:reason.is_none(),reason});
    }
    Ok(out)
}
