//! Exact 2D regions and chains from kernel bodies and sketch profiles, in
//! MACHINE space (lane C5 row 2). Lines and circular arcs are recognised
//! from the kernel's NURBS curves so the planar engine keeps them exact;
//! anything else becomes chords at the requested chordal tolerance.

use crate::model::{Point2, Point3, RigidTransform};
use crate::numeric::Tolerance;
use crate::planar::extract::chain_undirected;
use crate::planar::{Loop, PlanarError, Region, Segment, Vec2};
use brep_kernel::{AnalyticSurface, BrepSolid, FaceRecord, NurbsCurve, SketchProfile, Vec3};

pub use crate::planar::machining::{PlanarChain, PlanarRegion};

/// Why extraction refused.
#[derive(Clone, Debug, PartialEq)]
pub enum ExtractError {
    /// The named face does not exist on the body.
    NoSuchFace(String),
    NoSuchEdge(String),
    /// The face's surface is not a plane perpendicular to the machine Z axis.
    NotHorizontalPlane(String),
    /// A curve leaves the plane (its points span more than the tolerance in Z).
    NotPlanar { what: String, z_span: f64 },
    /// The kernel refused to evaluate a curve.
    Kernel(String),
    Planar(PlanarError),
}

impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExtractError::NoSuchFace(n) => write!(f, "face {n:?} is not on the body"),
            ExtractError::NoSuchEdge(n) => write!(f, "edge {n:?} is not on the body"),
            ExtractError::NotHorizontalPlane(n) => write!(f, "face {n:?} is not a plane perpendicular to the tool axis"),
            ExtractError::NotPlanar { what, z_span } => write!(f, "{what} is not planar (Z span {z_span})"),
            ExtractError::Kernel(m) => write!(f, "kernel: {m}"),
            ExtractError::Planar(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ExtractError {}

impl From<PlanarError> for ExtractError {
    fn from(e: PlanarError) -> Self {
        ExtractError::Planar(e)
    }
}

impl From<ExtractError> for crate::CamError {
    fn from(e: ExtractError) -> Self {
        crate::CamError::InvalidInput(e.to_string())
    }
}

fn to_machine(t: &RigidTransform, v: Vec3) -> Point3 {
    t.apply_point(Point3::new(v.x, v.y, v.z))
}

/// Samples per curve for line/arc recognition.
const SAMPLES: usize = 17;

/// Segments of one curve over `[t0, t1]` (reversed when `!forward`) in machine
/// XY, plus the Z range the samples span. A curve is a line when every sample
/// is on the chord, an arc when every sample is on the circle through its
/// ends and middle; otherwise chords within `chord_tol`.
pub fn curve_segments(
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    transform: &RigidTransform,
    chord_tol: f64,
    tol: &Tolerance,
) -> Result<(Vec<Segment>, [f64; 2]), ExtractError> {
    let eval = |t: f64| -> Result<Point3, ExtractError> {
        let v = curve.evaluate(t).map_err(ExtractError::Kernel)?;
        Ok(to_machine(transform, v))
    };
    let (a, b) = if forward { (t0, t1) } else { (t1, t0) };
    let samples: Vec<Point3> = (0..SAMPLES)
        .map(|i| eval(a + (b - a) * i as f64 / (SAMPLES - 1) as f64))
        .collect::<Result<_, _>>()?;
    let zmin = samples.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let zmax = samples.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    let xy = |p: Point3| Point2::new(p.x, p.y);
    let p0 = xy(samples[0]);
    let p1 = xy(samples[SAMPLES - 1]);
    let eps = tol.distance_eps.max(1e-9 * (zmax - zmin).abs().max(p0.distance(p1)));
    // A closed curve (full circle) splits in two halves first.
    if p0.distance(p1) <= tol.point_eps {
        let mid = 0.5 * (a + b);
        let (mut first, z1) = curve_segments(curve, a, mid, true, transform, chord_tol, tol)?;
        let (second, z2) = curve_segments(curve, mid, b, true, transform, chord_tol, tol)?;
        first.extend(second);
        return Ok((first, [z1[0].min(z2[0]), z1[1].max(z2[1])]));
    }
    // Line?
    let chord = Segment::line(p0, p1);
    if samples.iter().all(|p| chord.distance(xy(*p)) <= eps) {
        return Ok((vec![chord], [zmin, zmax]));
    }
    // Arc through start, middle, end?
    let pm = xy(samples[SAMPLES / 2]);
    if let Some(center) = circumcenter(p0, pm, p1) {
        let r = center.distance(p0);
        if samples.iter().all(|p| (center.distance(xy(*p)) - r).abs() <= eps) {
            let ccw = pm.sub(p0).cross(p1.sub(pm)) > 0.0;
            // The samples must also progress monotonically around the circle.
            let arc = Segment::arc(p0, p1, center, ccw);
            let on_arc = samples.iter().all(|p| arc.distance(xy(*p)) <= eps);
            if on_arc {
                return Ok((vec![arc], [zmin, zmax]));
            }
        }
    }
    // Chords within the chordal tolerance, by recursive bisection.
    let mut pts: Vec<Point3> = vec![samples[0]];
    subdivide(&eval, a, b, samples[0], samples[SAMPLES - 1], chord_tol.max(tol.distance_eps), 0, &mut pts)?;
    let zmin = pts.iter().map(|p| p.z).fold(zmin, f64::min);
    let zmax = pts.iter().map(|p| p.z).fold(zmax, f64::max);
    let mut segs = Vec::new();
    for w in pts.windows(2) {
        let s = Segment::line(xy(w[0]), xy(w[1]));
        if s.start.distance(s.end) > tol.point_eps {
            segs.push(s);
        }
    }
    Ok((segs, [zmin, zmax]))
}

fn subdivide(
    eval: &dyn Fn(f64) -> Result<Point3, ExtractError>,
    a: f64,
    b: f64,
    pa: Point3,
    pb: Point3,
    chord_tol: f64,
    depth: u32,
    out: &mut Vec<Point3>,
) -> Result<(), ExtractError> {
    let m = 0.5 * (a + b);
    let pm = eval(m)?;
    let sag = pm.sub(pa.lerp(pb, 0.5)).length();
    if sag > chord_tol && depth < 16 {
        subdivide(eval, a, m, pa, pm, chord_tol, depth + 1, out)?;
        subdivide(eval, m, b, pm, pb, chord_tol, depth + 1, out)?;
    } else {
        out.push(pb);
    }
    Ok(())
}

fn circumcenter(a: Point2, b: Point2, c: Point2) -> Option<Point2> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    let scale = a.distance(b).max(b.distance(c)).max(a.distance(c));
    if d.abs() <= 1e-12 * scale * scale {
        return None;
    }
    let a2 = a.x * a.x + a.y * a.y;
    let b2 = b.x * b.x + b.y * b.y;
    let c2 = c.x * c.x + c.y * c.y;
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    Some(Point2::new(ux, uy))
}

/// Faces are addressed by the same persistent names the mesh snapshot uses
/// ([`super::face_names`]: the record's name, or `#<index>` when it has none).
fn find_face<'a>(solid: &'a BrepSolid, name: &str) -> Option<&'a FaceRecord> {
    let names = super::face_names(solid);
    solid.shells.iter().flat_map(|s| s.faces.iter()).zip(names).find(|(_, n)| n == name).map(|(f, _)| f)
}

/// The machine-space plane of a face when it is a plane perpendicular to the
/// machine Z axis: `(z, upward)`.
pub fn horizontal_plane(face: &FaceRecord, transform: &RigidTransform, tol: &Tolerance) -> Option<(f64, bool)> {
    let Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else { return None };
    let n = u_dir.cross(*v_dir);
    let n = transform.apply_vector(Point3::new(n.x, n.y, n.z)).normalized()?;
    if !tol.is_vertical(n) {
        return None;
    }
    let o = to_machine(transform, *origin);
    let up = if face.same_sense { n.z > 0.0 } else { n.z < 0.0 };
    Some((o.z, up))
}

/// The region of a planar face perpendicular to the machine Z axis, from
/// its exact loops. Loop orientation comes from nesting (outer loops
/// counter-clockwise), not from the kernel's coedge sense.
pub fn face_region(
    solid: &BrepSolid,
    face_name: &str,
    transform: &RigidTransform,
    chord_tol: f64,
    tol: &Tolerance,
) -> Result<PlanarRegion, ExtractError> {
    let face = find_face(solid, face_name).ok_or_else(|| ExtractError::NoSuchFace(face_name.to_string()))?;
    let (z, upward) = horizontal_plane(face, transform, tol).ok_or_else(|| ExtractError::NotHorizontalPlane(face_name.to_string()))?;
    let mut loops = Vec::new();
    for l in &face.loops {
        let mut segs = Vec::new();
        for co in &l.coedges {
            let edge = solid.edges.iter().find(|e| e.id == co.edge_id).ok_or_else(|| ExtractError::Kernel(format!("edge {} missing", co.edge_id)))?;
            if edge.degenerate {
                continue;
            }
            let (s, zr) = curve_segments(&edge.curve, edge.t0, edge.t1, co.forward, transform, chord_tol, tol)?;
            let span = (zr[0] - z).abs().max((zr[1] - z).abs());
            if span > 100.0 * tol.distance_eps {
                return Err(ExtractError::NotPlanar { what: format!("edge {} of face {face_name:?}", co.edge_id), z_span: span });
            }
            segs.extend(s);
        }
        // The coedges are head to tail; chain tolerates either sense.
        let chains = chain_undirected(segs, tol);
        for c in chains {
            if c.closed {
                loops.push(Loop::new(c.segments));
            } else {
                return Err(ExtractError::Planar(PlanarError::Degenerate(format!("a loop of face {face_name:?} is open"))));
            }
        }
    }
    let region = Region::from_unoriented(loops, tol)?;
    Ok(PlanarRegion { region, z, upward })
}

/// Every face of the body that is a horizontal plane in machine space, as
/// `(name, region)`, in the kernel's face order.
pub fn horizontal_faces(solid: &BrepSolid, transform: &RigidTransform, chord_tol: f64, tol: &Tolerance) -> Vec<(String, PlanarRegion)> {
    let mut out = Vec::new();
    let names = super::face_names(solid);
    for (face, name) in solid.shells.iter().flat_map(|s| s.faces.iter()).zip(names) {
        if horizontal_plane(face, transform, tol).is_none() {
            continue;
        }
        if let Ok(r) = face_region(solid, &name, transform, chord_tol, tol) {
            out.push((name, r));
        }
    }
    out
}

/// The regions of a sketch profile whose plane is perpendicular to the
/// machine Z axis, one per profile region (outer + holes).
pub fn sketch_regions(profile: &SketchProfile, transform: &RigidTransform, chord_tol: f64, tol: &Tolerance) -> Result<Vec<PlanarRegion>, ExtractError> {
    let n = transform.apply_vector(Point3::new(profile.z_axis.x, profile.z_axis.y, profile.z_axis.z)).normalized().ok_or_else(|| ExtractError::Kernel("sketch plane has no normal".into()))?;
    if !tol.is_vertical(n) {
        return Err(ExtractError::NotHorizontalPlane("sketch".into()));
    }
    let z = to_machine(transform, profile.origin).z;
    let mut out = Vec::new();
    for region in &profile.regions {
        let mut loops = Vec::new();
        for pl in region {
            let mut segs = Vec::new();
            for curve in &pl.curves {
                let [t0, t1] = curve.domain().map_err(ExtractError::Kernel)?;
                let (s, zr) = curve_segments(curve, t0, t1, true, transform, chord_tol, tol)?;
                let span = (zr[0] - z).abs().max((zr[1] - z).abs());
                if span > 100.0 * tol.distance_eps {
                    return Err(ExtractError::NotPlanar { what: "sketch curve".into(), z_span: span });
                }
                segs.extend(s);
            }
            for c in chain_undirected(segs, tol) {
                if c.closed {
                    loops.push(Loop::new(c.segments));
                } else {
                    return Err(ExtractError::Planar(PlanarError::Degenerate("a sketch profile loop is open".into())));
                }
            }
        }
        out.push(PlanarRegion { region: Region::from_unoriented(loops, tol)?, z, upward: n.z > 0.0 });
    }
    Ok(out)
}

/// Chains through the named edges, at one machine height. Edges are joined
/// by shared endpoints in either sense; a closed ring is flagged closed.
pub fn edge_chains(solid: &BrepSolid, edge_names: &[&str], transform: &RigidTransform, chord_tol: f64, tol: &Tolerance) -> Result<Vec<PlanarChain>, ExtractError> {
    let mut segs = Vec::new();
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    for name in edge_names {
        let edge = solid.edges.iter().find(|e| e.name.as_deref() == Some(*name)).ok_or_else(|| ExtractError::NoSuchEdge(name.to_string()))?;
        let (s, zr) = curve_segments(&edge.curve, edge.t0, edge.t1, true, transform, chord_tol, tol)?;
        zmin = zmin.min(zr[0]);
        zmax = zmax.max(zr[1]);
        segs.extend(s);
    }
    if segs.is_empty() {
        return Err(ExtractError::Planar(PlanarError::InvalidInput("no edges selected".into())));
    }
    if zmax - zmin > 100.0 * tol.distance_eps {
        return Err(ExtractError::NotPlanar { what: "edge chain".into(), z_span: zmax - zmin });
    }
    let z = 0.5 * (zmin + zmax);
    Ok(chain_undirected(segs, tol).into_iter().map(|chain| PlanarChain { chain, z }).collect())
}

