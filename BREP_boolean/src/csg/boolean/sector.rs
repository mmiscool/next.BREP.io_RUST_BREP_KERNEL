//! A fragment that borders a section already knows which side of the other
//! operand it lies on: at a point of the section, the fragment's own surface
//! leaves the section in its in-material direction (the binormal, the
//! surface normal crossed with the section tangent), and that direction
//! points either into the other operand's material or out of it. The other
//! operand's face the section was cut on gives the test: its outward normal
//! against the fragment's binormal, both projected into the plane normal to
//! the section tangent, so a section curve that sits a fit residual off
//! either carrier does not matter. No point-in-solid query is needed.
//!
//! Where the two carriers are tangent at the section (the binormal within
//! 10° of the other face's tangent plane) the reading is taken further from
//! the section, on a ladder of step-off distances: the weld band, twice it,
//! then a third more per step for at most twenty steps, until the fragment's
//! surface has separated from the other carrier by the band, in which
//! direction it did so being the verdict. Eight consecutive readings that
//! stay inside the band declare the carriers coincident ("4 is not enough",
//! the one constant copied as a number), and a coincident reading is left to
//! the ray cast and the cosurface rules.
//!
//! The sector decides In/Out only. It declines a fragment whose face has a
//! cosurface partner, a section that rides an existing edge (`shared_edge`,
//! or its midpoint within the boundary band of the other face's own edges:
//! the sector there is bounded by that face's neighbour, which one face
//! cannot read), a sample it cannot evaluate (a pole, a degenerate tangent),
//! two sections of one fragment that disagree, and a coincident ladder. Every declined
//! fragment takes the ray-cast path exactly as before. Fragments that
//! border no section inherit by adjacency as before (the catalogue's
//! `CollectFaces`; `side_keep_decisions` already floods only across operand
//! coedges and never across a barrier).
//!
//! Behind `sector_classification` ([`BooleanOptions`], on by default since
//! the 2026-10-11 census) with `BREP_SECTOR_CLASS` overriding either way:
//! `off` or `0` is the ray cast alone, the escape hatch; `check` runs both on
//! every eligible fragment, returns the ray cast's verdict and appends one
//! line per fragment to `BREP_SECTOR_CENSUS=<file>` (`sector census:`, with
//! `BREP_CENSUS_LABEL`); anything else is the sector as the primary verdict,
//! the ray cast for the declined fragments only.
use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectorMode {
    Off,
    Check,
    On,
}

/// [`BooleanOptions::sector_classification`] under its environment override.
pub(super) fn sector_mode(options: &BooleanOptions) -> SectorMode {
    match mode_switch!(live "BREP_SECTOR_CLASS", options.sector_classification) {
        crate::env_read::Mode::Off => SectorMode::Off,
        crate::env_read::Mode::Check => SectorMode::Check,
        crate::env_read::Mode::On => SectorMode::On,
    }
}

/// Consecutive step-off readings whose chord stays within
/// [`COINCIDENT_ANGLE_SIN`] of the other carrier before the carriers are
/// declared coincident.
const COINCIDENT_READINGS: usize = 8;
/// sin 0.01°: a step-off chord that leaves the other carrier's tangent plane
/// by less than this reads as lying on it.
const COINCIDENT_ANGLE_SIN: f64 = 1.745_329_243_133_368e-4;
/// sin 10°: binormal pairs closer than this to the other carrier's tangent
/// plane are read on the step-off ladder.
const TANGENT_BAND_SIN: f64 = 0.173_648_177_666_930_33;
/// The least `side` an exact section's reading takes as a crossing under
/// `BREP_SECTOR_EXACT_SIDE`: far above the round-off of two unit normals on a
/// section that lies on both carriers, far below any crossing a model draws.
const EXACT_SIDE_FLOOR: f64 = 1e-9;

/// `BREP_SECTOR_EXACT_SIDE` (default on, `=0` restores the step-off ladder there): see
/// EXACT SIDE in [`sample`]. It acts only with
/// [`crate::feature_scale::imprint_enabled`], as `BREP_FEATURE_SCALE_SELECT` does:
/// a crossing below the band read right on a sliver the imprint never split out
/// is still a sliver the assembly sews shut, and with `BREP_FEATURE_SCALE_IMPRINT=0`
/// the wedge a millionth steep built a valid two-face solid a third short of its
/// volume where it had refused.
fn exact_side_enabled() -> bool {
    crate::switch!("BREP_SECTOR_EXACT_SIDE", true) && crate::feature_scale::imprint_enabled()
}
/// Most step-off readings before the ladder gives up.
const LADDER_STEPS: usize = 20;
/// Growth of the step-off distance per ladder step after the first two.
const LADDER_GROWTH: f64 = 3.0;
/// Fractions of the section coedge sampled, the midpoint first.
const SAMPLE_FRACTIONS: [f64; 11] = [
    0.5,
    1.0 / 11.0,
    2.0 / 11.0,
    3.0 / 11.0,
    4.0 / 11.0,
    5.0 / 11.0,
    6.0 / 11.0,
    7.0 / 11.0,
    8.0 / 11.0,
    9.0 / 11.0,
    10.0 / 11.0,
];

/// What the sector read for one fragment.
#[derive(Clone, Copy, Debug)]
pub struct SectorVerdict {
    /// `None` when the sector declined the fragment; `route` says why.
    pub class: Option<PointClass>,
    /// `transverse`, `ladder`, or the decline reason.
    pub route: &'static str,
    /// Step-off readings taken over every sample.
    pub ladder_steps: usize,
    /// Samples evaluated over every section coedge.
    pub samples: usize,
}

impl SectorVerdict {
    fn declined(route: &'static str, ladder_steps: usize, samples: usize) -> Self {
        Self { class: None, route, ladder_steps, samples }
    }
}

/// The counters one selection reports (`select.sector_*`, `select.votes`).
#[derive(Clone, Copy, Debug, Default)]
pub struct SelectCensus {
    /// Fragments bordering a section the sector read.
    pub eligible: u64,
    /// Of those, fragments the sector decided.
    pub sector_primary: u64,
    /// Of those, fragments the sector declined to the ray cast.
    pub sector_fallback: u64,
    /// Check mode: sector verdicts the ray cast contradicted.
    pub disagree: u64,
    /// Fragments the ray cast classified.
    pub ray_casts: u64,
    /// Ray casts whose primary point read On and whose extras voted.
    pub on_votes: u64,
    /// Ray casts whose In/Out primary the extras overturned.
    pub overturns: u64,
}

/// Everything the sector needs of one boolean, built once per selection.
pub struct SectorContext<'a> {
    pieces: HashMap<u64, &'a ImprintPieceRecord>,
    faces: HashMap<FaceKey, &'a FaceRecord>,
    edges: HashMap<(u8, u64), &'a EdgeRecord>,
    cosurface: HashSet<FaceKey>,
    /// A section whose midpoint lies within this of the other face's own
    /// edges rides that boundary: ten welds.
    boundary_band: f64,
    /// The classifier's On band: a step-off reading inside it has not
    /// separated the carriers.
    band: f64,
    /// Farther than this off the other carrier a section sample is not on
    /// it, and the ladder stops here.
    extent: f64,
}

impl<'a> SectorContext<'a> {
    pub fn new(
        imprint: &'a ImprintResultRecord,
        pristine_a: &'a BrepSolid,
        pristine_b: &'a BrepSolid,
        tolerance: f64,
    ) -> Self {
        let pieces = imprint.pieces.iter().map(|piece| (piece.id, piece)).collect();
        let mut faces: HashMap<FaceKey, &FaceRecord> = HashMap::default();
        let mut edges: HashMap<(u8, u64), &EdgeRecord> = HashMap::default();
        for (operand, solid) in [(0u8, pristine_a), (1u8, pristine_b)] {
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                faces.insert(FaceKey { operand, face_id: face.id }, face);
            }
            for edge in &solid.edges {
                edges.insert((operand, edge.id), edge);
            }
        }
        let cosurface = imprint
            .cosurface_pairs
            .iter()
            .flat_map(|(first, second)| [*first, *second])
            .collect();
        let mut bounds = crate::Aabb::from_points(pristine_a.vertices.iter().map(|vertex| vertex.point));
        bounds.include(crate::Aabb::from_points(pristine_b.vertices.iter().map(|vertex| vertex.point)));
        let extent = bounds.maximum.sub(bounds.minimum).length().max(1.0);
        Self {
            pieces,
            faces,
            edges,
            cosurface,
            boundary_band: 10.0 * assembler_weld(tolerance),
            band: tolerance * 10.0,
            extent,
        }
    }

    /// Is `point` within the boundary band of one of `face`'s own edges? A
    /// seam (an edge the face uses twice) bounds the face against itself,
    /// so the face's own normal gives the sector there; it is not a boundary.
    fn near_boundary(&self, operand: u8, face: &FaceRecord, point: Vec3) -> bool {
        let coedges: Vec<&CoedgeRecord> = face.loops.iter().flat_map(|loop_record| &loop_record.coedges).collect();
        for coedge in &coedges {
            if coedges.iter().filter(|other| other.edge_id == coedge.edge_id).count() > 1 {
                continue;
            }
            let Some(edge) = self.edges.get(&(operand, coedge.edge_id)) else {
                continue;
            };
            if let Ok(projection) = project_point_to_curve(&edge.curve, point) {
                if projection.distance <= self.boundary_band {
                    return true;
                }
            }
        }
        false
    }
}

/// Does the fragment border a section the sector can read?
pub fn borders_section(fragment: &FaceFragmentRecord) -> bool {
    fragment
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .any(|coedge| matches!(coedge.source, FragmentEdgeSource::Imprint { .. }))
}

enum Sample {
    Class(PointClass, usize),
    Coincident(usize),
    Skip,
}

fn oriented_normal(face: &FaceRecord, u: f64, v: f64) -> Option<Vec3> {
    let normal = face.surface.normal(u, v).ok()?;
    Some(if face.same_sense { normal } else { normal.scale(-1.0) })
}

fn precise_sector_foot(
    surface:&crate::NurbsSurface, point:Vec3, original:crate::SurfaceProjection,
)->Option<crate::SurfaceProjection> {
    let precision=1e-12*(1.0+point.length());
    if original.distance<=precision || original.distance>crate::tolerance::DEFAULT_MODEL {
        return None;
    }
    let refined=crate::projection::refine_trim_foot(surface,point,original).ok()?;
    (refined.distance<original.distance && point.sub(refined.point).length()<=precision)
        .then_some(refined)
}

/// One reading at parameter `t` of a section coedge of `fragment`, against
/// the other operand's face `other` the section was cut on.
fn sample(
    context: &SectorContext<'_>,
    fragment: &FaceFragmentRecord,
    pcurve: &NurbsCurve,
    t: f64,
    other: &FaceRecord,
    refine_projection: bool,
) -> Result<Sample, KernelRefusal> {
    let Ok(derivatives) = pcurve.derivatives(t, 1) else {
        return Ok(Sample::Skip);
    };
    let (uv, duv) = (derivatives[0], derivatives[1]);
    let Ok(surface) = fragment.surface.derivatives(uv.x, uv.y, 1) else {
        return Ok(Sample::Skip);
    };
    let (point, su, sv) = (surface[0][0], surface[1][0], surface[0][1]);
    // The section's 3-D tangent in the coedge's traversal direction, and the
    // fragment's oriented normal; the binormal is the in-material direction
    // (material lies to the left of a loop walked with the normal up).
    let Ok(tangent) = su.scale(duv.x).add(sv.scale(duv.y)).normalized() else {
        return Ok(Sample::Skip);
    };
    let Ok(normal) = su.cross(sv).normalized() else {
        return Ok(Sample::Skip);
    };
    let normal = if fragment.same_sense { normal } else { normal.scale(-1.0) };
    let Ok(binormal) = normal.cross(tangent).normalized() else {
        return Ok(Sample::Skip);
    };
    let projection = project_point_to_surface(&other.surface, point).or_refuse(KernelStage::Select, "project_point_to_surface")?;
    if projection.distance > 1e-3 * context.extent {
        return Ok(Sample::Skip);
    }
    let Some(other_normal) = oriented_normal(other, projection.u, projection.v) else {
        return Ok(Sample::Skip);
    };
    // Both directions already lie in the plane normal to the section
    // tangent up to the fit residual; project the other normal there too.
    let other_in_plane = other_normal.sub(tangent.scale(other_normal.dot(tangent)));
    let Ok(other_in_plane) = other_in_plane.normalized() else {
        return Ok(Sample::Skip);
    };
    let side = binormal.dot(other_in_plane);
    if side.abs() >= TANGENT_BAND_SIN {
        return Ok(Sample::Class(if side > 0.0 { PointClass::Out } else { PointClass::In }, 0));
    }
    // A nearest-foot query can stop at the model's on-carrier band while
    // this sector needs a round-off-sized residual. Refine only that existing
    // branch, and retain every original fallback unless it meets the same
    // exact-side admission test.
    if refine_projection && exact_side_enabled() && side.abs() >= EXACT_SIDE_FLOOR {
        if let Some(foot) = precise_sector_foot(&other.surface, point, projection) {
            if let Some(n) = oriented_normal(other, foot.u, foot.v) {
                if let Ok(n) = n.sub(tangent.scale(n.dot(tangent))).normalized() {
                    let refined_side=binormal.dot(n);
                    if refined_side.abs()>=EXACT_SIDE_FLOOR {
                        return Ok(Sample::Class(if refined_side>0.0 {PointClass::Out} else {PointClass::In},0));
                    }
                }
            }
        }
    }
    // EXACT SIDE (`BREP_SECTOR_EXACT_SIDE`, default on): on a section that
    // lies on the other carrier to round-off (an analytic lane's exact curve),
    // a small crossing angle is still a crossing, and `side` reads its sign.
    // The fragment's normal is orthogonal to its binormal, so `side` is the
    // binormal against the two normals' true difference; an error in the
    // section tangent changes it only relatively. A crossing of 0.05° reads
    // sin 8.7e-4 there, while the ladder below cannot see it: two balls of
    // radius 5 whose centres are 9.999999 apart meet in a lens 1e-6 thick,
    // the cap never parts from the other ball by the band, and the ladder
    // stepped past the far rim and read Out (thin-lens: no boundary faces).
    // A tangent pair's exact section reads `side` at round-off and still
    // takes the ladder; a fitted section sits off the carrier and does too.
    if exact_side_enabled()
        && side.abs() >= EXACT_SIDE_FLOOR
        && point.sub(projection.point).length() <= 1e-12 * (1.0 + point.length())
    {
        return Ok(Sample::Class(if side > 0.0 { PointClass::Out } else { PointClass::In }, 0));
    }
    // Tangent carriers: step off the section along the binormal, on the
    // fragment's own surface, until it has separated from the other carrier
    // by more than the band and more than the residual the section itself
    // sits off that carrier (a near-tangent section can lie 1e-4 off a
    // carrier; the reading must be the surfaces parting, not the fit).
    // The residual is subtracted from every reading: a sample displaced by
    // `e` along the binormal from the true contact reads `h(e)`, and the
    // reading at distance `d` is `h(e + d) - h(e)`; with the threshold at
    // four times `|h(e)|` the sign of that difference is the sign of the
    // separation for both the quadratic (tangent) and the linear (small
    // crossing angle) models.
    let height_at_section = point.sub(projection.point).dot(other_normal);
    let threshold = context.band.max(4.0 * height_at_section.abs());
    let mut distance = context.band;
    let mut flat_readings = 0usize;
    for step in 0..LADDER_STEPS {
        let probe = point.add(binormal.scale(distance));
        let on_fragment = project_point_to_surface(&fragment.surface, probe).or_refuse(KernelStage::Select, "project_point_to_surface")?.point;
        let on_other = project_point_to_surface(&other.surface, on_fragment).or_refuse(KernelStage::Select, "project_point_to_surface")?;
        let Some(other_normal) = oriented_normal(other, on_other.u, on_other.v) else {
            return Ok(Sample::Skip);
        };
        let separation = on_fragment.sub(on_other.point).dot(other_normal) - height_at_section;
        if separation.abs() > threshold {
            return Ok(Sample::Class(if separation > 0.0 { PointClass::Out } else { PointClass::In }, step + 1));
        }
        // The chord's angle out of the other carrier's tangent plane: a
        // tangent pair's grows with the distance, a coincident pair's stays
        // at nothing.
        let chord = on_fragment.sub(point).length().max(distance);
        if separation.abs() <= COINCIDENT_ANGLE_SIN * chord {
            flat_readings += 1;
            if flat_readings >= COINCIDENT_READINGS {
                return Ok(Sample::Coincident(step + 1));
            }
        } else {
            flat_readings = 0;
        }
        distance = if step == 0 { 2.0 * context.band } else { distance * LADDER_GROWTH };
        if distance > context.extent {
            break;
        }
    }
    Ok(Sample::Skip)
}

/// The sector's verdict for one fragment: In or Out against the other
/// operand, or declined with the reason.
pub fn sector_class(context: &SectorContext<'_>, fragment: &FaceFragmentRecord) -> Result<SectorVerdict, KernelRefusal> {
    let enabled=crate::switch!("BREP_SECTOR_PRECISE_FOOT",true);
    let check=crate::switch!("BREP_SECTOR_PRECISE_FOOT_CHECK",false);
    let result=sector_class_with_precision(context,fragment,enabled);
    if check {
        let alternate=sector_class_with_precision(context,fragment,!enabled);
        let changed=match (&result,&alternate) {
            (Ok(a),Ok(b))=>a.class!=b.class || a.route!=b.route,
            (Err(_),Err(_))=>false,
            _=>true,
        };
        if changed {
            let (old,new)=if enabled {(&alternate,&result)} else {(&result,&alternate)};
            eprintln!("SECTOR_PRECISE_FOOT operand={} face={} old={old:?} new={new:?} applied={enabled}",
                fragment.operand,fragment.source_face_id);
        }
    }
    result
}

fn sector_class_with_precision(context:&SectorContext<'_>,fragment:&FaceFragmentRecord,refine_projection:bool)->Result<SectorVerdict,KernelRefusal> {
    let enabled = crate::switch!("BREP_SECTOR_PARTIAL_BOUNDARY", true);
    let check = crate::switch!("BREP_SECTOR_PARTIAL_BOUNDARY_CHECK", false);
    if !check {
        return sector_class_impl(context, fragment, enabled, refine_projection);
    }
    let old = sector_class_impl(context, fragment, false, refine_projection);
    let new = sector_class_impl(context, fragment, true, refine_projection);
    let changed = match (&old, &new) {
        (Ok(a), Ok(b)) => a.class != b.class || a.route != b.route,
        (Err(_), Err(_)) => false,
        _ => true,
    };
    if changed {
        eprintln!("SECTOR_PARTIAL_BOUNDARY operand={} face={} old={old:?} new={new:?} applied={enabled}",
            fragment.operand, fragment.source_face_id);
    }
    if enabled { new } else { old }
}

fn sector_class_impl(context: &SectorContext<'_>, fragment: &FaceFragmentRecord, partial_boundary: bool, refine_projection: bool) -> Result<SectorVerdict, KernelRefusal> {
    let own = FaceKey { operand: fragment.operand, face_id: fragment.source_face_id };
    if context.cosurface.contains(&own) {
        return Ok(SectorVerdict::declined("cosurface", 0, 0));
    }
    let mut verdict: Option<PointClass> = None;
    let mut route = "transverse";
    let mut ladder_steps = 0usize;
    let mut samples = 0usize;
    let mut any = false;
    for coedge in fragment.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let FragmentEdgeSource::Imprint { piece_id } = coedge.source else {
            continue;
        };
        any = true;
        let Some(piece) = context.pieces.get(&piece_id) else {
            return Ok(SectorVerdict::declined("no_piece", ladder_steps, samples));
        };
        if piece.shared_edge.is_some() {
            return Ok(SectorVerdict::declined("shared_edge", ladder_steps, samples));
        }
        let Some((other_key, other)) = piece
            .support_faces
            .iter()
            .find(|key| key.operand != fragment.operand)
            .and_then(|key| context.faces.get(key).map(|face| (*key, *face)))
        else {
            return Ok(SectorVerdict::declined("no_other_face", ladder_steps, samples));
        };
        let [t0, t1] = coedge.pcurve.domain().or_refuse(KernelStage::Select, "domain")?;
        // A section running along the other face's boundary (the bore's
        // mouth ring, a tube's rim) is bounded there by that face's
        // neighbour as well; one face's normal does not give the sector.
        if let Ok(uv) = coedge.pcurve.evaluate((t0 + t1) * 0.5) {
            if let Ok(midpoint) = fragment.surface.evaluate(uv.x, uv.y) {
                if context.near_boundary(other_key.operand, other, midpoint) {
                    // This section cannot read a sector bounded by two faces.
                    // Another section through the interior of the other face
                    // can still classify the connected fragment. Keep those
                    // readings, requiring agreement as below; without any
                    // usable reading the result remains undecided.
                    if partial_boundary {
                        continue;
                    }
                    return Ok(SectorVerdict::declined("on_boundary", ladder_steps, samples));
                }
            }
        }
        let mut coedge_verdict = None;
        // RULING SIDE (`BREP_SECTOR_EXTRUSION_RULING_SIDE`, default on, `=0` restores;
        // `_CHECK` logs only): a crossing ruling of two exact extrusions
        // carries a certified crossing sign from the imprint; the side reads
        // from it, not from samples, which cannot resolve a crossing angle
        // below their own foot residual.
        let (ruling_on, ruling_check) = ruling_side_mode();
        let ruling = if ruling_on || ruling_check { certified_ruling_class(context, fragment, own, other_key, piece, coedge) } else { None };
        if ruling_on {
            if let Some(class) = ruling {
                coedge_verdict = Some(class);
            }
        }
        if coedge_verdict.is_none() {
        for fraction in SAMPLE_FRACTIONS {
            samples += 1;
            match sample(context, fragment, &coedge.pcurve, t0 + (t1 - t0) * fraction, other, refine_projection)? {
                Sample::Class(class, steps) => {
                    ladder_steps += steps;
                    if steps > 0 {
                        route = "ladder";
                    }
                    coedge_verdict = Some(class);
                    break;
                }
                Sample::Coincident(steps) => {
                    return Ok(SectorVerdict::declined("coincident", ladder_steps + steps, samples));
                }
                Sample::Skip => {}
            }
        }
        }
        if ruling_check && ruling.is_some() && ruling != coedge_verdict {
            eprintln!(
                "BREP_SECTOR_EXTRUSION_RULING_SIDE check: operand {} face {} piece {piece_id}: certified {:?}, sampled {:?}",
                fragment.operand, fragment.source_face_id, ruling, coedge_verdict
            );
        }
        let Some(class) = coedge_verdict else {
            return Ok(SectorVerdict::declined("undecided", ladder_steps, samples));
        };
        match verdict {
            None => verdict = Some(class),
            Some(known) if known != class => {
                return Ok(SectorVerdict::declined("conflict", ladder_steps, samples));
            }
            Some(_) => {}
        }
    }
    if !any {
        return Ok(SectorVerdict::declined("no_section", 0, 0));
    }
    Ok(SectorVerdict { class: verdict, route, ladder_steps, samples })
}

/// (applied, check-only) for `BREP_SECTOR_EXTRUSION_RULING_SIDE` (default on,
/// `=0` restores the sampled reading): on applies the certified ruling side;
/// with it `=0`, `_CHECK=1` only logs where the two differ.
fn ruling_side_mode() -> (bool, bool) {
    let on = crate::switch!("BREP_SECTOR_EXTRUSION_RULING_SIDE", true);
    let check = crate::switch!("BREP_SECTOR_EXTRUSION_RULING_SIDE_CHECK", false);
    (on, check && !on)
}

/// The side of `fragment` at its section coedge `coedge` (imprint piece
/// `piece`) from the piece's certified crossing ruling, or `None` when the
/// piece carries none that validates against its two support charts, or the
/// coedge does not provably run along that ruling.
///
/// On two extrusions along Z, with chart normals `f_k (P_k' x Z)` (`f_k` the
/// certificate's normal factors), oriented by each face's sense `σ`, and the
/// coedge running `q Z`, the sector's binormal is `-q σ_own f_own P_own'` and
/// its side reading against the other normal is
/// `-q σ_own σ_other f_0 f_1 cross2(P_own', P_other')`; the certificate's
/// crossing sign is `cross2(P_0', P_1')`, negated when this face is support 1.
/// Positive is Out, as in [`sample`].
fn certified_ruling_class(
    context: &SectorContext<'_>,
    fragment: &FaceFragmentRecord,
    own: FaceKey,
    other_key: FaceKey,
    piece: &ImprintPieceRecord,
    coedge: &crate::fragment::FragmentCoedge,
) -> Option<PointClass> {
    let supports = piece.support_faces;
    let index = supports.iter().position(|key| *key == own)?;
    if supports[1 - index] != other_key {
        return None;
    }
    let charts = [&context.faces.get(&supports[0])?.surface, &context.faces.get(&supports[1])?.surface];
    let ruling = piece.certified_ruling(supports, charts)?;
    let other_face = context.faces.get(&other_key)?;
    ruling_side(
        RulingFacts {
            crossing_sign: ruling.crossing_sign(),
            normal_factors: ruling.normal_factors(),
            profile_is_u: ruling.profile_is_u()[index],
            sweep_sign: ruling.sweep_sign()[index],
            profile_box: ruling.profile_boxes()[index],
        },
        index,
        &coedge.pcurve,
        fragment.same_sense,
        other_face.same_sense,
    )
}

/// What [`ruling_side`] reads from a validated certificate, for the face that
/// is support `index` of the ruling's piece.
#[derive(Clone, Copy, Debug)]
struct RulingFacts {
    crossing_sign: i8,
    normal_factors: [i8; 2],
    profile_is_u: bool,
    sweep_sign: i8,
    profile_box: [f64; 2],
}

/// The side of a fragment at a certified ruling it borders, from the
/// certificate's facts for its own support `index` (0 or 1), the bordering
/// coedge's `pcurve` (in traversal direction) and the two faces' senses.
fn ruling_side(facts: RulingFacts, index: usize, pcurve: &NurbsCurve, own_same_sense: bool, other_same_sense: bool) -> Option<PointClass> {
    if index > 1 {
        return None;
    }
    // The coedge's direction along Z: its pcurve moves only in the chart's
    // sweep coordinate, and the sign of that motion is certified by a
    // strictly one-signed control polygon (the sign of a floating-point
    // difference is exact). Unit weights only, as the producer: then the
    // control coordinates are the chart parameters the boxes bound.
    if pcurve.control_points.iter().any(|point| point.w != 1.0) {
        return None;
    }
    let profile_is_u = facts.profile_is_u;
    let sweep = |point: &crate::Vec4| if profile_is_u { point.y } else { point.x };
    let profile = |point: &crate::Vec4| if profile_is_u { point.x } else { point.y };
    let mut direction = 0i8;
    for pair in pcurve.control_points.windows(2) {
        let step = sweep(&pair[1]) - sweep(&pair[0]);
        let here = if step > 0.0 { 1 } else if step < 0.0 { -1 } else { return None };
        if direction == 0 {
            direction = here;
        } else if direction != here {
            return None;
        }
    }
    if direction == 0 {
        return None;
    }
    // The coedge lies on the certified ruling: its profile coordinate stays
    // inside the certified parameter box (a control-polygon hull bound).
    let [low, high] = facts.profile_box;
    if pcurve.control_points.iter().any(|point| !(profile(point) >= low && profile(point) <= high)) {
        return None;
    }
    let q = direction * facts.sweep_sign;
    let sense = |same: bool| if same { 1i8 } else { -1 };
    let [f0, f1] = facts.normal_factors;
    let crossing = if index == 0 { facts.crossing_sign } else { -facts.crossing_sign };
    let side = -crossing * f0 * f1 * sense(own_same_sense) * sense(other_same_sense) * q;
    match side {
        1 => Some(PointClass::Out),
        -1 => Some(PointClass::In),
        _ => None,
    }
}

/// `BREP_SECTOR_CENSUS=<file>`: one line per eligible fragment in check mode
/// and one per selection, labelled by `BREP_CENSUS_LABEL`.
pub fn sector_census_line(line: impl FnOnce() -> String) {
    if let Ok(path) = std::env::var("BREP_SECTOR_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let label = std::env::var("BREP_CENSUS_LABEL").unwrap_or_default();
            let _ = writeln!(file, "sector census: label {label} {}", line());
        }
    }
}




