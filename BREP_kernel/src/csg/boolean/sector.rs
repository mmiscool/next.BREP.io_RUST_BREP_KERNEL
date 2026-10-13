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
pub(super) enum SectorMode {
    Off,
    Check,
    On,
}

/// [`BooleanOptions::sector_classification`] under its environment override.
pub(super) fn sector_mode(options: &BooleanOptions) -> SectorMode {
    match std::env::var("BREP_SECTOR_CLASS").as_deref() {
        Ok("0") | Ok("off") => SectorMode::Off,
        Ok("check") => SectorMode::Check,
        Ok(_) => SectorMode::On,
        Err(_) => {
            if options.sector_classification {
                SectorMode::On
            } else {
                SectorMode::Off
            }
        }
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
pub(super) struct SectorVerdict {
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
pub(super) struct SelectCensus {
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
pub(super) struct SectorContext<'a> {
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
    pub(super) fn new(
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
pub(super) fn borders_section(fragment: &FaceFragmentRecord) -> bool {
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

/// One reading at parameter `t` of a section coedge of `fragment`, against
/// the other operand's face `other` the section was cut on.
fn sample(
    context: &SectorContext<'_>,
    fragment: &FaceFragmentRecord,
    pcurve: &NurbsCurve,
    t: f64,
    other: &FaceRecord,
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
pub(super) fn sector_class(context: &SectorContext<'_>, fragment: &FaceFragmentRecord) -> Result<SectorVerdict, KernelRefusal> {
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
                    return Ok(SectorVerdict::declined("on_boundary", ladder_steps, samples));
                }
            }
        }
        let mut coedge_verdict = None;
        for fraction in SAMPLE_FRACTIONS {
            samples += 1;
            match sample(context, fragment, &coedge.pcurve, t0 + (t1 - t0) * fraction, other)? {
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

/// `BREP_SECTOR_CENSUS=<file>`: one line per eligible fragment in check mode
/// and one per selection, labelled by `BREP_CENSUS_LABEL`.
pub(super) fn sector_census_line(line: impl FnOnce() -> String) {
    if let Ok(path) = std::env::var("BREP_SECTOR_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let label = std::env::var("BREP_CENSUS_LABEL").unwrap_or_default();
            let _ = writeln!(file, "sector census: label {label} {}", line());
        }
    }
}

/// One fragment's two readings, for the differential test and the census.
#[derive(Clone, Debug)]
pub(super) struct FragmentReading {
    pub operand: u8,
    pub face_id: u64,
    pub sector: SectorVerdict,
    pub ray: PointClass,
    pub ray_route: &'static str,
    /// The ray cast's primary point read On and its extras decided.
    pub ray_voted: bool,
}

/// Both verdicts for every fragment of the pair that borders a section, the
/// operands prepared and imprinted as the pipeline prepares them.
pub(super) fn sector_readings(
    first: &BrepSolid,
    second: &BrepSolid,
    options: &BooleanOptions,
) -> Result<Vec<FragmentReading>, KernelRefusal> {
    let policy = boolean_policy(first, second, options)?;
    let (first, second) = prepared_operands(first, second, &policy)?;
    let tolerance = policy.model;
    let imprint = build_imprints(&first, &second, &boolean_imprint_options(options, tolerance))?;
    let (split_first, _) = apply_edge_splits_with_map(&first, 0, &imprint)?;
    let (split_second, _) = apply_edge_splits_with_map(&second, 1, &imprint)?;
    let fragments_a = fragment_solid(&split_first, 0, &imprint)?;
    let fragments_b = fragment_solid(&split_second, 1, &imprint)?;
    let context = SectorContext::new(&imprint, &first, &second, tolerance);
    let classifier_a = SolidClassifier::new(&first, tolerance).or_refuse(KernelStage::Select, "new")?;
    let classifier_b = SolidClassifier::new(&second, tolerance).or_refuse(KernelStage::Select, "new")?;
    let mut readings = Vec::new();
    for (fragments, classifier) in [(&fragments_a, &classifier_b), (&fragments_b, &classifier_a)] {
        for fragment in fragments {
            if !borders_section(fragment) {
                continue;
            }
            let sector = sector_class(&context, fragment)?;
            let (ray, ray_route) = classify_fragment_traced(classifier, fragment, false)?;
            readings.push(FragmentReading {
                operand: fragment.operand,
                face_id: fragment.source_face_id,
                sector,
                ray: ray.class,
                ray_route,
                ray_voted: route_voted(ray_route),
            });
        }
    }
    Ok(readings)
}

/// Did the ray cast's extras decide the verdict?
pub(super) fn route_voted(route: &str) -> bool {
    matches!(route, "on_kept" | "on_to_in" | "on_to_out" | "overturned")
}
