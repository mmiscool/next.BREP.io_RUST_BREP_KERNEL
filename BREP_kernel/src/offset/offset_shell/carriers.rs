use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use crate::offset_carve::carve_folded_trim_scanned;
use crate::offset_regularity::{scan_offset_regularity, Certainty, ScanBudget, TrimRegion};
use crate::{NurbsCurve, NurbsSurface};

pub(super) fn standalone_face(solid: &BrepSolid, face_id: u64) -> Result<BrepSolid, KernelRefusal> {
    let face = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .cloned()
        .ok_or_else(|| format!("offset_shell: missing face {face_id}")).or_refuse(KernelStage::Refine, "offset_shell_missing_face")?;
    let edge_ids = face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect::<HashSet<_>>();
    let edges = solid
        .edges
        .iter()
        .filter(|edge| edge_ids.contains(&edge.id))
        .cloned()
        .collect::<Vec<_>>();
    let vertex_ids = edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    let vertices = solid
        .vertices
        .iter()
        .filter(|vertex| vertex_ids.contains(&vertex.id))
        .cloned()
        .collect();
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: solid.id,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: 1,
            faces: vec![face],
        }],
        genus: 0,
        kind: crate::BodyKind::Solid,
    })
}

/// A cylinder or sphere has no regular inner skin once its signed radius
/// reaches zero. Do this on the source support, before fitting an offset:
/// fitting a zero-radius surface leaves line edges, and fitting past zero
/// creates a reflected surface that is no longer part of the shell boundary.
/// Keep the source face itself for the outer skin and distance classification.
pub(super) fn offset_support_collapsed(face: &FaceRecord, distance: f64) -> Result<bool, KernelRefusal> {
    use crate::AnalyticSurface;
    // Existing analytic carriers keep their established radius-collapse bar.
    // Only the newly recognized chart uses the generic scan's fold factor.
    if face.surface.analytic().is_none() {
        if let Some(factor) = certified_sphere_offset_factor(face, distance)? {
            return Ok(factor <= COLLAPSE_FACTOR);
        }
    }
    let (center, axis, radius) = match face.surface.analytic() {
        Some(AnalyticSurface::Sphere { frame, radius }) => (frame.origin, None, *radius),
        Some(AnalyticSurface::RuledRevolution {
            frame, rho0, rho1, ..
        }) if (rho0 - rho1).abs() <= 1e-9 * rho0.abs().max(1.0) => {
            (frame.origin, Some(frame.axis), (rho0 + rho1) * 0.5)
        }
        Some(AnalyticSurface::Revolution {
            frame, generatrix, ..
        }) if generatrix.degree == 1 && generatrix.control_points.len() == 2 => {
            let a = generatrix.control_points[0].point().or_refuse(KernelStage::Refine, "point")?.sub(frame.origin);
            let b = generatrix.control_points[1].point().or_refuse(KernelStage::Refine, "point")?.sub(frame.origin);
            let ra = a.sub(frame.axis.scale(a.dot(frame.axis)));
            let rb = b.sub(frame.axis.scale(b.dot(frame.axis)));
            if ra.sub(rb).length() > 1e-9 * ra.length().max(1.0) {
                return Ok(false);
            }
            (frame.origin, Some(frame.axis), ra.length())
        }
        _ => return offset_curvature_collapsed(face, distance),
    };
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (u, v) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    let mut radial = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(center);
    if let Some(axis) = axis {
        radial = radial.sub(axis.scale(radial.dot(axis)));
    }
    let normal = face.surface.normal(u, v).or_refuse(KernelStage::Refine, "normal")?;
    let sense = if face.same_sense { 1.0 } else { -1.0 };
    let inward_change = distance * sense * normal.dot(radial.normalized().or_refuse(KernelStage::Refine, "normalized")?);
    Ok(radius - inward_change <= 1e-9 * radius.max(distance.abs()).max(1.0))
}

/// The factor a fold has to reach for the offset's area element to count as
/// collapsed. `thicken` refuses a sheet at the same 1e-6 on the same condition,
/// which is why both read it through the shared scan; the fitted supports this
/// lane sees carry curvature noise far under that.
pub(super) const COLLAPSE_FACTOR: f64 = 1e-6;

/// A proved sphere has constant principal curvature even where its chart
/// collapses to a pole. The native 814 chart transposes the usual sphere:
/// generic curvature at its explicit pole coedge reads O(1e15) from roundoff,
/// inventing a discontinuous fold locus on a sphere of radius 37.31.
/// Recognize the same net in either chart and calibrate its radius change at
/// a regular station with the shared outward-normal evaluator. No fold
/// threshold or non-spherical scan changes.
fn certified_sphere_offset_factor(face: &FaceRecord, distance: f64) -> Result<Option<f64>, KernelRefusal> {
    if std::env::var("BREP_OFFSET_FOLD_CHART").as_deref() == Ok("0") {
        return Ok(None);
    }
    let mut geometry = face.surface.analytic().and_then(|shape| shape.sphere_geometry());
    if geometry.is_none() && face.surface.degree_u == 2 && face.surface.degree_v == 2 {
        let transposed = crate::offset::transpose_offset_surface(&face.surface)?;
        geometry = transposed.analytic().and_then(|shape| shape.sphere_geometry());
    }
    let Some((center, radius)) = geometry else { return Ok(None); };
    let [u0,u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0,v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let sample = crate::OffsetEvaluator::new("offset_shell", &face.surface,
        crate::OffsetNormal::FaceStable { same_sense: face.same_sense })
        .at((u0+u1)*0.5, (v0+v1)*0.5, -distance).or_refuse(KernelStage::Refine, "at")?;
    let alignment = sample.normal.dot(sample.source.sub(center)) / radius;
    if alignment.abs() < 0.5 { return Ok(None); }
    let factor = 1.0 - distance * alignment.signum() / radius;
    os_debug!("certified sphere factor face {}: {factor:.12} radius={radius:.12}", face.id);
    Ok(Some(factor))
}

/// The displacement along the parametrization normal `Su × Sv` that offset
/// shell's signed `distance` means for this face.
///
/// [`crate::NurbsSurface::principal_curvatures`] signs κ against `Su × Sv`; the
/// face's outward normal is `same_sense · n`; and offset shell's positive
/// `distance` moves OPPOSITE that outward normal. One place, so no lane
/// re-derives it.
pub(super) fn shell_displacement(face: &FaceRecord, distance: f64) -> f64 {
    let sense = if face.same_sense { 1.0 } else { -1.0 };
    -distance * sense
}

/// The lane [`offset_support_collapsed`] takes for a support with no radius to
/// read: a FITTED face — a fillet blend along a curved spine, a swept
/// band, an imported patch — is never `analytic()`, so the radius lanes above
/// see nothing and the inner skin of a fillet thinner than the wall is fitted
/// and kept as a REFLECTED band. Read the support's principal curvatures
/// instead, which is the same regularity condition `thicken`'s
/// `ensure_offsets_regular` states for a sheet: the equidistant surface folds
/// through the evolute wherever the per-direction area factor reaches zero.
/// Both now ask it through the shared scan in `offset/regularity.rs`.
///
/// Sign. [`crate::NurbsSurface::principal_curvatures`] signs κ against the
/// PARAMETRIZATION normal `Su × Sv`; the face's outward normal is
/// `same_sense · n`; and offset-shell's positive `distance` moves OPPOSITE that
/// outward normal. The displacement along `Su × Sv` is therefore
/// `−distance · same_sense`, and the scan's `1 − δ·κ` is this lane's old
/// `1 + distance · same_sense · κ` unchanged. Shelled by 1, a bore of radius 6
/// reads 7/6, the r=18 convex blend it runs into reads 17/18, and an r=0.5
/// convex fillet between them reads −1 — that fillet's inner skin is a band
/// bounded by the same two rails traced the wrong way round, which no weld can
/// make part of the shell. Dropping it lets the two neighbours meet in the sharp
/// inner miter that a d > r fillet is supposed to leave.
///
/// WHOLE-TRIM, not whole-face. A constant-radius fillet collapses at every
/// sample, which is the case this answers. A support that collapses only in
/// PART still keeps its regular portion and stays — but the part that counts is
/// now the part the face's own TRIM covers, not the part its carrier covers. A
/// booleaned face keeps the whole surface it was cut from, so a carrier reaching
/// a cone's apex, or a torus's inner equator, used to vote here even where no
/// skin is ever built. A support that folds over only PART of its trim is not
/// this lane's to answer: [`carve_folded_support`] splits that trim along the
/// fold locus and keeps the regular side.
fn offset_curvature_collapsed(face: &FaceRecord, distance: f64) -> Result<bool, KernelRefusal> {
    if distance == 0.0 || face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(false);
    }
    let displacement = shell_displacement(face, distance);
    let region = TrimRegion::from_face(face)?;
    let scan = scan_offset_regularity(
        &face.surface,
        &region,
        &[displacement],
        COLLAPSE_FACTOR,
        ScanBudget::SHELL_FACE,
    )?;
    os_debug!(
        "curvature collapse probe face {}: worst factor {:.6} at {} over {}/{} collapsed samples \
         inside the trim",
        face.id,
        scan.worst.map(|worst| worst.factor).unwrap_or(f64::INFINITY),
        scan.worst
            .map(|worst| format!("(u={:.6}, v={:.6})", worst.u, worst.v))
            .unwrap_or_else(|| "nowhere".to_string()),
        scan.collapsed,
        scan.sampled
    );
    if scan.sampled > 0 && scan.collapsed == scan.sampled && scan.between_regular == 0 {
        return Ok(true);
    }
    if scan.collapsed > 0 && offset_sections_collapsed(face, displacement)? {
        return Ok(true);
    }
    Ok(false)
}

/// A fitted circular strip can collapse to its spine even though interpolation
/// noise makes its curvature straddle COLLAPSE_FACTOR. Marching that noise
/// produces spurious fold islands or an untraceable level set. Check the
/// pointwise image of each quadratic section against the existing geometric
/// carrier-consistency tolerance instead. This is only used after the scan
/// actually finds a collapse: a wholly regular, thin inner skin stays regular.
///
/// Read every knot span, including its ends, over the trim's parameter box.
/// A uniform grid missed the largest normal errors on the reported blend.
/// The whole box is conservative for nonrectangular trims: any section which
/// still has width prevents omission. Unreadable normals also prevent it.
fn offset_sections_collapsed(face: &FaceRecord, displacement: f64) -> Result<bool, KernelRefusal> {
    let surface = &face.surface;
    let tolerance = KernelTolerances::default().pcurve_consistency;
    let [u0, u1, v0, v1] = TrimRegion::from_face(face)?.bounds();
    // Single-span quadratic sections are the circular cross sections used by
    // fitted fillets; do not infer a general free-form patch's rank from them.
    if surface.degree_v != 2 || surface.control_points[0].len() != 3 {
        return Ok(false);
    }
    let mut sampled = false;
    for span in surface.knots_u.windows(2) {
        let lo = span[0].max(u0);
        let hi = span[1].min(u1);
        if hi <= lo {
            continue;
        }
        for i in 0..=4 {
            let u = lo + (hi - lo) * i as f64 / 4.0;
            let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut high = low.scale(-1.0);
            let first = surface.evaluate(u, v0).or_refuse(KernelStage::Refine, "evaluate")?;
            let last = surface.evaluate(u, v1).or_refuse(KernelStage::Refine, "evaluate")?;
            // Do not mistake an already tiny source strip for a collapse.
            if last.sub(first).length() <= 2.0 * tolerance {
                return Ok(false);
            }
            for j in 0..=16 {
                let v = v0 + (v1 - v0) * j as f64 / 16.0;
                let Ok(point) = surface.evaluate(u, v) else {
                    return Ok(false);
                };
                let Ok(normal) = surface.normal(u, v) else {
                    return Ok(false);
                };
                let image = point.add(normal.scale(displacement));
                low = Vec3::new(low.x.min(image.x), low.y.min(image.y), low.z.min(image.z));
                high = Vec3::new(high.x.max(image.x), high.y.max(image.y), high.z.max(image.z));
                // Every point must lie within the carrier-consistency band
                // of the section's box centre, not of its first endpoint.
                if high.sub(low).length() > 2.0 * tolerance {
                    return Ok(false);
                }
            }
            sampled = true;
        }
    }
    Ok(sampled)
}

/// The source solid with one face's FOLDED part carved away — the support a
/// carrier should actually be built over when the offset folds across part of
/// its trim.
///
/// Why a whole cloned SOLID and not just a face: the carrier builder derives its
/// construction band from the SOURCE SOLID's extent
/// (`offset/offset.rs`, `offset_face_carrier_impl`), so handing it a standalone
/// face would quietly change every tolerance it then works to. The clone
/// replaces the one face and appends the edges and vertices the new boundary
/// needs; every other face keeps the entities it already names.
///
/// Why the SOURCE and not the carrier: the carrier is the source's pcurves
/// mapped through the offset surface, and `offset_surface`'s apex-cone pinch
/// retrim re-parameterizes a net whose TRIM reaches past the pinch. Carving
/// first means the trim stops AT the pinch, so the retrim does not fire and the
/// carved pcurves still name the pointwise offset. The two changes only compose
/// in that order.
///
/// The source SKIN is deliberately not carved. The material boundary there is
/// real all the way to the face's own rim; it is the cavity side that ends at
/// the fold.
pub(super) struct CarvedSupport {
    pub(super) solid: BrepSolid,
    pub(super) face: FaceRecord,
    /// Parameter-space share of the trim that survived — a report quantity.
    pub(super) kept_fraction: f64,
    /// How many pcurves of the kept loops are pieces of the fold locus: the new
    /// boundary, the rim the neighbours have to be re-solved against.
    pub(super) fold_boundary: usize,
    /// A finite cusp admitted only because retained planes bury its image.
    pub(super) shielded_cusp: bool,
}

/// Lane O row 35a's switch, default on since batch 17 (`0`/`off` restores the scan): the carve certifies a
/// circular cylinder's constant area factor instead of scanning it.
fn cylinder_carve_cert_on() -> bool {
    switch!("BREP_OFFSET_CYLINDER_CARVE_CERT", true)
}


pub(super) fn carve_folded_support(
    source: &BrepSolid,
    face: &FaceRecord,
    distance: f64,
) -> Result<Option<CarvedSupport>, KernelRefusal> {
    carve_folded_support_impl(source, face, distance, None)
}

/// A shell can also discard a cusp buried behind retained planar offsets.
/// Opening faces cannot shield it: their planes do not bound the cavity.
pub(super) fn carve_shell_support(
    source: &BrepSolid,
    face: &FaceRecord,
    distance: f64,
    openings: &[u64],
) -> Result<Option<CarvedSupport>, KernelRefusal> {
    carve_folded_support_impl(source, face, distance, Some(openings))
}

fn carve_folded_support_impl(
    source: &BrepSolid,
    face: &FaceRecord,
    distance: f64,
    openings: Option<&[u64]>,
) -> Result<Option<CarvedSupport>, KernelRefusal> {
    if distance == 0.0 || face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    if certified_sphere_offset_factor(face, distance)?.is_some_and(|factor| factor > COLLAPSE_FACTOR) {
        return Ok(None);
    }
    // Lane O row 35a (`BREP_OFFSET_CYLINDER_CARVE_CERT`, default off): a
    // circular cylinder's offset area factor is the constant `(R ± d)/R`
    // across the ruling direction (1 along it), so a factor above the collapse
    // bar is the scan's `Certainty::Regular` everywhere, read without the scan.
    if cylinder_carve_cert_on() {
        if let Some((_, _, _, factor)) = crate::offset::cylinder_offset_factor(&face.surface, face.same_sense, distance)? {
            if factor > COLLAPSE_FACTOR {
                os_debug!("certified cylinder factor face {}: {factor:.12}", face.id);
                return Ok(None);
            }
        }
    }
    let loops = face
        .loops
        .iter()
        .map(|loop_record| {
            loop_record
                .coedges
                .iter()
                .map(|coedge| coedge.pcurve.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let displacement = shell_displacement(face, distance);
    let mut outcome = carve_folded_trim_scanned(
        &face.surface,
        &loops,
        &[displacement],
        COLLAPSE_FACTOR,
        ScanBudget::SHELL_FACE,
    )?;
    let Some(carved) = outcome.carved.take() else {
        // Nothing to carve is not yet "build it whole": that is the third
        // outcome, and it is the scan's CERTAINTY that decides between the two.
        // A census that found nothing collapsed but could not clear every risky
        // cell, or ran out of descent budget, has not seen enough to certify
        // the support, and a carrier folded between its samples validates.
        return match outcome.scan.certify(&face.surface, &outcome.region, &[displacement], COLLAPSE_FACTOR) {
            Certainty::Regular => Ok(None),
            // The certainty step's own refinement reached a collapsed sample the
            // scan had not: a fold between the samples, on a support the carve
            // has already declined to divide. Never "build it whole".
            Certainty::Folded(sample) => Err(super::refusals::fold_between_samples(face.id, distance, sample)),
            Certainty::Uncertain {
                unresolved,
                budget_exhausted,
                least,
            } => Err(super::refusals::regularity_uncertain(
                face.id,
                distance,
                unresolved,
                budget_exhausted,
                least,
            )),
        };
    };

    let mut solid = source.clone();
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .chain(
            solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .flat_map(|face| &face.loops)
                .flat_map(|loop_record| {
                    std::iter::once(loop_record.id)
                        .chain(loop_record.coedges.iter().map(|coedge| coedge.id))
                }),
        )
        .max()
        .unwrap_or(0)
        + 1;

    // The sweep rule `thicken` applies does NOT apply here, and the difference
    // is worth stating because the obvious reading is wrong.
    //
    // A thicken is the UNION of the sheet's normal segments, so a dropped part
    // of the sheet must have its sweep inside the kept bodies or the answer is
    // smaller than the sheet sweeps. A shell's cavity is the EROSION of the
    // source — the points farther than d from every boundary face — whose
    // boundary is the offset surface trimmed where it collapses. A carve that
    // ends the cavity at the fold's pinch IS that erosion boundary, and drops
    // nothing the operation owes.
    //
    // Measured, because the reading is counter-intuitive: the apex cone's
    // dropped sliver FAILS the thicken rule. Near the apex the cone is thinner
    // than the offset, so the inward normal at (u=0.041667, v=0.982666) leaves
    // the solid entirely, reaching (0.128607, -0.511031, 11.348227) at 80% of
    // the -1.000000 displacement — outside a cone whose surface at that height
    // sits at ρ = 0.435. Applying the thicken rule here would refuse the one
    // configuration this lane was built for.

    // A support whose offset folds in a BAND is refused HERE, by name, and the
    // reason is the SHELL's, not the carve's.
    //
    // `offset/carve.rs` divides the trim correctly — two chords, three pieces,
    // the band dropped and a regular piece kept on either side of it — and
    // `thicken` builds a body per piece from exactly that division. This lane
    // cannot use it, because it deliberately does NOT carve the source SKIN:
    // the material boundary there is real all the way to the face's own rim,
    // and it is the cavity side that ends at the fold. That is right for the
    // configuration this lane was built for — an apex cone, where the cavity
    // ends at a POINT strictly inside a skin that is real to its rim, so the
    // skin has nothing to close against and needs nothing. A dropped BAND is a
    // two-dimensional region of skin with no offset image above it at all, and
    // the weld has nothing to close it with: measured on the R = 3, r = 2 ring
    // grown outward by 4, the assembly came back with 25 one-use edges along
    // the band's two rims.
    //
    // Carving the skin as well would close it, and it would close the WRONG
    // solid: decided 2026-09-17, the skin is never carved. Over the band the
    // material between the skin and wherever the offset stops is real — on the
    // ring above, the point half a unit out from the inner equator is in front
    // of no opening — and a two-body answer drops it. Where the band CAN close,
    // an outward shell of the one circular face of a partial revolve, the caps
    // meet on the axis and `revolved_wedge.rs` builds it in the meridian
    // half-plane before this lane is asked. Every other band still arrives here,
    // and refusing by name is what keeps a shell that cannot close from
    // arriving as an unexplained weld failure.
    if carved.kept.len() > 1 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_fold_band", format!(
            "offset_shell: the offset of source face {} folds in a BAND across its trim. The \
             fold locus divides it into {} regular piece(s) with {} folded one(s) dropped \
             between them, and thicken builds a body per piece — but this lane keeps the source \
             SKIN whole, out to the face's own rim, and over a dropped band that skin has no \
             offset image to close against. The shell refuses rather than assemble one it \
             cannot close",
            face.id,
            carved.kept.len(),
            carved.dropped.len()
        )));
    }

    // The cavity may end at a fold only where the fold is a PINCH.
    //
    // On the apex cone the kept piece's fold boundary is a parallel whose
    // offset image is ONE point, the offset cone's apex: the cavity closes there
    // and nothing is left to re-solve. A fold whose boundary runs ACROSS the
    // collapsing direction has an image with length — a CUSP edge of the
    // offset — and the erosion boundary near it is not the kept offset ending
    // at that edge but the kept offset trimmed against the offset's own OTHER
    // sheet, which nothing in this lane computes. Measured on a block whose top
    // face is the biquadratic bump `z = (1 − x²)(1 − y²)` shelled inward
    // through its bottom: at d = 0.3 and 0.4 its four corners fold, the carve
    // divides the trim along four chords, and the image of each fold boundary
    // spans a tenth of a unit. The census behind this check (2026-09-17 carve
    // record): every support any lib fixture or case row carves in this lane is
    // a pinch.
    let (fold_image_extent, fold_image_bar) = fold_image_extent(source, face, &carved.kept[0], distance)?;
    let shielded = if fold_image_extent > fold_image_bar && distance > 0.0 {
        match openings {
            Some(openings) => fold_image_shielded(source, face, &carved.kept[0], distance, openings)?,
            None => false,
        }
    } else {
        false
    };
    if fold_image_extent > fold_image_bar && !shielded {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_cusp", format!(
            "offset_shell: the offset of source face {} folds across part of its trim, and the \
             fold locus that bounds the kept part is not a PINCH — its offset image spans \
             {fold_image_extent:.6e} against the {fold_image_bar:.3e} a pinch can reach at the \
             fold level. The cavity would end on a CUSP edge of the offset, which has to be \
             re-solved against the offset's own other sheet, and nothing in this lane does; the \
             shell refuses rather than end the cavity there ({} kept piece, {} folded one(s) \
             dropped)",
            face.id,
            carved.kept.len(),
            carved.dropped.len()
        )));
    }

    let mut fold_boundary = 0usize;
    let mut new_loops = Vec::with_capacity(carved.kept[0].len());
    for carved_loop in &carved.kept[0] {
        let count = carved_loop.pcurves.len();
        if count == 0 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "offset_shell_carve_empty_loop", "offset_shell: the carve produced an empty loop"));
        }
        // Junction j is the START of pcurve j: the same convention the carrier
        // builder and `thicken` both use for a loop of pcurves.
        let mut vertex_ids = Vec::with_capacity(count);
        for pcurve in &carved_loop.pcurves {
            let [t0, _] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let uv = pcurve.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?;
            solid.vertices.push(VertexRecord {
                id: next_id,
                point: face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?,
            });
            vertex_ids.push(next_id);
            next_id += 1;
        }
        let mut coedges = Vec::with_capacity(count);
        for (index, pcurve) in carved_loop.pcurves.iter().enumerate() {
            let source_edge = carved_loop.origin[index].and_then(|(loop_index, coedge_index)| {
                let coedge = face.loops.get(loop_index)?.coedges.get(coedge_index)?;
                source.edges.iter().find(|edge| edge.id == coedge.edge_id)
            });
            if carved_loop.origin[index].is_none() {
                fold_boundary += 1;
            }
            // A piece of a degenerate source edge is still degenerate; a piece
            // of the fold locus is a real curve on the SOURCE (a parallel
            // circle on a cone) whatever its offset image does.
            let degenerate = source_edge.is_some_and(|edge| edge.degenerate);
            let curve = source_image_curve(&face.surface, pcurve, degenerate)?;
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let edge_id = next_id;
            next_id += 1;
            solid.edges.push(EdgeRecord {
                id: edge_id,
                curve,
                t0,
                t1,
                start_vertex_id: vertex_ids[index],
                end_vertex_id: vertex_ids[(index + 1) % count],
                degenerate,
                // The source edge's NAME rides along, so the carrier still
                // stamps `{name}_Offset` on the image of a piece that is still
                // that edge. The fold boundary is new geometry and has no name
                // to inherit.
                name: source_edge.and_then(|edge| edge.name.clone()),
            });
            coedges.push(CoedgeRecord {
                id: next_id,
                edge_id,
                forward: true,
                pcurve: pcurve.clone(),
            });
            next_id += 1;
        }
        new_loops.push(LoopRecord {
            id: next_id,
            coedges,
        });
        next_id += 1;
    }

    let mut carved_face = face.clone();
    carved_face.loops = new_loops;
    if shielded {
        // The discarded taper tip is singular. Fitting over that unused tip
        // can stop refinement before the retained sheet meets its fit bar.
        // Exact knot splits restrict the support without changing its UV map.
        carved_face.surface = support_over_trim(&carved_face)?;
    }
    for shell in &mut solid.shells {
        for existing in &mut shell.faces {
            if existing.id == face.id {
                *existing = carved_face.clone();
            }
        }
    }
    Ok(Some(CarvedSupport {
        solid,
        face: carved_face,
        kept_fraction: carved.kept_fraction,
        fold_boundary,
        shielded_cusp: shielded,
    }))
}

fn support_over_trim(face: &FaceRecord) -> Result<NurbsSurface, KernelRefusal> {
    let surface = &face.surface;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    // Positive-weight pcurves lie in their control hull. Unlike a sampled box,
    // this cannot crop a curved trim's extremum between sample stations.
    let mut bounds = [u1, u0, v1, v0];
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        for control in &coedge.pcurve.control_points {
            let uv = control.point().or_refuse(KernelStage::Refine, "point")?;
            bounds[0] = bounds[0].min(uv.x);
            bounds[1] = bounds[1].max(uv.x);
            bounds[2] = bounds[2].min(uv.y);
            bounds[3] = bounds[3].max(uv.y);
        }
    }
    let restrict = |mut curve: NurbsCurve, low: f64, high: f64| -> Result<NurbsCurve, KernelRefusal> {
        let [a, b] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let band = (b - a) * 1e-12;
        if high < b - band {
            curve = curve.split(high.max(a)).or_refuse(KernelStage::Refine, "split")?.0;
        }
        if low > a + band {
            curve = curve.split(low.min(b)).or_refuse(KernelStage::Refine, "split")?.1;
        }
        Ok(curve)
    };
    let columns = (0..surface.control_points[0].len()).map(|j| {
        let curve = NurbsCurve::new(surface.degree_u, surface.knots_u.clone(),
            surface.control_points.iter().map(|row| row[j]).collect())
            .or_refuse(KernelStage::Refine, "new")?;
        restrict(curve, bounds[0], bounds[1])
    }).collect::<Result<Vec<_>, KernelRefusal>>()?;
    let rows = (0..columns[0].control_points.len()).map(|i| {
        let curve = NurbsCurve::new(surface.degree_v, surface.knots_v.clone(),
            columns.iter().map(|column| column.control_points[i]).collect())
            .or_refuse(KernelStage::Refine, "new")?;
        restrict(curve, bounds[2], bounds[3])
    }).collect::<Result<Vec<_>, KernelRefusal>>()?;
    NurbsSurface::new(surface.degree_u, surface.degree_v, columns[0].knots.clone(),
        rows[0].knots.clone(), rows.into_iter().map(|row| row.control_points).collect())
        .or_refuse(KernelStage::Refine, "new")
}

/// A convex cusp is not a cavity boundary when retained planar neighbors cover its
/// entire image. Their offset carriers cut the regular sheet before the cusp;
/// the ordinary imprint and skin selection must still close and validate it.
/// Read the same miter metric as that selection, including the planes' trims.
/// A tangent endpoint may lie on an offset plane, but the interior must also
/// contain a measurably covered sample. An exposed cusp still refuses.
fn fold_image_shielded(
    source: &BrepSolid,
    face: &FaceRecord,
    kept: &[crate::offset_carve::CarvedLoop],
    distance: f64,
    openings: &[u64],
) -> Result<bool, KernelRefusal> {
    let planes = source.shells.iter().flat_map(|shell| &shell.faces)
        .filter(|other| other.id != face.id && !openings.contains(&other.id))
        .filter(|other| other.surface.is_affine().unwrap_or(false))
        .map(|other| {
            let edges = source.edges.iter().filter(|edge| other.loops.iter()
                .flat_map(|lp| &lp.coedges).any(|coedge| coedge.edge_id == edge.id)).collect::<Vec<_>>();
            (other, edges)
        }).collect::<Vec<_>>();
    let tolerance = KernelTolerances::for_solid(source, 1e-7).model;
    let displacement = shell_displacement(face, distance);
    let mut covered = false;
    for lp in kept {
        for (curve, origin) in lp.pcurves.iter().zip(&lp.origin) {
            if origin.is_some() { continue; }
            let [a, b] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            for index in 0..=64 {
                let uv = curve.evaluate(a + (b-a) * index as f64 / 64.0)
                    .or_refuse(KernelStage::Refine, "evaluate")?;
                // This path handles a convex taper. Keep saddle folds on the
                // existing refusal path; their other sheet needs its own
                // intersection construction and qualification.
                let (low, high) = face.surface.principal_curvatures(uv.x, uv.y)
                    .or_refuse(KernelStage::Refine, "principal_curvatures")?;
                let outward_max = if face.same_sense { high } else { -low };
                if !outward_max.is_finite() || distance * outward_max > COLLAPSE_FACTOR {
                    return Ok(false);
                }
                let point = face.surface.evaluate(uv.x, uv.y)
                    .or_refuse(KernelStage::Refine, "evaluate")?
                    .add(face.surface.normal(uv.x, uv.y)
                        .or_refuse(KernelStage::Refine, "normal")?.scale(displacement));
                let mut nearest = f64::INFINITY;
                for (plane, edges) in &planes {
                    if let Some(separation) = miter_distance_to_trimmed_face(plane, edges, point)? {
                        nearest = nearest.min(separation);
                    }
                }
                os_debug!("fold shielding face {} sample {index}: {nearest:.12} / {distance:.12}", face.id);
                if nearest > distance + tolerance { return Ok(false); }
                covered |= nearest < distance - tolerance;
            }
        }
    }
    Ok(covered)
}

/// The largest 3D extent the POINTWISE offset gives any fold-locus pcurve of a
/// kept piece, and the bar a pinch stays under.
///
/// The bar is derived from what a pinch is at the level the carve cuts. The
/// carve traces `1 − δ·κ = COLLAPSE_FACTOR`, not zero, so even a true pinch's
/// boundary has an image: along a parallel of a surface of revolution the
/// offset scales the circle's radius ρ by exactly that factor, so the image is
/// a circle of radius `ρ·COLLAPSE_FACTOR`, and ρ is inside the solid's scale.
/// The extent read below is the diagonal of the image's bounding box, which for
/// a circle in any plane is at most `2√3` times its radius; that, plus the
/// solid's model tolerance for evaluation noise, is the most a pinch can span. A cusp edge's image has the length of the boundary
/// times the OTHER principal factor, which is O(1) on the kept side.
///
/// Measured through the pointwise offset — surface point plus displacement
/// along the unit parametrization normal, the carve's own convention — and not
/// through the fitted carrier, whose own error would be read as extent.
fn fold_image_extent(
    source: &BrepSolid,
    face: &FaceRecord,
    kept: &[crate::offset_carve::CarvedLoop],
    distance: f64,
) -> Result<(f64, f64), KernelRefusal> {
    const SAMPLES: usize = 32;
    let bar = 2.0 * 3.0f64.sqrt() * COLLAPSE_FACTOR * crate::solid_scale(source)
        + KernelTolerances::for_solid(source, 1e-7).model;
    let displacement = shell_displacement(face, distance);
    let mut worst = 0.0f64;
    for carved_loop in kept {
        for (pcurve, origin) in carved_loop.pcurves.iter().zip(&carved_loop.origin) {
            if origin.is_some() {
                continue;
            }
            let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let (mut lo, mut hi) = (
                Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            );
            for index in 0..=SAMPLES {
                let uv = pcurve.evaluate(t0 + (t1 - t0) * index as f64 / SAMPLES as f64).or_refuse(KernelStage::Refine, "evaluate")?;
                let point = face
                    .surface
                    .evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?
                    .add(face.surface.normal(uv.x, uv.y).or_refuse(KernelStage::Refine, "normal")?.scale(displacement));
                lo = Vec3::new(lo.x.min(point.x), lo.y.min(point.y), lo.z.min(point.z));
                hi = Vec3::new(hi.x.max(point.x), hi.y.max(point.y), hi.z.max(point.z));
            }
            worst = worst.max(hi.sub(lo).length());
        }
    }
    os_debug!(
        "carve of source face {}: fold boundary image extent {worst:.6e} against the pinch bar \
         {bar:.3e}",
        face.id
    );
    Ok((worst, bar))
}

/// The 3D image of a parameter-space curve on its own surface.
///
/// The carrier builder never reads this curve — it re-derives the image on the
/// OFFSET surface from the same pcurve — but the source solid it reads has to be
/// a solid, and a solid whose edges carry invented geometry is a solid that lies
/// to every later reader of it.
fn source_image_curve(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    degenerate: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    const SAMPLES: usize = 64;
    let mut points = Vec::with_capacity(SAMPLES + 1);
    for index in 0..=SAMPLES {
        let uv = pcurve.evaluate(t0 + (t1 - t0) * index as f64 / SAMPLES as f64).or_refuse(KernelStage::Refine, "evaluate")?;
        points.push(surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?);
    }
    if degenerate {
        return crate::make_line(points[0], points[points.len() - 1]).or_refuse(KernelStage::Refine, "make_line");
    }
    let mut parameters = Vec::with_capacity(points.len());
    let mut total = 0.0;
    parameters.push(0.0);
    for window in points.windows(2) {
        total += window[1].sub(window[0]).length().max(1e-12);
        parameters.push(total);
    }
    crate::interpolate_curve(&points, 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")
}

pub(super) fn carrier_solid(
    source: &BrepSolid,
    face_id: u64,
    distance: f64,
    planar_extension: f64,
) -> Result<BrepSolid, KernelRefusal> {
    carrier_solid_sided(
        source,
        face_id,
        distance,
        &crate::CarrierExtension::uniform(planar_extension),
    )
}

pub(super) fn carrier_solid_sided(
    source: &BrepSolid,
    face_id: u64,
    distance: f64,
    extension: &crate::CarrierExtension,
) -> Result<BrepSolid, KernelRefusal> {
    carrier_solid_sided_lane(source, face_id, distance, extension).map(|(solid, _)| solid)
}

/// [`carrier_solid_sided`], naming the lane the offset surface took, for the
/// pipeline's per-lane carrier timing (`profile.rs`).
pub(super) fn carrier_solid_sided_lane(
    source: &BrepSolid,
    face_id: u64,
    distance: f64,
    extension: &crate::CarrierExtension,
) -> Result<(BrepSolid, crate::OffsetSurfaceLane), KernelRefusal> {
    let carrier = crate::offset_face_carrier_sided(source, face_id, distance, extension)?;
    let lane = carrier.lane;
    Ok((
        BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
            id: 1,
            vertices: carrier.vertices,
            edges: carrier.edges,
            shells: vec![ShellRecord {
                id: 1,
                faces: vec![carrier.face],
            }],
            genus: 0,
            kind: crate::BodyKind::Solid,
        },
        lane,
    ))
}

/// Keep only the outer (largest |UV area|) loop of a one-face wall carrier and
/// prune the edges/vertices the dropped interior loops referenced. See the
/// call site in `offset_shell_impl` for why interior loops must go.
pub(super) fn drop_wall_interior_loops(wall: &mut BrepSolid) -> Result<(), KernelRefusal> {
    let face = &mut wall.shells[0].faces[0];
    if face.loops.len() < 2 {
        return Ok(());
    }
    let mut outer = 0usize;
    let mut outer_area = f64::NEG_INFINITY;
    for (index, loop_record) in face.loops.iter().enumerate() {
        let area = parameter_space_area(&FaceRecord {
            id: 0,
            surface: face.surface.clone(),
            same_sense: true,
            loops: vec![loop_record.clone()],
            name: None,
        }).or_refuse(KernelStage::Refine, "parameter_space_area")?
        .abs();
        if area > outer_area {
            outer_area = area;
            outer = index;
        }
    }
    let kept = face.loops[outer].clone();
    face.loops = vec![kept];
    let used_edges = face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect::<HashSet<_>>();
    wall.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices = wall
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    wall.vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));
    Ok(())
}

fn merge_vertex(
    vertices: &mut Vec<ImprintVertex>,
    point: Vec3,
    next_id: &mut u64,
    tolerance: f64,
    scale: f64,
) -> u64 {
    // `crate::tolerance::merge_scale` IS this band's size factor, and its doc
    // records why: the former `1 + ‖point‖` coupling gave a part far from the
    // origin a wrongly-inflated merge band, so the same geometry translated
    // away produced different volumes.  The shared imprint was moved off that
    // anti-pattern; offset-shell's private copy of `merge_vertex` was not, and
    // still read the distance from the world origin until audit slice 0.
    let merge = tolerance.max(1e-5) * crate::tolerance::merge_scale(scale);
    if let Some(vertex) = vertices
        .iter()
        .find(|vertex| vertex.point.sub(point).length() <= merge)
    {
        return vertex.id;
    }
    let id = *next_id;
    *next_id += 1;
    vertices.push(ImprintVertex { id, point });
    id
}

pub(super) fn merge_pair_imprint(
    global: &mut ImprintResultRecord,
    pair: ImprintResultRecord,
    first_operand: u8,
    second_operand: u8,
    next_piece_id: &mut u64,
    next_vertex_id: &mut u64,
    tolerance: f64,
    scale: f64,
) {
    let remap_operand = |operand| {
        if operand == 0 {
            first_operand
        } else {
            second_operand
        }
    };
    let mut vertex_map = HashMap::default();
    for vertex in pair.vertices {
        vertex_map.insert(
            vertex.id,
            merge_vertex(
                &mut global.vertices,
                vertex.point,
                next_vertex_id,
                tolerance,
                scale,
            ),
        );
    }
    let mut piece_map = HashMap::default();
    for piece in pair.pieces {
        let id = *next_piece_id;
        *next_piece_id += 1;
        piece_map.insert(piece.id, id);
        global.pieces.push(ImprintPieceRecord {
            id,
            curve: piece.curve,
            t0: piece.t0,
            t1: piece.t1,
            start_vertex_id: vertex_map[&piece.start_vertex_id],
            end_vertex_id: vertex_map[&piece.end_vertex_id],
            pcurves: piece
                .pcurves
                .into_iter()
                .map(|pcurve| FacePcurve {
                    operand: remap_operand(pcurve.operand),
                    face_id: pcurve.face_id,
                    pcurve: pcurve.pcurve,
                })
                .collect(),
            support_faces: piece.support_faces.map(|face| FaceKey {
                operand: remap_operand(face.operand),
                face_id: face.face_id,
            }),
            shared_edge: piece
                .shared_edge
                .map(|(operand, edge_id, aligned)| (remap_operand(operand), edge_id, aligned)),
        });
    }
    for by_face in pair.by_face {
        let operand = remap_operand(by_face.operand);
        let entry = if let Some(entry) = global
            .by_face
            .iter_mut()
            .find(|entry| entry.operand == operand && entry.face_id == by_face.face_id)
        {
            entry
        } else {
            global.by_face.push(FaceImprints {
                operand,
                face_id: by_face.face_id,
                piece_ids: Vec::new(),
            });
            global.by_face.last_mut().unwrap()
        };
        entry.piece_ids.extend(
            by_face
                .piece_ids
                .into_iter()
                .filter_map(|id| piece_map.get(&id).copied()),
        );
    }
    for split in pair.edge_splits {
        let operand = remap_operand(split.operand);
        if let Some(existing) = global
            .edge_splits
            .iter_mut()
            .find(|entry| entry.operand == operand && entry.edge_id == split.edge_id)
        {
            for parameter in split.parameters {
                if !existing
                    .parameters
                    .iter()
                    .any(|value| (*value - parameter).abs() <= 1e-8)
                {
                    existing.parameters.push(parameter);
                }
            }
        } else {
            global.edge_splits.push(EdgeSplitRecord {
                operand,
                edge_id: split.edge_id,
                parameters: split.parameters,
            });
        }
    }
}

pub(super) fn empty_imprint() -> ImprintResultRecord {
    ImprintResultRecord {
        vertices: Vec::new(),
        pieces: Vec::new(),
        by_face: Vec::new(),
        edge_splits: Vec::new(),
        barrier_edges: Vec::new(),
        tangent_nodes: Vec::new(),
        section_evidence: false,
        cosurface_pairs: Vec::new(),
        tangent_sections: Vec::new(),
    }
}

pub(super) fn flip_fragment(fragment: &mut FaceFragmentRecord) -> Result<(), KernelRefusal> {
    fragment.same_sense = !fragment.same_sense;
    for loop_record in &mut fragment.loops {
        loop_record.coedges.reverse();
        for coedge in &mut loop_record.coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    Ok(())
}

pub(super) fn fragment_as_trim(fragment: &FaceFragmentRecord) -> FaceRecord {
    FaceRecord {
        id: fragment.source_face_id,
        surface: fragment.surface.clone(),
        same_sense: fragment.same_sense,
        loops: fragment
            .loops
            .iter()
            .enumerate()
            .map(|(loop_index, loop_record)| LoopRecord {
                id: loop_index as u64 + 1,
                coedges: loop_record
                    .coedges
                    .iter()
                    .enumerate()
                    .map(|(index, coedge)| CoedgeRecord {
                        id: index as u64 + 1,
                        edge_id: index as u64 + 1,
                        forward: coedge.forward,
                        pcurve: coedge.pcurve.clone(),
                    })
                    .collect(),
            })
            .collect(),
        name: None,
    }
}

/// What the skin test measures a point against a retained face with: the
/// MITERED distance on a planar face, and the Euclidean one where there is no
/// analytic extension to miter along.
///
/// Past a TANGENT edge with another retained face there is no miter: the
/// neighbour's offset carries the skin on from the junction, and the plane's
/// distance there is the Euclidean one (see [`tangent_edge_euclid_on`]).
fn skin_distance_to_trimmed_face(
    face: &FaceRecord,
    edges: &[&EdgeRecord],
    point: Vec3,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
) -> Result<f64, KernelRefusal> {
    let measured = if tangent_edge_euclid_on() {
        miter_distance_with_tangent_edges(face, edges, point, &mut |edge| {
            edge_is_tangent_to_a_retained_face(face, edge, retained_faces)
        })?
    } else {
        miter_distance_to_trimmed_face(face, edges, point)?
    };
    match measured {
        Some(measured) => Ok(measured),
        None => distance_to_trimmed_face(face, edges, point),
    }
}

/// `BREP_OFFSET_TANGENT_EDGE_EUCLID` (default on; `=0` restores): the skin
/// test's mitered distance to a planar retained face reads the EUCLIDEAN
/// distance where the foot leaves the trim across the interior of an edge the
/// face shares TANGENTLY with another retained face.
///
/// Cause (user report, 2026-11-07, app 0.14.0): a Ø10 cylinder with a
/// rectangular post through its side, the post's four junctions filleted r1.5,
/// shelled 1 inward through the bottom. The mitered metric `max(height, past)`
/// is the distance a SHARP join's offsets meet at; past a tangent edge the
/// plane's offset does not continue as a miter, the fillet's offset does. Each
/// point of the two side fillets' offsets (r2.5 arcs under the post walls)
/// read the wall plane's height alone, 0.91–0.996 < 1, both fillets' skins were
/// dropped as SHADOWED, and the assembly refused with twenty-odd one-use
/// edges. Their true distance to the walls is `hypot(height, past)` ≥ 1.
pub(super) fn tangent_edge_euclid_on() -> bool {
    switch!("BREP_OFFSET_TANGENT_EDGE_EUCLID", true)
}


/// Is `edge` of `face` shared with another retained face, the two meeting
/// TANGENTLY along it ([`super::smooth_sync::uses_are_tangent`], the smooth
/// pairing's own test)? An edge with no retained neighbour — an opening's rim,
/// or a trim the neighbour no longer carries — is not.
fn edge_is_tangent_to_a_retained_face(
    face: &FaceRecord,
    edge: &EdgeRecord,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
) -> Result<bool, KernelRefusal> {
    let use_of = |owner: &FaceRecord| {
        owner
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .find(|coedge| coedge.edge_id == edge.id)
            .cloned()
    };
    let Some(own_use) = use_of(face) else {
        return Ok(false);
    };
    for (other, _) in retained_faces {
        if other.id == face.id {
            continue;
        }
        if let Some(other_use) = use_of(other) {
            return super::smooth_sync::uses_are_tangent(face, &own_use, other, &other_use);
        }
    }
    Ok(false)
}

/// Distance from a point to a TRIMMED face: surface projection when it lands
/// inside the trim, otherwise the nearest point on the face's boundary edges.
pub(super) fn distance_to_trimmed_face(
    face: &FaceRecord,
    edges: &[&EdgeRecord],
    point: Vec3,
) -> Result<f64, KernelRefusal> {
    Ok(nearest_on_trimmed_face(face, edges, point)?.0)
}

/// The MITERED distance from a point to a trimmed PLANAR face: how far it is in
/// the metric the shell's sharp join actually uses, rather than the Euclidean
/// one.
///
/// A face's offset skin runs to where its neighbours' offsets cut it, which for
/// a planar face is the L∞ distance in its own frame: the normal distance to its
/// carrier PLANE, extended past the patch, and — when the foot leaves the trim —
/// how far past the trim the foot went. The Euclidean distance mixes the two and
/// reads a miter's extension band as farther than the offset distance: a stair
/// riser 1 tall shelled by 1.5 has its whole skin in that band, and its own
/// Euclidean departure reads 0.496 on a shell that is exact to 5e-13.
///
/// `None` for a curved face, where the carrier has no analytic extension here
/// and the caller keeps the Euclidean reading.
fn miter_distance_to_trimmed_face(
    face: &FaceRecord,
    edges: &[&EdgeRecord],
    point: Vec3,
) -> Result<Option<f64>, KernelRefusal> {
    miter_distance_with_tangent_edges(face, edges, point, &mut |_| Ok(false))
}

/// [`miter_distance_to_trimmed_face`] where `tangent` marks the edges past
/// which the reading is Euclidean: a foot whose nearest boundary point is an
/// INTERIOR point of such an edge reads `hypot(height, past)`. A foot nearest
/// a vertex keeps the miter: a sharp neighbour's band turns that corner. The
/// predicate is asked only of edges that could decide the reading.
fn miter_distance_with_tangent_edges(
    face: &FaceRecord,
    edges: &[&EdgeRecord],
    point: Vec3,
    tangent: &mut dyn FnMut(&EdgeRecord) -> Result<bool, KernelRefusal>,
) -> Result<Option<f64>, KernelRefusal> {
    if !face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let origin = face.surface.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?;
    let along_u = face.surface.evaluate(u1, v0).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
    let along_v = face.surface.evaluate(u0, v1).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
    let normal = along_u.cross(along_v);
    let normal_length = normal.length();
    if !(normal_length > 0.0) {
        return Ok(None);
    }
    let normal = normal.scale(1.0 / normal_length);
    let relative = point.sub(origin);
    let height = relative.dot(normal);
    let foot = point.sub(normal.scale(height));
    // The foot's parameters, from the patch's own affine map.
    let (g11, g12, g22) = (
        along_u.dot(along_u),
        along_u.dot(along_v),
        along_v.dot(along_v),
    );
    let determinant = g11 * g22 - g12 * g12;
    if !(determinant.abs() > 0.0) {
        return Ok(None);
    }
    let offset_from_origin = foot.sub(origin);
    let (r1, r2) = (along_u.dot(offset_from_origin), along_v.dot(offset_from_origin));
    let uv = Vec2 {
        x: u0 + (u1 - u0) * (r1 * g22 - r2 * g12) / determinant,
        y: v0 + (v1 - v0) * (r2 * g11 - r1 * g12) / determinant,
    };
    if parameter_point_in_face(face, uv, 1e-9).or_refuse(KernelStage::Refine, "parameter_point_in_face")? != PolygonClass::Outside {
        return Ok(Some(height.abs()));
    }
    // (distance past the trim to this edge, the edge, nearest at a vertex?)
    let mut readings: Vec<(f64, &EdgeRecord, bool)> = Vec::with_capacity(edges.len());
    for edge in edges.iter().copied() {
        if edge.degenerate {
            continue;
        }
        // SEGMENT MITER (lane O row 13, 2026-10-28; on by default since the batch 4
        // promotion, `BREP_OFFSET_SEGMENT_MITER=0` takes the projector): on a straight edge (non-rational degree 1, two
        // equal-weight controls, linear in its parameter) the exact projection
        // is a dot product, clamped to the curve's domain as the projector
        // clamps it and then to [t0, t1] as below. Same reading to round-off;
        // it is only compared with the skin band.
        if segment_miter_enabled() {
            if let Some((distance, at_vertex)) = segment_edge_distance(&edge.curve, edge.t0, edge.t1, foot) {
                readings.push((distance, edge, at_vertex));
                continue;
            }
        }
        // The exact projection, not a sampled one: a sampled rim overstates how
        // far a foot lies past it, and that is the whole quantity here.
        let projection = crate::project_point_to_curve(&edge.curve, foot).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
        let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        let parameter = projection.u.clamp(low, high);
        let band = 1e-9 * (high - low).abs().max(1.0);
        let at_vertex = parameter - low <= band || high - parameter <= band;
        let distance = edge.curve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?.sub(foot).length();
        readings.push((distance, edge, at_vertex));
    }
    readings.sort_by(|a, b| a.0.total_cmp(&b.0));
    let Some(&(tangential, _, _)) = readings.first() else {
        return Ok(None);
    };
    if !tangential.is_finite() {
        return Ok(None);
    }
    let height = height.abs();
    // Nearest edge first: a sharp edge (or a vertex) ends the scan with the
    // miter, which no farther edge can undercut; a tangent edge's interior
    // reads Euclidean, and a farther edge may still read nearer.
    let mut best = f64::INFINITY;
    for &(past, edge, at_vertex) in &readings {
        if height.max(past) >= best {
            break;
        }
        if !at_vertex && tangent(edge)? {
            best = best.min(height.hypot(past));
        } else {
            best = best.min(height.max(past));
            break;
        }
    }
    Ok(Some(best))
}

/// How far `point` sits from its OWN retained face in the MITERED metric, less
/// the offset distance: zero everywhere its face's offset skin runs, including
/// the miter extension band past its own rim. A trace quantity.
/// Distance from `point` to the window `[t0, t1]` of a straight edge curve
/// (row 13): `None` for any other curve.
fn segment_edge_distance(curve: &NurbsCurve, t0: f64, t1: f64, point: Vec3) -> Option<(f64, bool)> {
    if curve.degree != 1 || curve.control_points.len() != 2 || curve.knots.len() != 4 {
        return None;
    }
    let [a, b] = [curve.control_points[0], curve.control_points[1]];
    if !(a.w > 0.0) || a.w != b.w {
        return None;
    }
    let (start, end) = (a.point().ok()?, b.point().ok()?);
    let [k0, k1] = curve.domain().ok()?;
    let direction = end.sub(start);
    let length_squared = direction.dot(direction);
    if !(length_squared > 0.0) || !(k1 > k0) {
        return None;
    }
    let along = (point.sub(start).dot(direction) / length_squared).clamp(0.0, 1.0);
    let (low, high) = (t0.min(t1), t0.max(t1));
    let parameter = (k0 + (k1 - k0) * along).clamp(low, high);
    let band = 1e-9 * (high - low).abs().max(1.0);
    let at_vertex = parameter - low <= band || high - parameter <= band;
    let nearest = start.add(direction.scale((parameter - k0) / (k1 - k0)));
    Some((nearest.sub(point).length(), at_vertex))
}

fn segment_miter_enabled() -> bool {
    switch!("BREP_OFFSET_SEGMENT_MITER", true)
}


pub(super) fn own_miter_departure(
    point: Vec3,
    own_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
) -> Option<f64> {
    let (face, edges) = retained_faces.iter().find(|(face, _)| face.id == own_face_id)?;
    let measured = miter_distance_to_trimmed_face(face, edges, point).ok()??;
    Some(measured - distance.abs())
}

/// Is this point ADRIFT of its own face's offset — farther from the face, in the
/// mitered metric, than the shell's distance?
///
/// A point of a planar face's offset skin is exactly the distance from it,
/// whether it sits over the face's trim or out in the miter band past its rim.
/// A fragment whose test point is FARTHER is not that face's skin: it is inside
/// the cavity, where a carrier reaches after its neighbours' offsets have cut
/// past it. Measured over the whole membrane census: every chosen fragment of
/// every exact member reads within 1.1e-14, while a rib, a pin and a divider
/// thinner than 2d — all three refused — read 1.04, 0.47 and 0.52.
///
/// `false` where the reading cannot be taken, which is every curved face: this
/// only ever removes a fragment, so an unreadable one keeps its old lane.
pub(super) fn point_adrift_of_its_own_offset(
    point: Vec3,
    own_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> bool {
    own_miter_departure(point, own_face_id, retained_faces, distance)
        .is_some_and(|departure| departure > tolerance)
}

/// The carrier's fragments that hold true offset skin, judged from the source
/// face rather than one test point per fragment: points of the trimmed face,
/// moved the distance into the solid along the normal, are skin where no other
/// retained face is nearer in true distance. A fragment is kept when most of the moved points
/// landing on it are skin. Empty when none is, so the carrier owns no skin.
pub(super) fn fragments_holding_true_skin(
    face: &FaceRecord,
    source: &BrepSolid,
    fragments: &[FaceFragmentRecord],
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    const SAMPLES: usize = 64;
    let u_domain = face.surface.domain_u().or_refuse(KernelStage::Select, "domain_u")?;
    let v_domain = face.surface.domain_v().or_refuse(KernelStage::Select, "domain_v")?;
    let classify_tolerance = 1e-8 * crate::solid_scale(source).max(1.0);
    let trims = fragments.iter().map(fragment_as_trim).collect::<Vec<_>>();
    let mut votes = vec![(0usize, 0usize); fragments.len()];
    for i in 0..SAMPLES {
        for j in 0..SAMPLES {
            let uv = Vec2 {
                x: u_domain[0] + (u_domain[1] - u_domain[0]) * (i as f64 + 0.5) / SAMPLES as f64,
                y: v_domain[0] + (v_domain[1] - v_domain[0]) * (j as f64 + 0.5) / SAMPLES as f64,
            };
            if parameter_point_in_face(face, uv, 1e-9).or_refuse(KernelStage::Select, "parameter_point_in_face")? != PolygonClass::Inside {
                continue;
            }
            let on_face = face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Select, "evaluate")?;
            let normal = face.surface.normal(uv.x, uv.y).or_refuse(KernelStage::Select, "normal")?;
            let mut moved = None;
            for sign in [1.0, -1.0] {
                let candidate = on_face.add(normal.scale(sign * distance));
                if classify_point(candidate, source, classify_tolerance).or_refuse(KernelStage::Select, "classify_point")?.class == PointClass::In {
                    moved = Some(candidate);
                    break;
                }
            }
            let Some(point) = moved else { continue };
            // Euclidean, not the shell's mitered reading: a miter extends a
            // planar wall past its trim and would read this skin as covered.
            // A tie is not skin either: two offsets on one plane are the medial
            // axis, wall through its thickness, and keeping them is a membrane.
            let mut skin = true;
            for (other, edges) in retained_faces {
                if other.id == face.id {
                    continue;
                }
                let reach = distance_to_trimmed_face(other, edges, point)?;
                // The tie is exact, so its band is the classifier's: at the skin
                // band it would take half of a real fragment's points.
                if reach < distance - tolerance || (reach - distance).abs() <= classify_tolerance {
                    skin = false;
                    break;
                }
            }
            for (index, fragment) in fragments.iter().enumerate() {
                let projection = project_point_to_surface(&fragment.surface, point).or_refuse(KernelStage::Select, "project_point_to_surface")?;
                if projection.distance > tolerance {
                    continue;
                }
                let foot = Vec2 { x: projection.u, y: projection.v };
                if parameter_point_in_face(&trims[index], foot, 1e-8).or_refuse(KernelStage::Select, "parameter_point_in_face")? != PolygonClass::Outside {
                    if skin {
                        votes[index].0 += 1;
                    } else {
                        votes[index].1 += 1;
                    }
                    break;
                }
            }
        }
    }
    os_debug!("  true skin votes for source face {}: {:?}", face.id, votes);
    Ok(fragments.iter().zip(&votes).filter(|(_, (on, off))| *on > *off).map(|(fragment, _)| fragment.clone()).collect())
}

/// Whether a point's projection onto the face's surface lands inside its trim.
pub(super) fn foot_inside_trim(face: &FaceRecord, point: Vec3) -> Result<bool, KernelRefusal> {
    let projection = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let uv = Vec2 { x: projection.u, y: projection.v };
    Ok(parameter_point_in_face(face, uv, 1e-9).or_refuse(KernelStage::Refine, "parameter_point_in_face")? == PolygonClass::Inside)
}

/// [`distance_to_trimmed_face`] with the foot it measured to. No foot and an
/// infinite distance when the projection leaves the trim and the face has no
/// real edge to fall back on.
fn nearest_on_trimmed_face(
    face: &FaceRecord,
    edges: &[&EdgeRecord],
    point: Vec3,
) -> Result<(f64, Option<Vec3>), KernelRefusal> {
    let projection = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let uv = Vec2 {
        x: projection.u,
        y: projection.v,
    };
    if parameter_point_in_face(face, uv, 1e-9).or_refuse(KernelStage::Refine, "parameter_point_in_face")? != PolygonClass::Outside {
        return Ok((projection.distance, Some(projection.point)));
    }
    let mut best = (f64::INFINITY, None);
    for edge in edges {
        if edge.degenerate {
            continue;
        }
        let samples = 48;
        for index in 0..=samples {
            let parameter = edge.t0 + (edge.t1 - edge.t0) * index as f64 / samples as f64;
            let sample = edge.curve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
            let separation = sample.sub(point).length();
            if separation < best.0 {
                best = (separation, Some(sample));
            }
        }
    }
    Ok(best)
}

/// Two retained faces whose offsets COINCIDE at `point`: a point of `own`'s
/// offset skin that is ALSO the offset distance, to `tolerance`, from another
/// retained face lying on its OPPOSITE side.
///
/// That is the material between the two faces being exactly twice the
/// distance thick — a pocket floor 2d above the face under it, a slot wall 2d
/// wide grown outward. The cavity between the two offsets has no thickness, and
/// both skins select the same sheet from opposite sides: measured on a pocket
/// floor of 3 over a 20 cube shelled inward by 1.5, a second, zero-volume shell
/// of four coincident 13 × 13 faces, area +676, validate-clean. Within the
/// tolerance a sheet of no thickness and a sliver thinner than the weld cannot
/// be told apart, so the caller refuses rather than choosing.
///
/// A point on ONE side of both faces is not this: two faces' offsets meeting
/// along a miter touch it only on a curve, never at a fragment's interior test
/// point, and coplanar neighbours put the point on the same side of both.
pub(super) fn coincident_offset_face(
    point: Vec3,
    own_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<Option<(u64, f64)>, KernelRefusal> {
    let Some(own_foot) = retained_foot(point, own_face_id, retained_faces)? else {
        return Ok(None);
    };
    let target = distance.abs();
    let bounded = coincident_bound_on();
    for (face, edges) in retained_faces {
        if face.id == own_face_id {
            continue;
        }
        if bounded && foot_box_distance(face, edges, point).is_some_and(|reach| reach > beyond_band(target + tolerance, point)) {
            continue;
        }
        let (separation, Some(foot)) = nearest_on_trimmed_face(face, edges, point)? else {
            continue;
        };
        if (separation - target).abs() <= tolerance
            && point.sub(own_foot).dot(point.sub(foot)) < 0.0
        {
            return Ok(Some((face.id, separation)));
        }
    }
    Ok(None)
}

/// `BREP_OFFSET_TANGENT_TOUCH` (default on; `=0` restores): a coincidence that
/// [`coincident_offset_face`] reads at a fragment's test point but that the
/// fragment's other interior points do not share is a TANGENT TOUCH, not a
/// sheet: refuse it by name rather than drop the fragment as shadowed.
pub(super) fn tangent_touch_on() -> bool {
    switch!("BREP_OFFSET_TANGENT_TOUCH", true)
}


/// Where `other`'s offset coincidence with a fragment is only a TOUCH: one of
/// the fragment's extra interior points lies farther than `distance +
/// tolerance` from `other`, and none lies nearer than `distance − tolerance`
/// (that would be a crossing, which the shadow rules own). Returns that
/// farther point and its separation.
///
/// Cause (user report, 2026-11-07): a box less a Ø12 bore whose −X
/// wall beside the bore is 4.0 thick, shelled inward by exactly 2.0. The −X
/// offset plane x = 2 touches the bore's offset cylinder r = 8 along one line;
/// the plane's single fragment had its test point within the skin band of that
/// line, read "coincident", and was dropped whole as a thin-wall sheet. The
/// −X skin went missing and the shell refused as an open assembly whose cause
/// named a different pair of faces. The coincidence construction is exact only
/// where the two offsets are one sheet; at a touch the cavity pinches to a
/// line, which no manifold shell carries.
pub(super) fn tangent_touch(
    fragment: &FaceFragmentRecord,
    other_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<Option<(Vec3, f64)>, KernelRefusal> {
    let Some((face, edges)) = retained_faces.iter().find(|(face, _)| face.id == other_face_id) else {
        return Ok(None);
    };
    let target = distance.abs();
    let mut farther = None;
    for &point in &fragment.extra_test_points {
        let (separation, _) = nearest_on_trimmed_face(face, edges, point)?;
        if !separation.is_finite() || separation < target - tolerance {
            return Ok(None);
        }
        if separation > target + tolerance && farther.is_none_or(|(_, best): (Vec3, f64)| separation > best) {
            farther = Some((point, separation));
        }
    }
    Ok(farther)
}

/// `BREP_OFFSET_COINCIDE_DEBRIS` (default on; `=0` restores the refusal): a selected
/// fragment that reads coincident with another offset but is no wider than
/// the skin band is corner debris, not a sheet, and is left out of the skin
/// instead of refusing the shell.
///
/// Cause (a reported box and pin model at 2.24, 2.34, 2.36, 2.42-2.48): where the
/// offsets of the tilted floor and of the wall or the bore cross at a shallow
/// angle next to the trim line y = 2.5, the arrangement leaves a sliver
/// narrower than the band between the crossing and that line (parameter areas
/// 2e-8 to 2.3e-6; on the bore's carrier a band-squared area test misses it,
/// since the sliver is long). Its test point sits
/// within the band of the other offset, so the whole shell was refused as a
/// cavity pinched to zero thickness. Selecting the sliver instead leaves
/// zero-thickness membrane shells in the result, so it is dropped.
pub(super) fn coincide_debris_on() -> bool {
    switch!("BREP_OFFSET_COINCIDE_DEBRIS", true)
}

/// Whether `fragment` is no wider than `band`: twice its area in 3D over its
/// boundary length in 3D. The area is its parameter-space area times the
/// surface's area element at its test point, capped by the isoperimetric
/// bound; the boundary is each coedge's pcurve sampled 24 times and lifted
/// onto the surface.
pub(super) fn fragment_is_band_debris(fragment: &FaceFragmentRecord, band: f64) -> Result<bool, KernelRefusal> {
    let area = parameter_space_area(&fragment_as_trim(fragment))
        .or_refuse(KernelStage::Select, "parameter_space_area")?
        .abs();
    let partials = fragment
        .surface
        .derivatives(fragment.test_uv.x, fragment.test_uv.y, 1)
        .or_refuse(KernelStage::Select, "derivatives")?;
    let element = partials[1][0].cross(partials[0][1]).length();
    let mut perimeter = 0.0;
    for coedge in fragment.loops.iter().flat_map(|l| &l.coedges) {
        let [t0, t1] = coedge.pcurve.domain().or_refuse(KernelStage::Select, "pcurve domain")?;
        let mut previous: Option<Vec3> = None;
        for step in 0..=24 {
            let uv = coedge
                .pcurve
                .evaluate(t0 + (t1 - t0) * step as f64 / 24.0)
                .or_refuse(KernelStage::Select, "pcurve")?;
            let point = fragment.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Select, "surface")?;
            if let Some(last) = previous {
                perimeter += point.sub(last).length();
            }
            previous = Some(point);
        }
    }
    // No region encloses more than its boundary's circle (P^2 / 4 pi); on a
    // rational carrier the area element at one point can overstate a sliver.
    let area = (area * element).min(perimeter * perimeter / (4.0 * std::f64::consts::PI));
    let width = if perimeter > 0.0 { 2.0 * area / perimeter } else { f64::INFINITY };
    os_debug!("  debris test: area {area:.3e} boundary {perimeter:.4} width {width:.3e} against band {band:.1e}");
    Ok(width <= band)
}

/// `BREP_OFFSET_SEALED_HOLLOW` (default on; `=0` restores): an assembly that
/// fails while part of the cavity skin has no path to the opening is refused
/// as a SEALED HOLLOW, naming the faces around it and the distance below
/// which the cavity stays one, rather than as the first crossing found with
/// the open-curved-rim class (whose feature message advises reordering the
/// history, which cannot help).
///
/// Cause (user report follow-up, 2026-11-07): the reported box less a
/// Ø12 bore, shelled inward through +X by 2.5. The −X and top walls beside
/// the bore are 4.0 thick, so past 2.0 their offsets cross the bore's offset
/// and cut the corner pockets beside the bore off the cavity. The pockets'
/// skins did not close (23 one-use edges, S = 3), and the refusal named the
/// tilted bottom against the bore — a pair that crosses at every distance
/// above ~0.4, including 1.99, which builds — and the feature added the
/// reorder advice.
pub(super) fn sealed_hollow_on() -> bool {
    switch!("BREP_OFFSET_SEALED_HOLLOW", true)
}


/// One part of the selected cavity skin that no chain of shared edges or
/// edge ends links to an opening wall.
pub(super) struct SealedSkin {
    /// The source faces whose offsets bound it, ascending.
    pub(super) sources: Vec<u64>,
    /// The mean of its fragments' test points.
    pub(super) centre: Vec3,
    /// The middle of each edge of its fragments' loops, with the source face
    /// whose offset that fragment lies on.
    pub(super) edge_middles: Vec<(u64, Vec3)>,
}

/// The parts of the selected cavity skin (`offsets`, kept skin fragments)
/// that touch no selected WALL fragment through any chain of shared edge
/// segments or shared edge ends: hollows sealed off from the opening. Read
/// before assembly, on the fragments the selection kept, so a weld the
/// assembly later fails cannot split a part off. Sharing an edge end joins
/// more than sharing a segment, so a part counted sealed touches the rest of
/// the skin nowhere at all. Empty when there is no wall to reach.
pub(super) fn sealed_skins(
    offsets: &[FaceFragmentRecord],
    walls: &[FaceFragmentRecord],
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    source_face_of: impl Fn(&FaceFragmentRecord) -> u64,
) -> Result<Vec<SealedSkin>, KernelRefusal> {
    if walls.is_empty() || offsets.is_empty() {
        return Ok(Vec::new());
    }
    let fragments = offsets.iter().chain(walls).collect::<Vec<_>>();
    let mut parent = (0..fragments.len()).collect::<Vec<_>>();
    fn root(parent: &mut [usize], mut index: usize) -> usize {
        while parent[index] != index {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        index
    }
    let mut first = HashMap::<[i64; 9], usize>::default();
    for (index, fragment) in fragments.iter().enumerate() {
        for coedge in fragment.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let key = fragment_edge_segment_key(&coedge.source, solids, imprint)?;
            let start = [key[0], key[1], key[2], 0, 0, 0, 0, 0, i64::MIN];
            let end = [key[6], key[7], key[8], 0, 0, 0, 0, 0, i64::MIN];
            for shared in [key, start, end] {
                let other = *first.entry(shared).or_insert(index);
                let (a, b) = (root(&mut parent, index), root(&mut parent, other));
                parent[a] = b;
            }
        }
    }
    let mut open = HashSet::<usize>::default();
    for index in offsets.len()..fragments.len() {
        open.insert(root(&mut parent, index));
    }
    let mut sealed = std::collections::BTreeMap::<usize, (Vec<u64>, Vec3, usize, Vec<(u64, Vec3)>)>::new();
    for (index, fragment) in offsets.iter().enumerate() {
        let group = root(&mut parent, index);
        if open.contains(&group) {
            continue;
        }
        let entry = sealed.entry(group).or_insert_with(|| (Vec::new(), Vec3::default(), 0, Vec::new()));
        entry.0.push(source_face_of(fragment));
        entry.1 = entry.1.add(fragment.test_point);
        entry.2 += 1;
        let own = source_face_of(fragment);
        for coedge in fragment.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let geometry = fragment_edge_geometry(&coedge.source, solids, imprint)?;
            let middle = geometry
                .curve
                .evaluate(0.5 * (geometry.t0 + geometry.t1))
                .or_refuse(KernelStage::Select, "evaluate")?;
            entry.3.push((own, middle));
        }
    }
    Ok(sealed
        .into_values()
        .map(|(mut sources, sum, count, edge_middles)| {
            sources.sort_unstable();
            sources.dedup();
            SealedSkin { sources, centre: sum.scale(1.0 / count as f64), edge_middles }
        })
        .collect())
}

/// The thinnest wall of material between retained faces `own` and `other`
/// near `seed`: their nearest pair of points by alternating projection from
/// `seed`, as `(thickness, midpoint)`. `None` when a projection has no foot.
fn thinnest_wall_near(
    own: (&FaceRecord, &[&EdgeRecord]),
    other: (&FaceRecord, &[&EdgeRecord]),
    seed: Vec3,
) -> Result<Option<(f64, Vec3)>, KernelRefusal> {
    let Some(mut near) = nearest_on_trimmed_face(own.0, own.1, seed)?.1 else {
        return Ok(None);
    };
    let mut far = None;
    for _ in 0..64 {
        let Some(next_far) = nearest_on_trimmed_face(other.0, other.1, near)?.1 else {
            break;
        };
        let Some(next_near) = nearest_on_trimmed_face(own.0, own.1, next_far)?.1 else {
            break;
        };
        let settled = next_near.sub(near).length() <= 1e-10 * (1.0 + near.length());
        near = next_near;
        far = Some(next_far);
        if settled {
            break;
        }
    }
    Ok(far.map(|far| (far.sub(near).length(), near.add(far).scale(0.5))))
}

/// The distance below which a sealed hollow's channel to the rest of the
/// cavity reopens, as `(skin index, face, face, thickness)`.
///
/// A sealed skin is bounded by SECTIONS where two offsets meet: an edge of a
/// skin fragment on face A's offset that also lies at the offset distance
/// from retained face B (within `tolerance`) is on the A × B section. A
/// crossing of two offsets exists from half the thinnest wall of material
/// between their source faces up (that wall read near the edge, its midpoint
/// inside `source`); a junction of adjacent faces reads no wall and bounds at
/// every distance. The hollow is sealed only once ALL its bounding crossings
/// exist, so it seals at the LARGEST of their half-walls. Of the sealed
/// skins, the one that sealed LAST names the distance: the nearest one below
/// this at which the cavity's split changes. `None` when no skin has a
/// bounding crossing that reads a wall.
pub(super) fn sealing_distance(
    skins: &[SealedSkin],
    source: &BrepSolid,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<Option<(usize, u64, u64, f64)>, KernelRefusal> {
    let target = distance.abs();
    let limit = 2.0 * (target - tolerance);
    let retained = |id: u64| {
        retained_faces
            .iter()
            .find(|(face, _)| face.id == id)
            .map(|(face, edges)| (*face, edges.as_slice()))
    };
    let mut best: Option<(usize, u64, u64, f64)> = None;
    for (index, skin) in skins.iter().enumerate() {
        let mut seals: Option<(u64, u64, f64)> = None;
        let mut read = HashSet::<(u64, u64)>::default();
        for &(own_id, middle) in &skin.edge_middles {
            let Some(own) = retained(own_id) else {
                continue;
            };
            for (other, other_edges) in retained_faces {
                if other.id == own_id || read.contains(&(own_id, other.id)) {
                    continue;
                }
                let (separation, _) = nearest_on_trimmed_face(other, other_edges, middle)?;
                if !separation.is_finite() || (separation - target).abs() > tolerance {
                    continue;
                }
                let Some((thickness, wall_middle)) = thinnest_wall_near(own, (other, other_edges), middle)? else {
                    continue;
                };
                os_debug!("sealing: skin {index} faces ({own_id},{}) wall {thickness:.6} at {wall_middle:?}", other.id);
                read.insert((own_id, other.id));
                if thickness <= tolerance || thickness >= limit {
                    continue;
                }
                if seals.is_some_and(|(_, _, held)| held >= thickness) {
                    continue;
                }
                let class = crate::classify_point(wall_middle, source, tolerance * 1e-3)
                    .or_refuse(KernelStage::Select, "classify_point")?
                    .class;
                if class == crate::PointClass::In {
                    seals = Some((own_id, other.id, thickness));
                }
            }
        }
        if let Some((own, other, thickness)) = seals {
            if best.is_none_or(|(_, _, _, held)| thickness > held) {
                best = Some((index, own, other, thickness));
            }
        }
    }
    Ok(best)
}

/// Lane O row 31 (2026-10-31, `BREP_OFFSET_COINCIDENT_BOUND`, default on since
/// batch 12; `0`/`off` projects onto every face): [`coincident_offset_face`] skips a face whose every
/// possible foot lies farther from the point than the band's far edge, before
/// projecting onto it. The feet [`nearest_on_trimmed_face`] can return are a
/// surface point (the projection, kept when it lands in the trim) or a sample
/// of one of the face's edge curves; with positive weights each lies in the
/// convex hull of its own control points, so in the box over the surface's and
/// the edges' control points ([`foot_box_distance`]). Such a face can only
/// fail the band test, so the first face that passes, and the result, are
/// unchanged; the one difference is that a skipped face's projection, which
/// could only have failed the test, can no longer refuse. The D1 bench case
/// spent 0.44 ms a run in these reads.
fn coincident_bound_on() -> bool {
    switch!("BREP_OFFSET_COINCIDENT_BOUND", true)
}


/// The distance from `point` to the box over `face`'s surface control points
/// and its non-degenerate edges' control points: no foot
/// [`nearest_on_trimmed_face`] returns is nearer. `None` (no bound) for a
/// non-positive or non-finite weight, or a non-finite coordinate.
fn foot_box_distance(face: &FaceRecord, edges: &[&EdgeRecord], point: Vec3) -> Option<f64> {
    let (mut low, mut high) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    let surface_points = face.surface.control_points.iter().flatten();
    let edge_points = edges.iter().filter(|edge| !edge.degenerate).flat_map(|edge| edge.curve.control_points.iter());
    for control in surface_points.chain(edge_points) {
        if !(control.w > 0.0 && control.w.is_finite()) {
            return None;
        }
        let p = control.point().ok()?;
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return None;
        }
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    if !(low.x <= high.x) {
        return None;
    }
    let nearest = Vec3::new(point.x.clamp(low.x, high.x), point.y.clamp(low.y, high.y), point.z.clamp(low.z, high.z));
    Some(point.sub(nearest).length())
}

/// `reach` widened past the rounding of the feet's evaluation and of the
/// distances read from them, and past a projection kept on the trim within
/// its 1e-9 parameter band (1e-6 relative to the reach and to the point's
/// magnitude, far above each).
fn beyond_band(reach: f64, point: Vec3) -> f64 {
    reach * (1.0 + 1e-6) + 1e-6 * (1.0 + point.length())
}

/// How far `point` sits from its OWN retained face, less the offset distance:
/// zero for a point of that face's offset skin over its trim, positive in the
/// extension band past its rim. A trace quantity.
pub(super) fn own_offset_departure(
    point: Vec3,
    own_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
) -> Option<f64> {
    let foot = retained_foot(point, own_face_id, retained_faces).ok()??;
    Some(point.sub(foot).length() - distance.abs())
}

/// The foot of `point` on retained face `face_id`, when that face is retained
/// and has one.
fn retained_foot(
    point: Vec3,
    face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
) -> Result<Option<Vec3>, KernelRefusal> {
    let Some((face, edges)) = retained_faces.iter().find(|(face, _)| face.id == face_id) else {
        return Ok(None);
    };
    Ok(nearest_on_trimmed_face(face, edges, point)?.1)
}

/// A retained face that a point of `own`'s offset lies BETWEEN `own` and,
/// facing it: nearer than the distance, with its foot nearly OPPOSITE `own`'s.
/// Then the two offsets cross there — the material between the faces is
/// thinner than twice the distance.
///
/// "Opposite" is within [`CROSSING_OPPOSITION`] of antiparallel. A merely
/// obtuse pair is the surplus past a convex miter, where the feet sit a right
/// angle apart; and a point lying ON a face has no direction to read.
pub(super) fn crossing_face_at(
    point: Vec3,
    own_face_id: u64,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<Option<(u64, f64)>, KernelRefusal> {
    let Some(own) = retained_foot(point, own_face_id, retained_faces)? else {
        return Ok(None);
    };
    let to_own = point.sub(own);
    let reach = to_own.length();
    if reach <= tolerance {
        return Ok(None);
    }
    for (face, edges) in retained_faces {
        if face.id == own_face_id {
            continue;
        }
        let (separation, Some(foot)) = nearest_on_trimmed_face(face, edges, point)? else {
            continue;
        };
        if separation <= tolerance || separation >= distance.abs() - tolerance {
            continue;
        }
        if to_own.dot(point.sub(foot)) < -CROSSING_OPPOSITION * reach * separation {
            return Ok(Some((face.id, separation)));
        }
    }
    Ok(None)
}

/// cos 150°: how close to antiparallel two feet must be for a shadowed point to
/// read as lying between two facing faces. Thin walls between parallel faces
/// read exactly −1; a convex miter's surplus reads 0.
pub(super) const CROSSING_OPPOSITION: f64 = 0.866;

/// How many shadowed fragment test points one pass keeps for a crossing to name
/// in a refusal. Reading one costs a projection per retained face, and it is
/// only ever quoted.
pub(super) const MAXIMUM_CROSSING_READS: usize = 64;

/// Where two retained faces' offsets CROSS, among the shadowed test points a
/// failed pass kept: `(point, the source face whose offset it lies on)`. Read
/// against the same retained faces and skin band the selection used; a point
/// that cannot be read names nothing.
pub(super) fn name_offset_crossing(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
    points: &[(Vec3, u64)],
) -> Option<String> {
    let edges = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let retained = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| !opening_face_ids.contains(&face.id))
        .map(|face| {
            let face_edges = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter_map(|coedge| edges.get(&coedge.edge_id).copied())
                .collect::<Vec<_>>();
            (face, face_edges)
        })
        .collect::<Vec<_>>();
    let tolerance = offset_skin_band(crate::solid_scale(source), distance);
    points.iter().find_map(|(point, own)| {
        let (face, separation) = crossing_face_at(*point, *own, &retained, distance, tolerance).ok()??;
        Some(format!(
            "the offsets of source faces {own} and {face} cross near ({:.4}, {:.4}, {:.4}), where \
             the material between them is thinner than twice the shell distance ({separation:.4} \
             from face {face} against {:.4})",
            point.x,
            point.y,
            point.z,
            distance.abs(),
        ))
    })
}

/// The band the skin test reads a distance to: the selection's, the one a
/// refusal naming a crossing re-reads with, and the unit of the outward ruled
/// wall's neighbour clearance (`RULED_NEIGHBOUR_CLEARANCE_BANDS` in pipeline.rs).
pub(super) fn offset_skin_band(scale: f64, distance: f64) -> f64 {
    2e-3f64.max(scale * 5e-5).max(distance.abs() * 1e-3)
}

/// What the skin test MEASURED at one point.
///
/// Two readings mean "not skin" for different reasons, and they must not be
/// folded together: a point measured nearer than the offset distance to some
/// retained face is SHADOWED — a fact about the geometry — while a point whose
/// distance to some face could not be read at all is UNMEASURED, which says
/// nothing about the geometry. Folding the second into either verdict is how a
/// selection comes to rest on a number nobody took.
#[derive(Clone, Debug)]
pub(super) enum SkinReading {
    /// At least the offset distance from every retained face.
    On,
    /// Measurably nearer than the offset distance to retained face `face`.
    Shadowed { face: u64, separation: f64 },
    /// No face was measured near, and the distance to `face` could not be read.
    Unmeasured { face: u64, reason: String },
}

/// Read [`SkinReading`] at `point`.
///
/// A single measured near face settles SHADOWED whatever another face failed
/// to report — the predicate is "nearer than the distance to SOME face" — so an
/// unreadable face only decides the verdict when nothing else was near.
pub(super) fn offset_skin_reading(
    point: Vec3,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> SkinReading {
    let target = distance.abs();
    let mut unmeasured = None;
    for (face, edges) in retained_faces {
        match skin_distance_to_trimmed_face(face, edges, point, retained_faces) {
            Ok(separation) if separation.is_finite() => {
                if separation < target - tolerance {
                    return SkinReading::Shadowed {
                        face: face.id,
                        separation,
                    };
                }
            }
            Ok(separation) => {
                unmeasured.get_or_insert((face.id, format!("the distance read {separation}")));
            }
            Err(error) => {
                unmeasured.get_or_insert((face.id, error.message));
            }
        }
    }
    match unmeasured {
        Some((face, reason)) => SkinReading::Unmeasured { face, reason },
        None => SkinReading::On,
    }
}

/// True when the point lies at the full offset distance from every retained
/// source face — the defining property of the shell's offset skin. Fragments
/// that sit closer to some OTHER retained face belong to that face's offset
/// region (they are "shadowed") and must not be selected, no matter where the
/// legacy parameter-space seed lands. A point that could not be measured is a
/// refusal, never either answer.
pub(super) fn point_on_offset_skin(
    point: Vec3,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    distance: f64,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    match offset_skin_reading(point, retained_faces, distance, tolerance) {
        SkinReading::On => Ok(true),
        SkinReading::Shadowed { .. } => Ok(false),
        SkinReading::Unmeasured { face, reason } => Err(unmeasured_skin_refusal(point, face, &reason)),
    }
}

/// At a reflex opening, the rim can move past the original outline. The
/// outward side of a retained plane is then cavity, not a second wall strip.
/// The unsigned miter metric would keep both sides of that plane, filling the
/// opening with a phantom strip beside the legitimate inward wall.
pub(super) fn point_on_inward_opening_skin(
    point: Vec3,
    retained_faces: &[(&FaceRecord, Vec<&EdgeRecord>)],
    reflex_neighbors: &HashSet<u64>,
    inside_source: bool,
    distance: f64,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let mut material_side = Vec::new();
    let mut reflex_side = Vec::new();
    for (face, edges) in retained_faces {
        if face
            .surface
            .is_affine()
            .or_refuse(KernelStage::Select, "is_affine")?
        {
            let projection = project_point_to_surface(&face.surface, point)
                .or_refuse(KernelStage::Select, "project_point_to_surface")?;
            let foot = face
                .surface
                .evaluate(projection.u, projection.v)
                .or_refuse(KernelStage::Select, "evaluate")?;
            let normal = face_normal(face, projection.u, projection.v)?;
            if point.sub(foot).dot(normal) > tolerance {
                continue;
            }
        }
        if inside_source && reflex_neighbors.contains(&face.id) {
            reflex_side.push((*face, edges.clone()));
        } else {
            material_side.push((*face, edges.clone()));
        }
    }
    let other_skin = point_on_offset_skin(point, &material_side, distance, tolerance)?;
    if inside_source {
        // Outside the original opening, a wall exists only in a reflex rim's
        // strip, and only where other offsets leave cavity. Near (say) the
        // top of a step both sides are material, so there is no wall there.
        let reflex_skin = point_on_offset_skin(point, &reflex_side, distance, tolerance)?;
        Ok(reflex_skin || !other_skin)
    } else {
        Ok(other_skin)
    }
}

/// A planar source face entirely on one side of an adjacent opening must
/// keep its offset on that side too. At a reflex rim this may be the FRONT
/// of the opening; a global opening halfspace would cut away the wrong side.
/// Bound the trimmed face with its pcurve control hull, using the affine
/// surface map so the side test is conservative between sample stations.
pub(super) fn opening_trim_halfspaces(
    face: &FaceRecord,
    openings: &[&FaceRecord],
    tolerance: f64,
) -> Result<Vec<(Vec3, Vec3)>, KernelRefusal> {
    let mut planes = Vec::new();
    if !face
        .surface
        .is_affine()
        .or_refuse(KernelStage::Select, "is_affine")?
    {
        return Ok(planes);
    }
    'opening: for opening in openings {
        if !source_faces_adjacent(face, opening) {
            continue;
        }
        let Some((origin, normal)) = planar_surface_frame(&opening.surface, tolerance)? else {
            continue;
        };
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for control in face
            .loops
            .iter()
            .flat_map(|l| &l.coedges)
            .flat_map(|c| &c.pcurve.control_points)
        {
            if control.w <= 0.0 {
                continue 'opening;
            }
            let Ok(point) = face
                .surface
                .evaluate(control.x / control.w, control.y / control.w)
            else {
                continue 'opening;
            };
            let side = point.sub(origin).dot(normal);
            low = low.min(side);
            high = high.max(side);
        }
        if low >= -tolerance && high > tolerance {
            planes.push((origin, normal));
        } else if high <= tolerance && low < -tolerance {
            planes.push((origin, normal.scale(-1.0)));
        }
    }
    Ok(planes)
}

/// Where a RETAINED face meets an OPENING along a sharp edge that is not
/// convex — reflex, or two coplanar planes — the retained face's offset has to
/// END inside the cavity, beside the opening, and the shell needs a wall face
/// there that no carrier supplies. Read for a refusal only.
///
/// Measured on a partial apex cone (r 8, h 12) shelled inward by 1 with one
/// azimuth cap retained and the other opened, over 180° (the caps coplanar) and
/// 270° (reflex along the axis): the retained cap's offset also shadows the
/// cone's apex pinch, so the lateral offset skin splits into two or three horns
/// that end in tangencies with the cap's offset, and one fragment per carrier
/// cannot hold them. The same cone over 90° (a convex axis edge) builds, to
/// 5.6e-7 of its closed form. Inward shells only: that is where the offset runs
/// into the cavity beside the opening.
pub(super) fn name_retained_opening_junction(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Option<String> {
    if distance <= 0.0 {
        return None;
    }
    let faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let edges = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    for face in faces.iter().filter(|face| !opening_face_ids.contains(&face.id)) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let Some(edge) = edges.get(&coedge.edge_id) else {
                continue;
            };
            if edge.degenerate {
                continue;
            }
            let Some((mate, _)) = junction_mate(&faces, face, coedge.edge_id) else {
                continue;
            };
            if !opening_face_ids.contains(&mate.id) {
                continue;
            }
            let Ok(stations) = junction_stations(face, coedge, edge, mate) else {
                continue;
            };
            let coplanar_planes = stations.is_empty()
                && face.surface.is_affine().unwrap_or(false)
                && mate.surface.is_affine().unwrap_or(false);
            let reflex = stations.iter().any(|station| !station.is_convex());
            if coplanar_planes || reflex {
                return Some(format!(
                    "retained face {} meets opening {} along edge {} at a {} junction, so its \
                     offset ends inside the cavity beside the opening, where the wall needs a face \
                     no carrier supplies (on a partial revolve whose cap is retained, that offset \
                     also shadows the apex pinch and splits the offset skin into horns ending in \
                     tangencies)",
                    face.id,
                    mate.id,
                    edge.id,
                    if reflex { "REFLEX" } else { "FLAT (coplanar)" },
                ));
            }
        }
    }
    None
}

/// The refusal for a skin test that could not be taken.
pub(super) fn unmeasured_skin_refusal(point: Vec3, face: u64, reason: &str) -> KernelRefusal {
    KernelRefusal::internal(
        KernelStage::Select,
        "offset_shell_unmeasured_skin",
        format!(
            "offset_shell: cannot tell whether ({:.4}, {:.4}, {:.4}) lies on the offset skin: its \
             distance to retained face {face} could not be measured ({reason})",
            point.x, point.y, point.z
        ),
    )
}

/// One seed per REGION of the source face, in arrangement order: a face whose
/// trim is pinched into several regions touching only at vertices — a bore
/// mouth tangent to all four edge setbacks of a top face — owns one offset skin
/// per region, and each needs its own seed. An ordinary face has one region, so
/// `seeds[0]` is the seed it always had.
pub(super) fn source_seeds(source: &BrepSolid, face_id: u64) -> Result<Vec<Vec2>, KernelRefusal> {
    let face = standalone_face(source, face_id)?;
    let fragments = fragment_solid(&face, SOURCE_OPERAND, &empty_imprint())?;
    if fragments.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Select, "offset_shell_no_interior_seed", "offset_shell: source face has no interior seed"));
    }
    Ok(fragments.iter().map(|fragment| fragment.test_uv).collect())
}

/// The seed an OFFSET carrier's fragment selection should prefer: the
/// source face's interior seed point moved `distance` along the face's
/// (inward-for-positive) offset direction, then located on the carrier
/// surface by projection.
///
/// Reusing the source seed's raw uv assumes the carrier is parameterised
/// like the source face. A padded plane is not — the pad slides the plane's
/// net under the cloned pcurves, so the source uv lands `extension` short of
/// (or past) its true offset image; a full-domain sphere rebuild replaces the
/// fitted net outright. Both put the raw seed in the wrong fragment (the
/// frustum-base annulus above: the seed read OUTSIDE its own annulus, and
/// the disk over the dropped hole survived only by a distance coin-flip).
/// `None` when the moved point does not lie on the carrier (a shadowed or
/// collapsed offset — keep the raw seed and let the fallbacks decide).
pub(super) fn offset_seed(
    source_face: &FaceRecord,
    raw_seed: Vec2,
    carrier_surface: &crate::NurbsSurface,
    distance: f64,
    band: f64,
) -> Result<Option<Vec2>, KernelRefusal> {
    // `face_offsets` is signed ALONG the outward normal; offset-shell's
    // positive distance moves opposite it.
    let sample = face_offsets(source_face).at(raw_seed.x, raw_seed.y, -distance).or_refuse(KernelStage::Refine, "at")?;
    let projection = project_point_to_surface(carrier_surface, sample.point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    if !projection.distance.is_finite() || projection.distance > band {
        return Ok(None);
    }
    Ok(Some(Vec2 {
        x: projection.u,
        y: projection.v,
    }))
}

#[derive(Clone)]
pub(super) struct FragmentEdgeGeometry {
    pub(super) curve: crate::NurbsCurve,
    pub(super) t0: f64,
    pub(super) t1: f64,
}

pub(super) fn fragment_edge_segment_key(
    source: &FragmentEdgeSource,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
) -> Result<[i64; 9], KernelRefusal> {
    let geometry = fragment_edge_geometry(source, solids, imprint)?;
    let mut start = geometry.curve.evaluate(geometry.t0).or_refuse(KernelStage::Refine, "evaluate")?;
    let middle = geometry.curve.evaluate((geometry.t0 + geometry.t1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
    let mut end = geometry.curve.evaluate(geometry.t1).or_refuse(KernelStage::Refine, "evaluate")?;
    if (end.x, end.y, end.z) < (start.x, start.y, start.z) {
        std::mem::swap(&mut start, &mut end);
    }
    let quantize = |value: f64| (value / 1e-6).round() as i64;
    Ok([
        quantize(start.x),
        quantize(start.y),
        quantize(start.z),
        quantize(middle.x),
        quantize(middle.y),
        quantize(middle.z),
        quantize(end.x),
        quantize(end.y),
        quantize(end.z),
    ])
}

fn fragment_edge_geometry(
    source: &FragmentEdgeSource,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
) -> Result<FragmentEdgeGeometry, KernelRefusal> {
    match source {
        FragmentEdgeSource::Boundary { operand, edge_id }
        | FragmentEdgeSource::SharedBoundary { operand, edge_id } => {
            let edge = solids
                .get(operand)
                .and_then(|solid| solid.edges.iter().find(|edge| edge.id == *edge_id))
                .ok_or_else(|| {
                    format!("offset_shell: missing boundary edge {operand}:{edge_id}")
                }).or_refuse(KernelStage::Refine, "offset_shell_missing_boundary_edge")?;
            Ok(FragmentEdgeGeometry {
                curve: edge.curve.clone(),
                t0: edge.t0,
                t1: edge.t1,
            })
        }
        FragmentEdgeSource::Imprint { piece_id } => {
            let piece = imprint
                .pieces
                .iter()
                .find(|piece| piece.id == *piece_id)
                .ok_or_else(|| format!("offset_shell: missing imprint piece {piece_id}")).or_refuse(KernelStage::Refine, "offset_shell_missing_imprint_piece")?;
            Ok(FragmentEdgeGeometry {
                curve: piece.curve.clone(),
                t0: piece.t0,
                t1: piece.t1,
            })
        }
        FragmentEdgeSource::Derived { curve, t0, t1, .. } => Ok(FragmentEdgeGeometry {
            curve: curve.clone(),
            t0: *t0,
            t1: *t1,
        }),
    }
}

pub(super) fn fragment_edge_lies_on(
    source: &FragmentEdgeGeometry,
    target: &FragmentEdgeGeometry,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let point = source
            .curve
            .evaluate(source.t0 + (source.t1 - source.t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        let projection = crate::project_point_to_curve(&target.curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
        if projection.distance > tolerance
            || projection.u < target.t0 - tolerance
            || projection.u > target.t1 + tolerance
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn fragment_edges_overlap(
    first: &FragmentEdgeGeometry,
    second: &FragmentEdgeGeometry,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    Ok(fragment_edge_lies_on(first, second, tolerance)?
        || fragment_edge_lies_on(second, first, tolerance)?)
}

fn fragment_edge_geometries(
    fragment: &FaceFragmentRecord,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
) -> Result<Vec<FragmentEdgeGeometry>, KernelRefusal> {
    fragment
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| fragment_edge_geometry(&coedge.source, solids, imprint))
        .collect()
}

/// An edge whose whole extent lies within `tolerance` of one point: a PINCH,
/// the collapsed boundary a carved offset carries where its cavity ends on an
/// axis.
fn fragment_edge_is_a_point(edge: &FragmentEdgeGeometry, tolerance: f64) -> Result<bool, KernelRefusal> {
    let start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
    for fraction in [0.25, 0.5, 0.75, 1.0] {
        let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        if point.sub(start).length() > tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn would_overuse_existing_boundary(
    candidate: &FaceFragmentRecord,
    existing: impl Iterator<Item = FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let existing_edges = existing
        .map(|fragment| fragment_edge_geometries(&fragment, solids, imprint))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .filter(|edge| !fragment_edge_is_a_point(edge, tolerance).unwrap_or(false))
        .collect::<Vec<_>>();
    for candidate_edge in fragment_edge_geometries(candidate, solids, imprint)? {
        if fragment_edge_is_a_point(&candidate_edge, tolerance)? {
            continue;
        }
        let mut matches = 0;
        for existing_edge in &existing_edges {
            if fragment_edges_overlap(&candidate_edge, existing_edge, tolerance)? {
                matches += 1;
                if matches >= 2 {
                    return Ok(true);
                }
            }
        }
    }
    Ok(false)
}

pub(super) fn fragment_has_sustained_source_contact(
    fragment: &FaceFragmentRecord,
    source_faces: &[&FaceRecord],
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    for edge in fragment_edge_geometries(fragment, solids, imprint)? {
        for source_face in source_faces {
            let mut sustained = true;
            for fraction in [0.2, 0.5, 0.8] {
                let point = edge
                    .curve
                    .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                let projection = project_point_to_surface(&source_face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
                if projection.distance > tolerance
                    || parameter_point_in_face(
                        source_face,
                        Vec2 {
                            x: projection.u,
                            y: projection.v,
                        },
                        tolerance,
                    ).or_refuse(KernelStage::Refine, "parameter_point_in_face")? == PolygonClass::Outside
                {
                    sustained = false;
                    break;
                }
            }
            if sustained {
                return Ok(true);
            }
        }
    }
    Ok(false)
}


/// Lane P3b row 6: one open sheet solid holding every imprint operand of a
/// shell (the carriers, then the retained source faces the walls are cut
/// against), with face, loop, coedge, edge and vertex ids renumbered per
/// member so nothing collides, and the lookups that map a composite face or
/// edge id back to its operand tag and original id.
pub(super) struct CompositeOperands {
    pub solid: BrepSolid,
    /// Composite face id -> (operand, original face id).
    pub faces: HashMap<u64, (u8, u64)>,
    /// Composite edge id -> (operand, original edge id).
    pub edges: HashMap<u64, (u8, u64)>,
    /// Each member's face index in the composite's face order.
    pub face_index: Vec<usize>,
}

const COMPOSITE_STRIDE: u64 = 1 << 32;

pub(super) fn compose_imprint_operands(members: &[(u8, &BrepSolid)]) -> CompositeOperands {
    let mut faces_out = Vec::new();
    let mut edges_out = Vec::new();
    let mut vertices_out = Vec::new();
    let mut faces = HashMap::default();
    let mut edges = HashMap::default();
    let mut face_index = Vec::with_capacity(members.len());
    for (member, (operand, solid)) in members.iter().enumerate() {
        let offset = (member as u64 + 1) * COMPOSITE_STRIDE;
        for vertex in &solid.vertices {
            vertices_out.push(VertexRecord { id: vertex.id + offset, ..vertex.clone() });
        }
        for edge in &solid.edges {
            edges.insert(edge.id + offset, (*operand, edge.id));
            edges_out.push(EdgeRecord {
                id: edge.id + offset,
                start_vertex_id: edge.start_vertex_id + offset,
                end_vertex_id: edge.end_vertex_id + offset,
                ..edge.clone()
            });
        }
        face_index.push(faces_out.len());
        for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
            faces.insert(face.id + offset, (*operand, face.id));
            let mut copy = face.clone();
            copy.id += offset;
            for loop_record in &mut copy.loops {
                loop_record.id += offset;
                for coedge in &mut loop_record.coedges {
                    coedge.id += offset;
                    coedge.edge_id += offset;
                }
            }
            faces_out.push(copy);
        }
    }
    CompositeOperands {
        solid: BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
            id: 1,
            vertices: vertices_out,
            edges: edges_out,
            shells: vec![ShellRecord { id: 1, faces: faces_out }],
            genus: 0,
            kind: crate::BodyKind::Solid,
        },
        faces,
        edges,
        face_index,
    }
}

/// The composite's imprint record in the shell's operand space: every
/// `FaceKey`, pcurve, edge split and barrier edge keyed by (operand,
/// original id); entries that fold onto the same key (a source edge shared by
/// two retained faces, a face tagged as operand 0 in one pair and 1 in
/// another) are merged the way `merge_pair_imprint` merges them across pairs.
/// Piece and vertex ids are the driver's own and stay unique.
/// `tolerance` / `scale`: the shell's own, for the cross-pair vertex identity
/// below (offset-shell lane 2026-10-27). The per-pair path never shared a
/// vertex between two pair calls except through [`merge_vertex`]: a positional
/// merge at `tolerance.max(1e-5) * merge_scale(scale)` that keeps the first
/// point and moves no curve end. The composite's builder registry is tighter
/// (`tolerance * 10 * merge_scale`), so the per-pair band is applied here to
/// the translated record — the slot one hair under twice the distance needs
/// its two wall-offset junctions 1e-4 apart to be one vertex, exactly as the
/// per-pair merge made them — while the driver's residual-evidence merge,
/// which reached 9.5e-4 across pairs and broke the successive-fillet shell,
/// is off for the composite call (`canonicalize_imprint_junctions`).
pub(super) fn translate_composite_imprint(
    record: ImprintResultRecord,
    composite: &CompositeOperands,
    tolerance: f64,
    scale: f64,
) -> Result<ImprintResultRecord, KernelRefusal> {
    let face = |key: &FaceKey| -> Result<FaceKey, KernelRefusal> {
        let (operand, face_id) = *composite.faces.get(&key.face_id).ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Intersect, "offset_shell.composite", format!("composite face {} unknown", key.face_id))
        })?;
        Ok(FaceKey { operand, face_id })
    };
    let edge = |operand_edge: u64| -> Result<(u8, u64), KernelRefusal> {
        composite.edges.get(&operand_edge).copied().ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Intersect, "offset_shell.composite", format!("composite edge {operand_edge} unknown"))
        })
    };
    let mut out = empty_imprint();
    out.vertices = record.vertices;
    out.tangent_nodes = record.tangent_nodes;
    out.section_evidence = record.section_evidence;
    out.tangent_sections = record.tangent_sections;
    for piece in record.pieces {
        let mut pcurves = Vec::with_capacity(piece.pcurves.len());
        for pcurve in piece.pcurves {
            let key = face(&FaceKey { operand: pcurve.operand, face_id: pcurve.face_id })?;
            pcurves.push(FacePcurve { operand: key.operand, face_id: key.face_id, pcurve: pcurve.pcurve });
        }
        let shared_edge = match piece.shared_edge {
            Some((_, edge_id, aligned)) => {
                let (operand, original) = edge(edge_id)?;
                Some((operand, original, aligned))
            }
            None => None,
        };
        out.pieces.push(ImprintPieceRecord {
            id: piece.id,
            curve: piece.curve,
            t0: piece.t0,
            t1: piece.t1,
            start_vertex_id: piece.start_vertex_id,
            end_vertex_id: piece.end_vertex_id,
            pcurves,
            support_faces: [face(&piece.support_faces[0])?, face(&piece.support_faces[1])?],
            shared_edge,
        });
    }
    for by_face in record.by_face {
        let key = face(&FaceKey { operand: by_face.operand, face_id: by_face.face_id })?;
        let entry = if let Some(entry) = out
            .by_face
            .iter_mut()
            .find(|entry| entry.operand == key.operand && entry.face_id == key.face_id)
        {
            entry
        } else {
            out.by_face.push(FaceImprints { operand: key.operand, face_id: key.face_id, piece_ids: Vec::new() });
            out.by_face.last_mut().unwrap()
        };
        for id in by_face.piece_ids {
            if !entry.piece_ids.contains(&id) {
                entry.piece_ids.push(id);
            }
        }
    }
    for split in record.edge_splits {
        let (operand, edge_id) = edge(split.edge_id)?;
        if let Some(existing) = out
            .edge_splits
            .iter_mut()
            .find(|entry| entry.operand == operand && entry.edge_id == edge_id)
        {
            for parameter in split.parameters {
                if !existing.parameters.iter().any(|value| (*value - parameter).abs() <= 1e-8) {
                    existing.parameters.push(parameter);
                }
            }
        } else {
            out.edge_splits.push(EdgeSplitRecord { operand, edge_id, parameters: split.parameters });
        }
    }
    for (_, edge_id) in record.barrier_edges {
        let key = edge(edge_id)?;
        if !out.barrier_edges.contains(&key) {
            out.barrier_edges.push(key);
        }
    }
    for (first, second) in record.cosurface_pairs {
        out.cosurface_pairs.push((face(&first)?, face(&second)?));
    }
    // The per-pair cross-pair vertex identity (see the doc above): a greedy
    // positional registry in the driver's minting order, first point kept.
    let merge = tolerance.max(1e-5) * crate::tolerance::merge_scale(scale);
    let mut kept: Vec<ImprintVertex> = Vec::with_capacity(out.vertices.len());
    let mut remap: HashMap<u64, u64> = HashMap::default();
    for vertex in &out.vertices {
        match kept.iter().find(|candidate| candidate.point.sub(vertex.point).length() <= merge) {
            Some(candidate) => {
                remap.insert(vertex.id, candidate.id);
            }
            None => kept.push(ImprintVertex { id: vertex.id, point: vertex.point }),
        }
    }
    if !remap.is_empty() {
        out.vertices = kept;
        for piece in &mut out.pieces {
            if let Some(id) = remap.get(&piece.start_vertex_id) {
                piece.start_vertex_id = *id;
            }
            if let Some(id) = remap.get(&piece.end_vertex_id) {
                piece.end_vertex_id = *id;
            }
        }
    }
    Ok(out)
}
