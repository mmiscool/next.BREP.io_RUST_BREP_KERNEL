use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

fn face_normal(fragment: &FaceFragmentRecord) -> Result<Vec3, KernelRefusal> {
    let normal = fragment
        .surface
        .normal(fragment.test_uv.x, fragment.test_uv.y).or_refuse(KernelStage::Select, "normal")?;
    Ok(if fragment.same_sense {
        normal
    } else {
        normal.scale(-1.0)
    })
}

/// Classify a fragment by its primary test point, letting the extra
/// interior points overturn the verdict only when they unanimously
/// contradict it.  An On primary keeps the normal-comparison path (coplanar
/// fragments legitimately sit On everywhere); extras voting On are ignored
/// as near-boundary noise.  A single test point landing in a pathological
/// spot must not decide the whole fragment.
fn classify_fragment(
    classifier: &SolidClassifier,
    fragment: &FaceFragmentRecord,
    debug: bool,
) -> Result<crate::PointClassification, KernelRefusal> {
    let primary = classifier.classify(fragment.test_point).or_refuse(KernelStage::Select, "classify")?;
    if debug {
        eprintln!(
            "frag src_face={} test=({:.6},{:.6},{:.6}) primary={:?} extras={}",
            fragment.source_face_id,
            fragment.test_point.x,
            fragment.test_point.y,
            fragment.test_point.z,
            primary.class,
            fragment.extra_test_points.len()
        );
    }
    if primary.class == PointClass::On {
        // A genuine coplanar overlap sits On over the WHOLE fragment; a
        // point or line TANGENCY is On only along a measure-zero locus that
        // can pass exactly through the max-clearance primary (a sphere
        // resting on a face touches its centre; a tangent cylinder's ruling
        // runs through a wall's midline). Triggering the coincident-face
        // logic there dissolves a face that merely touches the other body.
        // Trust On only when the extra interior points do not unanimously
        // disagree; otherwise adopt their verdict.
        let mut in_votes = 0usize;
        let mut out_votes = 0usize;
        let mut on_votes = 0usize;
        for &point in &fragment.extra_test_points {
            match classifier.classify(point).or_refuse(KernelStage::Select, "classify")?.class {
                PointClass::On => on_votes += 1,
                PointClass::In => in_votes += 1,
                PointClass::Out => out_votes += 1,
            }
        }
        // Disambiguate a measure-zero tangency from a partial coincidence
        // REGION that simply has no imprint splits (an open-sheet operand
        // overlapping part of a face): a region stays On just off the
        // primary, a tangency leaves the band immediately. Probe a short
        // step from the primary toward each extra before overturning.
        if on_votes == 0 && (in_votes >= 2 || out_votes >= 2) {
            for &extra in &fragment.extra_test_points {
                let step = fragment
                    .test_point
                    .add(extra.sub(fragment.test_point).scale(0.05));
                if classifier.classify(step).or_refuse(KernelStage::Select, "classify")?.class == PointClass::On {
                    on_votes += 1;
                    break;
                }
            }
        }
        if on_votes == 0 && in_votes == 0 && out_votes >= 2 {
            if debug {
                eprintln!(
                    "frag src_face={} tangential On overturned -> Out ({out_votes} extras)",
                    fragment.source_face_id
                );
            }
            return Ok(crate::PointClassification {
                class: PointClass::Out,
                on_normal: None,
            });
        }
        if on_votes == 0 && out_votes == 0 && in_votes >= 2 {
            if debug {
                eprintln!(
                    "frag src_face={} tangential On overturned -> In ({in_votes} extras)",
                    fragment.source_face_id
                );
            }
            return Ok(crate::PointClassification {
                class: PointClass::In,
                on_normal: None,
            });
        }
        return Ok(primary);
    }
    // A stray In/Out on a face that is actually coincident with the other
    // solid's boundary (near-coincident across a noise-scale gap that the
    // tight On band misses) is rescued to On here, using a size-derived band.
    // This never flips In<->Out; it only recovers the boundary verdict a
    // coplanar overlap should have produced.
    if let Some(on_normal) = classifier.coincident_on_normal(fragment.test_point).or_refuse(KernelStage::Select, "coincident_on_normal")? {
        // Proximity to another face's interior does not establish surface
        // coincidence: a thin transverse strip can lie entirely inside the
        // noise band beside a cylinder. Only parallel tangent planes admit
        // the broader On rescue; otherwise preserve the tight classification.
        if face_normal(fragment)?.cross(on_normal).length() <= 1e-4 {
            if debug {
                eprintln!(
                    "frag src_face={} coincident-On rescue {:?} -> On",
                    fragment.source_face_id, primary.class
                );
            }
            return Ok(crate::PointClassification {
                class: PointClass::On,
                on_normal: Some(on_normal),
            });
        }
    }
    if fragment.extra_test_points.is_empty() {
        return Ok(primary);
    }
    let mut agreeing = 0usize;
    let mut opposing = 0usize;
    for &point in &fragment.extra_test_points {
        match classifier.classify(point).or_refuse(KernelStage::Select, "classify")?.class {
            class if class == primary.class => agreeing += 1,
            PointClass::On => {}
            _ => opposing += 1,
        }
    }
    if opposing >= 2 && agreeing == 0 {
        let overturned = match primary.class {
            PointClass::In => PointClass::Out,
            _ => PointClass::In,
        };
        if debug {
            eprintln!(
                "frag src_face={} vote overturned {:?} -> {:?} ({} opposing)",
                fragment.source_face_id, primary.class, overturned, opposing
            );
        }
        return Ok(crate::PointClassification {
            class: overturned,
            on_normal: None,
        });
    }
    Ok(primary)
}

/// Adjacency key of a boundary coedge: the ORIGINAL edge id paired with
/// the quantized 3D midpoint of the coedge's chain.  The id distinguishes
/// geometrically coincident but distinct edges (tangency lines); the
/// midpoint distinguishes split siblings of one original edge, whose
/// fragment records only know the unsplit id.  True neighbors share both.
fn boundary_coedge_key(
    fragment: &FaceFragmentRecord,
    edge_id: u64,
    pcurve: &crate::NurbsCurve,
    quantum: f64,
) -> Option<(u64, [i64; 3])> {
    let [start, end] = pcurve.domain().ok()?;
    let uv = pcurve.evaluate((start + end) * 0.5).ok()?;
    let point = fragment.surface.evaluate(uv.x, uv.y).ok()?;
    Some((
        edge_id,
        [
            (point.x / quantum).round() as i64,
            (point.y / quantum).round() as i64,
            (point.z / quantum).round() as i64,
        ],
    ))
}

/// On against a COSURFACE PARTNER: a face of the other operand that the
/// imprint read as the same surface as this fragment's source face, and whose
/// trim holds the fragment's point.
///
/// The imprint exchanges boundary curves between such a pair instead of
/// intersecting them, so every section on the overlap is built on their being
/// one surface; point classification decides the same question again with its
/// own size-derived band, and at the band's edge the two answers split. The
/// revolve_pole union's side planes are the case: the wedge's plane stands
/// 0 .. 3.4e-6 off the revolve's over their 6 x 8 overlap. The wedge's
/// fragment there sits 2.0e-6 from the revolve's plane and was rescued On;
/// the revolve's twin sits 2.425e-6 from the wedge's plane against a band of
/// 2.33e-6 and read Out. A union (wedge first) then kept both copies, a
/// revolve − wedge kept the revolve's, and both refused with one-use edges.
///
/// The point must project onto the partner within the assembler's weld, land
/// inside its trim, and the two carriers' normals there must be parallel
/// within the imprint pair classifier's bound. Escape hatch
/// `BREP_COSURFACE_PAIR_ON=0`.
fn cosurface_on(
    fragment: &FaceFragmentRecord,
    partners: &HashMap<u64, Vec<&FaceRecord>>,
    tolerance: f64,
) -> Result<Option<crate::PointClassification>, KernelRefusal> {
    if std::env::var("BREP_COSURFACE_PAIR_ON").as_deref() == Ok("0") {
        return Ok(None);
    }
    let Some(faces) = partners.get(&fragment.source_face_id) else {
        return Ok(None);
    };
    let Ok(own) = face_normal(fragment) else {
        return Ok(None);
    };
    let weld = assembler_weld(tolerance);
    for partner in faces {
        let Ok(projection) = project_point_to_surface(&partner.surface, fragment.test_point) else {
            continue;
        };
        if projection.distance > weld {
            continue;
        }
        let uv = Vec2 {
            x: projection.u,
            y: projection.v,
        };
        if parameter_point_in_face(partner, uv, 1e-9).or_refuse(KernelStage::Select, "parameter_point_in_face")?
            != PolygonClass::Inside
        {
            continue;
        }
        let Ok(normal) = partner.surface.normal(projection.u, projection.v) else {
            continue;
        };
        let normal = if partner.same_sense { normal } else { normal.scale(-1.0) };
        if own.cross(normal).length() > PINCH_ANGULAR_TOLERANCE {
            continue;
        }
        return Ok(Some(crate::PointClassification {
            class: PointClass::On,
            on_normal: Some(normal),
        }));
    }
    Ok(None)
}

/// Keep/drop decisions for one operand's fragments (Golovanov §6.3:
/// untouched faces join the result shell by edge adjacency, not per-face
/// point classification).  Point classification runs only for fragments
/// the boolean TOUCHED (cut coedges), fragments inside the other solid's
/// On band, and fragments adjacent to an On-classified fragment (fate
/// flips along On-region boundaries, so On fates never propagate).
/// Everything else inherits its fate across shared boundary-coedge
/// midpoints; conflicting inheritance aborts loudly, and components with
/// no decided member classify one representative.
fn side_keep_decisions(
    fragments: &[FaceFragmentRecord],
    classifier: &SolidClassifier,
    first_operand: bool,
    operation: BooleanOperation,
    tolerance: f64,
    debug: bool,
    barrier_edges: &HashSet<(u8, u64)>,
    cosurface_partners: &HashMap<u64, Vec<&FaceRecord>>,
    coincident: &CoincidentPairs,
) -> Result<Vec<bool>, KernelRefusal> {
    let classify_side = |fragment: &FaceFragmentRecord| -> Result<crate::PointClassification, KernelRefusal> {
        let classification = classify_fragment(classifier, fragment, debug)?;
        if classification.class == PointClass::On {
            return Ok(classification);
        }
        match cosurface_on(fragment, cosurface_partners, tolerance)? {
            Some(on) => {
                if debug {
                    eprintln!(
                        "frag src_face={} {:?} -> On: its point lies in the trim of a face the imprint read as the same surface",
                        fragment.source_face_id, classification.class
                    );
                }
                Ok(on)
            }
            None => Ok(classification),
        }
    };
    let decide = |fragment: &FaceFragmentRecord,
                  classification: &crate::PointClassification|
     -> Result<(bool, bool), KernelRefusal> {
        if debug {
            eprintln!(
                "frag {} src_face={} test=({:.6},{:.6},{:.6}) class={:?}",
                if first_operand { "A" } else { "B" },
                fragment.source_face_id,
                fragment.test_point.x,
                fragment.test_point.y,
                fragment.test_point.z,
                classification.class,
            );
        }
        let propagatable = classification.class != PointClass::On;
        // A fragment On against a coincident copy of the other operand: the
        // result keeps the copy its material lies behind (`coincident.rs`).
        if classification.class == PointClass::On {
            if let Some(partner) = coincident_partner(fragment, cosurface_partners, tolerance)? {
                let key = if first_operand { (fragment.source_face_id, partner) } else { (partner, fragment.source_face_id) };
                if let Some(pair) = coincident.get(&key) {
                    let own = if first_operand { 0 } else { 1 };
                    let keep = keeper(*pair, operation) == Some(own);
                    if debug {
                        eprintln!(
                            "frag {} src_face={} On against coincident copy {partner}: {:?} keeps {:?}, keep={keep}",
                            if first_operand { "A" } else { "B" },
                            fragment.source_face_id,
                            operation,
                            keeper(*pair, operation),
                        );
                    }
                    return Ok((keep, false));
                }
            }
        }
        let keep = if first_operand {
            match classification.class {
                PointClass::On => {
                    let same = classification
                        .on_normal
                        .ok_or_else(|| {
                            "boolean selection: boundary classification lacks normal".to_string()
                        }).or_refuse(KernelStage::Select, "csg.boolean.select")?
                        .dot(face_normal(fragment)?)
                        > 0.0;
                    match operation {
                        BooleanOperation::Subtract => !same,
                        BooleanOperation::Union | BooleanOperation::Intersect => same,
                    }
                }
                PointClass::In => matches!(operation, BooleanOperation::Intersect),
                PointClass::Out => !matches!(operation, BooleanOperation::Intersect),
            }
        } else {
            match classification.class {
                PointClass::On => false,
                PointClass::Out => matches!(operation, BooleanOperation::Union),
                PointClass::In => matches!(
                    operation,
                    BooleanOperation::Intersect | BooleanOperation::Subtract
                ),
            }
        };
        Ok((keep, propagatable))
    };

    let mut keep: Vec<Option<bool>> = vec![None; fragments.len()];
    let mut propagatable: Vec<bool> = vec![false; fragments.len()];
    for (index, fragment) in fragments.iter().enumerate() {
        let touched = fragment
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .any(|coedge| !matches!(coedge.source, FragmentEdgeSource::Boundary { .. }));
        if touched || classifier.near_surface(fragment.test_point).or_refuse(KernelStage::Select, "near_surface")? {
            let classification = classify_side(fragment)?;
            let (value, source) = decide(fragment, &classification)?;
            keep[index] = Some(value);
            propagatable[index] = source;
        }
    }

    let quantum = tolerance.max(1e-5) * 10.0;
    let mut edge_fragments: HashMap<(u64, [i64; 3]), Vec<usize>> = HashMap::default();
    for (index, fragment) in fragments.iter().enumerate() {
        for coedge in fragment
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            if let FragmentEdgeSource::Boundary { edge_id, .. } = coedge.source {
                // A BARRIER edge carries the other operand's coincident
                // section (a shared ring: the inscribed sphere's equator IS
                // this operand's cap edge). Fate flips exactly there — the
                // wall is Outside while the cap disc lies Inside the other
                // solid — so adjacency inheritance must not flood across it;
                // isolated fragments then classify directly (equator-tangent:
                // the bottom cap wrongly inherited keep from the wall).
                let fragment_operand = if first_operand { 0u8 } else { 1u8 };
                if barrier_edges.contains(&(fragment_operand, edge_id)) {
                    continue;
                }
                if let Some(key) = boundary_coedge_key(fragment, edge_id, &coedge.pcurve, quantum) {
                    let entry = edge_fragments.entry(key).or_default();
                    if !entry.contains(&index) {
                        entry.push(index);
                    }
                }
            }
        }
    }

    // Fate flips along On-region boundaries: neighbors of On fragments
    // classify directly instead of inheriting.
    let mut on_adjacent: Vec<bool> = vec![false; fragments.len()];
    for neighbors in edge_fragments.values() {
        if neighbors
            .iter()
            .any(|&index| keep[index].is_some() && !propagatable[index])
        {
            for &index in neighbors {
                on_adjacent[index] = true;
            }
        }
    }
    for index in 0..fragments.len() {
        if on_adjacent[index] && keep[index].is_none() {
            let classification = classify_side(&fragments[index])?;
            let (value, source) = decide(&fragments[index], &classification)?;
            keep[index] = Some(value);
            propagatable[index] = source;
        }
    }

    loop {
        let mut progressed = false;
        for neighbors in edge_fragments.values() {
            let mut fate: Option<(bool, usize)> = None;
            for &index in neighbors {
                if !propagatable[index] {
                    continue;
                }
                if let Some(value) = keep[index] {
                    if let Some((known, source)) = fate {
                        if known != value {
                            // Tolerance-band geometry (near-coincident caps,
                            // tangency lines) can flip fate across pieces the
                            // guards cannot fully characterize.  Propagation
                            // is an optimization: fall back to classifying
                            // every fragment directly.
                            if debug {
                                eprintln!(
                                    "frag {} propagation conflict (faces {} vs {}) — \
                                     falling back to direct classification",
                                    if first_operand { "A" } else { "B" },
                                    fragments[source].source_face_id,
                                    fragments[index].source_face_id,
                                );
                            }
                            let mut direct = Vec::with_capacity(fragments.len());
                            for fragment in fragments {
                                let classification =
                                    classify_side(fragment)?;
                                direct.push(decide(fragment, &classification)?.0);
                            }
                            return Ok(direct);
                        }
                    } else {
                        fate = Some((value, index));
                    }
                }
            }
            if let Some((value, _)) = fate {
                for &index in neighbors {
                    if keep[index].is_none() {
                        keep[index] = Some(value);
                        propagatable[index] = true;
                        if debug {
                            eprintln!(
                                "frag {} src_face={} inherited keep={value} by adjacency",
                                if first_operand { "A" } else { "B" },
                                fragments[index].source_face_id,
                            );
                        }
                        progressed = true;
                    }
                }
            }
        }
        if !progressed {
            match keep.iter().position(Option::is_none) {
                Some(index) => {
                    let classification = classify_side(&fragments[index])?;
                    let (value, source) = decide(&fragments[index], &classification)?;
                    keep[index] = Some(value);
                    propagatable[index] = source;
                }
                None => break,
            }
        }
    }
    Ok(keep
        .into_iter()
        .map(|value| value.unwrap_or(false))
        .collect())
}

pub(super) fn select_fragments(
    fragments_a: Vec<FaceFragmentRecord>,
    fragments_b: Vec<FaceFragmentRecord>,
    pristine_a: &BrepSolid,
    pristine_b: &BrepSolid,
    operation: BooleanOperation,
    tolerance: f64,
    barrier_edges: &HashSet<(u8, u64)>,
    cosurface_pairs: &[(FaceKey, FaceKey)],
    coincident: &CoincidentPairs,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let mut selected = Vec::new();
    // Each operand's face id -> the other pristine operand's faces the imprint
    // read as the same surface.
    fn face_of(solid: &BrepSolid, face_id: u64) -> Option<&FaceRecord> {
        solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == face_id)
    }
    let mut partners_of_a: HashMap<u64, Vec<&FaceRecord>> = HashMap::default();
    let mut partners_of_b: HashMap<u64, Vec<&FaceRecord>> = HashMap::default();
    for (first, second) in cosurface_pairs {
        let (a_key, b_key) = if first.operand == 0 { (first, second) } else { (second, first) };
        if a_key.operand != 0 || b_key.operand != 1 {
            continue;
        }
        if let Some(face) = face_of(pristine_b, b_key.face_id) {
            partners_of_a.entry(a_key.face_id).or_default().push(face);
        }
        if let Some(face) = face_of(pristine_a, a_key.face_id) {
            partners_of_b.entry(b_key.face_id).or_default().push(face);
        }
    }
    let classifier_b = SolidClassifier::new(pristine_b, tolerance).or_refuse(KernelStage::Select, "new")?;
    let classifier_a = SolidClassifier::new(pristine_a, tolerance).or_refuse(KernelStage::Select, "new")?;
    let keep_a = side_keep_decisions(
        &fragments_a,
        &classifier_b,
        true,
        operation,
        tolerance,
        debug,
        barrier_edges,
        &partners_of_a,
        coincident,
    )?;
    for (fragment, keep) in fragments_a.into_iter().zip(keep_a) {
        if keep {
            selected.push(fragment);
        }
    }
    let keep_b = side_keep_decisions(
        &fragments_b,
        &classifier_a,
        false,
        operation,
        tolerance,
        debug,
        barrier_edges,
        &partners_of_b,
        coincident,
    )?;
    for (mut fragment, keep) in fragments_b.into_iter().zip(keep_b) {
        if !keep {
            continue;
        }
        if matches!(operation, BooleanOperation::Subtract) {
            fragment.same_sense = !fragment.same_sense;
            for loop_record in &mut fragment.loops {
                loop_record.coedges.reverse();
                for coedge in &mut loop_record.coedges {
                    coedge.forward = !coedge.forward;
                    coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Select, "reversed")?;
                }
            }
        }
        selected.push(fragment);
    }
    Ok(selected)
}

/// The verdict for one fragment in the n-ary selection.
enum NaryKeep {
    Drop,
    AsIs,
    Reversed,
}

/// Generalized per-fragment keep decision: "vs the SET of other solids".
///
/// `classifiers[k]` classifies a point against pristine operand `k`; `owner` is
/// the fragment's own operand index.  Every classification uses the shared
/// `classify_fragment` helper (coincident-On rescue + interior-vote), so a
/// coincident shared wall reads `On` with the other solid's outward normal.
/// The rules below reduce EXACTLY to the binary `side_keep_decisions` `decide`
/// closure when there is a single other solid, so N=2 reproduces the binary
/// selection; coincident walls are kept once (from the lowest-indexed operand,
/// oriented outward) by the `owner < other` / normal-agreement guards.
fn nary_keep(
    classifiers: &[SolidClassifier],
    owner: usize,
    fragment: &FaceFragmentRecord,
    operation: BooleanOperation,
    debug: bool,
) -> Result<NaryKeep, KernelRefusal> {
    let n = classifiers.len();
    // Cached outward normal for the On (coincident) orientation tests.
    let outward = |fragment: &FaceFragmentRecord| face_normal(fragment);

    match operation {
        BooleanOperation::Union => {
            // Keep a fragment iff it is OUTSIDE every OTHER operand (not buried);
            // a coincident shared wall is kept once, from the lower-indexed
            // operand, oriented outward.
            for other in 0..n {
                if other == owner {
                    continue;
                }
                let class = classify_fragment(&classifiers[other], fragment, debug)?;
                match class.class {
                    PointClass::In => return Ok(NaryKeep::Drop),
                    PointClass::On => {
                        if other < owner {
                            return Ok(NaryKeep::Drop);
                        }
                        let same = class
                            .on_normal
                            .ok_or_else(|| {
                                "n-ary selection: On classification lacks normal".to_string()
                            }).or_refuse(KernelStage::Select, "csg.boolean.select")?
                            .dot(outward(fragment)?)
                            > 0.0;
                        if !same {
                            return Ok(NaryKeep::Drop);
                        }
                    }
                    PointClass::Out => {}
                }
            }
            Ok(NaryKeep::AsIs)
        }
        BooleanOperation::Intersect => {
            // Keep a fragment iff it is INSIDE every OTHER operand; coincident
            // shared walls kept once, lower operand, oriented outward.
            for other in 0..n {
                if other == owner {
                    continue;
                }
                let class = classify_fragment(&classifiers[other], fragment, debug)?;
                match class.class {
                    PointClass::Out => return Ok(NaryKeep::Drop),
                    PointClass::On => {
                        if other < owner {
                            return Ok(NaryKeep::Drop);
                        }
                        let same = class
                            .on_normal
                            .ok_or_else(|| {
                                "n-ary selection: On classification lacks normal".to_string()
                            }).or_refuse(KernelStage::Select, "csg.boolean.select")?
                            .dot(outward(fragment)?)
                            > 0.0;
                        if !same {
                            return Ok(NaryKeep::Drop);
                        }
                    }
                    PointClass::In => {}
                }
            }
            Ok(NaryKeep::AsIs)
        }
        BooleanOperation::Subtract => {
            // operands[0] minus the union of operands[1..].
            if owner == 0 {
                // Keep base fragments OUTSIDE every cutter (as-is).  A wall
                // coincident with a cutter is kept only when the normals oppose
                // (the cutter lies on the far side), matching binary A-Subtract.
                for cutter in 1..n {
                    let class = classify_fragment(&classifiers[cutter], fragment, debug)?;
                    match class.class {
                        PointClass::In => return Ok(NaryKeep::Drop),
                        PointClass::On => {
                            let same = class
                                .on_normal
                                .ok_or_else(|| {
                                    "n-ary selection: On classification lacks normal".to_string()
                                }).or_refuse(KernelStage::Select, "csg.boolean.select")?
                                .dot(outward(fragment)?)
                                > 0.0;
                            if same {
                                return Ok(NaryKeep::Drop);
                            }
                        }
                        PointClass::Out => {}
                    }
                }
                Ok(NaryKeep::AsIs)
            } else {
                // A cutter contributes its INWARD-facing wall where it lies
                // inside the base AND outside every OTHER cutter, reversed.
                let base = classify_fragment(&classifiers[0], fragment, debug)?;
                match base.class {
                    // Outside the base cuts nothing; a wall coincident with the
                    // base boundary is owned by the base fragment (binary B-On
                    // drops), so both non-In verdicts drop here.
                    PointClass::Out | PointClass::On => return Ok(NaryKeep::Drop),
                    PointClass::In => {}
                }
                for other in 1..n {
                    if other == owner {
                        continue;
                    }
                    let class = classify_fragment(&classifiers[other], fragment, debug)?;
                    match class.class {
                        PointClass::In => return Ok(NaryKeep::Drop),
                        PointClass::On => {
                            // Shared wall between two cutters inside the base:
                            // the union of cutters keeps it once, lower operand,
                            // outward — then it is reversed like any cutter wall.
                            if other < owner {
                                return Ok(NaryKeep::Drop);
                            }
                            let same = class
                                .on_normal
                                .ok_or_else(|| {
                                    "n-ary selection: On classification lacks normal".to_string()
                                }).or_refuse(KernelStage::Select, "csg.boolean.select")?
                                .dot(outward(fragment)?)
                                > 0.0;
                            if !same {
                                return Ok(NaryKeep::Drop);
                            }
                        }
                        PointClass::Out => {}
                    }
                }
                Ok(NaryKeep::Reversed)
            }
        }
    }
}

/// One-pass n-ary fragment selection against the SET of pristine operands.
pub(super) fn select_fragments_nary(
    fragments: &[Vec<FaceFragmentRecord>],
    pristine: &[BrepSolid],
    operation: BooleanOperation,
    tolerance: f64,
    debug: bool,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    let classifiers = pristine
        .iter()
        .map(|solid| SolidClassifier::new(solid, tolerance))
        .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Select, "csg.boolean.select")?;
    let mut selected = Vec::new();
    for (owner, owner_fragments) in fragments.iter().enumerate() {
        for fragment in owner_fragments {
            match nary_keep(&classifiers, owner, fragment, operation, debug)? {
                NaryKeep::Drop => {}
                NaryKeep::AsIs => selected.push(fragment.clone()),
                NaryKeep::Reversed => {
                    let mut reversed = fragment.clone();
                    reverse_fragment(&mut reversed)?;
                    selected.push(reversed);
                }
            }
        }
    }
    Ok(selected)
}


