use super::*;

/// TRANSVERSALITY FLOOR for a mitre's bisector plane against the segments it
/// trims: `|n̂·û| ≥ 0.1`.
///
/// It is the SAME number `extrude_profile_brep` refuses a prism at ("sweep
/// direction is nearly parallel to profile plane") and the same one the skinning
/// builder's near-parallel guard uses, and for the same reason: a cap plane that
/// grazes the sweep direction does not cut the prism, it shears along it. A
/// mitre's bisector plane is a cap plane, so it inherits the bar — in two
/// places, and they are different questions:
///   - at a JOINT, `û·n̂ = cos(θ/2)`, so the floor IS a largest joint angle
///     ([`fold_angle`]);
///   - at the two CAPS, `û·n̂` is the section plane's own angle to the path,
///     which `Rigid` placement lets the user draw at any slant.
const MITER_TRANSVERSALITY_FLOOR: f64 = 0.1;

/// The FOLD ANGLE: the joint angle at which a mitre collapses,
/// `2·acos(0.1)` = 168.522°.
///
/// A mitre trims both arms to the plane that bisects their directions, and the
/// section's footprint on that plane is the section STRETCHED by `1/cos(θ/2)`
/// across the bend. At the fold angle that factor is 10 and the plane is within
/// ~5.7° of containing both segment directions; at 180° exactly the two
/// directions are antiparallel, `û + v̂ = 0`, every plane through the joint
/// bisects them equally and the section sweeps back along the path it came in
/// on. There is no mitre there to build, which is why this is refused by the
/// ANGLE rather than left to the room check below: past the fold the room check
/// would blame the section's extent for a path that has doubled back on itself.
fn fold_angle() -> f64 {
    2.0 * MITER_TRANSVERSALITY_FLOOR.acos()
}

/// Sweep a closed planar SECTION along a CORNERED POLYLINE, mitring every joint:
/// the section is swept along each segment and both sweeps are trimmed to the
/// joint's BISECTOR PLANE, where they share ONE face loop — the same vertices,
/// the same edges, no bulkhead face between them. A picture-frame corner.
///
/// The path may be OPEN — `n` segments, `n − 1` mitres, one cap at each end — or
/// CLOSED, which is a picture frame's whole loop: `n` segments, `n` mitres (the
/// last of them the CLOSING one), and NO caps. Everything below is written once
/// for both, and says so where they differ.
///
/// # Why one shared loop exists at all
///
/// Write `û`, `v̂` for the two segment directions at a joint `P`, `θ` for the
/// angle between them. The bisector plane's normal is
///
/// ```text
///     n̂ = (û + v̂)/|û + v̂|,   so   û·n̂ = v̂·n̂ = cos(θ/2)
/// ```
///
/// — equal and positive, which is what makes it the MITRE plane rather than
/// just some plane through the joint. Reflection `R` in that plane maps
/// `û → −v̂`, and composed with the joint's own rotation `Rot` (the minimal
/// rotation taking `û` to `v̂` about `ŵ = û × v̂`, which is exactly what
/// `pathAlign` does to the section at the joint) it satisf
/// `R ∘ Rot = S`, the reflection in the plane through `P` perpendicular to `û`.
/// Since `S` moves a point only ALONG `û`, and `R` maps lines along `û` to
/// lines along `v̂` while fixing the bisector plane pointwise,
///
/// ```text
///     Proj_v̂ ∘ Rot = Proj_û          (pointwise, for every point in space)
/// ```
///
/// The arriving section projected along `û` onto the bisector plane IS the
/// departing (rotated) section projected along `v̂` onto it. The shared loop is
/// therefore exact, for ANY section and ANY joint angle — it is not a fit, and
/// it does not need the section to be circular, centred on the path, or square
/// to it. That identity is the whole construction: this builder never forms
/// `Rot` at all, it just projects each ring along the current segment's own
/// direction onto the next plane, and projections along one direction compose
/// (`Proj_Q ∘ Proj_P = Proj_Q`), so the ring it lands on is the next segment's
/// section's own projection.
///
/// # What is built, exactly
///
/// `n` segments and a section of `m` curves give `n + 1` RINGS when the path is
/// OPEN — the start cap loop, one mitre loop per interior joint, the end cap loop
/// — and `n` when it is CLOSED, one per joint, the ring index wrapping. Each is
/// the affine image of the section under a parallel projection, so each is a
/// NURBS curve of the SAME degree, knots and weights (a projection is affine; a
/// circular arc mitres to an exact rational ellipse arc, not a fitted one). Wall
/// `(i, j)` is the ruled surface between ring `i`'s and ring `i+1`'s `j`-th curve,
/// and because both are projections of the same section along the same `û_i`,
/// every ruling is exactly along `û_i`: the wall is the exact swept surface of
/// that section curve, and all four of its pcurves are exact PARAMETER LINES.
///
/// ```text
///     OPEN:   V = (n+1)·m   E = (n+1)·m ring + n·m side   F = n·m walls + 2 caps
///             V − E + F = 2                                      (genus 0)
///     CLOSED: V = n·m       E = n·m ring + n·m side       F = n·m walls, no caps
///             V − E + F = 0                                      (genus 1)
/// ```
///
/// A CLOSED run has one fewer ring than an open one with the same segment count —
/// its rings are the `n` mitre loops and nothing else, because the loop that
/// would have been the start cap IS the loop that would have been the end cap —
/// and the walls close the surface on their own. A frame is a torus: one hole,
/// `V − E + F = 0`, `genus 1`.
///
/// No interior face either way: the two sweeps at a joint meet AT the shared
/// loop, so there is nothing between them to cap. That is the difference from
/// unioning one prism per segment, which leaves the inner wedge doubly covered
/// and the outer wedge empty.
///
/// # Why a CLOSED loop closes — the closing-frame policy
///
/// The section is carried from one segment to the next by the joint's own
/// rotation `Rot_j`, the minimal rotation `d_{j−1} → d_j` about the joint point,
/// which is what `pathAlign` does at a corner. Carried once round a closed loop
/// it comes back transformed by
///
/// ```text
///     M = Rot_0 ∘ Rot_{n−1} ∘ … ∘ Rot_1
/// ```
///
/// and the frame CLOSES exactly when `M` leaves the section where the ring at
/// joint 0 already puts it. For a PLANAR loop it does, and for two reasons that
/// compose:
///
///   - every `Rot_j` turns about the SAME axis — the path plane's normal — so
///     they commute and `M`'s rotation part is the rotation by the loop's TOTAL
///     TURNING. For a closed planar polyline that is `Σ θ_j = ±2π` signed (Hopf's
///     Umlaufsatz: a reflex corner subtracts, so an L-shaped frame closes as
///     surely as a rectangular one), and a `±2π` rotation is the IDENTITY. `M` is
///     therefore a pure TRANSLATION.
///   - a pure translation that maps segment 0's LINE to itself is a translation
///     ALONG `d_0`, and `M` does map it to itself: `Rot_1` carries line 0 onto
///     line 1, `Rot_2` line 1 onto line 2, …, `Rot_0` line `n−1` back onto
///     line 0. And a translation along `d_0` is exactly what `Proj_{d_0}`
///     annihilates.
///
/// So `M` moves the section only where the projection onto joint 0's plane cannot
/// see it. Ring 0 is the placed section's own footprint on the CLOSING joint's
/// bisector plane, and the ring the chain of projections lands on after the last
/// segment IS ring 0 — identically, not approximately. The closing wall is
/// therefore built exactly like every other wall, between two rings that are one
/// section projected along one direction, and the frame has no seam.
///
/// The builder MEASURES that rather than asserting it: it takes the last
/// projection, compares it to ring 0 control point by control point, and refuses
/// if the two differ by more than the position floor. On a planar frame the
/// residual is rounding — 0 exactly on a 40 × 28 rectangular frame, 1.0e-14 with
/// its section centred, 4.4e-15 on a triangle, against floors of ~1e-4, so ten
/// orders inside the bar; the check is there for the path where the identity does
/// NOT hold.
///
/// A SPATIAL closed polyline does not. Its joint axes stop being parallel, the
/// rotations no longer commute, and `M` is a SCREW about segment 0's line: the
/// translation along `d_0` the projection cannot see, plus a ROLL about `d_0` by
/// the loop's HOLONOMY ([`frame_holonomy`]), which it can. That roll is the solid
/// angle enclosed by the spherical polygon the segment DIRECTIONS trace, modulo
/// `2π`. Each minimal rotation is parallel transport along the great-circle arc
/// between two directions, so Gauss–Bonnet applies. A planar loop's directions
/// run along one great circle, whose hemisphere is `2π`, the identity.
///
/// # The COUNTER-TWIST, for a spatial loop
///
/// The frame closes with the smallest roll that closes it ([`SweepClosure`]), and
/// only where it has to. A holonomy that reads zero to the joint rotations' own
/// rounding — a loop whose indicatrix encloses a whole number of hemispheres,
/// which every loop with a mirror symmetry does — is built exactly as a planar
/// frame is and reports no twist. Otherwise `−holonomy` is SHARED over the
/// segments in proportion to their length. Each share is under `π/2`, because no
/// side of a closed polygon is longer than the other sides together. Each share
/// is laid down as a ROLL ABOUT ITS SEGMENT'S OWN LINE.
///
/// WHERE on the segment is forced. A roll AT a joint cannot keep the shared loop:
/// `Proj_v̂ ∘ Rot' = Proj_û` needs every point's displacement under `Rot'` to lie
/// along `v̂`, which pins `Rot'` to the minimal rotation and leaves no roll. Along
/// the segment it can: a roll about one segment's line, conjugated through the
/// joint rotation, is the same roll about the next line, so the mitre identity
/// still holds pointwise between the section as it ARRIVES and as it DEPARTS.
/// What a roll cannot do is happen inside a mitre's own reach, because the trim
/// there is the projection of ONE section. So each share lives on its segment's
/// MIDDLE HALF, `[L/4, 3L/4]` from the joint it departs, and the quarter-runs
/// either side stay exact prisms. The run is fixed by the PATH alone: a hole loop
/// or a second region is swept in a call of its own, and a run placed from each
/// section's own mitres would roll a hole somewhere its outer wall does not. The
/// section decides only whether its mitres leave that run untrimmed.
///
/// The twisted piece is EXACT, not fitted. A roll by `φ` is rational in
/// `u = tan(φ/2)`: `cos φ = (1−u²)/(1+u²)` and `sin φ = 2u/(1+u²)`. Taking
/// `u = tan(β/2)·(3v² − 2v³)` gives a roll that starts and ends at ZERO RATE, so
/// the wall is tangent-continuous into both prisms. With the axial advance linear
/// in `v`, every control point shares the weight `1 + u(v)²`, so the wall's
/// section at `v` is the section rigidly rolled by `φ(v)` in its own
/// perpendicular plane. That is a degree-7 rational patch in `v` on the section's
/// own knots in `u`. The whole side (prism, roll, prism) is ONE face on a
/// piecewise-Bézier `v`, so a frame still has one wall per (segment × section
/// curve), named exactly as before.
///
/// MATERIAL. A roll about the path keeps every perpendicular slice's area, so the
/// straight runs keep `A·L` (Cavalieri). The joint exchange does NOT carry over
/// unchanged. It is `tan(θⱼ/2)` times the section's first moment across the bend,
/// and a roll moves an offset section's centroid relative to each bend plane. The
/// law keeps its form, `V = A·ΣL − 2·Σⱼ Mⱼ·tan(θⱼ/2)`, with `Mⱼ` read off the
/// section AS IT ARRIVES at joint `j`, rolled by every share laid down before it.
///
/// # A REQUESTED twist
///
/// A twist the caller asks for joins the counter-twist in the shares, so after the
/// lap the section's slice square to the first side comes back rolled by exactly
/// the requested angle. That closes only when the roll carries the slice onto
/// itself ([`closing_twist`]): a whole number of turns, or a step of its own
/// symmetry about the path. Under a fractional step each curve lands on another,
/// so the closing side's walls end on ring 0's curve `curve_shift` along — still
/// one wall per (segment × section curve). A requested twist can hand a side more
/// than the quarter turn a counter-twist alone never exceeds, and the rational roll
/// is written in `tan(β/2)`, so such a share is laid down as several equal rolls,
/// each zero-rate at both ends, over equal runs of the middle half. A frame with a
/// zero holonomy and a requested twist takes the twisted construction with the
/// requested twist alone. An OPEN run takes no twist.
///
/// REFUSED BY NAME, each refusal quoting its measurement:
///   - a segment with NO UNTRIMMED MIDDLE: a mitre ring reaches into the middle
///     half its share rolls over;
///   - a middle too SHORT for its share. The roll's peak rate `ω` would tilt the
///     wall, at the section's farthest reach `ρ` from the path, past the
///     transversality floor the mitre planes and the prism builder refuse a
///     grazing sweep at: `ρ·ω ≤ √(1/0.1² − 1)`, derived from
///     [`MITER_TRANSVERSALITY_FLOOR`]. The rate is bounded by `3·tan(|β|/2)/Λ`
///     over a middle of length `Λ = L/2`, which is a bound rather than a sample.
///     Because shares go by length, that is about `3·|holonomy|/perimeter` on
///     every segment, so a short segment does not roll faster than a long one; the
///     bar binds for a section reaching about the perimeter from the path. A
///     share is never pushed onto a neighbour.
///
/// MATERIAL at a joint follows from the trim and needs no rule of its own: a
/// section point on the INNER side of the bend sits on the far side of the
/// bisector plane and is folded away, one on the OUTER side is extended past the
/// joint to reach the plane. Integrated over the section the two exchange
/// `tan(θ/2)·∫ξ dA` — zero for a section centred on the path (so the solid is
/// exactly `area × Σ segment length`), and the wedge term otherwise.
///
/// FACE ORDER, which is the caller's naming contract:
/// `[segment 0's walls in INPUT-curve order, segment 1's walls, …, START cap,
/// END cap]` — and for a CLOSED run the same list without the two caps, since
/// there are none.
///
/// # What it refuses
///
/// - a COLLAPSED bend — a joint at or past [`fold_angle`];
/// - a section plane grazing the path (`Rigid` placement can draw one), the
///   same sliver the prism builder refuses;
/// - a bend too TIGHT for the section: the fold reaches back past the far end of
///   a segment, so the bisector plane cuts the section a SECOND time and the
///   inner side of the solid intersects itself. On a FRAME every segment is
///   trimmed at BOTH ends, so this is also the OVERLAP refusal — a section wider
///   across the bend than a side is long leaves that side no wall at all;
/// - (CLOSED only) a SELF-CROSSING loop: two non-adjacent segments of the frame
///   path meeting or passing through each other. The mitres are all still
///   buildable there, and the solid they build passes through itself;
/// - (a COUNTER-TWISTED frame only) a segment with no untrimmed middle, or one
///   too short for its share of the twist, as above;
/// - (a requested twist only) a twist that does not carry the section's slice onto
///   itself, quoting the nearest twists that do.
///
/// `section` must already be PLACED in world (the caller's placement mode is
/// resolved before this runs) and validated closed and planar —
/// `profile_anchor` is what does that validating, and `section_normal` is the
/// normal it published.
pub(super) fn sweep_section_mitered(
    section: &[NurbsCurve],
    section_normal: Vec3,
    path: &SweepPath,
    twist: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let plan = plan_mitre(section, section_normal, path, twist)?;
    // --- 7. TOPOLOGY. Ids are positional, so every face's boundary references
    //     the same edge records its neighbours do — which is what makes the
    //     mitre loop SHARED rather than two coincident copies sewn together.
    //
    //     A FRAME's ring index WRAPS: the wall leaving the last segment lands on
    //     ring 0 again, so it references ring 0's own vertices and edges. That is
    //     the closing mitre's shared loop, and it is shared by exactly the same
    //     mechanism every other joint's is — positional ids, not a sewing pass.
    let MitrePlan {
        closed,
        segments,
        count,
        ring_count,
        rings,
        reversed_winding,
        start_frame: (x_axis, y_axis),
        end_frame: (end_x, end_y),
        bands,
        closure,
        ..
    } = plan;
    // Where a side's corner LANDS: the same corner of the next ring, except on a
    // frame's closing side under a twist that carries each section curve `shift`
    // along, where corner `j` arrives on ring 0's corner `j + shift`.
    let shift = closure.map_or(0, |closure| closure.curve_shift);
    let landing = |segment: usize, corner: usize| {
        let next_ring = (segment + 1) % ring_count;
        if next_ring == 0 {
            (0, (corner + shift) % count)
        } else {
            (next_ring, corner)
        }
    };
    let vertex_id = |ring: usize, corner: usize| (ring * count + corner) as u64 + 1;
    let ring_edge_id = |ring: usize, corner: usize| 1000 + (ring * count + corner) as u64;
    let side_edge_id = |segment: usize, corner: usize| {
        1000 + (ring_count * count + segment * count + corner) as u64
    };

    let mut vertices = Vec::with_capacity(ring_count * count);
    let mut ring_points: Vec<Vec<Vec3>> = Vec::with_capacity(ring_count);
    for (index, curves) in rings.iter().take(ring_count).enumerate() {
        let mut points = Vec::with_capacity(count);
        for curve in curves {
            let [t0, _] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            points.push(curve.evaluate(t0).or_refuse(KernelStage::Fragment, "evaluate")?);
        }
        for (corner, point) in points.iter().enumerate() {
            vertices.push(VertexRecord {
                id: vertex_id(index, corner),
                point: *point,
            });
        }
        ring_points.push(points);
    }

    let mut edges = Vec::with_capacity((ring_count + segments) * count);
    for (index, curves) in rings.iter().take(ring_count).enumerate() {
        for (corner, curve) in curves.iter().enumerate() {
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            edges.push(EdgeRecord {
                id: ring_edge_id(index, corner),
                curve: curve.clone(),
                t0,
                t1,
                start_vertex_id: vertex_id(index, corner),
                end_vertex_id: vertex_id(index, (corner + 1) % count),
                degenerate: false,
                name: None,
            });
        }
    }
    for segment in 0..segments {
        for corner in 0..count {
            let (next_ring, arriving) = landing(segment, corner);
            // A TWISTED side runs prism, roll, prism: the section vertex's own
            // trajectory, the wall's boundary column at the curve's start.
            let curve = match &bands {
                None => make_line(
                    ring_points[segment][corner],
                    ring_points[next_ring][arriving],
                ).or_refuse(KernelStage::Fragment, "make_line")?,
                Some(bands) => twisted_side(
                    &rings[segment][corner],
                    &rings[next_ring][arriving],
                    &bands[segment],
                )?,
            };
            edges.push(EdgeRecord {
                id: side_edge_id(segment, corner),
                curve,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: vertex_id(segment, corner),
                end_vertex_id: vertex_id(next_ring, arriving),
                degenerate: false,
                name: None,
            });
        }
    }

    let mut next_id = 1_000_000_u64;
    let mut faces: Vec<FaceRecord> = Vec::with_capacity(segments * count + 2);
    for segment in 0..segments {
        let mut walls = Vec::with_capacity(count);
        for corner in 0..count {
            let (next_ring, arriving) = landing(segment, corner);
            let bottom = &rings[segment][corner];
            let top = &rings[next_ring][arriving];
            let [t0, t1] = bottom.domain().or_refuse(KernelStage::Fragment, "domain")?;
            // Both rows are projections of one section along one direction, so
            // the ruling is exactly `û`, the patch is the exact swept surface,
            // and all four boundary pcurves are parameter lines. A TWISTED side
            // is the exact rolled sweep between the same two rows on the same
            // `[0, 1]` in `v`, so its pcurves are the same parameter lines.
            let surface = match &bands {
                None => ruled_between(bottom, top)?,
                Some(bands) => twisted_wall(bottom, top, &bands[segment])?,
            };
            let coedges = vec![
                CoedgeRecord {
                    id: next_id,
                    edge_id: ring_edge_id(segment, corner),
                    forward: true,
                    pcurve: parameter_line(t0, 0.0, t1, 0.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                },
                CoedgeRecord {
                    id: next_id + 1,
                    edge_id: side_edge_id(segment, (corner + 1) % count),
                    forward: true,
                    pcurve: parameter_line(t1, 0.0, t1, 1.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                },
                CoedgeRecord {
                    id: next_id + 2,
                    edge_id: ring_edge_id(next_ring, arriving),
                    forward: false,
                    pcurve: parameter_line(t1, 1.0, t0, 1.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                },
                CoedgeRecord {
                    id: next_id + 3,
                    edge_id: side_edge_id(segment, corner),
                    forward: false,
                    pcurve: parameter_line(t0, 1.0, t0, 0.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                },
            ];
            next_id += 4;
            walls.push(FaceRecord {
                id: next_id + 1,
                surface,
                same_sense: true,
                loops: vec![LoopRecord {
                    id: next_id,
                    coedges,
                }],
                name: None,
            });
            next_id += 2;
        }
        if reversed_winding {
            // The winding normalization reversed the section loop; callers name
            // walls by INPUT-curve order, so emit each segment's walls in it.
            walls.reverse();
        }
        faces.extend(walls);
    }

    // CAPS: a planar patch over each end ring's own footprint, padded like every
    // other cap in this module. The START cap's plane frame is the section's, the
    // END cap's is that frame carried by the joint rotations — so both are
    // right-handed about their own outward normal and `same_sense` alone states
    // which way the face looks.
    let cap = |index: usize,
                   ex: Vec3,
                   ey: Vec3,
                   forward: bool,
                   next_id: &mut u64|
     -> Result<FaceRecord, KernelRefusal> {
        let curves = &rings[index];
        let anchor = ring_points[index][0];
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for curve in curves {
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            for sample in 0..=16 {
                let point = curve.evaluate(t0 + (t1 - t0) * sample as f64 / 16.0).or_refuse(KernelStage::Fragment, "evaluate")?;
                let delta = point.sub(anchor);
                min_x = min_x.min(delta.dot(ex));
                max_x = max_x.max(delta.dot(ex));
                min_y = min_y.min(delta.dot(ey));
                max_y = max_y.max(delta.dot(ey));
            }
        }
        let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
        let origin = anchor
            .add(ex.scale(min_x - padding))
            .add(ey.scale(min_y - padding));
        let surface = make_plane(
            origin,
            ex,
            ey,
            max_x - min_x + 2.0 * padding,
            max_y - min_y + 2.0 * padding,
        ).or_refuse(KernelStage::Fragment, "make_plane")?;
        let mut coedges = Vec::with_capacity(count);
        if forward {
            for (corner, curve) in curves.iter().enumerate() {
                coedges.push(CoedgeRecord {
                    id: *next_id,
                    edge_id: ring_edge_id(index, corner),
                    forward: true,
                    pcurve: curve_to_plane_parameters(curve, origin, ex, ey).or_refuse(KernelStage::Fragment, "plane_parameters")?,
                });
                *next_id += 1;
            }
        } else {
            for corner in (0..count).rev() {
                coedges.push(CoedgeRecord {
                    id: *next_id,
                    edge_id: ring_edge_id(index, corner),
                    forward: false,
                    pcurve: curve_to_plane_parameters(&curves[corner], origin, ex, ey).or_refuse(KernelStage::Fragment, "plane_parameters")?.reversed().or_refuse(KernelStage::Fragment, "reversed")?,
                });
                *next_id += 1;
            }
        }
        let loop_id = *next_id;
        *next_id += 2;
        Ok(FaceRecord {
            id: loop_id + 1,
            surface,
            same_sense: forward,
            loops: vec![LoopRecord {
                id: loop_id,
                coedges,
            }],
            name: None,
        })
    };
    // A FRAME has no ends to cap: its walls already close the surface, and a
    // capped frame would carry two faces INSIDE the material.
    if !closed {
        faces.push(cap(0, x_axis, y_axis, false, &mut next_id)?);
        faces.push(cap(segments, end_x, end_y, true, &mut next_id)?);
    }

    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: next_id,
            faces,
        }],
        // `V − E + F = 2 − 2g` over one shell: 2 for an open run's capped tube,
        // 0 for a frame, whose one hole makes it a torus.
        genus: i64::from(closed),
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!(
                "sweepSolid: the mitred {} produced invalid topology: {issues:?}",
                if closed { "frame" } else { "sweep" }
            ),
        ));
    }
    // The mitred lane assembles its own topology rather than lofting, so it
    // needs the soundness acceptance the loft exit gives every other sweep: a
    // mitre tighter than the profile's own half-width folds the tube through
    // itself and `validate()` is silent on it.
    //
    // A FRAME is the shape that most needs it. Its every side is trimmed at both
    // ends, so the fold the room check bounds in ADVANCE — over the ring curves'
    // control points, per segment — is reachable from two directions there, and
    // the closing joint is the one no open run has. The room check is a proof and
    // this is the independent reading of the same thing: a frame that gets past
    // both is one whose inner side neither computes nor measures as folded.
    crate::accept_sound(solid, "sweepSolid")
}

/// The mitre CONSTRUCTION, everything the topology is built from: the chord
/// polyline, the joint planes, the rings, the closure and — on a counter-twisted
/// frame — where each segment's share rolls. Every refusal of the lane is asked
/// here, so the builder and the closure instrument ([`mitred_frame_closure`])
/// cannot disagree about what a path builds.
fn plan_mitre(
    section: &[NurbsCurve],
    section_normal: Vec3,
    path: &SweepPath,
    requested: f64,
) -> Result<MitrePlan, KernelRefusal> {
    let tolerance = 1e-6;
    let count = section.len();
    if count < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "section_count",
            "sweepSolid: a mitred sweep needs a section of at least 2 curves",
        ));
    }
    if requested != 0.0 && !path.closed {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "open_run_twist",
            "sweepSolid: a twist on a mitred run is rolled along the sides of a CLOSED frame; \
             this path is open",
        ));
    }
    // The lane's own precondition, restated where it is relied on: the caller
    // ([`sweep_profile_along_chain_stations`]) selects this builder on exactly
    // this predicate.
    if !path.cornered_polyline() {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "lane_precondition",
            "sweepSolid: the mitre lane takes a path of STRAIGHT segments with at least one \
             corner",
        ));
    }
    let segments = path.len();
    // The ONE branch that runs through this whole builder: a CLOSED run mitres
    // its closing joint too, so it has a bisector plane at joint 0 instead of a
    // cap, one ring per joint instead of one per joint plus two, and no caps.
    let closed = path.closed;
    if closed && segments < 3 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "frame_segments",
            format!(
            "sweepSolid: a CLOSED cornered path needs at least 3 segments to be a frame; this \
             one has {segments}. Two straight segments that close on each other are the same \
             line twice"
            ),
        ));
    }
    // The RINGS the topology carries: one per joint for a frame, one per joint
    // plus the two cap loops for an open run. Ring `segments` is the closing
    // ring a frame MEASURES and does not keep — it is ring 0 — which is why the
    // projection chain below still runs `segments + 1` times either way.
    let ring_count = if closed { segments } else { segments + 1 };

    // --- 1. The path as a CHORD polyline. The joint points accumulate the
    //     chords rather than reading each segment's own endpoints, so two
    //     consecutive segments share their joint EXACTLY even when the picks
    //     that produced them meet only within the chainer's join band — the
    //     same rule `SW`'s `translate` mode uses to make its portion caps
    //     coincide.
    let mut joints: Vec<Vec3> = Vec::with_capacity(segments + 1);
    let mut dirs: Vec<Vec3> = Vec::with_capacity(segments);
    let mut lengths: Vec<f64> = Vec::with_capacity(segments);
    let [first_t0, _] = path.curves[0].domain().or_refuse(KernelStage::Classify, "domain")?;
    joints.push(path.curves[0].evaluate(first_t0).or_refuse(KernelStage::Classify, "evaluate")?);
    for index in 0..segments {
        let curve = &path.curves[index];
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        let chord = curve.evaluate(t1).or_refuse(KernelStage::Classify, "evaluate")?.sub(curve.evaluate(t0).or_refuse(KernelStage::Classify, "evaluate")?);
        let length = chord.length();
        if length <= tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "segment_length",
                format!(
                    "sweepSolid: path segment '{}' has zero length",
                    path.name(index)
                ),
            ));
        }
        dirs.push(chord.scale(1.0 / length));
        lengths.push(length);
        joints.push(joints[index].add(chord));
    }
    let travel: f64 = lengths.iter().sum();

    // --- 2. One BISECTOR PLANE per interior joint, and the COLLAPSED-BEND
    //     refusal. For unit directions `|û + v̂| = 2·cos(θ/2)`, so the
    //     transversality floor is read off the bisector's own length — no
    //     inverse trigonometry in the test, only in the message.
    //
    //     `plane[0]` is the START cap (the section's own plane) and
    //     `plane[segments]` the END cap; both are filled in below.
    //     A CLOSED run's joint 0 is a mitre too — the CLOSING one, between the
    //     last segment and the first — so its loop starts at 0 and both cap
    //     slots are filled with that same plane: `plane[segments]` IS `plane[0]`,
    //     placed at `joints[0]` rather than at the accumulated `joints[segments]`.
    //     The two are the same point for a loop that closes, and taking the first
    //     one puts the closing mitre exactly where the section starts.
    let mut plane_point: Vec<Vec3> = vec![Vec3::default(); segments + 1];
    let mut plane_normal: Vec<Vec3> = vec![Vec3::default(); segments + 1];
    let mut turns: Vec<f64> = vec![0.0; segments + 1];
    for index in usize::from(!closed)..segments {
        let before = (index + segments - 1) % segments;
        let arriving = dirs[before];
        let departing = dirs[index];
        let bisector = arriving.add(departing);
        let cos_half = bisector.length() * 0.5;
        let turn = arriving.dot(departing).clamp(-1.0, 1.0).acos();
        if cos_half < MITER_TRANSVERSALITY_FLOOR {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "collapsed_bend",
                format!(
                "sweepSolid: the joint between path segments '{}' and '{}' is a COLLAPSED bend — \
                 it turns {:.3}°, at or past the {:.3}° fold angle a mitre can carry. The mitre \
                 plane bisects the two directions, so at {:.3}° it makes only {:.2}° with the \
                 path and the section's footprint on it is stretched {:.1}× across the bend; at \
                 180° the path doubles back on itself and there is no bisector plane at all. \
                 Open the bend, or sweep the two legs separately",
                path.name(before),
                path.name(index),
                turn.to_degrees(),
                fold_angle().to_degrees(),
                turn.to_degrees(),
                (90.0 - turn.to_degrees() / 2.0),
                1.0 / cos_half.max(f64::MIN_POSITIVE),
                ),
            ));
        }
        plane_normal[index] = bisector.scale(1.0 / (2.0 * cos_half));
        plane_point[index] = joints[index];
        turns[index] = turn;
    }
    if closed {
        plane_point[segments] = plane_point[0];
        plane_normal[segments] = plane_normal[0];
        turns[segments] = turns[0];
    }

    // --- 2b. A FRAME must not cross itself — asked of a closed run only, and
    //     BEFORE anything reads whether the loop is planar: a self-crossing loop
    //     can have no plane to be judged against. A figure-of-eight's two lobes
    //     enclose opposite signed areas, so the path classification's Newell
    //     normal cancels and the run reads as SPATIAL; refused for its crossing it
    //     is named for what is wrong with it, and it never reaches the twist
    //     policy a spatial frame would otherwise get. Non-adjacent segments only —
    //     neighbours SHARE a joint, the closing joint making segment 0 and segment
    //     n−1 neighbours too, and a shared point is not a crossing. A triangle therefore has no
    //     pair to test, which is right: three segments closing a loop cannot
    //     cross.
    if closed {
        for first in 0..segments {
            for second in (first + 2)..segments {
                if first == 0 && second == segments - 1 {
                    continue;
                }
                let gap = segment_gap(
                    joints[first],
                    joints[first + 1],
                    joints[second],
                    joints[second + 1],
                );
                if gap <= tolerance * travel {
                    return Err(KernelRefusal::input(
                        KernelStage::Classify,
                        "self_crossing",
                        format!(
                        "sweepSolid: this closed path CROSSES ITSELF — segments '{}' and '{}' \
                         pass within {gap:.6e} of each other, and they do not share a joint. \
                         Every joint of it still has a mitre, so the frame would be built and \
                         would pass through itself where those two sides meet. Redraw the loop \
                         so its sides only meet at its corners",
                        path.name(first),
                        path.name(second),
                        ),
                    ));
                }
            }
        }
    }

    // --- 3. The SECTION's own plane, oriented to agree with the first segment
    //     (the prism builder's convention: the normal is flipped to point the
    //     way the sweep goes, so the start cap faces backwards and the end cap
    //     forwards).
    //
    //     The near-parallel guard is checked ONCE, on the first segment, and
    //     that covers the whole chain: the section and the direction are carried
    //     by the SAME joint rotation, so `ν̂ᵢ·ûᵢ` is the same number at every
    //     segment. A `Rigid` sweep's section keeps the angle to the path it was
    //     drawn at, all the way round the corners.
    let mut normal = section_normal.normalized().map_err(|_| {
        KernelRefusal::internal(
            KernelStage::Classify,
            "section_normal",
            "sweepSolid: the section's plane normal is degenerate",
        )
    })?;
    if normal.dot(dirs[0]) < 0.0 {
        normal = normal.scale(-1.0);
    }
    if normal.dot(dirs[0]) < MITER_TRANSVERSALITY_FLOOR {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "sliver",
            "sweepSolid: the path is nearly parallel to the section plane, which sweeps a sliver \
             rather than a solid",
        ));
    }
    let x_axis = normal.perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?;
    let y_axis = normal.cross(x_axis).normalized().or_refuse(KernelStage::Classify, "normalized")?;

    // WINDING, normalized exactly as the prism builder normalizes it: the loop
    // is made counter-clockwise about the section normal, so every wall's
    // `T × û` points OUT of the material. A reversed input loop is un-permuted
    // at the end (the caller names walls by INPUT-curve index).
    let section_origin = {
        let [t0, _] = section[0].domain().or_refuse(KernelStage::Classify, "domain")?;
        section[0].evaluate(t0).or_refuse(KernelStage::Classify, "evaluate")?
    };
    let mut ring: Vec<NurbsCurve> = section.to_vec();
    let reversed_winding = profile_area(&ring, section_origin, x_axis, y_axis).or_refuse(KernelStage::Classify, "profile_area")? < 0.0;
    if reversed_winding {
        ring = ring
            .iter()
            .rev()
            .map(NurbsCurve::reversed)
            .collect::<Result<_, _>>().or_refuse(KernelStage::Classify, "collect")?;
    }
    // On an OPEN run the section's own plane IS the start cap's plane; on a
    // CLOSED one slot 0 is already the closing joint's bisector plane, and the
    // section's plane is only what ring 0 is projected FROM.
    if !closed {
        plane_point[0] = section_origin;
        plane_normal[0] = normal;
    }

    // --- 4. The END CAP's plane: the section's plane CARRIED by the path's own
    //     motion. Under `Rigid` the section keeps its drawn offset from the path
    //     and its drawn angle to it, so the last section plane is the first one
    //     rotated by every joint rotation in turn, through the point the
    //     section's anchor has travelled to:
    //
    //         plane_n = ( Pₙ + Q·(O₀ − P₀),  Q·ν̂₀ ),   Q = Rotₙ₋₁ ∘ … ∘ Rot₁
    //
    //     which reduces to `(Pₙ, ûₙ₋₁)` for a section transplanted square to the
    //     path (`O₀ = P₀`, `ν̂₀ = û₀`), the only case where the end cap is
    //     perpendicular to the last segment.
    //     A CLOSED run has no end cap to place: slot `segments` is the closing
    //     joint's plane, filled above, and the carry that would have produced an
    //     end frame is the closure identity instead.
    let mut end_x = x_axis;
    let mut end_y = y_axis;
    if !closed {
        let mut end_normal = normal;
        let mut anchor_offset = section_origin.sub(joints[0]);
        for index in 1..segments {
            let (from, to) = (dirs[index - 1], dirs[index]);
            end_normal = rotate_minimal(end_normal, from, to);
            end_x = rotate_minimal(end_x, from, to);
            end_y = rotate_minimal(end_y, from, to);
            anchor_offset = rotate_minimal(anchor_offset, from, to);
        }
        plane_normal[segments] = end_normal;
        plane_point[segments] = joints[segments].add(anchor_offset);
    }

    // --- 5. The RINGS: each one the previous ring projected along the segment
    //     it is leaving, onto the next plane.
    //     RING 0 is the placed section itself on an OPEN run (it IS the start cap
    //     loop) and the section's own footprint on the CLOSING joint's bisector
    //     plane on a frame — projected along `d₀`, the direction the first wall
    //     is swept along, so ring 0 and ring 1 are one section projected along
    //     one direction exactly as every other consecutive pair is.
    let mut rings: Vec<Vec<NurbsCurve>> = Vec::with_capacity(segments + 1);
    if closed {
        let mut first = Vec::with_capacity(count);
        for curve in &ring {
            first.push(project_curve_along(
                curve,
                dirs[0],
                plane_point[0],
                plane_normal[0],
            )?);
        }
        rings.push(first);
    } else {
        rings.push(ring);
    }
    for index in 0..segments {
        let mut next = Vec::with_capacity(count);
        for curve in &rings[index] {
            next.push(project_curve_along(
                curve,
                dirs[index],
                plane_point[index + 1],
                plane_normal[index + 1],
            )?);
        }
        rings.push(next);
    }

    // --- 6. CLOSURE. Ring `segments` is the section carried once round the whole
    //     loop and projected back onto joint 0's plane. On a PLANAR loop the
    //     construction says it IS ring 0: the lap's rigid motion is a translation
    //     along `d₀` (see the closing-frame policy above) and `Proj_{d₀}`
    //     annihilates exactly that. So the builder KEEPS ring 0, welds the last
    //     wall to it, and reads the difference as the residual of a claimed
    //     identity rather than as a gap to be closed.
    //
    //     The floor is `tolerance · travel`, the same one the room check uses, and
    //     for the same reason: below it two rings are one ring under rounding. On
    //     the planar test frames the residual reads 0 to 1.0e-14 against floors of
    //     ~1e-4, ten orders inside the bar.
    //
    //     WHICH frames take the counter-twist is decided on the PATH, never on the
    //     section: a hole loop and a second region are swept in calls of their own,
    //     and a decision that read their sections could roll a hole where its outer
    //     wall does not. And it is decided at ROUND-OFF, not at a geometric size: the
    //     error of skipping a roll `h` is `ρ·h`, which grows with the section, so a
    //     geometric bar would weld a seam on a large enough section. What the bar
    //     separates is a holonomy that IS zero — a planar loop, or a spatial one
    //     whose indicatrix encloses a whole number of hemispheres, like the skew
    //     hexagon round a cube's edges or any loop with a mirror symmetry — from
    //     the product of the joint rotations' rounding, a few `ε` per joint. Such a
    //     loop is built exactly as a planar frame is, and its residual is still
    //     MEASURED below.
    //
    //     A REQUESTED twist rides on the counter-twist, shared the same way. It is
    //     asked first whether it CLOSES: after the lap the section comes back
    //     rolled by it about segment 0's line, and since every wall is swept along
    //     its segment, what must map onto itself is the section's slice square to
    //     that line — ring 0 projected along `d₀` onto the plane through the joint
    //     perpendicular to `d₀` (projections along one direction compose, so ring
    //     0's slice is the section's). Read on the WINDING-NORMALIZED ring, whose
    //     curve order the rings and ids use ([`closing_twist`]).
    let room_floor = tolerance * travel;
    let mut closure = None;
    let mut bands: Option<Vec<TwistBand>> = None;
    if closed {
        let holonomy = frame_holonomy(&dirs)?;
        let round_off = 64.0 * f64::EPSILON * segments as f64;
        let requested = if requested == 0.0 {
            ClosingTwist {
                twist: 0.0,
                shift: 0,
            }
        } else {
            let slice = rings[0]
                .iter()
                .map(|curve| project_curve_along(curve, dirs[0], joints[0], dirs[0]))
                .collect::<Result<Vec<_>, KernelRefusal>>()?;
            closing_twist(&slice, joints[0], dirs[0], requested, room_floor)?
        };
        if holonomy.abs() <= round_off && requested.twist == 0.0 {
            let residual = ring_residual(&rings[segments], &rings[0])?;
            if residual > room_floor {
                return Err(KernelRefusal::internal(
                    KernelStage::Classify,
                    "frame_closure",
                    format!(
                    "sweepSolid: this closed path's frame does NOT close. Carried round the loop \
                     and projected back onto the closing joint's own bisector plane, the section \
                     returns {residual:.6e} away from the ring it started as, past the \
                     {room_floor:.6e} floor — so the last mitre would not share one loop with the \
                     first and the frame would carry a seam. Its holonomy reads {:.3e}°, which is \
                     the joint rotations' own rounding ({round_off:.1e} rad), so the loop should \
                     have closed as a planar one does",
                    holonomy.to_degrees(),
                    ),
                ));
            }
            closure = Some(SweepClosure {
                holonomy,
                requested_twist: 0.0,
                applied_twist: 0.0,
                curve_shift: 0,
            });
        } else {
            // --- 6b. The COUNTER-TWIST. `−holonomy` shared by segment length, each
            //     share a roll about its segment's own line. The rings are
            //     re-projected with the rolls in: ring `i + 1` is ring `i` rolled
            //     by share `i` and projected along `dᵢ` — a roll about a line
            //     commutes with translation along it, so where on the segment the
            //     roll happens does not move the ring it lands on.
            //     A requested twist adds to it; where the holonomy is round-off (a
            //     planar or mirror-symmetric loop) the requested twist is all there is.
            let counter = if holonomy.abs() <= round_off { 0.0 } else { -holonomy };
            let twist = if requested.twist == 0.0 {
                counter
            } else {
                requested.twist + counter
            };
            let shares: Vec<f64> = lengths.iter().map(|length| twist * length / travel).collect();
            let mut twisted: Vec<Vec<NurbsCurve>> = Vec::with_capacity(segments + 1);
            twisted.push(rings[0].clone());
            for index in 0..segments {
                let mut next = Vec::with_capacity(count);
                for curve in &twisted[index] {
                    let rolled = roll_curve_about(curve, joints[index], dirs[index], shares[index])?;
                    next.push(project_curve_along(
                        &rolled,
                        dirs[index],
                        plane_point[index + 1],
                        plane_normal[index + 1],
                    )?);
                }
                twisted.push(next);
            }
            // The counter-twisted lap CLOSES — an identity again (the lap's motion
            // is the screw along `d₀` with its roll cancelled, and a requested twist
            // that is a symmetry of the slice), measured again. Under a fractional
            // turn each curve returns onto the one `shift` along.
            let landed: Vec<NurbsCurve> = (0..count)
                .map(|index| twisted[0][(index + requested.shift) % count].clone())
                .collect();
            let residual = ring_residual(&twisted[segments], &landed)?;
            if residual > room_floor {
                return Err(KernelRefusal::internal(
                    KernelStage::Classify,
                    "twisted_closure",
                    format!(
                    "sweepSolid: this closed path's frame does NOT close under its twist. Its \
                     holonomy is {:.6}°, a twist of {:.6}° (requested {:.6}°) was shared over its \
                     segments, yet the section returns {residual:.6e} from the ring it started \
                     as, past the {room_floor:.6e} floor",
                    holonomy.to_degrees(),
                    twist.to_degrees(),
                    requested.twist.to_degrees(),
                    ),
                ));
            }
            // --- 6c. Where each share ROLLS: the MIDDLE HALF of its segment,
            //     `[L/4, 3L/4]` from the joint it departs. A path-only rule, for the
            //     reason above; the section decides only whether its mitres leave
            //     that run untrimmed.
            //
            //     A counter-twist alone never shares out more than a quarter turn to
            //     a side (`|h| ≤ π`, and no side is half the perimeter). A requested
            //     twist can: a whole turn on the skew quadrilateral hands its long
            //     side 154°. The rational roll is written in `tan(β/2)`, which is
            //     infinite at a half turn and concentrates the rate well before it,
            //     so a share past a quarter turn is laid down as PIECES of at most a
            //     quarter turn each, over equal runs of the middle half, one after
            //     another. Each piece starts and ends at zero rate like the single
            //     roll, so the wall stays tangent-continuous through them, and a
            //     share under a quarter turn is one piece, exactly as before.
            let max_tilt = (1.0 / (MITER_TRANSVERSALITY_FLOOR * MITER_TRANSVERSALITY_FLOOR) - 1.0).sqrt();
            let mut planned = Vec::with_capacity(segments);
            for index in 0..segments {
                let (origin, direction, length) = (joints[index], dirs[index], lengths[index]);
                let (start, end) = (length / 4.0, 3.0 * length / 4.0);
                let share = shares[index];
                // The farthest reach of the ring BEHIND along the segment, the
                // nearest reach of the ring AHEAD, and the farthest any section
                // point sits from the segment's line. The ROLL is asked about first:
                // a section that reaches far enough from the path to tilt its wall
                // past the floor is refused for that, whatever its mitres do. Each is affine or convex in
                // the point, so the control points bound it over the curves — a
                // proof, as the room check's is.
                let mut behind = f64::NEG_INFINITY;
                let mut reach = 0.0_f64;
                for curve in &twisted[index] {
                    for control in &curve.control_points {
                        let offset = control.point().or_refuse(KernelStage::Classify, "control_point")?.sub(origin);
                        let along = offset.dot(direction);
                        behind = behind.max(along);
                        reach = reach.max(offset.sub(direction.scale(along)).length());
                    }
                }
                let mut ahead = f64::INFINITY;
                for curve in &twisted[index + 1] {
                    for control in &curve.control_points {
                        ahead = ahead.min(control.point().or_refuse(KernelStage::Classify, "control_point")?.sub(origin).dot(direction));
                    }
                }
                let pieces = (share.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.0);
                let rate = 3.0 * (share.abs() / pieces / 2.0).tan() / ((end - start) / pieces);
                if reach * rate > max_tilt {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "twist_room_short",
                        format!(
                        "{MITRED_TWIST_ROOM_REFUSAL}: path segment '{}' is too SHORT for its share of \
                         the twist. This loop's holonomy is {:.6}° and the twist requested on top of \
                         its counter-twist is {:.6}°, and the segment's {:.6}° share rolls over the \
                         middle half of its {length:.6} length in {pieces} piece(s), a peak rate of \
                         {rate:.6} rad per unit length (bounded as 3·tan(|piece|/2) over the piece's \
                         run). At the section's farthest reach from the path, {reach:.6}, that tilts \
                         the wall to ρ·ω = {:.6}, past the {max_tilt:.6} the {MITER_TRANSVERSALITY_FLOOR} \
                         transversality floor allows — the floor the mitre planes and the prism \
                         builder refuse a grazing sweep at. Lengthen the segment, narrow the section, \
                         or twist less",
                        path.name(index),
                        holonomy.to_degrees(),
                        requested.twist.to_degrees(),
                        share.to_degrees(),
                        reach * rate,
                        ),
                    ));
                }
                if behind > start - room_floor || ahead < end + room_floor {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "twist_room_middle",
                        format!(
                        "{MITRED_TWIST_ROOM_REFUSAL}: path segment '{}' has NO UNTRIMMED MIDDLE for \
                         its share of the twist. This loop comes back rolled by its holonomy, \
                         {:.6}°, with {:.6}° of twist requested on top, and the frame closes by \
                         rolling each segment through a share of the counter-twist plus that twist \
                         in proportion to its length — {:.6}° here — over the middle half of the \
                         segment, from {start:.6} to {end:.6} along its {length:.6}. The mitre ring \
                         behind reaches {behind:.6} along it and the ring ahead starts at {ahead:.6}, \
                         so a trim reaches into that run, and a roll inside a mitre's reach would \
                         pull its shared loop apart. The share is not moved onto a neighbour. \
                         Lengthen the segment, or narrow the section across its bends",
                        path.name(index),
                        holonomy.to_degrees(),
                        requested.twist.to_degrees(),
                        share.to_degrees(),
                        ),
                    ));
                }
                planned.push(TwistBand {
                    origin,
                    direction,
                    start,
                    end,
                    roll: share,
                    pieces: pieces as usize,
                });
            }
            rings = twisted;
            bands = Some(planned);
            closure = Some(SweepClosure {
                holonomy,
                requested_twist: requested.twist,
                applied_twist: twist,
                curve_shift: requested.shift,
            });
        }
    }

    // --- 6d. ROOM: every wall must still have positive extent along its segment.
    //
    //     The surviving length of the wall at a section point `y` on ring `i` is
    //
    //         ℓ(y) = (P − y)·n̂ / (ûᵢ·n̂)        (P, n̂ = the segment's END plane)
    //
    //     which is AFFINE in `y`, so its minimum over a ring curve is bounded
    //     below by its minimum over that curve's CONTROL POINTS — a NURBS curve
    //     with positive weights lies in their convex hull. This is a proof, not
    //     a sampling (the same argument the skinning builder's fold-back guard
    //     makes for its own advance), and it matters on a curved section edge:
    //     an arc's interior can pinch to nothing while both its endpoints still
    //     have room, which a sampled check can step straight over. The bound is
    //     CONSERVATIVE — an arc's control polygon bulges past the arc — so a
    //     section within a hull's width of the floor is refused rather than
    //     built, which is the safe direction.
    //
    //     `ℓ(y) ≤ 0` is the inner-side self-intersection: the bisector plane has
    //     cut the section a second time, at the far end of the segment. The
    //     floor is the builders' position tolerance scaled by the path's own
    //     length, because a wall thinner than that is a collapsed edge rather
    //     than a thin face.
    //
    //     A TWISTED frame does not take this reading: its walls run between rings
    //     that are not one section projected along one direction, and its own
    //     measurement — the untrimmed middle every share needs — is stronger (see
    //     the counter-twist block above).
    if bands.is_none() {
        for index in 0..segments {
            let denominator = dirs[index].dot(plane_normal[index + 1]);
            let mut room = f64::INFINITY;
            for curve in &rings[index] {
                for control in &curve.control_points {
                    let point = control.point().or_refuse(KernelStage::Classify, "control_point")?;
                    room = room.min(plane_point[index + 1].sub(point).dot(plane_normal[index + 1]) / denominator);
                }
            }
            if room <= room_floor {
                //     EVERY segment of a frame is bounded by two mitres, so both
                //     ends are named there; an open run's first and last segments
                //     have a cap at one end and name only the mitre.
                let mut bounds: Vec<String> = Vec::with_capacity(2);
                if closed || index > 0 {
                    bounds.push(format!(
                        "{:.3}° where it meets '{}'",
                        turns[index].to_degrees(),
                        path.name((index + segments - 1) % segments)
                    ));
                }
                if closed || index + 1 < segments {
                    bounds.push(format!(
                        "{:.3}° where it meets '{}'",
                        turns[index + 1].to_degrees(),
                        path.name((index + 1) % segments)
                    ));
                }
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "mitre_room",
                    format!(
                    "sweepSolid: the bend is too TIGHT for the section at path segment '{}'. The \
                     segment is {:.6} long and {} the section {:.6} back into it ({}), \
                     leaving {:.6e} of wall at the innermost point of the section: the bisector plane \
                     would cut the section a SECOND time, at the segment's far end, and the inner \
                     side of the bend would intersect itself.{} Lengthen the segment, open the bend, \
                     or narrow the section across the bend",
                    path.name(index),
                    lengths[index],
                    if closed { "its two mitres fold" } else { "its mitre folds" },
                    lengths[index] - room,
                    bounds.join(" and "),
                    room,
                    if closed {
                        " On a FRAME every segment is trimmed at BOTH ends, so the two mitres OVERLAP \
                         here: a section wider across the bend than a side is long leaves that side \
                         no wall at all."
                    } else {
                        ""
                    },
                    ),
                ));
            }
        }
    }

    Ok(MitrePlan {
        closed,
        segments,
        count,
        ring_count,
        rings,
        reversed_winding,
        start_frame: (x_axis, y_axis),
        end_frame: (end_x, end_y),
        bands,
        closure,
    })
}

/// The prefix of the refusals a COUNTER-TWISTED frame adds (a segment with no
/// untrimmed middle for its share, or one too short for it), so a caller can
/// classify one without matching free text.
pub(crate) const MITRED_TWIST_ROOM_REFUSAL: &str =
    "sweepSolid: a spatial frame's counter-twist has no room";

/// The ROLL a counter-twisted frame lays down on one segment: its share of the
/// twist, and the axial run it rolls over.
#[derive(Debug, Clone, Copy)]
struct TwistBand {
    /// The joint the segment departs from, on the segment's line.
    origin: Vec3,
    /// The segment's unit direction — the axis of the roll.
    direction: Vec3,
    /// Where the roll starts and ends, as distances along `direction` from
    /// `origin`. Both lie clear of the two mitre rings' reach.
    start: f64,
    end: f64,
    /// The share, right-handed about `direction`.
    roll: f64,
    /// How many equal rolls the share is laid down in, one after another over
    /// equal runs of `[start, end]`, each at most a quarter turn — 1 for any share
    /// a counter-twist alone produces.
    pieces: usize,
}

/// Everything a mitred sweep's topology is built from (see [`plan_mitre`]).
struct MitrePlan {
    closed: bool,
    segments: usize,
    count: usize,
    ring_count: usize,
    /// `segments + 1` rings: an open run's start cap, mitre and end cap loops, or
    /// a frame's joint loops followed by the RETURNED ring 0 its closure was
    /// measured on.
    rings: Vec<Vec<NurbsCurve>>,
    reversed_winding: bool,
    /// The start and end caps' in-plane axes.
    start_frame: (Vec3, Vec3),
    end_frame: (Vec3, Vec3),
    /// One per segment on a COUNTER-TWISTED frame; `None` for every other run.
    bands: Option<Vec<TwistBand>>,
    /// What a frame did to close; `None` for an open run.
    closure: Option<SweepClosure>,
}

/// The CLOSURE a mitred frame applies — the holonomy of its loop and the
/// counter-twist it laid down — read off the same plan the builder builds from,
/// so it refuses whatever the builder refuses.
pub(super) fn mitred_frame_closure(
    section: &[NurbsCurve],
    section_normal: Vec3,
    path: &SweepPath,
    twist: f64,
) -> Result<SweepClosure, KernelRefusal> {
    plan_mitre(section, section_normal, path, twist)?
        .closure
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "open_run",
                "sweepSolid: an OPEN mitred run has no lap to close",
            )
        })
}

/// The largest control-point distance between two rings of the same
/// representation — how far a lap's returned ring lands from the one it started
/// as.
fn ring_residual(returned: &[NurbsCurve], start: &[NurbsCurve]) -> Result<f64, KernelRefusal> {
    let mut residual = 0.0_f64;
    for (a_curve, b_curve) in returned.iter().zip(start) {
        for (a, b) in a_curve.control_points.iter().zip(&b_curve.control_points) {
            residual = residual.max(a.point().or_refuse(KernelStage::Classify, "control_point")?.sub(b.point().or_refuse(KernelStage::Classify, "control_point")?).length());
        }
    }
    Ok(residual)
}

/// `curve` rolled by `angle` about the line through `origin` along unit
/// `direction`, right-handed. A rotation is affine, so the image is the same
/// degree, knots and weights through the rotated control points — exact.
fn roll_curve_about(
    curve: &NurbsCurve,
    origin: Vec3,
    direction: Vec3,
    angle: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let (sin, cos) = angle.sin_cos();
    let control_points = curve
        .control_points
        .iter()
        .map(|control| {
            let offset = control.point()?.sub(origin);
            let along = direction.scale(offset.dot(direction));
            let across = offset.sub(along);
            let rolled = along
                .add(across.scale(cos))
                .add(direction.cross(across).scale(sin));
            Ok(Vec4::from_point(origin.add(rolled), control.w))
        })
        .collect::<Result<Vec<_>, String>>()
        .or_refuse(KernelStage::Fragment, "control_point")?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
        .or_refuse(KernelStage::Fragment, "curve_new")
}

/// The degree of a counter-twisted wall across its roll: `u = tan(β/2)·(3v²−2v³)`
/// is cubic, the rotation it writes is `(1−u², 2u)/(1+u²)` — degree 6 — and the
/// axial advance, linear in `v`, rides the same weight.
const TWISTED_WALL_DEGREE: usize = 7;

/// Where a counter-twisted wall's pieces meet in `v`: prism on `[0, ¼]`, the
/// roll on `[¼, ¾]` — split into `pieces` equal spans when the share is laid down
/// in several rolls — and prism on `[¾, 1]`. Each is a Bézier span, joined with
/// full multiplicity so each keeps its own exact form.
fn twisted_wall_knots(pieces: usize) -> Vec<f64> {
    let degree = TWISTED_WALL_DEGREE;
    let mut knots = Vec::with_capacity(2 * (degree + 1) + (pieces + 1) * degree);
    knots.extend(std::iter::repeat(0.0).take(degree + 1));
    knots.extend(std::iter::repeat(0.25).take(degree));
    for piece in 1..pieces {
        knots.extend(std::iter::repeat(0.25 + 0.5 * piece as f64 / pieces as f64).take(degree));
    }
    knots.extend(std::iter::repeat(0.75).take(degree));
    knots.extend(std::iter::repeat(1.0).take(degree + 1));
    knots
}

/// `n` choose `k`, as a float — only for the small Bernstein degrees here.
fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |value, index| value * (n - index) as f64 / (index + 1) as f64)
}

/// The product of two polynomials given by their Bernstein coefficients.
fn bernstein_product(a: &[f64], b: &[f64]) -> Vec<f64> {
    let (n, m) = (a.len() - 1, b.len() - 1);
    (0..=n + m)
        .map(|k| {
            let low = k.saturating_sub(m);
            let high = k.min(n);
            (low..=high)
                .map(|i| binomial(n, i) * binomial(m, k - i) * a[i] * b[k - i])
                .sum::<f64>()
                / binomial(n + m, k)
        })
        .collect()
}

/// The same polynomial written `by` degrees higher (a product with 1).
fn bernstein_elevate(a: &[f64], by: usize) -> Vec<f64> {
    bernstein_product(a, &vec![1.0; by + 1])
}

/// One COLUMN of a counter-twisted wall: the trajectory of one section control
/// point, as the `TWISTED_WALL_DEGREE + 1 + (pieces + 1)·TWISTED_WALL_DEGREE`
/// homogeneous control points of its Bézier pieces.
///
/// `bottom` is the point on the ring behind (with its section weight), `top` its
/// image on the ring ahead. The prism piece carries `bottom` along the segment to
/// the roll's first plane; each roll piece turns it about the segment's line by
/// its part of the band's share while it advances over its part of the run; the
/// last prism piece carries the rolled point to `top`. In homogeneous form a roll
/// piece is
///
/// ```text
///     w(v)·X(v) = W(v)·H + C(v)·y + S(v)·(d × y) + L(v)·d
///     W = 1 + u²,  C = 1 − u²,  S = 2u,  L = W·Λ·v,  u = tan(β/2)·(3v² − 2v³)
/// ```
///
/// with `H` the piece's hub on its first plane, `y` the point's offset from it,
/// `β` the piece's roll and `Λ` its run. Each later piece takes the weight the
/// one before it ended on, `W(1) = 1 + tan²(β/2)`, on every one of its control
/// points: a uniform weight changes neither a Bézier piece's shape nor its
/// parameterization, and it makes the shared control point agree in homogeneous
/// space.
fn twisted_column(
    bottom: Vec3,
    weight: f64,
    top: Vec3,
    band: &TwistBand,
) -> Vec<Vec4> {
    let degree = TWISTED_WALL_DEGREE;
    let TwistBand {
        origin,
        direction,
        start,
        end,
        roll,
        pieces,
    } = *band;
    let piece_roll = roll / pieces as f64;
    let piece_run = (end - start) / pieces as f64;
    let u = {
        let half = (piece_roll / 2.0).tan();
        [0.0, 0.0, half, half]
    };
    let squared = bernstein_product(&u, &u);
    let w6: Vec<f64> = squared.iter().map(|value| 1.0 + value).collect();
    let c6: Vec<f64> = squared.iter().map(|value| 1.0 - value).collect();
    let s6 = bernstein_elevate(&u.map(|value| 2.0 * value), 3);
    let w = bernstein_elevate(&w6, 1);
    let c = bernstein_elevate(&c6, 1);
    let s = bernstein_elevate(&s6, 1);
    let l = bernstein_product(&w6, &[0.0, piece_run]);

    let first = bottom.add(direction.scale(start - bottom.sub(origin).dot(direction)));
    let mut column = Vec::with_capacity((pieces + 2) * degree + 1);
    for index in 0..=degree {
        let t = index as f64 / degree as f64;
        column.push(Vec4::from_point(bottom.add(first.sub(bottom).scale(t)), weight));
    }
    // The point each roll piece starts from, and the weight it carries in.
    let mut last = first;
    let mut carried = weight;
    for piece in 0..pieces {
        let hub = origin.add(direction.scale(start + piece_run * piece as f64));
        let offset = if piece == 0 {
            last.sub(hub)
        } else {
            let offset = last.sub(hub);
            offset.sub(direction.scale(offset.dot(direction)))
        };
        let across = direction.cross(offset);
        let rolled = |index: usize| {
            hub.add(
                offset
                    .scale(c[index])
                    .add(across.scale(s[index]))
                    .add(direction.scale(l[index]))
                    .scale(1.0 / w[index]),
            )
        };
        for index in 1..=degree {
            column.push(Vec4::from_point(rolled(index), carried * w[index]));
        }
        last = rolled(degree);
        carried *= w[degree];
    }
    for index in 1..=degree {
        let t = index as f64 / degree as f64;
        column.push(Vec4::from_point(last.add(top.sub(last).scale(t)), carried));
    }
    column
}

/// A counter-twisted WALL between two rings — exact, and parameterized like
/// `ruled_between` in `u`, on `[0, 1]` in `v` (see [`twisted_column`]).
fn twisted_wall(
    bottom: &NurbsCurve,
    top: &NurbsCurve,
    band: &TwistBand,
) -> Result<NurbsSurface, KernelRefusal> {
    if bottom.degree != top.degree
        || bottom.control_points.len() != top.control_points.len()
        || bottom.knots.len() != top.knots.len()
    {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "twisted_rows",
            "twisted_wall: rows are not representation-compatible",
        ));
    }
    let grid = bottom
        .control_points
        .iter()
        .zip(&top.control_points)
        .map(|(a, b)| Ok(twisted_column(a.point()?, a.w, b.point()?, band)))
        .collect::<Result<Vec<_>, String>>()
        .or_refuse(KernelStage::Fragment, "control_point")?;
    NurbsSurface::new(
        bottom.degree,
        TWISTED_WALL_DEGREE,
        bottom.knots.clone(),
        twisted_wall_knots(band.pieces),
        grid,
    )
    .or_refuse(KernelStage::Fragment, "surface_new")
}

/// A counter-twisted SIDE EDGE: the trajectory of the section vertex at the start
/// of `bottom` — the wall's boundary column there, as a curve.
fn twisted_side(
    bottom: &NurbsCurve,
    top: &NurbsCurve,
    band: &TwistBand,
) -> Result<NurbsCurve, KernelRefusal> {
    let (first, last) = match (bottom.control_points.first(), top.control_points.first()) {
        (Some(first), Some(last)) => (first, last),
        _ => {
            return Err(KernelRefusal::internal(
                KernelStage::Fragment,
                "twisted_side",
                "twisted_side: a ring curve has no control points",
            ))
        }
    };
    NurbsCurve::new(
        TWISTED_WALL_DEGREE,
        twisted_wall_knots(band.pieces),
        twisted_column(first.point().or_refuse(KernelStage::Fragment, "control_point")?, first.w, last.point().or_refuse(KernelStage::Fragment, "control_point")?, band),
    )
    .or_refuse(KernelStage::Fragment, "curve_new")
}

/// Rotate `x` by the MINIMAL rotation carrying unit `from` onto unit `to` — the
/// rotation about `from × to`, which is the one `pathAlign` applies to the
/// section at a joint (and the one a double-reflection RMF lands on across a
/// corner step).
///
/// Rodrigues with `sin` and `cos` read straight off the cross and dot products:
/// both are already exact for unit inputs, so there is no angle to recover and
/// no inverse trigonometry to lose bits in. Collinear directions give the
/// identity; ANTIPARALLEL ones would be ambiguous, and are refused as a
/// collapsed bend before this is reached.
fn rotate_minimal(x: Vec3, from: Vec3, to: Vec3) -> Vec3 {
    let axis = from.cross(to);
    let sin = axis.length();
    if sin <= f64::EPSILON {
        return x;
    }
    let cos = from.dot(to);
    let axis = axis.scale(1.0 / sin);
    x.scale(cos)
        .add(axis.cross(x).scale(sin))
        .add(axis.scale(axis.dot(x) * (1.0 - cos)))
}

/// The HOLONOMY of a CLOSED polyline's joint frame: the angle the section is
/// rolled by about the first segment's own direction after being carried once
/// round the loop — the number the spatial refusal quotes, and the input a frame
/// closure policy would take.
///
/// The carry at joint `j` is the minimal rotation `d_{j−1} → d_j`
/// ([`rotate_minimal`]), so one lap is `M = Rot_0 ∘ Rot_{n−1} ∘ … ∘ Rot_1`. `M`
/// FIXES `d_0` — it maps `d_0 → d_1 → … → d_{n−1} → d_0` — so it is a roll about
/// it, and its angle is read by carrying one unit vector perpendicular to `d_0`
/// round the same rotations and measuring where it comes back, signed
/// right-handed about `d_0`. `atan2` rather than `acos`, so a small roll keeps its
/// sign and its precision.
///
/// ZERO for a planar loop, and zero by CONSTRUCTION rather than by cancellation:
/// every joint turns about the path plane's own normal, so the rotations commute
/// and `M`'s rotation is the rotation by the loop's total turning, `±2π`. It
/// departs from zero only as the joint axes stop being parallel, which is exactly
/// what makes a closed polyline spatial.
fn frame_holonomy(dirs: &[Vec3]) -> Result<f64, KernelRefusal> {
    let reference = dirs[0].perpendicular().map_err(|_| {
        KernelRefusal::internal(
            KernelStage::Classify,
            "holonomy_reference",
            "sweepSolid: the first path segment's direction is degenerate",
        )
    })?;
    let across = dirs[0].cross(reference);
    let mut carried = reference;
    for index in 1..dirs.len() {
        carried = rotate_minimal(carried, dirs[index - 1], dirs[index]);
    }
    carried = rotate_minimal(carried, dirs[dirs.len() - 1], dirs[0]);
    Ok(carried.dot(across).atan2(carried.dot(reference)))
}

/// The smallest distance between two CLOSED segments `[p0, p1]` and `[q0, q1]` —
/// the measurement the frame's self-crossing refusal is read from.
///
/// The squared distance is a quadratic on the parameter square `[0,1]²`, so its
/// minimum is either at the interior stationary point or on the boundary, and the
/// boundary minimum is the smallest of four POINT-against-segment distances. Both
/// are computed and the smaller kept, which is a bound rather than a search: a
/// clamped one-shot solve can over-report, and over-reporting here would let a
/// crossing through.
///
/// Two segments that genuinely cross land in the interior branch with a
/// non-degenerate denominator, so the reading there is exact.
fn segment_gap(p0: Vec3, p1: Vec3, q0: Vec3, q1: Vec3) -> f64 {
    let u = p1.sub(p0);
    let v = q1.sub(q0);
    let w = p0.sub(q0);
    let (a, b, c) = (u.dot(u), u.dot(v), v.dot(v));
    let (d, e) = (u.dot(w), v.dot(w));
    let determinant = a * c - b * b;
    let mut gap = f64::INFINITY;
    if determinant > 0.0 {
        let s = (b * e - c * d) / determinant;
        let t = (a * e - b * d) / determinant;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            gap = p0.add(u.scale(s)).sub(q0.add(v.scale(t))).length();
        }
    }
    for (point, start, direction, squared) in
        [(p0, q0, v, c), (p1, q0, v, c), (q0, p0, u, a), (q1, p0, u, a)]
    {
        let parameter = if squared > 0.0 {
            (point.sub(start).dot(direction) / squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        gap = gap.min(point.sub(start.add(direction.scale(parameter))).length());
    }
    gap
}

/// The image of `curve` under the PARALLEL PROJECTION along `direction` onto the
/// plane `(plane_point, plane_normal)`.
///
/// The projection is AFFINE, so the image is the curve of the same degree, knots
/// and weights through the projected control points — exactly, not as a fit. A
/// line mitres to a line and a circular arc to a rational ellipse arc, which is
/// why a mitre on a non-circular section costs nothing extra here.
///
/// The caller guarantees `direction·plane_normal` clears the transversality
/// floor (both at a joint, where it is `cos(θ/2)`, and at the end cap, where it
/// is the section plane's angle to the path).
fn project_curve_along(
    curve: &NurbsCurve,
    direction: Vec3,
    plane_point: Vec3,
    plane_normal: Vec3,
) -> Result<NurbsCurve, KernelRefusal> {
    let denominator = direction.dot(plane_normal);
    if denominator.abs() < MITER_TRANSVERSALITY_FLOOR {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "projection_grazing",
            "sweepSolid: a mitre plane grazes the segment it trims (the transversality floor is \
             checked before this point; reaching it means the planes and the directions \
             disagree)",
        ));
    }
    let control_points = curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point()?;
            let advance = plane_point.sub(point).dot(plane_normal) / denominator;
            Ok(Vec4::from_point(point.add(direction.scale(advance)), control.w))
        })
        .collect::<Result<Vec<_>, String>>()
        .or_refuse(KernelStage::Fragment, "control_point")?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
        .or_refuse(KernelStage::Fragment, "curve_new")
}
