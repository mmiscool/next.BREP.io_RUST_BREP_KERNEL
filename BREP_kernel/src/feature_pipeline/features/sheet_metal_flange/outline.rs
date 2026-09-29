use super::*;

// ---------------------------------------------------------------------------
// Outline-edge grafting (joint per flat, corner-aware)
// ---------------------------------------------------------------------------

/// One outline pick routed into [`graft_outline_group`].
pub(super) struct OutlinePick {
    /// Position in the feature's selection list (drives closed-corner pairing).
    pub(super) selection: usize,
    pub(super) edge_id: String,
    pub(super) bend_id: String,
    pub(super) child_id: String,
}

/// A corner treatment applied to one end of a pick's bend band.
struct CornerOp {
    /// The treated corner point: where the two inset fold lines intersect. The
    /// outline vertex shared by the two picked edges moves here; both bend
    /// segments terminate here (through a fold-line rim when a user setback
    /// pulls the band shorter).
    c: [f64; 2],
    /// The band-boundary parameter along this pick's edge (from its start).
    s_c: f64,
    /// How this pick's wall end is shaped.
    wall: WallEnd,
}

/// One pick resolved against the PRE-SURGERY outline snapshot (the outline
/// mutates as picks splice in, so all geometry derives from these).
struct ResolvedPick {
    pick: usize,
    index: usize,
    a: [f64; 2],
    b: [f64; 2],
    length: f64,
    t: [f64; 2],
    m: [f64; 2],
    s0: f64,
    s1: f64,
    corner_start: Option<CornerOp>,
    corner_end: Option<CornerOp>,
}

/// Graft every outline pick of one feature on one flat, jointly: resolve all
/// picks against a snapshot, wire corner treatments between picks on ADJACENT
/// edges, then splice descending outline index (so earlier indices stay
/// valid), check the reshaped outline stays simple, and finally cut bend-
/// relief slots at setback band ends.
pub(super) fn graft_outline_group(
    flat: &mut Flat,
    picks: &[OutlinePick],
    geometry: &FlangeParams,
) -> Result<(), String> {
    let n = flat.outline.len();
    let mut resolved: Vec<ResolvedPick> = Vec::new();
    for (pick_index, pick) in picks.iter().enumerate() {
        let index = flat
            .edges
            .iter()
            .position(|edge| edge.id == pick.edge_id)
            .ok_or_else(|| {
                format!(
                    "sheet-metal flange: edge `{}` not found on flat `{}`",
                    pick.edge_id, flat.id
                )
            })?;
        if flat.edges[index].bend.is_some() {
            return Err(format!(
                "sheet-metal flange: edge `{}` already has a flange",
                pick.edge_id
            ));
        }
        let a = flat.outline[index];
        let b = flat.outline[(index + 1) % n];
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        if !(length > GEOM_EPS) {
            return Err(format!(
                "sheet-metal flange: edge `{}` is degenerate",
                pick.edge_id
            ));
        }
        let t = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
        resolved.push(ResolvedPick {
            pick: pick_index,
            index,
            a,
            b,
            length,
            t,
            m: [-t[1], t[0]], // inward (into the material) for the CCW outline
            s0: geometry.setback_start,
            s1: length - geometry.setback_end,
            corner_start: None,
            corner_end: None,
        });
    }

    // Shared corners: picked edges directly adjacent in the outline.
    for p in 0..resolved.len() {
        let next_index = (resolved[p].index + 1) % n;
        if let Some(q) = resolved.iter().position(|r| r.index == next_index) {
            apply_corner(&mut resolved, p, q, picks, geometry)?;
        }
    }

    // Effective band spans: corner trims combine with user setbacks (the
    // longer setback wins; without a user setback the corner point rules so a
    // negative shift can EXTEND the band out to the corner intersection).
    for r in &mut resolved {
        if let Some(op) = &r.corner_start {
            r.s0 = if geometry.setback_start > GEOM_EPS {
                r.s0.max(op.s_c)
            } else {
                op.s_c
            };
        }
        if let Some(op) = &r.corner_end {
            r.s1 = if geometry.setback_end > GEOM_EPS {
                r.s1.min(op.s_c)
            } else {
                op.s_c
            };
        }
        if !(r.s1 - r.s0 > MIN_SPAN) {
            return Err(format!(
                "sheet-metal flange: setbacks/corner trims consume the {:.4}-long edge `{}`",
                r.length, picks[r.pick].edge_id
            ));
        }
    }

    let mut order: Vec<usize> = (0..resolved.len()).collect();
    order.sort_by_key(|&i| std::cmp::Reverse(resolved[i].index));
    for &i in &order {
        let r = &resolved[i];
        let pick = &picks[r.pick];
        let needs_surgery = geometry.shift.abs() > GEOM_EPS
            || r.s0 > GEOM_EPS
            || r.length - r.s1 > GEOM_EPS
            || r.corner_start.is_some()
            || r.corner_end.is_some();
        let target_index = if needs_surgery {
            split_outline_edge(flat, r, geometry.shift, n)?
        } else {
            r.index
        };
        let child = corner_child_flat(
            pick.child_id.clone(),
            geometry.leg,
            r.s1 - r.s0,
            r.corner_start.as_ref().map_or(WallEnd::Flat, |op| op.wall),
            r.corner_end.as_ref().map_or(WallEnd::Flat, |op| op.wall),
            &pick.bend_id,
        )?;
        flat.edges[target_index].bend = Some(Bend {
            id: pick.bend_id.clone(),
            angle_deg: geometry.signed_angle,
            inside_radius: geometry.inside_radius,
            k_factor: geometry.k_factor,
            child: Box::new(child),
        });
    }

    // Joint validity: the reshaped outline must stay simple. (Checked once,
    // after ALL picks — mid-sequence states are legitimately transient.)
    let treated = resolved
        .iter()
        .any(|r| r.corner_start.is_some() || r.corner_end.is_some());
    if (geometry.shift.abs() > GEOM_EPS || treated) && !outline_is_simple(&flat.outline) {
        return Err(format!(
            "sheet-metal flange: the fold-line surgery makes flat `{}`'s outline self-intersect",
            flat.id
        ));
    }

    // Bend relief at setback band ends. Corner-treated ends never get a slot —
    // the corner trim already excised that material.
    if geometry.relief != ReliefType::None {
        for r in &resolved {
            if r.corner_start.is_none() && geometry.setback_start > GEOM_EPS {
                add_relief_slot(flat, geometry, r.a, r.t, r.m, r.s0, ReliefSide::Start)?;
            }
            if r.corner_end.is_none() && geometry.setback_end > GEOM_EPS {
                add_relief_slot(flat, geometry, r.a, r.t, r.m, r.s1, ReliefSide::End)?;
            }
        }
    }
    Ok(())
}

/// Wire the corner treatment between pick `p` (edge ENDS at the shared vertex)
/// and pick `q` (edge STARTS there): validate the corner class, intersect the
/// two inset fold lines, and stamp a [`CornerOp`] on both band ends.
fn apply_corner(
    resolved: &mut [ResolvedPick],
    p: usize,
    q: usize,
    picks: &[OutlinePick],
    geometry: &FlangeParams,
) -> Result<(), String> {
    let corner = format!(
        "between `{}` and `{}`",
        picks[resolved[p].pick].edge_id, picks[resolved[q].pick].edge_id
    );
    // Over-fold walls (|angle| > 90°) tuck back under the sheet, leave their
    // own edge's sector and cross near the corner — refuse rather than emit a
    // self-intersecting union.
    if geometry.signed_angle.abs() > 90.0 + 1e-9 {
        return Err(format!(
            "sheet-metal flange: the corner {corner} folds past 90° — over-folded walls collide at a shared corner; flange the edges in separate features with setbacks instead"
        ));
    }
    let (t1, m1) = (resolved[p].t, resolved[p].m);
    let (t2, m2) = (resolved[q].t, resolved[q].m);
    let cross = t1[0] * t2[1] - t1[1] * t2[0];
    let dot = t1[0] * t2[0] + t1[1] * t2[1];
    if cross <= 1e-9 {
        return Err(format!(
            "sheet-metal flange: the corner {corner} is reflex or straight — corner treatment needs a convex corner"
        ));
    }
    if dot < -1e-9 {
        return Err(format!(
            "sheet-metal flange: the corner {corner} is acute — the bend wedges would collide (interior angle must be >= 90°)"
        ));
    }
    if geometry.corner != CornerType::Open {
        let label = match geometry.corner {
            CornerType::Closed => "closed",
            CornerType::Miter => "miter",
            CornerType::Open => unreachable!(),
        };
        if dot.abs() > 1e-9 {
            return Err(format!(
                "sheet-metal flange: cornerType `{label}` supports only perpendicular corners; the corner {corner} is not 90° (use `open`)"
            ));
        }
        if (geometry.signed_angle.abs() - 90.0).abs() > 1e-9 {
            return Err(format!(
                "sheet-metal flange: cornerType `{label}` needs a 90° fold (got {}°) — the walls only meet face-to-face at 90° (use `open`)",
                geometry.signed_angle.abs()
            ));
        }
    }

    // C = the intersection of the two fold lines (each edge shifted inward by
    // the shared d). Both bands terminate there; the corner square between the
    // fold lines and the original edges is thereby excised (d > 0), preserved
    // (d = 0) or filled (d < 0) — uniformly.
    let d = geometry.shift;
    let p1 = [resolved[p].a[0] + m1[0] * d, resolved[p].a[1] + m1[1] * d];
    let p2 = [resolved[q].a[0] + m2[0] * d, resolved[q].a[1] + m2[1] * d];
    let rhs = [p2[0] - p1[0], p2[1] - p1[1]];
    let u = (rhs[0] * t2[1] - rhs[1] * t2[0]) / cross;
    let c = [p1[0] + t1[0] * u, p1[1] + t1[1] * u];
    let s_c_q = (c[0] - p2[0]) * t2[0] + (c[1] - p2[1]) * t2[1];

    // Wall shaping. `closed`: the EARLIER selection is the through wall — it
    // extends r_out = R + t past its band, covering the corner up to the
    // neighbour's outer face plane; the other butts r_in = R against the
    // through wall's inner face. Both are d-independent at a 90° corner.
    let r_in = geometry.inside_radius;
    let r_out = geometry.inside_radius + geometry.thickness;
    let (wall_p, wall_q) = match geometry.corner {
        CornerType::Open => (WallEnd::Flat, WallEnd::Flat),
        CornerType::Miter => (WallEnd::Miter, WallEnd::Miter),
        CornerType::Closed => {
            if picks[resolved[p].pick].selection <= picks[resolved[q].pick].selection {
                (WallEnd::Extend(r_out), WallEnd::Extend(r_in))
            } else {
                (WallEnd::Extend(r_in), WallEnd::Extend(r_out))
            }
        }
    };
    resolved[p].corner_end = Some(CornerOp {
        c,
        s_c: u,
        wall: wall_p,
    });
    resolved[q].corner_start = Some(CornerOp {
        c,
        s_c: s_c_q,
        wall: wall_q,
    });
    Ok(())
}

/// Replace outline segment `index` (A→B) with the setback/inset-shifted chain
///
/// ```text
///   A →(rim0)→ Q0 →(jog0)→ Q0+m̂d →(BEND)→ Q1+m̂d →(jog1)→ Q1 →(rim1)→ B
/// ```
///
/// where `m̂` is the inward (into-the-material) normal of the CCW outline and
/// `d` the net fold-line shift. Rims/jogs collapse when zero-length; a jog
/// landing exactly on an original vertex ABSORBS into a collinear neighbour
/// (the retired "trim adjacent" rule — the neighbour's endpoint moves to
/// the shifted corner) instead of doubling back into a zero-area spike; a
/// collinear neighbour that carries a bend refuses loudly.
///
/// A corner-treated end replaces the jog/rim/absorb machinery entirely: the
/// shared vertex slot becomes the corner point `C`, the bend terminates there
/// (through a fold-line rim when a user setback pulls the band shorter), and
/// no material bridges back to the original vertex — that is exactly the
/// corner-square excision.
///
/// Returns the index of the (new) bend-carrying segment; the bend segment
/// keeps the ORIGINAL edge id, rims/jogs get `:{rim0,jog0,jog1,rim1}`
/// suffixes. All geometry derives from the snapshot in `r`.
fn split_outline_edge(
    flat: &mut Flat,
    r: &ResolvedPick,
    shift: f64,
    original_n: usize,
) -> Result<usize, String> {
    let index = r.index;
    let (a, b, length, t, m) = (r.a, r.b, r.length, r.t, r.m);
    let (s0, s1) = (r.s0, r.s1);
    let n = flat.outline.len();
    // The B vertex slot, from the ORIGINAL topology: descending-index
    // processing guarantees later splices never disturb slots <= index + 1,
    // and the wraparound pick (index = n−1, slot 0) runs first.
    let b_slot = (index + 1) % original_n;
    let q = |s: f64, d: f64| [a[0] + t[0] * s + m[0] * d, a[1] + t[1] * s + m[1] * d];
    let shifted = shift.abs() > GEOM_EPS;
    let has_rim0 = r.corner_start.is_none() && s0 > GEOM_EPS;
    let has_rim1 = r.corner_end.is_none() && length - s1 > GEOM_EPS;
    let original = flat.edges[index].id.clone();

    // Absorption: only when the jog sits at an original vertex and the
    // neighbouring segment is collinear with the jog direction. `along` =
    // neighbour direction · m̂ (±|neighbour|); the DOUBLE-BACK case (jog0
    // opposing the incoming neighbour / jog1 opposing the outgoing one) must
    // not consume the neighbour entirely. Corner-treated ends never absorb.
    let mut absorb_start = false;
    if r.corner_start.is_none() && shifted && !has_rim0 {
        let prev = (index + n - 1) % n;
        let pa = flat.outline[prev];
        let dir = [a[0] - pa[0], a[1] - pa[1]];
        let plen = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
        if plen > GEOM_EPS && (dir[0] * m[1] - dir[1] * m[0]).abs() <= 1e-9 * plen {
            if flat.edges[prev].bend.is_some() {
                return Err(format!(
                    "sheet-metal flange: the inset/offset shift on `{original}` would reshape the flanged edge `{}`",
                    flat.edges[prev].id
                ));
            }
            let along = dir[0] * m[0] + dir[1] * m[1];
            if along * shift < 0.0 && shift.abs() >= plen - GEOM_EPS {
                return Err(format!(
                    "sheet-metal flange: the inset/offset shift consumes the edge `{}` adjacent to `{original}`",
                    flat.edges[prev].id
                ));
            }
            absorb_start = true;
        }
    }
    let mut absorb_end = false;
    if r.corner_end.is_none() && shifted && !has_rim1 {
        let next = (index + 1) % n;
        let nb = flat.outline[(index + 2) % n];
        let dir = [nb[0] - b[0], nb[1] - b[1]];
        let nlen = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
        if nlen > GEOM_EPS && (dir[0] * m[1] - dir[1] * m[0]).abs() <= 1e-9 * nlen {
            if flat.edges[next].bend.is_some() {
                return Err(format!(
                    "sheet-metal flange: the inset/offset shift on `{original}` would reshape the flanged edge `{}`",
                    flat.edges[next].id
                ));
            }
            // jog1 runs shifted→original (−m̂·d), so the double-back sign flips.
            let along = dir[0] * m[0] + dir[1] * m[1];
            if along * shift > 0.0 && shift.abs() >= nlen - GEOM_EPS {
                return Err(format!(
                    "sheet-metal flange: the inset/offset shift consumes the edge `{}` adjacent to `{original}`",
                    flat.edges[next].id
                ));
            }
            absorb_end = true;
        }
    }

    // Build the replacement chain. Each entry = (segment END point, record);
    // the LAST end point lives in vertex B's slot (possibly moved), all
    // earlier end points are inserted after A.
    let plain = |suffix: &str| Edge {
        id: format!("{original}:{suffix}"),
        bend: None,
    };
    let mut chain: Vec<([f64; 2], Edge)> = Vec::new();
    if let Some(op) = &r.corner_start {
        // Corner-treated start: the shared vertex slot becomes C; a user
        // setback beyond the corner trim leaves a fold-line rim C → band start.
        flat.outline[index] = op.c;
        if s0 > op.s_c + GEOM_EPS {
            chain.push((q(s0, shift), plain("rim0")));
        }
    } else if absorb_start {
        flat.outline[index] = q(s0, shift); // A moves to the shifted corner
    } else {
        if has_rim0 {
            chain.push((q(s0, 0.0), plain("rim0")));
        }
        if shifted {
            chain.push((q(s0, shift), plain("jog0")));
        }
    }
    let bend_depth = if shifted { shift } else { 0.0 };
    let bend_end = match &r.corner_end {
        // Band ends exactly at the corner: use C verbatim (no float drift
        // against the partner edge's chain).
        Some(op) if (s1 - op.s_c).abs() <= GEOM_EPS => op.c,
        _ => q(s1, bend_depth),
    };
    chain.push((
        bend_end,
        Edge {
            id: original.clone(),
            bend: None,
        },
    ));
    if let Some(op) = &r.corner_end {
        if s1 < op.s_c - GEOM_EPS {
            chain.push((op.c, plain("rim1"))); // fold-line rim band end → C
        }
        flat.outline[b_slot] = op.c;
    } else if absorb_end {
        flat.outline[b_slot] = q(s1, shift); // B moves to the shifted corner
    } else {
        if shifted {
            chain.push((q(s1, 0.0), plain("jog1")));
        }
        if has_rim1 {
            chain.push((b, plain("rim1")));
        }
    }

    let bend_offset = chain
        .iter()
        .position(|(_, edge)| edge.id == original)
        .expect("bend segment present");
    let inserted = chain.len() - 1;
    for (i, (point, _)) in chain[..inserted].iter().enumerate() {
        flat.outline.insert(index + 1 + i, *point);
    }
    let records: Vec<Edge> = chain.into_iter().map(|(_, edge)| edge).collect();
    flat.edges.splice(index..=index, records);
    // NOTE: outline simplicity is checked ONCE by the caller after every pick
    // on the flat has spliced — mid-sequence states are legitimately transient.
    Ok(index + bend_offset)
}

/// Which band end a relief slot hangs off.
pub(super) enum ReliefSide {
    Start,
    End,
}

/// Cut one bend-relief slot into the parent flat at a band end, as an exact
/// [`Hole`] baked into the tree (it survives every re-evaluation and shows in
/// the flat pattern).
///
/// Slot frame: `s` along the (snapshot) edge direction, `depth` along the
/// into-the-material normal `m̂`, both measured from the ORIGINAL edge line.
/// The slot spans `[s_end, s_end + w]` (End side, into the trailing rim) or
/// `[s_start − w, s_start]` (Start side) — one long side always flush with
/// the bend-band end plane — and reaches from `min(0, shift) − max(t, w)`
/// (an over-cut clear of the outermost rim/jog material, so the boolean is a
/// clean clearance cut) down to `shift + reliefDepth` (`reliefDepth` measured
/// PAST the fold line into the web; its default is the bend allowance, so the
/// slot spans the developed bend zone in the flat pattern).
///
/// Shapes:
/// * `rectangular` — 4 lines, square deep end.
/// * `obround` — a stadium (2 lines + 2 semicircle arcs, exact): the outer cap
///   is buried in the over-cut zone, the deep cap rounds the web end with its
///   apex exactly at `shift + reliefDepth`.
/// * `tear` — a triangular notch whose flush side lies in the band-end plane,
///   widening to `w` at the over-cut end: the manifold stand-in for a
///   zero-width tear (which an exact BREP cannot represent).
pub(super) fn add_relief_slot(
    flat: &mut Flat,
    geometry: &FlangeParams,
    a: [f64; 2],
    t: [f64; 2],
    m: [f64; 2],
    s_boundary: f64,
    side: ReliefSide,
) -> Result<(), String> {
    use std::f64::consts::{PI, TAU};
    // Normalize to a right-handed (t̂, m̂) frame: hole rims hand in a LEFT-
    // handed pair when their loop winds CCW; flipping the s-axis (boundary and
    // side with it) keeps the emitted loop CCW on the flat while the flush
    // side stays on the same physical band-end plane.
    let handed = t[0] * m[1] - t[1] * m[0];
    let (t, s_boundary, side) = if handed < 0.0 {
        let flipped = match side {
            ReliefSide::Start => ReliefSide::End,
            ReliefSide::End => ReliefSide::Start,
        };
        ([-t[0], -t[1]], -s_boundary, flipped)
    } else {
        (t, s_boundary, side)
    };
    let w = geometry.relief_width;
    let outer = geometry.shift.min(0.0) - geometry.thickness.max(w);
    let deep = geometry.shift + geometry.relief_depth;
    if !(deep - outer > GEOM_EPS) {
        return Err(format!(
            "sheet-metal flange: reliefDepth {:.4} leaves no slot",
            geometry.relief_depth
        ));
    }
    // (s, depth) → flat-local 3D at z = 0. (t̂, m̂) is right-handed, so a loop
    // CCW in slot coordinates stays CCW on the flat.
    let at = |s: f64, depth: f64| {
        Vec3::new(
            a[0] + t[0] * s + m[0] * depth,
            a[1] + t[1] * s + m[1] * depth,
            0.0,
        )
    };
    let (s_lo, s_hi, flush) = match side {
        ReliefSide::End => (s_boundary, s_boundary + w, s_boundary),
        ReliefSide::Start => (s_boundary - w, s_boundary, s_boundary),
    };
    let loop_curves: Vec<NurbsCurve> = match geometry.relief {
        ReliefType::None => return Ok(()),
        ReliefType::Rectangular => vec![
            make_line(at(s_lo, outer), at(s_hi, outer))?,
            make_line(at(s_hi, outer), at(s_hi, deep))?,
            make_line(at(s_hi, deep), at(s_lo, deep))?,
            make_line(at(s_lo, deep), at(s_lo, outer))?,
        ],
        ReliefType::Obround => {
            let r = w * 0.5;
            let s_mid = (s_lo + s_hi) * 0.5;
            let ax = Vec3::new(t[0], t[1], 0.0);
            let ay = Vec3::new(m[0], m[1], 0.0);
            vec![
                // Outer cap: s_lo → s_hi through (s_mid, outer).
                make_arc(at(s_mid, outer + r), ax, ay, r, PI, TAU)?,
                make_line(at(s_hi, outer + r), at(s_hi, deep - r))?,
                // Deep cap: s_hi → s_lo through the apex (s_mid, deep).
                make_arc(at(s_mid, deep - r), ax, ay, r, 0.0, PI)?,
                make_line(at(s_lo, deep - r), at(s_lo, outer + r))?,
            ]
        }
        ReliefType::Tear => {
            let apex = at(flush, deep);
            let (c0, c1) = (at(s_lo, outer), at(s_hi, outer));
            vec![
                make_line(c0, c1)?,
                make_line(c1, apex)?,
                make_line(apex, c0)?,
            ]
        }
    };
    flat.holes.push(Hole::through(loop_curves));
    Ok(())
}

/// O(n²) simple-polygon check: no two non-adjacent segments intersect.
fn outline_is_simple(outline: &[[f64; 2]]) -> bool {
    let n = outline.len();
    for i in 0..n {
        for j in (i + 1)..n {
            if j == i || (j + 1) % n == i || (i + 1) % n == j {
                continue; // adjacent segments share a vertex by construction
            }
            let (a1, a2) = (outline[i], outline[(i + 1) % n]);
            let (b1, b2) = (outline[j], outline[(j + 1) % n]);
            if segments_intersect(a1, a2, b1, b2) {
                return false;
            }
        }
    }
    true
}

/// Segment-segment intersection (proper crossing, or an endpoint on the other
/// segment / collinear overlap), with a small absolute epsilon.
fn segments_intersect(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> bool {
    let orient = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let (d1, d2) = (orient(p3, p4, p1), orient(p3, p4, p2));
    let (d3, d4) = (orient(p1, p2, p3), orient(p1, p2, p4));
    let eps = GEOM_EPS;
    if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
    {
        return true;
    }
    let on_segment = |a: [f64; 2], b: [f64; 2], c: [f64; 2], cross: f64| {
        cross.abs() <= eps
            && c[0] >= a[0].min(b[0]) - eps
            && c[0] <= a[0].max(b[0]) + eps
            && c[1] >= a[1].min(b[1]) - eps
            && c[1] <= a[1].max(b[1]) + eps
    };
    on_segment(p3, p4, p1, d1)
        || on_segment(p3, p4, p2, d2)
        || on_segment(p1, p2, p3, d3)
        || on_segment(p1, p2, p4, d4)
}
