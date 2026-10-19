use super::*;

pub(in crate::watertight_tessellation) fn triangle_area(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) * 0.5
}

pub(super) fn point_in_triangle(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let d1 = triangle_area(p, a, b);
    let d2 = triangle_area(p, b, c);
    let d3 = triangle_area(p, c, a);
    let has_negative = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_positive = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_negative && has_positive)
}

/// Proper (interior-to-interior) segment crossing with an area tolerance —
/// touching endpoints and collinear overlaps do not count.
pub(in crate::watertight_tessellation) fn segments_properly_cross(
    p: [f64; 2],
    q: [f64; 2],
    r: [f64; 2],
    s: [f64; 2],
    epsilon: f64,
) -> bool {
    let d1 = triangle_area(r, s, p);
    let d2 = triangle_area(r, s, q);
    let d3 = triangle_area(p, q, r);
    let d4 = triangle_area(p, q, s);
    ((d1 > epsilon && d2 < -epsilon) || (d1 < -epsilon && d2 > epsilon))
        && ((d3 > epsilon && d4 < -epsilon) || (d3 < -epsilon && d4 > epsilon))
}

/// Crossing-parity point-in-ring test over the CURRENT remainder ring
/// (`indices` order).  Horizontal ray toward +u; near-degenerate rings answer
/// false for points on their zero-width corridors, which is exactly what the
/// split-diagonal validity gate wants.
fn point_in_ring(vertices: &[FaceVertex], indices: &[usize], p: [f64; 2]) -> bool {
    crossing_parity_inside(indices.len(), |offset| vertices[indices[offset]].uv, p)
}

/// Crossing parity of a horizontal +u ray against a closed uv chain of `count`
/// vertices addressed by `uv`.  Shared by the ring and whole-polygon forms.
pub(super) fn crossing_parity_inside(count: usize, uv: impl Fn(usize) -> [f64; 2], p: [f64; 2]) -> bool {
    let mut inside = false;
    for offset in 0..count {
        let a = uv(offset);
        let b = uv((offset + 1) % count);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let t = (p[1] - a[1]) / (b[1] - a[1]);
            let x = a[0] + t * (b[0] - a[0]);
            if x > p[0] {
                inside = !inside;
            }
        }
    }
    inside
}

/// Find a VALID splitting diagonal of a stalled ear-clip ring: two
/// non-adjacent ring vertices whose connecting segment properly crosses no
/// ring edge and whose midpoint lies strictly inside the ring.  Preference
/// goes to the most balanced split (then the longest), so recursion depth
/// stays logarithmic.  Returns ring OFFSETS (not vertex ids).
pub(in crate::watertight_tessellation) fn find_split_diagonal(
    vertices: &[FaceVertex],
    indices: &[usize],
    epsilon: f64,
    region: &RegionGuard,
) -> Option<(usize, usize)> {
    let count = indices.len();
    let mut best: Option<(usize, usize, usize, f64)> = None; // (oa, ob, balance, len2)
    for oa in 0..count {
        for ob in (oa + 2)..count {
            if oa == 0 && ob == count - 1 {
                continue; // ring-adjacent around the seam
            }
            let a = vertices[indices[oa]].uv;
            let b = vertices[indices[ob]].uv;
            let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
            if len2 <= epsilon {
                continue;
            }
            let balance = (ob - oa).min(count - (ob - oa));
            if let Some((_, _, best_balance, best_len2)) = best {
                if balance < best_balance || (balance == best_balance && len2 <= best_len2) {
                    continue; // cannot beat the current best; skip the O(n) check
                }
            }
            let mut valid = true;
            for e in 0..count {
                let e1 = (e + 1) % count;
                if e == oa || e == ob || e1 == oa || e1 == ob {
                    continue;
                }
                if segments_properly_cross(
                    a,
                    b,
                    vertices[indices[e]].uv,
                    vertices[indices[e1]].uv,
                    epsilon,
                ) {
                    valid = false;
                    break;
                }
            }
            if !valid {
                continue;
            }
            // The diagonal must also be valid against the ORIGINAL region:
            // a stalled ring can already skirt a consumed hole void, and a
            // ring-local check happily splits straight across it — every
            // sub-ring then inherits the violation.
            if !region.diagonal_ok(indices[oa], indices[ob], epsilon) {
                continue;
            }
            let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
            if !point_in_ring(vertices, indices, mid) {
                continue;
            }
            best = Some((oa, ob, balance, len2));
        }
    }
    best.map(|(oa, ob, _, _)| (oa, ob))
}

/// Ear-clip a simple CCW polygon; returns index triples into `vertices`.
/// Region context for ear validity: the ORIGINAL bridged polygon. Stall
/// splits recurse on sub-rings that no longer contain the hole chains, so
/// ring-local tests cannot see hole voids; every candidate ear is therefore
/// validated against the original region — centroid inside, and the ear's
/// diagonal crossing no original edge (boxy frame face: a mid-game ear
/// diagonal sliced across the rounded hole after its chain was consumed and
/// every later remainder inherited the violation).
pub(in crate::watertight_tessellation) struct RegionGuard<'a> {
    pub(in crate::watertight_tessellation) vertices: &'a [FaceVertex],
    pub(in crate::watertight_tessellation) ring: &'a [usize],
    /// For a multi-period unwrapped ribbon, never recover from an ear-clip
    /// stall by cutting across many turns. Such a diagonal is valid in UV but
    /// its 3D chord crosses the cylinder; adaptive refinement then avalanches
    /// into hundreds of thousands of triangles. The periodic axis and maximum
    /// local span are supplied only for that narrowly-gated path.
    pub(in crate::watertight_tessellation) max_split_span: Option<(usize, f64)>,
}

impl RegionGuard<'_> {
    fn diagonal_ok(&self, from: usize, to: usize, epsilon: f64) -> bool {
        let a = self.vertices[from].uv;
        let b = self.vertices[to].uv;
        if let Some((axis, maximum)) = self.max_split_span {
            if (b[axis] - a[axis]).abs() > maximum {
                return false;
            }
        }
        let midpoint = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        if !point_in_ring(self.vertices, self.ring, midpoint) {
            return false;
        }
        let count = self.ring.len();
        for offset in 0..count {
            let e0 = self.ring[offset];
            let e1 = self.ring[(offset + 1) % count];
            if e0 == from || e0 == to || e1 == from || e1 == to {
                continue;
            }
            if segments_properly_cross(a, b, self.vertices[e0].uv, self.vertices[e1].uv, epsilon) {
                return false;
            }
        }
        true
    }

    fn ear_ok(&self, previous: usize, current: usize, next: usize, epsilon: f64) -> bool {
        let a = self.vertices[previous].uv;
        let b = self.vertices[current].uv;
        let c = self.vertices[next].uv;
        let centroid = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0];
        if triangle_area(a, b, c).abs() > epsilon.sqrt().max(1e-12)
            && !point_in_ring(self.vertices, self.ring, centroid)
        {
            return false;
        }
        let count = self.ring.len();
        for offset in 0..count {
            let e0 = self.ring[offset];
            let e1 = self.ring[(offset + 1) % count];
            if e0 == previous
                || e0 == current
                || e0 == next
                || e1 == previous
                || e1 == current
                || e1 == next
            {
                continue;
            }
            let p = self.vertices[e0].uv;
            let q = self.vertices[e1].uv;
            if segments_properly_cross(a, c, p, q, epsilon)
                || segments_properly_cross(a, b, p, q, epsilon)
                || segments_properly_cross(b, c, p, q, epsilon)
            {
                return false;
            }
        }
        true
    }
}

pub(in crate::watertight_tessellation) fn ear_clip(
    vertices: &[FaceVertex],
    indices: &mut Vec<usize>,
    region: &RegionGuard,
) -> Vec<[usize; 3]> {
    let mut triangles = Vec::new();
    let epsilon = {
        let total: f64 = indices
            .windows(2)
            .map(|pair| {
                let a = vertices[pair[0]].uv;
                let b = vertices[pair[1]].uv;
                ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt()
            })
            .sum();
        (total / indices.len().max(1) as f64).powi(2) * 1e-12
    };
    let mut guard = 0usize;
    while indices.len() > 3 {
        let count = indices.len();
        let mut clipped = false;
        for offset in 0..count {
            let previous = indices[(offset + count - 1) % count];
            let current = indices[offset];
            let next = indices[(offset + 1) % count];
            let a = vertices[previous].uv;
            let b = vertices[current].uv;
            let c = vertices[next].uv;
            if triangle_area(a, b, c) <= epsilon {
                continue;
            }
            let mut contains_other = false;
            for &other in indices.iter() {
                if other == previous || other == current || other == next {
                    continue;
                }
                let p = vertices[other].uv;
                // Bridged duplicates share coordinates with corners; those
                // coincident points must not block the ear.
                if (p[0] - a[0]).abs() + (p[1] - a[1]).abs() <= 1e-14
                    || (p[0] - b[0]).abs() + (p[1] - b[1]).abs() <= 1e-14
                    || (p[0] - c[0]).abs() + (p[1] - c[1]).abs() <= 1e-14
                {
                    continue;
                }
                if point_in_triangle(p, a, b, c) {
                    contains_other = true;
                    break;
                }
            }
            if contains_other {
                continue;
            }
            if !region.ear_ok(previous, current, next, epsilon) {
                continue;
            }
            triangles.push([previous, current, next]);
            indices.remove(offset);
            clipped = true;
            break;
        }
        if !clipped {
            guard += 1;
            if std::env::var("BREP_DEBUG_TESS").is_ok() {
                eprintln!(
                    "ear_clip stall #{guard}: {} vertices remain: {:?}",
                    indices.len(),
                    indices
                        .iter()
                        .map(|&i| (vertices[i].uv[0], vertices[i].uv[1]))
                        .collect::<Vec<_>>()
                );
            }
            // A stalled ring is usually pinched by near-collinear rows (e.g.
            // an outer-arc sample 1e-13 in v from a hole edge): every convex
            // ear is either near-zero area or "contains" an on-edge vertex.
            // Split the ring on a VALIDATED interior diagonal and recurse —
            // region-respecting by construction, unlike the fan fallback,
            // which sprays triangles across concavities (problemInbox
            // 2026-08-04: junk triangles on a merged planar face).
            if let Some((oa, ob)) = find_split_diagonal(&vertices, indices, epsilon, region) {
                let mut first: Vec<usize> = indices[oa..=ob].to_vec();
                let mut second: Vec<usize> = indices[ob..].to_vec();
                second.extend_from_slice(&indices[..=oa]);
                indices.clear();
                triangles.extend(ear_clip(vertices, &mut first, region));
                triangles.extend(ear_clip(vertices, &mut second, region));
                return triangles;
            }
            if guard > 2 {
                // Fully degenerate remainder (zero-area corridors, collinear
                // chains — no interior diagonal exists): fan it and accept;
                // the emission filter drops coincident-position triangles
                // parity-safely.
                for offset in 1..indices.len() - 1 {
                    triangles.push([indices[0], indices[offset], indices[offset + 1]]);
                }
                indices.clear();
                return triangles;
            }
            // Relax: clip the flattest vertex as a (near-)zero-area ear.  The
            // triangle MUST be emitted, not silently dropped — its two
            // boundary chords otherwise lose their only triangle and the
            // mesh opens exactly there (collinear circle samples on a
            // rectangle side, e.g. a welded closed circle on a cylinder
            // wall).  If two of its positions coincide, the emission filter
            // discards it parity-safely later.
            let flattest = (0..indices.len())
                .min_by(|&a_offset, &b_offset| {
                    let flatness = |offset: usize| {
                        let count = indices.len();
                        let a = vertices[indices[(offset + count - 1) % count]].uv;
                        let b = vertices[indices[offset]].uv;
                        let c = vertices[indices[(offset + 1) % count]].uv;
                        triangle_area(a, b, c).abs()
                    };
                    flatness(a_offset).total_cmp(&flatness(b_offset))
                })
                .unwrap_or(0);
            let count = indices.len();
            triangles.push([
                indices[(flattest + count - 1) % count],
                indices[flattest],
                indices[(flattest + 1) % count],
            ]);
            indices.remove(flattest);
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    triangles
}

/// Split interior edges whose 3D chord midpoint deviates from the surface by
/// more than the tolerance.  True trim-boundary segments — consecutive INDEX
/// pairs of the bridged polygon — are never split, so the shared edge
/// sampling stays authoritative and the mesh stays crack-free.  Boundary
/// identity must be by index, not position: on a seam, a cross-face diagonal
/// can connect the two seam copies and alias a boundary segment's positions
/// while being a genuine (and badly sagging) interior chord.
/// Lawson flips on interior edges using the intrinsic (3D) Delaunay angle
/// criterion — parameterization-independent, so anisotropic uv domains flip
/// correctly. Boundary segments are never flipped, and a flip is applied
/// only when both replacement triangles stay positively oriented in uv, so
/// the mesh remains a valid triangulation of the same polygon.
pub(super) fn lawson_flips(
    vertices: &[FaceVertex],
    triangles: &mut [[usize; 3]],
    boundary_pairs: &HashSet<(usize, usize)>,
    scale: [f64; 2],
) -> bool {
    lawson_flips_guarded(vertices, triangles, boundary_pairs, scale, None)
}

/// Lawson flips with an optional ACCURACY guard.
///
/// A Delaunay flip is a QUALITY move judged in scaled uv; it is blind to the 3D
/// chord. The other diagonal of a quad can be markedly longer on the surface, so
/// on a curved face a flip can hand back an edge that sags past the chord
/// tolerance — which `refine_interior` then splits, which reshapes the quad,
/// which the next flip round re-flips. That oscillation, not the geometry, drove
/// most of the interior refinement on trimmed fillet bands (ABC 00000039: the
/// interleaved loop converged at 3.3x the seed density, the guarded one at
/// 1.15x). With the guard a flip is refused when it would BOTH exceed the chord
/// tolerance AND sag more than the diagonal it replaces: quality moves that are
/// free, or that improve the chord, still happen; only the accuracy-destroying
/// ones are declined. `None` keeps the historical unguarded behavior, which is
/// what the fan-breaking passes want (there every edge may exceed tolerance and
/// the fan must be broken regardless).
pub(super) fn lawson_flips_guarded(
    vertices: &[FaceVertex],
    triangles: &mut [[usize; 3]],
    boundary_pairs: &HashSet<(usize, usize)>,
    scale: [f64; 2],
    chord_guard: Option<(&FaceRecord, f64, bool)>,
) -> bool {
    let mut memo = FaceMemo::default();
    lawson_flips_guarded_with_memo(vertices, triangles, boundary_pairs, scale, chord_guard, &mut memo)
}

/// [`lawson_flips_guarded`] reading and filling the caller's [`FaceMemo`], so
/// a midpoint the refinement already evaluated is not evaluated again for the
/// guard, nor across the flipper's own calls.
pub(super) fn lawson_flips_guarded_with_memo(
    vertices: &[FaceVertex],
    triangles: &mut [[usize; 3]],
    boundary_pairs: &HashSet<(usize, usize)>,
    scale: [f64; 2],
    chord_guard: Option<(&FaceRecord, f64, bool)>,
    memo: &mut FaceMemo,
) -> bool {
    lawson_flips_incremental(
        vertices, triangles, boundary_pairs, scale, chord_guard, memo,
    )
}

/// Surface readings that are pure functions of a face's vertices, which never
/// move once placed: the surface point at an edge's uv midpoint (keyed by the
/// edge's endpoint indices in either order — the midpoint is symmetric) and
/// the unit normal at a vertex. `refine_interior` evaluates every interior
/// edge's midpoint on every pass and the guarded flipper evaluates it again
/// for every flip candidate; a bore wall asked the cylinder 8 400 times for
/// what are about 1 500 distinct points. Each consumer still forms its own
/// derived number (the chord deflection, the normal dot) from the stored
/// reading exactly as it did, so the decisions are bit for bit the old ones.
#[derive(Default)]
pub(in crate::watertight_tessellation) struct FaceMemo {
    midpoints: FxHashMap<(usize, usize), Vec3>,
    normals: FxHashMap<usize, Option<Vec3>>,
}

impl FaceMemo {
    /// The surface at the uv midpoint of edge `(a, b)`, evaluated once; an
    /// evaluation error is returned every time and never stored.
    pub(in crate::watertight_tessellation) fn midpoint(
        &mut self,
        face: &FaceRecord,
        vertices: &[FaceVertex],
        a: usize,
        b: usize,
        periodic: bool,
    ) -> Result<Vec3, String> {
        let key = (a.min(b), a.max(b));
        if let Some(&point) = self.midpoints.get(&key) {
            return Ok(point);
        }
        let uv = [
            (vertices[a].uv[0] + vertices[b].uv[0]) * 0.5,
            (vertices[a].uv[1] + vertices[b].uv[1]) * 0.5,
        ];
        let point = if periodic {
            face.surface.evaluate_extended(uv[0], uv[1])?
        } else {
            face.surface.evaluate(uv[0], uv[1])?
        };
        self.midpoints.insert(key, point);
        Ok(point)
    }

    /// The unit normal at vertex `index` (`Su × Sv` normalised), `None` where
    /// it cannot be evaluated or is degenerate; evaluated once.
    pub(in crate::watertight_tessellation) fn normal(
        &mut self,
        face: &FaceRecord,
        vertices: &[FaceVertex],
        index: usize,
        periodic: bool,
    ) -> Option<Vec3> {
        if let Some(&normal) = self.normals.get(&index) {
            return normal;
        }
        let uv = vertices[index].uv;
        let normal = (|| {
            let (_, su, sv) = if periodic {
                face.surface.deriv1_extended(uv[0], uv[1]).ok()?
            } else {
                face.surface.deriv1(uv[0], uv[1]).ok()?
            };
            su.cross(sv).normalized().ok()
        })();
        self.normals.insert(index, normal);
        normal
    }
}

/// The triangles that use one edge, kept in ascending index order: two for an
/// interior edge, one for a boundary edge, more only for repeated corners of
/// degenerate triangles. Inline up to three so the owner table allocates
/// nothing per edge; the spill holds the rest in the same order.
#[derive(Default)]
struct Owners {
    count: usize,
    inline: [usize; 3],
    spill: Vec<usize>,
}

impl Owners {
    fn as_slice(&self) -> &[usize] {
        if self.spill.is_empty() {
            &self.inline[..self.count]
        } else {
            &self.spill
        }
    }

    fn len(&self) -> usize {
        self.as_slice().len()
    }

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn spill_out(&mut self) {
        if self.spill.is_empty() {
            self.spill = self.inline[..self.count].to_vec();
        }
    }

    fn push(&mut self, index: usize) {
        if !self.spill.is_empty() {
            self.spill.push(index);
        } else if self.count < self.inline.len() {
            self.inline[self.count] = index;
            self.count += 1;
        } else {
            self.spill_out();
            self.spill.push(index);
        }
    }

    fn insert(&mut self, offset: usize, index: usize) {
        if self.spill.is_empty() && self.count < self.inline.len() {
            self.inline.copy_within(offset..self.count, offset + 1);
            self.inline[offset] = index;
            self.count += 1;
        } else {
            self.spill_out();
            self.spill.insert(offset, index);
        }
    }

    fn remove(&mut self, offset: usize) {
        if self.spill.is_empty() {
            self.inline.copy_within(offset + 1..self.count, offset);
            self.count -= 1;
        } else {
            self.spill.remove(offset);
            if self.spill.is_empty() {
                self.count = 0;
            }
        }
    }
}

// One accepted flip removes (a,b), inserts (c,d), and transfers the owners
// of (b,c) and (a,d). The other two edges keep their owners. Applying these
// deltas after the scan yields exactly the rebuilt sorted owner multiset,
// including repeated corners and any pre-existing coincident edge owners.
fn apply_flip_patch(
    edge_owners: &mut FxHashMap<(usize, usize), Owners>,
    first: usize,
    second: usize,
    [a, b, c, d]: [usize; 4],
) {
    let key = |a: usize, b: usize| (a.min(b), a.max(b));
    let old = key(a, b);
    let empty = {
        let owners = edge_owners.get_mut(&old).expect("old diagonal owners");
        for index in [first, second] {
            let offset = owners.as_slice().binary_search(&index).expect("old diagonal owner index");
            owners.remove(offset);
        }
        owners.is_empty()
    };
    if empty { edge_owners.remove(&old); }
    let owners = edge_owners.entry(key(c, d)).or_default();
    for index in [first, second] {
        let offset = owners.as_slice().binary_search(&index).unwrap_or_else(|i| i);
        owners.insert(offset, index);
    }
    for (edge, from, to) in [(key(b, c), first, second), (key(a, d), second, first)] {
        let owners = edge_owners.get_mut(&edge).expect("transferred edge owners");
        let offset = owners.as_slice().binary_search(&from).expect("transferred owner index");
        owners.remove(offset);
        let offset = owners.as_slice().binary_search(&to).unwrap_or_else(|i| i);
        owners.insert(offset, to);
    }
}

// Angles belong to a triangle's CURRENT vertices, not an index across writes.
// One call owns the immutable vertices and its slots; there is no cross-call
// identity cache. Option stores even NaNs/signed zero without reclassification.
#[derive(Clone, Copy, Default)]
struct TriangleCornerAngles([Option<f64>; 3]);

impl TriangleCornerAngles {
    fn angle(&mut self, triangle: [usize; 3], apex: usize, compute: impl FnOnce() -> f64) -> f64 {
        let corner = triangle.iter().position(|&vertex| vertex == apex).expect("opposite corner in triangle");
        if let Some(value) = self.0[corner] {
            return value;
        }
        let value = compute();
        self.0[corner] = Some(value);
        value
    }

    fn clear(&mut self) {
        self.0 = [None; 3];
    }
}

fn lawson_flips_incremental(
    vertices: &[FaceVertex],
    triangles: &mut [[usize; 3]],
    boundary_pairs: &HashSet<(usize, usize)>,
    scale: [f64; 2],
    chord_guard: Option<(&FaceRecord, f64, bool)>,
    memo: &mut FaceMemo,
) -> bool {
    // Classic planar Delaunay flipping, but in SCALED uv — the intrinsic
    // (unrolled) metric of the face. In 3D chordal space a boundary fan on a
    // curved strip cannot reach the ladder without passing through folded
    // states, while flat-metric Lawson flipping provably converges to the
    // Delaunay triangulation from any start.
    let scaled = |index: usize| -> [f64; 2] {
        [
            vertices[index].uv[0] * scale[0],
            vertices[index].uv[1] * scale[1],
        ]
    };
    let corner_angle = |apex: usize, first: usize, second: usize| -> f64 {
        let p = scaled(apex);
        let a = scaled(first);
        let b = scaled(second);
        let u = [a[0] - p[0], a[1] - p[1]];
        let v = [b[0] - p[0], b[1] - p[1]];
        (u[0] * v[1] - u[1] * v[0])
            .abs()
            .atan2(u[0] * v[0] + u[1] * v[1])
    };
    let uv_area = |a: usize, b: usize, c: usize| -> f64 {
        triangle_area(vertices[a].uv, vertices[b].uv, vertices[c].uv)
    };
    // Fan-to-ladder conversion flips one triangle per round along a chain, so
    // convergence needs on the order of the strip length in rounds; the cap only
    // guards against cocircular flip cycling.
    //
    // That cap used to be a flat 512, which threw away the `triangles.len()`
    // budget it is written against on exactly the meshes that need it. The
    // reshaping wave advances one rung per round, so a strip sampled at more
    // than ~500 stations along its length never reaches the ladder: on the
    // 2026-08 onshape-unclamped-periodic wall a 2046-triangle ear-clip fan was
    // still flipping 464 edges per round when round 512 cut it off, and
    // `refine_interior` then spent all twelve of its passes bisecting the
    // surviving fan diagonals — 501518 triangles for a face that needs 2258.
    // `MAX_FLIP_ROUNDS` keeps a hard ceiling so a cocircular cycle still
    // terminates; 2048 clears the measured convergence horizon of that face
    // (~1000 rounds) with room to spare while staying well short of the point
    // where a large mesh would pay for rounds it does not need.
    const MAX_FLIP_ROUNDS: usize = 2048;
    let maximum_rounds = (triangles.len() + 16).min(MAX_FLIP_ROUNDS);
    // The guard's chord sag is a pure function of an edge's two endpoints, and
    // vertices never move inside one call, so memoize it: a candidate edge the
    // guard rejects is re-offered every round, and re-evaluating the surface for
    // it each time dominated the cost of the deeper round budget above.
    let mut sag_memo: FxHashMap<(usize, usize), f64> = FxHashMap::default();
    // Each current triangle has just three possible opposite corners. The
    // edge scan always supplies its endpoints in (min, max) order, so a slot
    // for that opposite corner has exactly the original angle operands. Fill
    // only on the original query path; every triangle write invalidates its
    // three slots before any later candidate can use the new triangle.
    let mut corner_angles = vec![TriangleCornerAngles::default(); triangles.len()];
    let mut total_flips = 0usize;
    let mut converged = false;
    let _timer = stage_timer(Stage::Flip);
    tess_profile(|p| p.flip_calls += 1);
    let build_started = profile_started();
    // This table is exactly the old sorted (edge, triangle-index) corner
    // multiset. Its iteration is a ROUND-START snapshot: successful flips
    // queue ownership deltas, applied only after every edge was visited. In
    // particular we must not expose a newly created edge later in this same
    // round. Owners include repeated corners of degenerate triangles.
    //
    // The table was a `BTreeMap<_, Vec<usize>>` walked whole every round. It
    // is a hash map with inline owner storage now, walked through `agenda`:
    // every edge in sorted order in the first round, and afterwards only the
    // edges of the triangles the previous round rewrote, still in sorted
    // order. An edge whose two triangles did not change since it was last
    // declined cannot flip now — the angle test, the sag guard and the
    // protected-segment test read nothing but those triangles' vertices, which
    // never move — and an edge skipped as `touched` had an owner rewritten in
    // that round, so it is on the next agenda. The decisions, their order
    // within a round, and the round count are the ones the full walk made
    // (the trace tests below compare them against the rebuilt reference);
    // on a bore wall the walk visited 20 000 edges for 486 flips.
    let mut edge_owners: FxHashMap<(usize, usize), Owners> = FxHashMap::default();
    edge_owners.reserve(triangles.len() * 3 / 2 + 1);
    for (index, triangle) in triangles.iter().enumerate() {
        for corner in 0..3 {
            let a = triangle[corner];
            let b = triangle[(corner + 1) % 3];
            edge_owners.entry((a.min(b), a.max(b))).or_default().push(index);
        }
    }
    let mut agenda: Vec<(usize, usize)> = edge_owners.keys().copied().collect();
    agenda.sort_unstable();
    let mut patches: Vec<(usize, usize, [usize; 4])> = Vec::new();
    let mut touched: Vec<bool> = Vec::with_capacity(triangles.len());
    // Protected segments, bucketed once per call — they are boundary vertices,
    // which no flip or split ever moves.
    let protected = ProtectedSegments::new(boundary_pairs.iter().copied(), |index| {
        vertices[index].uv
    });
    tess_profile(|p| p.flip_build_ms += elapsed_ms(build_started));
    for _round in 0..maximum_rounds {
        tess_profile(|p| p.flip_rounds += 1);
        patches.clear();
        touched.clear();
        touched.resize(triangles.len(), false);
        let mut flipped_any = false;
        let scan_started = profile_started();
        // Iterate edges in a DETERMINISTIC (sorted) order, not HashMap order.
        // The per-round `touched` guard makes the chosen flips order-sensitive
        // whenever a mesh has cocircular ambiguity (a dense sphere), so raw
        // HashMap order let serial and parallel face passes converge to
        // DIFFERENT triangulations of the same points. Sorted iteration removes
        // that: already-unambiguous meshes (box, cylinder) are unaffected, and
        // ambiguous ones resolve identically every run — the serial==parallel
        // fingerprint holds even where the curvature-adaptive refinement makes a
        // face dense enough to be cocircular.
        for &(a, b) in &agenda {
            tess_profile(|p| p.flip_edges_visited += 1);
            // An agenda edge may have left the mesh in the round that put it
            // there (the flipped diagonal itself); the snapshot never held it.
            let Some(owners) = edge_owners.get(&(a, b)) else {
                continue;
            };
            let owners = owners.as_slice();
            if owners.len() != 2 || boundary_pairs.contains(&(a, b)) {
                continue;
            }
            let (first, second) = (owners[0], owners[1]);
            if touched[first] || touched[second] {
                continue;
            }
            let other = |triangle: [usize; 3]| {
                triangle
                    .into_iter()
                    .find(|&vertex| vertex != a && vertex != b)
            };
            let (Some(c), Some(d)) = (other(triangles[first]), other(triangles[second])) else {
                continue;
            };
            if c == d {
                continue;
            }
            let first_angle = corner_angles[first].angle(triangles[first], c, || corner_angle(c, a, b));
            let second_angle = corner_angles[second].angle(triangles[second], d, || corner_angle(d, a, b));
            if first_angle + second_angle <= std::f64::consts::PI + 1e-9 {
                continue;
            }
            if let Some((face, chord_tolerance, periodic)) = chord_guard {
                let mut sag = |x: usize, y: usize| -> Option<f64> {
                    let key = (x.min(y), x.max(y));
                    if let Some(&cached) = sag_memo.get(&key) {
                        return Some(cached);
                    }
                    tess_profile(|p| p.flip_sag_evals += 1);
                    let middle = memo.midpoint(face, vertices, x, y, periodic).ok()?;
                    let value = deflection_to_chord(
                        middle,
                        vertices[x].position,
                        vertices[y].position,
                    );
                    sag_memo.insert(key, value);
                    Some(value)
                };
                let new_sag = sag(c, d);
                let old_sag = sag(a, b);
                if let (Some(new_sag), Some(old_sag)) = (new_sag, old_sag) {
                    if new_sag > chord_tolerance && new_sag > old_sag {
                        continue;
                    }
                }
            }
            // Replacement triangles, oriented to keep positive uv area. The
            // (min, max) edge key loses which side c sits on, so normalize
            // the edge direction to put c on the left first.
            let (a, b) = if uv_area(a, b, c) > 0.0 {
                (a, b)
            } else {
                (b, a)
            };
            let candidates = [[c, a, d], [c, d, b]];
            if candidates
                .iter()
                .any(|&[x, y, z]| uv_area(x, y, z) <= 1e-14)
            {
                continue;
            }
            // CONSTRAINED flip: the new diagonal (c, d) must not cross any
            // protected boundary segment. Plain Delaunay is trim-blind, and
            // near a concave hole rim the empty-circumcircle winner can be a
            // diagonal straight across the hole (boxy frame face: flip
            // diagonals dipped into the bolt holes).
            if !protected.is_empty()
                && protected.crosses(c, d, |index| vertices[index].uv, 1e-14)
            {
                continue;
            }
            patches.push((first, second, [a, b, c, d]));
            corner_angles[first].clear();
            corner_angles[second].clear();
            triangles[first] = candidates[0];
            triangles[second] = candidates[1];
            touched[first] = true;
            touched[second] = true;
            flipped_any = true;
            total_flips += 1;
            tess_profile(|p| p.flips += 1);
        }
        tess_profile(|p| p.flip_scan_ms += elapsed_ms(scan_started));
        let apply_started = profile_started();
        // Only the four changed edge groups need ownership updates. The
        // round-start table was immutable throughout the scan; these sorted
        // multiset deltas commute even when different patches share an edge.
        agenda.clear();
        for &(first, second, corners) in &patches {
            apply_flip_patch(&mut edge_owners, first, second, corners);
            // Keep ALL new triangle edges on the next agenda, including an
            // edge whose owner stayed the same but whose triangle changed.
            for index in [first, second] {
                let triangle = triangles[index];
                for corner in 0..3 {
                    let a = triangle[corner];
                    let b = triangle[(corner + 1) % 3];
                    agenda.push((a.min(b), a.max(b)));
                }
            }
        }
        agenda.sort_unstable();
        agenda.dedup();
        tess_profile(|p| p.flip_apply_ms += elapsed_ms(apply_started));
        if !flipped_any {
            converged = true;
            break;
        }
    }
    if std::env::var("BREP_DEBUG_TESS").is_ok() {
        eprintln!(
            "  lawson: {total_flips} flips over {} triangles",
            triangles.len()
        );
    }
    converged
}



