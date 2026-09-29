//! The OUTLINE of a copper layer as display edges: the boundary of the union of
//! everything drawn on it, not the outline of each primitive.
//!
//! A layer's copper is drawn as overlapping prisms — a track is one capsule per
//! segment, and a pad under a track's end is another shape under the same
//! copper — so tracing every primitive's own ring would draw a circle at every
//! bend and a pad's rectangle straight through the track that lands on it. What
//! a user reads as "the edge of the copper" is where copper stops, so each
//! primitive's ring is CLIPPED against every other shape on the layer and only
//! the parts outside all of them are kept. The kept pieces are then chained end
//! to end, so one side of a routed track comes out as one polyline rather than
//! one per segment: the polyline count is what the renderer iterates per frame.
//!
//! Every shape is a ring in millimetres. A [`Region::Convex`] is one
//! counter-clockwise convex ring — every copper primitive's `shape_loop` is one —
//! and a [`Region::Fill`] is a zone fill piece's outer ring and holes, read
//! even-odd. Both lookups go through a uniform grid, so the cost is the number
//! of ring edges times the handful of shapes near each one, not the square of
//! the layer's shape count.

/// A point strictly inside a shape by less than this, in millimetres, counts as
/// ON its boundary: two coincident rings (a pad placed twice, a track retraced)
/// keep their edge instead of clipping each other away to nothing.
const INSIDE_EPS: f64 = 1e-7;
/// Endpoints closer than this, in millimetres, are joined when chaining. Far
/// above the float noise between two computations of one crossing, far below
/// anything drawn.
const JOIN_QUANTUM: f64 = 1e-6;
/// The grid's cell, in millimetres: about a track's width, so a cell holds a
/// few shapes on a dense layer.
const CELL: f64 = 0.5;

/// One shape on a copper layer.
pub(super) enum Region {
    /// A convex ring, counter-clockwise.
    Convex(Vec<[f64; 2]>),
    /// A zone fill piece: its outer ring and its holes, even-odd.
    Fill(Vec<Vec<[f64; 2]>>),
}

impl Region {
    fn rings(&self) -> &[Vec<[f64; 2]>] {
        match self {
            Region::Convex(ring) => std::slice::from_ref(ring),
            Region::Fill(rings) => rings,
        }
    }
}

/// A uniform grid of item indices over the layer's extent, flat so a lookup is
/// an index and not a hash.
struct Grid {
    x0: f64,
    y0: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl Grid {
    fn new(extent: [f64; 4]) -> Self {
        let nx = (((extent[2] - extent[0]) / CELL).floor() as usize + 1).min(4096);
        let ny = (((extent[3] - extent[1]) / CELL).floor() as usize + 1).min(4096);
        Grid { x0: extent[0], y0: extent[1], nx, ny, cells: vec![Vec::new(); nx * ny] }
    }

    fn column(&self, x: f64) -> usize {
        (((x - self.x0) / CELL).floor().max(0.) as usize).min(self.nx - 1)
    }

    fn row(&self, y: f64) -> usize {
        (((y - self.y0) / CELL).floor().max(0.) as usize).min(self.ny - 1)
    }

    /// The cells a box touches, row by row.
    fn over(&self, b: [f64; 4]) -> impl Iterator<Item = usize> + '_ {
        let (c0, c1) = (self.column(b[0]), self.column(b[2]));
        (self.row(b[1])..=self.row(b[3])).flat_map(move |r| (c0..=c1).map(move |c| r * self.nx + c))
    }

    fn insert(&mut self, b: [f64; 4], item: u32) {
        let (c0, c1) = (self.column(b[0]), self.column(b[2]));
        for r in self.row(b[1])..=self.row(b[3]) {
            for cell in &mut self.cells[r * self.nx + c0..=r * self.nx + c1] {
                cell.push(item);
            }
        }
    }

    fn at(&self, p: [f64; 2]) -> &[u32] {
        &self.cells[self.row(p[1]) * self.nx + self.column(p[0])]
    }
}

/// A point snapped to [`JOIN_QUANTUM`], for matching ends.
fn quantize(p: [f64; 2]) -> (i64, i64) {
    ((p[0] / JOIN_QUANTUM).round() as i64, (p[1] / JOIN_QUANTUM).round() as i64)
}

fn segment_box(a: [f64; 2], b: [f64; 2]) -> [f64; 4] {
    [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])]
}

fn bbox(rings: &[Vec<[f64; 2]>]) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in rings.iter().flatten() {
        b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
    }
    b
}

/// Twice the signed area of `a b c`: positive when `c` is left of `a -> b`.
fn cross(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Whether `p` is strictly inside the counter-clockwise convex `ring`.
fn inside_convex(ring: &[[f64; 2]], p: [f64; 2]) -> bool {
    (0..ring.len()).all(|i| {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        // A zero-length edge (a repeated corner) says nothing about inside.
        len <= 0. || cross(a, b, p) > INSIDE_EPS * len
    })
}

/// A fill piece, bucketed by horizontal band so an even-odd test reads only the
/// edges that span the point's height — a fill ring round a board of pads has
/// thousands of vertices, and every copper edge over the fill asks about it.
struct Banded {
    y0: f64,
    band: f64,
    bands: Vec<Vec<([f64; 2], [f64; 2])>>,
}

impl Banded {
    fn new(rings: &[Vec<[f64; 2]>], bbox: [f64; 4]) -> Self {
        let count = (rings.iter().map(Vec::len).sum::<usize>() / 8).clamp(1, 1024);
        let band = ((bbox[3] - bbox[1]) / count as f64).max(1e-9);
        let mut bands = vec![Vec::new(); count];
        for ring in rings {
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                let lo = (((a[1].min(b[1]) - bbox[1]) / band).floor() as isize).clamp(0, count as isize - 1);
                let hi = (((a[1].max(b[1]) - bbox[1]) / band).floor() as isize).clamp(0, count as isize - 1);
                for k in lo..=hi {
                    bands[k as usize].push((a, b));
                }
            }
        }
        Banded { y0: bbox[1], band, bands }
    }

    /// Even-odd: inside when a ray to +x crosses an odd number of edges, and not
    /// within [`INSIDE_EPS`] of any of them.
    fn inside(&self, p: [f64; 2]) -> bool {
        let k = ((p[1] - self.y0) / self.band).floor();
        if k < 0. || k as usize >= self.bands.len() {
            return false;
        }
        let mut odd = false;
        for &(a, b) in &self.bands[k as usize] {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len2 = dx * dx + dy * dy;
            if len2 > 0. {
                let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0., 1.);
                let (qx, qy) = (a[0] + t * dx - p[0], a[1] + t * dy - p[1]);
                if qx * qx + qy * qy <= INSIDE_EPS * INSIDE_EPS {
                    return false;
                }
            }
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / dy * dx {
                odd = !odd;
            }
        }
        odd
    }
}

/// The boundary of the union of `regions`, as polylines in millimetres.
pub(super) fn union_boundary(regions: &[Region]) -> Vec<Vec<[f64; 2]>> {
    let boxes: Vec<[f64; 4]> = regions.iter().map(|r| bbox(r.rings())).collect();
    let banded: Vec<Option<Banded>> = regions
        .iter()
        .zip(&boxes)
        .map(|(r, b)| match r {
            Region::Fill(rings) => Some(Banded::new(rings, *b)),
            Region::Convex(_) => None,
        })
        .collect();
    // Every ring edge, and the grid cells each one and each shape's box touch.
    let mut edges: Vec<([f64; 2], [f64; 2], usize)> = Vec::new();
    for (owner, region) in regions.iter().enumerate() {
        for ring in region.rings() {
            for i in 0..ring.len() {
                edges.push((ring[i], ring[(i + 1) % ring.len()], owner));
            }
        }
    }
    let finite: Vec<&[f64; 4]> = boxes.iter().filter(|b| b[0].is_finite()).collect();
    if finite.is_empty() {
        return Vec::new();
    }
    let extent = finite.iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |e, b| {
        [e[0].min(b[0]), e[1].min(b[1]), e[2].max(b[2]), e[3].max(b[3])]
    });
    let mut edge_grid = Grid::new(extent);
    for (i, &(a, b, _)) in edges.iter().enumerate() {
        edge_grid.insert(segment_box(a, b), i as u32);
    }
    let mut shape_grid = Grid::new(extent);
    for (i, b) in boxes.iter().enumerate() {
        if b[0].is_finite() {
            shape_grid.insert(*b, i as u32);
        }
    }
    let covered = |p: [f64; 2], owner: usize| -> bool {
        shape_grid.at(p).iter().any(|&i| {
            let i = i as usize;
            let b = boxes[i];
            if i == owner || p[0] < b[0] || p[0] > b[2] || p[1] < b[1] || p[1] > b[3] {
                return false;
            }
            match (&regions[i], &banded[i]) {
                (Region::Convex(ring), _) => inside_convex(ring, p),
                (Region::Fill(_), Some(banded)) => banded.inside(p),
                (Region::Fill(_), None) => false,
            }
        })
    };

    let mut kept: Vec<([f64; 2], [f64; 2])> = Vec::new();
    let mut stamp = vec![usize::MAX; edges.len()];
    let mut ts: Vec<f64> = Vec::new();
    for (index, &(p, q, owner)) in edges.iter().enumerate() {
        let d = [q[0] - p[0], q[1] - p[1]];
        if d[0] == 0. && d[1] == 0. {
            continue;
        }
        // Where the edge crosses another shape's boundary: the only places
        // where it can go from outside the copper to inside it.
        ts.clear();
        ts.extend([0., 1.]);
        for cell in edge_grid.over(segment_box(p, q)) {
            {
                for &j in &edge_grid.cells[cell] {
                    let j = j as usize;
                    if stamp[j] == index || edges[j].2 == owner {
                        continue;
                    }
                    stamp[j] = index;
                    let (a, b, _) = edges[j];
                    let e = [b[0] - a[0], b[1] - a[1]];
                    let den = d[0] * e[1] - d[1] * e[0];
                    if den == 0. {
                        continue;
                    }
                    let w = [a[0] - p[0], a[1] - p[1]];
                    let t = (w[0] * e[1] - w[1] * e[0]) / den;
                    let u = (w[0] * d[1] - w[1] * d[0]) / den;
                    if t > 0. && t < 1. && (0. ..=1.).contains(&u) {
                        ts.push(t);
                    }
                }
            }
        }
        ts.sort_by(f64::total_cmp);
        let at = |t: f64| [p[0] + d[0] * t, p[1] + d[1] * t];
        for pair in ts.windows(2) {
            let (t0, t1) = (pair[0], pair[1]);
            if t1 - t0 <= 1e-12 {
                continue;
            }
            if !covered(at((t0 + t1) * 0.5), owner) {
                kept.push((at(t0), at(t1)));
            }
        }
    }
    // Two shapes can share a stretch of boundary exactly — the caps of two
    // capsules meeting at a bend lie on one circle at the same facet angles —
    // and neither is strictly inside the other there, so both kept it. Once is
    // the outline; twice would be a branch at each end and break the chain.
    let mut seen = std::collections::HashSet::with_capacity(kept.len());
    kept.retain(|&(a, b)| {
        let (ka, kb) = (quantize(a), quantize(b));
        ka != kb && seen.insert(if ka < kb { (ka, kb) } else { (kb, ka) })
    });
    chain(&kept)
}

/// Join segments that share an endpoint into polylines. A closed loop comes back
/// with its first point repeated at the end.
fn chain(segments: &[([f64; 2], [f64; 2])]) -> Vec<Vec<[f64; 2]>> {
    // Every segment end, sorted by where it is: the ends at one point are then
    // one run, found by a binary search rather than a hash per step.
    let mut ends: Vec<((i64, i64), u32)> = Vec::with_capacity(segments.len() * 2);
    for (i, &(a, b)) in segments.iter().enumerate() {
        ends.push((quantize(a), 2 * i as u32));
        ends.push((quantize(b), 2 * i as u32 + 1));
    }
    ends.sort_unstable();
    // The run of ends at `p`.
    let at = |p: [f64; 2]| -> &[((i64, i64), u32)] {
        let k = quantize(p);
        let start = ends.partition_point(|e| e.0 < k);
        let len = ends[start..].partition_point(|e| e.0 == k);
        &ends[start..start + len]
    };
    let mut used = vec![false; segments.len()];
    let mut out = Vec::new();
    // From the chain's tip, keep taking the one unused segment there — but only
    // where the point joins exactly two segments: a point three pieces meet at
    // is where the chain honestly branches.
    let extend = |line: &mut Vec<[f64; 2]>, used: &mut Vec<bool>| loop {
        let here = at(*line.last().expect("a chain has a point"));
        if here.len() != 2 {
            break;
        }
        let Some(&(_, end)) = here.iter().find(|(_, end)| !used[*end as usize / 2]) else { break };
        let next = end as usize / 2;
        used[next] = true;
        let (a, b) = segments[next];
        // Entered at its `b` end (odd), it is walked towards `a`.
        line.push(if end % 2 == 1 { a } else { b });
    };
    // Open chains first, from their ends, so none is cut in the middle; then
    // whatever is left is a closed loop and may start anywhere.
    let open: Vec<usize> = (0..segments.len())
        .filter(|&i| at(segments[i].0).len() != 2 || at(segments[i].1).len() != 2)
        .collect();
    for i in open.into_iter().chain(0..segments.len()) {
        if used[i] {
            continue;
        }
        used[i] = true;
        let (a, b) = segments[i];
        let mut line = vec![a, b];
        extend(&mut line, &mut used);
        // Walk the other way from the start, for a chain entered in its middle.
        line.reverse();
        extend(&mut line, &mut used);
        out.push(line);
    }
    out
}
