//! The arrangement: every input segment split at every intersection, vertices
//! merged within `point_eps`, coincident pieces grouped. Booleans and offsets
//! classify the pieces of one arrangement; nothing else ever has to reason
//! about crossings.

use super::{Segment, Vec2};
use crate::model::Point2;
use crate::numeric::Tolerance;

/// One input segment with the operand and source index it came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Input {
    pub seg: Segment,
    /// Operand tag (0 = A, 1 = B for booleans; raw-curve kinds for offsets).
    pub operand: u8,
    /// Position in the caller's segment list of that operand.
    pub source: usize,
}

/// A piece of an input segment between two consecutive split points.
#[derive(Clone, Debug, PartialEq)]
pub struct Piece {
    pub seg: Segment,
    pub v0: usize,
    pub v1: usize,
    pub operand: u8,
    pub source: usize,
}

/// Pieces, merged vertices and coincidence groups.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Arrangement {
    pub pieces: Vec<Piece>,
    pub vertices: Vec<Point2>,
    /// Indexes into `pieces`; each group holds geometrically identical pieces
    /// (same vertices, same support), the lowest index first.
    pub groups: Vec<Vec<usize>>,
}

/// Union-find with path compression; deterministic.
pub(crate) struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    pub(crate) fn new(n: usize) -> Self {
        Self { parent: (0..n).collect() }
    }
    pub(crate) fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }
    pub(crate) fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            // The smaller root wins so the representative is the earliest point.
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            self.parent[hi] = lo;
        }
    }
}

/// Build the arrangement of `inputs` (already canonical segments).
pub fn arrange(inputs: &[Input], tol: &Tolerance) -> Arrangement {
    let n = inputs.len();
    // Candidate points: endpoints first (2 per input), then hits.
    let mut points: Vec<Point2> = Vec::with_capacity(2 * n);
    for i in inputs {
        points.push(i.seg.start);
        points.push(i.seg.end);
    }
    // Split parameters per input: (t, point index).
    let mut splits: Vec<Vec<(f64, usize)>> = vec![Vec::new(); n];
    // Sweep on x over bounds for candidate pairs.
    let boxes: Vec<(Point2, Point2)> = inputs.iter().map(|s| s.seg.bounds()).collect();
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| boxes[a].0.x.total_cmp(&boxes[b].0.x).then(a.cmp(&b)));
    let eps = tol.distance_eps;
    let mut active: Vec<usize> = Vec::new();
    for &i in &order {
        let (lo, hi) = boxes[i];
        active.retain(|&j| boxes[j].1.x >= lo.x - eps);
        for &j in &active {
            let (jlo, jhi) = boxes[j];
            if jlo.y > hi.y + eps || lo.y > jhi.y + eps {
                continue;
            }
            let (a, b) = if i < j { (i, j) } else { (j, i) };
            let hits = inputs[a].seg.intersect(&inputs[b].seg, tol);
            for h in hits {
                let idx = points.len();
                points.push(h.point);
                splits[a].push((h.ta, idx));
                splits[b].push((h.tb, idx));
            }
        }
        active.push(i);
    }
    // Merge vertices within point_eps.
    let m = points.len();
    let mut dsu = Dsu::new(m);
    let mut by_x: Vec<usize> = (0..m).collect();
    by_x.sort_by(|&a, &b| points[a].x.total_cmp(&points[b].x).then(a.cmp(&b)));
    for (k, &i) in by_x.iter().enumerate() {
        for &j in &by_x[k + 1..] {
            if points[j].x - points[i].x > tol.point_eps {
                break;
            }
            if points[i].distance(points[j]) <= tol.point_eps {
                dsu.union(i, j);
            }
        }
    }
    // Vertex ids in order of first appearance of each class representative.
    let mut vid_of_root: Vec<Option<usize>> = vec![None; m];
    let mut vertices: Vec<Point2> = Vec::new();
    let mut vid: Vec<usize> = vec![0; m];
    for i in 0..m {
        let r = dsu.find(i);
        let v = match vid_of_root[r] {
            Some(v) => v,
            None => {
                vertices.push(points[r]);
                vid_of_root[r] = Some(vertices.len() - 1);
                vertices.len() - 1
            }
        };
        vid[i] = v;
    }
    // Pieces.
    let mut pieces: Vec<Piece> = Vec::new();
    for (i, input) in inputs.iter().enumerate() {
        let mut cuts = std::mem::take(&mut splits[i]);
        cuts.push((0.0, 2 * i));
        cuts.push((1.0, 2 * i + 1));
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut prev: Option<usize> = None;
        for &(_, pidx) in &cuts {
            let v = vid[pidx];
            if let Some(pv) = prev {
                if pv != v {
                    let seg = input.seg.sub_segment(vertices[pv], vertices[v]);
                    if seg.start.distance(seg.end) > tol.point_eps {
                        pieces.push(Piece { seg, v0: pv, v1: v, operand: input.operand, source: i });
                    }
                }
            }
            prev = Some(v);
        }
    }
    // Coincidence groups: the same unordered vertices and the same curve
    // between them, where "the same" is a midpoint within the point epsilon:
    // two arcs about one centre with one sense, or a line and an arc whose
    // sagitta is below the epsilon (a vertex circle's last fraction of a
    // quadrant beside its tangent parallel), are one piece of the
    // arrangement and must be classified once.
    let mut keyed: Vec<(usize, usize, usize)> = pieces.iter().enumerate().map(|(k, p)| (p.v0.min(p.v1), p.v0.max(p.v1), k)).collect();
    keyed.sort();
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut start = 0;
    while start < keyed.len() {
        let mut end = start + 1;
        while end < keyed.len() && keyed[end].0 == keyed[start].0 && keyed[end].1 == keyed[start].1 {
            end += 1;
        }
        let members: Vec<usize> = keyed[start..end].iter().map(|k| k.2).collect();
        let mut taken = vec![false; members.len()];
        for a in 0..members.len() {
            if taken[a] {
                continue;
            }
            let mut g = vec![members[a]];
            taken[a] = true;
            let ma = pieces[members[a]].seg.midpoint();
            for b in a + 1..members.len() {
                if !taken[b] && ma.distance(pieces[members[b]].seg.midpoint()) <= tol.point_eps {
                    g.push(members[b]);
                    taken[b] = true;
                }
            }
            groups.push(g);
        }
        start = end;
    }
    groups.sort();
    Arrangement { pieces, vertices, groups }
}
