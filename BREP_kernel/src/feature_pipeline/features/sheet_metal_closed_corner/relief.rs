//! Exact 2D relief shapes at the bend centres (NX "Bend Centers"), built from
//! line segments and circular arcs, clipped by half-planes and moved rigidly
//! between the flat-pattern frames of the parent, the bands and the walls.
//! Every shape here is convex, which keeps the clipper simple and exact.
use crate::{make_arc, make_line, NurbsCurve, Vec3};
use std::f64::consts::{PI, TAU};

type P = [f64; 2];
const EPS: f64 = 1e-12;

#[derive(Clone, Debug)]
pub(super) enum Seg {
    Line(P, P),
    /// Counter-clockwise from `a0` to `a1` (`a0 < a1`).
    Arc { c: P, r: f64, a0: f64, a1: f64 },
}

fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn scale(a: P, k: f64) -> P {
    [a[0] * k, a[1] * k]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn polar(c: P, r: f64, a: f64) -> P {
    [c[0] + r * a.cos(), c[1] + r * a.sin()]
}
fn close(a: P, b: P) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9
}

impl Seg {
    fn start(&self) -> P {
        match *self {
            Seg::Line(a, _) => a,
            Seg::Arc { c, r, a0, .. } => polar(c, r, a0),
        }
    }
    fn end(&self) -> P {
        match *self {
            Seg::Line(_, b) => b,
            Seg::Arc { c, r, a1, .. } => polar(c, r, a1),
        }
    }
    fn mid(&self) -> P {
        match *self {
            Seg::Line(a, b) => scale(add(a, b), 0.5),
            Seg::Arc { c, r, a0, a1 } => polar(c, r, 0.5 * (a0 + a1)),
        }
    }
}

fn line(a: P, b: P, out: &mut Vec<Seg>) {
    if !close(a, b) {
        out.push(Seg::Line(a, b));
    }
}

/// A slot of width `2r` and overall length `length` along unit `d`, centred on `o`.
pub(super) fn slot(o: P, d: P, r: f64, length: f64) -> Vec<Seg> {
    let h = (0.5 * length - r).max(0.0);
    let n = [-d[1], d[0]];
    let (c1, c2) = (sub(o, scale(d, h)), add(o, scale(d, h)));
    let th = d[1].atan2(d[0]);
    let mut out = Vec::new();
    line(sub(c1, scale(n, r)), sub(c2, scale(n, r)), &mut out);
    out.push(Seg::Arc { c: c2, r, a0: th - PI / 2.0, a1: th + PI / 2.0 });
    line(add(c2, scale(n, r)), add(c1, scale(n, r)), &mut out);
    out.push(Seg::Arc { c: c1, r, a0: th + PI / 2.0, a1: th + 1.5 * PI });
    out
}

/// The convex hull of the disc `(o, r)` and the points `p1`, `p2`, which must
/// lie outside it with `o, p1, p2` counter-clockwise. `r = 0` is the triangle.
pub(super) fn hull(o: P, r: f64, p1: P, p2: P) -> Result<Vec<Seg>, String> {
    let mut out = Vec::new();
    if r <= EPS {
        line(o, p1, &mut out);
        line(p1, p2, &mut out);
        line(p2, o, &mut out);
        return Ok(out);
    }
    let tangent = |p: P| -> Result<(f64, f64), String> {
        let v = sub(p, o);
        let dist = v[0].hypot(v[1]);
        if dist <= r * (1.0 + 1e-9) {
            return Err("each V flank must reach beyond the rounded apex".into());
        }
        Ok((v[1].atan2(v[0]), (r / dist).acos()))
    };
    let (t1, b1) = tangent(p1)?;
    let (t2, b2) = tangent(p2)?;
    // The segment p1 → p2 must clear the disc, or the circle bulges past it.
    let e = sub(p2, p1);
    let reach = ((p1[0] - o[0]) * e[1] - (p1[1] - o[1]) * e[0]) / e[0].hypot(e[1]);
    if reach <= r {
        return Err("the V flanks are too short for its rounded apex".into());
    }
    let (a_end, mut a_start) = (t1 - b1, t2 + b2);
    // Arc counter-clockwise from the p2 tangent round the back to the p1 tangent.
    let mut a_stop = a_end;
    while a_stop <= a_start {
        a_stop += TAU;
    }
    while a_stop - a_start > TAU {
        a_start += TAU;
    }
    let (q1, q2) = (polar(o, r, a_end), polar(o, r, a_start));
    line(q1, p1, &mut out);
    line(p1, p2, &mut out);
    line(p2, q2, &mut out);
    out.push(Seg::Arc { c: o, r, a0: a_start, a1: a_stop });
    Ok(out)
}

/// Keep the part of the convex loop where `n · x <= k` (unit `n`). Empty when
/// nothing is kept.
pub(super) fn clip(shape: &[Seg], n: P, k: f64) -> Vec<Seg> {
    let inside = |p: P| dot(n, p) <= k + 1e-12;
    let mut pieces: Vec<Seg> = Vec::new();
    for seg in shape {
        match *seg {
            Seg::Line(a, b) => {
                let (fa, fb) = (dot(n, a) - k, dot(n, b) - k);
                if fa <= 0.0 && fb <= 0.0 {
                    pieces.push(seg.clone());
                } else if fa < 0.0 || fb < 0.0 {
                    let x = add(a, scale(sub(b, a), fa / (fa - fb)));
                    if fa < 0.0 {
                        line(a, x, &mut pieces);
                    } else {
                        line(x, b, &mut pieces);
                    }
                }
            }
            Seg::Arc { c, r, a0, a1 } => {
                let m = (k - dot(n, c)) / r;
                let mut cuts = vec![a0, a1];
                if m.abs() < 1.0 {
                    let base = n[1].atan2(n[0]);
                    for root in [base + m.acos(), base - m.acos()] {
                        let mut a = root;
                        while a < a0 {
                            a += TAU;
                        }
                        while a > a1 {
                            a -= TAU;
                        }
                        if a > a0 + EPS && a < a1 - EPS {
                            cuts.push(a);
                        }
                    }
                }
                cuts.sort_by(f64::total_cmp);
                for pair in cuts.windows(2) {
                    let piece = Seg::Arc { c, r, a0: pair[0], a1: pair[1] };
                    if pair[1] - pair[0] > EPS && inside(piece.mid()) {
                        pieces.push(piece);
                    }
                }
            }
        }
    }
    if pieces.is_empty() {
        return pieces;
    }
    // A convex loop clipped by one line leaves at most one gap: rotate it to
    // the end and bridge it along the clip line.
    let count = pieces.len();
    if let Some(gap) = (0..count).find(|&i| !close(pieces[i].end(), pieces[(i + 1) % count].start())) {
        pieces.rotate_left(gap + 1);
        let (from, to) = (pieces[count - 1].end(), pieces[0].start());
        line(from, to, &mut pieces);
    }
    pieces
}

/// The loop in the frame whose origin is `origin` and whose x axis is the unit
/// `x_axis` (y = x_axis turned a quarter turn counter-clockwise).
pub(super) fn into_frame(shape: &[Seg], origin: P, x_axis: P) -> Vec<Seg> {
    let y_axis = [-x_axis[1], x_axis[0]];
    let map = |p: P| {
        let v = sub(p, origin);
        [dot(v, x_axis), dot(v, y_axis)]
    };
    let turn = x_axis[1].atan2(x_axis[0]);
    shape
        .iter()
        .map(|seg| match *seg {
            Seg::Line(a, b) => Seg::Line(map(a), map(b)),
            Seg::Arc { c, r, a0, a1 } => Seg::Arc { c: map(c), r, a0: a0 - turn, a1: a1 - turn },
        })
        .collect()
}

pub(super) fn curves(shape: &[Seg]) -> Result<Vec<NurbsCurve>, String> {
    let v = |p: P| Vec3::new(p[0], p[1], 0.0);
    shape
        .iter()
        .map(|seg| match *seg {
            Seg::Line(a, b) => make_line(v(a), v(b)),
            Seg::Arc { c, r, a0, a1 } => make_arc(v(c), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), r, a0, a1),
        })
        .collect()
}
