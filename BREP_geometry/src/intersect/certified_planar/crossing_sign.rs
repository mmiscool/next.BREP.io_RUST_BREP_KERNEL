//! The certified sign of a planar crossing between two polynomial profiles.
//!
//! Two curves cross at parameters `s*` (first) and `t*` (second) known only to
//! lie in boxes `S` and `T`. The side one curve takes at the crossing with
//! respect to the other is the sign of `cross2(dA/ds(s*), dB/dt(t*))` in the
//! plane `axes`. Over a Bezier piece restricted to its box, the derivative is a
//! polynomial whose control net (enclosed outward here) contains every value
//! it takes, and `cross2` is bilinear: if every pair of derivative control
//! vectors, one from each curve, gives a strictly positive (or strictly
//! negative) enclosed `cross2`, the sign holds for every `s` in `S` and `t` in
//! `T`. A common strict sign also rules out either net containing zero (a
//! curve that stops), so no separate speed test is needed. Anything else
//! declines.
use super::{spans, I, P};

/// The derivative control net of `span` restricted to the parameters
/// `[from, to]` inside it, enclosed outward: de Casteljau at the interval
/// local parameters of the two ends. Positive scale factors are left out (they
/// cannot change a sign).
fn restricted_net(span: &super::Span, from: f64, to: f64) -> Option<Vec<P>> {
    let width = I::point(span.end) - I::point(span.start);
    let local = |x: f64| -> Option<I> {
        let u = (I::point(x) - I::point(span.start)).divide(width)?;
        Some(I::range(u.lo.max(0.), u.hi.min(1.)))
    };
    let (u0, u1) = (local(from)?, local(to)?);
    if !(u0.finite() && u1.finite()) {
        return None;
    }
    // Keep [0, u1] of the net (left part of a split at u1).
    let split_left = |net: &[P], t: I| -> Vec<P> {
        let mut work = net.to_vec();
        let mut left = vec![work[0]];
        while work.len() > 1 {
            work = work.windows(2).map(|p| super::blend(p[1], p[0], t)).collect();
            left.push(work[0]);
        }
        left
    };
    // Keep [t, 1] of the net (right part of a split at t).
    let split_right = |net: &[P], t: I| -> Vec<P> {
        let mut work = net.to_vec();
        let mut right = vec![*work.last().unwrap()];
        while work.len() > 1 {
            work = work.windows(2).map(|p| super::blend(p[1], p[0], t)).collect();
            right.push(*work.last().unwrap());
        }
        right.reverse();
        right
    };
    if span.tangents.is_empty() {
        return None;
    }
    let left = split_left(&span.tangents, u1);
    // On [0, u1] the start of the box sits at u0 / u1.
    let inner = if u0.hi <= 0. {
        left
    } else {
        let ratio = u0.divide(u1)?;
        split_right(&left, I::range(ratio.lo.max(0.), ratio.hi.min(1.)))
    };
    inner.iter().all(|p| p.iter().all(|v| v.finite())).then_some(inner)
}

/// Every derivative control vector of `curve` over `[from, to]` (all spans the
/// box meets), enclosed outward. `None` for an unsupported curve (see
/// [`spans`]) or a box outside the curve's domain.
fn derivative_nets(curve: &crate::NurbsCurve, from: f64, to: f64) -> Option<Vec<P>> {
    if !(from.is_finite() && to.is_finite() && from <= to) {
        return None;
    }
    let pieces = spans(curve, true, None)?;
    let (lo, hi) = (pieces.first()?.start, pieces.last()?.end);
    if from < lo || to > hi {
        return None;
    }
    let mut vectors = Vec::new();
    for span in &pieces {
        let (a, b) = (from.max(span.start), to.min(span.end));
        if a > b {
            continue;
        }
        vectors.extend(restricted_net(span, a, b)?);
    }
    (!vectors.is_empty()).then_some(vectors)
}

/// `+1` or `-1`: the sign of `cross2(dA/ds, dB/dt)` (components `axes`) for
/// every `s` in `s_box` and `t` in `t_box`, where `A` is `first` and `B` is
/// `second`; `None` when it is not certified (unsupported curves, rational
/// weights, a box outside the domain, a tangent cone that meets zero or the
/// other curve's cone, or rounding that cannot separate the sign from zero).
pub(super) fn crossing_sign(
    first: &crate::NurbsCurve,
    s_box: [f64; 2],
    second: &crate::NurbsCurve,
    t_box: [f64; 2],
    axes: [usize; 2],
) -> Option<i8> {
    if axes[0] > 2 || axes[1] > 2 || axes[0] == axes[1] {
        return None;
    }
    let a = derivative_nets(first, s_box[0], s_box[1])?;
    let b = derivative_nets(second, t_box[0], t_box[1])?;
    let mut sign = 0i8;
    for u in &a {
        for v in &b {
            let cross = u[axes[0]] * v[axes[1]] - u[axes[1]] * v[axes[0]];
            if !cross.finite() {
                return None;
            }
            let here = if cross.lo > 0. {
                1
            } else if cross.hi < 0. {
                -1
            } else {
                return None;
            };
            if sign == 0 {
                sign = here;
            } else if sign != here {
                return None;
            }
        }
    }
    (sign != 0).then_some(sign)
}

