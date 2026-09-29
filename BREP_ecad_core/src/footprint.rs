//! Ordering pad and pin labels, which are free strings that usually carry numbers.
use crate::Symbol;
use std::cmp::Ordering;
use std::collections::BTreeSet;

/// Order strings with embedded numbers numerically: `2 < 10`, `A2 < A10 < B1`.
pub fn natural_cmp(a: &str, b: &str) -> Ordering {
    let (mut x, mut y) = (a.as_bytes(), b.as_bytes());
    loop {
        match (x.first(), y.first()) {
            (None, None) => return a.cmp(b),
            (None, Some(_)) => return Ordering::Less,
            (Some(_), None) => return Ordering::Greater,
            (Some(c), Some(d)) if c.is_ascii_digit() && d.is_ascii_digit() => {
                let i = x.iter().take_while(|c| c.is_ascii_digit()).count();
                let j = y.iter().take_while(|c| c.is_ascii_digit()).count();
                let trim = |s: &[u8]| -> Vec<u8> {
                    s.iter().copied().skip_while(|c| *c == b'0').collect()
                };
                let (m, n) = (trim(&x[..i]), trim(&y[..j]));
                let order = m.len().cmp(&n.len()).then_with(|| m.cmp(&n));
                if order != Ordering::Equal {
                    return order;
                }
                (x, y) = (&x[i..], &y[j..]);
            }
            (Some(c), Some(d)) => {
                if c != d {
                    return c.cmp(d);
                }
                (x, y) = (&x[1..], &y[1..]);
            }
        }
    }
}
/// Distinct symbol pin numbers in natural order.
pub fn pin_numbers(symbol: &Symbol) -> Vec<String> {
    let mut pins: Vec<String> = symbol
        .pins
        .iter()
        .map(|p| p.number.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    pins.sort_by(|a, b| natural_cmp(a, b));
    pins
}
