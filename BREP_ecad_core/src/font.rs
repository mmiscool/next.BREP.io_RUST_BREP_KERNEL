//! The stroke font the silkscreen is written in: reference designators and
//! drawn text.
use crate::Point;
use std::collections::BTreeSet;

/// Stroke-font glyphs on a 4 × 6 cell, Y up. Each glyph is polylines separated by
/// `;`, each polyline a list of `x,y` pairs.
fn glyph(c: char) -> Option<&'static str> {
    Some(match c {
        '0' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0;1,1 3,5",
        '1' => "1,5 2,6 2,0;1,0 3,0",
        '2' => "0,5 1,6 3,6 4,5 4,4 0,0 4,0",
        '3' => "0,5 1,6 3,6 4,5 4,4 3,3 4,2 4,1 3,0 1,0 0,1;1,3 3,3",
        '4' => "3,0 3,6 0,2 4,2",
        '5' => "4,6 0,6 0,3 3,3 4,2 4,1 3,0 0,0",
        '6' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2 3,3 0,3",
        '7' => "0,6 4,6 1,0",
        '8' => "1,3 0,4 0,5 1,6 3,6 4,5 4,4 3,3 1,3 0,2 0,1 1,0 3,0 4,1 4,2 3,3",
        '9' => "0,1 1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,4 1,3 4,3",
        'A' => "0,0 0,4 2,6 4,4 4,0;0,3 4,3",
        'B' => "0,3 3,3 4,4 4,5 3,6 0,6 0,0 3,0 4,1 4,2 3,3",
        'C' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1",
        'D' => "0,0 0,6 2,6 4,4 4,2 2,0 0,0",
        'E' => "4,6 0,6 0,0 4,0;0,3 3,3",
        'F' => "4,6 0,6 0,0;0,3 3,3",
        'G' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,3 2,3",
        'H' => "0,0 0,6;4,0 4,6;0,3 4,3",
        'I' => "1,6 3,6;2,6 2,0;1,0 3,0",
        'J' => "4,6 4,1 3,0 1,0 0,1",
        'K' => "0,0 0,6;4,6 0,2;1,3 4,0",
        'L' => "0,6 0,0 4,0",
        'M' => "0,0 0,6 2,3 4,6 4,0",
        'N' => "0,0 0,6 4,0 4,6",
        'O' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0",
        'P' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3",
        'Q' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0;2,2 4,0",
        'R' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3;2,3 4,0",
        'S' => "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1",
        'T' => "0,6 4,6;2,6 2,0",
        'U' => "0,6 0,1 1,0 3,0 4,1 4,6",
        'V' => "0,6 2,0 4,6",
        'W' => "0,6 1,0 2,3 3,0 4,6",
        'X' => "0,0 4,6;0,6 4,0",
        'Y' => "0,6 2,3 4,6;2,3 2,0",
        'Z' => "0,6 4,6 0,0 4,0",
        '-' => "1,3 3,3",
        '_' => "0,0 4,0",
        '+' => "1,3 3,3;2,2 2,4",
        '.' => "2,0 2,0",
        '/' => "0,0 4,6",
        '?' => "0,5 1,6 3,6 4,5 4,4 2,3 2,2;2,0 2,0",
        _ => return None,
    })
}

/// Polylines for `text` `height` tall, centred on `anchor` with its baseline
/// there. Bottom-side text is mirrored so it reads from below.
pub fn text_strokes(
    text: &str,
    anchor: Point,
    height: i32,
    mirrored: bool,
    unsupported: &mut BTreeSet<char>,
) -> Vec<Vec<Point>> {
    let unit = height / 6;
    let count = text.chars().count().min(i32::MAX as usize / 5) as i32;
    let width = (count * 5 - 1).max(0) * unit;
    let mut strokes = vec![];
    for (i, c) in text.chars().enumerate() {
        let c = c.to_ascii_uppercase();
        if c == ' ' {
            continue;
        }
        let shape = glyph(c).unwrap_or_else(|| {
            unsupported.insert(c);
            glyph('?').unwrap_or_default()
        });
        let left = i as i32 * 5;
        for polyline in shape.split(';') {
            strokes.push(
                polyline
                    .split(' ')
                    .filter_map(|pair| {
                        let (x, y) = pair.split_once(',')?;
                        let (x, y) = (x.parse::<i32>().ok()?, y.parse::<i32>().ok()?);
                        let offset = (left + x) * unit - width / 2;
                        let offset = if mirrored { -offset } else { offset };
                        Some(Point::new(anchor.x + offset, anchor.y - y * unit))
                    })
                    .collect(),
            );
        }
    }
    strokes
}
