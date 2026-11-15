//! `BREP_DEBUG_OPERAND_STATE`: the operand-state instrument. A boolean's
//! answer must be a function of its operands' serialized geometry. The caches
//! a surface, curve or thread carries beside that geometry are optimisations,
//! and the same query must read the same with them cold or warm. This
//! instrument replaces the operands at `boolean_pipeline`'s entry so a bisect
//! can find a cache that decides:
//!
//! | value | the operands the boolean sees |
//! | --- | --- |
//! | `clone` | a clone: every cache as it arrived |
//! | `json` | a `serde_json` round trip: caches cold, floats as JSON reads them back |
//! | `codec` | a `solid_codec` round trip: caches cold, floats bit-exact |
//! | `cold:<names>` | a clone with the named surface caches emptied (`analytic`, `validated`, `closed`, `grid`, `dense`, `ring`, `cells`, `all`; `curves` for the curves' validation) |
//! | `json+warm:<names>`, `codec+warm:<names>` | the round trip, with the named caches copied from the live operands |
//! | `tls` | a clone, with this thread's prepared-face caches emptied first |
//!
//! `BREP_DEBUG_OPERAND_STATE_OP` (`union`, `intersect`, `subtract`) limits it to
//! one operation. Every boolean it reaches prints its warm-cache census on
//! stderr. It changes nothing when unset.

use super::BooleanOperation;
use crate::topology::BrepSolid;

pub(super) fn replace_operands(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &super::BooleanOptions,
) -> Option<(BrepSolid, BrepSolid)> {
    let spec = std::env::var("BREP_DEBUG_OPERAND_STATE").ok().filter(|value| !value.is_empty())?;
    if let Ok(only) = std::env::var("BREP_DEBUG_OPERAND_STATE_OP") {
        let name = match operation {
            BooleanOperation::Union => "union",
            BooleanOperation::Intersect => "intersect",
            BooleanOperation::Subtract => "subtract",
        };
        if only != name {
            return None;
        }
    }
    eprintln!("operand-state {operation:?} options: {options:?}");
    for (label, solid) in [("first", first), ("second", second)] {
        eprintln!("operand-state {operation:?} {label}: {}", census(solid));
    }
    if let Ok(directory) = std::env::var("BREP_DEBUG_OPERAND_STATE_DUMP") {
        static SERIAL: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let serial = SERIAL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        for (label, solid) in [("first", first), ("second", second)] {
            let stem = std::path::Path::new(&directory).join(format!("{serial:03}-{label}"));
            let text = serde_json::to_string(solid).expect("operand-state: serialize");
            std::fs::write(stem.with_extension("json"), text).expect("operand-state: write json");
            // Bit-exact beside it: the codec's f64 stream, little-endian, and its names.
            let (data, names) = crate::encode_solid(solid).expect("operand-state: encode");
            let bytes = data.iter().flat_map(|value| value.to_le_bytes()).collect::<Vec<u8>>();
            std::fs::write(stem.with_extension("f64"), bytes).expect("operand-state: write f64");
            std::fs::write(stem.with_extension("names.json"), serde_json::to_string(&names).expect("names"))
                .expect("operand-state: write names");
        }
        eprintln!("operand-state {operation:?}: dumped {serial:03}-first/second to {directory}");
    }
    let (base, names) = match spec.split_once(':') {
        Some((base, names)) => (base, names.split(',').collect::<Vec<_>>()),
        None => (spec.as_str(), Vec::new()),
    };
    let replace = |solid: &BrepSolid| -> BrepSolid {
        match base {
            "clone" | "tls" => solid.clone(),
            "json" | "json+warm" => {
                let text = serde_json::to_string(solid).expect("operand-state: serialize");
                let mut reloaded: BrepSolid = serde_json::from_str(&text).expect("operand-state: deserialize");
                if base == "json+warm" {
                    adopt(&mut reloaded, solid, &names);
                }
                reloaded
            }
            "codec" | "codec+warm" => {
                let (data, solid_names) = crate::encode_solid(solid).expect("operand-state: encode");
                let mut reloaded = crate::decode_solid(&data, &solid_names).expect("operand-state: decode");
                // The codec's constructors run the validation; JSON's do not.
                for_each_surface(&mut reloaded, |surface| surface.exchange_caches(&["validated"], None));
                for_each_curve(&mut reloaded, |curve| curve.set_validated(false));
                if base == "codec+warm" {
                    adopt(&mut reloaded, solid, &names);
                }
                reloaded
            }
            "cold" => {
                let mut cold = solid.clone();
                for_each_surface(&mut cold, |surface| surface.exchange_caches(&names, None));
                if names.contains(&"curves") || names.contains(&"all") {
                    for_each_curve(&mut cold, |curve| curve.set_validated(false));
                }
                cold
            }
            other => panic!("BREP_DEBUG_OPERAND_STATE: unknown state {other:?}"),
        }
    };
    if base == "tls" {
        crate::classification::forget_prepared_faces();
    }
    let replaced = (replace(first), replace(second));
    for (label, solid) in [("first", &replaced.0), ("second", &replaced.1)] {
        eprintln!("operand-state {operation:?} {label} as {spec}: {}", census(solid));
    }
    Some(replaced)
}

fn census(solid: &BrepSolid) -> String {
    let mut counts: Vec<(&'static str, usize)> = Vec::new();
    let mut faces = 0;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        faces += 1;
        for name in face.surface.warm_caches() {
            match counts.iter_mut().find(|(held, _)| *held == name) {
                Some((_, count)) => *count += 1,
                None => counts.push((name, 1)),
            }
        }
    }
    let curves = solid.edges.iter().map(|edge| &edge.curve).chain(
        solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges).map(|coedge| &coedge.pcurve),
    );
    let (validated, total) = curves.fold((0, 0), |(validated, total), curve| {
        (validated + usize::from(curve.is_validated()), total + 1)
    });
    let warm = counts.iter().map(|(name, count)| format!("{name} {count}")).collect::<Vec<_>>().join(", ");
    format!("{faces} faces, warm surface caches [{warm}], curves validated {validated}/{total}")
}

/// Copies the named caches from `live` onto `reloaded`, face by face and
/// curve by curve in the same order (a round trip keeps both).
fn adopt(reloaded: &mut BrepSolid, live: &BrepSolid, names: &[&str]) {
    let live_faces = live.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    let mut index = 0;
    for_each_surface(reloaded, |surface| {
        surface.exchange_caches(names, Some(&live_faces[index].surface));
        index += 1;
    });
    if names.contains(&"curves") || names.contains(&"all") {
        let live_curves = live.edges.iter().map(|edge| edge.curve.is_validated()).chain(
            live.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops)
                .flat_map(|loop_record| &loop_record.coedges).map(|coedge| coedge.pcurve.is_validated()),
        ).collect::<Vec<_>>();
        let mut index = 0;
        for_each_curve(reloaded, |curve| {
            curve.set_validated(live_curves[index]);
            index += 1;
        });
    }
}

fn for_each_surface(solid: &mut BrepSolid, mut visit: impl FnMut(&mut crate::NurbsSurface)) {
    for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        visit(&mut face.surface);
    }
}

fn for_each_curve(solid: &mut BrepSolid, mut visit: impl FnMut(&mut crate::NurbsCurve)) {
    for edge in &mut solid.edges {
        visit(&mut edge.curve);
    }
    for coedge in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces)
        .flat_map(|face| &mut face.loops).flat_map(|loop_record| &mut loop_record.coedges)
    {
        visit(&mut coedge.pcurve);
    }
}
