//! The seam rejoin (`BREP_FACE_MERGE_RESEAM`): after the face merge, two
//! pieces of one cylinder or frustum carrier whose only shared edge is the
//! carrier's seam are joined into one face
//! ([`crate::face_merge::rejoin_seam_split_pairs`]).
//!
//! Joining renames: the absorbed piece's name (`X_1`) leaves the body, and
//! the derived edge names (`{faceA}|{faceB}[n]`, numbered per face pair in
//! encounter order) of every edge that bordered it, and possibly of the
//! surviving piece's own edges, change. So the join runs only when the
//! caller lists what later features reference
//! ([`super::BooleanOptions::seam_rejoin_later_references`]), and the body
//! as merged is kept whenever one of those references names something on it
//! that it would not name the same on the joined body: a face reference to
//! either piece, or a face or edge name that resolves to a different set of
//! edges, or to nothing. A reference that does not resolve on the body as
//! merged is not read.
use super::BooleanOptions;
use crate::face_names::stamp_derived_edge_names;
use crate::topology::BrepSolid;
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

/// `merged` joined across its seam splits, or `merged` itself when the
/// rejoin is off, the caller cannot say what follows, nothing joins, or a
/// later reference would read the joined body differently.
pub(super) fn rejoin_unless_referenced(merged: BrepSolid, options: &BooleanOptions, tolerance: f64) -> BrepSolid {
    let Some(later) = &options.seam_rejoin_later_references else { return merged };
    if merged.kind == crate::BodyKind::Cellular || !crate::face_merge::seam_reseam_enabled() {
        return merged;
    }
    let Ok(Some((joined, joins))) =
        crate::face_merge::rejoin_seam_split_pairs(&merged, tolerance, &options.keep_unmerged_name_substrs)
    else {
        return merged;
    };
    let Ok(joined) = crate::merge_curve_continuation_edges(&joined, tolerance) else { return merged };
    if !joined.validate().is_empty() {
        return merged;
    }
    let pieces: HashSet<u64> = joins.iter().flat_map(|&(survivor, absorbed)| [survivor, absorbed]).collect();
    let before = Names::of(&merged);
    let after = Names::of(&joined);
    // The ABSORBED pieces' names (the names the join removes), as a reference
    // spells a face inside an edge or container name.
    let absorbed: HashSet<u64> = joins.iter().map(|&(_, absorbed)| absorbed).collect();
    let absorbed_names: Vec<&str> = before
        .faces
        .iter()
        .filter(|(_, id)| absorbed.contains(id))
        .map(|(name, _)| name.as_str())
        .collect();
    let debug = std::env::var("BREP_DEBUG_MERGE").is_ok();
    for reference in later {
        let reference = reference.trim();
        let Some(read) = before.resolve(reference) else {
            // A reference this body does not name directly may still name it
            // through a container the feature publishes over these names (a
            // swept cap's `{base}_START`, its edges' container spelling), which
            // this reading cannot resolve. One that spells an ABSORBED piece's
            // name keeps the body as merged: the join removes that name. One
            // that spells only the surviving piece names nothing on this body
            // directly (a fillet saved against the joined wall), and the join
            // is what makes it resolve.
            if absorbed_names.iter().any(|name| spells(reference, name)) {
                if debug {
                    eprintln!("seam rejoin: kept the body as merged, a later reference `{reference}` spells an absorbed piece it cannot read");
                }
                return merged;
            }
            continue;
        };
        let kept = match &read {
            Reading::Face(id) => !pieces.contains(id),
            Reading::Edges(_) => after.resolve(reference).as_ref() == Some(&read),
        };
        if !kept {
            if debug {
                eprintln!("seam rejoin: kept the body as merged, a later reference `{reference}` reads it differently");
            }
            return merged;
        }
    }
    if debug {
        eprintln!("seam rejoin: joined {} pair(s) {joins:?}", joins.len());
    }
    joined
}

/// Whether `reference` spells `name` as a whole face name: bounded by the
/// reference's ends, `|` (an edge name's face separator) or `[` (its index),
/// so `P_S` is not read inside `P_S_1`.
fn spells(reference: &str, name: &str) -> bool {
    if name.is_empty() {
        return false;
    }
    reference.match_indices(name).any(|(at, _)| {
        let before = reference[..at].chars().last();
        let after = reference[at + name.len()..].chars().next();
        matches!(before, None | Some('|')) && matches!(after, None | Some('|') | Some('['))
    })
}

#[derive(Debug, PartialEq)]
enum Reading {
    Face(u64),
    /// Sorted edge ids.
    Edges(Vec<u64>),
}

/// A body's face names and its derived edge names, stamped as the feature
/// pipeline stamps them.
struct Names {
    faces: HashMap<String, u64>,
    edges: HashMap<String, u64>,
}

impl Names {
    fn of(solid: &BrepSolid) -> Self {
        let mut stamped = solid.clone();
        stamp_derived_edge_names(&mut stamped);
        let faces = stamped
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter_map(|face| face.name.clone().map(|name| (name, face.id)))
            .collect();
        let edges = stamped
            .edges
            .iter()
            .filter_map(|edge| edge.name.clone().map(|name| (name, edge.id)))
            .collect();
        Self { faces, edges }
    }

    /// What `reference` names here, read the ways the scene reads a name: a
    /// face, an edge, an edge with its two faces the other way round, or an
    /// un-numbered face pair standing for every edge between the two.
    fn resolve(&self, reference: &str) -> Option<Reading> {
        if let Some(&face) = self.faces.get(reference) {
            return Some(Reading::Face(face));
        }
        if let Some(&edge) = self.edges.get(reference) {
            return Some(Reading::Edges(vec![edge]));
        }
        let (base, index) = match reference.rfind('[') {
            Some(at) if reference.ends_with(']') && reference[at + 1..reference.len() - 1].bytes().all(|b| b.is_ascii_digit()) => {
                (&reference[..at], &reference[at..])
            }
            _ => (reference, ""),
        };
        let mut found: Vec<u64> = Vec::new();
        for (at, _) in base.match_indices('|') {
            let (left, right) = (&base[..at], &base[at + 1..]);
            if left.is_empty() || right.is_empty() {
                continue;
            }
            if index.is_empty() {
                for pair in [format!("{left}|{right}["), format!("{right}|{left}[")] {
                    found.extend(self.edges.iter().filter(|(name, _)| name.starts_with(&pair)).map(|(_, &id)| id));
                }
            } else if let Some(&edge) = self.edges.get(&format!("{right}|{left}{index}")) {
                found.push(edge);
            }
        }
        found.sort_unstable();
        found.dedup();
        (!found.is_empty()).then_some(Reading::Edges(found))
    }
}

