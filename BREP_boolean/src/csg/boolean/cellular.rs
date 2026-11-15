//! Closed operand regions retained at a boundary-only union contact.
use super::*;

pub(super) fn enabled(options: &BooleanOptions) -> bool {
    match std::env::var("BREP_KEEP_CELLS").as_deref() {
        Ok("1") => true, Ok("0") => false, _ => options.keep_cells,
    }
}

/// The contact lane keeps COMPLETE original boundaries. A cut, dropped
/// face, orientation reversal, or foreign boundary belongs to ordinary
/// regularized assembly. The declared kind is never a closure fallback.
pub(super) fn complete_contact(
    first: &BrepSolid,
    second: &BrepSolid,
    selected: &[FaceFragmentRecord],
    tolerance: f64,
) -> bool {
    if [first, second].iter().any(|s| s.kind != crate::BodyKind::Solid) { return false; }
    let band = crate::tolerance::assembler_weld(tolerance);
    let contacts = first.vertices.iter().filter(|a| second.vertices.iter()
        .any(|b| a.point.sub(b.point).length() <= band)).count();
    // Complete boundaries sharing vertices can meet at a vertex or along
    // whole edges. The assembler keeps each region's original edge pair;
    // the Cellular validator certifies the radial ordering of those pairs.
    if contacts == 0 { return false; }
    let mut seen = HashSet::default();
    for fragment in selected {
        if !seen.insert((fragment.operand, fragment.source_face_id)) { return false; }
        let source = match fragment.operand { 0 => first, 1 => second, _ => return false };
        let Some(face) = source.shells.iter().flat_map(|s| &s.faces)
            .find(|f| f.id == fragment.source_face_id) else { return false; };
        if face.same_sense != fragment.same_sense || face.loops.len() != fragment.loops.len() { return false; }
        let mut boundary: HashMap<(u64, bool), usize> = HashMap::default();
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            *boundary.entry((coedge.edge_id, coedge.forward)).or_default() += 1;
        }
        for coedge in fragment.loops.iter().flat_map(|l| &l.coedges) {
            let FragmentEdgeSource::Boundary { operand, edge_id } = &coedge.source else { return false; };
            if *operand != fragment.operand { return false; }
            let Some(count) = boundary.get_mut(&(*edge_id, coedge.forward)) else { return false; };
            if *count == 0 { return false; }
            *count -= 1;
        }
        if boundary.values().any(|count| *count != 0) { return false; }
    }
    seen.len() == first.shells.iter().map(|s| s.faces.len()).sum::<usize>()
        + second.shells.iter().map(|s| s.faces.len()).sum::<usize>()
}
