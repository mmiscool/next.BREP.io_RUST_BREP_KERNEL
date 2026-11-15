//! Face orientation of shells and solids: flip, and orient an open solid consistently.

use crate::topology::*;
use crate::*;


/// Normalize fragment orientation before tracing the open boundary. Selection
/// can combine independently generated carrier fragments whose `same_sense`
/// flags are individually correct but whose shared coedges use the same
/// direction. Propagate a consistent orientation over every connected shell.
pub fn orient_open_solid_faces(solid: &mut BrepSolid) -> Result<(), KernelRefusal> {
    let faces = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let mut edge_uses = HashMap::<u64, Vec<(usize, bool)>>::default();
    for (face_index, face) in faces.iter().enumerate() {
        for coedge in face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            edge_uses
                .entry(coedge.edge_id)
                .or_default()
                .push((face_index, coedge.forward));
        }
    }
    let mut adjacency = vec![Vec::<(usize, bool)>::new(); faces.len()];
    // Deterministic constraint order: on a CONSISTENT shell the flip
    // assignment is order-independent, but on a frustrated one (a genuinely
    // broken open shell headed for an honest refusal) the conflict-tolerant
    // BFS keeps whichever spanning constraints it saw first — HashMap order
    // would make the refusal MODE run-to-run nondeterministic.
    let mut ordered_edge_ids = edge_uses.keys().copied().collect::<Vec<_>>();
    ordered_edge_ids.sort_unstable();
    for uses in ordered_edge_ids
        .iter()
        .map(|edge_id| &edge_uses[edge_id])
        .filter(|uses| uses.len() == 2)
    {
        let (first_face, first_forward) = uses[0];
        let (second_face, second_forward) = uses[1];
        if first_face == second_face {
            continue;
        }
        // Equal current directions require exactly one face flip.
        let different_flip = first_forward == second_forward;
        adjacency[first_face].push((second_face, different_flip));
        adjacency[second_face].push((first_face, different_flip));
    }
    let mut flips = vec![None; faces.len()];
    for root in 0..faces.len() {
        if flips[root].is_some() {
            continue;
        }
        flips[root] = Some(false);
        let mut pending = vec![root];
        while let Some(face_index) = pending.pop() {
            let face_flip = flips[face_index].unwrap();
            for &(neighbor, different_flip) in &adjacency[face_index] {
                let required = face_flip ^ different_flip;
                if let Some(existing) = flips[neighbor] {
                    if existing != required {
                        // Redundant constraints can conflict while duplicate
                        // or open boundaries still exist. Keep the spanning
                        // assignment already established; final manifold
                        // validation remains authoritative.
                        continue;
                    }
                } else {
                    flips[neighbor] = Some(required);
                    pending.push(neighbor);
                }
            }
        }
    }
    drop(faces);
    for (face_index, face) in solid
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .enumerate()
    {
        if flips[face_index] != Some(true) {
            continue;
        }
        face.same_sense = !face.same_sense;
        for loop_record in &mut face.loops {
            loop_record.coedges.reverse();
            for coedge in &mut loop_record.coedges {
                coedge.forward = !coedge.forward;
                coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
            }
        }
    }
    Ok(())
}

pub fn flip_face_orientation(face: &mut FaceRecord) -> Result<(), KernelRefusal> {
    face.same_sense = !face.same_sense;
    for loop_record in &mut face.loops {
        loop_record.coedges.reverse();
        for coedge in &mut loop_record.coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
        }
    }
    Ok(())
}

pub fn flip_shell_faces(shell: &mut ShellRecord) -> Result<(), KernelRefusal> {
    for face in &mut shell.faces {
        flip_face_orientation(face)?;
    }
    Ok(())
}

pub fn flip_all_faces(solid: &mut BrepSolid) -> Result<(), KernelRefusal> {
    for shell in &mut solid.shells {
        flip_shell_faces(shell)?;
    }
    Ok(())
}
