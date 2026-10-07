use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use super::*;
use super::builder::{face_name_in_solids, snap_edge_curve_endpoints, source_edge, AssembleIndex};
use super::edge_conform::{conform_overlapping_one_use_edges, edge_lies_on};
use super::refusal_welds::{refresh_derived_genus, weld_identical_coincident_one_use_edges, weld_tangency_duplicate_vertices};

fn assemble_fragments_impl(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
    allow_open: bool,
) -> Result<BrepSolid, KernelRefusal> {
    if selected.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::ConservativeEmptyOverlap,
            KernelStage::Sew,
            "boolean assembly: operation produced no boundary faces",
        ));
    }
    let mut assembler = Assembler {
        tolerance,
        vertices: Vec::new(),
        vertex_aliases: HashMap::default(),
        edges: Vec::new(),
        edge_use_counts: HashMap::default(),
        edge_first_forward: HashMap::default(),
        edge_boundary_key: HashMap::default(),
            imprint_edges: Default::default(),
        next_vertex_id: 1,
        next_edge_id: 1,
        next_coedge_id: 1,
        next_loop_id: 1,
        next_face_id: 1,
    };
    let mut faces = Vec::new();
    let mut used_face_names = HashSet::<String>::default();
    let profile = std::env::var("BREP_PROFILE").is_ok();
    let stage_started = Instant::now();
    // id-index over the immutable operands + imprint, built once. Replaces the
    // per-coedge linear scans of solid.edges/vertices/faces during assembly.
    let index = AssembleIndex::build(solids, imprint);
    let dbg_asm = std::env::var("BREP_DEBUG_ASM_EDGES").is_ok();
    let mut dbg_edge_src: HashMap<u64, Vec<String>> = HashMap::default();
    for fragment in selected {
        let dbg_operand = fragment.operand;
        let dbg_face = fragment.source_face_id;
        let mut loops = Vec::new();
        for fragment_loop in fragment.loops {
            let mut coedges = Vec::new();
            for usage in fragment_loop.coedges {
                let source = source_edge(&usage.source, &index)?;
                let dbg_desc = if dbg_asm {
                    Some(format!(
                        "{:?} bkey={:?} name={:?} s=({:.5},{:.5},{:.5}) e=({:.5},{:.5},{:.5}) face={}:{}",
                        std::mem::discriminant(&usage.source),
                        source.boundary_key,
                        source.name,
                        source.start.x, source.start.y, source.start.z,
                        source.end.x, source.end.y, source.end.z,
                        dbg_operand, dbg_face,
                    ))
                } else {
                    None
                };
                let (edge_id, reversed) = assembler.edge(source, usage.forward)?;
                if let Some(desc) = dbg_desc {
                    dbg_edge_src.entry(edge_id).or_default().push(desc);
                }
                coedges.push(CoedgeRecord {
                    id: assembler.next_coedge_id,
                    edge_id,
                    forward: if reversed {
                        !usage.forward
                    } else {
                        usage.forward
                    },
                    pcurve: usage.pcurve,
                });
                assembler.next_coedge_id += 1;
            }
            loops.push(LoopRecord {
                id: assembler.next_loop_id,
                coedges,
            });
            assembler.next_loop_id += 1;
        }
        // The first fragment of a source face keeps its name; later
        // fragments of the same face (splits) get deterministic `_1`, `_2`
        // suffixes, mirroring the application's split convention.
        let name =
            face_name_in_solids(&index, fragment.operand, fragment.source_face_id).map(|base| {
                if used_face_names.insert(base.clone()) {
                    base
                } else {
                    let mut suffix = 1usize;
                    loop {
                        let candidate = format!("{base}_{suffix}");
                        if used_face_names.insert(candidate.clone()) {
                            break candidate;
                        }
                        suffix += 1;
                    }
                }
            });
        faces.push(FaceRecord {
            id: assembler.next_face_id,
            surface: fragment.surface,
            same_sense: fragment.same_sense,
            loops,
            name,
        });
        assembler.next_face_id += 1;
    }
    if profile {
        eprintln!(
            "assemble.weld_ms={:.2} edges={} vertices={}",
            stage_started.elapsed().as_secs_f64() * 1_000.0,
            assembler.edges.len(),
            assembler.vertices.len()
        );
    }
    if dbg_asm {
        let only_oneuse = std::env::var("BREP_DEBUG_ASM_EDGES").as_deref() != Ok("all");
        for edge in &assembler.edges {
            let uses = assembler.edge_use_counts.get(&edge.id).copied().unwrap_or(0);
            if only_oneuse && uses > 1 {
                continue;
            }
            eprintln!("ASM edge {} uses={} sources:", edge.id, uses);
            for desc in dbg_edge_src.get(&edge.id).into_iter().flatten() {
                eprintln!("    {desc}");
            }
        }
    }
    let stage_started = Instant::now();
    conform_overlapping_one_use_edges(&mut assembler, &mut faces)?;
    // A closed result must have NO one-use edges; any left are duplicated shared
    // boundaries that failed to weld. Fuse identical coincident duplicates so the
    // shell closes. Skipped for intentionally-open sheets (their boundary edges
    // are legitimately one-use).
    if !allow_open {
        weld_identical_coincident_one_use_edges(&mut assembler, &mut faces)?;
    }
    if profile {
        eprintln!(
            "assemble.conform_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    let stage_started = Instant::now();

    let mut parent = (0..faces.len()).collect::<Vec<_>>();
    fn find(parent: &mut [usize], mut index: usize) -> usize {
        while parent[index] != index {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        index
    }
    let mut edge_faces: HashMap<u64, Vec<usize>> = HashMap::default();
    for (face_index, face) in faces.iter().enumerate() {
        for coedge in face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            edge_faces
                .entry(coedge.edge_id)
                .or_default()
                .push(face_index);
        }
    }
    for connected in edge_faces.values() {
        for pair in connected.windows(2) {
            let a = find(&mut parent, pair[0]);
            let b = find(&mut parent, pair[1]);
            if a != b {
                parent[a] = b;
            }
        }
    }
    let mut groups: HashMap<usize, Vec<FaceRecord>> = HashMap::default();
    for (index, face) in faces.into_iter().enumerate() {
        let root = find(&mut parent, index);
        groups.entry(root).or_default().push(face);
    }
    let mut shells = groups
        .into_values()
        .enumerate()
        .map(|(index, faces)| ShellRecord {
            id: index as u64 + 1,
            faces,
        })
        .collect::<Vec<_>>();
    let used_edges = edge_faces.keys().copied().collect::<HashSet<_>>();
    assembler.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices = assembler
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    assembler
        .vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));
    if profile {
        eprintln!(
            "assemble.shells_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    if allow_open {
        let mut solid = BrepSolid {
            mass_properties_cache: Default::default(),
            id: 1,
            vertices: assembler.vertices,
            edges: assembler.edges,
            shells,
            genus: 0,
        };
        // EDGE RECONSTRUCTION on the open path (`edges.rs`): the offset
        // shell's welded edges stand off their carriers from here on
        // (2026-09-30 record §8: arc 40 3.2e-6 off the offset ball while its
        // trims sit on it). Vertices and trims are untouched, so no mass
        // property moves; an open sheet has no validate() gate to re-run and
        // the pass's own bounds decide each edge.
        let stage_started = Instant::now();
        let settled = settle_vertices_onto_carriers(&mut solid, tolerance, &assembler.imprint_edges)?;
        let rebuilt = reconstruct_edges_from_carriers(&mut solid, tolerance)?;
        let trims = close_trim_loop_joints(&mut solid, tolerance)?;
        if profile {
            eprintln!(
                "assemble.edges_ms={:.2} settled={} rebuilt={} trims={}",
                stage_started.elapsed().as_secs_f64() * 1_000.0,
                settled.len(),
                rebuilt.len(),
                trims.len()
            );
        }
        return Ok(solid);
    }
    let face_count: i64 = shells.iter().map(|shell| shell.faces.len() as i64).sum();
    let hole_count: i64 = shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.loops.len().saturating_sub(1) as i64)
        .sum();
    let mut edge_count = assembler
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .count() as i64;
    let euler = assembler.vertices.len() as i64 - edge_count + face_count - hole_count;
    let mut numerator = shells.len() as i64 * 2 - euler;
    if numerator < 0 || numerator % 2 != 0 {
        // TANGENCY DUPLICATE-VERTEX WELD (repair-not-refuse): where two blend
        // surfaces meet TANGENTIALLY (e.g. adjacent fillet cylinders sharing a
        // tangent plane at a chain corner), the marched SSI seam endpoint
        // converges only to O(√tol) — it lands a few 1e-5 from the operand's
        // exact tangent vertex, past the assembler's weld band, leaving one
        // extra vertex and a non-integral genus.  Weld nearest not-edge-
        // connected vertex pairs (greedy, deficit-gated: stop the moment the
        // genus is integral) and snap the drifted curve endpoints onto the
        // surviving vertex.  Only ever runs on an assembly that would refuse.
        let mut welded = 0usize;
        // Deficit gate: weld ONE nearest pair at a time and recheck the genus,
        // stopping the moment it is integral and non-negative. Welding the whole
        // greedy pair list in one shot overshoots (each vertex weld shifts the
        // numerator by +1), so a -1 deficit with two duplicate vertices lands at
        // +1 (non-integral again) instead of stopping at 0.
        while numerator < 0 || numerator % 2 != 0 {
            let step = weld_tangency_duplicate_vertices(
                &mut assembler.vertices,
                &mut assembler.edges,
                tolerance,
                1,
            );
            if step == 0 {
                break;
            }
            welded += step;
            let edge_count = assembler
                .edges
                .iter()
                .filter(|edge| !edge.degenerate)
                .count() as i64;
            let euler = assembler.vertices.len() as i64 - edge_count + face_count - hole_count;
            numerator = shells.len() as i64 * 2 - euler;
        }
        if welded > 0 && std::env::var("BREP_DEBUG_BOOL").is_ok() {
            eprintln!(
                "boolean assembly welded {welded} tangency duplicate vertices (numerator now {numerator})"
            );
        }
    }
    if numerator < 0 || numerator % 2 != 0 {
        // EDGE-CONFORMANCE REPAIR (sibling of the vertex weld above): where the
        // arrangement emitted the SAME boundary as two hairline-separated
        // one-use edge records, merge them; see
        // `conform_unmatched_one_use_edges`.  Only ever runs on an assembly
        // that would refuse.
        let conformed = conform_unmatched_one_use_edges(
            &mut assembler.vertices,
            &mut assembler.edges,
            &mut shells,
            tolerance,
        )?;
        if conformed > 0 {
            CONFORMANCE_REPAIRS.with(|count| count.set(count.get() + conformed as u64));
            edge_count = assembler
                .edges
                .iter()
                .filter(|edge| !edge.degenerate)
                .count() as i64;
            let euler = assembler.vertices.len() as i64 - edge_count + face_count - hole_count;
            numerator = shells.len() as i64 * 2 - euler;
            // The refusal diagnostics below read `edge_faces`; refresh it after
            // the merge (face records keep id == index + 1).
            edge_faces.clear();
            for shell in &shells {
                for face in &shell.faces {
                    for coedge in face
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                    {
                        edge_faces
                            .entry(coedge.edge_id)
                            .or_default()
                            .push(face.id as usize - 1);
                    }
                }
            }
            if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                eprintln!(
                    "boolean assembly conformed {conformed} unmatched one-use edges (numerator now {numerator})"
                );
            }
        }
    }
    if numerator < 0 || numerator % 2 != 0 {
        // Roadmap T4.4: before REFUSING a closed assembly whose one-use edges
        // (open boundaries from intersection-endpoint drift) knock the genus
        // off an integer, offer the same heal chain the offset pipeline runs.
        // This is reached ONLY when `!allow_open` (the open path returned
        // early above), so it never touches a currently succeeding boolean.
        if !allow_open {
            let provisional = BrepSolid {
                mass_properties_cache: Default::default(),
                id: 1,
                vertices: assembler.vertices.clone(),
                edges: assembler.edges.clone(),
                shells: shells.clone(),
                genus: 0,
            };
            if let Some(repaired) = repair_open_assembly_via_heal_chain(provisional, tolerance) {
                if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                    let one_use_before =
                        edge_faces.values().filter(|faces| faces.len() == 1).count();
                    eprintln!(
                        "boolean assembly repaired {one_use_before} one-use edges via heal chain"
                    );
                }
                return Ok(repaired);
            }
        }
        let one_use = edge_faces
            .iter()
            .filter(|(_, faces)| faces.len() == 1)
            .map(|(edge_id, _)| *edge_id)
            .collect::<Vec<_>>();
        let overused = edge_faces
            .iter()
            .filter(|(_, faces)| faces.len() > 2)
            .map(|(edge_id, faces)| (*edge_id, faces.len()))
            .collect::<Vec<_>>();
        let mut one_use_by_face = HashMap::<usize, usize>::default();
        for faces in edge_faces.values().filter(|faces| faces.len() == 1) {
            *one_use_by_face.entry(faces[0] + 1).or_default() += 1;
        }
        let shell_faces = shells
            .iter()
            .map(|shell| shell.faces.iter().map(|face| face.id).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let one_use_geometry = one_use
            .iter()
            .filter_map(|edge_id| {
                let edge = assembler.edges.iter().find(|edge| edge.id == *edge_id)?;
                let start = assembler
                    .vertices
                    .iter()
                    .find(|vertex| vertex.id == edge.start_vertex_id)?
                    .point;
                let end = assembler
                    .vertices
                    .iter()
                    .find(|vertex| vertex.id == edge.end_vertex_id)?
                    .point;
                let middle = edge
                    .curve
                    .evaluate((edge.t0 + edge.t1) / 2.0)
                    .unwrap_or(start);
                Some((
                    *edge_id,
                    edge.degenerate,
                    edge.curve.degree,
                    start,
                    middle,
                    end,
                ))
            })
            .take(40)
            .collect::<Vec<_>>();
        let mut geometric_matches = Vec::new();
        for (index, first_id) in one_use.iter().enumerate() {
            let first = assembler
                .edges
                .iter()
                .find(|edge| edge.id == *first_id)
                .unwrap();
            for second_id in one_use.iter().skip(index + 1) {
                let second = assembler
                    .edges
                    .iter()
                    .find(|edge| edge.id == *second_id)
                    .unwrap();
                if edge_lies_on(first, second, 2e-3).unwrap_or(false)
                    || edge_lies_on(second, first, 2e-3).unwrap_or(false)
                {
                    geometric_matches.push((*first_id, *second_id));
                }
            }
        }
        return Err(KernelRefusal::new(
            RefusalClass::NonIntegralGenus {
                shells: shells.len() as u32,
                euler: assembler.vertices.len() as i64 - edge_count as i64 + face_count as i64
                    - hole_count as i64,
            },
            KernelStage::Validate,
            format!(
            "boolean assembly: non-integral genus \
             (V={} E={} F={} H={} S={} shell_faces={:?} \
             one_use={} {:?} one_use_geometry={one_use_geometry:?} \
             geometric_matches={geometric_matches:?} \
             one_use_by_face={:?} overused={} {:?}); on faces [{}]",
            assembler.vertices.len(),
            edge_count,
            face_count,
            hole_count,
            shells.len(),
            shell_faces,
            one_use.len(),
            one_use.iter().take(24).collect::<Vec<_>>(),
            one_use_by_face,
            overused.len(),
            overused.iter().take(24).collect::<Vec<_>>(),
            {
                // The same faces by name (face records keep id == index + 1).
                let mut named = one_use_by_face.iter().collect::<Vec<_>>();
                named.sort();
                named
                    .into_iter()
                    .map(|(id, count)| {
                        let name = shells
                            .iter()
                            .flat_map(|shell| &shell.faces)
                            .find(|face| face.id == *id as u64)
                            .and_then(|face| face.name.clone())
                            .unwrap_or_default();
                        format!("{id} '{name}' x{count}")
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            },
        )));
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: 1,
        vertices: assembler.vertices,
        edges: assembler.edges,
        shells,
        genus: numerator / 2,
    };
    let stage_started = Instant::now();
    let mut solid = merge_curve_continuation_edges(&solid, tolerance)?;
    if profile {
        eprintln!(
            "assemble.continuation_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    let stage_started = Instant::now();
    let polished = polish_triple_junction_vertices(&mut solid, tolerance)?;
    if profile {
        eprintln!(
            "assemble.junction_ms={:.2} polished={polished}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    // Genus is derived bookkeeping and every post-assembly pass (edge
    // continuation merges, junction polish) shifts the V/E counts it came
    // from; a stale value makes the Euler validation reject a perfectly
    // manifold result (two touching shells: assembly-time chi drifted while
    // genus kept its old value). Recompute at the validation boundary with
    // the same shell-aware formula validate() checks against.
    refresh_derived_genus(&mut solid);
    let stage_started = Instant::now();
    let mut issues = solid.validate();
    if profile {
        eprintln!(
            "assemble.validate_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    if !issues.is_empty() {
        // EDGE-CONFORMANCE RETRY at the validation boundary: the overshoot
        // lanes can leave a hairline one-use sliver bridging an OPEN loop gap
        // (the neighbour's arrangement split at the drifted SSI endpoint, this
        // face's at the exact station) that only surfaces here.  The pass runs
        // exclusively on this refusal path and the re-validate below still
        // gates the result.
        let conformed = conform_unmatched_one_use_edges(
            &mut solid.vertices,
            &mut solid.edges,
            &mut solid.shells,
            tolerance,
        )?;
        if conformed > 0 {
            CONFORMANCE_REPAIRS.with(|count| count.set(count.get() + conformed as u64));
            refresh_derived_genus(&mut solid);
            issues = solid.validate();
            if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                eprintln!(
                    "boolean assembly conformed {conformed} unmatched one-use edges at validation ({} issues remain)",
                    issues.len()
                );
            }
        }
    }
    if !issues.is_empty()
        && issues.iter().all(|issue| {
            issue.severity == "error"
                && matches!(issue.kind, crate::IssueKind::CurveVertexGap { .. })
        })
    {
        // GEOMETRIC ENDPOINT-GAP REPAIR (repair-not-refuse): a TOPOLOGICALLY
        // CLOSED assembly (no open/over-used edges, integral genus) can still
        // carry an edge whose CURVE endpoint lands a couple 1e-4 off its
        // assigned vertex — an SSI march that reached the triple point only to
        // O(1e-4) on imported geometry, where the vertex is the authoritative
        // datum (a shared operand corner) and the curve end is the drifted one.
        // When EVERY remaining issue is such a curve<->vertex gap within a
        // commit-scale band, pin each offending edge's curve endpoint onto its
        // vertex via `snap_edge_curve_endpoints`. Purely geometric: vertex
        // positions and topology never change, and the re-validate below still
        // gates the result, so a mixed or larger issue set refuses exactly as
        // before.
        let gap_band = commit_weld(tolerance) * 10.0;
        let vertex_points = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        let mut snapped_any = false;
        for edge in &mut solid.edges {
            if edge.degenerate {
                continue;
            }
            let (Some(&start), Some(&end)) = (
                vertex_points.get(&edge.start_vertex_id),
                vertex_points.get(&edge.end_vertex_id),
            ) else {
                continue;
            };
            let start_gap = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?.sub(start).length();
            let end_gap = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?.sub(end).length();
            if start_gap <= 1e-9 && end_gap <= 1e-9 {
                continue;
            }
            if start_gap > gap_band || end_gap > gap_band {
                continue;
            }
            let curve = snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1, start, end)?;
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            edge.curve = curve;
            edge.t0 = t0;
            edge.t1 = t1;
            snapped_any = true;
        }
        if snapped_any {
            refresh_derived_genus(&mut solid);
            issues = solid.validate();
            if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                eprintln!(
                    "boolean assembly snapped curve endpoints onto vertices ({} issues remain)",
                    issues.len()
                );
            }
        }
    }
    if issues.is_empty() {
        // EDGE RECONSTRUCTION (repair-not-refuse; `edges.rs`): a two-use edge
        // that stands off one of its carriers is replaced by the exact image
        // of a planar use's trim with its ends pinned to the vertices.
        // validate() holds edges to a 4e-3 contract and never sees these
        // standoffs, so this runs on the validated result; a set the
        // validator then disagrees with is put back whole.
        let stage_started = Instant::now();
        let settled = settle_vertices_onto_carriers(&mut solid, tolerance, &assembler.imprint_edges)?;
        let rebuilt = reconstruct_edges_from_carriers(&mut solid, tolerance)?;
        let trims = close_trim_loop_joints(&mut solid, tolerance)?;
        // The DERIVED inherited-trim challenger (`inherited.rs`): output uses
        // of operand edges only, restored first if the stage goes back.
        let inherited = super::inherited::challenge_inherited_trims(&mut solid, tolerance, &assembler.edge_boundary_key, &assembler.imprint_edges)?;
        if !rebuilt.is_empty() || !settled.is_empty() || !trims.is_empty() || !inherited.is_empty() {
            let after = solid.validate();
            let count = rebuilt.len();
            if after.is_empty() {
                if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                    eprintln!("boolean assembly settled {} vertices and reconstructed {count} edges from their planar trims; {} inherited trims refit", settled.len(), inherited.len());
                }
            } else {
                restore_trims(&mut solid, inherited);
                restore_trims(&mut solid, trims);
                restore_edges(&mut solid, rebuilt);
                restore_vertices(&mut solid, settled);
                if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                    eprintln!(
                        "boolean assembly restored {count} reconstructed edges: validate() reported {} issues on them",
                        after.len()
                    );
                }
            }
        }
        if profile {
            eprintln!(
                "assemble.edges_ms={:.2}",
                stage_started.elapsed().as_secs_f64() * 1_000.0
            );
        }
    }
    if !issues.is_empty() {
        let mut use_counts = HashMap::<u64, usize>::default();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *use_counts.entry(coedge.edge_id).or_default() += 1;
        }
        let open_edges = solid
            .edges
            .iter()
            .filter(|edge| use_counts.get(&edge.id).copied().unwrap_or(0) == 1)
            .map(|edge| {
                let start = solid
                    .vertices
                    .iter()
                    .find(|vertex| vertex.id == edge.start_vertex_id)
                    .map(|vertex| vertex.point);
                let end = solid
                    .vertices
                    .iter()
                    .find(|vertex| vertex.id == edge.end_vertex_id)
                    .map(|vertex| vertex.point);
                format!("{}:{start:?}->{end:?}", edge.id)
            })
            .collect::<Vec<_>>();
        if std::env::var("BREP_DEBUG_VALIDATE").is_ok() {
            for issue in &issues {
                if let Some(rest) = issue.message.strip_prefix("coedge ") {
                    if let Some(edge_part) = rest.split("edge ").nth(1) {
                        if let Some(edge_id) = edge_part
                            .split_whitespace()
                            .next()
                            .and_then(|token| token.parse::<u64>().ok())
                        {
                            if let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) {
                                let [d0, d1] = edge.curve.domain().unwrap_or([0.0, 0.0]);
                                eprintln!(
                                    "VALIDATE edge {}: name={:?} degen={} t=[{:.6},{:.6}] domain=[{:.6},{:.6}] cps={}",
                                    edge.id,
                                    edge.name,
                                    edge.degenerate,
                                    edge.t0,
                                    edge.t1,
                                    d0,
                                    d1,
                                    edge.curve.control_points.len()
                                );
                                for fraction in [0.0f64, 0.5, 1.0] {
                                    if let Ok(point) = edge
                                        .curve
                                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                                    {
                                        eprintln!(
                                            "  edge point {:.2}: ({:.6},{:.6},{:.6})",
                                            fraction, point.x, point.y, point.z
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // The faces holding the open edges, by name: a name is its source
        // face's, so the refusal says which operand faces lost a partner — the
        // pair to measure — not only which minted edge ids did.
        let mut open_faces: Vec<(u64, String, usize)> = Vec::new();
        for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
            let count = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter(|coedge| use_counts.get(&coedge.edge_id).copied().unwrap_or(0) == 1)
                .count();
            if count > 0 {
                open_faces.push((face.id, face.name.clone().unwrap_or_default(), count));
            }
        }
        let open_faces = open_faces
            .iter()
            .map(|(id, name, count)| format!("{id} '{name}' x{count}"))
            .collect::<Vec<_>>();
        return Err(KernelRefusal::new(
            RefusalClass::DegenerateArrangement {
                open_edges: open_edges.len() as u32,
                issues: issues.len() as u32,
            },
            KernelStage::Validate,
            format!(
            "boolean assembly produced invalid topology (S={} genus={}): {issues:?}; open edges [{}]; on faces [{}]",
            solid.shells.len(),
            solid.genus,
            open_edges.join(", "),
            open_faces.join(", ")
        )));
    }
    // Exercise exact integration here so a geometrically inverted shell is
    // rejected at the kernel boundary instead of reaching the application.
    // Volume-only: the gate reads nothing but the sign.
    let stage_started = Instant::now();
    let _caller = crate::mass_caller("assemble.mass_check");
    let volume = solid_signed_volume(&solid).or_refuse(KernelStage::Sew, "solid_signed_volume")?;
    if profile {
        eprintln!(
            "assemble.mass_check_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    if volume <= tolerance.powi(3) {
        return Err(KernelRefusal::new(
            RefusalClass::NonPositiveVolume,
            KernelStage::Validate,
            "boolean assembly produced non-positive volume",
        ));
    }
    Ok(solid)
}

pub(crate) fn assemble_fragments(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, false)
}

pub(crate) fn assemble_open_fragments(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, true)
}

