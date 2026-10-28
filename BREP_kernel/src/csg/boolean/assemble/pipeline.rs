use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use super::*;
use super::builder::{face_name_in_solids, snap_edge_curve_endpoints, source_edge, AssembleIndex};
use super::edge_conform::{conform_overlapping_one_use_edges, edge_lies_on};
use super::refusal_welds::{
    refresh_derived_genus, split_vertices_shared_across_shells, weld_identical_coincident_one_use_edges,
    weld_open_loop_corners, weld_tangency_duplicate_vertices,
};

/// Which result faces the operation TOUCHED, read off the touch census of
/// the fragments they were built from (`touch.rs`).
pub(in crate::boolean) struct AssemblyScope {
    /// Result faces built from a touched fragment.
    pub(in crate::boolean) touched: HashSet<u64>,
    /// The touched faces and every face sharing a vertex with one: the faces
    /// whose geometry the acceptance passes read. A face outside this set is
    /// an operand's face carried through unchanged, with every vertex it
    /// stands on shared only with faces carried through the same way.
    pub(in crate::boolean) examined: HashSet<u64>,
    /// Result face id → the operand its fragment came from.
    pub(in crate::boolean) operand_of: HashMap<u64, u8>,
}

impl AssemblyScope {
    fn build(
        origins: &[(u64, u8, bool)],
        shells: &[ShellRecord],
        edges: &[EdgeRecord],
        aliases: &HashMap<u64, u64>,
    ) -> Self {
        let canonical = |mut id: u64| {
            while let Some(&next) = aliases.get(&id) {
                id = next;
            }
            id
        };
        let edge_vertices: HashMap<u64, (u64, u64)> = edges
            .iter()
            .map(|edge| (edge.id, (canonical(edge.start_vertex_id), canonical(edge.end_vertex_id))))
            .collect();
        let touched: HashSet<u64> = origins
            .iter()
            .filter(|(_, _, touched)| *touched)
            .map(|(face_id, _, _)| *face_id)
            .collect();
        let operand_of: HashMap<u64, u8> =
            origins.iter().map(|(face_id, operand, _)| (*face_id, *operand)).collect();
        let mut vertex_faces: HashMap<u64, Vec<u64>> = HashMap::default();
        let mut face_vertices: HashMap<u64, Vec<u64>> = HashMap::default();
        for face in shells.iter().flat_map(|shell| &shell.faces) {
            for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                let Some(&(start, end)) = edge_vertices.get(&coedge.edge_id) else {
                    continue;
                };
                for vertex in [start, end] {
                    let users = vertex_faces.entry(vertex).or_default();
                    if !users.contains(&face.id) {
                        users.push(face.id);
                    }
                    let owned = face_vertices.entry(face.id).or_default();
                    if !owned.contains(&vertex) {
                        owned.push(vertex);
                    }
                }
            }
        }
        let mut examined = touched.clone();
        for face_id in &touched {
            for vertex in face_vertices.get(face_id).map(Vec::as_slice).unwrap_or(&[]) {
                examined.extend(vertex_faces.get(vertex).into_iter().flatten().copied());
            }
        }
        Self { touched, examined, operand_of }
    }
}

fn assemble_fragments_impl(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
    allow_open: bool,
    touch: Option<&TouchCensus>,
    touched_passes: bool,
    declared_kind: crate::BodyKind,
) -> Result<(BrepSolid, Option<AssemblyScope>), KernelRefusal> {
    if selected.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::ConservativeEmptyOverlap,
            KernelStage::Sew,
            "boolean assembly: operation produced no boundary faces",
        ));
    }
    let selected = if canonical_fragment_order_enabled() { crate::fragment::canonical_fragment_order(selected) } else { selected };
    let mut assembler = Assembler {
        weld_candidates: Default::default(),
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
        edge_by_boundary_key: HashMap::default(),
        edge_tangent_side: HashMap::default(),
        cellular: declared_kind == crate::BodyKind::Cellular,
    };
    let mut faces = Vec::new();
    let mut used_face_names = HashSet::<String>::default();
    let profile = crate::imprint::profile_enabled();
    let stage_started = Instant::now();
    // id-index over the immutable operands + imprint, built once. Replaces the
    // per-coedge linear scans of solid.edges/vertices/faces during assembly.
    let index = AssembleIndex::build(solids, imprint);
    let dbg_asm = std::env::var("BREP_DEBUG_ASM_EDGES").is_ok();
    let mut dbg_edge_src: HashMap<u64, Vec<String>> = HashMap::default();
    // (result face id, operand, touched) per face built, for the scope.
    let mut origins: Vec<(u64, u8, bool)> = Vec::new();
    // The pieces an imprinted TANGENT section was minted as
    // (`TangentSectionRecord::piece_ids`): their coedges carry a material
    // side into the weld (`SourceEdge::tangent_side`).
    let tangent_pieces: HashSet<u64> = imprint
        .tangent_sections
        .iter()
        .flat_map(|section| section.piece_ids.iter().copied())
        .collect();
    for fragment in selected {
        let dbg_operand = fragment.operand;
        let dbg_face = fragment.source_face_id;
        let mut untouched = false;
        if let Some(touch) = touch {
            let touched = touch.is_touched(fragment.operand, fragment.source_face_id);
            origins.push((assembler.next_face_id, fragment.operand, touched));
            untouched = touched_passes && !touched;
        } else if declared_kind == crate::BodyKind::Cellular {
            origins.push((assembler.next_face_id, fragment.operand, true));
        }
        let mut loops = Vec::new();
        for fragment_loop in fragment.loops {
            let mut coedges = Vec::new();
            for usage in fragment_loop.coedges {
                let mut source = source_edge(&usage.source, &index)?;
                if let crate::fragment::FragmentEdgeSource::Imprint { piece_id } = &usage.source {
                    if tangent_pieces.contains(piece_id) {
                        source.tangent_side = tangent_material_side(
                            &source,
                            usage.forward,
                            &usage.pcurve,
                            &fragment.surface,
                            fragment.same_sense,
                        );
                        // The result names its tangent edges: `tangent` after
                        // the section's face-pair name (`a|b|tangent`), or
                        // `tangent` alone on unnamed operands.
                        source.name = Some(match source.name.take() {
                            Some(name) => format!("{name}|{TANGENT_EDGE_NAME}"),
                            None => TANGENT_EDGE_NAME.to_string(),
                        });
                    }
                }
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
                let (edge_id, reversed) = if untouched {
                    assembler.edge_untouched(source, usage.forward)?
                } else {
                    assembler.edge(source, usage.forward)?
                };
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
            "assemble.weld_ms={:.2} edges={} vertices={} candidates={} exhaustive={}",
            stage_started.elapsed().as_secs_f64() * 1_000.0,
            assembler.edges.len(),
            assembler.vertices.len(),
            assembler.weld_candidates.offered, assembler.weld_candidates.exhaustive
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
    // A closed loop that walks from one section curve onto another at an
    // isolated tangent node arrives at a DIFFERENT vertex than it left (the
    // two curves were split at the node by two arrangements, a chord error
    // apart); weld those corners shut. Section-to-section corners only: a
    // gap against an operand edge belongs to the conformance lanes below.
    if !allow_open {
        weld_identical_coincident_one_use_edges(&mut assembler, &mut faces)?;
        // Section edges: the imprint's pieces and the sub-edges the
        // arrangement derived from them — every edge that is not an
        // operand's (those carry a boundary key).
        let section_edges: HashSet<u64> = assembler
            .edges
            .iter()
            .map(|edge| edge.id)
            .filter(|id| !assembler.edge_boundary_key.contains_key(id))
            .collect();
        let welded_corners = weld_open_loop_corners(
            &mut assembler.vertices,
            &mut assembler.edges,
            &faces,
            &section_edges,
            tolerance,
        );
        if !welded_corners.is_empty() && std::env::var("BREP_DEBUG_BOOL").is_ok() {
            eprintln!(
                "boolean assembly welded {} open loop corner(s) at vertices {:?}",
                welded_corners.len(),
                welded_corners
            );
        }
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
    let scope = touch.map(|_| {
        AssemblyScope::build(&origins, &shells, &assembler.edges, &assembler.vertex_aliases)
    });
    // The geometry checks of a scoped validation read the examined faces
    // only; every structural check still reads the whole solid.
    let validate_in = |solid: &BrepSolid| {
        if profile {
            eprintln!("assemble.validate_reads faces={} examined={}", solid.shells.iter().map(|s| s.faces.len()).sum::<usize>(),
                scope.as_ref().map_or_else(|| solid.shells.iter().map(|s| s.faces.len()).sum::<usize>(), |s| s.examined.len()));
        }
        match &scope {
            Some(scope) => solid.validate_scoped(&scope.examined),
            None => solid.validate(),
        }
    };
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
    if allow_open || declared_kind == crate::BodyKind::Cellular {
        let cells = if declared_kind == crate::BodyKind::Cellular {
            let mut operands = solids.keys().copied().collect::<Vec<_>>();
            operands.sort_unstable();
            operands.into_iter().map(|operand| crate::CellRecord {
                id: operand as u64 + 1,
                faces: origins.iter().filter(|(_, source, _)| *source == operand)
                    .map(|(face, _, _)| (*face, true)).collect(),
                void: false,
            }).collect()
        } else { Vec::new() };
        let mut solid = BrepSolid {
            mass_properties_cache: Default::default(),
            cells,
            id: 1,
            vertices: assembler.vertices,
            edges: assembler.edges,
            shells,
            genus: 0,
            kind: declared_kind,
        };
        if declared_kind == crate::BodyKind::Cellular {
            let issues = solid.validate();
            if !issues.is_empty() {
                return Err(KernelRefusal::new(
                    RefusalClass::InvalidResultTopology { issues: issues.len() as u32 },
                    KernelStage::Validate, format!("Cellular contact assembly: {issues:?}")));
            }
            return Ok((solid, None));
        }
        // EDGE RECONSTRUCTION on the open path (`edges.rs`): the offset
        // shell's welded edges stand off their carriers from here on
        // (2026-09-30 record §8: arc 40 3.2e-6 off the offset ball while its
        // trims sit on it). Vertices and trims are untouched, so no mass
        // property moves; an open sheet has no validate() gate to re-run and
        // the pass's own bounds decide each edge.
        let stage_started = Instant::now();
        let settled = settle_vertices_onto_carriers(&mut solid, tolerance, &assembler.imprint_edges)?;
        if profile {
            eprintln!("assemble.open_settle_ms={:.2}", stage_started.elapsed().as_secs_f64() * 1_000.0);
        }
        let rebuilt = reconstruct_edges_from_carriers(&mut solid, tolerance)?;
        if profile {
            eprintln!("assemble.open_reconstruct_ms={:.2}", stage_started.elapsed().as_secs_f64() * 1_000.0);
        }
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
        return Ok((solid, None));
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
                cells: Default::default(),
                id: 1,
                vertices: assembler.vertices.clone(),
                edges: assembler.edges.clone(),
                shells: shells.clone(),
                genus: 0,
                kind: crate::BodyKind::Solid,
            };
            if let Some(repaired) = repair_open_assembly_via_heal_chain(provisional, tolerance) {
                if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                    let one_use_before =
                        edge_faces.values().filter(|faces| faces.len() == 1).count();
                    eprintln!(
                        "boolean assembly repaired {one_use_before} one-use edges via heal chain"
                    );
                }
                // The heal chain rebuilt the body; nothing passes through it.
                return Ok((repaired, None));
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
    // A vertex used by edges of two edge-disconnected shells is a point
    // contact between two bodies (the subtract that cuts a cylinder with an
    // equal-radius crossing cylinder leaves two pieces touching at the two
    // tangent points). Each shell gets its own vertex there: two closed
    // manifold shells, no pinch, and a genus read off each one's own count.
    // Runs after the Euler-gated welds above, whose nearest-pair search would
    // otherwise merge the copies straight back.
    if shells.len() > 1 {
        let split = split_vertices_shared_across_shells(
            &mut assembler.vertices,
            &mut assembler.edges,
            &shells,
            &mut assembler.next_vertex_id,
        );
        if split > 0 {
            edge_count = assembler
                .edges
                .iter()
                .filter(|edge| !edge.degenerate)
                .count() as i64;
            let euler = assembler.vertices.len() as i64 - edge_count + face_count - hole_count;
            numerator = shells.len() as i64 * 2 - euler;
            if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                eprintln!(
                    "boolean assembly split {split} welded node vertex/vertices between shells (numerator now {numerator})"
                );
            }
        }
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: 1,
        vertices: assembler.vertices,
        edges: assembler.edges,
        shells,
        genus: numerator / 2,
        kind: crate::BodyKind::Solid,
    };
    let (touched_faces, examined_faces) = match (&scope, touched_passes) {
        (Some(scope), true) => (Some(&scope.touched), Some(&scope.examined)),
        _ => (None, None),
    };
    let touched_vertices: Option<HashSet<u64>> = touched_faces.map(|faces| {
        let edge_vertices: HashMap<u64, (u64, u64)> = solid
            .edges
            .iter()
            .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
            .collect();
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| faces.contains(&face.id))
            .flat_map(|face| face.loops.iter().flat_map(|loop_record| &loop_record.coedges))
            .filter_map(|coedge| edge_vertices.get(&coedge.edge_id))
            .flat_map(|&(start, end)| [start, end])
            .collect()
    });
    let stage_started = Instant::now();
    let mut solid = match &touched_vertices {
        Some(vertices) => crate::coalesce::merge_curve_continuation_edges_at(&solid, tolerance, vertices)?,
        None => merge_curve_continuation_edges(&solid, tolerance)?,
    };
    if profile {
        eprintln!(
            "assemble.continuation_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    let stage_started = Instant::now();
    let polished = polish_triple_junction_vertices_in(&mut solid, tolerance, touched_vertices.as_ref())?;
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
    let mut issues = validate_in(&solid);
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
            issues = validate_in(&solid);
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
            issues = validate_in(&solid);
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
        let mut pass_started = Instant::now();
        let mut pass_ms = |label: &str, pass_started: &mut Instant| {
            if profile {
                eprintln!("assemble.edges.{label}_ms={:.2}", pass_started.elapsed().as_secs_f64() * 1_000.0);
                *pass_started = Instant::now();
            }
        };
        let settled = settle_vertices_onto_carriers_in(&mut solid, tolerance, &assembler.imprint_edges, touched_vertices.as_ref())?;
        pass_ms("settle", &mut pass_started);
        let rebuilt = reconstruct_edges_from_carriers_in(&mut solid, tolerance, touched_faces)?;
        pass_ms("reconstruct", &mut pass_started);
        let trims = close_trim_loop_joints_in(&mut solid, tolerance, examined_faces)?;
        pass_ms("joints", &mut pass_started);
        // The DERIVED inherited-trim challenger (`inherited.rs`): output uses
        // of operand edges only, restored first if the stage goes back.
        let separated = super::inherited::separated_faces(&solid, solids, &origins, tolerance);
        let inherited = super::inherited::challenge_inherited_trims_scoped(&mut solid, tolerance, &assembler.edge_boundary_key, &assembler.imprint_edges,
            std::env::var("BREP_INHERITED_TRIM_REFIT").as_deref() != Ok("0"), examined_faces, Some(&separated))?;
        pass_ms("inherited", &mut pass_started);
        if !rebuilt.is_empty() || !settled.is_empty() || !trims.is_empty() || !inherited.is_empty() {
            let after = validate_in(&solid);
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
    // Volume-only: the gate reads nothing but the sign, so it reads
    // `solid_signed_volume_for_sign`, which skips the sliver corrections on
    // chords whose pcurve is straight in uv (a bore's rims: 105 of the plate
    // case's 128 ms assembly were those corrections) and otherwise is the
    // exact integration, including bit for bit whenever the reading sits
    // within its margin of the bar.
    let stage_started = Instant::now();
    let _caller = crate::mass_caller("assemble.mass_check");
    let bar = tolerance.powi(3);
    // A shell with no touched face is an operand's shell as the operand held
    // it; its sign is the operand's and it is left out of the check. With no
    // shell touched (a disjoint union) there is nothing to integrate.
    let volume = match &scope {
        Some(scope) => {
            let touched_shells = solid
                .shells
                .iter()
                .filter(|shell| shell.faces.iter().any(|face| scope.touched.contains(&face.id)))
                .collect::<Vec<_>>();
            if touched_shells.is_empty() {
                None
            } else if touched_shells.len() == solid.shells.len() {
                Some(solid_signed_volume_for_sign(&solid, bar).or_refuse(KernelStage::Sew, "solid_signed_volume")?)
            } else {
                let subset = BrepSolid {
                    mass_properties_cache: Default::default(),
                    cells: Default::default(),
                    id: solid.id,
                    vertices: solid.vertices.clone(),
                    edges: solid.edges.clone(),
                    shells: touched_shells.into_iter().cloned().collect(),
                    genus: 0,
                    kind: crate::BodyKind::Solid,
                };
                Some(solid_signed_volume_for_sign(&subset, bar).or_refuse(KernelStage::Sew, "solid_signed_volume")?)
            }
        }
        None => Some(solid_signed_volume_for_sign(&solid, bar).or_refuse(KernelStage::Sew, "solid_signed_volume")?),
    };
    if profile {
        eprintln!(
            "assemble.mass_check_ms={:.2}",
            stage_started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    if volume.is_some_and(|volume| volume <= bar) {
        return Err(KernelRefusal::new(
            RefusalClass::NonPositiveVolume,
            KernelStage::Validate,
            "boolean assembly produced non-positive volume",
        ));
    }
    Ok((solid, scope))
}

/// `BREP_FRAGMENT_CANONICAL_ORDER` (lane X, fillet-bore-notch): the assembler
/// builds the fragments in [`crate::fragment::canonical_fragment_order`], so the
/// `_1`, `_2` names of a split face's pieces and every id do not follow a tie
/// between near-equal areas. On by default (promoted on the 239-document
/// census: 21 documents changed, every body geometrically identical, every
/// moved fragment swapped with one whose area lies within 4.2e-10 of its own);
/// `0`/`off` disables it.
fn canonical_fragment_order_enabled() -> bool {
    let enabled = !matches!(std::env::var("BREP_FRAGMENT_CANONICAL_ORDER").as_deref(), Ok("0") | Ok("off"));
    enabled
}


pub(crate) fn assemble_fragments(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, false, None, false, crate::BodyKind::Solid).map(|(solid, _)| solid)
}

/// [`assemble_fragments`] reporting which result faces the operation touched
/// (`BooleanOptions::untouched_passthrough`): the acceptance passes inside
/// read the geometry of the touched faces and their neighbours only, and the
/// caller's passes read the returned scope the same way. With
/// `touched_passes` (`BooleanOptions::touched_assembly`) the weld finds an
/// untouched face's mate by key and the settle, reconstruction, joint,
/// inherited-trim, junction and continuation passes read the touched faces
/// (and their neighbours where the pass reaches across an edge) only.
pub(in crate::boolean) fn assemble_fragments_scoped(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
    touch: &TouchCensus,
    touched_passes: bool,
) -> Result<(BrepSolid, Option<AssemblyScope>), KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, false, Some(touch), touched_passes, crate::BodyKind::Solid)
}

pub(crate) fn assemble_open_fragments(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, true, None, false, crate::BodyKind::Solid).map(|(solid, _)| solid)
}

/// The producer declares the Cellular kind before assembly. It is admitted
/// only after the boolean has kept both complete operand boundaries.
pub(in crate::boolean) fn assemble_cellular_fragments(
    selected: Vec<FaceFragmentRecord>,
    solids: &HashMap<u8, &BrepSolid>,
    imprint: &ImprintResultRecord,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    assemble_fragments_impl(selected, solids, imprint, tolerance, false, None, false,
        crate::BodyKind::Cellular).map(|(solid, _)| solid)
}


/// The name a tangent section's edge records carry in the result (alone on
/// unnamed operands, after the face-pair name otherwise).
pub const TANGENT_EDGE_NAME: &str = "tangent";

/// The direction INTO a face from a tangent section's midpoint, `n_out × t`
/// (see `SourceEdge::tangent_side`): the face's outward normal at the
/// section's midpoint crossed with the coedge's direction of travel along
/// the section. `None` where the geometry cannot be read (a pole, a collapsed
/// span), which leaves the coedge to the arrival-order weld.
fn tangent_material_side(
    source: &SourceEdge,
    forward: bool,
    pcurve: &NurbsCurve,
    surface: &crate::NurbsSurface,
    same_sense: bool,
) -> Option<Vec3> {
    let middle = 0.5 * (source.t0 + source.t1);
    let derivatives = source.curve.derivatives(middle, 1).ok()?;
    let mut travel = derivatives.get(1).copied()?.normalized().ok()?;
    if !forward {
        travel = travel.scale(-1.0);
    }
    let [p0, p1] = pcurve.domain().ok()?;
    let uv = pcurve.evaluate(0.5 * (p0 + p1)).ok()?;
    let normal = surface.normal(uv.x, uv.y).ok()?;
    let outward = if same_sense { normal } else { normal.scale(-1.0) };
    let side = outward.cross(travel);
    (side.length() > 1e-9).then(|| side.normalized().ok()).flatten()
}
