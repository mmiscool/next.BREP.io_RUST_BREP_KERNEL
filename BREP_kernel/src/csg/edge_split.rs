use crate::entity_tolerance::EntityTolerances;
use crate::imprint::ImprintResultRecord;
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, VertexRecord};
use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::{NurbsCurve, Vec3};
use rustc_hash::FxHashMap as HashMap;

fn subcurve_by_fraction(
    curve: &NurbsCurve,
    start_fraction: f64,
    end_fraction: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let first = start + (end - start) * start_fraction;
    let second = start + (end - start) * end_fraction;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if first > start + epsilon && first < end - epsilon {
        result = result
            .split(first)
            .or_refuse(KernelStage::Refine, "split")?
            .1;
    }
    let domain = result.domain().or_refuse(KernelStage::Refine, "domain")?;
    if second < domain[1] - epsilon && second > domain[0] + epsilon {
        result = result
            .split(second)
            .or_refuse(KernelStage::Refine, "split")?
            .0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && second < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in edge_split subcurve");
    }
    Ok(result)
}

/// Which vertex IS this split point?.
///
/// A split point is where a section curve crossed a boundary edge. The imprint
/// has already computed that same crossing as a junction VERTEX, from the
/// section side. On exact geometry the two agree to nanometres and the tight
/// band below finds each other trivially. On a vendor import they cannot: when
/// the edge's own 3D curve sits `d` off the surfaces it bounds, the crossing
/// solved ON the edge and the crossing solved as the section's endpoint are
/// necessarily up to about `d` apart, and welding them is not a convenience —
/// it is the only way one junction ends up as one vertex.
///
/// `entity_band` is that edge's own measured residual (`EntityTolerances::edge`),
/// already capped, and it widens the physical weld floor for this edge only.
/// The floor must not grow with distance from the world origin: a rigid move
/// cannot make two distinct junctions become one vertex. It is a search question
/// ("which candidates might be this point?"), not an acceptance gate — `validate`
/// still judges the assembled solid on the global band.
///
/// `BREP_DEBUG_SPLIT_VERTEX=1` reports each decision and the distance to the
/// nearest imprint junction, which is how the band was measured rather than
/// guessed.
fn claim_vertex(
    vertices: &mut Vec<VertexRecord>,
    imprint: &ImprintResultRecord,
    edge: &EdgeRecord,
    point: Vec3,
    entity_band: f64,
    next_id: &mut u64,
) -> u64 {
    // Keep a true section/cap intersection distinct from a nearby authored
    // seam vertex, regardless of placement. The former origin-coupled floor
    // merged fixture 30's exact crossing 1.422e-5 from its seam, and translating
    // the same cylinders by ten units made all four operations refuse.
    let floor: f64 = 1e-7;
    let tolerance = floor.max(if entity_band.is_finite() && entity_band > 0.0 {
        entity_band
    } else {
        0.0
    });
    let debug = std::env::var("BREP_DEBUG_SPLIT_VERTEX").is_ok();
    if let Some(vertex) = vertices
        .iter()
        .find(|vertex| vertex.point.sub(point).length() <= tolerance)
    {
        if debug {
            eprintln!(
                "split-vertex ({:.7},{:.7},{:.7}) band={tolerance:.3e} entity={entity_band:.3e}: reuse v{}",
                point.x, point.y, point.z, vertex.id
            );
        }
        return vertex.id;
    }
    let nearest = imprint
        .vertices
        .iter()
        .map(|vertex| (vertex.point, vertex.point.sub(point).length()))
        .min_by(|a, b| a.1.total_cmp(&b.1));
    // A computed section endpoint can be displaced along an accurate edge
    // by more than its carrier residual (e.g. a near-tangent root). Keep the
    // repair search for that explicit junction, but require it to lie on this
    // edge at identity precision. A nearby point on the opposite thin wall
    // therefore cannot authorize moving this split across the wall.
    let canonical = imprint.vertices.iter()
        .map(|vertex| (vertex.point, vertex.point.sub(point).length()))
        .filter(|(candidate, distance)| {
            if *distance <= tolerance {
                return true;
            }
            if *distance > tolerance.max(crate::tolerance::WELD_FLOOR) {
                return false;
            }
            crate::project_point_to_curve(&edge.curve, *candidate).is_ok_and(|projection| {
                projection.distance <= tolerance
                    && projection.u >= edge.t0.min(edge.t1)
                    && projection.u <= edge.t0.max(edge.t1)
            })
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(candidate, _)| candidate)
        .unwrap_or(point);
    if let Some(vertex) = vertices.iter()
        .find(|vertex| vertex.point.sub(canonical).length() <= tolerance)
    {
        return vertex.id;
    }
    if debug {
        eprintln!(
            "split-vertex ({:.7},{:.7},{:.7}) band={tolerance:.3e} entity={entity_band:.3e}: \
             nearest imprint junction {:.3e} -> {}",
            point.x,
            point.y,
            point.z,
            nearest.map(|(_, d)| d).unwrap_or(f64::INFINITY),
            if canonical.sub(point).length() > 0.0 {
                "SNAPPED"
            } else {
                "kept"
            }
        );
    }
    let id = *next_id;
    *next_id += 1;
    vertices.push(VertexRecord {
        id,
        point: canonical,
    });
    id
}

/// Apply the boundary-edge partition produced by imprint construction.
///
/// The original edge curve and parameter range semantics are preserved; only
/// coedge p-curves are restricted to traversal-aligned subcurves.
pub fn apply_edge_splits(
    solid: &BrepSolid,
    operand: u8,
    imprint: &ImprintResultRecord,
) -> Result<BrepSolid, KernelRefusal> {
    apply_edge_splits_with_map(solid, operand, imprint).map(|(result, _)| result)
}

/// [`apply_edge_splits`] plus the identity ledger: original edge id → the
/// minted sub-edge ids that replaced it. Consumers keyed on ORIGINAL edge
/// ids (the fragment-selection barrier set) must remap through it, since the
/// split solid's coedges reference the minted ids.
pub fn apply_edge_splits_with_map(
    solid: &BrepSolid,
    operand: u8,
    imprint: &ImprintResultRecord,
) -> Result<(BrepSolid, HashMap<u64, Vec<u64>>), KernelRefusal> {
    let requested = imprint
        .edge_splits
        .iter()
        .filter(|split| split.operand == operand)
        .map(|split| (split.edge_id, split.parameters.as_slice()))
        .collect::<HashMap<_, _>>();
    if requested.is_empty() {
        return Ok((solid.clone(), HashMap::default()));
    }

    let mut result = solid.clone();
    let mut next_vertex_id = result
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_edge_id = result.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;
    let mut next_coedge_id = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut pieces: HashMap<u64, Vec<EdgeRecord>> = HashMap::default();
    // Measured per-entity residuals for THIS operand, lazily and only for the
    // edges actually split. Built on the pre-split solid — the entry snapshot —
    // so nothing measured here can outlive the geometry it was taken from.
    //
    // Floor ZERO on purpose: measure the edge's actual residual separately
    // from the weld floor in `claim_vertex`. The view's floor also drives the
    // sampler's refinement: the tighter it is the harder the sampler looks,
    // and zero asks for its maximum. Only edges actually split pay that cost.
    let mut entity = EntityTolerances::with_floor(solid, 0.0);

    for edge in &solid.edges {
        let Some(parameters) = requested.get(&edge.id) else {
            continue;
        };
        let parameter_tolerance = (edge.t1 - edge.t0).abs() * 1e-10;
        let mut partition = parameters
            .iter()
            .copied()
            .filter(|parameter| {
                *parameter > edge.t0 + parameter_tolerance
                    && *parameter < edge.t1 - parameter_tolerance
            })
            .collect::<Vec<_>>();
        partition.sort_by(f64::total_cmp);
        partition.dedup_by(|a, b| (*a - *b).abs() <= parameter_tolerance);
        partition.insert(0, edge.t0);
        partition.push(edge.t1);
        if partition.len() <= 2 {
            continue;
        }
        let entity_band = entity.edge(edge.id);
        let mut vertex_ids = vec![edge.start_vertex_id];
        for parameter in &partition[1..partition.len() - 1] {
            let point = edge
                .curve
                .evaluate(*parameter)
                .or_refuse(KernelStage::Refine, "evaluate")?;
            vertex_ids.push(claim_vertex(
                &mut result.vertices,
                imprint,
                edge,
                point,
                entity_band,
                &mut next_vertex_id,
            ));
        }
        vertex_ids.push(edge.end_vertex_id);
        let mut edge_pieces = Vec::new();
        for index in 0..partition.len() - 1 {
            // First piece keeps the source name; later pieces get the
            // deterministic `_1`, `_2` split suffixes the application uses.
            let name = edge.name.as_ref().map(|name| {
                if index == 0 {
                    name.clone()
                } else {
                    format!("{name}_{index}")
                }
            });
            edge_pieces.push(EdgeRecord {
                id: next_edge_id,
                curve: edge.curve.clone(),
                t0: partition[index],
                t1: partition[index + 1],
                start_vertex_id: vertex_ids[index],
                end_vertex_id: vertex_ids[index + 1],
                degenerate: false,
                name,
            });
            next_edge_id += 1;
        }
        pieces.insert(edge.id, edge_pieces);
    }

    result.edges.retain(|edge| !pieces.contains_key(&edge.id));
    for edge_pieces in pieces.values() {
        result.edges.extend(edge_pieces.iter().cloned());
    }

    // id -> source edge, built once. The per-coedge `solid.edges.iter().find`
    // below was O(edges) inside the face/loop/coedge loops. `solid` is the
    // immutable source, so the lookup returns the identical record.
    let source_edges: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();

    for face in result
        .shells
        .iter_mut()
        .flat_map(|shell| shell.faces.iter_mut())
    {
        for loop_record in &mut face.loops {
            let mut rebuilt = Vec::new();
            for coedge in &loop_record.coedges {
                let Some(edge_pieces) = pieces.get(&coedge.edge_id) else {
                    rebuilt.push(coedge.clone());
                    continue;
                };
                let original = *source_edges.get(&coedge.edge_id).ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Refine,
                        "edge_split",
                        "apply_edge_splits: missing source edge",
                    )
                })?;
                let span = original.t1 - original.t0;
                let indices: Box<dyn Iterator<Item = usize>> = if coedge.forward {
                    Box::new(0..edge_pieces.len())
                } else {
                    Box::new((0..edge_pieces.len()).rev())
                };
                for index in indices {
                    let piece = &edge_pieces[index];
                    let start_fraction = if coedge.forward {
                        (piece.t0 - original.t0) / span
                    } else {
                        (original.t1 - piece.t1) / span
                    };
                    let end_fraction = if coedge.forward {
                        (piece.t1 - original.t0) / span
                    } else {
                        (original.t1 - piece.t0) / span
                    };
                    rebuilt.push(CoedgeRecord {
                        id: next_coedge_id,
                        edge_id: piece.id,
                        forward: coedge.forward,
                        pcurve: subcurve_by_fraction(&coedge.pcurve, start_fraction, end_fraction)?,
                    });
                    next_coedge_id += 1;
                }
            }
            loop_record.coedges = rebuilt;
        }
    }
    let map = pieces
        .iter()
        .map(|(source_id, edge_pieces)| {
            (
                *source_id,
                edge_pieces.iter().map(|piece| piece.id).collect(),
            )
        })
        .collect();
    Ok((result, map))
}

