use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{NurbsCurve, NurbsSurface, Vec3};
use rustc_hash::FxHashMap as HashMap;
use slotmap::{new_key_type, SlotMap};

new_key_type! {
    pub struct VertexId;
    pub struct EdgeId;
    pub struct CoedgeId;
    pub struct LoopId;
    pub struct FaceId;
    pub struct ShellId;
}

#[derive(Clone, Debug)]
pub struct ArenaVertex {
    pub wire_id: u64,
    pub point: Vec3,
}

#[derive(Clone, Debug)]
pub struct ArenaEdge {
    pub wire_id: u64,
    pub curve: NurbsCurve,
    pub t0: f64,
    pub t1: f64,
    pub start: VertexId,
    pub end: VertexId,
    pub degenerate: bool,
    pub name: Option<String>,
}

#[derive(Clone, Debug)]
pub struct ArenaCoedge {
    pub wire_id: u64,
    pub edge: EdgeId,
    pub forward: bool,
    pub pcurve: NurbsCurve,
}

#[derive(Clone, Debug)]
pub struct ArenaLoop {
    pub wire_id: u64,
    pub coedges: Vec<CoedgeId>,
}

#[derive(Clone, Debug)]
pub struct ArenaFace {
    pub wire_id: u64,
    pub surface: NurbsSurface,
    pub same_sense: bool,
    pub loops: Vec<LoopId>,
    pub name: Option<String>,
}

#[derive(Clone, Debug)]
pub struct ArenaShell {
    pub wire_id: u64,
    pub faces: Vec<FaceId>,
}

/// Type-safe mutable topology used inside the kernel.  The JSON/WASM record
/// model remains the stable wire format; conversion happens at that boundary.
#[derive(Clone, Debug)]
pub struct TopologyArena {
    pub wire_solid_id: u64,
    pub genus: i64,
    pub vertices: SlotMap<VertexId, ArenaVertex>,
    pub edges: SlotMap<EdgeId, ArenaEdge>,
    pub coedges: SlotMap<CoedgeId, ArenaCoedge>,
    pub loops: SlotMap<LoopId, ArenaLoop>,
    pub faces: SlotMap<FaceId, ArenaFace>,
    pub shells: SlotMap<ShellId, ArenaShell>,
}

impl TopologyArena {
    pub fn from_brep(solid: &BrepSolid) -> Result<Self, String> {
        let mut arena = Self {
            wire_solid_id: solid.id,
            genus: solid.genus,
            vertices: SlotMap::with_key(),
            edges: SlotMap::with_key(),
            coedges: SlotMap::with_key(),
            loops: SlotMap::with_key(),
            faces: SlotMap::with_key(),
            shells: SlotMap::with_key(),
        };
        let mut vertex_ids = HashMap::<u64, VertexId>::default();
        for vertex in &solid.vertices {
            if vertex_ids.contains_key(&vertex.id) {
                return Err(format!("duplicate vertex wire id {}", vertex.id));
            }
            let id = arena.vertices.insert(ArenaVertex {
                wire_id: vertex.id,
                point: vertex.point,
            });
            vertex_ids.insert(vertex.id, id);
        }
        let mut edge_ids = HashMap::<u64, EdgeId>::default();
        for edge in &solid.edges {
            if edge_ids.contains_key(&edge.id) {
                return Err(format!("duplicate edge wire id {}", edge.id));
            }
            let start = *vertex_ids.get(&edge.start_vertex_id).ok_or_else(|| {
                format!(
                    "edge {} references missing start vertex {}",
                    edge.id, edge.start_vertex_id
                )
            })?;
            let end = *vertex_ids.get(&edge.end_vertex_id).ok_or_else(|| {
                format!(
                    "edge {} references missing end vertex {}",
                    edge.id, edge.end_vertex_id
                )
            })?;
            let id = arena.edges.insert(ArenaEdge {
                wire_id: edge.id,
                curve: edge.curve.clone(),
                t0: edge.t0,
                t1: edge.t1,
                start,
                end,
                degenerate: edge.degenerate,
                name: edge.name.clone(),
            });
            edge_ids.insert(edge.id, id);
        }

        let mut seen_shells = HashMap::<u64, ShellId>::default();
        let mut seen_faces = HashMap::<u64, FaceId>::default();
        let mut seen_loops = HashMap::<u64, LoopId>::default();
        let mut seen_coedges = HashMap::<u64, CoedgeId>::default();
        for shell in &solid.shells {
            if seen_shells.contains_key(&shell.id) {
                return Err(format!("duplicate shell wire id {}", shell.id));
            }
            let mut face_keys = Vec::with_capacity(shell.faces.len());
            for face in &shell.faces {
                if seen_faces.contains_key(&face.id) {
                    return Err(format!("duplicate face wire id {}", face.id));
                }
                let mut loop_keys = Vec::with_capacity(face.loops.len());
                for loop_record in &face.loops {
                    if seen_loops.contains_key(&loop_record.id) {
                        return Err(format!("duplicate loop wire id {}", loop_record.id));
                    }
                    let mut coedge_keys = Vec::with_capacity(loop_record.coedges.len());
                    for coedge in &loop_record.coedges {
                        if seen_coedges.contains_key(&coedge.id) {
                            return Err(format!("duplicate coedge wire id {}", coedge.id));
                        }
                        let edge = *edge_ids.get(&coedge.edge_id).ok_or_else(|| {
                            format!(
                                "coedge {} references missing edge {}",
                                coedge.id, coedge.edge_id
                            )
                        })?;
                        let key = arena.coedges.insert(ArenaCoedge {
                            wire_id: coedge.id,
                            edge,
                            forward: coedge.forward,
                            pcurve: coedge.pcurve.clone(),
                        });
                        seen_coedges.insert(coedge.id, key);
                        coedge_keys.push(key);
                    }
                    let key = arena.loops.insert(ArenaLoop {
                        wire_id: loop_record.id,
                        coedges: coedge_keys,
                    });
                    seen_loops.insert(loop_record.id, key);
                    loop_keys.push(key);
                }
                let key = arena.faces.insert(ArenaFace {
                    wire_id: face.id,
                    surface: face.surface.clone(),
                    same_sense: face.same_sense,
                    loops: loop_keys,
                    name: face.name.clone(),
                });
                seen_faces.insert(face.id, key);
                face_keys.push(key);
            }
            let key = arena.shells.insert(ArenaShell {
                wire_id: shell.id,
                faces: face_keys,
            });
            seen_shells.insert(shell.id, key);
        }
        Ok(arena)
    }

    pub fn to_brep(&self) -> Result<BrepSolid, String> {
        let vertices = self
            .vertices
            .values()
            .map(|vertex| VertexRecord {
                id: vertex.wire_id,
                point: vertex.point,
            })
            .collect();
        let edges = self
            .edges
            .values()
            .map(|edge| {
                Ok(EdgeRecord {
                    id: edge.wire_id,
                    curve: edge.curve.clone(),
                    t0: edge.t0,
                    t1: edge.t1,
                    start_vertex_id: self
                        .vertices
                        .get(edge.start)
                        .ok_or_else(|| "arena edge has missing start vertex".to_string())?
                        .wire_id,
                    end_vertex_id: self
                        .vertices
                        .get(edge.end)
                        .ok_or_else(|| "arena edge has missing end vertex".to_string())?
                        .wire_id,
                    degenerate: edge.degenerate,
                    name: edge.name.clone(),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let shells = self
            .shells
            .values()
            .map(|shell| {
                let faces = shell
                    .faces
                    .iter()
                    .map(|face_id| {
                        let face = self
                            .faces
                            .get(*face_id)
                            .ok_or_else(|| "arena shell has missing face".to_string())?;
                        let loops = face
                            .loops
                            .iter()
                            .map(|loop_id| {
                                let loop_record = self
                                    .loops
                                    .get(*loop_id)
                                    .ok_or_else(|| "arena face has missing loop".to_string())?;
                                let coedges = loop_record
                                    .coedges
                                    .iter()
                                    .map(|coedge_id| {
                                        let coedge =
                                            self.coedges.get(*coedge_id).ok_or_else(|| {
                                                "arena loop has missing coedge".to_string()
                                            })?;
                                        Ok(CoedgeRecord {
                                            id: coedge.wire_id,
                                            edge_id: self
                                                .edges
                                                .get(coedge.edge)
                                                .ok_or_else(|| {
                                                    "arena coedge has missing edge".to_string()
                                                })?
                                                .wire_id,
                                            forward: coedge.forward,
                                            pcurve: coedge.pcurve.clone(),
                                        })
                                    })
                                    .collect::<Result<Vec<_>, String>>()?;
                                Ok(LoopRecord {
                                    id: loop_record.wire_id,
                                    coedges,
                                })
                            })
                            .collect::<Result<Vec<_>, String>>()?;
                        Ok(FaceRecord {
                            id: face.wire_id,
                            surface: face.surface.clone(),
                            same_sense: face.same_sense,
                            loops,
                            name: face.name.clone(),
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                Ok(ShellRecord {
                    id: shell.wire_id,
                    faces,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok(BrepSolid {
            mass_properties_cache: Default::default(),
            id: self.wire_solid_id,
            vertices,
            edges,
            shells,
            genus: self.genus,
        })
    }
}

