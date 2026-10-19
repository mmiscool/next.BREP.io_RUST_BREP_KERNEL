//! Flat `f64` binary encoding of a `BrepSolid` for zero-JSON WASM transfer.
//!
//! Every numeric field of the topology (ids, flags, knots, weighted control
//! points) is written into one `Vec<f64>` in a fixed traversal order; the
//! only non-numeric payload — face/edge names — travels in a tiny JSON side
//! channel keyed by entity id.  The decoder reads the same layout
//! straight out of a `Float64Array`, replacing `JSON.parse` of megabyte-scale
//! numeric text with sequential typed-array reads.
//!
//! Layout (all values f64):
//!   header:  [VERSION, solid_id, genus, n_vertices, n_edges, n_shells]
//!            at version 1.0, written for every `Solid` body: the bytes of
//!            every payload written before the kind existed, unchanged;
//!            [VERSION, solid_id, genus, kind, n_vertices, n_edges, n_shells]
//!            at version 1.1, written only when the body's kind is not
//!            `Solid` (kind: Solid 0, Sheet 1, Cellular 2, Wire 3). The
//!            reader accepts both; a 1.0 payload reads as `Solid`.
//!   vertex:  [id, x, y, z]
//!   edge:    [id, t0, t1, start_vertex_id, end_vertex_id, degenerate] curve
//!   shell:   [id, n_faces] face*
//!   face:    [id, same_sense] surface [n_loops] loop*
//!   loop:    [id, n_coedges] coedge*
//!   coedge:  [id, edge_id, forward] pcurve
//!   curve:   [degree, n_knots, knots..., n_control, (x, y, z, w)...]
//!   surface: [degree_u, degree_v, n_knots_u, knots_u..., n_knots_v,
//!             knots_v..., n_rows, n_columns, (x, y, z, w)... row-major]

use crate::topology::{
    BodyKind, BrepSolid, CellRecord, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord,
    VertexRecord,
};
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};
use std::collections::BTreeMap;

/// The header a `Solid` body writes, and the oldest version the reader
/// accepts. The snapshot container (`io/snapshot.rs`) names this format
/// durable: a Solid keeps writing 1.0 so no stored payload moves.
pub const SOLID_CODEC_VERSION: f64 = 1.0;
/// The header a body of any other kind writes: the kind slot follows `genus`.
pub const SOLID_CODEC_VERSION_KIND: f64 = 1.1;

/// Largest integer f64 represents exactly. Ids above this would silently
/// round during encode (and the decoder's integrality check cannot detect
/// it), corrupting edge/vertex cross-references.
const MAX_EXACT_ID: u64 = 1 << 53;

fn id_to_f64(id: u64) -> Result<f64, String> {
    if id > MAX_EXACT_ID {
        return Err(format!(
            "solid codec: id {id} exceeds 2^53 and cannot round-trip through f64"
        ));
    }
    Ok(id as f64)
}

#[derive(serde::Serialize, serde::Deserialize, Default, Debug)]
pub struct SolidNames {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub faces: BTreeMap<u64, String>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub edges: BTreeMap<u64, String>,
}

fn push_curve(data: &mut Vec<f64>, curve: &NurbsCurve) {
    data.push(curve.degree as f64);
    data.push(curve.knots.len() as f64);
    data.extend_from_slice(&curve.knots);
    data.push(curve.control_points.len() as f64);
    for point in &curve.control_points {
        data.extend_from_slice(&[point.x, point.y, point.z, point.w]);
    }
}

fn push_surface(data: &mut Vec<f64>, surface: &NurbsSurface) {
    data.push(surface.degree_u as f64);
    data.push(surface.degree_v as f64);
    data.push(surface.knots_u.len() as f64);
    data.extend_from_slice(&surface.knots_u);
    data.push(surface.knots_v.len() as f64);
    data.extend_from_slice(&surface.knots_v);
    data.push(surface.control_points.len() as f64);
    data.push(surface.control_points[0].len() as f64);
    for row in &surface.control_points {
        for point in row {
            data.extend_from_slice(&[point.x, point.y, point.z, point.w]);
        }
    }
}

pub fn encode_solid(solid: &BrepSolid) -> Result<(Vec<f64>, SolidNames), String> {
    if !solid.cells.is_empty() && solid.kind != BodyKind::Cellular {
        return Err("solid codec: cells require declared Cellular kind".into());
    }
    let mut names = SolidNames::default();
    let mut data = Vec::with_capacity(4096);
    if solid.kind.is_solid() {
        data.extend_from_slice(&[SOLID_CODEC_VERSION, id_to_f64(solid.id)?, solid.genus as f64]);
    } else {
        data.extend_from_slice(&[
            SOLID_CODEC_VERSION_KIND,
            id_to_f64(solid.id)?,
            solid.genus as f64,
            solid.kind.codec_code(),
        ]);
    }
    data.extend_from_slice(&[
        solid.vertices.len() as f64,
        solid.edges.len() as f64,
        solid.shells.len() as f64,
    ]);
    for vertex in &solid.vertices {
        data.extend_from_slice(&[
            id_to_f64(vertex.id)?,
            vertex.point.x,
            vertex.point.y,
            vertex.point.z,
        ]);
    }
    for edge in &solid.edges {
        data.extend_from_slice(&[
            id_to_f64(edge.id)?,
            edge.t0,
            edge.t1,
            id_to_f64(edge.start_vertex_id)?,
            id_to_f64(edge.end_vertex_id)?,
            if edge.degenerate { 1.0 } else { 0.0 },
        ]);
        push_curve(&mut data, &edge.curve);
        if let Some(name) = &edge.name {
            names.edges.insert(edge.id, name.clone());
        }
    }
    for shell in &solid.shells {
        data.push(id_to_f64(shell.id)?);
        data.push(shell.faces.len() as f64);
        for face in &shell.faces {
            data.push(id_to_f64(face.id)?);
            data.push(if face.same_sense { 1.0 } else { 0.0 });
            push_surface(&mut data, &face.surface);
            data.push(face.loops.len() as f64);
            for loop_record in &face.loops {
                data.push(id_to_f64(loop_record.id)?);
                data.push(loop_record.coedges.len() as f64);
                for coedge in &loop_record.coedges {
                    data.extend_from_slice(&[
                        id_to_f64(coedge.id)?,
                        id_to_f64(coedge.edge_id)?,
                        if coedge.forward { 1.0 } else { 0.0 },
                    ]);
                    push_curve(&mut data, &coedge.pcurve);
                }
            }
            if let Some(name) = &face.name {
                names.faces.insert(face.id, name.clone());
            }
        }
    }
    // Optional region trailer on a declared Cellular 1.1 image only.
    // Cell-less 1.0/1.1 images retain their exact layout and version.
    if !solid.cells.is_empty() {
        data.push(solid.cells.len() as f64);
        for cell in &solid.cells {
            data.extend_from_slice(&[id_to_f64(cell.id)?, if cell.void { 1.0 } else { 0.0 },
                cell.faces.len() as f64]);
            for &(face, forward) in &cell.faces {
                data.extend_from_slice(&[id_to_f64(face)?, if forward { 1.0 } else { 0.0 }]);
            }
        }
    }
    Ok((data, names))
}

struct Reader<'a> {
    data: &'a [f64],
    cursor: usize,
}

impl<'a> Reader<'a> {
    fn next(&mut self) -> Result<f64, String> {
        let value = *self
            .data
            .get(self.cursor)
            .ok_or("solid codec: truncated buffer")?;
        self.cursor += 1;
        Ok(value)
    }

    fn next_usize(&mut self) -> Result<usize, String> {
        let value = self.next()?;
        if value < 0.0 || value.fract() != 0.0 {
            return Err(format!("solid codec: expected count, got {value}"));
        }
        Ok(value as usize)
    }

    fn next_id(&mut self) -> Result<u64, String> {
        let value = self.next()?;
        if value < 0.0 || value.fract() != 0.0 || value > MAX_EXACT_ID as f64 {
            return Err(format!("solid codec: expected id, got {value}"));
        }
        Ok(value as u64)
    }

    fn next_slice(&mut self, count: usize) -> Result<&'a [f64], String> {
        // Checked: a corrupted count (up to 2^53 passes the integrality test)
        // must fail as a truncation error, never wrap the cursor on 32-bit.
        let end = self
            .cursor
            .checked_add(count)
            .ok_or("solid codec: truncated buffer")?;
        let slice = self
            .data
            .get(self.cursor..end)
            .ok_or("solid codec: truncated buffer")?;
        self.cursor = end;
        Ok(slice)
    }

    fn curve(&mut self) -> Result<NurbsCurve, String> {
        let degree = self.next_usize()?;
        let knot_count = self.next_usize()?;
        let knots = self.next_slice(knot_count)?.to_vec();
        let control_count = self.next_usize()?;
        let raw = self.next_slice(
            control_count
                .checked_mul(4)
                .ok_or("solid codec: truncated buffer")?,
        )?;
        let control_points = raw
            .chunks_exact(4)
            .map(|chunk| Vec4 {
                x: chunk[0],
                y: chunk[1],
                z: chunk[2],
                w: chunk[3],
            })
            .collect();
        NurbsCurve::new(degree, knots, control_points)
    }

    fn surface(&mut self) -> Result<NurbsSurface, String> {
        let degree_u = self.next_usize()?;
        let degree_v = self.next_usize()?;
        let knots_u_count = self.next_usize()?;
        let knots_u = self.next_slice(knots_u_count)?.to_vec();
        let knots_v_count = self.next_usize()?;
        let knots_v = self.next_slice(knots_v_count)?.to_vec();
        let rows = self.next_usize()?;
        let columns = self.next_usize()?;
        // A zero dimension would make the chunks_exact below panic; no valid
        // surface has an empty control net, so reject corrupted payloads here.
        if rows == 0 || columns == 0 {
            return Err("solid codec: surface with empty control net".into());
        }
        let raw = self.next_slice(
            rows.checked_mul(columns)
                .and_then(|cells| cells.checked_mul(4))
                .ok_or("solid codec: truncated buffer")?,
        )?;
        let control_points = raw
            .chunks_exact(columns * 4)
            .map(|row| {
                row.chunks_exact(4)
                    .map(|chunk| Vec4 {
                        x: chunk[0],
                        y: chunk[1],
                        z: chunk[2],
                        w: chunk[3],
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, control_points)
    }
}

pub fn decode_solid(data: &[f64], names: &SolidNames) -> Result<BrepSolid, String> {
    let mut reader = Reader { data, cursor: 0 };
    let version = reader.next()?;
    if version != SOLID_CODEC_VERSION && version != SOLID_CODEC_VERSION_KIND {
        return Err(format!("solid codec: unsupported version {version}"));
    }
    let id = reader.next_id()?;
    let genus = reader.next()? as i64;
    let kind = if version == SOLID_CODEC_VERSION_KIND {
        let code = reader.next()?;
        BodyKind::from_codec_code(code)
            .ok_or_else(|| format!("solid codec: unknown body kind {code}"))?
    } else {
        BodyKind::Solid
    };
    let vertex_count = reader.next_usize()?;
    let edge_count = reader.next_usize()?;
    let shell_count = reader.next_usize()?;
    let mut vertices = Vec::with_capacity(vertex_count);
    for _ in 0..vertex_count {
        vertices.push(VertexRecord {
            id: reader.next_id()?,
            point: Vec3::new(reader.next()?, reader.next()?, reader.next()?),
        });
    }
    let mut edges = Vec::with_capacity(edge_count);
    for _ in 0..edge_count {
        let id = reader.next_id()?;
        let t0 = reader.next()?;
        let t1 = reader.next()?;
        let start_vertex_id = reader.next_id()?;
        let end_vertex_id = reader.next_id()?;
        let degenerate = reader.next()? != 0.0;
        let curve = reader.curve()?;
        edges.push(EdgeRecord {
            id,
            curve,
            t0,
            t1,
            start_vertex_id,
            end_vertex_id,
            degenerate,
            name: names.edges.get(&id).cloned(),
        });
    }
    let mut shells = Vec::with_capacity(shell_count);
    for _ in 0..shell_count {
        let shell_id = reader.next_id()?;
        let face_count = reader.next_usize()?;
        let mut faces = Vec::with_capacity(face_count);
        for _ in 0..face_count {
            let face_id = reader.next_id()?;
            let same_sense = reader.next()? != 0.0;
            let surface = reader.surface()?;
            let loop_count = reader.next_usize()?;
            let mut loops = Vec::with_capacity(loop_count);
            for _ in 0..loop_count {
                let loop_id = reader.next_id()?;
                let coedge_count = reader.next_usize()?;
                let mut coedges = Vec::with_capacity(coedge_count);
                for _ in 0..coedge_count {
                    let coedge_id = reader.next_id()?;
                    let edge_id = reader.next_id()?;
                    let forward = reader.next()? != 0.0;
                    let pcurve = reader.curve()?;
                    coedges.push(CoedgeRecord {
                        id: coedge_id,
                        edge_id,
                        forward,
                        pcurve,
                    });
                }
                loops.push(LoopRecord {
                    id: loop_id,
                    coedges,
                });
            }
            faces.push(FaceRecord {
                id: face_id,
                surface,
                same_sense,
                loops,
                name: names.faces.get(&face_id).cloned(),
            });
        }
        shells.push(ShellRecord {
            id: shell_id,
            faces,
        });
    }
    let mut cells = Vec::new();
    if kind == BodyKind::Cellular && reader.cursor < data.len() {
        let count = reader.next_usize()?;
        if count == 0 || count > (data.len()-reader.cursor)/3 {
            return Err("solid codec: invalid cell count".into());
        }
        for _ in 0..count {
            let id = reader.next_id()?;
            let void = match reader.next()? {
                0.0 => false, 1.0 => true,
                _ => return Err("solid codec: invalid cell void flag".into()),
            };
            let face_count = reader.next_usize()?;
            if face_count > (data.len()-reader.cursor)/2 {
                return Err("solid codec: truncated cell faces".into());
            }
            let mut faces = Vec::with_capacity(face_count);
            for _ in 0..face_count {
                let face = reader.next_id()?;
                let forward = match reader.next()? {
                    0.0 => false, 1.0 => true,
                    _ => return Err("solid codec: invalid cell face sense".into()),
                };
                faces.push((face, forward));
            }
            cells.push(CellRecord { id, faces, void });
        }
    }
    if reader.cursor != data.len() {
        return Err("solid codec: trailing data".into());
    }
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        id,
        vertices,
        edges,
        shells,
        genus,
        kind,
        cells,
    })
}

