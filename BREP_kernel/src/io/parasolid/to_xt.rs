//! Kernel solids to a Parasolid text part (`.x_t`): the export.
//!
//! The part is written in Parasolid V30.1's schema (`SCH_3001168_30100`) with
//! its layouts described as embedded schema deltas against the base, exactly
//! as Parasolid 30.1 writes them (the 152,096 ABC part files; every one of the
//! 2000-file sample is written back byte for byte by [`super::writer`]).
//! Parasolid V30 and later read the layouts natively; V14 and later read them
//! through the deltas (§2.1.2, which promises older versions no more than
//! that).
//!
//! The topology is the reference's for a manifold solid (§4.3, §5.3): an
//! infinite void region and a solid region (and a void region for each
//! cavity), a shell per region side, faces chained as back faces of their
//! solid region's shell and front faces of the void region's, a loop per
//! boundary, and two fins per edge, the positive one running with the edge
//! and pointing at its end vertex. Geometry is exact: analytic carriers the
//! kernel recognises as planes, cylinders, cones, spheres and tori are
//! written as such, lines and circular arcs as lines and circles, everything
//! else as the kernel's own B-surfaces and B-curves. Lengths are metres.

use std::rc::Rc;

use rustc_hash::FxHashMap as HashMap;

use crate::analytic_surface::AnalyticSurface;
use crate::topology::BrepSolid;
use crate::{NurbsCurve, NurbsSurface, Vec3};

use super::geometry::MM_PER_METRE;
use super::reader::{Node, NodeLayout, Value};
use super::schema::{self, Count, Kind};
use super::writer::{records, Tokens};

/// The schema the part is written in, and the version that wrote it.
const SCHEMA: &str = "SCH_3001168_30100_13006";
const MODELLER: &str = ": TRANSMIT FILE created by modeller version 3001168";
/// The largest node type schema 30100 has (as its writers state it).
const MAX_NODE_TYPE: i64 = 214;

/// One body to export, with what the importer reads back as its name and
/// colour.
pub struct XtExportBody<'a> {
    pub solid: &'a BrepSolid,
    pub name: Option<&'a str>,
    /// RGB in 0..1.
    pub color: Option<[f64; 3]>,
}

/// What an export wrote, and how exact its edges are.
#[derive(Clone, Debug, Default)]
pub struct XtExportReport {
    pub text: String,
    pub edges: usize,
    /// Edges some point of which lies further from one of its faces than
    /// Parasolid's linear resolution (1e-8 m): written as accurate edges
    /// all the same (see the module doc).
    pub edges_over_resolution: usize,
    /// The furthest any edge lies from one of its faces, in mm.
    pub worst_edge_mm: f64,
}

/// Parasolid's linear resolution, in mm.
const RESOLUTION_MM: f64 = 1e-5;

/// Write `bodies` as a Parasolid text part: one body as the root, or several
/// listed by a POINTER_LIS_BLOCK root (§2.1, §5.4.2).
pub fn export_parasolid_text(bodies: &[XtExportBody], application: &str) -> Result<String, String> {
    export_parasolid_report(bodies, application).map(|r| r.text)
}

/// [`export_parasolid_text`] with how far the edges written lie from their
/// faces.
pub fn export_parasolid_report(bodies: &[XtExportBody], application: &str) -> Result<XtExportReport, String> {
    export(bodies, application, false)
}

/// The same part as a neutral binary file (`.x_b`): the keyword header in
/// text (with FORMAT=binary), then the part data in neutral binary (§3.3.3).
/// `text` holds the bytes, which are not UTF-8: use [`export_parasolid_binary`].
pub fn export_parasolid_binary(bodies: &[XtExportBody], application: &str) -> Result<Vec<u8>, String> {
    let (_, bytes) = build(bodies, application, true)?;
    Ok(bytes)
}

fn export(bodies: &[XtExportBody], application: &str, binary: bool) -> Result<XtExportReport, String> {
    let (text, _) = build(bodies, application, binary)?;
    let mut report = XtExportReport { text, ..Default::default() };
    for body in bodies {
        let (edges, over, worst) = edge_deviation(body.solid);
        report.edges += edges;
        report.edges_over_resolution += over;
        report.worst_edge_mm = report.worst_edge_mm.max(worst);
    }
    Ok(report)
}

/// The part as text, or (when `binary`) as bytes.
fn build(bodies: &[XtExportBody], application: &str, binary: bool) -> Result<(String, Vec<u8>), String> {
    if bodies.is_empty() {
        return Err("export_parasolid: nothing to export".into());
    }
    let mut b = Builder::default();
    let root = if bodies.len() == 1 { None } else { Some(b.reserve()) };
    let mut body_indices = Vec::new();
    for (k, body) in bodies.iter().enumerate() {
        let index = b.body(body).map_err(|e| format!("export_parasolid: body {}: {e}", k + 1))?;
        body_indices.push(index);
    }
    if let Some(root) = root {
        b.pointer_list(root, &body_indices)?;
    }
    // Node ids are distinct across the file, so every body's highest is the
    // file's.
    let highest = b.next_id;
    for &body in &body_indices {
        b.set(body, "highest_node_id", int(highest))?;
    }
    let first = root.unwrap_or(body_indices[0]);
    let mut order: Vec<u32> = b.nodes.keys().copied().filter(|&i| i != first).collect();
    order.sort_unstable();
    order.insert(0, first);
    let mut described = rustc_hash::FxHashSet::default();
    if binary {
        let mut tokens = super::writer::BinaryTokens::default();
        tokens.prologue(MODELLER, SCHEMA, Some(MAX_NODE_TYPE), 0)?;
        for index in order {
            let node = &b.nodes[&index];
            let describe = described.insert(node.node_type());
            tokens.node(node, describe, 0)?;
        }
        tokens.terminator()?;
        let mut bytes = header(application, "binary").into_bytes();
        bytes.extend(tokens.data);
        return Ok((String::new(), bytes));
    }
    let mut tokens = Tokens::default();
    tokens.data.push('T');
    tokens.string(MODELLER);
    tokens.string(SCHEMA);
    tokens.int(Some(MAX_NODE_TYPE));
    tokens.int(Some(0));
    for index in order {
        let node = &b.nodes[&index];
        let describe = described.insert(node.node_type());
        tokens.node(node, describe, 0)?;
    }
    tokens.int(Some(1));
    tokens.int(Some(0));
    Ok((format!("{}{}", header(application, "text"), records(&tokens.data)), Vec::new()))
}

/// For each non-degenerate edge, the largest distance of nine points along
/// it from the surfaces of the faces that use it: (edges, edges over the
/// resolution, worst in mm).
fn edge_deviation(solid: &BrepSolid) -> (usize, usize, f64) {
    let deviations = edge_deviations(solid);
    let over = deviations.values().filter(|&&d| d > RESOLUTION_MM).count();
    let worst = deviations.values().copied().fold(0.0f64, f64::max);
    (deviations.len(), over, worst)
}

/// Each non-degenerate edge's largest distance (mm) from the surfaces of the
/// faces using it, over nine points along it.
fn edge_deviations(solid: &BrepSolid) -> HashMap<u64, f64> {
    let mut faces_of: HashMap<u64, Vec<&NurbsSurface>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for lp in &face.loops {
                for c in &lp.coedges {
                    faces_of.entry(c.edge_id).or_default().push(&face.surface);
                }
            }
        }
    }
    let mut out = HashMap::default();
    for edge in solid.edges.iter().filter(|e| !e.degenerate) {
        let Some(surfaces) = faces_of.get(&edge.id) else { continue };
        let Ok(curve) = brep_topology::edge_subcurve::edge_subcurve(edge) else { continue };
        let Ok([a, b]) = curve.domain() else { continue };
        let mut edge_worst = 0.0f64;
        for k in 0..=8 {
            let Ok(p) = curve.evaluate(a + (b - a) * k as f64 / 8.0) else { continue };
            for surface in surfaces {
                if let Ok(projection) = crate::project_point_to_surface(surface, p) {
                    edge_worst = edge_worst.max(projection.point.sub(p).length());
                }
            }
        }
        out.insert(edge.id, edge_worst);
    }
    out
}

/// The keyword header (§3.1), as Parasolid writes it: a key a line.
fn header(application: &str, format: &str) -> String {
    let escape = |value: &str| {
        let mut out = String::new();
        for c in value.chars() {
            match c {
                '\n' => out.push_str("^n"),
                ' ' => out.push_str("^-"),
                ';' => out.push_str("^;"),
                '^' => out.push_str("^^"),
                c if c.is_ascii_graphic() => out.push(c),
                _ => out.push('?'),
            }
        }
        out
    };
    let mut out = String::new();
    out.push_str("**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n");
    out.push_str("**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789**************************\n");
    out.push_str("**PART1;\n");
    for (key, value) in [
        ("MC", "unknown"),
        ("MC_MODEL", "unknown"),
        ("MC_ID", "unknown"),
        ("OS", "unknown"),
        ("OS_RELEASE", "unknown"),
        ("FRU", "BREP Parasolid writer"),
        ("APPL", application),
        ("SITE", "unknown"),
        ("USER", "unknown"),
        ("FORMAT", format),
        ("GUISE", "transmit"),
        ("KEY", "unknown"),
        ("FILE", "unknown"),
        ("DATE", "unknown"),
    ] {
        out.push_str(&format!("{key}={};\n", escape(value)));
    }
    out.push_str("**PART2;\n");
    out.push_str(&format!("SCH={};\n", SCHEMA.trim_end_matches("_13006")));
    out.push_str("USFLD_SIZE=0;\n");
    out.push_str("**PART3;\n");
    out.push_str("**END_OF_HEADER*****************************************************************\n");
    out
}

#[derive(Default)]
struct Builder {
    nodes: HashMap<u32, Node>,
    definitions: HashMap<String, u32>,
    definition_order: Vec<u32>,
    layouts: HashMap<u16, Rc<NodeLayout>>,
    next_index: u32,
    next_id: i64,
}

/// What a surface was written as, and whether its natural normal is the
/// reverse of the kernel surface's.
struct WrittenSurface {
    index: u32,
    flipped: bool,
}

impl Builder {
    fn reserve(&mut self) -> u32 {
        self.next_index += 1;
        self.next_index
    }

    fn id(&mut self) -> i64 {
        self.next_id += 1;
        self.next_id
    }

    fn layout(&mut self, node_type: u16) -> Result<Rc<NodeLayout>, String> {
        if let Some(layout) = self.layouts.get(&node_type) {
            return Ok(layout.clone());
        }
        let mut layout = super::reader::stored_layout(schema::V30_SCHEMA, node_type)?;
        layout.delta = schema::v30_delta(node_type).map(str::to_string);
        let layout = Rc::new(layout);
        self.layouts.insert(node_type, layout.clone());
        Ok(layout)
    }

    /// Put a node of `node_type` at `index` with the named fields set; every
    /// other field is a null pointer, zero, an unset real or vector.
    fn put(&mut self, index: u32, node_type: u16, fields: Vec<(&str, Value)>) -> Result<(), String> {
        let layout = self.layout(node_type)?;
        let mut values: Vec<Value> = layout
            .fields
            .iter()
            .map(|f| match (f.count, f.kind) {
                (Count::Scalar, kind) => default_value(kind),
                (Count::Fixed(n), kind) => Value::Array(vec![default_value(kind); n]),
                (Count::Variable, Kind::Char) => Value::Text(String::new()),
                (Count::Variable, _) => Value::Array(Vec::new()),
            })
            .collect();
        for (name, value) in fields {
            let at = layout
                .fields
                .iter()
                .position(|f| f.name == name)
                .ok_or_else(|| format!("{} has no field {name}", layout.name))?;
            values[at] = value;
        }
        self.nodes.insert(index, Node { index, layout, values });
        Ok(())
    }

    fn add(&mut self, node_type: u16, fields: Vec<(&str, Value)>) -> Result<u32, String> {
        let index = self.reserve();
        self.put(index, node_type, fields)?;
        Ok(index)
    }

    fn set(&mut self, index: u32, name: &str, value: Value) -> Result<(), String> {
        let node = self.nodes.get_mut(&index).ok_or_else(|| format!("node {index} is not written yet"))?;
        let at = node
            .layout
            .fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| format!("{} has no field {name}", node.layout.name))?;
        node.values[at] = value;
        Ok(())
    }

    /// A POINTER_LIS_BLOCK root listing the bodies: blocks of 20 entries.
    fn pointer_list(&mut self, root: u32, bodies: &[u32]) -> Result<(), String> {
        let chunks: Vec<&[u32]> = bodies.chunks(20).collect();
        let mut indices = vec![root];
        for _ in 1..chunks.len() {
            indices.push(self.reserve());
        }
        for (k, chunk) in chunks.iter().enumerate() {
            let mut entries: Vec<Value> = chunk.iter().map(|&i| ptr(i)).collect();
            entries.resize(20, ptr(0));
            let next = indices.get(k + 1).copied().unwrap_or(0);
            self.put(
                indices[k],
                74,
                vec![
                    ("n_entries", int(if k == 0 { bodies.len() as i64 } else { chunk.len() as i64 })),
                    ("index_map_offset", int(0)),
                    ("next_block", ptr(next)),
                    ("entries", Value::Array(entries)),
                ],
            )?;
        }
        Ok(())
    }

    fn body(&mut self, export: &XtExportBody) -> Result<u32, String> {
        let solid = export.solid;
        if solid.kind != crate::BodyKind::Solid {
            return Err(format!("a {:?} body is not exported yet; only solids are", solid.kind));
        }
        let body = self.reserve();
        // Regions: the infinite void first (the body's region chain head),
        // then the solid, then a void per cavity shell.
        let outer = outer_shell(solid)?;
        let infinite = self.reserve();
        let material = self.reserve();
        let cavities: Vec<usize> = (0..solid.shells.len()).filter(|&s| s != outer).collect();
        let cavity_regions: Vec<u32> = cavities.iter().map(|_| self.reserve()).collect();
        // A shell on each side of every kernel shell: its solid side in the
        // solid region, its void side in the infinite void or its cavity.
        let mut solid_shells = Vec::new();
        let mut void_shells = Vec::new();
        for s in 0..solid.shells.len() {
            solid_shells.push(self.reserve());
            void_shells.push(self.reserve());
            let _ = s;
        }
        // Vertices and their points.
        let mut vertex_index: HashMap<u64, u32> = HashMap::default();
        let mut point_nodes = Vec::new();
        for v in &solid.vertices {
            let vertex = self.reserve();
            let point = self.reserve();
            vertex_index.insert(v.id, vertex);
            point_nodes.push((vertex, point, v.point));
        }
        // Edges: only those some face's loop uses (degenerate pole edges
        // are not edges in XT: the loop runs through the pole vertex).
        let mut used: HashMap<u64, Vec<(u32, bool, u32)>> = HashMap::default(); // edge -> (fin, forward, loop)
        let edges_by_id: HashMap<u64, &crate::topology::EdgeRecord> = solid.edges.iter().map(|e| (e.id, e)).collect();
        // Edges further from their faces than the resolution are written
        // tolerant (§5.3.11): no curve, a tolerance, and on each fin a
        // trimmed SP-curve, the coedge's own curve in its face's parameters.
        let tolerant: HashMap<u64, f64> = edge_deviations(solid)
            .into_iter()
            .filter(|&(_, d)| d > RESOLUTION_MM)
            .map(|(id, d)| (id, (2.0 * d).max(10.0 * RESOLUTION_MM) / MM_PER_METRE))
            .collect();
        let vertex_points: HashMap<u64, Vec3> = solid.vertices.iter().map(|v| (v.id, v.point)).collect();
        let mut vertex_tolerance: HashMap<u64, f64> = HashMap::default();
        for e in &solid.edges {
            if let Some(&t) = tolerant.get(&e.id) {
                for v in [e.start_vertex_id, e.end_vertex_id] {
                    let slot = vertex_tolerance.entry(v).or_insert(0.0);
                    *slot = slot.max(t);
                }
            }
        }
        let mut sp_curves = Vec::new();
        // Faces, loops and fins.
        let mut face_records = Vec::new();
        for (s, shell) in solid.shells.iter().enumerate() {
            for face in &shell.faces {
                let face_index = self.reserve();
                // A face with a tolerant edge keeps the kernel's own surface,
                // whose parameters its SP-curves are in.
                let has_tolerant = face.loops.iter().flat_map(|l| &l.coedges).any(|c| tolerant.contains_key(&c.edge_id));
                let surface = if has_tolerant {
                    let index = self.reserve();
                    let id = self.id();
                    self.b_surface(&face.surface, index, vec![("node_id", int(id)), ("owner", ptr(face_index)), ("sense", Value::Char(b'+'))])?
                } else {
                    self.surface(&face.surface, face_index)?
                };
                let positive = face.same_sense != surface.flipped;
                let mut loop_indices = Vec::new();
                for lp in &face.loops {
                    let coedges: Vec<&crate::topology::CoedgeRecord> = lp
                        .coedges
                        .iter()
                        .filter(|c| edges_by_id.get(&c.edge_id).is_some_and(|e| !e.degenerate))
                        .collect();
                    let fins: Vec<(u64, bool)> = coedges.iter().map(|c| (c.edge_id, c.forward)).collect();
                    if fins.is_empty() {
                        continue;
                    }
                    let loop_index = self.reserve();
                    let fin_indices: Vec<u32> = fins.iter().map(|_| self.reserve()).collect();
                    for (k, &(edge_id, forward)) in fins.iter().enumerate() {
                        used.entry(edge_id).or_default().push((fin_indices[k], forward, loop_index));
                    }
                    let id = self.id();
                    self.put(loop_index, 15, vec![("node_id", int(id)), ("halfedge", ptr(fin_indices[0])), ("face", ptr(face_index))])?;
                    for (k, &(edge_id, forward)) in fins.iter().enumerate() {
                        let n = fin_indices.len();
                        let edge = edges_by_id[&edge_id];
                        let vertex_id = if forward { edge.end_vertex_id } else { edge.start_vertex_id };
                        let vertex = *vertex_index.get(&vertex_id).ok_or_else(|| format!("edge {edge_id} has an unknown vertex"))?;
                        let mut fields = vec![
                            ("loop", ptr(loop_index)),
                            ("forward", ptr(fin_indices[(k + 1) % n])),
                            ("backward", ptr(fin_indices[(k + n - 1) % n])),
                            ("vertex", ptr(vertex)),
                            ("sense", Value::Char(if forward { b'+' } else { b'-' })),
                        ];
                        if tolerant.contains_key(&edge_id) {
                            let start_id = if forward { edge.start_vertex_id } else { edge.end_vertex_id };
                            let ends = (vertex_points[&start_id], vertex_points[&vertex_id]);
                            let (trimmed, basis) = self.fin_curve(&coedges[k].pcurve, surface.index, fin_indices[k], body, ends)?;
                            fields.push(("curve", ptr(trimmed)));
                            sp_curves.push(basis);
                            sp_curves.push(trimmed);
                        }
                        self.put(fin_indices[k], 17, fields)?;
                    }
                    loop_indices.push(loop_index);
                }
                for pair in loop_indices.windows(2) {
                    self.set(pair[0], "next", ptr(pair[1]))?;
                }
                face_records.push((s, face_index, surface.index, positive, loop_indices.first().copied().unwrap_or(0)));
            }
        }
        // Edges, their curves and the fin pairing.
        let mut edge_nodes = Vec::new();
        let mut curve_nodes = Vec::new();
        let mut vertex_fins: HashMap<u32, Vec<u32>> = HashMap::default();
        for edge in &solid.edges {
            let Some(fins) = used.get(&edge.id) else { continue };
            let (positive, negative) = match fins.as_slice() {
                [a, b] if a.1 && !b.1 => (a.0, b.0),
                [a, b] if !a.1 && b.1 => (b.0, a.0),
                _ => return Err(format!("edge {} is used by {} fins, not one each way; only manifold solids are exported", edge.id, fins.len())),
            };
            let edge_index = self.reserve();
            let (curve, tolerance) = match tolerant.get(&edge.id) {
                Some(&t) => (0, Some(t)),
                None => {
                    let curve = self.curve(edge, edge_index)?;
                    curve_nodes.push(curve);
                    (curve, None)
                }
            };
            let id = self.id();
            self.put(
                edge_index,
                16,
                vec![
                    ("node_id", int(id)),
                    ("tolerance", Value::Real(tolerance)),
                    ("halfedge", ptr(positive)),
                    ("curve", ptr(curve)),
                    ("owner", ptr(body)),
                ],
            )?;
            for (fin, other) in [(positive, negative), (negative, positive)] {
                self.set(fin, "other", ptr(other))?;
                self.set(fin, "edge", ptr(edge_index))?;
                let vertex = match &self.nodes[&fin].values[self.nodes[&fin].layout.fields.iter().position(|f| f.name == "vertex").unwrap()] {
                    Value::Int(Some(v)) => *v as u32,
                    _ => 0,
                };
                vertex_fins.entry(vertex).or_default().push(fin);
            }
            edge_nodes.push(edge_index);
        }
        // Fin chains at vertices, vertices and points.
        let mut vertex_nodes = Vec::new();
        let mut points = Vec::new();
        let vertex_ids: HashMap<u32, u64> = vertex_index.iter().map(|(&id, &v)| (v, id)).collect();
        for (vertex, point, position) in point_nodes {
            let fins = vertex_fins.get(&vertex).cloned().unwrap_or_default();
            if fins.is_empty() {
                continue; // a pole vertex only a degenerate edge reached
            }
            for pair in fins.windows(2) {
                self.set(pair[0], "next_at_vx", ptr(pair[1]))?;
            }
            let id = self.id();
            self.put(
                vertex,
                18,
                vec![
                    ("node_id", int(id)),
                    ("halfedge", ptr(fins[0])),
                    ("point", ptr(point)),
                    ("tolerance", Value::Real(vertex_ids.get(&vertex).and_then(|v| vertex_tolerance.get(v)).copied())),
                    ("owner", ptr(body)),
                ],
            )?;
            let id = self.id();
            self.put(point, 29, vec![("node_id", int(id)), ("owner", ptr(vertex)), ("pvec", metres(position))])?;
            vertex_nodes.push(vertex);
            points.push(point);
        }
        // Faces, chained in their shells.
        let mut back: HashMap<usize, Vec<u32>> = HashMap::default();
        let mut surfaces = Vec::new();
        for &(s, face, surface, positive, first_loop) in &face_records {
            let id = self.id();
            self.put(
                face,
                14,
                vec![
                    ("node_id", int(id)),
                    ("tolerance", Value::Real(None)),
                    ("loop", ptr(first_loop)),
                    ("shell", ptr(solid_shells[s])),
                    ("surface", ptr(surface)),
                    ("sense", Value::Char(if positive { b'+' } else { b'-' })),
                    ("front_shell", ptr(void_shells[s])),
                ],
            )?;
            back.entry(s).or_default().push(face);
            surfaces.push(surface);
        }
        for faces in back.values() {
            for pair in faces.windows(2) {
                self.set(pair[0], "next", ptr(pair[1]))?;
                self.set(pair[1], "previous", ptr(pair[0]))?;
                self.set(pair[0], "next_front", ptr(pair[1]))?;
                self.set(pair[1], "previous_front", ptr(pair[0]))?;
            }
        }
        // Shells and regions.
        for s in 0..solid.shells.len() {
            let faces = back.get(&s).cloned().unwrap_or_default();
            let head = faces.first().copied().unwrap_or(0);
            let id = self.id();
            self.put(solid_shells[s], 13, vec![("node_id", int(id)), ("body", ptr(body)), ("face", ptr(head)), ("region", ptr(material))])?;
            let void_region = if s == outer { infinite } else { cavity_regions[cavities.iter().position(|&c| c == s).unwrap()] };
            let id = self.id();
            self.put(void_shells[s], 13, vec![("node_id", int(id)), ("front_face", ptr(head)), ("region", ptr(void_region))])?;
        }
        // The solid region's shells: the outer first, then the cavities'.
        let mut material_shells = vec![solid_shells[outer]];
        material_shells.extend(cavities.iter().map(|&c| solid_shells[c]));
        for pair in material_shells.windows(2) {
            self.set(pair[0], "next", ptr(pair[1]))?;
        }
        let mut regions = vec![(infinite, void_shells[outer], b'V'), (material, material_shells[0], b'S')];
        for (k, &c) in cavities.iter().enumerate() {
            regions.push((cavity_regions[k], void_shells[c], b'V'));
        }
        for (k, &(region, shell, kind)) in regions.iter().enumerate() {
            let id = self.id();
            let next = regions.get(k + 1).map_or(0, |r| r.0);
            let previous = if k == 0 { 0 } else { regions[k - 1].0 };
            self.put(
                region,
                19,
                vec![("node_id", int(id)), ("body", ptr(body)), ("next", ptr(next)), ("previous", ptr(previous)), ("shell", ptr(shell)), ("type", Value::Char(kind))],
            )?;
        }
        // Body chains of edges, vertices and geometry.
        self.chain(&edge_nodes, "next", "previous")?;
        self.chain(&vertex_nodes, "next", "previous")?;
        let mut boundary_surfaces = surfaces.clone();
        boundary_surfaces.dedup();
        self.chain(&boundary_surfaces, "next", "previous")?;
        curve_nodes.extend(sp_curves);
        self.chain(&curve_nodes, "next", "previous")?;
        self.chain(&points, "next", "previous")?;
        // The body's name and colour, as Parasolid's SDL/TYSA attributes
        // (the definitions as Parasolid 30.1 writes them).
        let mut attributes = Vec::new();
        if let Some(name) = export.name.filter(|n| !n.is_empty()) {
            let definition = self.attribute_definition("SDL/TYSA_NAME", 8017, [0; 8], [true, true, true, true, true, true, true, true, true, true, true, true, true, false], 3)?;
            let values = self.add(84, vec![("values", Value::Text(name.to_string()))])?;
            attributes.push((definition, values));
        }
        if let Some([r, g, b]) = export.color {
            let definition = self.attribute_definition("SDL/TYSA_COLOUR_2", 8040, [0, 0, 0, 0, 3, 5, 0, 0], [true, true, true, false, false, false, false, false, false, false, false, false, false, false], 2)?;
            let values = self.add(83, vec![("values", Value::Array([r, g, b].iter().map(|&x| Value::Real(Some(x))).collect()))])?;
            attributes.push((definition, values));
        }
        let attribute_nodes: Vec<u32> = attributes.iter().map(|_| self.reserve()).collect();
        for (k, &(definition, values)) in attributes.iter().enumerate() {
            let id = self.id();
            let next = attribute_nodes.get(k + 1).copied().unwrap_or(0);
            let previous = if k == 0 { 0 } else { attribute_nodes[k - 1] };
            self.put(
                attribute_nodes[k],
                81,
                vec![
                    ("node_id", int(id)),
                    ("definition", ptr(definition)),
                    ("owner", ptr(body)),
                    ("next", ptr(next)),
                    ("previous", ptr(previous)),
                    ("fields", Value::Array(vec![ptr(values)])),
                ],
            )?;
        }
        let highest = self.next_id;
        self.put(
            body,
            12,
            vec![
                ("highest_node_id", int(highest)),
                ("attributes_features", ptr(attribute_nodes.first().copied().unwrap_or(0))),
                ("res_size", Value::Real(Some(1e3))),
                ("res_linear", Value::Real(Some(1e-8))),
                ("state", int(1)),
                ("body_type", int(1)),
                ("nom_geom_state", int(1)),
                ("shell", ptr(solid_shells[outer])),
                ("boundary_surface", ptr(boundary_surfaces.first().copied().unwrap_or(0))),
                ("boundary_curve", ptr(curve_nodes.first().copied().unwrap_or(0))),
                ("boundary_point", ptr(points.first().copied().unwrap_or(0))),
                ("region", ptr(infinite)),
                ("edge", ptr(edge_nodes.first().copied().unwrap_or(0))),
                ("vertex", ptr(vertex_nodes.first().copied().unwrap_or(0))),
            ],
        )?;
        Ok(body)
    }

    /// An attribute definition (§5.4): its identifier, type, actions, the
    /// node types it may be attached to and its one field's type (2 real, 3
    /// string). Definitions are chained through `next`, each written once.
    fn attribute_definition(&mut self, identifier: &str, type_id: i64, actions: [i64; 8], owners: [bool; 14], field: i64) -> Result<u32, String> {
        if let Some(&index) = self.definitions.get(identifier) {
            return Ok(index);
        }
        let index = self.reserve();
        let name = self.add(79, vec![("string", Value::Text(identifier.to_string()))])?;
        self.put(
            index,
            80,
            vec![
                ("identifier", ptr(name)),
                ("type_id", int(type_id)),
                ("actions", Value::Array(actions.iter().map(|&a| int(a)).collect())),
                ("field_names", ptr(0)),
                ("legal_owners", Value::Array(owners.iter().map(|&o| Value::Char(if o { b'T' } else { b'F' })).collect())),
                ("fields", Value::Array(vec![int(field)])),
            ],
        )?;
        if let Some(&last) = self.definition_order.last() {
            self.set(last, "next", ptr(index))?;
        }
        self.definition_order.push(index);
        self.definitions.insert(identifier.to_string(), index);
        Ok(index)
    }

    fn chain(&mut self, nodes: &[u32], next: &str, previous: &str) -> Result<(), String> {
        for pair in nodes.windows(2) {
            self.set(pair[0], next, ptr(pair[1]))?;
            self.set(pair[1], previous, ptr(pair[0]))?;
        }
        Ok(())
    }

    /// A face's surface, owned by the face.
    fn surface(&mut self, surface: &NurbsSurface, face: u32) -> Result<WrittenSurface, String> {
        let index = self.reserve();
        let id = self.id();
        let common = |fields: &mut Vec<(&str, Value)>| {
            fields.push(("node_id", int(id)));
            fields.push(("owner", ptr(face)));
            fields.push(("sense", Value::Char(b'+')));
        };
        let mut fields = Vec::new();
        common(&mut fields);
        let analytic = surface.analytic().cloned();
        let (node_type, flipped) = match analytic {
            Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
                let (Ok(normal), Ok(x)) = (u_dir.cross(v_dir).normalized(), u_dir.normalized()) else {
                    return self.b_surface(surface, index, fields);
                };
                fields.push(("pvec", metres(origin)));
                fields.push(("normal", unit(normal)));
                fields.push(("x_axis", unit(x)));
                (50, false)
            }
            Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) => {
                let flipped = height < 0.0;
                let scale = 1.0 + rho0.abs().max(rho1.abs());
                if (rho1 - rho0).abs() <= 1e-9 * scale {
                    if rho0 <= 0.0 {
                        return self.b_surface(surface, index, fields);
                    }
                    fields.push(("pvec", metres(frame.origin)));
                    fields.push(("axis", unit(frame.axis)));
                    fields.push(("radius", Value::Real(Some(rho0 / MM_PER_METRE))));
                    fields.push(("x_axis", unit(frame.x_axis)));
                    (51, flipped)
                } else {
                    // The radius grows along the written axis (as written
                    // files have it), from rho0 at the origin.
                    let slope = (rho1 - rho0) / height;
                    // The point the radius is given at: the generatrix's
                    // start, or its end when the start is the apex.
                    let (origin, radius) = if rho0 > 0.0 { (frame.origin, rho0) } else { (frame.origin.add(frame.axis.scale(height)), rho1) };
                    if radius <= 0.0 {
                        return self.b_surface(surface, index, fields);
                    }
                    let axis = if slope >= 0.0 { frame.axis } else { frame.axis.scale(-1.0) };
                    let half = slope.abs().atan();
                    fields.push(("pvec", metres(origin)));
                    fields.push(("axis", unit(axis)));
                    fields.push(("radius", Value::Real(Some(radius / MM_PER_METRE))));
                    fields.push(("sin_half_angle", Value::Real(Some(half.sin()))));
                    fields.push(("cos_half_angle", Value::Real(Some(half.cos()))));
                    fields.push(("x_axis", unit(frame.x_axis)));
                    (52, flipped)
                }
            }
            Some(AnalyticSurface::Sphere { frame, radius }) => {
                fields.push(("centre", metres(frame.origin)));
                fields.push(("radius", Value::Real(Some(radius / MM_PER_METRE))));
                fields.push(("axis", unit(frame.axis)));
                fields.push(("x_axis", unit(frame.x_axis)));
                (53, false)
            }
            Some(AnalyticSurface::Torus { frame, major_radius, minor_radius }) => {
                if minor_radius >= major_radius {
                    return self.b_surface(surface, index, fields);
                }
                fields.push(("centre", metres(frame.origin)));
                fields.push(("axis", unit(frame.axis)));
                fields.push(("major_radius", Value::Real(Some(major_radius / MM_PER_METRE))));
                fields.push(("minor_radius", Value::Real(Some(minor_radius / MM_PER_METRE))));
                fields.push(("x_axis", unit(frame.x_axis)));
                (54, false)
            }
            _ => return self.b_surface(surface, index, fields),
        };
        self.put(index, node_type, fields)?;
        Ok(WrittenSurface { index, flipped })
    }

    /// A B_SURFACE and its NURBS_SURF: the kernel's net, positions in
    /// metres, rational vertices premultiplied with the weight last, v
    /// varying fastest (§5.2.2.7).
    fn b_surface(&mut self, surface: &NurbsSurface, index: u32, mut fields: Vec<(&str, Value)>) -> Result<WrittenSurface, String> {
        let rational = surface.control_points.iter().flatten().any(|p| (p.w - 1.0).abs() > 1e-12);
        let nu = surface.control_points.len();
        let nv = surface.control_points.first().map_or(0, Vec::len);
        let dim = if rational { 4 } else { 3 };
        let mut vertices = Vec::with_capacity(nu * nv * dim);
        for row in &surface.control_points {
            for p in row {
                vertices.push(Value::Real(Some(p.x / MM_PER_METRE)));
                vertices.push(Value::Real(Some(p.y / MM_PER_METRE)));
                vertices.push(Value::Real(Some(p.z / MM_PER_METRE)));
                if rational {
                    vertices.push(Value::Real(Some(p.w)));
                }
            }
        }
        let (uk, um) = knot_runs(&surface.knots_u);
        let (vk, vm) = knot_runs(&surface.knots_v);
        let nurbs = self.reserve();
        let bv = self.add(45, vec![("vertices", Value::Array(vertices))])?;
        let ukm = self.add(127, vec![("mult", Value::Array(um.iter().map(|&m| int(m as i64)).collect()))])?;
        let vkm = self.add(127, vec![("mult", Value::Array(vm.iter().map(|&m| int(m as i64)).collect()))])?;
        let uks = self.add(128, vec![("knots", Value::Array(uk.iter().map(|&k| Value::Real(Some(k))).collect()))])?;
        let vks = self.add(128, vec![("knots", Value::Array(vk.iter().map(|&k| Value::Real(Some(k))).collect()))])?;
        self.put(
            nurbs,
            126,
            vec![
                ("u_periodic", Value::Char(b'F')),
                ("v_periodic", Value::Char(b'F')),
                ("u_degree", int(surface.degree_u as i64)),
                ("v_degree", int(surface.degree_v as i64)),
                ("n_u_vertices", int(nu as i64)),
                ("n_v_vertices", int(nv as i64)),
                ("u_knot_type", int(1)),
                ("v_knot_type", int(1)),
                ("n_u_knots", int(uk.len() as i64)),
                ("n_v_knots", int(vk.len() as i64)),
                ("rational", Value::Char(if rational { b'T' } else { b'F' })),
                ("u_closed", Value::Char(b'F')),
                ("v_closed", Value::Char(b'F')),
                ("surface_form", int(1)),
                ("vertex_dim", int(dim as i64)),
                ("bspline_vertices", ptr(bv)),
                ("u_knot_mult", ptr(ukm)),
                ("v_knot_mult", ptr(vkm)),
                ("u_knots", ptr(uks)),
                ("v_knots", ptr(vks)),
            ],
        )?;
        fields.push(("nurbs", ptr(nurbs)));
        self.put(index, 124, fields)?;
        Ok(WrittenSurface { index, flipped: false })
    }

    /// An edge's curve over the edge, start to end, owned by the edge.
    fn curve(&mut self, edge: &crate::topology::EdgeRecord, owner: u32) -> Result<u32, String> {
        let curve = brep_topology::edge_subcurve::edge_subcurve(edge)?;
        let index = self.reserve();
        let id = self.id();
        let mut fields = vec![("node_id", int(id)), ("owner", ptr(owner)), ("sense", Value::Char(b'+'))];
        let straight = curve.degree == 1 && curve.control_points.len() == 2 && curve.control_points.iter().all(|p| (p.w - 1.0).abs() <= 1e-12);
        if straight {
            let start = curve.control_points[0].point()?;
            let end = curve.control_points[1].point()?;
            if let Ok(direction) = end.sub(start).normalized() {
                fields.push(("pvec", metres(start)));
                fields.push(("direction", unit(direction)));
                self.put(index, 30, fields)?;
                return Ok(index);
            }
        }
        if let Some(arc) = crate::step::recognize_circular_arc(&curve) {
            fields.push(("centre", metres(arc.center)));
            fields.push(("normal", unit(arc.axis)));
            fields.push(("x_axis", unit(arc.x_axis)));
            fields.push(("radius", Value::Real(Some(arc.radius / MM_PER_METRE))));
            self.put(index, 31, fields)?;
            return Ok(index);
        }
        self.b_curve(&curve, index, fields)?;
        Ok(index)
    }

    /// A tolerant edge's fin curve: a TRIMMED_CURVE running along the fin
    /// (from the fin's start to its vertex), over an SP_CURVE whose 2D B-curve
    /// is the coedge's curve in its face's surface parameters. Returns
    /// (trimmed, SP-curve).
    fn fin_curve(&mut self, pcurve: &NurbsCurve, surface: u32, fin: u32, body: u32, ends: (Vec3, Vec3)) -> Result<(u32, u32), String> {
        let [p0, p1] = pcurve.domain()?;
        let basis = self.reserve();
        let b_curve = self.reserve();
        let rational = pcurve.control_points.iter().any(|p| (p.w - 1.0).abs() > 1e-12);
        let dim = if rational { 3 } else { 2 };
        let mut vertices = Vec::new();
        for p in &pcurve.control_points {
            vertices.push(Value::Real(Some(p.x)));
            vertices.push(Value::Real(Some(p.y)));
            if rational {
                vertices.push(Value::Real(Some(p.w)));
            }
        }
        let (knots, mults) = knot_runs(&pcurve.knots);
        let nurbs = self.reserve();
        let bv = self.add(45, vec![("vertices", Value::Array(vertices))])?;
        let km = self.add(127, vec![("mult", Value::Array(mults.iter().map(|&m| int(m as i64)).collect()))])?;
        let ks = self.add(128, vec![("knots", Value::Array(knots.iter().map(|&k| Value::Real(Some(k))).collect()))])?;
        self.put(
            nurbs,
            136,
            vec![
                ("degree", int(pcurve.degree as i64)),
                ("n_vertices", int(pcurve.control_points.len() as i64)),
                ("vertex_dim", int(dim)),
                ("n_knots", int(knots.len() as i64)),
                ("knot_type", int(1)),
                ("periodic", Value::Char(b'F')),
                ("closed", Value::Char(b'F')),
                ("rational", Value::Char(if rational { b'T' } else { b'F' })),
                ("curve_form", int(1)),
                ("bspline_vertices", ptr(bv)),
                ("knot_mult", ptr(km)),
                ("knots", ptr(ks)),
            ],
        )?;
        self.put(b_curve, 134, vec![("sense", Value::Char(b'+')), ("nurbs", ptr(nurbs))])?;
        let id = self.id();
        self.put(basis, 137, vec![("node_id", int(id)), ("owner", ptr(body)), ("sense", Value::Char(b'+')), ("surface", ptr(surface)), ("b_curve", ptr(b_curve))])?;
        let trimmed = self.reserve();
        let id = self.id();
        self.put(
            trimmed,
            133,
            vec![
                ("node_id", int(id)),
                ("owner", ptr(fin)),
                ("sense", Value::Char(b'+')),
                ("basis_curve", ptr(basis)),
                ("point_1", metres(ends.0)),
                ("point_2", metres(ends.1)),
                ("parm_1", Value::Real(Some(p0))),
                ("parm_2", Value::Real(Some(p1))),
            ],
        )?;
        Ok((trimmed, basis))
    }

    fn b_curve(&mut self, curve: &NurbsCurve, index: u32, mut fields: Vec<(&str, Value)>) -> Result<(), String> {
        let rational = curve.control_points.iter().any(|p| (p.w - 1.0).abs() > 1e-12);
        let dim = if rational { 4 } else { 3 };
        let mut vertices = Vec::new();
        for p in &curve.control_points {
            vertices.push(Value::Real(Some(p.x / MM_PER_METRE)));
            vertices.push(Value::Real(Some(p.y / MM_PER_METRE)));
            vertices.push(Value::Real(Some(p.z / MM_PER_METRE)));
            if rational {
                vertices.push(Value::Real(Some(p.w)));
            }
        }
        let (knots, mults) = knot_runs(&curve.knots);
        let nurbs = self.reserve();
        let bv = self.add(45, vec![("vertices", Value::Array(vertices))])?;
        let km = self.add(127, vec![("mult", Value::Array(mults.iter().map(|&m| int(m as i64)).collect()))])?;
        let ks = self.add(128, vec![("knots", Value::Array(knots.iter().map(|&k| Value::Real(Some(k))).collect()))])?;
        let closed = match (curve.control_points.first(), curve.control_points.last()) {
            (Some(a), Some(b)) => a.point()?.sub(b.point()?).length() <= 1e-9,
            _ => false,
        };
        self.put(
            nurbs,
            136,
            vec![
                ("degree", int(curve.degree as i64)),
                ("n_vertices", int(curve.control_points.len() as i64)),
                ("vertex_dim", int(dim as i64)),
                ("n_knots", int(knots.len() as i64)),
                ("knot_type", int(1)),
                ("periodic", Value::Char(b'F')),
                ("closed", Value::Char(if closed { b'T' } else { b'F' })),
                ("rational", Value::Char(if rational { b'T' } else { b'F' })),
                ("curve_form", int(1)),
                ("bspline_vertices", ptr(bv)),
                ("knot_mult", ptr(km)),
                ("knots", ptr(ks)),
            ],
        )?;
        fields.push(("nurbs", ptr(nurbs)));
        self.put(index, 134, fields)
    }
}

/// The kernel shell that bounds the others: the one whose signed volume is
/// positive (an outer shell faces out, a cavity's into the cavity), the
/// largest if several are.
fn outer_shell(solid: &BrepSolid) -> Result<usize, String> {
    if solid.shells.is_empty() {
        return Err("the solid has no shells".into());
    }
    if solid.shells.len() == 1 {
        return Ok(0);
    }
    let volume = |s: usize| {
        let mut one = solid.clone();
        one.shells = vec![solid.shells[s].clone()];
        one.mass_properties_cache = Default::default();
        crate::solid_signed_volume(&one).unwrap_or(f64::NAN)
    };
    let volumes: Vec<f64> = (0..solid.shells.len()).map(volume).collect();
    (0..solid.shells.len())
        .filter(|&s| volumes[s] > 0.0)
        .max_by(|&a, &b| volumes[a].total_cmp(&volumes[b]))
        .ok_or_else(|| format!("no shell of the solid encloses positive volume ({volumes:?})"))
}

/// Distinct knots and their multiplicities.
fn knot_runs(knots: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut values: Vec<f64> = Vec::new();
    let mut mults: Vec<usize> = Vec::new();
    for &k in knots {
        match values.last() {
            Some(&last) if (k - last).abs() <= 0.0 => *mults.last_mut().unwrap() += 1,
            _ => {
                values.push(k);
                mults.push(1);
            }
        }
    }
    (values, mults)
}

fn default_value(kind: Kind) -> Value {
    match kind {
        Kind::Int => Value::Int(Some(0)),
        Kind::Real => Value::Real(None),
        Kind::Vector => Value::Vector(None),
        Kind::Interval => Value::Interval([None; 2]),
        Kind::Box => Value::Box([None; 6]),
        Kind::Char | Kind::Unknown => Value::Char(b'?'),
    }
}

fn int(v: i64) -> Value {
    Value::Int(Some(v))
}

fn ptr(index: u32) -> Value {
    Value::Int(Some(i64::from(index)))
}

fn metres(p: Vec3) -> Value {
    Value::Vector(Some([p.x / MM_PER_METRE, p.y / MM_PER_METRE, p.z / MM_PER_METRE]))
}

fn unit(v: Vec3) -> Value {
    Value::Vector(Some([v.x, v.y, v.z]))
}
