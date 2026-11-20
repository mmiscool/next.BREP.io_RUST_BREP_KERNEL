//! XT body to Part 21: the front end that puts Parasolid on the STEP lane.
//!
//! Like the IGES reader, this says the body again in STEP and imports that,
//! so an XT body gets the STEP builder, closure gate, pcurve derivation,
//! seam passes, trim repair and refusals. XT is in metres (§4.2.1) and the
//! STEP text is read as millimetres, so every length is scaled by 1000.
//!
//! Orientation (§4.3, §5.2.8, §5.3.9): a face's normal is its surface's
//! natural normal when the face and surface senses agree; a loop runs with
//! the face on its left about that normal; a fin's vertex is its forward
//! vertex, so an edge runs from `halfedge.other.vertex` to
//! `halfedge.vertex`; and the edge runs along its curve's parameter when the
//! curve's sense is positive.
//!
//! Exact where STEP has the entity: swept and spun surfaces become
//! extrusions and revolutions (a spun line or circle the analytic surface
//! it is), offsets of non-analytic surfaces OFFSET_SURFACEs. Intersection
//! curves and most SP-curves have none: an intersection curve becomes a
//! cubic through its chart (points on both surfaces, §5.2.1.5), an SP-curve
//! a cubic through points of its 2D curve on its surface (exact on a plane
//! and along an analytic's iso-lines); the STEP lane then derives pcurves
//! from them as it does for any vendor's approximated 3D curve. A
//! rolling-ball blend becomes a B-surface through its exact cross-sections.
//!
//! `BREP_DEBUG_XT_STEP=<path>` writes each body's translated text to
//! `<path>.<body index>`.

use rustc_hash::FxHashMap as HashMap;

use super::geometry::{self, Curve, Frame, Surface};
use super::reader::{Node, XtFile};
use crate::{ImportedColor, NurbsCurve, NurbsSurface, Vec3};

use super::geometry::MM_PER_METRE;

/// Shortest round-trip real, Part 21 style (a `.` before any exponent).
fn real(value: f64) -> String {
    let text = format!("{value:E}");
    if text.contains('.') {
        text
    } else {
        text.replacen('E', ".E", 1)
    }
}

/// What one XT body became.
pub struct BodyStep {
    pub step: String,
    pub name: Option<String>,
    pub color: Option<ImportedColor>,
    /// Edges whose 3D curve was fitted (intersection or SP-curve).
    pub fitted_curves: usize,
    /// Intersection curves fitted through their chart alone (see
    /// `ParasolidImportReport::unverified_curves`).
    pub unverified_curves: usize,
}

/// An attribute's definition name and its first field's node.
fn attributes<'f>(file: &'f XtFile, node: &Node) -> Vec<(&'f str, Option<&'f Node>)> {
    let mut out = Vec::new();
    let mut at = node.ptr("attributes_features");
    let mut guard = 0;
    while let Some(attribute) = file.node(at) {
        guard += 1;
        if guard > 10_000 {
            break;
        }
        if attribute.type_name() == "ATTRIBUTE" {
            let name = file
                .follow(attribute, "definition")
                .and_then(|d| file.follow(d, "identifier"))
                .and_then(|id| id.text("string"))
                .unwrap_or("");
            let first = attribute.ints("fields").first().and_then(|&i| u32::try_from(i).ok()).and_then(|i| file.node(i));
            out.push((name, first));
        }
        at = attribute.ptr("next");
    }
    out
}

fn color_of(file: &XtFile, node: &Node, definition: &str) -> Option<ImportedColor> {
    attributes(file, node).into_iter().find(|(name, _)| *name == definition).and_then(|(_, field)| {
        let rgb = field?.reals("values");
        (rgb.len() >= 3 && rgb[..3].iter().all(|c| c.is_finite() && (0.0..=1.0).contains(c)))
            .then(|| ImportedColor { r: rgb[0], g: rgb[1], b: rgb[2] })
    })
}

/// A body's name: `SDL/TYSA_NAME` (or its unicode twin) on the body.
pub fn name_of(file: &XtFile, node: &Node) -> Option<String> {
    for (name, field) in attributes(file, node) {
        let Some(field) = field else { continue };
        let text = match name {
            "SDL/TYSA_NAME" => field.text("values").map(str::to_string),
            "SDL/TYSA_UNAME" => {
                let units: Vec<u16> = field.ints("values").into_iter().map(|c| c as u16).collect();
                String::from_utf16(&units).ok()
            }
            _ => None,
        };
        if let Some(text) = text.map(|t| t.trim_end_matches('\0').trim().to_string()).filter(|t| !t.is_empty()) {
            return Some(text);
        }
    }
    None
}

/// Whether a curve runs along the parameter of the geometry the translator
/// writes for it: its own sense, reversed again for each basis of sense `-`
/// beneath a trimmed curve. A trimmed curve runs from `parm_1` to `parm_2`
/// with its basis's direction, so over a `-` basis it runs down the
/// basis's parameter (written files put `parm_1 > parm_2` there), while the
/// translator writes the basis as it is parameterised.
pub(super) fn runs_forward(file: &XtFile, node: &Node) -> bool {
    let mut forward = node.char("sense") != Some(b'-');
    let mut at = node;
    for _ in 0..8 {
        if at.type_name() != "TRIMMED_CURVE" {
            break;
        }
        let Some(basis) = file.follow(at, "basis_curve") else { break };
        if basis.char("sense") == Some(b'-') {
            forward = !forward;
        }
        at = basis;
    }
    forward
}

/// Walk a `next`-linked chain from `first`.
pub(super) fn chain<'f>(file: &'f XtFile, first: u32, next: &'static str) -> impl Iterator<Item = &'f Node> + 'f {
    let mut at = first;
    let mut seen = rustc_hash::FxHashSet::default();
    std::iter::from_fn(move || {
        let node = file.node(at)?;
        if !seen.insert(at) {
            return None;
        }
        at = node.ptr(next);
        Some(node)
    })
}

struct EmittedSurface {
    id: usize,
    /// The STEP entity's normal is opposite the XT natural normal.
    flipped: bool,
}

struct Translator<'a> {
    file: &'a XtFile,
    text: String,
    next_id: usize,
    vertices: HashMap<u32, usize>,
    edges: HashMap<u32, usize>,
    surfaces: HashMap<u32, usize>,
    emitted: Vec<EmittedSurface>,
    fitted_curves: usize,
    unverified_curves: usize,
    /// While an edge's curve is written: the edge's (start, end) vertex
    /// positions, and then where it begins and ends along the curve.
    edge_ends: Option<(Vec3, Vec3)>,
    /// ...and each vertex's position with its tolerance (mm, 0 when exact).
    edge_end_tolerances: Vec<(Vec3, f64)>,
    anchor: Option<(Vec3, Vec3)>,
    /// Where each ring edge written so far closes (mm): the vertex STEP
    /// gives it.
    ring_points: HashMap<u32, Vec3>,
    /// Blend spines, fitted once each, or the reason one cannot be: a
    /// blend's boundary edges ask for its spine as well as its face, and a
    /// spine that does not fit is costly to fail again.
    spines: HashMap<u32, Result<Curve, String>>,
    /// Read an SP-curve on a blend through the blend's own parameters: only
    /// once no fin of the edge reads otherwise, the other face's statement
    /// of the edge being the better one where it has one.
    read_blend_sp: bool,
    /// Tolerant vertices' points as written, each found the first time it
    /// is asked for ([`Translator::seat_of`]), by vertex index.
    seats: std::cell::RefCell<HashMap<u32, Vec3>>,
}

/// Translate one BODY node to a Part 21 document.
pub fn body_to_step(file: &XtFile, body: &Node, tag: usize) -> Result<BodyStep, String> {
    let mut t = Translator {
        file,
        text: String::new(),
        next_id: 1,
        vertices: HashMap::default(),
        edges: HashMap::default(),
        surfaces: HashMap::default(),
        emitted: Vec::new(),
        fitted_curves: 0,
        unverified_curves: 0,
        edge_ends: None,
        edge_end_tolerances: Vec::new(),
        anchor: None,
        ring_points: HashMap::default(),
        spines: HashMap::default(),
        read_blend_sp: false,
        seats: Default::default(),
    };
    t.body(body)?;
    // The loosest tolerance the file gives its edges and vertices (at least
    // its linear resolution), said the STEP way so the import report checks
    // the geometry against it as it does a STEP file's stated precision.
    let loosest = file
        .nodes()
        .iter()
        .filter(|n| matches!(n.type_name(), "EDGE" | "VERTEX"))
        .filter_map(|n| n.real("tolerance"))
        .chain(body.real("res_linear"))
        .filter(|t| t.is_finite() && *t > 0.0)
        .fold(0.0f64, f64::max);
    if loosest > 0.0 {
        let unit = t.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))".into());
        t.add(format!(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{unit},'distance_accuracy_value','Parasolid tolerance')",
            real(loosest * MM_PER_METRE)
        ));
    }
    let document =format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n", t.text);
    if let Ok(path) = std::env::var("BREP_DEBUG_XT_STEP") {
        // A debugging aid must not be able to fail the import.
        let _ = std::fs::write(format!("{path}.{tag}"), &document);
    }
    Ok(BodyStep {
        step: document,
        name: name_of(file, body),
        color: color_of(file, body, "SDL/TYSA_COLOUR_2").or_else(|| color_of(file, body, "SDL/TYSA_COLOUR")),
        fitted_curves: t.fitted_curves,
        unverified_curves: t.unverified_curves,
    })
}

impl<'a> Translator<'a> {
    fn add(&mut self, body: String) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        self.text.push_str(&format!("#{id}={body};\n"));
        id
    }

    /// A CARTESIAN_POINT at `p` (mm).
    fn point_id(&mut self, p: Vec3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", real(p.x), real(p.y), real(p.z)))
    }

    fn direction_id(&mut self, d: Vec3) -> usize {
        self.add(format!("DIRECTION('',({},{},{}))", real(d.x), real(d.y), real(d.z)))
    }

    fn placement_id(&mut self, origin: Vec3, z: Vec3, x: Vec3) -> usize {
        let origin = self.point_id(origin);
        let z = self.direction_id(z);
        let x = self.direction_id(x);
        self.add(format!("AXIS2_PLACEMENT_3D('',#{origin},#{z},#{x})"))
    }

    fn node(&self, index: u32, what: &str) -> Result<&'a Node, String> {
        self.file.node(index).ok_or_else(|| format!("{what} {index} is missing"))
    }

    // -----------------------------------------------------------------
    // Topology
    // -----------------------------------------------------------------

    fn body(&mut self, body: &Node) -> Result<(), String> {
        let body_type = body.int("body_type").unwrap_or(0);
        let regions: Vec<&Node> = chain(self.file, body.ptr("region"), "next").collect();
        match body_type {
            1 => {
                let mut solids = Vec::new();
                for region in regions.iter().filter(|r| r.char("type") == Some(b'S')) {
                    let shells: Vec<&Node> = chain(self.file, region.ptr("shell"), "next").collect();
                    if shells.is_empty() {
                        continue;
                    }
                    // In a solid body every face has the solid on its back,
                    // so each solid shell lists its faces on the back chain.
                    let mut emitted = Vec::new();
                    for shell in &shells {
                        let faces: Vec<&Node> = chain(self.file, shell.ptr("face"), "next").collect();
                        if faces.is_empty() {
                            return Err(format!("solid shell {} has no faces", shell.index));
                        }
                        let mut ids = Vec::with_capacity(faces.len());
                        for face in faces {
                            ids.push(self.face(face)?);
                        }
                        let bounds = self.shell_box(shell);
                        emitted.push((ids, bounds));
                    }
                    // The outer shell is the one whose box holds every other
                    // shell's box; failing that, the largest.
                    let holds = |outer: &([f64; 3], [f64; 3]), inner: &([f64; 3], [f64; 3])| {
                        let slack = 1e-6;
                        (0..3).all(|k| outer.0[k] <= inner.0[k] + slack && inner.1[k] <= outer.1[k] + slack)
                    };
                    let diagonal = |b: &([f64; 3], [f64; 3])| (0..3).map(|k| (b.1[k] - b.0[k]).max(0.0).powi(2)).sum::<f64>();
                    let outer = (0..emitted.len())
                        .find(|&i| (0..emitted.len()).all(|j| j == i || holds(&emitted[i].1, &emitted[j].1)))
                        .or_else(|| {
                            (0..emitted.len()).max_by(|&a, &b| {
                                diagonal(&emitted[a].1).partial_cmp(&diagonal(&emitted[b].1)).unwrap_or(std::cmp::Ordering::Equal)
                            })
                        })
                        .unwrap_or(0);
                    let mut shell_ids = Vec::new();
                    for (ids, _) in &emitted {
                        let list: Vec<String> = ids.iter().map(|id| format!("#{id}")).collect();
                        shell_ids.push(self.add(format!("CLOSED_SHELL('',({}))", list.join(","))));
                    }
                    let solid = if emitted.len() == 1 {
                        self.add(format!("MANIFOLD_SOLID_BREP('',#{})", shell_ids[0]))
                    } else {
                        // A void's faces point into the void, away from the
                        // material, as a STEP void shell's do once reversed.
                        let voids: Vec<String> = (0..shell_ids.len())
                            .filter(|&i| i != outer)
                            .map(|i| format!("#{}", self.add(format!("ORIENTED_CLOSED_SHELL('',*,#{},.F.)", shell_ids[i]))))
                            .collect();
                        self.add(format!("BREP_WITH_VOIDS('',#{},({}))", shell_ids[outer], voids.join(",")))
                    };
                    solids.push(solid);
                }
                if solids.is_empty() {
                    return Err("the solid body has no solid region".into());
                }
                self.representation("ADVANCED_BREP_SHAPE_REPRESENTATION", &solids);
            }
            3 => {
                let mut shells = Vec::new();
                // A sheet's faces have void on both sides: each appears on a
                // shell's back or front chain (or both), and a closed sheet's
                // faces on the shells of both its void regions. Take each once
                // in the body.
                let mut seen = rustc_hash::FxHashSet::default();
                for region in &regions {
                    for shell in chain(self.file, region.ptr("shell"), "next") {
                        let mut ids = Vec::new();
                        let back = chain(self.file, shell.ptr("face"), "next");
                        let front: Vec<&Node> = chain(self.file, shell.ptr("front_face"), "next_front").collect();
                        for face in back.chain(front) {
                            if seen.insert(face.index) {
                                ids.push(self.face(face)?);
                            }
                        }
                        if ids.is_empty() {
                            continue;
                        }
                        let list: Vec<String> = ids.iter().map(|id| format!("#{id}")).collect();
                        shells.push(self.add(format!("OPEN_SHELL('',({}))", list.join(","))));
                    }
                }
                if shells.is_empty() {
                    return Err("the sheet body has no faces".into());
                }
                let list: Vec<String> = shells.iter().map(|id| format!("#{id}")).collect();
                let model = self.add(format!("SHELL_BASED_SURFACE_MODEL('',({}))", list.join(",")));
                self.representation("MANIFOLD_SURFACE_SHAPE_REPRESENTATION", &[model]);
            }
            2 => {
                // A wire body: one void region whose shells each chain their
                // wireframe edges (§4.3.12, §5.3.6); each edge's vertices are
                // on its (dummy) fins (§5.3.9.1), and a ring edge has none.
                let mut sets = Vec::new();
                for region in &regions {
                    for shell in chain(self.file, region.ptr("shell"), "next") {
                        let mut ids = Vec::new();
                        for edge in chain(self.file, shell.ptr("edge"), "next") {
                            ids.push(self.edge(edge.index)?);
                        }
                        if ids.is_empty() {
                            continue;
                        }
                        let list: Vec<String> = ids.iter().map(|id| format!("#{id}")).collect();
                        sets.push(self.add(format!("CONNECTED_EDGE_SET('',({}))", list.join(","))));
                    }
                }
                if sets.is_empty() {
                    return Err("the wire body has no edges".into());
                }
                let list: Vec<String> = sets.iter().map(|id| format!("#{id}")).collect();
                let model = self.add(format!("EDGE_BASED_WIREFRAME_MODEL('',({}))", list.join(",")));
                self.representation("EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION", &[model]);
            }
            6 => return Err("the body is a general (non-manifold) body, which is not read yet".into()),
            other => return Err(format!("the body has type {other}, which is not a Parasolid body type")),
        }
        Ok(())
    }

    fn representation(&mut self, kind: &str, items: &[usize]) {
        let list: Vec<String> = items.iter().map(|id| format!("#{id}")).collect();
        self.add(format!("{kind}('',({}),$)", list.join(",")));
    }

    /// A shell's bounding box (mm): which of a region's shells holds the
    /// others. Read from its vertices, the arcs of its circle and ellipse
    /// edges between their vertices (an arc bulges past them), its B-curve
    /// edges, and its loopless faces' closed surfaces (a whole sphere or
    /// torus has no edge).
    fn shell_box(&self, shell: &Node) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut include = |p: Vec3| {
            for (k, c) in [p.x, p.y, p.z].into_iter().enumerate() {
                lo[k] = lo[k].min(c);
                hi[k] = hi[k].max(c);
            }
        };
        for face in chain(self.file, shell.ptr("face"), "next") {
            // A face with no edge (no loop, or only isolated-vertex loops) is
            // a whole closed surface less points: bound it by the surface.
            let edgeless = chain(self.file, face.ptr("loop"), "next").all(|lp| self.fins(lp).iter().all(|fin| fin.ptr("edge") == 0));
            if edgeless {
                let surface = self.file.follow(face, "surface").and_then(|s| geometry::surface(self.file, s, 0).ok());
                let reach = match surface {
                    Some(Surface::Sphere { frame, radius }) => Some((frame.origin, radius)),
                    Some(Surface::Torus { frame, major, minor }) => Some((frame.origin, major + minor)),
                    _ => None,
                };
                if let Some((centre, r)) = reach {
                    include(centre.sub(Vec3::new(r, r, r)));
                    include(centre.add(Vec3::new(r, r, r)));
                }
            }
            for lp in chain(self.file, face.ptr("loop"), "next") {
                for fin in self.fins(lp) {
                    if let Ok(p) = self.vertex_location(fin.ptr("vertex")) {
                        include(p);
                    }
                    if let Some(edge) = self.file.follow(fin, "edge") {
                        for p in self.edge_samples(edge) {
                            include(p);
                        }
                    }
                }
            }
        }
        (lo, hi)
    }

    /// Points along an edge's curve between its vertices, for bounding.
    fn edge_samples(&self, edge: &Node) -> Vec<Vec3> {
        self.edge_samples_n(edge, 16, 0)
    }

    /// `pieces + 1` points along an edge's curve between its vertices, from
    /// its negative fin's vertex to its positive fin's, and with `per_span`
    /// above 0, `per_span - 1` more in each knot span of a B-curve.
    fn edge_samples_n(&self, edge: &Node, pieces: usize, per_span: usize) -> Vec<Vec3> {
        let Some(curve) = self.file.follow(edge, "curve").and_then(|c| geometry::curve(self.file, c).ok()) else {
            return Vec::new();
        };
        let ends = edge_fins(self.file, edge).ok().and_then(|(positive, negative)| {
            Some((self.vertex_location(negative.ptr("vertex")).ok()?, self.vertex_location(positive.ptr("vertex")).ok()?))
        });
        let forward = self.file.follow(edge, "curve").map_or(true, |c| runs_forward(self.file, c));
        let range = match (&curve, ends) {
            (Curve::Line { .. }, _) => return Vec::new(),
            (Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. }, Some((start, end))) => {
                // A vertex's parameter: the polar angle on a circle, the
                // eccentric angle on an ellipse.
                let (major, minor) = match curve {
                    Curve::Ellipse { major, minor, .. } => (major, minor),
                    _ => (1.0, 1.0),
                };
                let angle = |p: Vec3| {
                    let d = p.sub(frame.origin);
                    (d.dot(frame.y()) / minor).atan2(d.dot(frame.x) / major)
                };
                let (a, b) = (angle(start), angle(end));
                let sweep = if forward { (b - a).rem_euclid(std::f64::consts::TAU) } else { -(a - b).rem_euclid(std::f64::consts::TAU) };
                let sweep = if sweep == 0.0 { std::f64::consts::TAU } else { sweep };
                [a, a + sweep]
            }
            (Curve::Nurbs(nurbs), Some((start, end))) => {
                let at = |p: Vec3| crate::project_point_to_curve(nurbs, p).map(|f| f.u).ok();
                match (at(start), at(end)) {
                    (Some(a), Some(b)) if a != b => [a, b],
                    _ => match curve.natural_range() {
                        Some(r) => r,
                        None => return Vec::new(),
                    },
                }
            }
            (_, None) => match curve.natural_range() {
                Some(r) => r,
                None => return Vec::new(),
            },
        };
        let mut at: Vec<f64> = (0..=pieces).map(|k| range[0] + (range[1] - range[0]) * k as f64 / pieces as f64).collect();
        if let (Curve::Nurbs(nurbs), true) = (&curve, per_span > 0) {
            // Eight more points in each of the B-curve's knot spans the
            // edge crosses, however narrow, so none goes unsampled.
            let (lo, hi) = (range[0].min(range[1]), range[0].max(range[1]));
            for w in nurbs.knots.windows(2) {
                let (a, b) = (w[0].max(lo), w[1].min(hi));
                if b > a {
                    at.extend((1..per_span).map(|k| a + (b - a) * k as f64 / per_span as f64));
                }
            }
            let forward = range[1] >= range[0];
            at.sort_by(|a, b| if forward { a.total_cmp(b) } else { b.total_cmp(a) });
        }
        at.into_iter().filter_map(|t| curve.point(t).ok()).collect()
    }

    /// The fins of a loop in order: `forward` from `loop.halfedge`.
    fn fins(&self, lp: &Node) -> Vec<&'a Node> {
        let start = lp.ptr("halfedge");
        let mut out = Vec::new();
        let mut at = start;
        while let Some(fin) = self.file.node(at) {
            out.push(fin);
            at = fin.ptr("forward");
            if at == start || out.len() > 1_000_000 {
                break;
            }
        }
        out
    }

    fn face(&mut self, face: &Node) -> Result<usize, String> {
        let face_id = face.int("node_id").unwrap_or(face.index as i64);
        let surface_index = face.ptr("surface");
        let surface_node = self.node(surface_index, "the surface of a face").map_err(|e| format!("face {face_id}: {e}"))?;
        let emitted = if surface_node.type_name() == "BLENDED_EDGE" {
            self.blend_surface(face, surface_node)
        } else if let Some(band) = self.helical_band(face, surface_node) {
            Ok(band)
        } else if let Some(pinned) = self.pinned_offset(face, surface_node) {
            Ok(pinned)
        } else if let Some(extended) = self.extended_offset(face, surface_node) {
            Ok(extended)
        } else if let Some(turned) = self.turned_lemon(face, surface_node) {
            Ok(turned)
        } else {
            self.surface(surface_node)
        }
        .map_err(|e| format!("face {face_id}: {e}"))?;
        let flipped = self.emitted[emitted].flipped;
        let step_surface = self.emitted[emitted].id;
        let agrees = (face.char("sense") != Some(b'-')) == (surface_node.char("sense") != Some(b'-'));
        let same_sense = agrees != flipped;
        let mut bounds = Vec::new();
        for lp in chain(self.file, face.ptr("loop"), "next") {
            let fins = self.fins(lp);
            let loop_id = if fins.len() == 1 && fins[0].ptr("edge") == 0 {
                // An isolated loop: one fin naming the vertex (§5.3.8.1).
                let vertex = self.vertex(fins[0].ptr("vertex")).map_err(|e| format!("face {face_id}: {e}"))?;
                self.add(format!("VERTEX_LOOP('',#{vertex})"))
            } else {
                let mut members = Vec::with_capacity(fins.len());
                for fin in fins {
                    let edge = self.edge(fin.ptr("edge")).map_err(|e| format!("face {face_id}: {e}"))?;
                    let sense = if fin.char("sense") == Some(b'-') { ".F." } else { ".T." };
                    members.push(format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{edge},{sense})"))));
                }
                self.add(format!("EDGE_LOOP('',({}))", members.join(",")))
            };
            // An XT loop runs with the face on its left about the face
            // normal, which is how a STEP bound runs about the face's normal.
            bounds.push(format!("#{}", self.add(format!("FACE_BOUND('',#{loop_id},.T.)"))));
        }
        if bounds.is_empty() {
            // A face with no loop is the whole closed surface (§5.3.6). STEP
            // bounds even that, so say it as the kernel's own writer does: a
            // sphere by its seam meridian, a torus by its two seam circles.
            let surface = geometry::surface(self.file, surface_node, 0).map_err(|e| format!("face {face_id}: {e}"))?;
            let loop_id = self.whole_surface_loop(&surface, same_sense).map_err(|e| format!("face {face_id}: {e}"))?;
            bounds.push(format!("#{}", self.add(format!("FACE_OUTER_BOUND('',#{loop_id},.T.)"))));
        }
        let id = self.add(format!(
            "ADVANCED_FACE('',({}),#{step_surface},{})",
            bounds.join(","),
            if same_sense { ".T." } else { ".F." }
        ));
        if let Some(color) = color_of(self.file, face, "SDL/TYSA_COLOUR") {
            self.face_style(id, color);
        }
        Ok(id)
    }

    /// A lemon torus face with a vertex beside the seam of the surface
    /// written for it ([`SEAM_ROW_NEAR`]: near the arc its revolve starts at,
    /// in the frame's x direction, but not on it), as its own revolve turned
    /// about the axis to start in the middle of the widest turn between the
    /// face's vertices. A STEP reader seams a face along its surface's first
    /// section through the vertices on it; a vertex just beside that section
    /// leaves the face's loop open in the surface's parameters by that much.
    /// The turned surface is kept only when no vertex is beside its own seam
    /// either, as a blend's is ([`Self::widest_vertex_gap`]).
    fn turned_lemon(&mut self, face: &Node, surface_node: &Node) -> Option<usize> {
        if surface_node.type_name() != "TORUS" {
            return None;
        }
        let Ok(Surface::Torus { frame, major, minor }) = geometry::surface(self.file, surface_node, 0) else { return None };
        if !(major < 0.0) {
            return None;
        }
        let points: Vec<Vec3> = chain(self.file, face.ptr("loop"), "next")
            .into_iter()
            .flat_map(|lp| self.fins(lp))
            .filter_map(|fin| self.vertex_location(fin.ptr("vertex")).ok())
            .collect();
        let turned = turned_lemon_frame(frame, major, minor, &points)?;
        let Ok(Surface::Spun { profile: Curve::Nurbs(arc), axis_point, axis }) = geometry::lemon(turned, major, minor) else { return None };
        // Oriented as the torus, as the shared lemon surface is.
        let id = self.emit_nurbs_surface(&revolve(&arc, axis_point, axis).ok()?);
        self.emitted.push(EmittedSurface { id, flipped: false });
        Some(self.emitted.len() - 1)
    }

    /// The offset of a B-surface whose boundary row a face's vertex closes
    /// (a keycap's dish: the base's arc sections shrink to the offset distance,
    /// so the offset's last section shrinks to that point) as a B-surface with
    /// that row pinned to the vertex. Offset at a radius of curvature equal to
    /// its distance, the row's ends meet at the vertex but its middle runs out
    /// and back along itself (0.086 mm on the keycap, the way out and back
    /// within 1e-3 mm: no area), so a face's loop up one side, through the
    /// vertex and down the other cannot close in the surface's parameters.
    /// The offset is the carrier a STEP reader makes of the OFFSET_SURFACE
    /// otherwise written; only that row's poles move, onto the vertex.
    fn pinned_offset(&mut self, face: &Node, surface_node: &Node) -> Option<usize> {
        if surface_node.type_name() != "OFFSET_SURF" {
            return None;
        }
        let exact = geometry::surface(self.file, surface_node, 0).ok()?;
        let Surface::Offset { base, base_sense, distance } = &exact else { return None };
        let Surface::Nurbs(base) = base.as_ref() else { return None };
        // Read on the exact offset first, so only a face that needs it pays
        // for the fit.
        let domain = [base.domain_u().ok()?, base.domain_v().ok()?];
        let boundary = self.face_boundary_points(face);
        let pins = self.folded_rows(face, &exact, domain, &boundary);
        if pins.is_empty() {
            return None;
        }
        // As the OFFSET_SURFACE would be written and read back (an
        // unflipped B-surface basis offset along its own normal).
        let along = if *base_sense { *distance } else { -*distance };
        let carrier = crate::FaceRecord { id: 0, surface: base.clone(), same_sense: true, loops: Vec::new(), name: None };
        let offset = crate::offset::offset_surface_for_import(&carrier, -along).or_else(|_| crate::offset_surface(&carrier, -along, 0.0)).ok()?;
        if [offset.domain_u().ok()?, offset.domain_v().ok()?] != domain {
            return None;
        }
        let offset = pin_rows(offset, &pins)?;
        // The poles moved bend the surface across their spans: every point
        // of the face's boundary must still lie on it (cross review: an
        // edge beside the row would leave its face).
        if !boundary.iter().all(|p| lies_on(&offset, *p, PIN_ON)) {
            return None;
        }
        self.fitted_curves += 1;
        let id = self.emit_nurbs_surface(&offset);
        self.emitted.push(EmittedSurface { id, flipped: false });
        Some(self.emitted.len() - 1)
    }

    /// The boundary rows of `exact` over `domain` (u at each end, then v:
    /// sides 0 to 3) that close on a vertex of the face out and back along
    /// themselves ([`Translator::pinned_offset`]), each with its vertex.
    fn folded_rows(&self, face: &Node, exact: &Surface, [[u0, u1], [v0, v1]]: [[f64; 2]; 2], boundary: &[Vec3]) -> Vec<(usize, Vec3)> {
        let vertices: Vec<Vec3> = chain(self.file, face.ptr("loop"), "next")
            .into_iter()
            .flat_map(|lp| self.fins(lp))
            .filter_map(|fin| self.vertex_location(fin.ptr("vertex")).ok())
            .collect();
        let at = |side: usize, t: f64| match side {
            0 => exact.point(u0, v0 + (v1 - v0) * t),
            1 => exact.point(u1, v0 + (v1 - v0) * t),
            2 => exact.point(u0 + (u1 - u0) * t, v0),
            _ => exact.point(u0 + (u1 - u0) * t, v1),
        };
        let mut pins = Vec::new();
        for side in 0..4 {
            let (Ok(a), Ok(b)) = (at(side, 0.0), at(side, 1.0)) else { continue };
            let Some(&vertex) = vertices.iter().find(|v| a.sub(**v).length() <= PIN_ENDS && b.sub(**v).length() <= PIN_ENDS) else { continue };
            let samples: Vec<Vec3> = (0..=PIN_SAMPLES).filter_map(|k| at(side, k as f64 / PIN_SAMPLES as f64).ok()).collect();
            if samples.len() != PIN_SAMPLES + 1 {
                continue;
            }
            let excursion = samples.iter().map(|p| p.sub(vertex).length()).fold(0.0, f64::max);
            let width = (0..=PIN_SAMPLES).map(|k| samples[k].sub(samples[PIN_SAMPLES - k]).length()).fold(0.0, f64::max);
            // Out and back along itself, and no edge of the face along it:
            // a narrow closed boundary of the face would have its edges
            // there (cross review). The row's middle half stands at least a
            // quarter of its excursion from the vertex.
            let middle: Vec<Vec3> = samples[PIN_SAMPLES / 4..=3 * PIN_SAMPLES / 4].to_vec();
            let bounded = boundary.iter().any(|b| middle.iter().any(|m| m.sub(*b).length() <= PIN_FIN * excursion));
            if excursion > PIN_ENDS && excursion <= PIN_EXCURSION && width <= PIN_FIN * excursion && !bounded {
                pins.push((side, vertex));
            }
        }
        pins
    }

    /// The offset of a B-surface whose face runs past the base's domain, as
    /// Parasolid lets it: the base goes on along its boundary tangent planes
    /// (a keycap dish's triangle closes at an apex 0.3 mm past its base's
    /// last row, where that row's offset is 0.09 mm wide). An OFFSET_SURFACE
    /// would be read on the base's own domain and the face could not close,
    /// so the base is extended on each side the face's SP-curves cross, just
    /// as far as they go, and written as the B-surface a STEP reader makes
    /// of its offset. None for a face within its base, and when the
    /// extended offset does not carry every point of the face's boundary.
    fn extended_offset(&mut self, face: &Node, surface_node: &Node) -> Option<usize> {
        if surface_node.type_name() != "OFFSET_SURF" {
            return None;
        }
        let exact = geometry::surface(self.file, surface_node, 0).ok()?;
        let Surface::Offset { base, base_sense, distance } = &exact else { return None };
        let Surface::Nurbs(base) = base.as_ref() else { return None };
        let [u0, u1] = base.domain_u().ok()?;
        let [v0, v1] = base.domain_v().ok()?;
        // How far the face's SP-curves on this offset reach, over the
        // stretch each fin's trim keeps: read at even steps and in every knot
        // span, each extreme then refined between its neighbours (cross
        // review: a narrow excursion between even steps).
        let (mut low, mut high) = ([u0, v0], [u1, v1]);
        let mut on_curves: Vec<[f64; 2]> = Vec::new();
        for fin in chain(self.file, face.ptr("loop"), "next").into_iter().flat_map(|lp| self.fins(lp)) {
            let mut curve = self.file.follow(fin, "curve");
            let mut trim: Option<[f64; 2]> = None;
            for _ in 0..8 {
                let Some(c) = curve.filter(|c| c.type_name() == "TRIMMED_CURVE") else { break };
                if let (Some(a), Some(b)) = (c.real("parm_1"), c.real("parm_2")) {
                    trim = Some([a.min(b), a.max(b)]);
                }
                curve = self.file.follow(c, "basis_curve");
            }
            let Some(sp) = curve.filter(|c| c.type_name() == "SP_CURVE" && c.ptr("surface") == surface_node.index) else { continue };
            let uv = geometry::parameter_curve(self.file, self.file.follow(sp, "b_curve")?).ok()?;
            let [t0, t1] = uv.domain().ok()?;
            let [a, b] = trim.map(|[a, b]| [a.max(t0), b.min(t1)]).filter(|[a, b]| b > a).unwrap_or([t0, t1]);
            let mut ts: Vec<f64> = (0..=EXTEND_SAMPLES).map(|k| a + (b - a) * k as f64 / EXTEND_SAMPLES as f64).collect();
            for w in uv.knots.windows(2) {
                let (c, d) = (w[0].max(a), w[1].min(b));
                if d > c {
                    ts.extend((0..=8).map(|k| c + (d - c) * k as f64 / 8.0));
                }
            }
            ts.sort_by(f64::total_cmp);
            ts.dedup();
            let read = |t: f64| uv.evaluate(t).ok().map(|p| [p.x, p.y]);
            let points: Vec<[f64; 2]> = ts.iter().map(|&t| read(t)).collect::<Option<_>>()?;
            on_curves.extend(points.iter().copied());
            for p in &points {
                low = [low[0].min(p[0]), low[1].min(p[1])];
                high = [high[0].max(p[0]), high[1].max(p[1])];
            }
            // Each coordinate's least and greatest: every sample that stands
            // at least as far as both its neighbours is refined between them
            // by golden-section search (cross review: the best sample can sit
            // on a plateau beside a narrower, higher peak).
            let golden = 0.5 * (5f64.sqrt() - 1.0);
            for axis in 0..2 {
                for sign in [-1.0, 1.0] {
                    let value = |t: f64| read(t).map_or(f64::NEG_INFINITY, |p| sign * p[axis]);
                    let values: Vec<f64> = points.iter().map(|p| sign * p[axis]).collect();
                    for i in 0..ts.len() {
                        let (before, after) = (i.saturating_sub(1), (i + 1).min(ts.len() - 1));
                        if values[i] < values[before] || values[i] < values[after] {
                            continue;
                        }
                        let (mut lo, mut hi) = (ts[before], ts[after]);
                        for _ in 0..60 {
                            let (x1, x2) = (hi - golden * (hi - lo), lo + golden * (hi - lo));
                            if value(x1) >= value(x2) { hi = x2 } else { lo = x1 }
                        }
                        let extreme = read(0.5 * (lo + hi))?;
                        on_curves.push(extreme);
                        low = [low[0].min(extreme[0]), low[1].min(extreme[1])];
                        high = [high[0].max(extreme[0]), high[1].max(extreme[1])];
                    }
                }
            }
        }
        let (closed_u, closed_v) = base.closed_directions().ok()?;
        let past = |reach: f64, span: f64, closed: bool| if !closed && reach > EXTEND_PAST * span { reach } else { 0.0 };
        let sides = [
            (true, true, past(u0 - low[0], u1 - u0, closed_u)),
            (true, false, past(high[0] - u1, u1 - u0, closed_u)),
            (false, true, past(v0 - low[1], v1 - v0, closed_v)),
            (false, false, past(high[1] - v1, v1 - v0, closed_v)),
        ];
        if sides.iter().all(|s| s.2 == 0.0) {
            return None;
        }
        let mut extended = base.clone();
        for (along_u, at_low, delta) in sides {
            if delta > 0.0 {
                extended = tangent_extension(&extended, along_u, at_low, delta)?;
            }
        }
        // The strip follows the tangent planes the offset reads exactly only
        // where the weights hold across the side (cross review): it must
        // stand within the boundary's bar of them over the whole strip
        // (8.8e-6 mm on the keycap, at its apex).
        let [eu, ev] = [extended.domain_u().ok()?, extended.domain_v().ok()?];
        for i in 0..=EXTEND_GRID {
            for j in 0..=EXTEND_GRID {
                let (u, v) = (eu[0] + (eu[1] - eu[0]) * i as f64 / EXTEND_GRID as f64, ev[0] + (ev[1] - ev[0]) * j as f64 / EXTEND_GRID as f64);
                if !(u < u0 || u > u1 || v < v0 || v > v1) {
                    continue;
                }
                let tangent = base.derivatives_extended(u, v, 0).ok()?[0][0];
                if extended.evaluate(u, v).ok()?.sub(tangent).length() > EXTEND_ON {
                    return None;
                }
            }
        }
        // As the OFFSET_SURFACE would be written and read back (an
        // unflipped B-surface basis offset along its own normal).
        let along = if *base_sense { *distance } else { -*distance };
        let carrier = crate::FaceRecord { id: 0, surface: extended, same_sense: true, loops: Vec::new(), name: None };
        let domain = [carrier.surface.domain_u().ok()?, carrier.surface.domain_v().ok()?];
        let offset = crate::offset::offset_surface_for_import(&carrier, -along).or_else(|_| crate::offset_surface(&carrier, -along, 0.0)).ok()?;
        if [offset.domain_u().ok()?, offset.domain_v().ok()?] != domain {
            return None;
        }
        // Run on to a vertex, a side can close there out and back along
        // itself as a pinned offset's does: pinned the same way.
        let mut boundary = self.face_boundary_points(face);
        let pins = self.folded_rows(face, &exact, domain, &boundary);
        let offset = if pins.is_empty() { offset } else { pin_rows(offset, &pins)? };
        // The SP-curves' own points too, at every sample their reach was
        // read at (cross review: an edge's sampling could miss an excursion).
        boundary.extend(on_curves.iter().filter_map(|p| exact.point(p[0], p[1]).ok()));
        if !boundary.iter().all(|p| lies_on(&offset, *p, EXTEND_ON)) {
            return None;
        }
        self.fitted_curves += 1;
        let id = self.emit_nurbs_surface(&offset);
        self.emitted.push(EmittedSurface { id, flipped: false });
        Some(self.emitted.len() - 1)
    }

    /// A cylinder face that winds round its axis more than once, between
    /// two helices (a modelled thread's crest or root), carried by one of its
    /// helices swept along the axis: S(w, t) = C(t) + w·axis, the stretch of
    /// w it uses. Every such sweep lies on the cylinder, but its parameters
    /// run along the helix through every turn, where the cylinder's go round
    /// once and the face's loop cannot close in them. Taken only where the
    /// helix covers every turn of the face and the face is narrower along
    /// the axis than half the helix's pitch, so the sweep, a little wider
    /// than the face and narrower than the pitch, never meets itself;
    /// None for every other face. Its normal is the cylinder's, outward.
    fn helical_band(&mut self, face: &Node, surface_node: &Node) -> Option<usize> {
        use std::f64::consts::{PI, TAU};
        if surface_node.type_name() != "CYLINDER" {
            return None;
        }
        let Ok(Surface::Cylinder { frame, radius }) = geometry::surface(self.file, surface_node, 0) else { return None };
        let axis = frame.z.normalized().ok()?;
        let (x, y) = (frame.x, frame.y());
        let angle = |p: Vec3| {
            let d = p.sub(frame.origin);
            d.dot(y).atan2(d.dot(x))
        };
        let height = |p: Vec3| p.sub(frame.origin).dot(axis);
        let wrap = |a: f64| a - TAU * (a / TAU).round();
        let loops: Vec<&Node> = chain(self.file, face.ptr("loop"), "next").collect();
        let [lp] = loops[..] else { return None };
        // The loop walked densely, each fin from its start to its end, and
        // its B-curves, which may be helices.
        let fins = self.fins(lp);
        let mut walk: Vec<Vec3> = Vec::new();
        let mut guides: Vec<(NurbsCurve, usize)> = Vec::new();
        for (k, fin) in fins.iter().enumerate() {
            let edge = self.file.follow(fin, "edge")?;
            let start = self.vertex_location(fins[(k + fins.len() - 1) % fins.len()].ptr("vertex")).ok()?;
            let end = self.vertex_location(fin.ptr("vertex")).ok()?;
            let curve = geometry::curve(self.file, self.file.follow(edge, "curve")?).ok()?;
            let pieces = match &curve {
                Curve::Line { .. } => 1,
                Curve::Circle { .. } | Curve::Ellipse { .. } => 64,
                Curve::Nurbs(nurbs) => 32 * nurbs.control_points.len(),
            };
            let mut samples = self.edge_samples_n(edge, pieces, 8);
            if fin.char("sense") == Some(b'-') {
                samples.reverse();
            }
            walk.push(start);
            if let Curve::Nurbs(nurbs) = curve {
                guides.push((nurbs, walk.len() - 1));
            }
            walk.extend(samples);
            walk.push(end);
        }
        if walk.len() < 3 {
            return None;
        }
        // Every point on the cylinder, and the walk round it unwrapped. A
        // face that goes round the axis is a whole band, not this.
        let tolerance = 1e-6 * (1.0 + radius);
        if walk.iter().any(|p| (p.sub(frame.origin).sub(axis.scale(height(*p))).length() - radius).abs() > 1e3 * tolerance) {
            return None;
        }
        let mut turned = vec![angle(walk[0])];
        for k in 1..walk.len() {
            let step = wrap(angle(walk[k]) - angle(walk[k - 1]));
            if step.abs() > 0.5 * PI {
                return None;
            }
            turned.push(turned[k - 1] + step);
        }
        let back = turned[walk.len() - 1] + wrap(angle(walk[0]) - angle(walk[walk.len() - 1]));
        let (lo, hi) = turned.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &t| (a.min(t), b.max(t)));
        if (back - turned[0]).abs() > 1e-6 || hi - lo <= TAU {
            return None;
        }
        for (guide, anchor) in guides {
            let [t0, t1] = guide.domain().ok()?;
            // The helix tabled by its angle, unwrapped, and its height, at
            // even steps and eight to each knot span, however narrow.
            let pieces = 32 * guide.control_points.len();
            let mut ts: Vec<f64> = (0..=pieces).map(|k| t0 + (t1 - t0) * k as f64 / pieces as f64).collect();
            for w in guide.knots.windows(2) {
                let (a, b) = (w[0].max(t0), w[1].min(t1));
                if b > a {
                    ts.extend((1..8).map(|k| a + (b - a) * k as f64 / 8.0));
                }
            }
            ts.sort_by(f64::total_cmp);
            ts.dedup();
            let n = ts.len() - 1;
            let mut table: Vec<(f64, f64)> = Vec::with_capacity(n + 1);
            for &t in &ts {
                let p = guide.evaluate(t).ok()?;
                let a = match table.last() {
                    Some(&(last, _)) => last + wrap(angle(p) - last),
                    None => angle(p),
                };
                table.push((a, height(p)));
            }
            let rising = table[n].0 > table[0].0;
            if table.windows(2).any(|w| (w[1].0 > w[0].0) != rising || (w[1].0 - w[0].0).abs() > 0.5 * PI) {
                continue;
            }
            // How near its turns come along the axis anywhere: a helix whose
            // pitch varies is as narrow as its closest turns (cross review).
            let turn_height = |a: f64| -> Option<f64> {
                let k = table.partition_point(|&(b, _)| if rising { b < a } else { b > a }).clamp(1, n);
                let ((a0, h0), (a1, h1)) = (table[k - 1], table[k]);
                Some(h0 + (h1 - h0) * (a - a0) / (a1 - a0))
            };
            // Each table point against the helix a turn on and a turn back,
            // so a dip between table points is met from the other turn.
            let (first, last) = (table[0].0.min(table[n].0), table[0].0.max(table[n].0));
            let pitch = table
                .iter()
                .flat_map(|&(a, h)| [a + TAU, a - TAU].map(move |b| (b, h)))
                .filter(|&(b, _)| (first..=last).contains(&b))
                .filter_map(|(b, h)| Some((turn_height(b)? - h).abs()))
                .fold(f64::INFINITY, f64::min);
            if !pitch.is_finite() {
                continue;
            }
            // The walk's anchor, a vertex of the helix's own edge, set on its turn.
            let p = walk[anchor];
            let at = crate::project_point_to_curve(&guide, p).ok()?.u;
            if guide.evaluate(at).ok()?.sub(p).length() > 1e3 * tolerance {
                continue;
            }
            let i = ts.partition_point(|&t| t <= at).clamp(1, n) - 1;
            let shift = table[i].0 + wrap(angle(p) - table[i].0) - turned[anchor];
            // Each point's height above the helix on its own turn.
            let height_at = |a: f64| -> Option<f64> {
                let (first, last) = (table[0].0.min(table[n].0), table[0].0.max(table[n].0));
                if !(first..=last).contains(&a) {
                    return None;
                }
                let k = table.partition_point(|&(b, _)| if rising { b < a } else { b > a }).clamp(1, n);
                let (a0, h0) = table[k - 1];
                let (a1, h1) = table[k];
                Some(h0 + (h1 - h0) * (a - a0) / (a1 - a0))
            };
            let Some(offsets) = walk.iter().zip(&turned).map(|(p, t)| Some(height(*p) - height_at(t + shift)?)).collect::<Option<Vec<f64>>>() else {
                continue;
            };
            let (w0, w1) = offsets.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &w| (a.min(w), b.max(w)));
            if !(w1 - w0 < 0.5 * pitch) {
                continue;
            }
            // The boundary between samples may stand a little past them
            // (cross review), so the sweep reaches a quarter of the gap the
            // pitch leaves on each side: never a whole pitch wide, so it
            // never meets itself.
            let pad = 0.25 * (pitch - (w1 - w0));
            let (mut w0, mut w1) = (w0 - pad, w1 + pad);
            // Outward: (w1 − w0)·axis × C′ points away from the axis.
            let mid = 0.5 * (t0 + t1);
            let tangent = guide.evaluate(mid + 1e-6 * (t1 - t0)).ok()?.sub(guide.evaluate(mid - 1e-6 * (t1 - t0)).ok()?);
            let out = guide.evaluate(mid).ok()?.sub(frame.origin);
            if axis.cross(tangent).dot(out) < 0.0 {
                std::mem::swap(&mut w0, &mut w1);
            }
            let row = |w: f64| {
                guide
                    .control_points
                    .iter()
                    .map(|c| crate::Vec4 { x: c.x + w * axis.x * c.w, y: c.y + w * axis.y * c.w, z: c.z + w * axis.z * c.w, w: c.w })
                    .collect::<Vec<_>>()
            };
            let band = NurbsSurface::new(1, guide.degree, vec![0.0, 0.0, 1.0, 1.0], guide.knots.clone(), vec![row(w0), row(w1)]).ok()?;
            let id = self.emit_nurbs_surface(&band);
            self.emitted.push(EmittedSurface { id, flipped: false });
            return Some(self.emitted.len() - 1);
        }
        None
    }

    /// Points on a face's boundary (mm): its vertices, points along its
    /// edges' exact curves and its intersection edges' charts.
    fn face_boundary_points(&self, face: &Node) -> Vec<Vec3> {
        let mut points = Vec::new();
        for lp in chain(self.file, face.ptr("loop"), "next") {
            for fin in self.fins(lp) {
                if let Ok(p) = self.vertex_location(fin.ptr("vertex")) {
                    points.push(p);
                }
                let Some(edge) = self.file.follow(fin, "edge") else { continue };
                points.extend(self.edge_samples(edge));
                let mut curve = self.file.follow(edge, "curve");
                for _ in 0..8 {
                    match curve {
                        Some(c) if c.type_name() == "TRIMMED_CURVE" => curve = self.file.follow(c, "basis_curve"),
                        _ => break,
                    }
                }
                if let Some(c) = curve.filter(|c| c.type_name() == "INTERSECTION") {
                    points.extend(self.intersection_points(c).unwrap_or_default());
                }
            }
        }
        points
    }

    /// Each loop of a face walked in order (mm), for telling how often it
    /// goes round a closed spine: every fin from the vertex it starts at to
    /// the one it ends at, through its edge's samples in the fin's own
    /// direction. An intersection edge's chart runs over the whole curve,
    /// past the edge's vertices (two edges can share one chart round a
    /// ring), so only its stretch between them is taken, in the fin's
    /// direction (cross review); an edge with no vertex, a whole ring, takes
    /// its chart once.
    fn face_boundary_walks(&self, face: &Node) -> Vec<Vec<Vec3>> {
        let mut walks = Vec::new();
        for lp in chain(self.file, face.ptr("loop"), "next") {
            let mut points = Vec::new();
            let fins = self.fins(lp);
            for (k, fin) in fins.iter().enumerate() {
                // A fin's vertex is the one it ends at; the fin before it
                // in the loop ends where it starts.
                let end = self.vertex_location(fin.ptr("vertex")).ok();
                let start = self.vertex_location(fins[(k + fins.len() - 1) % fins.len()].ptr("vertex")).ok();
                let along = fin.char("sense") != Some(b'-');
                let Some(edge) = self.file.follow(fin, "edge") else { continue };
                let curve_node = self.file.follow(edge, "curve");
                let mut curve = curve_node;
                for _ in 0..8 {
                    match curve {
                        Some(c) if c.type_name() == "TRIMMED_CURVE" => curve = self.file.follow(c, "basis_curve"),
                        _ => break,
                    }
                }
                points.extend(start);
                if let Some(c) = curve.filter(|c| c.type_name() == "INTERSECTION") {
                    let mut chart = self.intersection_points(c).unwrap_or_default();
                    if chart.len() >= 2 {
                        if along != curve_node.map_or(true, |n| runs_forward(self.file, n)) {
                            chart.reverse();
                        }
                        let nearest = |p: Vec3| (0..chart.len()).min_by(|&i, &j| chart[i].sub(p).length().total_cmp(&chart[j].sub(p).length())).unwrap_or(0);
                        let n = chart.len();
                        let closed = chart[0].sub(chart[n - 1]).length() <= 1e-3;
                        match (start, end) {
                            (Some(s), Some(e)) => {
                                let (i0, i1) = (nearest(s), nearest(e));
                                if closed {
                                    // Forward from the start round to the
                                    // end (the repeated last point skipped);
                                    // a fin from a vertex back to it, the
                                    // whole turn.
                                    let m = n - 1;
                                    let (i0, i1) = (i0 % m, i1 % m);
                                    let steps = if i1 > i0 { i1 - i0 } else { i1 + m - i0 };
                                    points.extend((0..=steps).map(|k| chart[(i0 + k) % m]));
                                } else if i0 <= i1 {
                                    points.extend_from_slice(&chart[i0..=i1]);
                                }
                            }
                            _ => points.extend(chart),
                        }
                    }
                } else {
                    let mut samples = self.edge_samples(edge);
                    if !along {
                        samples.reverse();
                    }
                    points.extend(samples);
                }
                points.extend(end);
            }
            walks.push(points);
        }
        walks
    }

    /// Where on a closed B-curve spine a face is seamed: the foot of its
    /// seam edge's vertex (an edge both of whose fins bound the face), else
    /// for a face of vertexless rings only, of the point its first ring
    /// closes at (written here first, so that point is known: STEP gives
    /// the ring its vertex there, and a loop of one closed edge closes in
    /// the surface's parameters only across its seam). None for any other
    /// face: a loop of several edges can cross the seam anywhere along one
    /// (seamed at one of its vertices, three files were lost; seamed at a
    /// ring beside one, keycaps that import as they are took step_import
    /// up to ten times as long).
    fn seam_spine_parameter(&mut self, face: &Node, spine: &Curve) -> Option<f64> {
        let Curve::Nurbs(curve) = spine else { return None };
        let mut seen = std::collections::HashSet::new();
        let (mut ring, mut vertexed) = (None, false);
        for lp in chain(self.file, face.ptr("loop"), "next") {
            for fin in self.fins(lp) {
                let vertex = self.vertex_location(fin.ptr("vertex")).ok();
                vertexed |= vertex.is_some();
                let edge = fin.ptr("edge");
                if edge != 0 && vertex.is_none() {
                    ring = ring.or(Some(edge));
                }
                if edge != 0 && !seen.insert(edge) {
                    if let Some(point) = vertex {
                        return crate::project_point_to_curve(curve, point).ok().map(|foot| foot.u);
                    }
                }
            }
        }
        if vertexed {
            return None;
        }
        let edge = ring?;
        self.edge(edge).ok()?;
        let point = *self.ring_points.get(&edge)?;
        crate::project_point_to_curve(curve, point).ok().map(|foot| foot.u)
    }

    /// Whether a vertex of the face lies near the section at `u` of its
    /// fitted surface but not on it ([`SEAM_ROW_NEAR`]).
    fn vertex_beside_row(&self, face: &Node, surface: &NurbsSurface, u: f64) -> bool {
        let Ok(row) = surface.iso_curve_u(u) else { return false };
        let [on, near] = SEAM_ROW_NEAR;
        chain(self.file, face.ptr("loop"), "next").into_iter().flat_map(|lp| self.fins(lp)).any(|fin| {
            let Ok(point) = self.vertex_location(fin.ptr("vertex")) else { return false };
            let Ok(foot) = crate::project_point_to_curve(&row, point) else { return false };
            row.evaluate(foot.u).is_ok_and(|p| (on..near).contains(&p.sub(point).length()))
        })
    }

    /// The middle of the widest stretch of a closed spine's period `[a, b]`
    /// between the feet of a face's vertices, where its surface's period
    /// may start with no vertex of the face on its seam.
    fn widest_vertex_gap(&self, face: &Node, spine: &Curve, [a, b]: [f64; 2]) -> Option<f64> {
        let Curve::Nurbs(curve) = spine else { return None };
        let period = b - a;
        let mut feet = Vec::new();
        for lp in chain(self.file, face.ptr("loop"), "next") {
            // A fin of a vertexless ring has no vertex to keep away from.
            for point in self.fins(lp).into_iter().filter_map(|fin| self.vertex_location(fin.ptr("vertex")).ok()) {
                let foot = crate::project_point_to_curve(curve, point).ok()?;
                feet.push(a + (foot.u - a).rem_euclid(period));
            }
        }
        feet.sort_by(f64::total_cmp);
        let first = *feet.first()?;
        feet.push(first + period);
        let (gap, from) = feet.windows(2).map(|w| (w[1] - w[0], w[0])).max_by(|x, y| x.0.total_cmp(&y.0))?;
        (gap > 0.0).then_some(from + 0.5 * gap)
    }

    /// A blend's two supports, as the file orders them, and its radius (mm).
    /// A cliff-edge blend (§5.2.2.6) rolls along one surface and over an edge:
    /// its other support is a BLENDED_EDGE of range [0, 0] whose spine is that
    /// edge's curve. A rolling-ball or cliff-edge blend supporting it is the
    /// envelope of its balls, the pipe of its radius about its spine; where
    /// that blend's own centres need refining (it rests on a pipe or an edge
    /// itself) the pipe carries its supports, so it is the pipe about the
    /// refined centres its face was built from (cross review).
    fn blend_supports(&mut self, node: &Node, depth: usize) -> Result<([Support; 2], f64), String> {
        let file = self.file;
        let id = node.int("node_id").unwrap_or(node.index as i64);
        if depth > 4 {
            return Err(format!("blend {id}: blends rest on blends more than 4 deep"));
        }
        let cliff = match node.char("blend_type") {
            Some(b'R') => false,
            Some(b'E') => true,
            _ => return Err(format!("blend {id} has a blend type that is neither rolling-ball nor cliff-edge")),
        };
        let radius = node.reals("range").iter().map(|r| r.abs()).fold(0.0f64, f64::max) * MM_PER_METRE;
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(format!("blend {id} has no radius"));
        }
        let nodes: Vec<&Node> = node.ints("surface").into_iter().filter_map(|i| u32::try_from(i).ok()).filter_map(|i| file.node(i)).collect();
        let [first, second]: [&Node; 2] = nodes.try_into().map_err(|_| format!("blend {id} does not have two supporting surfaces"))?;
        let mut read = |s: &Node| -> Result<Support, String> {
            if s.type_name() != "BLENDED_EDGE" {
                return Ok(Support::Surface(geometry::surface(file, s, 0)?));
            }
            let support_id = s.int("node_id").unwrap_or(s.index as i64);
            if cliff && s.reals("range").iter().all(|r| *r == 0.0) {
                let edge = file.follow(s, "spine").ok_or_else(|| format!("blend {id}'s cliff edge has no curve"))?;
                return Ok(Support::Cliff(self.spine_curve(edge).map_err(|e| format!("blend {id}: cliff edge: {e}"))?));
            }
            let (inner, tube) = self.blend_supports(s, depth + 1).map_err(|e| format!("blend {id}: supporting blend {support_id}: {e}"))?;
            let spine = file.follow(s, "spine").ok_or_else(|| format!("blend {id}'s supporting blend {support_id} has no spine"))?;
            let spine = self.spine_curve(spine).map_err(|e| format!("blend {id}: supporting blend {support_id}: spine: {e}"))?;
            // Carried always: its centres are refined past an open spine's
            // ends even between two surfaces (cross review).
            let inner = Some(Box::new((inner, tube)));
            Ok(Support::Pipe { spine, radius: tube, inner })
        };
        let supports = [read(first)?, read(second)?];
        if cliff && !supports.iter().any(|s| matches!(s, Support::Cliff(_))) {
            return Err(format!("blend {id} is a cliff-edge blend with no cliff edge among its supports"));
        }
        Ok((supports, radius))
    }

    /// A rolling-ball blend's surface (§5.2.2.6) over the stretch of its spine
    /// that `face` spans: at each spine point the arc of radius r about it
    /// from its foot on the first supporting surface to its foot on the
    /// second (the other way round when the spine's sense is negative),
    /// perpendicular to the spine. STEP has no such surface, so the arcs,
    /// exact rational quadratics, are skinned into a B-surface along the
    /// spine at doubling densities until the surface between its stations
    /// lies within [`FIT_TOLERANCE`] of the exact arcs there. Its u runs
    /// with the spine and its v from the first support to the second, as
    /// the blend's own parameters do.
    fn blend_surface(&mut self, face: &Node, node: &Node) -> Result<usize, String> {
        let file = self.file;
        let id = node.int("node_id").unwrap_or(node.index as i64);
        let (mut supports, radius) = self.blend_supports(node, 0)?;
        let spine_node = file.follow(node, "spine").ok_or_else(|| format!("blend {id} has no spine"))?;
        if spine_node.char("sense") == Some(b'-') {
            supports.swap(0, 1);
        }
        let spine = self.spine_curve(spine_node).map_err(|e| format!("blend {id}: spine: {e}"))?;
        let points = self.face_boundary_points(face);
        // A face with a loop that goes once round a closed spine spans all
        // of it, however sparsely its boundary is sampled (an intersection's
        // chart round the ring leaves gaps wider than the eighth of it that
        // `spine_range` takes for a missing stretch, and the face was built
        // over part of the ring with its edges 3 mm off it).
        let whole = geometry::closed_range(&spine).filter(|_| self.face_boundary_walks(face).iter().any(|lp| winds_round(&spine, lp)));
        // A whole face starts its surface's period where its seam is
        // ([`Self::seam_spine_parameter`]). Started at the spine's start
        // instead, the face's loop ran on past the surface's end and could
        // not close (a SolidWorks screw's blend round its head, its rings'
        // vertices a sixth of a millimetre on).
        let seam = whole.and_then(|_| self.seam_spine_parameter(face, &spine));
        let whole = whole.map(|[a, b]| match seam {
            Some(s) => {
                let s = a + (s - a).rem_euclid(b - a);
                [s, s + (b - a)]
            }
            None => [a, b],
        });
        let section = |t: f64| blend_section(&supports, radius, &spine, t);
        // A blend whose supports are only C1 (or C0) along some knot lines (a
        // B-surface's, or a swept B-curve's) has sections that kink where a
        // foot crosses one: fitted piece by piece between those crossings,
        // found from the supports' own knots rather than from the sections.
        let pieces = |t0: f64, t1: f64| -> Option<NurbsSurface> {
            let breaks = support_knot_crossings(&supports, &section, t0, t1);
            fit_blend_pieces(&section, t0, t1, &breaks).ok()
        };
        // Within the spine's domain first, as every blend was read; past an
        // open spine's ends (feet taken on its extension, where a face runs
        // past the spine curve) only when that refuses: the extension can run
        // where the sections fail (cross review: a SolidWorks report's blend).
        let fit = |whole: Option<[f64; 2]>, extend: bool| -> Result<(NurbsSurface, [f64; 2]), String> {
            let [t0, t1] = match whole {
                Some(range) => range,
                None => spine_range(&spine, &points, 1.0 / 50.0, extend).ok_or_else(|| "the face does not lie along its spine".to_string())?,
            };
            let surface = match fit_blend(&section, t0, t1) {
                Ok(surface) => surface,
                Err(e) => match pieces(t0, t1) {
                    Some(surface) => surface,
                    // The fiftieth past the face each way can run past where
                    // a support is (a swept section's end): the face's span.
                    None => {
                        let [a, b] = spine_range(&spine, &points, 0.0, extend).ok_or_else(|| e.clone())?;
                        pieces(a, b).ok_or(e)?
                    }
                },
            };
            Ok((surface, [t0, t1]))
        };
        let fit_either = |whole: Option<[f64; 2]>| match fit(whole, false) {
            Ok(fitted) => Ok(fitted),
            Err(e) => fit(whole, true).map_err(|_| format!("blend {id}: {e}")),
        };
        let (mut surface, [mut t0, mut t1]) = fit_either(whole)?;
        // A whole face with vertices and no seam edge is seamed by its STEP
        // reader along the surface's first section, through the vertices on
        // it. Where a ring's vertex lies beside that section instead (a
        // keycap's, 0.013 mm across the blend from where it starts), the
        // loop is left open by that much: its period starts in the widest
        // stretch between its vertices, kept only when none of them is near
        // the seam there either (cross review: a ring of many vertices can
        // leave none of its gaps wide enough).
        if let (Some([a, b]), None) = (whole, seam) {
            if self.vertex_beside_row(face, &surface, t0) {
                if let Some(start) = self.widest_vertex_gap(face, &spine, [a, b]) {
                    if let Ok((refit, range)) = fit_either(Some([start, start + (b - a)])) {
                        if !self.vertex_beside_row(face, &refit, range[0]) {
                            (surface, t0, t1) = (refit, range[0], range[1]);
                        }
                    }
                }
            }
        }
        self.fitted_curves += 1;
        let step = self.emit_nurbs_surface(&surface);
        // A blend's natural normal points away from its spine, as a torus's
        // points away from its tube's centre line (checked against the same
        // model's STEP on ABC 00000222 part 4: the volumes agree to 1e-5).
        // The fitted surface's own du × dv may point either way.
        // Sampled where the section has not collapsed.
        let flipped = [0.5, 0.3, 0.7, 0.1, 0.9]
            .into_iter()
            .find_map(|f| {
                let t = t0 + (t1 - t0) * f;
                // Wrapped, as the sections were: a ring stretch runs past
                // the spine's domain (cross review).
                let away = surface.evaluate(t, 0.5).ok()?.sub(geometry::extended_point(&spine, t).ok()?);
                Some(surface.normal(t, 0.5).ok()?.dot(away) < 0.0)
            })
            .ok_or_else(|| format!("blend {id}: no regular point to orient it by"))?;
        self.emitted.push(EmittedSurface { id: step, flipped });
        Ok(self.emitted.len() - 1)
    }

    /// A blend's spine as a curve the translator can evaluate: the exact
    /// curve, or an intersection curve fitted as an edge's is.
    fn spine_curve(&mut self, node: &Node) -> Result<Curve, String> {
        if let Some(curve) = self.spines.get(&node.index) {
            return curve.clone();
        }
        let curve = self.spine_curve_uncached(node);
        self.spines.insert(node.index, curve.clone());
        curve
    }

    fn spine_curve_uncached(&mut self, node: &Node) -> Result<Curve, String> {
        let mut node = node;
        for _ in 0..8 {
            if node.type_name() != "TRIMMED_CURVE" {
                break;
            }
            node = self.file.follow(node, "basis_curve").ok_or("a trimmed curve has no basis curve")?;
        }
        if node.type_name() == "INTERSECTION" {
            let chart = uncrowd_chart_ends(self.intersection_points(node)?, [RESOLUTION_MM; 2]);
            let crossing = self.intersection_surfaces(node).map(|[a, b]| Crossing::Surfaces(a, b));
            let stations = ChartStations::new(crossing.as_ref(), &chart);
            let (fitted, fell_back) = stations.fit()?;
            if fell_back || stations.refined(2).len() == chart.len() {
                self.unverified_curves += 1;
            }
            return Ok(Curve::Nurbs(fitted));
        }
        geometry::curve(self.file, node)
    }

    /// The EDGE_LOOP bounding a whole sphere or torus: in its parameters the
    /// rectangle's sides, counter-clockwise about the face normal (the
    /// surface normal when `same_sense`). A sphere's bottom and top sides
    /// are its poles, so its loop is the seam meridian up and back down; a
    /// torus's are the circle at v = 0 (around the axis) and the one at u = 0
    /// (around the tube), both closed at the point they share.
    fn whole_surface_loop(&mut self, surface: &Surface, same_sense: bool) -> Result<usize, String> {
        let mut members: Vec<(usize, bool)> = match surface {
            Surface::Sphere { frame, radius } => {
                let south = self.point_id(frame.origin.sub(frame.z.scale(*radius)));
                let south = self.add(format!("VERTEX_POINT('',#{south})"));
                let north = self.point_id(frame.origin.add(frame.z.scale(*radius)));
                let north = self.add(format!("VERTEX_POINT('',#{north})"));
                // The meridian at u = 0: from the south pole through +x to
                // the north, a circle about −y.
                let meridian = Curve::Circle {
                    frame: Frame { origin: frame.origin, z: frame.y().scale(-1.0), x: frame.x },
                    radius: *radius,
                };
                let curve = self.emit_curve(&meridian)?;
                let seam = self.add(format!("EDGE_CURVE('',#{south},#{north},#{curve},.T.)"));
                vec![(seam, true), (seam, false)]
            }
            Surface::Torus { major, .. } if *major < 0.0 => {
                return Err("a whole lemon torus face (no edges) is not read yet".into());
            }
            Surface::Torus { frame, major, minor } => {
                let corner = self.point_id(frame.origin.add(frame.x.scale(major + minor)));
                let corner = self.add(format!("VERTEX_POINT('',#{corner})"));
                let around = Curve::Circle { frame: *frame, radius: major + minor };
                let tube = Curve::Circle {
                    frame: Frame { origin: frame.origin.add(frame.x.scale(*major)), z: frame.y().scale(-1.0), x: frame.x },
                    radius: *minor,
                };
                let around = self.emit_curve(&around)?;
                let around = self.add(format!("EDGE_CURVE('',#{corner},#{corner},#{around},.T.)"));
                let tube = self.emit_curve(&tube)?;
                let tube = self.add(format!("EDGE_CURVE('',#{corner},#{corner},#{tube},.T.)"));
                vec![(around, true), (tube, true), (around, false), (tube, false)]
            }
            // A B-surface closed both ways (a coil spring's tube) is bounded
            // as a torus is: its seam iso curves through the corner at the
            // start of both domains, v = v0 (u varying) and u = u0.
            Surface::Nurbs(nurbs) if nurbs.closed_directions()? == (true, true) => {
                let ([u0, _], [v0, _]) = (nurbs.domain_u()?, nurbs.domain_v()?);
                let corner = self.point_id(nurbs.evaluate(u0, v0)?);
                let corner = self.add(format!("VERTEX_POINT('',#{corner})"));
                let around = self.emit_curve(&Curve::Nurbs(nurbs.iso_curve_v(v0)?))?;
                let around = self.add(format!("EDGE_CURVE('',#{corner},#{corner},#{around},.T.)"));
                let tube = self.emit_curve(&Curve::Nurbs(nurbs.iso_curve_u(u0)?))?;
                let tube = self.add(format!("EDGE_CURVE('',#{corner},#{corner},#{tube},.T.)"));
                vec![(around, true), (tube, true), (around, false), (tube, false)]
            }
            _ => return Err("a face with no loop is not a whole sphere, torus or B-surface closed both ways, which is not read yet".into()),
        };
        if !same_sense {
            members.reverse();
            for member in &mut members {
                member.1 = !member.1;
            }
        }
        let oriented: Vec<String> = members
            .into_iter()
            .map(|(edge, forward)| format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{edge},{})", if forward { ".T." } else { ".F." }))))
            .collect();
        Ok(self.add(format!("EDGE_LOOP('',({}))", oriented.join(","))))
    }

    /// The presentation chain `step_import/styles.rs` reads a face colour from.
    fn face_style(&mut self, face: usize, color: ImportedColor) {
        let rgb = self.add(format!("COLOUR_RGB('',{},{},{})", real(color.r), real(color.g), real(color.b)));
        let fill_colour = self.add(format!("FILL_AREA_STYLE_COLOUR('',#{rgb})"));
        let fill = self.add(format!("FILL_AREA_STYLE('',(#{fill_colour}))"));
        let area = self.add(format!("SURFACE_STYLE_FILL_AREA(#{fill})"));
        let side = self.add(format!("SURFACE_SIDE_STYLE('',(#{area}))"));
        let usage = self.add(format!("SURFACE_STYLE_USAGE(.BOTH.,#{side})"));
        let assignment = self.add(format!("PRESENTATION_STYLE_ASSIGNMENT((#{usage}))"));
        self.add(format!("STYLED_ITEM('',(#{assignment}),#{face})"));
    }

    fn vertex_location(&self, index: u32) -> Result<Vec3, String> {
        let tolerance = self.node(index, "vertex")?.real("tolerance").filter(|t| t.is_finite()).map_or(0.0, |t| t * MM_PER_METRE);
        if tolerance <= SNAP_ENDS_MM {
            return self.file_vertex_location(index);
        }
        if let Some(&seat) = self.seats.borrow().get(&index) {
            return Ok(seat);
        }
        let at = self.file_vertex_location(index)?;
        let seat = self.seat_of(index, at, tolerance).unwrap_or(at);
        self.seats.borrow_mut().insert(index, seat);
        Ok(seat)
    }

    /// Where the file puts vertex `index` (mm).
    fn file_vertex_location(&self, index: u32) -> Result<Vec3, String> {
        let vertex = self.node(index, "vertex")?;
        let point = self.file.follow(vertex, "point").ok_or_else(|| format!("vertex {index} has no point"))?;
        point.vector("pvec").map(geometry::position).ok_or_else(|| format!("vertex {index}'s point has no position"))
    }

    /// Where tolerant vertex `index` (at `at` in the file, `tolerance` mm) is
    /// written, when the curves meeting there miss it by more than
    /// [`snap_ends`] closes: re-seated within its own stated tolerance.
    /// Parasolid's tolerant vertex lies within its tolerance of the curves
    /// meeting there (§4.3.9), which need not end at its point: a tight
    /// edge's curve is right where it is to its own tolerance, and two such
    /// curves can end 0.16 mm apart either side of a 0.3 mm vertex, one of
    /// them 0.10 mm from it. STEP has no vertex tolerance and the curves
    /// must end at the vertex. Where every edge there is long against the
    /// tolerances at its two ends and every curve there either ends there
    /// (an open SP-curve, fitted over its trim or whole: the end of it the
    /// edge pairs with this vertex) or is an open intersection cut to it
    /// ([`Self::follower`]), the vertex is written at [`reseat`]'s point.
    /// Found from the vertex's own fins (`next_at_vx`) the first time it is
    /// asked for; `None` keeps the file's point.
    fn seat_of(&self, index: u32, at: Vec3, tolerance: f64) -> Option<Vec3> {
        let file = self.file;
        // The vertex's edges, by its fins (each edge's fin ending there),
        // with the vertex at each edge's other end.
        let mut incident = Vec::new();
        let start = file.node(index)?.ptr("halfedge");
        let mut fin_index = start;
        for _ in 0..4096 {
            let fin = file.node(fin_index)?;
            if fin.ptr("vertex") != index {
                return None;
            }
            let edge = file.node(fin.ptr("edge"))?;
            let other = file.node(fin.ptr("other"))?.ptr("vertex");
            incident.push((edge, other));
            fin_index = fin.ptr("next_at_vx");
            if fin_index == start || fin_index == 0 {
                break;
            }
        }
        if fin_index != start && fin_index != 0 {
            return None;
        }
        let mut ends = Vec::new();
        let mut followers = Vec::new();
        let mut paths = Vec::new();
        let debug = std::env::var("BREP_DEBUG_XT_RESEAT").is_ok();
        for &(edge, other) in &incident {
            // Each vertex moves within its own tolerance: an edge to a vertex
            // further than twice both tolerances keeps its length, a shorter
            // one could close up (cross review).
            let there = self.file_vertex_location(other).ok()?;
            let other_tolerance = file.node(other).and_then(|n| n.real("tolerance")).filter(|t| t.is_finite()).map_or(0.0, |t| t * MM_PER_METRE);
            if there.sub(at).length() <= 2.0 * (tolerance + other_tolerance) {
                return None;
            }
            let curve = self.edge_written_curve(edge)?;
            match self.fixed_curve_ends(curve) {
                Some((pair, path)) => {
                    let end = paired_end(pair, at, there)?;
                    let place = if end.sub(pair[0]).length() == 0.0 { 0.0 } else { (path.len() - 1) as f64 };
                    ends.push(end);
                    paths.push((path, place));
                }
                // The cut must happen at the other end too: that vertex as
                // near the chart as a seat may be (cross review).
                None => {
                    let chart = self.follower(curve)?;
                    if chart_distance(&chart, there) > FOLLOW_CHART_MM {
                        return None;
                    }
                    followers.push(chart);
                }
            }
        }
        // An intersection follows the seat within the vertex's tolerance of
        // it, as the file's vertex must lie (§4.3.9: 0.15 mm off the chart of
        // a 0.3 mm one), well within its cut's reach, and on the same passage
        // of it: along the chart, no further from where the file's vertex
        // falls than twice the tolerance (a seat by another passage would cut
        // the edge the long way round, cross review). A curve that ends there
        // is trimmed by the STEP lane where the seat falls on it: by its own
        // end, not another passage of it (cross review).
        let follows = |seat: Vec3| {
            followers.iter().all(|chart| {
                let ((_, from), (off, to)) = (place_on_chart(chart, at), place_on_chart(chart, seat));
                off <= tolerance.min(FOLLOW_CHART_MM) && chart_length_between(chart, from, to) <= 2.0 * tolerance
            }) && paths.iter().all(|(path, end)| chart_length_between(path, *end, place_on_chart(path, seat).1) <= 2.0 * tolerance)
        };
        let seat = reseat(at, tolerance, &ends, follows);
        if debug && ends.iter().any(|end| end.sub(at).length() > SNAP_ENDS_MM) {
            eprintln!(
                "xt-reseat: vertex {index} (tolerance {tolerance:.6}) ends {:?}: {}",
                ends.iter().map(|end| end.sub(at).length()).collect::<Vec<_>>(),
                seat.map_or("kept".to_string(), |seat| format!("moved {:.6} mm", seat.sub(at).length())),
            );
        }
        seat
    }

    /// The curve node an edge is written from: its own, or else a fin's
    /// (the positive fin's first), as [`Self::edge_curve`] tries them.
    fn edge_written_curve(&self, edge: &Node) -> Option<&'a Node> {
        let file = self.file;
        file.follow(edge, "curve").or_else(|| {
            let (positive, negative) = edge_fins(file, edge).ok()?;
            file.follow(positive, "curve").or_else(|| file.follow(negative, "curve"))
        })
    }

    /// The two ends (mm) of an open SP-curve, over its trim or else its
    /// whole domain, as it is fitted: an edge written from it ends where it
    /// ends; with points along it, ends included. `None` for any other
    /// curve, and for a closed SP-curve, taken whole.
    fn fixed_curve_ends(&self, curve: &Node) -> Option<([Vec3; 2], Vec<Vec3>)> {
        let file = self.file;
        let mut node = curve;
        let mut trim = None;
        for _ in 0..8 {
            if node.type_name() != "TRIMMED_CURVE" {
                break;
            }
            if let (Some(a), Some(b)) = (node.real("parm_1"), node.real("parm_2")) {
                trim = Some([a.min(b), a.max(b)]);
            }
            node = file.follow(node, "basis_curve")?;
        }
        if node.type_name() != "SP_CURVE" {
            return None;
        }
        let (surface, uv) = self.sp_curve_parts(node).ok()?;
        // One written exactly (a conic, or a B-curve on a plane) is not
        // fitted to its ends (cross review).
        if exact_sp_curve(&surface, &uv).is_some() {
            return None;
        }
        let [t0, t1] = uv.domain().ok()?;
        if uv.evaluate(t0).ok()?.sub(uv.evaluate(t1).ok()?).length() <= 1e-9 {
            return None;
        }
        let [a, b] = trim.map(|[a, b]: [f64; 2]| [a.max(t0), b.min(t1)]).filter(|[a, b]| b - a > 1e-9 * (t1 - t0)).unwrap_or([t0, t1]);
        let at = |t: f64| uv.evaluate(t).ok().and_then(|p| surface.point(p.x, p.y).ok());
        // 64 even steps, and every knot inside with the middle of each span,
        // so no span's turn is missed (cross review).
        let mut params: Vec<f64> = (0..=64).map(|k| a + (b - a) * k as f64 / 64.0).collect();
        let knots: Vec<f64> = uv.knots.iter().copied().filter(|&k| k > a && k < b).collect();
        params.extend(&knots);
        params.extend(std::iter::once(a).chain(knots.iter().copied()).zip(knots.iter().copied().chain(std::iter::once(b))).map(|(p, q)| 0.5 * (p + q)));
        params.sort_by(f64::total_cmp);
        params.dedup();
        let path = params.into_iter().map(at).collect::<Option<Vec<Vec3>>>()?;
        Some(([path[0], path[path.len() - 1]], path))
    }

    /// The chart of an open intersection an edge is cut from where its
    /// vertices fall on it ([`cut_open_chart`]): the one kind of curve that
    /// follows a re-seated vertex. `None` for any other (a line, conic or
    /// B-curve is cut or snapped by rules of its own, which a moved vertex
    /// can defeat: cross reviews), and for a closed intersection, kept whole.
    fn follower(&self, curve: &Node) -> Option<Vec<Vec3>> {
        let file = self.file;
        let mut node = curve;
        for _ in 0..8 {
            if node.type_name() != "TRIMMED_CURVE" {
                break;
            }
            node = file.follow(node, "basis_curve")?;
        }
        if node.type_name() != "INTERSECTION" {
            return None;
        }
        self.intersection_points(node).ok().filter(|chart| chart.len() >= 2 && chart[0].sub(chart[chart.len() - 1]).length() > 1e-6)
    }

    fn vertex(&mut self, index: u32) -> Result<usize, String> {
        if let Some(&id) = self.vertices.get(&index) {
            return Ok(id);
        }
        let location = self.vertex_location(index)?;
        let point = self.point_id(location);
        let id = self.add(format!("VERTEX_POINT('',#{point})"));
        self.vertices.insert(index, id);
        Ok(id)
    }

    /// The EDGE_CURVE of edge `index`.
    fn edge(&mut self, index: u32) -> Result<usize, String> {
        if let Some(&id) = self.edges.get(&index) {
            return Ok(id);
        }
        let edge = self.node(index, "edge")?;
        let edge_id = edge.int("node_id").unwrap_or(index as i64);
        let (positive, negative) = edge_fins(self.file, edge).map_err(|e| format!("edge {edge_id} {e}"))?;
        let (end_vertex, start_vertex) = (positive.ptr("vertex"), negative.ptr("vertex"));
        self.edge_ends = self.vertex_location(start_vertex).ok().zip(self.vertex_location(end_vertex).ok());
        self.edge_end_tolerances = [start_vertex, end_vertex]
            .into_iter()
            .filter_map(|v| {
                let at = self.vertex_location(v).ok()?;
                let tolerance = self.file.node(v)?.real("tolerance").filter(|t| t.is_finite() && *t > 0.0).unwrap_or(0.0);
                Some((at, tolerance * MM_PER_METRE))
            })
            .collect();
        let written = self.edge_curve(edge, positive);
        (self.edge_ends, self.anchor) = (None, None);
        self.edge_end_tolerances.clear();
        let (curve, same_sense, closed_point) = written.map_err(|e| format!("edge {edge_id}: {e}"))?;
        let (start, end) = if start_vertex == 0 || end_vertex == 0 {
            // A ring edge has no vertices (§4.3.9); STEP closes it at one.
            let p = closed_point.ok_or_else(|| format!("ring edge {edge_id}'s curve has no closing point"))?;
            self.ring_points.insert(index, p);
            let point = self.point_id(p);
            let v = self.add(format!("VERTEX_POINT('',#{point})"));
            (v, v)
        } else {
            (self.vertex(start_vertex)?, self.vertex(end_vertex)?)
        };
        let id = self.add(format!(
            "EDGE_CURVE('',#{start},#{end},#{curve},{})",
            if same_sense { ".T." } else { ".F." }
        ));
        self.edges.insert(index, id);
        Ok(id)
    }

    /// An edge on a circle or ellipse whose vertices stand off it by more
    /// than the file's resolution but within their own tolerances (§4.3.9:
    /// a tolerant vertex lies within its tolerance of the curves meeting
    /// there): the arc between the vertices' angles, in the edge's direction,
    /// as an exact rational B-curve with its ends moved onto the vertices.
    /// STEP has no vertex tolerance, and an exact conic arc that misses its
    /// vertices by more than the STEP lane's conic band is refused there
    /// (8e-3 mm off a 1 mm circle, within a 2e-2 mm vertex tolerance).
    /// `None` for every other edge, emitted as before.
    fn tolerant_conic(&self, curve: &Node, sense: bool) -> Result<Option<NurbsCurve>, String> {
        let Some((start, end)) = self.edge_ends else { return Ok(None) };
        let mut basis = curve;
        for _ in 0..8 {
            match self.file.follow(basis, "basis_curve") {
                Some(b) if basis.type_name() == "TRIMMED_CURVE" => basis = b,
                _ => break,
            }
        }
        let (frame, major, minor) = match geometry::curve(self.file, basis) {
            Ok(Curve::Circle { frame, radius }) => (frame, radius, radius),
            Ok(Curve::Ellipse { frame, major, minor }) => (frame, major, minor),
            _ => return Ok(None),
        };
        let y = frame.y();
        let angle = |p: Vec3| {
            let d = p.sub(frame.origin);
            (d.dot(y) / minor).atan2(d.dot(frame.x) / major)
        };
        let on = |t: f64| frame.origin.add(frame.x.scale(major * t.cos())).add(y.scale(minor * t.sin()));
        let tolerance_of = |p: Vec3| self.edge_end_tolerances.iter().find(|(at, _)| at.sub(p).length() <= 1e-9).map_or(0.0, |(_, t)| *t);
        let gaps = [start, end].map(|p| (p.sub(on(angle(p))).length(), tolerance_of(p)));
        let snap = 10.0 * RESOLUTION_MM;
        // Only gaps `snap_ends` closes (cross review: a 0.2 mm gap inside a
        // 0.3 mm tolerance would have kept the arc's own ends).
        if gaps.iter().all(|(gap, _)| *gap <= snap)
            || gaps.iter().any(|(gap, tolerance)| *gap > snap && (*gap > tolerance + RESOLUTION_MM || *gap > SNAP_ENDS_MM))
        {
            return Ok(None);
        }
        // Along the curve's own direction: from the edge's start when the
        // edge runs with the curve, from its end otherwise.
        let (from, to) = if sense { (start, end) } else { (end, start) };
        let a0 = angle(from);
        let mut a1 = angle(to);
        while a1 <= a0 + 1e-12 {
            a1 += std::f64::consts::TAU;
        }
        let unit = crate::make_arc(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0, a0, a1)?;
        let mut arc = unit.clone();
        for q in &mut arc.control_points {
            let p = frame.origin.add(frame.x.scale(major * q.x / q.w)).add(y.scale(minor * q.y / q.w));
            *q = crate::Vec4 { x: p.x * q.w, y: p.y * q.w, z: p.z * q.w, w: q.w };
        }
        let arc = snap_ends(arc, (from, to));
        // The STEP edge runs with the arc, which starts at `from`.
        Ok(Some(if sense { arc } else { arc.reversed()? }))
    }

    /// (STEP curve, whether the edge runs along it, a point to close a ring
    /// edge at).
    fn edge_curve(&mut self, edge: &Node, positive: &Node) -> Result<(usize, bool, Option<Vec3>), String> {
        if let Some(curve) = self.file.follow(edge, "curve") {
            let sense = runs_forward(self.file, curve);
            if let Some(arc) = self.tolerant_conic(curve, sense)? {
                return Ok((self.emit_curve(&Curve::Nurbs(arc))?, true, None));
            }
            self.anchor = self.edge_ends.map(|(start, end)| if sense { (start, end) } else { (end, start) });
            let (id, closing) = match self.curve(curve) {
                Ok(read) => read,
                // An edge curve that is an SP-curve on a blend: through the
                // blend, as for a fin's (it has no other statement).
                Err(e) if !self.read_blend_sp => {
                    // A whole cross-section of a blend: placed by its vertices
                    // (the spine point the radius from both), as a fin's is.
                    // Only when it reads: it centres on the raw spine, which
                    // past an open spine's end the refined sections leave
                    // (cross review); the SP-curve path below refines.
                    if let Some((blend, uv)) = self.blend_under(curve) {
                        if let Some(Ok(section)) = self.blend_cross_section(blend, &uv).transpose() {
                            return Ok(section);
                        }
                    }
                    self.read_blend_sp = true;
                    let again = self.curve(curve);
                    self.read_blend_sp = false;
                    again.map_err(|_| e)?
                }
                Err(e) => return Err(e),
            };
            return Ok((id, sense, closing));
        }
        // A tolerant edge: no edge curve; each fin carries a trimmed
        // SP-curve (§4.3.8). Use the first fin whose curve reads; a fin on a
        // blend reads as the blend's cross-section when nothing else does.
        let mut at = positive.index;
        let mut first_error = None;
        let mut on_blend = None;
        for _ in 0..64 {
            let fin = self.node(at, "fin")?;
            if let Some(curve) = self.file.follow(fin, "curve") {
                // The fin's curve runs with the fin; the edge with the fin's sense.
                let fin_sense = fin.char("sense") != Some(b'-');
                let curve_sense = runs_forward(self.file, curve);
                self.anchor = self.edge_ends.map(|(start, end)| if fin_sense == curve_sense { (start, end) } else { (end, start) });
                match self.curve(curve) {
                    Ok((id, closing)) => return Ok((id, fin_sense == curve_sense, closing)),
                    Err(e) => {
                        if on_blend.is_none() {
                            on_blend = self.blend_under(curve);
                        }
                        first_error.get_or_insert(e);
                    }
                }
            }
            at = fin.ptr("other");
            if at == positive.index || at == 0 {
                break;
            }
        }
        if let Some((blend, uv)) = on_blend {
            if let Some(result) = self.blend_cross_section(blend, &uv).transpose() {
                return result;
            }
        }
        // Last: a fin's SP-curve on a blend, read through the blend.
        if !self.read_blend_sp {
            self.read_blend_sp = true;
            let again = self.edge_curve(edge, positive);
            self.read_blend_sp = false;
            if again.is_ok() {
                return again;
            }
        }
        Err(first_error.unwrap_or_else(|| "the edge has no curve and none of its fins has one".into()))
    }

    /// The blend and 2D curve of a fin's SP-curve that lies on a rolling-ball
    /// blend (through trimmed curves).
    fn blend_under(&self, curve: &Node) -> Option<(&'a Node, NurbsCurve)> {
        let mut node = self.file.node(curve.index)?;
        for _ in 0..8 {
            if node.type_name() != "TRIMMED_CURVE" {
                break;
            }
            node = self.file.follow(node, "basis_curve")?;
        }
        if node.type_name() != "SP_CURVE" {
            return None;
        }
        let blend = self.file.follow(node, "surface").filter(|s| s.type_name() == "BLENDED_EDGE")?;
        let uv = geometry::parameter_curve(self.file, self.file.follow(node, "b_curve")?).ok()?;
        Some((blend, uv))
    }

    /// An edge along a whole cross-section of a rolling-ball blend: its
    /// SP-curve runs at one spine parameter from v = 0 to v = 1, so the
    /// edge is the arc of the blend's radius about the spine point between
    /// its vertices, exactly; the spine point is the one standing the radius
    /// from both. `None` when the SP-curve is not such a section; an error
    /// when it is but the spine has no point the radius from both vertices.
    fn blend_cross_section(&mut self, blend: &Node, uv: &NurbsCurve) -> Result<Option<(usize, bool, Option<Vec3>)>, String> {
        let plain: Vec<(f64, f64)> = uv.control_points.iter().map(|c| (c.x / c.w, c.y / c.w)).collect();
        let (Some(first), Some(last)) = (plain.first(), plain.last()) else { return Ok(None) };
        let (u_low, u_high) = plain.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), p| (a.min(p.0), b.max(p.0)));
        let spans = ((first.1 - last.1).abs() - 1.0).abs() < 1e-6;
        if !(u_high - u_low <= 1e-6 * (1.0 + u_low.abs()) && spans) {
            return Ok(None);
        }
        let Some((start, end)) = self.edge_ends else { return Ok(None) };
        let id = blend.int("node_id").unwrap_or(blend.index as i64);
        let radius = blend.reals("range").iter().map(|r| r.abs()).fold(0.0f64, f64::max) * MM_PER_METRE;
        let spine_node = self.file.follow(blend, "spine").ok_or_else(|| format!("blend {id} has no spine"))?;
        let spine = self.spine_curve(spine_node).map_err(|e| format!("blend {id}: spine: {e}"))?;
        let miss = |c: Vec3| (start.sub(c).length() - radius).hypot(end.sub(c).length() - radius);
        let t = geometry::nearest_parameter(&spine, [true, true], miss)
            .ok_or_else(|| format!("blend {id}: no spine point for a cross-section edge"))?;
        let centre = geometry::extended_point(&spine, t)?;
        if miss(centre) > 1e-3 {
            return Err(format!(
                "blend {id}: a cross-section edge's vertices stand {:.3e} mm off the radius from the spine",
                miss(centre)
            ));
        }
        let x = start.sub(centre).normalized()?;
        let z = x.cross(end.sub(centre)).normalized()?;
        let circle = Curve::Circle { frame: Frame { origin: centre, z, x }, radius };
        Ok(Some((self.emit_curve(&circle)?, true, None)))
    }

    // -----------------------------------------------------------------
    // Curves
    // -----------------------------------------------------------------

    /// The STEP curve of an XT curve node and a point on it to close a ring
    /// edge at (its parameter start).
    fn curve(&mut self, node: &Node) -> Result<(usize, Option<Vec3>), String> {
        self.curve_at(node, 0, None)
    }

    /// `trim`: the basis parameters a trimmed curve directly over this one
    /// keeps, which an SP-curve is fitted over alone.
    fn curve_at(&mut self, node: &Node, depth: usize, trim: Option<[f64; 2]>) -> Result<(usize, Option<Vec3>), String> {
        if depth > 8 {
            return Err("trimmed curves nest more than 8 deep".into());
        }
        match node.type_name() {
            "LINE" | "CIRCLE" | "ELLIPSE" | "B_CURVE" => {
                let curve = geometry::curve(self.file, node)?;
                let closing = curve.natural_range().map(|r| curve.point(r[0])).transpose()?;
                Ok((self.emit_curve(&curve)?, closing))
            }
            "TRIMMED_CURVE" => {
                // The vertices bound the edge; the STEP lane trims by them.
                let basis = self.file.follow(node, "basis_curve").ok_or("a trimmed curve has no basis curve")?;
                let closing = node.vector("point_1").map(geometry::position);
                let trim = match (node.real("parm_1"), node.real("parm_2")) {
                    (Some(a), Some(b)) => Some([a.min(b), a.max(b)]),
                    _ => None,
                };
                let (id, basis_closing) = self.curve_at(basis, depth + 1, trim)?;
                Ok((id, closing.or(basis_closing)))
            }
            "INTERSECTION" => {
                let mut chart = self.intersection_points(node)?;
                // A closed chart is the whole loop, wherever it starts; an
                // edge on part of it is trimmed from its first vertex along
                // the curve, so the fitted curve must start there, or the
                // trim runs the long way round through the chart's seam.
                // An edge cut to the straight chord between its vertices
                // ([`cut_ring_to_short_edge`]), written as that chord.
                let mut chord_only = false;
                if let Some((begin, finish)) = self.anchor.take() {
                    // Vertices closer together than the chart's chords are
                    // long, on a loop that passes near them more than once
                    // (a lobe whose ends meet 0.45 mm apart, chords of
                    // 2-3 mm): the chart refined first, so each vertex is
                    // placed on the passage it is on, not the nearer chord.
                    let closed = chart.len() > 3 && chart[0].sub(chart[chart.len() - 1]).length() <= 1e-6;
                    let longest = chart.windows(2).map(|w| w[1].sub(w[0]).length()).fold(0.0f64, f64::max);
                    if closed && begin.sub(finish).length() < longest {
                        if let Some(crossing) = self.intersection_crossing(node) {
                            chart = ChartStations::new(Some(&crossing), &chart).refined(16);
                        }
                    }
                    chart = reseat_closed_chart(chart, begin, finish);
                    // A loop restarted in the edge's gap, on which the edge
                    // is too short for the STEP lane to trim to, is cut to
                    // the edge ([`cut_ring_to_short_edge`]). An edge that is
                    // the whole loop keeps it.
                    let ring = chart.len() > 3 && chart[0].sub(chart[chart.len() - 1]).length() <= 1e-6;
                    if ring && begin.sub(finish).length() > 1e-9 {
                        let crossing = self.intersection_crossing(node);
                        chart = cut_ring_to_short_edge(crossing.as_ref(), chart, begin, finish);
                        chord_only = chart.len() == 2;
                    }
                    // An open chart is cut to the stretch between the edge's
                    // vertices: a trimmed intersection's chart can run far
                    // past its edge (54 mm past a 7.8 mm one along a
                    // B-surface's crease, where the STEP lane cannot tell
                    // where to trim it).
                    chart = cut_open_chart(chart, begin, finish);
                }
                // Each end of the chart, cut to the edge's vertices, crowded
                // only within its own vertex's tolerance.
                let tolerance_at = |p: Option<&Vec3>| {
                    p.and_then(|p| self.edge_end_tolerances.iter().find(|(at, _)| at.sub(*p).length() <= 1e-9))
                        .map_or(RESOLUTION_MM, |(_, t)| t.max(RESOLUTION_MM))
                };
                let tolerances = [tolerance_at(chart.first()), tolerance_at(chart.last())];
                let chart = uncrowd_chart_ends(chart, tolerances);
                // Nothing is solved along a chord cut to its vertices: from a
                // chord tilted between unequally tolerant vertices the
                // solved points fall past its ends (cross review).
                let crossing = if chord_only { None } else { self.intersection_crossing(node) };
                // With nothing solved between chart points the densities all
                // give the chart, and the fit's check proves nothing: count
                // it as unverified rather than measured.
                let stations = ChartStations::new(crossing.as_ref(), &chart);
                let (fitted, fell_back) = stations.fit()
                    .map_err(|e| format!("intersection curve {}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                if fell_back || stations.refined(2).len() == chart.len() {
                    self.unverified_curves += 1;
                }
                self.fitted_curves += 1;
                Ok((self.emit_curve(&Curve::Nurbs(fitted))?, chart.first().copied()))
            }
            "SP_CURVE" if self.read_blend_sp && self.file.follow(node, "surface").is_some_and(|s| s.type_name() == "BLENDED_EDGE") => {
                // An SP-curve on a rolling-ball or cliff-edge blend: the blend
                // in its own parameters (§5.2.2.6), u along the spine and v
                // proportional to the angle across it. The reference puts v = 0
                // on the first support (the second when the spine runs
                // backwards); written files run it the other way round. Every
                // blend SP-curve that ends on its edge's vertices, in the
                // imported files checked alike (60 of 60,
                // elliptical spines forwards and intersection spines
                // backwards), starts at the foot on the support the reference
                // would put at v = 1: so v runs from that one here.
                let blend = self.file.follow(node, "surface").ok_or("an SP-curve has no surface")?;
                let blend_id = blend.int("node_id").unwrap_or(blend.index as i64);
                let (mut supports, radius) = self.blend_supports(blend, 0)?;
                let spine_node = self.file.follow(blend, "spine").ok_or_else(|| format!("blend {blend_id} has no spine"))?;
                // u is the file's spine parameter: only an exact spine has it.
                // A fitted intersection spine is parameterised by its fit, so
                // a point at the file's u would land elsewhere along the blend
                // (a section had landed 11.5 mm from its vertices).
                let mut basis = spine_node;
                for _ in 0..8 {
                    match self.file.follow(basis, "basis_curve") {
                        Some(b) if basis.type_name() == "TRIMMED_CURVE" => basis = b,
                        _ => break,
                    }
                }
                // An intersection spine's parameter is the file's own
                // (§5.2.1.5): the point there, found on the fitted spine.
                let along = match basis.type_name() {
                    "INTERSECTION" => Some(
                        self.intersection_parameters(basis)
                            .map_err(|e| format!("an SP-curve on blend {blend_id}: spine: {e}"))?,
                    ),
                    _ => None,
                };
                if spine_node.char("sense") == Some(b'-') {
                    supports.swap(0, 1);
                }
                let spine = self.spine_curve(spine_node).map_err(|e| format!("blend {blend_id}: spine: {e}"))?;
                let spine_u = |u: f64| -> Result<f64, String> {
                    let (Some(along), Curve::Nurbs(fitted)) = (&along, &spine) else { return Ok(u) };
                    parameter_on_extended(fitted, along.point(u)?)
                };
                let uv = geometry::parameter_curve(self.file, self.file.follow(node, "b_curve").ok_or("an SP-curve has no 2D curve")?)?;
                // The same rational quadratic section the face's surface is
                // fitted through, at the parameter where the angle from its
                // start is v of the whole (cross review: the nominal radius
                // put an edge off its carrier where the feet stand off it, and
                // a section collapsed where the supports meet had no frame).
                // On a circular rational quadratic of half-angle α the angle
                // φ from its start has tan((φ - α)/2) = tan(α/2)·(2w - 1).
                let point = |u: f64, v: f64| -> Result<Vec3, String> {
                    let u = spine_u(u)?;
                    let centre = ball_centre(&supports, radius, &spine, u)?;
                    Ok(section_at_angle(&blend_section(&supports, radius, &spine, u)?, centre, 1.0 - v))
                };
                let [t0, t1] = uv.domain()?;
                let open = uv.evaluate(t0)?.sub(uv.evaluate(t1)?).length() > 1e-9;
                let range = trim
                    .filter(|_| open)
                    .map(|[a, b]| [a.max(t0), b.min(t1)])
                    .filter(|[a, b]| b - a > 1e-9 * (t1 - t0))
                    .unwrap_or([t0, t1]);
                let trimmed = range != [t0, t1];
                let fitted = fit_sp_curve_by(&point, &uv, range, &[Vec::new(), Vec::new()], [0.0; 2])
                    .map_err(|e| format!("SP-curve {} on blend {blend_id}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                let fitted = match self.edge_ends {
                    Some(ends) if trimmed => snap_ends(fitted, ends),
                    _ => fitted,
                };
                self.fitted_curves += 1;
                let start = uv.evaluate(range[0])?;
                let closing = point(start.x, start.y).ok();
                Ok((self.emit_curve(&Curve::Nurbs(fitted))?, closing))
            }
            "SP_CURVE" => {
                let (surface, uv) = self.sp_curve_parts(node)?;
                // An open SP-curve is taken over the stretch its trimmed
                // curve keeps: past that it can run on along a crease of its
                // surface, or on into the next edge on the same curve, where
                // the STEP lane cannot tell where to trim it (a SolidWorks
                // part's B-surface face; two edges splitting one curve at a
                // 0.03 mm sliver). A closed one is taken whole, as its trim
                // may wrap through its seam.
                let [t0, t1] = uv.domain()?;
                let open = uv.evaluate(t0)?.sub(uv.evaluate(t1)?).length() > 1e-9;
                let range = trim
                    .filter(|_| open)
                    .map(|[a, b]| [a.max(t0), b.min(t1)])
                    .filter(|[a, b]| b - a > 1e-9 * (t1 - t0))
                    .unwrap_or([t0, t1]);
                let trimmed = range != [t0, t1];
                if let Some(exact) = exact_sp_curve(&surface, &uv) {
                    let closing = sp_curve_points(&surface, &uv, 1, uv.domain()?)?.first().copied();
                    // A B-curve on a plane is cut at the trim exactly.
                    let exact = match (exact, self.edge_ends) {
                        (Curve::Nurbs(nurbs), Some(ends)) if trimmed => {
                            Curve::Nurbs(snap_ends(crate::trimmed_curve::trimmed_curve(&nurbs, range[0], range[1])?, ends))
                        }
                        (exact, _) => exact,
                    };
                    return Ok((self.emit_curve(&exact)?, closing));
                }
                // An end a tolerant vertex stands off, in line with it, runs
                // on along the surface to it.
                let tolerance_of = |p: Vec3| self.edge_end_tolerances.iter().find(|(at, _)| at.sub(p).length() <= 1e-9).map_or(0.0, |(_, t)| *t);
                let extend = match self.anchor {
                    Some((from, to)) => sp_curve_extension(&|u, v| surface.point(u, v), &uv, range, [(from, tolerance_of(from)), (to, tolerance_of(to))]),
                    None => [0.0; 2],
                };
                let extended = extend.iter().any(|&d| d > 0.0);
                let fitted = fit_sp_curve(&surface, &uv, range, extend)
                    .map_err(|e| format!("SP-curve {}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                // A trimmed SP-curve ends at its trim parameters, which a
                // tolerant edge's vertices stand off by up to their
                // tolerance: move its ends onto them, so the edge runs
                // exactly vertex to vertex and nothing is left to trim.
                let fitted = match self.edge_ends {
                    Some(ends) if trimmed || extended => snap_ends(fitted, ends),
                    _ => fitted,
                };
                self.fitted_curves += 1;
                let closing = sp_curve_points(&surface, &uv, 1, range)?.first().copied();
                Ok((self.emit_curve(&Curve::Nurbs(fitted))?, closing))
            }
            other => Err(format!(
                "curve {} is a {other}, which is not read yet",
                node.int("node_id").unwrap_or(node.index as i64)
            )),
        }
    }

    /// The chart of an intersection curve (§5.2.1.5), with a terminator's
    /// exact position added at each end that has one.
    fn intersection_points(&self, node: &Node) -> Result<Vec<Vec3>, String> {
        let chart = self.file.follow(node, "chart").ok_or("an intersection curve has no chart")?;
        let mut points: Vec<Vec3> = chart
            .vectors("hvec")
            .into_iter()
            .map(|p| p.map(geometry::position).ok_or("an intersection chart has an unset point"))
            .collect::<Result<_, _>>()?;
        if points.len() < 2 {
            return Err("an intersection chart has fewer than two points".into());
        }
        let terminator = |field: &str| {
            self.file
                .follow(node, field)
                .filter(|limit| limit.char("type") == Some(b'T'))
                .and_then(|limit| limit.vectors("hvec").first().copied().flatten())
                .map(geometry::position)
        };
        if let Some(p) = terminator("start") {
            if p.sub(points[0]).length() > 0.0 {
                points.insert(0, p);
            }
        }
        if let Some(p) = terminator("end") {
            if p.sub(points[points.len() - 1]).length() > 0.0 {
                points.push(p);
            }
        }
        Ok(points)
    }

    /// An intersection curve in the file's own parameter (§5.2.1.5): its
    /// chart points' parameters, from the chart's base parameter and scale,
    /// each chord's factor carried across a chart point by the ratio of the
    /// angles the curve's tangent there makes with its two chords, so the
    /// parameter is C1.
    fn intersection_parameters(&self, node: &Node) -> Result<ChartParameters, String> {
        let chart = self.file.follow(node, "chart").ok_or("an intersection curve has no chart")?;
        let points: Vec<Vec3> = chart
            .vectors("hvec")
            .into_iter()
            .map(|p| p.map(geometry::position).ok_or("an intersection chart has an unset point"))
            .collect::<Result<_, _>>()?;
        let [first, second] = self.intersection_surfaces(node).ok_or("the intersection's surfaces are not both read")?;
        let (Some(mut t), Some(mut factor)) = (chart.real("base_parameter"), chart.real("base_scale")) else {
            return Err("an intersection chart has no parameterisation".into());
        };
        if points.len() < 2 {
            return Err("an intersection chart has fewer than two points".into());
        }
        let mut parameters = vec![t];
        for i in 1..points.len() {
            let previous = points[i].sub(points[i - 1]);
            // The file's parameter runs in its own unit, metres.
            t += previous.length() / MM_PER_METRE * factor;
            parameters.push(t);
            if i + 1 < points.len() {
                let normal = |s: &Surface| s.distance(points[i]).map(|(_, n)| n);
                let tangent = normal(&first)
                    .zip(normal(&second))
                    .and_then(|(a, b)| a.cross(b).normalized().ok())
                    .ok_or("the intersection has no tangent at a chart point")?;
                let next = points[i + 1].sub(points[i]);
                let (cos_b, cos_a) = (tangent.dot(previous) / previous.length(), tangent.dot(next) / next.length());
                if !(cos_a.abs() > 1e-9) {
                    return Err("a chart chord is normal to the curve's tangent".into());
                }
                factor *= cos_b / cos_a;
            }
        }
        Ok(ChartParameters { points, parameters, crossing: Crossing::Surfaces(first, second) })
    }

    /// The two surfaces of an intersection curve, when the reader reads both.
    fn intersection_surfaces(&self, node: &Node) -> Option<[Surface; 2]> {
        let surfaces: Vec<Surface> = node
            .ints("surface")
            .into_iter()
            .filter_map(|i| u32::try_from(i).ok())
            .filter_map(|i| self.file.node(i))
            .filter_map(|s| geometry::surface(self.file, s, 0).ok())
            .collect();
        surfaces.try_into().ok()
    }

    /// What solves an intersection curve's points between its chart points:
    /// its two surfaces where the reader reads both; for a surface meeting
    /// the boundary of a rolling-ball blend it supports (a BLEND_BOUND,
    /// §5.2.2.6), that blend's line of contact on it, each point the
    /// support's foot from the spine point whose cross-section passes there,
    /// as the blend's own surface is built ([`blend_section`]).
    fn intersection_crossing(&mut self, node: &Node) -> Option<Crossing> {
        if let Some([a, b]) = self.intersection_surfaces(node) {
            return Some(Crossing::Surfaces(a, b));
        }
        let file = self.file;
        let nodes: Vec<&Node> = node.ints("surface").into_iter().filter_map(|i| u32::try_from(i).ok()).filter_map(|i| file.node(i)).collect();
        let [first, second]: [&Node; 2] = nodes.try_into().ok()?;
        let (bound, other) = match (first.type_name(), second.type_name()) {
            ("BLEND_BOUND", _) => (first, second),
            (_, "BLEND_BOUND") => (second, first),
            _ => return None,
        };
        let blend = file.follow(bound, "blend").filter(|b| b.type_name() == "BLENDED_EDGE" && b.char("blend_type") == Some(b'R'))?;
        // The curve's other surface is the support the boundary lies on: one
        // of the blend's two, and only one (a SolidWorks part's boundary 0
        // lay on the blend's second support, so its index is not relied on).
        let supports = blend.ints("surface");
        if supports.len() != 2 || supports.iter().filter(|&&s| s == i64::from(other.index)).count() != 1 {
            return None;
        }
        let support = geometry::surface(file, other, 0).ok()?;
        let radius = blend.reals("range").iter().map(|r| r.abs()).fold(0.0f64, f64::max) * MM_PER_METRE;
        if !(radius > 0.0 && radius.is_finite()) {
            return None;
        }
        let spine = self.spine_curve(file.follow(blend, "spine")?).ok()?;
        Some(Crossing::Contact { spine, support, radius })
    }

    /// An SP-curve's surface and 2D curve (§5.2.1.8).
    fn sp_curve_parts(&self, node: &Node) -> Result<(Surface, NurbsCurve), String> {
        let surface_node = self.file.follow(node, "surface").ok_or("an SP-curve has no surface")?;
        let surface = geometry::surface(self.file, surface_node, 0)?;
        let b_curve = self.file.follow(node, "b_curve").ok_or("an SP-curve has no 2D curve")?;
        Ok((surface, geometry::parameter_curve(self.file, b_curve)?))
    }

    fn emit_curve(&mut self, curve: &Curve) -> Result<usize, String> {
        Ok(match curve {
            Curve::Line { point, direction } => {
                let p = self.point_id(*point);
                let d = self.direction_id(*direction);
                let v = self.add(format!("VECTOR('',#{d},1.)"));
                self.add(format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle { frame, radius } => {
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                self.add(format!("CIRCLE('',#{a},{})", real(*radius)))
            }
            Curve::Ellipse { frame, major, minor } => {
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                self.add(format!("ELLIPSE('',#{a},{},{})", real(*major), real(*minor)))
            }
            Curve::Nurbs(curve) => self.emit_nurbs_curve(curve),
        })
    }

    fn emit_nurbs_curve(&mut self, curve: &NurbsCurve) -> usize {
        let points: Vec<String> = curve
            .control_points
            .iter()
            .map(|c| format!("#{}", self.point_id(Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w))))
            .collect();
        let (mults, values) = knot_runs(&curve.knots);
        let rational = curve.control_points.iter().any(|c| (c.w - 1.0).abs() > 1e-15);
        let degree = curve.degree;
        if rational {
            let weights: Vec<String> = curve.control_points.iter().map(|c| real(c.w)).collect();
            self.add(format!(
                "(BOUNDED_CURVE()B_SPLINE_CURVE({degree},({}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS(({mults}),({values}),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({}))REPRESENTATION_ITEM(''))",
                points.join(","),
                weights.join(",")
            ))
        } else {
            self.add(format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{degree},({}),.UNSPECIFIED.,.F.,.F.,({mults}),({values}),.UNSPECIFIED.)",
                points.join(",")
            ))
        }
    }

    // -----------------------------------------------------------------
    // Surfaces
    // -----------------------------------------------------------------

    /// Index into `emitted` of the surface node's STEP surface.
    fn surface(&mut self, node: &Node) -> Result<usize, String> {
        if let Some(&i) = self.surfaces.get(&node.index) {
            return Ok(i);
        }
        let surface = geometry::surface(self.file, node, 0)?;
        let revolved = match self.spun_skew_line(node, &surface)? {
            Some(hyperboloid) => Some(hyperboloid),
            None => spun_conic_to_axis(node, &surface)?,
        };
        let (id, flipped) = match revolved {
            // Its exact rational B-surface (u along the profile, v round the
            // axis right-handedly, as XT's: the same natural normal).
            Some(revolved) => (self.emit_nurbs_surface(&revolved), false),
            None => self.emit_surface(&surface)?,
        };
        self.emitted.push(EmittedSurface { id, flipped });
        let i = self.emitted.len() - 1;
        self.surfaces.insert(node.index, i);
        Ok(i)
    }

    /// A line spun about an axis skew to it (a hyperboloid of one sheet) as
    /// the exact rational B-surface of revolution of the stretch of the line
    /// its faces use. The SPUN_SURF names no stretch for a line, so it is read
    /// from the faces: every point of the surface stands at one axial
    /// coordinate, and the line's axial coordinate is linear in its
    /// parameter, so the boundary points of every face on the surface bound
    /// the stretch (widened by a hundredth each way). The line runs as the
    /// profile does: the XT natural normal.
    fn spun_skew_line(&self, node: &Node, surface: &Surface) -> Result<Option<NurbsSurface>, String> {
        let Surface::Spun { profile: profile @ Curve::Line { point, direction }, axis_point, axis } = surface else { return Ok(None) };
        if !matches!(spun_analytic(profile, *axis_point, *axis), Ok(None)) {
            return Ok(None);
        }
        let axis = axis.normalized()?;
        let rate = direction.dot(axis) * MM_PER_METRE;
        if rate.abs() < 1e-9 {
            return Err("a line spun about an axis square to it and skew to it is not read yet".into());
        }
        let at = |p: Vec3| p.sub(*point).dot(axis) / rate;
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut take = |t: f64| {
            if t.is_finite() {
                (lo, hi) = (lo.min(t), hi.max(t));
            }
        };
        // A bound that holds whatever the boundary does between samples
        // (cross review): vertices; each edge's curve by its control points
        // (a B-curve lies in their hull), a conic by its exact axial reach, an
        // intersection by its chart; and each fin's SP-curve on this surface
        // by its 2D control points, whose u is this line's parameter.
        let file = self.file;
        for face in file.nodes().iter().filter(|n| n.type_name() == "FACE" && n.ptr("surface") == node.index) {
            for lp in chain(file, face.ptr("loop"), "next") {
                for fin in self.fins(lp) {
                    if let Ok(p) = self.vertex_location(fin.ptr("vertex")) {
                        take(at(p));
                    }
                    let mut curves: Vec<&Node> = Vec::new();
                    if let Some(edge) = file.follow(fin, "edge") {
                        curves.extend(file.follow(edge, "curve"));
                    }
                    curves.extend(file.follow(fin, "curve"));
                    for mut c in curves {
                        for _ in 0..8 {
                            match file.follow(c, "basis_curve") {
                                Some(basis) if c.type_name() == "TRIMMED_CURVE" => c = basis,
                                _ => break,
                            }
                        }
                        match c.type_name() {
                            "SP_CURVE" if c.ptr("surface") == node.index => {
                                if let Ok(uv) = file.follow(c, "b_curve").ok_or(()).and_then(|b| geometry::parameter_curve(file, b).map_err(|_| ())) {
                                    for q in &uv.control_points {
                                        take(q.x / q.w);
                                    }
                                }
                            }
                            "INTERSECTION" => {
                                for p in self.intersection_points(c).unwrap_or_default() {
                                    take(at(p));
                                }
                            }
                            _ => match geometry::curve(file, c) {
                                Ok(Curve::Nurbs(n)) => {
                                    for q in &n.control_points {
                                        take(at(Vec3::new(q.x / q.w, q.y / q.w, q.z / q.w)));
                                    }
                                }
                                Ok(Curve::Circle { frame, radius }) => {
                                    let reach = radius * frame.z.cross(axis).length();
                                    take(at(frame.origin.add(axis.scale(reach))));
                                    take(at(frame.origin.sub(axis.scale(reach))));
                                }
                                Ok(Curve::Ellipse { frame, major, minor }) => {
                                    let reach = (major * frame.x.dot(axis)).hypot(minor * frame.y().dot(axis));
                                    take(at(frame.origin.add(axis.scale(reach))));
                                    take(at(frame.origin.sub(axis.scale(reach))));
                                }
                                // A line edge runs between its vertices.
                                _ => {}
                            },
                        }
                    }
                }
            }
        }
        if !(hi > lo) {
            return Err("a line spun about an axis skew to it bounds no face".into());
        }
        let pad = 0.05 * (hi - lo);
        let (lo, hi) = (lo - pad, hi + pad);
        let ends = [profile.point(lo)?, profile.point(hi)?];
        let net = ends.iter().map(|p| crate::Vec4 { x: p.x, y: p.y, z: p.z, w: 1.0 }).collect();
        let segment = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], net)?;
        Ok(Some(revolve(&segment, *axis_point, axis)?))
    }

    fn emit_surface(&mut self, surface: &Surface) -> Result<(usize, bool), String> {
        Ok(match surface {
            Surface::Plane(f) => {
                let a = self.placement_id(f.origin, f.z, f.x);
                (self.add(format!("PLANE('',#{a})")), false)
            }
            Surface::Cylinder { frame, radius } => {
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                (self.add(format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*radius))), false)
            }
            Surface::Cone { frame, radius, sin, cos } => {
                // The radius grows along the axis, as STEP's does, and both
                // normals point away from the axis.
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                let angle = sin.atan2(*cos);
                (self.add(format!("CONICAL_SURFACE('',#{a},{},{})", real(*radius), real(angle))), false)
            }
            Surface::Sphere { frame, radius } => {
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                (self.add(format!("SPHERICAL_SURFACE('',#{a},{})", real(*radius))), false)
            }
            Surface::Torus { frame, major, minor } if *major < 0.0 => {
                // A lemon has no STEP surface: write the exact rational
                // B-surface of its spun arc, oriented as the torus.
                let Surface::Spun { profile: Curve::Nurbs(arc), axis_point, axis } = geometry::lemon(*frame, *major, *minor)? else {
                    return Err("a lemon torus's arc is not a B-curve".into());
                };
                (self.emit_nurbs_surface(&revolve(&arc, axis_point, axis)?), false)
            }
            Surface::Torus { frame, major, minor } => {
                let a = self.placement_id(frame.origin, frame.z, frame.x);
                (
                    self.add(format!(
                        "TOROIDAL_SURFACE('',#{a},{},{})",
                        real(*major),
                        real(*minor)
                    )),
                    false,
                )
            }
            Surface::Nurbs(nurbs) => (self.emit_nurbs_surface(nurbs), false),
            Surface::Offset { base, base_sense, distance } => match offset_analytic(base, *base_sense, *distance) {
                // The offset of an analytic is the same kind of surface with
                // the same natural normal.
                Ok(offset) => self.emit_surface(&offset)?,
                Err(_) if !base.is_analytic() => {
                    // STEP offsets along the basis entity's own normal, which
                    // is the XT natural normal unless the basis was written
                    // flipped; the offset keeps its basis's normal.
                    let (basis, flipped) = self.emit_surface(base)?;
                    let along = if *base_sense != flipped { *distance } else { -*distance };
                    (self.add(format!("OFFSET_SURFACE('',#{basis},{},.F.)", real(along))), flipped)
                }
                Err(error) => return Err(error),
            },
            Surface::Spun { profile, axis_point, axis } => {
                // A line or circle spun about an axis in its plane is a
                // plane, cylinder, cone, sphere or torus: say it as one, its
                // normal turned to the spun surface's natural normal.
                if let Some((analytic, sample)) = spun_analytic(profile, *axis_point, *axis).map_err(|e| format!("spun profile: {e}"))? {
                    let (id, flipped) = self.emit_surface(&analytic)?;
                    let spun_normal = surface.natural_normal(sample, 0.0).map_err(|e| format!("spun normal at {sample}: {e}"))?;
                    let point = surface.point(sample, 0.0)?;
                    let outward = analytic_outward(&analytic, point).map_err(|e| format!("analytic normal at {point:?} of {analytic:?}: {e}"))?;
                    let agrees = outward.dot(spun_normal) > 0.0;
                    return Ok((id, flipped == agrees));
                }
                // The STEP lane cannot build a revolution whose profile
                // reaches its axis (its poles are not analytic ones).
                if profile_meets_axis(profile, *axis_point, *axis) {
                    // Said instead as the exact rational B-surface of
                    // revolution (u along the profile, v round the axis
                    // right-handedly, as XT's: the same natural normal),
                    // whose row on the axis the STEP lane reads as a pole.
                    let Curve::Nurbs(nurbs) = profile else {
                        return Err("a spun surface whose profile meets its axis is not read yet".into());
                    };
                    return Ok((self.emit_nurbs_surface(&revolve(nurbs, *axis_point, *axis)?), false));
                }
                if matches!(profile, Curve::Line { .. }) {
                    return Err("a line spun about an axis skew to it (a hyperboloid) is not read yet".into());
                }
                // STEP's revolution takes the angle as u and the profile's
                // parameter as v, the reverse of XT's (§5.2.2.11), so its
                // normal σu × σv is the XT natural normal reversed.
                let curve = self.emit_curve(profile)?;
                let point = self.point_id(*axis_point);
                let direction = self.direction_id(*axis);
                let placement = self.add(format!("AXIS1_PLACEMENT('',#{point},#{direction})"));
                (self.add(format!("SURFACE_OF_REVOLUTION('',#{curve},#{placement})")), true)
            }
            Surface::Swept { section: Curve::Line { point, direction: along }, direction } => {
                // A line swept is a plane (the STEP lane reads no LINE as a
                // profile); its natural normal is C'(u) × D.
                let normal = along.cross(*direction).normalized().map_err(|_| "a line swept along itself")?;
                self.emit_surface(&Surface::Plane(Frame { origin: *point, z: normal, x: *along }))?
            }
            Surface::Swept { section, direction } => {
                // Both are C(u) + v·D: the same parameters, the same normal.
                let curve = self.emit_curve(section)?;
                let direction = self.direction_id(*direction);
                let vector = self.add(format!("VECTOR('',#{direction},1.)"));
                (self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{curve},#{vector})")), false)
            }
        })
    }

    fn emit_nurbs_surface(&mut self, surface: &NurbsSurface) -> usize {
        let mut rows = Vec::with_capacity(surface.control_points.len());
        let mut weight_rows = Vec::with_capacity(surface.control_points.len());
        let mut rational = false;
        for row in &surface.control_points {
            let mut ids = Vec::with_capacity(row.len());
            let mut weights = Vec::with_capacity(row.len());
            for c in row {
                ids.push(format!("#{}", self.point_id(Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w))));
                weights.push(real(c.w));
                rational |= (c.w - 1.0).abs() > 1e-15;
            }
            rows.push(format!("({})", ids.join(",")));
            weight_rows.push(format!("({})", weights.join(",")));
        }
        let (mults_u, values_u) = knot_runs(&surface.knots_u);
        let (mults_v, values_v) = knot_runs(&surface.knots_v);
        let (du, dv) = (surface.degree_u, surface.degree_v);
        if rational {
            self.add(format!(
                "(BOUNDED_SURFACE()B_SPLINE_SURFACE({du},{dv},({}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS(({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({}))REPRESENTATION_ITEM('')SURFACE())",
                rows.join(","),
                weight_rows.join(",")
            ))
        } else {
            self.add(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{du},{dv},({}),.UNSPECIFIED.,.F.,.F.,.F.,({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)",
                rows.join(",")
            ))
        }
    }
}

/// A spun line or circle as the analytic surface it sweeps out, with a
/// profile parameter at which the spun surface is regular (off the axis).
/// `None` for other profiles, and for a line skew to the axis (a
/// hyperboloid); a circle crossing its axis (an apple or lemon) is refused.
fn spun_analytic(profile: &Curve, axis_point: Vec3, axis: Vec3) -> Result<Option<(Surface, f64)>, String> {
    let foot = |p: Vec3| axis_point.add(axis.scale(p.sub(axis_point).dot(axis)));
    let tiny = 1e-9;
    Ok(match profile {
        Curve::Line { point, direction } => {
            if point.sub(axis_point).dot(direction.cross(axis)).abs() > tiny {
                return Ok(None);
            }
            // A point of the line off the axis, and its parameter (metres).
            let t = if point.sub(foot(*point)).length() > tiny { 0.0 } else { 1.0 };
            let p = point.add(direction.scale(t * MM_PER_METRE));
            let centre = foot(p);
            let radial = p.sub(centre);
            let radius = radial.length();
            if radius <= tiny {
                return Ok(None);
            }
            let x = radial.scale(1.0 / radius);
            let along = direction.dot(axis);
            let across = direction.dot(x);
            let surface = if across.abs() <= tiny {
                Surface::Cylinder { frame: Frame { origin: centre, z: axis, x }, radius }
            } else if along.abs() <= tiny {
                Surface::Plane(Frame { origin: centre, z: axis, x })
            } else {
                // The radius grows along z: take z the way the line moves
                // away from the axis.
                let z = if along * across > 0.0 { axis } else { axis.scale(-1.0) };
                let (sin, cos) = (across.abs(), along.abs());
                Surface::Cone { frame: Frame { origin: centre, z, x }, radius, sin, cos }
            };
            Some((surface, t))
        }
        Curve::Circle { frame, radius } => {
            // A torus or sphere only when the axis lies in the circle's plane.
            if frame.z.dot(axis).abs() > tiny || axis_point.sub(frame.origin).dot(frame.z).abs() > tiny * (1.0 + radius) {
                return Ok(None);
            }
            let centre = foot(frame.origin);
            let offset = frame.origin.sub(centre);
            let major = offset.length();
            // A parameter away from the axis: the point furthest from it.
            // (The circle's plane holds the axis, so z × axis lies in it.)
            let outward = if major > tiny { offset.scale(1.0 / major) } else { frame.z.cross(axis).normalized()? };
            let outward = outward.sub(frame.z.scale(outward.dot(frame.z))).normalized()?;
            let sample = outward.dot(frame.y()).atan2(outward.dot(frame.x));
            if major <= tiny * (1.0 + radius) {
                let x = axis.perpendicular()?;
                Some((Surface::Sphere { frame: Frame { origin: centre, z: axis, x }, radius: *radius }, sample))
            } else if major > *radius {
                Some((Surface::Torus { frame: Frame { origin: centre, z: axis, x: outward }, major, minor: *radius }, sample))
            } else {
                return Err("a spun circle that crosses its axis (a lemon or apple torus) has no STEP surface".into());
            }
        }
        _ => None,
    })
}

/// The natural normal of a plane, cylinder, cone, sphere or torus at a point
/// on it: each points away from the axis or centre (the plane's along its
/// normal).
fn analytic_outward(surface: &Surface, p: Vec3) -> Result<Vec3, String> {
    let from_axis = |frame: &Frame| {
        let d = p.sub(frame.origin);
        d.sub(frame.z.scale(d.dot(frame.z)))
    };
    match surface {
        Surface::Plane(f) => Ok(f.z),
        Surface::Cylinder { frame, .. } => from_axis(frame).normalized(),
        Surface::Cone { frame, sin, cos, .. } => {
            // Outward and back along the axis by the half angle.
            let radial = from_axis(frame).normalized()?;
            Ok(radial.scale(*cos).sub(frame.z.scale(*sin)))
        }
        Surface::Sphere { frame, .. } => p.sub(frame.origin).normalized(),
        Surface::Torus { frame, major, .. } => {
            let radial = from_axis(frame).normalized()?;
            p.sub(frame.origin.add(radial.scale(*major))).normalized()
        }
        _ => Err("not an analytic surface".into()),
    }
}

/// The frame of a lemon torus (`major` < 0) turned about its axis so the
/// revolve written for a face with vertices at `points` starts in the middle
/// of the widest stretch between their turns, or `None` to keep the shared
/// surface: when no vertex is beside its seam ([`SEAM_ROW_NEAR`]: near the
/// arc at the frame's x direction but not on it), or one would be beside the
/// turned seam too.
pub(super) fn turned_lemon_frame(frame: Frame, major: f64, minor: f64, points: &[Vec3]) -> Option<Frame> {
    let [on, near] = SEAM_ROW_NEAR;
    let beside = |frame: Frame| -> Option<bool> {
        let Ok(Surface::Spun { profile: Curve::Nurbs(arc), .. }) = geometry::lemon(frame, major, minor) else { return None };
        Some(points.iter().any(|&point| {
            crate::project_point_to_curve(&arc, point)
                .ok()
                .and_then(|foot| arc.evaluate(foot.u).ok())
                .is_some_and(|p| (on..near).contains(&p.sub(point).length()))
        }))
    };
    if !beside(frame)? {
        return None;
    }
    // Only a vertex on the axis (within the on-seam distance) is on every
    // section; one nearer the axis than `near` still has its turn (a thin
    // lemon's equator stands under 0.1 mm off it, cross review).
    let turn = widest_turn(frame, points, on)?;
    let turned = Frame { origin: frame.origin, z: frame.z, x: frame.x.scale(turn.cos()).add(frame.y().scale(turn.sin())) };
    (!beside(turned)?).then_some(turned)
}

/// The turn about `frame`'s axis, from its x direction right-handedly, in
/// the middle of the widest stretch between `points`' turns. A point within
/// `near` of the axis (a pole) has no turn and is on every section.
pub(super) fn widest_turn(frame: Frame, points: &[Vec3], near: f64) -> Option<f64> {
    use std::f64::consts::TAU;
    let y = frame.y();
    let mut turns: Vec<f64> = points
        .iter()
        .filter_map(|&point| {
            let d = point.sub(frame.origin);
            let (along_x, along_y) = (d.dot(frame.x), d.dot(y));
            (along_x.hypot(along_y) > near).then(|| along_y.atan2(along_x).rem_euclid(TAU))
        })
        .collect();
    turns.sort_by(f64::total_cmp);
    let first = *turns.first()?;
    turns.push(first + TAU);
    let (gap, from) = turns.windows(2).map(|w| (w[1] - w[0], w[0])).max_by(|a, b| a.0.total_cmp(&b.0))?;
    (gap > 0.0).then_some((from + 0.5 * gap).rem_euclid(TAU))
}

/// The rational B-surface of `profile` turned once about the axis through
/// `axis_point` along unit `axis`, right-handedly: u is the profile's
/// parameter and v runs once round, a quadratic circle of nine vertices.
pub(super) fn revolve(profile: &NurbsCurve, axis_point: Vec3, axis: Vec3) -> Result<NurbsSurface, String> {
    let axis = axis.normalized()?;
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let net = profile
        .control_points
        .iter()
        .map(|c| {
            let p = Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w);
            let foot = axis_point.add(axis.scale(p.sub(axis_point).dot(axis)));
            let radial = p.sub(foot);
            let turned = axis.cross(radial);
            (0..9)
                .map(|j| {
                    let angle = std::f64::consts::FRAC_PI_4 * j as f64;
                    let (stretch, weight) = if j % 2 == 0 { (1.0, 1.0) } else { (std::f64::consts::SQRT_2, half) };
                    let q = foot.add(radial.scale(stretch * angle.cos())).add(turned.scale(stretch * angle.sin()));
                    let w = c.w * weight;
                    crate::Vec4 { x: q.x * w, y: q.y * w, z: q.z * w, w }
                })
                .collect()
        })
        .collect();
    let around = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    NurbsSurface::new(profile.degree, 2, profile.knots.clone(), around, net)
}

/// A SPUN_SURF whose profile is a circle or ellipse meeting its axis, and
/// no plane, sphere or torus (a spheroid's cap: an ellipse spun about one of
/// its axes), as the exact rational B-surface of revolution of the stretch
/// of the profile the surface spins, `[start_param, end_param]` (§5.2.2.11):
/// the other stretch is the same points with the opposite normal. The arc is
/// rational quadratics of under π/2 each, an ellipse's being the affine image
/// of a circle's. `None` for every other surface.
pub(super) fn spun_conic_to_axis(node: &Node, surface: &Surface) -> Result<Option<NurbsSurface>, String> {
    let Surface::Spun { profile, axis_point, axis } = surface else { return Ok(None) };
    let (frame, major, minor) = match profile {
        Curve::Circle { frame, radius } => (frame, *radius, *radius),
        Curve::Ellipse { frame, major, minor } => (frame, *major, *minor),
        _ => return Ok(None),
    };
    if !conic_reaches_axis(frame, major, minor, *axis_point, *axis) || matches!(spun_analytic(profile, *axis_point, *axis), Ok(Some(_))) {
        return Ok(None);
    }
    let (Some(a), Some(b)) = (node.real("start_param"), node.real("end_param")) else { return Ok(None) };
    spun_conic_surface(frame, major, minor, [a, b], *axis_point, *axis).map(Some)
}

/// Whether a circle or ellipse (centre and frame `frame`, semi-axes `major`
/// along x and `minor` along y) in a plane holding the axis reaches the axis,
/// decided exactly, not by samples (cross review: a crossing between samples
/// was missed): with n the unit normal to the axis in that plane, the conic's
/// offset along n is d0 + major·(n·x)·cos t + minor·(n·y)·sin t, which
/// reaches 0 when |d0| is at most the amplitude.
pub(super) fn conic_reaches_axis(frame: &Frame, major: f64, minor: f64, axis_point: Vec3, axis: Vec3) -> bool {
    let Ok(axis) = axis.normalized() else { return false };
    let scale = 1.0 + major.abs().max(minor.abs());
    if frame.z.dot(axis).abs() > 1e-9 || axis_point.sub(frame.origin).dot(frame.z).abs() > 1e-9 * scale {
        return false;
    }
    let Ok(n) = frame.z.cross(axis).normalized() else { return false };
    let d0 = frame.origin.sub(axis_point).dot(n);
    let amplitude = (major * n.dot(frame.x)).hypot(minor * n.dot(frame.y()));
    d0.abs() <= amplitude + 1e-9 * scale
}

/// The rational B-surface of revolution of the conic arc `[a, b]` (centre
/// and frame of `frame`, semi-axes `major` along x and `minor` along y)
/// about the axis through `axis_point` along `axis`; see
/// [`spun_conic_to_axis`].
pub(super) fn spun_conic_surface(frame: &Frame, major: f64, minor: f64, [a, b]: [f64; 2], axis_point: Vec3, axis: Vec3) -> Result<NurbsSurface, String> {
    if !(b > a && b - a <= std::f64::consts::TAU + 1e-9) {
        return Err(format!("a spun profile's range [{a}, {b}] is not a stretch of one turn"));
    }
    let pieces = ((b - a) / std::f64::consts::FRAC_PI_2).ceil().max(1.0) as usize;
    let on = |t: f64| frame.origin.add(frame.x.scale(major * t.cos())).add(frame.y().scale(minor * t.sin()));
    let mut net = Vec::new();
    let mut knots = vec![0.0; 3];
    for k in 0..pieces {
        let (t0, t1) = (a + (b - a) * k as f64 / pieces as f64, a + (b - a) * (k + 1) as f64 / pieces as f64);
        let (half, middle) = ((t1 - t0) / 2.0, (t0 + t1) / 2.0);
        let w = half.cos();
        let apex = frame.origin.add(frame.x.scale(major * middle.cos() / w)).add(frame.y().scale(minor * middle.sin() / w));
        if k == 0 {
            let p = on(t0);
            net.push(crate::Vec4 { x: p.x, y: p.y, z: p.z, w: 1.0 });
        }
        let p = on(t1);
        net.push(crate::Vec4 { x: apex.x * w, y: apex.y * w, z: apex.z * w, w });
        net.push(crate::Vec4 { x: p.x, y: p.y, z: p.z, w: 1.0 });
        knots.extend([(k + 1) as f64, (k + 1) as f64]);
    }
    knots.push(pieces as f64);
    let arc = NurbsCurve::new(2, knots, net)?;
    revolve(&arc, axis_point, axis)
}

/// Whether a spun profile comes within a micron of its axis over its range.
fn profile_meets_axis(profile: &Curve, axis_point: Vec3, axis: Vec3) -> bool {
    let Some([a, b]) = profile.natural_range() else { return false };
    (0..=256).filter_map(|k| profile.point(a + (b - a) * k as f64 / 256.0).ok()).any(|p| {
        let d = p.sub(axis_point);
        d.sub(axis.scale(d.dot(axis))).length() < 1e-3
    })
}

/// The offset of an analytic surface by `distance` along its natural normal
/// (negated when the base's sense is negative), as the same kind of surface.
fn offset_analytic(base: &Surface, base_sense: bool, distance: f64) -> Result<Surface, String> {
    let d = if base_sense { distance } else { -distance };
    let positive = |r: f64, what: &str| {
        if r > 0.0 {
            Ok(r)
        } else {
            Err(format!("an offset {what} collapses (radius {r})"))
        }
    };
    Ok(match base {
        Surface::Plane(f) => Surface::Plane(Frame { origin: f.origin.add(f.z.scale(d)), ..*f }),
        // Natural normals of these point away from the axis or centre.
        Surface::Cylinder { frame, radius } => Surface::Cylinder { frame: *frame, radius: positive(radius + d, "cylinder")? },
        Surface::Sphere { frame, radius } => Surface::Sphere { frame: *frame, radius: positive(radius + d, "sphere")? },
        Surface::Torus { frame, major, minor } => {
            Surface::Torus { frame: *frame, major: *major, minor: positive(minor + d, "torus")? }
        }
        // The cone's natural normal points outwards, at the half angle to
        // the radial: a point moves by d (cos a · radial − sin a · A), so the
        // radius at a fixed axial station grows by d / cos a.
        Surface::Cone { frame, radius, sin, cos } => {
            let r = radius + d / cos;
            if r < 0.0 {
                return Err(format!("an offset cone passes its apex (radius {r})"));
            }
            Surface::Cone { frame: *frame, radius: r, sin: *sin, cos: *cos }
        }
        _ => return Err("an offset of a non-analytic surface is not read yet".into()),
    })
}

/// An open chart cut to the stretch between `begin` and `finish` (each
/// placed along it where it comes nearest, within a millimetre), in the
/// chart's own direction, with those two points as its ends. A closed chart,
/// or one either point stands off, is left as it is.
pub(super) fn cut_open_chart(chart: Vec<Vec3>, begin: Vec3, finish: Vec3) -> Vec<Vec3> {
    let n = chart.len();
    if n < 2 || chart[0].sub(chart[n - 1]).length() <= 1e-6 {
        return chart;
    }
    cut_chart(chart, begin, finish)
}

/// A closed chart restarted in an edge's gap ([`reseat_closed_chart`]) cut
/// to the edge: the seam lies in the gap, so the edge runs between its two
/// copies, from `begin` to `finish`, and is cut there as an open chart is.
/// Returned as it is when the cut would not run from `begin` to `finish`
/// in the chart's direction.
pub(super) fn cut_ring_to_edge(chart: Vec<Vec3>, begin: Vec3, finish: Vec3) -> Vec<Vec3> {
    let cut = cut_chart(chart.clone(), begin, finish);
    let runs = cut.len() >= 2 && cut[0].sub(begin).length() <= 1e-9 && cut[cut.len() - 1].sub(finish).length() <= 1e-9;
    if runs { cut } else { chart }
}

/// The share of a closed chart's length below which an edge on it may be
/// cut to its own stretch. The STEP lane trims a whole closed curve to an
/// edge by its vertices' parameters, and keeps the whole curve when they are
/// closer than 1e-6 of its domain, which the chord-length fit makes 1e-6 of
/// its length: two vertices 1.8e-4 mm apart on a 600 mm loop (a keycap's
/// dish meeting a plane at a glyph's corner) were taken as the whole loop.
/// A hundred times that bar leaves every longer edge as the whole loop the
/// STEP lane trims well, with its chart's own density (a lobe edge 12 mm
/// long between vertices 0.45 mm apart, cut, imported three times slower).
const RING_CUT_SHARE: f64 = 1e-4;

/// An edge at most [`RING_CUT_SHARE`] of its loop, whose straight chord
/// stays within [`FIT_TOLERANCE`] of the curve, written as that chord: its
/// two vertices alone, fitted with nothing solved between them. The chord's
/// sag is bounded by L²/8R, R the least radius through three consecutive
/// chart points about the stretch; it stands off the curve by that and by
/// its vertices' own distance from it, within their tolerance. Any
/// other edge keeps the whole loop. Cut to tolerant vertices off the
/// surfaces, a longer stretch is fitted unsolved from them, as a chord
/// 4.7e-3 mm off a 100 m radius or overshooting a tilted chord's ends
/// (cross reviews), so it is not cut at all; nor is one holding a chart
/// point more than [`FIT_TOLERANCE`] off the chord, which it would leave.
///
/// Where the two surfaces are known, the chord is also checked against the
/// curve itself: a point solved at its middle must lie on it within its
/// vertices' own distance from the surfaces and [`FIT_TOLERANCE`]
/// ([`chord_holds_on`]). Chart triples do not bound the curvature between
/// chart points (cross review: a thin ellipse's tip between them), and the
/// chart's own chordal error is far looser than the tolerance (5.4e-3 on a
/// keycap's charts).
pub(super) fn cut_ring_to_short_edge(crossing: Option<&Crossing>, chart: Vec<Vec3>, begin: Vec3, finish: Vec3) -> Vec<Vec3> {
    let n = chart.len();
    let length: f64 = chart.windows(2).map(|w| w[1].sub(w[0]).length()).sum();
    let chord = finish.sub(begin).length();
    if n < 3 || chord > RING_CUT_SHARE * length {
        return chart;
    }
    // A stretch holding a chart point off the chord keeps the loop: the
    // chord would drop it (cross review: a thin ellipse's tip, 5e-5 mm off).
    let cut = cut_ring_to_edge(chart.clone(), begin, finish);
    let direction = finish.sub(begin).scale(1.0 / chord);
    let off_chord = |p: &Vec3| {
        let along = p.sub(begin).dot(direction).clamp(0.0, chord);
        p.sub(begin.add(direction.scale(along))).length()
    };
    if cut.len() == n || cut.len() < 2 || cut[1..cut.len() - 1].iter().any(|p| off_chord(p) > FIT_TOLERANCE) {
        return chart;
    }
    let stretch = chord;
    // The least circumradius over the chart triples about the stretch.
    let ((_, sb), (_, sf)) = (place_on_chart(&chart, begin), place_on_chart(&chart, finish));
    let first = (sb.min(sf).floor() as usize).saturating_sub(1);
    let last = ((sb.max(sf).ceil() as usize) + 1).min(n - 1);
    let radius = (first..last.saturating_sub(1))
        .map(|i| {
            let (a, b, c) = (chart[i], chart[i + 1], chart[i + 2]);
            let twice_area = b.sub(a).cross(c.sub(a)).length();
            b.sub(a).length() * c.sub(b).length() * c.sub(a).length() / (2.0 * twice_area).max(1e-300)
        })
        .fold(f64::INFINITY, f64::min);
    let held = match crossing {
        Some(crossing @ Crossing::Surfaces(..)) => chord_holds_on(crossing, begin, finish),
        _ => true,
    };
    if held && radius.is_finite() && stretch * stretch / (8.0 * radius) <= FIT_TOLERANCE { vec![begin, finish] } else { chart }
}

/// Whether the chord from `begin` to `finish` lies on the curve two surfaces
/// cross in. Points of the curve are solved beside the chord's ends and at
/// each eighth along it, each in the plane normal to the curve there (the
/// surfaces' normals crossed; normal to the chord, a chord tilted between
/// unequally tolerant vertices reached past its ends). Each inner one must
/// lie within [`FIT_TOLERANCE`] of the chord, beyond the further vertex's
/// own distance from the curve (cross review: a vertex off both surfaces is
/// further from their curve than from either; one point at the middle
/// would miss an S-shaped excursion). Not where a point does not solve, or
/// the surfaces are tangent, or the crossing is a blend's contact.
pub(super) fn chord_holds_on(crossing: &Crossing, begin: Vec3, finish: Vec3) -> bool {
    let Crossing::Surfaces(first, second) = crossing else { return false };
    let chord = finish.sub(begin);
    let length = chord.length();
    if !(length > 0.0) {
        return false;
    }
    let axis = chord.scale(1.0 / length);
    // The point of the curve beside `p`, reaching no further than `reach`.
    let beside = |p: Vec3, reach: f64| -> Option<Vec3> {
        let (_, g1) = first.distance(p)?;
        let (_, g2) = second.distance(p)?;
        let direction = g1.cross(g2).normalized().ok()?;
        crossing.solve(p, direction, reach)
    };
    let reach = length + 1.0;
    let (Some(b), Some(f)) = (beside(begin, reach), beside(finish, reach)) else { return false };
    let slack = b.sub(begin).length().max(f.sub(finish).length()) + FIT_TOLERANCE;
    (1..8).all(|k| {
        let station = begin.add(chord.scale(k as f64 / 8.0));
        beside(station, 0.5 * length + 2.0 * slack).is_some_and(|p| {
            let along = p.sub(begin).dot(axis).clamp(0.0, length);
            p.sub(begin.add(axis.scale(along))).length() <= slack
        })
    })
}

/// A point's position along a chart (chord index plus the fraction along
/// it) and its distance from it: `(distance, position)`.
fn place_on_chart(chart: &[Vec3], p: Vec3) -> (f64, f64) {
    let mut best = (f64::INFINITY, 0.0);
    for i in 0..chart.len().saturating_sub(1) {
        let (a, b) = (chart[i], chart[i + 1]);
        let ab = b.sub(a);
        let t = (p.sub(a).dot(ab) / ab.dot(ab).max(1e-300)).clamp(0.0, 1.0);
        let d = p.sub(a.add(ab.scale(t))).length();
        if d < best.0 {
            best = (d, i as f64 + t);
        }
    }
    best
}

/// How far from a chart (mm) [`cut_chart`] places a vertex on it.
const CUT_CHART_REACH_MM: f64 = 1.0;

/// How far from an intersection's chart (mm) a re-seated vertex, and the
/// edge's other vertex, may stand for the edge to follow the seat: well
/// within [`CUT_CHART_REACH_MM`].
const FOLLOW_CHART_MM: f64 = 0.5 * CUT_CHART_REACH_MM;

fn cut_chart(chart: Vec<Vec3>, begin: Vec3, finish: Vec3) -> Vec<Vec3> {
    let n = chart.len();
    if n < 2 {
        return chart;
    }
    let place = |p: Vec3| place_on_chart(&chart, p);
    let ((db, sb), (df, sf)) = (place(begin), place(finish));
    if db > CUT_CHART_REACH_MM || df > CUT_CHART_REACH_MM || (sb - sf).abs() < 1e-12 {
        return chart;
    }
    let ((lo, first), (hi, last)) = if sb <= sf { ((sb, begin), (sf, finish)) } else { ((sf, finish), (sb, begin)) };
    let mut out = vec![first];
    for (j, p) in chart.iter().enumerate() {
        let s = j as f64;
        if s > lo && s < hi && p.sub(first).length() > 1e-9 && p.sub(last).length() > 1e-9 {
            out.push(*p);
        }
    }
    out.push(last);
    out
}

/// A closed chart (its last point its first) restarted in the gap an edge on
/// it leaves: the edge runs along the chart from `begin` to `finish` (points
/// on the loop), so the seam goes between `finish` and `begin`, at a chart
/// point there or, when none lies there, at the middle of the gap. Never at
/// a vertex of the edge (the STEP lane could then trim from the wrong end of
/// the curve), except for an edge that is the whole loop. Any other chart is
/// returned as it is.
pub(super) fn reseat_closed_chart(chart: Vec<Vec3>, begin: Vec3, finish: Vec3) -> Vec<Vec3> {
    let n = chart.len();
    if n < 4 || chart[0].sub(chart[n - 1]).length() > 1e-6 {
        return chart;
    }
    let ring = &chart[..n - 1];
    let m = ring.len();
    // A point's position along the ring: chord index plus the fraction along it.
    let position = |p: Vec3| -> f64 {
        let mut best = (f64::INFINITY, 0.0);
        for i in 0..m {
            let (a, b) = (ring[i], ring[(i + 1) % m]);
            let ab = b.sub(a);
            let t = (p.sub(a).dot(ab) / ab.dot(ab).max(1e-300)).clamp(0.0, 1.0);
            let d = p.sub(a.add(ab.scale(t))).length();
            if d < best.0 {
                best = (d, i as f64 + t);
            }
        }
        best.1
    };
    let at = |s: f64| {
        let i = (s.floor() as usize) % m;
        ring[i].add(ring[(i + 1) % m].sub(ring[i]).scale(s - s.floor()))
    };
    let rotated = |seam: Vec3, from: usize| {
        let mut out = vec![seam];
        out.extend((0..m).map(|i| ring[(from + i) % m]).filter(|p| p.sub(seam).length() > 1e-9));
        out.push(seam);
        out
    };
    let (start, end) = (position(begin), position(finish));
    // How far along the ring from `end` to `start`: the gap the edge leaves.
    let gap = (start - end).rem_euclid(m as f64);
    if begin.sub(finish).length() <= 1e-9 || gap <= 1e-12 {
        // The whole loop from one vertex: seam at that vertex.
        return rotated(begin, (start.floor() as usize + 1) % m);
    }
    // A chart point strictly inside the gap, nearest its middle.
    let middle = end + 0.5 * gap;
    let inside = (1..=m)
        .map(|k| end.floor() + k as f64)
        .filter(|&s| s > end && s < end + gap)
        .min_by(|a, b| (a - middle).abs().partial_cmp(&(b - middle).abs()).unwrap_or(std::cmp::Ordering::Equal));
    match inside {
        Some(s) => {
            let k = (s as usize) % m;
            rotated(ring[k], k)
        }
        None => rotated(at(middle), (middle.floor() as usize + 1) % m),
    }
}

/// What an intersection curve's points between chart points are solved on
/// ([`Translator::intersection_crossing`]).
pub(super) enum Crossing {
    /// Both surfaces, each with a distance.
    Surfaces(Surface, Surface),
    /// A rolling-ball blend's line of contact on one support: the ball of
    /// `radius` about each point of `spine` touches `support` there.
    Contact { spine: Curve, support: Surface, radius: f64 },
}

impl Crossing {
    /// A point of the curve near `station` (between chart points, a chord
    /// running along `direction`), no further than `reach` from it.
    fn solve(&self, station: Vec3, direction: Vec3, reach: f64) -> Option<Vec3> {
        match self {
            Crossing::Surfaces(first, second) => solve_on_both(first, second, station, direction, reach),
            Crossing::Contact { spine, support, radius } => {
                // The spine point nearest the station is the one whose
                // cross-section passes through it; the contact point of
                // that section is the support's foot from the spine point.
                let t = match spine {
                    Curve::Nurbs(curve) => crate::project_point_to_curve(curve, station).ok()?.u,
                    _ => geometry::nearest_parameter(spine, [true, true], |q| q.sub(station).length())?,
                };
                let centre = geometry::extended_point(spine, t).ok()?;
                let (distance, gradient) = support.distance(centre)?;
                if (distance.abs() - radius).abs() > 0.05 * radius + 1e-6 {
                    return None;
                }
                let p = centre.sub(gradient.scale(distance));
                (p.sub(station).length() <= reach).then_some(p)
            }
        }
    }
}

/// Parasolid's linear resolution (1e-8 m), in mm: points closer than this
/// are one point.
const RESOLUTION_MM: f64 = 1e-5;

/// An open chart without the points that crowd its ends. A terminator, or
/// the edge's vertex the chart is cut to, stands a fraction of a micron from
/// the chart's own end point, each where its own rounding puts it; a
/// chord-length cubic through such a cluster takes its end tangent from the
/// direction between them, which is noise, and swings far off the curve
/// (metres, on a 9-point chart along a blend boundary). Interior points
/// within `tolerances` (mm; the first end's, the last end's) of that end,
/// which the file states are that end (its vertex's tolerance, at least the
/// resolution), are dropped; the ends are kept, and any point the file
/// places apart from them. A closed chart is returned as it is.
pub(super) fn uncrowd_chart_ends(chart: Vec<Vec3>, [at_first, at_last]: [f64; 2]) -> Vec<Vec3> {
    let n = chart.len();
    if n < 3 || chart[0].sub(chart[n - 1]).length() <= 1e-6 {
        return chart;
    }
    let (first, last) = (chart[0], chart[n - 1]);
    let mut out = Vec::with_capacity(n);
    out.push(first);
    out.extend(chart[1..n - 1].iter().copied().filter(|p| p.sub(first).length() > at_first && p.sub(last).length() > at_last));
    out.push(last);
    out
}

/// The point on both surfaces and on the plane through `station` normal to
/// `direction`, by Newton from `station`; `None` when it does not converge
/// to the step tolerance or moves further than `reach` from the station (a
/// tangency, or a branch the chart does not mean).
fn solve_on_both(first: &Surface, second: &Surface, station: Vec3, direction: Vec3, reach: f64) -> Option<Vec3> {
    let mut x = station;
    for _ in 0..40 {
        if x.sub(station).length() > reach {
            return None;
        }
        let (f1, g1) = first.distance(x)?;
        let (f2, g2) = second.distance(x)?;
        let f3 = x.sub(station).dot(direction);
        let step = solve3([g1, g2, direction], [-f1, -f2, -f3])?;
        x = x.add(step);
        if step.length() < 1e-11 {
            let (f1, _) = first.distance(x)?;
            let (f2, _) = second.distance(x)?;
            return (f1.abs() < 1e-7 && f2.abs() < 1e-7).then_some(x);
        }
    }
    None
}

/// Solve the rows `a` · x = `b` by Cramer's rule; `None` when the rows are
/// nearly dependent (the surfaces meet tangentially there).
fn solve3(a: [Vec3; 3], b: [f64; 3]) -> Option<Vec3> {
    let det = a[0].dot(a[1].cross(a[2]));
    if det.abs() < 1e-9 {
        return None;
    }
    // Columns of the inverse are the cross products of row pairs.
    let x = a[1].cross(a[2]).scale(b[0]).add(a[2].cross(a[0]).scale(b[1])).add(a[0].cross(a[1]).scale(b[2]));
    Some(x.scale(1.0 / det))
}

/// How far a fitted curve may stray from the exact points it stands for (mm).
const FIT_TOLERANCE: f64 = 1e-5;
/// A whole blend face's vertex this near its surface's first section (mm),
/// but more than a hundred fits off it, is one STEP readers take for a
/// vertex on the seam ([`Translator::blend_surface`]).
const SEAM_ROW_NEAR: [f64; 2] = [100.0 * FIT_TOLERANCE, 0.1];
/// A boundary row of an offset pinned to a face's vertex
/// ([`Translator::pinned_offset`]): its ends within this of the vertex (mm),
/// its farthest point within the excursion of it (mm), the way out and the
/// way back within the fin's fraction of that apart, read at these samples.
const PIN_ENDS: f64 = 10.0 * FIT_TOLERANCE;
const PIN_EXCURSION: f64 = 0.5;
const PIN_FIN: f64 = 0.05;
const PIN_SAMPLES: usize = 32;
/// ...and every point of the face's boundary still this near the pinned
/// surface (mm).
const PIN_ON: f64 = 10.0 * FIT_TOLERANCE;
/// An offset extended past its base ([`Translator::extended_offset`]): its
/// face's SP-curves read at these samples each, reaching past the base's
/// domain by more than this fraction of it, and every point of the face's
/// boundary this near the extended offset (mm).
const EXTEND_SAMPLES: usize = 64;
const EXTEND_PAST: f64 = 1e-6;
const EXTEND_ON: f64 = 10.0 * FIT_TOLERANCE;
/// ...and the extended base read against its tangent planes at this many
/// steps a side over its strip.
const EXTEND_GRID: usize = 32;
/// Beyond this the fit is refused rather than handed on (mm).
const FIT_REFUSAL: f64 = 1e-3;
/// The densest sampling tried: 2^7 times the starting density, and never
/// more than [`MAX_FIT_STATIONS`] points.
const FIT_DOUBLINGS: usize = 7;
const MAX_FIT_STATIONS: usize = 4096;

/// A cubic through points taken piece by piece: `points(i, s)` gives piece
/// i sampled `s` times (its start included, its end not), and `last` ends
/// the run. Each piece starts at `start` samples; a piece where the fit
/// strays more than [`FIT_TOLERANCE`] from that piece sampled twice as
/// densely is doubled, and the others are left, until every piece is held,
/// one has doubled [`FIT_DOUBLINGS`] times, or every piece that misses is
/// within ten times the tolerance and no longer halves its miss when
/// doubled. Doubling only where the fit
/// misses keeps a curve whose curvature changes in one place (an
/// intersection on a swept surface running on into its section's
/// extension) from being sampled densely all along. The best fit is used if
/// none gets there, unless it misses by more than [`FIT_REFUSAL`].
pub(super) fn fit_by_pieces(
    pieces: usize,
    points: impl Fn(usize, usize) -> Result<Vec<Vec3>, String>,
    last: Vec3,
    start: usize,
) -> Result<NurbsCurve, String> {
    if pieces == 0 {
        return Err("the curve has no points".into());
    }
    let most = start << FIT_DOUBLINGS;
    let mut samples = vec![start; pieces];
    // Each piece's miss at the last round (none yet).
    let mut misses = vec![f64::INFINITY; pieces];
    let mut cache: HashMap<(usize, usize), Vec<Vec3>> = HashMap::default();
    let sampled = |i: usize, s: usize, cache: &mut HashMap<(usize, usize), Vec<Vec3>>| -> Result<Vec<Vec3>, String> {
        if let Some(found) = cache.get(&(i, s)) {
            return Ok(found.clone());
        }
        let found = points(i, s)?;
        cache.insert((i, s), found.clone());
        Ok(found)
    };
    let mut best: Option<(NurbsCurve, f64)> = None;
    loop {
        let mut run = Vec::new();
        for (i, &s) in samples.iter().enumerate() {
            run.extend(sampled(i, s, &mut cache)?);
        }
        run.push(last);
        let fit = fit_through(&run)?;
        let mut worst = 0.0f64;
        let mut missing = Vec::new();
        let mut doubling = Vec::new();
        for (i, &s) in samples.iter().enumerate() {
            let check = sampled(i, 2 * s, &mut cache)?;
            // A piece with nothing between its points (an intersection with
            // a surface the reader cannot measure gives only its chart) has
            // nothing to check: the fit passes through every point it has.
            let own = sampled(i, s, &mut cache)?;
            if check.len() == own.len() && check.iter().zip(&own).all(|(a, b)| a.sub(*b).length() == 0.0) {
                misses[i] = 0.0;
                continue;
            }
            let miss = check
                .iter()
                .map(|p| crate::project_point_to_curve(&fit, *p).map(|f| f.distance).unwrap_or(f64::INFINITY))
                .fold(0.0f64, f64::max);
            worst = worst.max(miss);
            if miss > FIT_TOLERANCE {
                missing.push(i);
                // A piece near the tolerance whose miss did not halve when it
                // was last doubled has met the floor its points give (a chart
                // end the file states only to its resolution): more samples
                // will not help.
                if miss > 10.0 * FIT_TOLERANCE || !(misses[i] <= 2.0 * miss) {
                    doubling.push(i);
                }
            }
            misses[i] = miss;
        }
        if missing.is_empty() {
            return Ok(fit);
        }
        if best.as_ref().map_or(true, |(_, b)| worst < *b) {
            best = Some((fit, worst));
        }
        let total: usize = samples.iter().sum();
        if doubling.is_empty()
            || doubling.iter().any(|&i| samples[i] >= most)
            || 2 * (total + doubling.iter().map(|&i| samples[i]).sum::<usize>()) > MAX_FIT_STATIONS
        {
            break;
        }
        for i in doubling {
            samples[i] *= 2;
        }
    }
    match best {
        Some((fit, worst)) if worst <= FIT_REFUSAL => Ok(fit),
        Some((_, worst)) => Err(format!("no cubic through its points comes within {FIT_REFUSAL} mm of it (closest {worst:.2e} mm)")),
        None => Err("the curve has no points".into()),
    }
}

/// An intersection's chart and what solves its exact points between chart
/// points ([`Crossing::solve`]). Under `BREP_IMPORT_LEAN` each station is
/// solved once: chord `i` at fraction k/s and at 2k/2s is the same station bit
/// for bit (both quotients of exact integers, correctly rounded), so the fit's
/// check density, every later round and the unverified count read the answer
/// already solved instead of solving it again.
struct ChartStations<'a> {
    crossing: Option<&'a Crossing>,
    chart: &'a [Vec3],
    memo: Option<std::cell::RefCell<HashMap<(usize, u64), Option<Vec3>>>>,
    /// Whether each chart point is on both surfaces as read
    /// ([`CHART_ON_SURFACE_MM`]): measured once, on the first chord asked for.
    on_surfaces: std::cell::OnceCell<Vec<bool>>,
}

/// The parameter of `curve`, continued past its ends along its end
/// tangents at its end speeds as [`geometry::extended_point`] continues it,
/// whose point is nearest `p`: a point past an end is not clamped to it.
pub(super) fn parameter_on_extended(curve: &NurbsCurve, p: Vec3) -> Result<f64, String> {
    let foot = crate::project_point_to_curve(curve, p)?.u;
    let [a, b] = curve.domain()?;
    let h = 1e-6 * (b - a);
    // How far past `end` (towards the outside, opposite `inward`) p lies.
    let beyond = |end: f64, inward: f64| -> Result<Option<f64>, String> {
        if (foot - end).abs() > h {
            return Ok(None);
        }
        let at = curve.evaluate(end)?;
        let velocity = at.sub(curve.evaluate(end + inward)?).scale(-1.0 / inward);
        let past = p.sub(at).dot(velocity) / velocity.dot(velocity).max(1e-300);
        Ok((past != 0.0 && (past > 0.0) == (inward < 0.0)).then_some(end + past))
    };
    Ok(beyond(b, -h)?.or(beyond(a, h)?).unwrap_or(foot))
}

/// An intersection curve's chart points with their parameters in the file
/// (§5.2.1.5): [`Translator::intersection_parameters`].
pub(super) struct ChartParameters {
    points: Vec<Vec3>,
    parameters: Vec<f64>,
    crossing: Crossing,
}

impl ChartParameters {
    /// The curve's point at the file's parameter `t`: a chart point's own
    /// position at its parameter; between two, the point of the curve on
    /// the plane normal to their chord through the chord's point
    /// proportioned by `t` (past an end, the end chord carried on).
    fn point(&self, t: f64) -> Result<Vec3, String> {
        let n = self.parameters.len();
        let i = self.parameters[1..n - 1].iter().take_while(|&&p| p <= t).count();
        let (t0, t1) = (self.parameters[i], self.parameters[i + 1]);
        let (a, b) = (self.points[i], self.points[i + 1]);
        let s = (t - t0) / (t1 - t0);
        if s == 0.0 {
            return Ok(a);
        }
        if s == 1.0 {
            return Ok(b);
        }
        let chord = b.sub(a);
        let direction = chord.normalized()?;
        self.crossing
            .solve(a.add(chord.scale(s)), direction, chord.length() * (1.0 + s.abs().max((s - 1.0).abs())))
            .ok_or_else(|| format!("no point of the intersection at its parameter {t}"))
    }
}

/// How far a chart point may stand from a surface it is stated on and still
/// count as on it: ten times the file's resolution, which a chart point is
/// rounded to (those of a curve the reader evaluates exactly sit within
/// 1e-11 mm of both surfaces; past a clamped base, 1e-2 mm and more).
const CHART_ON_SURFACE_MM: f64 = 10.0 * RESOLUTION_MM;

impl<'a> ChartStations<'a> {
    fn new(crossing: Option<&'a Crossing>, chart: &'a [Vec3]) -> Self {
        let memo = crate::step_import::import_lean().then(|| std::cell::RefCell::new(HashMap::default()));
        Self { crossing, chart, memo, on_surfaces: std::cell::OnceCell::new() }
    }

    /// The point of the curve at `fraction` of chord `i`, solved from the
    /// station there; `None` where the solve does not converge.
    fn station(&self, crossing: &Crossing, i: usize, fraction: f64, direction: Vec3) -> Option<Vec3> {
        let (a, b) = (self.chart[i], self.chart[i + 1]);
        let chord = b.sub(a);
        let solve = || crossing.solve(a.add(chord.scale(fraction)), direction, chord.length());
        let Some(memo) = &self.memo else { return solve() };
        if let Some(found) = memo.borrow().get(&(i, fraction.to_bits())) {
            return *found;
        }
        let found = solve();
        memo.borrow_mut().insert((i, fraction.to_bits()), found);
        found
    }

    /// The points of chord `i` of the chart, `splits - 1` of them solved
    /// exactly between its ends (its start included, its end not): see
    /// [`Self::refined`].
    fn chord(&self, i: usize, splits: usize) -> Vec<Vec3> {
        let (a, b) = (self.chart[i], self.chart[i + 1]);
        let mut out = vec![a];
        let Some(crossing) = self.crossing else { return out };
        let Ok(direction) = b.sub(a).normalized() else { return out };
        // A chord whose chart points are not on both surfaces as read runs
        // where the reader's surfaces are not the file's: past a B-surface's
        // domain, where an offset of it is extended and the reader's distance
        // stops at the domain's edge (chart points 0.04 mm off it, exact
        // elsewhere). Solving there would trace a different curve: the chart
        // alone stands for it.
        if let Crossing::Surfaces(first, second) = crossing {
            let on = self.on_surfaces.get_or_init(|| {
                let on = |p: &Vec3| [first, second].iter().all(|s| s.distance(*p).is_some_and(|(d, _)| d.abs() <= CHART_ON_SURFACE_MM));
                self.chart.iter().map(on).collect()
            });
            if !(on[i] && on[i + 1]) {
                return out;
            }
        }
        for k in 1..splits {
            // A point further from its station than the chord is long has
            // left the branch the chart traces.
            if let Some(p) = self.station(crossing, i, k as f64 / splits as f64, direction) {
                out.push(p);
            }
        }
        out
    }

    /// The chart with `splits - 1` exact points inside each chord: each solved
    /// onto both surfaces and the plane through it normal to the chord, which
    /// is how the reference defines the curve between chart points
    /// (§5.2.1.5). On a blend's boundary the points are the blend's contact
    /// points ([`Crossing::Contact`]). Without a solver (spun and swept
    /// surfaces, a cliff-edge blend's boundary), or where a point does not
    /// converge (a tangency), the chart alone stands there.
    fn refined(&self, splits: usize) -> Vec<Vec3> {
        let chart = self.chart;
        if self.crossing.is_none() {
            return chart.to_vec();
        }
        let mut out = Vec::with_capacity(chart.len() * splits);
        for i in 0..chart.len() - 1 {
            out.extend(self.chord(i, splits));
        }
        out.push(chart[chart.len() - 1]);
        out
    }

    /// The intersection curve fitted chord by chord ([`fit_by_pieces`]).
    ///
    /// A blend's line of contact that does not fit within the refusal bound
    /// (its spine or support not reaching part of it) is fitted through its
    /// chart alone, as it was before contact points were solved.
    /// The flag says the fit fell back to the chart, unchecked.
    fn fit(&self) -> Result<(NurbsCurve, bool), String> {
        let chart = self.chart;
        let fit = |stations: &ChartStations| fit_by_pieces(chart.len() - 1, |i, s| Ok(stations.chord(i, s)), chart[chart.len() - 1], 8);
        match (fit(self), self.crossing) {
            (Err(_), Some(Crossing::Contact { .. })) => Ok((fit(&ChartStations { crossing: None, chart, memo: None, on_surfaces: std::cell::OnceCell::new() })?, true)),
            (result, _) => Ok((result?, false)),
        }
    }
}


/// An SP-curve over [t0, t1] fitted knot span by knot span
/// ([`fit_by_pieces`]), and broken again where it crosses a crease of its
/// surface ([`surface_creases`]).
fn fit_sp_curve(surface: &Surface, uv: &NurbsCurve, range: [f64; 2], extend: [f64; 2]) -> Result<NurbsCurve, String> {
    fit_sp_curve_by(&|u, v| surface.point(u, v), uv, range, &surface_creases(surface), extend)
}

/// `offset` with each listed boundary row's poles (sides as
/// [`Translator::folded_rows`] numbers them) set on its vertex.
fn pin_rows(mut offset: NurbsSurface, pins: &[(usize, Vec3)]) -> Option<NurbsSurface> {
    let (nu, nv) = (offset.control_points.len(), offset.control_points[0].len());
    for &(side, vertex) in pins {
        let cells: Vec<(usize, usize)> = match side {
            0 => (0..nv).map(|j| (0, j)).collect(),
            1 => (0..nv).map(|j| (nu - 1, j)).collect(),
            2 => (0..nu).map(|i| (i, 0)).collect(),
            _ => (0..nu).map(|i| (i, nv - 1)).collect(),
        };
        for (i, j) in cells {
            let w = offset.control_points[i][j].w;
            offset.control_points[i][j] = crate::Vec4 { x: vertex.x * w, y: vertex.y * w, z: vertex.z * w, w };
        }
    }
    NurbsSurface::new(offset.degree_u, offset.degree_v, offset.knots_u.clone(), offset.knots_v.clone(), offset.control_points.clone()).ok()
}

/// Whether `p` lies within `tolerance` of `surface` (mm).
fn lies_on(surface: &NurbsSurface, p: Vec3, tolerance: f64) -> bool {
    crate::project_point_to_surface(surface, p).ok().and_then(|f| surface.evaluate(f.u, f.v).ok()).is_some_and(|q| q.sub(p).length() <= tolerance)
}

/// A clamped B-surface continued past one side of its domain by `delta`
/// along its boundary tangent plane, as Parasolid extends one: a new span
/// on that side, each control line's homogeneous point running on linearly
/// along its end derivative. Over the old domain the surface is unchanged
/// (its knots and poles kept), and with constant weights across the side
/// the strip is the tangent plane's ruled extension exactly; otherwise it
/// leaves it at second order in `delta` times the weights' change. None for
/// a side not clamped, or a strip whose weights would not stay positive.
fn tangent_extension(surface: &NurbsSurface, along_u: bool, at_low: bool, delta: f64) -> Option<NurbsSurface> {
    use crate::Vec4;
    let (degree, knots) = if along_u { (surface.degree_u, &surface.knots_u) } else { (surface.degree_v, &surface.knots_v) };
    let p = degree;
    let m = knots.len();
    if p == 0 || m < 2 * (p + 1) {
        return None;
    }
    let clamped = |slice: &[f64]| slice.iter().all(|k| *k == slice[0]);
    if (at_low && !clamped(&knots[..=p])) || (!at_low && !clamped(&knots[m - p - 1..])) {
        return None;
    }
    let (rows, columns) = (surface.control_points.len(), surface.control_points[0].len());
    let lines: Vec<Vec<Vec4>> = if along_u {
        (0..columns).map(|j| (0..rows).map(|i| surface.control_points[i][j]).collect()).collect()
    } else {
        surface.control_points.clone()
    };
    let lerp = |a: Vec4, d: Vec4, s: f64| Vec4 { x: a.x + d.x * s, y: a.y + d.y * s, z: a.z + d.z * s, w: a.w + d.w * s };
    let diff = |a: Vec4, b: Vec4, k: f64| Vec4 { x: (a.x - b.x) * k, y: (a.y - b.y) * k, z: (a.z - b.z) * k, w: (a.w - b.w) * k };
    let mut extended_lines = Vec::with_capacity(lines.len());
    for line in &lines {
        let n = line.len();
        // The homogeneous end derivative, and the line through the end
        // point along it as a Bézier span of the same degree over the strip:
        // a linear function's poles are its values at even steps.
        let (end, slope) = if at_low {
            (line[0], diff(line[1], line[0], p as f64 / (knots[p + 1] - knots[0])))
        } else {
            (line[n - 1], diff(line[n - 1], line[n - 2], p as f64 / (knots[m - 1] - knots[m - p - 2])))
        };
        let strip: Vec<Vec4> = (0..=p).map(|i| lerp(end, slope, if at_low { -delta * (p - i) as f64 / p as f64 } else { delta * i as f64 / p as f64 })).collect();
        if strip.iter().any(|q| !(q.w > 0.0)) {
            return None;
        }
        let mut out = Vec::with_capacity(n + p);
        if at_low {
            out.extend_from_slice(&strip[..p]);
            out.extend_from_slice(line);
        } else {
            out.extend_from_slice(line);
            out.extend_from_slice(&strip[1..]);
        }
        extended_lines.push(out);
    }
    // The end knot kept to multiplicity p between the old span and the
    // strip, the strip's far end clamped.
    let new_knots: Vec<f64> = if at_low {
        std::iter::repeat(knots[0] - delta).take(p + 1).chain(knots[1..].iter().copied()).collect()
    } else {
        knots[..m - 1].iter().copied().chain(std::iter::repeat(knots[m - 1] + delta).take(p + 1)).collect()
    };
    let net: Vec<Vec<Vec4>> = if along_u {
        let length = extended_lines[0].len();
        (0..length).map(|i| extended_lines.iter().map(|line| line[i]).collect()).collect()
    } else {
        extended_lines
    };
    let (knots_u, knots_v) = if along_u { (new_knots, surface.knots_v.clone()) } else { (surface.knots_u.clone(), new_knots) };
    NurbsSurface::new(surface.degree_u, surface.degree_v, knots_u, knots_v, net).ok()
}

/// The u and v values of a B-surface's interior knots of multiplicity at
/// least its degree, where it is only C0 (a surface written as Bézier
/// patches): a curve across one can turn there, which a cubic fitted
/// through evenly spaced points rounds off. An offset has its base's.
fn surface_creases(surface: &Surface) -> [Vec<f64>; 2] {
    let creases = |knots: &[f64], degree: usize| -> Vec<f64> {
        let (first, last) = (knots[0], knots[knots.len() - 1]);
        let mut out: Vec<f64> = Vec::new();
        for &k in knots.iter().filter(|&&k| k > first && k < last) {
            if out.last() != Some(&k) && knots.iter().filter(|&&j| j == k).count() >= degree {
                out.push(k);
            }
        }
        out
    };
    match surface {
        Surface::Nurbs(n) if !n.knots_u.is_empty() && !n.knots_v.is_empty() => {
            [creases(&n.knots_u, n.degree_u), creases(&n.knots_v, n.degree_v)]
        }
        Surface::Offset { base, .. } => surface_creases(base),
        _ => [Vec::new(), Vec::new()],
    }
}

/// [`fit_sp_curve`] on a surface given by its point function, with the
/// surface's creases in u and v.
fn fit_sp_curve_by(
    point: &impl Fn(f64, f64) -> Result<Vec3, String>,
    uv: &NurbsCurve,
    [t0, t1]: [f64; 2],
    creases: &[Vec<f64>; 2],
    extend: [f64; 2],
) -> Result<NurbsCurve, String> {
    let mut breaks = vec![t0];
    breaks.extend(uv.knots.iter().copied().filter(|&k| k > t0 && k < t1));
    breaks.push(t1);
    breaks.dedup();
    let at = |t: f64| -> Result<Vec3, String> {
        let p = uv_on_tangents(uv, [t0, t1], t)?;
        point(p.x, p.y)
    };
    if creases.iter().any(|c| !c.is_empty()) {
        // Where the 2D curve crosses a crease: each span is scanned in 16
        // steps for a change of side, then bisected; a step landing on the
        // crease is a crossing itself. (Two crossings inside one step are
        // not seen; that stretch is fitted as before, and checked.)
        let side = |t: f64, axis: usize, c: f64| -> Result<f64, String> {
            let p = uv.evaluate(t)?;
            Ok(if axis == 0 { p.x - c } else { p.y - c })
        };
        let mut crossings = Vec::new();
        for pair in breaks.windows(2) {
            for (axis, values) in creases.iter().enumerate() {
                for &c in values {
                    let mut a = pair[0];
                    let mut fa = side(a, axis, c)?;
                    for k in 1..=16 {
                        let b = pair[0] + (pair[1] - pair[0]) * k as f64 / 16.0;
                        let fb = side(b, axis, c)?;
                        if fb == 0.0 && k < 16 {
                            crossings.push(b);
                        } else if fa * fb < 0.0 {
                            let (mut lo, mut hi, mut flo) = (a, b, fa);
                            for _ in 0..60 {
                                let mid = 0.5 * (lo + hi);
                                let fm = side(mid, axis, c)?;
                                if flo * fm <= 0.0 {
                                    hi = mid;
                                } else {
                                    (lo, flo) = (mid, fm);
                                }
                            }
                            crossings.push(0.5 * (lo + hi));
                        }
                        (a, fa) = (b, fb);
                    }
                }
            }
        }
        // A crossing is added only away from every break already there (the
        // 2D curve's own knots are all kept), in parameter and on the
        // surface: one whose stretch to a break stays within ten times the
        // file's resolution of it (a crease a hair before the curve's end)
        // would only crowd that piece's points together. The stretch is
        // looked at in quarters, so a turn between two such creases keeps
        // them.
        let close = 1e-9 * (t1 - t0);
        for t in crossings {
            let mut apart = true;
            for &b in &breaks {
                let near = if (t - b).abs() <= close {
                    true
                } else {
                    let base = at(b)?;
                    let mut within = true;
                    for k in 1..=4 {
                        if at(b + (t - b) * f64::from(k) / 4.0)?.sub(base).length() > CHART_ON_SURFACE_MM {
                            within = false;
                            break;
                        }
                    }
                    within
                };
                if near {
                    apart = false;
                    break;
                }
            }
            if apart {
                breaks.push(t);
            }
        }
        breaks.sort_by(f64::total_cmp);
    }
    // The stretches run on past each end along the 2D curve's tangent there
    // ([`sp_curve_extension`]): a piece each.
    if extend[0] > 0.0 {
        breaks.insert(0, t0 - extend[0]);
    }
    if extend[1] > 0.0 {
        breaks.push(t1 + extend[1]);
    }
    fit_by_pieces(
        breaks.len() - 1,
        |i, s| (0..s).map(|k| at(breaks[i] + (breaks[i + 1] - breaks[i]) * k as f64 / s as f64)).collect(),
        at(t1 + extend[1].max(0.0))?,
        8,
    )
}

/// A 2D curve's point at `t`, inside [t0, t1] the curve's own and past
/// either end straight on along its tangent there.
fn uv_on_tangents(uv: &NurbsCurve, [t0, t1]: [f64; 2], t: f64) -> Result<Vec3, String> {
    if t >= t0 && t <= t1 {
        return uv.evaluate(t);
    }
    let h = 1e-6 * (t1 - t0).abs().max(1e-12);
    let (end, inner) = if t < t0 { (t0, t0 + h) } else { (t1, t1 - h) };
    let (p, q) = (uv.evaluate(end)?, uv.evaluate(inner)?);
    let velocity = p.sub(q).scale(1.0 / (end - inner));
    Ok(p.add(velocity.scale(t - end)))
}

/// How far past each end of [t0, t1] an SP-curve's 2D curve runs on along its
/// tangent for its surface point to reach that end's vertex: for an end whose
/// vertex (with its tolerance, in the curve's direction) stands off it by more
/// than ten times the resolution and within the tolerance, where the vertex
/// lies along that extension: it comes to within half the gap and within
/// [`SNAP_ENDS_MM`], which [`snap_ends`] then closes. Parasolid leaves a
/// curve short of a tolerant vertex by up to its tolerance (two SP-curves
/// stopping 0.33 mm either side of a 0.33 mm vertex, in line with it; on the
/// surface the extension passes 0.01 to 0.04 mm from it); the extension stays
/// on the surface, where moving the end the whole way would bend the curve off
/// it. 0 elsewhere.
fn sp_curve_extension(
    point: &impl Fn(f64, f64) -> Result<Vec3, String>,
    uv: &NurbsCurve,
    [t0, t1]: [f64; 2],
    vertices: [(Vec3, f64); 2],
) -> [f64; 2] {
    let at = |t: f64| -> Option<Vec3> {
        let p = uv_on_tangents(uv, [t0, t1], t).ok()?;
        point(p.x, p.y).ok()
    };
    let mut out = [0.0; 2];
    for (k, (end, outwards)) in [(t0, -1.0), (t1, 1.0)].into_iter().enumerate() {
        let (vertex, tolerance) = vertices[k];
        let Some(here) = at(end) else { continue };
        let gap = here.sub(vertex).length();
        if !(gap > 10.0 * RESOLUTION_MM && gap <= tolerance + RESOLUTION_MM) {
            continue;
        }
        // Golden sections, no further than the parameter that covers twice
        // the gap at the end's speed: the vertex stands just off the end, so
        // a stretch that goes further (round a periodic surface and back,
        // cross review) is not running on to it.
        let miss = |d: f64| at(end + outwards * d).map_or(f64::INFINITY, |p| p.sub(vertex).length());
        let h = 1e-6 * (t1 - t0).abs().max(1e-12);
        let speed = at(end + outwards * h).map_or(0.0, |p| p.sub(here).length() / h);
        if !(speed > 0.0) {
            continue;
        }
        let golden = 0.5 * (5f64.sqrt() - 1.0);
        let (mut lo, mut hi) = (0.0, (t1 - t0).abs().min(2.0 * gap / speed));
        for _ in 0..80 {
            let (a, b) = (hi - golden * (hi - lo), lo + golden * (hi - lo));
            if miss(a) <= miss(b) {
                hi = b;
            } else {
                lo = a;
            }
        }
        let d = 0.5 * (lo + hi);
        let left = miss(d);
        // Heading for it: halfway along, already nearer than the end.
        if d > 0.0 && left <= 0.5 * gap && left <= SNAP_ENDS_MM && miss(0.5 * d) < gap {
            out[k] = d;
        }
    }
    out
}

/// Points of an SP-curve, `per_span` to each knot span of its 2D curve,
/// mapped through its surface.
fn sp_curve_points(surface: &Surface, uv: &NurbsCurve, per_span: usize, [t0, t1]: [f64; 2]) -> Result<Vec<Vec3>, String> {
    // Knot spans clipped to [t0, t1].
    let mut breaks = vec![t0];
    breaks.extend(uv.knots.iter().copied().filter(|&k| k > t0 && k < t1));
    breaks.push(t1);
    breaks.dedup();
    let mut params = Vec::new();
    for pair in breaks.windows(2).filter(|w| w[1] > w[0]) {
        for k in 0..per_span {
            params.push(pair[0] + (pair[1] - pair[0]) * k as f64 / per_span as f64);
        }
    }
    params.push(t1);
    params
        .into_iter()
        .map(|t| {
            let p = uv.evaluate(t)?;
            surface.point(p.x, p.y)
        })
        .collect()
}

/// An SP-curve said exactly, running the way its 2D curve's parameter does:
/// on a plane, its 2D B-curve mapped through the plane (an affine map moves
/// the control points and keeps the weights); on a cylinder, cone, sphere or
/// torus, a 2D curve at constant u or v is that surface's straight or
/// circular iso-line. `None` where neither holds, and the curve is fitted.
fn exact_sp_curve(surface: &Surface, uv: &NurbsCurve) -> Option<Curve> {
    const M: f64 = MM_PER_METRE;
    if let Surface::Plane(f) = surface {
        let points = uv
            .control_points
            .iter()
            .map(|c| {
                let p = f.origin.scale(c.w).add(f.x.scale(c.x * M)).add(f.y().scale(c.y * M));
                crate::Vec4 { x: p.x, y: p.y, z: p.z, w: c.w }
            })
            .collect();
        return NurbsCurve::new(uv.degree, uv.knots.clone(), points).ok().map(Curve::Nurbs);
    }
    if !surface.is_analytic() {
        return None;
    }
    let plain: Vec<(f64, f64)> = uv.control_points.iter().map(|c| (c.x / c.w, c.y / c.w)).collect();
    let (first, last) = (*plain.first()?, *plain.last()?);
    let flat = |pick: fn(&(f64, f64)) -> f64| plain.iter().all(|p| (pick(p) - pick(&first)).abs() <= 1e-12 * (1.0 + pick(&first).abs()));
    let (constant_u, constant_v) = (flat(|p| p.0), flat(|p| p.1));
    // Which way the curve runs along the iso-line's own parameter.
    let (along, forward) = match (constant_u, constant_v) {
        (true, false) => (false, last.1 > first.1),
        (false, true) => (true, last.0 > first.0),
        _ => return None,
    };
    let (u0, v0) = first;
    // An iso-line in u (constant v) on a surface of revolution about the
    // frame's axis is a circle about that axis.
    let about_axis = |frame: &Frame| -> Option<Curve> {
        let p = surface.point(0.0, v0).ok()?;
        let height = p.sub(frame.origin).dot(frame.z);
        let centre = frame.origin.add(frame.z.scale(height));
        let radius = p.sub(centre).length();
        let z = if forward { frame.z } else { frame.z.scale(-1.0) };
        (radius > 1e-9).then(|| Curve::Circle { frame: Frame { origin: centre, z, x: frame.x }, radius })
    };
    // A circle in the half-plane at angle u0 (a sphere's meridian, a torus's
    // tube): v turns from the radial towards the axis about radial × axis.
    let in_half_plane = |frame: &Frame, centre: Vec3, radius: f64| -> Option<Curve> {
        let radial = frame.x.scale(u0.cos()).add(frame.y().scale(u0.sin()));
        let normal = radial.cross(frame.z);
        let normal = if forward { normal } else { normal.scale(-1.0) };
        (radius > 1e-9).then(|| Curve::Circle { frame: Frame { origin: centre, z: normal, x: radial }, radius })
    };
    match (surface, along) {
        (Surface::Cylinder { .. } | Surface::Cone { .. }, false) => {
            // A ruling: straight from v0 along v.
            let a = surface.point(u0, v0).ok()?;
            let b = surface.point(u0, v0 + 1.0).ok()?;
            let direction = b.sub(a).normalized().ok()?;
            let direction = if forward { direction } else { direction.scale(-1.0) };
            Some(Curve::Line { point: a, direction })
        }
        (Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Sphere { frame, .. } | Surface::Torus { frame, .. }, true) => {
            about_axis(frame)
        }
        (Surface::Sphere { frame, radius }, false) => in_half_plane(frame, frame.origin, *radius),
        (Surface::Torus { frame, major, minor }, false) => {
            let radial = frame.x.scale(u0.cos()).add(frame.y().scale(u0.sin()));
            in_half_plane(frame, frame.origin.add(radial.scale(*major)), *minor)
        }
        _ => None,
    }
}

/// How far `p` stands from a chart (mm), along its chords.
fn chart_distance(chart: &[Vec3], p: Vec3) -> f64 {
    place_on_chart(chart, p).0
}

/// The length (mm) of a chart between two places on it ([`place_on_chart`]:
/// chord index plus the share along it).
pub(super) fn chart_length_between(chart: &[Vec3], a: f64, b: f64) -> f64 {
    let (a, b) = (a.min(b), a.max(b));
    let at = |s: f64| {
        let i = (s.floor() as usize).min(chart.len().saturating_sub(2));
        chart[i].add(chart[i + 1].sub(chart[i]).scale(s - i as f64))
    };
    let (first, last) = (a.floor() as usize + 1, b.floor() as usize);
    if first > last {
        return at(b).sub(at(a)).length();
    }
    let mut length = chart[first].sub(at(a)).length() + at(b).sub(chart[last.min(chart.len() - 1)]).length();
    for k in first..last.min(chart.len() - 1) {
        length += chart[k + 1].sub(chart[k]).length();
    }
    length
}

/// Of a curve's two ends, the one an edge from `here` to `there` (its two
/// vertices) starts at: the pairing of ends with vertices that lies
/// nearer, as [`snap_ends`] pairs them, so the two vertices of a short
/// edge never take the same end (cross review). `None` when neither
/// pairing is nearer (by a nanometre): the vertices do not say which end
/// is whose.
pub(super) fn paired_end([first, last]: [Vec3; 2], here: Vec3, there: Vec3) -> Option<Vec3> {
    let straight = first.sub(here).length() + last.sub(there).length();
    let crossed = first.sub(there).length() + last.sub(here).length();
    if (straight - crossed).abs() <= 1e-6 {
        None
    } else if straight < crossed {
        Some(first)
    } else {
        Some(last)
    }
}

/// Where a tolerant vertex at `at` (mm), of stated `tolerance`, is written,
/// given the `ends` of the curves that end there and whether a point stays
/// near enough every intersection cut to the vertex (`follows`): the centre
/// of the least ball about the ends, when some end stands more than
/// [`SNAP_ENDS_MM`] from `at`, the ball's radius is within it (so
/// [`snap_ends`] then closes a trimmed SP-curve's end, and a whole one's is
/// left within it, as a vertex under 0.1 mm off is today), its centre is
/// within the tolerance of `at`, and it `follows`. `None` keeps the file's
/// point.
pub(super) fn reseat(at: Vec3, tolerance: f64, ends: &[Vec3], follows: impl Fn(Vec3) -> bool) -> Option<Vec3> {
    if ends.iter().any(|end| end.sub(at).length() > tolerance) || ends.iter().all(|end| end.sub(at).length() <= SNAP_ENDS_MM) {
        return None;
    }
    let (centre, radius) = least_ball(ends)?;
    (radius <= SNAP_ENDS_MM && centre.sub(at).length() <= tolerance && follows(centre)).then_some(centre)
}

/// The least ball (centre, radius, mm) holding every one of `points`, for
/// up to a few points: the least of the balls on one point, on two as a
/// diameter, or through three on their circumcircle, that holds them all.
/// `None` for no points, or when only a ball through four would hold them.
pub(super) fn least_ball(points: &[Vec3]) -> Option<(Vec3, f64)> {
    let holds = |centre: Vec3, radius: f64| points.iter().all(|p| p.sub(centre).length() <= radius * (1.0 + 1e-12) + 1e-12);
    let mut best: Option<(Vec3, f64)> = None;
    let mut offer = |centre: Vec3, radius: f64| {
        if best.is_none_or(|(_, r)| radius < r) && holds(centre, radius) {
            best = Some((centre, radius));
        }
    };
    let n = points.len();
    for i in 0..n {
        offer(points[i], 0.0);
        for j in i + 1..n {
            let centre = points[i].add(points[j]).scale(0.5);
            offer(centre, centre.sub(points[i]).length());
            for k in j + 1..n {
                // The circumcentre of the triangle, in its plane.
                let (a, b) = (points[j].sub(points[i]), points[k].sub(points[i]));
                let normal = a.cross(b);
                let area = normal.dot(normal);
                if area <= 0.0 {
                    continue;
                }
                let centre = points[i].add(b.cross(normal).scale(a.dot(a)).add(normal.cross(a).scale(b.dot(b))).scale(0.5 / area));
                offer(centre, centre.sub(points[i]).length());
            }
        }
    }
    best
}

/// `curve` with its end control points moved onto the edge's vertices (the
/// nearer to each end), where both stand within a tenth of a millimetre of
/// the ends they replace; otherwise unchanged. A clamped B-spline's ends
/// are its end control points, so only the spans at its ends move, by the
/// gap.
/// The furthest [`snap_ends`] moves a curve's end onto its vertex (mm).
const SNAP_ENDS_MM: f64 = 0.1;

fn snap_ends(mut curve: NurbsCurve, (start, end): (Vec3, Vec3)) -> NurbsCurve {
    let n = curve.control_points.len();
    if n < 2 {
        return curve;
    }
    let point = |c: &crate::Vec4| Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w);
    let (first, last) = (point(&curve.control_points[0]), point(&curve.control_points[n - 1]));
    let (a, b) = if first.sub(start).length() + last.sub(end).length() <= first.sub(end).length() + last.sub(start).length() {
        (start, end)
    } else {
        (end, start)
    };
    if first.sub(a).length() > SNAP_ENDS_MM || last.sub(b).length() > SNAP_ENDS_MM {
        return curve;
    }
    for (k, p) in [(0, a), (n - 1, b)] {
        let w = curve.control_points[k].w;
        curve.control_points[k] = crate::Vec4 { x: p.x * w, y: p.y * w, z: p.z * w, w };
    }
    curve
}

/// An edge's positive and negative fins. Parasolid points an edge at its
/// positive fin, but not every writer does (a V12 file seen points 23 of
/// its 54 edges at the negative one), so the fins' own senses decide.
pub(super) fn edge_fins<'f>(file: &'f XtFile, edge: &Node) -> Result<(&'f Node, &'f Node), &'static str> {
    let first = file.follow(edge, "halfedge").ok_or("has no fin")?;
    let other = file.follow(first, "other").ok_or("has one fin")?;
    Ok(if first.char("sense") == Some(b'-') && other.char("sense") != Some(b'-') { (other, first) } else { (first, other) })
}

/// Whether a loop's points, in order, go once round a closed B-curve spine:
/// their feet's steps, each taken the short way round the period, add up to
/// a whole period (to a quarter of one).
fn winds_round(spine: &Curve, points: &[Vec3]) -> bool {
    let (Curve::Nurbs(curve), Some([a, b])) = (spine, geometry::closed_range(spine)) else { return false };
    let period = b - a;
    let feet: Vec<f64> = points.iter().filter_map(|p| crate::project_point_to_curve(curve, *p).ok().map(|f| f.u)).collect();
    if feet.len() < 3 {
        return false;
    }
    let step = |from: f64, to: f64| {
        let d = (to - from).rem_euclid(period);
        if d > 0.5 * period { d - period } else { d }
    };
    let mut total = 0.0;
    for k in 0..feet.len() {
        total += step(feet[k], feet[(k + 1) % feet.len()]);
    }
    (total.abs() - period).abs() <= 0.25 * period
}

/// The parameter range of `spine` that the points lie along, widened by a
/// fiftieth each way (within a bounded curve's own range).
pub(super) fn spine_range(spine: &Curve, points: &[Vec3], margin: f64, extend: bool) -> Option<[f64; 2]> {
    let params: Vec<f64> = match spine {
        Curve::Line { point, direction } => points.iter().map(|p| p.sub(*point).dot(*direction) / MM_PER_METRE).collect(),
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => {
            // A circle's foot is the point's polar angle; an ellipse's
            // parameter is its eccentric angle, which the polar angle is not
            // (off the axes it fell short of a face's ends and clipped a blend
            // face by 2%), so its foot is searched for.
            let angle = |p: &Vec3| match spine {
                Curve::Ellipse { .. } => geometry::nearest_parameter(spine, [false, false], |q| q.sub(*p).length()).unwrap_or(f64::NAN),
                _ => {
                    let d = p.sub(frame.origin);
                    d.dot(frame.y()).atan2(d.dot(frame.x))
                }
            };
            // The face spans the circle less its largest gap between
            // boundary points; a face whose points leave no gap wider than
            // 45° (a ring all the way round, sampled) takes the whole circle,
            // which over-covers a face that is not quite whole, harmlessly.
            let mut angles: Vec<f64> = points.iter().map(|p| angle(p).rem_euclid(std::f64::consts::TAU)).filter(|a| a.is_finite()).collect();
            angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let first = *angles.first()?;
            let (mut gap, mut after) = (first + std::f64::consts::TAU - angles[angles.len() - 1], first);
            for pair in angles.windows(2) {
                if pair[1] - pair[0] > gap {
                    (gap, after) = (pair[1] - pair[0], pair[1]);
                }
            }
            if gap < std::f64::consts::FRAC_PI_4 {
                return Some([0.0, std::f64::consts::TAU]);
            }
            angles.iter().map(|a| after + (a - after).rem_euclid(std::f64::consts::TAU)).collect()
        }
        Curve::Nurbs(curve) => {
            let feet: Vec<f64> = points.iter().filter_map(|p| crate::project_point_to_curve(curve, *p).ok().map(|f| f.u)).collect();
            // A closed spine is a ring, read as a circle's is: the face
            // spans the ring less its largest gap between boundary points,
            // or all of it, exactly one period (its first and last sections
            // the same, so the fitted surface closes); a stretch across the
            // seam runs past the domain, where the spine wraps.
            if let Some([a, b]) = geometry::closed_range(spine) {
                let period = b - a;
                let mut along: Vec<f64> = feet.iter().map(|t| a + (t - a).rem_euclid(period)).collect();
                along.sort_by(|x, y| x.total_cmp(y));
                let first = *along.first()?;
                let (mut gap, mut after) = (first + period - along[along.len() - 1], first);
                for pair in along.windows(2) {
                    if pair[1] - pair[0] > gap {
                        (gap, after) = (pair[1] - pair[0], pair[1]);
                    }
                }
                if gap < period / 8.0 {
                    return Some([a, b]);
                }
                let span = period - gap;
                let pad = (span * margin).min(0.5 * gap);
                return Some([after - pad, after + span + pad]);
            }
            // An open spine's feet are taken on its extension too: a face
            // can run past where the spine curve stops (by 0.2 mm on one
            // part), and the sections follow the extension there.
            let [a, b] = curve.domain().ok()?;
            if !extend {
                return range_of(&feet, margin, [a, b]);
            }
            points
                .iter()
                .zip(&feet)
                .map(|(p, &foot)| {
                    if foot > a && foot < b {
                        return foot;
                    }
                    geometry::nearest_parameter(spine, [true, true], |q| q.sub(*p).length()).unwrap_or(foot)
                })
                .collect()
        }
    };
    let (lo, hi) = params.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
    if !(hi > lo) {
        return None;
    }
    let margin = (hi - lo) * margin;
    let (mut lo, mut hi) = (lo - margin, hi + margin);
    // Past the spine's ends only as far as the face goes.
    if let Curve::Nurbs(curve) = spine {
        let [a, b] = curve.domain().ok()?;
        let (first, last) = params.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(x, y), &t| (x.min(t), y.max(t)));
        (lo, hi) = (lo.max(a.min(first)), hi.min(b.max(last)));
    }
    Some([lo, hi])
}

/// `params`' span, widened by `margin` of itself each way, within `domain`.
fn range_of(params: &[f64], margin: f64, [a, b]: [f64; 2]) -> Option<[f64; 2]> {
    let (lo, hi) = params.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(x, y), &t| (x.min(t), y.max(t)));
    if !(hi > lo) {
        return None;
    }
    let pad = (hi - lo) * margin;
    Some([(lo - pad).max(a), (hi + pad).min(b)])
}

/// A blend's cross-section at spine parameter `t`: the arc about the spine
/// point from its foot on the first support to its foot on the second, as a
/// rational quadratic's homogeneous control points.
/// The point of a blend section (a circular rational quadratic about
/// `centre`) where the angle from its start is `v` of the whole: the blend's
/// own v (§5.2.2.6). On a rational quadratic of half-angle α the angle φ from
/// its start has tan((φ - α)/2) = tan(α/2)·(2w - 1). A section collapsed to
/// a point (the supports meeting tangentially) is that point.
pub(super) fn section_at_angle(s: &[crate::Vec4; 3], centre: Vec3, v: f64) -> Vec3 {
    let at = |c: &crate::Vec4| Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w);
    let (a, b) = (at(&s[0]).sub(centre), at(&s[2]).sub(centre));
    let angle = (a.dot(b) / (a.length() * b.length()).max(1e-300)).clamp(-1.0, 1.0).acos();
    if !(angle > 1e-9) {
        return at(&s[0]);
    }
    let half = 0.5 * angle;
    let w = 0.5 + 0.5 * ((v * angle - half) / 2.0).tan() / (half / 2.0).tan();
    let basis = [(1.0 - w) * (1.0 - w), 2.0 * w * (1.0 - w), w * w];
    let (mut x, mut y, mut z, mut h) = (0.0, 0.0, 0.0, 0.0);
    for (k, c) in basis.iter().zip(s) {
        (x, y, z, h) = (x + k * c.x, y + k * c.y, z + k * c.z, h + k * c.w);
    }
    Vec3::new(x / h, y / h, z / h)
}

/// A blend's support: a surface the ball rolls on, or the curve of the edge
/// a cliff-edge blend rolls over.
pub(super) enum Support {
    Surface(Surface),
    Cliff(Curve),
    /// Another blend: the pipe of `radius` about its spine, or, where that
    /// blend's centres are refined (`inner`: its supports and radius), about
    /// the refined centres.
    Pipe { spine: Curve, radius: f64, inner: Option<Box<([Support; 2], f64)>> },
}

/// The point of `curve` nearest `x`: in closed form on a line (wherever its
/// stored point is), by search otherwise; with its parameter.
fn nearest_on(curve: &Curve, x: Vec3) -> Option<(f64, Vec3)> {
    if let Curve::Line { point, direction } = curve {
        let d = direction.normalized().ok()?;
        let s = x.sub(*point).dot(d);
        return Some((s / (direction.length() * MM_PER_METRE), point.add(d.scale(s))));
    }
    let k = geometry::nearest_parameter(curve, [true, true], |q| q.sub(x).length())?;
    Some((k, geometry::extended_point(curve, k).ok()?))
}

/// A pipe's axis point nearest `x`: its spine's, or the refined centre curve's.
/// The refined curve c(k) is not the spine moved square to it (a chart can
/// lean against the true axis), so its nearest point is found on it: Newton
/// on (x - c(k))·c'(k) = 0 from the spine's nearest parameter, c' by central
/// differences (re-review: refining the spine's nearest point put a contact
/// 0.0044 mm inside a supporting blend).
fn pipe_axis_point(spine: &Curve, inner: &Option<Box<([Support; 2], f64)>>, x: Vec3) -> Option<Vec3> {
    let (mut k, near) = nearest_on(spine, x)?;
    let Some(inner) = inner else { return Some(near) };
    let centre = |k: f64| ball_centre(&inner.0, inner.1, spine, k).ok();
    // A step of a millionth of the range (of a millimetre on a line, whose
    // parameter is in metres).
    let h = match spine.natural_range() {
        Some([a, b]) if b > a => 1e-6 * (b - a),
        _ => 1e-6 / MM_PER_METRE,
    };
    // (x - c(k))·c'(k) and |c'(k)|².
    let f = |k: f64| -> Option<(f64, f64)> {
        let slope = centre(k + h)?.sub(centre(k - h)?).scale(0.5 / h);
        Some((x.sub(centre(k)?).dot(slope), slope.dot(slope)))
    };
    let (mut f0, speed) = f(k)?;
    if !(speed > 0.0) {
        return centre(k);
    }
    // The first step as if the curve were straight; then secant steps,
    // which take its bend into account. Stepped as if straight throughout,
    // each step is (x - c)·c''/|c'|² times the last: past the curve's
    // centre of curvature (36.7 mm from a centre curve bending at 12.2) it
    // grew threefold each time and stopped wherever it was, 1.7e-3 mm off
    // and different at each section, so no surface came within 1e-3 mm of
    // a blend resting on that pipe.
    // Each step is kept only where the centre it reaches is no farther from
    // `x` than the nearest so far, halved until it is: the point sought is
    // the nearest, and an unbounded step could run where the centre curve
    // has no centres and stop on a stray one (cross review: 0.33 mm off).
    let distance = |k: f64| centre(k).map_or(f64::INFINITY, |c| x.sub(c).length());
    let mut best = (k, distance(k));
    let mut k1 = k + f0 / speed;
    for _ in 0..30 {
        let mut d1 = distance(k1);
        let mut halvings = 0;
        while !(d1 <= best.1 + 1e-12 * (1.0 + best.1)) && halvings < 20 {
            k1 = 0.5 * (k1 + best.0);
            d1 = distance(k1);
            halvings += 1;
        }
        if !(d1 <= best.1 + 1e-12 * (1.0 + best.1)) {
            break;
        }
        best = (k1, d1);
        if (k1 - k).abs() <= 1e-13 * (1.0 + k1.abs()) {
            break;
        }
        let (f1, _) = f(k1)?;
        let denominator = f1 - f0;
        if denominator == 0.0 || !denominator.is_finite() {
            break;
        }
        let next = k1 - f1 * (k1 - k) / denominator;
        (k, f0, k1) = (k1, f1, next);
    }
    centre(best.0)
}


/// A support's signed distance from `x` and its unit gradient, where it has
/// one: a surface's own, a pipe's |x - spine| - radius. A cliff edge has none.
fn support_distance(s: &Support, x: Vec3) -> Option<(f64, Vec3)> {
    match s {
        Support::Surface(surface) => surface.distance(x),
        Support::Pipe { spine, radius, inner } => {
            let out = x.sub(pipe_axis_point(spine, inner, x)?);
            Some((out.length() - radius, out.normalized().ok()?))
        }
        // A ball resting on an edge has the edge's nearest point the radius
        // from its centre: the pipe of radius 0 about the edge.
        Support::Cliff(edge) => {
            let out = x.sub(nearest_on(edge, x)?.1);
            Some((out.length(), out.normalized().ok()?))
        }
    }
}

/// The ball's centre at spine parameter `t`. A blend on another blend or
/// over an edge has a spine the translator cannot refine against that
/// support (its intersection curve names a surface STEP has no word for), so
/// it is only as near as the file's chart: there the point is moved, in the
/// plane across the spine, to stand exactly `radius` from both supports
/// (Newton on the two distances; an edge's is to its nearest point).
fn ball_centre(supports: &[Support; 2], radius: f64, spine: &Curve, t: f64) -> Result<Vec3, String> {
    let mut centre = geometry::extended_point(spine, t)?;
    // Past an open spine's ends the extension is only the spine's tangent
    // line, not the ball's path: refined there as well.
    let beyond = matches!(spine, Curve::Nurbs(_))
        && geometry::closed_range(spine).is_none()
        && spine.natural_range().is_some_and(|[a, b]| t < a || t > b);
    if supports.iter().all(|s| matches!(s, Support::Surface(_))) && !beyond {
        return Ok(centre);
    }
    let Some(tangent) = geometry::curve_tangent(spine, t) else { return Ok(centre) };
    for _ in 0..8 {
        let (Some((d0, g0)), Some((d1, g1))) = (support_distance(&supports[0], centre), support_distance(&supports[1], centre)) else {
            return Ok(centre);
        };
        // The distance each should be: the radius, on the side it is on.
        let (f0, f1) = (d0 - radius * d0.signum(), d1 - radius * d1.signum());
        if f0.abs().max(f1.abs()) < 1e-10 * (1.0 + radius) {
            break;
        }
        let across = |g: Vec3| g.sub(tangent.scale(g.dot(tangent)));
        let (u, v) = (across(g0), across(g1));
        let (a, b, c, d) = (g0.dot(u), g0.dot(v), g1.dot(u), g1.dot(v));
        let det = a * d - b * c;
        if det.abs() < 1e-12 {
            break;
        }
        let alpha = (-f0 * d + f1 * b) / det;
        let beta = (-a * f1 + c * f0) / det;
        centre = centre.add(u.scale(alpha)).add(v.scale(beta));
    }
    Ok(centre)
}

pub(super) fn blend_section(supports: &[Support; 2], radius: f64, spine: &Curve, t: f64) -> Result<[crate::Vec4; 3], String> {
    let centre = ball_centre(supports, radius, spine, t)?;
    let foot = |s: &Support| -> Result<Vec3, String> {
        match s {
            Support::Surface(s) => {
                let (distance, gradient) = s.distance_near(centre, radius, false, 0.05 * radius + 1e-6).ok_or("a supporting surface has no distance there")?;
                if (distance.abs() - radius).abs() > 0.05 * radius + 1e-6 {
                    return Err(format!("the spine stands {:.3e} mm from a support, not the radius {radius:.3e} mm", distance.abs()));
                }
                Ok(centre.sub(gradient.scale(distance)))
            }
            Support::Pipe { spine: axis, radius: tube, inner } => {
                // The pipe's nearest point to the ball's centre: out from the
                // nearest point of its axis by its radius.
                let near = pipe_axis_point(axis, inner, centre).ok_or("the supporting blend's spine does not reach the section")?;
                let out = centre.sub(near);
                let distance = (out.length() - tube).abs();
                if (distance - radius).abs() > 0.05 * radius + 1e-6 {
                    return Err(format!("the spine stands {distance:.3e} mm from a supporting blend, not the radius {radius:.3e} mm"));
                }
                Ok(near.add(out.normalized()?.scale(*tube)))
            }
            Support::Cliff(edge) => {
                // Where the edge crosses the section's plane (through the
                // spine point, across the spine) the radius from it: the
                // ball touches the edge on its circle of contact.
                let tangent = geometry::curve_tangent(spine, t).ok_or("the spine has no tangent there")?;
                let miss = |q: Vec3| q.sub(centre).dot(tangent).hypot(q.sub(centre).length() - radius);
                let q = if let Curve::Line { point, direction } = edge {
                    // A line meets the plane at one point, wherever along it
                    // its stored point is (cross review: the search spans
                    // only a kilometre about it).
                    let along = direction.dot(tangent);
                    if along.abs() < 1e-12 {
                        return Err("the cliff edge runs along the section's plane".into());
                    }
                    point.add(direction.scale(centre.sub(*point).dot(tangent) / along))
                } else {
                    let k = geometry::nearest_parameter(edge, [true, true], miss).ok_or("the cliff edge does not reach the section")?;
                    geometry::extended_point(edge, k)?
                };
                // The spine is often a fitted intersection, its points that
                // far off; a surface support is held to 5% of the radius.
                if miss(q) > 0.01 * radius {
                    return Err(format!("the cliff edge passes {:.3e} mm off the section's circle", miss(q)));
                }
                Ok(q)
            }
        }
    };
    let (a, b) = (foot(&supports[0])?.sub(centre), foot(&supports[1])?.sub(centre));
    let (la, lb) = (a.length(), b.length());
    let cos = (a.dot(b) / (la * lb)).clamp(-1.0, 1.0);
    let angle = cos.acos();
    let at = |p: Vec3, w: f64| crate::Vec4 { x: p.x * w, y: p.y * w, z: p.z * w, w };
    if angle <= 1e-9 {
        // Where the supports touch tangentially the blend runs out to a
        // point (a spine with LIMITs, §5.2.2.6): the section collapses.
        let foot = centre.add(a);
        return Ok([at(foot, 1.0), at(foot, 1.0), at(foot, 1.0)]);
    }
    if angle >= 170f64.to_radians() {
        return Err(format!("a cross-section subtends {:.1} degrees", angle.to_degrees()));
    }
    let half = angle / 2.0;
    let bisector = a.scale(1.0 / la).add(b.scale(1.0 / lb)).normalized()?;
    let apex = centre.add(bisector.scale(0.5 * (la + lb) / half.cos()));
    let w = half.cos();
    Ok([at(centre.add(a), 1.0), at(apex, w), at(centre.add(b), 1.0)])
}

/// A degree (3, 2) B-surface through the cross-sections at spine stations
/// over [t0, t1], kept once it lies within [`FIT_TOLERANCE`] of the exact
/// sections halfway between its stations. It starts from 8 even stations and
/// adds a station halfway along each interval that misses: where a support
/// changes curvature (a swept section running on into its straight
/// extension) a cubic through the sections
/// closes on them slowly and unevenly as even stations double past the
/// change, but closes fast once stations gather there.
pub(super) fn fit_blend(section: impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64) -> Result<NurbsSurface, String> {
    fit_blend_split(&section, t0, t1, 0)
}

/// [`fit_blend`] over [t0, t1]. Where the closest fit misses by more than
/// [`FIT_REFUSAL`], the sections may have a kink: a support that is a
/// B-surface whose curvature jumps across a knot line moves the feet with a
/// jump in speed there, which a cubic through the sections only closes on
/// linearly. The kink is found (the largest jump between the sections' left
/// and right derivatives near the worst miss), each side fitted, and the two
/// joined C0 at the section they share. Fits that were accepted before are
/// accepted as before.
fn fit_blend_split(section: &impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64, depth: usize) -> Result<NurbsSurface, String> {
    let (surface, worst, worst_t) = fit_blend_once(section, t0, t1)?;
    if worst <= FIT_REFUSAL {
        return Ok(surface);
    }
    let refusal = || format!("no surface through its sections comes within {FIT_REFUSAL} mm of it (closest {worst:.2e} mm)");
    if depth >= 4 {
        return Err(refusal());
    }
    let Some(kink) = section_kink(section, t0, t1, worst_t) else { return Err(refusal()) };
    if !(kink > t0 + 1e-6 * (t1 - t0) && kink < t1 - 1e-6 * (t1 - t0)) {
        return Err(refusal());
    }
    let left = fit_blend_split(section, t0, kink, depth + 1)?;
    let right = fit_blend_split(section, kink, t1, depth + 1)?;
    join_c0_in_u(&left, &right).ok_or_else(refusal)
}

/// Where near `around` the sections' derivative jumps most: the jump between
/// one-sided differences, over narrowing windows. `None` where nothing jumps.
fn section_kink(section: &impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64, around: f64) -> Option<f64> {
    let point = |t: f64| -> Option<[Vec3; 3]> {
        let s = section(t).ok()?;
        Some([0, 1, 2].map(|j| Vec3::new(s[j].x / s[j].w, s[j].y / s[j].w, s[j].z / s[j].w)))
    };
    let jump = |t: f64, h: f64| -> f64 {
        let (Some(a), Some(b), Some(c)) = (point(t - h), point(t), point(t + h)) else { return 0.0 };
        (0..3).map(|j| c[j].sub(b[j]).sub(b[j].sub(a[j])).length() / h).fold(0.0, f64::max)
    };
    let (mut lo, mut hi) = ((around - 0.02 * (t1 - t0)).max(t0), (around + 0.02 * (t1 - t0)).min(t1));
    let mut best = around;
    for _ in 0..8 {
        // Stencils half a probe spacing wide each way tile the window, so a
        // kink anywhere in it falls inside one (cross review).
        let h = (hi - lo) / 128.0;
        best = (0..=64).map(|k| lo + (hi - lo) * k as f64 / 64.0).max_by(|a, b| jump(*a, h).total_cmp(&jump(*b, h)))?;
        let width = (hi - lo) / 16.0;
        (lo, hi) = ((best - width).max(t0), (best + width).min(t1));
    }
    (jump(best, 1e-9 * (t1 - t0)) > 0.0).then_some(best)
}

/// Two B-surfaces of the same degrees and v-knots, `left` ending where
/// `right` begins, as one: C0 across their shared row.
fn join_c0_in_u(left: &NurbsSurface, right: &NurbsSurface) -> Option<NurbsSurface> {
    if left.degree_u != right.degree_u || left.degree_v != right.degree_v || left.knots_v != right.knots_v {
        return None;
    }
    let p = left.degree_u;
    let mut knots = left.knots_u[..left.knots_u.len() - 1].to_vec();
    knots.extend_from_slice(&right.knots_u[p + 1..]);
    let mut net = left.control_points.clone();
    net.extend(right.control_points.iter().skip(1).cloned());
    NurbsSurface::new(p, left.degree_v, knots, left.knots_v.clone(), net).ok()
}

/// Where along [t0, t1] a blend's foot on a B-surface support crosses one of
/// its knot lines of multiplicity at least its degree less one (where the
/// surface is only C1, its curvature jumping): bisected on the foot's own
/// (u, v), from 256 stations. Sorted, inside the range.
pub(super) fn support_knot_crossings(supports: &[Support; 2], section: &impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for (j, support) in supports.iter().enumerate() {
        // A swept surface whose section is a B-curve has the section's knot
        // lines, swept: the foot's parameter is the section's at the foot,
        // measured across the sweep.
        if let Support::Surface(Surface::Swept { section: profile @ Curve::Nurbs(curve), direction }) = support {
            let Ok(along) = direction.normalized() else { continue };
            let lines = c1_lines(&curve.knots, curve.degree);
            if lines.is_empty() {
                continue;
            }
            let end = if j == 0 { 0 } else { 2 };
            let foot = |t: f64| -> Option<f64> {
                let s = section(t).ok()?;
                let p = Vec3::new(s[end].x / s[end].w, s[end].y / s[end].w, s[end].z / s[end].w);
                geometry::nearest_parameter(profile, [false, false], |q| {
                    let d = q.sub(p);
                    d.sub(along.scale(d.dot(along))).length()
                })
            };
            crossings_of(&foot, &lines, t0, t1, &mut out);
            continue;
        }
        let Support::Surface(Surface::Nurbs(surface)) = support else { continue };
        let lines_uv = [c1_lines(&surface.knots_u, surface.degree_u), c1_lines(&surface.knots_v, surface.degree_v)];
        if lines_uv.iter().all(|l| l.is_empty()) {
            continue;
        }
        // The section's end on this support: the first point for the first.
        let end = if j == 0 { 0 } else { 2 };
        let foot = |t: f64| -> Option<[f64; 2]> {
            let s = section(t).ok()?;
            let p = Vec3::new(s[end].x / s[end].w, s[end].y / s[end].w, s[end].z / s[end].w);
            let f = crate::project_point_to_surface(surface, p).ok()?;
            Some([f.u, f.v])
        };
        let stations = 256;
        let mut prev: Option<(f64, [f64; 2])> = None;
        for k in 0..=stations {
            let t = t0 + (t1 - t0) * k as f64 / stations as f64;
            let Some(here) = foot(t) else { prev = None; continue };
            if let Some((tp, there)) = prev {
                for axis in 0..2 {
                    for &line in &lines_uv[axis] {
                        // A station exactly on the line is the crossing (cross
                        // review: the strict test saw no sign change either side).
                        if (here[axis] - line).abs() <= 1e-12 * (1.0 + line.abs()) {
                            out.push(t);
                            continue;
                        }
                        if (there[axis] - line) * (here[axis] - line) < 0.0 {
                            let (mut a, mut b) = (tp, t);
                            for _ in 0..60 {
                                let m = 0.5 * (a + b);
                                match foot(m) {
                                    Some(f) if (there[axis] - line) * (f[axis] - line) > 0.0 => a = m,
                                    Some(_) => b = m,
                                    None => break,
                                }
                            }
                            out.push(0.5 * (a + b));
                        }
                    }
                }
            }
            prev = Some((t, here));
        }
    }
    let span = t1 - t0;
    out.retain(|&t| t > t0 + 1e-6 * span && t < t1 - 1e-6 * span);
    out.sort_by(|a, b| a.total_cmp(b));
    out.dedup_by(|a, b| (*a - *b).abs() <= 1e-9 * span);
    out
}

/// A knot vector's interior values of multiplicity at least `degree` less one
/// (the curve or surface only C1, or C0, across them). A file writes a
/// repeated knot as the same value: grouping by equality keeps two close
/// lines apart (cross review).
pub(super) fn c1_lines(knots: &[f64], degree: usize) -> Vec<f64> {
    let mut values: Vec<(f64, usize)> = Vec::new();
    for &k in knots {
        match values.last_mut() {
            Some((v, n)) if *v == k => *n += 1,
            _ => values.push((k, 1)),
        }
    }
    let (Some(&first), Some(&last)) = (knots.first(), knots.last()) else { return Vec::new() };
    values.into_iter().filter(|&(v, n)| n + 1 >= degree && n <= degree && v > first && v < last).map(|(v, _)| v).collect()
}

/// Where along [t0, t1] the one-parameter `foot` crosses any of `lines`: from
/// 256 stations, bisected; a station on a line (to 1e-12) is a crossing.
pub(super) fn crossings_of(foot: &impl Fn(f64) -> Option<f64>, lines: &[f64], t0: f64, t1: f64, out: &mut Vec<f64>) {
    let stations = 256;
    let mut prev: Option<(f64, f64)> = None;
    for k in 0..=stations {
        let t = t0 + (t1 - t0) * k as f64 / stations as f64;
        let Some(here) = foot(t) else { prev = None; continue };
        if let Some((tp, there)) = prev {
            for &line in lines {
                if (here - line).abs() <= 1e-12 * (1.0 + line.abs()) {
                    out.push(t);
                    continue;
                }
                if (there - line) * (here - line) < 0.0 {
                    let (mut a, mut b) = (tp, t);
                    for _ in 0..60 {
                        let m = 0.5 * (a + b);
                        match foot(m) {
                            Some(f) if (there - line) * (f - line) > 0.0 => a = m,
                            Some(_) => b = m,
                            None => break,
                        }
                    }
                    out.push(0.5 * (a + b));
                }
            }
        }
        prev = Some((t, here));
    }
}

/// A blend fitted between consecutive `breaks` (each piece as [`fit_blend`]
/// fits) and the pieces joined C0.
fn fit_blend_pieces(section: &impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64, breaks: &[f64]) -> Result<NurbsSurface, String> {
    let mut ends = vec![t0];
    ends.extend_from_slice(breaks);
    ends.push(t1);
    let mut joined: Option<NurbsSurface> = None;
    for pair in ends.windows(2) {
        let piece = fit_blend_split(section, pair[0], pair[1], 0)?;
        joined = Some(match joined {
            None => piece,
            Some(left) => join_c0_in_u(&left, &piece).ok_or("the pieces do not join")?,
        });
    }
    joined.ok_or_else(|| "the blend has no pieces".to_string())
}

/// One fit of [`fit_blend`]: the closest surface it found, its worst miss and
/// where that miss is.
fn fit_blend_once(section: &impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64) -> Result<(NurbsSurface, f64, f64), String> {
    let rational_point = |c: &[crate::Vec4; 3], v: f64| {
        let basis = [(1.0 - v) * (1.0 - v), 2.0 * v * (1.0 - v), v * v];
        let (mut x, mut y, mut z, mut w) = (0.0, 0.0, 0.0, 0.0);
        for (b, p) in basis.iter().zip(c) {
            (x, y, z, w) = (x + b * p.x, y + b * p.y, z + b * p.z, w + b * p.w);
        }
        Vec3::new(x / w, y / w, z / w)
    };
    const MAX_STATIONS: usize = 1024;
    let mut best: Option<(NurbsSurface, f64, f64)> = None;
    let mut params: Vec<f64> = (0..=8).map(|k| t0 + (t1 - t0) * k as f64 / 8.0).collect();
    let mut sections: Vec<[crate::Vec4; 3]> = params.iter().map(|&t| section(t)).collect::<Result<_, _>>()?;
    // Rounds in a row that did not halve the best miss.
    let mut stalled = 0;
    // The exact section halfway along each interval, where known.
    let mut kept: Vec<Option<[crate::Vec4; 3]>> = Vec::new();
    loop {
        let degree = (params.len() - 1).min(3);
        let mut columns = Vec::with_capacity(3);
        let mut knots = Vec::new();
        for j in 0..3 {
            let xyz: Vec<Vec3> = sections.iter().map(|s| Vec3::new(s[j].x, s[j].y, s[j].z)).collect();
            let weights: Vec<Vec3> = sections.iter().map(|s| Vec3::new(s[j].w, 0.0, 0.0)).collect();
            let xyz = crate::interpolate_curve(&xyz, degree, &params)?;
            let weights = crate::interpolate_curve(&weights, degree, &params)?;
            knots = xyz.knots.clone();
            let column: Vec<crate::Vec4> = xyz
                .control_points
                .iter()
                .zip(&weights.control_points)
                .map(|(p, w)| crate::Vec4 { x: p.x / p.w, y: p.y / p.w, z: p.z / p.w, w: w.x / w.w })
                .collect();
            columns.push(column);
        }
        let net: Vec<Vec<crate::Vec4>> = (0..columns[0].len()).map(|i| columns.iter().map(|c| c[i]).collect()).collect();
        let surface = NurbsSurface::new(degree, 2, knots, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], net)?;
        // The miss halfway along each interval, with the exact section there
        // (kept from the last round for an interval that was not split).
        let mut misses = Vec::with_capacity(params.len() - 1);
        for (k, pair) in params.windows(2).enumerate() {
            let t = 0.5 * (pair[0] + pair[1]);
            let exact = match kept.get(k) {
                Some(&Some(exact)) => exact,
                _ => section(t)?,
            };
            let mut miss = 0.0f64;
            for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
                miss = miss.max(surface.evaluate(t, v)?.sub(rational_point(&exact, v)).length());
            }
            misses.push((t, exact, miss));
        }
        let (worst_t, worst) = misses.iter().fold((t0, 0.0f64), |(wt, w), m| if m.2 > w { (m.0, m.2) } else { (wt, w) });
        if worst <= FIT_TOLERANCE {
            return Ok((surface, worst, worst_t));
        }
        // A cubic through exact sections closes on them at least 4x per
        // halving of the stations where it misses; a fit that does not even
        // halve its miss for three rounds running is not converging (a
        // section the spine or supports cannot give smoothly), so stop paying.
        stalled = if best.as_ref().is_some_and(|(_, b, _)| worst > 0.5 * b) { stalled + 1 } else { 0 };
        if best.as_ref().map_or(true, |(_, b, _)| worst < *b) {
            best = Some((surface, worst, worst_t));
        }
        let missing = misses.iter().filter(|m| m.2 > FIT_TOLERANCE).count();
        if stalled >= 3 || params.len() + missing > MAX_STATIONS + 1 {
            break;
        }
        let (mut next_params, mut next_sections) = (Vec::with_capacity(params.len() + missing), Vec::with_capacity(params.len() + missing));
        let mut next_kept = Vec::with_capacity(params.len() + missing);
        for (k, (t, exact, miss)) in misses.into_iter().enumerate() {
            next_params.push(params[k]);
            next_sections.push(sections[k]);
            if miss > FIT_TOLERANCE {
                next_params.push(t);
                next_sections.push(exact);
                next_kept.extend([None, None]);
            } else {
                next_kept.push(Some(exact));
            }
        }
        kept = next_kept;
        next_params.push(params[params.len() - 1]);
        next_sections.push(sections[sections.len() - 1]);
        (params, sections) = (next_params, next_sections);
    }
    best.ok_or_else(|| "the blend has no sections".to_string())
}

/// A cubic through `points` by chord length (a straight segment for two).
fn fit_through(points: &[Vec3]) -> Result<NurbsCurve, String> {
    let mut stations = Vec::with_capacity(points.len());
    for p in points {
        if stations.last().map_or(true, |q: &Vec3| p.sub(*q).length() > 1e-12) {
            stations.push(*p);
        }
    }
    if stations.len() < 2 {
        return Err("a fitted curve has fewer than two distinct points".into());
    }
    let mut params = vec![0.0];
    for w in stations.windows(2) {
        params.push(params[params.len() - 1] + w[1].sub(w[0]).length());
    }
    let degree = (stations.len() - 1).min(3);
    crate::interpolate_curve(&stations, degree, &params)
}

/// Distinct knots and their multiplicities as Part 21 lists.
fn knot_runs(knots: &[f64]) -> (String, String) {
    let mut mults = Vec::new();
    let mut values = Vec::new();
    for &k in knots {
        if values.last() == Some(&k) {
            *mults.last_mut().expect("paired with values") += 1;
        } else {
            values.push(k);
            mults.push(1usize);
        }
    }
    (
        mults.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","),
        values.iter().map(|v| real(*v)).collect::<Vec<_>>().join(","),
    )
}
