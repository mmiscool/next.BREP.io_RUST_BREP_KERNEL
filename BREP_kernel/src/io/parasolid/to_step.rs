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
    /// Blend spines, fitted once each, or the reason one cannot be: a
    /// blend's boundary edges ask for its spine as well as its face, and a
    /// spine that does not fit is costly to fail again.
    spines: HashMap<u32, Result<Curve, String>>,
    /// Read an SP-curve on a blend through the blend's own parameters: only
    /// once no fin of the edge reads otherwise, the other face's statement
    /// of the edge being the better one where it has one.
    read_blend_sp: bool,
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
        spines: HashMap::default(),
        read_blend_sp: false,
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
        (0..=16).filter_map(|k| curve.point(range[0] + (range[1] - range[0]) * k as f64 / 16.0).ok()).collect()
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
            let inner = inner.iter().any(|s| !matches!(s, Support::Surface(_))).then(|| Box::new((inner, tube)));
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
        let [t0, t1] = spine_range(&spine, &points, 1.0 / 50.0).ok_or_else(|| format!("blend {id}: the face does not lie along its spine"))?;
        let section = |t: f64| blend_section(&supports, radius, &spine, t);
        // A blend whose supports are only C1 (or C0) along some knot lines (a
        // B-surface's, or a swept B-curve's) has sections that kink where a
        // foot crosses one: fitted piece by piece between those crossings,
        // found from the supports' own knots rather than from the sections.
        let pieces = |t0: f64, t1: f64| -> Option<NurbsSurface> {
            let breaks = support_knot_crossings(&supports, &section, t0, t1);
            fit_blend_pieces(&section, t0, t1, &breaks).ok()
        };
        let surface = match fit_blend(&section, t0, t1) {
            Ok(surface) => surface,
            Err(e) => match pieces(t0, t1) {
                Some(surface) => surface,
                // The fiftieth past the face each way can run past where a
                // support is (a swept section's end): the face's own span.
                None => {
                    let [a, b] = spine_range(&spine, &points, 0.0).ok_or_else(|| format!("blend {id}: {e}"))?;
                    pieces(a, b).ok_or_else(|| format!("blend {id}: {e}"))?
                }
            },
        };
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
            _ => return Err("a face with no loop is not a whole sphere or torus, which is not read yet".into()),
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
        let vertex = self.node(index, "vertex")?;
        let point = self.file.follow(vertex, "point").ok_or_else(|| format!("vertex {index} has no point"))?;
        point.vector("pvec").map(geometry::position).ok_or_else(|| format!("vertex {index}'s point has no position"))
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

    /// (STEP curve, whether the edge runs along it, a point to close a ring
    /// edge at).
    fn edge_curve(&mut self, edge: &Node, positive: &Node) -> Result<(usize, bool, Option<Vec3>), String> {
        if let Some(curve) = self.file.follow(edge, "curve") {
            let sense = runs_forward(self.file, curve);
            self.anchor = self.edge_ends.map(|(start, end)| if sense { (start, end) } else { (end, start) });
            let (id, closing) = match self.curve(curve) {
                Ok(read) => read,
                // An edge curve that is an SP-curve on a blend: through the
                // blend, as for a fin's (it has no other statement).
                Err(e) if !self.read_blend_sp => {
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
                if let Some((begin, finish)) = self.anchor.take() {
                    chart = reseat_closed_chart(chart, begin, finish);
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
                let crossing = self.intersection_crossing(node);
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
                // from 0 at the first support (the second when the spine runs
                // backwards) to 1 at the other, proportional to the angle.
                let blend = self.file.follow(node, "surface").ok_or("an SP-curve has no surface")?;
                let blend_id = blend.int("node_id").unwrap_or(blend.index as i64);
                let (mut supports, radius) = self.blend_supports(blend, 0)?;
                let spine_node = self.file.follow(blend, "spine").ok_or_else(|| format!("blend {blend_id} has no spine"))?;
                if spine_node.char("sense") == Some(b'-') {
                    supports.swap(0, 1);
                }
                let spine = self.spine_curve(spine_node).map_err(|e| format!("blend {blend_id}: spine: {e}"))?;
                let uv = geometry::parameter_curve(self.file, self.file.follow(node, "b_curve").ok_or("an SP-curve has no 2D curve")?)?;
                // The same rational quadratic section the face's surface is
                // fitted through, at the parameter where the angle from its
                // start is v of the whole (cross review: the nominal radius
                // put an edge off its carrier where the feet stand off it, and
                // a section collapsed where the supports meet had no frame).
                // On a circular rational quadratic of half-angle α the angle
                // φ from its start has tan((φ - α)/2) = tan(α/2)·(2w - 1).
                let point = |u: f64, v: f64| -> Result<Vec3, String> {
                    let centre = ball_centre(&supports, radius, &spine, u)?;
                    Ok(section_at_angle(&blend_section(&supports, radius, &spine, u)?, centre, v))
                };
                let [t0, t1] = uv.domain()?;
                let open = uv.evaluate(t0)?.sub(uv.evaluate(t1)?).length() > 1e-9;
                let range = trim
                    .filter(|_| open)
                    .map(|[a, b]| [a.max(t0), b.min(t1)])
                    .filter(|[a, b]| b - a > 1e-9 * (t1 - t0))
                    .unwrap_or([t0, t1]);
                let trimmed = range != [t0, t1];
                let fitted = fit_sp_curve_by(&point, &uv, range)
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
                let fitted = fit_sp_curve(&surface, &uv, range)
                    .map_err(|e| format!("SP-curve {}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                // A trimmed SP-curve ends at its trim parameters, which a
                // tolerant edge's vertices stand off by up to their
                // tolerance: move its ends onto them, so the edge runs
                // exactly vertex to vertex and nothing is left to trim.
                let fitted = match self.edge_ends {
                    Some(ends) if trimmed => snap_ends(fitted, ends),
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
    // A point's position along the chart (chord index plus the fraction
    // along it) and its distance from it.
    let place = |p: Vec3| -> (f64, f64) {
        let mut best = (f64::INFINITY, 0.0);
        for i in 0..n - 1 {
            let (a, b) = (chart[i], chart[i + 1]);
            let ab = b.sub(a);
            let t = (p.sub(a).dot(ab) / ab.dot(ab).max(1e-300)).clamp(0.0, 1.0);
            let d = p.sub(a.add(ab.scale(t))).length();
            if d < best.0 {
                best = (d, i as f64 + t);
            }
        }
        best
    };
    let ((db, sb), (df, sf)) = (place(begin), place(finish));
    if db > 1.0 || df > 1.0 || (sb - sf).abs() < 1e-12 {
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
}

impl<'a> ChartStations<'a> {
    fn new(crossing: Option<&'a Crossing>, chart: &'a [Vec3]) -> Self {
        let memo = crate::step_import::import_lean().then(|| std::cell::RefCell::new(HashMap::default()));
        Self { crossing, chart, memo }
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
            (Err(_), Some(Crossing::Contact { .. })) => Ok((fit(&ChartStations { crossing: None, chart, memo: None })?, true)),
            (result, _) => Ok((result?, false)),
        }
    }
}


/// An SP-curve over [t0, t1] fitted knot span by knot span
/// ([`fit_by_pieces`]).
fn fit_sp_curve(surface: &Surface, uv: &NurbsCurve, range: [f64; 2]) -> Result<NurbsCurve, String> {
    fit_sp_curve_by(&|u, v| surface.point(u, v), uv, range)
}

/// [`fit_sp_curve`] on a surface given by its point function.
fn fit_sp_curve_by(point: &impl Fn(f64, f64) -> Result<Vec3, String>, uv: &NurbsCurve, [t0, t1]: [f64; 2]) -> Result<NurbsCurve, String> {
    let mut breaks = vec![t0];
    breaks.extend(uv.knots.iter().copied().filter(|&k| k > t0 && k < t1));
    breaks.push(t1);
    breaks.dedup();
    let at = |t: f64| -> Result<Vec3, String> {
        let p = uv.evaluate(t)?;
        point(p.x, p.y)
    };
    fit_by_pieces(
        breaks.len() - 1,
        |i, s| (0..s).map(|k| at(breaks[i] + (breaks[i + 1] - breaks[i]) * k as f64 / s as f64)).collect(),
        at(t1)?,
        8,
    )
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

/// `curve` with its end control points moved onto the edge's vertices (the
/// nearer to each end), where both stand within a tenth of a millimetre of
/// the ends they replace; otherwise unchanged. A clamped B-spline's ends
/// are its end control points, so only the spans at its ends move, by the
/// gap.
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
    if first.sub(a).length() > 0.1 || last.sub(b).length() > 0.1 {
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

/// The parameter range of `spine` that the points lie along, widened by a
/// fiftieth each way (within a bounded curve's own range).
pub(super) fn spine_range(spine: &Curve, points: &[Vec3], margin: f64) -> Option<[f64; 2]> {
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
            feet
        }
    };
    let (lo, hi) = params.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
    if !(hi > lo) {
        return None;
    }
    let margin = (hi - lo) * margin;
    let (mut lo, mut hi) = (lo - margin, hi + margin);
    if let Curve::Nurbs(curve) = spine {
        let [a, b] = curve.domain().ok()?;
        (lo, hi) = (lo.max(a), hi.min(b));
    }
    Some([lo, hi])
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
    for _ in 0..12 {
        let c = centre(k)?;
        let slope = centre(k + h)?.sub(centre(k - h)?).scale(0.5 / h);
        let speed = slope.dot(slope);
        if !(speed > 0.0) {
            return Some(c);
        }
        let step = x.sub(c).dot(slope) / speed;
        k += step;
        if step.abs() <= 1e-13 * (1.0 + k.abs()) {
            break;
        }
    }
    centre(k)
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
    if supports.iter().all(|s| matches!(s, Support::Surface(_))) {
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
                let (distance, gradient) = s.distance(centre).ok_or("a supporting surface has no distance there")?;
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
