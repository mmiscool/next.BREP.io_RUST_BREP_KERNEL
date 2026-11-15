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
fn runs_forward(file: &XtFile, node: &Node) -> bool {
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
fn chain<'f>(file: &'f XtFile, first: u32, next: &'static str) -> impl Iterator<Item = &'f Node> + 'f {
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
    anchor: Option<(Vec3, Vec3)>,
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
        anchor: None,
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
                for region in &regions {
                    for shell in chain(self.file, region.ptr("shell"), "next") {
                        // A sheet's faces have the same void region on both
                        // sides: each appears on the shell's back or front
                        // chain (or both); take each once.
                        let mut seen = rustc_hash::FxHashSet::default();
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
            2 => return Err("the body is a wire body, which is not read yet".into()),
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
                let angle = |p: Vec3| {
                    let d = p.sub(frame.origin);
                    d.dot(frame.y()).atan2(d.dot(frame.x))
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
        if node.char("blend_type") != Some(b'R') {
            return Err(format!("blend {id} is a cliff-edge blend, which is not read yet"));
        }
        let read = |s: &Node| -> Result<Surface, String> {
            let surface = geometry::surface(file, s, 0)?;
            if surface.has_distance() {
                Ok(surface)
            } else {
                Err(format!("blend {id} is supported by a {}, which is not read for blends yet", s.type_name()))
            }
        };
        let supports: Vec<&Node> = node.ints("surface").into_iter().filter_map(|i| u32::try_from(i).ok()).filter_map(|i| file.node(i)).collect();
        let [first, second]: [&Node; 2] = supports.try_into().map_err(|_| format!("blend {id} does not have two supporting surfaces"))?;
        if first.type_name() == "BLENDED_EDGE" || second.type_name() == "BLENDED_EDGE" {
            return Err(format!("blend {id} is supported by another blend, which is not read yet"));
        }
        let mut supports = [read(first)?, read(second)?];
        let radius = node.reals("range").iter().map(|r| r.abs()).fold(0.0f64, f64::max) * MM_PER_METRE;
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(format!("blend {id} has no radius"));
        }
        let spine_node = file.follow(node, "spine").ok_or_else(|| format!("blend {id} has no spine"))?;
        if spine_node.char("sense") == Some(b'-') {
            supports.swap(0, 1);
        }
        let spine = self.spine_curve(spine_node).map_err(|e| format!("blend {id}: spine: {e}"))?;
        let [t0, t1] = spine_range(&spine, &self.face_boundary_points(face)).ok_or_else(|| format!("blend {id}: the face does not lie along its spine"))?;
        let section = |t: f64| blend_section(&supports, radius, &spine, t);
        let surface = fit_blend(section, t0, t1).map_err(|e| format!("blend {id}: {e}"))?;
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
                let away = surface.evaluate(t, 0.5).ok()?.sub(spine.point(t).ok()?);
                Some(surface.normal(t, 0.5).ok()?.dot(away) < 0.0)
            })
            .ok_or_else(|| format!("blend {id}: no regular point to orient it by"))?;
        self.emitted.push(EmittedSurface { id: step, flipped });
        Ok(self.emitted.len() - 1)
    }

    /// A blend's spine as a curve the translator can evaluate: the exact
    /// curve, or an intersection curve fitted as an edge's is.
    fn spine_curve(&mut self, node: &Node) -> Result<Curve, String> {
        let mut node = node;
        for _ in 0..8 {
            if node.type_name() != "TRIMMED_CURVE" {
                break;
            }
            node = self.file.follow(node, "basis_curve").ok_or("a trimmed curve has no basis curve")?;
        }
        if node.type_name() == "INTERSECTION" {
            let chart = self.intersection_points(node)?;
            let surfaces = self.intersection_surfaces(node);
            if refined_intersection(surfaces.as_ref(), &chart, 2).len() == chart.len() {
                self.unverified_curves += 1;
            }
            return Ok(Curve::Nurbs(fit_adaptively(|splits| Ok(refined_intersection(surfaces.as_ref(), &chart, splits)), 8)?));
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
        let written = self.edge_curve(edge, positive);
        (self.edge_ends, self.anchor) = (None, None);
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
            let (id, closing) = self.curve(curve)?;
            return Ok((id, sense, closing));
        }
        // A tolerant edge: no edge curve; each fin carries a trimmed
        // SP-curve (§4.3.8). Use the first fin that has one.
        let mut at = positive.index;
        for _ in 0..64 {
            let fin = self.node(at, "fin")?;
            if let Some(curve) = self.file.follow(fin, "curve") {
                // The fin's curve runs with the fin; the edge with the fin's sense.
                let fin_sense = fin.char("sense") != Some(b'-');
                let curve_sense = runs_forward(self.file, curve);
                self.anchor = self.edge_ends.map(|(start, end)| if fin_sense == curve_sense { (start, end) } else { (end, start) });
                let (id, closing) = self.curve(curve)?;
                return Ok((id, fin_sense == curve_sense, closing));
            }
            at = fin.ptr("other");
            if at == positive.index || at == 0 {
                break;
            }
        }
        Err("the edge has no curve and none of its fins has one".into())
    }

    // -----------------------------------------------------------------
    // Curves
    // -----------------------------------------------------------------

    /// The STEP curve of an XT curve node and a point on it to close a ring
    /// edge at (its parameter start).
    fn curve(&mut self, node: &Node) -> Result<(usize, Option<Vec3>), String> {
        self.curve_at(node, 0)
    }

    fn curve_at(&mut self, node: &Node, depth: usize) -> Result<(usize, Option<Vec3>), String> {
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
                let (id, basis_closing) = self.curve_at(basis, depth + 1)?;
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
                }
                let surfaces = self.intersection_surfaces(node);
                // With nothing solved between chart points the densities all
                // give the chart, and the fit's check proves nothing: count
                // it as unverified rather than measured.
                if refined_intersection(surfaces.as_ref(), &chart, 2).len() == chart.len() {
                    self.unverified_curves += 1;
                }
                let fitted = fit_adaptively(|splits| Ok(refined_intersection(surfaces.as_ref(), &chart, splits)), 8)
                    .map_err(|e| format!("intersection curve {}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                self.fitted_curves += 1;
                Ok((self.emit_curve(&Curve::Nurbs(fitted))?, chart.first().copied()))
            }
            "SP_CURVE" => {
                let (surface, uv) = self.sp_curve_parts(node)?;
                if let Some(exact) = exact_sp_curve(&surface, &uv) {
                    let closing = sp_curve_points(&surface, &uv, 1)?.first().copied();
                    return Ok((self.emit_curve(&exact)?, closing));
                }
                let fitted = fit_adaptively(|per_span| sp_curve_points(&surface, &uv, per_span), 8)
                    .map_err(|e| format!("SP-curve {}: {e}", node.int("node_id").unwrap_or(node.index as i64)))?;
                self.fitted_curves += 1;
                let closing = sp_curve_points(&surface, &uv, 1)?.first().copied();
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

    /// The two surfaces of an intersection curve, when the reader has a
    /// distance for both.
    fn intersection_surfaces(&self, node: &Node) -> Option<[Surface; 2]> {
        let surfaces: Vec<Surface> = node
            .ints("surface")
            .into_iter()
            .filter_map(|i| u32::try_from(i).ok())
            .filter_map(|i| self.file.node(i))
            .filter_map(|s| geometry::surface(self.file, s, 0).ok())
            .collect();
        let [first, second]: [Surface; 2] = surfaces.try_into().ok()?;
        (first.has_distance() && second.has_distance()).then_some([first, second])
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
        let (id, flipped) = self.emit_surface(&surface)?;
        self.emitted.push(EmittedSurface { id, flipped });
        let i = self.emitted.len() - 1;
        self.surfaces.insert(node.index, i);
        Ok(i)
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
                    return Err("a spun surface whose profile meets its axis is not read yet".into());
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

/// A cubic through `stations(d)` at doubling densities d from `start`, kept
/// once every station of `stations(2d)` (the same stations and those between
/// them) lies within [`FIT_TOLERANCE`] of it. The densest fit is used if none
/// gets there, unless it misses by more than [`FIT_REFUSAL`].
pub(super) fn fit_adaptively(stations: impl Fn(usize) -> Result<Vec<Vec3>, String>, start: usize) -> Result<NurbsCurve, String> {
    let mut density = start;
    let mut best: Option<(NurbsCurve, f64)> = None;
    for _ in 0..=FIT_DOUBLINGS {
        let fit = fit_through(&stations(density)?)?;
        let check = stations(density * 2)?;
        let worst = check
            .iter()
            .map(|p| crate::project_point_to_curve(&fit, *p).map(|f| f.distance).unwrap_or(f64::INFINITY))
            .fold(0.0f64, f64::max);
        if worst <= FIT_TOLERANCE {
            return Ok(fit);
        }
        if best.as_ref().map_or(true, |(_, b)| worst < *b) {
            best = Some((fit, worst));
        }
        // The next check would sample twice as many points again.
        if check.len() * 2 > MAX_FIT_STATIONS {
            break;
        }
        density *= 2;
    }
    match best {
        Some((fit, worst)) if worst <= FIT_REFUSAL => Ok(fit),
        Some((_, worst)) => Err(format!("no cubic through its points comes within {FIT_REFUSAL} mm of it (closest {worst:.2e} mm)")),
        None => Err("the curve has no points".into()),
    }
}

/// The chart with `splits - 1` exact points inside each chord: each solved
/// onto both surfaces and the plane through it normal to the chord, which is
/// how the reference defines the curve between chart points (§5.2.1.5).
/// Without both surfaces' distances (blend boundaries, spun and swept
/// surfaces), or where a point does not converge (a tangency), the chart
/// alone stands there.
fn refined_intersection(surfaces: Option<&[Surface; 2]>, chart: &[Vec3], splits: usize) -> Vec<Vec3> {
    let Some([first, second]) = surfaces else {
        return chart.to_vec();
    };
    let mut out = Vec::with_capacity(chart.len() * splits);
    for pair in chart.windows(2) {
        out.push(pair[0]);
        let chord = pair[1].sub(pair[0]);
        let Ok(direction) = chord.normalized() else { continue };
        for k in 1..splits {
            let station = pair[0].add(chord.scale(k as f64 / splits as f64));
            // A point further from its station than the chord is long has
            // left the branch the chart traces.
            if let Some(p) = solve_on_both(first, second, station, direction, chord.length()) {
                out.push(p);
            }
        }
    }
    out.push(chart[chart.len() - 1]);
    out
}

/// Points of an SP-curve, `per_span` to each knot span of its 2D curve,
/// mapped through its surface.
fn sp_curve_points(surface: &Surface, uv: &NurbsCurve, per_span: usize) -> Result<Vec<Vec3>, String> {
    let [t0, t1] = uv.domain()?;
    let mut breaks: Vec<f64> = uv.knots.clone();
    breaks.dedup();
    let mut params = Vec::new();
    for pair in breaks.windows(2).filter(|w| w[1] > w[0] && w[0] >= t0 && w[1] <= t1) {
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

/// An edge's positive and negative fins. Parasolid points an edge at its
/// positive fin, but not every writer does (a V12 file seen points 23 of
/// its 54 edges at the negative one), so the fins' own senses decide.
fn edge_fins<'f>(file: &'f XtFile, edge: &Node) -> Result<(&'f Node, &'f Node), &'static str> {
    let first = file.follow(edge, "halfedge").ok_or("has no fin")?;
    let other = file.follow(first, "other").ok_or("has one fin")?;
    Ok(if first.char("sense") == Some(b'-') && other.char("sense") != Some(b'-') { (other, first) } else { (first, other) })
}

/// The parameter range of `spine` that the points lie along, widened by a
/// fiftieth each way (within a bounded curve's own range).
fn spine_range(spine: &Curve, points: &[Vec3]) -> Option<[f64; 2]> {
    let params: Vec<f64> = match spine {
        Curve::Line { point, direction } => points.iter().map(|p| p.sub(*point).dot(*direction) / MM_PER_METRE).collect(),
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => {
            let angle = |p: &Vec3| {
                let d = p.sub(frame.origin);
                d.dot(frame.y()).atan2(d.dot(frame.x))
            };
            // The face spans the circle less its largest gap between
            // boundary points; a face whose points leave no gap wider than
            // 45° (a ring all the way round, sampled) takes the whole circle,
            // which over-covers a face that is not quite whole, harmlessly.
            let mut angles: Vec<f64> = points.iter().map(|p| angle(p).rem_euclid(std::f64::consts::TAU)).collect();
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
        Curve::Nurbs(curve) => points.iter().filter_map(|p| crate::project_point_to_curve(curve, *p).ok().map(|f| f.u)).collect(),
    };
    let (lo, hi) = params.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
    if !(hi > lo) {
        return None;
    }
    let margin = (hi - lo) / 50.0;
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
fn blend_section(supports: &[Surface; 2], radius: f64, spine: &Curve, t: f64) -> Result<[crate::Vec4; 3], String> {
    let centre = spine.point(t)?;
    let foot = |s: &Surface| -> Result<Vec3, String> {
        let (distance, gradient) = s.distance(centre).ok_or("a supporting surface has no distance there")?;
        if (distance.abs() - radius).abs() > 0.05 * radius + 1e-6 {
            return Err(format!("the spine stands {:.3e} mm from a support, not the radius {radius:.3e} mm", distance.abs()));
        }
        Ok(centre.sub(gradient.scale(distance)))
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

/// A degree (3, 2) B-surface through the cross-sections at doubling numbers
/// of spine stations over [t0, t1], kept once it lies within
/// [`FIT_TOLERANCE`] of the exact sections halfway between its stations.
fn fit_blend(section: impl Fn(f64) -> Result<[crate::Vec4; 3], String>, t0: f64, t1: f64) -> Result<NurbsSurface, String> {
    let rational_point = |c: &[crate::Vec4; 3], v: f64| {
        let basis = [(1.0 - v) * (1.0 - v), 2.0 * v * (1.0 - v), v * v];
        let (mut x, mut y, mut z, mut w) = (0.0, 0.0, 0.0, 0.0);
        for (b, p) in basis.iter().zip(c) {
            (x, y, z, w) = (x + b * p.x, y + b * p.y, z + b * p.z, w + b * p.w);
        }
        Vec3::new(x / w, y / w, z / w)
    };
    let mut best: Option<(NurbsSurface, f64)> = None;
    let mut stations = 8usize;
    while stations <= 512 {
        let params: Vec<f64> = (0..=stations).map(|k| t0 + (t1 - t0) * k as f64 / stations as f64).collect();
        let sections: Vec<[crate::Vec4; 3]> = params.iter().map(|&t| section(t)).collect::<Result<_, _>>()?;
        let degree = stations.min(3);
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
        let mut worst = 0.0f64;
        for pair in params.windows(2) {
            let t = 0.5 * (pair[0] + pair[1]);
            let exact = section(t)?;
            for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
                worst = worst.max(surface.evaluate(t, v)?.sub(rational_point(&exact, v)).length());
            }
        }
        if worst <= FIT_TOLERANCE {
            return Ok(surface);
        }
        // A cubic through exact sections closes on them ~16x per doubling;
        // one that does not halve its miss is not converging (a section
        // the spine or supports cannot give smoothly), so stop paying.
        let stalled = best.as_ref().is_some_and(|(_, b)| worst > 0.5 * b);
        if best.as_ref().map_or(true, |(_, b)| worst < *b) {
            best = Some((surface, worst));
        }
        if stalled {
            break;
        }
        stations *= 2;
    }
    match best {
        Some((surface, worst)) if worst <= FIT_REFUSAL => Ok(surface),
        Some((_, worst)) => Err(format!("no surface through its sections comes within {FIT_REFUSAL} mm of it (closest {worst:.2e} mm)")),
        None => Err("the blend has no sections".into()),
    }
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
