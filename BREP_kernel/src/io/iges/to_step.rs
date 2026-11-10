//! IGES to Part 21 translation: the front-end that puts IGES on the STEP lane.
//!
//! `step_import`'s builder reads STEP keywords directly, so the way to give an
//! IGES file the same closure gate, refusals, seam passes and trim repair is to
//! say the file again in STEP and import that. Everything here is bookkeeping
//! between the two vocabularies; no geometry is approximated. 124 transforms and
//! the Global unit scale are baked into the coordinates, because the STEP text
//! carries no unit context and is read as millimetres.
//!
//! `BREP_DEBUG_IGES_STEP=<path>` writes each solid's translated text to
//! `<path>.<DE pointer>`, so any import can be read as the STEP it became.

use rustc_hash::FxHashMap as HashMap;

use super::reader::{IgesFile, ParsedEntity};
use crate::Vec3;

mod face_sets;
mod splines;
pub(super) use face_sets::{analyse_face_sets, FaceSets};

const MAX_TRANSFORM_DEPTH: usize = 32;
/// Relative slack for "this 124 is a rigid motion with one uniform scale".
const RIGID_TOLERANCE: f64 = 1e-6;

/// A shortest-round-trip real. `step::real` writes fixed decimals, which would
/// cost a translated file precision on small values.
fn real(value: f64) -> String {
    let text = format!("{value:E}");
    if text.contains('.') {
        text
    } else {
        text.replacen('E', ".E", 1)
    }
}

#[derive(Clone, Copy, Debug)]
struct Transform {
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
}

impl Transform {
    const IDENTITY: Self = Self {
        rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation: [0.0; 3],
    };

    fn vector(&self, v: [f64; 3]) -> [f64; 3] {
        let mut out = [0.0; 3];
        for (row, value) in self.rotation.iter().zip(out.iter_mut()) {
            *value = row[0] * v[0] + row[1] * v[1] + row[2] * v[2];
        }
        out
    }

    fn point(&self, p: [f64; 3]) -> [f64; 3] {
        let v = self.vector(p);
        [
            v[0] + self.translation[0],
            v[1] + self.translation[1],
            v[2] + self.translation[2],
        ]
    }

    /// `self` applied after `inner`.
    fn after(&self, inner: &Self) -> Self {
        let mut rotation = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                rotation[i][j] = (0..3).map(|k| self.rotation[i][k] * inner.rotation[k][j]).sum();
            }
        }
        Self {
            rotation,
            translation: self.point(inner.translation),
        }
    }

    fn determinant(&self) -> f64 {
        let r = &self.rotation;
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    }

    fn reflects(&self) -> bool {
        self.determinant() < 0.0
    }

    /// The transform's scale when it is a rigid motion times one uniform
    /// scale. Anything else would turn a circle into an ellipse, which the
    /// analytic entities cannot say.
    fn uniform_scale(&self) -> Option<f64> {
        let columns = [0, 1, 2].map(|j| [self.rotation[0][j], self.rotation[1][j], self.rotation[2][j]]);
        let dot = |a: &[f64; 3], b: &[f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let scale = dot(&columns[0], &columns[0]).sqrt();
        if scale <= 0.0 {
            return None;
        }
        let bar = RIGID_TOLERANCE * scale * scale;
        for j in 0..3 {
            if (dot(&columns[j], &columns[j]) - scale * scale).abs() > bar {
                return None;
            }
            if dot(&columns[j], &columns[(j + 1) % 3]).abs() > bar {
                return None;
            }
        }
        Some(scale)
    }
}

/// `point` turned by `angle` about the line through `origin` along unit `axis`.
fn rotated_about(point: Vec3, origin: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let d = point.sub(origin);
    let along = axis.scale(d.dot(axis));
    let radial = d.sub(along);
    origin.add(along).add(radial.scale(angle.cos())).add(axis.cross(radial).scale(angle.sin()))
}

fn vec3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// Any unit vector perpendicular to `n`.
fn perpendicular(n: Vec3) -> Result<Vec3, String> {
    let helper = if n.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    n.cross(helper).normalized()
}

/// A translated curve and the points it runs between, for choosing the sense
/// of the edge that uses it.
struct EmittedCurve {
    id: usize,
    start: Vec3,
    end: Vec3,
    /// Every control point is within the file's resolution of the first: a
    /// curve of no length by the file's own statement.
    collapsed: bool,
}

/// A translated surface. `reversed` says the STEP entity's normal is opposite
/// the IGES surface's, so the face sense has to change with it.
struct EmittedSurface {
    id: usize,
    reversed: bool,
    /// Where this surface's true seam is, when the entity says.
    seam: Option<SeamRule>,
    /// Where a B-spline's boundaries collapse to one point; `None` when the
    /// surface cannot be judged that way.
    poles: Option<Vec<Vec3>>,
    /// How to read a point in this STEP carrier's own parameters, for telling
    /// which way a boundary winds about the carrier's normal.
    chart: Option<Chart>,
    /// A chart v inside the part of the carrier the file's surface covers, when
    /// that surface is bounded along v.
    domain_v: Option<f64>,
    /// A point the file's surface covers, on a sphere, where a boundary's side
    /// can be read directly around it.
    interior: Option<Vec3>,
    /// A B-spline carrier as evaluated in model millimetres, its (u, v) the
    /// STEP entity's, so a boundary given only in model space can be read in
    /// the carrier's own parameters.
    spline: Option<std::rc::Rc<crate::NurbsSurface>>,
}

/// The carrier's (u, v) with du x dv along its STEP normal.
#[derive(Clone, Copy)]
enum Chart {
    Plane { origin: Vec3, axis: Vec3, x: Vec3 },
    /// Cylinder or cone: angle about the axis, distance along it.
    Axial { origin: Vec3, axis: Vec3, x: Vec3 },
    Sphere { origin: Vec3, axis: Vec3, x: Vec3 },
    Torus { origin: Vec3, axis: Vec3, x: Vec3, major: f64 },
    /// A B-spline, read through the file's own parameter-space curves.
    Parameter,
}

impl Chart {
    /// Whether u and v wrap, in that order.
    fn periodic(&self) -> (bool, bool) {
        match self {
            Self::Plane { .. } | Self::Parameter => (false, false),
            Self::Axial { .. } | Self::Sphere { .. } => (true, false),
            Self::Torus { .. } => (true, true),
        }
    }

    /// (u, v) of a model point, or `None` on the axis, where u has no value.
    fn read(&self, point: Vec3) -> Option<(f64, f64)> {
        let (Self::Plane { origin, axis, x }
        | Self::Axial { origin, axis, x }
        | Self::Sphere { origin, axis, x }
        | Self::Torus { origin, axis, x, .. }) = *self
        else {
            return Some((point.x, point.y));
        };
        let d = point.sub(origin);
        let y = axis.cross(x);
        let (a, b, c) = (d.dot(x), d.dot(y), d.dot(axis));
        let radial = a.hypot(b);
        match self {
            Self::Plane { .. } => Some((a, b)),
            Self::Axial { .. } => (radial > 1e-12).then(|| (b.atan2(a), c)),
            Self::Sphere { .. } => (radial > 1e-12).then(|| (b.atan2(a), c.atan2(radial))),
            Self::Torus { major, .. } => (radial > 1e-12).then(|| (b.atan2(a), c.atan2(radial - major))),
            Self::Parameter => None,
        }
    }
}

/// Enough about a surface to tell a seam from a bridge.
#[derive(Clone)]
enum SeamRule {
    /// Origin, axis and reference direction of a quadric.
    Quadric(Vec3, Vec3, Vec3),
}

/// The floor (mm) under the file's stated resolution: the importer's own
/// vertex identity tolerance.
const POLE_TOLERANCE: f64 = 1e-7;

fn on_seam_rule(rule: &SeamRule, point: Vec3) -> bool {
    let SeamRule::Quadric(origin, axis, x) = rule;
    on_seam(point, (*origin, *axis, *x))
}

/// Whether a zero-length boundary at `point` is a real pole of the surface.
fn is_pole(surface: &EmittedSurface, point: Vec3, tolerance: f64) -> Option<bool> {
    if let Some(SeamRule::Quadric(origin, axis, _)) = &surface.seam {
        let offset = point.sub(*origin);
        return Some(offset.sub(axis.scale(offset.dot(*axis))).length() <= tolerance);
    }
    let poles = surface.poles.as_ref()?;
    Some(poles.iter().any(|pole| pole.sub(point).length() <= tolerance))
}

/// How far (radians) an edge may sit from a quadric's reference direction and
/// still be its seam.
const SEAM_ANGLE_TOLERANCE: f64 = 1e-6;

/// Whether `point` is on the half-plane of a quadric's seam. A point on the
/// axis is on every half-plane.
fn on_seam(point: Vec3, (origin, axis, x): (Vec3, Vec3, Vec3)) -> bool {
    let offset = point.sub(origin);
    let radial = offset.sub(axis.scale(offset.dot(axis)));
    if radial.length() <= 1e-9 * (1.0 + offset.length()) {
        return true;
    }
    radial.dot(axis.cross(x)).atan2(radial.dot(x)).abs() <= SEAM_ANGLE_TOLERANCE
}

struct Translator<'a> {
    file: &'a IgesFile,
    scale: f64,
    /// What the file says it can tell apart, in mm, floored at the importer's
    /// vertex identity.
    resolution: f64,
    split_bridges: bool,
    text: String,
    next_id: usize,
    vertices: HashMap<(i64, i64), usize>,
    edges: HashMap<(i64, i64), usize>,
    /// EDGE_CURVE id to its (start, end) VERTEX_POINT ids.
    edge_ends: HashMap<usize, (usize, usize)>,
    /// EDGE_CURVE ids whose curve has no length.
    collapsed_edges: rustc_hash::FxHashSet<usize>,
    /// VERTEX_POINT id to its location.
    vertex_points: HashMap<usize, Vec3>,
}

/// Translate the 186 at `pointer` to a Part 21 document in millimetres.
/// `split_bridges` removes every edge a loop walks once each way; see
/// [`split_at_repeated_edges`] for when that is the right reading.
pub(super) fn manifold_solid_to_step(
    file: &IgesFile,
    pointer: i64,
    scale: f64,
    split_bridges: bool,
) -> Result<String, String> {
    let mut translator = Translator::new(file, scale, split_bridges);
    translator
        .solid(file.entity(pointer)?)
        .map_err(|error| format!("iges_import: solid DE {pointer}: {error}"))?;
    Ok(translator.finish(&pointer.to_string()))
}

/// Translate the 514 at `pointer`, a shell no 186 uses (the form an
/// OCC-written IGES file's free shells come from), to a Part 21 document: a
/// closed shell of several faces as a solid, any other as a surface model.
pub(super) fn free_shell_to_step(
    file: &IgesFile,
    pointer: i64,
    scale: f64,
    split_bridges: bool,
) -> Result<String, String> {
    let mut translator = Translator::new(file, scale, split_bridges);
    translator
        .surface_model(pointer)
        .map_err(|error| format!("iges_import: shell DE {pointer}: {error}"))?;
    Ok(translator.finish(&pointer.to_string()))
}

impl<'a> Translator<'a> {
    fn new(file: &'a IgesFile, scale: f64, split_bridges: bool) -> Self {
        Self {
            file,
            scale,
            resolution: (file.global.min_resolution * scale).max(POLE_TOLERANCE),
            split_bridges,
            text: String::new(),
            next_id: 1,
            vertices: HashMap::default(),
            edges: HashMap::default(),
            edge_ends: HashMap::default(),
            collapsed_edges: Default::default(),
            vertex_points: HashMap::default(),
        }
    }

    /// The finished Part 21 document. `tag` names the debug dump.
    fn finish(mut self, tag: &str) -> String {
        // The file's stated resolution, in mm, said the STEP way so the import
        // report checks the geometry against it as it does for STEP files.
        let stated = self.file.global.min_resolution * self.scale;
        if stated > 0.0 && stated.is_finite() {
            let unit = self.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))".into());
            self.add(format!(
                "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{unit},'distance_accuracy_value','IGES Global minimum resolution')",
                real(stated)
            ));
        }
        let document = format!(
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
            self.text
        );
        if let Ok(path) = std::env::var("BREP_DEBUG_IGES_STEP") {
            // A debugging aid must not be able to fail the import.
            let _ = std::fs::write(format!("{path}.{tag}"), &document);
        }
        document
    }

    fn add(&mut self, body: String) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        self.text.push_str(&format!("#{id}={body};\n"));
        id
    }

    /// The 124 at `pointer`, composed with the transforms it is itself under.
    fn matrix(&self, pointer: i64, depth: usize) -> Result<Transform, String> {
        if pointer == 0 {
            return Ok(Transform::IDENTITY);
        }
        if depth > MAX_TRANSFORM_DEPTH {
            return Err(format!("transform chain at DE {pointer} does not end"));
        }
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 124 {
            return Err(format!(
                "DE {pointer} is used as a transform but is entity {}",
                entity.de.entity_type
            ));
        }
        let at = |row: usize, column: usize| entity.real(row * 4 + column);
        let own = Transform {
            rotation: [
                [at(0, 0)?, at(0, 1)?, at(0, 2)?],
                [at(1, 0)?, at(1, 1)?, at(1, 2)?],
                [at(2, 0)?, at(2, 1)?, at(2, 2)?],
            ],
            translation: [at(0, 3)?, at(1, 3)?, at(2, 3)?],
        };
        Ok(self.matrix(entity.de.transform, depth + 1)?.after(&own))
    }

    /// The transform `entity` is under when its parent is under `parent`.
    fn placed(&self, entity: &ParsedEntity, parent: &Transform) -> Result<Transform, String> {
        Ok(parent.after(&self.matrix(entity.de.transform, 0)?))
    }

    /// A model-space point to millimetres.
    fn location(&self, transform: &Transform, p: [f64; 3]) -> Vec3 {
        vec3(transform.point(p)).scale(self.scale)
    }

    fn point_id(&mut self, p: Vec3) -> usize {
        self.add(format!(
            "CARTESIAN_POINT('',({},{},{}))",
            real(p.x),
            real(p.y),
            real(p.z)
        ))
    }

    fn direction_id(&mut self, d: Vec3) -> usize {
        self.add(format!(
            "DIRECTION('',({},{},{}))",
            real(d.x),
            real(d.y),
            real(d.z)
        ))
    }

    fn placement_id(&mut self, origin: Vec3, z: Vec3, x: Vec3) -> usize {
        let origin = self.point_id(origin);
        let z = self.direction_id(z);
        let x = self.direction_id(x);
        self.add(format!("AXIS2_PLACEMENT_3D('',#{origin},#{z},#{x})"))
    }

    /// The location held by a 116 under `parent`.
    fn point_116(&self, pointer: i64, parent: &Transform) -> Result<Vec3, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 116 {
            return Err(format!("DE {pointer} should be a point (116), is {}", entity.de.entity_type));
        }
        let transform = self.placed(entity, parent)?;
        Ok(self.location(&transform, [entity.real(0)?, entity.real(1)?, entity.real(2)?]))
    }

    /// The unit direction held by a 123 under `parent`.
    fn direction_123(&self, pointer: i64, parent: &Transform) -> Result<Vec3, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 123 {
            return Err(format!("DE {pointer} should be a direction (123), is {}", entity.de.entity_type));
        }
        let transform = self.placed(entity, parent)?;
        vec3(transform.vector([entity.real(0)?, entity.real(1)?, entity.real(2)?])).normalized()
    }

    // -----------------------------------------------------------------------
    // Topology
    // -----------------------------------------------------------------------

    fn solid(&mut self, entity: &ParsedEntity) -> Result<(), String> {
        let transform = self.placed(entity, &Transform::IDENTITY)?;
        let shell_pointer = entity.int(0)?;
        let shell_agrees = entity.int(1)? != 0;
        let void_count = entity.int(2)?;
        let shell = self.shell(shell_pointer, shell_agrees, &transform)?;
        if void_count <= 0 {
            self.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"));
            return Ok(());
        }
        let mut voids = Vec::with_capacity(void_count as usize);
        for index in 0..void_count as usize {
            let void_pointer = entity.int(3 + index * 2)?;
            let void_agrees = entity.int(4 + index * 2)? != 0;
            let void_shell = self.shell(void_pointer, void_agrees, &transform)?;
            voids.push(format!(
                "#{}",
                self.add(format!("ORIENTED_CLOSED_SHELL('',*,#{void_shell},.T.)"))
            ));
        }
        self.add(format!("BREP_WITH_VOIDS('',#{shell},({}))", voids.join(",")));
        Ok(())
    }

    /// A shell no 186 uses. A closed 514 (form 1) of more than one face is a
    /// solid's boundary by the file's own statement, so it is translated as the
    /// solid a 186 would make of it: built, retried with bridges split, or
    /// refused by name, as a 186 is. The STEP lane's surface model would count
    /// any failure to build as a sheet (m0273: a 184-face closed shell its
    /// STEP source holds as a MANIFOLD_SOLID_BREP came in as nothing). An open
    /// shell, or a closed one of a single face (a full untrimmed surface, OCC's
    /// reference geometry), stays a surface model: a sheet, skipped and counted.
    fn surface_model(&mut self, pointer: i64) -> Result<(), String> {
        let entity = self.file.entity(pointer)?;
        let closed_solid = entity.de.entity_type == 514 && entity.de.form != 2 && entity.int(0)? > 1;
        let shell = self.shell_of(pointer, true, &Transform::IDENTITY, true)?;
        if closed_solid {
            self.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"));
        } else {
            self.add(format!("SHELL_BASED_SURFACE_MODEL('',(#{shell}))"));
        }
        Ok(())
    }

    fn shell(&mut self, pointer: i64, agrees: bool, parent: &Transform) -> Result<usize, String> {
        self.shell_of(pointer, agrees, parent, false)
    }

    /// A 514 as a Part 21 shell. `open_allowed` admits form 2 (an open shell),
    /// written as an OPEN_SHELL; a solid's shells must be closed.
    fn shell_of(&mut self, pointer: i64, agrees: bool, parent: &Transform, open_allowed: bool) -> Result<usize, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 514 {
            return Err(format!("DE {pointer} should be a shell (514), is {}", entity.de.entity_type));
        }
        let open = entity.de.form == 2;
        if open && !open_allowed {
            return Err(format!(
                "shell DE {pointer} is an open shell (514 form 2); a solid needs a closed one"
            ));
        }
        let transform = self.placed(entity, parent)?;
        let count = entity.int(0)? as usize;
        let mut faces = Vec::with_capacity(count);
        for index in 0..count {
            let face = entity.int(1 + index * 2)?;
            let face_agrees = entity.int(2 + index * 2)? != 0;
            let id = self
                .face(face, face_agrees, agrees, &transform)
                .map_err(|error| format!("face DE {face}: {error}"))?;
            faces.push(format!("#{id}"));
        }
        let kind = if open { "OPEN_SHELL" } else { "CLOSED_SHELL" };
        Ok(self.add(format!("{kind}('',({}))", faces.join(","))))
    }

    fn face(
        &mut self,
        pointer: i64,
        face_agrees: bool,
        shell_agrees: bool,
        parent: &Transform,
    ) -> Result<usize, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 510 {
            return Err(format!("DE {pointer} should be a face (510), is {}", entity.de.entity_type));
        }
        let transform = self.placed(entity, parent)?;
        let surface = self.surface(entity.int(0)?, &transform)?;
        let loop_count = entity.int(1)? as usize;
        let first_is_outer = entity.int(2)? != 0;
        let mut bounds = Vec::with_capacity(loop_count);
        for index in 0..loop_count {
            let loops = self.face_loops(entity.int(3 + index)?, &transform, &surface)?;
            // A loop that was split has no single outer boundary to name.
            let outer = index == 0 && first_is_outer && loops.len() == 1;
            for loop_id in loops {
                let kind = if outer { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                // A 508 runs counter-clockwise about the surface normal, so on
                // a reversed face the loop is walked backwards.
                let bound = self.add(format!(
                    "{kind}('',#{loop_id},{})",
                    if face_agrees == shell_agrees { ".T." } else { ".F." }
                ));
                bounds.push(format!("#{bound}"));
            }
        }
        let same_sense = face_agrees == shell_agrees;
        let same_sense = same_sense != surface.reversed;
        Ok(self.add(format!(
            "ADVANCED_FACE('',({}),#{},{})",
            bounds.join(","),
            surface.id,
            if same_sense { ".T." } else { ".F." }
        )))
    }

    /// The STEP loops of one 508: usually one, more when it holds a bridge.
    fn face_loops(
        &mut self,
        pointer: i64,
        parent: &Transform,
        surface: &EmittedSurface,
    ) -> Result<Vec<usize>, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 508 {
            return Err(format!("DE {pointer} should be a loop (508), is {}", entity.de.entity_type));
        }
        let transform = self.placed(entity, parent)?;
        let count = entity.int(0)? as usize;
        let mut index = 1;
        let mut coedges: Vec<(usize, bool)> = Vec::with_capacity(count);
        for _ in 0..count {
            let kind = entity.int(index)?;
            let list = entity.int(index + 1)?;
            let member = entity.int(index + 2)?;
            let agrees = entity.int(index + 3)? != 0;
            let parameter_curves = entity.int(index + 4)?;
            // Each parameter-space curve is an (ISOP, CURV) pair. The STEP lane
            // derives its own trims, so they are read past, not used.
            index += 5 + (parameter_curves.max(0) as usize) * 2;
            if kind == 1 {
                // Inside a longer loop a vertex use is a collapsed pole edge,
                // which the STEP builder makes for itself.
                if count != 1 {
                    continue;
                }
                let vertex = self.vertex(list, member, &transform)?;
                return Ok(vec![self.add(format!("VERTEX_LOOP('',#{vertex})"))]);
            }
            coedges.push((self.edge(list, member, &transform)?, agrees));
        }
        self.emit_loops(coedges, surface, pointer)
    }

    /// STEP loops for one stated boundary: refuses an edge the file has lost,
    /// and reads twice-walked edges as seams or bridges.
    fn emit_loops(
        &mut self,
        coedges: Vec<(usize, bool)>,
        surface: &EmittedSurface,
        pointer: i64,
    ) -> Result<Vec<usize>, String> {
        let mut loops = Vec::new();
        // On a quadric the seam's place is known, so each twice-walked edge can
        // be told apart. Elsewhere the caller's reading decides.
        let split_all = self.split_bridges;
        for &(edge, _) in &coedges {
            if !self.collapsed_edges.contains(&edge) {
                continue;
            }
            let Some(&(start, end)) = self.edge_ends.get(&edge) else { continue };
            let Some(point) = self.vertex_points.get(&start).filter(|_| start == end) else {
                continue;
            };
            if is_pole(surface, *point, self.resolution) == Some(false) {
                return Err(format!(
                    "loop DE {pointer} has an edge curve of no length at ({:.4}, {:.4}, {:.4}), which is not a pole of its surface: the file has lost that boundary",
                    point.x, point.y, point.z
                ));
            }
        }
        let is_bridge = |edge: usize| match &surface.seam {
            Some(seam) => self.edge_ends.get(&edge).is_some_and(|(start, end)| {
                [start, end].iter().any(|vertex| {
                    self.vertex_points.get(vertex).is_some_and(|&point| !on_seam_rule(seam, point))
                })
            }),
            None => split_all,
        };
        let cycles = split_at_repeated_edges(coedges, is_bridge);
        for cycle in cycles {
            // One seam walked each way between two different vertices bounds
            // nothing as a loop: the face closes at those two poles, which is
            // how STEP says it.
            if let [(first, a), (second, b)] = cycle[..] {
                if first == second && a != b {
                    if let Some(&(start, end)) = self.edge_ends.get(&first) {
                        if start != end {
                            loops.push(self.add(format!("VERTEX_LOOP('',#{start})")));
                            loops.push(self.add(format!("VERTEX_LOOP('',#{end})")));
                            continue;
                        }
                    }
                }
            }
            let members: Vec<String> = cycle
                .iter()
                .map(|&(edge, agrees)| {
                    let sense = if agrees { ".T." } else { ".F." };
                    format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{edge},{sense})")))
                })
                .collect();
            loops.push(self.add(format!("EDGE_LOOP('',({}))", members.join(","))));
        }
        Ok(loops)
    }

    fn vertex_location(&self, list: i64, member: i64, parent: &Transform) -> Result<Vec3, String> {
        let entity = self.file.entity(list)?;
        if entity.de.entity_type != 502 {
            return Err(format!("DE {list} should be a vertex list (502), is {}", entity.de.entity_type));
        }
        if member < 1 {
            return Err(format!("vertex list DE {list} has no member {member}"));
        }
        let at = (member as usize - 1) * 3 + 1;
        let transform = self.placed(entity, parent)?;
        Ok(self.location(
            &transform,
            [entity.real(at)?, entity.real(at + 1)?, entity.real(at + 2)?],
        ))
    }

    fn vertex(&mut self, list: i64, member: i64, parent: &Transform) -> Result<usize, String> {
        if let Some(&id) = self.vertices.get(&(list, member)) {
            return Ok(id);
        }
        let location = self.vertex_location(list, member, parent)?;
        let point = self.point_id(location);
        let id = self.add(format!("VERTEX_POINT('',#{point})"));
        self.vertices.insert((list, member), id);
        self.vertex_points.insert(id, location);
        Ok(id)
    }

    fn edge(&mut self, list: i64, member: i64, parent: &Transform) -> Result<usize, String> {
        if let Some(&id) = self.edges.get(&(list, member)) {
            return Ok(id);
        }
        let entity = self.file.entity(list)?;
        if entity.de.entity_type != 504 {
            return Err(format!("DE {list} should be an edge list (504), is {}", entity.de.entity_type));
        }
        if member < 1 {
            return Err(format!("edge list DE {list} has no member {member}"));
        }
        let at = (member as usize - 1) * 5 + 1;
        let transform = self.placed(entity, parent)?;
        let curve_pointer = entity.int(at)?;
        let (start_list, start_member) = (entity.int(at + 1)?, entity.int(at + 2)?);
        let (end_list, end_member) = (entity.int(at + 3)?, entity.int(at + 4)?);
        let start = self.vertex_location(start_list, start_member, &transform)?;
        let end = self.vertex_location(end_list, end_member, &transform)?;
        let curve = self.curve(curve_pointer, &transform)?;
        // An IGES edge states no sense of its own: read it off the geometry.
        let same_sense = curve.start.sub(start).length() + curve.end.sub(end).length()
            <= curve.start.sub(end).length() + curve.end.sub(start).length();
        let start_id = self.vertex(start_list, start_member, &transform)?;
        let end_id = self.vertex(end_list, end_member, &transform)?;
        let id = self.add(format!(
            "EDGE_CURVE('',#{start_id},#{end_id},#{},{})",
            curve.id,
            if same_sense { ".T." } else { ".F." }
        ));
        self.edges.insert((list, member), id);
        self.edge_ends.insert(id, (start_id, end_id));
        if curve.collapsed {
            self.collapsed_edges.insert(id);
        }
        Ok(id)
    }

    // -----------------------------------------------------------------------
    // Curves
    // -----------------------------------------------------------------------

    fn curve(&mut self, pointer: i64, parent: &Transform) -> Result<EmittedCurve, String> {
        let entity = self.file.entity(pointer)?;
        let transform = self.placed(entity, parent)?;
        match entity.de.entity_type {
            100 => self.circular_arc(entity, &transform),
            104 => self.conic_arc(entity, &transform),
            106 => self.copious_data(entity, pointer, &transform),
            110 => self.line(entity, &transform),
            112 => self.parametric_spline(entity, &transform),
            126 => self.bspline_curve(entity, &transform),
            130 => self.offset_curve(entity, &transform),
            other => Err(format!("curve DE {pointer} is entity {other}, which a B-rep edge cannot use yet")),
        }
    }

    /// The right-handed frame of a curve drawn in the XY plane at `z`: the
    /// origin in mm, the images of the local axes, and the length scale.
    fn plane_frame(&self, transform: &Transform, origin: [f64; 3]) -> Result<(Vec3, Vec3, Vec3, f64), String> {
        let uniform = transform.uniform_scale().ok_or_else(|| {
            "the 124 on this conic is not a rigid motion with a uniform scale".to_string()
        })?;
        let x = vec3(transform.vector([1.0, 0.0, 0.0])).normalized()?;
        let y = vec3(transform.vector([0.0, 1.0, 0.0])).normalized()?;
        Ok((self.location(transform, origin), x, y, uniform * self.scale))
    }

    fn circular_arc(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        let z = entity.real(0)?;
        let (cx, cy) = (entity.real(1)?, entity.real(2)?);
        let (sx, sy) = (entity.real(3)?, entity.real(4)?);
        let (ex, ey) = (entity.real(5)?, entity.real(6)?);
        let (center, x, y, length) = self.plane_frame(transform, [cx, cy, z])?;
        let radius = (sx - cx).hypot(sy - cy) * length;
        let axis = x.cross(y).normalized()?;
        let placement = self.placement_id(center, axis, x);
        let id = self.add(format!("CIRCLE('',#{placement},{})", real(radius)));
        Ok(EmittedCurve {
            id,
            start: self.location(transform, [sx, sy, z]),
            end: self.location(transform, [ex, ey, z]),
            collapsed: false,
        })
    }

    fn conic_arc(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        let (a, b, c) = (entity.real(0)?, entity.real(1)?, entity.real(2)?);
        let (d, e, f) = (entity.real(3)?, entity.real(4)?, entity.real(5)?);
        let z = entity.real(6)?;
        let (sx, sy) = (entity.real(7)?, entity.real(8)?);
        let (ex, ey) = (entity.real(9)?, entity.real(10)?);
        let start = self.location(transform, [sx, sy, z]);
        let end = self.location(transform, [ex, ey, z]);
        // Principal axes of the quadratic form [[a, b/2], [b/2, c]].
        let angle = 0.5 * b.atan2(a - c);
        let (cos, sin) = (angle.cos(), angle.sin());
        let lambda_1 = a * cos * cos + b * cos * sin + c * sin * sin;
        let lambda_2 = a * sin * sin - b * cos * sin + c * cos * cos;
        let e1 = [cos, sin];
        let e2 = [-sin, cos];
        let form = entity.de.form;
        let id = match form {
            1 | 2 => {
                let det = 4.0 * a * c - b * b;
                if det == 0.0 {
                    return Err("conic has a singular quadratic form".into());
                }
                let cx = (b * e - 2.0 * c * d) / det;
                let cy = (b * d - 2.0 * a * e) / det;
                let constant = f + 0.5 * (d * cx + e * cy);
                let (mut r1, mut r2) = (-constant / lambda_1, -constant / lambda_2);
                let (mut u, mut v) = (e1, e2);
                if form == 1 {
                    if r1 <= 0.0 || r2 <= 0.0 {
                        return Err("104 form 1 is not an ellipse".into());
                    }
                    if r1 < r2 {
                        std::mem::swap(&mut r1, &mut r2);
                        (u, v) = (e2, [-e1[0], -e1[1]]);
                    }
                } else {
                    if r1 * r2 >= 0.0 {
                        return Err("104 form 2 is not a hyperbola".into());
                    }
                    if r1 < 0.0 {
                        std::mem::swap(&mut r1, &mut r2);
                        (u, v) = (e2, [-e1[0], -e1[1]]);
                    }
                    // The branch that holds the arc is the one on +x.
                    if (sx - cx) * u[0] + (sy - cy) * u[1] < 0.0 {
                        u = [-u[0], -u[1]];
                        v = [-v[0], -v[1]];
                    }
                }
                let (origin, x_image, y_image, length) = self.plane_frame(transform, [cx, cy, z])?;
                let x = x_image.scale(u[0]).add(y_image.scale(u[1])).normalized()?;
                let y = x_image.scale(v[0]).add(y_image.scale(v[1])).normalized()?;
                let axis = x.cross(y).normalized()?;
                let placement = self.placement_id(origin, axis, x);
                let (semi_1, semi_2) = (r1.abs().sqrt() * length, r2.abs().sqrt() * length);
                let keyword = if form == 1 { "ELLIPSE" } else { "HYPERBOLA" };
                self.add(format!("{keyword}('',#{placement},{},{})", real(semi_1), real(semi_2)))
            }
            3 => {
                // One eigenvalue is zero: `n` runs along the parabola's axis.
                let (lambda, p, n) = if lambda_1.abs() > lambda_2.abs() {
                    (lambda_1, e1, e2)
                } else {
                    (lambda_2, e2, [-e1[0], -e1[1]])
                };
                let d_p = d * p[0] + e * p[1];
                let e_n = d * n[0] + e * n[1];
                if lambda == 0.0 || e_n == 0.0 {
                    return Err("104 form 3 is not a parabola".into());
                }
                let p0 = -d_p / (2.0 * lambda);
                let n0 = (lambda * p0 * p0 - f) / e_n;
                let opening = if -e_n / lambda >= 0.0 { 1.0 } else { -1.0 };
                let vertex = [p0 * p[0] + n0 * n[0], p0 * p[1] + n0 * n[1]];
                let (origin, x_image, y_image, length) =
                    self.plane_frame(transform, [vertex[0], vertex[1], z])?;
                let along = [n[0] * opening, n[1] * opening];
                let x = x_image.scale(along[0]).add(y_image.scale(along[1])).normalized()?;
                let side = [-along[1], along[0]];
                let y = x_image.scale(side[0]).add(y_image.scale(side[1])).normalized()?;
                let axis = x.cross(y).normalized()?;
                let placement = self.placement_id(origin, axis, x);
                let focal = 0.25 * (e_n / lambda).abs() * length;
                self.add(format!("PARABOLA('',#{placement},{})", real(focal)))
            }
            other => return Err(format!("104 has unknown form {other}")),
        };
        Ok(EmittedCurve { id, start, end, collapsed: false })
    }

    fn line(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        let start = self.location(transform, [entity.real(0)?, entity.real(1)?, entity.real(2)?]);
        let end = self.location(transform, [entity.real(3)?, entity.real(4)?, entity.real(5)?]);
        let direction = end.sub(start).normalized()?;
        let origin = self.point_id(start);
        let along = self.direction_id(direction);
        let vector = self.add(format!("VECTOR('',#{along},1.)"));
        let id = self.add(format!("LINE('',#{origin},#{vector})"));
        Ok(EmittedCurve { id, start, end, collapsed: false })
    }

    fn copious_data(
        &mut self,
        entity: &ParsedEntity,
        pointer: i64,
        transform: &Transform,
    ) -> Result<EmittedCurve, String> {
        let form = entity.de.form;
        let count = entity.int(1)? as usize;
        let mut points = Vec::with_capacity(count);
        match form {
            11 | 1 => {
                let z = entity.real(2)?;
                for index in 0..count {
                    let at = 3 + index * 2;
                    points.push(self.location(transform, [entity.real(at)?, entity.real(at + 1)?, z]));
                }
            }
            12 | 2 => {
                for index in 0..count {
                    let at = 2 + index * 3;
                    points.push(self.location(
                        transform,
                        [entity.real(at)?, entity.real(at + 1)?, entity.real(at + 2)?],
                    ));
                }
            }
            other => return Err(format!("copious data DE {pointer} has form {other}, which is not a curve here")),
        }
        let (Some(&start), Some(&end)) = (points.first(), points.last()) else {
            return Err(format!("copious data DE {pointer} has no points"));
        };
        let ids: Vec<String> = points.iter().map(|&p| format!("#{}", self.point_id(p))).collect();
        let id = self.add(format!("POLYLINE('',({}))", ids.join(",")));
        Ok(EmittedCurve { id, start, end, collapsed: false })
    }

    /// A 112 is cubic segments in power form; each becomes one Bezier span of
    /// a degree 3 spline, which is the same curve.
    fn parametric_spline(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        let (control, multiplicities, knots) = splines::power_spline(entity)?;
        let control: Vec<Vec3> = control.into_iter().map(|p| self.location(transform, p)).collect();
        let start = control[0];
        let end = control[control.len() - 1];
        let id = self.spline_curve(3, &control, None, &multiplicities, &knots);
        Ok(EmittedCurve { id, start, end, collapsed: false })
    }

    fn bspline_curve(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedCurve, String> {
        let count = entity.int(0)? as usize + 1;
        let degree = entity.int(1)? as usize;
        let knot_count = count + degree + 1;
        let mut index = 6;
        let mut knots = Vec::with_capacity(knot_count);
        for _ in 0..knot_count {
            knots.push(entity.real(index)?);
            index += 1;
        }
        let mut weights = Vec::with_capacity(count);
        for _ in 0..count {
            weights.push(entity.real(index)?);
            index += 1;
        }
        let mut control = Vec::with_capacity(count);
        for _ in 0..count {
            control.push(self.location(
                transform,
                [entity.real(index)?, entity.real(index + 1)?, entity.real(index + 2)?],
            ));
            index += 3;
        }
        let (distinct, multiplicities) = knot_runs(&knots);
        let rational = weights.iter().any(|w| (w - 1.0).abs() > 1e-12);
        let start = control[0];
        let end = control[count - 1];
        let id = self.spline_curve(
            degree,
            &control,
            rational.then_some(weights.as_slice()),
            &multiplicities,
            &distinct,
        );
        let collapsed = control.iter().all(|p| p.sub(start).length() <= self.resolution);
        Ok(EmittedCurve { id, start, end, collapsed })
    }

    fn spline_curve(
        &mut self,
        degree: usize,
        control: &[Vec3],
        weights: Option<&[f64]>,
        multiplicities: &[usize],
        knots: &[f64],
    ) -> usize {
        let points = control
            .iter()
            .map(|&p| format!("#{}", self.point_id(p)))
            .collect::<Vec<_>>()
            .join(",");
        let mults = join_usize(multiplicities);
        let values = join_real(knots);
        match weights {
            None => self.add(format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{degree},({points}),.UNSPECIFIED.,.F.,.F.,({mults}),({values}),.UNSPECIFIED.)"
            )),
            Some(weights) => self.add(format!(
                "(BOUNDED_CURVE()B_SPLINE_CURVE({degree},({points}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS(({mults}),({values}),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({}))REPRESENTATION_ITEM(''))",
                join_real(weights)
            )),
        }
    }

    // -----------------------------------------------------------------------
    // Surfaces
    // -----------------------------------------------------------------------

    fn surface(&mut self, pointer: i64, parent: &Transform) -> Result<EmittedSurface, String> {
        let entity = self.file.entity(pointer)?;
        let transform = self.placed(entity, parent)?;
        match entity.de.entity_type {
            108 => self.plane_108(entity, &transform),
            114 => self.parametric_spline_surface(entity, &transform),
            118 => self.ruled(entity, &transform),
            190 => self.plane_190(entity, &transform),
            192 | 194 | 196 | 198 => self.quadric(entity, &transform),
            120 => self.revolution(entity, &transform),
            122 => self.extrusion(entity, &transform),
            128 => self.bspline_surface(entity, &transform),
            140 => self.offset(entity, &transform),
            other => Err(format!("surface DE {pointer} is entity {other}, which a B-rep face cannot use yet")),
        }
    }

    fn plane_entity(&mut self, origin: Vec3, normal: Vec3, x: Vec3) -> EmittedSurface {
        let placement = self.placement_id(origin, normal, x);
        let id = self.add(format!("PLANE('',#{placement})"));
        EmittedSurface { id, reversed: false, seam: None, poles: None, chart: Some(Chart::Plane { origin, axis: normal, x }), domain_v: None, interior: None, spline: None }
    }

    fn plane_108(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let n = [entity.real(0)?, entity.real(1)?, entity.real(2)?];
        let d = entity.real(3)?;
        let squared = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
        if squared == 0.0 {
            return Err("108 has a zero normal".into());
        }
        let on_plane = [n[0] * d / squared, n[1] * d / squared, n[2] * d / squared];
        let local = Vec3::new(n[0], n[1], n[2]).normalized()?;
        let u = perpendicular(local)?;
        let v = local.cross(u).normalized()?;
        let image_u = vec3(transform.vector([u.x, u.y, u.z])).normalized()?;
        let image_v = vec3(transform.vector([v.x, v.y, v.z])).normalized()?;
        let normal = image_u.cross(image_v).normalized()?;
        let origin = self.location(transform, on_plane);
        Ok(self.plane_entity(origin, normal, image_u))
    }

    fn plane_190(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let origin = self.point_116(entity.int(0)?, transform)?;
        let mut normal = self.direction_123(entity.int(1)?, transform)?;
        if transform.reflects() {
            normal = normal.scale(-1.0);
        }
        let x = match entity.int(2).unwrap_or(0) {
            0 => perpendicular(normal)?,
            reference => self.direction_123(reference, transform)?,
        };
        Ok(self.plane_entity(origin, normal, x))
    }

    fn quadric(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let uniform = transform.uniform_scale().ok_or_else(|| {
            format!("the 124 on this entity {} is not a rigid motion with a uniform scale", entity.de.entity_type)
        })?;
        let length = uniform * self.scale;
        let origin = self.point_116(entity.int(0)?, transform)?;
        let kind = entity.de.entity_type;
        let (axis_slot, refdir_slot) = match kind {
            192 => (1, 3),
            194 => (1, 4),
            196 => (2, 3),
            _ => (1, 4),
        };
        // A 196 of form 0 states no axis, and any axis describes that sphere.
        let axis = match entity.int(axis_slot) {
            Ok(pointer) if pointer != 0 => self.direction_123(pointer, transform)?,
            _ => Vec3::new(0.0, 0.0, 1.0),
        };
        let x = match entity.int(refdir_slot).unwrap_or(0) {
            0 => perpendicular(axis)?,
            pointer => self.direction_123(pointer, transform)?,
        };
        let quadric_chart = match kind {
            192 | 194 => Chart::Axial { origin, axis, x },
            196 => Chart::Sphere { origin, axis, x },
            _ => Chart::Torus { origin, axis, x, major: entity.real(2)? * length },
        };
        let placement = self.placement_id(origin, axis, x);
        let body = match kind {
            192 => format!("CYLINDRICAL_SURFACE('',#{placement},{})", real(entity.real(2)? * length)),
            194 => format!(
                "CONICAL_SURFACE('',#{placement},{},{})",
                real(entity.real(2)? * length),
                real(entity.real(3)?.to_radians())
            ),
            196 => format!("SPHERICAL_SURFACE('',#{placement},{})", real(entity.real(1)? * length)),
            _ => format!(
                "TOROIDAL_SURFACE('',#{placement},{},{})",
                real(entity.real(2)? * length),
                real(entity.real(3)? * length)
            ),
        };
        let id = self.add(body);
        // STEP closed analytics always point outward; a mirrored IGES one
        // points in, and the face sense has to say so.
        Ok(EmittedSurface { id, reversed: transform.reflects(), seam: Some(SeamRule::Quadric(origin, axis, x)), poles: None, chart: Some(quadric_chart), domain_v: None, interior: None, spline: None })
    }

    fn revolution(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let axis_entity = self.file.entity(entity.int(0)?)?;
        if axis_entity.de.entity_type != 110 {
            return Err(format!("the axis of a 120 is entity {}, expected 110", axis_entity.de.entity_type));
        }
        let axis_transform = self.placed(axis_entity, transform)?;
        let axis_start = self.location(&axis_transform, [axis_entity.real(0)?, axis_entity.real(1)?, axis_entity.real(2)?]);
        let axis_end = self.location(&axis_transform, [axis_entity.real(3)?, axis_entity.real(4)?, axis_entity.real(5)?]);
        let axis = axis_end.sub(axis_start).normalized()?;
        let generatrix_pointer = entity.int(1)?;
        let generatrix = self.file.entity(generatrix_pointer)?;
        let generatrix_transform = self.placed(generatrix, transform)?;
        let foot = |p: Vec3| axis_start.add(axis.scale(p.sub(axis_start).dot(axis)));
        // A 120 turns its generatrix right-handedly about its axis in its own
        // frame; a mirroring 124 turns that the other way in model space.
        let handed = if transform.reflects() { -1.0 } else { 1.0 };
        if generatrix.de.entity_type == 110 {
            let a = self.location(&generatrix_transform, [generatrix.real(0)?, generatrix.real(1)?, generatrix.real(2)?]);
            let b = self.location(&generatrix_transform, [generatrix.real(3)?, generatrix.real(4)?, generatrix.real(5)?]);
            return self.revolved_line(a, b, axis_start, axis, handed);
        }
        if generatrix.de.entity_type == 100 {
            let z = generatrix.real(0)?;
            let center = self.location(&generatrix_transform, [generatrix.real(1)?, generatrix.real(2)?, z]);
            let start = self.location(&generatrix_transform, [generatrix.real(3)?, generatrix.real(4)?, z]);
            let plane_normal = vec3(generatrix_transform.vector([0.0, 0.0, 1.0])).normalized()?;
            let radius = start.sub(center).length();
            let bar = 1e-9 * (1.0 + radius + center.length());
            if plane_normal.dot(axis).abs() <= 1e-9 && center.sub(foot(center)).dot(plane_normal).abs() <= bar {
                // A circle in a plane through the axis: a sphere or a torus.
                let center_foot = foot(center);
                let major = center.sub(center_foot).length();
                // The arc's middle, off the axis even when the arc runs pole to
                // pole: the 120's normal is read there, and a sphere's seam is
                // placed by it.
                let (local_cx, local_cy) = (generatrix.real(1)?, generatrix.real(2)?);
                let from = (generatrix.real(4)? - local_cy).atan2(generatrix.real(3)? - local_cx);
                let mut to = (generatrix.real(6)? - local_cy).atan2(generatrix.real(5)? - local_cx);
                if to <= from + 1e-12 {
                    to += std::f64::consts::TAU;
                }
                let middle_angle = 0.5 * (from + to);
                let local_radius = (generatrix.real(3)? - local_cx).hypot(generatrix.real(4)? - local_cy);
                let middle_point = self.location(
                    &generatrix_transform,
                    [local_cx + local_radius * middle_angle.cos(), local_cy + local_radius * middle_angle.sin(), z],
                );
                // A full turn's seam is where it starts: such a sphere's x is
                // the generatrix's half plane turned to the start angle, so the
                // file's seam is the STEP carrier's seam and is read as one,
                // not as a bridge (m0105, m0276). A partial turn has no seam;
                // its edge at the start angle is a boundary like any other, and
                // its placement is left as it was (m0263).
                let full_turn = ((entity.real(3)? - entity.real(2)?) - std::f64::consts::TAU).abs() <= 1e-6;
                let sphere_seam = major <= bar && full_turn;
                let x = if major > bar {
                    center.sub(center_foot).normalized()?
                } else if sphere_seam {
                    let start_angle = entity.real(2)? * handed;
                    rotated_about(middle_point, center_foot, axis, start_angle).sub(foot(middle_point)).normalized()?
                } else {
                    perpendicular(axis)?
                };
                let placement = self.placement_id(center_foot, axis, x);
                let id = if major <= bar {
                    self.add(format!("SPHERICAL_SURFACE('',#{placement},{})", real(radius)))
                } else {
                    self.add(format!("TOROIDAL_SURFACE('',#{placement},{},{})", real(major), real(radius)))
                };
                // The 120 normal is C' x (axis x radial); STEP's is outward.
                // Read at the arc's middle: its start is often a pole, where
                // the radial vanishes and axis x radial has no direction.
                let tangent = vec3(generatrix_transform.vector([-middle_angle.sin(), middle_angle.cos(), 0.0]));
                let sweep = axis.cross(middle_point.sub(foot(middle_point))).scale(handed);
                let outward = middle_point.sub(center);
                let chart = if major <= bar {
                    Chart::Sphere { origin: center_foot, axis, x }
                } else {
                    Chart::Torus { origin: center_foot, axis, x, major }
                };
                let middle = self.curve_point(generatrix, &generatrix_transform, 0.5);
                let domain_v = middle.and_then(|p| chart.read(p)).map(|(_, v)| v);
                let interior = match (major <= bar, middle) {
                    (true, Some(p)) => {
                        let angle = 0.5 * (entity.real(2)? + entity.real(3)?);
                        Some(rotated_about(p, center_foot, axis, angle * handed))
                    }
                    _ => None,
                };
                let seam = sphere_seam.then_some(SeamRule::Quadric(center_foot, axis, x));
                return Ok(EmittedSurface { id, reversed: tangent.cross(sweep).dot(outward) < 0.0, seam, poles: None, chart: Some(chart), domain_v, interior, spline: None });
            }
        }
        let curve = self.curve(generatrix_pointer, transform)?;
        let location = self.point_id(axis_start);
        let direction = self.direction_id(axis);
        let placement = self.add(format!("AXIS1_PLACEMENT('',#{location},#{direction})"));
        let id = self.add(format!("SURFACE_OF_REVOLUTION('',#{},#{placement})", curve.id));
        // IGES sweeps the curve as u and the angle as v; STEP has them the other
        // way round, which is the opposite normal.
        Ok(EmittedSurface { id, reversed: true, seam: None, poles: None, chart: None, domain_v: None, interior: None, spline: None })
    }

    fn revolved_line(
        &mut self,
        a: Vec3,
        b: Vec3,
        axis_start: Vec3,
        axis: Vec3,
        handed: f64,
    ) -> Result<EmittedSurface, String> {
        let foot = |p: Vec3| axis_start.add(axis.scale(p.sub(axis_start).dot(axis)));
        let (radial_a, radial_b) = (a.sub(foot(a)), b.sub(foot(b)));
        let (radius_a, radius_b) = (radial_a.length(), radial_b.length());
        let height = b.sub(a).dot(axis);
        let span = b.sub(a).length();
        let tangent = b.sub(a);
        // Sample where the line is off the axis, for the radial direction.
        let (sample, radial) = if radius_a >= radius_b { (a, radial_a) } else { (b, radial_b) };
        let sweep = axis.cross(radial).scale(handed);
        if height.abs() <= 1e-9 * span {
            // Perpendicular to the axis: the revolved line is a flat ring.
            let normal = tangent.cross(sweep).normalized()?;
            return Ok(self.plane_entity(foot(sample), normal, radial.normalized()?));
        }
        let x = if radial.length() > 0.0 {
            radial.normalized()?
        } else {
            perpendicular(axis)?
        };
        let normal_param = tangent.cross(sweep);
        if (radius_a - radius_b).abs() <= 1e-9 * span {
            let placement = self.placement_id(foot(a), axis, x);
            let id = self.add(format!("CYLINDRICAL_SURFACE('',#{placement},{})", real(radius_a)));
            let chart = Chart::Axial { origin: foot(a), axis, x };
            return Ok(EmittedSurface { id, reversed: normal_param.dot(radial) < 0.0, seam: None, poles: None, chart: Some(chart), domain_v: None, interior: None, spline: None });
        }
        // Keep the radius growing along the placement axis, as STEP requires.
        let growth = (radius_b - radius_a) / height;
        let (z, tangent_of_angle) = if growth > 0.0 { (axis, growth) } else { (axis.scale(-1.0), -growth) };
        let (near, near_radius) = (a, radius_a);
        let placement = self.placement_id(foot(near), z, x);
        let id = self.add(format!(
            "CONICAL_SURFACE('',#{placement},{},{})",
            real(near_radius),
            real(tangent_of_angle.atan())
        ));
        // A cone's outward normal leans off the radial; the sign against the
        // radial still decides which side the 120 is on.
        let chart = Chart::Axial { origin: foot(near), axis: z, x };
        let midpoint = a.add(b).scale(0.5);
        let domain_v = chart.read(midpoint).map(|(_, v)| v);
        let interior = None;
        Ok(EmittedSurface { id, reversed: normal_param.dot(radial) < 0.0, seam: None, poles: None, chart: Some(chart), domain_v, interior, spline: None })
    }

    fn extrusion(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let curve_pointer = entity.int(0)?;
        let curve_entity = self.file.entity(curve_pointer)?;
        let curve_transform = self.placed(curve_entity, transform)?;
        let terminate = self.location(transform, [entity.real(1)?, entity.real(2)?, entity.real(3)?]);
        if curve_entity.de.entity_type == 110 {
            let a = self.location(&curve_transform, [curve_entity.real(0)?, curve_entity.real(1)?, curve_entity.real(2)?]);
            let b = self.location(&curve_transform, [curve_entity.real(3)?, curve_entity.real(4)?, curve_entity.real(5)?]);
            let u = b.sub(a).normalized()?;
            let v = terminate.sub(a);
            let normal = u.cross(v).normalized()?;
            return Ok(self.plane_entity(a, normal, u));
        }
        let curve = self.curve(curve_pointer, transform)?;
        let extent = terminate.sub(curve.start);
        let direction = self.direction_id(extent.normalized()?);
        let vector = self.add(format!("VECTOR('',#{direction},{})", real(extent.length())));
        let id = self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{},#{vector})", curve.id));
        Ok(EmittedSurface { id, reversed: false, seam: None, poles: None, chart: None, domain_v: None, interior: None, spline: None })
    }

    fn offset(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let uniform = transform.uniform_scale().ok_or_else(|| {
            "the 124 on this offset surface is not a rigid motion with a uniform scale".to_string()
        })?;
        let distance = entity.real(3)? * uniform * self.scale;
        let basis = self.surface(entity.int(4)?, transform)?;
        // The offset goes along the basis' own normal, so a basis whose STEP
        // normal is reversed has to be offset the other way.
        let distance = if basis.reversed { -distance } else { distance };
        let id = self.add(format!("OFFSET_SURFACE('',#{},{},.F.)", basis.id, real(distance)));
        Ok(EmittedSurface { id, reversed: basis.reversed, seam: None, poles: None, chart: None, domain_v: None, interior: None, spline: None })
    }

    fn bspline_surface(&mut self, entity: &ParsedEntity, transform: &Transform) -> Result<EmittedSurface, String> {
        let count_u = entity.int(0)? as usize + 1;
        let count_v = entity.int(1)? as usize + 1;
        let (degree_u, degree_v) = (entity.int(2)? as usize, entity.int(3)? as usize);
        let mut index = 9;
        let read_run = |index: &mut usize, count: usize| -> Result<Vec<f64>, String> {
            let mut out = Vec::with_capacity(count);
            for _ in 0..count {
                out.push(entity.real(*index)?);
                *index += 1;
            }
            Ok(out)
        };
        let knots_u = read_run(&mut index, count_u + degree_u + 1)?;
        let knots_v = read_run(&mut index, count_v + degree_v + 1)?;
        let weights = read_run(&mut index, count_u * count_v)?;
        // IGES runs u fastest; STEP lists rows of constant u.
        let mut grid = vec![vec![String::new(); count_v]; count_u];
        let mut weight_grid = vec![vec![1.0f64; count_v]; count_u];
        let mut homogeneous = vec![vec![crate::Vec4::from_point(Vec3::default(), 1.0); count_v]; count_u];
        for j in 0..count_v {
            for i in 0..count_u {
                let p = self.location(transform, [entity.real(index)?, entity.real(index + 1)?, entity.real(index + 2)?]);
                index += 3;
                grid[i][j] = format!("#{}", self.point_id(p));
                weight_grid[i][j] = weights[j * count_u + i];
                homogeneous[i][j] = crate::Vec4::from_point(p, weights[j * count_u + i]);
            }
        }
        // Only to read model-space boundaries in (u, v); a surface the kernel
        // will not build here is left to the parameter-space curves.
        let spline = crate::NurbsSurface::new(degree_u, degree_v, knots_u.clone(), knots_v.clone(), homogeneous)
            .ok()
            .map(std::rc::Rc::new);
        let rows = grid
            .iter()
            .map(|row| format!("({})", row.join(",")))
            .collect::<Vec<_>>()
            .join(",");
        let (distinct_u, mults_u) = knot_runs(&knots_u);
        let (distinct_v, mults_v) = knot_runs(&knots_v);
        let (mults_u, mults_v) = (join_usize(&mults_u), join_usize(&mults_v));
        let (values_u, values_v) = (join_real(&distinct_u), join_real(&distinct_v));
        let rational = weights.iter().any(|w| (w - 1.0).abs() > 1e-12);
        let id = if rational {
            let weight_rows = weight_grid
                .iter()
                .map(|row| format!("({})", join_real(row)))
                .collect::<Vec<_>>()
                .join(",");
            self.add(format!(
                "(BOUNDED_SURFACE()B_SPLINE_SURFACE({degree_u},{degree_v},({rows}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS(({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({weight_rows}))REPRESENTATION_ITEM('')SURFACE())"
            ))
        } else {
            self.add(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{degree_u},{degree_v},({rows}),.UNSPECIFIED.,.F.,.F.,.F.,({mults_u}),({mults_v}),({values_u}),({values_v}),.UNSPECIFIED.)"
            ))
        };
        let poles = self.boundary_poles(entity, transform);
        Ok(EmittedSurface { id, reversed: false, seam: None, poles, chart: Some(Chart::Parameter), domain_v: None, interior: None, spline })
    }

    /// The points where a 128's four boundaries collapse, or `None` when it
    /// will not evaluate.
    fn boundary_poles(&self, entity: &ParsedEntity, transform: &Transform) -> Option<Vec<Vec3>> {
        const STATIONS: usize = 16;
        let surface = super::entities::surface_from_128(entity, 1.0).ok()?;
        let [u0, u1] = surface.domain_u().ok()?;
        let [v0, v1] = surface.domain_v().ok()?;
        let mut poles = Vec::new();
        let mut boundary = |point_at: &dyn Fn(f64) -> (f64, f64)| -> Option<()> {
            let mut points = Vec::with_capacity(STATIONS + 1);
            for i in 0..=STATIONS {
                let (u, v) = point_at(i as f64 / STATIONS as f64);
                let p = surface.evaluate(u, v).ok()?;
                points.push(self.location(transform, [p.x, p.y, p.z]));
            }
            if points.iter().all(|p| p.sub(points[0]).length() <= self.resolution) {
                poles.push(points[0]);
            }
            Some(())
        };
        boundary(&|t| (u0, v0 + (v1 - v0) * t))?;
        boundary(&|t| (u1, v0 + (v1 - v0) * t))?;
        boundary(&|t| (u0 + (u1 - u0) * t, v0))?;
        boundary(&|t| (u0 + (u1 - u0) * t, v1))?;
        Some(poles)
    }
}

/// Remove every edge a loop walks once each way, splitting the loop there.
///
/// OCC joins separate boundaries of a face with a bridge edge so that the face
/// has one loop. A bridge bounds nothing, and the STEP builder places a periodic
/// chart from the boundary, so it cannot follow one away from the chart edge;
/// the pieces are the boundaries the face has. A true seam at the chart edge
/// looks the same and must stay; `is_bridge` says which is which. A loop that is nothing but such edges is left whole.
pub(super) fn split_at_repeated_edges(
    cycle: Vec<(usize, bool)>,
    is_bridge: impl Fn(usize) -> bool,
) -> Vec<Vec<(usize, bool)>> {
    let original = cycle.clone();
    let mut pending = vec![cycle];
    let mut done = Vec::new();
    while let Some(current) = pending.pop() {
        let pair = (0..current.len()).find_map(|i| {
            (i + 1..current.len())
                .find(|&j| current[j].0 == current[i].0 && current[j].1 != current[i].1 && is_bridge(current[i].0))
                .map(|j| (i, j))
        });
        match pair {
            None => done.push(current),
            Some((i, j)) => {
                let between = current[i + 1..j].to_vec();
                let around: Vec<(usize, bool)> =
                    current[j + 1..].iter().chain(&current[..i]).copied().collect();
                pending.extend([around, between].into_iter().filter(|part| !part.is_empty()));
            }
        }
    }
    if done.is_empty() {
        vec![original]
    } else {
        done
    }
}

/// A full knot vector as (distinct values, multiplicities).
fn knot_runs(knots: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut distinct: Vec<f64> = Vec::new();
    let mut multiplicities: Vec<usize> = Vec::new();
    for &knot in knots {
        match distinct.last() {
            Some(&last) if last == knot => *multiplicities.last_mut().expect("runs stay paired") += 1,
            _ => {
                distinct.push(knot);
                multiplicities.push(1);
            }
        }
    }
    (distinct, multiplicities)
}

fn join_usize(values: &[usize]) -> String {
    values.iter().map(usize::to_string).collect::<Vec<_>>().join(",")
}

fn join_real(values: &[f64]) -> String {
    values.iter().map(|&v| real(v)).collect::<Vec<_>>().join(",")
}
