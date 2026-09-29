//! 3MF (3D Manufacturing Format) import — the core specification only.
//!
//! [`read_3mf`] reads a 3MF package (an OPC ZIP container) and returns the
//! meshes its `<resources>` define plus the flattened list of instances the
//! `<build>` places, in millimetres. It is a READER: the triangle soup it
//! produces goes into the same mesh-to-solid chain STL and OBJ use
//! (`brep_reconstruction::stl_conversion::convert_stl_mesh_to_step`); no
//! geometry is built here.
//!
//! # What is supported
//!
//! The core specification (3MF Core Specification 1.x,
//! `http://schemas.microsoft.com/3dmanufacturing/core/2015/02`):
//!
//! * `<model unit>` — `micron`, `millimeter`, `centimeter`, `inch`, `foot`,
//!   `meter`, defaulting to `millimeter`; every coordinate and translation is
//!   converted to millimetres, the unit the kernel works in.
//! * `<resources>` with `<object id type>` carrying either a `<mesh>`
//!   (`<vertices><vertex x y z>`, `<triangles><triangle v1 v2 v3>`) or a
//!   `<components>` list of `<component objectid transform>`.
//! * `<build>` with `<item objectid transform>`.
//! * The 3x4 row-vector `transform` attribute on components and items, composed
//!   inner-to-outer, with the winding of a mirrored instance reversed so the
//!   soup stays outward-facing.
//!
//! # What is ignored, by design
//!
//! Everything outside the core namespace: the materials-and-properties, slice,
//! production, beam-lattice and secure-content extensions. Extension elements
//! are skipped as whole subtrees and prefixed attributes (`p:UUID`, a
//! triangle's `pid`/`p1..p3` property indices) are never read, so a file that
//! carries them imports as pure geometry rather than failing. A package that
//! declares such an extension REQUIRED (`requiredextensions`) is refused
//! instead, because honouring it is exactly what this reader cannot do.
//! `<metadata>` and thumbnails carry no geometry and are skipped. An object's
//! `name` is kept on each [`ThreeMfInstance`] only so a consumer can say WHICH
//! object a refusal is about.
//!
//! Anything the core specification makes required and the file omits — the
//! start-part relationship, an object id, a triangle's vertex indices, a
//! component's or item's `objectid` — is refused with a [`ThreeMfError`]
//! naming the package part it was read from.
//!
//! # Bounds
//!
//! Nothing here grows to fit its input. The container refuses encryption,
//! ZIP64, multi-disk packages and any compression but stored and deflate; a
//! part is refused before decompression when it DECLARES more than
//! [`MAX_PART_BYTES`], the deflate output stops at the declared size, and every
//! part's CRC-32 and length are verified after it. The XML reader refuses a
//! `<!DOCTYPE>` outright (so no entity is expanded, internal or external) and
//! caps nesting. The model is held to [`MAX_MESH_VERTICES`],
//! [`MAX_MESH_TRIANGLES`], [`MAX_INSTANCES`], [`MAX_TOTAL_TRIANGLES`] and
//! [`MAX_COMPONENT_DEPTH`]. Every offset in the container is computed with
//! `checked_add`, because the kernel's release profile sets
//! `overflow-checks = false`.
//!
//! # Dependency review (why the container is hand-rolled)
//!
//! `BREP_kernel` publishes to crates.io and compiles to wasm on nine declared
//! dependencies, seven of them unconditional; `zip` and `flate2` appear nowhere
//! in `BREP_kernel/Cargo.lock` (they are in `BREP_app`'s lock only, pulled in by
//! the egui/image stack, not by anything the kernel links). `zip` with
//! `default-features = false, features = ["deflate"]` still adds `flate2`,
//! `miniz_oxide`, `crc32fast`, `adler2` and their support crates to a published
//! geometry kernel. The container here is ~1000 lines of reader for a format the
//! kernel needs in one place, matching how STEP, IGES, STL and OBJ are already
//! read: [`zip`] parses the central directory, [`inflate`] implements RFC 1951,
//! [`xml`] is a bounded pull parser. Every one of them refuses rather than grows.

mod inflate;
mod model;
mod xml;
mod zip;


use serde::Serialize;

/// The 3MF core-specification namespace this reader accepts elements from.
pub const CORE_NAMESPACE: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
/// The OPC relationships namespace of `_rels/.rels`.
const RELATIONSHIP_NAMESPACE: &str = "http://schemas.openxmlformats.org/package/2006/relationships";
/// The relationship type naming the package's primary 3D model part.
const MODEL_RELATIONSHIP: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
/// The package-level relationships part; required of every OPC package.
const RELATIONSHIP_PART: &str = "_rels/.rels";

/// Largest part this reader will expand, compressed or stored. A 3MF model part
/// is XML: 256 MiB is roughly five million triangles of it.
pub const MAX_PART_BYTES: usize = 256 * 1024 * 1024;
/// Largest vertex and triangle counts accepted for one `<mesh>`.
pub const MAX_MESH_VERTICES: usize = 8_000_000;
pub const MAX_MESH_TRIANGLES: usize = 16_000_000;
/// Largest number of placements the `<build>` may flatten to.
pub const MAX_INSTANCES: usize = 65_536;
/// Largest flattened triangle total across all instances.
pub const MAX_TOTAL_TRIANGLES: usize = 32_000_000;
/// Deepest `<components>` nesting followed when flattening. The core spec's
/// "an object must be defined before it is referenced" already forbids cycles;
/// this bounds the work a legal but pathological file can ask for.
pub const MAX_COMPONENT_DEPTH: usize = 32;

/// Why a 3MF package was refused. Every variant that belongs to a package part
/// names it, so the message says where in the container the fault is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ThreeMfError {
    /// The bytes are not a readable OPC/ZIP container at all.
    Container(String),
    /// A part is in the container but its bytes cannot be recovered: a bad
    /// local header, a short read, a failed CRC-32.
    Damaged { part: String, detail: String },
    /// A part the core specification requires is absent.
    MissingPart { part: String, detail: String },
    /// A container or format feature this reader refuses on purpose.
    Unsupported { part: String, detail: String },
    /// A part exceeds one of this reader's documented bounds.
    Bounds { part: String, detail: String },
    /// A part is not well-formed XML.
    Xml { part: String, detail: String },
    /// A part is well-formed XML but violates the core specification.
    Model { part: String, detail: String },
}

impl ThreeMfError {
    /// The package part the failure was found in, when it belongs to one.
    pub fn part(&self) -> Option<&str> {
        match self {
            Self::Container(_) => None,
            Self::Damaged { part, .. }
            | Self::MissingPart { part, .. }
            | Self::Unsupported { part, .. }
            | Self::Bounds { part, .. }
            | Self::Xml { part, .. }
            | Self::Model { part, .. } => Some(part),
        }
    }
}

impl core::fmt::Display for ThreeMfError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Container(detail) => write!(f, "3MF package: {detail}"),
            Self::Damaged { part, detail } => write!(f, "3MF '{part}' is damaged: {detail}"),
            Self::MissingPart { part, detail } => write!(f, "3MF '{part}' is missing: {detail}"),
            Self::Unsupported { part, detail } => write!(f, "3MF '{part}': {detail}"),
            Self::Bounds { part, detail } => write!(f, "3MF '{part}' exceeds a bound: {detail}"),
            Self::Xml { part, detail } => write!(f, "3MF '{part}' is not well-formed: {detail}"),
            Self::Model { part, detail } => write!(f, "3MF '{part}': {detail}"),
        }
    }
}

impl std::error::Error for ThreeMfError {}

/// The `unit` a model part declares, with its conversion to millimetres.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub enum ThreeMfUnit {
    Micron,
    Millimeter,
    Centimeter,
    Inch,
    Foot,
    Meter,
}

impl ThreeMfUnit {
    /// The spec's unit table (3MF Core, `<model unit>`), in millimetres.
    pub fn millimetres(self) -> f64 {
        match self {
            Self::Micron => 0.001,
            Self::Millimeter => 1.0,
            Self::Centimeter => 10.0,
            Self::Inch => 25.4,
            Self::Foot => 304.8,
            Self::Meter => 1000.0,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Micron => "micron",
            Self::Millimeter => "millimeter",
            Self::Centimeter => "centimeter",
            Self::Inch => "inch",
            Self::Foot => "foot",
            Self::Meter => "meter",
        }
    }

    fn parse(text: &str) -> Option<Self> {
        Some(match text {
            "micron" => Self::Micron,
            "millimeter" => Self::Millimeter,
            "centimeter" => Self::Centimeter,
            "inch" => Self::Inch,
            "foot" => Self::Foot,
            "meter" => Self::Meter,
            _ => return None,
        })
    }
}

/// One `<object>`'s mesh, in millimetres and in the object's own coordinates.
/// Shared: an object placed three times is ONE entry here and three
/// [`ThreeMfInstance`]s.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct ThreeMfMesh {
    /// The id of the `<object>` this mesh belongs to.
    pub object_id: u32,
    /// Vertex coordinates as xyz triples, millimetres.
    pub positions: Vec<f64>,
    /// Triangle corners, three indices into `positions`/3.
    pub indices: Vec<u32>,
}

impl ThreeMfMesh {
    pub fn vertex_count(&self) -> usize {
        self.positions.len() / 3
    }
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
}

/// One placement of one mesh: a `<build><item>`, or a `<component>` reached
/// through one, with the transforms from the item down composed.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ThreeMfInstance {
    /// The mesh-bearing object placed (not the item's `objectid` when the item
    /// pointed at a `<components>` object).
    pub object_id: u32,
    /// Index into [`ThreeMfReadResult::meshes`].
    pub mesh: usize,
    /// The composed 3x4 row-vector transform, in 3MF attribute order
    /// (`m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`), translation in mm.
    pub transform: [f64; 12],
    /// Where this instance's geometry sits in the flattened buffers: its
    /// vertices are `positions[first_vertex * 3 ..][.. vertex_count * 3]` and
    /// its triangles `indices[first_triangle * 3 ..][.. triangle_count * 3]`.
    /// Instances are laid out in order and never share a vertex, which is what
    /// lets a consumer take one body out without re-flattening the build.
    pub first_vertex: usize,
    pub vertex_count: usize,
    pub first_triangle: usize,
    pub triangle_count: usize,
    /// The placed object's `name`, or failing that the nearest named
    /// `<components>` object above it; `None` when nothing on the way was
    /// named. Carries no geometry.
    pub name: Option<String>,
}

impl ThreeMfInstance {
    /// Place an object-local point: `x' = x·m00 + y·m10 + z·m20 + m30`.
    pub fn transform_point(&self, point: [f64; 3]) -> [f64; 3] {
        transform_point(&self.transform, point)
    }
}

/// Everything [`read_3mf`] recovered from a package.
#[derive(Clone, Debug, Serialize)]
pub struct ThreeMfReadResult {
    /// The part the model was read from (from the start-part relationship).
    pub model_part: String,
    /// The unit the file declared, before conversion.
    pub unit: ThreeMfUnit,
    /// One entry per mesh-bearing `<object>` in `<resources>`, in document
    /// order — including any the build never places.
    pub meshes: Vec<ThreeMfMesh>,
    /// The `<build>`, flattened through `<components>`: one entry per placed
    /// mesh. Objects of a type other than `model` are not placed.
    pub instances: Vec<ThreeMfInstance>,
    /// Every instance's triangles in world coordinates, millimetres — the soup
    /// the mesh-to-solid chain reads.
    pub positions: Vec<f64>,
    /// Triangle corners into `positions`/3, winding preserved (reversed for a
    /// mirrored instance so the result stays outward-facing).
    pub indices: Vec<u32>,
}

impl ThreeMfReadResult {
    pub fn vertex_count(&self) -> usize {
        self.positions.len() / 3
    }
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// ONE instance's triangle soup: its world-space vertices and its triangles
    /// rebased onto them. A multi-instance build is several bodies in one
    /// buffer — this is how a consumer takes them apart by PLACEMENT without
    /// knowing how `read_3mf` laid them out. The importer does not need it: the
    /// mesh-to-solid chain splits the whole soup into its vertex-connected
    /// components and reconstructs each as its own body. It is the answer when
    /// the question is which instance a body came from, which the geometry
    /// alone cannot say (two instances that touch are one component).
    pub fn instance_mesh(&self, instance: usize) -> Option<(Vec<f64>, Vec<u32>)> {
        let instance = self.instances.get(instance)?;
        let positions = self
            .positions
            .get(instance.first_vertex * 3..(instance.first_vertex + instance.vertex_count) * 3)?
            .to_vec();
        let base = instance.first_vertex as u32;
        let indices = self
            .indices
            .get(
                instance.first_triangle * 3
                    ..(instance.first_triangle + instance.triangle_count) * 3,
            )?
            .iter()
            .map(|index| index - base)
            .collect();
        Some((positions, indices))
    }
}

/// The identity 3x4 transform, in 3MF attribute order.
pub const IDENTITY: [f64; 12] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

/// Place a point with a 3MF row-vector transform.
pub fn transform_point(transform: &[f64; 12], point: [f64; 3]) -> [f64; 3] {
    let [x, y, z] = point;
    [
        x * transform[0] + y * transform[3] + z * transform[6] + transform[9],
        x * transform[1] + y * transform[4] + z * transform[7] + transform[10],
        x * transform[2] + y * transform[5] + z * transform[8] + transform[11],
    ]
}

/// Compose `inner` then `outer` (row vectors: `p · inner · outer`) — a
/// component's transform inside the item that placed it.
pub fn compose(inner: &[f64; 12], outer: &[f64; 12]) -> [f64; 12] {
    let mut out = [0.0; 12];
    for row in 0..4 {
        for column in 0..3 {
            let mut sum = if row == 3 { outer[9 + column] } else { 0.0 };
            for k in 0..3 {
                sum += inner[row * 3 + k] * outer[k * 3 + column];
            }
            out[row * 3 + column] = sum;
        }
    }
    out
}

/// The determinant of the transform's linear part; negative means the instance
/// is mirrored and its triangle winding has to be reversed.
fn determinant(transform: &[f64; 12]) -> f64 {
    let m = transform;
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6])
}

/// Read a 3MF package: locate the model part through the OPC relationships,
/// parse the core model, and flatten the build into a millimetre triangle soup.
///
/// See the module documentation for what is read, what is ignored, and the
/// bounds every part is held to.
pub fn read_3mf(bytes: &[u8]) -> Result<ThreeMfReadResult, ThreeMfError> {
    let archive = zip::ZipArchive::open(bytes).map_err(map_zip)?;
    let relationships = archive.find(RELATIONSHIP_PART).ok_or_else(|| {
        ThreeMfError::MissingPart {
            part: RELATIONSHIP_PART.into(),
            detail: "an OPC package must carry the package relationships part".into(),
        }
    })?;
    let relationships_text = read_text(&archive, relationships, RELATIONSHIP_PART)?;
    let model_part = start_part(&relationships_text)?;
    let model_entry = archive.find(&model_part).ok_or_else(|| ThreeMfError::MissingPart {
        part: model_part.clone(),
        detail: format!("'{RELATIONSHIP_PART}' names it as the 3D model start part"),
    })?;
    let model_text = read_text(&archive, model_entry, &model_part)?;
    let document = model::parse_model(&model_part, &model_text)?;
    flatten(model_part, document)
}

/// Map a container fault onto the typed refusal, keeping the part name.
fn map_zip(error: zip::ZipError) -> ThreeMfError {
    let part = error.entry.clone().unwrap_or_else(|| "[package]".into());
    match error.fault {
        zip::ZipFault::Container => match &error.entry {
            Some(entry) => ThreeMfError::Container(format!("{entry}: {}", error.message)),
            None => ThreeMfError::Container(error.message),
        },
        zip::ZipFault::Damaged => ThreeMfError::Damaged { part, detail: error.message },
        zip::ZipFault::Unsupported => ThreeMfError::Unsupported { part, detail: error.message },
        zip::ZipFault::Bounds => ThreeMfError::Bounds { part, detail: error.message },
    }
}

fn read_text(
    archive: &zip::ZipArchive<'_>,
    entry: &zip::ZipEntry,
    part: &str,
) -> Result<String, ThreeMfError> {
    let bytes = archive.read(entry, MAX_PART_BYTES).map_err(map_zip)?;
    String::from_utf8(bytes).map_err(|_| ThreeMfError::Xml {
        part: part.to_owned(),
        detail: "the part is not UTF-8 text".into(),
    })
}

/// The `Target` of the package's 3D-model start-part relationship.
fn start_part(text: &str) -> Result<String, ThreeMfError> {
    let mut reader = xml::XmlReader::new(text);
    loop {
        let event = reader.next().map_err(|error| ThreeMfError::Xml {
            part: RELATIONSHIP_PART.into(),
            detail: error.to_string(),
        })?;
        match event {
            xml::Event::Start(tag) => {
                if tag.namespace != RELATIONSHIP_NAMESPACE {
                    reader.skip_subtree(&tag).map_err(|error| ThreeMfError::Xml {
                        part: RELATIONSHIP_PART.into(),
                        detail: error.to_string(),
                    })?;
                    continue;
                }
                if tag.name != "Relationship" {
                    continue;
                }
                if tag.attribute("Type") != Some(MODEL_RELATIONSHIP) {
                    continue;
                }
                let target = tag.attribute("Target").ok_or_else(|| ThreeMfError::Model {
                    part: RELATIONSHIP_PART.into(),
                    detail: "the 3D model relationship has no Target".into(),
                })?;
                let target = target.trim_start_matches('/');
                if target.is_empty() {
                    return Err(ThreeMfError::Model {
                        part: RELATIONSHIP_PART.into(),
                        detail: "the 3D model relationship has an empty Target".into(),
                    });
                }
                return Ok(target.to_owned());
            }
            xml::Event::End { .. } => {}
            xml::Event::Eof => break,
        }
    }
    Err(ThreeMfError::MissingPart {
        part: RELATIONSHIP_PART.into(),
        detail: format!("no relationship of type '{MODEL_RELATIONSHIP}' names a 3D model part"),
    })
}

/// Walk the build, composing transforms, into instances and one triangle soup.
fn flatten(
    model_part: String,
    document: model::ModelDocument,
) -> Result<ThreeMfReadResult, ThreeMfError> {
    let scale = document.unit.millimetres();
    let meshes = document
        .meshes
        .iter()
        .map(|mesh| ThreeMfMesh {
            object_id: mesh.object_id,
            positions: mesh.positions.iter().map(|value| value * scale).collect(),
            indices: mesh.indices.clone(),
        })
        .collect::<Vec<_>>();
    let mut instances = Vec::new();
    for item in &document.items {
        let mut transform = item.transform;
        for value in &mut transform[9..12] {
            *value *= scale;
        }
        place(&document, &model_part, item.object_id, transform, 0, None, &mut instances)?;
    }
    let mut positions = Vec::new();
    let mut indices = Vec::new();
    let mut triangles = 0usize;
    for instance in &mut instances {
        let mesh = &meshes[instance.mesh];
        triangles += mesh.triangle_count();
        if triangles > MAX_TOTAL_TRIANGLES {
            return Err(ThreeMfError::Bounds {
                part: model_part,
                detail: format!(
                    "the build places more than {MAX_TOTAL_TRIANGLES} triangles in total"
                ),
            });
        }
        instance.first_vertex = positions.len() / 3;
        instance.vertex_count = mesh.vertex_count();
        instance.first_triangle = indices.len() / 3;
        instance.triangle_count = mesh.triangle_count();
        let base = instance.first_vertex as u32;
        for point in mesh.positions.chunks_exact(3) {
            let placed = instance.transform_point([point[0], point[1], point[2]]);
            positions.extend(placed);
        }
        // A mirrored placement turns the mesh inside out; reversing the winding
        // keeps every triangle facing out of the body, which is what the
        // mesh-to-solid chain reads.
        let mirrored = determinant(&instance.transform) < 0.0;
        for triangle in mesh.indices.chunks_exact(3) {
            if mirrored {
                indices.extend([base + triangle[0], base + triangle[2], base + triangle[1]]);
            } else {
                indices.extend([base + triangle[0], base + triangle[1], base + triangle[2]]);
            }
        }
    }
    Ok(ThreeMfReadResult {
        model_part,
        unit: document.unit,
        meshes,
        instances,
        positions,
        indices,
    })
}

/// Place `object_id` under `transform`, recursing through `<components>`.
fn place(
    document: &model::ModelDocument,
    model_part: &str,
    object_id: u32,
    transform: [f64; 12],
    depth: usize,
    inherited_name: Option<&str>,
    instances: &mut Vec<ThreeMfInstance>,
) -> Result<(), ThreeMfError> {
    if depth > MAX_COMPONENT_DEPTH {
        return Err(ThreeMfError::Bounds {
            part: model_part.to_owned(),
            detail: format!("components nest deeper than {MAX_COMPONENT_DEPTH} levels"),
        });
    }
    let object = document.object(object_id).ok_or_else(|| ThreeMfError::Model {
        part: model_part.to_owned(),
        detail: format!("object {object_id} is placed but never defined"),
    })?;
    let name = object.name.as_deref().or(inherited_name);
    match &object.content {
        model::ObjectContent::Mesh(mesh) => {
            // Only PRINTABLE geometry reaches the soup: support, surface and
            // "other" objects describe the print job, not the part.
            if object.model_type {
                if instances.len() + 1 > MAX_INSTANCES {
                    return Err(ThreeMfError::Bounds {
                        part: model_part.to_owned(),
                        detail: format!("the build places more than {MAX_INSTANCES} instances"),
                    });
                }
                instances.push(ThreeMfInstance {
                    object_id: object.id,
                    mesh: *mesh,
                    transform,
                    // Filled by `flatten` when the soup is laid out.
                    first_vertex: 0,
                    vertex_count: 0,
                    first_triangle: 0,
                    triangle_count: 0,
                    name: name.map(str::to_owned),
                });
            }
            Ok(())
        }
        model::ObjectContent::Components(components) => {
            for component in components {
                let mut inner = component.transform;
                // The component's translation is in the file's unit; the
                // enclosing transform is already in millimetres, so scaling
                // here (not in `flatten`) keeps one conversion per level.
                let scale = document.unit.millimetres();
                for value in &mut inner[9..12] {
                    *value *= scale;
                }
                let composed = compose(&inner, &transform);
                place(
                    document,
                    model_part,
                    component.object_id,
                    composed,
                    depth + 1,
                    name,
                    instances,
                )?;
            }
            Ok(())
        }
    }
}
