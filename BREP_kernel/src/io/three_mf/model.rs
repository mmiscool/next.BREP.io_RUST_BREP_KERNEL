//! The 3MF core model part: `<model>` → `<resources>` → `<object>` → `<mesh>` /
//! `<components>`, and `<build>` → `<item>`.
//!
//! Every element is matched by NAMESPACE and local name, so an extension that
//! reuses a core name (or a core element written through a prefix) is
//! classified correctly. Supported materials are decoded; other extension subtrees are skipped — see the module
//! doc of [`super`] for what that deliberately discards.

use super::xml::{Event, StartTag, XmlReader};
use super::{ThreeMfError, ThreeMfUnit, CORE_NAMESPACE, IDENTITY, MAX_MESH_TRIANGLES,
    MAX_MESH_VERTICES};
use rustc_hash::FxHashMap;

/// One `<object>`'s `<mesh>`, in the file's own unit.
pub(super) struct RawMesh {
    pub object_id: u32,
    pub beam_lattice: Option<super::ThreeMfBeamLattice>,
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
    /// The object's own `pid`/`pindex`: the property every triangle inherits
    /// unless it states one.
    pub object_property: Option<Property>,
    /// One entry per triangle: its own `pid`/`p1`, or `None` to inherit.
    /// Resolved against [`ModelDocument::materials`] once the whole part is
    /// read, since the specification lets a group follow the objects that
    /// name it.
    pub properties: Vec<Option<Property>>,
}

/// A property reference as a triangle or object states it: the group id and
/// the indices into that group, in triangle corner order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Property {
    pub group: u32,
    pub index: u32,
    pub corners: [u32; 3],
}

pub(super) struct Component {
    pub object_id: u32,
    pub transform: [f64; 12],
}

pub(super) enum ObjectContent {
    /// Index into [`ModelDocument::meshes`].
    Mesh(usize),
    Components(Vec<Component>),
}

pub(super) struct Object {
    pub id: u32,
    /// `type="model"` (the default): printable geometry. The other types
    /// (`support`, `solidsupport`, `surface`, `other`) are resources of the
    /// print job and are not placed.
    pub model_type: bool,
    /// The `name` attribute, for naming the object in a refusal.
    pub name: Option<String>,
    pub content: ObjectContent,
}

pub(super) struct Item {
    pub object_id: u32,
    pub transform: [f64; 12],
}

pub(super) struct ModelDocument {
    pub unit: ThreeMfUnit,
    pub meshes: Vec<RawMesh>,
    pub objects: Vec<Object>,
    pub by_id: FxHashMap<u32, usize>,
    pub items: Vec<Item>,
    /// Every `<basematerials>` group by id: each `<base>`'s `displaycolor` as
    /// 8-bit sRGB, or `None` for one that does not spell `#RRGGBB[AA]`.
    pub materials: FxHashMap<u32, Vec<Option<[u8; 3]>>>,
    pub composites: FxHashMap<u32, (u32, Vec<u32>, Vec<Vec<f64>>)>,
    pub colors: FxHashMap<u32, Vec<[u8; 4]>>,
    pub textures: Vec<super::ThreeMfTexture>,
    pub texture_groups: FxHashMap<u32, (u32, Vec<[f64; 2]>)>,
}

impl ModelDocument {
    pub fn object(&self, id: u32) -> Option<&Object> {
        self.by_id.get(&id).map(|index| &self.objects[*index])
    }

    /// The colour a property reference names, if it names a base material.
    pub fn color(&self, property: Option<Property>) -> Option<[u8; 3]> {
        let property = property?;
        self.materials.get(&property.group)?.get(property.index as usize).copied().flatten()
    }
}

/// `#RRGGBB` or `#RRGGBBAA` (the alpha is dropped), either case. Anything
/// else — a named colour, a short form — is `None`: the base is kept so the
/// indices of its neighbours stay right, it just carries no colour.
pub(super) fn parse_display_color(text: &str) -> Option<[u8; 3]> {
    let hex = text.trim().strip_prefix('#')?;
    if hex.len() != 6 && hex.len() != 8 {
        return None;
    }
    let channel = |at: usize| u8::from_str_radix(hex.get(at..at + 2)?, 16).ok();
    Some([channel(0)?, channel(2)?, channel(4)?])
}

/// The reader plus the part name every error is reported against.
struct ModelParser<'a> {
    part: &'a str,
    reader: XmlReader<'a>,
}

impl<'a> ModelParser<'a> {
    fn fail<T>(&self, detail: impl Into<String>) -> Result<T, ThreeMfError> {
        Err(ThreeMfError::Model { part: self.part.to_owned(), detail: detail.into() })
    }

    fn next(&mut self) -> Result<Event, ThreeMfError> {
        self.reader.next().map_err(|error| ThreeMfError::Xml {
            part: self.part.to_owned(),
            detail: error.to_string(),
        })
    }

    fn skip(&mut self, tag: &StartTag) -> Result<(), ThreeMfError> {
        self.reader.skip_subtree(tag).map_err(|error| ThreeMfError::Xml {
            part: self.part.to_owned(),
            detail: error.to_string(),
        })
    }

    /// Consume a leaf element's end tag when it was not written `<tag/>`.
    fn close_leaf(&mut self, tag: &StartTag) -> Result<(), ThreeMfError> {
        if tag.empty {
            Ok(())
        } else {
            self.skip(tag)
        }
    }

    /// A required attribute, with the element named in the refusal.
    fn required<'t>(&self, tag: &'t StartTag, name: &str) -> Result<&'t str, ThreeMfError> {
        match tag.attribute(name) {
            Some(value) => Ok(value),
            None => Err(ThreeMfError::Model {
                part: self.part.to_owned(),
                detail: format!("<{}> has no {name} attribute", tag.name),
            }),
        }
    }

    fn coordinate(&self, tag: &StartTag, name: &str) -> Result<f64, ThreeMfError> {
        let text = self.required(tag, name)?;
        match text.trim().parse::<f64>() {
            Ok(value) if value.is_finite() => Ok(value),
            _ => self.fail(format!("<{}> has a non-finite {name}=\"{text}\"", tag.name)),
        }
    }

    fn identifier(&self, tag: &StartTag, name: &str) -> Result<u32, ThreeMfError> {
        let text = self.required(tag, name)?;
        match text.trim().parse::<u32>() {
            Ok(value) if value > 0 => Ok(value),
            _ => self.fail(format!("<{}> has an invalid {name}=\"{text}\"", tag.name)),
        }
    }

    fn index(&self, tag: &StartTag, name: &str) -> Result<u32, ThreeMfError> {
        let text = self.required(tag, name)?;
        match text.trim().parse::<u32>() {
            Ok(value) => Ok(value),
            _ => self.fail(format!("<{}> has an invalid {name}=\"{text}\"", tag.name)),
        }
    }

    /// A property reference (`pid` + `pindex`, or `pid` + `p1`): `None` when
    /// the element states none. A malformed one is `None` too, never a
    /// refusal — a property is appearance, not geometry.
    fn property(&self, tag: &StartTag, index_name: &str) -> Option<Property> {
        let group = tag.attribute("pid")?.trim().parse::<u32>().ok()?;
        let index = tag.attribute(index_name)?.trim().parse::<u32>().ok()?;
        Some(Property { group, index, corners: [index, index, index] })
    }

    /// The optional 3x4 `transform`, defaulting to the identity.
    fn transform(&self, tag: &StartTag) -> Result<[f64; 12], ThreeMfError> {
        let Some(text) = tag.attribute("transform") else {
            return Ok(IDENTITY);
        };
        let mut values = [0.0; 12];
        let mut count = 0usize;
        for field in text.split_ascii_whitespace() {
            if count == 12 {
                return self.fail(format!(
                    "<{}> transform has more than 12 numbers",
                    tag.name
                ));
            }
            match field.parse::<f64>() {
                Ok(value) if value.is_finite() => values[count] = value,
                _ => {
                    return self.fail(format!(
                        "<{}> transform has a non-finite value \"{field}\"",
                        tag.name
                    ))
                }
            }
            count += 1;
        }
        if count != 12 {
            return self.fail(format!(
                "<{}> transform has {count} numbers, not 12",
                tag.name
            ));
        }
        Ok(values)
    }
}

/// Parse a 3MF core model part.
pub(super) fn parse_model(part: &str, text: &str) -> Result<ModelDocument, ThreeMfError> {
    let mut parser = ModelParser { part, reader: XmlReader::new(text) };
    // The root: the first element in the document must be a core <model>.
    let root = loop {
        match parser.next()? {
            Event::Start(tag) => break tag,
            Event::End { .. } => {}
            Event::Eof => {
                return parser.fail("the part contains no <model> element");
            }
        }
    };
    if root.namespace != CORE_NAMESPACE || root.name != "model" {
        return parser.fail(format!(
            "the root element is <{}> in namespace '{}', not a 3MF core <model>",
            root.name, root.namespace
        ));
    }
    let unit = match root.attribute("unit") {
        None => ThreeMfUnit::Millimeter,
        Some(text) => match ThreeMfUnit::parse(text) {
            Some(unit) => unit,
            None => return parser.fail(format!("<model> declares an unknown unit=\"{text}\"")),
        },
    };
    if let Some(required) = root.attribute("requiredextensions") {
        for prefix in required.split_whitespace() {
            let namespace = parser.reader.resolve(prefix).unwrap_or_default();
            if namespace != super::MATERIAL_NAMESPACE && namespace != super::BEAM_NAMESPACE {
                return Err(ThreeMfError::Unsupported {
                    part: part.to_owned(),
                    detail: format!("the model requires the extension '{prefix}' ({namespace}), which is unsupported"),
                });
            }
        }
    }

    if root.empty {
        return parser.fail("<model> has no <resources> or <build>");
    }

    let mut document = ModelDocument {
        unit,
        meshes: Vec::new(),
        objects: Vec::new(),
        by_id: FxHashMap::default(),
        items: Vec::new(),
        materials: FxHashMap::default(),
        composites: FxHashMap::default(),
        colors: FxHashMap::default(),
        textures: Vec::new(),
        texture_groups: FxHashMap::default(),
    };
    let mut saw_build = false;
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                match tag.name.as_str() {
                    "resources" => parse_resources(&mut parser, &tag, &mut document)?,
                    "build" => {
                        saw_build = true;
                        parse_build(&mut parser, &tag, &mut document)?;
                    }
                    // <metadata>, <metadatagroup>: no geometry.
                    _ => parser.skip(&tag)?,
                }
            }
            Event::End { name } if name == "model" => break,
            Event::End { .. } => {}
            Event::Eof => return parser.fail("the part ends inside <model>"),
        }
    }
    if !saw_build {
        return parser.fail("<model> has no <build> element");
    }
    if document.items.is_empty() {
        return parser.fail("<build> places no items, so the file has no geometry to import");
    }
    Ok(document)
}

fn parse_resources(
    parser: &mut ModelParser<'_>,
    resources: &StartTag,
    document: &mut ModelDocument,
) -> Result<(), ThreeMfError> {
    if resources.empty {
        return Ok(());
    }
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace == super::MATERIAL_NAMESPACE {
                    parse_material_extension(parser, &tag, document)?;
                    continue;
                }
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                match tag.name.as_str() {
                    "object" => parse_object(parser, &tag, document)?,
                    // The core specification's one property group. The
                    // extension's colour groups, textures and composites are
                    // in another namespace and were skipped above.
                    "basematerials" => parse_base_materials(parser, &tag, document)?,
                    _ => parser.skip(&tag)?,
                }
            }
            Event::End { name } if name == "resources" => return Ok(()),
            Event::End { .. } => {}
            Event::Eof => return parser.fail("the part ends inside <resources>"),
        }
    }
}

/// `<basematerials id>` → its `<base displaycolor>`s, in order. A group with
/// an unreadable id is skipped (no triangle can name it); a repeated id keeps
/// the first definition.
fn parse_base_materials(
    parser: &mut ModelParser<'_>,
    group: &StartTag,
    document: &mut ModelDocument,
) -> Result<(), ThreeMfError> {
    let id = group.attribute("id").and_then(|text| text.trim().parse::<u32>().ok());
    let mut colors = Vec::new();
    if !group.empty {
        loop {
            match parser.next()? {
                Event::Start(tag) => {
                    if tag.namespace == CORE_NAMESPACE && tag.name == "base" {
                        colors.push(tag.attribute("displaycolor").and_then(parse_display_color));
                        parser.close_leaf(&tag)?;
                    } else {
                        parser.skip(&tag)?;
                    }
                }
                Event::End { name } if name == "basematerials" => break,
                Event::End { .. } => {}
                Event::Eof => return parser.fail("the part ends inside <basematerials>"),
            }
        }
    }
    if let Some(id) = id {
        document.materials.entry(id).or_insert(colors);
    }
    Ok(())
}

fn parse_object(
    parser: &mut ModelParser<'_>,
    object: &StartTag,
    document: &mut ModelDocument,
) -> Result<(), ThreeMfError> {
    let id = parser.identifier(object, "id")?;
    if document.by_id.contains_key(&id) {
        return parser.fail(format!("object id {id} is defined twice"));
    }
    let object_property = parser.property(object, "pindex");
    let model_type = match object.attribute("type") {
        None | Some("model") => true,
        Some("solidsupport" | "support" | "surface" | "other") => false,
        Some(other) => {
            return parser.fail(format!("object {id} has an unknown type=\"{other}\""))
        }
    };
    if object.empty {
        return parser.fail(format!("object {id} has neither <mesh> nor <components>"));
    }
    let mut content: Option<ObjectContent> = None;
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                match tag.name.as_str() {
                    "mesh" => {
                        if content.is_some() {
                            return parser
                                .fail(format!("object {id} has more than one geometry element"));
                        }
                        let mesh = parse_mesh(parser, &tag, id, model_type, object_property)?;
                        document.meshes.push(mesh);
                        content = Some(ObjectContent::Mesh(document.meshes.len() - 1));
                    }
                    "components" => {
                        if content.is_some() {
                            return parser
                                .fail(format!("object {id} has more than one geometry element"));
                        }
                        content = Some(ObjectContent::Components(parse_components(
                            parser, &tag, id, document,
                        )?));
                    }
                    _ => parser.skip(&tag)?,
                }
            }
            Event::End { name } if name == "object" => break,
            Event::End { .. } => {}
            Event::Eof => return parser.fail(format!("the part ends inside object {id}")),
        }
    }
    let Some(content) = content else {
        return parser.fail(format!("object {id} has neither <mesh> nor <components>"));
    };
    let name = object
        .attribute("name")
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .map(str::to_owned);
    document.objects.push(Object { id, model_type, name, content });
    document.by_id.insert(id, document.objects.len() - 1);
    Ok(())
}

fn parse_mesh(
    parser: &mut ModelParser<'_>,
    mesh: &StartTag,
    object_id: u32,
    model_type: bool,
    object_property: Option<Property>,
) -> Result<RawMesh, ThreeMfError> {
    let mut beam_lattice = None;
    let mut positions = Vec::<f64>::new();
    let mut indices = Vec::<u32>::new();
    let mut properties = Vec::<Option<Property>>::new();
    if !mesh.empty {
        loop {
            match parser.next()? {
                Event::Start(tag) => {
                    if tag.namespace == super::BEAM_NAMESPACE && tag.name == "beamlattice" {
                        if beam_lattice.is_some() { return parser.fail(format!("object {object_id} has duplicate beam lattices")); }
                        if object_property.is_some() {
                            return Err(ThreeMfError::Unsupported { part: parser.part.into(), detail: format!("beam lattice in object {object_id}: material properties on beams are unsupported") });
                        }
                        beam_lattice = Some(parse_beam_lattice(parser, &tag, object_id)?);
                        continue;
                    }
                    if tag.namespace != CORE_NAMESPACE {
                        parser.skip(&tag)?;
                        continue;
                    }
                    match tag.name.as_str() {
                        "vertices" => parse_vertices(parser, &tag, object_id, &mut positions)?,
                        "triangles" => {
                            parse_triangles(parser, &tag, object_id, object_property, &mut indices, &mut properties)?
                        }
                        // <beamlattice> is an extension; anything else here is
                        // not geometry the core spec defines.
                        _ => parser.skip(&tag)?,
                    }
                }
                Event::End { name } if name == "mesh" => break,
                Event::End { .. } => {}
                Event::Eof => {
                    return parser.fail(format!("the part ends inside object {object_id}'s <mesh>"))
                }
            }
        }
    }
    let vertex_count = positions.len() / 3;
    for (triangle, corners) in indices.chunks_exact(3).enumerate() {
        for corner in corners {
            if *corner as usize >= vertex_count {
                return parser.fail(format!(
                    "object {object_id} triangle {triangle} names vertex {corner}, but the mesh \
                     has {vertex_count} vertices"
                ));
            }
        }
        if corners[0] == corners[1] || corners[1] == corners[2] || corners[0] == corners[2] {
            return parser.fail(format!(
                "object {object_id} triangle {triangle} repeats a vertex index"
            ));
        }
    }
    // A model object is printable geometry; the core spec requires a mesh that
    // encloses volume, which an empty one cannot.
    if model_type && beam_lattice.is_none() && (vertex_count < 3 || indices.is_empty()) {
        return parser.fail(format!(
            "object {object_id} is a model object whose mesh has {vertex_count} vertices and {} \
             triangles",
            indices.len() / 3
        ));
    }
    if let Some(lattice) = &mut beam_lattice {
        for beam in &lattice.beams {
            if beam.vertices.iter().any(|v| *v as usize >= vertex_count) {
                return parser.fail(format!("beam lattice in object {object_id}: vertex index out of range"));
            }
        }
        // The extension requires consumers to ignore beams shorter than minlength.
        lattice.beams.retain(|beam| {
            let a = beam.vertices[0] as usize * 3;
            let b = beam.vertices[1] as usize * 3;
            (positions[a] - positions[b]).hypot(positions[a+1] - positions[b+1])
                .hypot(positions[a+2] - positions[b+2]) >= lattice.min_length
        });
    }
    Ok(RawMesh { object_id, beam_lattice, positions, indices, object_property, properties })
}

fn parse_vertices(
    parser: &mut ModelParser<'_>,
    vertices: &StartTag,
    object_id: u32,
    positions: &mut Vec<f64>,
) -> Result<(), ThreeMfError> {
    if vertices.empty {
        return Ok(());
    }
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                if tag.name != "vertex" {
                    parser.skip(&tag)?;
                    continue;
                }
                if positions.len() / 3 >= MAX_MESH_VERTICES {
                    return Err(ThreeMfError::Bounds {
                        part: parser.part.to_owned(),
                        detail: format!(
                            "object {object_id} declares more than {MAX_MESH_VERTICES} vertices"
                        ),
                    });
                }
                let x = parser.coordinate(&tag, "x")?;
                let y = parser.coordinate(&tag, "y")?;
                let z = parser.coordinate(&tag, "z")?;
                positions.extend([x, y, z]);
                parser.close_leaf(&tag)?;
            }
            Event::End { name } if name == "vertices" => return Ok(()),
            Event::End { .. } => {}
            Event::Eof => {
                return parser
                    .fail(format!("the part ends inside object {object_id}'s <vertices>"))
            }
        }
    }
}

fn parse_triangles(
    parser: &mut ModelParser<'_>,
    triangles: &StartTag,
    object_id: u32,
    object_property: Option<Property>,
    indices: &mut Vec<u32>,
    properties: &mut Vec<Option<Property>>,
) -> Result<(), ThreeMfError> {
    if triangles.empty {
        return Ok(());
    }
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                if tag.name != "triangle" {
                    parser.skip(&tag)?;
                    continue;
                }
                if indices.len() / 3 >= MAX_MESH_TRIANGLES {
                    return Err(ThreeMfError::Bounds {
                        part: parser.part.to_owned(),
                        detail: format!(
                            "object {object_id} declares more than {MAX_MESH_TRIANGLES} triangles"
                        ),
                    });
                }
                // Property corners follow the geometric corners; omitted
                // p2/p3 inherit p1. An omitted pid inherits the object's group.
                let v1 = parser.index(&tag, "v1")?;
                let v2 = parser.index(&tag, "v2")?;
                let v3 = parser.index(&tag, "v3")?;
                indices.extend([v1, v2, v3]);
                let mut property = parser.property(&tag, "p1");
                if tag.attribute("pid").is_none() && tag.attribute("p1").is_some() {
                    if let Some(default) = object_property {
                        let index = parser.index(&tag, "p1")?;
                        property = Some(Property { group: default.group, index, corners: [index; 3] });
                    }
                }
                if let Some(property) = &mut property {
                    for (corner, name) in [(1, "p2"), (2, "p3")] {
                        if tag.attribute(name).is_some() {
                            property.corners[corner] = parser.index(&tag, name)?;
                        }
                    }
                }
                properties.push(property);
                parser.close_leaf(&tag)?;
            }
            Event::End { name } if name == "triangles" => return Ok(()),
            Event::End { .. } => {}
            Event::Eof => {
                return parser
                    .fail(format!("the part ends inside object {object_id}'s <triangles>"))
            }
        }
    }
}

fn parse_components(
    parser: &mut ModelParser<'_>,
    components: &StartTag,
    object_id: u32,
    document: &ModelDocument,
) -> Result<Vec<Component>, ThreeMfError> {
    let mut out = Vec::new();
    if components.empty {
        return parser.fail(format!("object {object_id} has an empty <components>"));
    }
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                if tag.name != "component" {
                    parser.skip(&tag)?;
                    continue;
                }
                let referenced = parser.identifier(&tag, "objectid")?;
                // The core spec requires a component's object to be defined
                // BEFORE it is used, which is also what makes the resource
                // graph acyclic. A forward or self reference is refused here.
                if !document.by_id.contains_key(&referenced) {
                    return parser.fail(format!(
                        "object {object_id} has a component naming object {referenced}, which is \
                         not defined before it"
                    ));
                }
                let transform = parser.transform(&tag)?;
                out.push(Component { object_id: referenced, transform });
                parser.close_leaf(&tag)?;
            }
            Event::End { name } if name == "components" => break,
            Event::End { .. } => {}
            Event::Eof => {
                return parser
                    .fail(format!("the part ends inside object {object_id}'s <components>"))
            }
        }
    }
    if out.is_empty() {
        return parser.fail(format!("object {object_id} has an empty <components>"));
    }
    Ok(out)
}

fn parse_build(
    parser: &mut ModelParser<'_>,
    build: &StartTag,
    document: &mut ModelDocument,
) -> Result<(), ThreeMfError> {
    if build.empty {
        return Ok(());
    }
    loop {
        match parser.next()? {
            Event::Start(tag) => {
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                if tag.name != "item" {
                    parser.skip(&tag)?;
                    continue;
                }
                let object_id = parser.identifier(&tag, "objectid")?;
                if !document.by_id.contains_key(&object_id) {
                    return parser.fail(format!(
                        "<build> places object {object_id}, which <resources> does not define"
                    ));
                }
                let transform = parser.transform(&tag)?;
                document.items.push(Item { object_id, transform });
                parser.close_leaf(&tag)?;
            }
            Event::End { name } if name == "build" => return Ok(()),
            Event::End { .. } => {}
            Event::Eof => return parser.fail("the part ends inside <build>"),
        }
    }
}

fn parse_material_extension(
    parser: &mut ModelParser<'_>,
    tag: &StartTag,
    document: &mut ModelDocument,
) -> Result<(), ThreeMfError> {
    let id = parser.identifier(tag, "id")?;
    if document.colors.contains_key(&id) || document.texture_groups.contains_key(&id)
        || document.textures.iter().any(|t| t.id == id) || document.composites.contains_key(&id)
        || document.materials.contains_key(&id) || document.by_id.contains_key(&id) {
        return parser.fail(format!("duplicate material resource id {id}"));
    }
    if tag.name == "texture2d" {
        let path = parser.required(tag, "path")?.trim_start_matches('/').to_owned();
        if path.is_empty() || path.split('/').any(|s| s == ".." || s == "." || s.is_empty())
            || path.contains('\\') || path.contains(':') {
            return parser.fail(format!("texture2d {id} has an invalid package path"));
        }
        let content_type = parser.required(tag, "contenttype")?;
        if content_type != "image/png" {
            return Err(ThreeMfError::Unsupported { part: parser.part.into(),
                detail: format!("texture2d {id}: content type '{content_type}' is unsupported; expected packaged image/png") });
        }
        let tile_u = tag.attribute("tilestyleu").unwrap_or("wrap");
        let tile_v = tag.attribute("tilestylev").unwrap_or("wrap");
        let filter = tag.attribute("filter").unwrap_or("auto");
        if !["wrap", "mirror", "clamp", "none"].contains(&tile_u)
            || !["wrap", "mirror", "clamp", "none"].contains(&tile_v)
            || !["auto", "linear", "nearest"].contains(&filter) {
            return parser.fail(format!("texture2d {id} has invalid addressing or filter"));
        }
        document.textures.push(super::ThreeMfTexture { id, path, png: Vec::new(),
            tile_u: tile_u.into(), tile_v: tile_v.into(), filter: filter.into() });
        return parser.close_leaf(tag);
    }
    let mut colors = Vec::new();
    let mut coords = Vec::new();
    let mut mixtures = Vec::new();
    let texid = if tag.name == "texture2dgroup" { parser.identifier(tag, "texid")? } else { 0 };
    let matid = if tag.name == "compositematerials" { parser.identifier(tag, "matid")? } else { 0 };
    let constituents = if matid != 0 {
        parser.required(tag, "matindices")?.split_whitespace().map(|s| s.parse::<u32>())
            .collect::<Result<Vec<_>, _>>().map_err(|_| ThreeMfError::Model {
                part: parser.part.into(), detail: format!("compositematerials {id} has invalid matindices") })?
    } else { Vec::new() };
    if !["colorgroup", "texture2dgroup", "compositematerials"].contains(&tag.name.as_str()) {
        return Err(ThreeMfError::Unsupported { part: parser.part.into(),
            detail: format!("materials extension resource <{}> is unsupported", tag.name) });
    }
    if !tag.empty {
        loop {
            match parser.next()? {
                Event::Start(child) => {
                    if child.namespace != super::MATERIAL_NAMESPACE { parser.skip(&child)?; continue; }
                    match (tag.name.as_str(), child.name.as_str()) {
                        ("colorgroup", "color") => {
                            let value = parser.required(&child, "color")?.trim();
                            let Some(rgb) = parse_display_color(value) else {
                                return parser.fail(format!("colorgroup {id} has invalid color '{value}'"));
                            };
                            let alpha = if value.len() == 9 { u8::from_str_radix(&value[7..9], 16).ok() } else { Some(255) };
                            let Some(alpha) = alpha else { return parser.fail("invalid color alpha"); };
                            colors.push([rgb[0], rgb[1], rgb[2], alpha]);
                        }
                        ("texture2dgroup", "tex2coord") => coords.push([parser.coordinate(&child, "u")?, parser.coordinate(&child, "v")?]),
                        ("compositematerials", "composite") => {
                            let mut weights = parser.required(&child, "values")?.split_whitespace()
                                .take(constituents.len()).map(|s| s.parse::<f64>()).collect::<Result<Vec<_>, _>>()
                                .map_err(|_| ThreeMfError::Model { part: parser.part.into(), detail: "invalid composite values".into() })?;
                            weights.resize(constituents.len(), 0.0);
                            if constituents.len() < 2
                                || weights.iter().any(|v| !v.is_finite() || *v < 0.0 || *v > 1.0) {
                                return parser.fail(format!("compositematerials {id} has invalid fractions"));
                            }
                            mixtures.push(weights);
                        }
                        _ => return parser.fail(format!("unexpected <{}> in {}", child.name, tag.name)),
                    }
                    if colors.len() + coords.len() + mixtures.len() > MAX_MESH_TRIANGLES {
                        return Err(ThreeMfError::Bounds { part: parser.part.into(), detail: "too many material properties".into() });
                    }
                    parser.close_leaf(&child)?;
                }
                Event::End { name } if name == tag.name => break,
                Event::End { .. } => {},
                Event::Eof => return parser.fail("part ends inside material resource"),
            }
        }
    }
    match tag.name.as_str() {
        "colorgroup" => { document.colors.insert(id, colors); }
        "texture2dgroup" => { document.texture_groups.insert(id, (texid, coords)); }
        _ => { document.composites.insert(id, (matid, constituents, mixtures)); }
    }
    Ok(())
}

impl ModelDocument {
    pub fn appearance(&self, property: Option<Property>, part: &str) -> Result<Option<super::ThreeMfAppearance>, ThreeMfError> {
        use super::ThreeMfAppearance;
        let Some(p) = property else { return Ok(None); };
        let invalid = || ThreeMfError::Model { part: part.into(), detail: format!("invalid property reference in material group {}", p.group) };
        if let Some(colors) = self.colors.get(&p.group) {
            let mut out = [[0; 4]; 3];
            for i in 0..3 { out[i] = *colors.get(p.corners[i] as usize).ok_or_else(invalid)?; }
            return Ok(Some(ThreeMfAppearance::Colors(out)));
        }
        if let Some((texture_id, coords)) = self.texture_groups.get(&p.group) {
            if !self.textures.iter().any(|t| t.id == *texture_id) { return Err(invalid()); }
            let mut uv = [[0.0; 2]; 3];
            for i in 0..3 { uv[i] = *coords.get(p.corners[i] as usize).ok_or_else(invalid)?; }
            return Ok(Some(ThreeMfAppearance::Texture { texture_id: *texture_id, uv }));
        }
        if let Some((matid, indices, mixtures)) = self.composites.get(&p.group) {
            let base = self.materials.get(matid).ok_or_else(invalid)?;
            let weights = [
                mixtures.get(p.corners[0] as usize).ok_or_else(invalid)?.clone(),
                mixtures.get(p.corners[1] as usize).ok_or_else(invalid)?.clone(),
                mixtures.get(p.corners[2] as usize).ok_or_else(invalid)?.clone(),
            ];
            let colors = indices.iter().map(|i| base.get(*i as usize).copied().flatten().ok_or_else(invalid)).collect::<Result<Vec<_>, _>>()?;
            return Ok(Some(ThreeMfAppearance::Composite { colors, weights }));
        }
        Ok(self.color(property).map(|[r,g,b]| ThreeMfAppearance::Colors([[r,g,b,255]; 3])))
    }
}

fn parse_beam_lattice(parser: &mut ModelParser<'_>, tag: &StartTag, object_id: u32) -> Result<super::ThreeMfBeamLattice, ThreeMfError> {
    let unsupported = |reason: &str| ThreeMfError::Unsupported { part: parser.part.into(),
        detail: format!("beam lattice in object {object_id}: {reason}") };
    if tag.attribute("clippingmode").unwrap_or("none") != "none" {
        return Err(unsupported("clipping is unsupported for non-solid lattice data"));
    }
    if tag.attribute("ballmode").unwrap_or("none") != "none" {
        return Err(unsupported("ball modes are unsupported for non-solid lattice data"));
    }
    for attribute in ["representationmesh", "clippingmesh", "pid", "pindex"] {
        if tag.attribute(attribute).is_some() { return Err(unsupported(&format!("{attribute} is unsupported"))); }
    }
    let radius = parser.coordinate(tag, "radius")?;
    let min_length = parser.coordinate(tag, "minlength")?;
    let cap = tag.attribute("cap").unwrap_or("sphere");
    if radius <= 0.0 || min_length <= 0.0 || !["sphere", "hemisphere", "butt"].contains(&cap) {
        return parser.fail(format!("beam lattice in object {object_id}: invalid radius, minlength or cap"));
    }
    let mut beams = Vec::new();
    if !tag.empty {
        loop {
            match parser.next()? {
                Event::Start(group) => {
                    if group.namespace != super::BEAM_NAMESPACE || group.name != "beams" {
                        return Err(ThreeMfError::Unsupported { part: parser.part.into(), detail: format!("beam lattice in object {object_id}: <{}> is unsupported", group.name) });
                    }
                    if group.empty { continue; }
                    loop {
                        match parser.next()? {
                            Event::Start(beam) => {
                                if beam.namespace != super::BEAM_NAMESPACE || beam.name != "beam" {
                                    return parser.fail(format!("beam lattice in object {object_id}: expected <beam>"));
                                }
                                if beam.attribute("p1").is_some() || beam.attribute("p2").is_some() || beam.attribute("pid").is_some() {
                                    return Err(ThreeMfError::Unsupported { part: parser.part.into(), detail: format!("beam lattice in object {object_id}: beam properties are unsupported") });
                                }
                                let r1 = if beam.attribute("r1").is_some() { parser.coordinate(&beam, "r1")? } else { radius };
                                let r2 = if beam.attribute("r2").is_some() {
                                    if beam.attribute("r1").is_none() { return parser.fail("beam r2 requires r1"); }
                                    parser.coordinate(&beam, "r2")?
                                } else { r1 };
                                let c1 = beam.attribute("cap1").unwrap_or(cap);
                                let c2 = beam.attribute("cap2").unwrap_or(cap);
                                if r1 <= 0.0 || r2 <= 0.0 || !["sphere", "hemisphere", "butt"].contains(&c1) || !["sphere", "hemisphere", "butt"].contains(&c2) {
                                    return parser.fail(format!("beam lattice in object {object_id}: invalid radius or cap"));
                                }
                                if beams.len() >= MAX_MESH_TRIANGLES { return Err(ThreeMfError::Bounds { part: parser.part.into(), detail: "too many beams".into() }); }
                                beams.push(super::ThreeMfBeam { vertices: [parser.index(&beam, "v1")?, parser.index(&beam, "v2")?], radii: [r1, r2], caps: [c1.into(), c2.into()] });
                                parser.close_leaf(&beam)?;
                            }
                            Event::End { name } if name == "beams" => break,
                            Event::End { .. } => {},
                            Event::Eof => return parser.fail("part ends inside beams"),
                        }
                    }
                }
                Event::End { name } if name == "beamlattice" => break,
                Event::End { .. } => {},
                Event::Eof => return parser.fail("part ends inside beam lattice"),
            }
        }
    }
    Ok(super::ThreeMfBeamLattice { min_length, beams })
}
