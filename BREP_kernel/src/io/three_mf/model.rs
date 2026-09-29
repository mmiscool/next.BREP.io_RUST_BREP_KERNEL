//! The 3MF core model part: `<model>` → `<resources>` → `<object>` → `<mesh>` /
//! `<components>`, and `<build>` → `<item>`.
//!
//! Every element is matched by NAMESPACE and local name, so an extension that
//! reuses a core name (or a core element written through a prefix) is
//! classified correctly. Non-core subtrees are skipped whole — see the module
//! doc of [`super`] for what that deliberately discards.

use super::xml::{Event, StartTag, XmlReader};
use super::{ThreeMfError, ThreeMfUnit, CORE_NAMESPACE, IDENTITY, MAX_MESH_TRIANGLES,
    MAX_MESH_VERTICES};
use rustc_hash::FxHashMap;

/// One `<object>`'s `<mesh>`, in the file's own unit.
pub(super) struct RawMesh {
    pub object_id: u32,
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
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
}

impl ModelDocument {
    pub fn object(&self, id: u32) -> Option<&Object> {
        self.by_id.get(&id).map(|index| &self.objects[*index])
    }
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
        let required = required.trim();
        if !required.is_empty() {
            return Err(ThreeMfError::Unsupported {
                part: part.to_owned(),
                detail: format!(
                    "the model requires the extension(s) '{required}'; this reader implements \
                     the core specification only"
                ),
            });
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
                if tag.namespace != CORE_NAMESPACE {
                    parser.skip(&tag)?;
                    continue;
                }
                match tag.name.as_str() {
                    "object" => parse_object(parser, &tag, document)?,
                    // <basematerials> and the colour groups are properties, not
                    // geometry: ignored by design.
                    _ => parser.skip(&tag)?,
                }
            }
            Event::End { name } if name == "resources" => return Ok(()),
            Event::End { .. } => {}
            Event::Eof => return parser.fail("the part ends inside <resources>"),
        }
    }
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
                        let mesh = parse_mesh(parser, &tag, id, model_type)?;
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
) -> Result<RawMesh, ThreeMfError> {
    let mut positions = Vec::<f64>::new();
    let mut indices = Vec::<u32>::new();
    if !mesh.empty {
        loop {
            match parser.next()? {
                Event::Start(tag) => {
                    if tag.namespace != CORE_NAMESPACE {
                        parser.skip(&tag)?;
                        continue;
                    }
                    match tag.name.as_str() {
                        "vertices" => parse_vertices(parser, &tag, object_id, &mut positions)?,
                        "triangles" => parse_triangles(parser, &tag, object_id, &mut indices)?,
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
    if model_type && (vertex_count < 3 || indices.is_empty()) {
        return parser.fail(format!(
            "object {object_id} is a model object whose mesh has {vertex_count} vertices and {} \
             triangles",
            indices.len() / 3
        ));
    }
    Ok(RawMesh { object_id, positions, indices })
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
    indices: &mut Vec<u32>,
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
                // v1/v2/v3 are the geometry; p1/p2/p3 and pid are material
                // properties and are ignored by design.
                let v1 = parser.index(&tag, "v1")?;
                let v2 = parser.index(&tag, "v2")?;
                let v3 = parser.index(&tag, "v3")?;
                indices.extend([v1, v2, v3]);
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
