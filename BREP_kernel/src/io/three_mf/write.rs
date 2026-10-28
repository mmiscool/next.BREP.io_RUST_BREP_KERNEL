//! 3MF (3D Manufacturing Format) EXPORT of display meshes — the write half of
//! this module, for slicers and print services. Like the GLB and STL lanes it
//! writes tessellated triangles, not the exact B-rep: no surface, topology or
//! feature tree leaves here.
//!
//! # The package
//!
//! A 3MF is an OPC ZIP container with three parts: `[Content_Types].xml`,
//! `_rels/.rels` (the relationship that names the start part) and the model
//! part `3D/3dmodel.model`. Every entry is written STORED (method 0) with a
//! fixed timestamp, so the archive is a plain header + bytes + central
//! directory and the same input produces the same file on every run. Stored
//! rather than deflated on purpose: the kernel's deflate side is a decoder
//! only (`inflate.rs`), the dependency review in [`super`] rules out pulling a
//! compressor in for one lane, and a slicer reads either. The cost is size —
//! roughly the model part's XML, three to four times a binary STL of the same
//! triangles.
//!
//! # The model part
//!
//! The 3MF Core Specification 1.x only — the same namespace [`super::read_3mf`]
//! reads, and no `requiredextensions`:
//!
//! * `<model unit="millimeter">`: every export lane writes millimetres.
//! * ONE `<basematerials id="1">` holding every distinct colour in the file as
//!   a `<base name="color_rrggbb" displaycolor="#RRGGBB"/>`, deduplicated
//!   across bodies, in first-seen order. `<basematerials>` is a CORE element
//!   (the materials-and-properties extension adds colour groups, textures and
//!   composites on top of it), which is why a reader that knows only the core
//!   specification — a slicer, Windows' viewer, lib3mf, this module's own
//!   reader — sees the colours.
//! * One `<object type="model">` per solid, named by its scene name, carrying
//!   the body colour as `pid`/`pindex`; a triangle whose colour differs from
//!   the body's (a face the document painted) carries its own `pid`/`p1`. Only
//!   `p1` is ever written: a base material is one colour per triangle, and the
//!   specification requires `p2`/`p3` to match it when present.
//! * One `<build>` `<item>` per object, at the identity — the display mesh is
//!   already in world coordinates.
//!
//! A solid with no colour at all is written with no `pid`, which the format
//! defines as "no material": the slicer shows it in its own filament colour
//! instead of a grey this writer invented. (The GLB lane paints such a body in
//! the viewport's colour because glTF has no such state; 3MF does.)
//!
//! # Slicer painting, beside the base materials
//!
//! The slicers people print with do not read base materials: PrusaSlicer has
//! ignored them since 2020 (its issue #4503) and shows colour only as
//! multi-material PAINTING — per-triangle extruder assignments it stores on
//! the `<triangle>` as `slic3rpe:mmu_segmentation` (namespace
//! `http://schemas.slic3r.org/3mf/2017/06`), or, since 2.7.4, Bambu Studio's
//! (and its forks') `paint_color`, which is the same bitstream under the name
//! that fork chose. So every coloured triangle also carries both attributes:
//! each distinct colour, in first-seen order, becomes an extruder slot, up to
//! [`MAX_PAINT_SLOTS`] (the format stores a state in four bits and
//! PrusaSlicer's `TriangleStateType` stops at `Extruder15`); a colour past
//! that is still a base material but not painted. An uncoloured triangle has
//! no painting and takes the object's default extruder — which is extruder 1.
//! So when the file has any uncoloured triangle the colours start at slot 2:
//! otherwise the one painted face of an unpainted body would land on the same
//! extruder as the body and look exactly as it did before. A file whose every
//! triangle is coloured starts at slot 1.
//!
//! The string's layout was taken from how PrusaSlicer 2.9.2 reads it
//! (`FacetsAnnotation::set_triangle_from_string`, `TriangleSelector::deserialize`
//! — format references only; the encoder below is this module's own): hex
//! nibbles, the LAST
//! character holding the FIRST nibble, each nibble's bits least significant
//! first. An unsplit triangle is `split sides = 0` in two bits, then its
//! state: a state below 3 in two bits (so slot 1 is `4`, slot 2 is `8`), a
//! state from 3 up as the two bits `11` followed by a nibble of `state - 3`
//! (slot 3 is `0C`, slot 4 `1C`, slot 15 `CC`). Nothing here is split, so no
//! string is longer than two characters. The slicer shows the painting in ITS
//! filament colours, slot by slot, which is what painting means to it; the
//! colour itself stays in the base material for readers that honour one.
//!
//! # Vertices are welded
//!
//! The display mesh duplicates every vertex per face so each can carry its
//! face's normal. 3MF has no normals, and the core specification wants a
//! `model` object's mesh to be a closed, consistently wound manifold with each
//! edge shared by exactly two triangles — which a per-face vertex soup is not.
//! So positions are welded on bit-identical coordinates before writing: the
//! watertight tessellation places the same point at a shared boundary vertex,
//! so an exact match recovers the shared edges without inventing a tolerance.
//! A triangle that collapses onto a repeated index is dropped and counted
//! ([`ThreeMfWriteReport::degenerate_triangles`]) rather than written, since
//! the core specification — and this module's reader — refuse one.
//!
//! Coordinates are written from the `f32` the display mesh holds, through
//! Rust's shortest round-tripping `Display`, so `10` is `10` and not
//! `10.000000000000000`, and the file re-reads to the triangles an STL export
//! of the same scene carries.

use std::collections::{BTreeMap, HashMap};

use super::zip::crc32;
use super::CORE_NAMESPACE;
use crate::Mesh;

/// One solid to write: its display mesh plus the colours the document gave it.
#[derive(Clone, Debug, Default)]
pub struct ThreeMfSolid {
    /// The scene name — becomes the `<object name>`.
    pub name: String,
    /// Positions / triangle indices, and the per-TRIANGLE face id the
    /// `face_colors` keys match. Normals are not written. An empty `face_ids`
    /// means "one group".
    pub mesh: Mesh,
    /// The body colour, 8-bit sRGB. `None` writes the object with no material.
    pub color: Option<[u8; 3]>,
    /// Per-face colour overrides, keyed by the face id in `mesh.face_ids`.
    /// A face listed here wins over `color` for its own triangles.
    pub face_colors: BTreeMap<u32, [u8; 3]>,
}

/// What one written package contains — the counts a caller reports or asserts on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ThreeMfWriteReport {
    /// `<object>`s written (solids that kept at least one triangle).
    pub objects: usize,
    /// Triangles written, across every object.
    pub triangles: usize,
    /// Vertices written after welding, across every object.
    pub vertices: usize,
    /// Triangles dropped because welding collapsed two of their corners.
    pub degenerate_triangles: usize,
    /// Distinct colours — `<base>` entries of the one `<basematerials>`.
    pub materials: usize,
    /// Triangles carrying their own `pid`/`p1`, i.e. coloured unlike their body.
    pub colored_triangles: usize,
    /// Triangles carrying slicer painting (`slic3rpe:mmu_segmentation` and
    /// `paint_color`): every coloured triangle whose colour got a slot.
    pub painted_triangles: usize,
    /// The first extruder slot the colours take: 1 when every written
    /// triangle is coloured, 2 when any is not (slot 1 is then the default
    /// extruder the uncoloured triangles print with).
    pub first_paint_slot: usize,
    /// Distinct colours that got a slot: `min(materials, MAX_PAINT_SLOTS + 1 - first_paint_slot)`.
    pub paint_slots: usize,
    /// Distinct colours past the slot limit, written as base materials only.
    pub unpainted_colors: usize,
    /// Total package length in bytes.
    pub bytes: usize,
}

/// PrusaSlicer's painting namespace, declared on `<model>` so the prefixed
/// attribute is well-formed XML.
const SLIC3RPE_NAMESPACE: &str = "http://schemas.slic3r.org/3mf/2017/06";
/// The most extruder slots the painting can name: PrusaSlicer's
/// `TriangleStateType` runs `Extruder1 ..= Extruder15`.
pub const MAX_PAINT_SLOTS: usize = 15;

/// The painting string for an UNSPLIT triangle in extruder `slot` (1-based,
/// at most [`MAX_PAINT_SLOTS`]) — see the module doc for the bitstream.
fn painting_code(slot: usize) -> String {
    debug_assert!((1..=MAX_PAINT_SLOTS).contains(&slot));
    if slot < 3 {
        // Nibble bits (least significant first): split 0, 0, then the two
        // state bits — the state shifted up by two.
        format!("{:X}", slot << 2)
    } else {
        // First nibble `1100` (= C: no split, state prefix 11), then a nibble
        // of state - 3; the first nibble is the LAST character.
        format!("{:X}C", slot - 3)
    }
}

/// The package part names, as the OPC conventions and the reader expect them.
const CONTENT_TYPES_PART: &str = "[Content_Types].xml";
const RELATIONSHIPS_PART: &str = "_rels/.rels";
const MODEL_PART: &str = "3D/3dmodel.model";
/// The id of the one `<basematerials>`; objects number from the next id
/// whether or not it is written, so a plain export and a coloured one agree.
const BASE_MATERIALS_ID: u32 = 1;
const FIRST_OBJECT_ID: u32 = 2;

const CONTENT_TYPES_XML: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n  \
<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n  \
<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n\
</Types>\n";

const RELATIONSHIPS_XML: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n  \
<Relationship Id=\"rel0\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\" Target=\"/3D/3dmodel.model\"/>\n\
</Relationships>\n";

/// An XML attribute value or text node, with the five escapes the grammar
/// needs. Scene names carry `:` today and nothing stops `&` or `"` tomorrow.
fn xml_escape(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for ch in value.chars() {
        match ch {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            // A control character is not XML 1.0 text at all; drop it rather
            // than write a part the reader rejects as not well-formed.
            c if (c as u32) < 0x20 && c != '\t' && c != '\n' && c != '\r' => {}
            c => out.push(c),
        }
    }
    out
}

/// `#RRGGBB`, upper case — the spelling the document's `color` attribute
/// persists, so a round trip through the reader is byte-exact.
fn display_color(rgb: [u8; 3]) -> String {
    format!("#{:02X}{:02X}{:02X}", rgb[0], rgb[1], rgb[2])
}

/// A coordinate as the file carries it: the display mesh's `f32`, printed in
/// its shortest round-tripping form.
fn coordinate(value: f64) -> Result<String, String> {
    let narrow = value as f32;
    if !narrow.is_finite() {
        return Err("write_3mf: non-finite vertex coordinate".into());
    }
    // `-0` would be a different key from `0` below and a pointless sign here.
    let narrow = if narrow == 0.0 { 0.0 } else { narrow };
    Ok(format!("{narrow}"))
}

/// Colour of one triangle: its face's override if it has one, else the body's.
fn triangle_color(solid: &ThreeMfSolid, triangle: usize) -> Option<[u8; 3]> {
    solid
        .mesh
        .face_ids
        .get(triangle)
        .and_then(|face| solid.face_colors.get(face))
        .copied()
        .or(solid.color)
}

/// The colour palette: distinct colours in first-seen order, and each one's
/// index — the `pindex` / `p1` a triangle names.
#[derive(Default)]
struct Palette {
    colors: Vec<[u8; 3]>,
    index: HashMap<[u8; 3], u32>,
}

impl Palette {
    fn intern(&mut self, rgb: [u8; 3]) -> u32 {
        if let Some(index) = self.index.get(&rgb) {
            return *index;
        }
        let index = self.colors.len() as u32;
        self.colors.push(rgb);
        self.index.insert(rgb, index);
        index
    }
}

/// One object's welded mesh, ready to print.
struct WeldedObject {
    id: u32,
    name: String,
    /// The printed coordinate strings, three per vertex.
    vertices: Vec<[String; 3]>,
    /// Triangle corners into `vertices`, plus the triangle's palette index.
    triangles: Vec<([u32; 3], Option<u32>)>,
    /// The body's palette index.
    body: Option<u32>,
}

/// Weld one solid's vertices on their printed coordinates and resolve its
/// colours into the palette. `None` when no triangle survives.
fn weld(
    solid: &ThreeMfSolid,
    id: u32,
    palette: &mut Palette,
    report: &mut ThreeMfWriteReport,
) -> Result<Option<WeldedObject>, String> {
    solid.mesh.validate_geometry()?;
    if solid.mesh.indices.is_empty() {
        return Ok(None);
    }
    let body = solid.color.map(|rgb| palette.intern(rgb));
    let source_vertices = solid.mesh.positions.len() / 3;
    // Weld on the printed strings: two coordinates that print alike are the
    // same number in the file, whatever bits produced them.
    let mut by_key: HashMap<[String; 3], u32> = HashMap::new();
    let mut remap: Vec<u32> = Vec::with_capacity(source_vertices);
    let mut vertices: Vec<[String; 3]> = Vec::new();
    for point in solid.mesh.positions.chunks_exact(3) {
        let key = [coordinate(point[0])?, coordinate(point[1])?, coordinate(point[2])?];
        let index = match by_key.get(&key) {
            Some(index) => *index,
            None => {
                let index = vertices.len() as u32;
                vertices.push(key.clone());
                by_key.insert(key, index);
                index
            }
        };
        remap.push(index);
    }
    let mut triangles = Vec::with_capacity(solid.mesh.indices.len() / 3);
    for (triangle, corners) in solid.mesh.indices.chunks_exact(3).enumerate() {
        let a = remap[corners[0] as usize];
        let b = remap[corners[1] as usize];
        let c = remap[corners[2] as usize];
        if a == b || b == c || a == c {
            report.degenerate_triangles += 1;
            continue;
        }
        let color = triangle_color(solid, triangle).map(|rgb| palette.intern(rgb));
        triangles.push(([a, b, c], color));
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    Ok(Some(WeldedObject { id, name: solid.name.clone(), vertices, triangles, body }))
}

/// The extruder slot of palette index `color`, if it has one: slots run
/// from `first_slot` to [`MAX_PAINT_SLOTS`].
fn paint_slot(color: u32, first_slot: usize) -> Option<usize> {
    let slot = color as usize + first_slot;
    (slot <= MAX_PAINT_SLOTS).then_some(slot)
}

/// The model part's XML.
fn model_xml(objects: &[WeldedObject], palette: &Palette, document_name: &str, first_slot: usize) -> String {
    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str(&format!(
        "<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"{CORE_NAMESPACE}\" \
         xmlns:slic3rpe=\"{SLIC3RPE_NAMESPACE}\">\n"
    ));
    xml.push_str(&format!(
        "  <metadata name=\"Title\">{}</metadata>\n",
        xml_escape(document_name)
    ));
    xml.push_str("  <metadata name=\"Application\">next.BREP.io</metadata>\n");
    xml.push_str("  <resources>\n");
    if !palette.colors.is_empty() {
        xml.push_str(&format!("    <basematerials id=\"{BASE_MATERIALS_ID}\">\n"));
        for rgb in &palette.colors {
            xml.push_str(&format!(
                "      <base name=\"color_{:02x}{:02x}{:02x}\" displaycolor=\"{}\"/>\n",
                rgb[0],
                rgb[1],
                rgb[2],
                display_color(*rgb)
            ));
        }
        xml.push_str("    </basematerials>\n");
    }
    for object in objects {
        xml.push_str(&format!(
            "    <object id=\"{}\" name=\"{}\" type=\"model\"",
            object.id,
            xml_escape(&object.name)
        ));
        if let Some(body) = object.body {
            xml.push_str(&format!(" pid=\"{BASE_MATERIALS_ID}\" pindex=\"{body}\""));
        }
        xml.push_str(">\n      <mesh>\n        <vertices>\n");
        for [x, y, z] in &object.vertices {
            xml.push_str(&format!("          <vertex x=\"{x}\" y=\"{y}\" z=\"{z}\"/>\n"));
        }
        xml.push_str("        </vertices>\n        <triangles>\n");
        for ([a, b, c], color) in &object.triangles {
            xml.push_str(&format!("          <triangle v1=\"{a}\" v2=\"{b}\" v3=\"{c}\""));
            if let Some(color) = color {
                // The slicer painting: the colour's slot, on EVERY coloured
                // triangle (the slicer has no object default to inherit).
                if let Some(slot) = paint_slot(*color, first_slot) {
                    let code = painting_code(slot);
                    xml.push_str(&format!(
                        " slic3rpe:mmu_segmentation=\"{code}\" paint_color=\"{code}\""
                    ));
                }
                // A triangle in its body's colour inherits the object's
                // pid/pindex; only a differently coloured one — or a coloured
                // triangle of an uncoloured body — states its own.
                if Some(*color) != object.body {
                    xml.push_str(&format!(" pid=\"{BASE_MATERIALS_ID}\" p1=\"{color}\""));
                }
            }
            xml.push_str("/>\n");
        }
        xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
    }
    xml.push_str("  </resources>\n  <build>\n");
    for object in objects {
        xml.push_str(&format!("    <item objectid=\"{}\"/>\n", object.id));
    }
    xml.push_str("  </build>\n</model>\n");
    xml
}

/// A minimal STORED ZIP: local headers with their data in order, then the
/// central directory and the end record. Fixed timestamp (1980-01-01 00:00,
/// the earliest DOS date), no extra fields, no comments, no data descriptors.
struct ZipWriter {
    bytes: Vec<u8>,
    central: Vec<u8>,
    entries: u16,
}

/// 1980-01-01 as a DOS date: day 1, month 1, year 0 (+1980).
const DOS_DATE_1980_01_01: u16 = (1 << 5) | 1;

impl ZipWriter {
    fn new() -> Self {
        Self { bytes: Vec::new(), central: Vec::new(), entries: 0 }
    }

    fn add(&mut self, name: &str, data: &[u8]) -> Result<(), String> {
        if data.len() > u32::MAX as usize {
            return Err(format!("write_3mf: part '{name}' is larger than a ZIP entry can hold"));
        }
        let crc = crc32(data);
        let size = data.len() as u32;
        let offset = self.bytes.len() as u32;
        let name_bytes = name.as_bytes();

        let mut local = Vec::with_capacity(30 + name_bytes.len());
        push_u32(&mut local, 0x0403_4b50);
        push_u16(&mut local, 20); // version needed: 2.0
        push_u16(&mut local, 0); // flags
        push_u16(&mut local, 0); // method: stored
        push_u16(&mut local, 0); // time
        push_u16(&mut local, DOS_DATE_1980_01_01);
        push_u32(&mut local, crc);
        push_u32(&mut local, size);
        push_u32(&mut local, size);
        push_u16(&mut local, name_bytes.len() as u16);
        push_u16(&mut local, 0); // extra length
        local.extend_from_slice(name_bytes);
        self.bytes.extend_from_slice(&local);
        self.bytes.extend_from_slice(data);

        push_u32(&mut self.central, 0x0201_4b50);
        push_u16(&mut self.central, 20); // version made by
        push_u16(&mut self.central, 20); // version needed
        push_u16(&mut self.central, 0); // flags
        push_u16(&mut self.central, 0); // method
        push_u16(&mut self.central, 0); // time
        push_u16(&mut self.central, DOS_DATE_1980_01_01);
        push_u32(&mut self.central, crc);
        push_u32(&mut self.central, size);
        push_u32(&mut self.central, size);
        push_u16(&mut self.central, name_bytes.len() as u16);
        push_u16(&mut self.central, 0); // extra length
        push_u16(&mut self.central, 0); // comment length
        push_u16(&mut self.central, 0); // disk number start
        push_u16(&mut self.central, 0); // internal attributes
        push_u32(&mut self.central, 0); // external attributes
        push_u32(&mut self.central, offset);
        self.central.extend_from_slice(name_bytes);
        self.entries += 1;
        Ok(())
    }

    fn finish(mut self) -> Result<Vec<u8>, String> {
        let directory_offset = self.bytes.len();
        let directory_size = self.central.len();
        if directory_offset + directory_size + 22 > u32::MAX as usize {
            return Err("write_3mf: the package is larger than a ZIP can hold".into());
        }
        self.bytes.extend_from_slice(&self.central);
        push_u32(&mut self.bytes, 0x0605_4b50);
        push_u16(&mut self.bytes, 0); // this disk
        push_u16(&mut self.bytes, 0); // directory disk
        push_u16(&mut self.bytes, self.entries);
        push_u16(&mut self.bytes, self.entries);
        push_u32(&mut self.bytes, directory_size as u32);
        push_u32(&mut self.bytes, directory_offset as u32);
        push_u16(&mut self.bytes, 0); // comment length
        Ok(self.bytes)
    }
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

/// Write `solids` as one 3MF package in millimetres.
///
/// `document_name` becomes the model's `Title` metadata. Solids with no
/// triangles — before or after welding — are SKIPPED (a `model` object must
/// enclose volume); a set in which nothing has a triangle is refused, as the
/// OBJ, STL and GLB lanes refuse an empty scene.
pub fn write_3mf(
    solids: &[ThreeMfSolid],
    document_name: &str,
) -> Result<(Vec<u8>, ThreeMfWriteReport), String> {
    let mut report = ThreeMfWriteReport::default();
    let mut palette = Palette::default();
    let mut objects = Vec::new();
    for solid in solids {
        let id = FIRST_OBJECT_ID + objects.len() as u32;
        if let Some(object) = weld(solid, id, &mut palette, &mut report)? {
            objects.push(object);
        }
    }
    if objects.is_empty() {
        return Err("write_3mf: nothing to export, no solid has a triangle".into());
    }
    report.objects = objects.len();
    report.materials = palette.colors.len();
    let any_uncolored = objects.iter().any(|o| o.triangles.iter().any(|(_, c)| c.is_none()));
    let first_slot = if any_uncolored { 2 } else { 1 };
    report.first_paint_slot = first_slot;
    let slots = MAX_PAINT_SLOTS + 1 - first_slot;
    report.paint_slots = palette.colors.len().min(slots);
    report.unpainted_colors = palette.colors.len().saturating_sub(slots);
    for object in &objects {
        report.vertices += object.vertices.len();
        report.triangles += object.triangles.len();
        report.colored_triangles += object
            .triangles
            .iter()
            .filter(|(_, color)| color.is_some() && *color != object.body)
            .count();
        report.painted_triangles += object
            .triangles
            .iter()
            .filter(|(_, color)| color.is_some_and(|c| paint_slot(c, first_slot).is_some()))
            .count();
    }
    let model = model_xml(&objects, &palette, document_name, first_slot);

    let mut zip = ZipWriter::new();
    zip.add(CONTENT_TYPES_PART, CONTENT_TYPES_XML.as_bytes())?;
    zip.add(RELATIONSHIPS_PART, RELATIONSHIPS_XML.as_bytes())?;
    zip.add(MODEL_PART, model.as_bytes())?;
    let bytes = zip.finish()?;
    report.bytes = bytes.len();
    Ok((bytes, report))
}

