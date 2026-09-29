//! The live 3D model a sheet PDF can carry — a U3D stream ([`super::u3d`]) and
//! the PDF 1.7 dictionaries that show it (ISO 32000-1 §13.6).
//!
//! # What the reader sees
//!
//! One U3D stream holds the whole scene, shared by every 3D annotation in the
//! file:
//!
//! - each drawn solid as a MESH node named for the solid, and its edges as a
//!   dark LINE SET node beside it (`{solid} edges`), so the model reads as a
//!   CAD model and not a smooth blob;
//! - each enabled, resolved annotation of each saved view with a camera as a
//!   LINE SET node (`PMI {view} {annotation}`: dimension, extension and leader
//!   lines, frames, and the text STROKED with the PMI font — the same
//!   [`pmi_present`] layout the viewport and the STEP presentation use, sized
//!   as the viewport drew it when the view was saved), plus a small MESH node
//!   for its filled arrowheads (`… fill`).
//!
//! Each saved view is a `/3DView` in the stream's `/VA` list: its camera as the
//! camera-to-world matrix, its projection, and a `/NA` node list that shows
//! THAT view's annotations and hides every other view's, and hides the solids
//! the view hides. A reader's view menu is therefore the document's PMI view
//! list, and picking one shows exactly that view's PMI.
//!
//! Only readers that implement 3D annotations draw any of this — Adobe
//! Acrobat / Reader (with 3D enabled) and Foxit. Every other reader shows the
//! annotation's appearance, which is why the sheet's own vector drawing stays
//! on the page beneath a placement, and why the full 3D page draws the default
//! view as a hidden-line drawing under its 3D box.
//!
//! # The camera
//!
//! A view's `/C2W` maps camera coordinates to world coordinates: its columns
//! are the camera's x axis (right), y axis and z axis (the viewing direction,
//! eye → target), then the eye. ISO 32000-1 §13.6.5 states the camera's y axis
//! points UP; [`CAMERA_Y_UP`] records which way this writer points it, because
//! the claim is checked in a real reader, not taken from the text.

use brep_kernel::{
    pmi_present, PmiAnnotationReport, PmiCamera, PmiGeometry, PmiLayoutStyle, PmiPlane, PmiProjection, PmiReport,
    PmiState, PmiStatus, PmiView,
};

use super::pdf::{string_literal, PT_PER_MM};
use super::u3d::{self, Geometry, Lines, Material, Mesh, Model, Scene};

/// The camera's y axis is the view's UP vector (true) or its down vector
/// (false). See the module doc.
pub const CAMERA_Y_UP: bool = true;

use crate::engine_state::pmi_overlay::{ARROW_PX, TEXT_PX_AT_12PT};

/// Model units per viewport pixel under a saved camera, in the viewport the
/// view was saved from — `ViewCamera::world_per_pixel`'s rule.
pub fn world_per_pixel(camera: &PmiCamera) -> f64 {
    let height = camera.viewport[1].max(1.0);
    match camera.projection {
        PmiProjection::Orthographic { half_height } => 2.0 * half_height / height,
        PmiProjection::Perspective { fov_y_deg } => {
            let d = sub(camera.target, camera.eye);
            2.0 * (fov_y_deg.to_radians() * 0.5).tan() * dot(d, d).sqrt() / height
        }
    }
}

/// Edge lines: near-black, so they read on any body colour.
const EDGE_RGB: [u8; 3] = [0x20, 0x20, 0x24];
/// PMI: a deep orange — the viewport's amber darkened enough to read on the
/// PDF's white background, and clear of the blues and greys bodies default
/// to (a blue first drawn here vanished into a blue cube in Acrobat).
const PMI_RGB: [u8; 3] = [0xD9, 0x68, 0x00];

/// One solid, gathered by the engine from the display scene.
#[derive(Debug, Clone)]
pub struct Body {
    pub name: String,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    /// Three per triangle.
    pub indices: Vec<u32>,
    /// sRGB per triangle.
    pub triangle_colors: Vec<[u8; 3]>,
    /// World polylines of the solid's real edges.
    pub edges: Vec<Vec<[f32; 3]>>,
}

/// How a view projects.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Projection3d {
    /// Points of the annotation per model unit.
    Orthographic { scale: f64 },
    /// Vertical field of view, degrees.
    Perspective { fov_y_deg: f64 },
}

/// A saved view as the reader will show it.
#[derive(Debug, Clone, PartialEq)]
pub struct View3d {
    /// The PMI view id (`VIEW1`) — the dictionary's internal name.
    pub id: String,
    /// What the reader's view menu lists.
    pub name: String,
    pub camera: PmiCamera,
    pub wireframe: bool,
    /// Visibility per [`Model3d::nodes`] entry.
    pub visible: Vec<bool>,
}

/// The file's 3D content: the U3D bytes, the node names in stream order, and
/// every saved view that has a camera.
#[derive(Debug, Clone, PartialEq)]
pub struct Model3d {
    pub u3d: Vec<u8>,
    pub nodes: Vec<String>,
    pub views: Vec<View3d>,
    /// How many annotations are drawn, over every view.
    pub annotations: usize,
}

impl Model3d {
    pub fn view(&self, id: &str) -> Option<&View3d> {
        self.views.iter().find(|view| view.id == id)
    }
}

/// A node name a PDF `/3DNode /N` can match exactly: printable ASCII, unique.
fn node_name(wanted: &str, taken: &mut std::collections::HashSet<String>) -> String {
    let base: String = wanted
        .chars()
        .map(|c| if (' '..='~').contains(&c) { c } else { '_' })
        .collect::<String>()
        .trim()
        .to_string();
    let base = if base.is_empty() { "node".to_string() } else { base };
    let mut name = base.clone();
    let mut n = 2;
    while !taken.insert(name.clone()) {
        name = format!("{base} #{n}");
        n += 1;
    }
    name
}

fn material_for(rgb: [u8; 3], lit_line: bool) -> Material {
    let c = rgb.map(|v| f32::from(v) / 255.0);
    Material {
        name: format!("C{:02X}{:02X}{:02X}{}", rgb[0], rgb[1], rgb[2], if lit_line { "L" } else { "" }),
        ambient: c.map(|v| v * 0.3),
        diffuse: c,
        specular: [0.15; 3],
        // A line has no surface to light: it shows its own colour.
        emissive: if lit_line { c } else { [0.0; 3] },
        reflectivity: 0.1,
        opacity: 1.0,
    }
}

/// The index of `rgb`'s material, added once.
fn material_index(scene: &mut Scene, rgb: [u8; 3], line: bool) -> usize {
    let wanted = material_for(rgb, line);
    if let Some(index) = scene.materials.iter().position(|m| m.name == wanted.name) {
        return index;
    }
    scene.materials.push(wanted);
    scene.materials.len() - 1
}

fn to_f32(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

/// Polylines as one line set: consecutive points joined, zero-length steps
/// dropped.
fn line_set(polylines: &[Vec<[f32; 3]>]) -> Option<Lines> {
    let mut lines = Lines::default();
    for polyline in polylines {
        let mut previous: Option<u32> = None;
        for p in polyline {
            if !p.iter().all(|c| c.is_finite()) {
                previous = None;
                continue;
            }
            if let Some(prev) = previous {
                if lines.positions[prev as usize] == *p {
                    continue;
                }
            }
            let index = lines.positions.len() as u32;
            lines.positions.push(*p);
            if let Some(prev) = previous {
                lines.segments.push(([prev, index], 0));
            }
            previous = Some(index);
        }
    }
    (!lines.segments.is_empty()).then_some(lines)
}

/// Triangles as a flat-shaded mesh: three vertices each, carrying the face
/// normal.
fn triangle_mesh(triangles: &[[[f64; 3]; 3]]) -> Option<Mesh> {
    let mut mesh = Mesh::default();
    for t in triangles {
        let u = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
        let v = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if !(len > 1e-12) {
            continue;
        }
        let normal = to_f32([n[0] / len, n[1] / len, n[2] / len]);
        let base = mesh.positions.len() as u32;
        for corner in t {
            mesh.positions.push(to_f32(*corner));
            mesh.normals.push(normal);
        }
        mesh.triangles.push(([base, base + 1, base + 2], 0));
    }
    (!mesh.triangles.is_empty()).then_some(mesh)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let len = dot(a, a).sqrt();
    (len > 1e-12 && len.is_finite()).then(|| [a[0] / len, a[1] / len, a[2] / len])
}

/// The camera's right, up and forward (eye → target) unit vectors; `None`
/// for a degenerate camera (eye on target, or up along the view).
pub fn camera_frame(camera: &PmiCamera) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    let forward = unit(sub(camera.target, camera.eye))?;
    let right = unit(cross(forward, camera.up))?;
    let up = cross(right, forward);
    Some((right, up, forward))
}

/// Build the 3D model: the solids, then every saved view's annotations.
pub fn build(bodies: &[Body], pmi: &PmiState, report: Option<&PmiReport>) -> Result<Model3d, String> {
    let mut scene = Scene::default();
    let mut taken = std::collections::HashSet::new();
    // Per node: the scene object it belongs to (a solid name) or the view it
    // annotates — what decides its visibility in each view.
    enum Owner {
        Solid(String),
        View(String),
    }
    let mut owners: Vec<Owner> = Vec::new();

    for body in bodies {
        if body.indices.len() < 3 || body.positions.is_empty() {
            continue;
        }
        let mut shaders: Vec<usize> = Vec::new();
        let mut mesh = Mesh { positions: body.positions.clone(), normals: body.normals.clone(), triangles: Vec::new() };
        for (index, corners) in body.indices.chunks_exact(3).enumerate() {
            let rgb = body.triangle_colors.get(index).copied().unwrap_or([0xB0, 0xB4, 0xBC]);
            let material = material_index(&mut scene, rgb, false);
            let shading = match shaders.iter().position(|m| *m == material) {
                Some(slot) => slot,
                None => {
                    shaders.push(material);
                    shaders.len() - 1
                }
            };
            mesh.triangles.push(([corners[0], corners[1], corners[2]], shading as u32));
        }
        if mesh.triangles.is_empty() {
            continue;
        }
        let name = node_name(&body.name, &mut taken);
        scene.models.push(Model { name, geometry: Geometry::Mesh(mesh), shaders, two_sided: false });
        owners.push(Owner::Solid(body.name.clone()));
        if let Some(lines) = line_set(&body.edges) {
            let material = material_index(&mut scene, EDGE_RGB, true);
            let name = node_name(&format!("{} edges", body.name), &mut taken);
            scene.models.push(Model { name, geometry: Geometry::Lines(lines), shaders: vec![material], two_sided: false });
            owners.push(Owner::Solid(body.name.clone()));
        }
    }
    if scene.models.is_empty() {
        return Err("the scene has no solid to show in 3D".into());
    }

    let mut annotations = 0;
    let cameras: Vec<(&PmiView, &PmiCamera)> =
        pmi.views.iter().filter_map(|view| view.camera.as_ref().map(|camera| (view, camera))).collect();
    for (view, camera) in &cameras {
        let Some(rows) = report.and_then(|report| report.view(&view.id)) else { continue };
        let Some((right, _, forward)) = camera_frame(camera) else { continue };
        // Sized as the viewport sized them when the view was saved: the
        // overlay's pixel sizes times the saved camera's world per pixel. The
        // model units a U3D line lives in cannot stay a constant size on
        // screen, so this is exact at the view's own framing and scales with
        // the model when the reader zooms.
        let wpp = world_per_pixel(camera);
        let text_height = TEXT_PX_AT_12PT * wpp * (view.display.text_size_pt / 12.0);
        let style = PmiLayoutStyle {
            arrow: ARROW_PX * wpp,
            text_height,
            view_dir: forward,
            view_up: camera.up,
            plane: None,
        };
        for row in &rows.annotations {
            if !drawable(row) {
                continue;
            }
            // A view-aligned row is laid out in the view-parallel plane
            // through its label, facing the camera — the STEP presentation's
            // rule, so the two 3D exports draw the same thing.
            let row_style = PmiLayoutStyle {
                plane: row.plane.or(Some(PmiPlane {
                    origin: row.label_world,
                    normal: [-forward[0], -forward[1], -forward[2]],
                    x_axis: right,
                })),
                ..style
            };
            let drawn = pmi_present(&row.geometry, row.label_world, &row.text, &row_style);
            let mut polylines: Vec<Vec<[f32; 3]>> =
                drawn.polylines.iter().chain(&drawn.frames).map(|line| line.iter().copied().map(to_f32).collect()).collect();
            for run in &drawn.texts {
                polylines.extend(run.strokes().into_iter().map(|stroke| stroke.into_iter().map(to_f32).collect()));
            }
            let lines = line_set(&polylines);
            let fill = triangle_mesh(&drawn.arrows);
            if lines.is_none() && fill.is_none() {
                continue;
            }
            annotations += 1;
            let base = format!("PMI {} {}", view.id, row.id);
            if let Some(lines) = lines {
                let material = material_index(&mut scene, PMI_RGB, true);
                let name = node_name(&base, &mut taken);
                scene.models.push(Model { name, geometry: Geometry::Lines(lines), shaders: vec![material], two_sided: false });
                owners.push(Owner::View(view.id.clone()));
            }
            if let Some(fill) = fill {
                let material = material_index(&mut scene, PMI_RGB, true);
                let name = node_name(&format!("{base} fill"), &mut taken);
                scene.models.push(Model { name, geometry: Geometry::Mesh(fill), shaders: vec![material], two_sided: true });
                owners.push(Owner::View(view.id.clone()));
            }
        }
    }

    let u3d = u3d::write(&scene)?;
    let nodes: Vec<String> = scene.models.iter().map(|model| model.name.clone()).collect();
    let mut views: Vec<View3d> = cameras
        .iter()
        .map(|(view, camera)| View3d {
            id: view.id.clone(),
            name: if view.name.trim().is_empty() { view.id.clone() } else { view.name.clone() },
            camera: (*camera).clone(),
            wireframe: view.display.wireframe,
            visible: owners
                .iter()
                .map(|owner| match owner {
                    Owner::Solid(solid) => !view.display.hidden.iter().any(|hidden| hidden == solid),
                    Owner::View(id) => *id == view.id,
                })
                .collect(),
        })
        .collect();
    // A document with no saved view still opens on SOMETHING: an isometric
    // view fitted to the solids, every solid shown.
    if views.is_empty() {
        views.push(View3d {
            id: ISOMETRIC.into(),
            name: "Isometric".into(),
            camera: isometric_camera(bodies),
            wireframe: false,
            visible: owners.iter().map(|owner| matches!(owner, Owner::Solid(_))).collect(),
        });
    }
    Ok(Model3d { u3d, nodes, views, annotations })
}

/// The id of the view a document without saved views opens on.
pub const ISOMETRIC: &str = "ISOMETRIC";

/// Looking down the (−1, −1, −1) diagonal at the solids' box centre, +Y up
/// (the application's), orthographic, the bounding sphere filling the height.
pub fn isometric_camera(bodies: &[Body]) -> PmiCamera {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in bodies.iter().flat_map(|body| body.positions.iter()) {
        for axis in 0..3 {
            min[axis] = min[axis].min(f64::from(p[axis]));
            max[axis] = max[axis].max(f64::from(p[axis]));
        }
    }
    if !min[0].is_finite() {
        (min, max) = ([-1.0; 3], [1.0; 3]);
    }
    let centre = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5, (min[2] + max[2]) * 0.5];
    let radius = (dot(sub(max, min), sub(max, min)).sqrt() * 0.5).max(1e-3);
    let d = radius * 4.0 / 3f64.sqrt();
    PmiCamera {
        eye: [centre[0] + d, centre[1] + d, centre[2] + d],
        target: centre,
        up: [0.0, 1.0, 0.0],
        projection: PmiProjection::Orthographic { half_height: radius * 1.1 },
        viewport: [1280.0, 800.0],
    }
}

/// A report row the 3D model draws: enabled, resolved, and not an explode
/// (which moves solids rather than annotating them, and which the STEP
/// presentation leaves out for the same reason).
fn drawable(row: &PmiAnnotationReport) -> bool {
    row.enabled && row.status == PmiStatus::Ok && !matches!(row.geometry, PmiGeometry::Explode { .. })
}

// ---------------------------------------------------------------------------
// The PDF dictionaries
// ---------------------------------------------------------------------------

fn num(value: f64) -> String {
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" || text.is_empty() {
        "0".into()
    } else {
        text.into()
    }
}

/// The 12-element camera-to-world matrix and the distance to the centre of
/// orbit (the camera's target).
pub fn c2w(camera: &PmiCamera) -> Option<([f64; 12], f64)> {
    let (right, up, forward) = camera_frame(camera)?;
    let y = if CAMERA_Y_UP { up } else { [-up[0], -up[1], -up[2]] };
    let e = camera.eye;
    let m = [right[0], right[1], right[2], y[0], y[1], y[2], forward[0], forward[1], forward[2], e[0], e[1], e[2]];
    Some((m, dot(sub(camera.target, camera.eye), forward)))
}

/// A view's projection: a perspective camera keeps its field of view; an
/// orthographic one is scaled so its saved half height fills half of a 3D
/// box `box_height_pt` tall, unless `scale` fixes it.
pub fn projection(camera: &PmiCamera, box_height_pt: f64, scale: Option<f64>) -> Projection3d {
    match (&camera.projection, scale) {
        (_, Some(scale)) => Projection3d::Orthographic { scale },
        (PmiProjection::Perspective { fov_y_deg }, None) => Projection3d::Perspective { fov_y_deg: *fov_y_deg },
        (PmiProjection::Orthographic { half_height }, None) => {
            Projection3d::Orthographic { scale: box_height_pt * 0.5 / half_height.max(1e-9) }
        }
    }
}

/// A `/3DView` dictionary: `name` in the reader's menu, the view's camera,
/// `projection`, its render mode, a CAD lighting scheme, and every node's
/// visibility.
pub fn view_dict(model: &Model3d, view: &View3d, name: &str, projection: Projection3d) -> String {
    let (matrix, orbit) = c2w(&view.camera).unwrap_or(([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 1.0));
    let matrix: Vec<String> = matrix.iter().map(|v| num(*v)).collect();
    let projection = match projection {
        Projection3d::Orthographic { scale } => format!("<< /Subtype /O /OS {} >>", num(scale)),
        Projection3d::Perspective { fov_y_deg } => {
            format!("<< /Subtype /P /FOV {} /PS /H >>", num(fov_y_deg.clamp(1.0, 179.0)))
        }
    };
    let nodes: Vec<String> = model
        .nodes
        .iter()
        .zip(&view.visible)
        .map(|(node, visible)| format!("<< /Type /3DNode /N ({}) /V {visible} >>", string_literal(node)))
        .collect();
    format!(
        "<< /Type /3DView /XN ({}) /IN ({}) /MS /M /C2W [{}] /CO {} /P {} \
         /BG << /Type /3DBG /Subtype /SC /C [1 1 1] >> \
         /RM << /Type /3DRenderMode /Subtype /{} >> \
         /LS << /Type /3DLightingScheme /Subtype /Hard >> /NR true /NA [{}] >>",
        string_literal(name),
        string_literal(&view.id),
        matrix.join(" "),
        num(orbit.max(0.0)),
        projection,
        if view.wireframe { "Wireframe" } else { "Solid" },
        nodes.join(" ")
    )
}

/// The 3D stream: every saved view in `/VA`, the default the first, and the
/// U3D bytes ASCII85-encoded so the file stays ASCII (the PDF writer's
/// contract).
pub fn stream_object(model: &Model3d, va_box_height_pt: f64) -> String {
    let views: Vec<String> = model
        .views
        .iter()
        .map(|view| view_dict(model, view, &view.name, projection(&view.camera, va_box_height_pt, None)))
        .collect();
    // The stream's data ends with its own newline, counted in /Length — the
    // PDF writer's convention for every stream it writes.
    let data = ascii85(&model.u3d) + "\n";
    format!(
        "<< /Type /3D /Subtype /U3D /Filter /ASCII85Decode /DV 0 /VA [{}] /Length {} >>\nstream\n{}endstream",
        views.join(" "),
        data.len(),
        data
    )
}

/// ASCII85 (ISO 32000-1 §7.4.3), `~>`-terminated, in 76-character lines.
pub fn ascii85(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 5 / 4 + bytes.len() / 60 + 4);
    let mut column = 0;
    let mut push = |c: char, out: &mut String| {
        out.push(c);
        column += 1;
        if column == 76 {
            out.push('\n');
            column = 0;
        }
    };
    for chunk in bytes.chunks(4) {
        let mut word = [0u8; 4];
        word[..chunk.len()].copy_from_slice(chunk);
        let value = u32::from_be_bytes(word);
        if chunk.len() == 4 && value == 0 {
            push('z', &mut out);
            continue;
        }
        let mut digits = [0u8; 5];
        let mut v = value;
        for digit in digits.iter_mut().rev() {
            *digit = (v % 85) as u8;
            v /= 85;
        }
        for digit in &digits[..chunk.len() + 1] {
            push((b'!' + digit) as char, &mut out);
        }
    }
    out.push_str("~>");
    out
}

/// A 3D annotation's rectangle in PDF points: centred on `centre_mm` (sheet
/// millimetres from the top-left), `half_mm` either way, on a page
/// `page_height_mm` tall.
pub fn rect_pt(centre_mm: [f64; 2], half_mm: [f64; 2], page_height_mm: f64) -> [f64; 4] {
    [
        (centre_mm[0] - half_mm[0]) * PT_PER_MM,
        (page_height_mm - centre_mm[1] - half_mm[1]) * PT_PER_MM,
        (centre_mm[0] + half_mm[0]) * PT_PER_MM,
        (page_height_mm - centre_mm[1] + half_mm[1]) * PT_PER_MM,
    ]
}

// ---------------------------------------------------------------------------
// Where the model is shown
// ---------------------------------------------------------------------------

/// Everything 3D a sheet PDF carries.
#[derive(Debug, Clone)]
pub struct Pdf3d {
    pub model: Model3d,
    pub placements: Vec<Placement3d>,
    /// A last page given over to the model.
    pub page: Option<Page3d>,
}

/// A placement drawn live: a 3D box over the placement's drawing, opening on
/// the placement's own camera at the placement's own scale, so activating it
/// changes nothing until the reader turns the model.
#[derive(Debug, Clone, PartialEq)]
pub struct Placement3d {
    /// The sheet's index among the file's pages.
    pub page: usize,
    /// The placement id (`SV2`).
    pub id: String,
    /// The saved view it places (`VIEW1`).
    pub view: String,
    /// Sheet millimetres from the top-left: the camera target's point, and the
    /// box's half extents about it.
    pub centre_mm: [f64; 2],
    pub half_mm: [f64; 2],
    /// Paper millimetres per model unit.
    pub scale: f64,
}

/// A view button on the 3D page.
#[derive(Debug, Clone, PartialEq)]
pub struct Button {
    /// `[x0, y0, x1, y1]`, sheet millimetres from the top-left.
    pub rect_mm: [f64; 4],
    pub label: String,
    /// The view's index in [`Model3d::views`] (and so in `/VA`).
    pub view: usize,
}

/// The 3D page: a title, the 3D box, a button per saved view, a caption.
#[derive(Debug, Clone)]
pub struct Page3d {
    pub width_mm: f64,
    pub height_mm: f64,
    pub title: String,
    pub title_at: [f64; 2],
    pub title_mm: f64,
    /// `[x0, y0, x1, y1]` of the 3D box, sheet millimetres from the top-left.
    pub box_mm: [f64; 4],
    /// The view the box opens on — the first saved view with a camera.
    pub default_view: usize,
    /// The default view drawn as a sheet placement at the centre of the box —
    /// what a reader without 3D shows. `None` when there is nothing to draw.
    pub poster: Option<ViewDrawing>,
    /// The poster's scale (paper millimetres per model unit): the box opens at
    /// it, so activating the model does not jump.
    pub poster_scale: Option<f64>,
    pub buttons: Vec<Button>,
    pub button_text_mm: f64,
    pub caption: Vec<String>,
    pub caption_at: Vec<[f64; 2]>,
    pub caption_mm: f64,
}

use super::project::ViewDrawing;

const PAGE_MARGIN_MM: f64 = 12.0;
const TITLE_MM: f64 = 5.0;
const BUTTON_H_MM: f64 = 8.0;
const BUTTON_GAP_MM: f64 = 3.0;
const BUTTON_TEXT_MM: f64 = 2.8;
const BUTTONS_PER_ROW: usize = 8;
const CAPTION_MM: f64 = 2.8;

/// The paragraph under the 3D box — the truth about who can show it.
pub const CAPTION: [&str; 2] = [
    "Interactive 3D model: open this file in Adobe Acrobat or Reader (with 3D content enabled) or in Foxit to rotate, zoom and switch PMI views.",
    "Other PDF viewers show the drawing above in place of the model.",
];

impl Page3d {
    /// The page laid out on paper `width_mm` × `height_mm`: the title at the
    /// top, the caption at the foot, a row of buttons above it (wrapping
    /// every [`BUTTONS_PER_ROW`]), and the box filling what is left.
    pub fn layout(width_mm: f64, height_mm: f64, title: &str, model: &Model3d) -> Page3d {
        let (w, h) = (width_mm.max(100.0), height_mm.max(100.0));
        let inner = w - 2.0 * PAGE_MARGIN_MM;
        let caption_at: Vec<[f64; 2]> = (0..CAPTION.len())
            .map(|line| [w * 0.5, h - PAGE_MARGIN_MM - (CAPTION.len() - 1 - line) as f64 * CAPTION_MM * 1.8])
            .collect();
        let rows = model.views.len().div_ceil(BUTTONS_PER_ROW);
        let per_row = model.views.len().min(BUTTONS_PER_ROW).max(1);
        let button_w = ((inner - (per_row - 1) as f64 * BUTTON_GAP_MM) / per_row as f64).min(48.0);
        let buttons_bottom = caption_at[0][1] - CAPTION_MM * 2.0;
        let buttons_top = buttons_bottom - rows as f64 * (BUTTON_H_MM + BUTTON_GAP_MM) + BUTTON_GAP_MM;
        let buttons = model
            .views
            .iter()
            .enumerate()
            .map(|(index, view)| {
                let (row, column) = (index / BUTTONS_PER_ROW, index % BUTTONS_PER_ROW);
                let in_row = (model.views.len() - row * BUTTONS_PER_ROW).min(BUTTONS_PER_ROW);
                let row_w = in_row as f64 * button_w + (in_row - 1) as f64 * BUTTON_GAP_MM;
                let x0 = (w - row_w) * 0.5 + column as f64 * (button_w + BUTTON_GAP_MM);
                let y0 = buttons_top + row as f64 * (BUTTON_H_MM + BUTTON_GAP_MM);
                Button { rect_mm: [x0, y0, x0 + button_w, y0 + BUTTON_H_MM], label: view.name.clone(), view: index }
            })
            .collect();
        let box_top = PAGE_MARGIN_MM + TITLE_MM * 2.0;
        let box_bottom = if rows > 0 { buttons_top - BUTTON_GAP_MM * 2.0 } else { buttons_bottom };
        Page3d {
            width_mm: w,
            height_mm: h,
            title: title.to_string(),
            title_at: [w * 0.5, PAGE_MARGIN_MM + TITLE_MM * 0.5],
            title_mm: TITLE_MM,
            box_mm: [PAGE_MARGIN_MM, box_top, w - PAGE_MARGIN_MM, box_bottom.max(box_top + 20.0)],
            default_view: 0,
            poster: None,
            poster_scale: None,
            buttons,
            button_text_mm: BUTTON_TEXT_MM,
            caption: CAPTION.iter().map(|line| line.to_string()).collect(),
            caption_at,
            caption_mm: CAPTION_MM,
        }
    }

    pub fn box_centre_mm(&self) -> [f64; 2] {
        [(self.box_mm[0] + self.box_mm[2]) * 0.5, (self.box_mm[1] + self.box_mm[3]) * 0.5]
    }

    pub fn box_half_mm(&self) -> [f64; 2] {
        [(self.box_mm[2] - self.box_mm[0]) * 0.5, (self.box_mm[3] - self.box_mm[1]) * 0.5]
    }
}

/// The object numbers of every 3D object, assigned before any is written so
/// the page dictionaries can name their annotations.
#[derive(Debug, Clone)]
pub struct Plan {
    stream: usize,
    /// Per placement: (annotation, appearance).
    placements: Vec<(usize, usize)>,
    /// The 3D page's (annotation, appearance, links).
    page: Option<(usize, usize, Vec<usize>)>,
    /// Which page each placement sits on.
    placement_pages: Vec<usize>,
    /// The 3D page's index among the file's pages.
    page_index: usize,
}

fn rect_text(r: [f64; 4]) -> String {
    format!("[{:.4} {:.4} {:.4} {:.4}]", r[0], r[1], r[2], r[3])
}

/// An appearance that draws nothing: the page's own marks show through until
/// the reader activates the model.
fn empty_appearance(rect: [f64; 4]) -> String {
    format!(
        "<< /Type /XObject /Subtype /Form /BBox [0 0 {:.4} {:.4}] /Length 1 >>\nstream\n\nendstream",
        rect[2] - rect[0],
        rect[3] - rect[1]
    )
}

impl Plan {
    /// Numbers from `next` on, for the 3D content of `three_d` in a file of
    /// `sheets` sheet pages (the 3D page, when there is one, follows them).
    pub fn new(three_d: &Pdf3d, mut next: usize, sheets: usize) -> Plan {
        let stream = next;
        next += 1;
        let placements = three_d
            .placements
            .iter()
            .map(|_| {
                next += 2;
                (next - 2, next - 1)
            })
            .collect();
        let page = three_d.page.as_ref().map(|page| {
            let annot = next;
            let links = (0..page.buttons.len()).map(|i| annot + 2 + i).collect::<Vec<_>>();
            next += 2 + page.buttons.len();
            (annot, annot + 1, links)
        });
        Plan {
            stream,
            placements,
            page,
            placement_pages: three_d.placements.iter().map(|p| p.page).collect(),
            page_index: sheets,
        }
    }

    /// ` /Annots […]` for page `index` (a sheet, or — one past the sheets —
    /// the 3D page), or nothing when it carries none.
    pub fn annots_on(&self, index: usize) -> String {
        let mut refs: Vec<String> = self
            .placements
            .iter()
            .zip(&self.placement_pages)
            .filter(|(_, page)| **page == index)
            .map(|((annot, _), _)| format!("{annot} 0 R"))
            .collect();
        if let Some((annot, _, links)) = &self.page {
            if index == self.page_index {
                refs.push(format!("{annot} 0 R"));
                refs.extend(links.iter().map(|link| format!("{link} 0 R")));
            }
        }
        if refs.is_empty() {
            String::new()
        } else {
            format!(" /Annots [{}]", refs.join(" "))
        }
    }

    /// Every 3D object, in object-number order: the stream, each placement's
    /// annotation and appearance, the 3D page's.
    pub fn objects(&self, three_d: &Pdf3d, pages: &[&super::project::SheetDrawing], first_page: usize) -> Vec<(usize, String)> {
        let model = &three_d.model;
        let placement_box_pt = three_d
            .placements
            .iter()
            .map(|placement| placement.half_mm[1] * 2.0 * PT_PER_MM)
            .fold(0.0f64, f64::max);
        let va_box_pt = match &three_d.page {
            Some(page) => page.box_half_mm()[1] * 2.0 * PT_PER_MM,
            None if placement_box_pt > 0.0 => placement_box_pt,
            None => 400.0,
        };
        let mut out = vec![(self.stream, stream_object(model, va_box_pt))];
        for (placement, (annot, appearance)) in three_d.placements.iter().zip(&self.placements) {
            let page_h = pages.get(placement.page).map_or(297.0, |page| page.height_mm.max(1.0));
            let rect = rect_pt(placement.centre_mm, placement.half_mm, page_h);
            let view = model.view(&placement.view);
            let default = match view {
                Some(view) => view_dict(
                    model,
                    view,
                    &format!("{} {}", placement.id, view.name),
                    Projection3d::Orthographic { scale: placement.scale * PT_PER_MM },
                ),
                None => "0".to_string(),
            };
            out.push((
                *annot,
                format!(
                    "<< /Type /Annot /Subtype /3D /Rect {} /F 4 /P {} 0 R /NM ({}) \
                     /Contents (Click to turn this view in 3D: {}) /3DD {} 0 R /3DV {} \
                     /3DA << /A /XA /D /PC /TB true /NP false >> /AP << /N {} 0 R >> >>",
                    rect_text(rect),
                    first_page + placement.page * 2,
                    string_literal(&placement.id),
                    string_literal(&view.map_or(placement.view.clone(), |view| view.name.clone())),
                    self.stream,
                    default,
                    appearance
                ),
            ));
            out.push((*appearance, empty_appearance(rect)));
        }
        if let (Some(page), Some((annot, appearance, links))) = (&three_d.page, &self.page) {
            let page_number = first_page + self.page_index * 2;
            let rect = rect_pt(page.box_centre_mm(), page.box_half_mm(), page.height_mm);
            let default = match (model.views.get(page.default_view), page.poster_scale) {
                (Some(view), Some(scale)) => {
                    view_dict(model, view, &view.name, Projection3d::Orthographic { scale: scale * PT_PER_MM })
                }
                _ => page.default_view.to_string(),
            };
            out.push((
                *annot,
                format!(
                    "<< /Type /Annot /Subtype /3D /Rect {} /F 4 /P {page_number} 0 R /NM (MODEL3D) \
                     /Contents (The 3D model with its PMI views) /3DD {} 0 R /3DV {default} \
                     /3DA << /A /PO /D /PC /TB true /NP true >> /AP << /N {appearance} 0 R >> >>",
                    rect_text(rect),
                    self.stream
                ),
            ));
            out.push((*appearance, empty_appearance(rect)));
            for (button, link) in page.buttons.iter().zip(links) {
                let [x0, y0, x1, y1] = button.rect_mm;
                let r = rect_pt([(x0 + x1) * 0.5, (y0 + y1) * 0.5], [(x1 - x0) * 0.5, (y1 - y0) * 0.5], page.height_mm);
                out.push((
                    *link,
                    format!(
                        "<< /Type /Annot /Subtype /Link /Rect {} /Border [0 0 0] /P {page_number} 0 R \
                         /A << /S /GoTo3DView /TA {annot} 0 R /V {} >> >>",
                        rect_text(r),
                        button.view
                    ),
                ));
            }
        }
        out
    }
}

/// What a PDF carries in 3D, read back from its bytes: the count of 3D
/// annotations, of views in the stream's `/VA`, of view links, and the U3D
/// stream's decoded size — what the export door reports about the file.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize)]
pub struct Summary {
    pub annotations: usize,
    pub views: usize,
    pub links: usize,
    #[serde(rename = "u3dBytes")]
    pub u3d_bytes: usize,
}

/// [`Summary`] of a file this module wrote; `None` when it carries no 3D.
pub fn summary(pdf: &[u8]) -> Option<Summary> {
    let text = String::from_utf8_lossy(pdf);
    let stream_at = text.find("/Subtype /U3D")?;
    let dict_end = text[stream_at..].find("\nstream\n")? + stream_at;
    let dict = &text[stream_at..dict_end];
    let data_start = dict_end + "\nstream\n".len();
    let data_end = text[data_start..].find("~>")? + data_start;
    let data = &text[data_start..data_end];
    let mut u3d_bytes = 0;
    let mut group = 0;
    for c in data.chars().filter(|c| !c.is_whitespace()) {
        if c == 'z' {
            u3d_bytes += 4;
        } else {
            group += 1;
            if group == 5 {
                u3d_bytes += 4;
                group = 0;
            }
        }
    }
    if group > 0 {
        u3d_bytes += group - 1;
    }
    Some(Summary {
        annotations: text.matches("/Subtype /3D /Rect").count(),
        views: dict.matches("/Type /3DView").count(),
        links: text.matches("/S /GoTo3DView").count(),
        u3d_bytes,
    })
}

