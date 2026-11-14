//! Manufacturing instructions out of the app: the printed PDF, each step's
//! picture as PNG and SVG, and the interactive player's data and offline
//! `.html`.
//!
//! The PDF and the SVG are sheets ([`crate::sheets::print`]): each step's view
//! is projected as a placed view, hidden lines removed, exactly as a drawing
//! sheet draws a PMI view — by the exact pass over the solids' B-reps, which
//! the export asks the runner for first ([`EngineState::instructions_topology_ready`]),
//! as a sheet export does. A B-rep that has not arrived leaves its step
//! pictures to the mesh approximation, and the file says so
//! (`data-hidden-line` on the SVG's view, `/HiddenLine` in the PDF's Info).
//! The PNG is the thumbnail rasterizer from the step's camera, the parts
//! earlier steps placed drawn grey. None of them reads what the viewport
//! shows: a step shown in the player, its ghosts and an insertion half way
//! home do not change a file.
//!
//! The player data carries one mesh per part, each node with its placement,
//! each step's camera, picture and view PMI (laid out as the 3D PDF lays it
//! out, [`crate::sheets::pdf3d::view_annotations`]), and the assembly's tree
//! with a thumbnail per part (the page's Model and Parts tabs).

use super::{component_move::now_ms, EngineState};
use crate::scene::SolidDisplay;
use brep_instructions::wizard;
use crate::sheets::print::{self, PictureAt};
use crate::sheets::project::{poses_of, project_view_through, to_paper, Deferred, LineArtLayer, LinesMode, Model, Unposed, ViewDrawing, ViewFrame};
use crate::sheets::PlacedView;
use crate::thumbnail;
use crate::thumbnail::line_art;
use brep_instructions::edit;
use brep_instructions::model::{Arrow, Style};
use brep_instructions::layout::{self, Rect};
use brep_instructions::player::{self, Mode, PlayerData, PlayerMesh, PlayerNode};
use brep_kernel::PmiProjection;
use brep_kernel::PmiView;
use serde_json::json;
use std::collections::HashMap;

/// A per-step PNG's edge, in pixels.
pub const IMAGE_PX: u32 = 800;

/// The colour a PNG draws a part an earlier step placed in (sRGB 0..1).
const PLACED_RGB: [f32; 3] = [0.80, 0.81, 0.83];

/// A step picture's edge in the player data, in pixels: the picture the
/// player shows without WebGL.
const PLAYER_IMAGE_PX: u32 = 480;

/// A part thumbnail's edge in the player data, in pixels.
const PLAYER_THUMBNAIL_PX: u32 = 96;

const IDENTITY: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

/// How long an export waits for the runner's topology reply before it draws
/// what it has ([`EngineState::instructions_topology_ready`]).
pub const TOPOLOGY_WAIT: std::time::Duration = std::time::Duration::from_secs(120);

/// A PDF prepared over successive frames. Heavy exact passes run on the history runner.
/// Dropping this job cancels delivery; the document stamp prevents mixed-revision output.
pub struct InstructionsPdfExport {
    block: brep_instructions::model::Instructions,
    layout: Vec<layout::Page>,
    todo: Vec<(PictureAt, String, Rect)>,
    pictures: HashMap<PictureAt, ViewDrawing>,
    next: usize,
    stamp: (u64, u64),
    started: f64,
    unposed: Option<Unposed>,
    views: Vec<PmiView>,
}

impl InstructionsPdfExport {
    pub fn progress(&self) -> (usize, usize) { (self.next, self.todo.len()) }
}

impl EngineState {
    /// Begin without waiting for topology or projecting any picture.
    pub fn begin_instructions_pdf(&mut self) -> Result<InstructionsPdfExport, String> {
        if self.run_pending() { return Err("wait for the model to finish rebuilding before exporting".into()); }
        let block = self.instructions_required()?;
        let layout = print::layout(&block)?;
        let todo = print::picture_rects(&block, &layout);
        let names: Vec<String> = self.scene.solids().iter().filter(|solid| !solid.is_sketch).map(|solid| solid.name.clone()).collect();
        self.request_exact_solids(names.iter().map(String::as_str));
        Ok(InstructionsPdfExport {
            block, layout, todo, pictures: HashMap::new(), next: 0,
            stamp: (self.applied_generation(), self.history_revision()),
            started: now_ms(), unposed: None, views: self.pmi_state().views,
        })
    }

    /// Poll at most one picture per frame. Never wait for the runner or fall back
    /// to computing hidden lines on the UI thread if the runner refuses a job.
    pub fn poll_instructions_pdf(&mut self, job: &mut InstructionsPdfExport) -> Result<Option<(Vec<u8>, usize)>, String> {
        if job.stamp != (self.applied_generation(), self.history_revision()) {
            return Err("the document changed during PDF export; export again".into());
        }
        self.pump_topology();
        self.pump_sheet_lines();
        if self.topology_pending() {
            if now_ms() - job.started >= TOPOLOGY_WAIT.as_secs_f64() * 1000.0 { return Err("timed out loading exact geometry for PDF export".into()); }
            return Ok(None);
        }
        if self.sheet_lines_pending() { return Ok(None); }
        if job.unposed.is_none() { job.unposed = Some(self.instructions_export_displays()); }
        if let Some((at, step_id, rect)) = job.todo.get(job.next) {
            if let Some(view) = job.block.step(step_id).and_then(|step| step.view.as_deref()) {
                let placed = self.instructions_placed_names(&job.block, step_id);
                let pending = std::cell::Cell::new(false);
                // Discover at most one cache miss, then submit it after the scene borrow ends.
                let mut requests = Vec::new();
                let mut defer = |request| { requests.push(request); pending.set(true); Deferred::Pending };
                let arrows = (job.block.template.style == Style::LineArt).then(|| self.instructions_step_arrows(&job.block, step_id));
                let drawing = self.instructions_project_with(job.unposed.as_ref().unwrap(), &job.views, view, *rect, &placed,
                    arrows.as_deref(), &mut LinesMode::Export(&mut defer), &pending);
                for request in requests {
                    if matches!(self.sheet_lines.defer(self.runner.as_mut(), request), Deferred::Here) {
                        return Err("the background runner could not draw the PDF picture; export again".into());
                    }
                }
                let Some(drawing) = drawing else { return Ok(None) };
                if !drawing.error.is_empty() { return Err(drawing.error); }
                job.pictures.insert(*at, drawing);
            }
            job.next += 1;
            return Ok(None);
        }
        let pages = print::pages(&job.block, &job.layout, &job.pictures);
        let refs = pages.iter().collect::<Vec<_>>();
        Ok(Some((crate::sheets::pdf::write_pdf(&print::title(&job.block), &refs), pages.len())))
    }

    /// Ask the runner for the B-rep of every solid the step pictures draw
    /// (see [`Self::request_exact_solids`]) and wait for the reply, up to
    /// [`TOPOLOGY_WAIT`], on a runner that can be waited on (the Inline runner
    /// answers before the request returns; the thread runner is waited on; a
    /// worker cannot be, and the caller holds the export until
    /// [`Self::topology_pending`] clears). Returns whether every drawn solid's
    /// B-rep is in the scene now: the exact pass draws the pictures. Otherwise
    /// the mesh approximation does, and the file says so.
    /// Ask for the exact solids the step pictures draw WITHOUT waiting:
    /// returns whether they are all here already. The frame's `pump` lands the
    /// replies; a caller that may not block (automation) checks
    /// [`Self::topology_pending`] on later frames.
    pub fn instructions_request_topology(&mut self) -> bool {
        let names: Vec<String> = self.scene.solids().iter().filter(|solid| !solid.is_sketch).map(|solid| solid.name.clone()).collect();
        self.request_exact_solids(names.iter().map(String::as_str));
        self.pump_topology();
        !self.topology_pending()
    }

    pub fn instructions_topology_ready(&mut self) -> bool {
        let names: Vec<String> = self.scene.solids().iter().filter(|solid| !solid.is_sketch).map(|solid| solid.name.clone()).collect();
        self.request_exact_solids(names.iter().map(String::as_str));
        let deadline = now_ms() + TOPOLOGY_WAIT.as_secs_f64() * 1000.0;
        while self.topology_pending() {
            let now = now_ms();
            if now >= deadline || !self.runner.wait_topology(std::time::Duration::from_secs_f64((deadline - now) / 1000.0)) {
                break;
            }
            self.pump_topology();
        }
        !self.topology_pending()
    }

    /// Every display as the files draw it: unposed (the explode originals of
    /// the view the viewport shows) and shown, so only each step view's own
    /// hidden list and poses decide what a picture holds.
    fn instructions_export_displays(&self) -> Unposed {
        self.scene
            .solids()
            .iter()
            .map(|solid| {
                let mut display = self.pmi_explode_originals.get(&solid.name).unwrap_or(solid).clone();
                display.visible = true;
                (solid.name.clone(), display)
            })
            .collect()
    }

    /// The PMI view `view_id` projected into `rect`: projected once at scale
    /// 1 to measure it, then placed at the scale and position that fit it.
    /// The parts `placed` by earlier steps are drawn in the model stroke and
    /// the rest, the parts the step adds, in the highlight stroke
    /// ([`ViewDrawing::highlighted`]). Visibility is decided once over every
    /// part; a second pass at the same placement with the placed parts hidden
    /// says which visible runs lie on the added parts' own edges.
    fn instructions_project(&self, unposed: &Unposed, views: &[PmiView], view_id: &str, rect: Rect, placed: &[String], line_art: Option<&[Arrow]>) -> ViewDrawing {
        self.instructions_project_with(unposed, views, view_id, rect, placed, line_art, &mut LinesMode::Compute, &std::cell::Cell::new(false))
            .expect("synchronous projection")
    }

    /// With `line_art` (the step's arrows, [`Self::instructions_step_arrows`])
    /// the picture is a Line art one: its frame takes the arrows in, and the
    /// drawing carries the fills, outlines and arrows ([`LineArtLayer`]).
    #[allow(clippy::too_many_arguments)]
    fn instructions_project_with(
        &self,
        unposed: &Unposed,
        views: &[PmiView],
        view_id: &str,
        rect: Rect,
        placed: &[String],
        line_art: Option<&[Arrow]>,
        mode: &mut LinesMode,
        pending: &std::cell::Cell<bool>,
    ) -> Option<ViewDrawing> {
        let env = self.sheet_env();
        let model = Model { scene: &self.scene, unposed, env: &env };
        let view = views.iter().find(|view| view.id == view_id);
        let report = self.pmi_report.as_ref().and_then(|report| report.view(view_id));
        let placement = |position: [f64; 2], scale: f64| -> PlacedView {
            serde_json::from_value(json!({ "id": format!("IV-{view_id}"), "view": view_id, "position": position, "scale": scale }))
                .expect("a placement from its id, view, position and scale")
        };
        let measured = project_view_through(&model, &placement([0.0, 0.0], 1.0), view, report, None, None, mode);
        if pending.get() { return None; }
        if !measured.error.is_empty() {
            return Some(measured);
        }
        let frame = view.and_then(|view| view.camera.as_ref()).map(ViewFrame::of);
        // A Line art picture's parts, posed as the lines are, and its arrows.
        let art = match (line_art, &frame) {
            (Some(arrows), Some(frame)) => Some(line_art_scene(unposed, report, &measured.solids, placed, arrows, frame)),
            _ => None,
        };
        let mut bounds = measured.bounds;
        if let (Some(art), Some(frame)) = (&art, &frame) {
            // The frame takes in where the arrows start.
            for (tail, tip) in &art.arrows {
                for p in [tail, tip] {
                    let v = frame.to_view(*p);
                    let (x, y) = (v[0], -v[1]);
                    bounds = [bounds[0].min(x), bounds[1].min(y), bounds[2].max(x), bounds[3].max(y)];
                }
            }
        }
        let (position, scale) = print::fit(bounds, rect);
        let mut drawing = project_view_through(&model, &placement(position, scale), view, report, None, None, mode);
        if pending.get() { return None; }
        let Some(view) = view else { return Some(drawing) };
        if placed.is_empty() || drawing.edges.is_empty() {
            // Nothing placed yet: everything drawn is being added.
            drawing.highlighted = std::mem::take(&mut drawing.edges);
            if let (Some(art), Some(frame)) = (&art, &frame) {
                line_art_layer(&mut drawing, art, frame, &placement(position, scale), rect);
            }
            return Some(drawing);
        }
        // The same view with what earlier steps placed hidden too (a
        // transient hide step on a copy, never stored).
        let mut added_only = view.clone();
        let extra: Vec<String> = placed.iter().filter(|name| !added_only.hides(name)).cloned().collect();
        added_only.annotations.push(brep_kernel::PmiAnnotation::hide_step("HID-placed", &extra, true));
        let alone = project_view_through(&model, &placement(position, scale), Some(&added_only), report, None, None, mode);
        if pending.get() { return None; }
        if !alone.error.is_empty() { return Some(alone); }
        let (edges, highlighted) = split_highlighted(std::mem::take(&mut drawing.edges), &alone.edges);
        drawing.edges = edges;
        drawing.highlighted = highlighted;
        if let (Some(art), Some(frame)) = (&art, &frame) {
            line_art_layer(&mut drawing, art, frame, &placement(position, scale), rect);
        }
        Some(drawing)
    }

    /// The parts earlier steps placed, by solid name, for `step_id`'s picture.
    fn instructions_placed_names(&self, block: &brep_instructions::model::Instructions, step_id: &str) -> Vec<String> {
        let names: Vec<String> = self.scene.solids().iter().map(|solid| solid.name.clone()).collect();
        edit::presentation(block, step_id, &names).map(|presentation| presentation.placed).unwrap_or_default()
    }

    /// The instructions as one PDF: the cover, the parts list, the tools list
    /// and the steps on the document's page template, the pictures drawn by
    /// the exact pass ([`Self::instructions_topology_ready`]). Returns the
    /// bytes and the page count.
    pub fn instructions_pdf(&mut self) -> Result<(Vec<u8>, usize), String> {
        let block = self.instructions_required()?;
        self.instructions_topology_ready();
        let layout = print::layout(&block)?;
        let pmi = self.pmi_state();
        let unposed = self.instructions_export_displays();
        let mut pictures: HashMap<PictureAt, ViewDrawing> = HashMap::new();
        for (at, step_id, rect) in print::picture_rects(&block, &layout) {
            if let Some(view) = block.step(&step_id).and_then(|step| step.view.clone()) {
                let placed = self.instructions_placed_names(&block, &step_id);
                let arrows = (block.template.style == Style::LineArt).then(|| self.instructions_step_arrows(&block, &step_id));
                pictures.insert(at, self.instructions_project(&unposed, &pmi.views, &view, rect, &placed, arrows.as_deref()));
            }
        }
        let pages = print::pages(&block, &layout, &pictures);
        let refs: Vec<&crate::sheets::project::SheetDrawing> = pages.iter().collect();
        Ok((crate::sheets::pdf::write_pdf(&print::title(&block), &refs), pages.len()))
    }

    /// One step's picture as SVG: its number and title over its view, hidden
    /// lines removed by the exact pass ([`Self::instructions_topology_ready`]).
    pub fn instructions_step_svg(&mut self, step_id: &str) -> Result<String, String> {
        let block = self.instructions_required()?;
        let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
        self.instructions_topology_ready();
        let placed = self.instructions_placed_names(&block, step_id);
        let arrows = (block.template.style == Style::LineArt).then(|| self.instructions_step_arrows(&block, step_id));
        let picture = step.view.as_deref().map(|view| {
            self.instructions_project(&self.instructions_export_displays(), &self.pmi_state().views, view, print::image_picture_rect(), &placed, arrows.as_deref())
        });
        Ok(crate::sheets::svg::to_svg(&print::step_image(&block, step_id, picture)?))
    }

    /// One step's picture as a `size`-pixel square PNG from its view's camera:
    /// what the step adds in its colours with its edges, what earlier steps
    /// placed in grey, what comes later left out, each part where the view's
    /// explode rows put it.
    pub fn instructions_step_png(&self, step_id: &str, size: u32) -> Result<Vec<u8>, String> {
        let block = self.instructions_required()?;
        let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
        let view_id = step.view.as_deref().ok_or_else(|| format!("step '{step_id}' has no view: run the wizard or pick one"))?;
        let pmi = self.pmi_state();
        let view = pmi.views.iter().find(|view| view.id == view_id).ok_or_else(|| format!("no PMI view '{view_id}'"))?;
        let camera = view.camera.as_ref().ok_or_else(|| format!("PMI view '{view_id}' has no captured camera"))?;
        let frame = ViewFrame::of(camera);
        let back = frame.forward.map(|c| -c);
        let report = self.pmi_report.as_ref().and_then(|report| report.view(view_id));
        let names: Vec<String> = self.scene.solids().iter().map(|solid| solid.name.clone()).collect();
        let presentation = edit::presentation(&block, step_id, &names)?;
        let unposed = self.instructions_export_displays();
        if block.template.style == Style::LineArt {
            let mut art = line_art::LineArt::default();
            let mut bounds: HashMap<&str, ([f64; 3], [f64; 3])> = HashMap::new();
            for name in &names {
                if view.hides(name) || presentation.hidden.contains(name) {
                    continue;
                }
                let Some(display) = unposed.get(name).filter(|display| !display.is_sketch) else { continue };
                let Some(body) = posed_body(display, &poses_of(report, name), false) else { continue };
                if presentation.placed.contains(name) {
                    art.placed.push(body);
                } else {
                    bounds.insert(name.as_str(), body_bounds(&body));
                    art.adding.push(body);
                }
            }
            // Each arrow over the seated parts it moves, from where they start.
            for arrow in self.instructions_step_arrows(&block, step_id) {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for (name, (a, b)) in &bounds {
                    if under(name, &arrow.occurrence) {
                        lo = std::array::from_fn(|k| lo[k].min(a[k]));
                        hi = std::array::from_fn(|k| hi[k].max(b[k]));
                    }
                }
                if lo[0] <= hi[0] {
                    art.arrows.extend(line_art::still_arrow(lo, hi, arrow.start_offset()));
                }
            }
            return line_art::render_png(&art, size, (frame.right, frame.up, back))
                .ok_or_else(|| format!("step '{step_id}' shows nothing to draw"));
        }
        let mut capture = thumbnail::Capture::default();
        for name in &names {
            if view.hides(name) || presentation.hidden.contains(name) {
                continue;
            }
            let Some(display) = unposed.get(name).filter(|display| !display.is_sketch) else { continue };
            let Some(body) = posed_body(display, &poses_of(report, name), presentation.placed.contains(name)) else { continue };
            capture.bodies.push(body);
        }
        thumbnail::render_png_from(&capture, size, (frame.right, frame.up, back))
            .ok_or_else(|| format!("step '{step_id}' shows nothing to draw"))
    }

    /// The interactive player's data: the steps over every drawn solid, each
    /// step's camera from its view, its picture and its view's PMI, and the
    /// assembly's tree with each part's thumbnail. What PLM serves per revision and the offline copy inlines.
    pub fn instructions_player_data(&self) -> Result<PlayerData, String> {
        let block = self.instructions_required()?;
        let unposed = self.instructions_export_displays();
        let mut scene = player::Scene::default();
        // A step's motion is keyed by the assembly's own occurrences; the
        // nodes sit under the wrapper.
        scene.wrapper = self.instructions_wrapper().map(|(id, _)| id).unwrap_or_default();
        let flat = |values: &[[f32; 3]]| values.iter().flatten().copied().collect::<Vec<f32>>();
        for solid in self.scene.solids() {
            let Some(display) = unposed.get(&solid.name).filter(|display| !display.is_sketch && !display.mesh.indices.is_empty()) else {
                continue;
            };
            let color = display.color_override.unwrap_or_else(|| crate::color::solid_color_srgb(&display.name).map(|c| c as f32));
            scene.nodes.push(PlayerNode { occurrence: solid.name.clone(), mesh: scene.meshes.len(), matrix: IDENTITY, color });
            let edges = thumbnail::body_of(display).map(|body| body.edges).unwrap_or_default();
            scene.meshes.push(
                PlayerMesh::new(&solid.name, &flat(&display.mesh.positions), &flat(&display.mesh.normals), &display.mesh.indices)
                    .with_edges(&edges),
            );
        }
        // One mesh per part: a node's key is its part (the PLM id or the
        // name) and the member inside it, from the occurrence that holds it,
        // and its placement is the engine's pose of that occurrence.
        let level = self.instructions_level()?;
        let components = wizard::flatten(&level);
        let mut keys = Vec::with_capacity(scene.nodes.len());
        let mut placements = Vec::with_capacity(scene.nodes.len());
        for node in &scene.nodes {
            let holder = components
                .iter()
                .filter(|(_, c)| node.occurrence.strip_prefix(c.path.as_str()).is_some_and(|rest| rest.starts_with(':')))
                .max_by_key(|(_, c)| c.path.len())
                .map(|(_, c)| *c);
            keys.push(holder.map(|c| format!("{}|{}", c.part_key(), &node.occurrence[c.path.len()..])).unwrap_or_else(|| node.occurrence.clone()));
            placements.push(holder.and_then(|c| self.instructions_occurrence_placement(&c.path)));
        }
        scene.share_meshes(&keys, &placements);
        // A part's thumbnail, once per part: every solid of `occurrence` (a
        // part's, or a sub-assembly's), unposed.
        let thumbnail_of = |scene: &mut player::Scene, part: &str, occurrence: &str| {
            if scene.thumbnails.contains_key(part) {
                return;
            }
            let mut capture = thumbnail::Capture::default();
            for solid in self.scene.solids() {
                let name = solid.name.as_str();
                let inside = name == occurrence || name.strip_prefix(occurrence).is_some_and(|rest| rest.starts_with(':'));
                if let Some(display) = unposed.get(name).filter(|display| inside && !display.is_sketch) {
                    capture.bodies.extend(thumbnail::body_of(display));
                }
            }
            if let Some(png) = thumbnail::render_png(&capture, PLAYER_THUMBNAIL_PX) {
                scene.thumbnails.insert(part.to_string(), player::png_data_url(&png));
            }
        };
        let pmi = self.pmi_state();
        for (_, step) in block.steps() {
            if step.arrows.is_empty() {
                let derived = self.instructions_step_arrows(&block, &step.id);
                if !derived.is_empty() {
                    scene.arrows.insert(step.id.clone(), derived);
                }
            }
            // The step view's explode poses, a matrix per posed node, in the
            // order the pictures apply them.
            if let Some(view_id) = step.view.as_deref() {
                let report = self.pmi_report.as_ref().and_then(|report| report.view(view_id));
                let posed: Vec<player::Pose> = scene
                    .nodes
                    .iter()
                    .enumerate()
                    .filter_map(|(node, n)| {
                        let poses = crate::sheets::project::poses_of(report, &n.occurrence);
                        (!poses.is_empty()).then(|| player::Pose { node, matrix: poses.iter().fold(IDENTITY, |m, pose| mul4(pose.matrix(), m)) })
                    })
                    .collect();
                if !posed.is_empty() {
                    scene.poses.insert(step.id.clone(), posed);
                }
            }
            let view = step.view.as_deref().and_then(|id| pmi.views.iter().find(|view| view.id == id));
            let camera = view.and_then(|view| view.camera.as_ref());
            // The PMI the view shows, laid out as the 3D PDF lays it out.
            if let (Some(view), Some(camera)) = (view, camera) {
                if let Some(rows) = self.pmi_report.as_ref().and_then(|report| report.view(&view.id)) {
                    let drawn = crate::sheets::pdf3d::view_annotations(view, camera, rows);
                    let texts: Vec<String> = drawn.iter().map(|(row, _)| row.text.clone()).filter(|text| !text.trim().is_empty()).collect();
                    let lines: Vec<Vec<[f64; 3]>> = drawn.iter().flat_map(|(_, d)| d.lines.iter().cloned()).collect();
                    let arrows: Vec<[[f64; 3]; 3]> = drawn.iter().flat_map(|(_, d)| d.arrows.iter().copied()).collect();
                    if let Some(drawing) = player::PlayerPmi::new(&lines, &arrows, drawn.len(), texts) {
                        scene.pmi.insert(step.id.clone(), drawing);
                    }
                }
            }
            if let Some(camera) = camera {
                let (fov_deg, half_height) = match camera.projection {
                    PmiProjection::Perspective { fov_y_deg } => (Some(fov_y_deg), None),
                    PmiProjection::Orthographic { half_height } => (None, Some(half_height)),
                };
                scene.cameras.insert(step.id.clone(), player::Camera { eye: camera.eye, target: camera.target, up: camera.up, fov_deg, half_height });
                if let Ok(png) = self.instructions_step_png(&step.id, PLAYER_IMAGE_PX) {
                    scene.step_images.insert(step.id.clone(), player::png_data_url(&png));
                }
            }
            for add in &step.adds {
                // The part's first occurrence: its solid, or a sub-assembly's.
                if let Some(occurrence) = add.occurrences.first() {
                    thumbnail_of(&mut scene, &add.part, occurrence);
                }
            }
        }
        // The assembly's tree, each part with its thumbnail: the page's Model
        // and Parts tabs.
        scene.assembly = player::PlayerComponent::rows(&level);
        let parts: Vec<(String, String)> = scene.assembly.iter().map(|row| (row.part.clone(), row.path.clone())).collect();
        for (part, path) in parts {
            thumbnail_of(&mut scene, &part, &path);
        }
        PlayerData::build(&block, scene)
    }

    /// The engine's placement of an occurrence of the wrapper's assembly
    /// (`ACOMP1:ACOMP3`), column-major for the player: the component
    /// features' solved poses composed along the path, the top document's
    /// then each sub-assembly document's, as the kernel places them. `None`
    /// for a path the documents do not place.
    pub fn instructions_occurrence_placement(&self, occurrence: &str) -> Option<[f64; 16]> {
        let mut world: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let mut features: Vec<serde_json::Value> = self.history.features().to_vec();
        let mut library = self.history.parts_library().clone();
        for id in occurrence.split(':') {
            let feature = features
                .iter()
                .find(|f| super::components::is_acomp_feature_type(f["type"].as_str().unwrap_or_default()) && f["inputParams"]["id"].as_str() == Some(id))?;
            let pose = crate::pipeline::pose_params_to_transform(&feature["inputParams"]["transform"]);
            // Row-major product: the sub-assembly's pose applies first.
            let mut out = [0.0; 16];
            for row in 0..4 {
                for col in 0..4 {
                    out[row * 4 + col] = (0..4).map(|k| world[row * 4 + k] * pose[k * 4 + col]).sum();
                }
            }
            world = out;
            let part = feature["inputParams"]["partName"].as_str()?.to_string();
            let document = library[part.as_str()]["document"].clone();
            features = document["features"].as_array().cloned().unwrap_or_default();
            library = document["partsLibrary"].clone();
        }
        // Column-major for the page.
        Some(std::array::from_fn(|i| world[(i % 4) * 4 + i / 4]))
    }

    /// The offline player: one self-contained `.html` with the player data
    /// inlined, in `mode`. Returns the page and the data it carries.
    pub fn instructions_html(&self, mode: Mode) -> Result<(String, PlayerData), String> {
        let data = self.instructions_player_data()?;
        Ok((player::offline_html(&data, mode), data))
    }
}

/// `path` is `occurrence` or a solid under it.
fn under(path: &str, occurrence: &str) -> bool {
    path == occurrence || path.strip_prefix(occurrence).is_some_and(|rest| rest.starts_with(':'))
}

/// A body's world bounds over the vertices its triangles use.
fn body_bounds(body: &thumbnail::Body) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &i in &body.indices {
        let p = body.positions[i as usize];
        for k in 0..3 {
            lo[k] = lo[k].min(p[k] as f64);
            hi[k] = hi[k].max(p[k] as f64);
        }
    }
    (lo, hi)
}

/// Column-major 4×4 product `a · b` (b applied first).
fn mul4(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for col in 0..4 {
        for row in 0..4 {
            out[col * 4 + row] = (0..4).map(|k| a[k * 4 + row] * b[col * 4 + k]).sum();
        }
    }
    out
}

/// `display`'s body for a step picture: posed by `poses` in order (points,
/// and normals through the same map), grey and without edges when an earlier
/// step `placed` it.
fn posed_body(display: &SolidDisplay, poses: &[crate::sheets::project::SolidPose], placed: bool) -> Option<thumbnail::Body> {
    let mut body = thumbnail::body_of(display)?;
    if !poses.is_empty() {
        let pose = |p: [f64; 3]| poses.iter().fold(p, |p, pose| pose.apply(p));
        let f64s = |p: [f32; 3]| [p[0] as f64, p[1] as f64, p[2] as f64];
        let f32s = |p: [f64; 3]| [p[0] as f32, p[1] as f32, p[2] as f32];
        for (i, normal) in body.normals.iter_mut().enumerate() {
            let Some(&at) = body.positions.get(i) else { break };
            let (p, n) = (f64s(at), f64s(*normal));
            let (from, to) = (pose(p), pose([p[0] + n[0], p[1] + n[1], p[2] + n[2]]));
            *normal = f32s([to[0] - from[0], to[1] - from[1], to[2] - from[2]]);
        }
        for position in body.positions.iter_mut() {
            *position = f32s(pose(f64s(*position)));
        }
        for line in body.edges.iter_mut() {
            for point in line.iter_mut() {
                *point = f32s(pose(f64s(*point)));
            }
        }
    }
    if placed {
        body.colors.iter_mut().for_each(|colour| *colour = PLACED_RGB);
        body.edges.clear();
    }
    Some(body)
}

/// A Line art step picture's parts and arrows in world space: each drawn
/// solid posed by its view's explode rows, with whether the step adds it,
/// and each arrow over the seated parts it moves ([`line_art::still_arrow`]).
struct LineArtScene {
    bodies: Vec<(thumbnail::Body, bool)>,
    arrows: Vec<([f64; 3], [f64; 3])>,
}

fn line_art_scene(
    unposed: &Unposed,
    report: Option<&brep_kernel::PmiViewReport>,
    solids: &[String],
    placed: &[String],
    arrows: &[Arrow],
    _frame: &ViewFrame,
) -> LineArtScene {
    let mut bodies = Vec::new();
    let mut bounds: Vec<(&str, ([f64; 3], [f64; 3]))> = Vec::new();
    for name in solids {
        let Some(display) = unposed.get(name).filter(|display| !display.is_sketch) else { continue };
        let Some(body) = posed_body(display, &poses_of(report, name), false) else { continue };
        let adding = !placed.contains(name);
        if adding {
            bounds.push((name.as_str(), body_bounds(&body)));
        }
        bodies.push((body, adding));
    }
    let mut out = Vec::new();
    for arrow in arrows {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for (name, (a, b)) in &bounds {
            if under(name, &arrow.occurrence) {
                lo = std::array::from_fn(|k| lo[k].min(a[k]));
                hi = std::array::from_fn(|k| hi[k].max(b[k]));
            }
        }
        if lo[0] <= hi[0] {
            out.extend(line_art::still_arrow(lo, hi, arrow.start_offset()));
        }
    }
    LineArtScene { bodies, arrows: out }
}

/// The paper cell of a Line art picture's coverage grid, mm.
const LINE_ART_CELL_MM: f64 = 0.12;
/// How far (mm) to either side of a line its two sides are read.
const LINE_ART_SIDE_MM: f64 = 0.3;

/// Put the Line art layer on a step picture drawn at `placed`: the visible
/// faces as flat triangles painted back to front (back faces left out), the
/// lines that bound a part against the ground or another tier moved into
/// the outlines, and the arrows on paper.
fn line_art_layer(drawing: &mut ViewDrawing, art: &LineArtScene, frame: &ViewFrame, placed: &PlacedView, rect: Rect) {
    let paper = |p: [f64; 3]| -> ([f64; 2], f64) {
        let v = frame.to_view(p);
        (to_paper(placed, v[0], v[1]), v[2])
    };
    let mut fills: Vec<(f64, [[f64; 2]; 3], bool)> = Vec::new();
    for (body, adding) in &art.bodies {
        let projected: Vec<([f64; 2], f64)> = body.positions.iter().map(|p| paper([p[0] as f64, p[1] as f64, p[2] as f64])).collect();
        for tri in body.indices.chunks_exact(3) {
            let [a, b, c] = [tri[0] as usize, tri[1] as usize, tri[2] as usize];
            // A face turned away from the eye is behind the part's front.
            let n: [f64; 3] = std::array::from_fn(|k| (body.normals[a][k] + body.normals[b][k] + body.normals[c][k]) as f64);
            if n[0] * frame.forward[0] + n[1] * frame.forward[1] + n[2] * frame.forward[2] > 0.2 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() {
                continue;
            }
            let (pa, pb, pc) = (projected[a], projected[b], projected[c]);
            let area = (pb.0[0] - pa.0[0]) * (pc.0[1] - pa.0[1]) - (pc.0[0] - pa.0[0]) * (pb.0[1] - pa.0[1]);
            if area.abs() < 1e-6 {
                continue;
            }
            fills.push(((pa.1 + pb.1 + pc.1) / 3.0, [pa.0, pb.0, pc.0], *adding));
        }
    }
    // Furthest first.
    fills.sort_by(|a, b| b.0.total_cmp(&a.0));
    // Which tier shows in each cell of the picture, painted as the fills are.
    let cols = ((rect.w / LINE_ART_CELL_MM).ceil() as usize).max(1);
    let rows = ((rect.h / LINE_ART_CELL_MM).ceil() as usize).max(1);
    let mut tier = vec![0u8; cols * rows];
    for (_, t, adding) in &fills {
        let value = if *adding { 2 } else { 1 };
        let x0 = ((t.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - rect.x) / LINE_ART_CELL_MM).floor().max(0.0) as usize;
        let x1 = (((t.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max) - rect.x) / LINE_ART_CELL_MM).ceil() as usize).min(cols);
        let y0 = ((t.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - rect.y) / LINE_ART_CELL_MM).floor().max(0.0) as usize;
        let y1 = (((t.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max) - rect.y) / LINE_ART_CELL_MM).ceil() as usize).min(rows);
        let side = |a: [f64; 2], b: [f64; 2], p: [f64; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
        let orient = side(t[0], t[1], t[2]).signum();
        for y in y0..y1 {
            for x in x0..x1 {
                let p = [rect.x + (x as f64 + 0.5) * LINE_ART_CELL_MM, rect.y + (y as f64 + 0.5) * LINE_ART_CELL_MM];
                if (0..3).all(|k| side(t[k], t[(k + 1) % 3], p) * orient >= 0.0) {
                    tier[y * cols + x] = value;
                }
            }
        }
    }
    let tier_at = |p: [f64; 2]| -> u8 {
        let (x, y) = ((p[0] - rect.x) / LINE_ART_CELL_MM, (p[1] - rect.y) / LINE_ART_CELL_MM);
        if x < 0.0 || y < 0.0 || x >= cols as f64 || y >= rows as f64 {
            return 0;
        }
        tier[y as usize * cols + x as usize]
    };
    // A segment is on its part's outline when either side shows another
    // tier or the ground.
    let outline_of = |own: u8| {
        move |a: [f64; 2], b: [f64; 2]| -> bool {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let length = (dx * dx + dy * dy).sqrt();
            if !(length > 1e-9) {
                return false;
            }
            let (nx, ny) = (-dy / length * LINE_ART_SIDE_MM, dx / length * LINE_ART_SIDE_MM);
            let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
            tier_at([mid[0] + nx, mid[1] + ny]) != own || tier_at([mid[0] - nx, mid[1] - ny]) != own
        }
    };
    let mut outlines = Vec::new();
    let (edges, placed_outlines) = split_runs(std::mem::take(&mut drawing.edges), outline_of(1));
    outlines.extend(placed_outlines.into_iter().map(|run| (run, false)));
    let (lit, adding_outlines) = split_runs(std::mem::take(&mut drawing.highlighted), outline_of(2));
    outlines.extend(adding_outlines.into_iter().map(|run| (run, true)));
    drawing.edges = edges;
    drawing.highlighted = lit;
    let arrows = art.arrows.iter().map(|(tail, tip)| (paper(*tail).0, paper(*tip).0)).collect();
    drawing.line_art = Some(LineArtLayer {
        ground: [rect.x, rect.y, rect.x + rect.w, rect.y + rect.h],
        fills: fills.into_iter().map(|(_, t, adding)| (t, adding)).collect(),
        outlines,
        arrows,
    });
}

/// `runs` cut where `pred` changes along them: `(false runs, true runs)`.
fn split_runs(runs: Vec<Vec<[f64; 2]>>, pred: impl Fn([f64; 2], [f64; 2]) -> bool) -> (Vec<Vec<[f64; 2]>>, Vec<Vec<[f64; 2]>>) {
    let (mut no, mut yes) = (Vec::new(), Vec::new());
    for run in runs {
        let mut current: Vec<[f64; 2]> = Vec::new();
        let mut class: Option<bool> = None;
        for pair in run.windows(2) {
            let c = pred(pair[0], pair[1]);
            if class.is_some_and(|k| k != c) {
                let done = std::mem::take(&mut current);
                if class == Some(true) { yes.push(done) } else { no.push(done) }
            }
            if current.is_empty() {
                current.push(pair[0]);
            }
            current.push(pair[1]);
            class = Some(c);
        }
        if current.len() >= 2 {
            if class == Some(true) { yes.push(current) } else { no.push(current) }
        }
    }
    (no, yes)
}

/// How close (paper mm) a visible segment's midpoint must lie to one of the
/// added parts' own runs to be drawn in the highlight stroke.
const HIGHLIGHT_TOLERANCE_MM: f64 = 0.02;

/// `runs` split into the runs to draw in the model stroke and those to draw
/// highlighted: a segment is highlighted when its midpoint lies on a segment
/// of `added` (the same projection with only the added parts drawn, so every
/// run of theirs that survives occlusion by the rest is found there). Each
/// input run is cut where its class changes.
pub(super) fn split_highlighted(runs: Vec<Vec<[f64; 2]>>, added: &[Vec<[f64; 2]>]) -> (Vec<Vec<[f64; 2]>>, Vec<Vec<[f64; 2]>>) {
    let segments: Vec<([f64; 2], [f64; 2])> = added.iter().flat_map(|run| run.windows(2).map(|w| (w[0], w[1]))).collect();
    let on_added = |p: [f64; 2]| segments.iter().any(|(a, b)| point_segment_distance(p, *a, *b) <= HIGHLIGHT_TOLERANCE_MM);
    let (mut plain, mut lit) = (Vec::new(), Vec::new());
    for run in runs {
        let mut current: Vec<[f64; 2]> = Vec::new();
        let mut class: Option<bool> = None;
        for pair in run.windows(2) {
            let mid = [(pair[0][0] + pair[1][0]) * 0.5, (pair[0][1] + pair[1][1]) * 0.5];
            let highlighted = on_added(mid);
            if class.is_some_and(|c| c != highlighted) {
                let done = std::mem::take(&mut current);
                if class == Some(true) { lit.push(done) } else { plain.push(done) }
            }
            if current.is_empty() {
                current.push(pair[0]);
            }
            current.push(pair[1]);
            class = Some(highlighted);
        }
        if current.len() >= 2 {
            if class == Some(true) { lit.push(current) } else { plain.push(current) }
        }
    }
    (plain, lit)
}

fn point_segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 0.0 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) };
    ((p[0] - a[0] - t * dx).powi(2) + (p[1] - a[1] - t * dy).powi(2)).sqrt()
}
