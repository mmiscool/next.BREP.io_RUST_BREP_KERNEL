//! Picking the Work part's CONTEXT: the surrounding assembly, drawn
//! translucent in the part's frame ([`EngineState::context_scene`]). A context
//! face can be clicked and hovered, and lights in the selection colours, but it
//! is held apart from [`EngineState::emphasis`]: the part's own kernel cannot
//! resolve a context name, so no feature, reference, plugin or metadata write
//! may ever receive one. Copying its geometry (a datum on a face, edges into a
//! sketch) is what turns it into something the part owns.

use super::*;
use crate::pick::{PickCandidate, PickKind};

/// How the Work part's context is shown: on or off, and how opaque.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ContextDisplay {
    pub shown: bool,
    pub opacity: f32,
}

impl Default for ContextDisplay {
    fn default() -> Self {
        Self { shown: true, opacity: crate::render::WORK_PART_CONTEXT_RGBA[3] }
    }
}

impl EngineState {
    /// Whether the context is shown, and its opacity (0..=1).
    pub fn context_display(&self) -> (bool, f32) {
        (self.context_display.shown, self.context_display.opacity)
    }

    /// Show or hide the context and set its opacity (clamped to 0.05..=1).
    /// Hidden context is neither drawn nor picked, and drops its selection
    /// and hover. Kept across context rebuilds.
    pub fn set_context_display(&mut self, shown: bool, opacity: f32) {
        let display = ContextDisplay { shown, opacity: opacity.clamp(0.05, 1.0) };
        if display != self.context_display {
            self.context_display = display;
            if !shown {
                self.context_selected = None;
                self.context_hovered = None;
            }
            self.sync_context_highlight();
        }
    }

    /// The context FACE under CSS-pixel `(x, y)` when it is in front of the
    /// part's own geometry there (or the part has nothing there), else `None`.
    pub fn context_hit_at(&self, x: f64, y: f64) -> Option<PickCandidate> {
        if self.context_scene.solids().is_empty() {
            return None;
        }
        let options = self.pick_options();
        let front = pick::pick(&self.context_scene, &self.camera, x, y, &options)
            .into_iter()
            .filter(|candidate| candidate.kind == PickKind::Face)
            .min_by(|a, b| a.depth.total_cmp(&b.depth))?;
        let own = pick::nearest_face_depth(&self.scene, &self.camera, x, y, &options);
        // A face flush with the context (a mating face) belongs to the part.
        let tolerance = 1e-6 * front.depth.abs().max(1.0) + 1e-4;
        match own {
            Some(depth) if depth <= front.depth + tolerance => None,
            _ => Some(front),
        }
    }

    /// The context EDGE under CSS-pixel `(x, y)`, unless the part's own
    /// geometry is clearly in front of it (an edge flush with the part, a
    /// mating edge, still counts).
    pub fn context_edge_at(&self, x: f64, y: f64) -> Option<PickCandidate> {
        if self.context_scene.solids().is_empty() {
            return None;
        }
        let options = self.pick_options();
        let edge = pick::pick(&self.context_scene, &self.camera, x, y, &options)
            .into_iter()
            .filter(|candidate| candidate.kind == PickKind::Edge && !candidate.name.is_empty())
            .min_by(|a, b| a.screen_dist.total_cmp(&b.screen_dist).then(a.depth.total_cmp(&b.depth)))?;
        let own = pick::nearest_face_depth(&self.scene, &self.camera, x, y, &options);
        let tolerance = 1e-6 * edge.depth.abs().max(1.0) + 1e-4;
        match own {
            Some(depth) if depth + tolerance < edge.depth => None,
            _ => Some(edge),
        }
    }

    /// The sketch pickEdges tool on a Work part's context edge: copy it into
    /// the active sketch as plain lines / arcs / a circle
    /// ([`crate::sketch::external_ref::add_copied_edge`]), each noted "copied
    /// from `<part>`, not linked". Nothing links it back: if that part moves,
    /// the copy stays where it is. One undo step. Returns whether it copied.
    pub fn sketch_copy_context_edge(&mut self, edge: &PickCandidate) -> bool {
        let Some(polyline) = self
            .context_scene
            .solid(&edge.solid)
            .and_then(|solid| solid.edges.iter().find(|e| e.name == edge.name))
            .map(|e| e.polyline.iter().map(|p| p.map(f64::from)).collect::<Vec<_>>())
            .filter(|polyline| polyline.len() >= 2)
        else {
            return false;
        };
        let part = part_label(&edge.solid);
        let Some(sketch) = self.sketch_edit.as_mut() else {
            return false;
        };
        let uv = crate::sketch::external_ref::project_polyline(&sketch.session.plane, &polyline);
        let link = crate::sketch::external_ref::classify_uv(&uv);
        sketch.record_undo();
        let added = crate::sketch::external_ref::add_copied_edge(&mut sketch.session.doc, &link, &format!("copied from {part}, not linked"));
        if added.is_empty() {
            sketch.undo_stack.pop();
            return false;
        }
        self.resolve_active_sketch("copy context edge");
        self.refresh_sketch_overlay();
        self.hover_context(None);
        self.push_notice_as(
            NoticeSeverity::Info,
            format!("Copied an edge of {part} into the sketch as a {}: copied, not linked (it stays where it is if {part} moves)", link.kind()),
        );
        self.dirty = true;
        true
    }

    /// Select `candidate` (a [`Self::context_hit_at`] answer) as THE context
    /// selection, or clear it. `true` when it changed.
    pub fn select_context(&mut self, candidate: Option<PickCandidate>) -> bool {
        let changed = !same_entity(self.context_selected.as_ref(), candidate.as_ref());
        self.context_selected = candidate;
        if changed {
            self.sync_context_highlight();
        }
        changed
    }

    /// Light `candidate` as hovered context, or clear the hover. `true` when it
    /// changed.
    pub fn hover_context(&mut self, candidate: Option<PickCandidate>) -> bool {
        let changed = !same_entity(self.context_hovered.as_ref(), candidate.as_ref());
        self.context_hovered = candidate;
        if changed {
            self.sync_context_highlight();
        }
        changed
    }

    /// The context selection, if any.
    pub fn context_selection(&self) -> Option<&PickCandidate> {
        self.context_selected.as_ref()
    }

    /// Put the context selection and hover into the context scene's highlight
    /// list, dropping either whose face the (possibly rebuilt) context no
    /// longer has. Call after replacing [`Self::context_scene`].
    pub fn sync_context_highlight(&mut self) {
        let has = |scene: &RenderScene, c: &PickCandidate| {
            scene.solid(&c.solid).is_some_and(|solid| match c.kind {
                PickKind::Edge => solid.edges.iter().any(|e| e.name == c.name),
                _ => solid.faces.iter().any(|f| f.name == c.name),
            })
        };
        if self.context_selected.as_ref().is_some_and(|c| !has(&self.context_scene, c)) {
            self.context_selected = None;
        }
        if self.context_hovered.as_ref().is_some_and(|c| !has(&self.context_scene, c)) {
            self.context_hovered = None;
        }
        let mut highlight = Vec::new();
        if let Some(c) = &self.context_hovered {
            highlight.push((c.solid.clone(), c.name.clone(), false));
        }
        if let Some(c) = &self.context_selected {
            highlight.push((c.solid.clone(), c.name.clone(), true));
        }
        // A ghost a running instructions pick names is lit whole.
        for name in &self.context_pick_lit {
            if let Some(ghost) = self.context_scene.solid(name) {
                highlight.extend(ghost.faces.iter().map(|face| (name.clone(), face.name.clone(), true)));
            }
        }
        if self.context_scene.highlight != highlight {
            self.context_scene.highlight = highlight;
            self.dirty = true;
        }
        // The display settings, onto a (possibly rebuilt) context. Every ghost
        // comes from a visible assembly solid, so `shown` is each one's own.
        let [r, g, b, _] = crate::render::WORK_PART_CONTEXT_RGBA;
        let tint = Some([r, g, b, self.context_display.opacity]);
        if self.context_scene.tint != tint {
            self.context_scene.tint = tint;
            self.dirty = true;
        }
        let shown = self.context_display.shown;
        let flip: Vec<String> = self.context_scene.solids().iter().filter(|solid| solid.visible != shown).map(|solid| solid.name.clone()).collect();
        for name in flip {
            if let Some(solid) = self.context_scene.solid_mut(&name) {
                solid.visible = shown;
                self.dirty = true;
            }
        }
    }

    /// The plane tangent to the selected context face at the point it was
    /// picked: `(origin, unit normal)`, in the part's frame. For a flat face it
    /// is the face's own plane; for a curved one, the plane touching it there.
    pub fn context_tangent_plane(&self) -> Result<([f64; 3], [f64; 3]), String> {
        let picked = self.context_selected.as_ref().ok_or("select a face of another part first")?;
        let solid = self.context_scene.solid(&picked.solid).ok_or("that part is no longer shown")?;
        let face = solid.faces.iter().find(|face| face.name == picked.name).ok_or("that face is no longer shown")?;
        let mesh = &solid.mesh;
        let at = |index: u32| mesh.positions[index as usize].map(f64::from);
        let mut best: Option<(f64, [f64; 3])> = None;
        for triangle in face.tri_start..face.tri_start + face.tri_count {
            let base = triangle as usize * 3;
            let Some(&[i0, i1, i2]) = mesh.indices.get(base..base + 3).map(|s| <&[u32; 3]>::try_from(s).unwrap()) else { continue };
            let (a, b, c) = (at(i0), at(i1), at(i2));
            let mut normal = cross(sub(b, a), sub(c, a));
            let length = dot(normal, normal).sqrt();
            if length < 1e-12 {
                continue;
            }
            normal = normal.map(|v| v / length);
            // Outward as the shading normals say.
            let shading = [i0, i1, i2].iter().filter_map(|&i| mesh.normals.get(i as usize)).fold([0.0; 3], |acc, n| [acc[0] + n[0] as f64, acc[1] + n[1] as f64, acc[2] + n[2] as f64]);
            if dot(normal, shading) < 0.0 {
                normal = normal.map(|v| -v);
            }
            let distance = distance_to_triangle(picked.position, a, b, c);
            if best.is_none_or(|(d, _)| distance < d) {
                best = Some((distance, normal));
            }
        }
        let (_, normal) = best.ok_or("that face has no shape to place a plane on")?;
        Ok((picked.position, normal))
    }

    /// "Datum here": a datum (`D`) on the plane tangent to the selected context
    /// face at the picked point, named for the part it was copied from. It is a
    /// COPY: if that part moves, the datum stays where it is. Returns its id.
    pub fn datum_on_context(&mut self) -> Result<String, String> {
        let picked = self.context_selected.clone().ok_or("select a face of another part first")?;
        let (origin, normal) = self.context_tangent_plane()?;
        let part = part_label(&picked.solid);
        let id = self.history.next_feature_id(&format!("Copied{}", part.chars().filter(|c| c.is_ascii_alphanumeric()).collect::<String>()));
        let (a, b) = datum_angles(normal);
        let feature = serde_json::json!({
            "type": "D",
            "inputParams": {
                "id": id,
                "transform": {
                    "position": origin,
                    "rotationEuler": [a.to_degrees(), b.to_degrees(), 0.0],
                    "scale": [1.0, 1.0, 1.0],
                },
            },
            "persistentData": {},
        });
        self.add_feature(&feature.to_string())?;
        self.select_context(None);
        self.hover_context(None);
        self.push_notice_as(
            NoticeSeverity::Info,
            format!("Datum {id} placed on a face of {part}: copied, not linked (it stays where it is if {part} moves)"),
        );
        Ok(id)
    }

    /// "Sketch on this face": [`Self::datum_on_context`], then a new sketch on
    /// its XY plane. Returns the sketch's id.
    pub fn sketch_on_context(&mut self) -> Result<String, String> {
        let (origin, normal) = self.context_tangent_plane()?;
        let datum = self.datum_on_context()?;
        // The datum's own XY frame (its base axes turned by Rx(a)·Ry(b)), so
        // the sketch's axes are the plane's, not re-derived.
        let (a, b) = datum_angles(normal);
        let frame = crate::sketch::PlaneFrame {
            origin,
            x_axis: [b.cos(), a.sin() * b.sin(), -a.cos() * b.sin()],
            y_axis: [0.0, a.cos(), a.sin()],
            z_axis: [b.sin(), -b.cos() * a.sin(), a.cos() * b.cos()],
        };
        self.new_sketch_on(&format!("{datum}:XY"), frame)
    }

    /// `{"solid", "face"}` of the context selection, or `null` (automation).
    pub fn context_selection_json(&self) -> serde_json::Value {
        entity_json(self.context_selected.as_ref())
    }

    /// `{"solid", "face"}` of the hovered context face, or `null` (automation).
    pub fn context_hover_json(&self) -> serde_json::Value {
        entity_json(self.context_hovered.as_ref())
    }
}

/// The datum `rotationEuler` (radians, X then Y; Z = 0) that turns its XY
/// plane's normal +Z onto `normal`: the kernel's `rotate_euler_xyz` maps +Z to
/// `(sin b, −cos b·sin a, cos a·cos b)`.
fn datum_angles(normal: [f64; 3]) -> (f64, f64) {
    ((-normal[1]).atan2(normal[2]), normal[0].clamp(-1.0, 1.0).asin())
}

/// The part a context solid belongs to, for messages: `ACOMP2:Plate` → `Plate`.
fn part_label(solid: &str) -> String {
    solid.rsplit(':').next().unwrap_or(solid).to_string()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Distance from `p` to triangle `abc` (closest point on the triangle).
fn distance_to_triangle(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let closest = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let (d3, d4) = (dot(ab, bp), dot(ac, bp));
        let cp = sub(p, c);
        let (d5, d6) = (dot(ab, cp), dot(ac, cp));
        let (vc, vb, va) = (d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4);
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else if d6 >= 0.0 && d5 <= d6 {
            c
        } else if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            [a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v]
        } else if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            [a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w]
        } else if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            let bc = sub(c, b);
            [b[0] + bc[0] * w, b[1] + bc[1] * w, b[2] + bc[2] * w]
        } else {
            let denom = 1.0 / (va + vb + vc);
            let (v, w) = (vb * denom, vc * denom);
            [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
        }
    };
    let d = sub(p, closest);
    dot(d, d).sqrt()
}

fn entity_json(candidate: Option<&PickCandidate>) -> serde_json::Value {
    match candidate {
        Some(c) if c.kind == PickKind::Edge => serde_json::json!({ "solid": c.solid, "edge": c.name }),
        Some(c) => serde_json::json!({ "solid": c.solid, "face": c.name }),
        None => serde_json::Value::Null,
    }
}

fn same_entity(a: Option<&PickCandidate>, b: Option<&PickCandidate>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => a.solid == b.solid && a.name == b.name && a.kind == b.kind,
        (None, None) => true,
        _ => false,
    }
}

