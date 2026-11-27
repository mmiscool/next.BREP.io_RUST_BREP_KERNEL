//! The DRAFT ANALYSIS view state on the engine (`crate::draft_analysis` holds
//! the reading itself). Display only: opening, changing and closing it never
//! touches the history, the document or undo. While it is open every shown
//! solid's faces carry the analysis tint, re-read whenever the shown model
//! changes (`Self::ensure_draft_analysis_current`, run with the per-frame
//! overlay upkeep).

use super::*;
use crate::draft_analysis::{analyse_solid, face_normals, DraftClass, FaceDraft};

/// The minimum draft a fresh analysis opens with, degrees.
pub const DRAFT_ANALYSIS_DEFAULT_MIN_DEG: f64 = 1.0;

/// What the analysis was asked for and what it read.
#[derive(Debug, Clone, Default)]
pub struct DraftAnalysisState {
    /// The pull reference (a flat face, a datum plane or a straight edge), or
    /// `None` for the default pull.
    pub pull_ref: Option<String>,
    pub flip: bool,
    pub minimum_deg: f64,
    /// The pull it read with (unit, flip applied) and where it came from.
    pub pull: [f64; 3],
    pub pull_source: String,
    /// Why there is no reading (a pull reference that gives no direction).
    pub error: Option<String>,
    pub faces: Vec<FaceDraft>,
    /// The shown solids `(name, revision)` the tint was last written for.
    stamp: Vec<(String, u64)>,
}

impl EngineState {
    pub fn draft_analysis_active(&self) -> bool {
        self.draft_analysis.is_some()
    }

    /// Turn the analysis on (a no-op when it is on).
    pub fn draft_analysis_open(&mut self) {
        if self.draft_analysis.is_none() {
            self.draft_analysis = Some(DraftAnalysisState { minimum_deg: DRAFT_ANALYSIS_DEFAULT_MIN_DEG, ..Default::default() });
            self.recompute_draft_analysis();
        }
    }

    /// Turn the analysis off: every tint goes and the faces draw in their own
    /// colours again.
    pub fn draft_analysis_close(&mut self) {
        if self.draft_analysis.take().is_some() {
            let names: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
            for name in names {
                self.scene.set_face_analysis_colors(&name, &[]);
            }
            self.dirty = true;
        }
    }

    pub fn draft_analysis_toggle(&mut self) {
        if self.draft_analysis_active() {
            self.draft_analysis_close();
        } else {
            self.draft_analysis_open();
        }
    }

    /// Start the window's pull pick (one name; the form's reference field).
    pub fn begin_ref_select_for_draft_analysis(&mut self, path: Vec<String>, label: String, filter: Vec<String>, seed_names: Vec<String>) {
        let restore_index = self.history.rollback();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: "draftAnalysis".into(),
            path,
            label,
            filter,
            multiple: false,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::DraftAnalysis,
            picks: 0,
            claimed: false,
            finish_requested: false,
        });
        self.sync_ref_select_emphasis();
    }

    /// Change what the analysis reads with; `None` leaves a setting as it is
    /// (`Some(None)` clears the pull reference). Opens the analysis if it is
    /// off. A minimum outside 0..90° is refused and nothing changes.
    pub fn draft_analysis_set(
        &mut self,
        pull_ref: Option<Option<String>>,
        flip: Option<bool>,
        minimum_deg: Option<f64>,
    ) -> Result<(), String> {
        if let Some(m) = minimum_deg {
            if !m.is_finite() || !(0.0..90.0).contains(&m) {
                return Err(format!("the minimum draft must be at least 0° and under 90° (got {m})"));
            }
        }
        self.draft_analysis_open();
        let state = self.draft_analysis.as_mut().expect("opened above");
        if let Some(pull_ref) = pull_ref {
            state.pull_ref = pull_ref.filter(|name| !name.trim().is_empty());
        }
        if let Some(flip) = flip {
            state.flip = flip;
        }
        if let Some(m) = minimum_deg {
            state.minimum_deg = m;
        }
        self.recompute_draft_analysis();
        Ok(())
    }

    /// The form's values: `{pullDirection, flipPull, minAngle}`.
    pub fn draft_analysis_params(&self) -> serde_json::Value {
        let state = self.draft_analysis.clone().unwrap_or(DraftAnalysisState {
            minimum_deg: DRAFT_ANALYSIS_DEFAULT_MIN_DEG,
            ..Default::default()
        });
        serde_json::json!({
            "pullDirection": state.pull_ref,
            "flipPull": state.flip,
            "minAngle": state.minimum_deg,
        })
    }

    /// The reading for a window, a test or a script.
    pub fn draft_analysis_json(&self) -> serde_json::Value {
        let Some(state) = self.draft_analysis.as_ref() else {
            return serde_json::json!({ "open": false });
        };
        let mut counts = serde_json::Map::new();
        for class in DraftClass::ALL {
            counts.insert(class.key().into(), state.faces.iter().filter(|f| f.class == class).count().into());
        }
        serde_json::json!({
            "open": true,
            "pullDirection": state.pull_ref,
            "flipPull": state.flip,
            "minAngle": state.minimum_deg,
            "pull": state.pull,
            "pullSource": state.pull_source,
            "error": state.error,
            "counts": counts,
            "faces": state.faces,
        })
    }

    /// The open analysis' state, for the window.
    pub fn draft_analysis_state(&self) -> Option<&DraftAnalysisState> {
        self.draft_analysis.as_ref()
    }

    /// The reading of face `face`, when the analysis is open and read it.
    pub fn draft_analysis_face(&self, face: &str) -> Option<&FaceDraft> {
        self.draft_analysis.as_ref()?.faces.iter().find(|f| f.face == face)
    }

    /// The hovered face's reading as its hover line, while the analysis is open.
    pub fn draft_analysis_hover_text(&self) -> Option<String> {
        let state = self.draft_analysis.as_ref()?;
        let mut hovered: Vec<&String> = self.emphasis.hovered_faces.iter().collect();
        hovered.sort();
        hovered.into_iter().find_map(|name| state.faces.iter().find(|f| &f.face == name)).map(FaceDraft::hover_text)
    }

    /// The faces of one class, for a legend row's click.
    pub fn draft_analysis_faces_of(&self, class: DraftClass) -> Vec<String> {
        self.draft_analysis
            .as_ref()
            .map(|state| state.faces.iter().filter(|f| f.class == class && !f.face.is_empty()).map(|f| f.face.clone()).collect())
            .unwrap_or_default()
    }

    /// Per-frame upkeep: re-read when the shown solids changed since the tint
    /// was written (a rebuild, a roll, a show/hide). A quiet frame compares a
    /// short list and does nothing else.
    pub(super) fn ensure_draft_analysis_current(&mut self) {
        let Some(state) = self.draft_analysis.as_ref() else {
            return;
        };
        if state.stamp != self.draft_analysis_stamp() {
            self.recompute_draft_analysis();
        }
    }

    fn draft_analysis_stamp(&self) -> Vec<(String, u64)> {
        self.scene
            .solids()
            .iter()
            .filter(|s| s.visible && !s.is_sketch)
            .map(|s| (s.name.clone(), s.revision))
            .collect()
    }

    /// The default pull: the last built Draft feature's (up to the rolled-to
    /// step), else +Z.
    fn draft_analysis_default_pull(&self) -> ([f64; 3], String) {
        let rollback = self.history.rollback();
        for index in (0..=rollback).rev() {
            if self.history.feature_type(index).as_deref() != Some("DRAFT") {
                continue;
            }
            if let (Some(pull), _) = self.draft_pull_and_neutral(index) {
                let id = self.history.feature_id(index).unwrap_or_default();
                return (pull, format!("the pull of {id}"));
            }
        }
        ([0.0, 0.0, 1.0], "+Z (no Draft feature yet)".into())
    }

    /// The unit direction a pull reference names on the shown model: a datum
    /// plane's normal, a flat face's OUTWARD normal (as Draft reads a face
    /// pull) or a straight edge's direction.
    fn draft_analysis_reference_direction(&self, name: &str) -> Result<[f64; 3], String> {
        if let Some((_, frame)) = self.construction_frames.iter().find(|(n, _)| n == name) {
            let z = frame.z_axis;
            return unit3([z.x, z.y, z.z]).ok_or_else(|| format!("'{name}' has no normal"));
        }
        for solid in self.scene.solids() {
            if let Some(index) = solid.faces.iter().position(|f| f.name == name) {
                return match face_normals(solid, index) {
                    Some((normals, false)) => Ok(normals[0]),
                    Some((_, true)) => Err(format!("'{name}' is curved: pick a flat face, a datum plane or a straight edge")),
                    None => Err(format!("'{name}' has no shown area")),
                };
            }
            if let Some(edge) = solid.edges.iter().find(|e| e.name == name) {
                let (Some(a), Some(b)) = (edge.polyline.first(), edge.polyline.last()) else {
                    return Err(format!("'{name}' has no shown points"));
                };
                let d = [(b[0] - a[0]) as f64, (b[1] - a[1]) as f64, (b[2] - a[2]) as f64];
                let dir = unit3(d).ok_or_else(|| format!("'{name}' is closed: pick a straight edge"))?;
                let straight = edge.polyline.iter().all(|p| {
                    let v = [(p[0] - a[0]) as f64, (p[1] - a[1]) as f64, (p[2] - a[2]) as f64];
                    let along = v[0] * dir[0] + v[1] * dir[1] + v[2] * dir[2];
                    let off = [v[0] - along * dir[0], v[1] - along * dir[1], v[2] - along * dir[2]];
                    (off[0] * off[0] + off[1] * off[1] + off[2] * off[2]).sqrt() <= 1e-4 * (1.0 + norm3(d))
                });
                return if straight { Ok(dir) } else { Err(format!("'{name}' is curved: pick a straight edge")) };
            }
        }
        Err(format!("'{name}' is not on the shown model"))
    }

    /// Read the shown model again and write the tints.
    fn recompute_draft_analysis(&mut self) {
        let Some(state) = self.draft_analysis.as_ref() else {
            return;
        };
        let (pull_ref, flip, minimum) = (state.pull_ref.clone(), state.flip, state.minimum_deg);
        let resolved = match &pull_ref {
            Some(name) => self.draft_analysis_reference_direction(name).map(|d| (d, format!("from {name}"))),
            None => Ok(self.draft_analysis_default_pull()),
        };
        let mut faces = Vec::new();
        let mut tints: Vec<(String, Vec<Option<[f32; 3]>>)> = Vec::new();
        let (pull, source, error) = match resolved {
            Ok((d, source)) => {
                let pull = if flip { [-d[0], -d[1], -d[2]] } else { d };
                for solid in self.scene.solids().iter().filter(|s| s.visible && !s.is_sketch) {
                    let read = analyse_solid(solid, pull, minimum);
                    let colors = solid
                        .faces
                        .iter()
                        .map(|face| read.iter().find(|r| r.face == face.name).map(|r| r.class.color()))
                        .collect();
                    tints.push((solid.name.clone(), colors));
                    faces.extend(read);
                }
                (pull, source, None)
            }
            Err(error) => ([0.0; 3], String::new(), Some(error)),
        };
        let all: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
        for name in all {
            let colors = tints.iter().find(|(n, _)| *n == name).map(|(_, c)| c.as_slice()).unwrap_or(&[]);
            self.scene.set_face_analysis_colors(&name, colors);
        }
        let stamp = self.draft_analysis_stamp();
        if let Some(state) = self.draft_analysis.as_mut() {
            state.pull = pull;
            state.pull_source = source;
            state.error = error;
            state.faces = faces;
            state.stamp = stamp;
        }
        self.dirty = true;
    }
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn unit3(v: [f64; 3]) -> Option<[f64; 3]> {
    let l = norm3(v);
    (l > 1e-12).then(|| [v[0] / l, v[1] / l, v[2] / l])
}

