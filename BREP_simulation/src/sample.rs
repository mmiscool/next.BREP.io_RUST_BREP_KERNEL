//! A study at one time: what the solver and the display are asked for.
//!
//! [`Study::sample`] reads the motors and clips at a time and returns a
//! [`FrameSample`]: per mate, the value it is driven to or that it is off; and
//! the camera, hidden and transparent occurrences, section and explode state.
//! Motors hold their last value after their span and leave the mate at its
//! document value before it. Visibility, transparency and section clips act
//! only inside their span; camera and explode clips hold their last state
//! after it.

use crate::model::{ClipBody, Drive, Motor, Shape, Study, DEFAULT_ACCEL};
use std::collections::BTreeMap;

/// What one mate does at a frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MateOverride {
    /// The value the mate is driven to (model units or degrees).
    pub value: Option<f64>,
    /// `false` when a suppress clip has it off.
    pub enabled: Option<bool>,
}

/// The camera between two PMI views: `from` blending into `to` by `t` (0..1).
#[derive(Debug, Clone, PartialEq)]
pub struct CameraSample {
    pub from: String,
    pub to: String,
    pub t: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SectionSample {
    pub view: String,
    /// How far the view's planes move along their normals.
    pub offset: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExplodeSample {
    pub view: String,
    /// 0 collapsed .. 1 every step out.
    pub progress: f64,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct FrameSample {
    /// Constraint id → its override; mates absent here keep their document state.
    pub mates: BTreeMap<String, MateOverride>,
    pub camera: Option<CameraSample>,
    /// Hidden occurrences.
    pub hidden: Vec<String>,
    /// Occurrence → opacity (0..1) for the transparent ones.
    pub opacity: BTreeMap<String, f64>,
    pub section: Option<SectionSample>,
    /// The exploded views in play, in clip order (later ones compose on top).
    pub explode: Vec<ExplodeSample>,
}

/// Position along a profile, 0..1, at the span fraction `u` (0..1).
pub fn shape_at(shape: Shape, accel: Option<f64>, u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    match shape {
        Shape::Linear => u,
        Shape::Ease => u * u * (3.0 - 2.0 * u),
        Shape::Trapezoid => {
            let a = accel.unwrap_or(DEFAULT_ACCEL).clamp(1e-6, 0.5);
            // Peak rate so the area under the trapezoid is 1.
            let peak = 1.0 / (1.0 - a);
            if u < a {
                0.5 * peak * u * u / a
            } else if u <= 1.0 - a {
                peak * (u - 0.5 * a)
            } else {
                1.0 - 0.5 * peak * (1.0 - u) * (1.0 - u) / a
            }
        }
    }
}

impl Motor {
    /// The motor's value at `time`, given the mate's document value `base`
    /// (what a speed drive starts from); `None` before its span.
    pub fn value_at(&self, time: f64, base: f64) -> Option<f64> {
        if time < self.from {
            return None;
        }
        let elapsed = time.min(self.to) - self.from;
        Some(match &self.drive {
            Drive::Speed { speed } => base + speed * elapsed,
            Drive::Profile { shape, start, end, accel } => start + (end - start) * shape_at(*shape, *accel, elapsed / (self.to - self.from)),
        })
    }
}

impl Study {
    /// The study at `time`. `base` gives a mate's document value (by
    /// constraint id), which a speed motor starts from; a mate it does not
    /// know starts at 0.
    pub fn sample(&self, time: f64, base: &dyn Fn(&str) -> Option<f64>) -> FrameSample {
        let mut out = FrameSample::default();
        for motor in &self.motors {
            // Motors on one mate never overlap (validated): the latest started wins.
            if let Some(value) = motor.value_at(time, base(&motor.constraint).unwrap_or(0.0)) {
                let entry = out.mates.entry(motor.constraint.clone()).or_default();
                let later = self
                    .motors
                    .iter()
                    .any(|other| other.constraint == motor.constraint && other.from > motor.from && other.from <= time);
                if !later {
                    entry.value = Some(value);
                }
            }
        }
        // In start order, so a later clip's camera and explode state win.
        let mut clips: Vec<_> = self.clips.iter().collect();
        clips.sort_by(|a, b| a.from.total_cmp(&b.from));
        let mut camera: Option<CameraSample> = None;
        for clip in clips {
            let inside = time >= clip.from && time < clip.to;
            let u = ((time - clip.from) / (clip.to - clip.from)).clamp(0.0, 1.0);
            match &clip.body {
                ClipBody::Suppress { constraint } if inside => {
                    out.mates.entry(constraint.clone()).or_default().enabled = Some(false);
                }
                ClipBody::Camera { views } if time >= clip.from => {
                    camera = Some(if views.len() == 1 {
                        CameraSample { from: views[0].clone(), to: views[0].clone(), t: 0.0 }
                    } else {
                        let along = u * (views.len() - 1) as f64;
                        let at = (along.floor() as usize).min(views.len() - 2);
                        CameraSample { from: views[at].clone(), to: views[at + 1].clone(), t: along - at as f64 }
                    });
                }
                ClipBody::Visibility { occurrences } if inside => {
                    for occurrence in occurrences {
                        if !out.hidden.contains(occurrence) {
                            out.hidden.push(occurrence.clone());
                        }
                    }
                }
                ClipBody::Transparency { occurrences, opacity } if inside => {
                    let value = opacity[0] + (opacity[1] - opacity[0]) * u;
                    for occurrence in occurrences {
                        out.opacity.insert(occurrence.clone(), value);
                    }
                }
                ClipBody::Section { view, sweep } if inside => {
                    let offset = sweep.map_or(0.0, |s| s[0] + (s[1] - s[0]) * u);
                    out.section = Some(SectionSample { view: view.clone(), offset });
                }
                ClipBody::Explode { view, collapse } if time >= clip.from => {
                    let progress = if *collapse { 1.0 - u } else { u };
                    out.explode.retain(|e| e.view != *view);
                    if progress > 0.0 {
                        out.explode.push(ExplodeSample { view: view.clone(), progress });
                    }
                }
                _ => {}
            }
        }
        out.camera = camera;
        out
    }
}

