//! DRAFT ANALYSIS: every face of the shown model coloured by its draft angle
//! against a pull direction and a minimum (docs/features/draft.md).
//!
//! Display only. Nothing here touches the history, the document or undo: the
//! classes are read off the DISPLAY mesh (its outward normals, inside each
//! face's own triangle range, so a trimmed face is judged on what is there,
//! not on its whole carrier), and the colour is an analysis tint on the face
//! ([`crate::scene::FaceDisplay::analysis_color`]) that wins over the user's
//! face colour while the analysis is on and leaves it untouched.
//!
//! The draft angle of a surface point is the angle between its outward normal
//! and the plane square to the pull: `asin(n · pull)`, so a wall along the
//! pull reads 0°, a wall leaning away from the pull as it rises (the material
//! side narrowing, the way a drafted boss is) reads positive, and a face that
//! faces along the pull reads +90°. A flat face has one angle. A curved face
//! is SAMPLED at its mesh vertices, and its class comes from the least and the
//! greatest sample; the exact extreme can sit between samples, so a curved
//! face whose sampled range ends within [`NEAR_LIMIT_DEG`] of the minimum is
//! marked `nearLimit`.

use serde::Serialize;

/// Within this of the minimum, a CURVED face's sampled range is "near the
/// limit": the samples could have missed an extreme just over it.
pub const NEAR_LIMIT_DEG: f64 = 0.05;
/// A face within this of ±90° everywhere is square to the pull (a cap, or a
/// parting face).
pub const CAP_TOL_DEG: f64 = 0.01;
/// Triangle normals that agree to this (1 − cos) make a flat face.
const FLAT_TOL: f64 = 1e-6;

/// The classes, in legend order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum DraftClass {
    /// Every point leans with the pull by at least the minimum.
    Positive,
    /// Every point leans against the pull by at least the minimum.
    Negative,
    /// Some of the face is under the minimum either way (a wall straight
    /// along the pull, or a face that reaches one).
    NeedsDraft,
    /// The face runs both ways: part of it positive and part negative, each
    /// past the minimum (a sphere, a torus about the pull).
    Straddle,
    /// Square to the pull everywhere: a cap or a parting face.
    Cap,
}

impl DraftClass {
    pub const ALL: [DraftClass; 5] =
        [DraftClass::Positive, DraftClass::Negative, DraftClass::NeedsDraft, DraftClass::Straddle, DraftClass::Cap];

    /// The legend row's words.
    pub fn label(self) -> &'static str {
        match self {
            DraftClass::Positive => "Positive draft",
            DraftClass::Negative => "Negative draft",
            DraftClass::NeedsDraft => "Needs draft",
            DraftClass::Straddle => "Straddle",
            DraftClass::Cap => "Parting / cap (perpendicular)",
        }
    }

    /// The key a legend row and a JSON reading use.
    pub fn key(self) -> &'static str {
        match self {
            DraftClass::Positive => "positive",
            DraftClass::Negative => "negative",
            DraftClass::NeedsDraft => "needsDraft",
            DraftClass::Straddle => "straddle",
            DraftClass::Cap => "cap",
        }
    }

    /// The tint, sRGB 0..1. Green / red / yellow / orange as the mould tools
    /// draw them; the cap a neutral grey-blue so it reads apart from all four.
    pub fn color(self) -> [f32; 3] {
        match self {
            DraftClass::Positive => [0.20, 0.72, 0.30],
            DraftClass::Negative => [0.85, 0.22, 0.20],
            DraftClass::NeedsDraft => [0.95, 0.80, 0.15],
            DraftClass::Straddle => [0.96, 0.52, 0.12],
            DraftClass::Cap => [0.55, 0.62, 0.70],
        }
    }
}

/// One face's reading.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceDraft {
    pub solid: String,
    pub face: String,
    /// The least and greatest draft angle found on the face, degrees.
    pub min_deg: f64,
    pub max_deg: f64,
    pub class: DraftClass,
    /// Whether the face is curved (its angle is sampled, not exact).
    pub curved: bool,
    /// A curved face whose sampled range ends within [`NEAR_LIMIT_DEG`] of ±min.
    pub near_limit: bool,
}

impl FaceDraft {
    /// The hover line: `Box_PX: +3.00° (Positive draft)`, a curved face's
    /// range, and the near-limit note when it applies.
    pub fn hover_text(&self) -> String {
        let angle = if self.curved && (self.max_deg - self.min_deg).abs() > 0.005 {
            format!("{:+.2}° to {:+.2}° (sampled)", self.min_deg, self.max_deg)
        } else {
            format!("{:+.2}°", 0.5 * (self.min_deg + self.max_deg))
        };
        let mut text = format!("{}: {angle}, {}", self.face, self.class.label());
        if self.near_limit {
            text.push_str(&format!(
                "\nnear limit: the sampled range ends within {NEAR_LIMIT_DEG}° of the minimum, and sampling can miss the exact extreme"
            ));
        }
        text
    }
}

/// The class of a face whose draft runs from `min_deg` to `max_deg`, against
/// a minimum of `minimum_deg` (≥ 0).
pub fn classify(min_deg: f64, max_deg: f64, minimum_deg: f64) -> DraftClass {
    let m = minimum_deg.abs();
    if min_deg >= 90.0 - CAP_TOL_DEG || max_deg <= -(90.0 - CAP_TOL_DEG) {
        DraftClass::Cap
    } else if min_deg >= m {
        DraftClass::Positive
    } else if max_deg <= -m {
        DraftClass::Negative
    } else if min_deg <= -m && max_deg >= m && m > 0.0 {
        DraftClass::Straddle
    } else {
        DraftClass::NeedsDraft
    }
}

/// Whether a curved face's sampled range ends within [`NEAR_LIMIT_DEG`] of the
/// minimum on either side.
pub fn near_limit(min_deg: f64, max_deg: f64, minimum_deg: f64) -> bool {
    let m = minimum_deg.abs();
    [min_deg, max_deg].iter().any(|&a| (a.abs() - m).abs() < NEAR_LIMIT_DEG)
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let l = norm(v);
    (l > 1e-12).then(|| [v[0] / l, v[1] / l, v[2] / l])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn to64(v: [f32; 3]) -> [f64; 3] {
    [v[0] as f64, v[1] as f64, v[2] as f64]
}

/// The draft angle of a unit normal against a unit pull, degrees.
fn angle_deg(normal: [f64; 3], pull: [f64; 3]) -> f64 {
    dot(normal, pull).clamp(-1.0, 1.0).asin().to_degrees()
}

/// The outward normals one face of `solid` presents in WORLD space: one for
/// a flat face (its triangles' common normal, signed by the mesh normals),
/// else every vertex normal of its triangles. `None` for a face with none.
pub fn face_normals(solid: &crate::scene::SolidDisplay, face_index: usize) -> Option<(Vec<[f64; 3]>, bool)> {
    let face = solid.faces.get(face_index)?;
    let mesh = &solid.mesh;
    let start = face.tri_start as usize;
    let end = start + face.tri_count as usize;
    let mut tri_normals: Vec<[f64; 3]> = Vec::new();
    let mut vertex_normals: Vec<[f64; 3]> = Vec::new();
    for tri in start..end.min(mesh.indices.len() / 3) {
        let ids = [mesh.indices[3 * tri] as usize, mesh.indices[3 * tri + 1] as usize, mesh.indices[3 * tri + 2] as usize];
        let (Some(a), Some(b), Some(c)) = (mesh.positions.get(ids[0]), mesh.positions.get(ids[1]), mesh.positions.get(ids[2])) else {
            continue;
        };
        let (a, b, c) = (to64(*a), to64(*b), to64(*c));
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let mut geometric = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
        let mut shading = [0.0; 3];
        for &id in &ids {
            if let Some(n) = mesh.normals.get(id).copied().map(to64).and_then(unit) {
                vertex_normals.push(n);
                shading = [shading[0] + n[0], shading[1] + n[1], shading[2] + n[2]];
            }
        }
        if dot(geometric, shading) < 0.0 {
            geometric = [-geometric[0], -geometric[1], -geometric[2]];
        }
        if let Some(n) = unit(geometric) {
            tri_normals.push(n);
        }
    }
    let first = *tri_normals.first()?;
    let flat = tri_normals.iter().all(|n| dot(*n, first) >= 1.0 - FLAT_TOL);
    // The display's positions and normals are already where the solid is
    // shown (a `pose` only tells the renderer how they moved), so they are
    // read as they stand.
    let normals = if flat {
        // The area-free mean of the triangle normals: the face's own normal.
        let sum = tri_normals.iter().fold([0.0; 3], |s, n| [s[0] + n[0], s[1] + n[1], s[2] + n[2]]);
        vec![unit(sum).unwrap_or(first)]
    } else {
        vertex_normals
    };
    Some((normals, !flat))
}

/// Read every face of `solid` against the unit `pull` and `minimum_deg`.
pub fn analyse_solid(solid: &crate::scene::SolidDisplay, pull: [f64; 3], minimum_deg: f64) -> Vec<FaceDraft> {
    let mut out = Vec::with_capacity(solid.faces.len());
    for (index, face) in solid.faces.iter().enumerate() {
        if face.kind == crate::scene::FaceKind::SectionCap {
            continue;
        }
        let Some((normals, curved)) = face_normals(solid, index) else {
            continue;
        };
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for n in &normals {
            let a = angle_deg(*n, pull);
            lo = lo.min(a);
            hi = hi.max(a);
        }
        out.push(FaceDraft {
            solid: solid.name.clone(),
            face: face.name.clone(),
            min_deg: lo,
            max_deg: hi,
            class: classify(lo, hi, minimum_deg),
            curved,
            near_limit: curved && near_limit(lo, hi, minimum_deg),
        });
    }
    out
}

