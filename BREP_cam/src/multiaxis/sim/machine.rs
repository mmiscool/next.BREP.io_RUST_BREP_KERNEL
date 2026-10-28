//! Machine collision (lane C10c, row 2): simple solids (boxes, cylinders)
//! attached to the links of the kinematic chain — head/spindle housing on
//! the tool side, table, fixture and stock on the part side — checked
//! along the resolved machine motion.
//!
//! Between consecutive resolved blocks the joints and linear axes are
//! interpolated in machine space in steps bounded by `max_step_deg` and
//! `max_step_linear`; at every sample each tool-side solid is tested
//! against every part-side solid (surface samples of one inside the other,
//! both directions, exact containment of a point in a box or cylinder) and
//! against the stock (tri-dexel containment in the part frame). The first
//! hit is reported with its block index, interpolation fraction, joint
//! angles, the two bodies and the point. The controlled point of every
//! block is derived from its tip in setup coordinates through the chain
//! (TCP off), so the check is independent of the resolver's TCP mode and
//! of work-plane sections. Surface sampling bounds the
//! detection: an interpenetration shallower than `surface_spacing` between
//! samples can be missed; the bound is reported.

use crate::kinematics::{Chain, KinematicsOptions};
use crate::model::{Point3, RigidTransform};
use crate::multiaxis::sim::tridexel::TriDexel;
use crate::kinematics::post::ResolvedMotion;
use crate::{CamError, Runner};
use serde::{Deserialize, Serialize};

pub use crate::model::{Link, LinkShape, LinkSolid, MachineModel};

/// Geometric queries on the model's machine shapes (the model holds only data).
pub trait LinkShapeGeometry {
    /// Containment of a point given in the link frame.
    fn contains_local(&self, p: Point3) -> bool;
    /// Surface sample points in the link frame, at most `spacing` apart,
    /// including the edges and corners.
    fn surface_samples(&self, spacing: f64) -> Vec<Point3>;
}

impl LinkShapeGeometry for LinkShape {
    /// Containment of a point given in the link frame.
    fn contains_local(&self, p: Point3) -> bool {
        match self {
            LinkShape::Box { min, max } => p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z,
            LinkShape::Cylinder { base, axis, radius, height } => {
                let a = axis.normalized().unwrap_or(Point3::Z);
                let d = p.sub(*base);
                let along = d.dot(a);
                along >= 0.0 && along <= *height && d.sub(a.scale(along)).length() <= *radius
            }
        }
    }

    /// Surface sample points in the link frame, at most `spacing` apart,
    /// including the edges and corners.
    fn surface_samples(&self, spacing: f64) -> Vec<Point3> {
        let steps = |len: f64| ((len / spacing).ceil() as usize).max(1);
        let mut out = Vec::new();
        match self {
            LinkShape::Box { min, max } => {
                let n = [steps(max.x - min.x), steps(max.y - min.y), steps(max.z - min.z)];
                for i in 0..=n[0] {
                    for j in 0..=n[1] {
                        for k in 0..=n[2] {
                            let on_face = i == 0 || i == n[0] || j == 0 || j == n[1] || k == 0 || k == n[2];
                            if !on_face {
                                continue;
                            }
                            out.push(Point3::new(
                                min.x + (max.x - min.x) * i as f64 / n[0] as f64,
                                min.y + (max.y - min.y) * j as f64 / n[1] as f64,
                                min.z + (max.z - min.z) * k as f64 / n[2] as f64,
                            ));
                        }
                    }
                }
            }
            LinkShape::Cylinder { base, axis, radius, height } => {
                let a = axis.normalized().unwrap_or(Point3::Z);
                let e1 = a.cross(if a.z.abs() < 0.9 { Point3::Z } else { Point3::X }).normalized().unwrap_or(Point3::X);
                let e2 = a.cross(e1);
                let nh = steps(*height);
                let na = steps(std::f64::consts::TAU * radius).max(8);
                let nr = steps(*radius);
                for k in 0..=nh {
                    let h = height * k as f64 / nh as f64;
                    for i in 0..na {
                        let phi = std::f64::consts::TAU * i as f64 / na as f64;
                        out.push(base.add(a.scale(h)).add(e1.scale(radius * phi.cos())).add(e2.scale(radius * phi.sin())));
                    }
                }
                for h in [0.0, *height] {
                    for r in 0..nr {
                        let rr = radius * r as f64 / nr as f64;
                        let m = if r == 0 { 1 } else { na };
                        for i in 0..m {
                            let phi = std::f64::consts::TAU * i as f64 / m as f64;
                            out.push(base.add(a.scale(h)).add(e1.scale(rr * phi.cos())).add(e2.scale(rr * phi.sin())));
                        }
                    }
                }
            }
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct MachineCheckOptions {
    /// Surface sample spacing (the detection bound).
    pub surface_spacing: f64,
    /// Largest rotary change between samples of one block, degrees.
    pub max_step_deg: f64,
    /// Largest linear change between samples of one block.
    pub max_step_linear: f64,
    /// Work budget: point tests.
    pub max_work: u64,
}

impl Default for MachineCheckOptions {
    fn default() -> Self {
        Self { surface_spacing: 0.5, max_step_deg: 2.0, max_step_linear: 5.0, max_work: 2_000_000_000 }
    }
}

impl MachineCheckOptions {
    pub fn validate(&self) -> Result<(), CamError> {
        let bad = |m: &str| Err(CamError::InvalidInput(format!("machine check: {m}")));
        if !self.surface_spacing.is_finite() || self.surface_spacing <= 0.0 {
            return bad("surface spacing must be positive");
        }
        if !self.max_step_deg.is_finite() || self.max_step_deg <= 0.0 || !self.max_step_linear.is_finite() || self.max_step_linear <= 0.0 {
            return bad("steps must be positive");
        }
        if self.max_work == 0 {
            return bad("work budget must be positive");
        }
        Ok(())
    }
}

/// The first collision found.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MachineCollision {
    /// Index of the resolved block during which it happens (the sample lies
    /// between block `index - 1` and block `index`; 0 for the first block's pose).
    pub block: usize,
    /// Interpolation fraction from the previous block to this one.
    pub fraction: f64,
    pub angles_deg: Vec<f64>,
    pub linear: Point3,
    /// Tool-side body.
    pub body: String,
    /// Part-side body, or `"stock"`.
    pub against: String,
    /// The offending point in machine coordinates.
    pub point: Point3,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MachineCheckReport {
    pub first: Option<MachineCollision>,
    pub samples: u64,
    /// Point tests performed.
    pub work: u64,
    /// Detection bound: interpenetrations shallower than this may be missed.
    pub bound: f64,
}

struct Placed<'a> {
    solid: &'a LinkSolid,
    to_world: RigidTransform,
    to_local: RigidTransform,
}

fn is_tool_side(chain: &Chain, link: Link) -> bool {
    match link {
        Link::Tool => true,
        Link::Joint { index } => chain.joints.get(index).map_or(false, |j| j.carrier == crate::kinematics::Carrier::Head),
        Link::Base | Link::Part => false,
    }
}

/// Check the resolved motion against the machine model and the stock.
pub fn check_machine(
    chain: &Chain,
    model: &MachineModel,
    blocks: &[ResolvedMotion],
    stock: Option<&TriDexel>,
    opts: &MachineCheckOptions,
    runner: &mut Runner<'_>,
) -> Result<MachineCheckReport, CamError> {
    opts.validate()?;
    for s in &model.solids {
        s.shape.validate().map_err(CamError::InvalidInput)?;
        if let Link::Joint { index } = s.link {
            if index >= chain.joints.len() {
                return Err(CamError::InvalidInput(format!("machine solid {} sits on joint {index}, which the chain does not have", s.name)));
            }
        }
    }
    let samples: Vec<Vec<Point3>> = model.solids.iter().map(|s| s.shape.surface_samples(opts.surface_spacing)).collect();
    let tool_side: Vec<usize> = (0..model.solids.len()).filter(|&i| is_tool_side(chain, model.solids[i].link)).collect();
    let part_side: Vec<usize> = (0..model.solids.len()).filter(|&i| !is_tool_side(chain, model.solids[i].link)).collect();
    let mut report = MachineCheckReport { bound: opts.surface_spacing, ..Default::default() };
    let mut work_left = opts.max_work;
    let n_rot = chain.joints.len();
    let mut prev: Option<(Point3, Vec<f64>)> = None;
    let mut tip: Option<Point3> = None;
    let off = KinematicsOptions { tcp: false, ..Default::default() };
    runner.stage("machine-check", "Machine collision check", 0, blocks.len() as u64)?;
    for (bi, b) in blocks.iter().enumerate() {
        runner.work(1, "machine-check", bi as u64, blocks.len() as u64)?;
        let angles: Vec<f64> = b.rotary.iter().map(|w| w.angle_deg).collect();
        if angles.len() != n_rot {
            return Err(CamError::InvalidInput(format!("block {bi} carries {} rotary words, the chain has {n_rot} joints", angles.len())));
        }
        // The controlled point from the block's tip (setup coordinates):
        // independent of the resolver's TCP mode and of work planes, whose
        // `linear` words are tip or plane-local coordinates.
        if let Some(t) = b.motion.end() {
            tip = Some(t);
        }
        let Some(t) = tip else { continue };
        let linear = chain.linear_for(t, &angles, &off);
        let (steps, from) = match &prev {
            None => (1usize, (linear, angles.clone())),
            Some((pl, pa)) => {
                let rot = pa.iter().zip(&angles).map(|(a, c)| (c - a).abs()).fold(0.0, f64::max);
                let lin = pl.distance(linear);
                let n = ((rot / opts.max_step_deg).ceil().max((lin / opts.max_step_linear).ceil()) as usize).max(1);
                (n, (*pl, pa.clone()))
            }
        };
        let start_s = if prev.is_none() { 0 } else { 1 };
        for s in start_s..=steps {
            let f = s as f64 / steps as f64;
            let lin = from.0.lerp(linear, f);
            let ang: Vec<f64> = from.1.iter().zip(&angles).map(|(a, c)| a + (c - a) * f).collect();
            report.samples += 1;
            let placed: Vec<Placed<'_>> = model
                .solids
                .iter()
                .map(|sld| {
                    let t = chain.link_transform(sld.link, &ang, lin)?;
                    Ok(Placed { solid: sld, to_local: t.inverse(), to_world: t })
                })
                .collect::<Result<_, crate::kinematics::KinematicsError>>()?;
            let mut hit: Option<MachineCollision> = None;
            'pairs: for &ti in &tool_side {
                for &pi in &part_side {
                    for (a, bb) in [(ti, pi), (pi, ti)] {
                        let cost = samples[a].len() as u64;
                        if work_left < cost {
                            return Err(CamError::InvalidInput("machine check: work budget exceeded".into()));
                        }
                        work_left -= cost;
                        report.work += cost;
                        for p in &samples[a] {
                            let w = placed[a].to_world.apply_point(*p);
                            let q = placed[bb].to_local.apply_point(w);
                            if placed[bb].solid.shape.contains_local(q) {
                                hit = Some(MachineCollision {
                                    block: bi,
                                    fraction: f,
                                    angles_deg: ang.clone(),
                                    linear: lin,
                                    body: model.solids[ti].name.clone(),
                                    against: model.solids[pi].name.clone(),
                                    point: w,
                                });
                                break 'pairs;
                            }
                        }
                    }
                }
                if let Some(stock) = stock {
                    let cost = samples[ti].len() as u64;
                    if work_left < cost {
                        return Err(CamError::InvalidInput("machine check: work budget exceeded".into()));
                    }
                    work_left -= cost;
                    report.work += cost;
                    for p in &samples[ti] {
                        let w = placed[ti].to_world.apply_point(*p);
                        let in_part = chain.world_to_part(&ang, w);
                        if stock.contains(in_part) {
                            hit = Some(MachineCollision {
                                block: bi,
                                fraction: f,
                                angles_deg: ang.clone(),
                                linear: lin,
                                body: model.solids[ti].name.clone(),
                                against: "stock".into(),
                                point: w,
                            });
                            break 'pairs;
                        }
                    }
                }
            }
            if hit.is_some() {
                report.first = hit;
                return Ok(report);
            }
        }
        prev = Some((linear, angles));
    }
    Ok(report)
}
