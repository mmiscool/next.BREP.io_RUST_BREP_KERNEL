//! Face accounting: every face of every target body gets exactly one group.
//!
//! The groups, in precedence order:
//! 1. `feature`: one or more recognised features claim the face (a feature
//!    recognised but not machinable is still a claim);
//! 2. `refused`: recognition considered the face as a candidate and declined
//!    it, saying which candidate and why;
//! 3. `body`: a positive reason says the face is part of the body rather than
//!    of a feature (a stock-box side, the stock bottom, or a face another setup
//!    covers);
//! 4. `unexplained`: nothing explains it. These are the faces to review.
//!
//! Each entry keeps all of its evidence (claims, refusals, body reason and the
//! geometric description) and its group is derived from that evidence by
//! [`FaceAccount::settle`], so a consumer that drops claims (the multi-setup
//! planner keeps only the features a setup programs) re-derives the group
//! instead of leaving dangling feature ids.
//!
//! `Recognized::accounting` is `None` for documents written before accounting
//! existed: that means *unknown*, never "all explained". A part with no target
//! faces has `Some(vec![])`.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum FaceGroup {
    Feature,
    Refused,
    Body,
    Unexplained,
}

impl FaceGroup {
    pub const ALL: [FaceGroup; 4] = [FaceGroup::Feature, FaceGroup::Refused, FaceGroup::Body, FaceGroup::Unexplained];
    pub fn name(self) -> &'static str {
        match self {
            FaceGroup::Feature => "feature",
            FaceGroup::Refused => "refused",
            FaceGroup::Body => "body",
            FaceGroup::Unexplained => "unexplained",
        }
    }
}

/// Recognition declining a candidate for this face.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Refusal {
    /// The candidate declined: `hole`, `chamfer`, `fillet`, `face-or-boss`.
    pub candidate: String,
    pub reason: String,
}

/// A positive reason a face belongs to the body, not to a feature.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "code", rename_all = "kebab-case")]
pub enum BodyReason {
    /// An upward plane at the top of the body that no facing feature claims.
    StockTop,
    /// A downward plane at the bottom of the body (the fixtured face).
    StockBottom,
    /// A vertical plane lying on a side of the body's machine-axis bounding box.
    StockSide,
    /// Another setup's feature covers the face (`axis` names that setup's
    /// approach, `feature` that setup's feature id).
    #[serde(rename_all = "camelCase")]
    OtherSetup { axis: String, feature: String },
}

impl BodyReason {
    pub fn code(&self) -> &'static str {
        match self {
            BodyReason::StockTop => "stock-top",
            BodyReason::StockBottom => "stock-bottom",
            BodyReason::StockSide => "stock-side",
            BodyReason::OtherSetup { .. } => "other-setup",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceAccount {
    pub solid: String,
    pub face: String,
    /// Derived from the evidence below by [`FaceAccount::settle`].
    pub group: FaceGroup,
    /// Ids of every feature claiming the face.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub refusals: Vec<Refusal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<BodyReason>,
    /// What the face is geometrically, relative to the tool axis (the text
    /// `Recognized::unassigned` carries for unclaimed faces).
    pub why: String,
}

impl FaceAccount {
    pub fn new(solid: String, face: String, why: String) -> Self {
        Self { solid, face, group: FaceGroup::Unexplained, features: Vec::new(), refusals: Vec::new(), body: None, why }
    }

    /// Re-derive `group` from the evidence, in precedence order.
    pub fn settle(&mut self) {
        self.features.sort();
        self.features.dedup();
        self.group = if !self.features.is_empty() {
            FaceGroup::Feature
        } else if !self.refusals.is_empty() {
            FaceGroup::Refused
        } else if self.body.is_some() {
            FaceGroup::Body
        } else {
            FaceGroup::Unexplained
        };
    }
}

/// Face counts per group, in [`FaceGroup::ALL`] order.
pub fn group_counts(accounting: &[FaceAccount]) -> [usize; 4] {
    let mut counts = [0; 4];
    for a in accounting {
        counts[FaceGroup::ALL.iter().position(|g| *g == a.group).unwrap()] += 1;
    }
    counts
}

/// Keep only claims by the features in `keep` (renamed by `rename`), then
/// re-settle. Used when a consumer narrows a `Recognized` to some features.
pub fn retain_claims(accounting: &mut [FaceAccount], keep: impl Fn(&str) -> Option<String>) {
    for a in accounting {
        a.features = a.features.iter().filter_map(|id| keep(id)).collect();
        a.settle();
    }
}

/// The invariant every producer keeps: each listed face appears once, and
/// every claimed id names a feature of `features`.
pub fn check(accounting: &[FaceAccount], features: &[super::Feature]) -> Result<(), String> {
    let mut seen = std::collections::BTreeSet::new();
    for a in accounting {
        if !seen.insert((a.solid.as_str(), a.face.as_str())) {
            return Err(format!("face {}/{} is accounted twice", a.solid, a.face));
        }
        for id in &a.features {
            if !features.iter().any(|f| &f.id == id) {
                return Err(format!("face {}/{} names missing feature {id}", a.solid, a.face));
            }
        }
        let mut settled = a.clone();
        settled.settle();
        if settled.group != a.group {
            return Err(format!("face {}/{} is in group {} but its evidence says {}", a.solid, a.face, a.group.name(), settled.group.name()));
        }
    }
    Ok(())
}
