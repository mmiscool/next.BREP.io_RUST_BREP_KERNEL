//! Turning a Transform Face refusal into a sentence about the user's model.
//!
//! Move Face and Rotate Face refuse BY DESIGN — a motion that carries the
//! selection off the body it belongs to has no answer, and the kernel says so
//! rather than building something unsound. What it says, though, is written for
//! the lane that raised it:
//!
//! ```text
//! TF1: move_faces: the translation inverts the solid (signed volume changed sign) — refusing
//! ```
//!
//! `move_faces` is a function name, and "signed volume changed sign" is the
//! kernel's own criterion, not a statement about the part on screen. Worse, the
//! message never names the face the user picked and is dragging: the rotation
//! refusal names `Box_NZ` and `Box_PY`, which are the NEIGHBOURS that could not
//! follow, so the two faces it does name are both the wrong answer to "which
//! one did I break?".
//!
//! This module re-says it. It reads the kernel's text, recognises WHICH refusal
//! it is from the few stable phrases each one is built on, and rebuilds the
//! sentence around the face the user actually selected — which the app knows
//! from `params.faces` and the kernel message does not carry.
//!
//! One thing the sentences here must get right, because it is what the user is
//! looking at: a REFUSED Transform Face contributes NOTHING. The feature drops
//! out of the build entirely rather than falling back to its last good motion,
//! so the face springs back to where it was before the feature ran — measured
//! 2026-09-21, a block lifted to z = 17.9188 by a good drag snapped back to
//! z = 14 the moment a later drag refused. A hint that said the part was "still
//! exactly as it was" would be describing a different app.
//!
//! **The kernel text is never thrown away.** [`FaceTransformRefusal::kernel`]
//! keeps it verbatim, and an unrecognised message becomes the reason unchanged:
//! when the kernel lane rewords a refusal under us, this degrades to exactly
//! the message the app showed before this module existed, rather than losing
//! it. That is the whole reason the match is on phrases rather than on a
//! prefix — nothing here can make a refusal less legible than it was.

/// A Transform Face refusal, said three ways: what happened, why, and what to
/// do — plus the kernel's own words, kept.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FaceTransformRefusal {
    /// Which motion refused: `"Move Face"` or `"Rotate Face"`, named for the
    /// half of the feature the user was actually driving. Transform Face is
    /// both, and a user who dragged a rotation ring should not be told
    /// something about a translation.
    pub motion: &'static str,
    /// The refusal in one sentence, naming the selected face.
    pub reason: String,
    /// What to do instead. Always something the user can do from where they
    /// are — the model is at its last good answer, so every hint is a smaller
    /// value away rather than a restart.
    pub hint: String,
    /// The kernel's message, verbatim and unabridged.
    pub kernel: String,
}

impl FaceTransformRefusal {
    /// The reason and the hint as one line, for a chip.
    pub fn one_line(&self) -> String {
        format!("{} — {}", self.reason, self.hint)
    }
}

/// How the selection reads inside a sentence: the one face's name, or a count.
fn subject(faces: &[String]) -> String {
    match faces {
        [] => "the selected face".to_string(),
        [one] => format!("`{one}`"),
        many => format!("the {} selected faces", many.len()),
    }
}

/// Every \`backticked\` name in `message` that is not already in `faces` — the
/// NEIGHBOURS a refusal names, which are the faces that could not follow the
/// motion rather than the ones the user picked.
fn neighbours(message: &str, faces: &[String]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    // `split('`')` alternates outside/inside starting outside, so the odd
    // indices are the quoted spans.
    for (index, span) in message.split('`').enumerate() {
        if index % 2 == 1
            && !span.is_empty()
            && !faces.iter().any(|f| f == span)
            && !out.iter().any(|n| n == span)
        {
            out.push(span.to_string());
        }
    }
    out
}

/// Render `names` as `` `a` and `b` `` / `` `a`, `b` and `c` ``.
fn and_list(names: &[String]) -> String {
    match names {
        [] => String::new(),
        [one] => format!("`{one}`"),
        [rest @ .., last] => format!(
            "{} and `{last}`",
            rest.iter()
                .map(|n| format!("`{n}`"))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

/// The face names in a Transform Face's `faces` param, as the kernel resolves
/// them — re-exported here so the APP can name the selection in a refusal
/// without taking a dependency on the kernel of its own.
pub fn selected_face_names(faces: Option<&serde_json::Value>) -> Vec<String> {
    brep_kernel::reference_names(faces)
}

/// What the run report says about a feature's refusal beyond its text: WHY,
/// read off the kernel refusal's class and slug
/// ([`brep_kernel::face_transform_reason`]), and whether the motion that
/// refused was a TURN (`featureRefusals[id].step`, which the feature records).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RefusalFacts {
    pub reason: Option<brep_kernel::FaceTransformReason>,
    pub turning: bool,
}

impl RefusalFacts {
    /// The facts for feature `id` in a run report (`__brepReport` /
    /// `history_report`): its `featureRefusals[id]` entry. A feature whose
    /// refusal is text only — or a report from before the key existed — has no
    /// entry, so no reason and no turn: the help says the kernel's words.
    pub fn from_report(report: &serde_json::Value, id: &str) -> Self {
        let Some(entry) = report.get("featureRefusals").and_then(|map| map.get(id)) else {
            return Self::default();
        };
        let turning = entry.get("step").and_then(serde_json::Value::as_str) == Some("rotation");
        let mut refusal = entry.clone();
        if let serde_json::Value::Object(object) = &mut refusal {
            object.remove("step");
        }
        let reason = serde_json::from_value::<brep_kernel::KernelRefusal>(refusal)
            .ok()
            .and_then(|refusal| brep_kernel::face_transform_reason(&refusal));
        Self { reason, turning }
    }
}

/// Explain a Transform Face refusal.
///
/// `message` is the kernel's text for the feature WITHOUT the `"<id>: "` prefix
/// (what `feature_error_message` already returns); `facts` are the refusal's
/// class-read reason and the refused motion ([`RefusalFacts::from_report`]) —
/// the two DECISIONS, neither read from the text; `faces` is the feature's
/// `params.faces`, which is where the selected face's name comes from — the
/// kernel message does not carry it. The text is used only to say the kernel's
/// own words when the reason is not one this help explains, and to name the
/// neighbours the kernel named.
pub fn explain_refusal(message: &str, facts: RefusalFacts, faces: &[String]) -> FaceTransformRefusal {
    use brep_kernel::FaceTransformReason as Reason;
    let kernel = message.to_string();
    let subject = subject(faces);

    // WHICH motion refused: the one the feature recorded on its result (a
    // rotate-then-translate records the step that refused, so the user is told
    // about the ring they turned or the arrow they pulled).
    let motion = if facts.turning { "Rotate Face" } else { "Move Face" };
    let turning = motion == "Rotate Face";
    let motion_noun = if turning { "turn" } else { "move" };

    // The neighbours the kernel named, said as what they are.
    let blocked = neighbours(message, faces);
    let by_neighbour = if blocked.is_empty() {
        String::new()
    } else {
        format!(" {} could not follow it", and_list(&blocked))
    };

    // The refusals, by the reason the kernel's class gives.
    let (reason, hint) = if facts.reason == Some(Reason::InsideOut) {
        (
            format!(
                "That {motion_noun} carries {subject} clean through the far side of the body, \
                 leaving it inside out."
            ),
            "Ease it back until the model follows again — while it refuses the feature builds \
             nothing at all, so the face springs back to where it started."
                .to_string(),
        )
    } else if facts.reason == Some(Reason::PastNeighbour) {
        (
            format!(
                "That {motion_noun} carries {subject} past a face it has to meet, so{} \
                 without turning a corner of the body inside out.",
                if by_neighbour.is_empty() {
                    " its neighbour could not follow it".to_string()
                } else {
                    by_neighbour
                }
            ),
            if turning {
                "Ease the angle back, or move the Pivot — the turn happens about it, so a pivot \
                 nearer the blocked corner asks less of it."
                    .to_string()
            } else {
                "Ease the distance back until the model follows again, or select the blocked \
                 neighbours too so they move with it."
                    .to_string()
            },
        )
    } else if facts.reason == Some(Reason::TurnParallel) {
        (
            format!(
                "That turn lays {subject} parallel to a face it has to meet, and two parallel \
                 faces never make a corner."
            ),
            "Stop short of the angle that flattens it — while it refuses the feature builds \
             nothing at all, so the face springs back unturned."
                .to_string(),
        )
    } else if facts.reason == Some(Reason::RigidScale) {
        (
            format!("A face transform carries {subject} rigidly, so it cannot scale."),
            "Clear the scale back to 1, 1, 1 and move or turn the face instead.".to_string(),
        )
    } else if facts.reason == Some(Reason::NoPivot) {
        (
            format!("{subject} has no geometry to turn about."),
            "Pick the face again, or type a Pivot to turn about.".to_string(),
        )
    } else {
        // UNRECOGNISED — a refusal whose class this help has no explanation for
        // (or a text-only one): the kernel's own words, unchanged, which is what
        // the app showed before this help existed.
        (
            kernel.clone(),
            "Ease the value back until the model follows again — while it refuses the feature \
             builds nothing at all, so the face springs back to where it started."
                .to_string(),
        )
    };

    FaceTransformRefusal {
        motion,
        reason,
        hint,
        kernel,
    }
}

