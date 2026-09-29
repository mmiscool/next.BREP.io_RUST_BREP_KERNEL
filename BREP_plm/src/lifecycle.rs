//! The revision state machine, and the revision labels.
//!
//! Every lifecycle move in the server goes through [`check_transition`]. One
//! function means the rules can be read in one place and tested without a
//! server, and it is why "released is immutable" is a property of the type
//! rather than a check each handler remembers.
//!
//! # What is NOT here
//!
//! Release does not check children. This server has no BOM yet, so a gate that
//! claimed to verify "every referenced child is Released" would be verifying
//! nothing. The rule belongs in the architecture and lands with the BOM.

use crate::model::Lifecycle;

/// Why a transition was refused. The message is user-facing: it is what the
/// API returns and what the page shows.
pub type Refusal = String;

/// Whether `from` may become `to`, and why not when it may not.
///
/// The permitted moves:
///
/// * `Draft → InReview` — submit for review.
/// * `InReview → Draft` — send back for more work.
/// * `Draft → Released` / `InReview → Released` — review is OPTIONAL in this
///   slice, so a draft may release directly.
/// * `Released → Superseded` — applied automatically when a NEWER revision of
///   the same part releases; never requested directly.
/// * `Released → Obsolete`, `Superseded → Obsolete` — withdraw the part.
///
/// Everything else is refused, including every move OUT of `Obsolete` (it is
/// terminal) and every move back out of `Released` other than the two above —
/// which is the immutability rule stated as a transition.
pub fn check_transition(from: Lifecycle, to: Lifecycle) -> Result<(), Refusal> {
    use Lifecycle::*;
    let allowed = matches!(
        (from, to),
        (Draft, InReview)
            | (InReview, Draft)
            | (Draft, Released)
            | (InReview, Released)
            | (Released, Superseded)
            | (Released, Obsolete)
            | (Superseded, Obsolete)
    );
    if allowed {
        return Ok(());
    }
    Err(match from {
        Obsolete => "an obsolete revision is final and cannot change state".to_string(),
        Released | Superseded => format!(
            "a {} revision is immutable — revise the part instead of moving it to {}",
            from.as_str(),
            to.as_str()
        ),
        _ => format!(
            "a {} revision cannot become {}",
            from.as_str(),
            to.as_str()
        ),
    })
}

/// The label after `previous`: `A` → `B`, `Z` → `AA`, `AZ` → `BA`.
///
/// Plain base-26 over the full alphabet. Standards that skip the confusable
/// letters (`I`, `O`, `Q`) differ on WHICH to skip, so that belongs in part-type
/// configuration later rather than compiled in here.
pub fn next_revision_label(previous: Option<&str>) -> String {
    let Some(previous) = previous else {
        return "A".to_string();
    };
    let mut chars: Vec<u8> = previous.trim().to_ascii_uppercase().into_bytes();
    if chars.is_empty() || chars.iter().any(|c| !c.is_ascii_uppercase()) {
        // A label this function did not produce (a hand-edited file, an
        // imported vendor label like "1.2"): start a clean sequence rather
        // than guess at its successor.
        return "A".to_string();
    }
    let mut index = chars.len();
    loop {
        if index == 0 {
            chars.insert(0, b'A');
            break;
        }
        index -= 1;
        if chars[index] == b'Z' {
            chars[index] = b'A';
            continue;
        }
        chars[index] += 1;
        break;
    }
    String::from_utf8(chars).expect("ASCII uppercase stays UTF-8")
}

