//! Hole-cycle expansion. A caller chooses canned OR expanded output; never
//! append both to a machine program. Coordinates are machine-space tool tips.
use crate::model::{Cycle, Feed, FeedsSpeeds, Motion, MotionKind, Point3, SpindleDirection};
use crate::{CamError, Runner};

fn invalid(message: &str) -> CamError { CamError::InvalidInput(message.into()) }

/// Expand one canned cycle starting/ending at its retract plane. Peck depth is
/// measured from the retract plane, matching the complete Cycle payload. Full
/// retract pecks conservatively feed through the previously cleared interval;
/// no unspecified rapid re-entry distance is invented.
pub fn expand_cycle(at: Point3, cycle: &Cycle, feeds: &FeedsSpeeds, runner: &mut Runner<'_>) -> Result<Vec<Motion>, CamError> {
    runner.token().check()?;
    feeds.validate().map_err(CamError::InvalidInput)?;
    let top = cycle.retract_z();
    let bottom = cycle.depth_z();
    if !at.is_finite() || !cycle.is_finite() || at.z != top || bottom >= top {
        return Err(invalid("cycle needs finite coordinates, at retract Z, and depth below retract"));
    }
    let point = |z| Point3::new(at.x, at.y, z);
    let mut moves = vec![Motion::rapid(at)];
    let plunge = feeds.plunge_feed;
    let dwell = |moves: &mut Vec<Motion>, seconds: f64| -> Result<(), CamError> {
        if seconds < 0.0 { return Err(invalid("negative cycle dwell")); }
        if seconds > 0.0 { moves.push(Motion::new(MotionKind::Dwell { seconds }, Feed::RAPID)); }
        Ok(())
    };
    match *cycle {
        Cycle::Peck { peck, .. } | Cycle::ChipBreak { peck, .. } => {
            let partial = match *cycle { Cycle::ChipBreak { back_off, .. } => Some(back_off), _ => None };
            if peck <= 0.0 || partial.map_or(false, |d| d <= 0.0) {
                return Err(invalid("peck and chip-break retract must be positive"));
            }
            let count = ((top-bottom)/peck).ceil();
            if !count.is_finite() || count > 1_000_000.0 { return Err(invalid("cycle exceeds one million pecks")); }
            let mut previous = top;
            for i in 1..=count as u64 {
                runner.work(1, "drill-peck", i, count as u64)?;
                let z = (top - i as f64 * peck).max(bottom);
                if z >= previous { return Err(invalid("peck cannot advance at this coordinate precision")); }
                moves.push(Motion::plunge(point(z), plunge));
                let retract = if i == count as u64 { top } else { partial.map_or(top, |d| (z+d).min(top)) };
                moves.push(Motion::retract(point(retract), Feed::RAPID));
                previous = z;
            }
        }
        Cycle::Drill { dwell_seconds, .. } => {
            moves.push(Motion::plunge(point(bottom), plunge));
            dwell(&mut moves, dwell_seconds)?;
            moves.push(Motion::retract(at, Feed::RAPID));
        }
        Cycle::Ream { .. } => {
            moves.push(Motion::plunge(point(bottom), plunge));
            moves.push(Motion::retract(at, Feed::Rate(feeds.retract_feed.unwrap_or(plunge))));
        }
        Cycle::Tap { pitch, rigid, left_hand, .. } => {
            if pitch <= 0.0 || feeds.spindle_rpm <= 0.0 { return Err(invalid("tap pitch and rpm must be positive")); }
            let feed = pitch * feeds.spindle_rpm;
            if !feed.is_finite() { return Err(invalid("tap feed overflow")); }
            let direction = if left_hand { SpindleDirection::Ccw } else { SpindleDirection::Cw };
            let reverse = if left_hand { SpindleDirection::Cw } else { SpindleDirection::Ccw };
            if rigid { moves.push(Motion::new(MotionKind::SpindleSync { enabled: true }, Feed::RAPID)); }
            moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction }, Feed::RAPID));
            moves.push(Motion::plunge(point(bottom), feed));
            moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction: reverse }, Feed::RAPID));
            moves.push(Motion::retract(at, Feed::Rate(feed)));
            if rigid { moves.push(Motion::new(MotionKind::SpindleSync { enabled: false }, Feed::RAPID)); }
            moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction }, Feed::RAPID));
        }
        Cycle::Bore { dwell_seconds, shift, feed_out, .. } => {
            if shift < 0.0 || (shift > 0.0 && feed_out) { return Err(invalid("boring shift must be nonnegative and incompatible with feed-out")); }
            moves.push(Motion::plunge(point(bottom), plunge));
            dwell(&mut moves, dwell_seconds)?;
            if feed_out {
                moves.push(Motion::retract(at, Feed::Rate(feeds.retract_feed.unwrap_or(plunge))));
            } else {
                moves.push(Motion::new(MotionKind::SpindleOff, Feed::RAPID));
                if shift > 0.0 {
                    moves.push(Motion::new(MotionKind::SpindleOrient { angle_deg: 0.0 }, Feed::RAPID));
                    moves.push(Motion::rapid(Point3::new(at.x+shift, at.y, bottom)));
                }
                moves.push(Motion::retract(Point3::new(at.x+shift, at.y, top), Feed::RAPID));
                if shift > 0.0 { moves.push(Motion::rapid(at)); }
                moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction: feeds.spindle_direction }, Feed::RAPID));
            }
        }
        Cycle::BackBore { entry_z, shift, .. } => {
            if shift <= 0.0 || entry_z >= bottom { return Err(invalid("back boring needs positive shift and entry below finished back face")); }
            let shifted=|z| Point3::new(at.x+shift, at.y, z);
            moves.push(Motion::new(MotionKind::SpindleOff, Feed::RAPID));
            moves.push(Motion::new(MotionKind::SpindleOrient { angle_deg: 0.0 }, Feed::RAPID));
            moves.push(Motion::rapid(shifted(top)));
            moves.push(Motion::rapid(shifted(entry_z)));
            moves.push(Motion::rapid(point(entry_z)));
            moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction: feeds.spindle_direction }, Feed::RAPID));
            moves.push(Motion::cut(point(bottom), plunge));
            moves.push(Motion::plunge(point(entry_z), plunge));
            moves.push(Motion::new(MotionKind::SpindleOff, Feed::RAPID));
            moves.push(Motion::new(MotionKind::SpindleOrient { angle_deg: 0.0 }, Feed::RAPID));
            moves.push(Motion::rapid(shifted(entry_z)));
            moves.push(Motion::retract(shifted(top), Feed::RAPID));
            moves.push(Motion::rapid(at));
            moves.push(Motion::new(MotionKind::SpindleOn { rpm: feeds.spindle_rpm, direction: feeds.spindle_direction }, Feed::RAPID));
        }
    }
    if moves.iter().any(|m| !m.kind.is_finite() || m.feed.value().map_or(false, |f| !f.is_finite())) { return Err(invalid("cycle expansion overflow")); }
    runner.token().check()?;
    Ok(moves)
}


pub mod ordering;

#[path = "../strategy/drill.rs"]
pub mod strategy;
