//! Touch-only camera navigation. A single contact keeps the existing tap/drag
//! routing (including sketch tools and gizmos). Two contacts capture navigation
//! until ALL fingers lift; a leftover finger must never become a click or orbit.
use egui::{Event, Pos2, TouchDeviceId, TouchId, TouchPhase};

#[derive(Clone, Copy)]
struct Contact {
    device: TouchDeviceId,
    id: TouchId,
    pos: Pos2,
    admitted: bool,
}

#[derive(Default)]
pub(super) struct TouchNavigation {
    contacts: Vec<Contact>,
    captured: bool,
    last_frame: Option<(u64, bool)>,
}

#[derive(Default)]
pub(super) struct TouchFrame {
    pub suppress_pointer: bool,
    pub started: bool,
    pub motion: Option<(Pos2, Pos2, f64)>,
}

impl TouchNavigation {
    /// egui can lay out the same frame more than once. Consume motion once,
    /// while retaining pointer suppression on every pass, including release.
    pub fn update_for_frame(
        &mut self,
        frame: u64,
        events: &[Event],
        focused: bool,
        any_touches: bool,
        admit: impl Fn(Pos2) -> bool,
    ) -> TouchFrame {
        if let Some((previous, suppressed)) = self.last_frame {
            if previous == frame {
                return TouchFrame {
                    suppress_pointer: suppressed,
                    ..Default::default()
                };
            }
            if previous + 1 != frame {
                // A hidden/docked viewport may have missed touch releases.
                *self = Self::default();
            }
        }
        let result = self.update(events, focused, any_touches, admit);
        self.last_frame = Some((frame, result.suppress_pointer));
        result
    }

    fn pair(&self) -> Option<(Pos2, f32)> {
        let [a, b] = self.contacts.as_slice() else {
            return None;
        };
        if !a.admitted || !b.admitted || a.device != b.device {
            return None;
        }
        Some((a.pos + (b.pos - a.pos) * 0.5, a.pos.distance(b.pos)))
    }

    pub fn update(
        &mut self,
        events: &[Event],
        focused: bool,
        any_touches: bool,
        admit: impl Fn(Pos2) -> bool,
    ) -> TouchFrame {
        let mut result = TouchFrame {
            suppress_pointer: self.captured,
            ..Default::default()
        };
        if !focused {
            result.started = self.contacts.iter().any(|c| c.admitted);
            result.suppress_pointer |= result.started;
            *self = Self::default();
            return result;
        }
        let before = self.pair();
        let mut membership_changed = false;
        for event in events {
            let Event::Touch {
                device_id,
                id,
                phase,
                pos,
                ..
            } = *event
            else {
                continue;
            };
            let index = self
                .contacts
                .iter()
                .position(|c| c.device == device_id && c.id == id);
            match phase {
                TouchPhase::Start => {
                    membership_changed = true;
                    if index.is_none() {
                        self.contacts.push(Contact {
                            device: device_id,
                            id,
                            pos,
                            admitted: pos.is_finite() && admit(pos),
                        });
                    }
                    // Latch even when both fingers start and end in one frame.
                    if self.pair().is_some() && !self.captured {
                        self.captured = true;
                        result.started = true;
                        result.suppress_pointer = true;
                    }
                }
                TouchPhase::Move => {
                    if let Some(index) = index {
                        if pos.is_finite() {
                            self.contacts[index].pos = pos;
                        }
                    }
                }
                TouchPhase::End | TouchPhase::Cancel => {
                    membership_changed = true;
                    if let Some(index) = index {
                        if phase == TouchPhase::Cancel && self.contacts[index].admitted {
                            // Some backends send no synthetic mouse-up on cancel.
                            // Close even a single-finger camera/tool capture.
                            result.started = true;
                            result.suppress_pointer = true;
                            self.captured = true;
                        }
                        self.contacts.remove(index);
                    }
                }
            }
        }
        if self.captured && !membership_changed {
            if let (Some((from, old_distance)), Some((to, distance))) = (before, self.pair()) {
                // Nearly coincident fingers have no reliable scale. Keep panning,
                // then establish a new baseline before accepting pinch ratios.
                let scale = if old_distance >= 4.0 && distance >= 4.0 {
                    f64::from(distance) / f64::from(old_distance)
                } else {
                    1.0
                };
                if from != to || scale != 1.0 {
                    result.motion = Some((from, to, scale));
                }
            }
        }
        // Also recover if the viewport was hidden when the release arrived.
        if !any_touches || self.contacts.is_empty() {
            *self = Self::default();
        }
        result
    }
}

