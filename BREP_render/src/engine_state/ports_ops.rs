//! The PORTS surface: what the ports tail resolved on the last applied run.
//!
//! A part's declared connection points are DATA, not features
//! (`brep_kernel`'s `ports` module), and they are resolved at the TAIL of
//! every history run. The tail is not a feature, so its result reaches no
//! feature row: this is the only place a caller can read what it published,
//! what name it refused and which reference it could not resolve.
//!
//! The block itself is read and written through `History` (`ports_block`,
//! `set_ports_block`), which carries the pin/point follow inside the write —
//! so a caller that edits a point here edits its pin too, in one undo step.

use super::*;
use brep_kernel::PortsReport;
use serde_json::Value;


/// Which of a connection point's two reference fields a pick is filling.
///
/// They are two different questions about the same seat — WHERE it sits and
/// WHICH WAY it faces — so they admit different picks: a vertex locates a
/// point and implies no direction, which is why it is offered to one and not
/// the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortRefField {
    /// `pointRef` — the geometry the point sits on.
    Point,
    /// `directionRef` — the geometry it faces along.
    Direction,
}

impl PortRefField {
    /// The block key this field writes.
    pub fn key(self) -> &'static str {
        match self {
            PortRefField::Point => "pointRef",
            PortRefField::Direction => "directionRef",
        }
    }

    /// The picker's heading.
    pub fn label(self) -> &'static str {
        match self {
            PortRefField::Point => "Seat this connection point on",
            PortRefField::Direction => "Point it along",
        }
    }

    /// Which pick kinds this field admits. A VERTEX seats a position and
    /// implies no direction, so it is offered to `pointRef` alone; SOLID is
    /// offered to neither, because a click meant for a face that landed on the
    /// body instead would seat the point at the body's centre, which is never
    /// what the click meant.
    pub fn filter(self) -> Vec<String> {
        let kinds: &[&str] = match self {
            PortRefField::Point => &["VERTEX", "EDGE", "FACE", "PLANE"],
            PortRefField::Direction => &["EDGE", "FACE", "PLANE"],
        };
        kinds.iter().map(|kind| (*kind).to_string()).collect()
    }
}

impl EngineState {
    /// The tail's row for one point of THIS document, by part-local address —
    /// its seat, where it resolved and which way it faces. `None` for a point
    /// the last run did not resolve (one just typed in, or one whose group
    /// name is refused).
    pub fn port_point_row(&self, address: &str) -> Option<&brep_kernel::PortPointRow> {
        self.ports_report()?
            .points
            .iter()
            .find(|row| row.address == address)
    }

    /// The connection point of THIS DOCUMENT that the viewport selection
    /// names, if any — the 3D half of "select it there, see it here".
    ///
    /// A declared point draws as a synthesized sheet keyed by its ADDRESS, and
    /// its base vertex and line hang off that address, so a click anywhere on
    /// it selects one of three names that all peel back to the same point.
    ///
    /// A NAMESPACED address (`ACOMP1:J1.VCC`) is deliberately not adopted: it
    /// belongs to a component this assembly places, not to the document being
    /// qualified, and selecting its row here would offer to edit a point this
    /// document does not own.
    pub fn selected_port_point(&self) -> Option<String> {
        let declared: Vec<String> = self
            .history
            .declared_points()
            .into_iter()
            .map(|point| point.address())
            .collect();
        self.emphasis.selected_solids.iter().find_map(|name| {
            let address = name
                .strip_suffix(":Base")
                .or_else(|| name.strip_suffix(":PortLine"))
                .unwrap_or(name);
            declared
                .iter()
                .find(|candidate| candidate.as_str() == address)
                .cloned()
        })
    }

    /// Edit ONE declared point's JSON object in place, by address, and write
    /// the block back through `History::set_ports_block` (which carries the
    /// pin/point follow and re-runs). Returns whether the point was found.
    ///
    /// The point is handed to `edit` as its RAW object, one key at a time, for
    /// the reason the Qualify panel edits it that way: a round trip through
    /// `PortDeclaration` drops any field the struct does not model, and no
    /// lane may be the reason a later slice's field disappears when the field
    /// beside it is touched.
    pub fn edit_port_point(
        &mut self,
        address: &str,
        coalesce: Option<&str>,
        edit: impl FnOnce(&mut serde_json::Map<String, Value>),
    ) -> bool {
        let Some((port, point_name)) = brep_kernel::split_address(address) else {
            return false;
        };
        let Some(mut block) = self
            .history
            .ports_block()
            .and_then(Value::as_array)
            .cloned()
        else {
            return false;
        };
        let mut found = false;
        for group in block.iter_mut() {
            if group.get("name").and_then(Value::as_str).map(str::trim) != Some(port) {
                continue;
            }
            let Some(points) = group.get_mut("points").and_then(Value::as_array_mut) else {
                continue;
            };
            for candidate in points.iter_mut() {
                if candidate.get("name").and_then(Value::as_str).map(str::trim)
                    != Some(point_name)
                {
                    continue;
                }
                let Some(object) = candidate.as_object_mut() else {
                    continue;
                };
                edit(object);
                found = true;
                break;
            }
            break;
        }
        if found {
            self.history.set_ports_block(Some(Value::Array(block)), coalesce);
        }
        found
    }

    // --- the reference picker's connection-point flavour ---------------------

    /// Enter the reference picker to seat `address` on geometry.
    ///
    /// Unlike every feature field, this does NOT roll the history back: a
    /// connection point resolves at the TAIL of the run, against the finished
    /// part, so the geometry it may be seated on is exactly what is on screen.
    /// Rolling would hide from the user the very faces the point can name.
    pub fn begin_ref_select_for_port_point(&mut self, address: &str, field: PortRefField) {
        // The handles come OFF first. The picker takes over the shell and the
        // Qualify panel is not drawn while it is up, so nothing else would
        // take them off — and `route_drag_start` puts the transform gizmo
        // AHEAD of the camera with no picker guard, so a user dragging to
        // orbit round to the face they want to pick would drag the point
        // instead.
        if self.transform_armed_port_point() == Some(address) {
            self.disarm_transform();
        }
        let filter = field.filter();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        let seed = self
            .declared_point_value(address)
            .and_then(|point| {
                point
                    .get(field.key())
                    .and_then(Value::as_str)
                    .map(str::to_string)
            })
            .filter(|name| !name.trim().is_empty())
            .into_iter()
            .collect();
        self.ref_select = Some(RefSelectState {
            feature_id: address.to_string(),
            path: vec![field.key().to_string()],
            label: format!("{} \u{2014} {address}", field.label()),
            filter,
            multiple: false,
            names: seed,
            restore_index: self.history.rollback(),
            target: RefSelectTarget::PortPoint { field },
        });
        self.sync_ref_select_emphasis();
    }

    /// Unseat a connection point: clear one of its reference fields and
    /// re-run. The OFFSET is deliberately left as it is — it is on screen, and
    /// silently rewriting the numbers a user can see is worse than a point
    /// that visibly moved to where those numbers say.
    pub fn clear_port_point_ref(&mut self, address: &str, field: PortRefField) {
        self.ports_commit_ref(address, field, "");
        self.rerun_history();
    }

    /// One declared point's raw object, by address.
    pub(super) fn declared_point_value(&self, address: &str) -> Option<Value> {
        let (port, name) = brep_kernel::split_address(address)?;
        self.history
            .ports_block()?
            .as_array()?
            .iter()
            .find(|group| group.get("name").and_then(Value::as_str).map(str::trim) == Some(port))?
            .get("points")?
            .as_array()?
            .iter()
            .find(|point| point.get("name").and_then(Value::as_str).map(str::trim) == Some(name))
            .cloned()
    }

    /// Commit the picker's name into the point's reference field, WITHOUT
    /// re-running — the picker's shared tail re-runs.
    ///
    /// Installing the FIRST reference on a point also zeroes its transform.
    /// The numbers that were there described an absolute placement; the same
    /// numbers in a seat mean somewhere else entirely, so keeping them would
    /// fling the point off the face the user just picked. Clearing a reference
    /// does NOT restore them: the offset stays as it is, now read in the world
    /// frame, and it is on screen in the panel where the user can see it.
    pub(super) fn ports_commit_ref(&mut self, address: &str, field: PortRefField, name: &str) {
        let name = name.trim().to_string();
        let was_seated = self
            .declared_point_value(address)
            .map(|point| {
                ["pointRef", "directionRef"].iter().any(|key| {
                    point
                        .get(*key)
                        .and_then(Value::as_str)
                        .is_some_and(|value| !value.trim().is_empty())
                })
            })
            .unwrap_or(false);
        self.edit_port_point(address, None, |point| {
            match name.is_empty() {
                true => {
                    point.remove(field.key());
                }
                false => {
                    point.insert(field.key().into(), Value::String(name.clone()));
                }
            }
            if !was_seated && !name.is_empty() {
                point.insert(
                    "transform".into(),
                    serde_json::json!({
                        "position": [0.0, 0.0, 0.0],
                        "rotationEuler": [0.0, 0.0, 0.0],
                    }),
                );
            }
        });
    }

    /// The ports tail's report of the last APPLIED run: whether this document
    /// is an encapsulation boundary, every point it resolved, and everything
    /// about the block that does not hold. `None` before the first run, and
    /// for a document that declares no `ports` block.
    pub fn ports_report(&self) -> Option<&PortsReport> {
        self.ports_report.as_ref()
    }

    /// The Qualify panel's verifier global: the declared block and the tail's
    /// report, as one JSON object. `report` is null before the first run.
    pub fn ports_state_json(&self) -> String {
        serde_json::json!({
            "block": self.history.ports_block(),
            "report": self.ports_report,
        })
        .to_string()
    }
}

