//! The pin seat check on a part's own document: whether each pcb connection
//! point's 3D seat lands on the pad with its pin number
//! (`brep_ecad_core::seat_check`, which holds the rule). This module measures:
//! it reads the part's pads and declared points, and cuts each contact body's
//! display mesh with the board's slabs to get its section.
//!
//! The footprint's frame is the board's: top copper at z = 0, millimetres with
//! y up, where the pads are micrometres with y down (`ecad_parts::board_transform`
//! places a part by the same mapping). The part's model is moved into it by its
//! mount frame (the `mount` block, `brep_kernel::part_mount`), here and nowhere
//! else in the check: both the open part and a library entry are measured in
//! model coordinates, as their features build them, so the mount is applied
//! exactly once.
//!
//! Only a SEATED point is checked, one that names a contact body or a point
//! reference; an unseated point says nothing about where its pin is. A point
//! reference that resolves to nothing is the Ports report's problem, not a pad
//! finding: that point is skipped and the others are still checked. A point
//! reference wins over a contact body: it is the more specific answer, and a
//! contact body of a merged connector is no pin's own.

use brep_ecad_core::seat_check::{pad_seat_check, SeatContact, SeatVerdict, SUBSTRATE_THICKNESS, TOP_SLAB};
use brep_ecad_core::board::Footprint;
use brep_ecad_core::Point;
use brep_render::engine_state::EngineState;
use brep_render::brep_kernel::{PortDeclaration, PortPoint};

/// One checked pin.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub(crate) struct PinSeat {
    /// The point's part-local address (`J1.3`).
    pub address: String,
    pub pin: String,
    /// `seat` (a point reference, read by position) or `body` (a contact body,
    /// read by its section).
    pub by: &'static str,
    pub passes: bool,
    /// The finding in words, `None` when it passes.
    pub message: Option<String>,
}

/// What the check reads of a part, from wherever the part is: its pads block,
/// its ports block, the ports tail's report and its solids' display meshes by
/// name. The open part (Qualify) and a part a board places ([`part_pin_seats`])
/// feed the same measurement, so the two read the same verdict.
struct PartView<'a> {
    pads: Option<&'a serde_json::Value>,
    ports: Option<&'a serde_json::Value>,
    report: Option<&'a brep_render::brep_kernel::PortsReport>,
    solids: &'a [brep_render::scene::SolidDisplay],
    /// The part's mount frame, `None` for identity (no block).
    mount: Option<brep_render::brep_kernel::AffineTransform>,
}

/// Every seated pcb point of the open part, checked. Empty when the part has
/// no pads or no pcb points, and when its mount block does not read as a pose
/// (Qualify's mount section says why: a check in the wrong frame would be
/// worse than none).
pub(crate) fn pin_seats(engine: &EngineState) -> Vec<PinSeat> {
    let Ok(mount) = engine.history.mount_block().map(brep_render::brep_kernel::mount_from_block).transpose() else {
        return Vec::new();
    };
    check(&PartView {
        pads: engine.history.pads_block(),
        ports: engine.history.ports_block(),
        report: engine.ports_report(),
        solids: engine.scene.solids(),
        mount,
    })
}

/// Every seated pcb point of a part document a board places, checked as
/// Qualify checks it on the part itself. The part is built on its own
/// (`brep_kernel::part_pin_geometry`, the library's isolated run), so nothing
/// of the open document's run is touched, and the verdict is kept per
/// document content: a part is measured again only when its document changes.
pub(crate) fn part_pin_seats(document: &serde_json::Value) -> std::rc::Rc<Result<Vec<PinSeat>, String>> {
    thread_local! {
        static VERDICTS: std::cell::RefCell<std::collections::HashMap<u64, std::rc::Rc<Result<Vec<PinSeat>, String>>>> =
            std::cell::RefCell::new(std::collections::HashMap::new());
    }
    // Only a part that has pads and pcb points is worth building.
    let has_pcb = document["ports"].as_array().is_some_and(|groups| {
        groups.iter().any(|group| group["purpose"].as_str().is_some_and(|p| p.trim() == "pcb"))
    });
    if !document["pads"].is_object() || !has_pcb {
        return std::rc::Rc::new(Ok(Vec::new()));
    }
    let key = brep_render::brep_kernel::stable_json_hash(document);
    if let Some(hit) = VERDICTS.with(|cache| cache.borrow().get(&key).cloned()) {
        return hit;
    }
    let verdict = brep_render::brep_kernel::part_mount(document).and_then(|mount| {
        brep_render::brep_kernel::part_pin_geometry(document, 1.0).map(|geometry| (mount, geometry))
    }).map(|(mount, geometry)| {
        let solids: Vec<brep_render::scene::SolidDisplay> = geometry
            .solids
            .into_iter()
            .map(|(name, payload)| brep_render::scene::solid_display_from_payload(&name, payload))
            .collect();
        check(&PartView {
            pads: Some(&document["pads"]),
            ports: Some(&document["ports"]),
            report: geometry.ports.as_ref(),
            solids: &solids,
            mount,
        })
    });
    let verdict = std::rc::Rc::new(verdict);
    VERDICTS.with(|cache| {
        let mut cache = cache.borrow_mut();
        // A long session meets many versions of many parts: keep the cache bounded.
        if cache.len() > 256 {
            cache.clear();
        }
        cache.insert(key, verdict.clone());
    });
    verdict
}

fn check(part: &PartView) -> Vec<PinSeat> {
    let Some(pads) = part.pads.and_then(|v| serde_json::from_value::<Footprint>(v.clone()).ok()) else {
        return Vec::new();
    };
    let groups = part.ports.map(declared_groups).unwrap_or_default();
    let mut out = Vec::new();
    for (group, points) in groups.iter().filter(|(group, _)| group.purpose.trim() == "pcb") {
        for point in points {
            let address = brep_render::brep_kernel::port_address(&group.name, &point.name);
            let pin = point.name.trim().to_owned();
            if let Some(reference) = set(&point.point_ref).filter(|reference| !resolves(part.report, &address, reference)) {
                out.push(PinSeat {
                    message: Some(format!(
                        "Pin {pin}'s seat '{reference}' resolves to no geometry, so it is not checked against its pad: pick its seat again or clear it"
                    )),
                    passes: false,
                    address,
                    pin,
                    by: "seat",
                });
                continue;
            }
            let Some((by, contact)) = contact_of(part, point, &address) else { continue };
            let verdict = pad_seat_check(&pads, &pin, &contact);
            out.push(PinSeat {
                message: verdict.message(&pin, &contact, true),
                passes: verdict == SeatVerdict::OnPad,
                address,
                pin,
                by,
            });
        }
    }
    out
}

/// Align to footprint on the open part (`brep_kernel::mount_aligned_to_footprint`):
/// the mount refitted from the part's seated pcb points onto its pads, with the
/// leftover. The fit reads each point's resolved seat from the ports report,
/// in model coordinates, so it does not depend on the frame the part is in now;
/// an unplaced point and a reference that found nothing are left out, as the
/// pad check leaves them.
pub(crate) fn align_to_footprint(engine: &EngineState) -> Result<brep_render::brep_kernel::AlignReport, String> {
    use brep_render::brep_kernel::{AlignPad, AlignSeat};
    let mount = engine
        .history
        .mount_block()
        .map(brep_render::brep_kernel::mount_from_block)
        .transpose()
        .map_err(|error| format!("the mount block does not read as a pose ({error}): reset it first"))?;
    let pads = engine
        .history
        .pads_block()
        .and_then(|v| serde_json::from_value::<Footprint>(v.clone()).ok())
        .ok_or("the part has no footprint to align to")?;
    let report = engine.ports_report().ok_or("the part's connection points have not been resolved yet")?;
    let groups = engine.history.ports_block().map(declared_groups).unwrap_or_default();
    let mut seats = Vec::new();
    for (group, points) in groups.iter().filter(|(group, _)| group.purpose.trim() == "pcb") {
        for point in points {
            let address = brep_render::brep_kernel::port_address(&group.name, &point.name);
            let reference = set(&point.point_ref);
            if reference.is_none() && set(&point.contact_body).is_none() {
                continue;
            }
            if reference.is_some_and(|reference| !resolves(Some(report), &address, reference)) {
                continue;
            }
            let Some(row) = report.points.iter().find(|row| row.address == address && !row.unplaced) else { continue };
            let support = match (reference, set(&point.contact_body)) {
                (None, Some(body)) => engine
                    .scene
                    .solids()
                    .iter()
                    .find(|solid| solid.name == body)
                    .map(|solid| solid.mesh.positions.iter().map(|p| p.map(f64::from)).collect())
                    .unwrap_or_default(),
                _ => Vec::new(),
            };
            seats.push(AlignSeat { pin: point.name.trim().to_owned(), position: row.position, direction: row.direction, support });
        }
    }
    let pads: Vec<AlignPad> = pads
        .pads
        .iter()
        .filter(|pad| !pad.number.trim().is_empty())
        .map(|pad| AlignPad {
            number: pad.number.trim().to_owned(),
            at: [f64::from(pad.at.x) / 1_000., -f64::from(pad.at.y) / 1_000.],
            smd: pad.drill.is_none(),
        })
        .collect();
    brep_render::brep_kernel::mount_aligned_to_footprint(mount.as_ref(), &seats, &pads)
}

/// The block's groups with their points, read one point at a time: a point
/// the panel has half-edited (a number typed into a reference field, a field
/// of the wrong type) is read with its scalar text fields as text, or skipped,
/// and never takes the other pins' checks down with it.
fn declared_groups(block: &serde_json::Value) -> Vec<(PortDeclaration, Vec<PortPoint>)> {
    let text = |value: Option<&serde_json::Value>| match value {
        Some(serde_json::Value::String(text)) => text.clone(),
        Some(serde_json::Value::Number(n)) => n.to_string(),
        _ => String::new(),
    };
    let Some(groups) = block.as_array() else { return Vec::new() };
    groups
        .iter()
        .map(|group| {
            let declaration = PortDeclaration {
                name: text(group.get("name")),
                purpose: text(group.get("purpose")),
                ..PortDeclaration::default()
            };
            let points = group
                .get("points")
                .and_then(serde_json::Value::as_array)
                .map(|points| {
                    points
                        .iter()
                        .filter_map(|point| {
                            let mut point = point.as_object()?.clone();
                            for key in ["name", "pointRef", "directionRef", "contactBody"] {
                                if let Some(value) = point.get_mut(key).filter(|v| v.is_number() || v.is_boolean()) {
                                    *value = serde_json::Value::String(value.to_string());
                                }
                            }
                            serde_json::from_value::<PortPoint>(serde_json::Value::Object(point)).ok()
                        })
                        .collect()
                })
                .unwrap_or_default();
            (declaration, points)
        })
        .collect()
}

/// Whether a point reference found its geometry: the ports run names the one
/// that did not (`{address}: '{name}' resolves to no geometry`) and still
/// places the point, at its offset, which must not be read as where the pin is.
fn resolves(report: Option<&brep_render::brep_kernel::PortsReport>, address: &str, reference: &str) -> bool {
    let unresolved = format!("{address}: '{reference}' resolves to no geometry");
    !report.is_some_and(|report| report.problems.iter().any(|problem| problem == &unresolved))
}

fn set(text: &Option<String>) -> Option<&str> {
    text.as_deref().map(str::trim).filter(|t| !t.is_empty())
}

/// What `point`'s 3D model puts on the board, in footprint micrometres.
fn contact_of(part: &PartView, point: &PortPoint, address: &str) -> Option<(&'static str, SeatContact)> {
    if set(&point.point_ref).is_some() {
        let row = part.report?.points.iter().find(|row| row.address == address)?;
        let [x, y, _] = mounted(part.mount.as_ref(), row.position);
        return Some(("seat", SeatContact::Seat(footprint_point(x, y))));
    }
    let body = set(&point.contact_body)?;
    let solid = part.solids.iter().find(|solid| solid.name == body)?;
    let Some(mount) = part.mount.as_ref() else {
        return Some(("body", body_contact(&solid.mesh.positions, &solid.mesh.indices)));
    };
    let positions: Vec<[f32; 3]> = solid
        .mesh
        .positions
        .iter()
        .map(|p| {
            let [x, y, z] = mounted(Some(mount), [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])]);
            [x as f32, y as f32, z as f32]
        })
        .collect();
    Some(("body", body_contact(&positions, &solid.mesh.indices)))
}

/// A model-frame point in the footprint frame: moved by the part's mount.
fn mounted(mount: Option<&brep_render::brep_kernel::AffineTransform>, p: [f64; 3]) -> [f64; 3] {
    match mount {
        None => p,
        Some(mount) => {
            let q = mount.point(brep_render::brep_kernel::Vec3::new(p[0], p[1], p[2]));
            [q.x, q.y, q.z]
        }
    }
}

/// A part-frame point (mm, y up) in footprint coordinates (µm, y down).
fn footprint_point(x: f64, y: f64) -> Point {
    Point::new((x * 1_000.).round() as i32, (-y * 1_000.).round() as i32)
}

/// A body's sections in the top-copper slab and through the board (`SUBSTRATE_THICKNESS`),
/// from its triangle mesh, with its lowest point's height in micrometres.
pub(crate) fn body_contact(positions: &[[f32; 3]], indices: &[u32]) -> SeatContact {
    let slab = f64::from(TOP_SLAB) / 1_000.;
    let thickness = f64::from(SUBSTRATE_THICKNESS) / 1_000.;
    let mut top = Vec::new();
    let mut through = Vec::new();
    for tri in indices.chunks_exact(3) {
        let corners: Vec<[f64; 3]> = tri
            .iter()
            .filter_map(|i| positions.get(*i as usize))
            .map(|p| [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])])
            .collect();
        if corners.len() != 3 { continue }
        top.extend(clip(&corners, -slab, slab).into_iter().map(|p| footprint_point(p[0], p[1])));
        through.extend(clip(&corners, -thickness, 0.).into_iter().map(|p| footprint_point(p[0], p[1])));
    }
    let lowest = positions.iter().map(|p| f64::from(p[2])).fold(f64::INFINITY, f64::min);
    let lowest = if lowest.is_finite() { (lowest * 1_000.).round() as i32 } else { i32::MAX };
    SeatContact::Body { top, through, lowest, thickness: SUBSTRATE_THICKNESS }
}

/// The part of a triangle between the planes z = `low` and z = `high`, as the
/// corners of its clipped polygon (empty when it misses the slab).
fn clip(polygon: &[[f64; 3]], low: f64, high: f64) -> Vec<[f64; 3]> {
    let keep = |polygon: Vec<[f64; 3]>, inside: &dyn Fn(f64) -> bool, plane: f64| -> Vec<[f64; 3]> {
        let mut out = Vec::new();
        for i in 0..polygon.len() {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            let (a_in, b_in) = (inside(a[2]), inside(b[2]));
            if a_in {
                out.push(a);
            }
            if a_in != b_in {
                let t = (plane - a[2]) / (b[2] - a[2]);
                out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), plane]);
            }
        }
        out
    };
    let above = keep(polygon.to_vec(), &|z| z >= low, low);
    if above.is_empty() {
        return above;
    }
    keep(above, &|z| z <= high, high)
}

