//! Center (`CNTR`) — hold one component's element(s) centred between two
//! parallel planar faces of another (the "width" mate of other CAD systems).
//! The WIDTH is two parallel planar faces on one component; the TAB is one
//! element, or two like elements, on a second component. The tab's CENTRE
//! geometry — the element itself, or the mid-geometry of the pair (mid-plane
//! of two parallel planes, mid-line of two parallel lines, midpoint of two
//! points) — is held on the width's MID-PLANE:
//!
//! | tab centre | mates | DOF removed |
//! |---|---|---|
//! | plane | `coincident_plane_plane` (mid-plane ↔ tab plane, captured facing, `reverse` flips) | 3 |
//! | line (cylindrical/conical face, straight edge) | `perpendicular` (line ⊥ mid-plane normal) + `coincident_point_plane` | 2 |
//! | point (vertex, circular edge / sphere centre) | `coincident_point_plane` | 1 |
//!
//! The element list is ONE flat `elements` array of THREE refs (two faces to
//! one element) or FOUR (two faces to two elements); the count is the mode.
//! The ROLES are inferred from the selection, never from its pick order: the
//! elements group by owning component (exactly two groups), the group that is
//! two parallel planar faces is the width, the other group is the tab. When
//! both groups qualify (two planes to two planes — a symmetric pairing) the
//! group holding the lexicographically first ref is the width, so the
//! measure's sign and the facing cache are stable under reorders.
//!
//! Every mid-geometry takes its direction from the pair's lexicographically
//! first ref (its "lead") for the same reason: [`effective_align`] keys the
//! `preferredOppose` cache on the ORDER-INDEPENDENT pair signature, so the
//! direction it captured must not flip when the user reorders the picks.
//!
//! A width split across two components (centring C between a face of A and a
//! face of B) is NOT supported: a mate plane lives in one body's frame, and no
//! solver atom expresses "equidistant from two moving planes". It is refused
//! with `unsupported-selection` naming the shape that is accepted.

use super::super::mapping::{
    effective_align, mate, ConstraintFailure, MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::SelectionProbe;
use crate::{MateAxis, MateKind, MatePlane, SelectionGeometry, Vec3};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "center",
    label: "Center",
    short_name: "CNTR",
    icon: "\u{25EB}",
    long_name: "\u{25EB} Center",
    min_elements: 3,
    max_elements: 4,
    duplicate_family: true,
    applicable,
};

/// Two width faces plus a one- or two-element tab, across two components:
/// three or four faces/edges with at least two faces (the width is always
/// planar faces; vertex refs come from the modal picker only).
fn applicable(probe: &SelectionProbe) -> bool {
    probe.all_component
        && probe.components == 2
        && probe.faces >= 2
        && (3..=4).contains(&(probe.faces + probe.edges))
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE", "VERTEX"], 3, 4,
            "Two parallel planar faces on one component (the width) and, on another component, one element or two like elements to centre between them (the tab)"),
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Flip the captured facing preference (planar tab)"
        },
    }))
}

/// Sine of the angle two unit directions may make and still count as
/// parallel: 1e-4 (0.006°) absorbs recognition noise on imported geometry
/// while refusing any face pair that is genuinely not parallel.
const PARALLEL_SIN_TOL: f64 = 1e-4;

/// What accepts the surfacing-tolerance test for "parallel or anti-parallel".
fn parallel(a: Vec3, b: Vec3) -> bool {
    a.cross(b).length() <= PARALLEL_SIN_TOL
}

/// A plane in one frame (world or component-local).
#[derive(Clone, Copy)]
struct Plane {
    origin: Vec3,
    normal: Vec3,
}

impl Plane {
    fn of(geometry: &SelectionGeometry) -> Option<Self> {
        match *geometry {
            SelectionGeometry::Plane { origin, normal } => Some(Self { origin, normal }),
            _ => None,
        }
    }

    /// The plane midway between `self` (the lead: its normal is kept) and
    /// `other`, which must be parallel to it.
    fn mid(self, other: Plane) -> Plane {
        let separation = other.origin.sub(self.origin).dot(self.normal);
        Plane {
            origin: self.origin.add(self.normal.scale(separation * 0.5)),
            normal: self.normal,
        }
    }

    fn mate(self) -> MatePlane {
        MatePlane {
            origin: [self.origin.x, self.origin.y, self.origin.z],
            normal: [self.normal.x, self.normal.y, self.normal.z],
        }
    }

    /// Signed height of `point` above the plane along its normal.
    fn height(self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.normal)
    }
}

/// A line in one frame.
#[derive(Clone, Copy)]
struct Line {
    origin: Vec3,
    direction: Vec3,
}

impl Line {
    fn of(geometry: &SelectionGeometry) -> Option<Self> {
        match *geometry {
            SelectionGeometry::Line { origin, direction }
            | SelectionGeometry::Axis {
                origin, direction, ..
            } => Some(Self { origin, direction }),
            _ => None,
        }
    }

    /// The line midway between `self` (the lead: its direction is kept) and
    /// `other`, which must be parallel to it: through the midpoint of the
    /// lead's origin and its foot on the other line.
    fn mid(self, other: Line) -> Line {
        let along = self.origin.sub(other.origin).dot(other.direction);
        let foot = other.origin.add(other.direction.scale(along));
        Line {
            origin: self.origin.add(foot).scale(0.5),
            direction: self.direction,
        }
    }

    fn mate(self) -> MateAxis {
        MateAxis {
            origin: [self.origin.x, self.origin.y, self.origin.z],
            direction: [self.direction.x, self.direction.y, self.direction.z],
        }
    }
}

/// The tab's centre geometry, classified like the distance module's sides:
/// planes stay planes; cylindrical/conical faces and straight edges are their
/// carrier lines; circular edges, spheres, vertices and whole components are
/// points.
#[derive(Clone, Copy)]
enum Centre {
    Plane(Plane),
    Line(Line),
    Point(Vec3),
}

impl Centre {
    fn of(geometry: &SelectionGeometry) -> Self {
        if let Some(plane) = Plane::of(geometry) {
            Self::Plane(plane)
        } else if let Some(line) = Line::of(geometry) {
            Self::Line(line)
        } else {
            Self::Point(geometry.representative_point())
        }
    }

    /// The mid-geometry of two like elements (`lead` keeps its direction).
    fn mid(lead: Self, other: Self, lead_name: &str, other_name: &str) -> Result<Self, ConstraintFailure> {
        match (lead, other) {
            (Self::Plane(a), Self::Plane(b)) => {
                if !parallel(a.normal, b.normal) {
                    return Err(ConstraintFailure::unsupported(format!(
                        "center: the two tab faces '{lead_name}' and '{other_name}' must be parallel"
                    )));
                }
                Ok(Self::Plane(a.mid(b)))
            }
            (Self::Line(a), Self::Line(b)) => {
                if !parallel(a.direction, b.direction) {
                    return Err(ConstraintFailure::unsupported(format!(
                        "center: the two tab axes '{lead_name}' and '{other_name}' must be parallel"
                    )));
                }
                Ok(Self::Line(a.mid(b)))
            }
            (Self::Point(a), Self::Point(b)) => Ok(Self::Point(a.add(b).scale(0.5))),
            _ => Err(ConstraintFailure::unsupported(format!(
                "center: the two tab elements '{lead_name}' and '{other_name}' must be alike \
                 (two planar faces, two axes, or two points)"
            ))),
        }
    }

    fn anchor(self) -> Vec3 {
        match self {
            Self::Plane(plane) => plane.origin,
            Self::Line(line) => line.origin,
            Self::Point(point) => point,
        }
    }
}

/// One element group (all on one component), lead first: the pair's
/// lexicographically first ref, so every derived direction is pick-order
/// independent.
struct Group<'a> {
    /// Indexes into the entry's `elements`, lead first.
    indexes: Vec<usize>,
    elements: Vec<&'a ResolvedElement>,
}

impl<'a> Group<'a> {
    fn lead(&self) -> &'a ResolvedElement {
        self.elements[0]
    }

    fn names(&self) -> Vec<&str> {
        self.elements.iter().map(|element| element.name.as_str()).collect()
    }

    /// The width mid-plane (world, local) when this group is two parallel
    /// planar faces.
    fn width(&self) -> Option<(Plane, Plane)> {
        let [lead, other] = self.elements.as_slice() else {
            return None;
        };
        let (lw, ow) = (Plane::of(&lead.world)?, Plane::of(&other.world)?);
        if !parallel(lw.normal, ow.normal) {
            return None;
        }
        let (ll, ol) = (Plane::of(&lead.local)?, Plane::of(&other.local)?);
        Some((lw.mid(ow), ll.mid(ol)))
    }

    /// The tab centre (world, local): the single element's own geometry, or
    /// the mid-geometry of the pair.
    fn centre(&self) -> Result<(Centre, Centre), ConstraintFailure> {
        match self.elements.as_slice() {
            [single] => Ok((Centre::of(&single.world), Centre::of(&single.local))),
            [lead, other] => Ok((
                Centre::mid(Centre::of(&lead.world), Centre::of(&other.world), &lead.name, &other.name)?,
                Centre::mid(Centre::of(&lead.local), Centre::of(&other.local), &lead.name, &other.name)?,
            )),
            _ => Err(ConstraintFailure::unsupported(
                "center: the tab is one element or two like elements",
            )),
        }
    }
}

/// Split the resolved elements into their two per-component groups, each
/// lead-first; `unsupported-selection` unless exactly two components take
/// part (the lifecycle has already refused a single component).
fn groups(resolved: &[ResolvedElement]) -> Result<[Group<'_>; 2], ConstraintFailure> {
    let mut by_component: Vec<(&str, Vec<usize>)> = Vec::new();
    for (index, element) in resolved.iter().enumerate() {
        match by_component
            .iter_mut()
            .find(|(component, _)| *component == element.component)
        {
            Some((_, members)) => members.push(index),
            None => by_component.push((element.component.as_str(), vec![index])),
        }
    }
    if by_component.len() != 2 {
        return Err(ConstraintFailure::unsupported(format!(
            "center acts between two components: two parallel planar faces on one (the width) \
             and one or two elements on another (the tab) — the selection spans {} components",
            by_component.len()
        )));
    }
    let build = |mut indexes: Vec<usize>| {
        indexes.sort_by(|&a, &b| resolved[a].name.cmp(&resolved[b].name));
        let elements = indexes.iter().map(|&index| &resolved[index]).collect();
        Group { indexes, elements }
    };
    let mut groups = by_component.into_iter().map(|(_, indexes)| build(indexes));
    Ok([groups.next().expect("two groups"), groups.next().expect("two groups")])
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &mut ConstraintEntry,
    resolved: &[ResolvedElement],
) -> Result<MappedConstraint, ConstraintFailure> {
    let [first, second] = groups(resolved)?;
    // The width: the group that is two parallel planar faces. Both qualifying
    // (2 planes to 2 planes) is symmetric — the group holding the overall
    // lexicographically first ref leads, for a pick-order-independent sign.
    let (width, tab, mid_world, mid_local) = match (first.width(), second.width()) {
        (Some(_), Some(_)) if second.lead().name < first.lead().name => {
            let (world, local) = second.width().expect("checked");
            (second, first, world, local)
        }
        (Some((world, local)), _) => (first, second, world, local),
        (None, Some((world, local))) => (second, first, world, local),
        (None, None) => {
            return Err(ConstraintFailure::unsupported(format!(
                "center needs two parallel planar faces on one component (the width) and one or two \
                 elements on another (the tab); '{}' and '{}' hold no such face pair",
                first.names().join("', '"),
                second.names().join("', '"),
            )));
        }
    };
    let (centre_world, centre_local) = tab.centre()?;
    let width_names = width.names();
    let note = format!(
        "{} centred between {} and {}",
        tab.names().join(" + "),
        width_names[0],
        width_names[1]
    );
    let groups = vec![width.indexes.clone(), tab.indexes.clone()];
    // The tab centre's signed offset from the mid-plane, along its normal.
    let offset = mid_world.height(centre_world.anchor());
    let (w, t) = (width.lead(), tab.lead());
    let mates = match (centre_world, centre_local) {
        // Planar tab: mid-plane and tab plane coplanar, with the captured
        // facing preference (`reverse` flips it) — the touch-align shape.
        (Centre::Plane(plane_world), Centre::Plane(plane_local)) => {
            let reverse = entry.flag("reverse");
            let align = effective_align(entry, mid_world.normal, plane_world.normal, reverse);
            vec![mate(w, t, MateKind::CoincidentPlanePlane {
                plane_a: mid_local.mate(),
                plane_b: plane_local.mate(),
                align,
            })]
        }
        // Line tab: the line lies IN the mid-plane — parallel to it (its
        // direction ⊥ the normal) with its origin on it.
        (_, Centre::Line(line_local)) => vec![
            mate(w, t, MateKind::Perpendicular {
                direction_a: mid_local.mate().normal,
                direction_b: line_local.mate().direction,
            }),
            mate(t, w, MateKind::CoincidentPointPlane {
                point_a: line_local.mate().origin,
                plane_b: mid_local.mate(),
            }),
        ],
        // Point tab: the point lies on the mid-plane.
        (_, Centre::Point(point_local)) => vec![mate(t, w, MateKind::CoincidentPointPlane {
            point_a: [point_local.x, point_local.y, point_local.z],
            plane_b: mid_local.mate(),
        })],
        (_, Centre::Plane(_)) => unreachable!("world and local centres share a kind"),
    };
    Ok(MappedConstraint {
        mates,
        measured: Some((offset, "mm")),
        groups,
        note: Some(note),
        ..Default::default()
    })
}
