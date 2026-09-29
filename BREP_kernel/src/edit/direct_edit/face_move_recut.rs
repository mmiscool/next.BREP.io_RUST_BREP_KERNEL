//! Move Face by RE-CUTTING — the second road, for a motion whose answer is a
//! different topology rather than the same topology re-solved.
//!
//! # Why a second road at all
//!
//! `face_move.rs` and `face_move_carrier.rs` both MOVE A CARRIER and re-solve
//! the boundary that is already there. Where the motion leaves the body with
//! the same faces meeting the same faces, that is the whole answer, and where
//! it leaves two of them crossing, `healing/accept.rs` splits the pair along
//! their exact section and drops the overshoot. Between them they cover every
//! Transform Face the capability matrix records as building.
//!
//! They do not cover a motion whose result needs a face to STOP EXISTING.
//! The 2026-09-22 inbox document is the first report of one: a rectangular
//! through-slot cut into a block, with the slot's CEILING and one of its SIDE
//! WALLS dragged together, the ceiling carried 90 mm — clean out of the top of
//! the part. The right body is the slot re-cut as an open channel, which has
//! the ceiling gone, both side walls re-trimmed against two different outer
//! faces, and the slot's mouth loops on the two end faces merged into those
//! faces' outer loops. Measured against the boolean that cuts the same channel
//! directly: **10F 24E 16V**, where the direct edit leaves 11F.
//!
//! The pairwise crossing repair cannot reach that body, and the reason is
//! structural rather than a threshold. It splits ONE pair of crossing faces
//! along their carriers' section and drops pieces, closed over the faces the
//! imprint cut. Here the two crossings (`Box_PZ | P.CU6_NX` and `Box_PX |
//! P.CU6_PX`) BOUND EACH OTHER — the strip of the top face that has to go is
//! bounded by both sections at once — and the face that has to go whole, the
//! ceiling, is crossed by nothing at all, so no imprint ever cuts it.
//!
//! # The construction
//!
//! A planar face of a solid states a constraint on where the material is:
//! everything inside lies on one side of its carrier. Moving that face moves
//! the constraint, and the material that changes hands is the SLAB the carrier
//! sweeps, bounded sideways by the faces that bound the moved face today. So:
//!
//! 1. Each moved face must carry a PLANE. `delta = translation · n̂`, with `n̂`
//!    the face's outward normal, is how far it moves along its own normal; a
//!    face whose `delta` is zero only slides inside its own carrier and moves
//!    no material at all, so it contributes no tool.
//! 2. The TOOL for that face is the slab between the old and the new carrier,
//!    intersected with one half-space per neighbouring face — each neighbour's
//!    carrier taken at its MOVED position when the neighbour is itself in the
//!    selection, so the tools do not depend on the order they are built in —
//!    and clipped to a box around the body big enough to hold the motion.
//! 3. Which SIDE of each neighbour the tool lies on is read from a PROBE POINT
//!    rather than classified: a point inside the moved face's own trim,
//!    displaced half way along its own normal motion, lies inside the slab by
//!    construction, and the tool is on the side of every neighbour that
//!    contains it. That one rule covers a face moving INTO the body and a face
//!    moving OUT of it without a convexity branch.
//! 4. `delta < 0` — the face moving against its own outward normal — takes
//!    material away, so the tools are SUBTRACTED; `delta > 0` adds it, so they
//!    are UNIONED. A selection whose faces disagree about which is refused by
//!    name: the two tools would bound each other and the order would decide the
//!    answer.
//!
//! The tools are unioned into one before the body is cut, because two tools
//! that share a carrier — and the reported document's two share both `x` and
//! `z` planes — leave coplanar residue when they are subtracted one at a time.
//!
//! # A turn, too
//!
//! Transform Face's motion is rotate-then-translate, and a plane moved rigidly
//! is still a plane, so `recut_moved_planes_rigid` takes the whole motion: the
//! new carrier is the old one carried by it, the tool is the WEDGE between the
//! two (inside one, outside the other — the slab's own two half-spaces, no
//! longer parallel), `delta` is how far a seat inside the trim travels along
//! `n̂` to reach the new carrier, and the probe sits half way there. A turn
//! can carry ONE face into the body and out of it at once, when the line the
//! two carriers meet in crosses the face's bounds; that is refused by name
//! (`one_way_only`). A pure translation keeps this road's original arithmetic
//! to the bit. The 2026-09-22 report that needed it is the slot document with
//! its wall also turned 16.9°: see
//! `tests/suites/inbox_20260922_turned_slot_wall_transform_face.rs`.
//!
//! # What it is not
//!
//! It is not a repair: nothing about the self-intersecting body the direct
//! edit built is read. It is the same edit performed the other way round, from
//! the ORIGINAL body and the motion, which is why it can return a body with a
//! different face count. `transform_face` reaches it only after `accept_sound`
//! has refused, so no motion that builds today changes by a bit, and a motion
//! `move_faces` / `rotate_faces` itself refuses never reaches it.
//!
//! Names are RE-STAMPED rather than collected, because the boolean mints its
//! own faces: a result face whose carrier is an original face's carrier (at its
//! moved position, for a moved face) takes that face's name, a carrier that
//! arrives as two faces takes `name_1` for the second as the breakout already
//! does, and a face whose carrier left the body keeps no name because it is no
//! longer there.

use super::*;
use crate::offset_retrim::plane_of_surface;
use crate::topology::ShellRecord;
use crate::{
    boolean_operation, make_plane, parameter_point_in_face, BooleanOperation, BooleanOptions,
    PolygonClass, Vec2,
};
use crate::curve::parameter_line;
use crate::{KernelRefusal, KernelStage, OrRefuse};

const OP: &str = "move_faces(re-cut)";

/// A closed half-space `n̂·x ≤ offset`, with `n̂` a unit vector.
#[derive(Clone, Copy, Debug)]
struct HalfSpace {
    normal: Vec3,
    offset: f64,
}

/// Stations per axis when an interior point of a face's trim is looked for.
/// A face whose trim no 17×17 lattice point lands inside is refused rather
/// than guessed at.
const PROBE_STATIONS: usize = 16;

/// Re-cut a solid for a pure TRANSLATION of planar faces.
///
/// `solid` is the body BEFORE the edit and `moved` its face ids; the result is
/// the body the same selection and translation define, built as a boolean
/// rather than as a carrier re-solve. See the module header for the
/// construction and for every refusal.
pub fn recut_moved_planes(
    solid: &BrepSolid,
    moved: &[u64],
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    recut_moved_planes_rigid(solid, moved, None, translation)
}

/// The rigid motion a re-cut carries its selection by: an optional rotation
/// (`p ↦ a + R·(p − a)`, the affine `rotate_faces` applies) FOLLOWED by a
/// translation — Transform Face's rotate-then-translate, in its own order.
struct Motion {
    rotation: Option<AffineTransform>,
    translation: Vec3,
}

impl Motion {
    fn point(&self, point: Vec3) -> Vec3 {
        match &self.rotation {
            None => point.add(self.translation),
            Some(rotation) => rotation.point(point).add(self.translation),
        }
    }

    /// The oriented plane `n̂·x = offset`, carried. A pure translation keeps
    /// the exact arithmetic the translation-only road always used —
    /// `offset + t·n̂` — so no body it built changes by a bit.
    fn plane(&self, normal: Vec3, offset: f64) -> (Vec3, f64) {
        match &self.rotation {
            None => (normal, offset + self.translation.dot(normal)),
            Some(rotation) => {
                let turned = super::face_rotate::rotate_direction(rotation, normal);
                (turned, turned.dot(self.point(normal.scale(offset))))
            }
        }
    }
}

/// Re-cut a solid for a RIGID motion of planar faces: `rotation`, when given as
/// `(axis point, unit axis, angle in radians)`, is applied first and
/// `translation` after it — Transform Face's rotate-then-translate.
///
/// A plane moved rigidly is still a plane, so everything the translation road
/// does carries over with one generalisation: the old and the new carrier need
/// no longer be parallel, and the material that changes hands is the WEDGE
/// between them rather than a slab. That wedge is the same two half-spaces —
/// inside the old carrier and outside the new one (material taken away), or
/// the other way round (material added) — so `convex_solid` builds it as it
/// built the slab. What a turn adds is a way for ONE face to do both: when the
/// line the two carriers meet in crosses the face's own bounds, part of it
/// moves into the body and part out. That is refused by name, read from the
/// OTHER wedge rather than from sample points — it is the same half-space set
/// with the carrier pair swapped, and any corner it has off that line is
/// material this tool would have ignored.
pub fn recut_moved_planes_rigid(
    solid: &BrepSolid,
    moved: &[u64],
    rotation: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let moved_set: HashSet<u64> = moved.iter().copied().collect();
    if moved_set.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "selection",
            format!("{OP}: no face was selected"),
        ));
    }
    let motion = Motion {
        rotation: match rotation {
            None => None,
            Some((point, axis, angle)) => {
                let axis = axis.normalized().map_err(|_| {
                    KernelRefusal::input(
                        KernelStage::Collect,
                        "rotation_axis",
                        format!("{OP}: the rotation axis has zero length"),
                    )
                })?;
                Some(
                    super::face_rotate::rotation_about(point, axis, angle)
                        .or_input(KernelStage::Collect, "rotation")?,
                )
            }
        },
        translation,
    };

    // Every moved face's carrier, as an outward-pointing plane. A curved
    // carrier is refused here and not approximated: the slab this road sweeps
    // is a PLANE's slab and nothing else.
    let mut carriers: Vec<(u64, Vec3, f64)> = Vec::new();
    for &id in &moved_set {
        let face = face_by_id(solid, id)?;
        let plane = plane_of_surface(&face.surface, tolerance.max(1e-9) * 10.0, OP).map_err(
            |error| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "curved_carrier",
                    format!(
                        "{error} — this road sweeps a PLANE's slab, so face {id}{} cannot take it",
                        named(face)
                    ),
                )
            },
        )?;
        let normal = if face.same_sense {
            plane.normal
        } else {
            plane.normal.scale(-1.0)
        };
        carriers.push((id, normal, normal.dot(plane.origin)));
    }

    // Which way each moved face travels along its OWN normal, read at a seat
    // inside its trim. For a translation that is `t·n̂`, the same everywhere on
    // the face; for a turn it is how far along `n̂` the seat must go to reach
    // the NEW carrier, and whether it is the same everywhere is read from the
    // other wedge below. A face that only slides inside its carrier moves no
    // material and contributes no tool.
    let motion_floor = (scale * 1e-9).max(1e-12);
    let mut contributors: Vec<(u64, Vec3, f64, f64, Option<Vec3>)> = Vec::new();
    for &(id, normal, offset) in &carriers {
        // A translation reads its seat only once the face is known to move,
        // exactly where the translation-only road read it; a turn needs it to
        // know how far the face moves at all.
        let mut seat = None;
        let delta = if motion.rotation.is_none() {
            translation.dot(normal)
        } else {
            let seat = *seat.insert(interior_point(solid, id, tolerance)?);
            let (moved_normal, moved_offset) = motion.plane(normal, offset);
            let facing = moved_normal.dot(normal);
            if facing <= 1e-6 {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "turned_past_right_angle",
                    format!(
                        "{OP}: the motion turns face {id}{} through a right angle or more, so its \
                         new carrier does not face the way the old one did and there is no wedge \
                         between them for a tool to fill; refusing",
                        named(face_by_id(solid, id)?)
                    ),
                ));
            }
            let reach = (moved_offset - moved_normal.dot(seat)) / facing;
            let unturned = moved_normal.sub(normal).length() <= 1e-12;
            if reach.abs() <= motion_floor && !unturned {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "turned_through_trim",
                    format!(
                        "{OP}: the motion turns face {id}{} about a line through its own trim, so \
                         part of it moves into the body and part out of it; one tool would add \
                         material where the other takes it away, and this road refuses rather than \
                         choosing",
                        named(face_by_id(solid, id)?)
                    ),
                ));
            }
            reach
        };
        if delta.abs() <= motion_floor {
            continue;
        }
        contributors.push((id, normal, offset, delta, seat));
    }
    if contributors.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "in_plane_motion",
            format!(
                "{OP}: every selected face slides inside its own carrier (the motion has no \
                 component along any of their normals), so there is no material for a re-cut to \
                 move"
            ),
        ));
    }
    let adds = contributors[0].3 > 0.0;
    if contributors.iter().any(|entry| (entry.3 > 0.0) != adds) {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "mixed_directions",
            format!(
                "{OP}: the selection moves {} face(s) OUT along their own normals and {} face(s) IN, \
                 so one tool would add material where another takes it away and the order they are \
                 applied in would decide the answer; refusing rather than picking one",
                contributors.iter().filter(|entry| entry.3 > 0.0).count(),
                contributors.iter().filter(|entry| entry.3 < 0.0).count(),
            ),
        ));
    }

    // The clip box: the body's own bounds, grown by the motion and a margin,
    // so a tool is a bounded polytope whatever its half-spaces leave open. A
    // turn carries corners further than its translation alone says, so the
    // bounds then also take every vertex where the motion leaves it.
    let bounds = solid_bounds(solid)?;
    let bounds = if motion.rotation.is_none() {
        bounds
    } else {
        solid.vertices.iter().fold(bounds, |(low, high), vertex| {
            let point = motion.point(vertex.point);
            (
                Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z)),
                Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z)),
            )
        })
    };
    let margin = translation.length() + scale.max(1.0);
    let clip = box_half_spaces(bounds, margin);

    let mut tools: Vec<BrepSolid> = Vec::new();
    for &(id, normal, offset, delta, seat) in &contributors {
        let seat = match seat {
            Some(seat) => seat,
            None => interior_point(solid, id, tolerance)?,
        };
        // The two carriers the face moves between. The tool is inside one and
        // outside the other: inside the OLD and outside the NEW when the face
        // moves in (material leaves), the other way round when it moves out.
        let (moved_normal, moved_offset) = motion.plane(normal, offset);
        let (inner, outer) = if delta < 0.0 {
            ((normal, offset), (moved_normal, moved_offset))
        } else {
            ((moved_normal, moved_offset), (normal, offset))
        };
        let wedge = |inner: (Vec3, f64), outer: (Vec3, f64)| {
            [
                HalfSpace { normal: inner.0, offset: inner.1 },
                HalfSpace { normal: outer.0.scale(-1.0), offset: -outer.1 },
            ]
        };
        let mut spaces = clip.clone();
        if motion.rotation.is_none() {
            // The slab the carrier sweeps, in the arithmetic this road has
            // always used for it.
            spaces.push(HalfSpace {
                normal,
                offset: offset.max(offset + delta),
            });
            spaces.push(HalfSpace {
                normal: normal.scale(-1.0),
                offset: -offset.min(offset + delta),
            });
        } else {
            spaces.extend(wedge(inner, outer));
        }

        // A point inside the moved face's trim, carried half way along its own
        // normal to where its new carrier is: inside the slab (or wedge) by
        // construction, and on the tool's side of every face that bounds it.
        let probe = seat.add(normal.scale(delta * 0.5));

        // The bounds as they stand TODAY, kept beside the ones the tool uses
        // so the trim can be read against them — see `contains_its_own_trim`.
        let mut at_home: Vec<HalfSpace> = Vec::new();
        // The bounds alone, for the other wedge below.
        let mut sides: Vec<HalfSpace> = Vec::new();

        for neighbour in neighbours_of(solid, id) {
            if neighbour == id {
                continue;
            }
            let face = face_by_id(solid, neighbour)?;
            let plane = plane_of_surface(&face.surface, tolerance.max(1e-9) * 10.0, OP).map_err(
                |error| {
                    KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "curved_neighbour",
                        format!(
                            "{error} — face {id}{} is bounded by face {neighbour}{}, and this road \
                             needs every bounding carrier as a half-space",
                            named(face_by_id(solid, id).unwrap_or(face)),
                            named(face)
                        ),
                    )
                },
            )?;
            let mut normal_g = if face.same_sense {
                plane.normal
            } else {
                plane.normal.scale(-1.0)
            };
            let mut home_g = normal_g.dot(plane.origin);
            let mut offset_g = home_g;
            if moved_set.contains(&neighbour) {
                // A neighbour that is itself in the selection bounds the tool
                // where the motion LEAVES it, not where it started: that is
                // what makes the tools independent of the order they are
                // built in.
                (normal_g, offset_g) = motion.plane(normal_g, home_g);
            }
            // HOME stays the unmoved carrier — the plane the trim is read
            // against; only the tool's bound turns with the selection.
            let mut home_normal = if face.same_sense {
                plane.normal
            } else {
                plane.normal.scale(-1.0)
            };
            let reach = normal_g.dot(probe) - offset_g;
            if reach.abs() <= tolerance {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "probe_on_carrier",
                    format!(
                        "{OP}: the probe point for face {id} lies ON the carrier of its neighbour \
                         {neighbour} ({reach:.3e} from it), so which side of that carrier the tool \
                         is on is not decided by the geometry; refusing rather than choosing"
                    ),
                ));
            }
            if reach > 0.0 {
                normal_g = normal_g.scale(-1.0);
                offset_g = -offset_g;
                home_g = -home_g;
                home_normal = home_normal.scale(-1.0);
            }
            let bound = HalfSpace {
                normal: normal_g,
                offset: offset_g,
            };
            spaces.push(bound);
            sides.push(bound);
            at_home.push(HalfSpace {
                normal: home_normal,
                offset: home_g,
            });
        }
        contains_its_own_trim(solid, id, &at_home, scale)?;
        if motion.rotation.is_some() {
            one_way_only(id, &clip, &sides, wedge(outer, inner), tolerance, || {
                named(face_by_id(solid, id).expect("the face was read above"))
            })?;
        }

        tools.push(convex_solid(&spaces, probe, tolerance).map_err(|error| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "tool_polytope",
                format!("{OP}: the tool for face {id} could not be built — {error}"),
            )
        })?);
    }

    // One tool, then one cut. Two tools that share a carrier leave coplanar
    // residue when they are applied one at a time.
    let mut tool = tools.remove(0);
    for next in tools {
        // The boolean's own class rides through: a degeneracy inside the
        // union is still a degeneracy at this road's boundary.
        tool = boolean_operation(&tool, &next, BooleanOperation::Union, &BooleanOptions::default())
            .map_err(|error| {
                error.with_message(|message| {
                    format!("{OP}: the tools could not be unioned into one — {message}")
                })
            })?;
    }
    let operation = if adds {
        BooleanOperation::Union
    } else {
        BooleanOperation::Subtract
    };
    let cut = boolean_operation(solid, &tool, operation, &BooleanOptions::default()).map_err(
        |error| {
            error.with_message(|message| {
                format!(
                    "{OP}: the re-cut's {} refused — {message}",
                    if adds { "union" } else { "subtract" },
                )
            })
        },
    )?;
    Ok(restamp(cut, solid, &moved_set, &motion, tolerance))
}

/// A TURNED face must move one way only. The wedge on the other side of the
/// carriers' common line — the same bounds, the carrier pair swapped — is the
/// material the face would carry the other way; any corner it has further than
/// the band from BOTH carriers means that wedge has volume inside the face's
/// bounds, and the tool this road builds would leave it untouched.
fn one_way_only(
    id: u64,
    clip: &[HalfSpace],
    sides: &[HalfSpace],
    other: [HalfSpace; 2],
    tolerance: f64,
    name: impl Fn() -> String,
) -> Result<(), KernelRefusal> {
    let band = tolerance.max(1e-9) * 10.0;
    let spaces: Vec<HalfSpace> = clip.iter().chain(sides).chain(&other).copied().collect();
    let mut depth = 0.0f64;
    for a in 0..spaces.len() {
        for b in (a + 1)..spaces.len() {
            for c in (b + 1)..spaces.len() {
                let Some(point) = three_plane_point(&spaces[a], &spaces[b], &spaces[c]) else {
                    continue;
                };
                if spaces
                    .iter()
                    .any(|space| space.normal.dot(point) - space.offset > band)
                {
                    continue;
                }
                // How far inside the two carriers this corner is.
                let inside = other
                    .iter()
                    .map(|space| space.offset - space.normal.dot(point))
                    .fold(0.0f64, f64::max);
                depth = depth.max(inside);
            }
        }
    }
    if depth > band * 1e3 {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "one_way_only",
            format!(
                "{OP}: the motion turns face {id}{} so that the line its old and new carriers meet \
                 in crosses its own bounds — part of the face moves into the body and part out of it \
                 (the other wedge reaches {depth:.3e} past them), so one tool would add material \
                 where the other takes it away; refusing rather than choosing",
                name()
            ),
        ));
    }
    Ok(())
}

/// The one way this road could return a plausible WRONG body, closed.
///
/// The tool is the INTERSECTION of the half-spaces the moved face's bounding
/// carriers state, and that intersection is the face's own trim only while the
/// trim IS the region they enclose. It is, for a convex trim every one of whose
/// edges lies on one of those carriers — the pocket, the slot, the boss, the
/// step. It is not for a REFLEX trim: intersecting the bounding half-spaces of
/// an L-shaped floor yields its CONVEX CORE, which can be arbitrarily smaller
/// than the trim, and the material outside that core is simply never cut.
/// Measured, before the guard, on an L pocket whose arms are
/// `x[5,15]×y[5,10]` and `x[5,10]×y[10,15]` in a `20×20×10` block: the core is
/// `x[5,10]×y[5,10]`, the floor pushed out through the bottom built
/// **16F 39E 25V, 3600** where the through-L is **3250**, and `validate()` was
/// clean, the scan was clean, no closed form existed downstream and nothing
/// refused. (The core has the same AREA as the second arm on that fixture and
/// is not that arm — a coincidence of those dimensions, which is exactly why
/// "one arm of the L" reads plausibly and is wrong.) The same shape of error
/// takes a floor with an island down to a strip.
///
/// So the trim is READ against its own bounds, at their HOME positions: a
/// neighbour that is itself in the selection legitimately moves where the trim
/// ends, and one that is not may not exclude any of it. Five points per coedge,
/// every loop — `boundary_samples`, the same sampling a retrim grows a carrier
/// around.
fn contains_its_own_trim(
    solid: &BrepSolid,
    id: u64,
    at_home: &[HalfSpace],
    scale: f64,
) -> Result<(), KernelRefusal> {
    // The trim's boundary lies ON these carriers, so the reading is an equality
    // and the band only has to absorb the residual a built body carries. Every
    // violation this is here to catch is of the order of the feature's own size.
    let band = (scale * 1e-4).max(1e-6);
    let face = face_by_id(solid, id)?;
    let edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    // Every (sample, half-space) pair is compared and the WORST is carried,
    // rather than returning at the first one over the band. Detection is the
    // same either way — any pair over the band refuses — but the number in the
    // refusal is then the geometry's own worst overhang instead of whichever
    // pair the sampling order happened to reach first, so the figure a reader
    // tunes the band against means what it says. On the L pocket that is the
    // difference between 5.000e0 (the whole `x = 15` coedge against `x ≤ 10`)
    // and 2.500e0 (the midpoint of the `y = 10` coedge against the same).
    let mut worst = 0.0f64;
    for point in boundary_samples(face, &edges, OP)? {
        for space in at_home {
            worst = worst.max(space.normal.dot(point) - space.offset);
        }
    }
    if worst > band {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "reflex_trim",
            format!(
                "{OP}: face {id}{} has a boundary point {worst:.3e} outside one of its own \
                 bounding carriers, so its trim is not the region its bounding carriers \
                 enclose — their intersection is the trim's CONVEX CORE, and the tool this \
                 road would build is that core rather than all the material the motion \
                 moves. Refusing rather than cutting some of it",
                named(face)
            ),
        ));
    }
    Ok(())
}

/// The face record, by id.
fn face_by_id(solid: &BrepSolid, id: u64) -> Result<&FaceRecord, KernelRefusal> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == id)
        .ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "face_id",
                format!("{OP}: no face with id {id} on this solid"),
            )
        })
}

fn named(face: &FaceRecord) -> String {
    face.name
        .as_ref()
        .map(|name| format!(" '{name}'"))
        .unwrap_or_default()
}

/// Every face that shares an edge with `id`.
fn neighbours_of(solid: &BrepSolid, id: u64) -> Vec<u64> {
    let mut edges: HashSet<u64> = HashSet::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face.id != id {
            continue;
        }
        for use_ in face.loops.iter().flat_map(|wire| &wire.coedges) {
            edges.insert(use_.edge_id);
        }
    }
    let mut found: Vec<u64> = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face.id == id || found.contains(&face.id) {
            continue;
        }
        if face
            .loops
            .iter()
            .flat_map(|wire| &wire.coedges)
            .any(|use_| edges.contains(&use_.edge_id))
        {
            found.push(face.id);
        }
    }
    found
}

/// A point strictly inside a face's trim, in space.
///
/// The face's own parameter square is sampled on a lattice and every sample
/// `parameter_point_in_face` calls `Inside` is averaged; the average is checked
/// too, so a face whose trim is not convex cannot hand back a point outside it
/// — it falls back to the single sample furthest from the trim's boundary
/// samples instead.
fn interior_point(solid: &BrepSolid, id: u64, tolerance: f64) -> Result<Vec3, KernelRefusal> {
    let face = face_by_id(solid, id)?;
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain")?;
    let mut inside: Vec<Vec2> = Vec::new();
    for iu in 1..PROBE_STATIONS {
        for iv in 1..PROBE_STATIONS {
            let point = Vec2 {
                x: u0 + (u1 - u0) * iu as f64 / PROBE_STATIONS as f64,
                y: v0 + (v1 - v0) * iv as f64 / PROBE_STATIONS as f64,
            };
            if matches!(
                parameter_point_in_face(face, point, tolerance),
                Ok(PolygonClass::Inside)
            ) {
                inside.push(point);
            }
        }
    }
    if inside.is_empty() {
        return Err(KernelRefusal::non_convergence(
            KernelStage::Classify,
            "seat_lattice",
            format!(
                "{OP}: no point of a {PROBE_STATIONS}x{PROBE_STATIONS} lattice over face {id}'s \
                 parameter square lands inside its trim, so this road has no seat to read the tool's \
                 side from"
            ),
        ));
    }
    let mean = Vec2 {
        x: inside.iter().map(|point| point.x).sum::<f64>() / inside.len() as f64,
        y: inside.iter().map(|point| point.y).sum::<f64>() / inside.len() as f64,
    };
    let seat = if matches!(
        parameter_point_in_face(face, mean, tolerance),
        Ok(PolygonClass::Inside)
    ) {
        mean
    } else {
        // A non-convex trim: take the sample furthest from every other
        // classification boundary the lattice found, which is the sample with
        // the most inside neighbours around it.
        *inside
            .iter()
            .max_by(|a, b| {
                let count = |point: &Vec2| {
                    inside
                        .iter()
                        .filter(|other| {
                            (other.x - point.x).abs() <= (u1 - u0) / PROBE_STATIONS as f64 * 1.5
                                && (other.y - point.y).abs()
                                    <= (v1 - v0) / PROBE_STATIONS as f64 * 1.5
                        })
                        .count()
                };
                count(a).cmp(&count(b))
            })
            .expect("the list is not empty")
    };
    face.surface
        .evaluate(seat.x, seat.y)
        .or_refuse(KernelStage::Classify, "evaluate")
}

/// The body's axis-aligned bounds, from its vertices.
fn solid_bounds(solid: &BrepSolid) -> Result<(Vec3, Vec3), KernelRefusal> {
    if solid.vertices.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "empty_solid",
            format!("{OP}: the solid has no vertices to bound"),
        ));
    }
    let mut low = Vec3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Vec3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &solid.vertices {
        low = Vec3::new(
            low.x.min(vertex.point.x),
            low.y.min(vertex.point.y),
            low.z.min(vertex.point.z),
        );
        high = Vec3::new(
            high.x.max(vertex.point.x),
            high.y.max(vertex.point.y),
            high.z.max(vertex.point.z),
        );
    }
    Ok((low, high))
}

/// Six half-spaces around `bounds`, grown by `margin` on every side.
fn box_half_spaces((low, high): (Vec3, Vec3), margin: f64) -> Vec<HalfSpace> {
    let low = Vec3::new(low.x - margin, low.y - margin, low.z - margin);
    let high = Vec3::new(high.x + margin, high.y + margin, high.z + margin);
    vec![
        HalfSpace { normal: Vec3::new(1.0, 0.0, 0.0), offset: high.x },
        HalfSpace { normal: Vec3::new(-1.0, 0.0, 0.0), offset: -low.x },
        HalfSpace { normal: Vec3::new(0.0, 1.0, 0.0), offset: high.y },
        HalfSpace { normal: Vec3::new(0.0, -1.0, 0.0), offset: -low.y },
        HalfSpace { normal: Vec3::new(0.0, 0.0, 1.0), offset: high.z },
        HalfSpace { normal: Vec3::new(0.0, 0.0, -1.0), offset: -low.z },
    ]
}

/// The bounded convex polytope a set of half-spaces encloses, as a solid.
///
/// The vertices are every triple of half-spaces' common point that satisfies
/// all of them; the faces are one per half-space that carries three or more of
/// those vertices, wound so the face normal is the half-space's own — which is
/// outward, because the polytope is on the `≤` side of every one. A half-space
/// that carries fewer than three is redundant and contributes no face.
///
/// `inside` is a point the polytope must contain; it is what makes an empty or
/// degenerate intersection a refusal rather than a body nobody asked for.
fn convex_solid(
    spaces: &[HalfSpace],
    inside: Vec3,
    tolerance: f64,
) -> Result<BrepSolid, String> {
    let band = tolerance.max(1e-9) * 10.0;
    for space in spaces {
        if space.normal.dot(inside) - space.offset > -band {
            return Err(format!(
                "the seat point is not strictly inside the half-space set (it is {:.3e} past one \
                 of them), so the slab and its bounds enclose nothing",
                space.normal.dot(inside) - space.offset
            ));
        }
    }
    // Corners: every triple's common point that every half-space admits.
    let mut corners: Vec<Vec3> = Vec::new();
    for a in 0..spaces.len() {
        for b in (a + 1)..spaces.len() {
            for c in (b + 1)..spaces.len() {
                let Some(point) = three_plane_point(&spaces[a], &spaces[b], &spaces[c]) else {
                    continue;
                };
                if spaces
                    .iter()
                    .any(|space| space.normal.dot(point) - space.offset > band)
                {
                    continue;
                }
                if corners
                    .iter()
                    .any(|other| other.sub(point).length() <= band)
                {
                    continue;
                }
                corners.push(point);
            }
        }
    }
    if corners.len() < 4 {
        return Err(format!(
            "the half-space set has {} corner(s), which is not a bounded volume",
            corners.len()
        ));
    }

    let vertices: Vec<VertexRecord> = corners
        .iter()
        .enumerate()
        .map(|(index, point)| VertexRecord { id: index as u64 + 1, point: *point })
        .collect();
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let mut edge_index: HashMap<(usize, usize), u64> = HashMap::default();
    let mut faces: Vec<FaceRecord> = Vec::new();
    let mut next_id = 1000u64;

    for space in spaces {
        let mut on: Vec<usize> = (0..corners.len())
            .filter(|&index| (space.normal.dot(corners[index]) - space.offset).abs() <= band)
            .collect();
        if on.len() < 3 {
            continue;
        }
        // Wind them about the face's own centroid, counter-clockwise as seen
        // from OUTSIDE (the `+normal` side), which is the sense a solid's face
        // needs.
        let centroid = on
            .iter()
            .fold(Vec3::new(0.0, 0.0, 0.0), |sum, &index| sum.add(corners[index]))
            .scale(1.0 / on.len() as f64);
        let u_dir = corners[on[0]].sub(centroid).normalized()?;
        let v_dir = space.normal.cross(u_dir).normalized()?;
        on.sort_by(|&a, &b| {
            let angle = |index: usize| {
                let delta = corners[index].sub(centroid);
                delta.dot(v_dir).atan2(delta.dot(u_dir))
            };
            angle(a).total_cmp(&angle(b))
        });
        faces.push(polygon_face(
            &vertices,
            &on,
            space.normal,
            &mut edges,
            &mut edge_index,
            &mut next_id,
        )?);
    }
    if faces.len() < 4 {
        return Err(format!(
            "the half-space set produced {} face(s), which does not close a volume",
            faces.len()
        ));
    }

    let shell_id = next_id + 1;
    let solid_id = next_id + 2;
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid_id,
        vertices,
        edges,
        shells: vec![ShellRecord { id: shell_id, faces }],
        genus: 0,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the tool polytope did not validate: {issues:?}"));
    }
    Ok(solid)
}

/// Where three half-spaces' carriers meet, or `None` when they do not meet in
/// a point.
fn three_plane_point(a: &HalfSpace, b: &HalfSpace, c: &HalfSpace) -> Option<Vec3> {
    let cross = b.normal.cross(c.normal);
    let determinant = a.normal.dot(cross);
    if determinant.abs() <= 1e-9 {
        return None;
    }
    let point = cross
        .scale(a.offset)
        .add(c.normal.cross(a.normal).scale(b.offset))
        .add(a.normal.cross(b.normal).scale(c.offset))
        .scale(1.0 / determinant);
    Some(point)
}

/// One planar face of the polytope, from corners already wound about `normal`.
/// Shared edges are minted once and reused, so the shell closes.
fn polygon_face(
    vertices: &[VertexRecord],
    corners: &[usize],
    normal: Vec3,
    edges: &mut Vec<EdgeRecord>,
    edge_index: &mut HashMap<(usize, usize), u64>,
    next_id: &mut u64,
) -> Result<FaceRecord, String> {
    let origin_point = vertices[corners[0]].point;
    let u_dir = vertices[corners[1]]
        .point
        .sub(origin_point)
        .normalized()?;
    let v_dir = normal.cross(u_dir).normalized()?;
    let mut low_u = f64::MAX;
    let mut low_v = f64::MAX;
    let mut high_u = f64::MIN;
    let mut high_v = f64::MIN;
    let projected: Vec<(f64, f64)> = corners
        .iter()
        .map(|&index| {
            let delta = vertices[index].point.sub(origin_point);
            let (u, v) = (delta.dot(u_dir), delta.dot(v_dir));
            low_u = low_u.min(u);
            low_v = low_v.min(v);
            high_u = high_u.max(u);
            high_v = high_v.max(v);
            (u, v)
        })
        .collect();
    let plane_origin = origin_point.add(u_dir.scale(low_u)).add(v_dir.scale(low_v));
    let surface = make_plane(
        plane_origin,
        u_dir,
        v_dir,
        high_u - low_u,
        high_v - low_v,
    )?;

    let mut coedges: Vec<CoedgeRecord> = Vec::with_capacity(corners.len());
    for index in 0..corners.len() {
        let start = corners[index];
        let end = corners[(index + 1) % corners.len()];
        let key = if start < end { (start, end) } else { (end, start) };
        let edge_id = match edge_index.get(&key) {
            Some(id) => *id,
            None => {
                let id = *next_id;
                *next_id += 1;
                edges.push(EdgeRecord {
                    id,
                    curve: make_line(vertices[key.0].point, vertices[key.1].point)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: vertices[key.0].id,
                    end_vertex_id: vertices[key.1].id,
                    degenerate: false,
                    name: None,
                });
                edge_index.insert(key, id);
                id
            }
        };
        let start_uv = projected[index];
        let end_uv = projected[(index + 1) % corners.len()];
        coedges.push(CoedgeRecord {
            id: *next_id,
            edge_id,
            forward: vertices[start].id
                == edges
                    .iter()
                    .find(|edge| edge.id == edge_id)
                    .expect("the edge was just minted")
                    .start_vertex_id,
            pcurve: parameter_line(
                start_uv.0 - low_u,
                start_uv.1 - low_v,
                end_uv.0 - low_u,
                end_uv.1 - low_v,
            )?,
        });
        *next_id += 1;
    }
    let loop_id = *next_id;
    *next_id += 1;
    let face_id = *next_id;
    *next_id += 1;
    Ok(FaceRecord {
        id: face_id,
        surface,
        same_sense: true,
        loops: vec![LoopRecord { id: loop_id, coedges }],
        name: None,
    })
}

/// Give the re-cut body the ORIGINAL body's face names back.
///
/// The boolean mints its own faces, so a collect would lose every name the
/// cut touched. A result face whose carrier is an original face's carrier —
/// at its MOVED position, for a face in the selection — is that face, and takes
/// its name; a carrier that comes back as two faces gives the second `name_1`,
/// the spelling the crossing repair's own split already uses. A name whose
/// carrier is no longer on the body is simply not stamped: the face is gone,
/// which for this road is an answer rather than a loss.
fn restamp(
    mut cut: BrepSolid,
    original: &BrepSolid,
    moved: &HashSet<u64>,
    motion: &Motion,
    tolerance: f64,
) -> BrepSolid {
    let band = tolerance.max(1e-9) * 1e3;
    // Every original name, with the plane it should be found on.
    let mut wanted: Vec<(String, Vec3, f64)> = Vec::new();
    for face in original.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(name) = face.name.clone() else { continue };
        let Ok(plane) = plane_of_surface(&face.surface, band, OP) else {
            continue;
        };
        let normal = if face.same_sense {
            plane.normal
        } else {
            plane.normal.scale(-1.0)
        };
        let (normal, offset) = if moved.contains(&face.id) {
            motion.plane(normal, normal.dot(plane.origin))
        } else {
            (normal, normal.dot(plane.origin))
        };
        wanted.push((name, normal, offset));
    }
    let taken: HashSet<String> = cut
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter_map(|face| face.name.clone())
        .collect();
    let mut used: HashMap<String, usize> = HashMap::default();
    for name in &taken {
        used.insert(name.clone(), 1);
    }
    for shell in &mut cut.shells {
        for face in &mut shell.faces {
            if face.name.is_some() {
                continue;
            }
            let Ok(plane) = plane_of_surface(&face.surface, band, OP) else {
                continue;
            };
            let normal = if face.same_sense {
                plane.normal
            } else {
                plane.normal.scale(-1.0)
            };
            let offset = normal.dot(plane.origin);
            let Some((name, _, _)) = wanted.iter().find(|(_, other, other_offset)| {
                other.sub(normal).length() <= 1e-9 && (other_offset - offset).abs() <= band
            }) else {
                continue;
            };
            let count = used.entry(name.clone()).or_insert(0);
            *count += 1;
            face.name = Some(if *count == 1 {
                name.clone()
            } else {
                format!("{name}_{}", *count - 1)
            });
        }
    }
    cut
}
