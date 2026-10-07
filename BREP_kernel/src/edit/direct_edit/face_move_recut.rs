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
//! 1. Each moved face must carry a PLANE, and its trim must be one loop of
//!    straight edges (`trim_polygon`). `delta` is how far the face moves
//!    along its own outward normal `n̂` — `translation · n̂` for a
//!    translation; a face whose `delta` is zero only slides inside its own
//!    carrier, moves no material at all, and contributes no tool.
//! 2. The trim is cut into convex CELLS (`convex_cells`): along its own
//!    reflex corners — each cut extends the edge arriving at the corner across
//!    the face to the boundary, so the new edge still lies on that edge's own
//!    neighbour carrier — and, for a turn, along the HINGE the old and new
//!    carriers meet in, wherever it crosses a cell. Every cell then lies on
//!    one side of every plane that bounds it and moves one way.
//! 3. The TOOL for a cell is the slab (for a turn, the wedge) between the
//!    face's old and new carrier, intersected with one half-space per cell
//!    edge — the neighbour's carrier across that edge, or the plane through
//!    the hinge — and clipped to a box around the body big enough to hold the
//!    motion.
//! 4. Which SIDE of each bound the tool lies on is read from a PROBE POINT
//!    rather than classified: the cell's centroid, displaced half way along
//!    the cell's own normal motion, lies inside the slab (or wedge) by
//!    construction, and the tool is on the side of every bound that contains
//!    it. That one rule covers a cell moving INTO the body and a cell moving
//!    OUT of it without a convexity branch.
//! 5. Which POSITION of a neighbour that is itself in the selection bounds
//!    the tool — where it was, or where the motion leaves it — is set algebra
//!    on the feature the two faces bound together. For a boss `P = ∩ Mⱼ` (the
//!    material half-spaces) the material added is `P' − P = ∪ᵢ (P' ∩ ¬Mᵢ)`,
//!    bounded by the neighbours where the motion LEAVES them, and the material
//!    removed is `P − P' = ∪ᵢ (P ∩ ¬M'ᵢ)`, bounded where they WERE; for a
//!    cavity `C = ∩ ¬Mⱼ` the two swap. Read locally per (cell, neighbour): the
//!    neighbour's MOVED plane iff the cell's seat is on the neighbour's
//!    material side exactly when the cell moves out; its HOME plane otherwise.
//!    (Always-moved, the rule until 2026-09-30, is the growth half of this; a
//!    slot wall swinging out below a ceiling lifted clean out of the part would
//!    have been filled up to that ceiling, a fin above the part.)
//! 6. `delta < 0` — a cell moving against the face's outward normal — takes
//!    material away, `delta > 0` adds it. The removing cells are unioned into
//!    one tool and SUBTRACTED, the adding cells into another and UNIONED, in
//!    that order. In the set algebra above the two tools are disjoint (what
//!    a face removes lies inside the body's old bounds, what it adds outside
//!    them), and on every fixture measured the order made no difference; a
//!    selection that does both is what a turn across the hinge IS, and is no
//!    longer refused.
//!
//! The cells of a direction are unioned before the body is cut, because two
//! tools that share a carrier — the two arms of an L, or the reported
//! document's two faces, which share both `x` and `z` planes — leave coplanar
//! residue when they are subtracted one at a time.
//!
//! Two guards read every cell against its own bounds, so a decomposition that
//! left something for a tool to miss refuses by name rather than cutting some
//! of the material: `cell_is_its_bounds` (class `reflex_trim`; the pre-cell
//! road's convex-core failure, measured at 3600 for a through-L of 3250) and
//! `one_way_only` (the other wedge of a turned cell has volume inside the
//! cell's bounds). Neither fires on a cell the cut produced; both fire the
//! moment the cut is stubbed out.
//!
//! # A turn, too
//!
//! Transform Face's motion is rotate-then-translate, and a plane moved rigidly
//! is still a plane, so `recut_moved_planes_rigid` takes the whole motion: the
//! new carrier is the old one carried by it, the tool is the WEDGE between the
//! two (inside one, outside the other — the slab's own two half-spaces, no
//! longer parallel), `delta` is how far a cell's seat travels along `n̂` to
//! reach the new carrier, and the probe sits half way there. A turn whose
//! hinge crosses the face carries one side into the body and the other out;
//! the hinge split gives each side its own cell and its own tool. A pure
//! translation keeps this road's original SLAB arithmetic (`offset.max(offset
//! + delta)`), so a convex face's slab is the same set of planes it was. The
//! 2026-09-22 report that needed the turn is the slot document with its wall
//! also turned 16.9°
//! (`tests/suites/inbox_20260922_turned_slot_wall_transform_face.rs`); the
//! same document turned 10° about a line ACROSS the wall is the case the
//! hinge split closed on 2026-09-30.
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
    boolean_operation, make_plane, BooleanOperation, BooleanOptions,

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
    /// `offset + t·n̂`.
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
/// built the slab.
///
/// The tool for one face is built per CELL of its trim rather than once for
/// the face (`convex_cells`): the trim is cut into convex pieces along its own
/// reflex edges, and a turned face is cut along the HINGE its two carriers
/// meet in, so a cell lies on one side of every plane that bounds it and moves
/// one way only. Each cell's tool is the slab or wedge intersected with one
/// half-space per cell edge, the cells that take material away are unioned
/// into one tool and subtracted, and the cells that add material are unioned
/// into another and added. Two guards read every cell against its own bounds
/// (`cell_is_its_bounds`, `one_way_only`), so a decomposition that left a cell
/// reflex or straddling the hinge refuses by name rather than cutting some of
/// the material.
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
    // Deterministic order: the tools are unioned in this order.
    carriers.sort_by_key(|entry| entry.0);

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

    let motion_floor = (scale * 1e-9).max(1e-12);
    let edge_band = (scale * 1e-7).max(1e-9);
    let mut removing: Vec<BrepSolid> = Vec::new();
    let mut adding: Vec<BrepSolid> = Vec::new();

    for &(id, normal, offset) in &carriers {
        let face = face_by_id(solid, id)?;
        let (moved_normal, moved_offset) = motion.plane(normal, offset);
        let facing = moved_normal.dot(normal);
        if motion.rotation.is_some() && facing <= 1e-6 {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "turned_past_right_angle",
                format!(
                    "{OP}: the motion turns face {id}{} through a right angle or more, so its \
                     new carrier does not face the way the old one did and there is no wedge \
                     between them for a tool to fill; refusing",
                    named(face)
                ),
            ));
        }
        let unturned = moved_normal.sub(normal).length() <= 1e-12;
        // A pure translation reads the SAME delta everywhere on the face, in
        // the arithmetic the translation-only road always used.
        let translation_delta = translation.dot(normal);
        if motion.rotation.is_none() && translation_delta.abs() <= motion_floor {
            // Slides inside its own carrier: no material moves, no tool.
            continue;
        }

        // Every face that bounds this one, with its outward plane at HOME and
        // where the motion LEAVES it (the same plane for an unmoved face).
        let mut neighbour_planes: HashMap<u64, ((Vec3, f64), (Vec3, f64))> = HashMap::default();
        let polygon = trim_polygon(solid, face, normal, edge_band)?;
        for source in &polygon.sources {
            let &EdgeSource::Neighbour(neighbour) = source else { continue };
            if neighbour_planes.contains_key(&neighbour) {
                continue;
            }
            let other = face_by_id(solid, neighbour)?;
            let plane = plane_of_surface(&other.surface, tolerance.max(1e-9) * 10.0, OP)
                .map_err(|error| {
                    KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "curved_neighbour",
                        format!(
                            "{error} — face {id}{} is bounded by face {neighbour}{}, and this road \
                             needs every bounding carrier as a half-space",
                            named(face),
                            named(other)
                        ),
                    )
                })?;
            let outward = if other.same_sense {
                plane.normal
            } else {
                plane.normal.scale(-1.0)
            };
            let home = (outward, outward.dot(plane.origin));
            let left = if moved_set.contains(&neighbour) {
                motion.plane(home.0, home.1)
            } else {
                home
            };
            neighbour_planes.insert(neighbour, (home, left));
        }

        // The hinge: the line the old and the new carrier meet in, in the
        // face's own 2-D frame — only a TURN has one.
        let hinge = if unturned {
            None
        } else {
            polygon.frame.hinge(normal, offset, moved_normal, moved_offset)
        };
        let cells = convex_cells(&polygon, hinge, edge_band, scale).map_err(|error| {
            KernelRefusal::internal(
                KernelStage::Classify,
                "cell_split",
                format!("{OP}: face {id}{}'s trim could not be cut into convex cells — {error}", named(face)),
            )
        })?;

        for cell in &cells {
            let seat = polygon.frame.lift(cell.centroid());
            // How far this cell travels along the face's own normal to reach
            // the new carrier: `t·n̂` for a translation, the same everywhere;
            // for a turn, read at the cell's own seat, which is on one side of
            // the hinge by construction.
            let delta = if motion.rotation.is_none() {
                translation_delta
            } else {
                (moved_offset - moved_normal.dot(seat)) / facing
            };
            if delta.abs() <= motion_floor {
                if unturned {
                    continue;
                }
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "turned_through_trim",
                    format!(
                        "{OP}: the motion turns face {id}{} about a line through its own trim, and a \
                         cell of the trim still sits on that line after the hinge split, so which way \
                         it moves is not decided by the geometry; refusing rather than choosing",
                        named(face)
                    ),
                ));
            }
            let adds = delta > 0.0;

            // The two carriers the cell moves between. The tool is inside one
            // and outside the other: inside the OLD and outside the NEW when
            // the cell moves in (material leaves), the other way round when it
            // moves out.
            let (inner, outer) = if adds {
                ((moved_normal, moved_offset), (normal, offset))
            } else {
                ((normal, offset), (moved_normal, moved_offset))
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

            // A point inside the cell, carried half way along the face's own
            // normal to where its new carrier is: inside the slab (or wedge) by
            // construction, and on the tool's side of every plane that bounds it.
            let probe = seat.add(normal.scale(delta * 0.5));

            // The bounds as they stand TODAY, kept beside the ones the tool
            // uses so the cell can be read against them (`cell_is_its_bounds`).
            let mut at_home: Vec<HalfSpace> = Vec::new();
            // The bounds alone, for the other wedge (`one_way_only`).
            let mut sides: Vec<HalfSpace> = Vec::new();

            for (edge, source) in cell.sources.iter().enumerate() {
                let (home, bound) = match source {
                    EdgeSource::Neighbour(neighbour) => {
                        let &(home, left) = neighbour_planes
                            .get(neighbour)
                            .expect("every neighbour's plane was read above");
                        // Which position of a MOVED neighbour bounds the tool is
                        // decided by set algebra on the feature the two faces
                        // bound together. For a convex edge (the cell is on the
                        // neighbour's MATERIAL side — a boss's top against its
                        // wall) material added is `P' − P`, bounded by the
                        // neighbours where the motion leaves them, and material
                        // removed is `P − P'`, bounded where they were. For a
                        // concave edge (the cell is on the neighbour's EMPTY
                        // side — a slot's ceiling against its wall) it is the
                        // other way round: the cavity that grows is bounded by
                        // its new walls, the cavity that shrinks by its old ones.
                        // So: the neighbour's MOVED plane iff the cell sits on
                        // its material side exactly when the cell moves out.
                        let material_side = home.0.dot(seat) - home.1 < 0.0;
                        let bound = if material_side == adds { left } else { home };
                        (home, bound)
                    }
                    EdgeSource::Hinge => {
                        let plane = polygon.frame.edge_plane(cell, edge);
                        (plane, plane)
                    }
                };
                let reach = bound.0.dot(probe) - bound.1;
                if reach.abs() <= tolerance {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "probe_on_carrier",
                        format!(
                            "{OP}: the probe point for face {id} lies ON the carrier of one of its \
                             bounds ({reach:.3e} from it), so which side of that carrier the tool \
                             is on is not decided by the geometry; refusing rather than choosing"
                        ),
                    ));
                }
                let sign = if reach > 0.0 { -1.0 } else { 1.0 };
                let bound = HalfSpace {
                    normal: bound.0.scale(sign),
                    offset: bound.1 * sign,
                };
                // HOME is oriented by the SEAT, which lies in the face's plane
                // and strictly inside the cell: the side the trim is on.
                let home_sign = if home.0.dot(seat) - home.1 > 0.0 { -1.0 } else { 1.0 };
                push_unique(&mut spaces, bound, tolerance);
                push_unique(&mut sides, bound, tolerance);
                push_unique(
                    &mut at_home,
                    HalfSpace {
                        normal: home.0.scale(home_sign),
                        offset: home.1 * home_sign,
                    },
                    tolerance,
                );
            }
            cell_is_its_bounds(face, cell, &polygon.frame, &at_home, scale)?;
            if motion.rotation.is_some() {
                one_way_only(id, &clip, &sides, wedge(outer, inner), tolerance, || named(face))?;
            }

            let tool = convex_solid(&spaces, probe, tolerance).map_err(|error| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "tool_polytope",
                    format!("{OP}: the tool for face {id} could not be built — {error}"),
                )
            })?;
            if adds {
                adding.push(tool);
            } else {
                removing.push(tool);
            }
        }
    }
    if removing.is_empty() && adding.is_empty() {
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

    // One tool per direction, then one cut per direction. Two tools that share
    // a carrier leave coplanar residue when they are applied one at a time,
    // so the cells of a direction are unioned before the body is touched. The
    // two directions' tools are disjoint by construction — what a face removes
    // lies inside the body's old bounds and what it adds lies outside them —
    // so the order of the two cuts is not a choice of answer; the subtract
    // runs first so the added material meets a body already at its new bounds.
    let unite = |mut tools: Vec<BrepSolid>| -> Result<Option<BrepSolid>, KernelRefusal> {
        if tools.is_empty() {
            return Ok(None);
        }
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
        Ok(Some(tool))
    };
    let mut cut = solid.clone();
    if let Some(tool) = unite(removing)? {
        cut = boolean_operation(&cut, &tool, BooleanOperation::Subtract, &BooleanOptions::default())
            .map_err(|error| {
                error.with_message(|message| format!("{OP}: the re-cut's subtract refused — {message}"))
            })?;
    }
    if let Some(tool) = unite(adding)? {
        cut = boolean_operation(&cut, &tool, BooleanOperation::Union, &BooleanOptions::default())
            .map_err(|error| {
                error.with_message(|message| format!("{OP}: the re-cut's union refused — {message}"))
            })?;
    }
    Ok(restamp(cut, solid, &moved_set, &motion, tolerance))
}

/// Add a half-space unless an equal one (same normal, same offset within the
/// band) is already listed: two cell edges on one carrier — a bookkeeping split,
/// or an edge and the extension the reflex cut drew through it — must not put
/// two coincident faces on the polytope.
fn push_unique(spaces: &mut Vec<HalfSpace>, space: HalfSpace, tolerance: f64) {
    let band = tolerance.max(1e-9) * 10.0;
    if spaces.iter().any(|other| {
        other.normal.sub(space.normal).length() <= 1e-9 && (other.offset - space.offset).abs() <= band
    }) {
        return;
    }
    spaces.push(space);
}

/// What a cell's edge lies on: the carrier of the face across that edge (an
/// edge of the trim, or the extension of one the reflex cut drew), or the
/// hinge line a turn splits the face along.
#[derive(Clone, Copy, Debug, PartialEq)]
enum EdgeSource {
    Neighbour(u64),
    Hinge,
}

/// A 2-D frame on a moved face's carrier: `origin + x·u + y·v`, with
/// `u × v = n̂`, so counter-clockwise in `(x, y)` is counter-clockwise about
/// the OUTWARD normal.
#[derive(Clone, Copy, Debug)]
struct Frame {
    origin: Vec3,
    u: Vec3,
    v: Vec3,
}

impl Frame {
    fn new(origin: Vec3, normal: Vec3) -> Result<Self, String> {
        let u = normal.perpendicular()?.normalized()?;
        let v = normal.cross(u).normalized()?;
        Ok(Self { origin, u, v })
    }

    fn drop(&self, point: Vec3) -> [f64; 2] {
        let delta = point.sub(self.origin);
        [delta.dot(self.u), delta.dot(self.v)]
    }

    fn lift(&self, point: [f64; 2]) -> Vec3 {
        self.origin.add(self.u.scale(point[0])).add(self.v.scale(point[1]))
    }

    /// The line the carrier `n̂·x = o` and the carrier `m̂·x = p` meet in, in
    /// this frame, as `(point, unit direction)`; `None` when they are parallel.
    fn hinge(&self, normal: Vec3, offset: f64, moved: Vec3, moved_offset: f64) -> Option<Line2> {
        let direction = normal.cross(moved);
        if direction.length() <= 1e-12 {
            return None;
        }
        let direction = direction.normalized().ok()?;
        // A point on both planes: the frame's origin is on the first; slide it
        // within the first plane along the in-plane normal of the second.
        let across = direction.cross(normal).normalized().ok()?;
        let rate = moved.dot(across);
        if rate.abs() <= 1e-12 {
            return None;
        }
        let _ = offset;
        let point = self.origin.add(across.scale((moved_offset - moved.dot(self.origin)) / rate));
        let [px, py] = self.drop(point);
        let [qx, qy] = self.drop(point.add(direction));
        Some(Line2 {
            point: [px, py],
            direction: [qx - px, qy - py],
        })
    }

    /// The plane through the cell's `edge`, perpendicular to the face, as an
    /// outward half-space read from the cell's inside: the cell is on its `≤`
    /// side.
    fn edge_plane(&self, cell: &Cell, edge: usize) -> (Vec3, f64) {
        let a = cell.points[edge];
        let b = cell.points[(edge + 1) % cell.points.len()];
        // Counter-clockwise polygon: the outward normal of edge a→b is the
        // edge direction turned clockwise.
        let d = [b[0] - a[0], b[1] - a[1]];
        let length = (d[0] * d[0] + d[1] * d[1]).sqrt().max(1e-300);
        let out = [d[1] / length, -d[0] / length];
        let normal = self.u.scale(out[0]).add(self.v.scale(out[1]));
        let normal = normal.normalized().unwrap_or(normal);
        (normal, normal.dot(self.lift(a)))
    }
}

/// A line in a `Frame`.
#[derive(Clone, Copy, Debug)]
struct Line2 {
    point: [f64; 2],
    direction: [f64; 2],
}

impl Line2 {
    /// Signed distance of `p` from the line: positive to the LEFT of the
    /// direction.
    fn side(&self, p: [f64; 2]) -> f64 {
        let d = self.direction;
        let length = (d[0] * d[0] + d[1] * d[1]).sqrt().max(1e-300);
        (d[0] * (p[1] - self.point[1]) - d[1] * (p[0] - self.point[0])) / length
    }
}

/// A planar polygon with each edge's source: edge `k` runs from `points[k]`
/// to `points[k + 1]` (wrapping) and lies on `sources[k]`. Wound
/// counter-clockwise about the face's outward normal.
#[derive(Clone, Debug)]
struct Cell {
    points: Vec<[f64; 2]>,
    sources: Vec<EdgeSource>,
}

impl Cell {
    fn signed_area(&self) -> f64 {
        let n = self.points.len();
        (0..n)
            .map(|i| {
                let a = self.points[i];
                let b = self.points[(i + 1) % n];
                a[0] * b[1] - b[0] * a[1]
            })
            .sum::<f64>()
            * 0.5
    }

    /// The area centroid — strictly inside a convex cell.
    fn centroid(&self) -> [f64; 2] {
        let n = self.points.len();
        let area = self.signed_area();
        if area.abs() <= 1e-300 {
            let mean = self.points.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0], s[1] + p[1]]);
            return [mean[0] / n as f64, mean[1] / n as f64];
        }
        let mut cx = 0.0;
        let mut cy = 0.0;
        for i in 0..n {
            let a = self.points[i];
            let b = self.points[(i + 1) % n];
            let w = a[0] * b[1] - b[0] * a[1];
            cx += (a[0] + b[0]) * w;
            cy += (a[1] + b[1]) * w;
        }
        [cx / (6.0 * area), cy / (6.0 * area)]
    }

    /// The first vertex where the boundary turns RIGHT (a reflex corner of a
    /// counter-clockwise polygon), or `None` for a convex one. A straight
    /// continuation (a bookkeeping split of one edge) is not a turn.
    fn reflex_vertex(&self, band: f64) -> Option<usize> {
        let n = self.points.len();
        (0..n).find(|&i| {
            let p = self.points[(i + n - 1) % n];
            let q = self.points[i];
            let r = self.points[(i + 1) % n];
            let a = [q[0] - p[0], q[1] - p[1]];
            let b = [r[0] - q[0], r[1] - q[1]];
            let la = (a[0] * a[0] + a[1] * a[1]).sqrt();
            let lb = (b[0] * b[0] + b[1] * b[1]).sqrt();
            if la <= band || lb <= band {
                return false;
            }
            // The turn's sine times the shorter edge: a right turn deeper than
            // the band.
            (a[0] * b[1] - a[1] * b[0]) / la.max(lb) < -band
        })
    }
}

/// The polygon a moved face's trim is, with its frame.
struct TrimPolygon {
    frame: Frame,
    points: Vec<[f64; 2]>,
    sources: Vec<EdgeSource>,
}

/// Read a moved face's trim as a polygon in its own frame: one loop, every
/// edge straight, every edge shared with exactly one other face, which is the
/// edge's source. Refuses by name what it cannot read: an island (a second
/// loop, whose neighbours bound nothing this decomposition sees), a curved
/// edge (which a planar neighbour cannot produce) and an edge no other face
/// holds.
fn trim_polygon(
    solid: &BrepSolid,
    face: &FaceRecord,
    normal: Vec3,
    band: f64,
) -> Result<TrimPolygon, KernelRefusal> {
    let id = face.id;
    if face.loops.len() != 1 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "island_trim",
            format!(
                "{OP}: face {id}{} has {} loops, so its trim has an island whose walls bound no cell \
                 of the outer trim, and this road cuts one convex cell at a time; refusing rather \
                 than filling the island",
                named(face),
                face.loops.len()
            ),
        ));
    }
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertices: HashMap<u64, Vec3> =
        solid.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    // The face across each edge of this face.
    let mut across: HashMap<u64, u64> = HashMap::default();
    for other in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if other.id == id {
            continue;
        }
        for coedge in other.loops.iter().flat_map(|wire| &wire.coedges) {
            across.entry(coedge.edge_id).or_insert(other.id);
        }
    }
    let mut points3: Vec<Vec3> = Vec::new();
    let mut sources: Vec<EdgeSource> = Vec::new();
    for coedge in &face.loops[0].coedges {
        let edge = edges.get(&coedge.edge_id).ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "missing_edge",
                format!("{OP}: face {id} uses edge {} which the solid does not hold", coedge.edge_id),
            )
        })?;
        if edge.degenerate {
            continue;
        }
        let (start_id, end_id) = if coedge.forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        };
        let start = *vertices.get(&start_id).ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "missing_vertex",
                format!("{OP}: edge {} starts at vertex {start_id}, which the solid does not hold", edge.id),
            )
        })?;
        let end = *vertices.get(&end_id).ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "missing_vertex",
                format!("{OP}: edge {} ends at vertex {end_id}, which the solid does not hold", edge.id),
            )
        })?;
        // Straight: the curve's midpoint is the chord's.
        let middle = edge
            .curve
            .evaluate(0.5 * (edge.t0 + edge.t1))
            .or_refuse(KernelStage::Classify, "evaluate")?;
        let chord_middle = start.add(end).scale(0.5);
        if middle.sub(chord_middle).length() > band.max(1e-9) * 100.0 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "curved_edge",
                format!(
                    "{OP}: edge {} of face {id}{} is not straight (its midpoint is {:.3e} off its \
                     chord), so the trim is not a polygon this road can cut into cells",
                    edge.id,
                    named(face),
                    middle.sub(chord_middle).length()
                ),
            ));
        }
        let neighbour = *across.get(&edge.id).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "open_edge",
                format!(
                    "{OP}: edge {} of face {id}{} is held by no other face, so nothing bounds the \
                     tool there",
                    edge.id,
                    named(face)
                ),
            )
        })?;
        points3.push(start);
        sources.push(EdgeSource::Neighbour(neighbour));
    }
    if points3.len() < 3 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "thin_trim",
            format!("{OP}: face {id}{} has fewer than three corners", named(face)),
        ));
    }
    let frame = Frame::new(points3[0], normal).map_err(|error| {
        KernelRefusal::internal(KernelStage::Classify, "frame", format!("{OP}: {error}"))
    })?;
    let mut points: Vec<[f64; 2]> = points3.iter().map(|point| frame.drop(*point)).collect();
    let probe = Cell { points: points.clone(), sources: sources.clone() };
    if probe.signed_area() < 0.0 {
        // Wound clockwise about the outward normal: reverse, keeping each
        // edge's source with its edge.
        points.reverse();
        sources.reverse();
        sources.rotate_left(1);
        let _ = &mut points;
    }
    Ok(TrimPolygon { frame, points, sources })
}

/// Cut a trim polygon into convex cells: first along its own reflex corners
/// (each cut extends the edge ARRIVING at the corner through the interior to
/// the boundary, so the new edge lies on that edge's own carrier), then, for
/// a turn, along the hinge wherever it crosses a cell. Cells thinner than the
/// band are dropped.
fn convex_cells(
    polygon: &TrimPolygon,
    hinge: Option<Line2>,
    band: f64,
    scale: f64,
) -> Result<Vec<Cell>, String> {
    let area_floor = (band * scale).max(1e-18);
    let mut pending = vec![Cell {
        points: polygon.points.clone(),
        sources: polygon.sources.clone(),
    }];
    let mut convex: Vec<Cell> = Vec::new();
    let mut rounds = 0usize;
    while let Some(cell) = pending.pop() {
        rounds += 1;
        if rounds > 256 {
            return Err("the reflex cut did not converge in 256 rounds".into());
        }
        let cell = dedupe(cell, band);
        if cell.points.len() < 3 || cell.signed_area().abs() <= area_floor {
            continue;
        }
        match cell.reflex_vertex(band) {
            None => convex.push(cell),
            Some(vertex) => {
                let (first, second) = split_at_reflex(&cell, vertex, band)?;
                pending.push(first);
                pending.push(second);
            }
        }
    }
    let Some(hinge) = hinge else { return Ok(convex) };
    let mut cells: Vec<Cell> = Vec::new();
    for cell in convex {
        let sides: Vec<f64> = cell.points.iter().map(|point| hinge.side(*point)).collect();
        let high = sides.iter().copied().fold(f64::MIN, f64::max);
        let low = sides.iter().copied().fold(f64::MAX, f64::min);
        if high > band && low < -band {
            for half in split_by_line(&cell, &sides, band) {
                let half = dedupe(half, band);
                if half.points.len() >= 3 && half.signed_area().abs() > area_floor {
                    cells.push(half);
                }
            }
        } else {
            cells.push(cell);
        }
    }
    Ok(cells)
}

/// Drop consecutive points closer than the band, keeping the source of the
/// edge that LEAVES the surviving point.
fn dedupe(cell: Cell, band: f64) -> Cell {
    let mut points: Vec<[f64; 2]> = Vec::new();
    let mut sources: Vec<EdgeSource> = Vec::new();
    for (point, source) in cell.points.iter().zip(&cell.sources) {
        if let Some(last) = points.last() {
            if (last[0] - point[0]).hypot(last[1] - point[1]) <= band {
                // The edge leaving the merged point is this one's.
                *sources.last_mut().expect("as many sources as points") = *source;
                continue;
            }
        }
        points.push(*point);
        sources.push(*source);
    }
    while points.len() >= 2 {
        let first = points[0];
        let last = points[points.len() - 1];
        if (last[0] - first[0]).hypot(last[1] - first[1]) <= band {
            points.pop();
            let source = sources.pop().expect("as many sources as points");
            let _ = source;
        } else {
            break;
        }
    }
    Cell { points, sources }
}

/// Cut `cell` at its reflex `vertex`: the edge arriving there is extended
/// through the interior to the first boundary edge it meets, and the polygon
/// is split along that segment. Both pieces keep every edge's source, and
/// the new edge takes the arriving edge's.
fn split_at_reflex(cell: &Cell, vertex: usize, band: f64) -> Result<(Cell, Cell), String> {
    let n = cell.points.len();
    // Rotate so the reflex vertex is index 0; the arriving edge is then n − 1.
    let points: Vec<[f64; 2]> = (0..n).map(|i| cell.points[(vertex + i) % n]).collect();
    let sources: Vec<EdgeSource> = (0..n).map(|i| cell.sources[(vertex + i) % n]).collect();
    let p0 = points[0];
    let prev = points[n - 1];
    let d = [p0[0] - prev[0], p0[1] - prev[1]];
    let length = (d[0] * d[0] + d[1] * d[1]).sqrt();
    if length <= band {
        return Err("a reflex corner with a zero-length arriving edge".into());
    }
    let d = [d[0] / length, d[1] / length];
    // The first edge the ray meets, not counting the two at the corner.
    let mut best: Option<(usize, f64, f64)> = None;
    for j in 1..(n - 1) {
        let a = points[j];
        let b = points[(j + 1) % n];
        let e = [b[0] - a[0], b[1] - a[1]];
        let denominator = d[0] * e[1] - d[1] * e[0];
        if denominator.abs() <= 1e-15 {
            continue;
        }
        let w = [a[0] - p0[0], a[1] - p0[1]];
        let t = (w[0] * e[1] - w[1] * e[0]) / denominator;
        let s = (w[0] * d[1] - w[1] * d[0]) / denominator;
        let e_length = (e[0] * e[0] + e[1] * e[1]).sqrt().max(1e-300);
        let s_band = band / e_length;
        if t <= band || s < -s_band || s > 1.0 + s_band {
            continue;
        }
        if best.map_or(true, |(_, best_t, _)| t < best_t) {
            best = Some((j, t, s.clamp(0.0, 1.0)));
        }
    }
    let Some((j, t, s)) = best else {
        return Err(format!("the extension of the edge arriving at corner {vertex} meets no boundary edge"));
    };
    let hit = [p0[0] + d[0] * t, p0[1] + d[1] * t];
    let a = points[j];
    let b = points[(j + 1) % n];
    let e_length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt().max(1e-300);
    let at_a = s * e_length <= band;
    let at_b = (1.0 - s) * e_length <= band;
    let arriving = sources[n - 1];
    // First piece: P0 … Pj (, hit), closed back to P0 along the extension.
    let mut first_points: Vec<[f64; 2]> = points[0..=j].to_vec();
    let mut first_sources: Vec<EdgeSource> = sources[0..j].to_vec();
    if at_b {
        first_points.push(b);
        first_sources.push(sources[j]);
    } else if !at_a {
        first_points.push(hit);
        first_sources.push(sources[j]);
    }
    first_sources.push(arriving);
    // Second piece: (hit,) P(j+1) … P(n−1), closed back along the extension
    // through P0, which is a straight continuation and is dropped.
    let mut second_points: Vec<[f64; 2]> = Vec::new();
    let mut second_sources: Vec<EdgeSource> = Vec::new();
    if at_a {
        second_points.push(a);
        second_sources.push(sources[j]);
    } else if !at_b {
        second_points.push(hit);
        second_sources.push(sources[j]);
    }
    for k in (j + 1)..n {
        second_points.push(points[k]);
        second_sources.push(sources[k]);
    }
    // The last source is that of P(n−1) → P0 → hit: the arriving edge's.
    *second_sources.last_mut().expect("the second piece has an edge") = arriving;
    Ok((
        Cell { points: first_points, sources: first_sources },
        Cell { points: second_points, sources: second_sources },
    ))
}

/// Split a CONVEX cell by a line, given each vertex's signed side; the two
/// halves keep their edges' sources and the edges along the line are `Hinge`.
fn split_by_line(cell: &Cell, sides: &[f64], band: f64) -> [Cell; 2] {
    let n = cell.points.len();
    let mut positive = Cell { points: Vec::new(), sources: Vec::new() };
    let mut negative = Cell { points: Vec::new(), sources: Vec::new() };
    for j in 0..n {
        let p = cell.points[j];
        let q = cell.points[(j + 1) % n];
        let (fp, fq) = (sides[j], sides[(j + 1) % n]);
        let source = cell.sources[j];
        let p_on = fp.abs() <= band;
        let q_on = fq.abs() <= band;
        // P itself, to every side it is on; an on-line point's leaving edge
        // runs along the line unless Q is on that side too.
        if fp > band || p_on {
            positive.points.push(p);
            positive.sources.push(if fq > band || q_on { source } else { EdgeSource::Hinge });
        }
        if fp < -band || p_on {
            negative.points.push(p);
            negative.sources.push(if fq < -band || q_on { source } else { EdgeSource::Hinge });
        }
        // A strict crossing: the crossing point goes to both sides.
        if (fp > band && fq < -band) || (fp < -band && fq > band) {
            let t = fp / (fp - fq);
            let x = [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
            if fp > band {
                positive.points.push(x);
                positive.sources.push(EdgeSource::Hinge);
                negative.points.push(x);
                negative.sources.push(source);
            } else {
                negative.points.push(x);
                negative.sources.push(EdgeSource::Hinge);
                positive.points.push(x);
                positive.sources.push(source);
            }
        }
    }
    [positive, negative]
}

/// A TURNED cell must move one way only. The wedge on the other side of the
/// carriers' common line — the same bounds, the carrier pair swapped — is the
/// material the cell would carry the other way; any corner it has further than
/// the band from BOTH carriers means that wedge has volume inside the cell's
/// bounds, and the tool this road builds would leave it untouched. A cell the
/// hinge split left on one side of the hinge has that hinge among its bounds,
/// so the other wedge is squeezed onto the line and reads zero.
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
/// A cell's tool is the INTERSECTION of the half-spaces its edges state, and
/// that intersection is the cell only while the cell IS the region they
/// enclose — true of every convex polygon, and of nothing reflex. Measured,
/// before the reflex cut existed, on an L pocket whose arms are
/// `x[5,15]×y[5,10]` and `x[5,10]×y[10,15]` in a `20×20×10` block: the
/// intersection of the L's six bounds is its CONVEX CORE `x[5,10]×y[5,10]`, the
/// floor pushed out through the bottom built **16F 39E 25V, 3600** where the
/// through-L is **3250**, and `validate()` was clean, the scan was clean, no
/// closed form existed downstream and nothing refused. The reflex cut is what
/// makes every cell convex; this reads each cell's corners and edge midpoints
/// against its bounds at their HOME positions so that a cell the cut left
/// reflex refuses by name instead of cutting its core.
fn cell_is_its_bounds(
    face: &FaceRecord,
    cell: &Cell,
    frame: &Frame,
    at_home: &[HalfSpace],
    scale: f64,
) -> Result<(), KernelRefusal> {
    // The cell's boundary lies ON these carriers, so the reading is an
    // equality and the band only has to absorb the residual a built body
    // carries. Every violation this is here to catch is of the order of the
    // feature's own size.
    let band = (scale * 1e-4).max(1e-6);
    let n = cell.points.len();
    let mut worst = 0.0f64;
    for i in 0..n {
        let a = cell.points[i];
        let b = cell.points[(i + 1) % n];
        for point in [a, [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]] {
            let point = frame.lift(point);
            for space in at_home {
                worst = worst.max(space.normal.dot(point) - space.offset);
            }
        }
    }
    if worst > band {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "reflex_trim",
            format!(
                "{OP}: face {}{} has a boundary point {worst:.3e} outside one of its own \
                 bounding carriers, so its trim is not the region its bounding carriers \
                 enclose — their intersection is the trim's CONVEX CORE, and the tool this \
                 road would build is that core rather than all the material the motion \
                 moves. Refusing rather than cutting some of it",
                face.id,
                named(face)
            ),
        ));
    }
    Ok(())
}


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
