//! SW — Sweep.
//!
//! # What this feature is
//!
//! The `Sweep` feature is NOT the dedicated swept-surface op — that is `SWP`
//! (`path_sweep.rs`), which drives the profile along a curve on a
//! rotation-minimizing frame. `SW` is the TRANSLATIONAL sweep: the profile keeps
//! its orientation and slides along the path, which for a polyline path is
//! exactly one oblique prism per path segment, unioned.
//!
//! # The path: a whole SKETCH, or several individually picked EDGES
//!
//! `path` is a `multiple` `["SKETCH","EDGE"]` selection, resolved by
//! `common::resolve_path_chain`:
//!   - a whole SKETCH resolves to its published chain — its longest open run, or a
//!     lone closed loop, which `translate` refuses and `pathAlign` sweeps as a
//!     capless ring (see below) — so one selection covers the entire trajectory. `SKETCH` is a real pick lane
//!     (`SelectionFilter::sketch`), so clicking the sketch's sheet in the 3D view
//!     picks the whole sketch while clicking one of its drawn segments picks that
//!     edge;
//!   - individual sketch segments (`{sketchId}:G{gid}`) and resident solid EDGES
//!     resolve one curve each, and several of them CHAIN by endpoint coincidence.
//! The FIRST pick seeds the chain in its own drawn direction and every other
//! selection is ordered and reversed to join it, so which of the rest you clicked
//! first is irrelevant — but the first one still decides which WAY the sweep runs,
//! exactly as a single picked edge always has. (A whole-sketch pick is seeded by
//! the sketch's own published chain order.) A selection that does not form ONE
//! connected run names the stragglers and fails.
//!
//! # The build: one PORTION per path segment
//!
//! Portion `i` extrudes a COPY of the profile — translated by the sum of the
//! preceding segments' CHORDS, so consecutive portions share a cap exactly even
//! when individually picked edges meet only within tolerance — along segment `i`'s
//! chord. Portion 0 is un-translated, so a single-segment path builds the very
//! same geometry this feature always did (its faces are named per segment now —
//! see below — with the old spelling kept as their container). The union of the portions IS the exact
//! translational sweep of the profile along the polyline (the swept volume of a
//! rigid body along a polyline is the union of its per-segment sweeps), holes
//! included: each portion subtracts its own hole prisms BEFORE the union, which
//! is what carries a hole channel around a corner.
//!
//! Every segment must be STRAIGHT (degree-1, two control points), must not run
//! (nearly) inside the profile plane (the builder refuses a direction within ~6°
//! of it — `|n̂·d̂| < 0.1`), and must ADVANCE THROUGH THE PROFILE THE SAME WAY as the first — the
//! profile does not rotate, so a segment that reverses relative to it drives the
//! profile back through material already swept. Every refusal names the
//! offending segment.
//!
//! A CLOSED path is refused under `translate` BY ITS CLOSURE, not by the segment
//! that happens to reverse: the path classification
//! (`common::resolve_sweep_path`) says the run is a ring, and the refusal points
//! at `pathAlign`, which builds one. Going round a loop without rotating the
//! profile cannot work — the advances must sum to zero — so this is a real
//! boundary between the modes rather than a missing port.
//!
//! That one rule is also what makes the cap naming exact: with all segments
//! advancing the same way, portion `i` occupies `[s_i, s_{i+1}]` through the
//! profile, so its far cap and portion `i+1`'s near cap are coincident and
//! ANTI-parallel and annihilate in the union. Only the chain's own two ends
//! survive, and they survive as themselves.
//!
//! # Headless profile + path contract
//!
//! Both inputs resolve FROM THE SCENE by name, HEADLESS (the `extrude.rs` /
//! `revolve.rs` template):
//!   - `profile` names a SKETCH whose profile the SKETCH feature already extracted
//!     (`SceneMap::resolve_profile`), OR a resident solid FACE — resolved via
//!     `scene.resolve_face` → `face_profile::face_profile` (the extrude.rs /
//!     revolve.rs two-step; `from_face` switches the cap base to the FACE name
//!     and skips sketch consumption). Sweep reads
//!     each region's OUTER loop curves + carried `{sketchId}:G{gid}` edge names —
//!     no marshaled curves, no caller round-trip. Region holes subtract as the same
//!     over-long through prisms extrude uses (`common::subtract_region_holes`);
//!     region solids union into ONE result.
//!   - `path` names the trajectory, as described above.
//!
//! # Name fidelity — a face per (path segment × profile edge)
//!
//! `extrude_profile_brep` emits faces UNNAMED, in order `[side faces (INPUT curve
//! order), bottom cap (= profile base / START), top cap (= far end / END)]`. Each
//! portion stamps that order:
//!   - side `i`  → `${tag}${edgeName[i]}:${segment}_SW` (the carried
//!                 `{sketchId}:G{gid}` profile edge name, else `EDGE_${i}`, joined
//!                 to the PATH SEGMENT's own name), with the un-keyed
//!                 `${tag}${edgeName[i]}_SW` registered as the CONTAINER standing
//!                 for that profile edge's wall along every segment. So a
//!                 single-segment sweep's reference still resolves exactly, and a
//!                 reference into a multi-segment sweep names both the profile edge
//!                 and the path segment it belongs to — neither moves when the
//!                 OTHER end of the path is edited.
//!   - START cap → `${tag}${faceName}:L{loopId}_START` (portion 0)
//!   - END cap   → `${tag}${faceName}:L{loopId}_END`   (the last portion)
//!     (one pair PER REGION, keyed by the region's outer loop's stable identity —
//!     see the sketch feature's `loop_ids`; the un-keyed `..._START/_END` is
//!     registered as the CONTAINER standing for all of them. A profile with no
//!     loop identity behind it — a FACE profile — keeps the un-keyed spelling.)
//!     Interior caps are keyed by segment too (`common::CapNames::interior`); the
//!     fold-back rule above guarantees they annihilate, so they never reach the
//!     result — the keyed spelling is what they would carry if one ever did.
//!   - hole walls → `${id}:HOLE:{loop key}:${segment}`, with `${id}:HOLE:{loop key}`
//!     as their container — the same per-segment treatment as the sidewalls.
//!   - a RING (a closed path under `pathAlign`) has NO caps, so it names none and
//!     registers neither cap name nor cap container: a container standing for a
//!     face that does not exist is a name a later feature can select and nothing
//!     can resolve. Its walls take the same un-keyed container spelling every
//!     `pathAlign` wall does.
//! where `tag` = `""` for an empty id, else `${id}:`,
//! and `faceName` is the sketch profile face name `{sketchId}:PROFILE`.
//!
//! The portion union PINS `_SW` and `:HOLE:` faces out of the coplanar merge
//! (`common::union_solids_keeping`). Without it, two portions' coplanar walls —
//! guaranteed for collinear segments, common either side of a turn — fuse into one
//! face carrying one of the two names, and WHICH one depends on the boolean's
//! internal face order. Pinning is what makes the per-segment names mean something
//! stable; the cost is a seam edge where two coplanar portions meet.
//!
//! The pin covers THIS feature's own unions (portions, then regions). The user's
//! `boolean` runs through `finalize_solid_grouped` un-pinned, like every other
//! feature's, so folding a multi-segment sweep into a target can still merge
//! portion walls that end up coplanar with each other or with the target.
//!
//! # The two orientation modes, and which path belongs to which
//!
//! | the path is | mode | what it builds |
//! |---|---|---|
//! | cornered (straight segments meeting at an angle) | `translate` | one oblique prism per segment, unioned; the corner is handled; per-SEGMENT wall names |
//! | cornered, and the profile must FOLLOW it | `pathAlign` | one MITRE per joint: the section swept along each segment, both sweeps trimmed to the joint's bisector plane and sharing ONE face loop there — inner side folded, outer side extended, no bulkhead; per-SEGMENT wall names |
//! | smooth — curved, or segments meeting tangentially | `pathAlign` | ONE tube: the profile is CARRIED by the path's own rigid motion, keeping the position and angle it was drawn at; per-PROFILE-EDGE wall names |
//! | smooth AND CLOSED, and PLANAR | `pathAlign` | ONE RING: the same carry, all the way round and back to the section it started as — no end caps, `V − E + F = 0` |
//! | cornered AND CLOSED, and PLANAR | `pathAlign` | a picture FRAME: `n` mitres for `n` segments, the closing joint mitred like the rest, and NO caps — `V − E + F = 0`; per-SEGMENT wall names |
//! | cornered at a CURVED segment | neither | unbuilt (no single direction for a bisector plane to bisect, and no lane that mixes mitring with skinning); refused by name |
//! | smooth AND CLOSED, and SPATIAL | `pathAlign` | ONE RING, closed by a COUNTER-TWIST: carried once round, the section comes back rolled by the path's HOLONOMY (the solid angle its tangent indicatrix encloses, mod 2π), and minus that angle is laid down linearly in arc length so the ring meets itself (`sweep_closure` reports both) |
//! | cornered AND CLOSED, and SPATIAL | `pathAlign` | a FRAME closed the same way: the counter-twist is shared over the sides by length, each share rolled over the middle half of its side as an exact twisted wall, so the mitres stay exact and there is still one wall per (segment × profile edge); a loop whose holonomy is zero (a mirror-symmetric one) is a plain frame |
//!
//! The gate between the two LANES of `pathAlign` is the JOINT ANGLE, not
//! curvature: a joint inside the tangent-break band is skinned as one tube, and
//! one past it is a corner — mitred when both its segments are straight,
//! refused when either is curved. `translate` refuses a CURVED segment for the
//! mirror reason — a chord does not follow a curve. So the two modes now differ
//! on a cornered polyline only in WHERE the section sits: `translate` slides it
//! without rotating, `pathAlign` turns it through each joint.
//!
//! # What `pathAlign` means, exactly
//!
//! The section at path parameter `t` is the profile moved by THE SAME RIGID MOTION
//! THE PATH UNDERGOES between its start and `t`:
//!
//! ```text
//!     section(t) = P(t) + R(t) · (x − P(0))
//! ```
//!
//! `R(t)` is the rotation carrying the path's start tangent frame to its frame at
//! `t` (`SectionPlacement::Rigid` in the builder). So the profile's POSITION
//! relative to the path and its ANGLE to the path are the drawn ones, and the only
//! thing that changes along the sweep is what the path itself does. Two readings
//! fall out, and they are the reason the mode exists:
//!   - a STRAIGHT path has `R ≡ I`, so the profile translates along the segment's
//!     vector for the segment's length — the oblique prism `translate` builds from
//!     the same inputs, exactly, not merely to within the station sampling. The
//!     modes agree on a straight path rather than merely agreeing in volume;
//!   - a circular ARC has `R` = the rotation about the ARC'S OWN CENTRE AXIS, so
//!     the profile is carried round that pivot and the end cap keeps the angle to
//!     the tangent the start cap had. A full circle is exactly a revolve;
//!   - a HELIX has `R` = the SCREW about its own axis, so the profile keeps its
//!     place relative to the coil's axis and a whole number of turns lands the end
//!     cap on the start cap lifted by the height. The helix's axis comes WITH the
//!     path — the HX feature publishes it beside its edge, and
//!     `common::resolve_sweep_path` carries it onto `SweepPath::screw_axes` — and
//!     without it a helical curve is framed like any other curve, by the
//!     rotation-minimizing frame, which rolls away from the screw by the helix's
//!     integrated torsion. That roll was the hosted-app report of 2026-09-15 ("Sweep
//!     seems to rotate the profile in an unexpected way as it follows the path").
//! The START cap therefore lies in the sketch plane. The path need not start on
//! the profile, or touch it: an offset profile sweeps the ring its offset traces.
//!
//! This is NOT what `SWP` (path_sweep.rs) does with the same builder: `SWP`
//! TRANSPLANTS the profile onto the path (centroid on the path, plane square to
//! it), discarding both the offset and the angle. The two features build the same
//! solid only for a profile drawn centred on and square to the path start.
//!
//! NAMING, `pathAlign`: one tube means exactly one wall per profile edge, so a
//! wall takes the un-keyed CONTAINER spelling `{tag}{profileEdge}_SW` rather than
//! the per-segment `{tag}{profileEdge}:{segment}_SW`. Nothing positional is in
//! either: both key on the carried `{sketchId}:G{gid}` name. Because `translate`
//! already registers that un-keyed string as the container standing for that
//! edge's walls across every segment, a reference stored under it resolves under
//! BOTH modes — flipping `orientationMode` does not break a downstream reference.
//! Caps keep the loop-keyed spelling, and one portion has no interior caps.
//!
//! # Deferred paths (clear errors, never a silent geometry drop)
//!
//! - UNDER `translate`, a CURVED path segment — the loft-through-transformed-copies
//!   branch — is not yet migrated; the error names the segment and points at
//!   `pathAlign`. What is deferred is that LANE, not the path: `pathAlign` sweeps a
//!   curved path TODAY, as one tube, and skips the straight-segment loop below
//!   outright (`filter(|_| !path_align)`), so the refusal is never reached in that
//!   mode.
//! - a path that DOUBLES BACK through the profile, under `translate` — sweep the
//!   run one direction at a time. A CLOSED path is the special case of it that
//!   `pathAlign` builds instead (see the mode table), and its refusal says so.
//! - `orientationMode: "pathAlign"` on a cornered path with a CURVED segment.
//!   An all-straight cornered polyline is MITRED (see the mode table); a corner
//!   at a curve is not, and the reason is the mitre's own construction — it trims
//!   both sweeps to the plane that BISECTS their directions, and a curve has no
//!   single direction to bisect. Refused by name, saying which segment is curved.
//! - a CLOSED cornered polyline which CROSSES ITSELF: it would build a frame that
//!   passes through itself, and the refusal names the two sides. A SPATIAL one is
//!   built (see the mode table), and refused only where its counter-twist has no
//!   room: a side whose mitres reach into the middle half its share rolls over,
//!   or a section reaching so far from the path that the roll would tilt a wall
//!   past the 0.1 transversality floor. Both quote their measurement.
//! - a mitre with no ROOM for it: a bend too TIGHT for the section across it (the
//!   inner fold would reach past the far end of a segment, so the bisector plane
//!   cuts the section a second time and the inner side self-intersects), and a
//!   COLLAPSED bend at or past the 168.522° fold angle. Both quote the measured
//!   quantity — the surviving wall length against the segment's own, or the angle
//!   against the bar.
//! - a non-zero twist on a cornered path: a twist is laid down over the sampled
//!   stations, a mitre has none, and rolling the section between two exact pieces
//!   would pull apart the shared loop that IS the construction.
//! - a non-zero `twistAngle` (a no-op upstream, but a geometry-shaped param we refuse to
//!   silently ignore) is not yet migrated.

use crate::extrude_profile_brep;
use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult, SketchProfile};
use crate::{KernelStage, OrRefuse};
use crate::Vec3;

/// Face-name substrings PINNED out of the portion union's coplanar merge — every
/// face this feature names per PATH SEGMENT (see the module header).
const KEEP_UNMERGED: [&str; 2] = ["_SW", ":HOLE:"];

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// One straight path segment, reduced to what the build needs: its source NAME,
/// the OFFSET the profile copy is translated by (the sum of the preceding
/// segments' chords), and the chord itself.
struct Step {
    name: String,
    offset: Vec3,
    direction: Vec3,
    distance: f64,
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    // ---- Profile: resolve the referenced sketch's extracted profile, else a
    // resident FACE via `face_profile` (the extrude.rs/revolve.rs two-step).
    // `from_face` gates the cap-name base and skips the consumed-sketch
    // bookkeeping.
    // Normalize a `{sketch}:FACE` display-sheet pick to the sketch profile base.
    let profile_name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("sweep: missing `profile` reference selection")?,
    );
    let mut face_profile_storage: Option<SketchProfile> = None;
    let (profile, from_face): (&SketchProfile, bool) =
        match ctx.scene.resolve_profile(&profile_name) {
            Some(profile) => (profile, false),
            None => match ctx.scene.resolve_face(&profile_name) {
                Some(face) => {
                    let built = super::face_profile::face_profile(face)
                        .map_err(|error| format!("sweep: {error}"))?;
                    (&*face_profile_storage.insert(built), true)
                }
                None => {
                    return Err(format!(
                        "sweep: profile '{profile_name}' not found (no sketch profile or resident face)"
                    ).into());
                }
            },
        };

    // ---- Path: a whole sketch chain and/or individually picked edges, ordered
    // head-to-tail into ONE connected run. A missing selection is a hard error.
    let path_names = common::reference_names(ctx.param("path"));
    if path_names.is_empty() {
        return Err("sweep: requires a path edge selection".into());
    }
    // The resolved chain AND its classification — closure, the plane it lies in if
    // it lies in one, and every joint's continuity — computed once here and read
    // by the refusals below and by the builder (`common::resolve_sweep_path`).
    let path = common::resolve_sweep_path(ctx, &path_names)
        .map_err(|error| format!("sweep: {error}"))?;

    // ---- Orientation mode, read BEFORE the straight-segment loop below.
    // ORDER MATTERS: that loop refuses a CURVED segment, and a curved segment is
    // exactly what `pathAlign` exists to sweep. Reading the mode afterwards (as
    // this did) meant a curved path was refused before orientation was ever
    // consulted, so the mode could never take effect on the paths that need it.
    let path_align = ctx
        .param("orientationMode")
        .and_then(|value| value.as_str())
        .map(|mode| mode.trim().eq_ignore_ascii_case("pathAlign"))
        .unwrap_or(false);

    // Each segment reduces to `(offset, chord)`. The offset accumulates the
    // CHORDS, not the resolved endpoints, so portion `i`'s far cap and portion
    // `i+1`'s near cap are the same translated profile exactly — individually
    // picked solid edges that meet only within tolerance would otherwise leave a
    // sliver between the two.
    //
    // TRANSLATE ONLY: `pathAlign` sweeps the whole run as one tube and never
    // reduces a segment to a chord, so it neither needs nor may run this loop.
    let mut steps: Vec<Step> = Vec::with_capacity(path.len());
    let mut offset = Vec3::default();
    if !path_align {
        // A CLOSED path, named as such. `translate` cannot sweep one and never
        // could: the profile does not rotate, so the portions' advances through it
        // must sum to zero and some segment has to reverse. The fold-back rule
        // below catches that consequence — but it reports the REVERSING SEGMENT,
        // which tells a user who drew a loop nothing about the loop. The
        // classification knows the path is a ring, so this says so, and points at
        // the mode that now builds one.
        if path.closed {
            return Err(
                "sweep: this path is CLOSED (a ring), and `orientationMode: translate` slides \
                 the profile along each segment's chord without rotating it — so going round a \
                 loop it must drive the profile back through material it has already swept. \
                 Set `orientationMode: pathAlign` to sweep a closed PLANAR path, which builds \
                 the ring as one capless body, or sweep the run one direction at a time."
                    .into(),
            );
        }
        for index in 0..path.len() {
            let curve = &path.curves[index];
            // STRAIGHT segments only: a degree-1 two-control-point curve. Anything
            // else is the curved-sweep (loft) branch, deferred loudly BY NAME.
            if curve.degree != 1 || curve.control_points.len() != 2 {
                return Err(format!(
                    "sweep: path segment '{}' is curved, and `orientationMode: translate` slides \
                     the profile along each segment's CHORD, which would not follow the curve. \
                     Set `orientationMode: pathAlign` to sweep this path — the profile then \
                     rotates to follow it. (Path Sweep, SWP, builds the same shape as a \
                     separate feature.)",
                    path.name(index)
                ).into());
            }
            let [t0, t1] = curve.domain()?;
            let direction = curve.evaluate(t1)?.sub(curve.evaluate(t0)?);
            let distance = direction.length();
            if distance <= 1e-12 {
                return Err(format!(
                    "sweep: path segment '{}' has zero length",
                    path.name(index)
                ).into());
            }
            steps.push(Step {
                name: path.name(index).to_string(),
                offset,
                direction,
                distance,
            });
            offset = offset.add(direction);
        }
    }

    // A segment lying exactly IN the profile plane sweeps nothing and leaves the
    // fold-back guard below with no sense to compare against. (The builder owns
    // the real threshold — it refuses anything within ~84° of the plane — but it
    // runs after that guard, so the degenerate case is named here first.)
    for step in &steps {
        if step.direction.dot(profile.z_axis).abs() <= 1e-9 * step.distance {
            return Err(format!(
                "sweep: path segment '{}' runs inside the profile plane — it would sweep the \
                 profile through itself rather than along the path",
                step.name
            ).into());
        }
    }

    // ---- Fold-back guard. Every segment must advance through the profile in the
    // SAME sense. The profile does not rotate here, so a segment that reverses
    // relative to the profile normal drives the profile back through material it
    // has already swept: the result self-intersects, and the two portions either
    // side of the reversal share a cap they now face the SAME way, so the caps
    // MERGE instead of cancelling and the surviving face's name falls to the
    // boolean's face order.
    //
    // A CLOSED path always reverses somewhere (its segments' advances must sum to
    // zero), and this rule is what used to refuse one — reporting the reversing
    // segment. The closure refusal above now speaks first and says what the path
    // IS; what is left here is the OPEN chain that drifts back on itself, which is
    // the only case this rule was ever the right reporter for.
    //
    // The profile's own plane normal (`SketchProfile::z_axis`, published by the
    // sketch and by `face_profile` alike) fixes the reference; the sign comes from
    // the FIRST segment and every later segment is compared against it.
    //
    // Comparing to the FIRST rather than to the PREDECESSOR is right HERE, and
    // that is worth saying because the loft's winding check had to go the other
    // way (a section's winding is compared to its predecessor, since past a 90°
    // bend the frame has rotated away from section 0). The difference: the loft's
    // reference frame MOVES along the path, this one does not. The profile never
    // rotates in a translational sweep, so `z_axis` is a global reference and
    // `sign(dᵢ·n) == sign(d₀·n)` is transitive — predecessor and first agree, and
    // only the global form catches a chain that drifts back a segment at a time.
    if steps.len() > 1 {
        let normal = profile.z_axis;
        let sense = steps[0].direction.dot(normal).signum();
        for step in &steps[1..] {
            if step.direction.dot(normal) * sense <= 0.0 {
                return Err(format!(
                    "sweep: path segment '{}' doubles back through the profile — this sweep \
                     TRANSLATES the profile without rotating it, so a segment that reverses \
                     relative to the profile drives it back through material already swept. \
                     Every path segment must advance through the profile the same way. Sweep \
                     the run one direction at a time.",
                    step.name
                ).into());
            }
        }
    }

    // ---- No orientation guard: `pathAlign` is BUILT on every path shape this
    // feature accepts. A smooth run is one tube on a single rotation-minimizing
    // frame; a cornered POLYLINE is MITRED, one exact piece per segment joined on
    // each joint's bisector plane. `sweep_profile_along_chain` owns both lanes and
    // picks between them off the path's own classification — a CLOSED polyline is
    // the same mitre lane with no caps, a picture frame — and it refuses what
    // neither covers (a corner at a curved segment, a spatial or self-crossing
    // frame loop, a bend with no room for its mitre) by name. Nothing to gate
    // here.

    // ---- Twist guard: a non-zero twist is a no-op we refuse to silently honor.
    // Non-finite / absent / zero → treated as 0 (`Number(twist)` NaN-guard).
    if let Some(value) = ctx.param("twistAngle") {
        if !value.is_null() {
            if let Ok(angle) = ctx.number("twistAngle") {
                if angle.is_finite() && angle.abs() > 1e-12 {
                    return Err(
                        "sweep: non-zero `twistAngle` is not yet migrated to the Rust pipeline \
                         (a no-op in the previous kernel)"
                            .into(),
                    );
                }
            }
        }
    }

    // ---- Build: one PORTION per (region × path segment), unioned -------------
    let tag = normalize_feature_tag(&ctx.id);
    // Cap base (the source-profile base name): the sketch profile face name
    // `{sketchId}:PROFILE` for a sketch, the FACE name verbatim for a face
    // profile (the `:PROFILE` branch strips-then-reappends, a no-op).
    let base = if from_face {
        profile_base_name(&tag, &profile_name)
    } else {
        let sketch_id = profile_name
            .strip_suffix(":PROFILE")
            .unwrap_or(&profile_name);
        profile_base_name(&tag, &format!("{sketch_id}:PROFILE"))
    };

    // Per-loop cap names keyed off each region's STABLE loop identity, with the
    // un-keyed `{base}_START/_END` as the containers standing for all of them
    // (see `common::CapNames`).
    let caps = common::CapNames::new(&base, &profile.regions);
    let pinned: Vec<String> = KEEP_UNMERGED.iter().map(|s| s.to_string()).collect();
    // A CLOSED path sweeps a capless RING — no start cap, no end cap, and so no
    // cap name and no cap container. Registering the names anyway would publish
    // container entries standing for faces that do not exist, which is a name a
    // later feature can select and nothing can resolve.
    let ring = path.closed;
    // A CORNERED POLYLINE under `pathAlign` is MITRED: one exact piece per
    // segment, joined on each joint's bisector plane, so the build emits a wall
    // per (segment × profile edge) and names them per segment. The ONE predicate
    // the builder selects the lane on is the one read here
    // (`SweepPath::cornered_polyline`), so the names and the geometry cannot
    // disagree about which lane ran.
    let mitred = path_align && path.cornered_polyline();
    // The path segments a WALL NAME is keyed by: every segment under
    // `translate`, every segment under a MITRED `pathAlign`, and none under the
    // smooth one-tube lane (where the un-keyed container IS the wall's name).
    let wall_segments: Vec<String> = if path_align {
        if mitred {
            (0..path.len()).map(|index| path.name(index).to_string()).collect()
        } else {
            Vec::new()
        }
    } else {
        steps.iter().map(|step| step.name.clone()).collect()
    };
    // Every cap name the build stamps, so the role pass marks an interior cap a
    // cap in the rare case one survives the union.
    let mut start_names = if ring { Vec::new() } else { caps.starts() };
    let mut end_names = if ring { Vec::new() } else { caps.ends() };
    // `container -> members` for the per-segment sidewall and hole-wall names.
    let mut containers = if ring { Vec::new() } else { caps.containers() };

    // `pathAlign` sweeps the run as ONE tube per region — or as one RING when the
    // path is closed. It needs NO placement anchor: `SectionPlacement::Rigid`
    // carries every loop from where it was drawn, so a hole's offset from its
    // outer loop, and a later region's offset from the first, are preserved by
    // the construction itself rather than by lending one loop's frame to the
    // rest. (That anchor exists for `Transplant`, which moves each loop onto the
    // path and would otherwise re-centre every one of them. SWP still needs it.)

    let mut region_solids = Vec::with_capacity(profile.regions.len());

    // The station lane's measured bounds, one per swept loop (`sweep.stations`).

    let mut approximations: Vec<crate::Approximation> = Vec::new();
    // Which regions the swept ENVELOPE lane built. Past `ρκ = 1` the sections
    // cross and the kernel answers with a REVOLVE — the section disc truncated
    // at the path's own axis — whose face set is nothing like the skinned tube's:
    // ONE revolved wall carrying every section edge, plus the usual two caps on
    // an open path. Read from the same recognition the builder selects the lane
    // on, so the names and the geometry cannot disagree about which one ran.
    let mut envelope_regions = vec![false; profile.regions.len()];
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region.first().ok_or("sweep: profile has no outer loop")?;
        if outer.curves.len() < 2 {
            return Err(format!(
                "sweep: outer loop needs >= 2 curves, got {}",
                outer.curves.len()
            ).into());
        }
        let side_count = outer.curves.len();

        if path_align {
            let envelope = matches!(
                crate::recognize_swept_envelope(
                    &outer.curves,
                    &path,
                    crate::SectionPlacement::Rigid,
                ),
                Ok(Ok(_))
            );
            envelope_regions[region_index] = envelope;
            if envelope && region.len() > 1 {
                return Err(format!(
                    "sweep: region {} bends TIGHTER than its own section, so its swept solid is \
                     the envelope trimmed at its own self-intersection — a revolve, not a skinned \
                     tube — and a HOLE loop through one is not built: the hole's own envelope \
                     would have to be trimmed against the outer one, which nothing solves. Sweep \
                     the outer loop alone, or ease the bend",
                    region_index + 1
                ).into());
            }
            // ONE tube for the whole run, every loop carried by the PATH'S OWN
            // rigid motion from where it was drawn (`SectionPlacement::Rigid`).
            // The station lane's MEASURED BOUND rides into the result's
            // `approximations` (`sweep.stations`); the mitre and envelope lanes
            // are exact and report none.
            let swept = crate::sweep_profile_along_chain_reported(
                &outer.curves,
                &path,
                0.0,
                Some(&ctx.id),
                None,
                crate::SectionPlacement::Rigid,
                "A corner between two STRAIGHT segments is MITRED under `pathAlign` (both sweeps trimmed to the joint's bisector plane, sharing one loop there), and `orientationMode: translate` slides the profile along each straight segment's chord without rotating it — but neither builds a corner at a CURVED segment, because a curve has no single direction for a bisector plane to bisect. Split the path at that corner and sweep each run separately, or round the corner so the joint is tangent-continuous.",
                None,
            )
            .map_err(|error| format!("sweep: {error}"))?;
            let mut solid = swept.solid;
            if let Some(report) = swept.report {
                approximations.push(report.approximation(sweep_body_name(&ctx.id)));
            }

            // NAMES. One tube means exactly ONE wall per profile edge, so a wall
            // takes the CONTAINER spelling — the un-keyed `{tag}{profileEdge}_SW`
            // that already stands for "that profile edge's wall along every path
            // segment". Nothing positional survives: the name is keyed on the
            // carried `{sketchId}:G{gid}` profile-edge name, and no path segment
            // appears in it because there is no per-segment wall to name. The
            // payoff is compatibility — a reference stored under the container
            // resolves under BOTH modes, so flipping `orientationMode` does not
            // break a downstream reference. Caps keep the loop-keyed spelling,
            // and with one portion there are no interior caps at all.
            //
            // A MITRED path is the exception, and it takes `translate`'s
            // spelling: a cornered polyline is built as one exact piece per
            // segment joined on each joint's bisector plane, so there IS a wall
            // per (path segment × profile edge), in that order, and naming them
            // all after the profile edge alone would hand several faces one
            // name. The per-segment names are the same
            // `{tag}{profileEdge}:{segment}_SW` strings `translate` stamps, under
            // the same un-keyed container — so a reference into a mitred
            // `pathAlign` sweep and one into a `translate` sweep of the same path
            // resolve identically, and the container resolves under the smooth
            // lane too.
            let mut face_names: Vec<String> = Vec::with_capacity(side_count + 2);
            if envelope {
                // A section CROSSING the axis leaves ONE envelope face, the image
                // of the WHOLE section boundary — the near arc runs across both
                // of a circle's half-arcs — so no per-edge wall exists to name.
                // It takes the first section edge's container spelling, and
                // every other edge's container lists that one name as its member
                // below. A section merely TANGENT to the axis leaves one revolved
                // wall per profile arc, and each takes its own edge's spelling.
                let caps_here = if ring { 0 } else { 2 };
                let walls = solid
                    .shells
                    .first()
                    .map_or(0, |shell| shell.faces.len())
                    .saturating_sub(caps_here);
                if walls == 1 {
                    face_names.push(wall_container_name(&tag, &outer.edge_names, 0));
                } else {
                    envelope_regions[region_index] = false;
                    for index in 0..side_count {
                        face_names.push(wall_container_name(&tag, &outer.edge_names, index));
                    }
                }
            } else if mitred {
                for segment in 0..path.len() {
                    for index in 0..side_count {
                        face_names.push(side_face_name(
                            &tag,
                            &outer.edge_names,
                            index,
                            path.name(segment),
                        ));
                    }
                }
            } else {
                for index in 0..side_count {
                    face_names.push(wall_container_name(&tag, &outer.edge_names, index));
                }
            }
            // A RING's faces are its walls and nothing else: the closed loft
            // emits one face per profile edge, in the same INPUT-curve order the
            // open loft puts its walls in, and no caps.
            if !ring {
                let (chain_start, chain_end) = &caps.per_loop[region_index];
                face_names.push(chain_start.clone());
                face_names.push(chain_end.clone());
            }

            let faces = &mut solid
                .shells
                .get_mut(0)
                .ok_or("sweep builder produced no shell")?
                .faces;
            if faces.len() != face_names.len() {
                return Err(format!(
                    "sweep builder produced {} faces, expected {} ({} {} + {})",
                    faces.len(),
                    face_names.len(),
                    if mitred {
                        format!("{} x {side_count}", path.len())
                    } else {
                        side_count.to_string()
                    },
                    if mitred { "mitred walls" } else { "sides" },
                    if ring { "no caps (a closed path)" } else { "2 caps" }
                ).into());
            }
            for (face, name) in faces.iter_mut().zip(&face_names) {
                face.name = Some(name.clone());
            }

            // Hole loops sweep the SAME run and subtract — a translated
            // through-prism would not follow the curve. Under `Rigid` the hole
            // rides the outer wall by construction: both are carried by the same
            // path motion from their own drawn positions, so the channel stays
            // where the sketch put it without an anchor to tie them together.
            if region.len() > 1 {
                solid = common::subtract_region_holes(
                    solid,
                    profile,
                    region,
                    "sweep",
                    &ctx.id,
                    None,
                    &mut |loop_index, _depth| {
                        crate::sweep_profile_along_chain_reported(
                            &region[loop_index].curves,
                            &path,
                            0.0,
                            None,
                            None,
                            crate::SectionPlacement::Rigid,
                            "A corner between two STRAIGHT segments is MITRED under `pathAlign` (both sweeps trimmed to the joint's bisector plane, sharing one loop there), and `orientationMode: translate` slides the profile along each straight segment's chord without rotating it — but neither builds a corner at a CURVED segment, because a curve has no single direction for a bisector plane to bisect. Split the path at that corner and sweep each run separately, or round the corner so the joint is tangent-continuous.",
                            None,
                        )
                        .map(|swept| {
                            if let Some(report) = swept.report {
                                approximations.push(report.approximation(format!(
                                    "{} hole {loop_index}",
                                    sweep_body_name(&ctx.id)
                                )));
                            }
                            swept.solid
                        })
                    },
                )?;
            }
            region_solids.push(solid);
            continue;
        }

        let mut portions = Vec::with_capacity(steps.len());
        for (step_index, step) in steps.iter().enumerate() {
            // The profile copy this segment sweeps.
            let outer_curves = common::translate_curves(&outer.curves, step.offset)?;
            let mut solid = extrude_profile_brep(&outer_curves, step.direction, step.distance)
                .map_err(|error| {
                    format!("sweep: path segment '{}': {error}", step.name)
                })?;

            // Face order (sweep_topology.rs): [sidewalls in INPUT order..., bottom, top].
            let mut face_names: Vec<String> = Vec::with_capacity(side_count + 2);
            for index in 0..side_count {
                face_names.push(side_face_name(&tag, &outer.edge_names, index, &step.name));
            }
            // The chain's own two ends keep the plain per-loop cap spelling;
            // every interior cap is keyed by its segment (and cancels in the
            // union — see `CapNames::interior`).
            let (interior_start, interior_end) = caps.interior(region_index, &step.name);
            let (chain_start, chain_end) = &caps.per_loop[region_index];
            let start = if step_index == 0 {
                chain_start.clone()
            } else {
                start_names.push(interior_start.clone());
                interior_start
            };
            let end = if step_index + 1 == steps.len() {
                chain_end.clone()
            } else {
                end_names.push(interior_end.clone());
                interior_end
            };
            face_names.push(start);
            face_names.push(end);

            let faces = &mut solid
                .shells
                .get_mut(0)
                .ok_or("sweep builder produced no shell")?
                .faces;
            if faces.len() != face_names.len() {
                return Err(format!(
                    "sweep builder produced {} faces, expected {} ({} sides + 2 caps)",
                    faces.len(),
                    face_names.len(),
                    side_count
                ).into());
            }
            for (face, name) in faces.iter_mut().zip(&face_names) {
                face.name = Some(name.clone());
            }

            // Region holes: this portion IS an extrude along its segment's chord,
            // so each hole loop subtracts as the same over-long through prism
            // extrude uses — cut PER PORTION, before the union, which is what
            // carries the hole channel around a corner. The prism margin grows
            // with nesting depth so a nested cutter strictly overhangs its
            // parent's caps.
            if region.len() > 1 {
                solid = common::subtract_region_holes(
                    solid,
                    profile,
                    region,
                    "sweep",
                    &ctx.id,
                    Some(step.name.as_str()),
                    &mut |loop_index, depth| {
                        let hole_curves =
                            common::translate_curves(&region[loop_index].curves, step.offset)
                                .or_refuse(KernelStage::Collect, "translate_curves")?;
                        common::hole_prism(&hole_curves, step.direction, step.distance, depth)
                    },
                )?;
            }
            portions.push(solid);
        }

        // The portions of ONE region union into that region's solid, keeping each
        // portion's own walls addressable (see `KEEP_UNMERGED`).
        region_solids.push(
            common::union_solids_keeping(portions, &pinned)
                .map_err(|error| format!("sweep: {error}"))?,
        );
    }

    // Sidewall + hole-wall containers: the un-keyed name stands for that profile
    // edge's (or hole loop's) wall along EVERY path segment.
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = &region[0];
        for index in 0..outer.curves.len() {
            let container = wall_container_name(&tag, &outer.edge_names, index);
            let members = if envelope_regions[region_index] {
                // One revolved face for every section edge: edge 0's container
                // IS its name, and the rest name it as their member.
                if index == 0 {
                    Vec::new()
                } else {
                    vec![wall_container_name(&tag, &outer.edge_names, 0)]
                }
            } else {
                wall_segments
                    .iter()
                    .map(|segment| side_face_name(&tag, &outer.edge_names, index, segment))
                    .collect()
            };
            containers.push((container, members));
        }
        for loop_index in 1..region.len() {
            let key = common::hole_key(&region[loop_index], loop_index);
            let container = common::hole_face_name(&ctx.id, &key, None);
            // `pathAlign` cuts each hole ONCE along the whole run (mitred or
            // smooth), so its cutter's faces all carry the un-keyed name and the
            // container stands for itself; only `translate` cuts per segment.
            let members = if path_align {
                Vec::new()
            } else {
                steps
                    .iter()
                    .map(|step| common::hole_face_name(&ctx.id, &key, Some(&step.name)))
                    .collect()
            };
            containers.push((container, members));
        }
    }

    let solid = common::union_solids_keeping(region_solids, &pinned)
        .map_err(|error| format!("sweep: {error}"))?;

    // Role metadata: sidewall/cap convention, from the actual named faces.
    common::stamp_sweep_roles_multi(&solid, "_SW", &start_names, &end_names);

    // Solid name: `featureID || sweeps[0].name` → `${id}` or "Sweep".
    let base_name = sweep_body_name(&ctx.id);
    let mut result = common::finalize_solid_grouped(ctx, solid, base_name, &containers);
    if result.error.is_some() {
        return Ok(result);
    }
    result.approximations.extend(approximations);
    // Consume the referenced profile sketch (default true). A FACE profile has
    // no sketch to consume — the source solid stays resident.
    if !from_face {
        common::consume_sketch(ctx, &profile_name, &mut result);
    }
    Ok(result)
}

/// Solid name: `featureID || sweeps[0].name` → `${id}` or "Sweep".
fn sweep_body_name(id: &str) -> &str {
    if id.is_empty() {
        "Sweep"
    } else {
        id
    }
}

/// Feature tag: trim; empty → `""`; already `:`-ended →
/// as-is; otherwise append `:`.
fn normalize_feature_tag(id: &str) -> String {
    let tag = id.trim();
    if tag.is_empty() {
        String::new()
    } else if tag.ends_with(':') {
        tag.to_string()
    } else {
        format!("{tag}:")
    }
}

/// The profile edge's own name token: the carried source edge name when present +
/// non-empty, else the INDEX-based default `EDGE_${i}`. Trimmed.
fn profile_edge_token(edge_names: &[Option<String>], index: usize) -> String {
    match edge_names.get(index) {
        Some(Some(name)) if !name.trim().is_empty() => name.trim().to_string(),
        _ => format!("EDGE_{index}"),
    }
}

/// One portion's side face name: the profile edge joined to the PATH SEGMENT that
/// swept it, `${tag}${edge}:${segment}_SW`.
fn side_face_name(
    tag: &str,
    edge_names: &[Option<String>],
    index: usize,
    segment: &str,
) -> String {
    format!("{tag}{}:{segment}_SW", profile_edge_token(edge_names, index))
}

/// The CONTAINER standing for one profile edge's wall along every path segment,
/// `${tag}${edge}_SW` — the spelling a single-segment sweep has always used.
fn wall_container_name(tag: &str, edge_names: &[Option<String>], index: usize) -> String {
    format!("{tag}{}_SW", profile_edge_token(edge_names, index))
}

/// Source-profile base name: `${tag}${faceName}`. The `:PROFILE`
/// branch is a NO-OP (both branches yield `${tag}${faceName}`), so a plain concat
/// is exact; an empty/blank face name falls back to `PROFILE`.
fn profile_base_name(tag: &str, face_name: &str) -> String {
    let trimmed = face_name.trim();
    let name = if trimmed.is_empty() { "PROFILE" } else { trimmed };
    format!("{tag}{name}")
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected profile drives `profile`, an edge `path`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile() || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SW",
    "shortName": "SW",
    "longName": "Sweep",
    "ribbonPath": "Home/Sweep/Sweep",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the sweep feature"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "FACE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the profile to sweep"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove the referenced sketch after creating the sweep. Turn off to keep it in the scene."
        },
        "path": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "EDGE"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Sweep path: pick a whole sketch, or one or more connected edges — they are chained head-to-tail, with the FIRST pick setting the direction the sweep runs. Straight segments only."
        },
        "orientationMode": {
            "type": "options",
            "options": [
                "translate",
                "pathAlign"
            ],
            "default_value": "translate",
            "hint": "How the profile is carried along the path. 'translate' (default): the profile keeps its orientation and slides along each segment's chord — one portion per segment, so CORNERS are handled, but straight segments only. 'pathAlign': the profile is carried by the PATH'S OWN motion, keeping the position and the angle you drew it at and turning exactly as much as the path turns — a straight path extrudes it from where it sits, an arc swings it about the arc's centre, and a Helix feature's edge screws it round the helix axis and up the pitch. A smooth path is swept as one continuous tube; a CORNERED path of straight segments is MITRED, each segment's sweep trimmed to the joint's bisector plane where the two share one face loop, the inner side folded and the outer extended, like a picture frame. A CLOSED cornered path builds the whole FRAME — every joint mitred, the closing one included, and no end caps. A closed loop that is not flat comes back from one lap rolled by the path's own holonomy, and the sweep closes it with the smallest counter-twist, spread evenly along the path. Refused by name: a corner at a CURVED segment, a closed loop that crosses itself, a bend too tight for the section, and a fold at or past 168.5 degrees."
        },
        "twistAngle": {
            "type": "number",
            "default_value": 0,
            "hint": "Twist angle for the sweep. Must be 0 here — for a twisted sweep use Path Sweep (SWP)."
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

