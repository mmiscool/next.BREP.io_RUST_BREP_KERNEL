use super::*;
use super::stations::{
    StationBudget, StationParam, SweepStationReport, SWEEP_DEFAULT_TOLERANCE, SWEEP_MAX_STATIONS,
    SWEEP_REFINEMENT_ROUNDS,
};

/// Sweep a CLOSED PLANAR profile loop along a path curve (§3.3/§5.7),
/// TRANSPLANTING the profile onto the path — see [`SectionPlacement`] for the
/// other reading, and for why the choice changes the solid.
///
/// Reuses the loft builder rather than a bespoke swept surface: the path is
/// sampled at `STATIONS` uniform stations, a rotation-minimizing frame is
/// propagated along it (double-reflection RMF, Wang et al. 2008 — the frame
/// does NOT spin at inflections the way raw Frenet does), a rigidly
/// transformed copy of the profile is placed at each station (identical
/// degree/knots/weights, only the control points moved — which guarantees
/// loft's per-section compatibility), and the stations are lofted through
/// to produce the tube plus planar end caps.
///
/// Guards return a clear `Err` on an open/non-planar profile, a degenerate
/// path tangent, or a loft failure. A path that bends TIGHTER than the section
/// reaches is refused by name before the loft (`ρκ ≥ 1`, §4c of
/// [`sweep_sections_through_samples`]); it used to be out of scope and the
/// caller's problem, and what the caller got instead was a wall that passed
/// through itself.
pub fn sweep_profile_along_path(
    profile: &[NurbsCurve],
    path: &NurbsCurve,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // 32 stations is the original fixed sampling — golden parity pins the
    // emitted geometry to it — and it stays the count for every path whose
    // tangent turns through at most a full turn. A path that turns further
    // (a coil) gets the helix builder's density; see [`path_turning_stations`].
    sweep_profile_along_path_stations(
        profile,
        path,
        name,
        path_turning_stations(path),
        0.0,
        None,
        SectionPlacement::Transplant,
        SWEEP_DEFAULT_TOLERANCE,
    )
    .map(|swept| swept.solid)
}

/// Twisted path sweep (§5.7): identical to [`sweep_profile_along_path`], but
/// the profile additionally ROTATES about the path tangent, linearly in ARC
/// LENGTH, from 0 at the sweep start to `twist_angle` radians (right-handed
/// about the tangent) at the end.  The arc-length fraction comes from the
/// sampled station polyline, not the raw path parameter, so a non-uniformly
/// parameterized path still twists uniformly in space.
///
/// STATION LAW: 16 stations per quarter turn of twist, floored at the path
/// sweep's 32 and capped at 1024 (the loft's dense interpolation solve is
/// O(stations³) per control column — the same cap the helix uses).  At
/// 16/quarter-turn the inter-station twist step is Δφ ≈ 5.6°, so the cubic
/// v-interpolation error on a profile point circling at radius r is
/// ≈ r·Δφ⁴/384 ≈ 2.4·10⁻¹⁰·r — far below any geometric tolerance.  The cap
/// holds that density up to 16 full turns (|twist| = 32π = 1024/16 quarter
/// turns); a larger twist would silently alias under the cap, so it is
/// REJECTED with an honest error instead.  Everything else the path sweep
/// documents (profile validity, the tight-bend refusal) applies unchanged, and
/// the tight-bend guard measures the TWISTED section's reach because it reads
/// the axes the sections were actually placed on; twisting about the profile's
/// own centroid adds no new radial extent of its own.
pub fn sweep_profile_twisted(
    profile: &[NurbsCurve],
    path: &NurbsCurve,
    twist_angle: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use std::f64::consts::{FRAC_PI_2, TAU};

    if !twist_angle.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "twist_finite",
            "sweep_profile_twisted: twist angle must be finite",
        ));
    }
    // 16 turns is where the 1024-station cap meets 16 stations/quarter-turn;
    // beyond it the cap would degrade the twist sampling density silently.
    const MAX_TURNS: f64 = 16.0;
    if twist_angle.abs() > MAX_TURNS * TAU {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "twist_cap",
            format!(
            "sweep_profile_twisted: twist of {:.3} turns exceeds the {MAX_TURNS}-turn \
             limit the 1024-station cap can resolve at 16 stations per quarter turn; \
             split the sweep or reduce the twist",
            twist_angle.abs() / TAU
            ),
        ));
    }
    let quarter_turns = (twist_angle.abs() / FRAC_PI_2).ceil() as usize;
    let stations = (quarter_turns * 16).clamp(32, 1024).max(path_turning_stations(path));
    sweep_profile_along_path_stations(
        profile,
        path,
        name,
        stations,
        twist_angle,
        None,
        SectionPlacement::Transplant,
        SWEEP_DEFAULT_TOLERANCE,
    )
    .map(|swept| swept.solid)
    .map_err(|error| error.with_message(|error| format!("sweep_profile_twisted: {error}")))
}

/// The station budget of a single-curve path sweep, from how far the path's
/// TANGENT turns: 8 stations per quarter turn of tangent — the 32 per turn
/// [`sweep_profile_helix`] lofts a coil with — floored at the original 32 and
/// capped at the loft's 1024.
///
/// A fixed 32 was a budget for a path that bends once. Along a coil it is
/// FOUR stations a turn (an 8-turn helix edge, 2026-09-25), and a loft through
/// sections a quarter turn apart does not follow the path between them: the
/// wall cut its own corner and folded through itself, and the Tube and Path
/// Sweep features refused the edge. The floor keeps every path that turns
/// through at most a full turn on exactly the geometry it always had.
///
/// The turning is [`segment_turning`]'s — the chain sampler's own reading —
/// independent of the station count it decides. A path it cannot read is
/// reported by the sampler itself, in its own words, after the profile has been
/// checked; here it only leaves the budget on the floor.
fn path_turning_stations(path: &NurbsCurve) -> usize {
    use std::f64::consts::FRAC_PI_2;
    let turning = path
        .domain()
        .and_then(|domain| segment_turning(path, domain))
        .unwrap_or(0.0);
    // The slack keeps a path of EXACTLY a full turn (a circle) on the floor
    // rather than one quarter over it from sampling round-off.
    let quarter_turns = (turning / FRAC_PI_2 - 1e-3).ceil().max(0.0) as usize;
    (quarter_turns * 8).clamp(32, 1024)
}

/// HOW a section is placed at each station — the two things "sweep a profile
/// along a path" can mean, and the reason they are not the same shape.
///
/// `Transplant` MOVES the profile onto the path: the anchor origin lands on the
/// station point and the profile plane becomes the station's NORMAL plane. The
/// profile's own position and its angle to the path are both discarded — every
/// sweep comes out centred on the path and square to it. That is the classic
/// swept-surface reading, and it is what `SWP` builds.
///
/// `Rigid` CARRIES the profile: the section at station `k` is the profile moved
/// by the same rigid motion the PATH undergoes between its start and station
/// `k`,
///
/// ```text
///     section_k(x) = P_k + R_k · (x − P_0)
/// ```
///
/// where `R_k` is the rotation taking the start frame `(T_0, r_0, s_0)` to the
/// station's `(T_k, r_k, s_k)`. The profile keeps its drawn POSITION and its
/// drawn ANGLE to the path; what changes along the sweep is only what the path
/// itself does. Two consequences are the whole point of the mode:
///   - a STRAIGHT path makes every `R_k` the identity, so the section merely
///     translates and the result is the oblique prism `extrude_profile_brep`
///     builds from the same profile and vector — `pathAlign` and `translate`
///     agree on a straight path rather than merely having the same volume;
///   - a circular ARC makes `R_k` the rotation about the ARC'S OWN CENTRE AXIS
///     (for a planar path the RMF's transport rotation IS that rotation), so
///     the profile is carried round the pivot and the end cap arrives at the
///     angle the start cap had to the tangent. A full-circle path degenerates
///     to exactly a revolve of the profile about that axis;
///   - a HELIX makes `R_k` the SCREW about the helix's own axis — but only when
///     the path says it is one ([`SweepPath::screw_axes`]), because the frame
///     that realizes it is not the RMF. A helix is carried into itself by the
///     screw, which spins the tangent frame at the helix's torsion; the RMF has
///     no spin about the tangent by construction, so it rolls away from the
///     screw by `∫τ ds` (169.75° over three turns of a radius-5, pitch-5 coil).
///     A helix segment's stations are therefore framed off its axis
///     (`screw_reference`), and every other segment keeps the RMF — which on a
///     chain of lines and arcs in ANY planes is still exactly each arc's turn
///     about its own centre, composed. The two hand over at a joint without a
///     roll.
///
/// Why not one frame for every curve: the Frenet frame is the screw on a helix
/// but spins where a spatial curve nearly straightens (a planar S-cubic lifted
/// 1e-3 out of its plane carries a 180° roll through the inflection, which 32
/// stations read as 0.4°), and one axis fitted to the whole run is the screw on
/// a helix but rolls a two-arc spatial chain by up to 256° against its arcs.
///
/// `R_k = F_k · F_0ᵀ` does NOT depend on which perpendicular `r_0` the RMF was
/// seeded with: re-seeding rolls `F_0` and every `F_k` by the same angle about
/// the tangent, and the two rolls cancel in the product. That matters because
/// `Transplant` maps `pu → r_k` directly, so under IT the profile's roll about
/// the path is whatever `tangents[0].perpendicular()` happened to return — an
/// arbitrary orientation no caller can predict. `Rigid` has no such freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectionPlacement {
    /// Move the profile onto the path, square to it (the classic sweep).
    Transplant,
    /// Carry the profile by the path's own rigid motion (see above).
    Rigid,
}

/// The placement anchor a path sweep transplants its profile with: the plane
/// frame `(origin = boundary-sample centroid, normal, pu, pv)` the station
/// loop maps profile points through (`local = p − origin` → `station + ri·(local
/// ·pu) + si·(local·pv)`). Extracted as data so a HOLE loop can sweep with its
/// OUTER loop's anchor — sweeping each loop with its OWN centroid would
/// re-center every loop onto the path and lose the hole's in-plane offset.
#[derive(Debug, Clone, Copy)]
pub struct ProfileAnchor {
    pub origin: Vec3,
    pub normal: Vec3,
    pub pu: Vec3,
    pub pv: Vec3,
}

/// [`sweep_profile_along_path`] with an explicit placement anchor (see
/// [`ProfileAnchor`]) — the hole-loop cutter path: the swept loop is validated
/// as usual but PLACED in its outer loop's frame.
pub fn sweep_profile_along_path_anchored(
    profile: &[NurbsCurve],
    path: &NurbsCurve,
    name: Option<&str>,
    anchor: ProfileAnchor,
) -> Result<BrepSolid, KernelRefusal> {
    sweep_profile_along_path_stations(
        profile,
        path,
        name,
        path_turning_stations(path),
        0.0,
        Some(anchor),
        SectionPlacement::Transplant,
        SWEEP_DEFAULT_TOLERANCE,
    )
    .map(|swept| swept.solid)
}

/// [`sweep_profile_twisted`] with an explicit placement anchor: the hole loop
/// twists about the SAME path axis as its outer loop (shared anchor), so the
/// cutter stays registered with the outer wall through the whole twist.
pub fn sweep_profile_twisted_anchored(
    profile: &[NurbsCurve],
    path: &NurbsCurve,
    twist_angle: f64,
    name: Option<&str>,
    anchor: ProfileAnchor,
) -> Result<BrepSolid, KernelRefusal> {
    use std::f64::consts::{FRAC_PI_2, TAU};

    if !twist_angle.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "twist_finite",
            "sweep_profile_twisted: twist angle must be finite",
        ));
    }
    const MAX_TURNS: f64 = 16.0;
    if twist_angle.abs() > MAX_TURNS * TAU {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "twist_cap",
            format!(
            "sweep_profile_twisted: twist of {:.3} turns exceeds the {MAX_TURNS}-turn \
             limit the 1024-station cap can resolve at 16 stations per quarter turn; \
             split the sweep or reduce the twist",
            twist_angle.abs() / TAU
            ),
        ));
    }
    let quarter_turns = (twist_angle.abs() / FRAC_PI_2).ceil() as usize;
    let stations = (quarter_turns * 16).clamp(32, 1024).max(path_turning_stations(path));
    sweep_profile_along_path_stations(
        profile,
        path,
        name,
        stations,
        twist_angle,
        Some(anchor),
        SectionPlacement::Transplant,
        SWEEP_DEFAULT_TOLERANCE,
    )
    .map(|swept| swept.solid)
    .map_err(|error| error.with_message(|error| format!("sweep_profile_twisted: {error}")))
}

/// Sweep a CLOSED PLANAR profile along a CHAIN of path curves joined
/// head-to-tail — the multi-segment entry point, with the twist and anchor
/// options the single-curve family exposes as separate functions folded into
/// one signature (the feature layer picks all three per loop, so splitting them
/// four ways here would only push the same match into the caller).
///
/// `path` carries the chain AND its classification ([`SweepPath`]): the segment
/// names for the joint refusal (a short or empty name list degrades to
/// `<unnamed>` rather than failing), whether the run is CLOSED, the plane it lies
/// in if it lies in one, and the continuity of every joint. The refusals below
/// READ that classification rather than each re-deriving it — the corner gate no
/// longer measures its own tangent break while sampling, and closure is a
/// property of the path rather than something the sampler notices about its first
/// and last station.
///
/// `corner_advice` is appended to the cornered-joint refusal and is the CALLER's
/// to write, because the builder has two callers with different answers: path
/// sweep sends the user to SW (a different feature), while SW's own `pathAlign`
/// sends them to its `translate` mode. A builder that named one feature would be
/// telling half its users to switch to the feature they are already in.
///
/// A ONE-segment OPEN chain delegates to the single-curve builders unchanged, so
/// the geometry every existing path sweep emits is untouched by this entry point
/// existing — the multi-segment sampler is reached only by a path that actually
/// has a joint. A one-segment CLOSED chain (a resident circular edge) does NOT
/// delegate: the single-curve samplers cap both ends, which is exactly what a
/// ring must not do.
///
/// A CORNER (a joint past [`MAX_JOINT_TANGENT_BREAK`]) selects the MITRE lane
/// when every segment is straight — see `sweep_topology/miter.rs` for the
/// construction and [`CornerPolicy`] for why this entry mitres while
/// [`sweep_profile_along_chain_with_stations`] refuses. A CLOSED cornered
/// polyline is a picture FRAME: its closing joint is a mitre like the rest and
/// the result has no caps. A corner at a CURVED segment is refused by name.
pub fn sweep_profile_along_chain(
    profile: &[NurbsCurve],
    path: &SweepPath,
    twist_angle: f64,
    name: Option<&str>,
    anchor: Option<ProfileAnchor>,
    placement_mode: SectionPlacement,
    corner_advice: &str,
) -> Result<BrepSolid, KernelRefusal> {
    sweep_profile_along_chain_reported(
        profile,
        path,
        twist_angle,
        name,
        anchor,
        placement_mode,
        corner_advice,
        None,
    )
    .map(|swept| swept.solid)
}

/// A swept solid with the MEASURED BOUND of its construction
/// ([`SweepStationReport`]): `None` where the construction is exact — the
/// mitre lane (exact pieces on bisector planes) and the envelope lane (a
/// revolve) — and the station lane's report otherwise.
#[derive(Debug)]
pub struct SweptChain {
    pub solid: BrepSolid,
    pub report: Option<SweepStationReport>,
}

impl SweptChain {
    fn exact(solid: BrepSolid) -> Self {
        Self {
            solid,
            report: None,
        }
    }
}

/// [`sweep_profile_along_chain`] with the requested wall TOLERANCE (mm;
/// `None` for [`SWEEP_DEFAULT_TOLERANCE`]) and the measured bound the build
/// reached, for a caller that carries it on (the sweep features put it in
/// `FeatureResult::approximations`). The station lane refines its stations
/// until the loft's own interpolant stands within `tolerance` of the kinematic
/// sweep's wall, or the station cap stops it; the report says which.
#[allow(clippy::too_many_arguments)]
pub fn sweep_profile_along_chain_reported(
    profile: &[NurbsCurve],
    path: &SweepPath,
    twist_angle: f64,
    name: Option<&str>,
    anchor: Option<ProfileAnchor>,
    placement_mode: SectionPlacement,
    corner_advice: &str,
    tolerance: Option<f64>,
) -> Result<SweptChain, KernelRefusal> {
    use std::f64::consts::{FRAC_PI_2, TAU};
    let tolerance = sweep_tolerance(tolerance)?;

    if path.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_empty",
            "sweepSolid: path chain is empty",
        ));
    }
    // A bend TIGHTER than the section is the envelope lane's, not the loft's, and
    // it is asked BEFORE the single-curve shortcut below — that shortcut lands in
    // the same skinning builder, so taking it first would hide the one path shape
    // this lane exists for (one circular arc) behind it.
    let envelope_lane = super::envelope::recognize_swept_envelope(profile, path, placement_mode)?
        .is_ok();

    if !twist_angle.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "twist_finite",
            if path.curves.len() == 1 && placement_mode == SectionPlacement::Transplant {
                "sweep_profile_twisted: twist angle must be finite"
            } else {
                "sweep_profile_along_chain: twist angle must be finite"
            },
        ));
    }
    const MAX_TURNS: f64 = 16.0;
    if twist_angle.abs() > MAX_TURNS * TAU {
        let who = if path.curves.len() == 1 && placement_mode == SectionPlacement::Transplant {
            "sweep_profile_twisted"
        } else {
            "sweep_profile_along_chain"
        };
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "twist_cap",
            format!(
            "{who}: twist of {:.3} turns exceeds the {MAX_TURNS}-turn \
             limit the 1024-station cap can resolve at 16 stations per quarter turn; \
             split the sweep or reduce the twist",
            twist_angle.abs() / TAU
            ),
        ));
    }
    // Single segment under `Transplant`: the single-curve sampler's own budget
    // on the single-curve lane, so an ordinary one-edge path sweep emits the
    // geometry it always has wherever it reads inside the tolerance. `Rigid`
    // has no single-curve budget and falls through to the chain sampler, which
    // samples a 1-element chain over exactly the domain the single-curve
    // sampler would (no joints, the whole budget on one segment). A CLOSED
    // single segment falls through too — the single-curve lane ends in a capped
    // loft, and a ring has no caps.
    if let ([single], SectionPlacement::Transplant, false, false) =
        (path.curves.as_slice(), placement_mode, path.closed, envelope_lane)
    {
        // The single-curve builders' own budgets ([`path_turning_stations`],
        // the twisted law), on the single-curve lane, at this tolerance.
        let stations = if twist_angle == 0.0 {
            path_turning_stations(single)
        } else {
            let quarter_turns = (twist_angle.abs() / FRAC_PI_2).ceil() as usize;
            (quarter_turns * 16).clamp(32, 1024).max(path_turning_stations(single))
        };
        return sweep_profile_along_path_stations(
            profile,
            single,
            name,
            stations,
            twist_angle,
            anchor,
            SectionPlacement::Transplant,
            tolerance,
        )
        .map_err(|error| {
            if twist_angle == 0.0 {
                error
            } else {
                error.with_message(|error| format!("sweep_profile_twisted: {error}"))
            }
        });
    }

    // Everything else — any multi-segment chain, and a single segment under
    // `Rigid` — takes the same station law the single-curve twisted builder
    // uses, so density scales with twist identically on a chained path.
    // A CLOSED SPATIAL path lays its counter-twist down on top of the requested
    // twist, which adds up to half a turn, so the density is set for the most the
    // lap can roll.
    let stations = if twist_angle == 0.0 {
        32
    } else {
        let counter = if path.closed && path.planar.is_none() {
            std::f64::consts::PI
        } else {
            0.0
        };
        let quarter_turns = ((twist_angle.abs() + counter) / FRAC_PI_2).ceil() as usize;
        (quarter_turns * 16).clamp(32, 1024)
    };
    let _ = name;
    sweep_profile_along_chain_stations(
        profile,
        path,
        stations,
        twist_angle,
        anchor,
        placement_mode,
        corner_advice,
        CornerPolicy::Mitre,
        tolerance,
    )
}

/// The wall tolerance a sweep is refined to: the caller's, or the default;
/// refused when it is not a positive finite length.
fn sweep_tolerance(tolerance: Option<f64>) -> Result<f64, KernelRefusal> {
    let tolerance = tolerance.unwrap_or(SWEEP_DEFAULT_TOLERANCE);
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "sweep_tolerance",
            format!("sweepSolid: the requested wall tolerance must be a positive length, not {tolerance}"),
        ));
    }
    Ok(tolerance)
}

/// [`sweep_profile_along_chain`] with an explicit STATION budget and no twist
/// — the entry a caller with a long, smooth chain uses when the default 32
/// stations would under-sample it (a wire-harness bundle along a many-anchor
/// spline: three pieces per span, so a handful of spans already starves each
/// piece of stations). The budget is spread over the chain by arc length
/// exactly as the twisted builder's law spreads its own; the 1024-station cap
/// and the tangent-continuity refusal apply unchanged.
pub fn sweep_profile_along_chain_with_stations(
    profile: &[NurbsCurve],
    path: &SweepPath,
    stations: usize,
    corner_advice: &str,
) -> Result<BrepSolid, KernelRefusal> {
    sweep_profile_along_chain_with_stations_reported(profile, path, stations, corner_advice, None)
        .map(|swept| swept.solid)
}

/// [`sweep_profile_along_chain_with_stations`] with the requested wall
/// tolerance (`None` for the default) and the measured bound: the caller's
/// budget is the FLOOR, refined where the measured deviation is over the
/// tolerance.
pub fn sweep_profile_along_chain_with_stations_reported(
    profile: &[NurbsCurve],
    path: &SweepPath,
    stations: usize,
    corner_advice: &str,
    tolerance: Option<f64>,
) -> Result<SweptChain, KernelRefusal> {
    let tolerance = sweep_tolerance(tolerance)?;
    sweep_profile_along_chain_stations(
        profile,
        path,
        stations.clamp(2, 1024),
        0.0,
        None,
        SectionPlacement::Transplant,
        corner_advice,
        CornerPolicy::Refuse,
        tolerance,
    )
}

/// Validate a closed planar profile loop and derive its placement anchor —
/// the path sweep's §1 block, extracted bit-identically: 16 samples per curve,
/// closure at `tolerance`, Newell normal, boundary-sample-mean origin,
/// planarity at `tolerance * 100`, `pu = np.perpendicular()`, `pv = np × pu`.
pub fn profile_anchor(profile: &[NurbsCurve]) -> Result<ProfileAnchor, KernelRefusal> {
    let tolerance = 1e-6;
    if profile.len() < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_count",
            "sweepSolid: profile needs at least 2 curves forming a closed loop",
        ));
    }
    let mut samples = Vec::new();
    for (index, curve) in profile.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        let next = &profile[(index + 1) % profile.len()];
        let next_start = next.domain().or_refuse(KernelStage::Collect, "domain")?[0];
        if curve
            .evaluate(end).or_refuse(KernelStage::Collect, "evaluate")?
            .sub(next.evaluate(next_start).or_refuse(KernelStage::Collect, "evaluate")?)
            .length()
            > tolerance
        {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "profile_closed",
                format!("sweepSolid: profile is not closed at curve {index}"),
            ));
        }
        for sample in 0..16 {
            samples.push(curve.evaluate(start + (end - start) * sample as f64 / 16.0).or_refuse(KernelStage::Collect, "evaluate")?);
        }
    }
    let normal = crate::polygon::newell_normal(&samples);
    let centroid = samples.iter().fold(Vec3::default(), |sum, &point| sum.add(point));
    let np = normal.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "profile_area",
            "sweepSolid: profile is degenerate (zero enclosed area)",
        )
    })?;
    let origin = centroid.scale(1.0 / samples.len() as f64);
    if samples
        .iter()
        .any(|point| point.sub(origin).dot(np).abs() > tolerance * 100.0)
    {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_planar",
            "sweepSolid: profile is not planar",
        ));
    }
    let pu = np.perpendicular().or_refuse(KernelStage::Collect, "perpendicular")?;
    let pv = np.cross(pu).normalized().or_refuse(KernelStage::Collect, "normalized")?;
    Ok(ProfileAnchor {
        origin,
        normal: np,
        pu,
        pv,
    })
}

/// Core of the path sweep with an explicit station count.  Every consumer of
/// the station count (path sampling, RMF propagation, section placement)
/// derives from the one `stations` argument so the density scales as a unit.
/// `twist_angle` (radians; 0 for the untwisted variants) rotates the placed
/// profile about the path tangent linearly in sampled arc length — the
/// `twist_angle == 0.0` fast path leaves the RMF axes bit-identical, so the
/// untwisted callers keep golden parity.
fn sweep_profile_along_path_stations(
    profile: &[NurbsCurve],
    path: &NurbsCurve,
    name: Option<&str>,
    stations: usize,
    twist_angle: f64,
    anchor: Option<ProfileAnchor>,
    placement_mode: SectionPlacement,
    tolerance: f64,
) -> Result<SweptChain, KernelRefusal> {
    // Loft carries no face names; the app stamps them onto the emitted face
    // order.  Accept `name` for ABI symmetry with the other builders.
    let _ = name;
    if stations < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "stations",
            "sweepSolid: need at least 2 stations",
        ));
    }

    // --- 1. Validate the profile: closed + planar; derive (origin, np, pu, pv).
    //        A caller-supplied anchor OVERRIDES the placement frame (the swept
    //        loop is still validated against its own plane), so a hole loop
    //        rides the path in its outer loop's frame instead of re-centering.
    //        Resolved HERE, before the path is touched, so a bad profile is
    //        still reported ahead of a bad path exactly as it always was.
    let placement = resolve_placement(profile, anchor, placement_mode)?;

    // --- 2. Sample the path as a one-segment chain on the single-curve lane:
    //        `stations` uniform in the parameter, the sampling this builder
    //        always had, refined only where the measured deviation is over the
    //        tolerance (`chain_stations`).
    let chain = std::slice::from_ref(path);
    let (samples, budget) = chain_stations(chain, false, SamplerLane::SingleCurve, stations, tolerance)?;
    let StationSamples {
        points,
        tangents,
        curvatures,
        params,
        ..
    } = &samples;

    sweep_sections_through_samples(
        profile,
        points,
        tangents,
        curvatures,
        twist_angle,
        placement,
        placement_mode,
        RingFrame::Open,
        None,
        None,
        StationRun {
            chain,
            params,
            budget,
            tolerance,
        },
    )
}

/// What a CHAIN builder does with a CORNER (a `G0` joint).
///
/// Two of this module's entry points sweep chains, and they answer the question
/// differently — not because the geometry differs but because their callers'
/// corner POLICIES do, and a builder that guessed would be overriding one of
/// them:
///   - [`sweep_profile_along_chain`] MITRES a polyline's corners, a closed
///     polyline's closing joint included (`sweep_topology/miter.rs`). Its callers
///     are the two sweep FEATURES, and a mitred corner is the shape they are
///     asked for.
///   - [`sweep_profile_along_chain_with_stations`] REFUSES every corner. Its
///     callers build their own chains and have already decided what a corner
///     means: the tube ROUNDS one into a tangent bend or joins it with a ball,
///     and the wire harness routes tangent-continuous splines. A corner
///     reaching either of them means their own construction did not come out,
///     and mitring it instead would silently ship a shape neither feature
///     offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CornerPolicy {
    /// A polyline's corners are mitred, open or closed; anything else is refused.
    Mitre,
    /// Every corner is refused by name.
    Refuse,
}

/// The SECTION a mitred sweep carries, placed in world, with its plane normal —
/// the two things `sweep_topology/miter.rs` needs, and all it needs.
///
/// The placement modes are resolved HERE rather than in the mitre builder, on
/// the same frame the skinning lane's station 0 uses (`û₀.perpendicular()`), so
/// the two lanes place a section identically and a path that crosses the
/// continuity gate does not also jump in space:
///   - `Transplant` maps the profile's plane frame onto the path's first station
///     (anchor origin onto the path, profile square to it), so the section's
///     normal is the first tangent;
///   - `Rigid` takes the profile WHERE IT IS, keeping the offset from the path
///     and the angle to it that it was drawn with, so the section's normal is
///     the profile's own.
fn place_section(
    profile: &[NurbsCurve],
    placement: ProfileAnchor,
    placement_mode: SectionPlacement,
    start: Vec3,
    tangent: Vec3,
) -> Result<(Vec<NurbsCurve>, Vec3), KernelRefusal> {
    if placement_mode == SectionPlacement::Rigid {
        return Ok((profile.to_vec(), placement.normal));
    }
    let ProfileAnchor { origin, pu, pv, .. } = placement;
    let r0 = tangent.perpendicular().or_refuse(KernelStage::Fragment, "perpendicular")?;
    let s0 = tangent.cross(r0).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let mut placed = Vec::with_capacity(profile.len());
    for curve in profile {
        let control_points = curve
            .control_points
            .iter()
            .map(|control| {
                let point = control.point()?;
                let local = point.sub(origin);
                let world = start
                    .add(r0.scale(local.dot(pu)))
                    .add(s0.scale(local.dot(pv)));
                Ok(Vec4::from_point(world, control.w))
            })
            .collect::<Result<Vec<_>, String>>()
            .or_refuse(KernelStage::Fragment, "control_point")?;
        placed.push(
            NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                .or_refuse(KernelStage::Fragment, "curve_new")?,
        );
    }
    Ok((placed, tangent))
}

/// The placement frame a sweep transplants its profile with: the caller's
/// `anchor` when it supplied one (a hole loop riding its outer loop's frame),
/// otherwise the profile's own. Either way the profile is VALIDATED — closed and
/// planar — because `profile_anchor` is what does that validating, and skipping
/// it for an anchored loop would let an open hole loop through.
///
/// Called by each SAMPLER before it touches the path, so the "bad profile" error
/// still precedes the "bad path" one, as it did when the two lived in one
/// function.
fn resolve_placement(
    profile: &[NurbsCurve],
    anchor: Option<ProfileAnchor>,
    placement_mode: SectionPlacement,
) -> Result<ProfileAnchor, KernelRefusal> {
    let computed = profile_anchor(profile)?;
    Ok(match placement_mode {
        // RIGID never transplants, so there is no frame to borrow: every loop
        // keeps its own position already, which is the whole reason the anchor
        // exists under `Transplant`. Taking a caller's anchor here would silently
        // swap in another loop's NORMAL for the fold-back guard below.
        SectionPlacement::Rigid => computed,
        SectionPlacement::Transplant => anchor.unwrap_or(computed),
    })
}


/// Sweep a CLOSED PLANAR profile along a CHAIN of path curves, joined
/// head-to-tail, as one continuous tube — or, for a CLOSED path, as a capless
/// ring.
///
/// The chain is sampled as a single trajectory and handed to the shared core,
/// so ONE rotation-minimizing frame is propagated across the whole path: a joint
/// is just another pair of adjacent stations to the RMF, which is what makes the
/// profile arrive at the far end in the orientation the near end implies rather
/// than snapping at each segment.
///
/// STATION BUDGET. `stations` are distributed across the segments in proportion
/// to estimated ARC LENGTH (chord sums over 16 samples per curve), floored at 2
/// per segment so a short fillet between two long lines still carries its
/// curvature. The shared endpoint of two adjacent segments is sampled ONCE, not
/// twice — a duplicated station would land in the RMF's coincident-step branch
/// and waste a section on zero advance. A CLOSED path drops one more station for
/// the same reason: its last station IS its first.
///
/// JOINTS decide the LANE. A break within [`MAX_JOINT_TANGENT_BREAK`] is skinned
/// here; a break past it is a CORNER, which under [`CornerPolicy::Mitre`] hands an
/// open all-straight run to the mitre builder (`sweep_topology/miter.rs`) and is
/// otherwise refused by name, with the angle and the two segments. The
/// classification carried on the path is what says so — this sampler no longer
/// measures its own.
#[allow(clippy::too_many_arguments)]
fn sweep_profile_along_chain_stations(
    profile: &[NurbsCurve],
    path: &SweepPath,
    stations: usize,
    twist_angle: f64,
    anchor: Option<ProfileAnchor>,
    placement_mode: SectionPlacement,
    corner_advice: &str,
    corners: CornerPolicy,
    tolerance: f64,
) -> Result<SweptChain, KernelRefusal> {
    let chain = path.curves.as_slice();
    if chain.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_empty",
            "sweepSolid: path chain is empty",
        ));
    }
    if stations < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "stations",
            "sweepSolid: need at least 2 stations",
        ));
    }
    // Profile first, matching the single-curve sampler's order of complaint.
    let placement = resolve_placement(profile, anchor, placement_mode)?;

    // --- The swept ENVELOPE, trimmed at its own self-intersection.
    //
    //     Past `ρκ = 1` the sections CROSS and the loft below skins a wall that
    //     passes through itself. For a CIRCLE carried along one circular planar
    //     arc the correct solid is a REVOLVE — the section disc truncated at the
    //     axis, plus the reflection of the part that crossed it, each revolved
    //     through the path's own turn (`sweep_topology/envelope.rs`) — and this
    //     is where that lane is taken instead of the loft.
    //
    //     Anything else tight is left to the §4c refusal below, which now carries
    //     WHICH half of the configuration this sweep missed. Nothing under
    //     `ρκ = 1` reaches the lane at all: the recognition refuses a bend that
    //     is not tighter than its section, so every shape that builds today keeps
    //     the builder it has.
    let envelope_gap =
        match super::envelope::recognize_swept_envelope(profile, path, placement_mode)? {
            Ok(plan) => {
                let mut bodies = super::envelope::build_swept_envelope(&plan)?;
                if bodies.len() == 1 {
                    return Ok(SweptChain::exact(bodies.remove(0)));
                }
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "envelope_two_bodies",
                    format!(
                    "{}: a section that crosses the path's axis (ρ·κ = {:.6}) sweeps TWO \
                     bodies — the near lobe over the path's own azimuths and the far lobe half \
                     a turn from them, meeting only along the axis — and this entry returns one \
                     solid. {} were built; ask `sweep_envelope_bodies` for the pair",
                    SWEEP_ENVELOPE_REFUSAL,
                    plan.ratio(),
                    bodies.len()
                    ),
                ));
            }
            Err(reason) => reason,
        };

    // --- A CORNER, read off the path's classification. Every joint was measured
    //     once when the path was resolved, the CLOSING joint of a closed path
    //     included — which is how a ring's seam gets the same gate as its
    //     interior joints instead of being the one joint nobody checked.
    //
    //     A corner now selects a LANE rather than ending the build. A POLYLINE —
    //     every segment straight, at least one corner — is MITRED: the section is
    //     swept along each segment and the two sweeps are trimmed to the joint's
    //     bisector plane, sharing one face loop there
    //     (`sweep_topology/miter.rs`). A CLOSED one is a picture FRAME, whose
    //     closing joint is mitred like the rest and which has no caps; the frame
    //     lane's own refusals (a spatial loop, by its holonomy; a self-crossing
    //     loop; overlapping mitres) live with the construction. What remains
    //     refused HERE is a corner on a path that is not all straight — there is
    //     no bisector plane for a curve to be trimmed to, and no lane that mixes
    //     the two.
    if let Some(corner) = path.corner() {
        if corners == CornerPolicy::Mitre && path.cornered_polyline() {
            // A twist is laid down in arc length over the stations, and a mitre has
            // none. A CLOSED frame rolls it along the middle of each side instead,
            // where a roll keeps the shared joint loops (`sweep_topology/miter.rs`),
            // exactly as it rolls its own counter-twist. An OPEN cornered run does
            // not have that construction and is refused.
            if twist_angle != 0.0 && !path.closed {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "cornered_twist",
                    "sweepSolid: a CORNERED path cannot carry a twist — its corners are mitred \
                     from exact pieces that meet on the joint's bisector plane, and a twist laid \
                     down along the path would roll the two sides of that plane apart, so they \
                     would no longer share one loop there. Only a CLOSED frame rolls a twist along \
                     the middle of its sides. Sweep the run without the twist; a \
                     tangent-continuous (smooth) path is skinned through stations and carries one",
                ));
            }
            let (section, section_normal, _) =
                mitre_section(profile, placement, placement_mode, path)?;
            return super::miter::sweep_section_mitered(
                &section,
                section_normal,
                path,
                twist_angle,
            )
            .map(SweptChain::exact);
        }
        // A path with BOTH a corner and a curved segment is in NEITHER lane, and
        // this is the refusal that says so: the mitre lane needs every segment
        // straight (one direction per side for the bisector plane to bisect) and
        // the skinning lane needs every joint tangent-continuous (it interpolates
        // stations, so it would round the corner off). A rounded rectangle with
        // ONE corner left sharp is exactly that path — three G0 joints the mitre
        // lane could carry and two G1 joints at an arc it cannot — and mitring
        // the sharp corners while skinning the arc is a MIXED construction this
        // builder does not have. Refused with both halves named rather than built
        // as whichever lane happens to be asked first.
        let curved = (0..path.len())
            .find(|index| {
                let curve = &path.curves[*index];
                curve.degree != 1 || curve.control_points.len() != 2
            })
            .map(|index| format!(" — segment '{}' is CURVED", path.name(index)))
            .unwrap_or_default();
        // The MIXED-LANE sentence belongs only to a path that actually has a
        // curved segment. Under [`CornerPolicy::Refuse`] an ALL-STRAIGHT cornered
        // chain reaches this same refusal (the tube's and the wire harness's
        // corners are refused whatever their segments are), and telling those
        // callers that a straight cornered run is mitred would name a lane they
        // deliberately do not have — their own `corner_advice` says what to do
        // instead.
        let mixed = if curved.is_empty() {
            String::new()
        } else {
            " A path that MIXES the two — corners at some joints and curved segments elsewhere              — is in neither lane: every segment straight is mitred (open or closed), every              joint tangent-continuous is skinned, and nothing builds one construction for some              joints and the other for the rest."
                .to_string()
        };
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "corner",
            format!(
            "the path must be tangent-continuous: segments '{}' and '{}' meet at a {:.1}° \
             corner{curved}. The profile is skinned between sampled stations along the path, so a \
             corner would be ROUNDED OFF rather than mitred, and a mitre needs two STRAIGHT \
             segments: it trims both sweeps to the plane that bisects their directions, and a \
             curve has no single direction to bisect.{mixed} {corner_advice}",
            path.name(corner.before),
            path.name(corner.after),
            corner.tangent_break.to_degrees()
            ),
        ));
    }

    // --- A CLOSED path takes a REQUESTED twist only where it closes. After one lap
    //     the section comes back rolled by the whole twist, so the twist must be a
    //     rotational symmetry of the section about the path. The ring lane reads
    //     that off the section and refuses by name a twist that does not close
    //     ([`closing_twist`]).

    // --- A bend TIGHTER than the section is refused on the FLOOR stations,
    //     before any are spent refining a run that folds, and so that the
    //     refusal quotes the same stations whatever tolerance was asked
    //     (`sweep_bend_profile` reads the same). Untwisted `Transplant` only —
    //     the reach of a rolled section is the twisted placement's to read, in
    //     §4c of `sweep_sections_through_samples`, which stays the backstop.
    if placement_mode == SectionPlacement::Transplant
        && twist_angle == 0.0
        && std::env::var("BREP_SWEEP_BEND_GUARD").as_deref() != Ok("0")
    {
        let (floor, _) = chain_stations(path.curves.as_slice(), path.closed, SamplerLane::Chain, stations, 0.0)?;
        let profile_stations = floor_bend_stations(profile, &placement, path, &floor)?;
        if let Some(refusal) = tight_bend_refusal(&profile_stations, floor.points.len()) {
            return Err(with_envelope_gap(refusal, Some(envelope_gap.as_str())));
        }
    }

    // --- Sample the chain as ONE trajectory (budget, domains, stations), and
    //     refine it to the requested tolerance.
    let (samples, budget) = chain_stations(
        path.curves.as_slice(),
        path.closed,
        SamplerLane::Chain,
        stations,
        tolerance,
    )?;
    let StationSamples {
        points,
        tangents,
        curvatures,
        segments,
        params,
    } = &samples;

    // --- CLOSED: the ring needs a frame that comes back to itself. A PLANAR path
    //     has one for free (see `ring_normal`); a SPATIAL one comes back rolled by
    //     its own holonomy and is closed by the counter-twist policy
    //     ([`RingFrame::Spatial`]).
    let closure = ring_frame(path, tangents)?;

    sweep_sections_through_samples(
        profile,
        points,
        tangents,
        curvatures,
        twist_angle,
        placement,
        placement_mode,
        closure,
        Some(envelope_gap.as_str()),
        Some((segments.as_slice(), path)),
        StationRun {
            chain: path.curves.as_slice(),
            params,
            budget,
            tolerance,
        },
    )
}

/// How a station run CLOSES — which frame a ring's sections are placed on.
///
/// Three states, because "is this a ring" and "is its frame written down in
/// closed form" are different questions and only a planar ring answers yes to
/// both:
///   - `Open`: the transported frame and two end caps;
///   - `Planar(normal)`: a ring whose reference axis is the path plane's normal
///     at every station, which returns to itself EXACTLY (see [`ring_normal`]);
///   - `Spatial`: a ring on the TRANSPORTED frame, which returns rolled by the
///     path's own holonomy and is closed by the counter-twist policy — see
///     [`ring_closure_frames`].
#[derive(Debug, Clone, Copy)]
enum RingFrame {
    Open,
    Planar(Vec3),
    Spatial,
}

impl RingFrame {
    fn is_ring(self) -> bool {
        !matches!(self, RingFrame::Open)
    }
}

/// The ring frame a path's stations close on, read off its classification.
fn ring_frame(path: &SweepPath, tangents: &[Vec3]) -> Result<RingFrame, KernelRefusal> {
    Ok(match (path.closed, path.planar) {
        (false, _) => RingFrame::Open,
        (true, Some(normal)) => RingFrame::Planar(ring_normal(normal, tangents)?),
        (true, None) => RingFrame::Spatial,
    })
}

/// What a CLOSED sweep did to make its section meet itself after one lap — the
/// applied twist the frame-closure policy REPORTS.
///
/// THE POLICY. Carried once round a closed path, the section frame comes back
/// ROLLED about the start tangent by the path's HOLONOMY. That angle is not a
/// defect of the sampling: the frame is parallel transport of the section's
/// axes in the path's normal plane, which is parallel transport on the unit
/// sphere along the path's TANGENT INDICATRIX, so by Gauss–Bonnet the roll is
/// the solid angle that indicatrix encloses, modulo `2π`. A planar path's
/// indicatrix runs along one great circle and encloses a hemisphere — `2π`,
/// the identity — which is why a planar ring closes on its own; a spatial one
/// generally does not. (A curve lying ON a sphere reads zero too: the radial
/// direction is itself rotation-minimizing along any such curve. Its enclosed
/// solid angle is not its holonomy; its indicatrix's is.)
///
/// The sweep closes the ring with the SMALLEST roll that does it: the
/// counter-twist `−holonomy`, wrapped to `(−π, π]`.
///   - SMOOTH lane: laid down LINEARLY IN ARC LENGTH over the stations, the wrap
///     step from the last station back to station 0 included, so every span —
///     the seam span too — rolls at the same rate and the closed loft sees no
///     seam at all. A uniform rate is the distribution with the smallest peak
///     rate and the smallest `∫(roll rate)² ds`.
///   - CORNERED lane (`sweep_topology/miter.rs`): shared over the SEGMENTS in
///     proportion to their length, each share laid down over its segment's
///     untrimmed middle — a roll AT a mitre would pull the shared loop apart —
///     and nothing at all when the untwisted lap already closes within the
///     lane's own weld floor.
///
/// A REQUESTED twist rides on top of that counter-twist and is laid down the same
/// way, so `applied_twist = requested_twist − holonomy` (or `requested_twist` alone
/// where no counter-twist is needed). It closes only as a rotational symmetry of
/// the section about the path, `2π·n/k` for a section with `k` of them, and is
/// refused by name otherwise ([`crate::SWEEP_TWIST_CLOSURE_REFUSAL`]). A
/// FRACTIONAL turn on a symmetric section lands each wall on a different section
/// curve than it left: `curve_shift`.
///
/// `holonomy` is what the untwisted frame measures; `applied_twist` is what was
/// laid down. All three angles are radians, right-handed about the path tangent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepClosure {
    /// The roll the UNTWISTED frame returns with after one lap, right-handed
    /// about the start tangent, wrapped to `(−π, π]`. Zero for a planar path.
    pub holonomy: f64,
    /// The twist the caller asked for, snapped to the section symmetry it closes
    /// on (it matched at round-off). Zero for an untwisted sweep.
    pub requested_twist: f64,
    /// The twist laid down over the lap: the requested twist plus the
    /// counter-twist `−holonomy`, which is zero where the lane closes without one
    /// (a planar path, or a frame whose untwisted lap already closes within the
    /// weld floor).
    pub applied_twist: f64,
    /// How far along the section's curve list one lap carries each curve: the
    /// wall leaving section curve `j` arrives on curve `(j + curve_shift) mod m`.
    /// Zero for no twist and for a whole number of turns. The curves are counted
    /// in the order the builder carries them, which on the MITRE lane is the
    /// section's order after its winding is made counter-clockwise about the path.
    pub curve_shift: usize,
}

/// The CLOSURE a sweep of `profile` along `path` applies — the frame-closure
/// policy's report, exposed so a caller can read the twist a closed sweep laid
/// down (see [`SweepClosure`]). `None` for an OPEN path, which has no lap to
/// close.
///
/// Reads the very construction the sweep builds, not a second measurement of
/// it: a cornered polyline asks the mitre lane's own frame plan (its placed
/// section, its rings, its weld floor), and any other closed path asks the
/// smooth lane's own stations and frames at `stations` — the budget the sweep
/// was given (32 for [`sweep_profile_along_chain`] untwisted, the caller's for
/// [`sweep_profile_along_chain_with_stations`]). Whatever the builder would
/// refuse, this refuses with the same words, a requested `twist_angle` that does
/// not close included.
pub fn sweep_closure(
    profile: &[NurbsCurve],
    path: &SweepPath,
    placement_mode: SectionPlacement,
    stations: usize,
    twist_angle: f64,
) -> Result<Option<SweepClosure>, KernelRefusal> {
    if !path.closed {
        return Ok(None);
    }
    let placement = resolve_placement(profile, None, placement_mode)?;
    if path.cornered_polyline() {
        let (section, section_normal, _) = mitre_section(profile, placement, placement_mode, path)?;
        return super::miter::mitred_frame_closure(&section, section_normal, path, twist_angle)
            .map(Some);
    }
    let (
        StationSamples {
            points,
            tangents,
            segments,
            ..
        },
        _,
    ) = chain_stations(
        path.curves.as_slice(),
        path.closed,
        SamplerLane::Chain,
        stations.clamp(2, 1024),
        SWEEP_DEFAULT_TOLERANCE,
    )?;
    let screw = Some((segments.as_slice(), path)).filter(|_| placement_mode == SectionPlacement::Rigid);
    let closure = ring_frame(path, &tangents)?;
    let (section, _) = place_section(profile, placement, placement_mode, points[0], tangents[0])?;
    let requested = ring_closing_twist(&section, &points, &tangents, twist_angle)?;
    Ok(station_frames(&points, &tangents, closure, screw, requested)?.2)
}

/// The requested twist a RING closes on ([`closing_twist`]), read off the
/// section as placed at station 0, about the path's first tangent, against the
/// ring's own position floor: `1e-6` of the lap's chord length, the floor the
/// mitred frame measures its own closure against.
fn ring_closing_twist(
    section: &[NurbsCurve],
    points: &[Vec3],
    tangents: &[Vec3],
    twist_angle: f64,
) -> Result<ClosingTwist, KernelRefusal> {
    let lap: f64 = (0..points.len())
        .map(|index| points[(index + 1) % points.len()].sub(points[index]).length())
        .sum();
    closing_twist(section, points[0], tangents[0], twist_angle, 1e-6 * lap)
}

/// The placed section a MITRED sweep carries and its plane normal, with the
/// first segment's chord direction — shared by the builder and by
/// [`sweep_closure`], so the instrument reads the section the frame is built
/// from.
fn mitre_section(
    profile: &[NurbsCurve],
    placement: ProfileAnchor,
    placement_mode: SectionPlacement,
    path: &SweepPath,
) -> Result<(Vec<NurbsCurve>, Vec3, Vec3), KernelRefusal> {
    let [t0, t1] = path.curves[0].domain().or_refuse(KernelStage::Collect, "domain")?;
    let start = path.curves[0].evaluate(t0).or_refuse(KernelStage::Collect, "evaluate")?;
    // The CHORD direction, which is both the first station's tangent (a
    // straight segment's derivative is its chord) and the direction the
    // mitre builder sweeps along, so the placed section is square to the
    // same vector the construction uses.
    let tangent = path.curves[0]
        .evaluate(t1).or_refuse(KernelStage::Collect, "evaluate")?
        .sub(start)
        .normalized()
        .map_err(|_| {
            KernelRefusal::input(
                KernelStage::Collect,
                "segment_length",
                "sweepSolid: path segment 0 has zero length",
            )
        })?;
    let (section, section_normal) =
        place_section(profile, placement, placement_mode, start, tangent)?;
    Ok((section, section_normal, tangent))
}

/// The station frames §3 places its sections on, extracted so the tight-bend
/// INSTRUMENT ([`sweep_bend_profile`]) reads the frame the sweep will use — the
/// section's reach into a bend depends on how the section is ROLLED about the
/// path, so a second frame that merely agreed on a circle would be a different
/// measurement on anything else.
///
/// `screw` is the chain's segment-of-station map and the path whose
/// [`SweepPath::screw_axes`] it indexes, passed only by the `Rigid` placement.
/// A station on a HELIX segment is framed off the helix's own axis instead of
/// transported (see [`screw_reference`]); every other station takes the RMF step
/// from its predecessor, so the two compose across a joint in either direction.
///
/// The third value is the ring's CLOSURE report: `None` for an open run, zero
/// for a planar ring, and the measured holonomy with the counter-twist that
/// closes it for a spatial one — whose axes are returned already twisted.
///
/// `requested` is a RING's requested twist, already known to close
/// ([`closing_twist`]). It is laid down in the ring's frames over the whole lap,
/// the wrap chord included, together with any counter-twist, so the seam span
/// rolls at the rate every other span does. An OPEN run ignores it: its twist is
/// laid down by the placement, over the stations it has.
fn station_frames(
    points: &[Vec3],
    tangents: &[Vec3],
    closure: RingFrame,
    screw: Option<(&[usize], &SweepPath)>,
    requested: ClosingTwist,
) -> Result<(Vec<Vec3>, Vec<Vec3>, Option<SweepClosure>), KernelRefusal> {
    match closure {
        RingFrame::Open => {
            let (r_axes, s_axes) = transported_frames(points, tangents, screw)?;
            Ok((r_axes, s_axes, None))
        }
        RingFrame::Planar(normal) => {
            let mut r_axes = Vec::with_capacity(points.len());
            let mut s_axes = Vec::with_capacity(points.len());
            for tangent in tangents {
                // Project out any tangent component and re-normalize, so the frame is
                // orthonormal even where the fitted plane and the sampled tangent
                // disagree in the last bits.
                let r = normal
                    .sub(tangent.scale(normal.dot(*tangent)))
                    .normalized()
                    .map_err(|_| {
                        KernelRefusal::internal(
                            KernelStage::Classify,
                            "planar_frame",
                            "sweepSolid: the closed path's plane normal is parallel to its tangent",
                        )
                    })?;
                s_axes.push(tangent.cross(r).normalized().or_refuse(KernelStage::Classify, "normalized")?);
                r_axes.push(r);
            }
            if requested.twist != 0.0 {
                roll_over_the_lap(points, &mut r_axes, &mut s_axes, requested.twist)?;
            }
            Ok((
                r_axes,
                s_axes,
                Some(SweepClosure {
                    holonomy: 0.0,
                    requested_twist: requested.twist,
                    applied_twist: requested.twist,
                    curve_shift: requested.shift,
                }),
            ))
        }
        RingFrame::Spatial => {
            let (r_axes, s_axes, closure) =
                ring_closure_frames(points, tangents, screw, requested)?;
            Ok((r_axes, s_axes, Some(closure)))
        }
    }
}

/// The frames of a closed SPATIAL ring: the transported frame, counter-twisted
/// by its own holonomy so the section meets itself (see [`SweepClosure`]).
///
/// MEASURED, not assumed. The run is transported ONE STEP FURTHER than an open
/// run goes — from the last station back to station 0, by the very rule every
/// other step takes (the RMF, or a helix segment's screw frame under `Rigid`) —
/// and the returned axis is compared with station 0's: the signed angle between
/// them, right-handed about the start tangent, is the holonomy `h`.
///
/// Then station `k`, at chord-length `s_k` round a lap of chord length `L`
/// (the wrap chord included), is rolled about its tangent by `φ_k = −h·s_k/L`.
/// Rolling about the tangent commutes with the transport step (each step is a
/// rotation carrying one tangent onto the next, and it carries a roll about the
/// first onto the same roll about the second), so the frame transported from
/// the last twisted station across the wrap arrives rolled by
/// `h − h·s_{n−1}/L`, and station 0 sits `−h·(L − s_{n−1})/L` further on — the
/// same rate as every other span. The ring closes with no seam to skin.
///
/// No new station law: the closed lane already floors its budget at 16 stations
/// per quarter turn of the path's TURNING, and a closed curve turns at least
/// `2π` (Fenchel), so a lap carries at least 64 stations. The largest roll this
/// lays down is `π`, under 3° a station — finer than the 16 per quarter turn a
/// requested twist gets.
///
/// A REQUESTED twist `T` that closes ([`closing_twist`]) rides on the same rate:
/// each station rolls by `(T − h)·s_k/L`, so the section comes back from the lap
/// rolled by `T` alone, onto itself. [`sweep_profile_along_chain`] sizes the
/// station budget for `|T| + π`.
fn ring_closure_frames(
    points: &[Vec3],
    tangents: &[Vec3],
    screw: Option<(&[usize], &SweepPath)>,
    requested: ClosingTwist,
) -> Result<(Vec<Vec3>, Vec<Vec3>, SweepClosure), KernelRefusal> {
    let stations = points.len();
    if stations < 2 {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "station_run",
            "sweepSolid: need at least 2 stations",
        ));
    }
    // The lap with its wrap: station 0 again, arriving as the LAST segment's end.
    let mut lap_points = points.to_vec();
    lap_points.push(points[0]);
    let mut lap_tangents = tangents.to_vec();
    lap_tangents.push(tangents[0]);
    let lap_segments: Option<Vec<usize>> = screw.map(|(segments, _)| {
        let mut extended = segments.to_vec();
        extended.push(segments[stations - 1]);
        extended
    });
    let lap_screw = match (&lap_segments, screw) {
        (Some(segments), Some((_, path))) => Some((segments.as_slice(), path)),
        _ => None,
    };
    let (mut r_axes, mut s_axes) = transported_frames(&lap_points, &lap_tangents, lap_screw)?;
    let returned = r_axes.pop().expect("a lap has a wrap station");
    s_axes.pop();
    let holonomy = returned.dot(s_axes[0]).atan2(returned.dot(r_axes[0]));

    let applied = if requested.twist == 0.0 {
        -holonomy
    } else {
        requested.twist - holonomy
    };
    roll_over_the_lap(points, &mut r_axes, &mut s_axes, applied)?;
    Ok((
        r_axes,
        s_axes,
        SweepClosure {
            holonomy,
            requested_twist: requested.twist,
            applied_twist: applied,
            curve_shift: requested.shift,
        },
    ))
}

/// Roll a ring's station frames about their tangents by `roll` over the lap,
/// linearly in chord length with the WRAP chord included: station `k` at `s_k`
/// of a lap of length `L` rolls by `roll·s_k/L`.
fn roll_over_the_lap(
    points: &[Vec3],
    r_axes: &mut [Vec3],
    s_axes: &mut [Vec3],
    roll: f64,
) -> Result<(), KernelRefusal> {
    let stations = points.len();
    let mut travelled = Vec::with_capacity(stations);
    let mut length = 0.0;
    for index in 0..stations {
        travelled.push(length);
        length += points[(index + 1) % stations].sub(points[index]).length();
    }
    if length <= 0.0 {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "lap_length",
            "sweepSolid: the closed path has zero length",
        ));
    }
    for station in 0..stations {
        let phi = roll * travelled[station] / length;
        let (sin_phi, cos_phi) = phi.sin_cos();
        let (r, s) = (r_axes[station], s_axes[station]);
        r_axes[station] = r.scale(cos_phi).add(s.scale(sin_phi));
        s_axes[station] = s.scale(cos_phi).sub(r.scale(sin_phi));
    }
    Ok(())
}

/// The TRANSPORTED frames of a station run: seeded on an arbitrary perpendicular
/// to the first tangent and carried station to station — by the RMF, or by a
/// helix segment's screw frame under `Rigid` (see [`station_frames`]).
fn transported_frames(
    points: &[Vec3],
    tangents: &[Vec3],
    screw: Option<(&[usize], &SweepPath)>,
) -> Result<(Vec<Vec3>, Vec<Vec3>), KernelRefusal> {
    let stations = points.len();
    let mut r_axes = Vec::with_capacity(stations);
    let mut s_axes = Vec::with_capacity(stations);
    {
        let r0 = tangents[0].perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?; // any unit vector ⟂ T0
        s_axes.push(tangents[0].cross(r0).normalized().or_refuse(KernelStage::Classify, "normalized")?);
        r_axes.push(r0);
        // The helix segment the running roll was fixed for, and that roll.
        let mut roll: Option<(usize, f64)> = None;
        for index in 0..stations - 1 {
            let t_next = tangents[index + 1];
            let helix = screw.and_then(|(segments, path)| {
                let segment = segments[index + 1];
                path.screw_axes[segment].map(|axis| (segment, axis, path))
            });
            let r_next = match helix {
                None => rmf_step(
                    r_axes[index],
                    tangents[index],
                    t_next,
                    points[index + 1].sub(points[index]),
                )
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "frame_step",
                        format!("sweepSolid: frame degenerated at station {index}"),
                    )
                })?,
                Some((segment, axis, path)) => {
                    // The roll is fixed ONCE per helix segment, at the station it
                    // departs from — station 0, or the joint the previous segment
                    // arrived at — so the incoming frame continues without a snap.
                    let phi = match roll {
                        Some((fixed, phi)) if fixed == segment => phi,
                        _ => {
                            let tangent = tangents[index];
                            let reference =
                                screw_reference(axis, tangent, path.name(segment), index)?;
                            let r = r_axes[index];
                            let phi = tangent.dot(reference.cross(r)).atan2(reference.dot(r));
                            roll = Some((segment, phi));
                            phi
                        }
                    };
                    let reference = screw_reference(axis, t_next, path.name(segment), index + 1)?;
                    let (sin_phi, cos_phi) = phi.sin_cos();
                    reference
                        .scale(cos_phi)
                        .add(t_next.cross(reference).scale(sin_phi))
                }
            };
            s_axes.push(t_next.cross(r_next).normalized().or_refuse(KernelStage::Classify, "normalized")?);
            r_axes.push(r_next);
        }
    }
    Ok((r_axes, s_axes))
}

/// A HELIX station's reference axis: the screw axis with its tangent component
/// removed, `normalize(W − (W·T)T)`.
///
/// Why this and not the transported frame: a helix is carried into itself by
/// ONE rigid motion — the screw about its axis — and that motion fixes both the
/// axis and the tangent's angle to it, so the axis' projection onto the normal
/// plane is the same body-fixed direction at every station. A frame built on it
/// IS the screw, so `R_k = F_k·F_0ᵀ` carries the profile about the coil's axis
/// and up its pitch. A rotation-minimizing frame has no spin about the tangent,
/// while the screw's frame spins at the helix's torsion `τ`, so the RMF rolls
/// away from it by `∫τ ds` — the rotation the user saw. On a TAPERED helix the
/// tangent's angle to the axis drifts with the radius, and this frame keeps the
/// section square to the axis all the same, which is what a tapered coil is.
///
/// Undefined only where the path runs ALONG its own axis, which a helix of
/// positive radius never does; refused by name there rather than guessed.
fn screw_reference(axis: Vec3, tangent: Vec3, segment: &str, station: usize) -> Result<Vec3, KernelRefusal> {
    axis.sub(tangent.scale(axis.dot(tangent)))
        .normalized()
        .map_err(|_| {
            KernelRefusal::ill_posed(
                KernelStage::Classify,
                "screw_reference",
                format!(
                "sweepSolid: path segment '{segment}' runs along its own helix axis at station \
                 {station}, so the screw motion that carries the section has no direction \
                 about the path there"
                ),
            )
        })
}

/// The stations a path is sampled at, and what each one carries: the point, the
/// unit tangent, and the CURVATURE VECTOR `κ·N̂` — the path's own bend there,
/// measured from the curve rather than differenced off the station polyline,
/// because it is the number the tight-bend refusal quotes.
pub(crate) struct StationSamples {
    pub(crate) points: Vec<Vec3>,
    pub(crate) tangents: Vec<Vec3>,
    /// `κ·N̂` per station: its length is the curvature and its direction points
    /// at the centre of the osculating circle. Zero on a straight piece.
    pub(crate) curvatures: Vec<Vec3>,
    /// The path SEGMENT each station belongs to. A joint station is emitted by
    /// the segment ARRIVING at it, so it carries that segment's index.
    pub(crate) segments: Vec<usize>,
    /// Where each station sits for the span AFTER it: the segment it departs
    /// into and its parameter there ([`StationParam`]).
    pub(crate) params: Vec<StationParam>,
}

/// The CURVATURE VECTOR of a curve from its first two derivatives:
///
/// ```text
///     κ·N̂ = (r̈ − (r̈·T̂)T̂) / |ṙ|²,      T̂ = ṙ/|ṙ|
/// ```
///
/// the component of the second derivative perpendicular to the tangent over the
/// squared speed — the reparameterization-invariant form, so a non-uniformly
/// parameterized path reads its own geometric bend rather than its
/// parameterization's. A degree-1 piece has a zero second derivative
/// (`derivatives` pads past the degree), so a straight segment reads exactly
/// zero and needs no special case.
pub(crate) fn curvature_vector(first: Vec3, second: Vec3) -> Vec3 {
    let speed_squared = first.dot(first);
    if speed_squared <= 0.0 {
        return Vec3::default();
    }
    let tangent = first.scale(1.0 / speed_squared.sqrt());
    second
        .sub(tangent.scale(second.dot(tangent)))
        .scale(1.0 / speed_squared)
}

/// Which sampler asked for the stations — the two differ in their FLOOR and
/// in the words of the degenerate-tangent refusal, both pinned by callers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SamplerLane {
    /// The chain sweep: every segment floored at its own turning, 16 stations
    /// a quarter turn.
    Chain,
    /// The single-curve builders (`sweep_profile_along_path` and family): the
    /// budget [`path_turning_stations`] hands them, uniform in the parameter,
    /// exactly as they always sampled — no turning floor on top, so an
    /// untwisted single-curve sweep that reads inside the tolerance is
    /// bit-identical to the tree before refinement existed.
    SingleCurve,
}

/// Sample a CHAIN as ONE trajectory: the per-segment arc-length estimate, the
/// station budget it implies, and the stations themselves — then REFINE the
/// stations where the loft's own interpolant of them would stand off the path
/// by more than `tolerance` (`sweep_topology/stations.rs`).
///
/// Extracted from the chain sweep so the tight-bend INSTRUMENT
/// ([`sweep_bend_profile`]) measures the very stations the sweep lofts through,
/// rather than a second sampling that agrees with it today.
///
/// STATION BUDGET. `stations` are distributed across the segments in proportion
/// to estimated ARC LENGTH (chord sums over 16 samples per curve), floored at 2
/// per segment so a short fillet between two long lines still carries its
/// curvature, and — on the chain lane — at the segment's own TURNING at 16
/// stations per quarter turn, the density the twisted builder already uses.
/// That floor is the measured case behind the error rule: shared out by length
/// alone, a rounded rectangle's four short corner arcs got the 2-station floor
/// and the ring cut the corners, 6.2 % under `A · perimeter` on a 30 × 18 path
/// with `r = 4`; the turning floor took the same path to 4.8e-4. The floor is
/// kept as the STARTING budget on both lanes; the error rule below is what
/// takes the rest.
///
/// WHAT THE ERROR RULE REPLACES. Until 2026-09-30 an OPEN path had its straight
/// segments LIFTED to the density its turning segments were floored at
/// (`floor_total × straight length / curved length`), because a coil with a
/// straight lead of 2 stations bowed through the lead (2.7e-5 off Pappus on a
/// lead of 10, 5.0e-4 on 100). Measured on 2026-09-30, that lift put a pipe of
/// two 100-long legs round a 2-radius quarter bend at the 1024-station cap
/// (correct, at 25× the stations it needs), and REFUSED the same pipe at
/// `r = 1.5`, and any spatial variant, as "past the 1024-station cap" — a
/// legitimate sweep refused for its own budget. The rule below reads the bow
/// where it is and puts stations there; the lead of 100 gets a handful next to
/// the joint instead of a thousand along its length.
///
/// THE RULE. The loft interpolates each control-point column with one cubic
/// through the stations at shared chord parameters; the same fit through the
/// station POINTS is the loft's centre track. Per span, the track's distance
/// from the true path at the quarter points is measured
/// (`stations::spine_deviations`); a span over `tolerance` is split into
/// `ceil((deviation / tolerance)^(1/4))` equal parameter steps (the cubic's
/// `h⁴` law), and the run is re-measured. At most [`SWEEP_REFINEMENT_ROUNDS`]
/// rounds; a round that would pass the [`SWEEP_MAX_STATIONS`] cap refines the
/// worst spans as far as the cap allows and the budget records `at_cap` — the
/// sweep still builds and its report says the tolerance was not met.
/// `BREP_SWEEP_STATION_REFINEMENT=0` switches the rule off (the bite proof).
///
/// The shared endpoint of two adjacent segments is sampled ONCE, not twice — a
/// duplicated station would land in the RMF's coincident-step branch and waste
/// a section on zero advance. A CLOSED path drops one more station for the
/// same reason: its last station IS its first.
fn chain_stations(
    chain: &[NurbsCurve],
    closed: bool,
    lane: SamplerLane,
    stations: usize,
    tolerance: f64,
) -> Result<(StationSamples, StationBudget), KernelRefusal> {
    let position_tolerance = 1e-6;
    // --- Per-segment arc-length estimate, and the domain each one is sampled
    //     over. A degenerate domain is refused here rather than producing a
    //     silently collapsed station block.
    //
    //     Each segment's TURNING — the total angle its tangent sweeps — is measured
    //     too, because that, not its length, is what decides how many stations it
    //     needs (see the floor below).
    let mut domains = Vec::with_capacity(chain.len());
    let mut lengths = Vec::with_capacity(chain.len());
    let mut turning = Vec::with_capacity(chain.len());
    for (index, curve) in chain.iter().enumerate() {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        if (t1 - t0).abs() <= position_tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "path_domain",
                match lane {
                    SamplerLane::Chain => format!("sweepSolid: path segment {index} has a degenerate domain"),
                    SamplerLane::SingleCurve => "sweepSolid: path domain is degenerate".to_string(),
                },
            ));
        }
        let mut length = 0.0;
        let mut previous = curve.evaluate(t0).or_refuse(KernelStage::Collect, "evaluate")?;
        for sample in 1..=16 {
            let point = curve.evaluate(t0 + (t1 - t0) * sample as f64 / 16.0).or_refuse(KernelStage::Collect, "evaluate")?;
            length += point.sub(previous).length();
            previous = point;
        }
        domains.push([t0, t1]);
        lengths.push(length);
        // `segment_turning` is a stringly reading of the curve's unit tangents;
        // a failure there is a degenerate tangent, which is the path's fault.
        // The single-curve lane never reads it: its budget was decided by
        // [`path_turning_stations`] and a path it cannot read is refused by the
        // sampling below, in that lane's own words.
        turning.push(match lane {
            SamplerLane::Chain => segment_turning(curve, [t0, t1]).map_err(|error| {
                KernelRefusal::input(
                    KernelStage::Classify,
                    "path_tangent",
                    format!("sweepSolid: path tangent is degenerate on segment {index}: {error}"),
                )
            })?,
            SamplerLane::SingleCurve => 0.0,
        });
    }
    let total_length: f64 = lengths.iter().sum();
    if total_length <= position_tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "path_length",
            "sweepSolid: path chain has zero length",
        ));
    }

    // --- Station budget per segment: proportional to arc length, floored at 2
    //     (a segment needs its two ends) and at the segment's turning, and the
    //     remainder handed to the longest segment so the totals land exactly on
    //     `stations`.
    let floors: Vec<usize> = turning
        .iter()
        .map(|turn| ((turn / std::f64::consts::FRAC_PI_2) * 16.0).ceil() as usize)
        .collect();
    let mut budget: Vec<usize> = lengths
        .iter()
        .zip(&floors)
        .map(|(length, floor)| {
            let share = (stations as f64 * length / total_length).round() as usize;
            share.max(*floor).max(2)
        })
        .collect();
    // Each interior joint shares one station between its two segments, so the
    // emitted total is sum(budget) - (segments - 1). A CLOSED path drops its very
    // last station too — it is the first one — so it emits one fewer again.
    let shared = chain.len() - 1 + usize::from(closed);
    let emitted: usize = budget.iter().sum::<usize>() - shared;
    // The 2-per-segment floor means a chain of n segments emits at least n+1
    // stations, so a long enough chain cannot be held under the loft's station
    // cap by any distribution. The loft's interpolation solve is O(stations³)
    // per control column, so silently accepting such a path would look like a
    // hang. Refuse with the count instead — the same honesty the twist cap uses.
    if emitted > SWEEP_MAX_STATIONS {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "station_cap",
            format!(
            "sweepSolid: a {}-segment path needs at least {emitted} stations, past the \
             {SWEEP_MAX_STATIONS}-station cap the loft solve can carry; sweep it in fewer, \
             longer pieces",
            chain.len()
            ),
        ));
    }
    if emitted < stations {
        let longest = lengths
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(b.1))
            .map(|(index, _)| index)
            .unwrap_or(0);
        budget[longest] += stations - emitted;
    }

    // --- The parameters each segment is sampled at: uniform over its domain at
    //     its budget, then refined where the measured deviation says so.
    let mut per_segment: Vec<Vec<f64>> = domains
        .iter()
        .zip(&budget)
        .map(|([t0, t1], count)| {
            (0..*count)
                .map(|sample| t0 + (t1 - t0) * sample as f64 / (count - 1) as f64)
                .collect()
        })
        .collect();
    let initial = per_segment.iter().map(Vec::len).sum::<usize>() - shared;
    let mut budget = StationBudget {
        initial,
        stations: initial,
        rounds: 0,
        at_cap: false,
        rounds_exhausted: false,
    };
    let refine = tolerance.is_finite() && tolerance > 0.0 && super::stations::refinement_enabled();
    let mut samples = sample_chain(chain, closed, lane, &domains, &per_segment)?;
    if refine {
        loop {
            let deviations = super::stations::spine_deviations(chain, &samples.params, &samples.points, closed)
                .map_err(|error| {
                    KernelRefusal::internal(
                        KernelStage::Refine,
                        "station_deviation",
                        format!("sweepSolid: the station deviation could not be read: {error}"),
                    )
                })?;
            if deviations.iter().all(|deviation| *deviation <= tolerance) {
                break;
            }
            if budget.rounds >= SWEEP_REFINEMENT_ROUNDS {
                budget.rounds_exhausted = true;
                break;
            }
            let emitted = samples.points.len();
            let (inserted, at_cap) = super::stations::refine_parameters(
                chain,
                &mut per_segment,
                &samples.params,
                &deviations,
                tolerance,
                closed,
                emitted,
            )
            .map_err(|error| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "station_refinement",
                    format!("sweepSolid: the stations could not be refined: {error}"),
                )
            })?;
            budget.at_cap |= at_cap;
            if inserted == 0 {
                break;
            }
            budget.rounds += 1;
            samples = sample_chain(chain, closed, lane, &domains, &per_segment)?;
            if at_cap {
                break;
            }
        }
    }
    budget.stations = samples.points.len();
    Ok((samples, budget))
}

/// Sample the chain at `per_segment`'s parameters as ONE trajectory: the
/// point, unit tangent and curvature vector at every station, the segment it
/// arrived on, and where it sits ([`StationParam`]).
fn sample_chain(
    chain: &[NurbsCurve],
    closed: bool,
    lane: SamplerLane,
    domains: &[[f64; 2]],
    per_segment: &[Vec<f64>],
) -> Result<StationSamples, KernelRefusal> {
    let mut points: Vec<Vec3> = Vec::new();
    let mut tangents: Vec<Vec3> = Vec::new();
    let mut curvatures: Vec<Vec3> = Vec::new();
    let mut segments: Vec<usize> = Vec::new();
    let mut params: Vec<StationParam> = Vec::new();
    for (index, curve) in chain.iter().enumerate() {
        let [t0, _] = domains[index];
        let list = &per_segment[index];
        let count = list.len();
        // Skip the first sample of every segment after the first: that station
        // is the joint, already emitted as the previous segment's last. On a
        // CLOSED path the LAST segment's last sample is skipped too — it is
        // station 0, and a ring that emitted it twice would interpolate a
        // zero-length span across its own seam.
        let first = usize::from(index > 0);
        if first == 1 {
            // THE JOINT reads the TIGHTER of the two pieces that meet there. A
            // G1 joint is tangent-continuous, not curvature-continuous — a line
            // running into a cubic hands over at κ = 0 and the cubic starts at
            // its own spike — and the station is emitted once, by the piece that
            // ARRIVES. Left as that, a path whose tightest bend sits at the
            // START of a curved piece would quote the straight piece's κ = 0 and
            // the bend would go unnamed; on the wire harness's own span the two
            // ends are mirror images and only ONE of them was being read.
            let derivatives = curve.derivatives_small(t0, 2).or_refuse(KernelStage::Collect, "derivatives")?;
            let arriving = curvature_vector(derivatives[1], derivatives[2]);
            if let Some(departing) = curvatures.last_mut() {
                if arriving.length() > departing.length() {
                    *departing = arriving;
                }
            }
            // The joint station departs into THIS segment.
            if let Some(joint) = params.last_mut() {
                *joint = StationParam { segment: index, t: t0 };
            }
        }
        let last = count - usize::from(closed && index + 1 == chain.len());
        for (sample, &t) in list.iter().enumerate().take(last).skip(first) {
            let derivatives = curve.derivatives_small(t, 2).or_refuse(KernelStage::Collect, "derivatives")?;
            let tangent = derivatives[1].normalized().map_err(|_| {
                KernelRefusal::input(
                    KernelStage::Classify,
                    "path_tangent",
                    match lane {
                        SamplerLane::Chain => format!("sweepSolid: path tangent is degenerate on segment {index}"),
                        SamplerLane::SingleCurve => format!("sweepSolid: path tangent is degenerate at station {sample}"),
                    },
                )
            })?;
            points.push(derivatives[0]);
            tangents.push(tangent);
            curvatures.push(curvature_vector(derivatives[1], derivatives[2]));
            segments.push(index);
            params.push(StationParam { segment: index, t });
        }
    }
    // A CLOSED path's seam is a joint like any other, and station 0 is the only
    // one whose two sides are sampled at opposite ends of the loop: the last
    // segment's end is station 0, and it was dropped rather than emitted.
    if closed {
        if let (Some(curve), Some(domain)) = (chain.last(), domains.last()) {
            let derivatives = curve.derivatives_small(domain[1], 2).or_refuse(KernelStage::Collect, "derivatives")?;
            let arriving = curvature_vector(derivatives[1], derivatives[2]);
            if let Some(departing) = curvatures.first_mut() {
                if arriving.length() > departing.length() {
                    *departing = arriving;
                }
            }
        }
    }
    Ok(StationSamples {
        points,
        tangents,
        curvatures,
        segments,
        params,
    })
}

/// The ring frame's fixed reference axis: the path plane's own normal, oriented
/// once and reused at every station.
///
/// A planar path's rotation-minimizing frame IS the planar frame — the transport
/// has no component about the tangent, so the axis perpendicular to the plane is
/// carried unchanged the whole way round and comes back to itself EXACTLY. That
/// is what makes a closed planar sweep need no twist policy: the holonomy is
/// zero by construction, not by cancellation. Taking the plane normal as the
/// reference axis says so in the construction rather than hoping a
/// station-by-station transport returns to where it started.
///
/// The guard is the one case a fixed axis cannot serve: a tangent parallel to the
/// normal, which a planar path cannot have and a mis-fitted plane could.
fn ring_normal(normal: Vec3, tangents: &[Vec3]) -> Result<Vec3, KernelRefusal> {
    for (index, tangent) in tangents.iter().enumerate() {
        if tangent.cross(normal).length() < 0.5 {
            return Err(KernelRefusal::internal(
                KernelStage::Classify,
                "ring_normal",
                format!(
                "sweepSolid: the closed path's own plane normal is within 30° of its tangent \
                 at station {index}, so the path does not lie in that plane after all"
                ),
            ));
        }
    }
    normal.normalized().map_err(|_| {
        KernelRefusal::internal(
            KernelStage::Classify,
            "ring_normal_degenerate",
            "sweepSolid: the closed path's plane normal is degenerate",
        )
    })
}

/// The total TURNING of one path segment: the angle its unit tangent sweeps over
/// its own domain, summed over 16 uniform steps — or, where a step turns more
/// than [`TURNING_ALIAS_STEP`], over 16 steps per knot span.
///
/// Not the same question as its length, and the one a station budget has to
/// answer: a straight segment of any length is carried exactly by two stations,
/// while a quarter arc of any radius needs enough stations that the cubic
/// interpolant through them does not cut the corner. 16 samples is the density
/// every other estimate in this builder uses — and it ALIASES a segment that
/// turns more than a few times: on an 8-turn coil each step is half a turn, and
/// the angle between two tangents cannot read more than π however far the curve
/// turned between them. A step that large is the sign, so only then is the
/// segment re-read densely; everything that reads calmly at 16 keeps exactly the
/// reading, and so exactly the floor, it always had (a half circle reads a hair
/// over π there, and a floor of 33 — the budget the closed lane is pinned at).
fn segment_turning(curve: &NurbsCurve, domain: [f64; 2]) -> Result<f64, String> {
    let (total, widest) = turning_at(curve, domain, 16)?;
    if widest <= TURNING_ALIAS_STEP {
        return Ok(total);
    }
    let spans = curve.knots.windows(2).filter(|pair| pair[1] > pair[0]).count();
    Ok(turning_at(curve, domain, (spans * 16).clamp(64, 8192))?.0)
}

/// The widest tangent step [`segment_turning`] trusts at 16 samples: π/8, twice
/// the step of a quarter arc read at that density.
const TURNING_ALIAS_STEP: f64 = std::f64::consts::PI / 8.0;

/// `(total turning, widest single step)` over `steps` uniform steps.
fn turning_at(curve: &NurbsCurve, domain: [f64; 2], steps: usize) -> Result<(f64, f64), String> {
    let [t0, t1] = domain;
    let mut total = 0.0;
    let mut widest: f64 = 0.0;
    // A stationary sample — a cusp in the parameterization, which an involute
    // flank's Hermite fit carries by construction — still has a direction of
    // travel, and the turning is a question about directions.
    let mut previous = curve.unit_tangent(t0, t0, t1)?;
    for sample in 1..=steps {
        let tangent = curve.unit_tangent(t0 + (t1 - t0) * sample as f64 / steps as f64, t0, t1)?;
        let step = previous.dot(tangent).clamp(-1.0, 1.0).acos();
        total += step;
        widest = widest.max(step);
        previous = tangent;
    }
    Ok((total, widest))
}

/// ONE double-reflection step (Wang et al. 2008) of a rotation-minimizing frame:
/// carry the reference axis `r` from a station with tangent `t` to the next with
/// tangent `t_next` across the step vector `step`, then re-orthogonalize against
/// the new tangent to shed floating drift. `None` when the result degenerates.
///
/// Extracted so the frame the sections are placed on and the frame the holonomy
/// is measured with are the SAME transport rather than two copies of it.
fn rmf_step(r: Vec3, t: Vec3, t_next: Vec3, step: Vec3) -> Option<Vec3> {
    let c1 = step.dot(step);
    let candidate = if c1 <= 1e-18 {
        // Coincident stations: carry the reference axis forward unchanged.
        r
    } else {
        // First reflection across the plane bisecting the step vector.
        let reflected_r = r.sub(step.scale(2.0 / c1 * step.dot(r)));
        let reflected_t = t.sub(step.scale(2.0 / c1 * step.dot(t)));
        // Second reflection across the plane bisecting the tangents.
        let v2 = t_next.sub(reflected_t);
        let c2 = v2.dot(v2);
        if c2 <= 1e-18 {
            reflected_r
        } else {
            reflected_r.sub(v2.scale(2.0 / c2 * v2.dot(reflected_r)))
        }
    };
    candidate
        .sub(t_next.scale(candidate.dot(t_next)))
        .normalized()
        .ok()
}

// ===========================================================================
// The TIGHT BEND: a section carried through a bend tighter than its own reach
// ===========================================================================

/// How many points each profile curve contributes to the section's REACH.
///
/// The reach has to be a WITNESS — a point of the section that really does turn
/// back — so it is read off the curve itself and never off the control hull:
/// `make_arc` places a half-circle's middle controls at `r√2`, so a hull bound
/// would refuse a round section 41 % early. Sampling under-reads instead, which
/// is the safe direction: at 64 samples per curve a circle's support is short by
/// `r(1 − cos(π/128))` = 3.0e-4·r, so the guard can only ever refuse LATE, and a
/// bend that slips past it still meets the fold acceptance at the loft's exit.
const BEND_SECTION_SAMPLES: usize = 64;

/// One station's tight-bend reading: what the path does there and how far the
/// section reaches into it.
///
/// The quantity that decides the sweep is [`BendStation::ratio`] — `reach·κ`,
/// the same `ρκ` the fillet's canal surface folds at. A section point offset `d`
/// towards the centre of the osculating circle advances `(1 − d·κ)` per unit of
/// path, so at `d·κ = 1` it stops and past it it goes BACKWARDS: the swept
/// envelope has turned back through itself and the solid is the envelope trimmed
/// at that self-intersection, not the loft through the crossing sections.
#[derive(Debug, Clone, Copy)]
pub struct BendStation {
    /// The station's index in the sweep's own station list.
    pub station: usize,
    /// Where it is on the path.
    pub point: Vec3,
    /// The path's curvature `κ` there (zero on a straight piece).
    pub curvature: f64,
    /// The path's radius of curvature `ρ = 1/κ`, `f64::INFINITY` where straight.
    pub radius: f64,
    /// How far the placed section reaches towards the centre of the bend —
    /// its support in the curvature-normal direction. Negative when the whole
    /// section sits on the OUTSIDE of the bend, zero where the path is straight.
    pub reach: f64,
    /// `reach·κ`. `< 1` builds; `≥ 1` folds.
    pub ratio: f64,
}

/// The profile's sample points in the placement frame's own 2D coordinates
/// `(a, b) = ((x − origin)·pu, (x − origin)·pv)`.
///
/// Sampled ONCE: `Transplant` places the same 2D shape at every station
/// (`world = station + rᵢ·a + sᵢ·b`), so a station's reach is a pair of dot
/// products against the precomputed cloud rather than a re-evaluation of the
/// placed curves.
fn section_offsets(
    profile: &[NurbsCurve],
    placement: &ProfileAnchor,
) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    let ProfileAnchor { origin, pu, pv, .. } = *placement;
    let mut offsets = Vec::with_capacity(profile.len() * BEND_SECTION_SAMPLES);
    for curve in profile {
        let [start, end] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        for sample in 0..BEND_SECTION_SAMPLES {
            let t = start + (end - start) * sample as f64 / BEND_SECTION_SAMPLES as f64;
            let local = curve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?.sub(origin);
            offsets.push((local.dot(pu), local.dot(pv)));
        }
    }
    Ok(offsets)
}

/// The `ρκ` profile: one [`BendStation`] per station, in station order.
///
/// `axes` are the frames the sections are PLACED on (twist included), because
/// the reach is the section's support in the curvature-normal direction and a
/// non-round section rolled about the path presents a different support there.
fn bend_stations(
    points: &[Vec3],
    curvatures: &[Vec3],
    axes: &[(Vec3, Vec3)],
    offsets: &[(f64, f64)],
) -> Vec<BendStation> {
    (0..points.len())
        .map(|station| {
            let curvature = curvatures[station].length();
            if curvature <= 0.0 {
                return BendStation {
                    station,
                    point: points[station],
                    curvature: 0.0,
                    radius: f64::INFINITY,
                    reach: 0.0,
                    ratio: 0.0,
                };
            }
            // The curvature vector is perpendicular to the tangent by
            // construction, so its components on (r, s) are the whole of it.
            let normal = curvatures[station].scale(1.0 / curvature);
            let (r_axis, s_axis) = axes[station];
            let (nu, nv) = (normal.dot(r_axis), normal.dot(s_axis));
            let reach = offsets
                .iter()
                .map(|(a, b)| a * nu + b * nv)
                .fold(f64::NEG_INFINITY, f64::max);
            BendStation {
                station,
                point: points[station],
                curvature,
                radius: 1.0 / curvature,
                reach,
                ratio: reach * curvature,
            }
        })
        .collect()
}

/// The `ρκ` profile a TRANSPLANT sweep of `profile` along `path` would carry at
/// `stations` stations — the instrument behind the refusal below, exposed so a
/// caller (or a probe) can measure a path BEFORE asking for the solid.
///
/// Reads the very stations the sweep's tight-bend refusal reads: the FLOOR
/// budget, before the error-driven refinement (`chain_stations`) adds any —
/// the same sampling, the same rotation-minimizing frame. A bend too tight
/// is decided and quoted on the floor stations so that a path which folds
/// is refused before stations are spent refining it, and so that the
/// refusal's numbers do not move with the tolerance. Untwisted, which is the
/// lane every caller of [`sweep_profile_along_chain_with_stations`] is in.
pub fn sweep_bend_profile(
    profile: &[NurbsCurve],
    path: &SweepPath,
    stations: usize,
) -> Result<Vec<BendStation>, KernelRefusal> {
    let placement = resolve_placement(profile, None, SectionPlacement::Transplant)?;
    let (samples, _) = chain_stations(
        path.curves.as_slice(),
        path.closed,
        SamplerLane::Chain,
        stations.clamp(2, 1024),
        0.0,
    )?;
    floor_bend_stations(profile, &placement, path, &samples)
}

/// The `ρκ` profile of a TRANSPLANT sweep over `samples`, on the untwisted
/// ring or open frame those stations take — what [`sweep_bend_profile`]
/// reports and what the chain sweep refuses on before it refines.
fn floor_bend_stations(
    profile: &[NurbsCurve],
    placement: &ProfileAnchor,
    path: &SweepPath,
    samples: &StationSamples,
) -> Result<Vec<BendStation>, KernelRefusal> {
    let closure = ring_frame(path, &samples.tangents)?;
    let untwisted = ClosingTwist {
        twist: 0.0,
        shift: 0,
    };
    let (r_axes, s_axes, _) = station_frames(&samples.points, &samples.tangents, closure, None, untwisted)?;
    let axes: Vec<(Vec3, Vec3)> = r_axes.into_iter().zip(s_axes).collect();
    let offsets = section_offsets(profile, placement)?;
    Ok(bend_stations(&samples.points, &samples.curvatures, &axes, &offsets))
}

/// A tight-bend refusal with the envelope lane's own measurement of WHICH
/// half of its one configuration the sweep missed appended, when there is
/// one — so the refusal names the boundary of what is built rather than
/// only the bend that is refused.
fn with_envelope_gap(refusal: KernelRefusal, envelope_gap: Option<&str>) -> KernelRefusal {
    match envelope_gap {
        Some(gap) if !gap.is_empty() => refusal.with_message(|refusal| {
            format!(
            "{refusal}. The swept envelope trimmed at its own self-intersection is built \
             for a CIRCLE carried along one circular planar arc, and this sweep is not \
             that: {gap}"
            )
        }),
        _ => refusal,
    }
}

/// The prefix every tight-bend refusal starts with, so a caller can CLASSIFY one
/// (the wire harness reports it as a bundle status) without matching free text.
pub const SWEEP_TIGHT_BEND_REFUSAL: &str = "sweepSolid: the path bends TIGHTER than the section";

/// The slug of the tight-bend refusal (`UnsupportedGeometry`), shared by its
/// one mint ([`tight_bend_refusal`]) and [`is_sweep_tight_bend`].
pub(crate) const SWEEP_TIGHT_BEND_WHAT: &str = "tight_bend";

/// Is this the sweep's tight-bend refusal ([`SWEEP_TIGHT_BEND_REFUSAL`])? Read
/// off the class and its slug, so a caller that decides on it (the wire
/// harness's bundle status) follows the refusal, not its wording.
pub(crate) fn is_sweep_tight_bend(refusal: &KernelRefusal) -> bool {
    matches!(
        &refusal.class,
        crate::RefusalClass::UnsupportedGeometry { what } if what == SWEEP_TIGHT_BEND_WHAT
    )
}

/// The refusal for a path that bends tighter than the section reaches, or `None`
/// when every station builds.
///
/// Quotes the WORST station rather than the first one scanned — the tightest
/// bend is the one the user has to ease, and a path can cross the bar in several
/// places (both ends of one Hermite span, typically).
fn tight_bend_refusal(profile: &[BendStation], stations: usize) -> Option<KernelRefusal> {
    let folded = profile.iter().filter(|station| station.ratio >= 1.0).count();
    if folded == 0 {
        return None;
    }
    let worst = profile
        .iter()
        .max_by(|a, b| a.ratio.total_cmp(&b.ratio))
        .expect("a folded station exists");
    // The solid exists — the envelope trimmed at its self-intersection — and
    // this lane does not build it: a named deferral.
    Some(KernelRefusal::unsupported(
        KernelStage::Classify,
        SWEEP_TIGHT_BEND_WHAT,
        format!(
        "{SWEEP_TIGHT_BEND_REFUSAL} at station {} of {stations} ({:.4}, {:.4}, {:.4}): the path's \
         radius of curvature there is {:.6} and the section reaches {:.6} into the bend, so \
         ρ·κ = {:.3} ≥ 1 ({folded} of {stations} stations do). A section carried through a bend \
         tighter than its own reach sweeps an envelope that passes through ITSELF — the solid \
         there is that envelope trimmed at its own self-intersection, which this lane does not \
         build: it skins the section between stations, and past ρ·κ = 1 the sections CROSS and \
         the wall folds. Ease the bend to a radius above {:.6}, or narrow the section until it \
         reaches no further than {:.6} (a round section of diameter {:.6})",
        worst.station,
        worst.point.x,
        worst.point.y,
        worst.point.z,
        worst.radius,
        worst.reach,
        worst.ratio,
        worst.reach,
        worst.radius,
        2.0 * worst.radius,
        ),
    ))
}

/// The part of the sweep that does not care HOW the stations were sampled:
/// validate the profile, distribute the twist over the sampled polyline,
/// propagate the rotation-minimizing frame, place a profile copy per station
/// and loft through them.
///
/// Split out so a MULTI-SEGMENT path can share it
/// ([`sweep_profile_along_chain_stations`]). The single-curve sampler above is
/// deliberately left calling this with its OWN sampling rather than being
/// re-expressed as a one-element chain: two code paths that agree today are not
/// the same thing as one code path, and the untwisted single-curve geometry is
/// pinned by volume tests that must not move.
///
/// The RMF is propagated over the station sequence as a whole, so a chain's
/// frame carries across a joint exactly as it carries across any other pair of
/// adjacent stations — there is no per-segment restart to reconcile.
///
/// `closure` turns the run into a RING: anything but [`RingFrame::Open`] says the
/// stations come back round to station 0 (which is therefore NOT repeated at the
/// end). `Planar(normal)` says the path lies in the plane of that normal, so the
/// reference axis is fixed rather than transported — a planar frame, whose
/// holonomy round the loop is zero by construction. `Spatial` transports the
/// frame and closes it with the counter-twist policy ([`SweepClosure`]). Either
/// way the sections skin through a CLOSED loft: one face per section curve, no
/// end caps, `V − E + F = 0`. `Open` is the open run: transported frame, two end
/// caps, and the refusal below for a trajectory that closes anyway.
///
/// `chain` is the station-to-segment map and the classified path, from the chain
/// sampler; the single-curve sampler has none. Only `Rigid` reads it, to frame a
/// HELIX segment's stations off its screw axis ([`screw_reference`]).
/// The sampled run's geometry and budget, carried into
/// [`sweep_sections_through_samples`] for the report it writes.
struct StationRun<'a> {
    chain: &'a [NurbsCurve],
    params: &'a [StationParam],
    budget: StationBudget,
    tolerance: f64,
}

#[allow(clippy::too_many_arguments)]
fn sweep_sections_through_samples(
    profile: &[NurbsCurve],
    points: &[Vec3],
    tangents: &[Vec3],
    curvatures: &[Vec3],
    twist_angle: f64,
    placement: ProfileAnchor,
    placement_mode: SectionPlacement,
    closure: RingFrame,
    envelope_gap: Option<&str>,
    chain: Option<(&[usize], &SweepPath)>,
    run: StationRun<'_>,
) -> Result<SweptChain, KernelRefusal> {
    let tolerance = 1e-6;
    let stations = points.len();
    if stations < 2 {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "station_run",
            "sweepSolid: need at least 2 stations",
        ));
    }

    let ProfileAnchor { origin, pu, pv, .. } = placement;

    // --- 2b. Twist distribution: cumulative ARC-LENGTH fractions over the
    //         sampled station polyline (chord sums), so the twist advances
    //         uniformly in space even on a non-uniformly parameterized path.
    //         Only computed when a twist is actually requested — the
    //         `twist_angle == 0.0` path must stay bit-identical to the
    //         pre-twist builder — and only on an OPEN run. These fractions stop at
    //         the last station, so a RING laid down by them would roll its wrap
    //         span at a different rate from every other; a ring rolls its twist
    //         in its frames instead, over the whole lap (§3).
    let twist_fractions: Option<Vec<f64>> = if twist_angle != 0.0 && !closure.is_ring() {
        let mut cumulative = vec![0.0; stations];
        let mut total = 0.0;
        for index in 1..stations {
            total += points[index].sub(points[index - 1]).length();
            cumulative[index] = total;
        }
        if total <= tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "path_length",
                "sweepSolid: path has zero length; cannot distribute the twist",
            ));
        }
        for length in &mut cumulative {
            *length /= total;
        }
        Some(cumulative)
    } else {
        None
    };

    // --- 3. The station frames.
    //
    //   OPEN: rotation-minimizing frames via the double-reflection method, seeded
    //   on an arbitrary perpendicular to T0 — arbitrary because `Rigid` cancels
    //   the seed out (`R_k = F_k·F_0ᵀ`) and `Transplant` has always taken whatever
    //   roll `perpendicular()` returned.
    //
    //   RING: the plane normal is the reference axis AT EVERY STATION. A planar
    //   path's RMF is that frame already — the transport has no component about
    //   the tangent — so this is not a different frame from the open lane's, it is
    //   the same frame written down in closed form instead of accumulated. Which
    //   matters for exactly one reason: written down, it returns to itself at the
    //   seam EXACTLY, so the ring's last section meets its first with no residual
    //   roll to skin away.
    //
    //   SPATIAL RING: the open lane's transported frame, carried one step further
    //   round the wrap to MEASURE the roll it returns with (the holonomy), and
    //   counter-twisted by that roll linearly in arc length so the lap's last span
    //   rolls at the same rate as every other and the ring meets itself
    //   (`ring_closure_frames`).
    //
    //   A REQUESTED twist on a RING is laid down in those frames too, over the
    //   whole lap with the wrap, on top of the counter-twist — but only a twist
    //   that closes: after the lap the section comes back rolled by it, so it must
    //   carry the section onto itself ([`closing_twist`], refused by name
    //   otherwise). A fractional turn on a symmetric section closes onto a
    //   DIFFERENT curve than it left, `shift` along, and the loft below is told.
    //
    //   RIGID on a HELIX segment: the helix's own screw frame (see
    //   `screw_reference`), which the RMF rolls away from by the helix's
    //   integrated torsion. `Transplant` keeps the RMF everywhere — it squares
    //   the section to the path and centres it there, and its callers (SWP, the
    //   tube, the harness) are not this contract.
    let screw = chain.filter(|_| placement_mode == SectionPlacement::Rigid);
    let requested = if closure.is_ring() && twist_angle != 0.0 {
        let (section, _) = place_section(profile, placement, placement_mode, points[0], tangents[0])?;
        ring_closing_twist(&section, points, tangents, twist_angle)?
    } else {
        ClosingTwist {
            twist: 0.0,
            shift: 0,
        }
    };
    let (r_axes, s_axes, _) = station_frames(points, tangents, closure, screw, requested)?;

    // --- 3b. An OPEN trajectory that comes back to where it started cannot be
    //         capped: the first and last sections land on top of each other, and
    //         the loft's own complaint about that ("end sections coincide") says
    //         nothing about the path. Name it here instead, with the two things
    //         that do work. The check is on the PATH's first and last station
    //         POINTS, which is enough: a path that comes back to where it started
    //         puts the two caps in the same place whatever direction it arrives
    //         from — coincident when it also arrives pointing the same way,
    //         interpenetrating when it does not, and neither is a solid worth
    //         building.
    //
    //         TWO CLOSURE NOTIONS COEXIST, and deliberately. A path whose ends
    //         meet within the chainer's join band is CLASSIFIED closed and swept
    //         as a ring above — that is the shape a user drew as a loop. This band
    //         is a thousandth of the distance TRAVELLED, a hundred times looser,
    //         and it catches the run that is nearly a loop without being one: a
    //         359.7° arc whose caps would interpenetrate rather than coincide. The
    //         first is a shape with a construction; the second is an accident, and
    //         it is still refused. On a circular arc the band is the last ~0.36°
    //         before closure, so a 359° sweep still builds.
    if !closure.is_ring() {
        let travel: f64 = points
            .windows(2)
            .map(|pair| pair[1].sub(pair[0]).length())
            .sum();
        let gap = points[stations - 1].sub(points[0]).length();
        if travel > tolerance && gap <= 1e-3 * travel {
            // A run that is nearly a loop without being one: the input does not
            // say whether it is a ring or an open sweep, so neither is chosen.
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "near_closed_path",
                format!(
                "sweepSolid: the path returns to where it started ({gap:.3e} apart after \
                 travelling {travel:.3e}), so the sweep's two end caps would land on top of each \
                 other. A closed path swept this way IS a revolution — build it with Revolve \
                 about the same axis, or close the path exactly so it sweeps as a capless ring, \
                 or sweep the run in two halves and union them"
                ),
            ));
        }
    }

    // --- 4. Place a transformed copy of the profile at each station, by the mode
    //        the caller asked for (see `SectionPlacement`).
    //
    //   TRANSPLANT maps the profile's own plane frame onto the station frame,
    //       world = P_k + r_k·(l·pu) + s_k·(l·pv),   l = x − origin
    //   which lands the anchor origin ON the path and the profile square to it.
    //   The profile's distance from the path and its angle to the path are both
    //   discarded — there is no term along the tangent to carry them.
    //
    //   RIGID applies the path's own motion to the profile where it stands,
    //       world = P_k + a·T_k + b·r_k + c·s_k,     (a,b,c) = (l·T_0, l·r_0, l·s_0)
    //                                                l = x − P_0
    //   which is `P_k + R_k·(x − P_0)` for `R_k = F_k·F_0ᵀ`, written in the frame
    //   basis rather than as a matrix. The tangent term `a·T_k` is exactly what
    //   TRANSPLANT drops, and carrying it is what preserves both the profile's
    //   offset from the path and its angle to it. At station 0 the frames
    //   coincide and the section IS the profile, so the START cap lies in the
    //   plane it was drawn in.
    let path_start = points[0];
    let (t0, r0, s0) = (tangents[0], r_axes[0], s_axes[0]);
    let rigid = placement_mode == SectionPlacement::Rigid;
    let mut sections: Vec<Vec<NurbsCurve>> = Vec::with_capacity(stations);
    // Only the RIGID guard below reads these; TRANSPLANT leaves them empty.
    let mut station_points: Vec<Vec<Vec3>> = Vec::with_capacity(if rigid { stations } else { 0 });
    let mut section_normals: Vec<Vec3> = Vec::with_capacity(if rigid { stations } else { 0 });
    let mut anchor_track: Vec<Vec3> = Vec::with_capacity(if rigid { stations } else { 0 });
    // The axes the section is actually PLACED on, twist included — what the
    // tight-bend guard below measures the section's reach in.
    let mut placed_axes: Vec<(Vec3, Vec3)> = Vec::with_capacity(stations);
    // The section's AFFINE placement per station, for the report: the image of
    // the anchor origin (the centre track) and the images of `pu` and `pv`.
    let mut centres: Vec<Vec3> = Vec::with_capacity(stations);
    let mut axes_1: Vec<Vec3> = Vec::with_capacity(stations);
    let mut axes_2: Vec<Vec3> = Vec::with_capacity(stations);
    for station in 0..stations {
        let station_origin = points[station];
        // Rotate the RMF axes about the tangent by the station's twist angle
        // (Rodrigues on vectors ⟂ the tangent: r' = r·cosφ + s·sinφ,
        // s' = s·cosφ − r·sinφ, since s = t × r and t × s = −r).
        let (ri, si) = match &twist_fractions {
            Some(fractions) => {
                let phi = twist_angle * fractions[station];
                let (sin_phi, cos_phi) = phi.sin_cos();
                let r = r_axes[station];
                let s = s_axes[station];
                (
                    r.scale(cos_phi).add(s.scale(sin_phi)),
                    s.scale(cos_phi).sub(r.scale(sin_phi)),
                )
            }
            None => (r_axes[station], s_axes[station]),
        };
        placed_axes.push((ri, si));
        let tk = tangents[station];
        let place = |x: Vec3| -> Vec3 {
            if rigid {
                let local = x.sub(path_start);
                station_origin
                    .add(tk.scale(local.dot(t0)))
                    .add(ri.scale(local.dot(r0)))
                    .add(si.scale(local.dot(s0)))
            } else {
                let local = x.sub(origin);
                station_origin
                    .add(ri.scale(local.dot(pu)))
                    .add(si.scale(local.dot(pv)))
            }
        };
        {
            let centre = place(origin);
            centres.push(centre);
            axes_1.push(place(origin.add(pu)).sub(centre));
            axes_2.push(place(origin.add(pv)).sub(centre));
        }
        if rigid {
            // The section's own normal takes the same rotation (a direction, so
            // `R_k` without the translation), and the anchor origin's track is
            // the profile's bulk motion for the near-parallel guard.
            let n0 = placement.normal;
            section_normals.push(
                tk.scale(n0.dot(t0))
                    .add(ri.scale(n0.dot(r0)))
                    .add(si.scale(n0.dot(s0))),
            );
            anchor_track.push(place(origin));
            station_points.push(Vec::with_capacity(
                profile.iter().map(|c| c.control_points.len()).sum(),
            ));
        }
        let mut section = Vec::with_capacity(profile.len());
        for curve in profile {
            let control_points = curve
                .control_points
                .iter()
                .map(|point| {
                    let weight = point.w;
                    let euclidean = Vec3::new(point.x / weight, point.y / weight, point.z / weight);
                    let world = place(euclidean);
                    if rigid {
                        station_points[station].push(world);
                    }
                    Vec4 {
                        x: world.x * weight,
                        y: world.y * weight,
                        z: world.z * weight,
                        w: weight,
                    }
                })
                .collect();
            section.push(NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                control_points,
            ).or_refuse(KernelStage::Fragment, "curve_new")?);
        }
        sections.push(section);
    }

    // --- 4b. RIGID only: refuse a sweep whose sections fold through each other.
    //
    // TRANSPLANT keeps every section square to the path, so it can only fold
    // where the path's curvature radius drops below the profile's extent — the
    // case this builder has always documented as the caller's to avoid. RIGID
    // can fold for a second, much more reachable reason: a turning path carries
    // the profile around the TURN'S OWN AXIS, so a profile straddling that axis
    // has one half advancing while the other retreats. That is not a tolerance
    // question, it is a sign question, so it is answered exactly.
    //
    // The placement map is AFFINE in `x`, so the advance is affine in `x` too and
    // its extremes over the section lie on the control points. Checking the hull
    // therefore bounds the advance over the curves AND over the region they
    // enclose — this is a proof, not a sampling.
    //
    // ON A RING the guard walks one step FURTHER: the wrap from the last station
    // back to station 0 is a step the sections take like any other, and it is the
    // one step an open run does not have. A ring whose wrap step folded while
    // every interior step advanced would otherwise pass. On a ring whose twist
    // lands each curve `shift` along, the wrap step's control points arrive on
    // that curve's: the flat control-point list of station 0, turned left by the
    // points of the `shift` curves before it (curves `j` and `j + shift` carry the
    // same number, which is what makes the turn a relabelling).
    let step_count = if closure.is_ring() {
        stations
    } else {
        stations - 1
    };
    let next_station = |station: usize| (station + 1) % stations;
    if rigid {
        let landing_offset: usize = profile[..requested.shift]
            .iter()
            .map(|curve| curve.control_points.len())
            .sum();
        let mut most_positive = 0.0f64;
        let mut most_negative = 0.0f64;
        let mut longest_step = 0.0f64;
        for station in 0..step_count {
            let normal = section_normals[station];
            let arriving = &station_points[next_station(station)];
            let turn = if next_station(station) == 0 { landing_offset } else { 0 };
            for (before, after) in station_points[station]
                .iter()
                .zip(arriving[turn..].iter().chain(&arriving[..turn]))
            {
                let step = after.sub(*before);
                longest_step = longest_step.max(step.length());
                let advance = step.dot(normal);
                most_positive = most_positive.max(advance);
                most_negative = most_negative.min(advance);
            }
        }
        // Scale-relative, and far below any geometric tolerance: this band only
        // has to separate a sign from floating noise, never a small motion from
        // a large one.
        let noise = 1e-9 * longest_step.max(tolerance);
        if most_positive > noise && most_negative < -noise {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "straddles_axis",
                "sweepSolid: the profile sweeps back through itself — part of it advances along \
                 the path while part of it retreats. A turning path carries the profile around \
                 the turn's own axis, so a profile that STRADDLES that axis folds into itself; \
                 move the profile clear of the axis, or sweep the run in pieces",
            ));
        }
        if most_positive <= noise && most_negative >= -noise {
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "no_advance",
                "sweepSolid: the path does not advance through the profile — it runs inside the \
                 profile plane, so the sweep encloses no volume",
            ));
        }
        // The near-parallel case, on the anchor origin's own track: the same 0.1
        // threshold `extrude_profile_brep` refuses a sliver at, so a STRAIGHT
        // path is accepted here exactly when the translational sweep of the same
        // profile and vector is. A station whose origin barely moves is skipped —
        // an origin sitting on the turn axis means the profile straddles it, and
        // the fold-back guard above has already spoken.
        for station in 0..step_count {
            let step = anchor_track[next_station(station)].sub(anchor_track[station]);
            let length = step.length();
            if length <= noise {
                continue;
            }
            if step.dot(section_normals[station]).abs() / length < 0.1 {
                return Err(KernelRefusal::input(
                    KernelStage::Classify,
                    "sliver",
                    "sweepSolid: the path is nearly parallel to the profile plane, which sweeps a \
                     sliver rather than a solid",
                ));
            }
        }
    }

    // --- 4c. TRANSPLANT: refuse a path that bends TIGHTER than the section
    //         reaches, before the loft rather than after it.
    //
    // A section square to the path, offset `d` towards the centre of the
    // osculating circle, advances `(1 − d·κ)` per unit of path: at `d·κ = 1` it
    // stands still and past it it RETREATS, so consecutive sections cross and
    // the wall the loft skins through them passes through itself. That is the
    // same `ρκ > 1` the fillet's canal surface folds at, and the same defect the
    // soundness acceptance catches at the loft's exit — but caught THERE it can
    // only name a face and two parameters, where here it can name the station,
    // the bend and the section that does not fit through it.
    //
    // RIGID is not measured here: it does not keep the section square to the
    // path, so `ρκ` is not its criterion, and §4b above is its guard.
    //
    // The reach is a WITNESS and the curvature is the PATH's own, so this refuses
    // only where a real section point really does turn back. What it does not see
    // is a bend that peaks BETWEEN two stations — the loft interpolates those,
    // and the acceptance at the exit remains the backstop.
    if !rigid && std::env::var("BREP_SWEEP_BEND_GUARD").as_deref() != Ok("0") {
        let offsets = section_offsets(profile, &placement)?;
        let profile_stations = bend_stations(points, curvatures, &placed_axes, &offsets);
        if let Some(refusal) = tight_bend_refusal(&profile_stations, stations) {
            // The ENVELOPE lane builds this shape exactly for ONE configuration
            // (a circle on one circular planar arc), and a caller that lands here
            // has missed it (`with_envelope_gap`). An untwisted chain sweep was
            // already asked this on its floor stations; this is the backstop
            // for the twisted and single-curve runs, and for a bend that peaks
            // between floor stations and shows on the refined ones.
            return Err(with_envelope_gap(refusal, envelope_gap));
        }
    }

    // --- 5. Loft through the swept sections.
    //
    //   OPEN: side walls plus two planar end caps.
    //
    //   RING: the CLOSED loft — the sections form a cycle, the last flowing back
    //   into the first through the same C² cyclic interpolation every interior
    //   span gets, so the seam is not a weld. One face per section curve and NO
    //   CAPS, which is the whole point: a ring has no ends to cap, and capping it
    //   would be two faces inside the material. The topology it emits is
    //   `c` corner vertices, `c` corner rings, `c` seam edges and `c` faces, so
    //   `V − E + F = c − 2c + c = 0` — the Euler characteristic of a torus, which
    //   is what a section carried once round a loop encloses.
    //   A RING whose requested twist lands each curve `shift` along skins each
    //   wall from its own curve into that one (`loft_profile_brep_closed_shifted`),
    //   so there is still one wall per section curve.
    //   The loft mints its own refusals; the wrap below keeps that class and
    //   prefixes the text.
    let solid = match closure {
        RingFrame::Planar(_) | RingFrame::Spatial => {
            // The loft mints its own refusals; its class rides through the
            // sweep's wrap (a station cap stays a deferral, the acceptance's
            // fold stays an UnsoundResult).
            crate::loft_profile_brep_closed_shifted(&sections, requested.shift).map_err(|error| {
                error.with_message(|error| {
                    format!("sweepSolid: closed loft through swept sections failed: {error}")
                })
            })?
        }
        RingFrame::Open => loft_profile_brep(&sections).map_err(|error| {
            error.with_message(|error| {
                format!("sweepSolid: loft through swept sections failed: {error}")
            })
        })?,
    };

    // --- 6. The MEASURED BOUND of what was just built
    //        (`sweep_topology/stations.rs`): the loft's own interpolants of the
    //        centre track and the section axes against the true path and the
    //        transported frame, and the built volume by quadrature against the
    //        kinematic sweep's. Read off the very sections the loft skinned.
    let moments = super::stations::section_moments(profile, origin, pu, pv).map_err(|error| {
        KernelRefusal::internal(
            KernelStage::Refine,
            "section_moments",
            format!("sweepSolid: the section's area could not be read: {error}"),
        )
    })?;
    let (pu_in_frame, pv_in_frame) = if rigid {
        ([pu.dot(t0), pu.dot(r0), pu.dot(s0)], [pv.dot(t0), pv.dot(r0), pv.dot(s0)])
    } else {
        ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    };
    let report = super::stations::station_report(&super::stations::PlacedRun {
        chain: run.chain,
        params: run.params,
        points,
        tangents,
        axes: &placed_axes,
        centres: &centres,
        e1: &axes_1,
        e2: &axes_2,
        pu_in_frame,
        pv_in_frame,
        closed: closure.is_ring(),
        sections: &sections,
        moments,
        reach: super::stations::section_reach(profile, origin),
        shift: requested.shift,
        budget: run.budget,
        tolerance: run.tolerance,
    })?;
    Ok(SweptChain {
        solid,
        report: Some(report),
    })
}

/// Helical sweep (§5.7): sweep a CLOSED PLANAR profile loop along a helix of
/// `helix_radius` about the axis through `axis_origin` with direction
/// `axis_direction`, rising `pitch` per revolution for `turns` revolutions.
///
/// Sample a helix — `turns` revolutions about `axis_direction` from
/// `axis_origin`, starting at angle `start_angle` (radians, measured from the
/// axis frame's `u`) and rising `pitch` per turn, the radius running linearly
/// from `start_radius` to `end_radius` — uniformly in angle: 64 stations per
/// turn (at least 9, at most 4097 in total, so beyond 64 turns the per-turn
/// density thins). Returns the points and their chord parameters `s ∈ [0,1]`
/// (uniform-in-angle IS chord-length for a constant-radius helix, which is
/// what the averaged-knot interpolation assumes). `left_handed` winds the
/// angle the other way. The axis frame is `w = axis`, `u` = the component of
/// `reference` perpendicular to the axis (so angle zero points AT the
/// reference — the helix feature's local +X, or its start point), falling
/// back to `w.perpendicular()` when there is no usable reference, and
/// `v = w × u`. Shared by [`sweep_profile_helix`] and the HX feature, so the
/// coil a sweep builds and the edge a helix feature publishes agree exactly.
///
/// Errors on a degenerate axis, non-finite inputs, `turns ≤ 0`, more than 256
/// turns, a negative radius or pitch, or a helix that is a single point (both
/// radii AND the pitch zero).
#[allow(clippy::too_many_arguments)]
pub fn helix_sample_points(
    axis_origin: Vec3,
    axis_direction: Vec3,
    reference: Option<Vec3>,
    start_radius: f64,
    end_radius: f64,
    pitch: f64,
    turns: f64,
    start_angle: f64,
    left_handed: bool,
) -> Result<(Vec<Vec3>, Vec<f64>), KernelRefusal> {
    use std::f64::consts::TAU;
    let w = axis_direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "helix_axis",
            "helix: axis direction is degenerate",
        )
    })?;
    for (name, value) in [
        ("radius", start_radius),
        ("end radius", end_radius),
        ("pitch", pitch),
        ("turns", turns),
        ("start angle", start_angle),
    ] {
        if !value.is_finite() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "helix_finite",
                format!("helix: {name} must be a finite number"),
            ));
        }
    }
    if start_radius < 0.0 || end_radius < 0.0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_radius",
            "helix: radius must not be negative",
        ));
    }
    if pitch < 0.0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_pitch",
            "helix: pitch must not be negative",
        ));
    }
    if turns <= 0.0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_turns",
            "helix: turns must be positive",
        ));
    }
    if turns > 256.0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_turns_limit",
            "helix: turns must be at most 256",
        ));
    }
    if start_radius == 0.0 && end_radius == 0.0 && pitch == 0.0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_point",
            "helix: zero radius and zero pitch describe a single point",
        ));
    }
    let u = match reference {
        Some(reference) => {
            let radial = reference.sub(w.scale(reference.dot(w)));
            radial
                .normalized()
                .or_else(|_| w.perpendicular())
                .or_refuse(KernelStage::Collect, "perpendicular")?
        }
        None => w.perpendicular().or_refuse(KernelStage::Collect, "perpendicular")?,
    };
    let v = w.cross(u).normalized().or_refuse(KernelStage::Collect, "normalized")?;
    let total_angle = turns * TAU;
    let height = pitch * turns;
    let sign = if left_handed { -1.0 } else { 1.0 };
    let count = ((turns * 64.0).ceil() as usize + 1).clamp(9, 4097);
    let mut points = Vec::with_capacity(count);
    let mut parameters = Vec::with_capacity(count);
    for index in 0..count {
        let s = index as f64 / (count - 1) as f64;
        let theta = start_angle + sign * total_angle * s;
        let radius = start_radius + (end_radius - start_radius) * s;
        points.push(
            axis_origin
                .add(u.scale(radius * theta.cos()))
                .add(v.scale(radius * theta.sin()))
                .add(w.scale(height * s)),
        );
        parameters.push(s);
    }
    Ok((points, parameters))
}

/// [`helix_sample_points`] fitted as ONE cubic curve (global interpolation —
/// `interpolate_curve`, the machinery every fitted path here uses). Cubic
/// interpolation error on a circle of radius R with node spacing Δθ is
/// ≈ R·Δθ⁴/384: ~2·10⁻⁷·R at 64 stations per turn.
#[allow(clippy::too_many_arguments)]
pub fn fit_helix_curve(
    axis_origin: Vec3,
    axis_direction: Vec3,
    reference: Option<Vec3>,
    start_radius: f64,
    end_radius: f64,
    pitch: f64,
    turns: f64,
    start_angle: f64,
    left_handed: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    let (points, parameters) = helix_sample_points(
        axis_origin,
        axis_direction,
        reference,
        start_radius,
        end_radius,
        pitch,
        turns,
        start_angle,
        left_handed,
    )?;
    interpolate_curve(&points, 3, &parameters).or_refuse(KernelStage::Refine, "interpolate")
}

/// The helix is transcendental, not exactly NURBS-representable, so the path
/// is FIT: points (R·cosθ, R·sinθ, pitch·θ/2π) in the axis frame are sampled
/// uniformly in θ and globally interpolated with a cubic (`interpolate_curve`
/// — the same machinery every other fitted path here uses).  DENSITY: 64
/// samples per turn, capped at 1025 total nodes.  Cubic interpolation error
/// on a circle of radius R with node spacing Δθ is ≈ R·Δθ⁴/384: ~2·10⁻⁷·R at
/// 64/turn, and still ~6·10⁻⁵·R at the cap's worst case (16/turn at the
/// 64-turn limit) — orders below any profile a caller could sweep without
/// self-intersecting.  Uniform-in-θ parameters are exact chord-length for a
/// helix (constant speed), which is what the averaged-knot interpolation
/// assumes.
///
/// The fitted path then drives the EXISTING path-sweep core with 32 stations
/// per turn (min 32, capped at 1024 — the loft's dense interpolation solve is
/// O(stations³) per control column, so the cap trades per-turn density, never
/// correctness, at high turn counts).
///
/// GUARDS beyond the path sweep's own: the path sweep documents
/// self-intersection as caller responsibility, so the helix variant — which
/// knows its curvature analytically — rejects the two garbage modes itself:
///   • fold-over: the helix curvature radius (R² + c²)/R (c = pitch/2π) must
///     exceed the profile's max extent about its centroid, or the tube folds
///     through itself on the inner side (the torus tube-radius > major-radius
///     failure, pitch-relaxed);
///   • coil collision (turns ≥ 1): the normal gap between consecutive coils,
///     pitch·2πR/√((2πR)² + pitch²), must exceed the profile diameter.
/// Both use the profile's max sample distance from its centroid — conservative
/// for asymmetric profiles (extent in a harmless direction still counts), but
/// a false reject beats silent garbage.
///
/// SHALLOW COILS BUILD. They did not until 2026-09-08: the loft measured its
/// advance by the END-TO-END chord, which on a whole-turn helix is purely
/// AXIAL while both caps face nearly tangentially, so `|n·chord|` fell under
/// the loft's 0.1 sliver band and everything with pitch ≲ 0.63·R — most
/// springs — was refused. The loft measures the advance at each END now
/// (`loft_topology::basic::advance_from`), which is the direction the tube
/// actually grows through the cap it is orienting, so the band only ever
/// catches a real sliver.
pub fn sweep_profile_helix(
    profile: &[NurbsCurve],
    axis_origin: Vec3,
    axis_direction: Vec3,
    helix_radius: f64,
    pitch: f64,
    turns: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use std::f64::consts::TAU;

    // --- 1. Validate the helix parameters with honest errors.
    let w = axis_direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "helix_axis",
            "sweep_profile_helix: axis direction is degenerate",
        )
    })?;
    if !(helix_radius.is_finite() && helix_radius > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_radius",
            "sweep_profile_helix: helix radius must be positive",
        ));
    }
    if !(pitch.is_finite() && pitch > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_pitch",
            "sweep_profile_helix: pitch must be positive",
        ));
    }
    if !(turns.is_finite() && turns > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "helix_turns",
            "sweep_profile_helix: turns must be positive",
        ));
    }
    // 64 turns bounds the loft's O(stations³) interpolation solve; beyond it
    // the station cap would silently degrade per-turn density anyway.
    if turns > 64.0 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "helix_turns_cap",
            "sweep_profile_helix: turns must be at most 64",
        ));
    }

    // --- 2. Profile extent about its centroid, sampled exactly like the path
    //        sweep derives its placement origin (boundary-sample mean), so the
    //        extent is measured about the point that actually rides the path.
    let mut samples = Vec::new();
    for curve in profile {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        for sample in 0..16 {
            samples.push(curve.evaluate(start + (end - start) * sample as f64 / 16.0).or_refuse(KernelStage::Collect, "evaluate")?);
        }
    }
    if !samples.is_empty() {
        let mut centroid = Vec3::default();
        for point in &samples {
            centroid = centroid.add(*point);
        }
        let origin = centroid.scale(1.0 / samples.len() as f64);
        let extent = samples
            .iter()
            .map(|point| point.sub(origin).length())
            .fold(0.0, f64::max);
        let c = pitch / TAU; // axial rise per radian
                             // Fold-over: profile reaches past the helix's center of curvature.
        let curvature_radius = (helix_radius * helix_radius + c * c) / helix_radius;
        if extent >= curvature_radius {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "helix_fold",
                format!(
                "sweep_profile_helix: profile extent {extent:.6} reaches the helix \
                 curvature radius {curvature_radius:.6}; the tube would fold through \
                 itself — increase the helix radius or pitch, or shrink the profile"
                ),
            ));
        }
        // Coil collision: only possible once the sweep spans a full revolution.
        if turns >= 1.0 {
            let circumference = TAU * helix_radius;
            let turn_length = (circumference * circumference + pitch * pitch).sqrt();
            let coil_gap = pitch * circumference / turn_length;
            if coil_gap <= 2.0 * extent {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "coil_collision",
                    format!(
                    "sweep_profile_helix: consecutive turns would self-intersect — \
                     coil gap {coil_gap:.6} does not clear the profile diameter {:.6}; \
                     increase the pitch or shrink the profile",
                    2.0 * extent
                    ),
                ));
            }
        }
    }

    // --- 3. Fit the helical path (see the density rationale in the fn docs) —
    //        the SAME sampler + fit the HX feature publishes as its edge.
    let path = fit_helix_curve(
        axis_origin,
        w,
        None,
        helix_radius,
        helix_radius,
        pitch,
        turns,
        0.0,
        false,
    )
    .map_err(|error| {
        error.with_message(|error| format!("sweep_profile_helix: helix path fit failed: {error}"))
    })?;

    // --- 4. Drive the existing sweep core; its (or the loft's) failures
    //        propagate with helix context prepended.
    let stations = ((turns * 32.0).ceil() as usize).clamp(32, 1024);
    // The helix builder places the profile ON the fitted helix (the coil's
    // section is drawn once and carried round), which is `Transplant` by
    // construction — the radius comes from the helix argument, not the profile's
    // own position.
    sweep_profile_along_path_stations(
        profile,
        &path,
        name,
        stations,
        0.0,
        None,
        SectionPlacement::Transplant,
        SWEEP_DEFAULT_TOLERANCE,
    )
    .map(|swept| swept.solid)
    .map_err(|error| error.with_message(|error| format!("sweep_profile_helix: {error}")))
}
