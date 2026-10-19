//! Stage attribution for the watertight tessellator, on the same switch the
//! boolean's `boolean.<stage>_ms` and the integrator's `mass.profile` lines
//! use: set `BREP_PROFILE` and one `tess.profile` line per
//! [`tessellate_brep_watertight`](super::tessellate_brep_watertight) call goes
//! to stderr. Disabled (one `Cell` read) otherwise.
//!
//! The counters are the ones that told the 2026-09-13 audit where a fine-chord
//! tessellation actually spends its time: the interior seed's per-station
//! rejection scans (`margin_tests`, `parity_tests` — both linear in the
//! boundary before the grid index) against the refinement passes.
use std::cell::Cell;
use web_time::Instant;

#[derive(Default, Clone, Copy)]
pub(in crate::watertight_tessellation) struct TessProfile {
    pub enabled: bool,
    pub faces: u64,
    pub edge_sampling_ms: f64,
    pub encode_ms: f64,
    pub decode_ms: f64,
    pub cdt_insert_ms: f64,
    pub cdt_constraint_ms: f64,
    pub validation_ms: f64,
    pub seed_ms: f64,
    pub refine_ms: f64,
    /// Grid stations the seed offered (`nu`×`nv` after the budget clamp).
    pub stations: u64,
    /// Stations that survived the margin/parity test and were inserted.
    pub stations_placed: u64,
    /// Point-to-segment distance tests the margin filter performed.
    pub margin_tests: u64,
    /// Ring-crossing tests the parity containment test performed.
    pub parity_tests: u64,
    /// Triangles visited by the host-triangle search.
    pub host_visits: u64,
    pub refine_passes: u64,
    pub splits: u64,
    /// Lawson flipping, from both the seed and the refinement.
    pub flip_ms: f64,
    pub flip_rounds: u64,
    pub flips: u64,
    /// The seed's host-triangle search.
    pub host_ms: f64,
    /// Protected-segment crossing tests a candidate flip performed.
    pub boundary_cross_tests: u64,
    /// The stages the planar multi-hole path spends its time in, which the
    /// seed/refine/flip split left out (the acceptance lane):
    /// `bridge_holes`, `surface_cdt`, `ear_clip` (outermost call), the
    /// planar certification and its direct CDT, and `polygon_is_simple`
    /// (called per candidate bridge) with its call count.
    pub bridge_ms: f64,
    pub cdt_ms: f64,
    pub earclip_ms: f64,
    pub certify_ms: f64,
    pub simple_ms: f64,
    pub simple_tests: u64,
    /// `lawson_flips` calls, and the sag evaluations its guard asked for.
    pub flip_calls: u64,
    pub flip_sag_evals: u64,
    /// `refine_interior`'s angular gate: calls and its time.
    pub angle_gate_ms: f64,
    pub angle_gate_tests: u64,
    /// Inside the flipper: building the edge-owner table and the protected
    /// segments, the per-round edge scan, applying a round's changes.
    pub flip_build_ms: f64,
    pub flip_scan_ms: f64,
    pub flip_apply_ms: f64,
    pub flip_edges_visited: u64,
    /// The per-face split (`tess.face`). `route` is the dispatch branch that
    /// built the face's boundary triangulation (`Route`), `cdt_used` whether
    /// `surface_cdt` owned it; the seed's directional curvatures, its grid
    /// after the budget clamp and whether the diagonal correction fired;
    /// `valid_planar_pslg` (inside `cdt_ms`) with its segment-pair tests; the
    /// planar certification's calls and centroid containment tests.
    pub route: u64,
    pub cdt_used: u64,
    pub seed_suu: f64,
    pub seed_svv: f64,
    pub seed_suv: f64,
    pub seed_grid_u: u64,
    pub seed_grid_v: u64,
    pub seed_diagonal_shrunk: u64,
    pub pslg_ms: f64,
    pub pslg_segment_tests: u64,
    pub certify_calls: u64,
    pub certify_centroid_tests: u64,
    /// `surface_cdt`'s own split (inside `cdt_ms`, after `pslg_ms`): the
    /// Delaunay build with its constraints, the inside flood, the manifold
    /// check.
    pub cdt_build_ms: f64,
    pub cdt_flood_ms: f64,
    pub cdt_check_ms: f64,
    /// Boundary-preserving tensor-grid lane for rectangular NURBS faces.
    pub rectangle_ms: f64,
    pub rectangle_faces: u64,
    pub rectangle_vertices: u64,
    pub rectangle_declines: u64,
}

/// Which branch of `tessellate_face_watertight` built a face's boundary
/// triangulation, as the `tess.face` line names it. The numbers are the
/// line's `route=` value; `name` is printed beside it.
#[derive(Clone, Copy)]
pub(in crate::watertight_tessellation) enum Route {
    None = 0,
    SphereAtlas = 1,
    PolePatch = 2,
    RecutWall = 3,
    SeamStraddle = 4,
    BiperiodicStraddle = 5,
    BiperiodicUnwrapped = 6,
    PeriodicTrim = 7,
    BiperiodicSeamBand = 8,
    PeriodicBand = 9,
    SpiralAnnulus = 10,
    SeamCrossing = 11,
    WavyBand = 12,
    WrapWall = 13,
    Generic = 14,
    FullBiperiodic = 15,
    PointedCone = 16,
    NurbsRectangle = 17,
}

impl Route {
    fn name(code: u64) -> &'static str {
        match code {
            0 => "none",
            1 => "sphere_atlas",
            2 => "pole_patch",
            3 => "recut_wall",
            4 => "seam_straddle",
            5 => "biperiodic_straddle",
            6 => "biperiodic_unwrapped",
            7 => "periodic_trim",
            8 => "biperiodic_seam_band",
            9 => "periodic_band",
            10 => "spiral_annulus",
            11 => "seam_crossing",
            12 => "wavy_band",
            13 => "wrap_wall",
            14 => "generic",
            15 => "full_biperiodic",
            16 => "pointed_cone",
            17 => "nurbs_rectangle",
            _ => "?",
        }
    }
}

/// Record the dispatch branch of the face being meshed.
pub(in crate::watertight_tessellation) fn tess_route(route: Route) {
    tess_profile(|p| p.route = route as u64);
}

thread_local! {
    static PROFILE: Cell<TessProfile> = const { Cell::new(TessProfile {
        enabled: false, faces: 0, edge_sampling_ms: 0.0,
        encode_ms: 0.0, decode_ms: 0.0, cdt_insert_ms: 0.0, cdt_constraint_ms: 0.0, validation_ms: 0.0, seed_ms: 0.0, refine_ms: 0.0,
        stations: 0, stations_placed: 0, margin_tests: 0, parity_tests: 0, host_visits: 0,
        refine_passes: 0, splits: 0, flip_ms: 0.0, flip_rounds: 0, flips: 0, host_ms: 0.0,
        boundary_cross_tests: 0, bridge_ms: 0.0, cdt_ms: 0.0, earclip_ms: 0.0, certify_ms: 0.0,
        simple_ms: 0.0, simple_tests: 0, flip_calls: 0, flip_sag_evals: 0, angle_gate_ms: 0.0,
        angle_gate_tests: 0, flip_build_ms: 0.0, flip_scan_ms: 0.0, flip_apply_ms: 0.0,
        flip_edges_visited: 0, route: 0, cdt_used: 0, seed_suu: 0.0, seed_svv: 0.0,
        seed_suv: 0.0, seed_grid_u: 0, seed_grid_v: 0, seed_diagonal_shrunk: 0, pslg_ms: 0.0,
        pslg_segment_tests: 0, certify_calls: 0, certify_centroid_tests: 0, cdt_build_ms: 0.0,
        cdt_flood_ms: 0.0, cdt_check_ms: 0.0,
        rectangle_ms: 0.0, rectangle_faces: 0, rectangle_vertices: 0, rectangle_declines: 0,
    }) };
}

/// Read-modify-write the thread-local profile when it is enabled; a no-op
/// (one flag read) otherwise.
pub(in crate::watertight_tessellation) fn tess_profile(update: impl FnOnce(&mut TessProfile)) {
    PROFILE.with(|cell| {
        let mut profile = cell.get();
        if profile.enabled {
            update(&mut profile);
            cell.set(profile);
        }
    });
}

pub(in crate::watertight_tessellation) fn tess_profile_enabled() -> bool {
    PROFILE.with(|cell| cell.get().enabled)
}

/// Milliseconds since `started` when profiling is on; `None` otherwise.
pub(in crate::watertight_tessellation) fn profile_started() -> Option<Instant> {
    tess_profile_enabled().then(Instant::now)
}

pub(in crate::watertight_tessellation) fn elapsed_ms(started: Option<Instant>) -> f64 {
    started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0)
}

/// Start a profiled tessellation: resets the counters and arms them when
/// `BREP_PROFILE` is set.
pub(in crate::watertight_tessellation) fn begin() -> Option<Instant> {
    begin_with(profile_switch())
}

/// Arm (or not) the counters, whatever the environment says.
fn begin_with(enabled: bool) -> Option<Instant> {
    PROFILE.with(|cell| {
        cell.set(TessProfile {
            enabled,
            ..TessProfile::default()
        })
    });
    enabled.then(Instant::now)
}

#[cfg(not(target_arch = "wasm32"))]
fn profile_switch() -> bool {
    std::env::var("BREP_PROFILE").is_ok()
}

/// No env vars in the browser; the shipped wasm path never arms this.
#[cfg(target_arch = "wasm32")]
fn profile_switch() -> bool {
    false
}

/// `BREP_PROFILE_TESS_FACES`: one `tess.face` line per face, from the
/// face-stride entry every caller (the display, the soundness scan) reaches.
/// The scan never passes through [`begin`], so with this switch the stride
/// entry arms the counters itself when nothing outer has, and prints the
/// whole call's `tess.profile` line at its end. Serial build only: under the
/// `parallel` feature a worker thread's counters are its own and unarmed.
#[cfg(not(target_arch = "wasm32"))]
pub(in crate::watertight_tessellation) fn face_lines_enabled() -> bool {
    std::env::var("BREP_PROFILE_TESS_FACES").is_ok()
}

#[cfg(target_arch = "wasm32")]
pub(in crate::watertight_tessellation) fn face_lines_enabled() -> bool {
    false
}

/// Arm the counters for a face-stride call that nothing outer profiles;
/// `None` when they are already armed (the orient entry's call) or the face
/// lines are off. The caller passes the value back to [`end`].
pub(in crate::watertight_tessellation) fn arm_for_face_lines() -> Option<Instant> {
    if face_lines_enabled() && !tess_profile_enabled() {
        begin_with(true)
    } else {
        None
    }
}

/// The counters as they stand, for a per-face delta.
pub(in crate::watertight_tessellation) fn snapshot() -> TessProfile {
    PROFILE.with(|cell| cell.get())
}

/// One `tess.face` line: the counters since `before` (one face's), the
/// face's kind, loops and coedges, its triangle count and wall time.
pub(in crate::watertight_tessellation) fn report_face(
    face_index: u32,
    face: &crate::topology::FaceRecord,
    triangles: usize,
    wall_ms: f64,
    before: TessProfile,
) {
    let now = snapshot();
    if !now.enabled {
        return;
    }
    let kind = face.surface.analytic().map_or("Nurbs", |analytic| analytic.kind_label());
    let coedges: usize = face.loops.iter().map(|record| record.coedges.len()).sum();
    eprintln!(
        "tess.face face={face_index} id={} kind={kind} deg={}x{} loops={} coedges={coedges} tris={triangles} \
         ms={wall_ms:.3} route={} cdt={} | seed_ms={:.3} refine_ms={:.3} flip_ms={:.3} bridge_ms={:.3} \
         cdt_ms={:.3} pslg_ms={:.3} earclip_ms={:.3} certify_ms={:.3} simple_ms={:.3} angle_gate_ms={:.3} \
         | suu={:.3e} svv={:.3e} suv={:.3e} grid={}x{} diagonal_shrunk={} stations={} placed={} \
         refine_passes={} splits={} flip_calls={} flip_rounds={} flips={} flip_edges_visited={} \
         flip_sag_evals={} angle_gate_tests={} simple_tests={} pslg_tests={} certify_calls={} \
         certify_centroid_tests={} cdt_build_ms={:.3} cdt_flood_ms={:.3} cdt_check_ms={:.3} rectangle_ms={:.3} rectangle_faces={} rectangle_vertices={} rectangle_declines={} encode_ms={:.6} decode_ms={:.6} cdt_insert_ms={:.6} cdt_constraint_ms={:.6} validation_ms={:.6}",
        face.id,
        face.surface.degree_u,
        face.surface.degree_v,
        face.loops.len(),
        Route::name(now.route),
        now.cdt_used - before.cdt_used,
        now.seed_ms - before.seed_ms,
        now.refine_ms - before.refine_ms,
        now.flip_ms - before.flip_ms,
        now.bridge_ms - before.bridge_ms,
        now.cdt_ms - before.cdt_ms,
        now.pslg_ms - before.pslg_ms,
        now.earclip_ms - before.earclip_ms,
        now.certify_ms - before.certify_ms,
        now.simple_ms - before.simple_ms,
        now.angle_gate_ms - before.angle_gate_ms,
        now.seed_suu - before.seed_suu,
        now.seed_svv - before.seed_svv,
        now.seed_suv - before.seed_suv,
        now.seed_grid_u - before.seed_grid_u,
        now.seed_grid_v - before.seed_grid_v,
        now.seed_diagonal_shrunk - before.seed_diagonal_shrunk,
        now.stations - before.stations,
        now.stations_placed - before.stations_placed,
        now.refine_passes - before.refine_passes,
        now.splits - before.splits,
        now.flip_calls - before.flip_calls,
        now.flip_rounds - before.flip_rounds,
        now.flips - before.flips,
        now.flip_edges_visited - before.flip_edges_visited,
        now.flip_sag_evals - before.flip_sag_evals,
        now.angle_gate_tests - before.angle_gate_tests,
        now.simple_tests - before.simple_tests,
        now.pslg_segment_tests - before.pslg_segment_tests,
        now.certify_calls - before.certify_calls,
        now.certify_centroid_tests - before.certify_centroid_tests,
        now.cdt_build_ms - before.cdt_build_ms,
        now.cdt_flood_ms - before.cdt_flood_ms,
        now.cdt_check_ms - before.cdt_check_ms,
        now.rectangle_ms - before.rectangle_ms,
        now.rectangle_faces - before.rectangle_faces,
        now.rectangle_vertices - before.rectangle_vertices,
        now.rectangle_declines - before.rectangle_declines,
        now.encode_ms - before.encode_ms,
        now.decode_ms - before.decode_ms,
        now.cdt_insert_ms - before.cdt_insert_ms,
        now.cdt_constraint_ms - before.cdt_constraint_ms,
        now.validation_ms - before.validation_ms,
    );
}

/// End a profiled tessellation: print the line and disarm the counters.
pub(in crate::watertight_tessellation) fn end(started: Option<Instant>, chord: f64, triangles: usize) {
    let Some(started) = started else {
        return;
    };
    let p = PROFILE.with(|cell| cell.get());
    eprintln!(
        "tess.profile ms={:.2} chord={chord:.3e} faces={} tris={triangles} | \
         edge_sampling_ms={:.2} seed_ms={:.2} refine_ms={:.2} | stations={} placed={} \
         margin_tests={} parity_tests={} host_ms={:.2} host_visits={} \
         flip_ms={:.2} flip_rounds={} flips={} cross_tests={} refine_passes={} splits={} | \
         bridge_ms={:.2} cdt_ms={:.2} earclip_ms={:.2} certify_ms={:.2} simple_ms={:.2} simple_tests={} \
         flip_calls={} flip_sag_evals={} angle_gate_ms={:.2} angle_gate_tests={} \
         flip_build_ms={:.2} flip_scan_ms={:.2} flip_apply_ms={:.2} flip_edges_visited={} \
         cdt_used={} pslg_ms={:.2} pslg_tests={} certify_calls={} certify_centroid_tests={} \
         cdt_build_ms={:.2} cdt_flood_ms={:.2} cdt_check_ms={:.2} rectangle_ms={:.2} rectangle_faces={} rectangle_vertices={} rectangle_declines={} encode_ms={:.6} decode_ms={:.6} cdt_insert_ms={:.6} cdt_constraint_ms={:.6} validation_ms={:.6}",
        started.elapsed().as_secs_f64() * 1_000.0,
        p.faces,
        p.edge_sampling_ms,
        p.seed_ms,
        p.refine_ms,
        p.stations,
        p.stations_placed,
        p.margin_tests,
        p.parity_tests,
        p.host_ms,
        p.host_visits,
        p.flip_ms,
        p.flip_rounds,
        p.flips,
        p.boundary_cross_tests,
        p.refine_passes,
        p.splits,
        p.bridge_ms,
        p.cdt_ms,
        p.earclip_ms,
        p.certify_ms,
        p.simple_ms,
        p.simple_tests,
        p.flip_calls,
        p.flip_sag_evals,
        p.angle_gate_ms,
        p.angle_gate_tests,
        p.flip_build_ms,
        p.flip_scan_ms,
        p.flip_apply_ms,
        p.flip_edges_visited,
        p.cdt_used,
        p.pslg_ms,
        p.pslg_segment_tests,
        p.certify_calls,
        p.certify_centroid_tests,
        p.cdt_build_ms,
        p.cdt_flood_ms,
        p.cdt_check_ms,
        p.rectangle_ms, p.rectangle_faces, p.rectangle_vertices, p.rectangle_declines,
        p.encode_ms, p.decode_ms, p.cdt_insert_ms, p.cdt_constraint_ms, p.validation_ms,
    );
    PROFILE.with(|cell| cell.set(TessProfile::default()));
}

/// Which accumulator a [`StageTimer`] adds its lifetime to.
#[derive(Clone, Copy)]
pub(in crate::watertight_tessellation) enum Stage {
    Seed,
    Refine,
    Flip,
    Host,
    Bridge,
    Cdt,
    EarClip,
    Certify,
    Simple,
    AngleGate,
    Pslg,
    CdtBuild,
    CdtFlood,
    CdtCheck,
    Rectangle,
    Encode,
    Decode,
    CdtInsert,
    CdtConstraint,
    Validation,
}

/// Scope timer: adds its lifetime to `stage` when it drops, so a function with
/// several early returns is attributed once at the top.
pub(in crate::watertight_tessellation) struct StageTimer {
    started: Option<Instant>,
    stage: Stage,
}

pub(in crate::watertight_tessellation) fn stage_timer(stage: Stage) -> StageTimer {
    StageTimer {
        started: profile_started(),
        stage,
    }
}

impl Drop for StageTimer {
    fn drop(&mut self) {
        let ms = elapsed_ms(self.started);
        match self.stage {
            Stage::Seed => tess_profile(|p| p.seed_ms += ms),
            Stage::Refine => tess_profile(|p| p.refine_ms += ms),
            Stage::Flip => tess_profile(|p| p.flip_ms += ms),
            Stage::Host => tess_profile(|p| p.host_ms += ms),
            Stage::Bridge => tess_profile(|p| p.bridge_ms += ms),
            Stage::Cdt => tess_profile(|p| p.cdt_ms += ms),
            Stage::EarClip => tess_profile(|p| p.earclip_ms += ms),
            Stage::Certify => tess_profile(|p| p.certify_ms += ms),
            Stage::Simple => tess_profile(|p| p.simple_ms += ms),
            Stage::AngleGate => tess_profile(|p| p.angle_gate_ms += ms),
            Stage::Pslg => tess_profile(|p| p.pslg_ms += ms),
            Stage::CdtBuild => tess_profile(|p| p.cdt_build_ms += ms),
            Stage::CdtFlood => tess_profile(|p| p.cdt_flood_ms += ms),
            Stage::CdtCheck => tess_profile(|p| p.cdt_check_ms += ms),
            Stage::Rectangle => tess_profile(|p| p.rectangle_ms += ms),
            Stage::Encode => tess_profile(|p| p.encode_ms += ms),
            Stage::Decode => tess_profile(|p| p.decode_ms += ms),
            Stage::CdtInsert => tess_profile(|p| p.cdt_insert_ms += ms),
            Stage::CdtConstraint => tess_profile(|p| p.cdt_constraint_ms += ms),
            Stage::Validation => tess_profile(|p| p.validation_ms += ms),
        }
    }
}

