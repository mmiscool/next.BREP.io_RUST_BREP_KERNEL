use std::cell::Cell;
use web_time::Instant;

#[derive(Default, Clone, Copy)]
pub(crate) struct MassProfile {
    pub enabled: bool,
    pub faces: u64,
    pub trimmed_faces: u64,
    pub route_biperiodic: u64,
    pub route_winding: u64,
    pub route_seam_band: u64,
    pub route_cap: u64,
    pub route_wall: u64,
    pub route_hopped_raw: u64,
    pub route_hopped_unwrapped: u64,
    pub route_straddle: u64,
    pub route_periodic_raw: u64,
    pub route_raw: u64,
    pub route_untrimmed: u64,
    pub route_affine: u64,
    pub polygon_chords: u64,
    pub base_cells: u64,
    pub full_cells: u64,
    pub full_stations: u64,
    pub leaf_cells: u64,
    pub tri_stations: u64,
    pub corr_chords: u64,
    pub corr_stations: u64,
    pub polygons_ms: f64,
    pub base_clip_ms: f64,
    pub cells_ms: f64,
    /// Hierarchical child clipping inside the quadtree (a subset of `cells_ms`).
    pub child_clip_ms: f64,
    /// Tensor Gauss stations over fully covered cells (a subset of `cells_ms`).
    pub full_ms: f64,
    /// Triangle cubature over depth-limit leaf cells (a subset of `cells_ms`).
    pub leaf_ms: f64,
    pub corr_ms: f64,
    pub untrimmed_ms: f64,
    pub gate_ms: f64,
    /// Choosing the panels and orders of a span from its weights
    /// (`rule`), and how many spans had to read a weight at all: the cost of
    /// the rational rule's decision, apart from the stations it then asks for.
    pub rule_ms: f64,
    pub rule_spans: u64,
    pub rule_sampled: u64,
    /// Emit one `mass.face` line per face (`BREP_PROFILE_MASS_FACES`).
    pub face_lines: bool,
}

thread_local! {
    static PROFILE: Cell<MassProfile> = const { Cell::new(MassProfile {
        enabled: false, faces: 0, trimmed_faces: 0, route_biperiodic: 0, route_winding: 0,
        route_seam_band: 0, route_cap: 0, route_wall: 0, route_hopped_raw: 0,
        route_hopped_unwrapped: 0, route_straddle: 0, route_periodic_raw: 0, route_raw: 0,
        route_untrimmed: 0, route_affine: 0, polygon_chords: 0, base_cells: 0, full_cells: 0,
        full_stations: 0, leaf_cells: 0, tri_stations: 0, corr_chords: 0, corr_stations: 0,
        polygons_ms: 0.0, base_clip_ms: 0.0, cells_ms: 0.0, child_clip_ms: 0.0, full_ms: 0.0,
        leaf_ms: 0.0, corr_ms: 0.0, untrimmed_ms: 0.0, gate_ms: 0.0, rule_ms: 0.0,
        rule_spans: 0, rule_sampled: 0, face_lines: false,
    }) };
}

/// Read-modify-write the thread-local profile when it is enabled; a no-op
/// (one flag read) otherwise.
pub(crate) fn mass_profile(update: impl FnOnce(&mut MassProfile)) {
    PROFILE.with(|cell| {
        let mut profile = cell.get();
        if profile.enabled {
            update(&mut profile);
            cell.set(profile);
        }
    });
}

pub(crate) fn mass_profile_enabled() -> bool {
    PROFILE.with(|cell| cell.get().enabled)
}

/// The live counters, for a before/after delta around one face.
pub(crate) fn snapshot() -> MassProfile {
    PROFILE.with(|cell| cell.get())
}

pub(crate) fn face_lines_enabled() -> bool {
    PROFILE.with(|cell| cell.get().face_lines)
}

/// Which route the face just took, read off the counter that moved.
fn route_name(delta: &MassProfile) -> &'static str {
    for (count, name) in [
        (delta.route_affine, "affine"),
        (delta.route_untrimmed, "untrimmed"),
        (delta.route_biperiodic, "biperiodic"),
        (delta.route_winding, "winding"),
        (delta.route_seam_band, "seam_band"),
        (delta.route_cap, "cap"),
        (delta.route_wall, "wall"),
        (delta.route_hopped_raw, "hopped_raw"),
        (delta.route_hopped_unwrapped, "hopped_unwrapped"),
        (delta.route_straddle, "straddle"),
        (delta.route_periodic_raw, "periodic_raw"),
        (delta.route_raw, "raw"),
    ] {
        if count > 0 {
            return name;
        }
    }
    "none"
}

/// One `mass.face` line: the face's surface kind and trim size beside the
/// route it took, the stations it cost and where its milliseconds went.
/// `before` is [`snapshot`] taken before the face's integration, `wall_ms`
/// the face's whole elapsed time. Only called when `face_lines` is armed.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_face(
    call: &str,
    shell: usize,
    face: usize,
    kind: &str,
    degree_u: usize,
    degree_v: usize,
    closed_u: bool,
    closed_v: bool,
    loops: usize,
    coedges: usize,
    before: MassProfile,
    wall_ms: f64,
) {
    let now = PROFILE.with(|cell| cell.get());
    let d = MassProfile {
        route_biperiodic: now.route_biperiodic - before.route_biperiodic,
        route_winding: now.route_winding - before.route_winding,
        route_seam_band: now.route_seam_band - before.route_seam_band,
        route_cap: now.route_cap - before.route_cap,
        route_wall: now.route_wall - before.route_wall,
        route_hopped_raw: now.route_hopped_raw - before.route_hopped_raw,
        route_hopped_unwrapped: now.route_hopped_unwrapped - before.route_hopped_unwrapped,
        route_straddle: now.route_straddle - before.route_straddle,
        route_periodic_raw: now.route_periodic_raw - before.route_periodic_raw,
        route_raw: now.route_raw - before.route_raw,
        route_untrimmed: now.route_untrimmed - before.route_untrimmed,
        route_affine: now.route_affine - before.route_affine,
        polygon_chords: now.polygon_chords - before.polygon_chords,
        base_cells: now.base_cells - before.base_cells,
        full_cells: now.full_cells - before.full_cells,
        full_stations: now.full_stations - before.full_stations,
        leaf_cells: now.leaf_cells - before.leaf_cells,
        tri_stations: now.tri_stations - before.tri_stations,
        corr_chords: now.corr_chords - before.corr_chords,
        corr_stations: now.corr_stations - before.corr_stations,
        polygons_ms: now.polygons_ms - before.polygons_ms,
        base_clip_ms: now.base_clip_ms - before.base_clip_ms,
        cells_ms: now.cells_ms - before.cells_ms,
        child_clip_ms: now.child_clip_ms - before.child_clip_ms,
        full_ms: now.full_ms - before.full_ms,
        leaf_ms: now.leaf_ms - before.leaf_ms,
        corr_ms: now.corr_ms - before.corr_ms,
        untrimmed_ms: now.untrimmed_ms - before.untrimmed_ms,
        gate_ms: now.gate_ms - before.gate_ms,
        rule_ms: now.rule_ms - before.rule_ms,
        rule_spans: now.rule_spans - before.rule_spans,
        rule_sampled: now.rule_sampled - before.rule_sampled,
        ..MassProfile::default()
    };
    eprintln!(
        "mass.face call={call} shell={shell} face={face} kind={kind} deg={degree_u}x{degree_v} closed={}{} loops={loops} coedges={coedges} route={} chords={} base_cells={} full_cells={} full_stations={} leaf_cells={} tri_stations={} corr_chords={} corr_stations={} ms={:.3} gate_ms={:.3} polygons_ms={:.3} base_clip_ms={:.3} child_clip_ms={:.3} full_ms={:.3} leaf_ms={:.3} corr_ms={:.3} untrimmed_ms={:.3} cells_ms={:.3}",
        if closed_u { "u" } else { "-" },
        if closed_v { "v" } else { "-" },
        route_name(&d), d.polygon_chords, d.base_cells, d.full_cells, d.full_stations,
        d.leaf_cells, d.tri_stations, d.corr_chords, d.corr_stations, wall_ms, d.gate_ms,
        d.polygons_ms, d.base_clip_ms, d.child_clip_ms, d.full_ms, d.leaf_ms, d.corr_ms,
        d.untrimmed_ms, d.cells_ms,
    );
}

/// Milliseconds since `started` when profiling is on; `None` otherwise.
pub(crate) fn profile_started() -> Option<Instant> {
    mass_profile_enabled().then(Instant::now)
}

pub(crate) fn elapsed_ms(started: Option<Instant>) -> f64 {
    started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0)
}

/// Start a profiled call: resets the counters and arms them when
/// `BREP_PROFILE` is set. Returns whether profiling is on.
pub(crate) fn begin(call: &str) -> Option<(String, Instant)> {
    let enabled = std::env::var("BREP_PROFILE").is_ok();
    let face_lines = enabled && std::env::var("BREP_PROFILE_MASS_FACES").is_ok();
    PROFILE.with(|cell| {
        cell.set(MassProfile {
            enabled,
            face_lines,
            ..MassProfile::default()
        })
    });
    enabled.then(|| (call.to_string(), Instant::now()))
}

/// End a profiled call: print the line and disarm the counters.
pub(crate) fn end(token: Option<(String, Instant)>) {
    let Some((call, started)) = token else {
        return;
    };
    let p = PROFILE.with(|cell| cell.get());
    eprintln!(
        "mass.profile call={call} ms={:.2} faces={} trimmed={} routes: affine={} untrimmed={} biperiodic={} winding={} seam_band={} cap={} wall={} hopped_raw={} hopped_unwrapped={} straddle={} periodic_raw={} raw={} | chords={} base_cells={} full_cells={} full_stations={} leaf_cells={} tri_stations={} corr_chords={} corr_stations={} | gate_ms={:.2} polygons_ms={:.2} base_clip_ms={:.2} cells_ms={:.2} child_clip_ms={:.2} full_ms={:.2} leaf_ms={:.2} corr_ms={:.2} untrimmed_ms={:.2} rule_ms={:.2} rule_spans={} rule_sampled={}",
        started.elapsed().as_secs_f64() * 1_000.0,
        p.faces, p.trimmed_faces, p.route_affine, p.route_untrimmed, p.route_biperiodic,
        p.route_winding, p.route_seam_band, p.route_cap, p.route_wall, p.route_hopped_raw,
        p.route_hopped_unwrapped, p.route_straddle, p.route_periodic_raw, p.route_raw,
        p.polygon_chords, p.base_cells, p.full_cells, p.full_stations, p.leaf_cells,
        p.tri_stations, p.corr_chords, p.corr_stations, p.gate_ms, p.polygons_ms,
        p.base_clip_ms, p.cells_ms, p.child_clip_ms, p.full_ms, p.leaf_ms, p.corr_ms,
        p.untrimmed_ms,
        p.rule_ms,
        p.rule_spans,
        p.rule_sampled,
    );
    PROFILE.with(|cell| cell.set(MassProfile::default()));
}

// ---------------------------------------------------------------------------
// Sign-check attribution: the identity of the face set a call integrated.
// ---------------------------------------------------------------------------

thread_local! {
    /// The call site that is asking, for the duration of one integrator call.
    static CALLER: Cell<Option<&'static str>> = const { Cell::new(None) };
}

/// Names the call site for every integrator call made while it is alive, so
/// the `mass.identity` line can say WHO paid for the integral rather than
/// leaving a log reader to infer it from the neighbouring profile lines.
///
/// Diagnostic only: nothing but the identity line reads it, and the identity
/// line only exists under `BREP_PROFILE_MASS_IDENTITY` /
/// `BREP_PROFILE_MASS_DUMP`. The guard restores the previous name on drop so
/// a labelled site nested inside another labelled site reports itself and
/// then hands the outer name back.
pub struct MassCaller(Option<&'static str>);

impl Drop for MassCaller {
    fn drop(&mut self) {
        CALLER.with(|cell| cell.set(self.0));
    }
}

pub fn mass_caller(name: &'static str) -> MassCaller {
    MassCaller(CALLER.with(|cell| cell.replace(Some(name))))
}

fn current_caller() -> &'static str {
    CALLER.with(|cell| cell.get()).unwrap_or("unlabelled")
}

/// One line per integrator call naming the CONTENT of the face set it read
/// (`BREP_PROFILE_MASS_IDENTITY`), and optionally the solid itself written to
/// disk (`BREP_PROFILE_MASS_DUMP=<dir>`, which implies the line).
///
/// This is the instrument behind "how many sign checks integrate a face set a
/// later measurement integrates again". The 2026-09-13 integrator record
/// paired them by STATION SIGNATURE — face count and station counts — and said
/// plainly that two congruent solids would match too. A content hash settles
/// that: equal hashes mean the two calls saw the same geometry in the same
/// order, so the second one's answer was already computed.
///
/// It is a measurement lever, not a product path: hashing every solid costs
/// its control points, which is why it is off unless asked for, and why the
/// timing tables in the record are taken with it OFF.
pub(crate) struct IdentityDump {
    /// `None` unless `BREP_PROFILE_MASS_DUMP` named a directory.
    directory: Option<String>,
}

thread_local! {
    /// Per-process call counter so a dump's file name orders the calls the way
    /// stderr does. Thread-local like the rest of this module; the populations
    /// this instrument serves are single-threaded replays.
    static DUMP_SEQUENCE: Cell<u64> = const { Cell::new(0) };
}

/// Read the two switches. Returns `None` when neither is set, which is the
/// only cost on the shipped path.
pub(crate) fn identity_dump() -> Option<IdentityDump> {
    let directory = std::env::var("BREP_PROFILE_MASS_DUMP").ok();
    if directory.is_none() && std::env::var("BREP_PROFILE_MASS_IDENTITY").is_err() {
        return None;
    }
    Some(IdentityDump { directory })
}

/// Emit the identity line for a finished call, and write the solid when a dump
/// directory was named. `values` are the raw f64 bits the call returned, in
/// the order the probe reads them back.
pub(crate) fn emit_identity(
    dump: IdentityDump,
    call: &str,
    solid: &crate::BrepSolid,
    values: &[(&str, f64)],
) {
    let started = Instant::now();
    let identity = super::identity::face_set_identity(solid);
    let hash_ms = started.elapsed().as_secs_f64() * 1_000.0;
    let sequence = DUMP_SEQUENCE.with(|cell| {
        let next = cell.get() + 1;
        cell.set(next);
        next
    });
    let printed: Vec<String> = values
        .iter()
        .map(|(name, value)| format!("{name}={:016x}", value.to_bits()))
        .collect();
    let mut path = String::new();
    if let Some(directory) = dump.directory {
        let file = format!("{directory}/{sequence:05}-{call}.json");
        match serde_json::to_string(solid) {
            Ok(text) => {
                if let Err(error) = std::fs::write(&file, text) {
                    eprintln!("mass.dump write failed for {file}: {error}");
                } else {
                    path = file;
                }
            }
            Err(error) => eprintln!("mass.dump serialize failed for {file}: {error}"),
        }
        // A sidecar beside the solid so the probe needs only the directory:
        // the stderr line carries the same fields, but correlating a probe run
        // against a captured log is one more thing to get wrong.
        let meta = format!(
            "{{\"call\":\"{call}\",\"seq\":{sequence},\"faces\":{},\"hash\":\"{}\",\"values\":\"{}\"}}",
            identity.faces,
            identity.hex(),
            printed.join(" "),
        );
        let meta_file = format!("{directory}/{sequence:05}-{call}.meta.json");
        if let Err(error) = std::fs::write(&meta_file, meta) {
            eprintln!("mass.dump write failed for {meta_file}: {error}");
        }
    }
    eprintln!(
        "mass.identity call={call} caller={} seq={sequence} faces={} hash={} hash_ms={hash_ms:.3} {}{}",
        current_caller(),
        identity.faces,
        identity.hex(),
        printed.join(" "),
        if path.is_empty() {
            String::new()
        } else {
            format!(" path={path}")
        },
    );
}
