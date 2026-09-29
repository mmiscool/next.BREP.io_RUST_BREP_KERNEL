//! Command-line STL-to-STEP converter with analytic recognition and safe fallback.

use brep_reconstruction::{
    stl::{read_stl_with_options, StlFormat, StlImport, StlReadOptions},
    stl_conversion::{
        binary_stl_coordinate_precision_tolerance, convert_stl_mesh_to_step, ConversionBackend,
        ConversionPolicy, StlConversionOptions, StlConversionOutput,
    },
};
use serde_json::json;
use std::{
    env,
    ffi::{OsStr, OsString},
    fs::{self, File, OpenOptions},
    io::{self, Write},
    path::{Path, PathBuf},
    process::ExitCode,
    time::{Instant, SystemTime, UNIX_EPOCH},
};

const EXIT_USAGE: u8 = 2;
const EXIT_INPUT: u8 = 3;
const EXIT_CONVERSION: u8 = 4;
const EXIT_OUTPUT: u8 = 5;

const HELP: &str = r#"Convert an STL triangle mesh to a validated AP214 STEP solid.

Usage: stl2step [OPTIONS] INPUT.stl

The output is INPUT.step in the same directory. STL files contain no unit
metadata, so coordinates are interpreted as millimeters unless --unit is set.
A file holding several disjoint closed shells converts to one STEP document
with one solid per shell.
Recognized plane, cylinder, cone, and sphere regions are retained analytically
whenever their shared boundaries can be reconstructed safely. Arbitrary closed,
repairable remainder is preserved using a faceted BREP.

The RANSAC options below apply only when the kernel segmentation reads the
mesh as a whole primitive (every triangle on at most three analytic regions);
every other mesh skips RANSAC and is rebuilt from the segmentation alone.

Options:
      --strict-analytic                 Reject instead of using faceted fallback
  -f, --force                           Replace an existing output/report file
      --unit UNIT                       mm, cm, m, micron, inch, or foot [default: mm]
      --weld-tolerance DISTANCE         Absolute STL vertex-weld distance
      --distance-tolerance DISTANCE     Absolute RANSAC residual tolerance
      --relative-tolerance FRACTION     Scale-relative RANSAC tolerance
      --normal-tolerance DEGREES        Maximum normal-angle residual
      --feature-angle DEGREES           Feature-edge dihedral threshold
      --min-support COUNT               Minimum supporting triangle count
      --max-hypotheses COUNT            Maximum generic RANSAC hypotheses
      --seed U64                        Deterministic RANSAC seed
      --report-json PATH                Also write a machine-readable report
  -h, --help                            Print help
  -V, --version                         Print version

Exit status: 0 success, 2 usage, 3 STL input, 4 conversion, 5 output I/O.
"#;

#[derive(Debug)]
struct Cli {
    input: PathBuf,
    output: PathBuf,
    report_json: Option<PathBuf>,
    force: bool,
    unit: String,
    read_options: StlReadOptions,
    conversion_options: StlConversionOptions,
}

enum ParseResult {
    Run(Box<Cli>),
    Help,
    Version,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err((code, message)) => {
            eprintln!("error: {message}");
            if code == EXIT_USAGE {
                eprintln!("Try 'stl2step --help' for usage.");
            }
            ExitCode::from(code)
        }
    }
}

fn run() -> Result<(), (u8, String)> {
    let mut cli = match parse_args(env::args_os().skip(1))? {
        ParseResult::Help => {
            print!("{HELP}");
            return Ok(());
        }
        ParseResult::Version => {
            println!("stl2step {}", env!("CARGO_PKG_VERSION"));
            return Ok(());
        }
        ParseResult::Run(cli) => *cli,
    };

    preflight_destination(&cli.output, cli.force)?;
    if let Some(report_path) = &cli.report_json {
        if report_path == &cli.output {
            return Err((
                EXIT_USAGE,
                "--report-json must not name the STEP output path".to_owned(),
            ));
        }
        preflight_destination(report_path, cli.force)?;
    }

    eprintln!(
        "warning: STL is unitless; interpreting coordinates as {}",
        display_unit(&cli.unit)
    );
    let total_started = Instant::now();
    let import_started = Instant::now();
    let imported = read_stl_with_options(&cli.input, &cli.read_options)
        .map_err(|error| (EXIT_INPUT, error.to_string()))?;
    let import_seconds = import_started.elapsed().as_secs_f64();
    cli.conversion_options.coordinate_precision_tolerance = match imported.format {
        StlFormat::Binary => binary_stl_coordinate_precision_tolerance(&imported.mesh),
        StlFormat::Ascii => 0.0,
    };
    let (positions, indices) = mesh_buffers(&imported);
    let part_name = cli
        .input
        .file_stem()
        .and_then(OsStr::to_str)
        .filter(|name| !name.is_empty())
        .unwrap_or("converted_stl");
    let timestamp = unix_timestamp();
    let converted = convert_stl_mesh_to_step(
        &imported.mesh,
        &positions,
        Some(&indices),
        &cli.conversion_options,
        part_name,
        &cli.unit,
        &timestamp,
    )
    .map_err(|error| (EXIT_CONVERSION, error.to_string()))?;

    let write_started = Instant::now();
    atomic_write(&cli.output, converted.step_text.as_bytes(), cli.force)?;
    let step_write_seconds = write_started.elapsed().as_secs_f64();
    let total_seconds = total_started.elapsed().as_secs_f64();

    if let Some(report_path) = &cli.report_json {
        let report = json!({
            "schema_version": 2,
            "input": {
                "path": cli.input,
                "format": format_name(imported.format),
                "source_triangles": imported.source_triangle_count,
                "source_corner_vertices": imported.source_vertex_count,
                "welded_vertices": imported.welded_vertex_count,
                "weld_tolerance": imported.weld_tolerance,
                "interpreted_unit": cli.unit,
            },
            "output": {
                "path": cli.output,
                "bytes": converted.report.exported_step_bytes,
            },
            "conversion": converted.report,
            "cli_timings_seconds": {
                "stl_import": import_seconds,
                "step_file_write": step_write_seconds,
                "total_through_step_write": total_seconds,
            },
        });
        let mut bytes = serde_json::to_vec_pretty(&report)
            .map_err(|error| (EXIT_OUTPUT, format!("could not serialize report: {error}")))?;
        bytes.push(b'\n');
        atomic_write(report_path, &bytes, cli.force)?;
    }

    let final_total_seconds = total_started.elapsed().as_secs_f64();

    print_report(
        &cli,
        &imported,
        &converted,
        import_seconds,
        step_write_seconds,
        final_total_seconds,
    );
    Ok(())
}

fn parse_args(args: impl Iterator<Item = OsString>) -> Result<ParseResult, (u8, String)> {
    let mut options = StlConversionOptions::default();
    let mut read_options = StlReadOptions::default();
    let mut force = false;
    let mut unit = "millimeter".to_owned();
    let mut report_json = None;
    let mut input = None;
    let mut args = args.peekable();
    let mut options_enabled = true;

    while let Some(argument) = args.next() {
        if options_enabled && argument == "--" {
            options_enabled = false;
            continue;
        }
        if options_enabled && (argument == "-h" || argument == "--help") {
            return Ok(ParseResult::Help);
        }
        if options_enabled && (argument == "-V" || argument == "--version") {
            return Ok(ParseResult::Version);
        }
        if options_enabled && (argument == "-f" || argument == "--force") {
            force = true;
            continue;
        }
        if options_enabled && argument == "--strict-analytic" {
            options.policy = ConversionPolicy::RequireFullyAnalytic;
            continue;
        }

        if options_enabled && argument.to_string_lossy().starts_with('-') {
            let (name, inline_value) = split_option(argument)?;
            let value = match inline_value {
                Some(value) => value,
                None => args
                    .next()
                    .ok_or_else(|| (EXIT_USAGE, format!("option '{name}' requires a value")))?,
            };
            let text = value
                .to_str()
                .ok_or_else(|| (EXIT_USAGE, format!("value for '{name}' is not valid UTF-8")))?;
            match name.as_str() {
                "--unit" => unit = normalize_unit(text)?,
                "--weld-tolerance" => {
                    let parsed = finite_nonnegative(&name, text)?;
                    read_options.weld_tolerance = Some(parsed);
                    options.weld_tolerance = parsed;
                }
                "--distance-tolerance" => {
                    options.recognition.distance_tolerance = finite_positive(&name, text)?;
                }
                "--relative-tolerance" => {
                    options.recognition.relative_tolerance = finite_nonnegative(&name, text)?;
                }
                "--normal-tolerance" | "--normal-tolerance-degrees" => {
                    options.recognition.normal_tolerance = degrees(&name, text)?;
                }
                "--feature-angle" | "--feature-angle-degrees" => {
                    options.recognition.feature_angle = degrees(&name, text)?;
                }
                "--min-support" => {
                    options.recognition.minimum_support = positive_usize(&name, text)?;
                }
                "--max-hypotheses" => {
                    options.recognition.max_hypotheses = positive_usize(&name, text)?;
                }
                "--seed" => {
                    options.recognition.deterministic_seed = Some(text.parse().map_err(|_| {
                        (
                            EXIT_USAGE,
                            format!("'{text}' is not a valid u64 for {name}"),
                        )
                    })?);
                }
                "--report-json" => report_json = Some(PathBuf::from(value)),
                _ => return Err((EXIT_USAGE, format!("unknown option '{name}'"))),
            }
            continue;
        }
        if input.replace(PathBuf::from(argument)).is_some() {
            return Err((EXIT_USAGE, "exactly one INPUT.stl is required".to_owned()));
        }
    }

    let input = input.ok_or_else(|| (EXIT_USAGE, "missing INPUT.stl".to_owned()))?;
    if !input
        .extension()
        .and_then(OsStr::to_str)
        .is_some_and(|extension| extension.eq_ignore_ascii_case("stl"))
    {
        return Err((
            EXIT_USAGE,
            "input path must have an .stl extension".to_owned(),
        ));
    }
    let output = input.with_extension("step");
    Ok(ParseResult::Run(Box::new(Cli {
        input,
        output,
        report_json,
        force,
        unit,
        read_options,
        conversion_options: options,
    })))
}

fn split_option(argument: OsString) -> Result<(String, Option<OsString>), (u8, String)> {
    let text = argument
        .to_str()
        .ok_or_else(|| (EXIT_USAGE, "option name is not valid UTF-8".to_owned()))?;
    Ok(match text.split_once('=') {
        Some((name, value)) => (name.to_owned(), Some(OsString::from(value))),
        None => (text.to_owned(), None),
    })
}

fn finite_positive(name: &str, text: &str) -> Result<f64, (u8, String)> {
    let value = finite(name, text)?;
    if value <= 0.0 {
        return Err((EXIT_USAGE, format!("{name} must be positive")));
    }
    Ok(value)
}

fn finite_nonnegative(name: &str, text: &str) -> Result<f64, (u8, String)> {
    let value = finite(name, text)?;
    if value < 0.0 {
        return Err((EXIT_USAGE, format!("{name} must be non-negative")));
    }
    Ok(value)
}

fn finite(name: &str, text: &str) -> Result<f64, (u8, String)> {
    let value: f64 = text
        .parse()
        .map_err(|_| (EXIT_USAGE, format!("'{text}' is not a number for {name}")))?;
    if !value.is_finite() {
        return Err((EXIT_USAGE, format!("{name} must be finite")));
    }
    Ok(value)
}

fn degrees(name: &str, text: &str) -> Result<f64, (u8, String)> {
    let value = finite_nonnegative(name, text)?;
    if value > 180.0 {
        return Err((EXIT_USAGE, format!("{name} must be at most 180 degrees")));
    }
    Ok(value.to_radians())
}

fn positive_usize(name: &str, text: &str) -> Result<usize, (u8, String)> {
    let value: usize = text.parse().map_err(|_| {
        (
            EXIT_USAGE,
            format!("'{text}' is not a valid count for {name}"),
        )
    })?;
    if value == 0 {
        return Err((EXIT_USAGE, format!("{name} must be non-zero")));
    }
    Ok(value)
}

fn normalize_unit(value: &str) -> Result<String, (u8, String)> {
    let unit = match value.to_ascii_lowercase().as_str() {
        "mm" | "millimeter" | "millimetre" => "millimeter",
        "cm" | "centimeter" | "centimetre" => "centimeter",
        "m" | "meter" | "metre" => "meter",
        "um" | "micron" | "micrometer" | "micrometre" => "micron",
        "in" | "inch" => "inch",
        "ft" | "foot" => "foot",
        _ => {
            return Err((
                EXIT_USAGE,
                format!("unsupported unit '{value}'; use mm, cm, m, micron, inch, or foot"),
            ));
        }
    };
    Ok(unit.to_owned())
}

fn mesh_buffers(imported: &StlImport) -> (Vec<f64>, Vec<u32>) {
    let positions = imported
        .mesh
        .vertices
        .iter()
        .flat_map(|point| [point.x, point.y, point.z])
        .collect();
    let indices = imported.mesh.triangles.iter().flatten().copied().collect();
    (positions, indices)
}

fn preflight_destination(path: &Path, force: bool) -> Result<(), (u8, String)> {
    if path.is_dir() {
        return Err((
            EXIT_OUTPUT,
            format!("output path '{}' is a directory", path.display()),
        ));
    }
    if path.exists() && !force {
        return Err((
            EXIT_OUTPUT,
            format!(
                "output '{}' already exists; use --force to replace it",
                path.display()
            ),
        ));
    }
    let parent = path
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    if !parent.is_dir() {
        return Err((
            EXIT_OUTPUT,
            format!("output directory '{}' does not exist", parent.display()),
        ));
    }
    Ok(())
}

fn atomic_write(path: &Path, bytes: &[u8], force: bool) -> Result<(), (u8, String)> {
    let parent = path
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let name = path.file_name().unwrap_or_else(|| OsStr::new("output"));
    let mut temporary = None;
    for attempt in 0..100_u32 {
        let candidate = parent.join(format!(
            ".{}.{}.{}.tmp",
            name.to_string_lossy(),
            std::process::id(),
            attempt
        ));
        match OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&candidate)
        {
            Ok(file) => {
                temporary = Some((candidate, file));
                break;
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => {
                return Err((
                    EXIT_OUTPUT,
                    format!(
                        "could not create temporary output beside '{}': {error}",
                        path.display()
                    ),
                ))
            }
        }
    }
    let (temporary_path, mut file) = temporary.ok_or_else(|| {
        (
            EXIT_OUTPUT,
            format!(
                "could not reserve a temporary output beside '{}'",
                path.display()
            ),
        )
    })?;
    let result = (|| -> io::Result<()> {
        file.write_all(bytes)?;
        file.sync_all()?;
        drop(file);
        if force {
            fs::rename(&temporary_path, path)?;
        } else {
            fs::hard_link(&temporary_path, path)?;
            fs::remove_file(&temporary_path)?;
        }
        File::open(parent)?.sync_all()?;
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temporary_path);
    }
    result.map_err(|error| {
        (
            EXIT_OUTPUT,
            format!("could not atomically write '{}': {error}", path.display()),
        )
    })
}

fn print_report(
    cli: &Cli,
    imported: &StlImport,
    converted: &StlConversionOutput,
    import_seconds: f64,
    write_seconds: f64,
    total_seconds: f64,
) {
    let report = &converted.report;
    println!("STL to STEP conversion complete");
    println!("  Input:              {}", cli.input.display());
    println!("  Output:             {}", cli.output.display());
    println!("  STL format:         {}", imported.format);
    println!("  Interpreted unit:   {}", display_unit(&cli.unit));
    println!("  Source facets:      {}", imported.source_triangle_count);
    println!(
        "  Vertices:           {} source corners -> {} welded",
        imported.source_vertex_count, imported.welded_vertex_count
    );
    println!("  Weld tolerance:     {:.6e}", imported.weld_tolerance);
    println!(
        "  Recognition dist.:  requested {:.6e} | source floor {:.6e} | effective {:.6e}",
        report.requested_distance_tolerance,
        report.coordinate_precision_tolerance,
        report.effective_distance_tolerance
    );
    println!(
        "  Closed manifold:    {}{}",
        report.source_closed_manifold,
        if report.analytic_used_repaired_mesh {
            " (repaired before the analytic rebuild)"
        } else {
            ""
        }
    );
    println!("  Backend:            {}", backend_name(report.backend));
    println!("  Backend reason:     {}", report.backend_reason);
    if let Some(cause) = &report.fallback_cause {
        println!("  Fallback cause:     {cause}");
    }
    if let Some(hybrid) = &report.hybrid_rebuild {
        println!(
            "  Mixed rebuild:      {} plane + {} cylinder + {} cone + {} sphere + {} faceted face(s)",
            hybrid.analytic_plane_faces,
            hybrid.analytic_cylinder_faces,
            hybrid.analytic_cone_faces,
            hybrid.analytic_sphere_faces,
            hybrid.faceted_faces
        );
        println!(
            "  Triangle coverage:  {} plane + {} cylinder + {} cone + {} sphere + {} faceted",
            hybrid.analytic_plane_triangles,
            hybrid.analytic_cylinder_triangles,
            hybrid.analytic_cone_triangles,
            hybrid.analytic_sphere_triangles,
            hybrid.faceted_triangles
        );
        if hybrid.demoted_regions > 0 {
            println!(
                "  Conservative demotion: {} region(s), {} triangle(s)",
                hybrid.demoted_regions, hybrid.demoted_triangles
            );
        }
    }
    match &report.recognition_skipped {
        Some(reason) => println!("  Recognition:        skipped; {reason}"),
        None => println!(
            "  Recognition:        {} region(s), {}/{} triangles, {} unresolved",
            report.recognized_regions,
            report.recognized_triangles,
            report.input_triangles,
            report.unresolved_triangles
        ),
    }
    for (index, region) in report.regions.iter().enumerate() {
        println!(
            "    Region {}: {} | support={} | rms={:.6e} | max={:.6e} | confidence={:.6} | orientation={:+}",
            index + 1,
            region.surface_type.name(),
            region.support_triangles,
            region.rms_error,
            region.max_error,
            region.confidence,
            region.orientation
        );
        println!(
            "      normal rms/max: {:.4}/{:.4} deg | {}",
            region.rms_normal_error_radians.to_degrees(),
            region.max_normal_error_radians.to_degrees(),
            region.reason
        );
        println!(
            "      phase time:     {:.6} s",
            phase_total(&region.phase_timings)
        );
    }
    println!(
        "  STEP body:          {} bytes, {} advanced face(s), {} validated solid(s)",
        report.exported_step_bytes, report.exported_advanced_faces, report.roundtrip_solids
    );
    // A file holding several disjoint closed shells is several bodies, each
    // reconstructed on its own: the document's backend line above is the first
    // body's, so say what every body actually did.
    if report.components.len() > 1 {
        println!("  Bodies:             {}", report.components.len());
        for component in &report.components {
            println!(
                "    Body {}: {} triangle(s) from source triangle {} -> {} face(s) | {}",
                component.index + 1,
                component.input_triangles,
                component.first_triangle,
                component.faces,
                backend_name(component.backend)
            );
            if let Some(refusal) = &component.refusal {
                println!("      refused: {refusal}");
            }
        }
    }
    println!("  Timings:");
    println!("    STL import:       {:.6} s", import_seconds);
    println!(
        "    Segmentation:     {:.6} s",
        report.timings.segmentation_seconds
    );
    println!(
        "    Recognition:      {:.6} s",
        report.timings.recognition_seconds
    );
    println!(
        "    Topology build:   {:.6} s",
        report.timings.topology_build_seconds
    );
    println!(
        "    STEP export:      {:.6} s",
        report.timings.step_export_seconds
    );
    println!(
        "    STEP validation:  {:.6} s",
        report.timings.step_validation_seconds
    );
    println!("    File write:       {:.6} s", write_seconds);
    println!("    Total:            {:.6} s", total_seconds);
    for message in &report.messages {
        println!("  Note: {message}");
    }
    if cli.report_json.is_none() {
        println!("  Tip: use --report-json PATH to save this result as structured JSON.");
    }
}

fn phase_total(timings: &brep_reconstruction::PhaseTimings) -> f64 {
    [
        timings.metadata_inspection_seconds,
        timings.candidate_generation_seconds,
        timings.candidate_evaluation_seconds,
        timings.region_growth_seconds,
        timings.refinement_seconds,
        timings.validation_seconds,
    ]
    .into_iter()
    .flatten()
    .sum()
}

fn backend_name(backend: ConversionBackend) -> &'static str {
    match backend {
        ConversionBackend::RansacSphere => "RANSAC exact sphere",
        ConversionBackend::RansacTorus => "RANSAC exact torus",
        ConversionBackend::RansacCappedCylinder => "RANSAC exact capped cylinder",
        ConversionBackend::RansacCappedCone => "RANSAC exact capped cone",
        ConversionBackend::KernelAnalyticRebuild => "analytic multi-region rebuild",
        ConversionBackend::HybridAnalyticRebuild => "mixed analytic plane/cylinder/cone/sphere rebuild",
        ConversionBackend::FacetedRepair => "faceted repair fallback",
        ConversionBackend::FacetedRepairCoplanarMerged => {
            "faceted repair + coplanar merge fallback"
        }
        ConversionBackend::Refused => "refused, not in the document",
    }
}

fn format_name(format: StlFormat) -> &'static str {
    match format {
        StlFormat::Binary => "binary",
        StlFormat::Ascii => "ascii",
    }
}

fn display_unit(unit: &str) -> &str {
    match unit {
        "millimeter" => "millimeters (mm)",
        "centimeter" => "centimeters (cm)",
        "meter" => "meters (m)",
        "micron" => "micrometers (micron)",
        "inch" => "inches",
        "foot" => "feet",
        _ => unit,
    }
}

fn unix_timestamp() -> String {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| format!("unix:{}", duration.as_secs()))
        .unwrap_or_else(|_| "unix:0".to_owned())
}
