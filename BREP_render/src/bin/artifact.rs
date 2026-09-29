//! brep-render-artifact — the headless presentation shell (R32/R41).
//!
//! Usage: brep-render-artifact <history-request.json> <out.png> [width height]
//!
//! Runs the whole feature history natively through the kernel pipeline,
//! populates the render scene from the resident solids, renders one frame with
//! the artifact framing, and writes the PNG. History/feature errors are
//! reported on stderr but still produce a frame — seeing what a failing
//! history DID build is the point of the artifact. Exit code is non-zero only
//! when no PNG could be produced at all.

use brep_render::camera::artifact_camera;
use brep_render::pipeline::scene_from_history_json;
use brep_render::render::{create_headless_device, RenderCore, COLOR_FORMAT};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: brep-render-artifact <history-request.json> <out.png> [width height]");
        std::process::exit(2);
    }
    let request_path = &args[1];
    let out_path = &args[2];
    let width: u32 = args
        .get(3)
        .and_then(|value| value.parse().ok())
        .unwrap_or(640);
    let height: u32 = args
        .get(4)
        .and_then(|value| value.parse().ok())
        .unwrap_or(480);

    let request_json = match std::fs::read_to_string(request_path) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("[brep-render-artifact] cannot read {request_path}: {error}");
            std::process::exit(1);
        }
    };

    // A request that fails to PARSE — or a history that PANICS inside the
    // kernel (a pre-existing crash class some failing tests exercise) — still
    // yields an artifact (the empty background frame) so every test keeps a
    // persistent image.
    let build = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        scene_from_history_json(&request_json)
    }));
    let (scene, report) = match build {
        Ok(Ok(ok)) => ok,
        Ok(Err(error)) => {
            eprintln!("[brep-render-artifact] history failed: {error}");
            (Default::default(), Default::default())
        }
        Err(panic) => {
            let message = panic
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| panic.downcast_ref::<&str>().copied())
                .unwrap_or("kernel panic");
            eprintln!("[brep-render-artifact] history panicked: {message}");
            (Default::default(), Default::default())
        }
    };
    for error in &report.feature_errors {
        eprintln!("[brep-render-artifact] feature error: {error}");
    }
    for name in &report.unresolved {
        eprintln!("[brep-render-artifact] unresolved reference: {name}");
    }
    for error in &report.display_errors {
        eprintln!("[brep-render-artifact] display error: {error}");
    }

    let (device, queue, adapter) = match create_headless_device() {
        Ok(ok) => ok,
        Err(error) => {
            eprintln!("[brep-render-artifact] {error}");
            std::process::exit(1);
        }
    };
    if std::env::var_os("BREP_RENDER_VERBOSE").is_some() {
        eprintln!("[brep-render-artifact] adapter: {adapter}");
    }
    let mut core = RenderCore::new(device, queue, COLOR_FORMAT);
    let camera = artifact_camera(&scene.bbox(), width, height);
    let png = match core.render_to_png(&scene, &camera, width, height) {
        Ok(png) => png,
        Err(error) => {
            eprintln!("[brep-render-artifact] render failed: {error}");
            std::process::exit(1);
        }
    };
    if let Err(error) = std::fs::write(out_path, &png) {
        eprintln!("[brep-render-artifact] cannot write {out_path}: {error}");
        std::process::exit(1);
    }
}
