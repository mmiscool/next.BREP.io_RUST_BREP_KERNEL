//! Headless ViewCube demo: renders the orientation cube (with FRONT hovered)
//! from an iso viewpoint so all of F / R / T read at once, via the CPU
//! rasterizer, and writes a PNG. Not a production path — a way to eyeball the
//! gizmo in isolation.
//!
//! Run: `cargo run --bin viewcube-demo [out.png]`
use brep_gizmos::view_cube::ViewCube;
use brep_gizmos::{raster, GizmoCamera, Gizmo, Vec3};

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "viewcube_demo.png".into());

    // Big footprint so the eyeball render is crisp; target_view/hit are
    // size-independent, so this only affects pixels.
    let cube = ViewCube::with_size(360.0);

    // Main camera in the +X+Y+Z octant, looking at the origin — the iso view
    // that shows the FRONT (+Z), RIGHT (+X) and TOP (+Y) faces together.
    let eye = Vec3::new(7.0, 7.0, 5.5);
    let main = GizmoCamera {
        view_proj: raster::test_view_proj([eye.x, eye.y, eye.z], [0.0, 0.0, 0.0], 100.0, 100.0),
        eye,
        forward: eye.scale(-1.0).normalized(),
        // The Y-up world up an un-rolled engine camera has at this pose (what
        // the cube's old forward-only heuristic reproduced).
        up: Vec3::Y,
        viewport: [100.0, 100.0],
        orthographic: true,
    };

    let overlay = cube.geometry(&main, Some(ViewCube::FRONT), None);
    let mini = cube.mini_camera(&main);
    let s = cube.size() as u32;
    let png = raster::render_overlay_png(&overlay, &mini, s, s);
    std::fs::write(&out, &png).expect("write png");
    eprintln!(
        "wrote {out} ({} bytes) — iso view, FRONT hovered; faces shown: F (front,+Z), R (right,+X), T (top,+Y)",
        png.len()
    );
}
