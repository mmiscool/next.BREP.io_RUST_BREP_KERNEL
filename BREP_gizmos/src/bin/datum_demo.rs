//! Headless datum-display demo: renders a datum plane, a coordinate-frame triad
//! sitting on it, and a standalone revolve-style axis with an arrowhead — all
//! through `raster::render_overlay_png` so the datum builders can be eyeballed
//! in isolation.
//!
//! Run: `cargo run --bin datum-demo -- datum_demo.png`
use brep_gizmos::datum::{datum_axis, datum_frame, datum_plane, PLANE_COLOR};
use brep_gizmos::{raster, GizmoCamera, Overlay, Vec3};

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "datum_demo.png".into());
    let (w, h) = (720u32, 540u32);

    // Isometric-ish view, +Z up — this demo's own framing (`test_view_proj`);
    // the application's world is +Y up.
    let eye = Vec3::new(9.0, -12.0, 9.0);
    let view_proj = raster::test_view_proj([eye.x, eye.y, eye.z], [0.5, 0.0, 1.0], w as f32, h as f32);
    let camera = GizmoCamera {
        view_proj,
        eye,
        forward: Vec3::new(-9.0, 12.0, -9.0).normalized(),
        // test_view_proj's up for this oblique pose (Z-up heuristic).
        up: Vec3::Z,
        viewport: [w as f32, h as f32],
        orthographic: true,
    };

    let mut overlay = Overlay::new();

    // A datum plane in the world XY plane (origin, x=+X, y=+Y), 7 units across.
    overlay.extend(&datum_plane(Vec3::ZERO, Vec3::X, Vec3::Y, 7.0, PLANE_COLOR));

    // A coordinate-frame triad standing on the plane origin (screen-constant).
    overlay.extend(&datum_frame(
        Vec3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec3::Z,
        70.0,
        &camera,
    ));

    // A standalone revolve-style axis (construction line) with an arrowhead,
    // offset to the side and pointing up +Z.
    overlay.extend(&datum_axis(
        Vec3::new(4.0, 3.0, 0.0),
        Vec3::Z,
        5.5,
        [1.0, 0.62, 0.20, 1.0],
    ));

    let png = raster::render_overlay_png(&overlay, &camera, w, h);
    std::fs::write(&out, png).expect("write png");
    eprintln!("wrote {out} ({} tris, {} line verts)", overlay.tris.len(), overlay.lines.len());
}
