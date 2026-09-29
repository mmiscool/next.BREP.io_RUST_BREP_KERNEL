//! Headless gizmo demo harness: renders a sample scene of every registered
//! gizmo to PNGs so they can be eyeballed in isolation. Each gizmo module adds
//! its own demo entry here (or ships a per-gizmo demo).
use brep_gizmos::transform::{TransformGizmo, HANDLE_RING_Z};
use brep_gizmos::{raster, Gizmo, GizmoCamera, Vec3};

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "gizmo_demo.png".into());
    // Smaller frame so the screen-constant (~90px) gizmo fills a good fraction.
    let (w, h) = (300u32, 300u32);
    // A 3/4 CAD-style oblique view (+Z up).
    let eye = [7.0, 7.0, 5.5];
    let view_proj = raster::test_view_proj(eye, [0.0, 0.0, 0.0], w as f32, h as f32);
    let camera = GizmoCamera {
        view_proj,
        eye: Vec3::from(eye),
        forward: Vec3::new(-eye[0], -eye[1], -eye[2]).normalized(),
        // test_view_proj's up for this oblique pose (Z-up heuristic).
        up: Vec3::Z,
        viewport: [w as f32, h as f32],
        orthographic: true,
    };

    // The transform gizmo alone, with the Z rotation ring hovered (drawn gold).
    let gizmo = TransformGizmo::default();
    let overlay = gizmo.geometry(&camera, Some(HANDLE_RING_Z), None);

    let png = raster::render_overlay_png(&overlay, &camera, w, h);
    std::fs::write(&out, png).expect("write png");
    eprintln!("wrote {out} ({} lines, {} tris)", overlay.lines.len(), overlay.tris.len());
}
