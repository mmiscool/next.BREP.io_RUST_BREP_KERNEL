//! Headless demo for the dimension leaders (R29) and curve display (R30).
//!
//! Renders, into one PNG, a linear distance dimension, an angular dimension, a
//! sampled spline polyline (with control-point handles) and a radial dimension,
//! all in the world XY plane viewed top-down so they read like CAD dimension
//! leaders. Run: `cargo run --bin dimension-demo -- out.png`.
use brep_gizmos::curve_display::{
    control_point_handles, polyline_display, CURVE_COLOR, HANDLE_COLOR,
};
use brep_gizmos::dimension::{angular_dimension, linear_dimension, radial_dimension};
use brep_gizmos::{raster, GizmoCamera, Overlay, Vec3};

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "dimension_demo.png".into());
    let (w, h) = (960u32, 640u32);

    // Top-down ortho over the XY plane: clearest CAD read.
    let view_proj = raster::test_view_proj([0.0, 0.0, 30.0], [0.0, 0.0, 0.0], w as f32, h as f32);
    let camera = GizmoCamera {
        view_proj,
        eye: Vec3::new(0.0, 0.0, 30.0),
        forward: Vec3::new(0.0, 0.0, -1.0),
        // test_view_proj's up for the near-vertical -Z pose (+Y fallback).
        up: Vec3::Y,
        viewport: [w as f32, h as f32],
        orthographic: true,
    };

    let mut overlay = Overlay::new();

    // 1) Linear distance dimension across the bottom.
    let a = Vec3::new(-6.0, -2.0, 0.0);
    let b = Vec3::new(6.0, -2.0, 0.0);
    let lin = linear_dimension(a, b, Vec3::new(0.0, -1.0, 0.0), 1.6, &camera);
    overlay.extend(&lin.overlay);
    // Mark the measured feature (the two witness endpoints) faintly.
    overlay.line(
        Vec3::new(-6.0, -1.7, 0.0),
        Vec3::new(-6.0, -2.3, 0.0),
        [0.5, 0.5, 0.55, 1.0],
    );
    overlay.line(
        Vec3::new(6.0, -1.7, 0.0),
        Vec3::new(6.0, -2.3, 0.0),
        [0.5, 0.5, 0.55, 1.0],
    );

    // 2) Angular dimension top-left (70 degrees).
    let vertex = Vec3::new(-4.5, 2.0, 0.0);
    let dir_a = Vec3::new(1.0, 0.0, 0.0);
    let ang70 = 70.0_f32.to_radians();
    let dir_b = Vec3::new(ang70.cos(), ang70.sin(), 0.0);
    let ang = angular_dimension(vertex, dir_a, dir_b, 2.6, &camera);
    overlay.extend(&ang.overlay);

    // 3) Sampled spline polyline top-right (a smooth wave) + control handles.
    let ctrl: Vec<Vec3> = vec![
        Vec3::new(1.0, 3.6, 0.0),
        Vec3::new(2.5, 1.6, 0.0),
        Vec3::new(4.0, 3.4, 0.0),
        Vec3::new(5.5, 1.8, 0.0),
        Vec3::new(7.0, 3.2, 0.0),
    ];
    let sampled = catmull_rom(&ctrl, 24);
    overlay.extend(&polyline_display(&sampled, CURVE_COLOR, false));
    overlay.extend(&control_point_handles(&ctrl, &camera, HANDLE_COLOR));

    // 4) Radial dimension bottom-right: a circle with an R leader.
    let center = Vec3::new(4.5, -0.5, 0.0);
    let radius = 1.6_f32;
    let circle = circle_points(center, radius, 48);
    overlay.extend(&polyline_display(&circle, [0.55, 0.85, 0.6, 1.0], true));
    let dir = Vec3::new(0.8, 0.6, 0.0).normalized();
    let pc = center.add(dir.scale(radius));
    let rad = radial_dimension(center, pc, &camera);
    overlay.extend(&rad.overlay);

    let png = raster::render_overlay_png(&overlay, &camera, w, h);
    std::fs::write(&out, png).expect("write png");

    // Report where the text labels would be pinned (engine does this via
    // world_to_screen at integration time).
    for (name, anchor) in [
        ("linear", lin.label_anchor),
        ("angular", ang.label_anchor),
        ("radial", rad.label_anchor),
    ] {
        if let Some(s) = camera.world_to_screen(anchor) {
            eprintln!("label[{name}] anchor world={anchor:?} screen=({:.1},{:.1})", s[0], s[1]);
        }
    }
    eprintln!("wrote {out}");
}

/// A tiny Catmull-Rom sampler so the demo's spline reads as a smooth curve
/// (stands in for the kernel's NURBS sampling the engine would do).
fn catmull_rom(ctrl: &[Vec3], per_seg: usize) -> Vec<Vec3> {
    if ctrl.len() < 2 {
        return ctrl.to_vec();
    }
    let mut out = Vec::new();
    let n = ctrl.len();
    for i in 0..n - 1 {
        let p0 = ctrl[i.saturating_sub(1)];
        let p1 = ctrl[i];
        let p2 = ctrl[i + 1];
        let p3 = ctrl[(i + 2).min(n - 1)];
        for j in 0..per_seg {
            let t = j as f32 / per_seg as f32;
            out.push(catmull(p0, p1, p2, p3, t));
        }
    }
    out.push(ctrl[n - 1]);
    out
}

fn catmull(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f32) -> Vec3 {
    let t2 = t * t;
    let t3 = t2 * t;
    // 0.5 * ( (2 p1) + (-p0 + p2) t + (2p0 -5p1 +4p2 -p3) t^2 + (-p0 +3p1 -3p2 +p3) t^3 )
    let a = p1.scale(2.0);
    let b = p2.sub(p0).scale(t);
    let c = p0
        .scale(2.0)
        .add(p1.scale(-5.0))
        .add(p2.scale(4.0))
        .sub(p3)
        .scale(t2);
    let d = p0
        .scale(-1.0)
        .add(p1.scale(3.0))
        .add(p2.scale(-3.0))
        .add(p3)
        .scale(t3);
    a.add(b).add(c).add(d).scale(0.5)
}

fn circle_points(center: Vec3, radius: f32, n: usize) -> Vec<Vec3> {
    (0..n)
        .map(|i| {
            let ang = (i as f32 / n as f32) * std::f32::consts::TAU;
            center.add(Vec3::new(ang.cos() * radius, ang.sin() * radius, 0.0))
        })
        .collect()
}
