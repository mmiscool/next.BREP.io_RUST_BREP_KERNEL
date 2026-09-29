//! `render_mesh` — the native STEP-validation renderer as a command line.
//!
//! A CPU software rasteriser: no GPU, no window, no browser. It renders a
//! triangle mesh `{positions, indices}` to a PNG with the same isometric Z-up
//! camera, framing modes and silhouette/inside-out measurements as the
//! three.js renderer it replaces (`brep_mcp_core::render_mesh`). The same
//! command is available as `brep-mcp render-mesh`; this binary exists so the
//! step-validation scripts can call it without building the MCP server's app
//! host.
//!
//!     cargo build --release --manifest-path BREP_mcp_core/Cargo.toml --example render_mesh
//!     BREP_mcp_core/target/release/examples/render_mesh mesh.json out.png --normalize
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match brep_mcp_core::render_mesh::cli(&args) {
        Ok(out) => {
            if !out.is_empty() {
                println!("{out}");
            }
        }
        Err(e) => {
            eprintln!("render-mesh: {e}");
            std::process::exit(2);
        }
    }
}
