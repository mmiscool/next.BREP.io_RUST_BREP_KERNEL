//! What this build is running on — the ONE record of it.
//!
//! Two places need to say which renderer is in use: the Info window (so a user
//! can read it) and the problem report (so a triager can read it about a user
//! who is not there to ask). They must never be able to disagree, so there is
//! one collector — this — and both read the SAME [`Diagnostics`] instance. The
//! window renders [`Diagnostics::rows`]; the report embeds
//! [`Diagnostics::report_text`], which is those very rows joined. A row added
//! here appears in both, and neither can carry a value the other does not.
//!
//! # Why it is captured, never probed
//!
//! The renderer is CHOSEN once, at startup, by eframe: `wgpu`'s
//! `new_instance_with_webgpu_detection` drops `BROWSER_WEBGPU` from the backend
//! set when the browser has no WebGPU adapter (or the page is not a secure
//! context), so the instance falls through to WebGL2. That choice is a fact of
//! the running session, and the adapter that made it is the one drawing every
//! frame — so we read it where eframe hands it to us
//! ([`Diagnostics::from_render_state`], called from `BrepApp::new_with`) and
//! keep it.
//!
//! Asking the question a SECOND time later — a fresh `request_adapter`, say —
//! would be a different question with its own answer: an adapter enumeration at
//! report time can succeed where the startup one failed (or pick differently),
//! and a diagnostic that disagrees with what is on screen is worse than none at
//! all, precisely in the case someone is filing a bug about the renderer.
//!
//! # What it may contain
//!
//! Hardware and build facts only: the backend, the adapter, the one adapter
//! limit that bounds what the viewport can allocate, the app version and the
//! target. The report already sends a screenshot, the model and an optional
//! email; this adds nothing that identifies a PERSON, and it must stay that way
//! — a report's description is served publicly by the reports endpoint.

use eframe::egui_wgpu::RenderState;

/// The startup facts about this session's renderer and build.
///
/// Constructed ONCE (`BrepApp::new_with`) and then read; there is no refresh,
/// because nothing it records can change without restarting the app.
#[derive(Debug, Clone, PartialEq)]
pub struct Diagnostics {
    /// The backend in the words a user would use: `WebGPU`, `WebGL2`, `Vulkan`,
    /// `Metal`, `Direct3D 12`, `OpenGL`. This is THE answer to "is it WebGPU or
    /// the WebGL fallback".
    renderer: String,
    /// wgpu's own name for the same backend (`BrowserWebGpu`, `Gl`, …), kept
    /// beside the friendly one so a report can be matched against wgpu's docs
    /// and issues without guessing at the translation.
    backend: String,
    /// The adapter's self-reported name (`NVIDIA GeForce RTX 3060`, `llvmpipe
    /// (LLVM 15.0.7, 256 bits)`, `ANGLE (Intel, …)`).
    adapter: String,
    /// `DiscreteGpu` / `IntegratedGpu` / `Cpu` / `Other` — a software rasteriser
    /// answers a whole class of "why is it slow" reports on its own.
    device_type: String,
    /// Driver name + version, when the backend reports one. The browser
    /// backends usually do not, so this row is omitted when empty rather than
    /// shown blank.
    driver: String,
    /// The largest 2D texture edge the ADAPTER supports. The viewport allocates
    /// its offscreen colour/depth targets at the viewport's pixel size, so this
    /// is the hardware ceiling on how large a window the 3D view can be drawn
    /// into.
    ///
    /// The adapter's, deliberately, not the device's: eframe requests a FIXED
    /// `max_texture_dimension_2d: 8192` for every device it creates (see
    /// `egui_wgpu::WgpuSetupCreateNew`), so `device.limits()` would read 8192 in
    /// every report ever filed and say nothing about the machine. The rest of
    /// that requested set is `Limits::default()` — or `downlevel_webgl2_defaults`
    /// on the GL backend — and is a constant of the eframe version for the same
    /// reason, which is why no other limit is reported. (`max_buffer_size` is
    /// worse than constant: wgpu-hal's GLES adapter reports `i32::MAX` for it,
    /// a sentinel, on exactly the WebGL2 path this diagnostic exists for.)
    max_texture_dimension_2d: u32,
    /// The application version (`CARGO_PKG_VERSION`), so a report names the
    /// build it came from.
    version: &'static str,
    /// `wasm32 (browser)` or `native <os>/<arch>` — which of the two shells this
    /// is, since they take different code paths into the same renderer.
    platform: String,
}

impl Diagnostics {
    /// Capture the facts from the render state eframe built — the adapter and
    /// device that are ACTUALLY drawing this session.
    pub fn from_render_state(render_state: &RenderState) -> Self {
        let info = render_state.adapter.get_info();
        let limits = render_state.adapter.limits();
        Self {
            renderer: renderer_name(info.backend).to_string(),
            backend: format!("{:?}", info.backend),
            adapter: info.name.clone(),
            device_type: format!("{:?}", info.device_type),
            driver: match (info.driver.trim(), info.driver_info.trim()) {
                ("", "") => String::new(),
                ("", detail) => detail.to_string(),
                (name, "") => name.to_string(),
                (name, detail) => format!("{name} {detail}"),
            },
            max_texture_dimension_2d: limits.max_texture_dimension_2d,
            version: env!("CARGO_PKG_VERSION"),
            platform: platform(),
        }
    }

    /// The rows, in display order: `(label, value)`. The Info window draws
    /// these, [`Self::report_text`] joins these, and
    /// [`Self::json`] keys off these — so no reader can show a field another
    /// one lacks.
    ///
    /// A row whose value is empty is dropped (a browser adapter reports no
    /// driver string, and a blank row reads as a missing value rather than an
    /// absent one).
    pub fn rows(&self) -> Vec<(&'static str, String)> {
        let rows = [
            ("Renderer", self.renderer.clone()),
            ("wgpu backend", self.backend.clone()),
            ("Adapter", self.adapter.clone()),
            ("Device type", self.device_type.clone()),
            ("Driver", self.driver.clone()),
            ("Max texture size", format!("{} px", self.max_texture_dimension_2d)),
            ("App version", self.version.to_string()),
            ("Platform", self.platform.clone()),
        ];
        rows.into_iter().filter(|(_, value)| !value.is_empty()).collect()
    }

    /// The rows as one plain-text block, `label: value` per line, under a
    /// heading. This is what a problem report carries; the window shows the
    /// same rows in a grid.
    pub fn report_text(&self) -> String {
        let mut text = String::from("--- diagnostics ---");
        for (label, value) in self.rows() {
            text.push_str(&format!("\n{label}: {value}"));
        }
        text
    }

    /// The rows as a JSON object, for the automation surface (`diagnostics`)
    /// and the `__brepDiagnostics` state blob.
    pub fn json(&self) -> serde_json::Value {
        serde_json::Value::Object(
            self.rows()
                .into_iter()
                .map(|(label, value)| (label.to_string(), serde_json::Value::String(value)))
                .collect(),
        )
    }

    /// The renderer in a user's words (`WebGPU` / `WebGL2` / `Vulkan` / …).
    pub fn renderer(&self) -> &str {
        &self.renderer
    }

    /// One line naming the adapter and how it is reached — what the MCP banner
    /// prints for the host that started the app. It reads this rather than
    /// formatting `AdapterInfo` a second time, so the server's banner and the
    /// user's Info window cannot name different hardware.
    pub fn adapter_line(&self) -> String {
        format!("{} ({}, {})", self.adapter, self.backend, self.device_type)
    }
}

/// What a user calls the backend wgpu picked. `Gl` is the interesting one: in
/// the browser it IS WebGL2 (the fallback path), natively it is desktop GL.
fn renderer_name(backend: wgpu::Backend) -> &'static str {
    match backend {
        wgpu::Backend::BrowserWebGpu => "WebGPU",
        wgpu::Backend::Gl if cfg!(target_arch = "wasm32") => "WebGL2",
        wgpu::Backend::Gl => "OpenGL",
        wgpu::Backend::Vulkan => "Vulkan",
        wgpu::Backend::Metal => "Metal",
        wgpu::Backend::Dx12 => "Direct3D 12",
        wgpu::Backend::Noop => "none (no-op device)",
    }
}

/// Which shell this is. The browser build has no `std::env::consts` worth
/// printing (`unknown` OS), so it names itself instead.
fn platform() -> String {
    #[cfg(target_arch = "wasm32")]
    {
        "wasm32 (browser)".to_string()
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        format!("native {}/{}", std::env::consts::OS, std::env::consts::ARCH)
    }
}

