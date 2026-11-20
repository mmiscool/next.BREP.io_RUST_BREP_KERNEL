//! SpaceMouse / Spaceball navigation: a 6-DoF cap that moves the 3D view.
//!
//! Everything but the transport is Rust and shared by both builds:
//!
//! - [`parse`] reads one raw HID input report (the layout below),
//! - the [`Hub`] holds the device's latest axes and buttons,
//! - [`motion`] turns axes + settings + a time step into a
//!   [`SixDofMotion`], which [`drive`] applies each frame through the
//!   engine's own pan / zoom / orbit primitives (so the orbit centre, the view
//!   cube and the screen-constant overlays follow it as they follow a drag).
//!
//! The transports only deliver raw reports to [`feed`]:
//!
//! - native (feature `spacemouse`): a reader thread over the `hidapi` crate's
//!   pure-Rust backends (Linux hidraw, Windows HID), `native`;
//! - web: `web/spacemouse.js`, a small WebHID shim, calls the
//!   `#[wasm_bindgen]` exports in `web`. WebHID needs a user gesture to
//!   grant a device once; after that the shim reopens it on every load.
//! - automation: the `spacemouse_report` command injects reports, so a test
//!   drives the same path without hardware.
//!
//! HID report layout (3Dconnexion / Logitech 6-DoF devices; the report ID is
//! the first byte of a hidraw/hidapi read and separate on WebHID):
//!
//! | ID | Payload                   | Devices                                  |
//! |----|---------------------------|------------------------------------------|
//! | 1  | 3 × i16 LE: x, y, z       | older (split translation / rotation)     |
//! | 1  | 6 × i16 LE: x y z rx ry rz| newer (one motion report)                |
//! | 2  | 3 × i16 LE: rx, ry, rz    | older                                    |
//! | 3  | button bitmask, LE        | all                                      |
//!
//! The device frame is x right, y toward the user, z down; the values run to
//! about ±350 at full deflection (older devices a little further).
use brep_render::controls::SixDofMotion;
use brep_render::engine_state::EngineState;
use brep_render::style::RenderSettings;
use std::sync::Mutex;

/// Full deflection, in raw report units.
pub const FULL_SCALE: f64 = 350.0;
/// Deflection, as a fraction of [`FULL_SCALE`], read as rest: the cap never
/// reports exactly zero while a hand rests on it.
const DEADZONE: f64 = 0.04;
/// Rotation at full deflection and speed 1, radians per second.
const ROTATE_RAD_PER_S: f64 = 2.0;
/// Pan at full deflection and speed 1, in viewport sizes (the smaller side)
/// per second.
const PAN_VIEWS_PER_S: f64 = 1.0;
/// Zoom at full deflection and speed 1: e-folds of the view size per second.
const ZOOM_E_PER_S: f64 = 1.5;
/// The longest time step one frame integrates, so a stalled frame does not
/// fling the view.
const MAX_DT: f64 = 0.05;

/// One parsed input report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Report {
    Translation([i16; 3]),
    Rotation([i16; 3]),
    Motion([i16; 6]),
    Buttons(u32),
}

fn axes<const N: usize>(payload: &[u8]) -> [i16; N] {
    std::array::from_fn(|i| i16::from_le_bytes([payload[2 * i], payload[2 * i + 1]]))
}

/// Parse one report: its ID and its payload (without the ID byte). The layout
/// is chosen by length, not by vendor: some newer devices still send the split
/// reports.
pub fn parse(report_id: u8, payload: &[u8]) -> Option<Report> {
    match report_id {
        1 if payload.len() >= 12 => Some(Report::Motion(axes(payload))),
        1 if payload.len() >= 6 => Some(Report::Translation(axes(payload))),
        2 if payload.len() >= 6 => Some(Report::Rotation(axes(payload))),
        3 if !payload.is_empty() => {
            let mut bits = [0u8; 4];
            let n = payload.len().min(4);
            bits[..n].copy_from_slice(&payload[..n]);
            Some(Report::Buttons(u32::from_le_bytes(bits)))
        }
        _ => None,
    }
}

/// Parse a hidraw / hidapi read, whose first byte is the report ID.
pub fn parse_numbered(bytes: &[u8]) -> Option<Report> {
    let (&id, payload) = bytes.split_first()?;
    parse(id, payload)
}

/// The device's live state.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Hub {
    /// tx, ty, tz, rx, ry, rz in raw units, held until the next report.
    pub axes: [i16; 6],
    pub buttons: u32,
    /// Buttons pressed since the last frame took them.
    pressed: u32,
    /// The open device's name, or `None` while none is open.
    pub device: Option<String>,
    /// The transport's last notice (no WebHID, permission denied, …).
    pub status: Option<String>,
    /// Reports received since start, for the automation readback.
    pub reports: u64,
}

impl Hub {
    pub fn apply(&mut self, report: Report) {
        match report {
            Report::Translation(t) => self.axes[..3].copy_from_slice(&t),
            Report::Rotation(r) => self.axes[3..].copy_from_slice(&r),
            Report::Motion(m) => self.axes = m,
            Report::Buttons(b) => {
                self.pressed |= b & !self.buttons;
                self.buttons = b;
            }
        }
        self.reports += 1;
    }
}

static HUB: Mutex<Hub> = Mutex::new(Hub {
    axes: [0; 6],
    buttons: 0,
    pressed: 0,
    device: None,
    status: None,
    reports: 0,
});
static REPAINT: Mutex<Option<egui::Context>> = Mutex::new(None);

fn hub() -> std::sync::MutexGuard<'static, Hub> {
    HUB.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn wake() {
    if let Some(ctx) = REPAINT.lock().ok().and_then(|c| c.clone()) {
        ctx.request_repaint();
    }
}

/// Hand the module the egui context (so a report arriving while the app is
/// idle wakes a frame) and start the native reader.
pub fn install(ctx: &egui::Context) {
    if let Ok(mut slot) = REPAINT.lock() {
        *slot = Some(ctx.clone());
    }
    #[cfg(all(not(target_arch = "wasm32"), feature = "spacemouse"))]
    native::start();
}

/// Deliver one raw report from any transport. Returns whether it parsed.
pub fn feed(report_id: u8, payload: &[u8]) -> bool {
    let Some(report) = parse(report_id, payload) else { return false };
    hub().apply(report);
    wake();
    true
}

/// A device opened (`Some(name)`) or closed (`None`). Closing zeroes the axes,
/// so a device unplugged mid-push does not keep the view moving.
pub fn set_device(name: Option<String>) {
    let mut hub = hub();
    if name.is_none() {
        hub.axes = [0; 6];
        hub.buttons = 0;
    } else {
        hub.status = None;
    }
    hub.device = name;
    drop(hub);
    wake();
}

pub fn set_status(status: Option<String>) {
    hub().status = status;
    wake();
}

pub fn snapshot() -> Hub {
    hub().clone()
}

/// One sentence for the settings window: the device, or why there is none.
pub fn status_line() -> String {
    let hub = hub();
    match (&hub.device, &hub.status) {
        (Some(name), _) => format!("Connected: {name}"),
        (None, Some(status)) => status.clone(),
        #[cfg(target_arch = "wasm32")]
        (None, None) => "No device connected. Connect grants this page access to one.".into(),
        #[cfg(not(target_arch = "wasm32"))]
        (None, None) => "No device found. A SpaceMouse is picked up when it is plugged in.".into(),
    }
}

/// Ask for a device. On the web this is WebHID's chooser (it must run from a
/// click: the shim's `request` is called within the click's activation). The
/// native reader finds devices on its own, so there it does nothing.
pub fn request_device() {
    #[cfg(target_arch = "wasm32")]
    web::request();
}

/// One raw axis as a signed fraction of full deflection, past the deadzone
/// (rescaled so motion starts from zero at its edge), flipped if `invert`.
fn axis(raw: i16, invert: bool) -> f64 {
    let v = (f64::from(raw) / FULL_SCALE).clamp(-1.5, 1.5);
    let magnitude = ((v.abs() - DEADZONE) / (1.0 - DEADZONE)).max(0.0);
    let v = magnitude.copysign(v);
    if invert { -v } else { v }
}

/// The view motion for one frame: `axes` held for `dt` seconds over a
/// viewport whose smaller side is `view_px` CSS px.
///
/// Object mode — the model follows the cap. Device frame (x right, y toward
/// the user, z down) to screen (x right, y up, z toward the viewer):
/// - x → pan right; z (press) → pan down; y (pull toward you) → zoom in;
/// - rotation vector (rx, ry, rz) → (rx, -rz, ry) in camera space, the
///   proper rotation that takes the device frame to the screen frame.
pub fn motion(axes_raw: [i16; 6], settings: &RenderSettings, dt: f64, view_px: f64) -> SixDofMotion {
    let speed = f64::from(settings.spacemouse_sensitivity) * dt.clamp(0.0, MAX_DT);
    let a: [f64; 6] = std::array::from_fn(|i| axis(axes_raw[i], settings.spacemouse_invert[i]));
    let pan_px = PAN_VIEWS_PER_S * view_px.max(1.0) * speed;
    let rot = ROTATE_RAD_PER_S * speed;
    SixDofMotion {
        pan: [a[0] * pan_px, a[2] * pan_px],
        zoom: (-a[1] * ZOOM_E_PER_S * speed).exp(),
        rotate: [a[3] * rot, -a[5] * rot, a[4] * rot],
    }
}

/// Apply this frame's device motion and button presses to the 3D view. Called
/// by the viewport only while it draws the 3D scene.
pub fn drive(ctx: &egui::Context, state: &mut EngineState) {
    let (axes_raw, pressed) = {
        let mut hub = hub();
        (hub.axes, std::mem::take(&mut hub.pressed))
    };
    if !state.settings.spacemouse_enabled {
        return;
    }
    // Natively the reader hears the device whichever window is in front, so
    // only the focused window follows it (the cap may be driving another
    // program). On the web eframe's `focused` is the CANVAS's DOM focus, which
    // the Connect chooser and the page's own buttons take; a hidden tab
    // already draws no frames, so the web needs no gate.
    #[cfg(not(target_arch = "wasm32"))]
    if !ctx.input(|i| i.focused) {
        return;
    }
    // Two buttons on every model: the left snaps to the isometric view, the
    // right fits the model (3Dconnexion's own default for its right button).
    // The isometric snap turns the view, so a locked sketch camera ignores it.
    if pressed & 1 != 0 && !state.camera_rotation_locked() {
        state.standard_view("ISO");
    }
    if pressed & 2 != 0 {
        state.zoom_to_fit();
    }
    if axes_raw == [0; 6] {
        return;
    }
    let dt = f64::from(ctx.input(|i| i.stable_dt));
    let view_px = state.camera.width.min(state.camera.height);
    let m = motion(axes_raw, &state.settings, dt, view_px);
    if !m.is_idle() {
        state.six_dof(&m);
    }
    // The cap is held off centre: keep frames coming until it is released.
    ctx.request_repaint();
}

/// The native transport: one background thread that finds a 6-DoF device,
/// reads it until it goes away, and looks again.
#[cfg(all(not(target_arch = "wasm32"), feature = "spacemouse"))]
mod native {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    static STARTED: AtomicBool = AtomicBool::new(false);

    /// 3Dconnexion devices that are not 6-DoF (CadMouse, keyboards, numpad),
    /// which a vendor match must not take.
    const NOT_SIX_DOF: &[u16] = &[0xc650, 0xc651, 0xc654, 0xc655, 0xc656, 0xc657, 0xc658, 0xc664, 0xc668, 0xc665, 0xc62c, 0xc641];
    /// Logitech-era 6-DoF devices (vendor 0x046d, shared with every Logitech
    /// keyboard and mouse, so matched by product only).
    const LOGITECH_SIX_DOF: &[u16] = &[0xc603, 0xc605, 0xc606, 0xc621, 0xc623, 0xc625, 0xc626, 0xc627, 0xc628, 0xc629, 0xc62b, 0xc640];

    /// A 6-DoF controller: HID's Generic Desktop / Multi-axis Controller
    /// collection (what the WebHID shim filters on), or a known product. A
    /// 3Dconnexion collection must be the multi-axis one whenever the backend
    /// reports a usage (usage page 0 = it could not read the descriptor); only
    /// then does the not-6-DoF product list decide.
    pub(super) fn is_six_dof(vendor: u16, product: u16, usage_page: u16, usage: u16) -> bool {
        let multi_axis = (usage_page, usage) == (1, 8);
        match vendor {
            0x046d => LOGITECH_SIX_DOF.contains(&product),
            0x256f if usage_page != 0 => multi_axis,
            0x256f => !NOT_SIX_DOF.contains(&product),
            _ => multi_axis,
        }
    }

    pub(super) fn start() {
        if STARTED.swap(true, Ordering::SeqCst) {
            return;
        }
        let _ = std::thread::Builder::new().name("spacemouse".into()).spawn(run);
    }

    fn run() {
        let mut api = match hidapi::HidApi::new() {
            Ok(api) => api,
            Err(err) => {
                super::set_status(Some(format!("SpaceMouse support is unavailable: {err}")));
                return;
            }
        };
        let mut last_failure: Option<String> = None;
        loop {
            let _ = api.refresh_devices();
            // A device can list several collections; prefer the multi-axis one.
            let mut found: Vec<&hidapi::DeviceInfo> = api
                .device_list()
                .filter(|d| is_six_dof(d.vendor_id(), d.product_id(), d.usage_page(), d.usage()))
                .collect();
            found.sort_by_key(|d| (d.usage_page(), d.usage()) != (1, 8));
            let Some(info) = found.first().map(|d| (*d).clone()) else {
                std::thread::sleep(Duration::from_secs(2));
                continue;
            };
            let name = info.product_string().filter(|s| !s.is_empty()).unwrap_or("SpaceMouse").to_string();
            match info.open_device(&api) {
                Ok(device) => {
                    last_failure = None;
                    super::set_device(Some(name));
                    read(&device);
                    super::set_device(None);
                }
                Err(err) => {
                    let message = failure(&name, &err.to_string());
                    if last_failure.as_deref() != Some(message.as_str()) {
                        super::set_status(Some(message.clone()));
                        last_failure = Some(message);
                    }
                    std::thread::sleep(Duration::from_secs(3));
                }
            }
        }
    }

    fn read(device: &hidapi::HidDevice) {
        let mut buf = [0u8; 64];
        loop {
            match device.read_timeout(&mut buf, 500) {
                Ok(0) => {}
                Ok(n) => {
                    if let Some(report) = super::parse_numbered(&buf[..n]) {
                        super::hub().apply(report);
                        super::wake();
                    }
                }
                Err(_) => return,
            }
        }
    }

    fn failure(name: &str, err: &str) -> String {
        let hint = if cfg!(target_os = "linux") {
            " On Linux, give your user access with a udev rule, e.g. \
             /etc/udev/rules.d/99-spacemouse.rules: \
             KERNEL==\"hidraw*\", ATTRS{idVendor}==\"256f\", MODE=\"0660\", TAG+=\"uaccess\" \
             (and 046d for older Logitech-branded devices), then replug it."
        } else {
            ""
        };
        format!("Found {name} but could not open it: {err}.{hint}")
    }
}

/// The web transport's entry points, called by `web/spacemouse.js`.
#[cfg(target_arch = "wasm32")]
pub mod web {
    use wasm_bindgen::prelude::*;
    use wasm_bindgen::JsCast;

    /// One WebHID `inputreport`: its `reportId` and its `data` bytes.
    #[wasm_bindgen]
    pub fn spacemouse_report(report_id: u8, data: &[u8]) -> bool {
        super::feed(report_id, data)
    }

    /// The shim opened a device (its product name) or closed it (undefined).
    #[wasm_bindgen]
    pub fn spacemouse_device(name: Option<String>) {
        super::set_device(name);
    }

    /// A notice for the settings window (no WebHID, access refused, …).
    #[wasm_bindgen]
    pub fn spacemouse_status(text: String) {
        super::set_status(Some(text));
    }

    pub(super) fn request() {
        let shim = web_sys::window()
            .and_then(|w| js_sys::Reflect::get(w.as_ref(), &JsValue::from_str("__brepSpaceMouse")).ok())
            .filter(|v| v.is_object());
        let request = shim
            .as_ref()
            .and_then(|s| js_sys::Reflect::get(s, &JsValue::from_str("request")).ok())
            .and_then(|f| f.dyn_into::<js_sys::Function>().ok());
        match (shim, request) {
            (Some(shim), Some(request)) => {
                let _ = request.call0(&shim);
            }
            _ => super::set_status(Some("SpaceMouse support did not load on this page.".into())),
        }
    }
}

