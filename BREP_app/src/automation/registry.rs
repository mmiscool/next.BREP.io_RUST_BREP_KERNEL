//! The state registry: every JSON blob the app publishes per frame, by name.
//!
//! This replaces the per-module `publish_to_js` helpers (which wrote
//! `window.<name>` on wasm only). A publisher hands [`publish`] a name, a
//! one-line doc, and the JSON text; a typed publisher ([`publish_typed`], with
//! the `automation` feature) also records the derived schema once. Readers —
//! the automation queue's `state_get`, the `hit_rects` merge, the generated
//! docs — see the same map. On wasm every publish still mirrors to
//! `window.<name>` so the browser build stays inspectable from a devtools
//! console; that mirror is why the verify scripts keep working unchanged.
//!
//! The registry is process-global, like the `window.*` globals it replaces:
//! one app instance per process is the only configuration that exists (a host
//! owns exactly one). Publishing is gated by [`enabled`] so a plain native run
//! never pays for serialising the blobs; a host enables it, and wasm has it on
//! by default.
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, MutexGuard};

/// One published blob.
#[derive(Debug, Clone)]
pub struct Published {
    pub json: String,
    pub doc: &'static str,
    /// The derived JSON Schema when the blob was published from a typed value;
    /// `None` = "by example" (a `serde_json::json!`-built blob).
    pub schema: Option<serde_json::Value>,
}

#[derive(Debug, Default)]
pub struct Registry {
    map: BTreeMap<String, Published>,
}

static REGISTRY: Mutex<Registry> = Mutex::new(Registry { map: BTreeMap::new() });
static ENABLED: AtomicBool = AtomicBool::new(cfg!(target_arch = "wasm32"));

/// Are publishers active this frame? Off in a plain native run; a host turns
/// it on; always on for the wasm build (its globals are part of the page).
pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

pub fn set_enabled(on: bool) {
    ENABLED.store(on, Ordering::Relaxed);
}

/// The registry, locked. Keep the guard short.
pub fn lock() -> MutexGuard<'static, Registry> {
    REGISTRY.lock().unwrap_or_else(|p| p.into_inner())
}

/// Empty the registry.
///
/// The map is process-global but an app instance is not: a native HOST can
/// build one app, stop it, and build the next in the same process (that is what
/// the test-mcp runner does with every script). Everything the stopped app
/// published stays readable — `state_get`, `hit_rects` and `hit_keys_check`
/// would answer the NEW session from the OLD one's frame for any key the new
/// app has not published yet. Clearing at the host boundary makes a session's
/// first frame the only thing its readers can see. There is no race with the
/// app being replaced: `HostHandle::stop` joins the host thread, so the previous
/// app is past its last frame before the next `spawn` clears the map. (Nothing
/// calls this on wasm, where a page really does hold exactly one app, and the
/// `window.*` mirror is the page's own.)
pub fn clear() {
    lock().map.clear();
}

/// Publish an untyped JSON string (schema by example).
///
/// A value EQUAL to the one already published costs a comparison and stops
/// there: no copy into the stored blob, and — the expensive half — no new JS
/// string. See [`Registry::insert`].
pub fn publish(name: &str, doc: &'static str, json: &str) {
    if lock().insert(name, doc, json, None) {
        mirror_to_js(name, json);
    }
}

/// Publish a typed value; its JSON Schema is recorded the first time.
#[cfg(feature = "automation")]
pub fn publish_typed<T: serde::Serialize + schemars::JsonSchema>(name: &str, doc: &'static str, value: &T) {
    let json = serde_json::to_string(value).unwrap_or_else(|_| "null".into());
    let mut r = lock();
    let schema = if r.map.contains_key(name) { None } else { serde_json::to_value(schemars::schema_for!(T)).ok() };
    let changed = r.insert(name, doc, &json, schema);
    drop(r);
    if changed {
        mirror_to_js(name, &json);
    }
}

impl Registry {
    /// Store `json` under `name`; returns whether the VALUE changed (a first
    /// publish counts as a change).
    ///
    /// Most blobs are byte-identical frame after frame — a camera spin changes
    /// the camera and nothing else — and the ones that are not small are very
    /// much not small: the rolled-to feature's params carry an imported model's
    /// embedded text. Overwriting an unchanged blob is a copy of that text per
    /// frame here, and on the browser build ALSO a fresh JS string per frame
    /// (`mirror_to_js` transcodes UTF-8 to UTF-16), on the one thread that has
    /// to answer the pointer. A compare is O(n) too, but it is a memcmp against
    /// a hot buffer that allocates nothing and, for the common case, ends at
    /// the first differing byte.
    fn insert(&mut self, name: &str, doc: &'static str, json: &str, schema: Option<serde_json::Value>) -> bool {
        match self.map.get_mut(name) {
            Some(p) => {
                // The doc line and the schema are cheap and may legitimately
                // arrive later than the first value, so they refresh either way.
                p.doc = doc;
                if schema.is_some() {
                    p.schema = schema;
                }
                if p.json == json {
                    return false;
                }
                p.json.clear();
                p.json.push_str(json);
                true
            }
            None => {
                self.map.insert(name.to_string(), Published { json: json.to_string(), doc, schema });
                true
            }
        }
    }

    pub fn get(&self, name: &str) -> Option<&Published> {
        self.map.get(name)
    }

    pub fn names(&self) -> Vec<&str> {
        self.map.keys().map(String::as_str).collect()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, &Published)> {
        self.map.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// Every hit-rect blob merged under `panel/key`, in egui points. The panel
    /// name derives from the blob name (`__brepSceneHit` → `scene`); the
    /// historical exceptions (`__brepHit` is the history panel, `__brepToolbar`
    /// and `__brepWorkbenchToolbar` carry no `Hit` suffix) are named once in
    /// [`hit_blob_panel`].
    pub fn hit_rects(&self, prefix: Option<&str>) -> BTreeMap<String, [f32; 4]> {
        let mut out = BTreeMap::new();
        for (name, p) in &self.map {
            let Some(panel) = hit_blob_panel(name) else { continue };
            let Ok(serde_json::Value::Object(map)) = serde_json::from_str::<serde_json::Value>(&p.json) else { continue };
            for (key, rect) in map {
                let full = format!("{panel}/{key}");
                if let Some(pre) = prefix {
                    if !full.starts_with(pre) {
                        continue;
                    }
                }
                if let Some(a) = rect.as_array() {
                    if a.len() == 4 {
                        let f = |i: usize| a[i].as_f64().unwrap_or(0.0) as f32;
                        out.insert(full, [f(0), f(1), f(2), f(3)]);
                    }
                }
            }
        }
        out
    }
}

/// The panel a hit-rect blob belongs to, or `None` for a state blob.
pub fn hit_blob_panel(name: &str) -> Option<String> {
    match name {
        "__brepHit" => Some("history".into()),
        "__brepToolbar" => Some("toolbar".into()),
        "__brepWorkbenchToolbar" => Some("wbtoolbar".into()),
        _ => name
            .strip_prefix("__brep")
            .and_then(|n| n.strip_suffix("Hit"))
            .filter(|n| !n.is_empty())
            .map(|n| n.to_ascii_lowercase()),
    }
}

#[cfg(target_arch = "wasm32")]
fn mirror_to_js(name: &str, json: &str) {
    if let Some(win) = web_sys::window() {
        let _ = js_sys::Reflect::set(
            &win,
            &wasm_bindgen::JsValue::from_str(name),
            &wasm_bindgen::JsValue::from_str(json),
        );
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn mirror_to_js(_name: &str, _json: &str) {}

