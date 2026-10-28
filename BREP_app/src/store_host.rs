//! Optional embedded-host persistence. Only a shell that installs the explicit
//! bridge before WASM boot can select this backend.
use super::mirror_store::{BackendFuture, Entry, StoreBackend};
use wasm_bindgen::{JsCast, JsValue};

pub(super) fn available() -> bool {
    web_sys::window()
        .and_then(|w| js_sys::Reflect::get(&w, &"__brepHostRequest".into()).ok())
        .is_some_and(|v| v.is_function())
}

pub(super) async fn request(value: serde_json::Value) -> Result<serde_json::Value, String> {
    let window = web_sys::window().ok_or("no window")?;
    let function = js_sys::Reflect::get(&window, &"__brepHostRequest".into())
        .map_err(|e| format!("host bridge: {e:?}"))?
        .dyn_into::<js_sys::Function>()
        .map_err(|_| "host bridge unavailable")?;
    let result = function
        .call1(&window, &JsValue::from_str(&value.to_string()))
        .map_err(|e| format!("host request: {e:?}"))?;
    let result = wasm_bindgen_futures::JsFuture::from(js_sys::Promise::resolve(&result))
        .await
        .map_err(|e| format!("host request: {e:?}"))?;
    serde_json::from_str(
        &result
            .as_string()
            .ok_or("host returned non-JSON response")?,
    )
    .map_err(|e| e.to_string())
}

pub(super) struct HostBackend;
impl StoreBackend for HostBackend {
    fn label(&self) -> String {
        "OrcaSlicer local workspace".into()
    }
    fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
        Box::pin(async { Err("host storage uses a lazy index".into()) })
    }
    fn load_index(&self) -> BackendFuture<Vec<(String, Entry)>> {
        Box::pin(async {
            let rows: Vec<(String, u64, Option<String>)> =
                serde_json::from_value(request(serde_json::json!({"op": "index"})).await?)
                    .map_err(|e| e.to_string())?;
            Ok(rows
                .into_iter()
                .map(|(key, bytes, value)| {
                    (
                        key,
                        value
                            .map(Entry::Resident)
                            .unwrap_or(Entry::OnDisk { bytes }),
                    )
                })
                .collect())
        })
    }
    fn get(&self, key: &str) -> BackendFuture<Option<String>> {
        let key = key.to_owned();
        Box::pin(async move {
            serde_json::from_value(request(serde_json::json!({"op":"get", "key":key})).await?)
                .map_err(|e| e.to_string())
        })
    }
    fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
        let payload = serde_json::json!({"op":"put", "key":key, "value":value});
        Box::pin(async move { request(payload).await.map(|_| ()) })
    }
    fn delete(&self, key: &str) -> BackendFuture<()> {
        let payload = serde_json::json!({"op":"delete", "key":key});
        Box::pin(async move { request(payload).await.map(|_| ()) })
    }
}
