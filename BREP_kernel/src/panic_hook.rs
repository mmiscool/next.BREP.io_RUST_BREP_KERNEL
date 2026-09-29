//! The wasm panic hook: a Rust panic reaches the browser console instead of
//! the bare `RuntimeError: unreachable` wasm traps produce by default.
//!
//! [`set_once`] is what every wasm entry point in the family calls first —
//! the kernel's `#[wasm_bindgen]` ABI, `brep_render::Engine::new`, the app's
//! `start` and its history worker. It replaces the `console_error_panic_hook`
//! crate, whose whole content this is; the kernel already links `wasm-bindgen`,
//! so owning it costs nothing and drops a dependency from three crates.
//!
//! On every non-wasm target it is a no-op, deliberately — and that is a change
//! from the crate it replaces. `console_error_panic_hook::set_once()` installed
//! a native hook too, one that printed the panic message alone; because the ABI
//! functions that call it also run in native tests and in `BREP_mcp`'s headless
//! host, the first ABI call there quietly cost every later panic its backtrace
//! and the `RUST_BACKTRACE` hint. Doing nothing leaves the standard hook in
//! place, which prints both.

#[cfg(target_arch = "wasm32")]
mod imp {
    use wasm_bindgen::prelude::*;

    #[wasm_bindgen]
    extern "C" {
        #[wasm_bindgen(js_namespace = console)]
        fn error(msg: String);

        /// A JS `Error`, constructed only to read the stack it captures — wasm
        /// frames do not appear in a Rust backtrace, but they do appear here.
        type Error;

        #[wasm_bindgen(constructor)]
        fn new() -> Error;

        #[wasm_bindgen(structural, method, getter)]
        fn stack(error: &Error) -> String;
    }

    /// `panic!` → `console.error`, with the JS-side stack appended.
    fn hook(info: &std::panic::PanicHookInfo<'_>) {
        // `PanicHookInfo`'s Display is already "panicked at src/x.rs:1:2:\nmsg".
        let mut msg = info.to_string();
        msg.push_str("\n\nStack:\n\n");
        msg.push_str(&Error::new().stack());
        msg.push_str("\n\n");
        error(msg);
    }

    pub fn set_once() {
        use std::sync::Once;
        static SET: Once = Once::new();
        SET.call_once(|| std::panic::set_hook(Box::new(hook)));
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod imp {
    /// No-op: the standard hook already prints the panic and a backtrace.
    pub fn set_once() {}
}

/// Install the panic hook, at most once per module instance. Cheap enough to
/// call from every entry point, which is how it is used.
pub fn set_once() {
    imp::set_once();
}
