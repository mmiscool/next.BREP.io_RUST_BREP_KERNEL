# BREP_script_core — a small, bounded JavaScript call on Boa

`BREP_script_core` runs one JavaScript file on the pure-Rust
[Boa](https://crates.io/crates/boa_engine) engine, calls one global function
with one JSON argument, and hands back the function's JSON result or a
message a person can read. It is the interpreter behind the BREP PLM server's
administrator hooks and the BREP CAD application's scripts, natively and in
the browser (`wasm32-unknown-unknown`).

It is for a host that wants user-written rules — "what number does this part
get", "may this revision be released" — without giving the script anything
the host did not hand it. The crate depends on Boa and `serde_json` only: no
file system, no network, no clock. Every binding that reaches outside the
interpreter is installed by the host.

## Example

```rust
use brep_script_core::{call, Failure};
use serde_json::json;

let source = r#"
    function onRelease(part) {
        console.log("checking " + part.number);
        if (!part.description) throw new Error("a released part needs a description");
        return { ok: true, revision: part.revision + 1 };
    }
"#;

// No host bindings beyond `console`: pass an `install` that adds nothing.
let no_bindings = |_: &mut _, _: &_| Ok(());
let input = json!({"number": "P-100", "revision": 1, "description": "Bracket"});
match call(source, "hooks.js", "onRelease", &input, &no_bindings) {
    Ok(done) => println!("{} (log: {:?})", done.value, done.logs),
    Err(Failure { message, logs }) => eprintln!("refused: {message} (log: {logs:?})"),
}
```

This prints `{"ok":true,"revision":2} (log: ["checking P-100"])`. Without a
`description` the call returns `Failure` with the message
`a released part needs a description`.

## How it behaves

- **A fresh interpreter per call.** Each `call` builds a new Boa `Context`,
  evaluates the file, calls the function and drops the context: no state leaks
  from one call into the next, and a `Context` (which is not `Send`) never
  crosses a thread.
- **Host bindings.** `install(&mut Context, &Logs)` runs after `console` is
  defined and before the file is evaluated; add your own globals there.
  `brep_script_core::boa_engine` re-exports Boa so your bindings name the same
  engine version.
- **Failures are sentences.** A throw, a parse error, or a function the file
  does not define all return `Failure { message, logs }`; for
  `throw new Error("x")` and `throw "x"` the message is `x`.
- **Bounded, honestly.** Each loop may run at most `LOOP_ITERATION_LIMIT`
  (10,000,000) iterations and the call depth is at most `RECURSION_LIMIT`
  (512), so `while (true) {}` and runaway recursion throw instead of hanging.
  There is **no wall-clock limit**: Boa cannot pre-empt a running script, so
  many individually bounded loops can still run long, and a script blocked in
  a host function is bounded only by that function's own timeout.

## Feature flags

None. On `wasm32` the crate turns on Boa's `js` feature by itself (a
`getrandom` backend and a browser clock), so a host never has to.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
