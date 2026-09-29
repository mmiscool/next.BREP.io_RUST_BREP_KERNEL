# Third-party notices

This repository's own code is governed by [LICENSE.md](LICENSE.md). The
following third-party material is bundled into shipped artifacts.

## Fonts compiled into the application

`brep-app` bundles **four** font faces, and none of them are ours. This
repository ships no font file at all: the `BrepIcons.ttf` icon face it used to
carry was retired when every icon became inline SVG artwork
(`BREP_app/assets/glyphs/`, compiled into `src/icons.rs` by `build.rs`), so an
icon character is now only a key into that catalog, never something a font
renders. The four faces arrive through `eframe`'s `default_fonts` feature, which
pulls `epaint_default_fonts` — a crate whose entire contents are four
`include_bytes!` constants, so those faces are linked into every native binary
and every wasm module built from `BREP_app/` whether or not the UI ends up
drawing with them.

| Font | File | License | Notice text |
| --- | --- | --- | --- |
| Ubuntu Light | `Ubuntu-Light.ttf` (via `epaint_default_fonts`) | Ubuntu Font Licence 1.0 | `fonts/UFL.txt` in that crate |
| Noto Emoji | `NotoEmoji-Regular.ttf` (via `epaint_default_fonts`) | SIL Open Font License 1.1 | `fonts/OFL.txt` in that crate |
| Hack | `Hack-Regular.ttf` (via `epaint_default_fonts`) | MIT (Source Foundry), over public-domain DejaVu and the Bitstream Vera License | `fonts/Hack-Regular.txt` in that crate |
| emoji-icon-font | `emoji-icon-font.ttf` (via `epaint_default_fonts`) | MIT | `fonts/emoji-icon-font-mit-license.txt` in that crate |

`epaint_default_fonts` declares its terms as
`(MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0`. The `AND` is the part
that matters: the OFL-1.1 and Ubuntu-font-1.0 obligations are **not** discharged
by taking the MIT or Apache-2.0 option on the crate's own code — both licences
require their notice to travel with the font. **A binary release of `brep-app`
(the `.exe`, the native binary, or the wasm bundle) must therefore carry the
four notice files listed above**, copied from the `epaint_default_fonts` source
in the Cargo registry. Publishing `BREP_app` to crates.io as source does not
raise this, because the consumer resolves `epaint_default_fonts` from crates.io
with its notice files intact. The generated help site (`BREP_docs`, run by
`./build.sh app` into `BREP_app/web/help/`) reproduces those four notice texts
on its *Third-party crate licences* page, read from the `epaint_default_fonts`
package at build time, so the wasm bundle served together with `web/help/`
carries them; a native binary shipped on its own still needs the files beside it.

All UI **text** is drawn with the operating system's default monospace font,
located and read at runtime via fontconfig — it is not bundled or redistributed.
On wasm, or on a native machine without fontconfig, the `epaint_default_fonts`
faces above are the fallback that is actually rendered.

## Rust crate dependencies

Verified with `cargo metadata --format-version 1` on 2026-09-09 against the
`BREP_app` graph (the superset — it resolves the kernel, gizmos, render,
reconstruction, RANSAC and MCP-core crates plus every registry dependency of all
of them, across every target and including dev-dependencies).

- **488 packages** in the `BREP_app` graph, 481 of them third-party; **83** in
  the `BREP_kernel` graph, 82 third-party.
- **Zero** packages carry a copyleft-only licence. Every non-repo package is
  usable under a permissive term: 238 `MIT OR Apache-2.0`, 93 `MIT`, 36
  `Apache-2.0 OR MIT`, 18 `Unicode-3.0` (the ICU4X family), 15 `MIT/Apache-2.0`,
  12 `Apache-2.0`, and a long tail of Zlib / ISC / BSD / Unlicense / BSL-1.0 /
  CC0-1.0 / CDLA-Permissive-2.0.
- Three packages offer a copyleft term **as one option among others**, and the
  permissive option is taken in each case: `self_cell` (`Apache-2.0 OR
  GPL-2.0-only`) and `r-efi` ×2 (`MIT OR Apache-2.0 OR LGPL-2.1-or-later`).
- **Four** packages use `AND` in their licence expression — the packages whose
  obligation cannot be satisfied by picking one term. Every one of them is a
  normal (not dev-only) dependency of `BREP_app`:

| Package | Expression | Reached through | Present in |
| --- | --- | --- | --- |
| `epaint_default_fonts` | `(MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0` | `egui` / `eframe` | native + wasm |
| `unicode-ident` | `(MIT OR Apache-2.0) AND Unicode-3.0` | `wgpu` → `naga` | native + wasm |
| `ring` | `Apache-2.0 AND ISC` | `ehttp` → `ureq` → `rustls` | native only |
| `dpi` | `Apache-2.0 AND MIT` | `eframe` → `winit` | native only |

  Only `epaint_default_fonts` conjoins *font* licences, which is why it gets the
  table above; the other three impose ordinary permissive-notice obligations, so
  a binary release must ship each of those crates' own licence texts alongside
  the four font notices. In the `BREP_kernel` graph `unicode-ident` is the sole
  `AND` expression; it arrives through the kernel's unconditional `wasm-bindgen`
  dependency (`wasm-bindgen` → `wasm-bindgen-shared`), so it is linked on every
  target the kernel builds for, not only wasm.
- Every `BREP*` package itself declares no SPDX `license`; they carry
  `license-file = "LICENSE.md"`, which is this repository's own non-standard
  licence. Automated scanners will report them as unknown rather than permissive.

To regenerate the inventory after a dependency change:

```sh
cargo install cargo-license
cd BREP_app && cargo license
```

(Neither `cargo-license`, `cargo-deny` nor `cargo-about` was installed when the
above was verified; `cargo metadata`'s `license` field was read directly
instead. It reports the same declared expressions those tools consume.)

The counts above are a snapshot. The help site's *Third-party crate licences*
page is generated from `cargo metadata` on every `./build.sh app`, so it lists
the crates actually resolved for the build it ships with, grouped by licence
expression.

## Network endpoints reached by the application

`brep-app` contacts two of this project's own public services. Neither carries a
credential and neither is contacted unless the user acts:

- `https://api.step.parts/v1` — the step.parts model library, queried when the
  user opens the Assembly workbench's library panel (`src/panels/step_parts.rs`).
- `https://v2.brep.io/api/report` — the reports endpoint, POSTed to only when the
  user submits the in-app "Submit Bug" form (`src/panels/bug_report.rs`).
