# BREP_docs — the help-site generator for the BREP CAD application

`BREP_docs` builds the BREP CAD application's in-app help: it renders a BREP
source tree's user documentation from Markdown into a static, searchable HTML
site that the application opens from its toolbar and that any static web
server can host.

It is a build tool for people building the BREP application from source, not
a general-purpose static-site generator: it reads the BREP tree's layout and
the application's own icon artwork.

## Quick start

```sh
cargo install BREP_docs
brep-docs /path/to/brep-source-tree
```

With no argument it reads the source tree the binary was built in, which is
how the BREP build script runs it (`cargo run --release`).

It reads, relative to the tree:

- `docs/` — the user documentation, one page per Markdown file, every other
  file (screenshots and the like) copied through. Its `developer` section is
  never published into the site.
- `BREP_app/assets/glyphs/` — the application's icon SVGs. An image of a glyph
  file, or a catalogued glyph character in the prose, becomes an inline `<svg>`
  that follows the page's light or dark theme.
- `LICENSE.md` and `THIRD-PARTY-NOTICES.md`, and `cargo metadata` on
  `BREP_app/Cargo.toml` (so `cargo` must be on the `PATH` and the lockfile
  current) — the site's closing **Licences** section: the project licence, the
  maintained notices, and an inventory of every crate the application ships,
  grouped by licence, with the notice texts that must accompany embedded fonts.

It writes `BREP_app/web/help/`, wiping it first:

- one HTML page per Markdown file, mirroring the `docs/` tree so every relative
  link works as authored (the only rewrite is `.md` → `.html`), with embedded
  CSS, a sidebar tree, breadcrumbs and a client-side search box;
- `index.html`, a home page with the section contents;
- `search-index.json`, the search data.

## Feature flags

None.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
