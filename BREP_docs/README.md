# BREP_docs — the help-site generator for the BREP CAD application

Part of the [next.BREP.io](https://next.brep.io) Rust B-rep kernel: `cargo install BREP_docs` · [licence summary](#licence)

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

With no argument it reads the parent of the directory the crate was compiled
from, which is the source tree when the BREP build script runs it with
`cargo run --release`; an installed binary needs the tree as its argument.

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

In plain words: the crates are free to use as they are, including in a commercial project. If you
fix or change something in the kernel, send the change back as a pull request to
[github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL). If you do not want to share your
changes, buy a commercial licence from [Autodrop3d LLC](https://autodrop3d.com); with it you never
have to share your code back.

The exact terms are in `LICENSE.md`, shipped in the crate (`license-file` in the manifest) and at
[LICENSE.md](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL/blob/master/LICENSE.md); if this summary and the file ever differ, the file wins.
