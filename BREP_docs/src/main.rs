//! brep-docs — the static HTML help-site generator for user-facing documentation
//! in the repo's `docs/` tree.
//!
//! Successor to the retired docs build (`generateLicenses`), with
//! the deliberate simplification the port was asked for: the output MIRRORS the
//! docs directory structure (no `__` path flattening), so every relative link
//! and image reference works as-authored and the only rewrite is `.md` → `.html`.
//!
//! What it produces (into `BREP_app/web/help/`, gitignored, wiped each run):
//!   * one HTML page per user-facing markdown file, with embedded CSS, a sidebar doc tree,
//!     breadcrumbs, and an inline client-side search widget;
//!   * every non-markdown asset (the dialog PNGs etc.) copied through verbatim;
//!   * `index.html` — a generated home page with the section table of contents;
//!   * `search-index.json` — `{title, href, summary, content}` per page;
//!   * a `licences/` section that no `docs/` file authors: the project licence
//!     (`LICENSE.md`), the maintained third-party notices, and a crate inventory
//!     generated from `cargo metadata` — see [`licences`].
//!
//! # Icons
//!
//! Two things in a page name the app's artwork, and BOTH end up as an inline
//! `<svg>` element in the HTML — never a file reference:
//!
//!   * a markdown IMAGE of a glyph file
//!     (`![Extrude icon](../../BREP_app/assets/glyphs/icon_E03D.svg)`), which is
//!     how every feature page and the feature index show their icon. Authored
//!     as an image, it renders in any markdown reader as well as here; and when
//!     it leads a page's `# ` heading it is lifted off the title, so the sidebar
//!     and the home page can draw the same icon beside the page name;
//!   * a catalogued glyph CHARACTER in the prose (`✕`, `▤`, `⌒`) — the app has
//!     no icon font any more, so a bare character would be a box on most
//!     machines. See [`iconify`].
//!
//! Inlining is what makes a monochrome icon work in both of the site's themes:
//! its `#111` ink sentinel becomes `currentColor`, exactly as the app multiplies
//! the same artwork by its live text colour. Nothing is copied into the site.
//!
//! The app links to `help/index.html` from its toolbar; any static server that
//! serves the repo can serve the docs.

use pulldown_cmark::{CodeBlockKind, CowStr, Event, Options, Parser, Tag, TagEnd};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

mod licences;

/// Root-level section order for the sidebar and the home-page TOC. Directories
/// not listed sort after these, alphabetically, except the trailing ones.
const SECTION_ORDER: &[&str] = &["features", "panels"];
/// Sections listed after every other named section: the licences close the
/// site, the way an About box does.
const TRAILING_SECTIONS: &[&str] = &[licences::SECTION];
const EXCLUDED_SECTIONS: &[&str] = &["developer"];
/// The app's icon SVGs — the SAME files `BREP_app/build.rs` compiles into the
/// app's own inline-icon catalog, read here as the source of every icon the site
/// draws. There is no icon font any more, and nothing is copied: see [`Icons`].
const GLYPH_SOURCE: &str = "BREP_app/assets/glyphs";

struct Page {
    /// Path relative to `docs/`, e.g. `features/extrude.md`.
    rel: PathBuf,
    title: String,
    /// The artwork the page's heading leads with, as the `<svg>` to inline —
    /// `None` for a page whose heading is text alone. Lifted off the title so
    /// the sidebar and home page draw the icon beside the name.
    icon: Option<String>,
    /// Rendered HTML body.
    body: String,
    /// Plain text (for the search index).
    text: String,
}

fn main() {
    let repo = repo_root();
    let docs = repo.join("docs");
    let out = repo.join("BREP_app/web/help");
    if !docs.is_dir() {
        eprintln!("brep-docs: no docs/ tree at {}", docs.display());
        std::process::exit(1);
    }
    // Refuse to wipe anything that isn't the expected output location.
    assert!(out.ends_with("BREP_app/web/help"), "unexpected out dir");
    if out.exists() {
        fs::remove_dir_all(&out).expect("wipe help output");
    }
    fs::create_dir_all(&out).expect("create help output");

    let icons = Icons::load(&repo);

    let mut pages: Vec<Page> = Vec::new();
    let mut assets: Vec<PathBuf> = Vec::new();
    walk(&docs, &docs, &icons, &mut pages, &mut assets);
    // The Licences section: sourced from the repo root and `cargo metadata`,
    // not from `docs/`, so it is appended after the walk.
    pages.extend(licences::pages(&repo, &icons));

    // Assets copy through at their authored relative paths.
    for rel in &assets {
        let dst = out.join(rel);
        fs::create_dir_all(dst.parent().unwrap()).unwrap();
        fs::copy(docs.join(rel), &dst).unwrap();
    }

    let tree = SidebarTree::build(&pages);
    for page in &pages {
        let html = render_page(page, &tree, &icons);
        let dst = out.join(page.rel.with_extension("html"));
        fs::create_dir_all(dst.parent().unwrap()).unwrap();
        fs::write(dst, html).unwrap();
    }

    fs::write(out.join("index.html"), render_home(&pages, &tree, &icons)).unwrap();
    fs::write(out.join("search-index.json"), search_index(&pages)).unwrap();

    println!(
        "brep-docs: {} pages, {} assets, {} icons -> {}",
        pages.len(),
        assets.len() + 1,
        icons.len(),
        out.display()
    );
}

/// The BREP source tree to read: the one argument when given (an installed
/// binary has no tree of its own), else the checkout this crate was built in.
fn repo_root() -> PathBuf {
    match std::env::args_os().nth(1) {
        Some(root) => PathBuf::from(root),
        None => PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .to_path_buf(),
    }
}

fn walk(root: &Path, dir: &Path, icons: &Icons, pages: &mut Vec<Page>, assets: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).collect();
    entries.sort();
    for path in entries {
        if is_excluded_docs_path(root, &path) {
            continue;
        }
        if path.is_dir() {
            walk(root, &path, icons, pages, assets);
        } else if path.extension().is_some_and(|e| e == "md") {
            let rel = path.strip_prefix(root).unwrap().to_path_buf();
            let page_dir = path.parent().unwrap().to_path_buf();
            let source = fs::read_to_string(&path).unwrap();
            let (body, text) = render_markdown(&source, &page_dir, icons);
            let (title, icon) = extract_title(&source, &rel, &page_dir, icons);
            pages.push(Page {
                rel,
                title,
                icon,
                body,
                text,
            });
        } else {
            assets.push(path.strip_prefix(root).unwrap().to_path_buf());
        }
    }
}

fn is_excluded_docs_path(root: &Path, path: &Path) -> bool {
    path.strip_prefix(root)
        .ok()
        .and_then(|rel| rel.iter().next())
        .is_some_and(|section| {
            EXCLUDED_SECTIONS
                .iter()
                .any(|excluded| section == *excluded)
        })
}

/// A page's `# ` heading, split into the plain title and the icon it leads with.
///
/// The icon is authored as a markdown IMAGE of the app's own glyph file, so the
/// heading shows it in any markdown reader. Lifting it off here is what keeps
/// the `<title>`, the breadcrumbs and the search index the NAME alone, while the
/// sidebar and home page still get the artwork to draw beside it.
fn extract_title(
    source: &str,
    rel: &Path,
    page_dir: &Path,
    icons: &Icons,
) -> (String, Option<String>) {
    let fallback = || {
        let stem = rel.file_stem().unwrap().to_string_lossy();
        let mut pretty = stem.replace(['-', '_'], " ");
        if let Some(first) = pretty.get_mut(0..1) {
            first.make_ascii_uppercase();
        }
        pretty
    };
    let Some(heading) = source.lines().find_map(|l| l.strip_prefix("# ").map(str::trim)) else {
        return (fallback(), None);
    };
    let (dest, rest) = split_leading_image(heading);
    let icon = dest.and_then(|d| icons.from_link(page_dir, d).map(str::to_string));
    let title = rest.trim();
    let title = if title.is_empty() { fallback() } else { title.to_string() };
    (title, icon)
}

/// A leading `![alt](dest)` split off a heading: `(Some(dest), rest)`, else
/// `(None, all of it)`. The alt text is discarded — the artwork replaces it.
fn split_leading_image(heading: &str) -> (Option<&str>, &str) {
    let Some(after_alt) = heading.strip_prefix("![") else {
        return (None, heading);
    };
    let Some(open) = after_alt.find("](") else {
        return (None, heading);
    };
    let dest_start = open + 2;
    let Some(close) = after_alt[dest_start..].find(')') else {
        return (None, heading);
    };
    (
        Some(&after_alt[dest_start..dest_start + close]),
        &after_alt[dest_start + close + 1..],
    )
}

/// Render markdown to (html, plain_text).
///
/// Two rewrites: a relative `*.md` link destination (with optional `#fragment`)
/// becomes `*.html`, and an image of a glyph file becomes that icon's artwork,
/// inlined (see [`Icons`]). `page_dir` is the markdown file's own directory —
/// what an authored relative image destination is resolved against.
fn render_markdown(source: &str, page_dir: &Path, icons: &Icons) -> (String, String) {
    let mut opts = Options::empty();
    opts.insert(Options::ENABLE_TABLES);
    opts.insert(Options::ENABLE_STRIKETHROUGH);
    opts.insert(Options::ENABLE_FOOTNOTES);
    opts.insert(Options::ENABLE_TASKLISTS);

    let mut text = String::new();
    let mut in_code = false;
    // Inside an image the artwork has REPLACED: its alt text is neither markup
    // nor something the search index should match on, so it is dropped.
    let mut in_replaced_image = false;
    let mut events: Vec<Event> = Vec::new();
    for ev in Parser::new_ext(source, opts) {
        match ev {
            Event::Start(Tag::Link {
                link_type,
                dest_url,
                title,
                id,
            }) => events.push(Event::Start(Tag::Link {
                link_type,
                dest_url: rewrite_md_link(dest_url),
                title,
                id,
            })),
            Event::Start(Tag::Image {
                link_type,
                dest_url,
                title,
                id,
            }) => match icons.from_link(page_dir, &dest_url) {
                Some(svg) => {
                    in_replaced_image = true;
                    events.push(Event::Html(CowStr::from(svg.to_string())));
                }
                None => events.push(Event::Start(Tag::Image {
                    link_type,
                    dest_url,
                    title,
                    id,
                })),
            },
            Event::End(TagEnd::Image) if in_replaced_image => in_replaced_image = false,
            _ if in_replaced_image => {}
            Event::Start(Tag::CodeBlock(kind)) => {
                in_code = matches!(kind, CodeBlockKind::Fenced(_));
                events.push(Event::Start(Tag::CodeBlock(kind)));
            }
            Event::End(TagEnd::CodeBlock) => {
                in_code = false;
                events.push(Event::End(TagEnd::CodeBlock));
            }
            Event::Text(t) => {
                if !in_code {
                    text.push_str(&t);
                    text.push(' ');
                }
                events.push(Event::Text(t));
            }
            Event::Code(t) => {
                text.push_str(&t);
                text.push(' ');
                events.push(Event::Code(t));
            }
            Event::SoftBreak | Event::HardBreak => {
                text.push(' ');
                events.push(ev);
            }
            other => events.push(other),
        }
    }

    let mut html = String::new();
    pulldown_cmark::html::push_html(&mut html, events.into_iter());
    (html, text)
}

fn rewrite_md_link(dest: CowStr) -> CowStr {
    if dest.starts_with("http://") || dest.starts_with("https://") || dest.starts_with("mailto:") {
        return dest;
    }
    let (path, frag) = match dest.split_once('#') {
        Some((p, f)) => (p, Some(f)),
        None => (dest.as_ref(), None),
    };
    if !path.ends_with(".md") {
        return dest;
    }
    let mut rewritten = format!("{}.html", &path[..path.len() - 3]);
    if let Some(f) = frag {
        rewritten.push('#');
        rewritten.push_str(f);
    }
    CowStr::from(rewritten)
}

// ---------------------------------------------------------------------------
// Sidebar tree

/// One page in the sidebar / home-page listing: where it is, what it is called,
/// and the artwork to draw beside the name.
struct Entry {
    href: String,
    title: String,
    icon: Option<String>,
}

#[derive(Default)]
struct SidebarTree {
    /// section name ("" = docs root) -> pages
    sections: BTreeMap<String, Vec<Entry>>,
}

impl SidebarTree {
    fn build(pages: &[Page]) -> Self {
        let mut tree = SidebarTree::default();
        for page in pages {
            let href = page.rel.with_extension("html");
            let href = href.to_string_lossy().replace('\\', "/");
            let section = match page.rel.iter().next() {
                Some(first) if page.rel.iter().count() > 1 => {
                    first.to_string_lossy().into_owned()
                }
                _ => String::new(),
            };
            tree.sections.entry(section).or_default().push(Entry {
                href,
                title: page.title.clone(),
                icon: page.icon.clone(),
            });
        }
        // `index.*` first within each section, then alphabetical by href.
        for list in tree.sections.values_mut() {
            list.sort_by(|a, b| {
                let ai = a.href.ends_with("index.html");
                let bi = b.href.ends_with("index.html");
                bi.cmp(&ai).then_with(|| a.href.cmp(&b.href))
            });
        }
        tree
    }

    fn ordered_sections(&self) -> Vec<&String> {
        let mut names: Vec<&String> = self.sections.keys().collect();
        names.sort_by_key(|n| {
            SECTION_ORDER
                .iter()
                .position(|s| *s == n.as_str())
                .map(|i| (0, i, n.to_string()))
                .unwrap_or_else(|| {
                    if n.is_empty() {
                        (3, 0, String::new()) // loose root files last
                    } else if TRAILING_SECTIONS.contains(&n.as_str()) {
                        (2, 0, n.to_string())
                    } else {
                        (1, 0, n.to_string())
                    }
                })
        });
        names
    }

    fn html(&self, root_rel: &str, current: &str) -> String {
        let mut out = String::new();
        for name in self.ordered_sections() {
            let heading = if name.is_empty() {
                "reference".to_string()
            } else {
                name.clone()
            };
            out.push_str(&format!("<h3>{}</h3><ul>", escape(&heading)));
            for entry in &self.sections[name] {
                let class = if entry.href == current { " class=\"current\"" } else { "" };
                out.push_str(&format!(
                    "<li{class}><a href=\"{root_rel}{href}\">{icon}{title}</a></li>",
                    href = entry.href,
                    icon = entry.icon.as_deref().unwrap_or(""),
                    title = escape(&entry.title),
                ));
            }
            out.push_str("</ul>");
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Icons

/// The app's icon artwork, read from `BREP_app/assets/glyphs/`.
///
/// These are the very files `BREP_app/build.rs` compiles into the app's own
/// inline-icon catalog, so an icon in the docs IS the icon in the app — there is
/// no second copy to drift. Nothing is written into the site either: every icon
/// reaches a page as an inline `<svg>` element (see [`inline_svg`]), which is
/// what lets monochrome artwork follow the page's text colour in both themes.
struct Icons {
    /// The glyph directory, canonicalised — an image destination that resolves
    /// into it is an icon rather than an ordinary asset.
    dir: PathBuf,
    /// Codepoint → the `<svg>` element to inline for it.
    art: BTreeMap<char, String>,
}

impl Icons {
    fn load(repo: &Path) -> Icons {
        let src = repo.join(GLYPH_SOURCE);
        let dir = fs::canonicalize(&src).unwrap_or_else(|e| panic!("read {}: {e}", src.display()));
        let mut art = BTreeMap::new();
        for entry in fs::read_dir(&dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display())) {
            let path = entry.expect("glyph dir entry").path();
            let Some(ch) = glyph_codepoint(&path) else { continue };
            let svg = fs::read_to_string(&path).expect("read glyph svg");
            art.insert(ch, inline_svg(&svg, &path));
        }
        assert!(!art.is_empty(), "no glyph SVGs found in {}", dir.display());
        Icons { dir, art }
    }

    fn len(&self) -> usize {
        self.art.len()
    }

    /// The artwork for a catalogued character, or `None` if the character is
    /// ordinary text.
    fn inline(&self, ch: char) -> Option<&str> {
        self.art.get(&ch).map(String::as_str)
    }

    /// The artwork a markdown image destination names, resolved relative to the
    /// page it was authored in. `None` for an ordinary image — a dialog PNG,
    /// say — which copies through and renders as an `<img>` as before.
    ///
    /// A destination that NAMES a glyph file but does not resolve to one is a
    /// hard error rather than a silent `<img>`: a mistyped codepoint would
    /// otherwise ship as a broken image on a page nobody re-reads.
    fn from_link(&self, page_dir: &Path, dest: &str) -> Option<&str> {
        if dest.starts_with("http://") || dest.starts_with("https://") {
            return None;
        }
        let named_a_glyph = Path::new(dest)
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("icon_") && n.ends_with(".svg"));
        let art = fs::canonicalize(page_dir.join(dest))
            .ok()
            .filter(|p| p.parent() == Some(self.dir.as_path()))
            .as_deref()
            .and_then(glyph_codepoint)
            .and_then(|ch| self.inline(ch));
        assert!(
            art.is_some() || !named_a_glyph,
            "{}: `{dest}` names a glyph file that is not in {}",
            page_dir.display(),
            self.dir.display(),
        );
        art
    }
}

/// The character a glyph file draws, from its `icon_<hex>.svg` name.
fn glyph_codepoint(path: &Path) -> Option<char> {
    let name = path.file_name()?.to_str()?;
    let hex = name.strip_prefix("icon_")?.strip_suffix(".svg")?;
    u32::from_str_radix(hex, 16).ok().and_then(char::from_u32)
}

/// One glyph file as the `<svg>` element to inline.
///
/// The root tag is rewritten wholesale rather than patched: the file's own
/// `width`/`height` would fight the stylesheet that sizes the icon to the text
/// around it, and its `brep:` bookkeeping attributes mean nothing in a page. The
/// artwork itself is untouched — this parses no path geometry, exactly as
/// `BREP_app/build.rs` does not.
///
/// The one edit is the ink sentinel. Artwork whose every fill is `#111` is
/// MONOCHROME (the same predicate `build.rs` decides `mono` with) and is meant
/// to take the colour of whatever draws it: the app multiplies white artwork by
/// its live text colour, and `currentColor` is the CSS spelling of the same
/// thing. Without it a `#111` icon is all but invisible against this site's dark
/// theme. Artwork that carries its own colours is never touched.
fn inline_svg(svg: &str, path: &Path) -> String {
    let file = path.display();
    let open = svg.find("<svg").unwrap_or_else(|| panic!("{file}: no <svg> element"));
    let open_end = open
        + svg[open..]
            .find('>')
            .unwrap_or_else(|| panic!("{file}: unterminated <svg> tag"));
    let close = svg
        .rfind("</svg>")
        .unwrap_or_else(|| panic!("{file}: no </svg>"));
    let view_box = attr(&svg[open..=open_end], "viewBox")
        .unwrap_or_else(|| panic!("{file}: no viewBox"));
    let inner = &svg[open_end + 1..close];
    let mono = fills(inner).all(|f| f.eq_ignore_ascii_case("#111"));
    let inner = if mono {
        inner.replace("\"#111\"", "\"currentColor\"")
    } else {
        inner.to_string()
    };
    format!(
        "<svg class=\"brep-icon\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{view_box}\" \
         aria-hidden=\"true\" focusable=\"false\">{inner}</svg>"
    )
}

/// The value of an XML attribute, matched literally as `name="value"` — enough
/// for these machine-written files, and it keeps this crate dependency-free.
fn attr<'a>(tag: &'a str, name: &str) -> Option<&'a str> {
    let at = tag.find(&format!("{name}=\""))? + name.len() + 2;
    let end = tag[at..].find('"')?;
    Some(&tag[at..at + end])
}

/// Every `fill="…"` value in an SVG — the same three lines `BREP_app/build.rs`
/// decides monochrome with.
fn fills(svg: &str) -> impl Iterator<Item = &str> {
    svg.match_indices("fill=\"").filter_map(|(i, m)| {
        let rest = &svg[i + m.len()..];
        rest.find('"').map(|end| &rest[..end])
    })
}

/// Replace every catalogued glyph character in `html` with its artwork, inlined.
/// This is the prose route — `✕`, `▤`, `⌒` written straight into a sentence.
/// The app ships no icon font, so the character alone would be a box.
///
/// Only TEXT is rewritten: the scan skips anything between `<` and `>`, so a
/// glyph inside an attribute (a heading anchor `id`, a link `href`) is left
/// exactly as it was and in-page links keep working. Already-inlined artwork is
/// skipped for the same reason — an `<svg>` here is nothing but tags.
fn iconify(html: &str, icons: &Icons) -> String {
    let mut out = String::with_capacity(html.len());
    let mut in_tag = false;
    for ch in html.chars() {
        match ch {
            '<' => {
                in_tag = true;
                out.push(ch);
            }
            '>' => {
                in_tag = false;
                out.push(ch);
            }
            _ => match icons.inline(ch).filter(|_| !in_tag) {
                Some(svg) => out.push_str(svg),
                None => out.push(ch),
            },
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Page assembly

const CSS: &str = r#"
:root { --bg:#ffffff; --fg:#1a1d21; --muted:#5c6570; --line:#e2e5e9; --accent:#0b6bcb; --side:#f6f7f9; --code:#f2f3f5; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#14161a; --fg:#e6e8eb; --muted:#9aa3ad; --line:#2a2e34; --accent:#5aa2e8; --side:#1a1d22; --code:#1f2227; }
}
* { box-sizing: border-box; }
body { margin:0; font:15px/1.6 ui-monospace,"Cascadia Mono","Segoe UI Mono",monospace; color:var(--fg); background:var(--bg); }
/* Icons are inline SVG, sized in `em` so one rule follows the text everywhere it
   appears — 13.5px sidebar row, 15px prose, 2em page heading. The artwork sits
   in the middle of a tall em box (these files were once font glyphs), so the box
   is drawn larger than the text and dropped below the baseline to put the ink
   itself on the line. Monochrome artwork is `currentColor` and takes the colour
   of the text it sits in; colour artwork paints itself. */
.brep-icon { height:2em; width:auto; vertical-align:-0.68em; }
nav.side .brep-icon, .home-toc .brep-icon { margin-right:.15em; }
a { color:var(--accent); text-decoration:none; } a:hover { text-decoration:underline; }
.layout { display:flex; min-height:100vh; }
nav.side { width:250px; flex:none; background:var(--side); border-right:1px solid var(--line); padding:16px 14px; overflow-y:auto; position:sticky; top:0; height:100vh; }
nav.side h3 { font-size:11px; text-transform:uppercase; letter-spacing:.08em; color:var(--muted); margin:16px 0 4px; }
nav.side ul { list-style:none; margin:0; padding:0; }
nav.side li a { display:block; padding:2px 6px; border-radius:4px; color:var(--fg); font-size:13.5px; }
nav.side li a:hover { background:var(--line); text-decoration:none; }
nav.side li.current a { background:var(--accent); color:#fff; }
main { flex:1; min-width:0; padding:28px 40px 80px; max-width:900px; }
.crumbs { font-size:12.5px; color:var(--muted); margin-bottom:18px; }
.crumbs a { color:var(--muted); }
main img { max-width:100%; height:auto; border:1px solid var(--line); border-radius:6px; }
main pre { background:var(--code); padding:12px 14px; border-radius:6px; overflow-x:auto; font-size:13px; }
main code { background:var(--code); padding:1px 4px; border-radius:3px; font-size:.92em; }
main pre code { background:none; padding:0; }
main table { border-collapse:collapse; display:block; overflow-x:auto; }
main th, main td { border:1px solid var(--line); padding:5px 10px; text-align:left; }
main th { background:var(--side); }
main h1,main h2,main h3 { line-height:1.25; }
main h1 { margin-top:0; }
main blockquote { border-left:3px solid var(--line); margin:0; padding:0 0 0 14px; color:var(--muted); }
.search { position:relative; margin-bottom:10px; }
.search input { width:100%; padding:6px 8px; border:1px solid var(--line); border-radius:6px; background:var(--bg); color:var(--fg); font-size:13px; }
.search .results { position:absolute; z-index:10; left:0; right:0; background:var(--bg); border:1px solid var(--line); border-radius:6px; max-height:340px; overflow-y:auto; display:none; }
.search .results a { display:block; padding:6px 8px; border-bottom:1px solid var(--line); font-size:13px; color:var(--fg); }
.search .results a b { color:var(--accent); display:block; }
.home-toc h2 { border-bottom:1px solid var(--line); padding-bottom:4px; }
.home-toc ul { columns:2; gap:32px; list-style:none; padding:0; }
@media (max-width:900px) { .layout{flex-direction:column} nav.side{width:auto;height:auto;position:static} .home-toc ul{columns:1} }
"#;

const SEARCH_JS: &str = r#"
(function(){
  var input=document.getElementById('doc-search'), box=document.getElementById('doc-search-results');
  var root=input.dataset.root, index=null;
  input.addEventListener('input', function(){
    var q=input.value.trim().toLowerCase();
    if(q.length<2){ box.style.display='none'; return; }
    var run=function(){
      var hits=index.map(function(p){
        var t=p.title.toLowerCase().indexOf(q)>=0?2:0;
        var c=p.content.toLowerCase().indexOf(q)>=0?1:0;
        return [t+c,p];
      }).filter(function(h){return h[0]>0;});
      hits.sort(function(a,b){return b[0]-a[0];});
      box.innerHTML=hits.slice(0,12).map(function(h){
        var p=h[1];
        return '<a href="'+root+p.href+'"><b>'+p.title+'</b>'+p.summary+'</a>';
      }).join('')||'<a>no matches</a>';
      box.style.display='block';
    };
    if(index){run();}
    else fetch(root+'search-index.json').then(function(r){return r.json();}).then(function(d){index=d;run();});
  });
  document.addEventListener('click',function(e){ if(!e.target.closest('.search')) box.style.display='none'; });
})();
"#;

fn shell(title: &str, root_rel: &str, crumbs: &str, sidebar: &str, body: &str) -> String {
    format!(
        r#"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} — BREP Docs</title>
<style>{CSS}</style>
</head>
<body>
<div class="layout">
<nav class="side">
  <div class="search">
    <input id="doc-search" data-root="{root_rel}" type="search" placeholder="Search docs…" autocomplete="off">
    <div class="results" id="doc-search-results"></div>
  </div>
  <a href="{root_rel}index.html"><strong>BREP Docs</strong></a>
  {sidebar}
</nav>
<main>
<div class="crumbs">{crumbs}</div>
{body}
</main>
</div>
<script>{SEARCH_JS}</script>
</body>
</html>
"#,
        title = escape(title),
    )
}

fn render_page(page: &Page, tree: &SidebarTree, icons: &Icons) -> String {
    let depth = page.rel.iter().count().saturating_sub(1);
    let root_rel = "../".repeat(depth);
    let href = page.rel.with_extension("html");
    let href = href.to_string_lossy().replace('\\', "/");

    let mut crumbs = format!("<a href=\"{root_rel}index.html\">docs</a>");
    let mut acc = PathBuf::new();
    for part in page.rel.iter() {
        acc.push(part);
        let name = part.to_string_lossy();
        if acc == page.rel {
            crumbs.push_str(&format!(" / {}", escape(&page.title)));
        } else {
            // Directory crumb links to the section's index page when one exists.
            crumbs.push_str(&format!(" / {}", escape(&name)));
        }
    }
    // Glyph characters become the app's own icon artwork — done on the WHOLE
    // page so body, crumbs and sidebar are all covered by one pass. (The icons
    // authored as markdown IMAGES are already inline by now; this is the prose
    // route.)
    let html = shell(&page.title, &root_rel, &crumbs, &tree.html(&root_rel, &href), &page.body);
    iconify(&html, icons)
}

fn render_home(pages: &[Page], tree: &SidebarTree, icons: &Icons) -> String {
    let mut toc = String::from(
        "<h1>BREP Documentation</h1>\
         <p>User documentation for the BREP CAD application.</p>\
         <div class=\"home-toc\">",
    );
    for name in tree.ordered_sections() {
        let heading = if name.is_empty() { "reference" } else { name };
        toc.push_str(&format!("<h2>{}</h2><ul>", escape(heading)));
        for entry in &tree.sections[name.as_str()] {
            toc.push_str(&format!(
                "<li><a href=\"{href}\">{icon}{title}</a></li>",
                href = entry.href,
                icon = entry.icon.as_deref().unwrap_or(""),
                title = escape(&entry.title),
            ));
        }
        toc.push_str("</ul>");
    }
    toc.push_str("</div>");
    let _ = pages;
    iconify(&shell("Home", "", "docs", &tree.html("", ""), &toc), icons)
}

fn search_index(pages: &[Page]) -> String {
    let entries: Vec<serde_json::Value> = pages
        .iter()
        .map(|p| {
            // Trimmed: a heading's icon leaves the space it was written with,
            // and a summary must not open on it.
            let text = p.text.trim();
            serde_json::json!({
                "title": p.title,
                "href": p.rel.with_extension("html").to_string_lossy().replace('\\', "/"),
                "summary": text.chars().take(160).collect::<String>(),
                "content": text.chars().take(4000).collect::<String>(),
            })
        })
        .collect();
    serde_json::to_string(&entries).unwrap()
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

