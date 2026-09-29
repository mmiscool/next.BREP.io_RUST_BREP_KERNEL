//! The **Licences** section of the help site.
//!
//! Three sources feed it, and none of them lives under `docs/`:
//!
//!   * `LICENSE.md` — the project licence, rendered as the section's first page;
//!   * `THIRD-PARTY-NOTICES.md` — the maintained notices, rendered as authored;
//!   * `cargo metadata` — the crate inventory, GENERATED on every run so the
//!     list of third-party licences is the dependency graph itself and never a
//!     hand-kept table. Every crate the application ships is grouped under its
//!     declared licence expression, and the licences that require their notice
//!     to accompany an embedded file (the fonts) have those notice texts
//!     reproduced on the page, so the web bundle carries them wherever
//!     `web/help/` is served beside it.
//!
//! The inventory is the BREP_app resolve graph walked from the root through
//! every non-dev edge, across every target. That deliberately includes build
//! dependencies and target-specific crates (the Windows API bindings, say): a
//! crate that might ship in one artifact is listed rather than argued away.

use crate::{escape, extract_title, render_markdown, Icons, Page};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The section directory — pages land under `web/help/licences/`.
pub(crate) const SECTION: &str = "licences";

/// The files rendered as authored: (repo-relative source, heading prepended when
/// the file has none). Authored licence pages. Every entry covers material the
/// application actually ships; test fixtures and development-only tooling are
/// not distributed and are deliberately not attributed here.
const AUTHORED: &[(&str, Option<&str>)] = &[
    ("LICENSE.md", Some("# Licence\n\n")),
    ("THIRD-PARTY-NOTICES.md", None),
];

const INVENTORY_PAGE: &str = "third-party-crates.md";

pub(crate) fn pages(repo: &Path, icons: &Icons) -> Vec<Page> {
    let mut pages: Vec<Page> = AUTHORED
        .iter()
        .map(|(src, heading)| authored_page(repo, icons, src, *heading))
        .collect();
    let inventory = Inventory::from_metadata(&cargo_metadata(repo));
    pages.push(inventory.page());
    pages
}

fn authored_page(repo: &Path, icons: &Icons, src: &str, heading: Option<&str>) -> Page {
    let path = repo.join(src);
    let mut source = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    if let Some(h) = heading {
        source.insert_str(0, h);
    }
    let page_dir = path.parent().unwrap().to_path_buf();
    let rel = Path::new(SECTION).join(src);
    let (body, text) = render_markdown(&source, &page_dir, icons);
    let (title, icon) = extract_title(&source, &rel, &page_dir, icons);
    Page { rel, title, icon, body, text }
}

/// `cargo metadata` for the application graph. A failure here is a failure of
/// the help build: a Licences section without its inventory would ship a page
/// that claims to list what it does not.
fn cargo_metadata(repo: &Path) -> Value {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let manifest = repo.join("BREP_app/Cargo.toml");
    let out = Command::new(&cargo)
        .args(["metadata", "--format-version", "1", "--locked", "--manifest-path"])
        .arg(&manifest)
        .output()
        .unwrap_or_else(|e| panic!("run {} metadata: {e}", cargo.to_string_lossy()));
    assert!(
        out.status.success(),
        "cargo metadata for {} failed:\n{}",
        manifest.display(),
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).expect("parse cargo metadata")
}

/// One crate in the shipped graph.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct Crate {
    name: String,
    version: String,
    /// The declared expression, or a label for a crate that ships a licence
    /// file instead of one.
    licence: String,
    /// Author names, without their e-mail addresses.
    authors: Vec<String>,
    /// Where the crate comes from: its repository, else its crates.io page.
    link: String,
    /// The package directory in the registry — where notice files are read.
    dir: PathBuf,
}

struct Inventory {
    /// The application crate the graph was resolved for.
    root: (String, String),
    /// This repository's own crates (path dependencies), covered by LICENSE.md.
    own: Vec<Crate>,
    /// Everything else that ships, by licence expression.
    by_licence: BTreeMap<String, Vec<Crate>>,
}

impl Inventory {
    fn from_metadata(meta: &Value) -> Inventory {
        let shipped = shipped_package_ids(meta);
        let packages: BTreeMap<&str, &Value> = meta["packages"]
            .as_array()
            .expect("packages")
            .iter()
            .map(|p| (p["id"].as_str().expect("package id"), p))
            .collect();
        let root_id = meta["resolve"]["root"].as_str().expect("resolve root");
        let root_pkg = packages[root_id];
        let root = (str_of(root_pkg, "name"), str_of(root_pkg, "version"));

        let mut own = Vec::new();
        let mut by_licence: BTreeMap<String, Vec<Crate>> = BTreeMap::new();
        for id in &shipped {
            if *id == root_id {
                continue;
            }
            let pkg = packages[id.as_str()];
            let krate = crate_of(pkg);
            if pkg["source"].is_null() {
                own.push(krate);
            } else {
                by_licence.entry(krate.licence.clone()).or_default().push(krate);
            }
        }
        own.sort();
        for list in by_licence.values_mut() {
            list.sort();
        }
        Inventory { root, own, by_licence }
    }

    fn third_party_count(&self) -> usize {
        self.by_licence.values().map(Vec::len).sum()
    }

    /// The crates whose licence requires its notice to travel with the
    /// embedded file, with every notice file found in the package.
    fn bundled_notices(&self) -> Vec<(&Crate, Vec<(String, String)>)> {
        self.by_licence
            .values()
            .flatten()
            .filter(|c| bundled_notice_required(&c.licence))
            .map(|c| {
                let files = notice_files(&c.dir);
                assert!(
                    !files.is_empty(),
                    "{} {} is licensed `{}`, which requires its notices to travel with the \
                     embedded files, but no notice file was found under {}",
                    c.name,
                    c.version,
                    c.licence,
                    c.dir.display()
                );
                (c, files)
            })
            .collect()
    }

    /// The page, built as HTML directly: crate descriptions and author strings
    /// carry `*`, `_`, `<` and `[` freely, which the markdown parser would read
    /// as markup.
    fn page(&self) -> Page {
        let mut html = String::new();
        let mut text = String::new();
        let title = "Third-party crate licences";
        html.push_str(&format!("<h1>{title}</h1>"));
        text.push_str(title);
        text.push(' ');

        let intro = format!(
            "Generated from <code>cargo metadata</code> for <code>{} {}</code> at build time: \
             <b>{}</b> third-party crates under <b>{}</b> licence expressions. The graph is every \
             non-development dependency resolved across every target, so build dependencies and \
             target-specific crates (the Windows API bindings, say) are listed rather than argued \
             away. Each crate's full licence text is in its published source package, linked from \
             its name.",
            escape(&self.root.0),
            escape(&self.root.1),
            self.third_party_count(),
            self.by_licence.len()
        );
        html.push_str(&format!("<p>{intro}</p>"));

        html.push_str(
            "<p>This repository's own crates are covered by the <a href=\"LICENSE.html\">BREP \
             licence</a>: ",
        );
        html.push_str(
            &self
                .own
                .iter()
                .map(|c| format!("<code>{} {}</code>", escape(&c.name), escape(&c.version)))
                .collect::<Vec<_>>()
                .join(", "),
        );
        html.push_str(".</p>");

        html.push_str("<h2 id=\"summary\">Summary</h2><table><tr><th>Licence</th><th>Crates</th></tr>");
        let mut summary: Vec<(&String, usize)> =
            self.by_licence.iter().map(|(l, v)| (l, v.len())).collect();
        summary.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(b.0)));
        for (licence, n) in summary {
            html.push_str(&format!(
                "<tr><td><a href=\"#{id}\">{l}</a></td><td>{n}</td></tr>",
                id = anchor(licence),
                l = escape(licence)
            ));
        }
        html.push_str("</table>");

        html.push_str("<h2 id=\"crates\">Crates by licence</h2>");
        for (licence, crates) in &self.by_licence {
            html.push_str(&format!(
                "<h3 id=\"{id}\">{l} <small>({n})</small></h3>\
                 <table><tr><th>Crate</th><th>Version</th><th>Authors</th></tr>",
                id = anchor(licence),
                l = escape(licence),
                n = crates.len()
            ));
            text.push_str(licence);
            text.push(' ');
            for c in crates {
                html.push_str(&format!(
                    "<tr><td><a href=\"{link}\">{name}</a></td><td>{version}</td><td>{authors}</td></tr>",
                    link = escape(&c.link),
                    name = escape(&c.name),
                    version = escape(&c.version),
                    authors = escape(&c.authors.join(", ")),
                ));
                text.push_str(&c.name);
                text.push(' ');
            }
            html.push_str("</table>");
        }

        let notices = self.bundled_notices();
        if !notices.is_empty() {
            html.push_str(
                "<h2 id=\"notices\">Bundled notices</h2>\
                 <p>These licences require their notice to accompany the files they cover, and \
                 those files are compiled into the application. The notices are reproduced here \
                 in full so they travel with the web bundle.</p>",
            );
            for (c, files) in notices {
                html.push_str(&format!(
                    "<h3>{name} {version} <small>{licence}</small></h3>",
                    name = escape(&c.name),
                    version = escape(&c.version),
                    licence = escape(&c.licence)
                ));
                for (file, body) in files {
                    html.push_str(&format!(
                        "<h4>{file}</h4><pre>{body}</pre>",
                        file = escape(&file),
                        body = escape(&body)
                    ));
                }
            }
        }

        Page {
            rel: Path::new(SECTION).join(INVENTORY_PAGE),
            title: title.to_string(),
            icon: None,
            body: html,
            text,
        }
    }
}

/// Every package reachable from the resolve root through a non-dev edge — what
/// can end up in a shipped artifact. A `dev` edge is followed for nothing:
/// a crate used only by tests is not distributed.
fn shipped_package_ids(meta: &Value) -> BTreeSet<String> {
    let resolve = &meta["resolve"];
    let root = resolve["root"].as_str().expect("resolve root");
    let nodes: BTreeMap<&str, &Value> = resolve["nodes"]
        .as_array()
        .expect("resolve nodes")
        .iter()
        .map(|n| (n["id"].as_str().expect("node id"), n))
        .collect();
    let mut seen = BTreeSet::new();
    let mut stack = vec![root];
    while let Some(id) = stack.pop() {
        if !seen.insert(id.to_string()) {
            continue;
        }
        let Some(node) = nodes.get(id) else { continue };
        for dep in node["deps"].as_array().into_iter().flatten() {
            let shipped = dep["dep_kinds"]
                .as_array()
                .into_iter()
                .flatten()
                .any(|k| k["kind"].as_str() != Some("dev"));
            if shipped {
                if let Some(pkg) = dep["pkg"].as_str() {
                    stack.push(pkg);
                }
            }
        }
    }
    seen
}

fn crate_of(pkg: &Value) -> Crate {
    let name = str_of(pkg, "name");
    let licence = match pkg["license"].as_str().map(str::trim).filter(|l| !l.is_empty()) {
        Some(l) => l.to_string(),
        None => match pkg["license_file"].as_str() {
            Some(f) => format!("Licence file: {f}"),
            None => "Unspecified".to_string(),
        },
    };
    let authors = pkg["authors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(|a| a.split('<').next().unwrap_or("").trim().to_string())
        .filter(|a| !a.is_empty())
        .collect();
    let link = pkg["repository"]
        .as_str()
        .or_else(|| pkg["homepage"].as_str())
        .map(str::to_string)
        .unwrap_or_else(|| source_link(pkg, &name));
    let dir = Path::new(pkg["manifest_path"].as_str().expect("manifest_path"))
        .parent()
        .expect("manifest dir")
        .to_path_buf();
    Crate {
        name,
        version: str_of(pkg, "version"),
        licence,
        authors,
        link,
        dir,
    }
}

/// A crate with neither repository nor homepage: its registry page, or the git
/// URL it was fetched from.
fn source_link(pkg: &Value, name: &str) -> String {
    match pkg["source"].as_str() {
        Some(s) if s.starts_with("git+") => s[4..].split('#').next().unwrap_or("").to_string(),
        _ => format!("https://crates.io/crates/{name}"),
    }
}

fn str_of(v: &Value, key: &str) -> String {
    v[key].as_str().unwrap_or("").to_string()
}

/// A licence whose notice must accompany the covered files. The fonts compiled
/// into the application are under the SIL Open Font License and the Ubuntu
/// Font Licence, both of which require exactly that; a permissive code licence
/// is discharged by the inventory entry itself.
fn bundled_notice_required(licence: &str) -> bool {
    let l = licence.to_ascii_lowercase();
    l.contains("ofl") || l.contains("ubuntu-font")
}

/// The notice texts in a package directory: every `.txt` file and every
/// `LICENSE*` / `LICENCE*` / `COPYING*` / `NOTICE*` file, as (relative path,
/// contents), in path order.
fn notice_files(dir: &Path) -> Vec<(String, String)> {
    assert!(
        dir.is_dir(),
        "package directory {} is missing — the notice texts cannot be read",
        dir.display()
    );
    let mut files = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        let Ok(entries) = fs::read_dir(&d) else { continue };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with('.') || name == "target" {
                continue;
            }
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            let lower = name.to_ascii_lowercase();
            let is_notice = lower.ends_with(".txt")
                || ["license", "licence", "copying", "notice"]
                    .iter()
                    .any(|p| lower.starts_with(p));
            if !is_notice {
                continue;
            }
            let rel = path.strip_prefix(dir).unwrap().to_string_lossy().replace('\\', "/");
            let body = String::from_utf8_lossy(&fs::read(&path).expect("read notice")).into_owned();
            files.push((rel, body));
        }
    }
    files.sort();
    files
}

/// An `id` for a licence expression heading, so the summary can link to it.
fn anchor(licence: &str) -> String {
    let mut id = String::from("licence-");
    for ch in licence.chars() {
        if ch.is_ascii_alphanumeric() {
            id.push(ch.to_ascii_lowercase());
        } else if !id.ends_with('-') {
            id.push('-');
        }
    }
    id.trim_end_matches('-').to_string()
}

