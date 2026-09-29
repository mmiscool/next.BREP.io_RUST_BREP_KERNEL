//! `brep-plm` — serve the PLM.
//!
//! ```text
//! brep-plm serve [--data <dir>] [--bind <addr>] [--scripts <dir>]
//!                [--secure-cookies off|on|auto] [--trust-proxy]
//!                [--tls-cert <pem> --tls-key <pem>] [--lock-script-editor]
//!                [--max-attachment-mb <n>]
//!                [--backup-dir <dir> [--backup-every <minutes>] [--backup-keep <n>]]
//!                [--cad-app <dir>] [--cad-connect-src <origin>]...
//! brep-plm migrate [--data <dir>] [--dry-run]
//! brep-plm backup  [--data <dir>] [--scripts <dir>] --out <file.tar.gz>
//! brep-plm restore --from <file.tar.gz> --data <new dir>
//! brep-plm export  [--data <dir>] --out <file> [--format json|parts-csv|structure-csv]
//! ```
//!
//! The hardening flags are described in `brep_plm::security` and GUIDE.md.
//!
//! The data directory holds `plm.sqlite` (the metadata and the audit log) and
//! a `docs/` tree; both are created on first run. The first run also seeds one
//! administrator and prints its generated password ONCE. No build of this
//! server ships a known password, which is why it cannot be recovered — make
//! another admin before you lose it.
//!
//! A data directory from before SQLite (`plm.json`, `audit.jsonl`) is imported
//! on the first `serve`. `migrate` does the same import without serving, and
//! `migrate --dry-run` reports what it would import without changing the
//! directory.
//!
//! `--scripts` names the administrator's scripts directory (default
//! `<data>/scripts`), typically a git checkout. See `brep_plm::scripting`.
//!
//! `--cad-app` names the directory the wasm CAD app is served from (default
//! `<data>/cad-app`), at `/cad/app/web/index.html`. See `brep_plm::cad`.

use std::sync::Arc;

use brep_plm::api;
use brep_plm::db::Db;
use brep_plm::security::{CookieSecurity, ServerConfig};

const DEFAULT_BIND: &str = "127.0.0.1:8088";
const DEFAULT_DATA: &str = "./plm-data";

fn usage() -> ! {
    use brep_plm::version::MIN_CLIENT_VERSION as MIN_CLIENT;
    eprintln!(
        "brep-plm — the BREP PLM server\n\
         \n\
         USAGE:\n    \
             brep-plm serve [OPTIONS]\n    \
             brep-plm migrate [--data <dir>] [--dry-run]\n    \
             brep-plm backup  [--data <dir>] [--scripts <dir>] --out <file.tar.gz>\n    \
             brep-plm restore --from <file.tar.gz> --data <new dir>\n    \
             brep-plm export  [--data <dir>] --out <file> [--format json|parts-csv|structure-csv]\n\
         \n\
         OPTIONS:\n    \
             --data <dir>             where plm.sqlite and docs/ live [default: {DEFAULT_DATA}]\n    \
             --bind <addr>            the listen address              [default: {DEFAULT_BIND}]\n    \
             --scripts <dir>          the admin's hook scripts        [default: <data>/scripts]\n    \
             --secure-cookies <mode>  off | on | auto                 [default: off; on with --tls-cert]\n    \
             --trust-proxy            believe X-Forwarded-For / -Proto from a reverse proxy\n    \
             --tls-cert <pem>         serve HTTPS with this certificate chain\n    \
             --tls-key <pem>          ... and this private key\n    \
             --lock-script-editor     turn the in-browser script editor off, whatever Settings say\n    \
             --max-attachment-mb <n>  the largest attachment upload       [default: 100]\n    \
             --max-document-mb <n>    the largest document or recovery write [default: 64]\n    \
             --min-client-version <v> refuse CAD apps older than v (raise only) [default: {MIN_CLIENT}]\n    \
             --backup-dir <dir>       where \"Back up now\" and scheduled backups are saved\n    \
             --backup-every <min>     take a backup every <min> minutes    [needs --backup-dir]\n    \
             --backup-keep <n>        how many saved backups to keep       [default: 7]\n    \
             --cad-app <dir>          the hosted CAD app's web/ and pkg/   [default: <data>/cad-app]\n    \
             --cad-connect-src <url>  an origin the hosted app may call (repeatable)\n"
    );
    std::process::exit(2)
}

#[tokio::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "serve".into());
    if command == "-h" || command == "--help" {
        usage();
    }
    if command == "migrate" {
        migrate(args);
    }
    if command == "backup" {
        backup(args);
    }
    if command == "restore" {
        restore(args);
    }
    if command == "export" {
        export(args);
    }
    if command != "serve" {
        eprintln!("brep-plm: unknown command '{command}'");
        usage();
    }

    let mut bind = DEFAULT_BIND.to_string();
    let mut data = DEFAULT_DATA.to_string();
    let mut scripts: Option<std::path::PathBuf> = None;
    let mut config = ServerConfig::default();
    let mut secure_cookies: Option<CookieSecurity> = None;
    let mut tls_cert: Option<std::path::PathBuf> = None;
    let mut tls_key: Option<std::path::PathBuf> = None;
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--bind" => bind = args.next().unwrap_or_else(|| usage()),
            "--data" => data = args.next().unwrap_or_else(|| usage()),
            "--scripts" => scripts = Some(args.next().unwrap_or_else(|| usage()).into()),
            "--secure-cookies" => {
                let mode = args.next().unwrap_or_else(|| usage());
                secure_cookies = Some(mode.parse().unwrap_or_else(|e| {
                    eprintln!("brep-plm: --secure-cookies: {e}");
                    usage()
                }));
            }
            "--trust-proxy" => config.trust_proxy = true,
            "--tls-cert" => tls_cert = Some(args.next().unwrap_or_else(|| usage()).into()),
            "--tls-key" => tls_key = Some(args.next().unwrap_or_else(|| usage()).into()),
            "--lock-script-editor" => config.lock_script_editor = true,
            "--backup-dir" => config.backup_dir = Some(args.next().unwrap_or_else(|| usage()).into()),
            "--backup-every" => {
                let text = args.next().unwrap_or_else(|| usage());
                config.backup_every_minutes = text.parse().ok().filter(|n| *n > 0).unwrap_or_else(|| {
                    eprintln!("brep-plm: --backup-every takes a whole number of minutes above 0, not '{text}'");
                    usage()
                });
            }
            "--backup-keep" => {
                let text = args.next().unwrap_or_else(|| usage());
                config.backup_keep = text.parse().ok().filter(|n| *n > 0).unwrap_or_else(|| {
                    eprintln!("brep-plm: --backup-keep takes a whole number above 0, not '{text}'");
                    usage()
                });
            }
            "--cad-app" => config.cad_app_dir = Some(args.next().unwrap_or_else(|| usage()).into()),
            "--cad-connect-src" => {
                let origin = args.next().unwrap_or_else(|| usage());
                if let Err(error) = brep_plm::cad::check_connect_origin(&origin) {
                    eprintln!("brep-plm: --cad-connect-src: {error}");
                    usage();
                }
                config.cad_connect_src.push(origin);
            }
            "--max-attachment-mb" => {
                let text = args.next().unwrap_or_else(|| usage());
                let mb: u64 = text.parse().ok().filter(|n| *n > 0).unwrap_or_else(|| {
                    eprintln!("brep-plm: --max-attachment-mb takes a whole number of megabytes above 0, not '{text}'");
                    usage()
                });
                config.max_attachment_bytes = mb.saturating_mul(1024 * 1024);
            }
            "--max-document-mb" => {
                let text = args.next().unwrap_or_else(|| usage());
                let mb: u64 = text.parse().ok().filter(|n| *n > 0).unwrap_or_else(|| {
                    eprintln!("brep-plm: --max-document-mb takes a whole number of megabytes above 0, not '{text}'");
                    usage()
                });
                config.max_document_bytes = mb.saturating_mul(1024 * 1024);
            }
            "--min-client-version" => {
                let text = args.next().unwrap_or_else(|| usage());
                if let Err(error) = brep_plm::version::check_floor(&text) {
                    eprintln!("brep-plm: --min-client-version: {error}");
                    std::process::exit(2);
                }
                config.min_client_version = Some(text);
            }
            other => {
                eprintln!("brep-plm: unknown option '{other}'");
                usage();
            }
        }
    }
    if config.backup_every_minutes > 0 && config.backup_dir.is_none() {
        eprintln!("brep-plm: --backup-every needs --backup-dir");
        usage();
    }
    if let Some(dir) = &config.backup_dir {
        if let Err(error) = std::fs::create_dir_all(dir) {
            eprintln!("brep-plm: --backup-dir {}: {error}", dir.display());
            std::process::exit(1);
        }
    }
    let tls = match (&tls_cert, &tls_key) {
        (Some(cert), Some(key)) => match brep_plm::tls::load(cert, key) {
            Ok(tls) => Some(tls),
            Err(error) => {
                eprintln!("brep-plm: TLS: {error}");
                std::process::exit(1);
            }
        },
        (None, None) => None,
        _ => {
            eprintln!("brep-plm: --tls-cert and --tls-key go together");
            usage();
        }
    };
    config.tls = tls.is_some();
    // Serving HTTPS itself, the server knows every request is secure.
    config.secure_cookies = secure_cookies.unwrap_or(if config.tls { CookieSecurity::On } else { CookieSecurity::Off });

    let (db, seeded) = match Db::open_with_scripts(&data, scripts) {
        Ok(opened) => opened,
        Err(error) => {
            eprintln!("brep-plm: cannot open {data}: {error}");
            std::process::exit(1);
        }
    };

    if let Some(password) = seeded {
        // Printed once, to stdout, and never stored in recoverable form.
        println!("\n  ── first run ──────────────────────────────────────────");
        println!("  an administrator account was created:");
        println!("      username  admin");
        println!("      password  {password}");
        println!("  this password is shown ONCE and is not recoverable.");
        println!("  ───────────────────────────────────────────────────────\n");
    }

    if config.cad_app_dir.is_none() {
        config.cad_app_dir = Some(std::path::Path::new(&data).join(brep_plm::cad::DEFAULT_DIR));
    }
    if let Some(dir) = &config.cad_app_dir {
        let state = if dir.join("web/index.html").is_file() { "" } else { " (no web/index.html there yet)" };
        println!("brep-plm: CAD app from {}{state}", dir.display());
    }
    let db = db.with_config(config);
    println!("brep-plm: scripts in {}", db.scripts().dir().display());
    if db.security().config.lock_script_editor {
        println!("brep-plm: the in-browser script editor is locked off");
    }
    let db = Arc::new(db);
    let every = db.security().config.backup_every_minutes;
    if every > 0 {
        println!(
            "brep-plm: a backup every {every} minute(s) into {}",
            db.security().config.backup_dir.as_ref().map(|d| d.display().to_string()).unwrap_or_default()
        );
        let scheduled = Arc::clone(&db);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(std::time::Duration::from_secs(every * 60));
            tick.tick().await; // the first tick is immediate; the first backup is one period in
            loop {
                tick.tick().await;
                let db = Arc::clone(&scheduled);
                match tokio::task::spawn_blocking(move || db.backup_now("schedule")).await {
                    Ok(Ok(last)) => println!("brep-plm: backup {} ({} files, {} bytes)", last.file, last.files, last.size),
                    Ok(Err(error)) => eprintln!("brep-plm: scheduled backup failed: {}", error.message),
                    Err(error) => eprintln!("brep-plm: scheduled backup panicked: {error}"),
                }
            }
        });
    }
    let app = api::router(db);
    let listener = match tokio::net::TcpListener::bind(&bind).await {
        Ok(listener) => listener,
        Err(error) => {
            eprintln!("brep-plm: cannot bind {bind}: {error}");
            std::process::exit(1);
        }
    };
    let shutdown = async {
        let _ = tokio::signal::ctrl_c().await;
        println!("\nbrep-plm: shutting down");
    };
    // One line naming the address actually bound, so a runner that asked
    // for port 0 learns the port without racing for one itself.
    match listener.local_addr() {
        Ok(addr) => println!("brep-plm: listening on {addr}"),
        Err(error) => eprintln!("brep-plm: cannot read the bound address: {error}"),
    }
    if let Some(tls) = tls {
        println!("brep-plm: serving {data} over https");
        brep_plm::tls::serve(listener, app, tls, shutdown).await;
        return;
    }
    println!("brep-plm: serving {data} over http");
    let service = app.into_make_service_with_connect_info::<std::net::SocketAddr>();
    if let Err(error) = axum::serve(listener, service).with_graceful_shutdown(shutdown).await {
        eprintln!("brep-plm: {error}");
        std::process::exit(1);
    }
}

/// `brep-plm migrate [--data <dir>] [--dry-run]`: import a file-store data
/// directory into SQLite, or with `--dry-run` only say what it would import.
fn migrate(mut args: impl Iterator<Item = String>) -> ! {
    let mut data = DEFAULT_DATA.to_string();
    let mut dry_run = false;
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--data" => data = args.next().unwrap_or_else(|| usage()),
            "--dry-run" => dry_run = true,
            other => {
                eprintln!("brep-plm: unknown option '{other}'");
                usage();
            }
        }
    }
    // A real import renames files in the directory, so it waits for no
    // server to be using it; a dry run only reads, and builds elsewhere.
    let _lock = if dry_run {
        None
    } else {
        match brep_plm::dirlock::DirLock::acquire(std::path::Path::new(&data)) {
            Ok(lock) => Some(lock),
            Err(error) => {
                eprintln!("brep-plm: migrate {data}: {error}");
                std::process::exit(1)
            }
        }
    };
    match brep_plm::db::migrate_files(std::path::Path::new(&data), dry_run) {
        Ok(Some(report)) => {
            println!("brep-plm: {report}");
            std::process::exit(0)
        }
        Ok(None) => {
            println!("brep-plm: nothing to migrate in {data} (no plm.json, or plm.sqlite already exists)");
            std::process::exit(0)
        }
        Err(error) => {
            eprintln!("brep-plm: migrate {data}: {error}");
            std::process::exit(1)
        }
    }
}

/// Pull `--flag value` pairs out of the arguments.
fn flags(args: impl Iterator<Item = String>, known: &[&str]) -> std::collections::BTreeMap<String, String> {
    let mut out = std::collections::BTreeMap::new();
    let mut args = args;
    while let Some(flag) = args.next() {
        if !known.contains(&flag.as_str()) {
            eprintln!("brep-plm: unknown option '{flag}'");
            usage();
        }
        let value = args.next().unwrap_or_else(|| usage());
        out.insert(flag, value);
    }
    out
}

/// `brep-plm backup`: a consistent backup, while a server runs or not.
fn backup(args: impl Iterator<Item = String>) -> ! {
    let f = flags(args, &["--data", "--scripts", "--out"]);
    let data = std::path::PathBuf::from(f.get("--data").cloned().unwrap_or_else(|| DEFAULT_DATA.into()));
    let Some(out) = f.get("--out").map(std::path::PathBuf::from) else {
        eprintln!("brep-plm: backup needs --out <file.tar.gz>");
        usage()
    };
    let scripts = f.get("--scripts").map(std::path::PathBuf::from);
    match brep_plm::backup::backup(&data, scripts.as_deref(), &out) {
        Ok(manifest) => {
            println!(
                "brep-plm: backed up {} to {}: {} files, {} bytes of data, seq {}{}",
                data.display(),
                out.display(),
                manifest.files.len(),
                manifest.total_size(),
                manifest.seq,
                if manifest.attempts > 1 { format!(", {} tries", manifest.attempts) } else { String::new() }
            );
            if let Some(notice) = manifest.adjusted_notice() {
                println!("brep-plm: note: {notice}");
            }
            std::process::exit(0)
        }
        Err(error) => {
            eprintln!("brep-plm: backup {}: {error}", data.display());
            std::process::exit(1)
        }
    }
}

/// `brep-plm restore`: into an empty directory, every file checked.
fn restore(args: impl Iterator<Item = String>) -> ! {
    let f = flags(args, &["--from", "--data"]);
    let (Some(from), Some(data)) = (f.get("--from"), f.get("--data")) else {
        eprintln!("brep-plm: restore needs --from <file.tar.gz> and --data <new dir>");
        usage()
    };
    match brep_plm::backup::restore(std::path::Path::new(from), std::path::Path::new(data)) {
        Ok(manifest) => {
            println!(
                "brep-plm: restored {} files ({} bytes) into {}, seq {}; serve it with --data {data}",
                manifest.files.len(),
                manifest.total_size(),
                data,
                manifest.seq
            );
            if let Some(notice) = manifest.adjusted_notice() {
                println!("brep-plm: note: {notice}");
            }
            std::process::exit(0)
        }
        Err(error) => {
            eprintln!("brep-plm: restore {from}: {error}");
            std::process::exit(1)
        }
    }
}

/// `brep-plm export`: the metadata as JSON, or parts or structure as CSV.
fn export(args: impl Iterator<Item = String>) -> ! {
    let f = flags(args, &["--data", "--out", "--format"]);
    let data = std::path::PathBuf::from(f.get("--data").cloned().unwrap_or_else(|| DEFAULT_DATA.into()));
    let Some(out) = f.get("--out") else {
        eprintln!("brep-plm: export needs --out <file>");
        usage()
    };
    let format = f.get("--format").map(String::as_str).unwrap_or("json");
    let state = match brep_plm::backup::load_snapshot(&data) {
        Ok(state) => state,
        Err(error) => {
            eprintln!("brep-plm: export {}: {error}", data.display());
            std::process::exit(1)
        }
    };
    let text = match format {
        "json" => serde_json::to_string_pretty(&brep_plm::backup::export_json(&state)).expect("JSON"),
        "parts-csv" => brep_plm::backup::parts_csv(&state),
        "structure-csv" => brep_plm::backup::structure_csv(&state),
        other => {
            eprintln!("brep-plm: --format is json, parts-csv or structure-csv, not '{other}'");
            usage()
        }
    };
    if let Err(error) = std::fs::write(out, text) {
        eprintln!("brep-plm: export {out}: {error}");
        std::process::exit(1)
    }
    println!("brep-plm: exported {} parts from {} to {out} ({format})", state.parts.len(), data.display());
    std::process::exit(0)
}
