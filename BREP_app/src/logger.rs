//! The native binary's `log` sink: `RUST_LOG`-filtered lines to stderr.
//!
//! This exists so the warnings eframe, winit and wgpu emit through the `log`
//! facade reach the terminal — the app itself logs three lines. It replaces
//! `env_logger`, which cost twelve crates for that: a full `regex`, `jiff` +
//! `jiff-core` (a date library, for timestamps we never print) and the
//! `anstream`/`anstyle`/`colorchoice` colour stack.
//!
//! What it keeps of `env_logger`'s behaviour: the `RUST_LOG` variable, its
//! comma-separated `level` / `target=level` directives (longest matching target
//! prefix wins, exactly as `env_logger` resolves them), the `Error` default
//! when `RUST_LOG` is unset or unparseable, and the `[LEVEL target] message`
//! line shape. What it drops: colour, timestamps, and regex message filters
//! (`RUST_LOG=…/pattern`) — none of which this binary ever asked for. A `/`
//! suffix is parsed off and ignored rather than misread as part of a target.
//!
//! Native only: the wasm build logs through the browser console.

use std::io::Write as _;

/// One `target=level` directive, or a bare `level` when `target` is empty.
struct Directive {
    target: String,
    level: log::LevelFilter,
}

struct StderrLogger {
    /// Longest-prefix-first, so `resolve` can take the first match.
    directives: Vec<Directive>,
    /// The loosest level any directive allows — what `enabled` answers with
    /// before a target is known, and the ceiling `log::set_max_level` gets.
    max: log::LevelFilter,
}

impl StderrLogger {
    /// The level that applies to `target`: the longest directive prefix that
    /// matches it, else the bare-level directive, else `Off`.
    fn resolve(&self, target: &str) -> log::LevelFilter {
        for d in &self.directives {
            if d.target.is_empty() || target.starts_with(&d.target) {
                return d.level;
            }
        }
        log::LevelFilter::Off
    }
}

impl log::Log for StderrLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        // Cheap reject before the per-target scan: nothing above the loosest
        // directive can pass, whatever its target.
        metadata.level() <= self.max && metadata.level() <= self.resolve(metadata.target())
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }
        // Ignore a closed/broken stderr: a logger must never take the app down.
        let _ = writeln!(
            std::io::stderr(),
            "[{} {}] {}",
            record.level(),
            record.target(),
            record.args()
        );
    }

    fn flush(&self) {
        let _ = std::io::stderr().flush();
    }
}

/// `RUST_LOG` as directives, longest target prefix first. An empty or
/// all-unparseable spec yields the `Error` default `env_logger` also uses.
fn parse(spec: &str) -> Vec<Directive> {
    // `env_logger` splits a regex message filter off at the first `/`.
    let spec = spec.split('/').next().unwrap_or("");
    let mut directives: Vec<Directive> = Vec::new();
    for part in spec.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let (target, level) = match part.split_once('=') {
            // `target=level`; a bare `target=` means "everything from target".
            // A bare `=` names nothing, so it is dropped rather than read as a
            // global `trace` — which is what `env_logger` does with it too.
            Some((t, "")) if t.trim().is_empty() => continue,
            Some((t, "")) => (t.trim(), log::LevelFilter::Trace),
            Some((t, l)) => match l.trim().parse() {
                Ok(level) => (t.trim(), level),
                Err(_) => continue,
            },
            // A bare word is a level if it parses as one, a target otherwise.
            None => match part.parse() {
                Ok(level) => ("", level),
                Err(_) => (part, log::LevelFilter::Trace),
            },
        };
        directives.push(Directive { target: target.to_owned(), level });
    }
    if directives.is_empty() {
        directives.push(Directive { target: String::new(), level: log::LevelFilter::Error });
    }
    // Longest target first so the most specific directive wins; the bare-level
    // directive (empty target) sorts last and acts as the fallback.
    directives.sort_by(|a, b| b.target.len().cmp(&a.target.len()));
    directives
}

/// Install the logger. Mirrors `env_logger::try_init`: `Err` if some other
/// logger is already installed, and the caller is free to ignore that.
pub fn try_init() -> Result<(), log::SetLoggerError> {
    let directives = parse(&std::env::var("RUST_LOG").unwrap_or_default());
    let max = directives.iter().map(|d| d.level).max().unwrap_or(log::LevelFilter::Error);
    log::set_boxed_logger(Box::new(StderrLogger { directives, max }))?;
    log::set_max_level(max);
    Ok(())
}

