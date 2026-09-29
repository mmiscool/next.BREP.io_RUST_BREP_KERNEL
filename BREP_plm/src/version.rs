//! The version handshake: what this server is, and the oldest CAD app it
//! serves.
//!
//! Both are semver strings, `MAJOR.MINOR.PATCH`, and both ride `/api/me`
//! (`server_version`, `min_client_version`) so the CAD app checks them right
//! after sign-in and refuses to connect on a mismatch, naming both
//! (plm-cad-integration-todo §2 P1). This file is their single source: every
//! route that reports them (`/api/me`, and `GET /cad/config` when it lands)
//! reads these constants — through `ServerConfig::min_client_version`, which
//! is [`MIN_CLIENT_VERSION`] unless `serve --min-client-version` raised it.
//!
//! # Who compares what
//!
//! The CAD app compares its OWN version (`BREP_app`'s `Cargo.toml` version,
//! `CARGO_PKG_VERSION` there) against [`MIN_CLIENT_VERSION`]: an app older
//! than it is refused. The server does not see the client's version, so the
//! refusal is the client's; [`serves`] is the comparison, written once here
//! so a test on either side can pin the same rule.
//!
//! # Bumping [`MIN_CLIENT_VERSION`]
//!
//! Only when this server stops honouring a contract an older app relies on —
//! a route removed or reshaped, a field renamed, a refusal that changes
//! meaning (the contracts are plm-cad-integration-todo §4). Adding a route or
//! a field never bumps it. Set it to the first `BREP_app` version that speaks
//! the new contract, in the same commit as the change that breaks the old
//! one, and say so in that commit's validation record.

/// This server's version: `BREP_plm`'s own `Cargo.toml` version.
pub const SERVER_VERSION: &str = env!("CARGO_PKG_VERSION");

/// The oldest CAD app (`BREP_app` version) this server serves. `0.4.0` is the
/// app's version when the handshake was built, before any app spoke to a PLM.
pub const MIN_CLIENT_VERSION: &str = "0.4.0";

/// What this server can do beyond the contracts every served client may
/// assume, by name. It rides `/api/me` and `GET /cad/config` as `features`.
///
/// A client reads it to tell an old server from a new one where the
/// difference is otherwise invisible: a server without `attribute-filters`
/// ignores `attr.*` query parameters and answers UNFILTERED, which looks
/// exactly like a filter every part matched.
///
/// Add a name in the commit that adds the capability. A name is never
/// removed or renamed without bumping [`MIN_CLIENT_VERSION`] (an older app may
/// rely on it). The names are listed in plm-cad-integration-todo §4.
pub const FEATURES: &[&str] = &[
    // P6: /api/workspaces and /api/workspace/*, and `newest_revision` on
    // GET /api/parts/:id, which also takes a number.
    "workspaces",
    // P7: attr.<key>=, attr.<key>.min / .max and include=attributes on
    // GET /api/parts.
    "attribute-filters",
    // P8: external_ref and origin on POST /api/parts (external_ref unique per
    // part type), origin on POST /api/parts/:id/revisions, and
    // ?external_ref= / ?part_type= with external_ref and latest_* on rows.
    "external-ref",
];

/// `MAJOR.MINOR.PATCH` as numbers. A pre-release or build suffix
/// (`-rc.1`, `+abc`) is ignored — the handshake compares releases.
pub fn parse(version: &str) -> Option<(u64, u64, u64)> {
    let core = version.trim().split(['-', '+']).next()?;
    let mut fields = core.split('.').map(|f| f.parse::<u64>().ok());
    let triple = (fields.next()??, fields.next()??, fields.next()??);
    fields.next().is_none().then_some(triple)
}

/// Whether a client at `client_version` is served by a server whose oldest
/// served client is `min_client_version`. A version that does not parse is
/// not served: the app would otherwise connect on a guess.
pub fn serves(min_client_version: &str, client_version: &str) -> bool {
    match (parse(min_client_version), parse(client_version)) {
        (Some(min), Some(client)) => client >= min,
        _ => false,
    }
}

/// Check a `--min-client-version` floor: a version, and not below
/// [`MIN_CLIENT_VERSION`] — lowering it would claim to serve apps that speak
/// a contract this server no longer honours.
pub fn check_floor(floor: &str) -> Result<(), String> {
    if parse(floor).is_none() {
        return Err(format!("'{floor}' is not a MAJOR.MINOR.PATCH version"));
    }
    if !serves(MIN_CLIENT_VERSION, floor) {
        return Err(format!("{floor} is below this server's oldest served client, {MIN_CLIENT_VERSION}; the floor can only be raised"));
    }
    Ok(())
}

