# BREP_plm — a PLM server for the BREP CAD application

`BREP_plm` is a self-hosted product lifecycle management server: parts with
part numbers as their database keys, revisions with a lifecycle,
explicit checkout, bills of materials, change orders and review, over one
embedded SQLite file. It serves its own browser UI and the document API that
the BREP CAD application saves and opens through, and it can host the CAD
application's web build on the same origin.

It is for a team that wants its CAD documents under revision control on a
machine it runs, without a separate database server: one binary, one data
directory.

## Quick start

```sh
cargo install BREP_plm
brep-plm serve --data ./plm-data --bind 127.0.0.1:8088
```

The UI is embedded in the executable by default. To override it with files from
an existing frontend directory, start with `--web-dir`:

```sh
brep-plm serve --data ./plm-data --web-dir ./BREP_plm/web
```

Files in that directory override embedded assets at the same relative URLs
(`/` uses `index.html`). Missing files fall back to the embedded bundle.
Additional frontend assets and templates can also be served from this directory.
Files are read on each request and responses use `Cache-Control: no-store` while
this option is enabled, so editing HTML, JavaScript, or CSS only needs a browser
refresh. API and hosted CAD routes remain separate. Without `--web-dir`, the
server continues to use its embedded frontend.

The first run creates `plm-data/` and seeds one administrator, printing its
generated password **once**:

```text
  an administrator account was created:
      username  admin
      password  ...
  this password is shown ONCE and is not recoverable.
brep-plm: listening on 127.0.0.1:8088
```

Open `http://127.0.0.1:8088/`, sign in as `admin`, and create a second
administrator before you lose the password. SQLite is compiled in, so there is
no system library to install.

The part number is the actual database ID. Revision IDs are their labels,
scoped by part (for example, `CPART000000001` / `A`). Documents are stored
under `models/<part-number>/<revision-label>.json`, with unsafe filename
characters percent-encoded.

This development version requires a fresh data directory when upgrading from
opaque model IDs. Stop the old server, reset its data directory and CAD caches,
then start this version. Existing model data is not migrated.

## What it does

- **Batch resave.** Select parts with the row checkboxes, Shift/Ctrl, or
  **Select all matching**, then **Force resave selected parts**. Each saved
  model gets a persistent job in the existing bake queue. The bake worker
  rebuilds and resaves it with an embedded thumbnail; progress, failures and
  retries remain available in **Bake queue** after the page closes. Existing
  checkouts remain held; jobs wait on another user's checkout. A newer save
  supersedes an older queued result. Run the normal `brep-app --bake-worker`
  with a worker token to process these jobs alongside family/template bakes.
- **Parts and revisions.** Part types number by counter (`CPART` + 9 digits by
  default), free text, an administrator's pattern, or a script. Revisions move
  `Draft → InReview → Released → Superseded | Obsolete`; a released revision is
  immutable, and a draft is written only by the user holding its checkout.
- **Structure.** Each revision's uses list, an expandable BOM table tree with
  connector lines, per-node toggles, expand/collapse all and expansion to a
  chosen depth. The flat BOM rolls up quantities and costs; revision diff
  and where-used show changes and dependencies.
- **Revision and occurrence fields.** Administrators define typed fields on
  each numbered part type through **Part types → Fields**, and placement fields
  through **Occurrence fields**. Part values belong to the part revision;
  occurrence values belong to the owning assembly revision and stable CAD
  feature ID. Both the PLM BOM and CAD BOM edit the same stored values.
- **BOM column configurations.** The separate **BOM columns** page manages
  named shared and personal configurations, with available and configured
  fields in two lists. Add/remove multiple fields and move selected columns
  up or down while preserving their order. The CAD chooser uses the same
  server configurations and can save personal configurations. Changing a view
  does not remove attribute data. CSV exports follow the selected configuration.
- **Change orders and review.** A set of revisions released or obsoleted
  together after one review round, all or none; reviewers, a release gate,
  threaded discussion and an inbox.
- **Catalog and sourcing.** A category tree with inherited typed attributes,
  tags, manufacturers and suppliers, and each part's manufacturer part numbers
  with supplier offers and price breaks.
- **Families and templates.** Table-driven part families and template
  spin-out, with a bake queue for headless CAD workers.
- **Attachments and workspaces.** Files on parts and versioned per-user
  workspaces, with upload limits.
- **Administrator scripts.** JavaScript hooks (part number, revision label,
  before and after release, review events) evaluated by
  [`BREP_script_core`](https://crates.io/crates/BREP_script_core), with an
  in-browser editor and a test run. A directory of scripts can live in git.
- **An audit log** of every committed change, written in the same transaction.
- **Security.** PBKDF2 password verifiers, cookie sessions with CSRF tokens,
  scoped API tokens (`Authorization: Bearer plm_…`) for everything that is not a
  browser, a sign-in throttle, session limits, security headers, and optional
  native TLS.

## Running it on a network

```sh
brep-plm serve --data /srv/plm --bind 0.0.0.0:443 \
               --tls-cert /etc/plm/fullchain.pem --tls-key /etc/plm/privkey.pem
```

Behind a reverse proxy instead, serve plain HTTP on loopback and pass
`--trust-proxy --secure-cookies on`. `--lock-script-editor` turns the
in-browser script editor off whatever the settings say. `brep-plm --help`
lists every option, including the upload limits and the oldest CAD client
version the server accepts.

One server owns a data directory: a second one started on the same directory
is refused and names the holder's process id. Each change is committed with a
full fsync, so give the data directory a disk that is not busy with other
syncing work if write latency matters.

## Backup, restore and export

```sh
brep-plm backup  --data ./plm-data --out plm-backup.tar.gz   # while serve runs, or not
brep-plm restore --from plm-backup.tar.gz --data ./plm-restored
brep-plm export  --data ./plm-data --out plm.json [--format json|parts-csv|structure-csv]
brep-plm serve   --data ./plm-data --backup-dir /backups/plm --backup-every 60 --backup-keep 48
```

A backup is one `.tar.gz` holding a consistent SQLite snapshot, every revision
document, every attachment and the scripts directory, with a manifest of sizes
and SHA-256 hashes; `restore` checks it into an empty directory.

## Hosting the CAD application

`--cad-app <dir>` (default `<data>/cad-app`) serves a web build of the BREP CAD
application at `/cad/app/web/index.html`, same-origin with the API. The
directory holds the build's `web/` and `pkg/` directories side by side;
upgrading the CAD application is a file drop into it, not a server release.

## Embedding the server

The binary is a thin wrapper over the library, whose [`axum`](https://docs.rs/axum)
router can be served by any host:

```rust,no_run
use std::net::SocketAddr;
use std::sync::Arc;

use brep_plm::{api, db::Db};

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let (db, first_admin_password) = Db::open("./plm-data")?;
    if let Some(password) = first_admin_password {
        println!("admin password (shown once): {password}");
    }
    let app = api::router(Arc::new(db));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:8088").await?;
    axum::serve(listener, app.into_make_service_with_connect_info::<SocketAddr>()).await
}
```

This needs `tokio` (with `macros` and `rt-multi-thread`) and `axum` 0.7 beside
`BREP_plm` in your own manifest.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.

Administrators can use **Administration → Setup wizard** to configure numbering,
catalog taxonomy, accounts and lifecycle labels, with optional KiCad symbols, linked footprints and STEP models in KiCad’s
library taxonomy, plus mechanical family imports. **Workflows** provides a visual process editor,
parallel reviews, smart forms with field permissions, JavaScript automation,
lifecycle steps, an inbox and durable execution history. See the
[operator guide](GUIDE.md#workflow-processes-and-smart-forms) for configuration
and integration details.

Admins can start/stop a server-owned native bake worker from the Bake queue
when the operator configures `--bake-worker-executable`,
`--bake-worker-token-file` and `--bake-worker-url`. Queued work survives Stop;
interrupted claims can be taken back or recovered after lease expiry.

KiCad setup defaults to all pinned official symbol libraries and the dedicated
`kicad` counter (`ELEC00000000001` initially), with library-based taxonomy and
linked available footprint/STEP assets. Browser checkpoints support pause,
resume, detailed reports and retry of libraries with failures. Generic symbols
and unavailable linked models remain visible gaps.
