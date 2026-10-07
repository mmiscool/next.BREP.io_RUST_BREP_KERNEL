//! The metadata store on disk: one SQLite file, `plm.sqlite`, in WAL mode.
//!
//! # Shape
//!
//! Each kind of record has its own table. The columns the store queries or
//! constrains on are real columns (a part's number, a revision's label, a
//! use's child part, an MPN), and the rest of the record is a JSON `body`
//! column. That keeps every field the model has without a column per field,
//! and a field a later slice adds needs no schema change: it rides in `body`.
//! A record's children live in their own tables, not in its `body`:
//!
//! | table | one row per | constrained / indexed on |
//! |---|---|---|
//! | `parts` | part | `ord` (creation order, also the FTS rowid), `id`, `number_lc` UNIQUE, `category_lc` |
//! | `revisions` | revision | `(part_id, label_lc)` UNIQUE |
//! | `uses` | line of a revision's uses list | `child_part`, `child_revision` |
//! | `manufacturer_parts` | MPN on a part | `manufacturer`, `mpn_lc` |
//! | `supplier_offers` | offer under an MPN | `supplier`, `spn_lc` |
//! | `part_tags` | tag on a part | `tag_lc` |
//! | `users`, `sessions`, `api_tokens`, `part_types`, `categories`, `manufacturers`, `suppliers` | record | their ids |
//! | `change_orders` | change order | `number_lc` UNIQUE, `state` |
//! | `workspace` | workspace folder, link or file | `(owner, parent)`, `ord` |
//! | `changes` | document key in the change log | `seq` |
//! | `preferences` | one reserved key of one user ([`crate::api::preferences`]), not in the in-memory state | `(user_id, key)`, `(user_id, version)` |
//! | `audit` | audit event | `at`, the actor, `kind`, `entity_id`, `part_id`, `action` |
//! | `meta` | the sequence, the settings, and any state field with no table | `key` |
//! | `parts_fts` | part (FTS5, trigram, contentless) | number, name, description, category, tags, attribute values, MPNs, SPNs |
//!
//! Documents are NOT here: they stay one file each under `models/`, as the CAD
//! app writes them.
//!
//! # Reads
//!
//! The server still answers reads from the whole state held in memory
//! ([`crate::db::Db::read`]), which this file loads at start-up. What SQL
//! adds on the read side is the search index and the audit log's indexed
//! queries, through a second connection so a search never waits on a write.
//!
//! # Versions
//!
//! `PRAGMA user_version` is the schema version. [`MIGRATIONS`] are applied in
//! order, each in its own transaction, from whatever version the file has.
//! A new schema change is a new entry at the end — never an edit to an old
//! one, which a file in the field has already run.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use rusqlite::{params, params_from_iter, Connection, OptionalExtension, Transaction};
use serde_json::{Map, Value};

use crate::audit::{AuditFilter, AuditSink};
use crate::catalog;
use crate::db::State;
use crate::model::{AuditEvent, Category, ManufacturerPart, Part, Revision, Session, SupplierOffer, Use, WorkspaceEntry};
use crate::table::Parts;

/// The schema, one entry per version. Index `i` takes a file from version `i`
/// to `i + 1`.
const MIGRATIONS: &[&str] = &[
    // 1: the first schema.
    r#"
    CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
    CREATE TABLE users (id TEXT PRIMARY KEY, username_lc TEXT NOT NULL UNIQUE, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE TABLE sessions (token TEXT PRIMARY KEY, user_id TEXT NOT NULL, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE INDEX sessions_user ON sessions(user_id);
    CREATE TABLE api_tokens (id TEXT PRIMARY KEY, user_id TEXT NOT NULL, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE TABLE part_types (id TEXT PRIMARY KEY, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE TABLE categories (id TEXT PRIMARY KEY, id_lc TEXT NOT NULL UNIQUE, parent TEXT NOT NULL, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE TABLE manufacturers (id TEXT PRIMARY KEY, ord INTEGER NOT NULL, body TEXT NOT NULL);
    CREATE TABLE suppliers (id TEXT PRIMARY KEY, ord INTEGER NOT NULL, body TEXT NOT NULL);

    CREATE TABLE parts (
        ord INTEGER PRIMARY KEY,
        id TEXT NOT NULL UNIQUE,
        number TEXT NOT NULL,
        number_lc TEXT NOT NULL UNIQUE,
        part_type TEXT NOT NULL,
        category_lc TEXT NOT NULL,
        document_class TEXT NOT NULL,
        created_at INTEGER NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX parts_category ON parts(category_lc);
    CREATE TABLE revisions (
        id TEXT PRIMARY KEY,
        part_id TEXT NOT NULL REFERENCES parts(id) ON DELETE CASCADE,
        ord INTEGER NOT NULL,
        label_lc TEXT NOT NULL,
        lifecycle TEXT NOT NULL,
        body TEXT NOT NULL,
        UNIQUE (part_id, label_lc)
    );
    CREATE TABLE uses (
        revision_id TEXT NOT NULL REFERENCES revisions(id) ON DELETE CASCADE,
        ord INTEGER NOT NULL,
        child_part TEXT NOT NULL,
        child_revision TEXT NOT NULL,
        quantity REAL NOT NULL,
        body TEXT NOT NULL,
        PRIMARY KEY (revision_id, ord)
    );
    CREATE INDEX uses_child_part ON uses(child_part);
    CREATE INDEX uses_child_revision ON uses(child_revision);
    CREATE TABLE manufacturer_parts (
        id TEXT PRIMARY KEY,
        part_id TEXT NOT NULL REFERENCES parts(id) ON DELETE CASCADE,
        ord INTEGER NOT NULL,
        manufacturer TEXT NOT NULL,
        mpn_lc TEXT NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX mp_part ON manufacturer_parts(part_id);
    CREATE INDEX mp_manufacturer ON manufacturer_parts(manufacturer);
    CREATE INDEX mp_mpn ON manufacturer_parts(mpn_lc);
    CREATE TABLE supplier_offers (
        id TEXT PRIMARY KEY,
        manufacturer_part_id TEXT NOT NULL REFERENCES manufacturer_parts(id) ON DELETE CASCADE,
        ord INTEGER NOT NULL,
        supplier TEXT NOT NULL,
        spn_lc TEXT NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX offer_mp ON supplier_offers(manufacturer_part_id);
    CREATE INDEX offer_supplier ON supplier_offers(supplier);
    CREATE INDEX offer_spn ON supplier_offers(spn_lc);
    CREATE TABLE part_tags (
        part_id TEXT NOT NULL REFERENCES parts(id) ON DELETE CASCADE,
        tag_lc TEXT NOT NULL,
        PRIMARY KEY (part_id, tag_lc)
    );
    CREATE INDEX tags_tag ON part_tags(tag_lc);

    CREATE TABLE changes (key TEXT PRIMARY KEY, seq INTEGER NOT NULL);
    CREATE INDEX changes_seq ON changes(seq);

    CREATE TABLE audit (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        at INTEGER NOT NULL,
        actor_user_id TEXT NOT NULL,
        actor_username_lc TEXT NOT NULL,
        action TEXT NOT NULL,
        kind TEXT NOT NULL,
        entity_id TEXT NOT NULL,
        part_id TEXT NOT NULL,
        seq INTEGER NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX audit_at ON audit(at);
    CREATE INDEX audit_actor ON audit(actor_user_id);
    CREATE INDEX audit_actor_name ON audit(actor_username_lc);
    CREATE INDEX audit_kind ON audit(kind);
    CREATE INDEX audit_entity ON audit(entity_id);
    CREATE INDEX audit_part ON audit(part_id);
    CREATE INDEX audit_action ON audit(action);

    CREATE VIRTUAL TABLE parts_fts USING fts5(
        number, name, description, category_id, category_path, tags, attributes, mpn, spn,
        tokenize = 'trigram', content = '', contentless_delete = 1
    );
    "#,
    // 2: change orders.
    r#"
    CREATE TABLE change_orders (
        id TEXT PRIMARY KEY,
        ord INTEGER NOT NULL,
        number_lc TEXT NOT NULL UNIQUE,
        state TEXT NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX change_orders_state ON change_orders(state);
    "#,
    // 3: each user's preferences (`crate::api::preferences`). Not part of the
    // in-memory state: a value is read and written here directly. A row whose
    // `value` is NULL is a deleted key, kept so a client polling `since` sees
    // the deletion; `version` counts per user and never goes down.
    r#"
    CREATE TABLE preferences (
        user_id TEXT NOT NULL,
        key TEXT NOT NULL,
        version INTEGER NOT NULL,
        modified INTEGER NOT NULL,
        size INTEGER NOT NULL,
        value TEXT,
        PRIMARY KEY (user_id, key)
    );
    CREATE INDEX preferences_version ON preferences(user_id, version);
    "#,
    // 4: workspaces (plm-cad-integration P6): one row per folder, link or file.
    r#"
    CREATE TABLE IF NOT EXISTS workspace (
        id TEXT PRIMARY KEY,
        ord INTEGER NOT NULL,
        owner TEXT NOT NULL,
        parent TEXT NOT NULL,
        body TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS workspace_folder ON workspace(owner, parent);
    CREATE INDEX IF NOT EXISTS workspace_ord ON workspace(ord);
    "#,
    // 5: readable part primary keys and revisions scoped to their owning part.
    r#"
    CREATE TABLE parts_new (
        id TEXT PRIMARY KEY NOT NULL,
        ord INTEGER NOT NULL UNIQUE,
        number TEXT NOT NULL,
        number_lc TEXT NOT NULL UNIQUE,
        part_type TEXT NOT NULL,
        category_lc TEXT NOT NULL,
        document_class TEXT NOT NULL,
        created_at INTEGER NOT NULL,
        body TEXT NOT NULL
    );
    INSERT INTO parts_new SELECT id, ord, number, number_lc, part_type, category_lc, document_class, created_at, body FROM parts;
    CREATE TABLE revisions_new (
        id TEXT NOT NULL,
        part_id TEXT NOT NULL REFERENCES parts(id) ON DELETE CASCADE,
        ord INTEGER NOT NULL,
        label_lc TEXT NOT NULL,
        lifecycle TEXT NOT NULL,
        body TEXT NOT NULL,
        PRIMARY KEY(part_id, id),
        UNIQUE(part_id, label_lc)
    );
    INSERT INTO revisions_new SELECT * FROM revisions;
    CREATE TABLE uses_new (
        part_id TEXT NOT NULL,
        revision_id TEXT NOT NULL,
        ord INTEGER NOT NULL,
        child_part TEXT NOT NULL,
        child_revision TEXT NOT NULL,
        quantity REAL NOT NULL,
        body TEXT NOT NULL,
        PRIMARY KEY(part_id, revision_id, ord),
        FOREIGN KEY(part_id, revision_id) REFERENCES revisions(part_id, id) ON DELETE CASCADE
    );
    INSERT INTO uses_new SELECT r.part_id, u.revision_id, u.ord, u.child_part, u.child_revision, u.quantity, u.body FROM uses u JOIN revisions r ON r.id = u.revision_id;
    DROP TABLE uses;
    DROP TABLE revisions;
    DROP TABLE parts;
    ALTER TABLE parts_new RENAME TO parts;
    ALTER TABLE revisions_new RENAME TO revisions;
    ALTER TABLE uses_new RENAME TO uses;
    CREATE INDEX parts_category ON parts(category_lc);
    CREATE INDEX uses_child_part ON uses(child_part);
    CREATE INDEX uses_child_revision ON uses(child_part, child_revision);
    "#,

];

/// The schema version this build writes. A file with a HIGHER version was
/// written by a newer server, and this one refuses to open it rather than
/// guess what the newer columns mean.
pub fn schema_version() -> i32 {
    MIGRATIONS.len() as i32
}

/// Separates the values of one multi-valued search column (tags, attribute
/// values, MPNs), so a search cannot match across two of them. A trigram
/// phrase never contains it: the search text is one line a person typed.
const SEP: &str = "\u{1f}";

/// The tables whose rows are one keyed record of a top-level state list, and
/// how to fill their indexed columns from a record's JSON.
struct Keyed {
    /// The field of [`State`] and the table's name.
    name: &'static str,
    /// The record field that is the primary key.
    key: &'static str,
    /// Indexed columns: (column, record field, lower-cased?).
    columns: &'static [(&'static str, &'static str, bool)],
}

const KEYED: &[Keyed] = &[
    Keyed { name: "users", key: "id", columns: &[("username_lc", "username", true)] },
    Keyed { name: "api_tokens", key: "id", columns: &[("user_id", "user_id", false)] },
    Keyed { name: "part_types", key: "id", columns: &[] },
    Keyed { name: "categories", key: "id", columns: &[("id_lc", "id", true), ("parent", "parent", false)] },
    Keyed { name: "manufacturers", key: "id", columns: &[] },
    Keyed { name: "suppliers", key: "id", columns: &[] },
    Keyed {
        name: "change_orders",
        key: "id",
        columns: &[("number_lc", "number", true), ("state", "state", false)],
    },
];

/// The state fields with a home of their own; any OTHER top-level field is
/// kept whole in `meta` under `state.<field>`, so a field a later slice adds
/// to [`State`] is persisted before anyone gives it a table.
const OWN_HOME: &[&str] = &["seq", "changes_from", "changes", "settings", "parts", "workspace", "sessions"];

/// How long a commit waits for the disk.
///
/// The server always runs [`Durability::Full`]: a commit is on disk before
/// the response that reports it. [`Durability::Normal`] exists for the test
/// suites, which open hundreds of short-lived stores on a shared disk and
/// spent most of their wall time in fsync. In WAL mode `NORMAL` stays
/// consistent after a crash but may lose the last commits to a power cut, so
/// nothing that serves users opens with it: `tests/storage.rs`
/// (`the_server_opens_its_store_with_full_sync`) pins that.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// `PRAGMA synchronous = FULL`.
    Full,
    /// `PRAGMA synchronous = NORMAL`. Tests only.
    Normal,
}

impl Durability {
    fn pragma(self) -> &'static str {
        match self {
            Durability::Full => "PRAGMA synchronous = FULL;",
            Durability::Normal => "PRAGMA synchronous = NORMAL;",
        }
    }
}

/// The SQLite store: a writer connection used under the store's write lock,
/// a reader for searches and audit queries, and a checkpointer thread.
pub struct Sql {
    path: PathBuf,
    writer: Mutex<Connection>,
    reader: Mutex<Connection>,
    checkpointer: Option<Checkpointer>,
}

/// Folds the write-ahead log back into the main file, off the request path.
///
/// SQLite's default is to checkpoint inside whichever commit pushes the log
/// past 1,000 pages — so one unlucky request pays for copying and syncing
/// everything written since the last one (half a second at small scale,
/// seconds after a bulk import). Here the writer never checkpoints
/// (`wal_autocheckpoint = 0`); this thread does, every second, in PASSIVE
/// mode, which never blocks a writer or a reader.
struct Checkpointer {
    stop: std::sync::mpsc::Sender<()>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Checkpointer {
    fn start(path: &Path, durability: Durability) -> rusqlite::Result<Checkpointer> {
        let conn = Connection::open(path)?;
        configure(&conn, durability)?;
        let (stop, stopped) = std::sync::mpsc::channel::<()>();
        let thread = std::thread::Builder::new()
            .name("plm-checkpoint".into())
            .spawn(move || loop {
                let done = !matches!(
                    stopped.recv_timeout(std::time::Duration::from_secs(1)),
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout)
                );
                if let Err(error) = conn.execute_batch("PRAGMA wal_checkpoint(PASSIVE);") {
                    eprintln!("brep-plm: checkpoint: {error}");
                }
                if done {
                    break;
                }
            })
            .map_err(|e| rusqlite::Error::ToSqlConversionFailure(Box::new(e)))?;
        Ok(Checkpointer { stop, thread: Some(thread) })
    }
}

impl Drop for Sql {
    fn drop(&mut self) {
        if let Some(mut checkpointer) = self.checkpointer.take() {
            let _ = checkpointer.stop.send(());
            if let Some(thread) = checkpointer.thread.take() {
                let _ = thread.join();
            }
        }
    }
}

fn configure(conn: &Connection, durability: Durability) -> rusqlite::Result<()> {
    // WAL: readers never block the writer or each other. synchronous=FULL,
    // the server's only setting (see [`Durability`]): a commit is on disk
    // before the response that reports it, so a number handed out survives a
    // power cut, as it did with the file store.
    conn.execute_batch(durability.pragma())?;
    conn.execute_batch("PRAGMA journal_mode = WAL; PRAGMA foreign_keys = ON;")?;
    conn.busy_timeout(std::time::Duration::from_secs(10))?;
    Ok(())
}

/// Bring `conn`'s schema up to [`schema_version`].
fn migrate(conn: &mut Connection) -> rusqlite::Result<()> {
    let version: i32 = conn.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    if version > schema_version() {
        return Err(rusqlite::Error::InvalidParameterName(format!(
            "plm.sqlite is schema version {version}, and this server only knows up to {} — run a newer server",
            schema_version()
        )));
    }
    for (index, script) in MIGRATIONS.iter().enumerate().skip(version.max(0) as usize) {
        let tx = conn.transaction()?;
        // Development fixtures may replay earlier migrations on a database
        // that already has scoped revision keys.
        let scoped: bool = index == 4 && tx.query_row(
            "SELECT EXISTS(SELECT 1 FROM pragma_table_info('uses') WHERE name = 'part_id')",
            [], |r| r.get(0))?;
        if !scoped { tx.execute_batch(script)?; }
        tx.pragma_update(None, "user_version", index as i32 + 1)?;
        tx.commit()?;
    }
    Ok(())
}

impl Sql {
    /// Open (or create) the store file at `path`, migrating its schema, with
    /// [`Durability::Full`].
    pub fn open(path: impl Into<PathBuf>) -> rusqlite::Result<Sql> {
        Sql::open_with(path, Durability::Full)
    }

    /// [`Sql::open`] at a chosen [`Durability`].
    pub fn open_with(path: impl Into<PathBuf>, durability: Durability) -> rusqlite::Result<Sql> {
        let path = path.into();
        let mut writer = Connection::open(&path)?;
        configure(&writer, durability)?;
        writer.pragma_update(None, "foreign_keys", false)?;
        migrate(&mut writer)?;
        writer.pragma_update(None, "foreign_keys", true)?;
        // The checkpointer thread folds the log back in; once it has, the next
        // write restarts the log from the top, and the file is cut back to
        // 64 MB rather than keeping the size of the largest burst ever.
        writer.execute_batch("PRAGMA wal_autocheckpoint = 0; PRAGMA journal_size_limit = 67108864;")?;
        let reader = Connection::open(&path)?;
        configure(&reader, durability)?;
        let checkpointer = Some(Checkpointer::start(&path, durability)?);
        Ok(Sql { path, writer: Mutex::new(writer), reader: Mutex::new(reader), checkpointer })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The writer connection's `PRAGMA synchronous`: 2 is FULL, 1 NORMAL.
    pub fn synchronous(&self) -> rusqlite::Result<i64> {
        self.writer().query_row("PRAGMA synchronous", [], |row| row.get(0))
    }

    fn writer(&self) -> std::sync::MutexGuard<'_, Connection> {
        self.writer.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn reader(&self) -> std::sync::MutexGuard<'_, Connection> {
        self.reader.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A `meta` value.
    pub fn meta(&self, key: &str) -> rusqlite::Result<Option<String>> {
        self.writer()
            .query_row("SELECT value FROM meta WHERE key = ?1", [key], |row| row.get(0))
            .optional()
    }

    /// Set a `meta` value outside any change (start-up bookkeeping).
    pub fn set_meta(&self, key: &str, value: &str) -> rusqlite::Result<()> {
        self.writer().execute(
            "INSERT INTO meta(key, value) VALUES (?1, ?2) ON CONFLICT(key) DO UPDATE SET value = excluded.value",
            params![key, value],
        )?;
        Ok(())
    }

    /// The whole state, or `None` for a store nothing has been written to.
    pub fn load(&self) -> rusqlite::Result<Option<State>> {
        let conn = self.writer();
        let mut object = Map::new();
        let mut stmt = conn.prepare("SELECT key, value FROM meta")?;
        let mut seen_seq = false;
        for row in stmt.query_map([], |row| Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?)))? {
            let (key, value) = row?;
            let parsed: Value = serde_json::from_str(&value).map_err(json_error)?;
            match key.as_str() {
                "seq" => {
                    seen_seq = true;
                    object.insert(key, parsed);
                }
                "changes_from" | "settings" => {
                    object.insert(key, parsed);
                }
                other => {
                    if let Some(field) = other.strip_prefix("state.") {
                        object.insert(field.to_string(), parsed);
                    }
                }
            }
        }
        if !seen_seq {
            return Ok(None);
        }
        for keyed in KEYED {
            let mut stmt = conn.prepare(&format!("SELECT body FROM {} ORDER BY ord", keyed.name))?;
            let rows: Vec<Value> = stmt
                .query_map([], |row| row.get::<_, String>(0))?
                .map(|body| body.and_then(|b| serde_json::from_str(&b).map_err(json_error)))
                .collect::<rusqlite::Result<_>>()?;
            object.insert(keyed.name.to_string(), Value::Array(rows));
        }
        let mut stmt = conn.prepare("SELECT key, seq FROM changes ORDER BY seq, key")?;
        let changes: Vec<Value> = stmt
            .query_map([], |row| {
                Ok(serde_json::json!({ "key": row.get::<_, String>(0)?, "seq": row.get::<_, i64>(1)? }))
            })?
            .collect::<rusqlite::Result<_>>()?;
        object.insert("changes".into(), Value::Array(changes));

        let mut state: State = serde_json::from_value(Value::Object(object)).map_err(json_error)?;
        state.parts = load_parts(&conn)?;
        state.workspace = load_workspace(&conn)?;
        state.sessions = load_sessions(&conn)?;
        Ok(Some(state))
    }

    /// Write one change in ONE transaction: the state fields that moved, the
    /// parts that moved, the search index for them, and the change's audit
    /// events. Either all of it is on disk when this returns `Ok`, or none of
    /// it is. The events come back numbered.
    pub fn write(&self, change: &Change<'_>, events: Vec<AuditEvent>) -> rusqlite::Result<Vec<AuditEvent>> {
        let mut conn = self.writer();
        let tx = conn.transaction()?;
        write_small(&tx, change.before, change.after)?;
        for (position, part) in &change.parts {
            write_part(&tx, *position, part, change.categories)?;
        }
        for &position in &change.reindex {
            index_part(&tx, position, &change.all_parts[position], change.categories)?;
        }
        for (id, entry) in &change.workspace {
            write_entry(&tx, id, *entry)?;
        }
        for (token, session) in &change.sessions {
            write_session(&tx, token, *session)?;
        }
        let events = insert_events(&tx, events)?;
        tx.commit()?;
        Ok(events)
    }

    /// Every part whose number, name, description, category, tags,
    /// attribute values, MPNs or SPNs contain `needle` as a substring (at
    /// least three characters, ignoring case) — the FTS5 trigram index —
    /// together with the indexed narrowing in `query`. Creation order, from
    /// after `query.after`, at most `query.limit`.
    pub fn candidates(&self, query: &Candidates<'_>) -> rusqlite::Result<Vec<(i64, String)>> {
        let mut sql = String::from("SELECT p.ord, p.id FROM parts p WHERE 1");
        let mut args: Vec<rusqlite::types::Value> = Vec::new();
        if let Some(needle) = query.needle {
            sql.push_str(" AND p.ord IN (SELECT rowid FROM parts_fts WHERE parts_fts MATCH ?)");
            args.push(format!("\"{}\"", needle.replace('"', "\"\"")).into());
        }
        if let Some(ids) = query.categories {
            if ids.is_empty() {
                sql.push_str(" AND 0");
            } else {
                sql.push_str(&format!(" AND p.category_lc IN ({})", vec!["?"; ids.len()].join(",")));
                args.extend(ids.iter().map(|id| id.to_ascii_lowercase().into()));
            }
        }
        if query.uncategorized {
            sql.push_str(" AND p.category_lc NOT IN (SELECT id_lc FROM categories)");
        }
        // `IN (subquery)`, not a correlated `EXISTS`: SQLite then walks the
        // filter's own index once instead of probing it for every part.
        if let Some(tag) = query.tag {
            sql.push_str(" AND p.id IN (SELECT t.part_id FROM part_tags t WHERE t.tag_lc = ?)");
            args.push(tag.to_ascii_lowercase().into());
        }
        if let Some(manufacturer) = query.manufacturer {
            sql.push_str(" AND p.id IN (SELECT m.part_id FROM manufacturer_parts m WHERE m.manufacturer = ?)");
            args.push(manufacturer.to_string().into());
        }
        if let Some(supplier) = query.supplier {
            sql.push_str(
                " AND p.id IN (SELECT m.part_id FROM supplier_offers o JOIN manufacturer_parts m \
                 ON m.id = o.manufacturer_part_id WHERE o.supplier = ?)",
            );
            args.push(supplier.to_string().into());
        }
        if let Some(after) = query.after {
            sql.push_str(" AND p.ord > ?");
            args.push(after.into());
        }
        sql.push_str(" ORDER BY p.ord");
        if let Some(limit) = query.limit {
            sql.push_str(" LIMIT ?");
            args.push((limit as i64).into());
        }
        let conn = self.reader();
        let mut stmt = conn.prepare(&sql)?;
        let rows = stmt.query_map(params_from_iter(args), |row| Ok((row.get(0)?, row.get(1)?)))?;
        rows.collect()
    }

    /// Replace everything in the file with `state` and `events`, in one
    /// transaction — the import from the file store.
    pub fn import(&self, state: &mut State, events: Vec<AuditEvent>) -> rusqlite::Result<()> {
        let after = small_value(state);
        let mut conn = self.writer();
        let tx = conn.transaction()?;
        let empty = Value::Object(Map::new());
        write_small(&tx, &empty, &after)?;
        for (position, part) in state.parts.iter().enumerate() {
            write_part(&tx, position, part, &state.categories)?;
        }
        for entry in state.workspace.iter() {
            write_entry(&tx, &entry.id, Some(entry))?;
        }
        for session in state.sessions.iter() {
            write_session(&tx, &session.token, Some(session))?;
        }
        insert_events(&tx, events)?;
        tx.commit()
    }

    /// Fold the write-ahead log into the main file, so the file alone is
    /// the whole store (before it is renamed or copied).
    pub fn checkpoint(&self) -> rusqlite::Result<()> {
        self.writer().execute_batch("PRAGMA wal_checkpoint(TRUNCATE);")
    }

    /// Row counts by table, for the migration report.
    pub fn counts(&self) -> rusqlite::Result<BTreeMap<String, i64>> {
        let conn = self.writer();
        let mut out = BTreeMap::new();
        for table in [
            "users",
            "sessions",
            "api_tokens",
            "part_types",
            "categories",
            "manufacturers",
            "suppliers",
            "change_orders",
            "workspace",
            "parts",
            "revisions",
            "uses",
            "manufacturer_parts",
            "supplier_offers",
            "part_tags",
            "changes",
            "audit",
            "preferences",
        ] {
            let n: i64 = conn.query_row(&format!("SELECT count(*) FROM {table}"), [], |row| row.get(0))?;
            out.insert(table.to_string(), n);
        }
        Ok(out)
    }
}

/// One row of a user's preferences, without its value.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct PreferenceRow {
    pub key: String,
    pub version: u64,
    pub size: u64,
    pub modified: u64,
    /// A deleted key, listed only to a client asking `since` a version before
    /// the deletion.
    pub deleted: bool,
}

impl Sql {
    /// A user's preferences version (0 before their first write) and their
    /// rows: every live key by name when `since` is 0, else every row — live
    /// or deleted — written after `since`, oldest first.
    pub fn preferences(&self, user_id: &str, since: u64) -> rusqlite::Result<(u64, Vec<PreferenceRow>)> {
        let conn = self.reader();
        let version: i64 = conn.query_row(
            "SELECT COALESCE(MAX(version), 0) FROM preferences WHERE user_id = ?1",
            [user_id],
            |row| row.get(0),
        )?;
        let sql = if since == 0 {
            "SELECT key, version, size, modified, value IS NULL FROM preferences
             WHERE user_id = ?1 AND value IS NOT NULL ORDER BY key"
        } else {
            "SELECT key, version, size, modified, value IS NULL FROM preferences
             WHERE user_id = ?1 AND version > ?2 ORDER BY version"
        };
        let mut stmt = conn.prepare(sql)?;
        let map = |row: &rusqlite::Row<'_>| {
            Ok(PreferenceRow {
                key: row.get(0)?,
                version: row.get::<_, i64>(1)? as u64,
                size: row.get::<_, i64>(2)? as u64,
                modified: row.get::<_, i64>(3)? as u64,
                deleted: row.get(4)?,
            })
        };
        let rows = if since == 0 {
            stmt.query_map([user_id], map)?.collect::<rusqlite::Result<_>>()?
        } else {
            stmt.query_map(params![user_id, since as i64], map)?.collect::<rusqlite::Result<_>>()?
        };
        Ok((version.max(0) as u64, rows))
    }

    /// One live preference's value and version.
    pub fn preference(&self, user_id: &str, key: &str) -> rusqlite::Result<Option<(String, u64)>> {
        self.reader()
            .query_row(
                "SELECT value, version FROM preferences WHERE user_id = ?1 AND key = ?2 AND value IS NOT NULL",
                params![user_id, key],
                |row| Ok((row.get::<_, String>(0)?, row.get::<_, i64>(1)? as u64)),
            )
            .optional()
    }

    /// Write (`Some`) or delete (`None`) one preference, and its audit event
    /// if it has one, in ONE transaction. Returns the user's version after:
    /// bumped by a write, and by a delete of a live key; a delete of a key
    /// that is not there changes nothing and logs nothing.
    pub fn set_preference(
        &self,
        user_id: &str,
        key: &str,
        value: Option<&str>,
        at: u64,
        event: Option<AuditEvent>,
    ) -> rusqlite::Result<u64> {
        let mut conn = self.writer();
        let tx = conn.transaction()?;
        let current: i64 = tx.query_row(
            "SELECT COALESCE(MAX(version), 0) FROM preferences WHERE user_id = ?1",
            [user_id],
            |row| row.get(0),
        )?;
        if value.is_none() {
            let live: bool = tx.query_row(
                "SELECT EXISTS(SELECT 1 FROM preferences WHERE user_id = ?1 AND key = ?2 AND value IS NOT NULL)",
                params![user_id, key],
                |row| row.get(0),
            )?;
            if !live {
                return Ok(current.max(0) as u64);
            }
        }
        let version = current + 1;
        tx.execute(
            "INSERT INTO preferences(user_id, key, version, modified, size, value) VALUES (?1, ?2, ?3, ?4, ?5, ?6)
             ON CONFLICT(user_id, key) DO UPDATE SET
                 version = excluded.version, modified = excluded.modified,
                 size = excluded.size, value = excluded.value",
            params![user_id, key, version, at as i64, value.map_or(0, |v| v.len() as i64), value],
        )?;
        insert_events(&tx, event.into_iter().collect())?;
        tx.commit()?;
        Ok(version as u64)
    }
}

/// What one committed change moved, for [`Sql::write`].
pub struct Change<'a> {
    /// The state without its parts, before and after ([`small_value`]).
    pub before: &'a Value,
    pub after: &'a Value,
    /// Each part that changed or was created, at its creation-order position.
    pub parts: Vec<(usize, &'a Part)>,
    /// The catalog after the change: a part's category path is indexed.
    pub categories: &'a [Category],
    /// Positions of parts whose search row must be rewritten although the
    /// part itself did not change: its category's path moved.
    pub reindex: Vec<usize>,
    pub all_parts: &'a Parts,
    /// Each workspace entry that changed: its id, and what it is now
    /// (`None`: removed).
    pub workspace: Vec<(String, Option<&'a WorkspaceEntry>)>,
    /// Each session that changed: its token, and what it is now.
    pub sessions: Vec<(String, Option<&'a Session>)>,
}

/// The indexed narrowing of a parts search ([`Sql::candidates`]).
#[derive(Default)]
pub struct Candidates<'a> {
    /// Search text, at least three characters.
    pub needle: Option<&'a str>,
    /// Category ids the part must be in (ignoring case).
    pub categories: Option<&'a [String]>,
    /// Only parts whose category names no category.
    pub uncategorized: bool,
    pub tag: Option<&'a str>,
    /// A manufacturer id.
    pub manufacturer: Option<&'a str>,
    /// A supplier id.
    pub supplier: Option<&'a str>,
    /// Creation-order position to continue after (a page cursor).
    pub after: Option<i64>,
    pub limit: Option<usize>,
}

/// The state as JSON without its parts, workspaces or sessions: every other
/// field, whatever it is. Those lists are swapped out for the length of the
/// serialization, so they are never serialized (they are what grows); each
/// has its own journal and table.
pub fn small_value(state: &mut State) -> Value {
    let parts = std::mem::take(&mut state.parts);
    let workspace = std::mem::take(&mut state.workspace);
    let sessions = std::mem::take(&mut state.sessions);
    let mut value = serde_json::to_value(&*state).unwrap_or(Value::Null);
    state.parts = parts;
    state.workspace = workspace;
    state.sessions = sessions;
    if let Some(object) = value.as_object_mut() {
        object.remove("parts");
        object.remove("workspace");
        object.remove("sessions");
    }
    value
}

fn json_error(error: serde_json::Error) -> rusqlite::Error {
    rusqlite::Error::ToSqlConversionFailure(Box::new(error))
}

fn text(value: &Value, field: &str) -> String {
    match value.get(field) {
        Some(Value::String(s)) => s.clone(),
        Some(Value::Null) | None => String::new(),
        Some(other) => other.to_string(),
    }
}

/// Write the state fields that differ between `before` and `after`.
fn write_small(tx: &Transaction<'_>, before: &Value, after: &Value) -> rusqlite::Result<()> {
    let empty = Map::new();
    let old = before.as_object().unwrap_or(&empty);
    let new = after.as_object().unwrap_or(&empty);
    let set_meta = |key: &str, value: &Value| -> rusqlite::Result<()> {
        tx.execute(
            "INSERT INTO meta(key, value) VALUES (?1, ?2) ON CONFLICT(key) DO UPDATE SET value = excluded.value",
            params![key, value.to_string()],
        )?;
        Ok(())
    };
    let keys: BTreeSet<&String> = old.keys().chain(new.keys()).collect();
    for key in keys {
        let was = old.get(key).unwrap_or(&Value::Null);
        let now = new.get(key).unwrap_or(&Value::Null);
        if was == now {
            continue;
        }
        match key.as_str() {
            "seq" | "changes_from" | "settings" => set_meta(key, now)?,
            "changes" => write_changes(tx, was, now)?,
            "parts" => {}
            name => match KEYED.iter().find(|k| k.name == name) {
                Some(keyed) => write_keyed(tx, keyed, was, now)?,
                None if !OWN_HOME.contains(&name) => set_meta(&format!("state.{name}"), now)?,
                None => {}
            },
        }
    }
    Ok(())
}

/// Row-diff a keyed list: delete what went, upsert what is new, changed or moved.
fn write_keyed(tx: &Transaction<'_>, keyed: &Keyed, before: &Value, after: &Value) -> rusqlite::Result<()> {
    let empty = Vec::new();
    let old: HashMap<String, (usize, &Value)> = before
        .as_array()
        .unwrap_or(&empty)
        .iter()
        .enumerate()
        .map(|(i, v)| (text(v, keyed.key), (i, v)))
        .collect();
    let new_rows = after.as_array().unwrap_or(&empty);
    let new_keys: BTreeSet<String> = new_rows.iter().map(|v| text(v, keyed.key)).collect();
    let pk = if keyed.key == "token" { "token" } else { "id" };
    for gone in old.keys().filter(|k| !new_keys.contains(*k)) {
        tx.execute(&format!("DELETE FROM {} WHERE {pk} = ?1", keyed.name), [gone])?;
    }
    let mut columns = vec![pk, "ord", "body"];
    columns.extend(keyed.columns.iter().map(|(c, _, _)| *c));
    let placeholders: Vec<String> = (1..=columns.len()).map(|i| format!("?{i}")).collect();
    let updates: Vec<String> = columns.iter().skip(1).map(|c| format!("{c} = excluded.{c}")).collect();
    let sql = format!(
        "INSERT INTO {}({}) VALUES ({}) ON CONFLICT({pk}) DO UPDATE SET {}",
        keyed.name,
        columns.join(", "),
        placeholders.join(", "),
        updates.join(", ")
    );
    let mut stmt = tx.prepare_cached(&sql)?;
    for (ord, row) in new_rows.iter().enumerate() {
        let key = text(row, keyed.key);
        if old.get(&key).is_some_and(|(i, v)| *i == ord && *v == row) {
            continue;
        }
        let mut values: Vec<rusqlite::types::Value> =
            vec![key.into(), (ord as i64).into(), row.to_string().into()];
        for (_, field, lower) in keyed.columns {
            let value = text(row, field);
            values.push(if *lower { value.to_ascii_lowercase() } else { value }.into());
        }
        stmt.execute(params_from_iter(values))?;
    }
    Ok(())
}

/// The change log: upsert keys that moved, delete keys that were dropped.
fn write_changes(tx: &Transaction<'_>, before: &Value, after: &Value) -> rusqlite::Result<()> {
    let empty = Vec::new();
    let pairs = |v: &Value| -> HashMap<String, i64> {
        v.as_array()
            .unwrap_or(&empty)
            .iter()
            .map(|c| (text(c, "key"), c.get("seq").and_then(Value::as_i64).unwrap_or(0)))
            .collect()
    };
    let old = pairs(before);
    let new = pairs(after);
    for key in old.keys().filter(|k| !new.contains_key(*k)) {
        tx.execute("DELETE FROM changes WHERE key = ?1", [key])?;
    }
    let mut stmt =
        tx.prepare_cached("INSERT INTO changes(key, seq) VALUES (?1, ?2) ON CONFLICT(key) DO UPDATE SET seq = excluded.seq")?;
    for (key, seq) in &new {
        if old.get(key) != Some(seq) {
            stmt.execute(params![key, seq])?;
        }
    }
    Ok(())
}

/// Write one workspace entry, or delete it (`None`). A new row goes after
/// every other (`ord`), so entries load in creation order; an updated row
/// keeps its place.
fn write_entry(tx: &Transaction<'_>, id: &str, entry: Option<&WorkspaceEntry>) -> rusqlite::Result<()> {
    match entry {
        None => {
            tx.execute("DELETE FROM workspace WHERE id = ?1", [id])?;
        }
        Some(entry) => {
            let body = serde_json::to_string(entry).map_err(json_error)?;
            tx.prepare_cached(
                "INSERT INTO workspace(id, ord, owner, parent, body) \
                 VALUES (?1, (SELECT coalesce(max(ord), 0) + 1 FROM workspace), ?2, ?3, ?4) \
                 ON CONFLICT(id) DO UPDATE SET owner = excluded.owner, parent = excluded.parent, body = excluded.body",
            )?
            .execute(params![id, entry.owner, entry.parent, body])?;
        }
    }
    Ok(())
}

/// Write one session, or delete it (`None`). `ord` is the sign-in time, so
/// sessions load in the order they were made without a scan for the largest
/// `ord` on every `last_seen` stamp.
fn write_session(tx: &Transaction<'_>, token: &str, session: Option<&Session>) -> rusqlite::Result<()> {
    match session {
        None => {
            tx.execute("DELETE FROM sessions WHERE token = ?1", [token])?;
        }
        Some(session) => {
            let body = serde_json::to_string(session).map_err(json_error)?;
            tx.prepare_cached(
                "INSERT INTO sessions(token, user_id, ord, body) VALUES (?1, ?2, ?3, ?4) \
                 ON CONFLICT(token) DO UPDATE SET user_id = excluded.user_id, body = excluded.body",
            )?
            .execute(params![token, session.user_id, session.created_at as i64, body])?;
        }
    }
    Ok(())
}

fn load_sessions(conn: &Connection) -> rusqlite::Result<crate::journal::Journaled<Session>> {
    let mut stmt = conn.prepare("SELECT body FROM sessions ORDER BY ord, rowid")?;
    let sessions: Vec<Session> = stmt
        .query_map([], |row| row.get::<_, String>(0))?
        .map(|body| body.and_then(|b| serde_json::from_str(&b).map_err(json_error)))
        .collect::<rusqlite::Result<_>>()?;
    Ok(crate::journal::Journaled::from_vec(sessions))
}

fn load_workspace(conn: &Connection) -> rusqlite::Result<crate::workspace::Workspace> {
    let mut stmt = conn.prepare("SELECT body FROM workspace ORDER BY ord")?;
    let entries: Vec<WorkspaceEntry> = stmt
        .query_map([], |row| row.get::<_, String>(0))?
        .map(|body| body.and_then(|b| serde_json::from_str(&b).map_err(json_error)))
        .collect::<rusqlite::Result<_>>()?;
    Ok(crate::workspace::Workspace::from_vec(entries))
}

/// A record's JSON with its child lists taken out: they have tables.
fn body_without(value: &impl serde::Serialize, children: &[&str]) -> rusqlite::Result<String> {
    let mut value = serde_json::to_value(value).map_err(json_error)?;
    if let Some(object) = value.as_object_mut() {
        for child in children {
            object.remove(*child);
        }
    }
    Ok(value.to_string())
}

/// Write one part and everything under it, and its search row.
fn write_part(tx: &Transaction<'_>, position: usize, part: &Part, categories: &[Category]) -> rusqlite::Result<()> {
    tx.prepare_cached(
        "INSERT INTO parts(ord, id, number, number_lc, part_type, category_lc, document_class, created_at, body)
         VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)
         ON CONFLICT(ord) DO UPDATE SET id = excluded.id, number = excluded.number, number_lc = excluded.number_lc,
           part_type = excluded.part_type, category_lc = excluded.category_lc,
           document_class = excluded.document_class, created_at = excluded.created_at, body = excluded.body",
    )?
    .execute(params![
        position as i64,
        part.id,
        part.number,
        part.number.to_ascii_lowercase(),
        part.part_type,
        part.category.to_ascii_lowercase(),
        part.document_class.as_str(),
        part.created_at as i64,
        body_without(part, &["revisions", "sourcing"])?,
    ])?;
    tx.prepare_cached("DELETE FROM revisions WHERE part_id = ?1")?.execute([&part.id])?;
    tx.prepare_cached("DELETE FROM manufacturer_parts WHERE part_id = ?1")?.execute([&part.id])?;
    tx.prepare_cached("DELETE FROM part_tags WHERE part_id = ?1")?.execute([&part.id])?;
    for (ord, revision) in part.revisions.iter().enumerate() {
        tx.prepare_cached(
            "INSERT INTO revisions(id, part_id, ord, label_lc, lifecycle, body) VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
        )?
        .execute(params![
            revision.id,
            part.id,
            ord as i64,
            revision.label.to_ascii_lowercase(),
            revision.lifecycle.as_str(),
            body_without(revision, &["uses"])?,
        ])?;
        for (line, use_) in revision.uses.iter().enumerate() {
            tx.prepare_cached(
                "INSERT INTO uses(revision_id, ord, child_part, child_revision, quantity, body, part_id) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            )?
            .execute(params![
                revision.id,
                line as i64,
                use_.part,
                use_.revision,
                use_.quantity,
                serde_json::to_string(use_).map_err(json_error)?,
                part.id,
            ])?;
        }
    }
    for (ord, mp) in part.sourcing.iter().enumerate() {
        tx.prepare_cached(
            "INSERT INTO manufacturer_parts(id, part_id, ord, manufacturer, mpn_lc, body) VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
        )?
        .execute(params![mp.id, part.id, ord as i64, mp.manufacturer, mp.mpn.to_ascii_lowercase(), body_without(mp, &["offers"])?])?;
        for (line, offer) in mp.offers.iter().enumerate() {
            tx.prepare_cached(
                "INSERT INTO supplier_offers(id, manufacturer_part_id, ord, supplier, spn_lc, body) VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            )?
            .execute(params![
                offer.id,
                mp.id,
                line as i64,
                offer.supplier,
                offer.spn.to_ascii_lowercase(),
                serde_json::to_string(offer).map_err(json_error)?,
            ])?;
        }
    }
    let tags: BTreeSet<String> = part.tags.iter().map(|t| t.to_ascii_lowercase()).collect();
    for tag in tags {
        tx.prepare_cached("INSERT INTO part_tags(part_id, tag_lc) VALUES (?1, ?2)")?.execute(params![part.id, tag])?;
    }
    index_part(tx, position, part, categories)
}

/// (Re)write a part's search row.
fn index_part(tx: &Transaction<'_>, position: usize, part: &Part, categories: &[Category]) -> rusqlite::Result<()> {
    tx.prepare_cached("DELETE FROM parts_fts WHERE rowid = ?1")?.execute([position as i64])?;
    let attributes: Vec<String> = part
        .attributes
        .values()
        .filter_map(|value| match value {
            Value::String(text) => Some(text.clone()),
            Value::Number(n) => Some(n.to_string()),
            _ => None,
        })
        .collect();
    let mpn: Vec<&str> = part.sourcing.iter().map(|mp| mp.mpn.as_str()).collect();
    let spn: Vec<&str> = part.sourcing.iter().flat_map(|mp| mp.offers.iter().map(|o| o.spn.as_str())).collect();
    tx.prepare_cached(
        "INSERT INTO parts_fts(rowid, number, name, description, category_id, category_path, tags, attributes, mpn, spn)
         VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
    )?
    .execute(params![
        position as i64,
        part.number,
        part.name,
        part.description,
        part.category,
        catalog::path_name(categories, &part.category),
        part.tags.join(SEP),
        attributes.join(SEP),
        mpn.join(SEP),
        spn.join(SEP),
    ])?;
    Ok(())
}

/// Every part with its revisions, uses and sourcing, in creation order.
fn load_parts(conn: &Connection) -> rusqlite::Result<Parts> {
    let rows = |sql: &str| -> rusqlite::Result<Vec<(String, String)>> {
        let mut stmt = conn.prepare(sql)?;
        let found = stmt.query_map([], |row| Ok((row.get(0)?, row.get(1)?)))?;
        found.collect()
    };
    let parse = |body: &str| -> rusqlite::Result<Value> { serde_json::from_str(body).map_err(json_error) };

    let mut offers: HashMap<String, Vec<SupplierOffer>> = HashMap::new();
    for (mp, body) in rows("SELECT manufacturer_part_id, body FROM supplier_offers ORDER BY manufacturer_part_id, ord")? {
        offers.entry(mp).or_default().push(serde_json::from_value(parse(&body)?).map_err(json_error)?);
    }
    let mut sourcing: HashMap<String, Vec<ManufacturerPart>> = HashMap::new();
    for (part, body) in rows("SELECT part_id, body FROM manufacturer_parts ORDER BY part_id, ord")? {
        let mut mp: ManufacturerPart = serde_json::from_value(parse(&body)?).map_err(json_error)?;
        mp.offers = offers.remove(&mp.id).unwrap_or_default();
        sourcing.entry(part).or_default().push(mp);
    }
    let mut uses: HashMap<String, Vec<Use>> = HashMap::new();
    for (revision, body) in rows("SELECT json_array(part_id, revision_id), body FROM uses ORDER BY part_id, revision_id, ord")? {
        uses.entry(revision).or_default().push(serde_json::from_value(parse(&body)?).map_err(json_error)?);
    }
    let mut revisions: HashMap<String, Vec<Revision>> = HashMap::new();
    for (part, body) in rows("SELECT part_id, body FROM revisions ORDER BY part_id, ord")? {
        let mut revision: Revision = serde_json::from_value(parse(&body)?).map_err(json_error)?;
        revision.uses = uses.remove(&serde_json::json!([part, revision.id]).to_string()).unwrap_or_default();
        revisions.entry(part).or_default().push(revision);
    }
    let mut parts = Vec::new();
    for (_, body) in rows("SELECT id, body FROM parts ORDER BY ord")? {
        let mut part: Part = serde_json::from_value(parse(&body)?).map_err(json_error)?;
        part.revisions = revisions.remove(&part.id).unwrap_or_default();
        part.sourcing = sourcing.remove(&part.id).unwrap_or_default();
        parts.push(part);
    }
    Ok(Parts::from_vec(parts))
}

/// Append audit events inside `tx`, numbering those that carry id 0 and
/// keeping the ids of imported ones.
fn insert_events(tx: &Transaction<'_>, mut events: Vec<AuditEvent>) -> rusqlite::Result<Vec<AuditEvent>> {
    let mut stmt = tx.prepare_cached(
        "INSERT INTO audit(id, at, actor_user_id, actor_username_lc, action, kind, entity_id, part_id, seq, body)
         VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
    )?;
    for event in &mut events {
        let id: Option<i64> = (event.id != 0).then_some(event.id as i64);
        let mut stored = event.clone();
        stored.id = 0;
        let part_id = if event.entity.kind == "part" && event.entity.part_id.is_empty() {
            event.entity.id.clone()
        } else {
            event.entity.part_id.clone()
        };
        stmt.execute(params![
            id,
            event.at as i64,
            event.actor.user_id,
            event.actor.username.to_ascii_lowercase(),
            event.action,
            event.entity.kind,
            event.entity.id,
            part_id,
            event.seq as i64,
            serde_json::to_string(&stored).map_err(json_error)?,
        ])?;
        event.id = tx.last_insert_rowid() as u64;
    }
    Ok(events)
}

impl AuditSink for Sql {
    fn append(&self, events: Vec<AuditEvent>) -> std::io::Result<Vec<AuditEvent>> {
        if events.is_empty() {
            return Ok(events);
        }
        let mut conn = self.writer();
        let run = || -> rusqlite::Result<Vec<AuditEvent>> {
            let tx = conn.transaction()?;
            let events = insert_events(&tx, events)?;
            tx.commit()?;
            Ok(events)
        };
        run().map_err(std::io::Error::other)
    }

    fn query(&self, filter: &AuditFilter) -> std::io::Result<Vec<AuditEvent>> {
        let mut sql = String::from("SELECT id, body FROM audit WHERE 1");
        let mut args: Vec<rusqlite::types::Value> = Vec::new();
        let mut eq = |sql: &mut String, column: &str, value: &str| {
            sql.push_str(&format!(" AND {column} = ?"));
            args.push(value.to_string().into());
        };
        if !filter.kind.is_empty() {
            eq(&mut sql, "kind", &filter.kind);
        }
        if !filter.entity.is_empty() {
            eq(&mut sql, "entity_id", &filter.entity);
        }
        if !filter.action.is_empty() {
            eq(&mut sql, "action", &filter.action);
        }
        if !filter.part.is_empty() {
            eq(&mut sql, "part_id", &filter.part);
        }
        if !filter.kinds.is_empty() {
            sql.push_str(&format!(" AND kind IN ({})", vec!["?"; filter.kinds.len()].join(",")));
            args.extend(filter.kinds.iter().map(|k| k.clone().into()));
        }
        if !filter.user.is_empty() {
            sql.push_str(" AND (actor_user_id = ? OR actor_username_lc = ?)");
            args.push(filter.user.clone().into());
            args.push(filter.user.to_ascii_lowercase().into());
        }
        if let Some(since) = filter.since {
            sql.push_str(" AND at >= ?");
            args.push((since as i64).into());
        }
        if let Some(until) = filter.until {
            sql.push_str(" AND at <= ?");
            args.push((until as i64).into());
        }
        if let Some(before) = filter.before {
            sql.push_str(" AND id < ?");
            args.push((before as i64).into());
        }
        sql.push_str(" ORDER BY id DESC");
        if let Some(limit) = filter.limit {
            sql.push_str(" LIMIT ?");
            args.push((limit as i64).into());
        }
        let run = || -> rusqlite::Result<Vec<AuditEvent>> {
            let conn = self.reader();
            let mut stmt = conn.prepare(&sql)?;
            let rows = stmt.query_map(params_from_iter(args), |row| Ok((row.get::<_, i64>(0)?, row.get::<_, String>(1)?)))?;
            let mut out = Vec::new();
            for row in rows {
                let (id, body) = row?;
                let mut event: AuditEvent = serde_json::from_str(&body).map_err(json_error)?;
                event.id = id as u64;
                out.push(event);
            }
            Ok(out)
        };
        run().map_err(std::io::Error::other)
    }
}

/// Whether `error` is a UNIQUE (or other) constraint the schema enforces —
/// a rule the store's own checks should have refused first.
pub fn is_constraint(error: &rusqlite::Error) -> bool {
    matches!(
        error,
        rusqlite::Error::SqliteFailure(e, _) if e.code == rusqlite::ErrorCode::ConstraintViolation
    )
}
