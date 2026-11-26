//! `brep-mcp`'s command line, parsed by hand. It replaced `clap` (derive),
//! whose output it reproduces: the same commands, aliases, flags, defaults,
//! `--help`/`-h`/`help <command>` text (verbatim, in [`text`]), `--version`,
//! error messages and exit codes (0 for help and version, 2 for a usage
//! error). `tests/cli.rs` replays the clap build's own record, case for case.
//!
//! The rules, as clap applied them to this command set:
//! * arguments are read in order and the first help, version or error wins;
//! * `--name value` and `--name=value` both work; an option given twice, a
//!   flag given a value, or a missing value is an error; a value that starts
//!   with `-` is read as a flag, so `--http -1` is refused (`--http=-1` is
//!   read and then range-checked);
//! * `--` makes every later argument a positional one;
//! * conflicting options and then missing required ones are checked last;
//! * an unknown argument or command gets a "similar" tip when one scores
//!   over 0.7 on Jaro similarity, as clap's did.

use std::ffi::{OsStr, OsString};
use std::path::PathBuf;

pub(crate) struct Cli {
    pub command: Command,
}

pub(crate) enum Command {
    Skills { command: SkillCommand },
    Serve {
        session_root: Option<PathBuf>,
        state_dir: Option<PathBuf>,
        instance: Option<String>,
        ephemeral: bool,
        event_journal_dir: Option<PathBuf>,
        http: Option<u16>,
    },
    Schema { feature_type: Option<String>, all: bool, write: Option<PathBuf> },
    Test {
        scripts: Vec<PathBuf>,
        backend: String,
        out: Option<PathBuf>,
        update_baselines: bool,
        session_root: Option<PathBuf>,
        order_check: bool,
        video_root: Option<PathBuf>,
    },
    /// Everything after `render-mesh` (a leading `--` dropped), unparsed.
    RenderMesh { args: Vec<String> },
    Typescript { command: TypescriptCommand },
    Shot { doc: PathBuf, png: PathBuf, view: String, size: String, ppp: f32, session_root: Option<PathBuf> },
}

pub(crate) enum TypescriptCommand {
    Status,
    Provision,
}

pub(crate) enum SkillCommand {
    Bundle { output: PathBuf },
    Install { bundle: PathBuf, sha256: String, directory: PathBuf },
    Inspect { directory: PathBuf },
    Rollback { directory: PathBuf, sha256: String },
}

const BIN: &str = "brep-mcp";

/// How an option's value is read.
#[derive(Clone, Copy, PartialEq)]
enum Kind {
    /// No value.
    Flag,
    /// Any text, empty included.
    Text,
    /// A path: empty is a missing value.
    Path,
    /// `u16`, read as an integer and range-checked.
    Port,
    /// `f32`.
    Float,
}

struct Opt {
    long: &'static str,
    /// The value's placeholder (`PORT`); empty for a flag.
    value: &'static str,
    kind: Kind,
    required: bool,
    conflicts: &'static [&'static str],
}

impl Opt {
    fn display(&self) -> String {
        if self.kind == Kind::Flag { format!("--{}", self.long) } else { format!("--{} <{}>", self.long, self.value) }
    }
}

struct Pos {
    name: &'static str,
    required: bool,
    multi: bool,
}

impl Pos {
    /// As a usage line shows it: `<DOC>`, `<SCRIPTS>...`, `[SCRIPTS]...`.
    fn display(&self, used: bool) -> String {
        let dots = if self.multi { "..." } else { "" };
        if self.required || used { format!("<{}>{dots}", self.name) } else { format!("[{}]{dots}", self.name) }
    }
}

/// A command that takes arguments (not further subcommands).
struct Leaf {
    /// The words after `brep-mcp`: `serve`, `skills bundle`.
    path: &'static str,
    opts: &'static [Opt],
    pos: &'static [Pos],
    help: &'static str,
    /// The usage line of the help text (after `Usage: `).
    usage: &'static str,
}

const fn opt(long: &'static str, value: &'static str, kind: Kind) -> Opt {
    Opt { long, value, kind, required: false, conflicts: &[] }
}
const fn req(long: &'static str, value: &'static str, kind: Kind) -> Opt {
    Opt { long, value, kind, required: true, conflicts: &[] }
}
const HELP_FLAG: Opt = opt("help", "", Kind::Flag);
const VERSION_FLAG: Opt = opt("version", "", Kind::Flag);

const SERVE: Leaf = Leaf {
    path: "serve",
    opts: &[
        opt("session-root", "SESSION_ROOT", Kind::Path),
        Opt { conflicts: &["instance", "ephemeral"], ..opt("state-dir", "STATE_DIR", Kind::Path) },
        Opt { conflicts: &["state-dir", "ephemeral"], ..opt("instance", "INSTANCE", Kind::Text) },
        Opt { conflicts: &["state-dir", "instance"], ..opt("ephemeral", "", Kind::Flag) },
        opt("event-journal-dir", "EVENT_JOURNAL_DIR", Kind::Path),
        opt("http", "PORT", Kind::Port),
        HELP_FLAG,
    ],
    pos: &[],
    help: text::SERVE,
    usage: "brep-mcp serve [OPTIONS]",
};
const SCHEMA: Leaf = Leaf {
    path: "schema",
    opts: &[opt("type", "FEATURE_TYPE", Kind::Text), opt("all", "", Kind::Flag), opt("write", "WRITE", Kind::Path), HELP_FLAG],
    pos: &[],
    help: text::SCHEMA,
    usage: "brep-mcp schema [OPTIONS]",
};
const TEST: Leaf = Leaf {
    path: "test",
    opts: &[
        opt("backend", "BACKEND", Kind::Text),
        opt("out", "OUT", Kind::Path),
        opt("update-baselines", "", Kind::Flag),
        opt("session-root", "SESSION_ROOT", Kind::Path),
        opt("order-check", "", Kind::Flag),
        opt("video-root", "DIR", Kind::Path),
        HELP_FLAG,
    ],
    pos: &[Pos { name: "SCRIPTS", required: false, multi: true }],
    help: text::TEST,
    usage: "brep-mcp test [OPTIONS] [SCRIPTS]...",
};
const SHOT: Leaf = Leaf {
    path: "shot",
    opts: &[
        opt("view", "VIEW", Kind::Text),
        opt("size", "SIZE", Kind::Text),
        opt("ppp", "PPP", Kind::Float),
        opt("session-root", "SESSION_ROOT", Kind::Path),
        HELP_FLAG,
    ],
    pos: &[Pos { name: "DOC", required: true, multi: false }, Pos { name: "PNG", required: true, multi: false }],
    help: text::SHOT,
    usage: "brep-mcp shot [OPTIONS] <DOC> <PNG>",
};
const RENDER_MESH: Leaf = Leaf {
    path: "render-mesh",
    opts: &[],
    pos: &[Pos { name: "ARGS", required: false, multi: true }],
    help: text::RENDER_MESH,
    usage: "brep-mcp render-mesh [ARGS]...",
};
const BUNDLE: Leaf = Leaf {
    path: "skills bundle",
    opts: &[req("output", "OUTPUT", Kind::Path), HELP_FLAG],
    pos: &[],
    help: text::SKILLS_BUNDLE,
    usage: "brep-mcp skills bundle --output <OUTPUT>",
};
const INSTALL: Leaf = Leaf {
    path: "skills install",
    opts: &[req("bundle", "BUNDLE", Kind::Path), req("sha256", "SHA256", Kind::Text), req("directory", "DIRECTORY", Kind::Path), HELP_FLAG],
    pos: &[],
    help: text::SKILLS_INSTALL,
    usage: "brep-mcp skills install --bundle <BUNDLE> --sha256 <SHA256> --directory <DIRECTORY>",
};
const INSPECT: Leaf = Leaf {
    path: "skills inspect",
    opts: &[req("directory", "DIRECTORY", Kind::Path), HELP_FLAG],
    pos: &[],
    help: text::SKILLS_INSPECT,
    usage: "brep-mcp skills inspect --directory <DIRECTORY>",
};
const ROLLBACK: Leaf = Leaf {
    path: "skills rollback",
    opts: &[req("directory", "DIRECTORY", Kind::Path), req("sha256", "SHA256", Kind::Text), HELP_FLAG],
    pos: &[],
    help: text::SKILLS_ROLLBACK,
    usage: "brep-mcp skills rollback --directory <DIRECTORY> --sha256 <SHA256>",
};
const STATUS: Leaf = Leaf {
    path: "typescript status",
    opts: &[HELP_FLAG],
    pos: &[],
    help: text::TYPESCRIPT_STATUS,
    usage: "brep-mcp typescript status",
};
const PROVISION: Leaf = Leaf {
    path: "typescript provision",
    opts: &[HELP_FLAG],
    pos: &[],
    help: text::TYPESCRIPT_PROVISION,
    usage: "brep-mcp typescript provision",
};

/// A command made of subcommands: the top level, `skills`, `typescript`.
struct Group {
    /// The words after `brep-mcp`; empty at the top.
    path: &'static str,
    /// Name, aliases, node, in help order (`help` comes last, implicitly).
    subs: &'static [(&'static str, &'static [&'static str], Node)],
    help: &'static str,
    /// `--version` / `-V` are the top level's only.
    version: bool,
}

#[derive(Clone, Copy)]
enum Node {
    Group(&'static Group),
    Leaf(&'static Leaf),
}

static TOP: Group = Group {
    path: "",
    subs: &[
        ("skills", &[], Node::Group(&SKILLS)),
        ("serve", &[], Node::Leaf(&SERVE)),
        ("schema", &[], Node::Leaf(&SCHEMA)),
        ("test", &[], Node::Leaf(&TEST)),
        ("render-mesh", &[], Node::Leaf(&RENDER_MESH)),
        ("typescript", &[], Node::Group(&TYPESCRIPT)),
        ("shot", &[], Node::Leaf(&SHOT)),
    ],
    help: text::TOP,
    version: true,
};
static SKILLS: Group = Group {
    path: "skills",
    subs: &[
        ("bundle", &["fetch"], Node::Leaf(&BUNDLE)),
        ("install", &["update"], Node::Leaf(&INSTALL)),
        ("inspect", &[], Node::Leaf(&INSPECT)),
        ("rollback", &[], Node::Leaf(&ROLLBACK)),
    ],
    help: text::SKILLS,
    version: false,
};
static TYPESCRIPT: Group = Group {
    path: "typescript",
    subs: &[("status", &[], Node::Leaf(&STATUS)), ("provision", &[], Node::Leaf(&PROVISION))],
    help: text::TYPESCRIPT,
    version: false,
};

impl Group {
    fn usage(&self) -> String {
        if self.path.is_empty() { format!("{BIN} <COMMAND>") } else { format!("{BIN} {} <COMMAND>", self.path) }
    }

    fn find(&self, name: &str) -> Option<Node> {
        self.subs.iter().find(|(n, aliases, _)| *n == name || aliases.contains(&name)).map(|(_, _, node)| *node)
    }

    /// Every name a subcommand answers to, `help` last, as clap listed them.
    fn names(&self) -> Vec<&'static str> {
        let mut names: Vec<&'static str> = Vec::new();
        for (name, aliases, _) in self.subs {
            names.push(name);
            names.extend(aliases.iter().copied());
        }
        names.push("help");
        names
    }

    /// The text `help help` prints at this level.
    fn help_of_help(&self) -> String {
        let path = if self.path.is_empty() { String::new() } else { format!("{} ", self.path) };
        format!(
            "Print this message or the help of the given subcommand(s)\n\nUsage: {BIN} {path}help [COMMAND]...\n\nArguments:\n  [COMMAND]...  Print help for the subcommand(s)\n"
        )
    }
}

/// Parse the process's arguments, printing help or version, or a usage
/// error, and exiting where clap did.
pub(crate) fn parse() -> Cli {
    let raw: Vec<OsString> = std::env::args_os().skip(1).collect();
    match parse_from(&raw) {
        Ok(cli) => cli,
        Err(Exit { code, stdout, text }) => {
            use std::io::Write as _;
            if stdout {
                print!("{text}");
                let _ = std::io::stdout().flush();
            } else {
                eprint!("{text}");
            }
            std::process::exit(code);
        }
    }
}

/// Why parsing stopped short of a command: help or version (exit 0, on
/// stdout) or a usage error (exit 2, on stderr).
#[derive(Debug)]
pub(crate) struct Exit {
    pub code: i32,
    pub stdout: bool,
    pub text: String,
}

fn shown(text: impl Into<String>) -> Exit {
    Exit { code: 0, stdout: true, text: text.into() }
}

fn refused(text: impl Into<String>) -> Exit {
    Exit { code: 2, stdout: false, text: text.into() }
}

const MORE: &str = "For more information, try '--help'.\n";

/// `error: …`, an optional tip, an optional usage line, the closing hint.
fn error(message: &str, tip: Option<String>, usage: Option<String>) -> Exit {
    let mut text = format!("error: {message}\n\n");
    if let Some(tip) = tip {
        text.push_str(&format!("  tip: {tip}\n\n"));
    }
    if let Some(usage) = usage {
        text.push_str(&format!("Usage: {usage}\n\n"));
    }
    text.push_str(MORE);
    refused(text)
}

/// Parse `raw` (the arguments after the program name). Matching reads each
/// argument as text (lossily, for messages); a path value or positional keeps
/// its exact OS string, as clap's `PathBuf` arguments did, and a text value
/// that is not UTF-8 is refused as clap refused it.
pub(crate) fn parse_from(raw: &[OsString]) -> Result<Cli, Exit> {
    let args: Vec<String> = raw.iter().map(|a| a.to_string_lossy().into_owned()).collect();
    Ok(Cli { command: group(&TOP, &args, raw)? })
}

/// The part of `raw` after its first `skip` bytes (`--name=` before a value).
fn raw_suffix(raw: &OsStr, skip: usize) -> OsString {
    #[cfg(unix)]
    {
        use std::os::unix::ffi::OsStrExt as _;
        OsStr::from_bytes(&raw.as_bytes()[skip..]).to_os_string()
    }
    #[cfg(not(unix))]
    {
        // Elsewhere an OS string is not bytes; the lossy text is the value
        // (it differs only for unpaired UTF-16 surrogates).
        OsString::from(&raw.to_string_lossy()[skip..])
    }
}

fn invalid_utf8(usage: &str) -> Exit {
    error("invalid UTF-8 was detected in one or more arguments", None, Some(usage.to_string()))
}

fn group(g: &'static Group, args: &[String], raw: &[OsString]) -> Result<Command, Exit> {
    // The first word decides: a flag, a command, or a refusal.
    let i = 0;
    {
        let Some(arg) = args.first().map(String::as_str) else {
            // No subcommand: the help, as a refusal.
            return Err(refused(g.help));
        };
        if arg == "--" {
            // After `--` the next word is not a command name.
            return match args.get(i + 1) {
                None => Err(refused(g.help)),
                Some(next) if g.find(next).is_some() || next == "help" => Err(error(
                    &format!("unexpected argument '{next}' found"),
                    Some(format!("subcommand '{next}' exists; to use it, remove the '--' before it")),
                    Some(g.usage()),
                )),
                Some(next) => Err(unrecognized(g, next)),
            };
        }
        if let Some(long) = arg.strip_prefix("--") {
            let (name, value) = split_value(long);
            let flags: &[&Opt] = if g.version { &[&HELP_FLAG, &VERSION_FLAG] } else { &[&HELP_FLAG] };
            match flags.iter().find(|f| f.long == name) {
                Some(flag) => {
                    if let Some(value) = value {
                        let usage = format!("{BIN} {}{} <COMMAND>", prefix(g.path), flag.display());
                        return Err(error(&format!("unexpected value '{value}' for '{}' found; no more were expected", flag.display()), None, Some(usage)));
                    }
                    return Err(if flag.long == "help" { shown(g.help) } else { version() });
                }
                None => {
                    let longs: Vec<&str> = flags.iter().map(|f| f.long).collect();
                    let message = format!("unexpected argument '--{name}' found");
                    return Err(match similar(name, &longs).pop() {
                        Some(best) => error(
                            &message,
                            Some(format!("a similar argument exists: '--{best}'")),
                            Some(format!("{BIN} {}--{best} <COMMAND>", prefix(g.path))),
                        ),
                        None => error(&message, None, Some(g.usage())),
                    });
                }
            }
        }
        if arg.len() > 1 && arg.starts_with('-') {
            // A cluster acts on its first letter (`-hV` is help, `-Vh` version).
            let c = arg[1..].chars().next().expect("a character after '-'");
            return Err(match c {
                'h' => shown(g.help),
                'V' if g.version => version(),
                _ => error(&format!("unexpected argument '-{c}' found"), None, Some(g.usage())),
            });
        }
        if arg == "help" {
            return Err(help_command(g, &args[i + 1..]));
        }
        match g.find(arg) {
            Some(Node::Group(sub)) => group(sub, &args[i + 1..], &raw[i + 1..]),
            Some(Node::Leaf(leaf)) => command(leaf, &args[i + 1..], &raw[i + 1..]),
            None => Err(unrecognized(g, arg)),
        }
    }
}

fn prefix(path: &str) -> String {
    if path.is_empty() { String::new() } else { format!("{path} ") }
}

fn version() -> Exit {
    shown(format!("{BIN} {}\n", env!("CARGO_PKG_VERSION")))
}

fn unrecognized(g: &Group, name: &str) -> Exit {
    let close = similar(name, &g.names());
    let tip = match close.len() {
        0 => None,
        1 => Some(format!("a similar subcommand exists: '{}'", close[0])),
        _ => Some(format!("some similar subcommands exist: {}", close.iter().map(|n| format!("'{n}'")).collect::<Vec<_>>().join(", "))),
    };
    error(&format!("unrecognized subcommand '{name}'"), tip, Some(g.usage()))
}

/// `help [COMMAND]...` at group `g`: the named command's help on stdout.
fn help_command(g: &'static Group, words: &[String]) -> Exit {
    enum At {
        Group(&'static Group),
        Leaf(&'static Leaf),
        HelpOf(&'static Group),
    }
    let mut at = At::Group(g);
    for word in words {
        at = match at {
            At::Group(group) if word == "help" => At::HelpOf(group),
            At::Group(group) => match group.find(word) {
                Some(Node::Group(sub)) => At::Group(sub),
                Some(Node::Leaf(leaf)) => At::Leaf(leaf),
                None => return error(&format!("unrecognized subcommand '{word}'"), None, Some(group.usage())),
            },
            At::Leaf(leaf) => {
                let refusal = error(&format!("unrecognized subcommand '{word}'"), None, Some(leaf.usage.to_string()));
                // A command without its own `--help` (render-mesh) gets no
                // "try '--help'" line.
                return if leaf.opts.iter().any(|o| o.long == "help") { refusal } else { refused(refusal.text.replace(&format!("\n{MORE}"), "")) };
            }
            At::HelpOf(group) => {
                let usage = format!("{BIN} {}help [COMMAND]...", prefix(group.path));
                return error(&format!("unrecognized subcommand '{word}'"), None, Some(usage));
            }
        };
    }
    match at {
        At::Group(group) => shown(group.help),
        At::Leaf(leaf) => shown(leaf.help),
        At::HelpOf(group) => shown(group.help_of_help()),
    }
}

fn split_value(long: &str) -> (&str, Option<&str>) {
    match long.split_once('=') {
        Some((name, value)) => (name, Some(value)),
        None => (long, None),
    }
}

/// What a leaf's arguments were, in the order given.
#[derive(Default)]
struct Matches {
    /// Option index and its value ("" for a flag).
    opts: Vec<(usize, String)>,
    /// Each option's value as given, in the same order as `opts`.
    raw_opts: Vec<OsString>,
    positionals: Vec<String>,
    /// Each positional as given.
    raw_positionals: Vec<OsString>,
}

impl Matches {
    fn get(&self, leaf: &Leaf, long: &str) -> Option<String> {
        let index = leaf.opts.iter().position(|o| o.long == long).expect("a declared option");
        self.opts.iter().find(|(i, _)| *i == index).map(|(_, v)| v.clone())
    }

    fn has(&self, leaf: &Leaf, long: &str) -> bool {
        self.get(leaf, long).is_some()
    }

    /// A path option's value exactly as given (not UTF-8 included).
    fn path(&self, leaf: &Leaf, long: &str) -> Option<PathBuf> {
        let index = leaf.opts.iter().position(|o| o.long == long).expect("a declared option");
        self.opts.iter().position(|(i, _)| *i == index).map(|k| PathBuf::from(&self.raw_opts[k]))
    }
}

/// The usage line clap built from what was given: the command's required
/// arguments and the `used` options, options first, then the positionals
/// (`<X>` when required or given, `[X]` otherwise).
fn smart_usage(leaf: &Leaf, used: &[usize], positionals_given: usize) -> String {
    let mut order: Vec<usize> = leaf.opts.iter().enumerate().filter(|(_, o)| o.required).map(|(i, _)| i).collect();
    for &i in used {
        if !order.contains(&i) {
            order.push(i);
        }
    }
    let mut words: Vec<String> = vec![format!("{BIN} {}", leaf.path)];
    words.extend(order.iter().map(|&i| leaf.opts[i].display()));
    for (k, pos) in leaf.pos.iter().enumerate() {
        words.push(pos.display(positionals_given > k));
    }
    words.join(" ")
}

fn command(leaf: &'static Leaf, args: &[String], raw: &[OsString]) -> Result<Command, Exit> {
    if leaf.path == "render-mesh" {
        let skip = usize::from(args.first().map(String::as_str) == Some("--"));
        // Its arguments are text (clap's `Vec<String>`).
        if raw[skip..].iter().any(|a| a.to_str().is_none()) {
            let refusal = invalid_utf8(leaf.usage);
            return Err(refused(refusal.text.replace(&format!("\n{MORE}"), "")));
        }
        return Ok(Command::RenderMesh { args: args[skip..].to_vec() });
    }
    let m = read(leaf, args, raw)?;
    let s = |long: &str| m.get(leaf, long);
    let p = |long: &str| m.path(leaf, long);
    let f = |long: &str| m.has(leaf, long);
    let pos = |k: usize| PathBuf::from(&m.raw_positionals[k]);
    Ok(match leaf.path {
        "serve" => Command::Serve {
            session_root: p("session-root"),
            state_dir: p("state-dir"),
            instance: s("instance"),
            ephemeral: f("ephemeral"),
            event_journal_dir: p("event-journal-dir"),
            http: s("http").map(|v| v.parse::<i64>().expect("checked") as u16),
        },
        "schema" => Command::Schema { feature_type: s("type"), all: f("all"), write: p("write") },
        "test" => Command::Test {
            scripts: m.raw_positionals.iter().map(PathBuf::from).collect(),
            backend: s("backend").unwrap_or_else(|| "headless".into()),
            out: p("out"),
            update_baselines: f("update-baselines"),
            session_root: p("session-root"),
            order_check: f("order-check"),
            video_root: p("video-root"),
        },
        "shot" => Command::Shot {
            doc: pos(0),
            png: pos(1),
            view: s("view").unwrap_or_else(|| "ISO".into()),
            size: s("size").unwrap_or_else(|| "1280x800".into()),
            ppp: s("ppp").map(|v| v.parse::<f32>().expect("checked")).unwrap_or(1.0),
            session_root: p("session-root"),
        },
        "skills bundle" => Command::Skills { command: SkillCommand::Bundle { output: p("output").expect("required") } },
        "skills install" => Command::Skills {
            command: SkillCommand::Install {
                bundle: p("bundle").expect("required"),
                sha256: s("sha256").expect("required"),
                directory: p("directory").expect("required"),
            },
        },
        "skills inspect" => Command::Skills { command: SkillCommand::Inspect { directory: p("directory").expect("required") } },
        "skills rollback" => Command::Skills {
            command: SkillCommand::Rollback { directory: p("directory").expect("required"), sha256: s("sha256").expect("required") },
        },
        "typescript status" => Command::Typescript { command: TypescriptCommand::Status },
        "typescript provision" => Command::Typescript { command: TypescriptCommand::Provision },
        other => unreachable!("no command {other}"),
    })
}

/// Read a leaf's arguments, clap's way.
fn read(leaf: &'static Leaf, args: &[String], raw: &[OsString]) -> Result<Matches, Exit> {
    let mut m = Matches::default();
    let capacity = if leaf.pos.iter().any(|p| p.multi) { usize::MAX } else { leaf.pos.len() };
    let has_positionals = !leaf.pos.is_empty();
    let pass_tip = |arg: &str| has_positionals.then(|| format!("to pass '{arg}' as a value, use '-- {arg}'"));
    let used_opts = |m: &Matches| m.opts.iter().map(|(i, _)| *i).collect::<Vec<_>>();
    // An unknown `--name`: a similar option, or the pass-as-value tip.
    let unknown_long = |m: &Matches, name: &str| {
        let message = format!("unexpected argument '--{name}' found");
        let longs: Vec<&str> = leaf.opts.iter().map(|o| o.long).collect();
        match similar(name, &longs).pop() {
            Some(best) => {
                let mut used = used_opts(m);
                used.push(leaf.opts.iter().position(|o| o.long == best).expect("a declared option"));
                error(&message, Some(format!("a similar argument exists: '--{best}'")), Some(smart_usage(leaf, &used, m.positionals.len())))
            }
            None => {
                let usage = if m.opts.is_empty() && m.positionals.is_empty() {
                    leaf.usage.to_string()
                } else {
                    smart_usage(leaf, &used_opts(m), m.positionals.len())
                };
                error(&message, pass_tip(&format!("--{name}")), Some(usage))
            }
        }
    };
    let unknown_short = |c: char| error(&format!("unexpected argument '-{c}' found"), pass_tip(&format!("-{c}")), Some(leaf.usage.to_string()));
    let mut i = 0;
    let mut escaped = false;
    while i < args.len() {
        let arg = args[i].as_str();
        i += 1;
        if !escaped && arg == "--" {
            escaped = true;
            continue;
        }
        if !escaped && arg.starts_with("--") {
            let (name, inline) = split_value(&arg[2..]);
            let Some(index) = leaf.opts.iter().position(|o| o.long == name) else {
                return Err(unknown_long(&m, name));
            };
            let o = &leaf.opts[index];
            if o.long == "help" {
                if let Some(value) = inline {
                    return Err(error(
                        &format!("unexpected value '{value}' for '--help' found; no more were expected"),
                        None,
                        Some(smart_usage(leaf, &[index], m.positionals.len())),
                    ));
                }
                return Err(shown(leaf.help));
            }
            if m.opts.iter().any(|(j, _)| *j == index) {
                return Err(error(&format!("the argument '{}' cannot be used multiple times", o.display()), None, Some(leaf.usage.to_string())));
            }
            if o.kind == Kind::Flag {
                if let Some(value) = inline {
                    return Err(error(
                        &format!("unexpected value '{value}' for '{}' found; no more were expected", o.display()),
                        None,
                        Some(smart_usage(leaf, &[index], m.positionals.len())),
                    ));
                }
                m.opts.push((index, String::new()));
                m.raw_opts.push(OsString::new());
                continue;
            }
            let missing = || error(&format!("a value is required for '{}' but none was supplied", o.display()), None, None);
            let (value, raw_value) = match inline {
                Some(value) => (value.to_string(), raw_suffix(&raw[i - 1], 2 + name.len() + 1)),
                None => match args.get(i) {
                    None => return Err(missing()),
                    Some(next) if next.len() > 1 && next.starts_with('-') => {
                        // A flag where the value should be: refused as that
                        // flag would be when unknown, else as a missing value.
                        if let Some(long) = next.strip_prefix("--") {
                            let (name, _) = split_value(long);
                            if !long.is_empty() && !leaf.opts.iter().any(|o| o.long == name) {
                                return Err(unknown_long(&m, name));
                            }
                        } else if let Some(c) = next[1..].chars().next().filter(|c| *c != 'h') {
                            return Err(unknown_short(c));
                        }
                        return Err(missing());
                    }
                    Some(next) => {
                        i += 1;
                        (next.clone(), raw[i - 1].clone())
                    }
                },
            };
            // Only a path may be other than UTF-8.
            if o.kind != Kind::Path && raw_value.to_str().is_none() {
                return Err(invalid_utf8(leaf.usage));
            }
            check_value(o, &value)?;
            m.opts.push((index, value));
            m.raw_opts.push(raw_value);
            continue;
        }
        if !escaped && arg.len() > 1 && arg.starts_with('-') {
            let c = arg[1..].chars().next().expect("a character after '-'");
            return Err(if c == 'h' { shown(leaf.help) } else { unknown_short(c) });
        }
        if m.positionals.len() >= capacity {
            return Err(error(&format!("unexpected argument '{arg}' found"), None, Some(leaf.usage.to_string())));
        }
        m.positionals.push(arg.to_string());
        m.raw_positionals.push(raw[i - 1].clone());
    }
    // Conflicts: the first given option that conflicts with others given.
    for (index, _) in &m.opts {
        let o = &leaf.opts[*index];
        let clashing: Vec<&Opt> = m
            .opts
            .iter()
            .map(|(j, _)| &leaf.opts[*j])
            .filter(|other| o.conflicts.contains(&other.long))
            .collect();
        if !clashing.is_empty() {
            let message = if clashing.len() == 1 {
                format!("the argument '{}' cannot be used with '{}'", o.display(), clashing[0].display())
            } else {
                let list: String = clashing.iter().map(|c| format!("\n  {}", c.display())).collect();
                format!("the argument '{}' cannot be used with:{list}", o.display())
            };
            return Err(error(&message, None, Some(smart_usage(leaf, &[*index], m.positionals.len()))));
        }
    }
    // Missing required arguments, positionals then options.
    let mut missing: Vec<String> = leaf.pos.iter().skip(m.positionals.len()).filter(|p| p.required).map(|p| p.display(true)).collect();
    missing.extend(
        leaf.opts
            .iter()
            .enumerate()
            .filter(|(i, o)| o.required && !m.opts.iter().any(|(j, _)| j == i))
            .map(|(_, o)| o.display()),
    );
    if !missing.is_empty() {
        let list: String = missing.iter().map(|a| format!("\n  {a}")).collect();
        return Err(error(
            &format!("the following required arguments were not provided:{list}"),
            None,
            Some(smart_usage(leaf, &used_opts(&m), m.positionals.len())),
        ));
    }
    Ok(m)
}

/// Check an option's value as clap's value parsers did.
fn check_value(o: &Opt, value: &str) -> Result<(), Exit> {
    let invalid = |why: String| error(&format!("invalid value '{value}' for '{}': {why}", o.display()), None, None);
    match o.kind {
        Kind::Flag | Kind::Text => Ok(()),
        Kind::Path if value.is_empty() => Err(error(&format!("a value is required for '{}' but none was supplied", o.display()), None, None)),
        Kind::Path => Ok(()),
        // clap read a `u16` as an `i64`, then checked the range.
        Kind::Port => match value.parse::<i64>() {
            Err(e) => Err(invalid(e.to_string())),
            Ok(n) if !(0..=65535).contains(&n) => Err(invalid(format!("{n} is not in 0..=65535"))),
            Ok(_) => Ok(()),
        },
        Kind::Float => value.parse::<f32>().map(|_| ()).map_err(|e| invalid(e.to_string())),
    }
}

/// The candidates scoring over 0.7 on Jaro similarity to `word`, weakest
/// first (clap's `did_you_mean`).
fn similar<'a>(word: &str, candidates: &[&'a str]) -> Vec<&'a str> {
    let mut scored: Vec<(f64, &'a str)> = candidates.iter().map(|c| (jaro(word, c), *c)).filter(|(score, _)| *score > 0.7).collect();
    scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    scored.into_iter().map(|(_, c)| c).collect()
}

/// Jaro similarity over characters (as `strsim::jaro`).
fn jaro(a: &str, b: &str) -> f64 {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    if a.is_empty() && b.is_empty() {
        return 1.0;
    }
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let range = (a.len().max(b.len()) / 2).saturating_sub(1);
    let mut a_flags = vec![false; a.len()];
    let mut b_flags = vec![false; b.len()];
    let mut matches = 0usize;
    for (i, ca) in a.iter().enumerate() {
        let low = i.saturating_sub(range);
        let high = b.len().min(i + range + 1);
        for j in low..high {
            if !b_flags[j] && b[j] == *ca {
                a_flags[i] = true;
                b_flags[j] = true;
                matches += 1;
                break;
            }
        }
    }
    if matches == 0 {
        return 0.0;
    }
    let mut transpositions = 0usize;
    let mut j = 0;
    for (i, ca) in a.iter().enumerate() {
        if a_flags[i] {
            while !b_flags[j] {
                j += 1;
            }
            if b[j] != *ca {
                transpositions += 1;
            }
            j += 1;
        }
    }
    let m = matches as f64;
    (m / a.len() as f64 + m / b.len() as f64 + (m - (transpositions / 2) as f64) / m) / 3.0
}

/// The help texts, verbatim from the clap-built binary (0.24.0).
mod text {
pub(crate) const TOP: &str = concat!(
    "BREP MCP automation server\n",
    "\n",
    "Usage: brep-mcp <COMMAND>\n",
    "\n",
    "Commands:\n",
    "  skills       Export and manage the reviewed Markdown skill bundle offline\n",
    "  serve        Serve MCP over stdio, or over streamable HTTP with `--http`\n",
    "  schema       Print the kernel feature catalogue rendered as JSON Schema\n",
    "  test         Run test-mcp scripts through a headless session each\n",
    "  render-mesh  Rasterise a triangle mesh `{positions, indices}` to a PNG on the CPU and report its silhouette — the native STEP-validation renderer. No host, no GPU, no window; `--help` prints the full option list. The same command is `BREP_mcp_core`'s `render_mesh` example, which step-validation calls directly so it need not build this binary's app host\n",
    "  typescript   The TypeScript toolchain `run_script` checks TypeScript with\n",
    "  shot         Render a document to a PNG through the headless host\n",
    "  help         Print this message or the help of the given subcommand(s)\n",
    "\n",
    "Options:\n",
    "  -h, --help     Print help\n",
    "  -V, --version  Print version\n",
);
pub(crate) const SKILLS: &str = concat!(
    "Export and manage the reviewed Markdown skill bundle offline\n",
    "\n",
    "Usage: brep-mcp skills <COMMAND>\n",
    "\n",
    "Commands:\n",
    "  bundle    Export this binary's reviewed bundle; refuses to replace an existing file [alias: fetch]\n",
    "  install   Install or update the managed selection from this binary's exact bundle [alias: update]\n",
    "  inspect   Verify the installed selection and report its version directory\n",
    "  rollback  Restore the previous installed version after verifying its reviewed pin\n",
    "  help      Print this message or the help of the given subcommand(s)\n",
    "\n",
    "Options:\n",
    "  -h, --help  Print help\n",
);
pub(crate) const SERVE: &str = concat!(
    "Serve MCP over stdio, or over streamable HTTP with `--http`\n",
    "\n",
    "Usage: brep-mcp serve [OPTIONS]\n",
    "\n",
    "Options:\n",
    "      --session-root <SESSION_ROOT>\n",
    "          Where sessions keep their store, shots and logs (default: the OS temp dir)\n",
    "      --state-dir <STATE_DIR>\n",
    "          Private persistent execution state for this server identity. Reuse after restart; interrupted work is reconciled, never replayed\n",
    "      --instance <INSTANCE>\n",
    "          Stable named execution-state identity (default: headless). Use a different name for each concurrently running server\n",
    "      --ephemeral\n",
    "          Keep execution state only in memory for this process\n",
    "      --event-journal-dir <EVENT_JOURNAL_DIR>\n",
    "          Opt-in private diagnostic metadata archive. Independent of execution state; bounded history and gaps, never automatic mutation replay\n",
    "      --http <PORT>\n",
    "          Serve the streamable-HTTP transport on this loopback port instead of stdio — the same transport `brep-app --mcp` exposes, with no window. 0 picks a free port; the URL is printed on stderr\n",
    "  -h, --help\n",
    "          Print help\n",
);
pub(crate) const SCHEMA: &str = concat!(
    "Print the kernel feature catalogue rendered as JSON Schema\n",
    "\n",
    "Usage: brep-mcp schema [OPTIONS]\n",
    "\n",
    "Options:\n",
    "      --type <FEATURE_TYPE>  One feature (type or shortName) instead of the whole catalogue\n",
    "      --all                  Also print the test-mcp script format schema\n",
    "      --write <WRITE>        Write the generated docs (commands / features, md + json) into this directory instead of printing\n",
    "  -h, --help                 Print help\n",
);
pub(crate) const TEST: &str = concat!(
    "Run test-mcp scripts through a headless session each\n",
    "\n",
    "Usage: brep-mcp test [OPTIONS] [SCRIPTS]...\n",
    "\n",
    "Arguments:\n",
    "  [SCRIPTS]...\x20\x20\n",
    "\n",
    "Options:\n",
    "      --backend <BACKEND>            [default: headless]\n",
    "      --out <OUT>\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\n",
    "      --update-baselines\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\n",
    "      --session-root <SESSION_ROOT>\x20\x20\n",
    "      --order-check                  ORDER CHECK: play the whole list a second time in REVERSE, and fail if any script's session STARTED differently in the two passes. One script must not change what the next one measures; scripts run through one process, so only what a session owns is actually fresh. Artefacts go under `<out>/forward` and `<out>/reverse`. Costs two runs of the list — point it at a subset, not the whole glob\n",
    "      --video-root <DIR>             Where a script's `video.out` is resolved — the repo root, for the documentation walkthroughs (`./build.sh docs-videos`). WITHOUT it a script that declares a video is still recorded and encoded, but the GIF only reaches the run's output directory: the `test-mcp` gate plays every script and must not rewrite documentation assets as a side effect of being run\n",
    "  -h, --help                         Print help\n",
);
pub(crate) const RENDER_MESH: &str = concat!(
    "Rasterise a triangle mesh `{positions, indices}` to a PNG on the CPU and report its silhouette — the native STEP-validation renderer. No host, no GPU, no window; `--help` prints the full option list. The same command is `BREP_mcp_core`'s `render_mesh` example, which step-validation calls directly so it need not build this binary's app host\n",
    "\n",
    "Usage: brep-mcp render-mesh [ARGS]...\n",
    "\n",
    "Arguments:\n",
    "  [ARGS]...  Everything after `render-mesh`, passed through unparsed\n",
);
pub(crate) const TYPESCRIPT: &str = concat!(
    "The TypeScript toolchain `run_script` checks TypeScript with\n",
    "\n",
    "Usage: brep-mcp typescript <COMMAND>\n",
    "\n",
    "Commands:\n",
    "  status     Report the compiler `run_script` would check TypeScript with: the same discovery and launcher probe `tools/list` advertises. Exits 1 when TypeScript is unavailable\n",
    "  provision  Install the pinned TypeScript into the user-local prefix discovery checks, with npm (`BREP_NPM` or PATH). Downloads; never run by a tool call\n",
    "  help       Print this message or the help of the given subcommand(s)\n",
    "\n",
    "Options:\n",
    "  -h, --help  Print help\n",
);
pub(crate) const SHOT: &str = concat!(
    "Render a document to a PNG through the headless host\n",
    "\n",
    "Usage: brep-mcp shot [OPTIONS] <DOC> <PNG>\n",
    "\n",
    "Arguments:\n",
    "  <DOC>\x20\x20\n",
    "  <PNG>\x20\x20\n",
    "\n",
    "Options:\n",
    "      --view <VIEW>                  [default: ISO]\n",
    "      --size <SIZE>                  [default: 1280x800]\n",
    "      --ppp <PPP>                    [default: 1]\n",
    "      --session-root <SESSION_ROOT>\x20\x20\n",
    "  -h, --help                         Print help\n",
);
pub(crate) const SKILLS_BUNDLE: &str = concat!(
    "Export this binary's reviewed bundle; refuses to replace an existing file\n",
    "\n",
    "Usage: brep-mcp skills bundle --output <OUTPUT>\n",
    "\n",
    "Options:\n",
    "      --output <OUTPUT>\x20\x20\n",
    "  -h, --help             Print help\n",
);
pub(crate) const SKILLS_INSTALL: &str = concat!(
    "Install or update the managed selection from this binary's exact bundle\n",
    "\n",
    "Usage: brep-mcp skills install --bundle <BUNDLE> --sha256 <SHA256> --directory <DIRECTORY>\n",
    "\n",
    "Options:\n",
    "      --bundle <BUNDLE>\x20\x20\x20\x20\x20\x20\x20\x20\n",
    "      --sha256 <SHA256>        SHA256 obtained from the reviewed bundle, not an untrusted download\n",
    "      --directory <DIRECTORY>\x20\x20\n",
    "  -h, --help                   Print help\n",
);
pub(crate) const SKILLS_INSPECT: &str = concat!(
    "Verify the installed selection and report its version directory\n",
    "\n",
    "Usage: brep-mcp skills inspect --directory <DIRECTORY>\n",
    "\n",
    "Options:\n",
    "      --directory <DIRECTORY>\x20\x20\n",
    "  -h, --help                   Print help\n",
);
pub(crate) const SKILLS_ROLLBACK: &str = concat!(
    "Restore the previous installed version after verifying its reviewed pin\n",
    "\n",
    "Usage: brep-mcp skills rollback --directory <DIRECTORY> --sha256 <SHA256>\n",
    "\n",
    "Options:\n",
    "      --directory <DIRECTORY>\x20\x20\n",
    "      --sha256 <SHA256>\x20\x20\x20\x20\x20\x20\x20\x20\n",
    "  -h, --help                   Print help\n",
);
pub(crate) const TYPESCRIPT_STATUS: &str = concat!(
    "Report the compiler `run_script` would check TypeScript with: the same discovery and launcher probe `tools/list` advertises. Exits 1 when TypeScript is unavailable\n",
    "\n",
    "Usage: brep-mcp typescript status\n",
    "\n",
    "Options:\n",
    "  -h, --help  Print help\n",
);
pub(crate) const TYPESCRIPT_PROVISION: &str = concat!(
    "Install the pinned TypeScript into the user-local prefix discovery checks, with npm (`BREP_NPM` or PATH). Downloads; never run by a tool call\n",
    "\n",
    "Usage: brep-mcp typescript provision\n",
    "\n",
    "Options:\n",
    "  -h, --help  Print help\n",
);
}
