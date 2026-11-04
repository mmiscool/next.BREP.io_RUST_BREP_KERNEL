//! READ ONCE (lane O row 19, 2026-10-29; on by default since the batch 5
//! promotion, `BREP_ENV_READ_ONCE=0` reads every call).
//!
//! The kernel reads its debug and escape-hatch variables at their call sites,
//! on every call: on the D2 outward shells about 3 000 environment reads a
//! shell call (counted with a `getenv` shim), each a process-wide locked
//! lookup and a fresh `String`. `env_read!` gives each call site its own
//! once-cell: with the switch on, the first read is kept for the process; off,
//! every call reads the environment as before. Only variables nothing sets at
//! run time go through it — a probe or example that sets or removes one
//! (`BREP_DEBUG_MERGE`, `BREP_DEBUG_PAIRS`, `BREP_NO_SPHERE_CHART_TRIM`,
//! `BREP_SUPPLIED_PCURVES`, ...) keeps its live `std::env::var`.

//!
//! SWITCHES (consolidation S1, 2026-11-01). [`switch!`], [`debug_flag!`] and
//! [`mode_switch!`] are the one reader every on/off lane switch, debug flag
//! and `0|off / check / on` mode goes through: the same per-call-site
//! once-cell as `env_read!` (or, with `live`, a read on every call for a
//! variable a probe sets at run time) and, in the kernel's own tests, one
//! override keyed by the variable's name (`with_switch_for_test`,
//! `set_switch_for_test`) in place of a thread-local per switch. The cell
//! keeps the variable's reading (`None`: unset or not a value the grammar
//! names); the default is applied at the call, so it may vary per call.

/// The value of `name` as `std::env::var(name).ok()` reads it, through the
/// call site's `cell` when the switch is on.
#[doc(hidden)]
pub fn read(name: &'static str, cell: &'static std::sync::OnceLock<Option<String>>) -> Option<std::borrow::Cow<'static, str>> {
    if read_once_enabled() {
        cell.get_or_init(|| std::env::var(name).ok()).as_deref().map(std::borrow::Cow::Borrowed)
    } else {
        std::env::var(name).ok().map(std::borrow::Cow::Owned)
    }
}

/// How a switch reads its variable (consolidation S1 step 3, the
/// owner-approved grammar of 2026-11-01).
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Grammar {
    /// An on/off switch ([`switch!`]): `0`/`off`/`false` off, `1`/`on`/`true`
    /// on; unset or anything else: the switch's default.
    Switch,
    /// A debug or `BREP_NO_*` flag ([`debug_flag!`]): set to anything but
    /// `0`/`off`/`false` on; unset or one of those, off. The default is
    /// never read.
    Flag,
}

/// The reading `grammar` takes from `name`'s value: `None` takes the default.
fn reading(name: &str, grammar: Grammar) -> Option<bool> {
    let value = std::env::var_os(name);
    let value = value.as_ref().map(|value| value.to_str());
    match grammar {
        Grammar::Switch => match value {
            Some(Some("0" | "off" | "false")) => Some(false),
            Some(Some("1" | "on" | "true")) => Some(true),
            _ => None,
        },
        Grammar::Flag => Some(!matches!(value, None | Some(Some("0" | "off" | "false")))),
    }
}

/// [`switch!`]: the switch `name` through its call site's `cell` (kept for
/// the process while `BREP_ENV_READ_ONCE` is on).
#[doc(hidden)]
pub fn switch(name: &'static str, cell: &'static std::sync::OnceLock<Option<bool>>, default: bool, grammar: Grammar) -> bool {
    let read = if read_once_enabled() { *cell.get_or_init(|| reading(name, grammar)) } else { reading(name, grammar) };
    overridden(name, read.unwrap_or(default))
}

/// [`switch!`]`(live ..)`: the switch `name` read on every call.
#[doc(hidden)]
pub fn switch_live(name: &'static str, default: bool, grammar: Grammar) -> bool {
    overridden(name, reading(name, grammar).unwrap_or(default))
}

/// A `0|off / check / on` mode ([`mode_switch!`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Off,
    Check,
    On,
}

fn mode_reading(name: &str) -> Option<Mode> {
    match std::env::var(name).as_deref() {
        Ok("0") | Ok("off") => Some(Mode::Off),
        Ok("check") => Some(Mode::Check),
        Ok(_) => Some(Mode::On),
        Err(_) => None,
    }
}

/// [`mode_switch!`]: `0`/`off` off, `check` check, any other value on; unset,
/// on when `default_on` and off otherwise.
#[doc(hidden)]
pub fn mode(name: &'static str, cell: &'static std::sync::OnceLock<Option<Mode>>, default_on: bool) -> Mode {
    let read = if read_once_enabled() { *cell.get_or_init(|| mode_reading(name)) } else { mode_reading(name) };
    read.unwrap_or(if default_on { Mode::On } else { Mode::Off })
}

/// [`mode_switch!`]`(live ..)`: the mode read on every call.
#[doc(hidden)]
pub fn mode_live(name: &'static str, default_on: bool) -> Mode {
    mode_reading(name).unwrap_or(if default_on { Mode::On } else { Mode::Off })
}

#[inline]
fn overridden(name: &'static str, value: bool) -> bool {
    let _ = name;
    value
}


#[doc(hidden)]
pub fn read_once_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let enabled = *ON.get_or_init(|| !matches!(std::env::var("BREP_ENV_READ_ONCE").as_deref(), Ok("0") | Ok("off")));
    enabled
}


