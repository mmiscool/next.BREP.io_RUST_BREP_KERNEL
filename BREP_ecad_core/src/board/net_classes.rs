//! Net classes: the clearance, track width and via a net is routed with.
//!
//! # The model
//!
//! A [`NetClass`] is a name, a clearance, a track width, a via diameter and drill,
//! and the name PATTERNS that put nets in it. There is always a class called
//! [`DEFAULT_CLASS`]; it is not stored — it IS the board's own
//! [`DesignRules::track_width`], [`DesignRules::clearance`],
//! [`DesignRules::via_diameter`] and [`DesignRules::via_drill`], read as a class
//! ([`DesignRules::default_class`]). A board saved before net classes existed
//! therefore opens with exactly one class, Default, built from its rules, and
//! nothing is written or changed on the way in: the two stored fields
//! ([`DesignRules::net_classes`], [`DesignRules::net_class_of`]) are left out of
//! the saved block while they are empty, so the block reads back byte for byte.
//!
//! # Which class a net is in
//!
//! In this order, the first that applies wins:
//!
//! 1. an explicit assignment, [`DesignRules::net_class_of`] (`net → class`), when
//!    it names a class that exists — one that names a deleted class is ignored;
//! 2. the first class, in list order, one of whose [`NetClass::patterns`] matches
//!    the net's whole name. A pattern is a glob: `*` any run of characters, `?` any
//!    one character, everything else itself, case-sensitive (`+*V*` holds `+5V` and
//!    `+3V3`, not `GND`). KiCad 7 introduced the same kind of pattern; the
//!    tie-break by list order here is this model's choice, not a claim about KiCad's;
//! 3. Default.
//!
//! Copper that carries no net (an unrouted stub, a mechanical pad) is Default.
//!
//! # The lookups every consumer calls
//!
//! * [`DesignRules::clearance_between`]: the clearance between copper of two
//!   nets is the LARGER of their two classes' clearances (KiCad's rule), and the
//!   class that set it, for the message;
//! * [`DesignRules::width_for`]: a net's track width — its per-net width
//!   ([`DesignRules::net_widths`], the Inspector's "Use this width for NET") if it
//!   has one, else its class's;
//! * [`DesignRules::via_for`]: a net's via diameter and drill, its class's.
//!
//! [`DesignRules::max_clearance`] and [`DesignRules::min_clearance`] bound them all,
//! for a spatial prefilter (the max: a pair culled at the Default clearance may
//! still be too close for Power) and for a sampling pitch (the min).
//!
//! The differential-pair width and gap are stored and edited, and read by nothing
//! yet.

use super::DesignRules;
use serde::{Deserialize, Serialize};

/// The class every net is in unless something puts it in another.
pub const DEFAULT_CLASS: &str = "Default";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetClass {
    pub name: String,
    pub clearance: i32,
    pub track_width: i32,
    pub via_diameter: i32,
    pub via_drill: i32,
    /// Name globs that put a net in this class ([module docs](self)).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub patterns: Vec<String>,
    /// Stored for a later round; nothing reads it yet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub diff_pair_width: Option<i32>,
    /// Stored for a later round; nothing reads it yet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub diff_pair_gap: Option<i32>,
}

/// Whether glob `pattern` (`*`, `?`) matches ALL of `name`.
pub fn glob_matches(pattern: &str, name: &str) -> bool {
    let (p, n): (Vec<char>, Vec<char>) = (pattern.chars().collect(), name.chars().collect());
    let (mut i, mut j) = (0, 0);
    // Where the last `*` was, and how much of the name it had taken when it was
    // last retried.
    let mut star: Option<(usize, usize)> = None;
    while j < n.len() {
        if i < p.len() && (p[i] == '?' || (p[i] != '*' && p[i] == n[j])) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some((i, j));
            i += 1;
        } else if let Some((s, taken)) = star {
            i = s + 1;
            j = taken + 1;
            star = Some((s, taken + 1));
        } else {
            return false;
        }
    }
    p[i..].iter().all(|&c| c == '*')
}

/// How a net came to be in its class, for the Inspector's readout.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ClassReason {
    /// [`DesignRules::net_class_of`] names it.
    Assigned,
    /// This pattern of the class matched.
    Pattern(String),
    /// Nothing else applied.
    Default,
}

impl DesignRules {
    /// The Default class: this board's own track width, clearance and via.
    pub fn default_class(&self) -> NetClass {
        NetClass {
            name: DEFAULT_CLASS.into(),
            clearance: self.clearance,
            track_width: self.track_width,
            via_diameter: self.via_diameter,
            via_drill: self.via_drill,
            patterns: vec![],
            diff_pair_width: None,
            diff_pair_gap: None,
        }
    }
    /// Every class, Default first, then the stored ones in list order.
    pub fn classes(&self) -> Vec<NetClass> {
        std::iter::once(self.default_class()).chain(self.net_classes.iter().cloned()).collect()
    }
    fn stored_class(&self, name: &str) -> Option<&NetClass> {
        self.net_classes.iter().find(|c| c.name == name)
    }
    /// The class `net` is in and why ([module docs](self) for the order).
    pub fn class_reason(&self, net: Option<&str>) -> (NetClass, ClassReason) {
        let Some(net) = net else {
            return (self.default_class(), ClassReason::Default);
        };
        if let Some(class) = self.net_class_of.get(net).and_then(|name| self.stored_class(name)) {
            return (class.clone(), ClassReason::Assigned);
        }
        for class in &self.net_classes {
            if let Some(pattern) = class.patterns.iter().find(|p| glob_matches(p, net)) {
                return (class.clone(), ClassReason::Pattern(pattern.clone()));
            }
        }
        (self.default_class(), ClassReason::Default)
    }
    /// The class `net` is in; `None` (copper on no net) is Default.
    pub fn class_of(&self, net: Option<&str>) -> NetClass {
        self.class_reason(net).0
    }
    /// The class that sets the clearance for copper carrying `nets` (several for
    /// an island that shorts two nets; none for netless copper): the one with the
    /// largest clearance, Default for none.
    pub fn class_of_nets<'a>(&self, nets: impl IntoIterator<Item = &'a str>) -> NetClass {
        nets.into_iter()
            .map(|net| self.class_of(Some(net)))
            .reduce(|a, b| if b.clearance > a.clearance { b } else { a })
            .unwrap_or_else(|| self.default_class())
    }
    /// The clearance copper of `a` must keep from copper of `b`: the larger of
    /// their classes' clearances, with the class that set it (`a`'s on a tie).
    pub fn clearance_between(&self, a: Option<&str>, b: Option<&str>) -> (i32, NetClass) {
        Self::wider(self.class_of(a), self.class_of(b))
    }
    /// [`Self::clearance_between`] for two sets of nets ([`Self::class_of_nets`]).
    pub fn clearance_between_nets<'a>(
        &self,
        a: impl IntoIterator<Item = &'a str>,
        b: impl IntoIterator<Item = &'a str>,
    ) -> (i32, NetClass) {
        Self::wider(self.class_of_nets(a), self.class_of_nets(b))
    }
    /// [`Self::clearance_between`] for two classes already looked up, for a caller
    /// that looks each net's class up once and then pairs them many times (the
    /// autorouter).
    pub fn clearance_between_classes(a: &NetClass, b: &NetClass) -> (i32, NetClass) {
        Self::wider(a.clone(), b.clone())
    }
    fn wider(a: NetClass, b: NetClass) -> (i32, NetClass) {
        let class = if b.clearance > a.clearance { b } else { a };
        (class.clearance.max(0), class)
    }
    /// The largest clearance any class asks for.
    pub fn max_clearance(&self) -> i32 {
        self.net_classes.iter().map(|c| c.clearance).fold(self.clearance, i32::max).max(0)
    }
    /// The smallest clearance any class asks for.
    pub fn min_clearance(&self) -> i32 {
        self.net_classes.iter().map(|c| c.clearance).fold(self.clearance, i32::min).max(0)
    }
    /// The track width for `net`: its own width ([`Self::net_widths`]) if it has
    /// one, else its class's. Every width a new track is given comes from here.
    pub fn width_for(&self, net: &str) -> i32 {
        self.net_widths
            .get(net)
            .copied()
            .unwrap_or_else(|| self.class_of(Some(net)).track_width)
    }
    /// The via diameter and drill for `net` (`None`: Default's).
    pub fn via_for(&self, net: Option<&str>) -> (i32, i32) {
        let class = self.class_of(net);
        (class.via_diameter, class.via_drill)
    }
    /// Why the class list or assignment can not be saved, if it can not.
    pub fn check_net_classes(&self) -> Result<(), String> {
        let size = |v: i32| (1..=100_000_000).contains(&v);
        let mut names = std::collections::BTreeSet::new();
        for class in &self.net_classes {
            let name = class.name.trim();
            if name.is_empty() || name != class.name {
                return Err("A net class needs a name without spaces round it".into());
            }
            if name == DEFAULT_CLASS {
                return Err(format!("{DEFAULT_CLASS} is the board's own rules, and is always there"));
            }
            if !names.insert(name) {
                return Err(format!("Two net classes are called {name}"));
            }
            if !(0..=100_000_000).contains(&class.clearance)
                || !size(class.track_width)
                || !size(class.via_drill)
                || !size(class.via_diameter)
                || class.via_diameter <= class.via_drill
                || class.diff_pair_width.is_some_and(|w| !size(w))
                || class.diff_pair_gap.is_some_and(|g| !(0..=100_000_000).contains(&g))
            {
                return Err(format!("Net class {name} has an invalid size"));
            }
            if class.patterns.iter().any(|p| p.trim().is_empty()) {
                return Err(format!("Net class {name} has an empty pattern"));
            }
        }
        Ok(())
    }
}

