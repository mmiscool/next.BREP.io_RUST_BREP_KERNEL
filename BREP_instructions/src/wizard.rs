//! The step-order wizard: proposes the step tree from an assembly's
//! occurrence tree and mate graph.
//!
//! The order, per assembly level:
//! 1. every sub-assembly is one step of its parent's level whose children are
//!    its own parts' steps: they build it first, then the step places it;
//! 2. the grounded components come first, then mate order: the next component
//!    is the one mated to what is already placed, earliest in the assembly's
//!    own order. When the assembly already has an exploded view, its steps
//!    seed the order instead (an exploded view lists parts coming OFF, so it
//!    reads backwards);
//! 3. fasteners come last, one step per fastener part, in the order of the
//!    parts they fasten, with the torque taken from the part's attribute.
//!
//! One step per component or sub-assembly. The author merges, splits and
//! reorders afterwards ([`crate::edit`]).

use crate::model::{Add, Instructions, Step, Torque};
use std::collections::{HashMap, HashSet};

/// One assembly level: its components and the mates between them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Level {
    /// The occurrence path of this assembly; `""` for the target itself.
    pub path: String,
    /// The assembly's part name.
    pub name: String,
    /// Components in the assembly's own (history) order.
    pub components: Vec<Component>,
    /// Mates, as pairs of component paths of THIS level. A mate between parts
    /// inside two sub-assemblies is given as the two sub-assembly paths.
    pub mates: Vec<(String, String)>,
    /// Component paths in the order the assembly's exploded view moves them
    /// out; empty when it has none.
    pub exploded_order: Vec<String>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Component {
    /// Occurrence path relative to the instructions document (`ACOMP1:ACOMP3`).
    pub path: String,
    /// The part's name (titles, the display name).
    pub part: String,
    /// The PLM part id when the part was placed from a PLM revision
    /// (`part/<id>/rev/<rev>`): what an add's `part` names then.
    pub part_id: Option<String>,
    /// The placed part's content signature (re-target compares it).
    pub signature: String,
    pub find_number: String,
    pub grounded: bool,
    pub fastener: bool,
    pub serial_tracked: bool,
    /// The fastener's `Torque` attribute text.
    pub torque: Option<String>,
    /// A sub-assembly's own level.
    pub children: Option<Box<Level>>,
}

/// Propose the steps for `root` into `block`, replacing its step tree.
pub fn propose(block: &mut Instructions, root: &Level) {
    block.steps.clear();
    block.steps = level_steps(block, root);
}

/// The steps that build `level`, in proposed order; a sub-assembly's step
/// holds the steps that build it as its children.
pub fn level_steps(block: &mut Instructions, level: &Level) -> Vec<Step> {
    let mut steps = Vec::new();
    for group in order(level) {
        let mut step = step_for(block, &group);
        if let Some(children) = &group[0].children {
            step.steps = level_steps(block, children);
        }
        steps.push(step);
    }
    steps
}

/// The level's components grouped into steps, in proposed order. Every group
/// but a fastener group holds one component.
pub fn order(level: &Level) -> Vec<Vec<&Component>> {
    let parts: Vec<&Component> = level.components.iter().filter(|c| !c.fastener).collect();
    let mut placed: Vec<&Component> = Vec::new();
    let mut placed_set: HashSet<&str> = HashSet::new();
    let neighbours = adjacency(level);
    let seed: HashMap<&str, usize> =
        level.exploded_order.iter().rev().enumerate().map(|(i, p)| (p.as_str(), i)).collect();

    // Grounded first, in assembly order; with none grounded, the first part.
    for c in parts.iter().filter(|c| c.grounded) {
        placed.push(c);
        placed_set.insert(c.path.as_str());
    }
    if placed.is_empty() && seed.is_empty() {
        if let Some(first) = parts.first() {
            placed.push(first);
            placed_set.insert(first.path.as_str());
        }
    }
    while placed.len() < parts.len() {
        let remaining = parts.iter().filter(|c| !placed_set.contains(c.path.as_str()));
        let next = if !seed.is_empty() {
            // The exploded view's reverse order; parts it does not move go
            // after, in mate order.
            remaining
                .clone()
                .filter(|c| seed.contains_key(c.path.as_str()))
                .min_by_key(|c| seed[c.path.as_str()])
                .copied()
                .or_else(|| mated_first(remaining.clone(), &neighbours, &placed_set))
        } else {
            mated_first(remaining.clone(), &neighbours, &placed_set)
        };
        let next = next.or_else(|| remaining.clone().next().copied()).expect("a remaining part");
        placed.push(next);
        placed_set.insert(next.path.as_str());
    }

    let mut groups: Vec<Vec<&Component>> = placed.into_iter().map(|c| vec![c]).collect();

    // Fasteners: one group per part, ordered by the latest placed part each
    // one is mated to (unmated ones last, in assembly order).
    let position: HashMap<&str, usize> = groups.iter().enumerate().map(|(i, g)| (g[0].path.as_str(), i)).collect();
    let mut fastener_groups: Vec<(usize, usize, Vec<&Component>)> = Vec::new();
    for (index, c) in level.components.iter().enumerate().filter(|(_, c)| c.fastener) {
        let after = neighbours
            .get(c.path.as_str())
            .into_iter()
            .flatten()
            .filter_map(|n| position.get(n).copied())
            .max()
            .unwrap_or(usize::MAX);
        match fastener_groups.iter_mut().find(|(_, _, g)| g[0].part == c.part) {
            Some((rank, _, group)) => {
                *rank = (*rank).max(after);
                group.push(c);
            }
            None => fastener_groups.push((after, index, vec![c])),
        }
    }
    fastener_groups.sort_by_key(|(rank, index, _)| (*rank, *index));
    groups.extend(fastener_groups.into_iter().map(|(_, _, g)| g));
    groups
}

/// The remaining part mated to something placed, earliest in assembly order.
fn mated_first<'a, 'b>(
    mut remaining: impl Iterator<Item = &'b &'a Component>,
    neighbours: &HashMap<&str, Vec<&str>>,
    placed: &HashSet<&str>,
) -> Option<&'a Component>
where
    'a: 'b,
{
    remaining
        .find(|c| neighbours.get(c.path.as_str()).is_some_and(|n| n.iter().any(|p| placed.contains(p))))
        .copied()
}

fn adjacency(level: &Level) -> HashMap<&str, Vec<&str>> {
    let mut map: HashMap<&str, Vec<&str>> = HashMap::new();
    for (a, b) in &level.mates {
        if a == b {
            continue;
        }
        map.entry(a.as_str()).or_default().push(b.as_str());
        map.entry(b.as_str()).or_default().push(a.as_str());
    }
    map
}

/// The step that adds `group`.
pub fn step_for(block: &mut Instructions, group: &[&Component]) -> Step {
    let first = group[0];
    let quantity = group.len();
    let subassembly = first.children.is_some();
    let title = match (subassembly, quantity) {
        (true, _) => format!("Install sub-assembly {}", first.part),
        (false, 1) => format!("Install {}", first.part),
        (false, n) => format!("Install {n} × {}", first.part),
    };
    let torque = if first.fastener { first.torque.as_deref().and_then(Torque::from_attribute) } else { None };
    Step {
        id: block.next_id("MS"),
        title,
        adds: vec![Add {
            find_number: first.find_number.clone(),
            subassembly,
            fastener: first.fastener,
            serial_tracked: first.serial_tracked,
            ..Add::named(group.iter().map(|c| c.path.clone()).collect(), first.part_key(), &first.part)
        }],
        torque,
        ..Step::default()
    }
}

impl Component {
    /// What an add's `part` names for this component: the PLM part id, else
    /// the name.
    pub fn part_key(&self) -> &str {
        self.part_id.as_deref().unwrap_or(&self.part)
    }
}

/// Every component of `level` and its sub-assemblies, depth first, each with
/// the path of the level that holds it.
pub fn flatten(level: &Level) -> Vec<(&str, &Component)> {
    let mut out = Vec::new();
    fn walk<'a>(level: &'a Level, out: &mut Vec<(&'a str, &'a Component)>) {
        for c in &level.components {
            out.push((level.path.as_str(), c));
            if let Some(children) = &c.children {
                walk(children, out);
            }
        }
    }
    walk(level, &mut out);
    out
}

/// The level at `path` inside `root` (`""` is the root).
pub fn find_level<'a>(root: &'a Level, path: &str) -> Option<&'a Level> {
    if root.path == path {
        return Some(root);
    }
    root.components.iter().filter_map(|c| c.children.as_deref()).find_map(|l| find_level(l, path))
}

