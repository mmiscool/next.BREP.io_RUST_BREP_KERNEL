//! Grid-based autorouter.
//!
//! Each net is routed island to island (a Prim-style tree) with A* over a uniform
//! grid on every copper layer, using 45° moves and through vias.
//!
//! Clearance is exact, not approximated by the grid. Every copper obstacle is convex
//! (rectangles, capsules, discs, outline edges). If both ends of a move of length `L`
//! lie at least `T` from a convex shape, every point of the move lies at least
//! `√(T² − L²/4)` from it: a chord that dips closer would be shorter than `L`. Nodes
//! are therefore legal only at `T = √(Q² + pitch²/2)`, with `Q = clearance + width/2`,
//! which guarantees `Q` along every straight or diagonal move, so routed copper passes
//! the exact-geometry DRC while the extra margin stays a few micrometres.
//!
//! Net classes. Each net is routed at its class's track width and via
//! ([`crate::board::DesignRules::width_for`], [`crate::board::DesignRules::via_for`]),
//! and the `clearance` above is PER PAIR: between the net being routed and each
//! obstacle it is [`crate::board::DesignRules::clearance_between_classes`] of their
//! two classes, the larger of the two, which is the rule DRC checks. The grid keeps
//! one set of maps per `Profile` (a track width or via diameter, and the class of
//! the net using it), and grows each obstacle on a map by the pair's clearance; a
//! board with no classes has exactly the maps, and routes exactly as, before
//! classes existed. A net that is blocked by earlier routes is retried with those routes allowed
//! at a high cost; the nets it crosses are ripped up and queued again, with bounded
//! attempts. Work is split into [`RouteJob::step`] calls, measured in node
//! expansions rather than wall time, so interactive and WebAssembly hosts stay
//! responsive.
//!
//! Vias follow the board's [`ViaPolicy`]. `Avoid` routes exactly as `Allow` does, so
//! it connects every net `Allow` would, and then removes vias: each net that has any
//! is rerouted against the finished board, first searching for a route with no via
//! at all, and keeps the new copper only if it connects everything with fewer vias.
//! Routing with vias priced out from the start was tried and rejected: long
//! single-layer detours fill the board early and later nets fail to route.
use crate::board::{
    Board, Connectivity, CopperItem, CopperRef, DesignRules, NetClass, Shape, Track, Via, ViaPolicy,
};
use crate::{Document, Netlist, Point, Uuid};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap, HashSet, VecDeque};

/// Pad entry points for one net.
#[derive(Default)]
struct Stubs {
    /// Per seed node, the stub from the pad centre up to (not including) the node.
    paths: HashMap<u32, Vec<Point>>,
    /// Seed nodes inside large pads, such as exposed thermal pads, where a via fits.
    vias: HashSet<u32>,
}

/// Claim owner for copper that no routed net may touch.
const BLOCKED: u32 = u32::MAX;
/// Upper bound on grid cells per layer; the pitch is coarsened to stay below it.
const MAX_CELLS: usize = 1_500_000;
const DIRECTIONS: [(i64, i64); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];
const NO_DIRECTION: u8 = 8;
const NO_PARENT: u32 = u32::MAX;
/// Width of the band around routed pads that other nets are discouraged from using.
const APPROACH: i32 = 800;
/// Rip-up retries a single net may initiate.
const MAX_ATTEMPTS: u32 = 4;
/// Rerouting passes over nets with vias; another pass runs only if one improved.
const MAX_CLEANUP_PASSES: u32 = 3;

#[derive(Clone, Copy, Debug)]
struct Grid {
    origin: Point,
    pitch: i32,
    width: usize,
    height: usize,
    layers: usize,
}
impl Grid {
    fn cells(&self) -> usize {
        self.width * self.height
    }
    fn centre(&self, cell: usize) -> Point {
        Point::new(
            self.origin.x + (cell % self.width) as i32 * self.pitch,
            self.origin.y + (cell / self.width) as i32 * self.pitch,
        )
    }
    /// Inclusive cell rectangle whose centres lie within the world bounds.
    fn span(&self, min: Point, max: Point) -> Option<(usize, usize, usize, usize)> {
        let pitch = i64::from(self.pitch);
        let low = |v: i32, o: i32| (i64::from(v) - i64::from(o)).div_euclid(pitch) + 1;
        let high = |v: i32, o: i32| (i64::from(v) - i64::from(o)).div_euclid(pitch);
        let x0 = (low(min.x - 1, self.origin.x)).max(0);
        let y0 = (low(min.y - 1, self.origin.y)).max(0);
        let x1 = high(max.x, self.origin.x).min(self.width as i64 - 1);
        let y1 = high(max.y, self.origin.y).min(self.height as i64 - 1);
        (x0 <= x1 && y0 <= y1).then_some((x0 as usize, x1 as usize, y0 as usize, y1 as usize))
    }
}

/// Whether a disc lies entirely within a shape.
fn disc_inside(shape: &Shape, at: Point, radius: i32) -> bool {
    match *shape {
        Shape::Rect { min, max } => {
            at.x - radius >= min.x
                && at.x + radius <= max.x
                && at.y - radius >= min.y
                && at.y + radius <= max.y
        }
        Shape::Capsule { a, b, radius: r } => {
            crate::board::point_segment_distance(at, a, b) + f64::from(radius) <= f64::from(r)
        }
    }
}

/// Visit cells whose centres are closer than `radius` to the shape.
fn for_cells_near(grid: &Grid, shape: &Shape, radius: f64, mut visit: impl FnMut(usize)) {
    let (min, max) = shape.bounds();
    let reach = radius.ceil() as i32 + 1;
    let Some((x0, x1, y0, y1)) = grid.span(
        Point::new(min.x.saturating_sub(reach), min.y.saturating_sub(reach)),
        Point::new(max.x.saturating_add(reach), max.y.saturating_add(reach)),
    ) else {
        return;
    };
    for y in y0..=y1 {
        for x in x0..=x1 {
            let cell = y * grid.width + x;
            if shape.distance_to_point(grid.centre(cell)) < radius {
                visit(cell);
            }
        }
    }
}

/// Per-cell claims by copper owners. Each owner claims a cell at most once, so a
/// cell is usable by net `n` when unclaimed or claimed only by `n`.
#[derive(Clone)]
struct Claims {
    count: Vec<u16>,
    sum: Vec<u32>,
}
impl Claims {
    fn new(cells: usize) -> Self {
        Self {
            count: vec![0; cells],
            sum: vec![0; cells],
        }
    }
    fn free(&self, cell: usize, net: u32) -> bool {
        let count = self.count[cell];
        count == 0 || (count == 1 && self.sum[cell] == net)
    }
    fn add(&mut self, cell: usize, owner: u32) {
        self.count[cell] = self.count[cell].saturating_add(1);
        self.sum[cell] = self.sum[cell].wrapping_add(owner);
    }
    fn remove(&mut self, cell: usize, owner: u32) {
        self.count[cell] -= 1;
        self.sum[cell] = self.sum[cell].wrapping_sub(owner);
    }
}

/// What one set of grid maps is inflated for: copper `size` across (a track width
/// or a via diameter) of a net in class `class`, an index into
/// [`RouteJob::gaps`]. Obstacle `o` is kept `gaps[class][o's class]` from it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Profile {
    size: i32,
    class: usize,
}

struct RouteNet {
    name: String,
    width: i32,
    /// The net's class, an index into [`RouteJob::gaps`].
    class: usize,
    /// Its via diameter and drill, its class's.
    via: (i32, i32),
    /// The [`Profile`]s its tracks and its vias are searched on.
    profile: usize,
    via_profile: usize,
    /// Copper item indices of each island carrying the net.
    islands: Vec<Vec<usize>>,
    span: i64,
}

#[derive(Default)]
struct NetState {
    tracks: Vec<Track>,
    vias: Vec<Via>,
    /// Route claims as `map * cells + cell`.
    claimed: HashSet<u64>,
    /// Island connections that could not be made in the last attempt.
    missing: usize,
    routed: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    pub total_nets: usize,
    pub completed_nets: usize,
    pub failed_nets: usize,
    pub rip_ups: usize,
    /// Every net is routed and nets with vias are being rerouted to drop them.
    pub removing_vias: bool,
    pub done: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RouteOutcome {
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    /// Nets left with unrouted connections.
    pub unrouted: Vec<String>,
    /// Grid pitch actually used, after any coarsening for large boards.
    pub pitch: i32,
}
impl RouteOutcome {
    /// Add the routed copper to the board. Existing copper is kept.
    pub fn apply(self, board: &mut Board) {
        board.tracks.extend(self.tracks);
        board.vias.extend(self.vias);
    }
}

/// A resumable routing run over a snapshot of the board.
pub struct RouteJob {
    grid: Grid,
    /// The track profiles of routed nets, then their via profiles; one set of
    /// maps each.
    profiles: Vec<Profile>,
    /// The clearance between copper of two classes, by class index: filled once
    /// from [`crate::board::DesignRules::clearance_between_classes`], the one
    /// lookup DRC and the zone fill use, and the only clearance this router reads.
    /// Class 0 is the class of copper on no net, Default.
    gaps: Vec<Vec<i32>>,
    /// The largest entry of `gaps`, for bounding-box prefilters.
    max_gap: i32,
    items: Vec<CopperItem>,
    /// The class of each static item: that of the nets its island carries.
    item_class: Vec<usize>,
    /// Claim owner of each static item: a routed net index or `BLOCKED`.
    owners: Vec<u32>,
    outline: Vec<Point>,
    edge_clearance: i32,
    static_claims: Vec<Claims>,
    route_claims: Vec<Claims>,
    /// Per layer, the approaches to routed nets' pads. Other nets pay to pass
    /// through them, which leaves room for fan-out from fine-pitch parts.
    approaches: Vec<Claims>,
    nets: Vec<RouteNet>,
    states: Vec<NetState>,
    queue: VecDeque<usize>,
    attempts: Vec<u32>,
    rip_ups: usize,
    rip_up_limit: usize,
    expansions: u64,
    cost: Vec<u32>,
    parent: Vec<u32>,
    direction: Vec<u8>,
    stamp: Vec<u32>,
    generation: u32,
    /// How often each node has been contested; raises its cost for every net.
    history: Vec<u16>,
    /// Cells where a via would touch a pad; through vias span every layer.
    no_via: Vec<bool>,
    vias: ViaPolicy,
    /// Nets still to reroute in the current via clean-up pass; `None` until routing
    /// has finished and the first pass begins.
    cleanup: Option<VecDeque<usize>>,
    cleanup_passes: u32,
    /// Where copper given up by nets that lost vias this pass sat, grown by the
    /// search margin. Only nets near it can gain from another pass.
    cleanup_freed: Vec<(Point, Point)>,
}

impl RouteJob {
    /// Prepare to connect every unrouted connection. Existing copper is kept and
    /// treated as part of its net, or as an obstacle.
    pub fn new(board: &Board, netlist: &Netlist) -> Self {
        let conn = board.connectivity(netlist);
        let rules = &board.rules;
        // Every class in play, Default first, so netless copper is class 0.
        let mut classes: Vec<NetClass> = vec![rules.default_class()];
        let mut class_index = |class: NetClass| {
            classes
                .iter()
                .position(|c| c.name == class.name)
                .unwrap_or_else(|| {
                    classes.push(class);
                    classes.len() - 1
                })
        };
        let island_class: Vec<usize> = conn
            .island_nets
            .iter()
            .map(|nets| class_index(rules.class_of_nets(nets.iter().map(String::as_str))))
            .collect();
        let item_class: Vec<usize> = conn.islands.iter().map(|&i| island_class[i]).collect();
        let mut island_items = vec![vec![]; conn.island_count];
        for (i, island) in conn.islands.iter().enumerate() {
            island_items[*island].push(i);
        }
        let mut net_islands: BTreeMap<&str, BTreeSet<usize>> = BTreeMap::new();
        for (i, net) in conn.item_nets.iter().enumerate() {
            if let Some(net) = net {
                net_islands.entry(net).or_default().insert(conn.islands[i]);
            }
        }
        let mut nets: Vec<RouteNet> = net_islands
            .into_iter()
            .filter(|(_, islands)| islands.len() > 1)
            .map(|(name, islands)| {
                let centres: Vec<Point> = islands
                    .iter()
                    .flat_map(|&s| island_items[s].iter())
                    .map(|&i| conn.items[i].shape.center())
                    .collect();
                let (x0, x1) = centres
                    .iter()
                    .fold((i32::MAX, i32::MIN), |(a, b), p| (a.min(p.x), b.max(p.x)));
                let (y0, y1) = centres
                    .iter()
                    .fold((i32::MAX, i32::MIN), |(a, b), p| (a.min(p.y), b.max(p.y)));
                let class = rules.class_of(Some(name));
                RouteNet {
                    name: name.to_owned(),
                    width: rules.width_for(name),
                    via: (class.via_diameter, class.via_drill),
                    class: class_index(class),
                    profile: 0,
                    via_profile: 0,
                    islands: islands.iter().map(|&s| island_items[s].clone()).collect(),
                    span: i64::from(x1) - i64::from(x0) + i64::from(y1) - i64::from(y0),
                }
            })
            .collect();
        nets.sort_by(|a, b| a.span.cmp(&b.span).then_with(|| a.name.cmp(&b.name)));
        let gaps: Vec<Vec<i32>> = classes
            .iter()
            .map(|a| {
                classes
                    .iter()
                    .map(|b| DesignRules::clearance_between_classes(a, b).0)
                    .collect()
            })
            .collect();
        let max_gap = gaps.iter().flatten().copied().max().unwrap_or(0);
        let tracks: Vec<Profile> = nets
            .iter()
            .map(|n| Profile {
                size: n.width,
                class: n.class,
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        let vias: Vec<Profile> = nets
            .iter()
            .map(|n| Profile {
                size: n.via.0,
                class: n.class,
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        for net in &mut nets {
            let (track, via) = (
                Profile {
                    size: net.width,
                    class: net.class,
                },
                Profile {
                    size: net.via.0,
                    class: net.class,
                },
            );
            net.profile = tracks.binary_search(&track).unwrap_or(0);
            net.via_profile = tracks.len() + vias.binary_search(&via).unwrap_or(0);
        }
        let thinnest = tracks
            .iter()
            .map(|p| &p.size)
            .chain(board.tracks.iter().map(|t| &t.width))
            .min()
            .copied()
            .unwrap_or(rules.track_width);
        let profiles: Vec<Profile> = tracks.into_iter().chain(vias).collect();
        let (min, max) = board.outline_bounds();
        let dims = |pitch: i32| {
            (
                ((i64::from(max.x) - i64::from(min.x)) / i64::from(pitch) + 1) as usize,
                ((i64::from(max.y) - i64::from(min.y)) / i64::from(pitch) + 1) as usize,
            )
        };
        let mut pitch = rules.routing_grid.min(thinnest * 7 / 10).max(10);
        while {
            let (w, h) = dims(pitch);
            w * h > MAX_CELLS
        } {
            pitch += pitch / 4 + 1;
        }
        let (width, height) = dims(pitch);
        let grid = Grid {
            origin: min,
            pitch,
            width,
            height,
            layers: usize::from(board.layer_count.max(1)),
        };
        let cells = grid.cells();
        let nodes = cells * grid.layers;
        let maps = profiles.len() * grid.layers;
        let count = nets.len();
        let mut job = Self {
            grid,
            gaps,
            max_gap,
            items: vec![],
            item_class,
            owners: vec![],
            outline: board.outline.clone(),
            edge_clearance: rules.edge_clearance,
            static_claims: vec![Claims::new(cells); maps],
            route_claims: vec![Claims::new(cells); maps],
            approaches: vec![Claims::new(cells); grid.layers],
            states: (0..count).map(|_| NetState::default()).collect(),
            queue: (0..count).collect(),
            attempts: vec![0; count],
            rip_ups: 0,
            rip_up_limit: count * MAX_ATTEMPTS as usize + 16,
            expansions: 0,
            cost: vec![0; nodes],
            parent: vec![NO_PARENT; nodes],
            direction: vec![NO_DIRECTION; nodes],
            stamp: vec![0; nodes],
            generation: 0,
            history: vec![0; nodes],
            no_via: vec![false; cells],
            vias: rules.autoroute_vias,
            cleanup: None,
            cleanup_passes: 0,
            cleanup_freed: vec![],
            profiles,
            nets,
        };
        job.paint_static(board, &conn);
        job.items = conn.items;
        job
    }

    /// Distance from a node to an obstacle below which the node is unusable for a
    /// profile, so that moves between usable nodes keep `gap` (see the module docs).
    fn keep_out(&self, gap: i32, profile: usize) -> f64 {
        let q = f64::from(gap) + f64::from(self.profiles[profile].size) / 2.;
        let pitch = f64::from(self.grid.pitch);
        (q * q + pitch * pitch / 2.).sqrt() + 1.
    }

    fn next_generation(&mut self) -> u32 {
        if self.generation == u32::MAX {
            self.stamp.fill(0);
            self.generation = 0;
        }
        self.generation += 1;
        self.generation
    }

    fn paint_static(&mut self, board: &Board, conn: &Connectivity) {
        let index: BTreeMap<&str, u32> = self
            .nets
            .iter()
            .enumerate()
            .map(|(i, n)| (n.name.as_str(), i as u32))
            .collect();
        self.owners = conn
            .islands
            .iter()
            .map(|island| {
                conn.island_net(*island)
                    .and_then(|net| index.get(net))
                    .copied()
                    .unwrap_or(BLOCKED)
            })
            .collect();
        let mut owners: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (i, owner) in self.owners.iter().enumerate() {
            owners.entry(*owner).or_default().push(i);
        }
        let grid = self.grid;
        let cells = grid.cells();
        // Vias stay off pads, including pads of their own net: a via in a solder
        // land wicks solder. Large thermal pads opt back in through `Stubs::vias`.
        // One map serves every net, so it is sized for the largest via and its
        // class's own clearance; other nets' pads also keep their pair clearance
        // through the static claims.
        let via_reach = self
            .profiles
            .iter()
            .filter(|p| {
                self.nets
                    .iter()
                    .any(|n| self.profiles[n.via_profile] == **p)
            })
            .map(|p| f64::from(p.size / 2 + self.gaps[p.class][p.class] / 2))
            .fold(0., f64::max);
        for item in conn
            .items
            .iter()
            .filter(|i| matches!(i.owner, CopperRef::Pad { .. }))
        {
            let no_via = &mut self.no_via;
            for_cells_near(&grid, &item.shape, via_reach, |cell| no_via[cell] = true);
        }
        let outside: Vec<bool> = (0..cells)
            .map(|cell| !board.outline_contains(grid.centre(cell)))
            .collect();
        // The approach band around routed pads: as wide as the widest track keeps
        // from copper of its own class, plus `APPROACH`.
        let approach = (0..self.profiles.len())
            .filter(|&p| self.nets.iter().any(|n| n.profile == p))
            .map(|p| self.keep_out(self.gaps[self.profiles[p].class][self.profiles[p].class], p))
            .fold(0., f64::max)
            + f64::from(APPROACH);
        for class in 0..self.profiles.len() {
            let routing = self.profiles[class].class;
            for (&owner, members) in &owners {
                let generation = self.next_generation();
                for &i in members {
                    let item = &conn.items[i];
                    let radius = self.keep_out(self.gaps[routing][self.item_class[i]], class);
                    for layer in item.layers.0..=item.layers.1 {
                        let layer = usize::from(layer);
                        let claims = &mut self.static_claims[class * grid.layers + layer];
                        let stamp = &mut self.stamp;
                        for_cells_near(&grid, &item.shape, radius, |cell| {
                            let node = layer * cells + cell;
                            if stamp[node] != generation {
                                stamp[node] = generation;
                                claims.add(cell, owner);
                            }
                        });
                    }
                }
            }
            if class == 0 {
                let radius = approach;
                for (&owner, members) in owners.iter().filter(|(o, _)| **o != BLOCKED) {
                    let generation = self.next_generation();
                    for &i in members {
                        let item = &conn.items[i];
                        if !matches!(item.owner, CopperRef::Pad { .. }) {
                            continue;
                        }
                        for layer in item.layers.0..=item.layers.1 {
                            let layer = usize::from(layer);
                            let claims = &mut self.approaches[layer];
                            let stamp = &mut self.stamp;
                            for_cells_near(&grid, &item.shape, radius, |cell| {
                                let node = layer * cells + cell;
                                if stamp[node] != generation {
                                    stamp[node] = generation;
                                    claims.add(cell, owner);
                                }
                            });
                        }
                    }
                }
            }
            let edge_radius = self.keep_out(board.rules.edge_clearance, class);
            for layer in 0..grid.layers {
                let claims = &mut self.static_claims[class * grid.layers + layer];
                for (cell, out) in outside.iter().enumerate() {
                    if *out {
                        claims.add(cell, BLOCKED);
                    }
                }
                for (a, b) in board.outline_edges() {
                    for_cells_near(&grid, &Shape::segment(a, b, 0), edge_radius, |cell| {
                        claims.add(cell, BLOCKED)
                    });
                }
            }
        }
    }

    fn map(&self, class: usize, node: usize) -> (usize, usize) {
        let cells = self.grid.cells();
        (class * self.grid.layers + node / cells, node % cells)
    }
    /// Whether a node may carry the net's track, and whether that uses another
    /// net's route (only permitted in soft mode).
    fn legal(&self, net: usize, node: usize, soft: bool) -> Option<bool> {
        let (map, cell) = self.map(self.nets[net].profile, node);
        if !self.static_claims[map].free(cell, net as u32) {
            return None;
        }
        let conflict = !self.route_claims[map].free(cell, net as u32);
        (!conflict || soft).then_some(conflict)
    }
    fn via_legal(&self, net: usize, cell: usize, soft: bool) -> Option<bool> {
        let class = self.nets[net].via_profile;
        let mut conflict = false;
        for layer in 0..self.grid.layers {
            let map = class * self.grid.layers + layer;
            if !self.static_claims[map].free(cell, net as u32) {
                return None;
            }
            conflict |= !self.route_claims[map].free(cell, net as u32);
        }
        (!conflict || soft).then_some(conflict)
    }

    /// Statically legal nodes inside an island's copper. Pads record the stub back to
    /// their centre; a pad with no usable interior node escapes to nearby grid nodes.
    fn seeds(&self, net: usize, island: usize, stubs: &mut Stubs) -> Vec<u32> {
        let cells = self.grid.cells();
        let class = self.nets[net].profile;
        let usable = |node: u32| {
            let (map, cell) = self.map(class, node as usize);
            self.static_claims[map].free(cell, net as u32)
        };
        let mut seeds = vec![];
        for &i in &self.nets[net].islands[island] {
            let item = &self.items[i];
            let is_pad = matches!(item.owner, CopperRef::Pad { .. });
            for layer in item.layers.0..=item.layers.1 {
                let base = usize::from(layer) * cells;
                let mut inside = vec![];
                for_cells_near(&self.grid, &item.shape, 0.5, |cell| {
                    let node = (base + cell) as u32;
                    if usable(node) {
                        inside.push(node);
                    }
                });
                if is_pad {
                    if inside.is_empty() {
                        for (node, stub) in self.escapes(net, item, layer) {
                            stubs.paths.entry(node).or_insert(stub);
                            inside.push(node);
                        }
                    } else {
                        let (min, max) = item.shape.bounds();
                        let diameter = self.nets[net].via.0;
                        let radius = diameter / 2;
                        let large = (max.x - min.x).min(max.y - min.y) >= 3 * diameter;
                        for node in &inside {
                            stubs
                                .paths
                                .entry(*node)
                                .or_insert_with(|| vec![item.shape.center()]);
                            let at = self.grid.centre(*node as usize % cells);
                            if large && disc_inside(&item.shape, at, radius) {
                                stubs.vias.insert(*node);
                            }
                        }
                    }
                }
                seeds.extend(inside);
            }
        }
        seeds.sort_unstable();
        seeds.dedup();
        seeds
    }

    /// Fan-out from a pad whose interior is too tight for the grid: leave the pad
    /// centre along each axis to a grid line past the pad, then jog onto the nearest
    /// grid node. Each stub is checked against exact copper geometry.
    fn escapes(&self, net: usize, pad: &CopperItem, layer: u8) -> Vec<(u32, Vec<Point>)> {
        let grid = self.grid;
        let pitch = i64::from(grid.pitch);
        let width = self.nets[net].width;
        let centre = pad.shape.center();
        let (min, max) = pad.shape.bounds();
        let column = |v: i32, o: i32| (i64::from(v) - i64::from(o)) as f64 / pitch as f64;
        let mut escapes = vec![];
        for (dx, dy) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
            for k in 0..6 {
                let (gx, gy) = if dx != 0 {
                    let edge = column(if dx > 0 { max.x } else { min.x }, grid.origin.x);
                    let gx = if dx > 0 {
                        edge.floor() as i64 + 1 + k
                    } else {
                        edge.ceil() as i64 - 1 - k
                    };
                    (gx, column(centre.y, grid.origin.y).round() as i64)
                } else {
                    let edge = column(if dy > 0 { max.y } else { min.y }, grid.origin.y);
                    let gy = if dy > 0 {
                        edge.floor() as i64 + 1 + k
                    } else {
                        edge.ceil() as i64 - 1 - k
                    };
                    (column(centre.x, grid.origin.x).round() as i64, gy)
                };
                if gx < 0 || gy < 0 || gx >= grid.width as i64 || gy >= grid.height as i64 {
                    break;
                }
                let cell = gy as usize * grid.width + gx as usize;
                let node = (usize::from(layer) * grid.cells() + cell) as u32;
                let target = grid.centre(cell);
                let jog = if dx != 0 {
                    Point::new(target.x, centre.y)
                } else {
                    Point::new(centre.x, target.y)
                };
                let (map, _) = self.map(self.nets[net].profile, node as usize);
                if !self.static_claims[map].free(cell, net as u32) {
                    continue;
                }
                let path = crate::board::simplify_path(vec![centre, jog, target]);
                if path
                    .windows(2)
                    .all(|s| self.clear(net, &Shape::segment(s[0], s[1], width), layer))
                {
                    let mut stub = path;
                    stub.pop();
                    escapes.push((node, stub));
                    break;
                }
            }
        }
        escapes
    }

    /// Exact clearance of new copper against the board edge, other owners' static
    /// copper, and other nets' routes.
    fn clear(&self, net: usize, shape: &Shape, layer: u8) -> bool {
        let gaps = &self.gaps[self.nets[net].class];
        let (min, max) = shape.bounds();
        let near = |other: &Shape| {
            let (a, b) = other.bounds();
            a.x <= max.x.saturating_add(self.max_gap)
                && b.x >= min.x.saturating_sub(self.max_gap)
                && a.y <= max.y.saturating_add(self.max_gap)
                && b.y >= min.y.saturating_sub(self.max_gap)
        };
        let statics = self
            .items
            .iter()
            .zip(&self.owners)
            .zip(&self.item_class)
            .all(|((item, owner), class)| {
                *owner == net as u32
                    || layer < item.layers.0
                    || layer > item.layers.1
                    || !near(&item.shape)
                    || shape.distance(&item.shape) >= f64::from(gaps[*class])
            });
        let inside = crate::board::polygon_contains(&self.outline, shape.center())
            && self.outline_edges().all(|(a, b)| {
                shape.distance(&Shape::segment(a, b, 0)) >= f64::from(self.edge_clearance)
            });
        let routes = self.states.iter().enumerate().all(|(j, state)| {
            let clearance = f64::from(gaps[self.nets[j].class]);
            j == net
                || (state
                    .tracks
                    .iter()
                    .filter(|t| t.layer == layer)
                    .flat_map(|t| t.segments())
                    .all(|other| !near(&other) || shape.distance(&other) >= clearance)
                    && state.vias.iter().all(|v| {
                        let other = Shape::circle(v.at, v.diameter / 2);
                        !near(&other) || shape.distance(&other) >= clearance
                    }))
        });
        statics && inside && routes
    }
    fn outline_edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let n = self.outline.len();
        (0..n).map(move |i| (self.outline[i], self.outline[(i + 1) % n]))
    }

    fn cell_box(&self, nodes: impl Iterator<Item = u32>) -> Option<(i64, i64, i64, i64)> {
        let cells = self.grid.cells();
        nodes.fold(None, |b, node| {
            let cell = node as usize % cells;
            let (x, y) = (
                (cell % self.grid.width) as i64,
                (cell / self.grid.width) as i64,
            );
            Some(match b {
                None => (x, x, y, y),
                Some((x0, x1, y0, y1)) => (x0.min(x), x1.max(x), y0.min(y), y1.max(y)),
            })
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn search(
        &mut self,
        net: usize,
        sources: &[u32],
        targets: &HashMap<u32, usize>,
        boxes: &[(i64, i64, i64, i64)],
        window: Option<(i64, i64, i64, i64)>,
        soft: bool,
        stubs: &Stubs,
        via_cost: Option<u32>,
    ) -> Option<Vec<u32>> {
        let generation = self.next_generation();
        let grid = self.grid;
        let cells = grid.cells();
        let pitch = u32::try_from(grid.pitch).unwrap_or(1);
        let straight = pitch;
        let diagonal = (f64::from(pitch) * std::f64::consts::SQRT_2 * 1.15).round() as u32;
        let diagonal_floor = (f64::from(pitch) * std::f64::consts::SQRT_2).floor() as u32;
        let cross_grain = pitch / 2;
        let bends = [0, pitch, 3 * pitch];
        // Sharing is cheap on the first retry, so a blocked net takes its natural path
        // and displaces the other net; repeated retries make sharing progressively
        // dearer so contested nets settle instead of trading places.
        let conflict_cost = pitch * (1 + 2 * self.attempts[net]);
        let history_cost = pitch.div_ceil(2);
        let approach_cost = 3 * pitch;
        // Paths start and end near pad centres, so stubs stay short and square.
        let stub = |node: usize| {
            stubs.paths.get(&(node as u32)).map_or(0, |points| {
                let mut length = 0.;
                let mut last = grid.centre(node % cells);
                for p in points.iter().rev() {
                    length += f64::from(last.x - p.x).hypot(f64::from(last.y - p.y));
                    last = *p;
                }
                length.round() as u32
            })
        };
        let (x0, x1, y0, y1) =
            window.unwrap_or((0, grid.width as i64 - 1, 0, grid.height as i64 - 1));
        let heuristic = |node: usize| -> u32 {
            let cell = node % cells;
            let (x, y) = ((cell % grid.width) as i64, (cell / grid.width) as i64);
            boxes
                .iter()
                .map(|&(bx0, bx1, by0, by1)| {
                    let dx = (bx0 - x).max(x - bx1).max(0) as u32;
                    let dy = (by0 - y).max(y - by1).max(0) as u32;
                    straight
                        .saturating_mul(dx.max(dy) - dx.min(dy))
                        .saturating_add(diagonal_floor.saturating_mul(dx.min(dy)))
                })
                .min()
                .unwrap_or(0)
        };
        let mut heap = BinaryHeap::new();
        for &source in sources {
            let node = source as usize;
            if self.legal(net, node, soft).is_none() {
                continue;
            }
            let start = stub(node);
            self.stamp[node] = generation;
            self.cost[node] = start;
            self.parent[node] = NO_PARENT;
            self.direction[node] = NO_DIRECTION;
            heap.push(Reverse((start + heuristic(node), start, source)));
        }
        while let Some(Reverse((_, g, node))) = heap.pop() {
            let node = node as usize;
            if g != self.cost[node] {
                continue;
            }
            self.expansions += 1;
            if targets.contains_key(&(node as u32)) {
                let mut path = vec![node as u32];
                while let Some(&last) = path.last() {
                    let parent = self.parent[last as usize];
                    if parent == NO_PARENT {
                        break;
                    }
                    path.push(parent);
                }
                path.reverse();
                return Some(path);
            }
            let (layer, cell) = (node / cells, node % cells);
            let (x, y) = ((cell % grid.width) as i64, (cell / grid.width) as i64);
            let came = self.direction[node];
            let mut relax = |job: &mut Self, next: usize, cost: u32, direction: u8| {
                if job.stamp[next] != generation || cost < job.cost[next] {
                    job.stamp[next] = generation;
                    job.cost[next] = cost;
                    job.parent[next] = node as u32;
                    job.direction[next] = direction;
                    heap.push(Reverse((
                        cost.saturating_add(heuristic(next)),
                        cost,
                        next as u32,
                    )));
                }
            };
            for (d, (dx, dy)) in DIRECTIONS.iter().enumerate() {
                let (nx, ny) = (x + dx, y + dy);
                if nx < x0 || nx > x1 || ny < y0 || ny > y1 {
                    continue;
                }
                let turn = if came == NO_DIRECTION {
                    0
                } else {
                    let t = (d as i64 - i64::from(came)).rem_euclid(8);
                    t.min(8 - t) as usize
                };
                if turn > 2 {
                    continue;
                }
                let next = layer * cells + ny as usize * grid.width + nx as usize;
                let Some(conflict) = self.legal(net, next, soft) else {
                    continue;
                };
                let mut step = if d % 2 == 1 {
                    diagonal
                } else if grid.layers > 1 && (d % 4 == 0) != (layer % 2 == 0) {
                    straight + cross_grain
                } else {
                    straight
                };
                step += bends[turn] + u32::from(self.history[next]) * history_cost;
                if !self.approaches[layer].free(next % cells, net as u32) {
                    step += approach_cost;
                }
                if conflict {
                    step += conflict_cost;
                }
                if targets.contains_key(&(next as u32)) {
                    step += stub(next);
                }
                relax(self, next, g.saturating_add(step), d as u8);
            }
            if grid.layers > 1
                && let Some(via_cost) = via_cost
                && (stubs.vias.contains(&(node as u32))
                    || (!stubs.paths.contains_key(&(node as u32)) && !self.no_via[cell]))
                && let Some(via_conflict) = self.via_legal(net, cell, soft)
            {
                for other in (0..grid.layers).filter(|l| *l != layer) {
                    let next = other * cells + cell;
                    let Some(conflict) = self.legal(net, next, soft) else {
                        continue;
                    };
                    let mut step =
                        via_cost.saturating_add(u32::from(self.history[next]) * history_cost);
                    if conflict || via_conflict {
                        step = step.saturating_add(conflict_cost);
                    }
                    relax(self, next, g.saturating_add(step), NO_DIRECTION);
                }
            }
        }
        None
    }

    /// Nets whose route claims a soft path overlaps.
    fn blockers(&self, net: usize, path: &[u32]) -> BTreeSet<usize> {
        let cells = self.grid.cells();
        let via_class = self.nets[net].via_profile;
        let mut keys = vec![];
        for (i, &node) in path.iter().enumerate() {
            let (map, cell) = self.map(self.nets[net].profile, node as usize);
            keys.push((map * cells + cell) as u64);
            if i > 0 && path[i - 1] as usize % cells == cell {
                for layer in 0..self.grid.layers {
                    keys.push(((via_class * self.grid.layers + layer) * cells + cell) as u64);
                }
            }
        }
        self.states
            .iter()
            .enumerate()
            .filter(|(j, state)| *j != net && keys.iter().any(|k| state.claimed.contains(k)))
            .map(|(j, _)| j)
            .collect()
    }

    fn rip_up(&mut self, net: usize) {
        let cells = self.grid.cells();
        let state = std::mem::take(&mut self.states[net]);
        for key in state.claimed {
            let key = key as usize;
            self.route_claims[key / cells].remove(key % cells, net as u32);
        }
        self.rip_ups += 1;
        self.queue.push_back(net);
    }

    fn claim(&mut self, net: usize, shape: &Shape, layers: (usize, usize)) {
        let grid = self.grid;
        let cells = grid.cells();
        for class in 0..self.profiles.len() {
            let gap = self.gaps[self.profiles[class].class][self.nets[net].class];
            let radius = self.keep_out(gap, class);
            for layer in layers.0..=layers.1 {
                let map = class * grid.layers + layer;
                let claims = &mut self.route_claims[map];
                let claimed = &mut self.states[net].claimed;
                for_cells_near(&grid, shape, radius, |cell| {
                    if claimed.insert((map * cells + cell) as u64) {
                        claims.add(cell, net as u32);
                    }
                });
            }
        }
    }

    /// Turn a grid path into tracks and vias and claim their clearance zones.
    fn commit(&mut self, net: usize, path: &[u32], stubs: &Stubs) {
        let cells = self.grid.cells();
        let width = self.nets[net].width;
        let mut runs = vec![];
        let mut vias = vec![];
        let mut points = vec![];
        points.extend(stubs.paths.get(&path[0]).into_iter().flatten().copied());
        for (i, &node) in path.iter().enumerate() {
            let (layer, cell) = (node as usize / cells, node as usize % cells);
            if i > 0 && path[i - 1] as usize % cells == cell {
                runs.push((path[i - 1] as usize / cells, std::mem::take(&mut points)));
                vias.push(self.grid.centre(cell));
            }
            points.push(self.grid.centre(cell));
            if i + 1 == path.len() {
                points.extend(stubs.paths.get(&node).into_iter().flatten().rev().copied());
                runs.push((layer, std::mem::take(&mut points)));
            }
        }
        let last_layer = self.grid.layers - 1;
        for (layer, points) in runs {
            let track = Track::new(layer as u8, width, points);
            if track.points.len() < 2 {
                continue;
            }
            let segments: Vec<Shape> = track.segments().collect();
            for segment in &segments {
                self.claim(net, segment, (layer, layer));
            }
            self.states[net].tracks.push(track);
        }
        let (diameter, drill) = self.nets[net].via;
        for at in vias {
            self.claim(net, &Shape::circle(at, diameter / 2), (0, last_layer));
            self.states[net].vias.push(Via {
                id: Uuid::new_v4(),
                at,
                diameter,
                drill,
            });
        }
    }

    /// What a via costs when one may be placed.
    fn via_cost(&self) -> u32 {
        (20 * u32::try_from(self.grid.pitch).unwrap_or(1)).max(2000)
    }

    /// Route one net. `cleanup` reroutes a finished net for fewer vias; it never rips
    /// up another net.
    fn route_net(&mut self, net: usize, cleanup: bool) {
        self.states[net].missing = 0;
        self.states[net].routed = true;
        let islands = self.nets[net].islands.len();
        let mut stubs = Stubs::default();
        let seeds: Vec<Vec<u32>> = (0..islands)
            .map(|i| self.seeds(net, i, &mut stubs))
            .collect();
        let mut connected = vec![false; islands];
        connected[0] = true;
        let mut tree = seeds[0].clone();
        for remaining in (1..islands).rev() {
            let mut targets = HashMap::new();
            let mut boxes = vec![];
            for (island, nodes) in seeds.iter().enumerate().filter(|(i, _)| !connected[*i]) {
                boxes.extend(self.cell_box(nodes.iter().copied()));
                for &node in nodes {
                    targets.insert(node, island);
                }
            }
            if boxes.len() > 16
                && let Some(all) = self.cell_box(targets.keys().copied())
            {
                boxes = vec![all];
            }
            let window = self
                .cell_box(tree.iter().chain(targets.keys()).copied())
                .map(|(x0, x1, y0, y1)| {
                    let pad = 40.max((x1 - x0).max(y1 - y0) / 3);
                    (x0 - pad, x1 + pad, y0 - pad, y1 + pad)
                });
            // Via costs to try in order; `None` searches without vias. A clean-up
            // reroute looks for a route with no via before accepting one.
            let allow = Some(self.via_cost());
            let stages = match self.vias {
                ViaPolicy::Never => vec![None],
                ViaPolicy::Avoid if cleanup => vec![None, allow],
                ViaPolicy::Allow | ViaPolicy::Avoid => vec![allow],
            };
            let mut path = None;
            for &vias in &stages {
                path = self.search(net, &tree, &targets, &boxes, window, false, &stubs, vias);
                if path.is_none() {
                    path = self.search(net, &tree, &targets, &boxes, None, false, &stubs, vias);
                }
                if path.is_some() {
                    break;
                }
            }
            let soft_vias = *stages.last().unwrap();
            if path.is_none()
                && !cleanup
                && self.attempts[net] < MAX_ATTEMPTS
                && self.rip_ups < self.rip_up_limit
                && let Some(soft) =
                    self.search(net, &tree, &targets, &boxes, None, true, &stubs, soft_vias)
            {
                self.attempts[net] += 1;
                for &node in &soft {
                    if self.legal(net, node as usize, false).is_none() {
                        let h = &mut self.history[node as usize];
                        *h = h.saturating_add(1);
                    }
                }
                for blocker in self.blockers(net, &soft) {
                    self.rip_up(blocker);
                }
                path = Some(soft);
            }
            let Some(path) = path else {
                self.states[net].missing = remaining;
                return;
            };
            let reached = targets[path.last().unwrap()];
            self.commit(net, &path, &stubs);
            connected[reached] = true;
            tree.extend(path);
            tree.extend(seeds[reached].iter().copied());
        }
    }

    /// Reroute a finished net against the board as it now stands. The new copper is
    /// kept only if it still connects every island with fewer vias; otherwise the
    /// old copper and its claims are put back exactly.
    fn reduce_vias(&mut self, net: usize) {
        let before = self.states[net].vias.len();
        if before == 0 || self.states[net].missing > 0 {
            return;
        }
        let cells = self.grid.cells();
        let old = std::mem::take(&mut self.states[net]);
        for &key in &old.claimed {
            let key = key as usize;
            self.route_claims[key / cells].remove(key % cells, net as u32);
        }
        self.route_net(net, true);
        let new = &self.states[net];
        if new.missing == 0 && new.vias.len() < before {
            if let Some(region) = self.copper_region(&old) {
                self.cleanup_freed.push(region);
            }
            return;
        }
        let new = std::mem::replace(&mut self.states[net], old);
        for key in new.claimed {
            let key = key as usize;
            self.route_claims[key / cells].remove(key % cells, net as u32);
        }
        for &key in &self.states[net].claimed {
            let key = key as usize;
            self.route_claims[key / cells].add(key % cells, net as u32);
        }
    }

    /// Bounds of a net's routed copper, grown by the margin a search window adds.
    fn copper_region(&self, state: &NetState) -> Option<(Point, Point)> {
        let points = state
            .tracks
            .iter()
            .flat_map(|t| t.points.iter().copied())
            .chain(state.vias.iter().map(|v| v.at));
        let (min, max) = points.fold(None, |b: Option<(Point, Point)>, p| {
            Some(b.map_or((p, p), |(a, b)| {
                (
                    Point::new(a.x.min(p.x), a.y.min(p.y)),
                    Point::new(b.x.max(p.x), b.y.max(p.y)),
                )
            }))
        })?;
        let margin = 40 * self.grid.pitch + (max.x - min.x).max(max.y - min.y) / 3;
        Some((
            Point::new(min.x - margin, min.y - margin),
            Point::new(max.x + margin, max.y + margin),
        ))
    }

    /// Whether the via clean-up that follows routing under Avoid has more to do.
    fn cleanup_pending(&self) -> bool {
        self.vias == ViaPolicy::Avoid
            && match &self.cleanup {
                None => true,
                Some(queue) => {
                    !queue.is_empty()
                        || (!self.cleanup_freed.is_empty()
                            && self.cleanup_passes < MAX_CLEANUP_PASSES)
                }
            }
    }

    /// The next net to reroute for fewer vias, starting a pass when one is due.
    fn next_cleanup(&mut self) -> Option<usize> {
        if !self.cleanup_pending() {
            return None;
        }
        if self.cleanup.as_ref().is_none_or(VecDeque::is_empty) {
            self.cleanup_passes += 1;
            let freed = std::mem::take(&mut self.cleanup_freed);
            let first = self.cleanup.is_none();
            let near = |region: Option<(Point, Point)>| {
                region.is_some_and(|(a, b)| {
                    freed
                        .iter()
                        .any(|(c, d)| a.x <= d.x && c.x <= b.x && a.y <= d.y && c.y <= b.y)
                })
            };
            self.cleanup = Some(
                (0..self.nets.len())
                    .filter(|&i| !self.states[i].vias.is_empty() && self.states[i].missing == 0)
                    .filter(|&i| first || near(self.copper_region(&self.states[i])))
                    .collect(),
            );
        }
        self.cleanup.as_mut()?.pop_front()
    }

    /// Route queued nets until roughly `budget` grid nodes have been expanded.
    pub fn step(&mut self, budget: u64) -> Progress {
        let stop = self.expansions.saturating_add(budget);
        while self.expansions < stop {
            if let Some(net) = self.queue.pop_front() {
                self.route_net(net, false);
            } else if let Some(net) = self.next_cleanup() {
                self.reduce_vias(net);
            } else {
                break;
            }
        }
        self.progress()
    }
    pub fn progress(&self) -> Progress {
        let queued: BTreeSet<usize> = self.queue.iter().copied().collect();
        let settled = |i: &usize| !queued.contains(i) && self.states[*i].routed;
        Progress {
            total_nets: self.nets.len(),
            completed_nets: (0..self.nets.len())
                .filter(|i| settled(i) && self.states[*i].missing == 0)
                .count(),
            failed_nets: (0..self.nets.len())
                .filter(|i| settled(i) && self.states[*i].missing > 0)
                .count(),
            rip_ups: self.rip_ups,
            removing_vias: self.queue.is_empty()
                && self.cleanup.is_some()
                && self.cleanup_pending(),
            done: self.queue.is_empty() && !self.cleanup_pending(),
        }
    }
    /// Copper routed so far, for previews.
    pub fn routed(&self) -> impl Iterator<Item = (&[Track], &[Via])> {
        self.states
            .iter()
            .map(|s| (s.tracks.as_slice(), s.vias.as_slice()))
    }
    pub fn pitch(&self) -> i32 {
        self.grid.pitch
    }
    pub fn run(mut self) -> RouteOutcome {
        while !self.step(u64::MAX).done {}
        self.outcome()
    }
    pub fn outcome(&self) -> RouteOutcome {
        let queued: BTreeSet<usize> = self.queue.iter().copied().collect();
        RouteOutcome {
            tracks: self.states.iter().flat_map(|s| s.tracks.clone()).collect(),
            vias: self.states.iter().flat_map(|s| s.vias.clone()).collect(),
            unrouted: self
                .nets
                .iter()
                .enumerate()
                .filter(|(i, _)| queued.contains(i) || self.states[*i].missing > 0)
                .map(|(_, n)| n.name.clone())
                .collect(),
            pitch: self.grid.pitch,
        }
    }
}

impl Document {
    /// Route every unrouted connection and add the copper to the board.
    pub fn autoroute(&mut self) -> RouteOutcome {
        let outcome = RouteJob::new(&self.board, &self.netlist()).run();
        outcome.clone().apply(&mut self.board);
        outcome
    }
}

