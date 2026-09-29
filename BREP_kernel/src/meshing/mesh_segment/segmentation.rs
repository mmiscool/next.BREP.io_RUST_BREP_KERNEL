use super::*;

struct RegionOutcome {
    fit: CandidateFit,
    accepted: bool,
    area: f64,
    bbox: (Vec3, Vec3),
    triangle_count: usize,
}

/// Try the analytic cascade plane → cylinder → cone → sphere → torus on one
/// region; the first carrier within tolerance wins.
fn fit_region(data: &MeshData, tri_ids: &[u32], options: &SegmentOptions) -> RegionOutcome {
    let vert_ids = region_vertices(data, tri_ids);
    let bbox = region_bbox(data, &vert_ids);
    let area: f64 = tri_ids.iter().map(|&t| data.tris[t as usize].area).sum();
    let scale = {
        let diag = bbox.1.sub(bbox.0).length();
        if diag > 0.0 {
            diag
        } else {
            data.diag.max(1e-12)
        }
    };
    let tol_abs = options.fit_tolerance * scale;
    let mut outcome = RegionOutcome {
        fit: CandidateFit {
            carrier: RegionCarrier::Freeform,
            max_dev: 0.0,
            rms_dev: 0.0,
            max_normal_angle_deg: 0.0,
        },
        accepted: false,
        area,
        bbox,
        triangle_count: tri_ids.len(),
    };
    if tri_ids.len() < options.min_region_triangles || vert_ids.len() < 3 {
        return outcome;
    }
    let candidates = [
        fit_plane(data, tri_ids, &vert_ids),
        fit_cylinder(data, tri_ids, &vert_ids),
        fit_cone(data, tri_ids, &vert_ids, tol_abs),
        fit_sphere(data, tri_ids, &vert_ids),
        fit_torus(data, tri_ids, &vert_ids),
    ];
    for candidate in candidates {
        if let Some(fit) = candidate {
            if std::env::var("BREP_DEBUG_SEG").is_ok() {
                eprintln!(
                    "[seg] region tris={} candidate={} max_dev={:.3e} tol_abs={:.3e} max_ang={:.3}",
                    tri_ids.len(),
                    fit.carrier.kind(),
                    fit.max_dev,
                    tol_abs,
                    fit.max_normal_angle_deg
                );
            }
            if fit.max_dev.is_finite()
                && fit.max_dev <= tol_abs
                && fit.max_normal_angle_deg <= options.normal_tolerance_deg
            {
                outcome.fit = fit;
                outcome.accepted = true;
                return outcome;
            }
        }
    }
    outcome
}

// ---------------------------------------------------------------------------
// Region growing and refinement
// ---------------------------------------------------------------------------

fn grow_regions(data: &MeshData, options: &SegmentOptions) -> Vec<Vec<u32>> {
    let cos_gate = options.deflection_angle_deg.to_radians().cos();
    let tri_count = data.tris.len();
    let mut assigned = vec![false; tri_count];
    let mut regions = Vec::new();
    let mut stack = Vec::new();
    for seed in 0..tri_count {
        if assigned[seed] || data.tris[seed].skip {
            continue;
        }
        assigned[seed] = true;
        stack.push(seed as u32);
        let mut region = Vec::new();
        while let Some(t) = stack.pop() {
            region.push(t);
            let tri = &data.tris[t as usize];
            for corner in 0..3 {
                let first = tri.verts[corner];
                let second = tri.verts[(corner + 1) % 3];
                if first == second {
                    continue;
                }
                let Some(incident) = data.edges.get(&edge_key(first, second)) else {
                    continue;
                };
                // Only manifold (two-sided) edges are smooth crossings.
                if incident.len() != 2 {
                    continue;
                }
                for &other in incident {
                    if other == t || assigned[other as usize] {
                        continue;
                    }
                    let neighbor = &data.tris[other as usize];
                    if neighbor.skip {
                        continue;
                    }
                    if tri.normal.dot(neighbor.normal) >= cos_gate {
                        assigned[other as usize] = true;
                        stack.push(other);
                    }
                }
            }
        }
        region.sort_unstable();
        regions.push(region);
    }
    regions
}

/// Regions whose area is below this fraction of the total mesh area are
/// dissolved into their largest neighboring region before fitting.  Needle
/// triangles at tessellation poles wall themselves off into 1–2-triangle
/// islands; the app's minimum-region-size gating served the same purpose.
const REGION_ABSORB_AREA_FRACTION: f64 = 1e-6;

/// Merge negligible regions into their largest-area neighbor (repeats until
/// stable so chains of islands resolve).  Regions with no neighbor at all
/// stay as they are.
fn absorb_tiny_regions(data: &MeshData, regions: &mut Vec<Vec<u32>>) {
    let total_area: f64 = data
        .tris
        .iter()
        .filter(|tri| !tri.skip)
        .map(|tri| tri.area)
        .sum();
    let floor = total_area * REGION_ABSORB_AREA_FRACTION;
    if !(floor > 0.0) {
        return;
    }
    let mut region_of = vec![usize::MAX; data.tris.len()];
    for (r, list) in regions.iter().enumerate() {
        for &t in list {
            region_of[t as usize] = r;
        }
    }
    let mut areas: Vec<f64> = regions
        .iter()
        .map(|list| list.iter().map(|&t| data.tris[t as usize].area).sum())
        .collect();
    let mut changed = true;
    while changed {
        changed = false;
        for r in 0..regions.len() {
            if regions[r].is_empty() || areas[r] >= floor {
                continue;
            }
            // Find adjacent regions, traversing THROUGH region-less
            // (skipped sliver) triangles — pole islands are often fenced
            // off from the main region by a ring of skipped needles.
            let mut target: Option<usize> = None;
            let mut visited: rustc_hash::FxHashSet<u32> = regions[r].iter().copied().collect();
            let mut frontier: Vec<u32> = regions[r].clone();
            while let Some(t) = frontier.pop() {
                let tri = &data.tris[t as usize];
                for corner in 0..3 {
                    let first = tri.verts[corner];
                    let second = tri.verts[(corner + 1) % 3];
                    if first == second {
                        continue;
                    }
                    if let Some(incident) = data.edges.get(&edge_key(first, second)) {
                        for &other in incident {
                            if visited.contains(&other) {
                                continue;
                            }
                            let neighbor = region_of[other as usize];
                            if neighbor == usize::MAX {
                                // Skipped triangle: pass through it.
                                visited.insert(other);
                                frontier.push(other);
                                continue;
                            }
                            if neighbor == r {
                                continue;
                            }
                            target = match target {
                                None => Some(neighbor),
                                Some(current) => Some(if areas[neighbor] > areas[current] {
                                    neighbor
                                } else {
                                    current
                                }),
                            };
                        }
                    }
                }
            }
            if let Some(target) = target {
                let moved = std::mem::take(&mut regions[r]);
                for &t in &moved {
                    region_of[t as usize] = target;
                }
                areas[target] += areas[r];
                areas[r] = 0.0;
                regions[target].extend(moved);
                changed = true;
            }
        }
    }
    regions.retain(|list| !list.is_empty());
    for list in regions.iter_mut() {
        list.sort_unstable();
    }
}

/// Connected components of `tri_ids` across shared (welded) edges,
/// restricted to the given set.
fn connected_components(data: &MeshData, tri_ids: &[u32]) -> Vec<Vec<u32>> {
    let member: rustc_hash::FxHashSet<u32> = tri_ids.iter().copied().collect();
    let mut assigned: rustc_hash::FxHashSet<u32> = rustc_hash::FxHashSet::default();
    let mut components = Vec::new();
    let mut stack = Vec::new();
    for &seed in tri_ids {
        if assigned.contains(&seed) {
            continue;
        }
        assigned.insert(seed);
        stack.push(seed);
        let mut component = Vec::new();
        while let Some(t) = stack.pop() {
            component.push(t);
            let tri = &data.tris[t as usize];
            for corner in 0..3 {
                let first = tri.verts[corner];
                let second = tri.verts[(corner + 1) % 3];
                if first == second {
                    continue;
                }
                if let Some(incident) = data.edges.get(&edge_key(first, second)) {
                    for &other in incident {
                        if other != t && member.contains(&other) && !assigned.contains(&other) {
                            assigned.insert(other);
                            stack.push(other);
                        }
                    }
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }
    components
}

/// Split a region the cascade could not classify: peel tangent cylinders,
/// cones and spheres out of it first (the most specific test — a group
/// must lie on ONE carrier at the mesh's precision), then seed-plane-
/// anchored coplanar groups (the app's planar extraction) out of what is
/// left, and re-fit the remaining connected components.  Planes come second
/// because a coarse strip of a cylinder is a coplanar group too, and one
/// wide enough to pass the planar area gate would otherwise be peeled as a
/// plane before the cylinder it belongs to was ever tried.  Returns the
/// partition with each piece's fit outcome.
fn refine_region(
    data: &MeshData,
    tri_ids: &[u32],
    options: &SegmentOptions,
) -> Vec<(Vec<u32>, RegionOutcome)> {
    let vert_ids = region_vertices(data, tri_ids);
    let (low, high) = region_bbox(data, &vert_ids);
    let scale = {
        let diag = high.sub(low).length();
        if diag > 0.0 {
            diag
        } else {
            data.diag.max(1e-12)
        }
    };
    let dist_gate = options.fit_tolerance * scale;
    let cos_gate = options.planar_extraction_angle_deg.to_radians().cos();
    let region_area: f64 = tri_ids.iter().map(|&t| data.tris[t as usize].area).sum();
    let min_group_area = region_area * (options.planar_min_area_percent / 100.0).clamp(0.0, 1.0);
    let min_group_tris = options.min_region_triangles.max(2);

    let (mut pieces, leftover) = peel_carriers(data, tri_ids, options);
    let member: rustc_hash::FxHashSet<u32> = leftover.iter().copied().collect();
    let mut order: Vec<u32> = leftover.clone();
    order.sort_by(|&a, &b| {
        data.tris[b as usize]
            .area
            .total_cmp(&data.tris[a as usize].area)
            .then(a.cmp(&b))
    });

    let mut group_of: HashMap<u32, usize> = HashMap::default();
    let mut planar_groups: Vec<Vec<u32>> = Vec::new();
    let mut stack = Vec::new();
    for &seed in &order {
        if group_of.contains_key(&seed) {
            continue;
        }
        let seed_tri = &data.tris[seed as usize];
        let seed_normal = seed_tri.normal;
        let seed_origin = seed_tri.centroid;
        let coplanar = |t: u32| -> bool {
            let tri = &data.tris[t as usize];
            if tri.normal.dot(seed_normal) < cos_gate {
                return false;
            }
            tri.verts
                .iter()
                .all(|&v| data.verts[v].sub(seed_origin).dot(seed_normal).abs() <= dist_gate)
        };
        if !coplanar(seed) {
            continue;
        }
        // Grow the seed plane over not-yet-grouped region triangles.
        let mut visited: rustc_hash::FxHashSet<u32> = rustc_hash::FxHashSet::default();
        visited.insert(seed);
        stack.push(seed);
        let mut group = Vec::new();
        while let Some(t) = stack.pop() {
            group.push(t);
            let tri = &data.tris[t as usize];
            for corner in 0..3 {
                let first = tri.verts[corner];
                let second = tri.verts[(corner + 1) % 3];
                if first == second {
                    continue;
                }
                if let Some(incident) = data.edges.get(&edge_key(first, second)) {
                    for &other in incident {
                        if other == t
                            || !member.contains(&other)
                            || group_of.contains_key(&other)
                            || visited.contains(&other)
                        {
                            continue;
                        }
                        if coplanar(other) {
                            visited.insert(other);
                            stack.push(other);
                        }
                    }
                }
            }
        }
        let group_area: f64 = group.iter().map(|&t| data.tris[t as usize].area).sum();
        if group.len() >= min_group_tris && group_area >= min_group_area {
            let id = planar_groups.len();
            for &t in &group {
                group_of.insert(t, id);
            }
            group.sort_unstable();
            planar_groups.push(group);
        }
    }

    let remainder: Vec<u32> = leftover
        .iter()
        .copied()
        .filter(|t| !group_of.contains_key(t))
        .collect();

    for group in planar_groups {
        let outcome = fit_region(data, &group, options);
        pieces.push((group, outcome));
    }
    for component in connected_components(data, &remainder) {
        let outcome = fit_region(data, &component, options);
        pieces.push((component, outcome));
    }
    pieces
}

/// Precision bar of the cylinder peel as a fraction of the compound
/// region's scale: far below the acceptance tolerance (so the first strip
/// past a tangent junction is refused) and far above the vertex noise of an
/// exact tessellation.
const CYLINDER_PEEL_PRECISION_FRACTION: f64 = 1e-5;
/// Floor of that bar from the coordinate magnitude: single-precision
/// sources quantize at a few ulps of their largest coordinate.
const CYLINDER_PEEL_COORDINATE_FRACTION: f64 = 1e-6;
/// Adjacent triangles whose normals agree within this angle are one planar
/// facet (a tessellated strip) and are peeled together.
const CYLINDER_PEEL_FACET_ANGLE_RAD: f64 = 1e-4;
/// Rings of facets gathered around a seed facet before the seed fit: three
/// rows at least, because two rows of cells lie on two coaxial circles,
/// which every revolve fits exactly.
const CYLINDER_PEEL_SEED_RINGS: usize = 3;

/// Carrier kinds the peel tries, in cascade order, with the smallest vertex
/// support that makes a claim at the precision bar evidence rather than an
/// interpolation: three times the carrier's degrees of freedom (a cylinder
/// has five, a cone six, a sphere four).  Each pass works on what the
/// previous ones left.
const PEEL_KINDS: [(&str, usize); 4] =
    [("cylinder", 15), ("cone", 18), ("sphere", 12), ("torus", 21)];

/// Peel single-carrier groups off a tangent-smooth compound the cascade
/// could not classify: a rounded outline (arcs of different radii joined on
/// rulings), a fillet running out into a boss, a dome sitting on a chamfer
/// ring.
///
/// The acceptance tolerance is far too loose to separate tangent surfaces:
/// at the junction of two arcs the first strip on the far side deviates by
/// a fraction of the acceptance bar and the normal gate admits several
/// more.  Growth therefore runs at a precision bar and is judged by the
/// re-fit residual of the grown group: a wave of neighboring facets is kept
/// when the joint fit still sits at precision, otherwise each facet is
/// tried alone.  Facets — coplanar triangle groups, the strips of a
/// tessellated cylinder — move as a whole so a region boundary never runs
/// down a strip's diagonal.  Each finished group is then accepted through
/// the ordinary cascade at the user tolerance.  Cylinders are peeled
/// first, then cones, then spheres.  Returns the pieces and the triangles
/// no carrier claimed.
fn peel_carriers(
    data: &MeshData,
    tri_ids: &[u32],
    options: &SegmentOptions,
) -> (Vec<(Vec<u32>, RegionOutcome)>, Vec<u32>) {
    let debug = std::env::var("BREP_DEBUG_SEG").is_ok();
    let local: HashMap<u32, usize> = tri_ids
        .iter()
        .enumerate()
        .map(|(index, &t)| (t, index))
        .collect();
    let vert_ids = region_vertices(data, tri_ids);
    let (low, high) = region_bbox(data, &vert_ids);
    let scale = {
        let diag = high.sub(low).length();
        if diag > 0.0 {
            diag
        } else {
            data.diag.max(1e-12)
        }
    };
    let accept_tol = options.fit_tolerance * scale;
    let coordinate = vert_ids
        .iter()
        .map(|&v| {
            let p = data.verts[v];
            p.x.abs().max(p.y.abs()).max(p.z.abs())
        })
        .fold(0.0_f64, f64::max);
    let bar = (scale * CYLINDER_PEEL_PRECISION_FRACTION)
        .max(coordinate * CYLINDER_PEEL_COORDINATE_FRACTION);
    if bar >= accept_tol {
        return (Vec::new(), tri_ids.to_vec());
    }
    let cos_normal_gate = options.normal_tolerance_deg.to_radians().cos();
    let cos_facet = CYLINDER_PEEL_FACET_ANGLE_RAD.cos();

    // Facets: union of coplanar edge-adjacent member triangles.
    let mut parent: Vec<usize> = (0..tri_ids.len()).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); tri_ids.len()];
    for (index, &t) in tri_ids.iter().enumerate() {
        let tri = &data.tris[t as usize];
        for corner in 0..3 {
            let first = tri.verts[corner];
            let second = tri.verts[(corner + 1) % 3];
            if first == second {
                continue;
            }
            let Some(incident) = data.edges.get(&edge_key(first, second)) else {
                continue;
            };
            for &other in incident {
                let Some(&other_index) = local.get(&other) else {
                    continue;
                };
                if other_index == index {
                    continue;
                }
                adjacency[index].push(other_index);
                if tri.normal.dot(data.tris[other as usize].normal) >= cos_facet {
                    let a = find(&mut parent, index);
                    let b = find(&mut parent, other_index);
                    if a != b {
                        parent[a] = b;
                    }
                }
            }
        }
    }
    let mut facet_of = vec![usize::MAX; tri_ids.len()];
    let mut facets: Vec<Vec<usize>> = Vec::new();
    for index in 0..tri_ids.len() {
        let root = find(&mut parent, index);
        if facet_of[root] == usize::MAX {
            facet_of[root] = facets.len();
            facets.push(Vec::new());
        }
        facet_of[index] = facet_of[root];
        facets[facet_of[index]].push(index);
    }
    let mut facet_neighbors: Vec<Vec<usize>> = vec![Vec::new(); facets.len()];
    for (index, others) in adjacency.iter().enumerate() {
        for &other in others {
            let (a, b) = (facet_of[index], facet_of[other]);
            if a != b && !facet_neighbors[a].contains(&b) {
                facet_neighbors[a].push(b);
            }
        }
    }
    let facet_area: Vec<f64> = facets
        .iter()
        .map(|facet| {
            facet
                .iter()
                .map(|&index| data.tris[tri_ids[index] as usize].area)
                .sum()
        })
        .collect();

    let facet_tris = |facet_ids: &[usize]| -> Vec<u32> {
        facet_ids
            .iter()
            .flat_map(|&facet| facets[facet].iter().map(|&index| tri_ids[index]))
            .collect()
    };
    let fit_kind = |kind: &str, facet_ids: &[usize]| -> Option<CandidateFit> {
        let tris = facet_tris(facet_ids);
        let verts = region_vertices(data, &tris);
        match kind {
            "cylinder" => fit_cylinder(data, &tris, &verts),
            "cone" => fit_cone(data, &tris, &verts, accept_tol),
            "sphere" => fit_sphere(data, &tris, &verts),
            "torus" => fit_torus_refined(data, &tris, &verts, 10.0 * accept_tol),
            _ => None,
        }
    };
    let fit_facets = |kind: &str, facet_ids: &[usize]| -> Option<CandidateFit> {
        fit_kind(kind, facet_ids).filter(|fit| fit.max_dev.is_finite() && fit.max_dev <= bar)
    };
    // A growing torus refits from its own current carrier: the cold fit
    // (normal-line axis, then a vertex refinement) per wave is what made a
    // dense fillet band cost a minute.
    let grow_fit = |kind: &str, facet_ids: &[usize], carrier: &RegionCarrier| -> Option<CandidateFit> {
        if kind != "torus" {
            return fit_facets(kind, facet_ids);
        }
        let tris = facet_tris(facet_ids);
        let verts = region_vertices(data, &tris);
        refine_torus_carrier(data, &tris, &verts, carrier, TORUS_REFINE_ITERATIONS)
            .filter(|fit| fit.max_dev.is_finite() && fit.max_dev <= bar)
    };
    // Loose pre-filter against the current carrier: the refit decides.
    let plausible = |carrier: &RegionCarrier, facet: usize| -> bool {
        let sense = carrier_sense(carrier);
        facets[facet].iter().all(|&index| {
            let tri = &data.tris[tri_ids[index] as usize];
            let Some(normal) = carrier_normal(carrier, tri.centroid) else {
                return false;
            };
            tri.normal.dot(normal) * sense >= cos_normal_gate
                && tri
                    .verts
                    .iter()
                    .all(|&v| carrier_distance(carrier, data.verts[v]) <= accept_tol)
        })
    };

    let mut order: Vec<usize> = (0..facets.len()).collect();
    order.sort_by(|&a, &b| facet_area[b].total_cmp(&facet_area[a]).then(a.cmp(&b)));
    let mut assigned = vec![false; facets.len()];
    let mut pieces: Vec<(Vec<u32>, RegionOutcome)> = Vec::new();
    // A torus seed is tried once across all rounds: the rounds exist for a
    // one-strip wall whose seed neighbourhood clears once its dome is
    // claimed, and a torus band that failed once does not become one.
    let mut torus_tried = vec![false; facets.len()];
    // Rounds until nothing new is claimed: a wall one strip tall has its
    // seed neighborhood polluted by the dome or fillet beyond its rim until
    // that neighbor has been claimed by its own pass.
    loop {
    let claimed = pieces.len();
    for (kind, min_vertices) in PEEL_KINDS {
    let mut tried = vec![false; facets.len()];
    for &seed in &order {
        if assigned[seed] || tried[seed] || (kind == "torus" && torus_tried[seed]) {
            continue;
        }
        tried[seed] = true;
        if kind == "torus" {
            torus_tried[seed] = true;
        }
        let mut group = vec![seed];
        let mut in_group = vec![false; facets.len()];
        in_group[seed] = true;
        let mut frontier = vec![seed];
        for _ in 0..CYLINDER_PEEL_SEED_RINGS {
            let mut next = Vec::new();
            for &facet in &frontier {
                for &other in &facet_neighbors[facet] {
                    if !assigned[other] && !in_group[other] {
                        in_group[other] = true;
                        group.push(other);
                        next.push(other);
                    }
                }
            }
            frontier = next;
        }
        let Some(seed_fit) = fit_facets(kind, &group) else {
            continue;
        };
        let mut carrier = seed_fit.carrier;
        let mut blocked = vec![false; facets.len()];
        loop {
            let mut wave = Vec::new();
            for &facet in &group {
                for &other in &facet_neighbors[facet] {
                    if !assigned[other]
                        && !in_group[other]
                        && !blocked[other]
                        && !wave.contains(&other)
                        && plausible(&carrier, other)
                    {
                        wave.push(other);
                    }
                }
            }
            if wave.is_empty() {
                break;
            }
            let mut trial = group.clone();
            trial.extend(wave.iter().copied());
            if let Some(fit) = grow_fit(kind, &trial, &carrier) {
                for &facet in &wave {
                    in_group[facet] = true;
                }
                group = trial;
                carrier = fit.carrier;
                continue;
            }
            // The wave crossed a junction: admit its facets one at a time.
            // A torus admits the facets its current carrier already holds at
            // the bar, then refits once: a refit per junction facet is what
            // a dense freeform compound cannot afford.
            if kind == "torus" {
                let holds = |facet: usize| {
                    facets[facet].iter().all(|&index| {
                        data.tris[tri_ids[index] as usize]
                            .verts
                            .iter()
                            .all(|&v| carrier_distance(&carrier, data.verts[v]) <= bar)
                    })
                };
                let (inside, outside): (Vec<usize>, Vec<usize>) =
                    wave.into_iter().partition(|&facet| holds(facet));
                for facet in outside {
                    blocked[facet] = true;
                }
                if inside.is_empty() {
                    break;
                }
                let mut trial = group.clone();
                trial.extend(inside.iter().copied());
                let Some(fit) = grow_fit(kind, &trial, &carrier) else {
                    for facet in inside {
                        blocked[facet] = true;
                    }
                    break;
                };
                for &facet in &inside {
                    in_group[facet] = true;
                }
                group = trial;
                carrier = fit.carrier;
                continue;
            }
            let mut accepted_any = false;
            for facet in wave {
                let mut trial = group.clone();
                trial.push(facet);
                if let Some(fit) = grow_fit(kind, &trial, &carrier) {
                    in_group[facet] = true;
                    group = trial;
                    carrier = fit.carrier;
                    accepted_any = true;
                } else {
                    blocked[facet] = true;
                }
            }
            if !accepted_any {
                break;
            }
        }
        let mut tris = facet_tris(&group);
        if tris.len() < options.min_region_triangles.max(4)
            || region_vertices(data, &tris).len() < min_vertices
        {
            continue;
        }
        tris.sort_unstable();
        let outcome = fit_peeled(data, &tris, options);
        if debug {
            let size = match outcome.fit.carrier {
                RegionCarrier::Cylinder { radius, .. } | RegionCarrier::Sphere { radius, .. } => {
                    format!(" r={radius:.4}")
                }
                RegionCarrier::Cone { half_angle_rad, .. } => {
                    format!(" ha={:.2}deg", half_angle_rad.to_degrees())
                }
                _ => String::new(),
            };
            let (low, high) = region_bbox(data, &region_vertices(data, &tris));
            eprintln!(
                "[seg] peel {kind}: seed facet {seed} grew {} facets / {} tris at {bar:.3e}; cascade {}{size} accepted={} max_dev={:.3e} max_ang={:.2} bbox [{:.2},{:.2},{:.2}]..[{:.2},{:.2},{:.2}]",
                group.len(),
                tris.len(),
                outcome.fit.carrier.kind(),
                outcome.accepted,
                outcome.fit.max_dev,
                outcome.fit.max_normal_angle_deg,
                low.x, low.y, low.z, high.x, high.y, high.z
            );
        }
        if !outcome.accepted || outcome.fit.carrier.kind() != kind {
            continue;
        }
        // A torus is claimed only as a FULL band around its axis — the
        // shape a mesh rebuild can bound by closed rings. A partial torus
        // (a fillet running along an arc of an outline) stays with the
        // planar strips it has always been read as.
        if kind == "torus" && !covers_full_azimuth(data, &tris, &outcome.fit.carrier) {
            continue;
        }
        for &facet in &group {
            assigned[facet] = true;
        }
        pieces.push((tris, outcome));
    }
    }
    if pieces.len() == claimed {
        break;
    }
    }
    if pieces.is_empty() {
        return (pieces, tri_ids.to_vec());
    }
    // A leftover strip whose every peeled neighbor is the same carrier is an
    // interior gap — a tessellator T-vertex on a rim chord — not a
    // junction: it rejoins that carrier when the union still passes the
    // cascade at the user tolerance.  A strip straddling the tangent point
    // of two different carriers stays out and becomes its own small facet.
    let mut piece_of_facet = vec![usize::MAX; facets.len()];
    for (piece, (tris, _)) in pieces.iter().enumerate() {
        for &t in tris {
            piece_of_facet[facet_of[local[&t]]] = piece;
        }
    }
    let leftover_tris: Vec<u32> = (0..tri_ids.len())
        .filter(|&index| !assigned[facet_of[index]])
        .map(|index| tri_ids[index])
        .collect();
    let compat_tol = options.fit_tolerance * data.diag.max(1e-12);
    let mut remainder = Vec::new();
    for component in connected_components(data, &leftover_tris) {
        let mut adjacent_pieces: Vec<usize> = Vec::new();
        for &t in &component {
            for &other in &facet_neighbors[facet_of[local[&t]]] {
                let piece = piece_of_facet[other];
                if piece != usize::MAX && !adjacent_pieces.contains(&piece) {
                    adjacent_pieces.push(piece);
                }
            }
        }
        let Some(&first) = adjacent_pieces.first() else {
            remainder.extend(component);
            continue;
        };
        let same_carrier = adjacent_pieces.iter().all(|&piece| {
            carriers_compatible(
                &pieces[first].1.fit.carrier,
                &pieces[piece].1.fit.carrier,
                compat_tol,
                true,
            )
        });
        if !same_carrier {
            remainder.extend(straddle_rejoin(data, options, bar, &mut pieces, &adjacent_pieces, component));
            continue;
        }
        let mut union: Vec<u32> = pieces[first].0.iter().chain(component.iter()).copied().collect();
        union.sort_unstable();
        let outcome = fit_peeled(data, &union, options);
        let rejoined = outcome.accepted
            && outcome.fit.carrier.kind() == pieces[first].1.fit.carrier.kind();
        if debug {
            eprintln!(
                "[seg] peel: leftover of {} tris between {} peeled piece(s): union {} accepted={} max_dev={:.3e}",
                component.len(),
                adjacent_pieces.len(),
                outcome.fit.carrier.kind(),
                outcome.accepted,
                outcome.fit.max_dev
            );
        }
        if rejoined {
            pieces[first] = (union, outcome);
        } else {
            remainder.extend(straddle_rejoin(data, options, bar, &mut pieces, &adjacent_pieces, component));
        }
    }
    remainder.sort_unstable();
    (pieces, remainder)
}

/// The cascade's verdict on a peeled piece. The cascade keeps its plain
/// torus fit; a torus the peel grew and judged at the precision bar carries
/// the vertex-refined carrier instead, so a mesh rebuild gets its true
/// axis and radii.
fn fit_peeled(data: &MeshData, tris: &[u32], options: &SegmentOptions) -> RegionOutcome {
    let mut outcome = fit_region(data, tris, options);
    if outcome.accepted && outcome.fit.carrier.kind() == "torus" {
        let verts = region_vertices(data, tris);
        if let Some(refined) =
            refine_torus_carrier(data, tris, &verts, &outcome.fit.carrier, TORUS_REFINE_ITERATIONS)
        {
            if refined.max_dev < outcome.fit.max_dev {
                outcome.fit = refined;
            }
        }
    }
    outcome
}

/// Whether a torus region's vertices surround its axis with no azimuthal gap
/// wider than a twelfth of a turn.
fn covers_full_azimuth(data: &MeshData, tri_ids: &[u32], carrier: &RegionCarrier) -> bool {
    let RegionCarrier::Torus { center, axis_dir, .. } = carrier else {
        return false;
    };
    let Ok(axis) = axis_dir.normalized() else {
        return false;
    };
    let Ok(x_axis) = axis.perpendicular() else {
        return false;
    };
    let y_axis = axis.cross(x_axis);
    let mut angles = region_vertices(data, tri_ids)
        .into_iter()
        .map(|v| {
            let d = data.verts[v].sub(*center);
            d.dot(y_axis).atan2(d.dot(x_axis))
        })
        .collect::<Vec<_>>();
    if angles.len() < 3 {
        return false;
    }
    angles.sort_by(f64::total_cmp);
    let wrap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
    let widest = angles
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .fold(wrap, f64::max);
    widest <= std::f64::consts::TAU / 12.0
}

/// A leftover strip at the tangent junction of DIFFERENT carriers (the
/// first row of a fillet torus against its cylinder) is not one carrier's
/// gap, but each of its triangles still lies on exactly one side. A triangle
/// joins the adjacent piece whose carrier holds all three of its vertices
/// within the peel's precision bar (the closest when several do), and a
/// piece that took any is re-fitted and must still pass the cascade as the
/// same kind, or it keeps its old triangles and they stay out. A triangle
/// no adjacent carrier holds at the bar is returned.
fn straddle_rejoin(
    data: &MeshData,
    options: &SegmentOptions,
    bar: f64,
    pieces: &mut [(Vec<u32>, RegionOutcome)],
    adjacent_pieces: &[usize],
    component: Vec<u32>,
) -> Vec<u32> {
    let mut taken: Vec<Vec<u32>> = vec![Vec::new(); adjacent_pieces.len()];
    let mut left = Vec::new();
    for t in component {
        let tri = &data.tris[t as usize];
        let best = adjacent_pieces
            .iter()
            .enumerate()
            .filter_map(|(slot, &piece)| {
                let carrier = &pieces[piece].1.fit.carrier;
                let worst = tri
                    .verts
                    .iter()
                    .map(|&v| carrier_distance(carrier, data.verts[v]))
                    .fold(0.0_f64, f64::max);
                (worst <= bar).then_some((slot, worst))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match best {
            Some((slot, _)) => taken[slot].push(t),
            None => left.push(t),
        }
    }
    for (slot, extra) in taken.into_iter().enumerate() {
        if extra.is_empty() {
            continue;
        }
        let piece = adjacent_pieces[slot];
        let mut union: Vec<u32> = pieces[piece].0.iter().chain(extra.iter()).copied().collect();
        union.sort_unstable();
        let outcome = fit_peeled(data, &union, options);
        if outcome.accepted && outcome.fit.carrier.kind() == pieces[piece].1.fit.carrier.kind() {
            pieces[piece] = (union, outcome);
        } else {
            left.extend(extra);
        }
    }
    left
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Segment a triangle mesh into smooth regions and recognize each region's
/// analytic carrier.  `positions` are xyz triples; `indices` is a triangle
/// index buffer, or empty to treat `positions` as a raw soup of consecutive
/// triangles (the binary-STL layout of `read_binary_stl`).
pub fn segment_mesh_faces(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
) -> Result<MeshSegmentation, String> {
    if !(options.deflection_angle_deg > 0.0) || !options.deflection_angle_deg.is_finite() {
        return Err("segment_mesh_faces: deflection angle must be positive".into());
    }
    if !(options.fit_tolerance > 0.0) || !options.fit_tolerance.is_finite() {
        return Err("segment_mesh_faces: fit tolerance must be positive".into());
    }
    let data = build_mesh_data(positions, indices, options)?;
    Ok(segment_prepared(&data, options))
}

/// Segmentation core on an already-welded mesh: shared by the analysis entry
/// (`segment_mesh_faces`) and the BREP rebuild (`mesh_regions_to_brep`), so
/// both see identical triangle/vertex indexing.
pub(super) fn segment_prepared(data: &MeshData, options: &SegmentOptions) -> MeshSegmentation {
    let mut raw_regions = grow_regions(data, options);
    absorb_tiny_regions(data, &mut raw_regions);

    let mut final_regions: Vec<(Vec<u32>, RegionOutcome)> = Vec::new();
    for region in raw_regions {
        let outcome = fit_region(data, &region, options);
        if outcome.accepted {
            final_regions.push((region, outcome));
            continue;
        }
        // Tangent-smooth compound (or genuinely freeform): try to split it.
        let pieces = refine_region(data, &region, options);
        if pieces.len() <= 1 {
            // Refinement did not split anything; keep the region whole.
            final_regions.push((region, outcome));
        } else {
            final_regions.extend(pieces);
        }
    }

    // Reunite regions the growing/refinement pipeline split although they
    // lie on ONE carrier (a cap triangle isolated behind skipped slivers, a
    // blend peeled into two coaxial pieces).  Downstream consumers — the
    // BREP rebuild above all — rely on "one region per face".
    merge_compatible_regions(data, &mut final_regions, options);

    let mut triangle_region_ids = vec![UNASSIGNED_REGION; data.tris.len()];
    let mut regions = Vec::with_capacity(final_regions.len());
    for (id, (tri_ids, outcome)) in final_regions.into_iter().enumerate() {
        for &t in &tri_ids {
            triangle_region_ids[t as usize] = id as u32;
        }
        regions.push(MeshRegion {
            id: id as u32,
            triangle_count: outcome.triangle_count,
            area: outcome.area,
            bbox_min: outcome.bbox.0,
            bbox_max: outcome.bbox.1,
            carrier: outcome.fit.carrier,
            max_deviation: outcome.fit.max_dev,
            rms_deviation: outcome.fit.rms_dev,
            max_normal_angle_deg: outcome.fit.max_normal_angle_deg,
        });
    }

    // Attach degenerate/micro triangles to an adjacent region when one
    // exists: the neighbor whose carrier the sliver's vertices actually sit
    // on (a T-junction needle lies in one of its neighboring planes, not
    // on the fillet it happens to touch first).  Multi-pass so chains of
    // skipped slivers resolve through their assigned neighbors.
    let mut changed = true;
    while changed {
        changed = false;
        for t in 0..data.tris.len() {
            if triangle_region_ids[t] != UNASSIGNED_REGION {
                continue;
            }
            let tri = &data.tris[t];
            let mut adopted: Option<(f64, u32)> = None;
            for corner in 0..3 {
                let first = tri.verts[corner];
                let second = tri.verts[(corner + 1) % 3];
                if first == second {
                    continue;
                }
                if let Some(incident) = data.edges.get(&edge_key(first, second)) {
                    for &other in incident {
                        let region = triangle_region_ids[other as usize];
                        if other as usize == t || region == UNASSIGNED_REGION {
                            continue;
                        }
                        let carrier = &regions[region as usize].carrier;
                        let distance = tri
                            .verts
                            .iter()
                            .map(|&v| carrier_distance(carrier, data.verts[v]))
                            .fold(0.0_f64, f64::max);
                        let candidate = (distance, region);
                        adopted = Some(match adopted {
                            Some(current)
                                if current.0 < candidate.0
                                    || (current.0 == candidate.0 && current.1 <= candidate.1) =>
                            {
                                current
                            }
                            _ => candidate,
                        });
                    }
                }
            }
            if let Some((_, region)) = adopted {
                triangle_region_ids[t] = region;
                changed = true;
            }
        }
    }

    MeshSegmentation {
        triangle_region_ids,
        regions,
        triangle_count: data.tris.len(),
        welded_vertex_count: data.verts.len(),
    }
}

/// Outward carrier normal at a point (before the region's sense), or `None`
/// where the field is singular (on an axis, at a center) or for `Freeform`.
fn carrier_normal(carrier: &RegionCarrier, p: Vec3) -> Option<Vec3> {
    match carrier {
        RegionCarrier::Plane { normal, .. } => Some(*normal),
        RegionCarrier::Cylinder {
            axis_point,
            axis_dir,
            ..
        } => {
            let d = p.sub(*axis_point);
            d.sub(axis_dir.scale(d.dot(*axis_dir))).normalized().ok()
        }
        RegionCarrier::Cone {
            apex,
            axis_dir,
            half_angle_rad,
            ..
        } => {
            let d = p.sub(*apex);
            let radial = d.sub(axis_dir.scale(d.dot(*axis_dir))).normalized().ok()?;
            let (sin_a, cos_a) = half_angle_rad.sin_cos();
            Some(radial.scale(cos_a).sub(axis_dir.scale(sin_a)))
        }
        RegionCarrier::Sphere { center, .. } => p.sub(*center).normalized().ok(),
        RegionCarrier::Torus {
            center,
            axis_dir,
            major_radius,
            ..
        } => {
            let d = p.sub(*center);
            let s = d.dot(*axis_dir);
            let radial = d.sub(axis_dir.scale(s)).normalized().ok()?;
            let tube_center = center.add(radial.scale(*major_radius));
            p.sub(tube_center).normalized().ok()
        }
        RegionCarrier::Freeform => None,
    }
}

/// The region's orientation sense as a sign, `1.0` for planes and freeform.
fn carrier_sense(carrier: &RegionCarrier) -> f64 {
    match carrier {
        RegionCarrier::Cylinder { sense, .. }
        | RegionCarrier::Cone { sense, .. }
        | RegionCarrier::Sphere { sense, .. }
        | RegionCarrier::Torus { sense, .. } => f64::from(*sense),
        _ => 1.0,
    }
}

/// Distance of a point from a carrier; `Freeform` is infinitely far.
fn carrier_distance(carrier: &RegionCarrier, p: Vec3) -> f64 {
    match carrier {
        RegionCarrier::Plane { origin, normal } => p.sub(*origin).dot(*normal).abs(),
        RegionCarrier::Cylinder {
            axis_point,
            axis_dir,
            radius,
            ..
        } => {
            let d = p.sub(*axis_point);
            (d.sub(axis_dir.scale(d.dot(*axis_dir))).length() - radius).abs()
        }
        RegionCarrier::Cone {
            apex,
            axis_dir,
            half_angle_rad,
            ..
        } => {
            let d = p.sub(*apex);
            let s = d.dot(*axis_dir);
            let r = d.sub(axis_dir.scale(s)).length();
            ((r - s * half_angle_rad.tan()) * half_angle_rad.cos()).abs()
        }
        RegionCarrier::Sphere { center, radius, .. } => (p.sub(*center).length() - radius).abs(),
        RegionCarrier::Torus {
            center,
            axis_dir,
            major_radius,
            minor_radius,
            ..
        } => {
            let d = p.sub(*center);
            let s = d.dot(*axis_dir);
            let rho = d.sub(axis_dir.scale(s)).length();
            ((rho - major_radius).hypot(s) - minor_radius).abs()
        }
        RegionCarrier::Freeform => f64::INFINITY,
    }
}

/// Do two accepted carriers describe the same surface (within `tol` for
/// distances, tight angular bands for directions)?  Freeform never matches.
/// `adjacent` says the regions share an edge: only then may two cylinder
/// fits disagree on their axis by the noise over the axial extent, because
/// two disjoint faces of one infinite cylinder (the two ends of a mirrored
/// part, the two levels of a slot) must stay two regions.
fn carriers_compatible(a: &RegionCarrier, b: &RegionCarrier, tol: f64, adjacent: bool) -> bool {
    const ANGULAR: f64 = 1e-6;
    match (a, b) {
        (
            RegionCarrier::Plane {
                origin: o1,
                normal: n1,
            },
            RegionCarrier::Plane {
                origin: o2,
                normal: n2,
            },
        ) => {
            n1.cross(*n2).length() <= ANGULAR
                && n1.dot(*n2) > 0.0
                && o2.sub(*o1).dot(*n1).abs() <= tol
        }
        (
            RegionCarrier::Cylinder {
                axis_point: p1,
                axis_dir: a1,
                radius: r1,
                sense: s1,
            },
            RegionCarrier::Cylinder {
                axis_point: p2,
                axis_dir: a2,
                radius: r2,
                sense: s2,
            },
        ) => {
            // Independent fits of one cylinder from noisy vertices disagree
            // on the axis direction by the noise over the axial extent; the
            // band is the angle that moves the surface by `tol` at the rim.
            // The union re-fit is the real test.
            let angular = if adjacent {
                ANGULAR.max(tol / r1.max(*r2).max(1e-12))
            } else {
                ANGULAR
            };
            let offset = p2.sub(*p1);
            s1 == s2
                && a1.cross(*a2).length() <= angular
                && (r1 - r2).abs() <= tol
                && offset.sub(a1.scale(offset.dot(*a1))).length() <= tol
        }
        (
            RegionCarrier::Cone {
                apex: p1,
                axis_dir: a1,
                half_angle_rad: h1,
                sense: s1,
            },
            RegionCarrier::Cone {
                apex: p2,
                axis_dir: a2,
                half_angle_rad: h2,
                sense: s2,
            },
        ) => {
            s1 == s2
                && a1.cross(*a2).length() <= ANGULAR
                && a1.dot(*a2) > 0.0
                && (h1 - h2).abs() <= ANGULAR
                && p2.sub(*p1).length() <= tol
        }
        (
            RegionCarrier::Sphere {
                center: c1,
                radius: r1,
                sense: s1,
            },
            RegionCarrier::Sphere {
                center: c2,
                radius: r2,
                sense: s2,
            },
        ) => s1 == s2 && c2.sub(*c1).length() <= tol && (r1 - r2).abs() <= tol,
        (
            RegionCarrier::Torus {
                center: c1,
                axis_dir: a1,
                major_radius: mj1,
                minor_radius: mn1,
                sense: s1,
            },
            RegionCarrier::Torus {
                center: c2,
                axis_dir: a2,
                major_radius: mj2,
                minor_radius: mn2,
                sense: s2,
            },
        ) => {
            s1 == s2
                && a1.cross(*a2).length() <= ANGULAR
                && c2.sub(*c1).length() <= tol
                && (mj1 - mj2).abs() <= tol
                && (mn1 - mn2).abs() <= tol
        }
        _ => false,
    }
}

/// Merge edge-adjacent regions whose accepted carriers agree, repeating
/// until stable.  Adjacency counts paths THROUGH skipped (degenerate/micro)
/// triangles — exactly the fences that isolate cap islands in the first
/// place.  A merge is only applied when the union re-fits cleanly on the
/// same carrier kind, so this pass can only improve the segmentation.
fn merge_compatible_regions(
    data: &MeshData,
    regions: &mut Vec<(Vec<u32>, RegionOutcome)>,
    options: &SegmentOptions,
) {
    let tol = options.fit_tolerance * data.diag.max(1e-12);
    loop {
        let mut region_of = vec![usize::MAX; data.tris.len()];
        for (index, (tris, _)) in regions.iter().enumerate() {
            for &t in tris {
                region_of[t as usize] = index;
            }
        }
        // Directly edge-adjacent pairs.
        let mut direct: std::collections::BTreeSet<(usize, usize)> =
            std::collections::BTreeSet::new();
        for incident in data.edges.values() {
            if incident.len() != 2 {
                continue;
            }
            let (a, b) = (
                region_of[incident[0] as usize],
                region_of[incident[1] as usize],
            );
            if a != b && a != usize::MAX && b != usize::MAX {
                direct.insert((a.min(b), a.max(b)));
            }
        }
        let mut pairs = direct.clone();
        // Pairs bridged by connected runs of region-less (skipped) triangles.
        let mut seen = vec![false; data.tris.len()];
        for seed in 0..data.tris.len() {
            if seen[seed] || region_of[seed] != usize::MAX {
                continue;
            }
            let mut stack = vec![seed];
            let mut touched: Vec<usize> = Vec::new();
            seen[seed] = true;
            while let Some(t) = stack.pop() {
                let tri = &data.tris[t];
                for corner in 0..3 {
                    let first = tri.verts[corner];
                    let second = tri.verts[(corner + 1) % 3];
                    if first == second {
                        continue;
                    }
                    if let Some(incident) = data.edges.get(&edge_key(first, second)) {
                        for &other in incident {
                            let other = other as usize;
                            if other == t {
                                continue;
                            }
                            let region = region_of[other];
                            if region == usize::MAX {
                                if !seen[other] {
                                    seen[other] = true;
                                    stack.push(other);
                                }
                            } else if !touched.contains(&region) {
                                touched.push(region);
                            }
                        }
                    }
                }
            }
            touched.sort_unstable();
            for i in 0..touched.len() {
                for j in i + 1..touched.len() {
                    pairs.insert((touched[i], touched[j]));
                }
            }
        }

        let mut merged_any = false;
        'pairs: for (a, b) in pairs {
            let (first, second) = (&regions[a], &regions[b]);
            if !first.1.accepted || !second.1.accepted {
                continue;
            }
            if !carriers_compatible(
                &first.1.fit.carrier,
                &second.1.fit.carrier,
                tol,
                direct.contains(&(a, b)),
            ) {
                continue;
            }
            let mut union: Vec<u32> = first.0.iter().chain(second.0.iter()).copied().collect();
            union.sort_unstable();
            let outcome = fit_region(data, &union, options);
            if !outcome.accepted || outcome.fit.carrier.kind() != first.1.fit.carrier.kind() {
                continue;
            }
            regions[a] = (union, outcome);
            regions.remove(b);
            merged_any = true;
            break 'pairs;
        }
        if !merged_any {
            return;
        }
    }
}
