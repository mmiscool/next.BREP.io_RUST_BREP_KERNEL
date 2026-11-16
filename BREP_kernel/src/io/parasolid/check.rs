//! Structural invariants of a manifold body's XT data (§4.3, §5.3), checked
//! over the links the importer never follows: fin rings and pairs, the fin
//! chains at vertices, back-pointers, the face, shell, region, edge, vertex
//! and geometry chains, owners and node ids. A writer's output must satisfy
//! everything Parasolid's own files do; the check is first run over real
//! files, where it finds nothing.

use super::reader::{Node, XtFile};

/// Every invariant a solid or sheet body breaks, as messages; empty when it
/// keeps them all.
pub fn check_xt_body(file: &XtFile, body: &Node) -> Vec<String> {
    let mut bad = Vec::new();
    let mut say = |m: String| {
        if bad.len() < 50 {
            bad.push(m);
        }
    };
    let get = |i: u32| file.node(i);
    let kind = |i: u32| get(i).map_or("null", |n| n.type_name());
    // Chains: walk `next` from a head, stopping on a repeat or at a node the
    // file does not hold (so every index walked names a node); a chain that
    // reaches a missing node is reported below.
    let missing = std::cell::RefCell::new(Vec::new());
    let walk = |head: u32, next: &'static str| {
        let mut out = Vec::new();
        let mut seen = rustc_hash::FxHashSet::default();
        let mut at = head;
        while at != 0 && seen.insert(at) {
            let Some(node) = get(at) else {
                missing.borrow_mut().push((out.last().copied(), next, at));
                break;
            };
            out.push(at);
            at = node.ptr(next);
        }
        out
    };
    let body_index = body.index;
    // Regions.
    let regions = walk(body.ptr("region"), "next");
    if regions.is_empty() {
        say("the body has no regions".into());
    }
    if let Some(&first) = regions.first() {
        if get(first).and_then(|r| r.char("type")) != Some(b'V') {
            say("the body's first region (the infinite one) is not void".into());
        }
    }
    for pair in regions.windows(2) {
        if get(pair[1]).map_or(0, |r| r.ptr("previous")) != pair[0] {
            say(format!("region {} does not point back at {}", pair[1], pair[0]));
        }
    }
    let mut shells = Vec::new();
    for &r in &regions {
        let region = get(r).unwrap();
        if region.ptr("body") != body_index {
            say(format!("region {r} does not name the body"));
        }
        for s in walk(region.ptr("shell"), "next") {
            if kind(s) != "SHELL" {
                say(format!("region {r}'s shell chain reaches a {}", kind(s)));
                continue;
            }
            if get(s).unwrap().ptr("region") != r {
                say(format!("shell {s} does not name its region {r}"));
            }
            shells.push(s);
        }
    }
    // Faces: each a back face of exactly one shell and a front face of
    // exactly one (manifold solid), with back-pointers.
    let mut back_of = rustc_hash::FxHashMap::default();
    let mut front_of = rustc_hash::FxHashMap::default();
    for &s in &shells {
        let shell = get(s).unwrap();
        let back = walk(shell.ptr("face"), "next");
        for pair in back.windows(2) {
            if get(pair[1]).map_or(0, |f| f.ptr("previous")) != pair[0] {
                say(format!("face {} does not point back at {} in shell {s}", pair[1], pair[0]));
            }
        }
        for f in back {
            if get(f).map_or(0, |n| n.ptr("shell")) != s {
                say(format!("face {f} in shell {s}'s back chain names shell {}", get(f).map_or(0, |n| n.ptr("shell"))));
            }
            if back_of.insert(f, s).is_some() {
                say(format!("face {f} is a back face of two shells"));
            }
        }
        let front = walk(shell.ptr("front_face"), "next_front");
        for pair in front.windows(2) {
            if get(pair[1]).map_or(0, |f| f.ptr("previous_front")) != pair[0] {
                say(format!("face {} does not point back at {} in shell {s}'s front chain", pair[1], pair[0]));
            }
        }
        for f in front {
            if get(f).map_or(0, |n| n.ptr("front_shell")) != s {
                say(format!("face {f} in shell {s}'s front chain names front shell {}", get(f).map_or(0, |n| n.ptr("front_shell"))));
            }
            if front_of.insert(f, s).is_some() {
                say(format!("face {f} is a front face of two shells"));
            }
        }
    }
    let solid = body.int("body_type") == Some(1);
    let mut faces: Vec<u32> = back_of.keys().chain(front_of.keys()).copied().collect();
    faces.sort_unstable();
    faces.dedup();
    if solid {
        for &f in &faces {
            if !back_of.contains_key(&f) || !front_of.contains_key(&f) {
                say(format!("face {f} of a solid is not both a back and a front face"));
            }
        }
    }
    // Loops and fins.
    let mut fins = Vec::new();
    for &f in &faces {
        let face = get(f).unwrap();
        if get(face.ptr("surface")).is_none() {
            say(format!("face {f} has no surface"));
        }
        for l in walk(face.ptr("loop"), "next") {
            let lp = get(l).unwrap();
            if lp.ptr("face") != f {
                say(format!("loop {l} does not name its face {f}"));
            }
            let ring = walk(lp.ptr("halfedge"), "forward");
            for (k, &h) in ring.iter().enumerate() {
                let fin = get(h).unwrap();
                if fin.ptr("loop") != l {
                    say(format!("fin {h} does not name its loop {l}"));
                }
                let next = ring[(k + 1) % ring.len()];
                if fin.ptr("forward") != next {
                    say(format!("loop {l}'s ring does not close at fin {h}"));
                }
                if get(next).map_or(0, |n| n.ptr("backward")) != h {
                    say(format!("fin {next} does not point back at {h}"));
                }
                // A fin runs to its vertex, and the next fin starts there:
                // the next fin's start is its other fin's vertex.
                let start_of_next = get(next).and_then(|n| get(n.ptr("other"))).map_or(0, |o| o.ptr("vertex"));
                if fin.ptr("edge") != 0 && get(next).map_or(0, |n| n.ptr("edge")) != 0 && fin.ptr("vertex") != start_of_next {
                    say(format!("fin {h} ends at vertex {} but the next fin {next} starts at {start_of_next}", fin.ptr("vertex")));
                }
                fins.push(h);
            }
        }
    }
    // Edges: two fins, opposite senses, paired; the positive one heads the
    // edge's fin chain and points at the vertex the edge runs to.
    let edges = walk(body.ptr("edge"), "next");
    for pair in edges.windows(2) {
        if get(pair[1]).map_or(0, |e| e.ptr("previous")) != pair[0] {
            say(format!("edge {} does not point back at {}", pair[1], pair[0]));
        }
    }
    let mut primary_secondary = rustc_hash::FxHashSet::default();
    for &e in &edges {
        let edge = get(e).unwrap();
        if edge.ptr("owner") != body_index {
            say(format!("edge {e} is not owned by the body"));
        }
        let first = edge.ptr("halfedge");
        let Some(a) = get(first) else {
            say(format!("edge {e} has no fin"));
            continue;
        };
        let second = a.ptr("other");
        let Some(b) = get(second) else {
            say(format!("edge {e}'s fin {first} has no other"));
            continue;
        };
        if a.char("sense") != Some(b'+') {
            say(format!("edge {e}'s first fin {first} is not positive"));
        }
        if solid && b.ptr("other") != first {
            say(format!("edge {e}'s fins {first} and {second} are not each other's other"));
        }
        if solid && b.char("sense") != Some(b'-') {
            say(format!("edge {e}'s second fin {second} is not negative"));
        }
        for h in [first, second] {
            if get(h).map_or(0, |n| n.ptr("edge")) != e {
                say(format!("fin {h} does not name its edge {e}"));
            }
        }
        if edge.ptr("curve") == 0 && edge.real("tolerance").is_none() {
            say(format!("edge {e} has no curve and is not tolerant"));
        }
        primary_secondary.insert(first);
        primary_secondary.insert(second);
    }
    for &h in &fins {
        let fin = get(h).unwrap();
        if fin.ptr("edge") != 0 && !edges.contains(&fin.ptr("edge")) {
            say(format!("fin {h}'s edge {} is not in the body's edge chain", fin.ptr("edge")));
        }
    }
    // Vertices: their fin chains hold exactly the primary and secondary fins
    // that run to them.
    let vertices = walk(body.ptr("vertex"), "next");
    for pair in vertices.windows(2) {
        if get(pair[1]).map_or(0, |v| v.ptr("previous")) != pair[0] {
            say(format!("vertex {} does not point back at {}", pair[1], pair[0]));
        }
    }
    let mut chained = rustc_hash::FxHashSet::default();
    for &v in &vertices {
        let vertex = get(v).unwrap();
        if vertex.ptr("owner") != body_index {
            say(format!("vertex {v} is not owned by the body"));
        }
        match get(vertex.ptr("point")) {
            Some(p) if p.type_name() == "POINT" => {
                if p.ptr("owner") != v {
                    say(format!("vertex {v}'s point does not name it"));
                }
            }
            _ => say(format!("vertex {v} has no point")),
        }
        for h in walk(vertex.ptr("halfedge"), "next_at_vx") {
            if get(h).map_or(0, |n| n.ptr("vertex")) != v {
                say(format!("fin {h} in vertex {v}'s chain runs to vertex {}", get(h).map_or(0, |n| n.ptr("vertex"))));
            }
            // An isolated loop's fin (no edge) is chained at its vertex too
            // (§5.3.8.1).
            let isolated = get(h).is_some_and(|n| n.ptr("edge") == 0 && n.ptr("forward") == h);
            if !isolated && !primary_secondary.contains(&h) {
                say(format!("fin {h} in vertex {v}'s chain is neither an edge's primary nor secondary fin"));
            }
            chained.insert(h);
        }
    }
    for &h in &primary_secondary {
        // A dummy fin (a sheet's laminar edge: no loop) is in no chain
        // (§5.3.9.1).
        let dummy = get(h).is_some_and(|n| n.ptr("loop") == 0);
        if !dummy && get(h).map_or(0, |n| n.ptr("vertex")) != 0 && !chained.contains(&h) {
            say(format!("fin {h} is an edge's primary or secondary fin but no vertex chains it"));
        }
    }
    // Geometry: each surface, curve and point of the body in its boundary
    // chain, owned by what uses it (or by another geometry node).
    for (head, what) in [("boundary_surface", "surface"), ("boundary_curve", "curve"), ("boundary_point", "point")] {
        let chain = walk(body.ptr(head), "next");
        for pair in chain.windows(2) {
            if get(pair[1]).map_or(0, |g| g.ptr("previous")) != pair[0] {
                say(format!("{what} {} does not point back at {} in the body's {head} chain", pair[1], pair[0]));
            }
        }
        for g in chain {
            if get(g).map_or(0, |n| n.ptr("owner")) == 0 && get(g).map_or(0, |n| n.ptr("geometric_owner")) == 0 {
                say(format!("{what} {g} in the body's chain has no owner"));
            }
        }
    }
    for (from, field, to) in missing.borrow().iter() {
        match from {
            Some(from) => say(format!("node {from}'s {field} points at node {to}, which the file does not hold")),
            None => say(format!("a chain's head is node {to}, which the file does not hold")),
        }
    }
    // Node ids: distinct, positive, at most the body's highest.
    let highest = body.int("highest_node_id").unwrap_or(0);
    let mut ids = rustc_hash::FxHashMap::default();
    for n in file.nodes() {
        let Some(id) = n.int("node_id") else { continue };
        if id == 0 || n.layout.fields.iter().all(|f| f.name != "node_id") {
            continue;
        }
        if id > highest {
            say(format!("{} {} has node id {id} above the body's highest {highest}", n.type_name(), n.index));
        }
        if let Some(other) = ids.insert(id, n.index) {
            say(format!("node id {id} is on nodes {other} and {}", n.index));
        }
    }
    bad
}
