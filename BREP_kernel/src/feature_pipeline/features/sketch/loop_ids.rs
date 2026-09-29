//! Stable per-LOOP identity for a sketch's closed loops.
//!
//! # Why
//!
//! A profile-swept feature (extrude/sweep/revolve/loft/path-sweep) builds one
//! prism per profile REGION and stamps names on the result. Sidewalls are safe:
//! they are named from the source geometry's persistent `{sketchId}:G{gid}`. The
//! CAPS were not — every region stamped the SAME `{cap_base}_START/_END`, so the
//! post-union `ensure_unique_face_names` pass disambiguated them POSITIONALLY
//! (`X[0]`, `X[1]`, …) in shell/face encounter order. Editing the sketch to add
//! or remove a loop shifted those indices, and a fillet that referenced `X[1]`
//! silently jumped to a different island on replay.
//!
//! The fix is to give every loop an identity of its own that the cap names embed.
//! That identity must survive editing the loop itself, so it is PERSISTED: each
//! sketch geometry carries a `loopId` in `persistentData.sketch`, written back by
//! the editor on commit (`assign_sketch_loop_ids`).
//!
//! # The rule
//!
//! For each assembled closed loop:
//! - if ANY member edge carries a stored `loopId`, the loop claims the LOWEST of
//!   them ("lowest number wins") and that value is written back onto every other
//!   member edge, so the loop keeps its identity as edges come and go;
//! - otherwise the loop is new and SEEDS its id from the lowest geometry id
//!   among its edges.
//!
//! Both halves are computed the same way whether or not the document has ever
//! been committed, so a sketch that has never been through the editor derives
//! exactly the id it would have been assigned — reading and writing never
//! disagree, and no document renames faces merely by being opened.
//!
//! # Resilience
//!
//! - ADDING an edge to a loop: the new edge carries no id and inherits the
//!   loop's — unchanged. (Minting also hands out ids above the current maximum,
//!   so a seed can only move down, never up.)
//! - REMOVING a non-anchor edge: the id lives on the survivors — unchanged.
//! - REMOVING the anchor edge (the one the id was seeded from): still unchanged,
//!   because the id was persisted onto the whole loop, not recomputed from the
//!   anchor. This is the case a derive-only scheme cannot handle, and the reason
//!   the id is stored at all.
//! - Editing a DIFFERENT loop: never observed here — a loop's claim reads only
//!   its own edges. That is the reported bug, fixed.
//!
//! # Collisions
//!
//! Two loops can claim the same id: a loop that SPLIT in two (both halves carry
//! the same stored id), or — because the editor mints `max(existing gid) + 1` and
//! therefore REUSES the ids of deleted geometry — a freshly drawn loop whose
//! seed collides with an older loop's stored id. Ties break in this order:
//! 1. a loop that INHERITED the id (stored on its edges) beats one that merely
//!    seeded it from a gid, so a new loop can never steal a live loop's identity;
//! 2. between two inheritors (the split case), the one holding the lower minimum
//!    gid keeps it — deterministic, and independent of loop order.
//! The loser re-seeds from its own lowest gid, or, if that is taken too, from one
//! above the highest id in play. A split therefore keeps the original id on one
//! half and mints a fresh one for the other; if the halves are later rejoined,
//! the survivor's id wins again.

use super::geometry::Loop;

/// Resolve every loop's [`Loop::loop_id`], in place.
///
/// `loops` is the fully assembled set (chained loops plus whole circles and
/// ellipses) BEFORE region classification, so ids are decided over the sketch as
/// a whole and never depend on how the loops later group into regions.
pub(super) fn resolve(loops: &mut [Loop]) {
    let claims: Vec<Claim> = loops.iter().map(Claim::of).collect();
    for (loop_data, id) in loops.iter_mut().zip(settle(&claims)) {
        loop_data.loop_id = id;
    }
}

/// One loop's claim: the id it wants, whether it INHERITED that id from stored
/// geometry (as opposed to seeding it from a gid), and its lowest gid — the
/// tie-break and the re-seed source.
struct Claim {
    id: Option<u64>,
    inherited: bool,
    min_gid: Option<u64>,
}

impl Claim {
    fn of(loop_data: &Loop) -> Self {
        let stored = loop_data
            .entries
            .iter()
            .filter_map(|(segment, _)| segment.ids.loop_id)
            .min();
        let min_gid = loop_data
            .entries
            .iter()
            .filter_map(|(segment, _)| segment.ids.gid)
            .min();
        Self {
            id: stored.or(min_gid),
            inherited: stored.is_some(),
            min_gid,
        }
    }
}

/// Award each claim a UNIQUE id, applying the tie-break rules in the module doc.
/// Returns one id per claim, in input order.
fn settle(claims: &[Claim]) -> Vec<Option<u64>> {
    // Award order: inheritors first (they cannot be displaced by a seeder), then
    // by ascending minimum gid, then by input position — a total order, so the
    // outcome never depends on the order the loops happened to be assembled in.
    let mut order: Vec<usize> = (0..claims.len()).collect();
    order.sort_by_key(|&index| {
        let claim = &claims[index];
        (!claim.inherited, claim.min_gid.unwrap_or(u64::MAX), index)
    });

    let mut taken: Vec<u64> = Vec::with_capacity(claims.len());
    let mut resolved = vec![None; claims.len()];
    for index in order {
        let claim = &claims[index];
        let Some(wanted) = claim.id else { continue };
        let id = if taken.contains(&wanted) {
            // Displaced: fall back to this loop's own lowest gid, else to one
            // above everything in play (ids are just numbers — a re-seed is free
            // to leave the gid space).
            let own = claim.min_gid.filter(|gid| !taken.contains(gid));
            own.unwrap_or_else(|| next_free(claims, &taken))
        } else {
            wanted
        };
        taken.push(id);
        resolved[index] = Some(id);
    }
    resolved
}

/// One above the highest id anywhere in play (awarded, claimed, or a gid) — the
/// last-resort id for a loop displaced off both its stored id and its own gid.
fn next_free(claims: &[Claim], taken: &[u64]) -> u64 {
    let highest = taken
        .iter()
        .copied()
        .chain(claims.iter().flat_map(|claim| claim.id.into_iter()))
        .chain(claims.iter().flat_map(|claim| claim.min_gid.into_iter()))
        .max()
        .unwrap_or(0);
    highest.saturating_add(1)
}


// ===========================================================================
// Write-back — the editor side
// ===========================================================================

/// Assign every closed loop's id onto the geometries of a SOLVED sketch document,
/// in place, and report whether anything changed.
///
/// This is the persistence half of the scheme: the kernel DERIVES ids the same
/// way every run (so a document that has never been through here still names its
/// faces correctly), and this writes the derived ids down so they survive the one
/// edit deriving cannot: deleting the edge an id was seeded from. The editor calls
/// it when a sketch is committed.
///
/// `doc` is the `persistentData.sketch` object. Geometry that is construction-only
/// or not part of a closed loop is left untouched — only a loop has an identity.
pub fn assign_sketch_loop_ids(doc: &mut serde_json::Value) -> bool {
    let Some(loops) = closed_loops(doc) else {
        return false;
    };
    let mut changed = false;
    let Some(geometries) = doc
        .get_mut("geometries")
        .and_then(serde_json::Value::as_array_mut)
    else {
        return false;
    };
    for geometry in geometries.iter_mut() {
        let Some(gid) = geometry.get("id").and_then(numeric) else {
            continue;
        };
        let Some(loop_id) = loops.get(&gid) else {
            continue;
        };
        let already = geometry.get("loopId").and_then(numeric);
        if already == Some(*loop_id) {
            continue;
        }
        if let Some(object) = geometry.as_object_mut() {
            object.insert("loopId".into(), serde_json::json!(loop_id));
            changed = true;
        }
    }
    changed
}

/// `geometry id -> loop id` for every geometry that belongs to a closed loop,
/// derived exactly as the sketch feature derives it: chain the segments, settle
/// the claims, then hand each loop's awarded id to ALL of its members ("lowest
/// number wins and updates the rest of the edges in that loop to match").
fn closed_loops(doc: &serde_json::Value) -> Option<std::collections::HashMap<u64, u64>> {
    let members = loop_members(doc)?;
    let claims: Vec<Claim> = members
        .iter()
        .map(|edges| {
            let stored = edges.iter().filter_map(|(_, stored)| *stored).min();
            let min_gid = edges.iter().map(|(gid, _)| *gid).min();
            Claim {
                id: stored.or(min_gid),
                inherited: stored.is_some(),
                min_gid,
            }
        })
        .collect();
    let mut assignment = std::collections::HashMap::new();
    for (edges, id) in members.iter().zip(settle(&claims)) {
        let Some(id) = id else { continue };
        for (gid, _) in edges {
            assignment.insert(*gid, id);
        }
    }
    Some(assignment)
}

/// Group the document's model geometry into closed loops, as
/// `(geometry id, stored loop id)` per member.
///
/// Whole circles and ellipses are loops on their own. The rest chain end-to-end
/// through shared POINT ids — the same adjacency the profile chainer walks, read
/// off the document's point references rather than solved coordinates, which is
/// exact and needs no tolerance. A chain is closed when it returns to its start.
///
/// At a BRANCHING junction (three or more segments meeting at one point — not a
/// well-formed profile) this walk takes the first candidate, which may group the
/// edges differently than the profile chainer does. That degrades safely rather
/// than corrupting anything: the kernel resolves each loop's id from the ids
/// stored on ITS OWN edges, so a mis-grouped stamp at worst makes two loops claim
/// the same id, which [`settle`] then breaks apart deterministically.
fn loop_members(doc: &serde_json::Value) -> Option<Vec<Vec<(u64, Option<u64>)>>> {
    let geometries = doc.get("geometries")?.as_array()?;
    let mut loops: Vec<Vec<(u64, Option<u64>)>> = Vec::new();
    // (gid, stored, endpoint a, endpoint b) for the chainable segments.
    let mut segments: Vec<(u64, Option<u64>, String, String)> = Vec::new();
    for geometry in geometries {
        if geometry.get("construction").and_then(serde_json::Value::as_bool) == Some(true) {
            continue;
        }
        let Some(gid) = geometry.get("id").and_then(numeric) else {
            continue;
        };
        let stored = geometry.get("loopId").and_then(numeric);
        let geom_type = geometry
            .get("type")
            .and_then(serde_json::Value::as_str)
            .unwrap_or("");
        let points: Vec<String> = geometry
            .get("points")
            .and_then(serde_json::Value::as_array)
            .map(|ids| ids.iter().map(point_key).collect())
            .unwrap_or_default();
        match geom_type {
            // Self-contained loops.
            "circle" | "ellipse" => loops.push(vec![(gid, stored)]),
            // `[start, end]`.
            "line" if points.len() == 2 => {
                segments.push((gid, stored, points[0].clone(), points[1].clone()))
            }
            // `[center, start, end]`.
            "arc" if points.len() == 3 => {
                segments.push((gid, stored, points[1].clone(), points[2].clone()))
            }
            // `[p0, …, pn]` — the spline's endpoints are its first and last.
            "bezier" if points.len() >= 2 => {
                segments.push((gid, stored, points[0].clone(), points[points.len() - 1].clone()))
            }
            _ => {}
        }
    }

    // Walk each connected component; keep it only if it closes.
    let mut visited = vec![false; segments.len()];
    for start in 0..segments.len() {
        if visited[start] {
            continue;
        }
        let first_point = segments[start].2.clone();
        let mut chain = vec![start];
        visited[start] = true;
        let mut tail = segments[start].3.clone();
        loop {
            if tail == first_point {
                // Closed: a chain of one segment closes only if it is a loop on
                // its own (a full-circle arc), which the endpoints already say.
                loops.push(
                    chain
                        .iter()
                        .map(|&index| (segments[index].0, segments[index].1))
                        .collect(),
                );
                break;
            }
            let next = (0..segments.len()).find(|&index| {
                !visited[index] && (segments[index].2 == tail || segments[index].3 == tail)
            });
            let Some(next) = next else { break };
            visited[next] = true;
            chain.push(next);
            tail = if segments[next].2 == tail {
                segments[next].3.clone()
            } else {
                segments[next].2.clone()
            };
        }
    }
    Some(loops)
}

/// A POINT id as a join key: a numeric id (number or numeric string) by its
/// number, so `3` and `"3"` meet as [`numeric`] reads them, and any other id —
/// the part-origin reference point's `"origin"` — by its JSON text, which no
/// number can spell.
fn point_key(value: &serde_json::Value) -> String {
    match numeric(value) {
        Some(number) => number.to_string(),
        None => value.to_string(),
    }
}

/// A JSON id as a number (a numeric string counts) — the write-back's copy of the
/// sketch pipeline's `numeric_id`, so both halves read ids identically.
fn numeric(value: &serde_json::Value) -> Option<u64> {
    match value {
        serde_json::Value::Number(number) => number.as_u64(),
        serde_json::Value::String(text) => text.parse().ok(),
        _ => None,
    }
}

