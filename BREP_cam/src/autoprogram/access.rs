//! Protect open-edge extensions against the complete part above the floor.
use super::*;
use std::collections::BTreeMap;

fn floor(feature: &Feature) -> Option<(&Region, f64)> {
    match &feature.kind {
        FeatureKind::Pocket { floor, floor_z, open_edges, .. } | FeatureKind::Step { floor, floor_z, open_edges, .. }
            if !open_edges.is_empty() => Some((floor, *floor_z)),
        _ => None,
    }
}

fn shadow(mesh: &MeshSnapshot, z: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let level = z + tol.distance_eps;
    let mut loops = Vec::new();
    for triangle in &mesh.triangles {
        if let Some(boundary) = crate::features::recognize::projected_above(triangle, level, tol) { loops.push(boundary); }
    }
    if loops.is_empty() { return Ok(Region::empty()); }
    Ok(crate::planar::boolean(&Region { loops }, &Region::empty(), crate::planar::BoolOp::Union, tol)?)
}

fn clip(geometry: &mut OperationGeometry, floor: &Region, shadow: &Region, tol: &Tolerance) -> Result<(), CamError> {
    let OperationGeometry::Planar(input) = geometry else { return Ok(()); };
    for region in &mut input.regions {
        // Keep the exact, already access-certified floor. Mesh chords across
        // curved openings must not replace its analytic boundary.
        let extension = region.region.difference(floor, tol)?;
        if extension.is_empty() { continue; }
        let allowed = extension.difference(shadow, tol)?;
        region.region = floor.union(&allowed, tol)?;
    }
    Ok(())
}

/// Call after choosing document/fallback rules, before binding geometry. The
/// projected material is shared by operations at the same floor elevation.
pub fn constrain_open_extensions(plan: &mut Plan, recognized: &Recognized, mesh: &MeshSnapshot, tol: &Tolerance) -> Result<(), CamError> {
    let mut shadows = BTreeMap::new();
    for planned in &mut plan.operations {
        if !matches!(planned.operation.strategy, Strategy::Pocket2D(_)) { continue; }
        let feature = recognized.features.iter().find(|f| f.id == planned.feature).ok_or_else(|| invalid("planned feature disappeared"))?;
        let Some((floor, z)) = floor(feature) else { continue; };
        if !shadows.contains_key(&z.to_bits()) { shadows.insert(z.to_bits(), shadow(mesh, z, tol).map_err(|e| invalid(format!("feature {} open-edge projection: {e}", feature.id)))?); }
        clip(&mut planned.geometry, floor, &shadows[&z.to_bits()], tol).map_err(|e| invalid(format!("feature {} open-edge clipping: {e}", feature.id)))?;
    }
    Ok(())
}

pub(crate) fn constrain_rebound(geometry: &mut OperationGeometry, feature: &Feature, mesh: &MeshSnapshot, tol: &Tolerance) -> Result<(), CamError> {
    if let Some((floor, z)) = floor(feature) { clip(geometry, floor, &shadow(mesh, z, tol)?, tol)?; }
    Ok(())
}

