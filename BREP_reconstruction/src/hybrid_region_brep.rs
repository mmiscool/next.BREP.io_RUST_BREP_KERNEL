//! Conservative local BREP reconstruction from the kernel's public mesh segmentation.
//!
//! Plane and safely bounded cylinder/cone/sphere regions remain analytic.
//! Unsupported regions, and revolved regions whose interfaces cannot be shared
//! exactly, are represented by maximal edge-connected, provably coplanar planar
//! patches. A sphere is framed as a revolution about the common axis of the
//! rings that bound it; a cap around the pole carries the seam and the
//! degenerate pole edge of the kernel's own sphere solid. A sphere whose
//! plane rings lie on several axes, and two equal-radius cylinders whose axes
//! cross, share exact circles and ellipses whose pcurves are fitted; a
//! whole-body failure after such a lane demotes only its regions.

use brep_kernel::{
    make_sphere_surface_framed,
    build_pcurve_on_surface_range, circle_angle_to_parameter, intersect_analytic_pair, make_arc,
    make_line, make_revolution, mesh_to_faceted_brep, project_point_to_curve, segment_mesh_faces,
    solid_signed_volume, ArenaCoedge, ArenaEdge, ArenaFace, ArenaLoop, ArenaShell, ArenaVertex,
    BrepSolid, EdgeId, FaceId, MeshRegion, MeshSegmentation, NurbsCurve, NurbsSurface, RegionCarrier,
    SegmentOptions,
    TopologyArena, Vec3, Vec4, KNOT_IDENTITY_TOL,
};
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::f64::consts::TAU;

#[derive(Clone, Copy)]
struct PlaneInfo {
    origin: Vec3,
    normal: Vec3,
}

#[derive(Clone, Copy)]
struct CylinderCarrier {
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    sense: i8,
    /// Largest radial deviation of a source vertex the segmenter accepted
    /// for this carrier: how far a ring vertex may sit off the exact circle.
    deviation: f64,
}

#[derive(Clone, Copy)]
struct SphereCarrier {
    center: Vec3,
    radius: f64,
    sense: i8,
    /// Largest radial deviation of a source vertex the segmenter accepted
    /// for this carrier.
    deviation: f64,
}

/// A torus region's fitted carrier.
#[derive(Clone, Copy)]
struct TorusCarrier {
    center: Vec3,
    axis: Vec3,
    major_radius: f64,
    minor_radius: f64,
    sense: i8,
    /// Largest deviation of a source vertex the segmenter accepted for
    /// this carrier.
    deviation: f64,
}

#[derive(Clone, Copy)]
struct ConeCarrier {
    apex: Vec3,
    axis: Vec3,
    half_angle: f64,
    sense: i8,
    /// Largest deviation of a source vertex the segmenter accepted for
    /// this carrier.
    deviation: f64,
}

/// How far a ring or arc vertex may sit off its exact circle: the exact
/// topology tolerance, or twice the deviation the segmenter already
/// accepted for the carrier when that is wider (twice, because the carrier
/// is snapped to its neighbors after the acceptance was measured). A vertex
/// the tessellator left on a rim chord is still on the ring the exact
/// circle replaces.
fn accepted_radial_bar(tolerance: f64, deviation: f64) -> f64 {
    tolerance.max(2.0 * deviation)
}

struct CylinderInfo {
    carrier: CylinderCarrier,
    base_station: f64,
    height: f64,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
}

struct ConeInfo {
    carrier: ConeCarrier,
    /// The region reaches the apex: its surface starts there (station 0,
    /// v = 0) and its face closes through the seam and a degenerate apex
    /// edge, the way the kernel's own pointed cone solid is built.
    pointed: bool,
    base_station: f64,
    height: f64,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
}

struct SphereInfo {
    carrier: SphereCarrier,
    /// Polar axis of the revolution parameterization: the common normal of
    /// the ring planes bounding the region, pointing into the region; or,
    /// when the rings are on several axes, one whose poles both lie in
    /// removed caps (see [`tilted_sphere_frame`]).
    axis: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
    /// The region is a cap around the north pole (one ring), so its face
    /// carries the seam and the degenerate pole edge.
    north_pole_inside: bool,
}

/// A torus band between coaxial rings, on the kernel's own torus template
/// (the tube circle starts at the outer equator and turns toward +axis), so
/// it recognizes as a torus and writes TOROIDAL_SURFACE. The frame axis is
/// the carrier's or its reverse, whichever keeps the band clear of the
/// template's tube seam; the seam meridian is a coaxial neighbour's, so a
/// ring shared with it is one parameter line on both faces.
struct TorusInfo {
    carrier: TorusCarrier,
    axis: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    /// Tube angle at the band's middle, measured in the frame: the band's
    /// tube angles are unwrapped around it into [0, 2π].
    band_middle: f64,
    surface: NurbsSurface,
}

#[derive(Clone)]
struct Chain {
    vertices: Vec<usize>,
    closed: bool,
    adjacent: (u32, u32),
}

#[derive(Clone, Copy)]
struct Traversal {
    chain: usize,
    forward: bool,
}

struct RegionBoundary {
    cycles: Vec<Vec<Traversal>>,
}

struct LocalRegions {
    triangle_regions: Vec<u32>,
    planes: HashMap<u32, PlaneInfo>,
    cylinders: HashMap<u32, CylinderCarrier>,
    cones: HashMap<u32, ConeCarrier>,
    spheres: HashMap<u32, SphereCarrier>,
    tori: HashMap<u32, TorusCarrier>,
    local_to_original: HashMap<u32, u32>,
    count: usize,
    stats: HybridBrepStats,
}

enum HybridBuildFailure {
    Region { original: u32, message: String },
    /// The assembled shell failed a whole-body check after regions were
    /// built on a lane whose edges carry fitted pcurves on a sphere or
    /// cylinder (a tilted sphere ring, a crossing-bore ellipse). Those
    /// regions are demoted together and the build retried, so a new lane
    /// can only ever cost its own regions, never the body.
    Speculative { originals: Vec<u32>, message: String },
    Global(String),
}

impl From<String> for HybridBuildFailure {
    fn from(message: String) -> Self {
        Self::Global(message)
    }
}

impl From<&str> for HybridBuildFailure {
    fn from(message: &str) -> Self {
        Self::Global(message.to_owned())
    }
}

/// Auditable accounting for an analytic/faceted reconstruction.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, serde::Serialize)]
pub(crate) struct HybridBrepStats {
    pub(crate) total_faces: usize,
    pub(crate) analytic_plane_faces: usize,
    pub(crate) analytic_plane_triangles: usize,
    pub(crate) analytic_cylinder_faces: usize,
    pub(crate) analytic_cylinder_triangles: usize,
    pub(crate) analytic_cone_faces: usize,
    pub(crate) analytic_cone_triangles: usize,
    pub(crate) analytic_sphere_faces: usize,
    pub(crate) analytic_sphere_triangles: usize,
    pub(crate) analytic_torus_faces: usize,
    pub(crate) analytic_torus_triangles: usize,
    pub(crate) faceted_faces: usize,
    pub(crate) faceted_triangles: usize,
    pub(crate) demoted_regions: usize,
    pub(crate) demoted_triangles: usize,
    pub(crate) segmentation_plane_regions: usize,
    pub(crate) segmentation_cylinder_regions: usize,
    pub(crate) segmentation_cone_regions: usize,
    pub(crate) segmentation_sphere_regions: usize,
    pub(crate) segmentation_torus_regions: usize,
    pub(crate) segmentation_unsupported_regions: usize,
}

/// Validated reconstructed solid and the accounting used to select policy.
pub(crate) struct HybridBrepOutput {
    pub(crate) solid: BrepSolid,
    pub(crate) stats: HybridBrepStats,
}

#[derive(Clone, Copy)]
struct ChainUv {
    region: u32,
    start: [f64; 2],
    end: [f64; 2],
}

struct BuiltEdge {
    edge: EdgeId,
    curve_along_chain: bool,
    revolve_uvs: Vec<ChainUv>,
    custom_revolve_pcurves: HashMap<u32, NurbsCurve>,
}

struct Ids(u64);

impl Ids {
    fn next(&mut self) -> u64 {
        let value = self.0;
        self.0 += 1;
        value
    }
}

/// Rebuild a validated analytic/faceted shell from public segmentation.
///
/// This is intentionally narrower than a general mesh-to-BREP converter.
/// Plane and safely bounded cylinder/cone regions remain analytic. Unsupported
/// regions become maximal edge-connected coplanar planar patches. A cylinder touching
/// those facets is retained only when every shared edge is an exact axial
/// ruling; otherwise the complete cylinder region is recursively demoted to
/// planar triangle faces. Every shared boundary must still be provably a line,
/// circular arc, complete ring, or cylinder ruling.
pub(crate) fn hybrid_plane_cylinder_brep(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
) -> Result<HybridBrepOutput, String> {
    hybrid_plane_cylinder_brep_retry(positions, indices, options, None)
}

/// Localized failures demote one region per attempt; past this many, the
/// mesh is not a case of a few unsafe regions but of a segmentation the
/// builder cannot use, and the faceted fallback is the honest answer.
const HYBRID_RETRY_LIMIT: usize = 512;

fn hybrid_plane_cylinder_brep_retry(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
    mut injected_failure: Option<u32>,
) -> Result<HybridBrepOutput, String> {
    // The segmentation does not depend on the demotions; run it once.
    let owned_indices;
    let index_buffer: &[u32] = match indices {
        Some(value) => value,
        None => {
            owned_indices = (0..positions.len() as u32 / 3).collect::<Vec<_>>();
            &owned_indices
        }
    };
    let segmentation = segment_mesh_faces(positions, index_buffer, options)?;
    // A faceted seed gives us a public, type-safe TopologyArena without
    // depending directly on SlotMap.  The seed is also an independent closed-
    // manifold precondition.  Its topology is cleared before reconstruction;
    // built once, it is cloned per attempt.
    let seed = mesh_to_faceted_brep(positions, Some(index_buffer), options.weld_tolerance)?;
    let mut blank = TopologyArena::from_brep(&seed)?;
    blank.vertices.clear();
    blank.edges.clear();
    blank.coedges.clear();
    blank.loops.clear();
    blank.faces.clear();
    blank.shells.clear();
    blank.wire_solid_id = 1;
    blank.genus = 0;
    let mut forced_demotions = BTreeSet::new();
    loop {
        let attempt = if let Some(original) = injected_failure.take() {
            Err(HybridBuildFailure::Region {
                original,
                message: "injected localized carrier construction failure".into(),
            })
        } else {
            hybrid_plane_cylinder_brep_once(
                positions,
                index_buffer,
                options,
                &segmentation,
                blank.clone(),
                &forced_demotions,
            )
        };
        match attempt {
            Ok(output) => return Ok(output),
            Err(HybridBuildFailure::Region { original, message }) => {
                if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
                    eprintln!("[hybrid] retry: region {original} failed: {message}");
                }
                if !forced_demotions.insert(original) {
                    return Err(format!(
                        "hybrid BREP: region {original} remained unconstructible after local demotion: {message}"
                    ));
                }
                if forced_demotions.len() > HYBRID_RETRY_LIMIT {
                    return Err(format!(
                        "hybrid BREP: more than {HYBRID_RETRY_LIMIT} regions needed local demotion; last: region {original}: {message}"
                    ));
                }
            }
            Err(HybridBuildFailure::Speculative { originals, message }) => {
                if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
                    eprintln!("[hybrid] retry: fitted-pcurve regions {originals:?} failed: {message}");
                }
                let before = forced_demotions.len();
                forced_demotions.extend(originals);
                if forced_demotions.len() == before {
                    return Err(message);
                }
                if forced_demotions.len() > HYBRID_RETRY_LIMIT {
                    return Err(format!(
                        "hybrid BREP: more than {HYBRID_RETRY_LIMIT} regions needed local demotion; last: {message}"
                    ));
                }
            }
            Err(HybridBuildFailure::Global(message)) => return Err(message),
        }
    }
}

fn hybrid_plane_cylinder_brep_once(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    segmentation: &MeshSegmentation,
    mut arena: TopologyArena,
    forced_demotions: &BTreeSet<u32>,
) -> Result<HybridBrepOutput, HybridBuildFailure> {
    if positions.is_empty()
        || !positions.len().is_multiple_of(3)
        || !indices.len().is_multiple_of(3)
    {
        return Err("hybrid BREP: invalid triangle buffers".into());
    }
    let mut vertices = positions
        .chunks_exact(3)
        .map(|p| Vec3::new(p[0], p[1], p[2]))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|t| [t[0] as usize, t[1] as usize, t[2] as usize])
        .collect::<Vec<_>>();
    if triangles
        .iter()
        .flatten()
        .any(|&vertex| vertex >= vertices.len())
    {
        return Err("hybrid BREP: triangle index outside vertex buffer".into());
    }

    if segmentation.welded_vertex_count != vertices.len() {
        return Err(format!(
            "hybrid BREP: segmentation welded {} input vertices to {}; pre-weld the mesh before exact reconstruction",
            vertices.len(), segmentation.welded_vertex_count
        ).into());
    }
    if segmentation.triangle_region_ids.len() != triangles.len() {
        return Err("hybrid BREP: segmentation triangle count drift".into());
    }

    let diagonal = mesh_diagonal(&vertices);
    // Exact shared topology must be substantially tighter than carrier
    // recognition. This still covers deterministic f32 STL quantization.
    let tolerance = (options.fit_tolerance * diagonal)
        .min(2.0e-6 * diagonal)
        .max(1.0e-10);
    let incidence = edge_incidence(&triangles)?;
    if incidence.values().any(|incident| incident.len() != 2) {
        return Err("hybrid BREP: input is not a closed two-manifold".into());
    }
    let local = local_regions(
        &segmentation.regions,
        &segmentation.triangle_region_ids,
        &triangles,
        &vertices,
        &incidence,
        tolerance,
        forced_demotions,
    )?;
    let stats = local.stats;
    let local_to_original = local.local_to_original;
    let planes = local.planes;
    let mut cylinder_carriers = local.cylinders;
    let mut cone_carriers = local.cones;
    let sphere_carriers = local.spheres;
    let torus_carriers = local.tori;
    let mut region_centroids = HashMap::<u32, (Vec3, usize)>::new();
    let mut region_samples = HashMap::<u32, Vec<Vec3>>::new();
    for (triangle, &region) in local.triangle_regions.iter().enumerate() {
        if !sphere_carriers.contains_key(&region) && !torus_carriers.contains_key(&region) {
            continue;
        }
        let [a, b, c] = triangles[triangle];
        let centroid = vertices[a].add(vertices[b]).add(vertices[c]).scale(1.0 / 3.0);
        region_samples.entry(region).or_default().push(centroid);
        let entry = region_centroids.entry(region).or_insert((Vec3::new(0.0, 0.0, 0.0), 0));
        entry.0 = entry.0.add(centroid);
        entry.1 += 1;
    }
    let region_centroids = region_centroids
        .into_iter()
        .map(|(region, (sum, count))| (region, sum.scale(1.0 / count as f64)))
        .collect::<HashMap<_, _>>();
    stage_trace("local regions done");
    let (chains, mut boundaries) =
        region_boundaries(&triangles, &local.triangle_regions, local.count, &incidence)?;
    let is_revolve = |region: u32| {
        cylinder_carriers.contains_key(&region)
            || cone_carriers.contains_key(&region)
            || sphere_carriers.contains_key(&region)
            || torus_carriers.contains_key(&region)
    };
    let chains = split_bent_planar_chains(chains, &mut boundaries, &vertices, tolerance, is_revolve);
    stage_trace("boundaries done");

    snap_cylinder_axes(
        &mut cylinder_carriers,
        &planes,
        &chains,
        &vertices,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
    })?;
    snap_cone_carriers(
        &mut cone_carriers,
        &cylinder_carriers,
        &planes,
        &chains,
        &vertices,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
    })?;
    snap_cylinder_ruling_vertices(
        &mut vertices,
        &chains,
        &planes,
        &cylinder_carriers,
        tolerance,
    );
    let pointed_cones = pointed_cone_regions(
        &cone_carriers,
        &local.triangle_regions,
        &triangles,
        &vertices,
        &chains,
        tolerance,
    );
    for (&region, cone) in &cone_carriers {
        if pointed_cones.contains(&region) {
            continue;
        }
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
        {
            for point in chain_points(chain, &vertices) {
                let station = cone_station(*cone, point);
                low = low.min(station);
                high = high.max(station);
            }
        }
        validate_cone_span(
            local_to_original[&region],
            low,
            high,
            cone.half_angle,
            tolerance,
        )?;
    }
    stage_trace("axes snapped");
    let cylinders = build_cylinder_surfaces(&cylinder_carriers, &chains, &vertices, tolerance)
        .map_err(|(region, message)| HybridBuildFailure::Region {
            original: local_to_original[&region],
            message,
        })?;
    let cones = build_cone_surfaces(
        &cone_carriers,
        &pointed_cones,
        &cylinders,
        &chains,
        &vertices,
        tolerance,
    )
        .map_err(|(region, message)| HybridBuildFailure::Region {
            original: local_to_original[&region],
            message,
        })?;
    let spheres = build_sphere_surfaces(
        &sphere_carriers,
        &planes,
        &cylinders,
        &cones,
        &chains,
        &vertices,
        &region_centroids,
        &region_samples,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
    })?;
    let tori = build_torus_surfaces(
        &torus_carriers,
        &cylinders,
        &cones,
        &chains,
        &vertices,
        &region_samples,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
    })?;
    stage_trace("surfaces built");
    sort_region_cycles(
        &mut boundaries,
        &planes,
        &cylinders,
        &cones,
        &spheres,
        &tori,
        &chains,
        &vertices,
    );
    stage_trace("cycles sorted");

    let mut ids = Ids(2);
    let mut arena_vertices = HashMap::new();
    // The regions of the chain that first placed each endpoint vertex.  A
    // line between two planes can meet an arc whose exact end disagrees
    // with the mesh vertex (a revolved region that absorbed chord points
    // where a boolean cut its tessellation); the line itself has no region
    // to demote, so the blame goes to the revolve that placed the vertex.
    let mut vertex_placers = HashMap::<usize, (u32, u32)>::new();

    let mut built_edges = Vec::with_capacity(chains.len());
    let chain_trace = std::env::var("BREP_DEBUG_HYBRID").is_ok();
    for (chain_index, chain) in chains.iter().enumerate() {
        let involves_revolve = [chain.adjacent.0, chain.adjacent.1].iter().any(|region| {
            cylinders.contains_key(region)
                || cones.contains_key(region)
                || spheres.contains_key(region)
                || tori.contains_key(region)
        });
        if chain_trace && involves_revolve {
            let kind = |region: u32| {
                if planes.contains_key(&region) {
                    "plane"
                } else if cylinders.contains_key(&region) {
                    "cylinder"
                } else if cones.contains_key(&region) {
                    "cone"
                } else if spheres.contains_key(&region) {
                    "sphere"
                } else if tori.contains_key(&region) {
                    "torus"
                } else {
                    "?"
                }
            };
            eprintln!(
                "[hybrid] chain {chain_index}: {} {} / {} {}, {} points, closed={}",
                chain.adjacent.0,
                kind(chain.adjacent.0),
                chain.adjacent.1,
                kind(chain.adjacent.1),
                chain.vertices.len(),
                chain.closed
            );
        }
        let built = match build_chain_edge(
            chain_index,
            chain,
            &vertices,
            &planes,
            &cylinders,
            &cones,
            &spheres,
            &tori,
            tolerance,
            &mut arena,
            &mut arena_vertices,
            &mut ids,
        ) {
            Ok(built) => built,
            Err(message) => {
                let local = [chain.adjacent.0, chain.adjacent.1]
                    .into_iter()
                    .find(|region| spheres.contains_key(region) || tori.contains_key(region))
                    .or_else(|| {
                        [chain.adjacent.0, chain.adjacent.1]
                            .into_iter()
                            .find(|region| cones.contains_key(region))
                    })
                    .or_else(|| {
                        [chain.adjacent.0, chain.adjacent.1]
                            .into_iter()
                            .find(|region| cylinders.contains_key(region))
                    });
                let local = local.or_else(|| {
                    let endpoints = [chain.vertices.first(), chain.vertices.last()];
                    endpoints
                        .into_iter()
                        .flatten()
                        .filter_map(|vertex| vertex_placers.get(vertex))
                        .flat_map(|&(a, b)| [a, b])
                        .find(|region| {
                            cylinders.contains_key(region)
                                || cones.contains_key(region)
                                || spheres.contains_key(region)
                                || tori.contains_key(region)
                        })
                });
                if let Some(local) = local {
                    return Err(HybridBuildFailure::Region {
                        original: local_to_original[&local],
                        message,
                    });
                }
                return Err(HybridBuildFailure::Global(message));
            }
        };
        for vertex in [chain.vertices.first(), chain.vertices.last()].into_iter().flatten() {
            vertex_placers.entry(*vertex).or_insert(chain.adjacent);
        }
        built_edges.push(built);
    }

    stage_trace("chain edges built");
    let mut faces = Vec::<FaceId>::new();
    for region in 0..local.count as u32 {
        let face = if let Some(plane) = planes.get(&region) {
            build_plane_face(
                region,
                *plane,
                &boundaries[region as usize],
                &chains,
                &built_edges,
                &vertices,
                &mut arena,
                &mut ids,
            )?
        } else if let Some(cylinder) = cylinders.get(&region) {
            build_cylinder_face(
                region,
                cylinder,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
            })?
        } else if let Some(torus) = tori.get(&region) {
            build_revolve_band_face(
                region,
                "torus",
                &torus.surface,
                torus.carrier.sense == 1,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
            })?
        } else if let Some(sphere) = spheres.get(&region) {
            build_sphere_face(
                region,
                sphere,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
            })?
        } else {
            build_cone_face(
                region,
                &cones[&region],
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
            })?
        };
        faces.push(face);
    }
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        for (region, &face) in faces.iter().enumerate() {
            let record = &arena.faces[face];
            let kind = if planes.contains_key(&(region as u32)) {
                "plane"
            } else if cylinders.contains_key(&(region as u32)) {
                "cylinder"
            } else if cones.contains_key(&(region as u32)) {
                "cone"
            } else if tori.contains_key(&(region as u32)) {
                "torus"
            } else {
                "sphere"
            };
            let coedges = record
                .loops
                .iter()
                .flat_map(|&l| arena.loops[l].coedges.iter())
                .map(|&c| {
                    let coedge = &arena.coedges[c];
                    let edge = &arena.edges[coedge.edge];
                    format!("c{}:e{}{}", coedge.wire_id, edge.wire_id, if edge.degenerate { "*" } else { "" })
                })
                .collect::<Vec<_>>()
                .join(" ");
            eprintln!(
                "[hybrid] face {} = region {region} ({kind}, original {}): {coedges}",
                record.wire_id,
                local_to_original.get(&(region as u32)).copied().unwrap_or(u32::MAX)
            );
        }
    }
    arena.shells.insert(ArenaShell {
        wire_id: ids.next(),
        faces,
    });
    stage_trace("faces built");
    let speculative = built_edges
        .iter()
        .flat_map(|built| built.custom_revolve_pcurves.keys())
        .filter(|region| spheres.contains_key(region) || cylinders.contains_key(region))
        .map(|region| local_to_original[region])
        .collect::<BTreeSet<_>>();
    finish_shell(arena, stats).map_err(|message| {
        if speculative.is_empty() {
            HybridBuildFailure::Global(message)
        } else {
            HybridBuildFailure::Speculative {
                originals: speculative.into_iter().collect(),
                message,
            }
        }
    })
}

/// Convert the assembled arena and run the whole-body checks.
fn finish_shell(arena: TopologyArena, stats: HybridBrepStats) -> Result<HybridBrepOutput, String> {
    let mut solid = arena.to_brep()?;
    stage_trace("arena converted");
    let vertex_count = solid.vertices.len() as i64;
    // A degenerate pole edge is not a 1-cell of the shell.
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let face_count = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len())
        .sum::<usize>() as i64;
    let extra_loops = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.loops.len().saturating_sub(1) as i64)
        .sum::<i64>();
    let numerator = 2 - (vertex_count - edge_count + face_count - extra_loops);
    if numerator < 0 || numerator % 2 != 0 {
        return Err(format!(
            "hybrid BREP: non-manifold Euler accounting (2-(V-E+F-H)={numerator})"
        ));
    }
    solid.genus = numerator / 2;
    let issues = solid.validate();
    stage_trace("validated");
    if !issues.is_empty() {
        return Err(format!("hybrid BREP: topology validation failed: {issues:?}"));
    }
    let volume = solid_signed_volume(&solid)?;
    stage_trace("volume computed");
    if !volume.is_finite() || volume <= 0.0 {
        return Err(format!(
            "hybrid BREP: reconstructed shell is inverted or empty (signed volume {volume:.6e})"
        ));
    }
    Ok(HybridBrepOutput { solid, stats })
}

fn validate_cone_span(
    original: u32,
    low: f64,
    high: f64,
    half_angle: f64,
    tolerance: f64,
) -> Result<(), HybridBuildFailure> {
    if !low.is_finite() || high - low <= tolerance || low * half_angle.tan() <= tolerance {
        return Err(HybridBuildFailure::Region {
            original,
            message: "pointed or zero-span cone requires unsupported pole topology".into(),
        });
    }
    Ok(())
}

fn mesh_diagonal(vertices: &[Vec3]) -> f64 {
    let mut low = vertices[0];
    let mut high = vertices[0];
    for &point in &vertices[1..] {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    high.sub(low).length()
}

type EdgeKey = (usize, usize);

fn edge_key(a: usize, b: usize) -> EdgeKey {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn edge_incidence(triangles: &[[usize; 3]]) -> Result<BTreeMap<EdgeKey, Vec<usize>>, String> {
    let mut incidence = BTreeMap::<EdgeKey, Vec<usize>>::new();
    for (triangle_id, triangle) in triangles.iter().enumerate() {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return Err("hybrid BREP: repeated vertex in triangle".into());
        }
        for corner in 0..3 {
            incidence
                .entry(edge_key(triangle[corner], triangle[(corner + 1) % 3]))
                .or_default()
                .push(triangle_id);
        }
    }
    Ok(incidence)
}

fn carrier_cylinder(region: &MeshRegion) -> Result<Option<CylinderCarrier>, String> {
    let RegionCarrier::Cylinder {
        axis_point,
        axis_dir,
        radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !radius.is_finite() || radius <= 0.0 || !matches!(sense, -1 | 1) {
        return Ok(None);
    }
    Ok(Some(CylinderCarrier {
        origin: axis_point,
        axis: axis_dir.normalized()?,
        radius,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn carrier_cone(region: &MeshRegion) -> Result<Option<ConeCarrier>, String> {
    let RegionCarrier::Cone {
        apex,
        axis_dir,
        half_angle_rad,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !half_angle_rad.is_finite()
        || !(1.0e-6..std::f64::consts::FRAC_PI_2 - 1.0e-6).contains(&half_angle_rad)
        || !matches!(sense, -1 | 1)
    {
        return Ok(None);
    }
    Ok(Some(ConeCarrier {
        apex,
        axis: axis_dir.normalized()?,
        half_angle: half_angle_rad,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn carrier_sphere(region: &MeshRegion) -> Result<Option<SphereCarrier>, String> {
    let RegionCarrier::Sphere {
        center,
        radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !radius.is_finite() || radius <= 0.0 || !matches!(sense, -1 | 1) {
        return Ok(None);
    }
    Ok(Some(SphereCarrier {
        center,
        radius,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn sphere_radial_error(sphere: SphereCarrier, point: Vec3) -> f64 {
    (point.sub(sphere.center).length() - sphere.radius).abs()
}

/// A sphere/plane edge is a circle whenever both ends lie in the plane and
/// on the sphere; whether that circle is a ring of the chosen polar axis is
/// settled when the sphere surface is framed.
fn sphere_plane_edge_supported(
    sphere: SphereCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let bar = accepted_radial_bar(tolerance, sphere.deviation);
    a.sub(plane.origin).dot(normal).abs() <= tolerance
        && b.sub(plane.origin).dot(normal).abs() <= tolerance
        && sphere_radial_error(sphere, a) <= bar
        && sphere_radial_error(sphere, b) <= bar
}

/// A sphere meets a coaxial revolve (its center on the axis) along a ring:
/// both ends at one station on the axis, on the sphere and on the revolve.
fn sphere_revolve_edge_supported(
    sphere: SphereCarrier,
    axis_origin: Vec3,
    axis: Vec3,
    on_revolve: impl Fn(Vec3) -> bool,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let offset = sphere.center.sub(axis_origin);
    if offset.sub(axis.scale(offset.dot(axis))).length() > tolerance * 4.0 {
        return false;
    }
    let bar = accepted_radial_bar(tolerance * 4.0, sphere.deviation);
    (a.sub(axis_origin).dot(axis) - b.sub(axis_origin).dot(axis)).abs() <= tolerance * 4.0
        && sphere_radial_error(sphere, a) <= bar
        && sphere_radial_error(sphere, b) <= bar
        && on_revolve(a)
        && on_revolve(b)
}

fn sphere_cylinder_edge_supported(
    sphere: SphereCarrier,
    cylinder: CylinderCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    sphere_revolve_edge_supported(
        sphere,
        cylinder.origin,
        cylinder.axis,
        |point| {
            (radial(cylinder, point).length() - cylinder.radius).abs()
                <= accepted_radial_bar(tolerance * 4.0, cylinder.deviation)
        },
        a,
        b,
        tolerance,
    )
}

fn sphere_cone_edge_supported(
    sphere: SphereCarrier,
    cone: ConeCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    sphere_revolve_edge_supported(
        sphere,
        cone.apex,
        cone.axis,
        |point| point_on_cone(cone, point, accepted_radial_bar(tolerance * 4.0, cone.deviation)),
        a,
        b,
        tolerance,
    )
}

fn carrier_torus(region: &MeshRegion) -> Result<Option<TorusCarrier>, String> {
    let RegionCarrier::Torus {
        center,
        axis_dir,
        major_radius,
        minor_radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    let Ok(axis) = axis_dir.normalized() else {
        return Ok(None);
    };
    if !major_radius.is_finite()
        || !minor_radius.is_finite()
        || minor_radius <= 0.0
        || major_radius <= minor_radius
        || !matches!(sense, -1 | 1)
    {
        return Ok(None);
    }
    Ok(Some(TorusCarrier {
        center,
        axis,
        major_radius,
        minor_radius,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn torus_radial_error(torus: TorusCarrier, point: Vec3) -> f64 {
    let d = point.sub(torus.center);
    let z = d.dot(torus.axis);
    let rho = d.sub(torus.axis.scale(z)).length();
    (((rho - torus.major_radius).powi(2) + z * z).sqrt() - torus.minor_radius).abs()
}

/// A torus meets a plane square to its axis along a latitude ring: both
/// ends in the plane and on the torus.
fn torus_plane_edge_supported(
    torus: TorusCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let bar = accepted_radial_bar(tolerance, torus.deviation);
    normal.cross(torus.axis).length() * torus.major_radius <= tolerance * 4.0
        && a.sub(plane.origin).dot(normal).abs() <= tolerance
        && b.sub(plane.origin).dot(normal).abs() <= tolerance
        && torus_radial_error(torus, a) <= bar
        && torus_radial_error(torus, b) <= bar
}

/// A torus meets a coaxial revolve (the same axis line) along a latitude
/// ring: both ends at one station, on the torus and on the revolve.
fn torus_revolve_edge_supported(
    torus: TorusCarrier,
    axis_origin: Vec3,
    axis: Vec3,
    on_revolve: impl Fn(Vec3) -> bool,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let offset = torus.center.sub(axis_origin);
    if offset.sub(axis.scale(offset.dot(axis))).length() > tolerance * 4.0
        || axis.cross(torus.axis).length() * torus.major_radius > tolerance * 4.0
    {
        return false;
    }
    let bar = accepted_radial_bar(tolerance * 4.0, torus.deviation);
    (a.sub(axis_origin).dot(axis) - b.sub(axis_origin).dot(axis)).abs() <= tolerance * 4.0
        && torus_radial_error(torus, a) <= bar
        && torus_radial_error(torus, b) <= bar
        && on_revolve(a)
        && on_revolve(b)
}

fn edge_is_cylinder_ruling(cylinder: CylinderCarrier, a: Vec3, b: Vec3, tolerance: f64) -> bool {
    let ra = radial(cylinder, a);
    let rb = radial(cylinder, b);
    let Ok(ua) = ra.normalized() else {
        return false;
    };
    let Ok(ub) = rb.normalized() else {
        return false;
    };
    let chord = b.sub(a);
    let Ok(direction) = chord.normalized() else {
        return false;
    };
    (ra.length() - cylinder.radius).abs() <= tolerance
        && (rb.length() - cylinder.radius).abs() <= tolerance
        && ua.cross(ub).length() <= 0.01
        && direction.cross(cylinder.axis).length() <= 0.01
}

/// Where two cylinders of one radius whose axes cross meet: the two planes
/// through the crossing point normal to the sum and to the difference of the
/// axes (each cuts both walls in the same ellipse), or `None` when the pair
/// is not such a crossing within the exact-topology bar.
fn cylinder_pair_section(
    first: CylinderCarrier,
    second: CylinderCarrier,
    tolerance: f64,
) -> Option<(Vec3, [Vec3; 2])> {
    let bar = accepted_radial_bar(tolerance * 4.0, first.deviation.max(second.deviation));
    if (first.radius - second.radius).abs() > bar {
        return None;
    }
    let (a1, a2) = (first.axis.normalized().ok()?, second.axis.normalized().ok()?);
    let across = a1.cross(a2);
    if across.length() <= 0.1 {
        return None;
    }
    // Closest points of the two axis lines.
    let w0 = first.origin.sub(second.origin);
    let (b, d, e) = (a1.dot(a2), a1.dot(w0), a2.dot(w0));
    let denominator = 1.0 - b * b;
    let s = (b * e - d) / denominator;
    let t = (e - b * d) / denominator;
    let p1 = first.origin.add(a1.scale(s));
    let p2 = second.origin.add(a2.scale(t));
    if p1.sub(p2).length() > tolerance * 4.0 {
        return None;
    }
    let normals = [a1.add(a2).normalized().ok()?, a1.sub(a2).normalized().ok()?];
    Some((p1.add(p2).scale(0.5), normals))
}

/// The section plane normal a cylinder/cylinder edge lies in, when both
/// ends are on both walls and on one plane of [`cylinder_pair_section`].
fn cylinder_pair_section_normal(
    first: CylinderCarrier,
    second: CylinderCarrier,
    points: &[Vec3],
    tolerance: f64,
) -> Option<(Vec3, Vec3)> {
    let (center, normals) = cylinder_pair_section(first, second, tolerance)?;
    let bar = |cylinder: CylinderCarrier| accepted_radial_bar(tolerance * 4.0, cylinder.deviation);
    if points.iter().any(|&point| {
        (radial(first, point).length() - first.radius).abs() > bar(first)
            || (radial(second, point).length() - second.radius).abs() > bar(second)
    }) {
        return None;
    }
    normals
        .into_iter()
        .find(|normal| {
            points
                .iter()
                .all(|point| point.sub(center).dot(*normal).abs() <= tolerance * 4.0)
        })
        .map(|normal| (center, normal))
}

fn cylinder_plane_edge_supported(
    cylinder: CylinderCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    if a.sub(plane.origin).dot(normal).abs() > tolerance
        || b.sub(plane.origin).dot(normal).abs() > tolerance
    {
        return false;
    }
    let ra = radial(cylinder, a);
    let rb = radial(cylinder, b);
    let looks_like_ruling = (station(cylinder, a) - station(cylinder, b)).abs() > tolerance
        && ra
            .normalized()
            .ok()
            .zip(rb.normalized().ok())
            .is_some_and(|(ua, ub)| ua.cross(ub).length() <= 0.05);
    if looks_like_ruling {
        // The exact axial line is contained by the plane only when its
        // direction is tangent to that plane.
        return normal.dot(cylinder.axis).abs() <= 0.01;
    }
    let on_ring = (station(cylinder, a) - station(cylinder, b)).abs() <= tolerance * 4.0;
    // A constant-station circle lies in the neighboring plane only when that
    // plane is perpendicular to the cylinder axis.
    on_ring && normal.cross(cylinder.axis).length() <= 0.01
}

fn cone_station(cone: ConeCarrier, point: Vec3) -> f64 {
    point.sub(cone.apex).dot(cone.axis)
}

fn cone_radial(cone: ConeCarrier, point: Vec3) -> Vec3 {
    let delta = point.sub(cone.apex);
    delta.sub(cone.axis.scale(delta.dot(cone.axis)))
}

fn point_on_cone(cone: ConeCarrier, point: Vec3, tolerance: f64) -> bool {
    let station = cone_station(cone, point);
    station >= -tolerance
        && (cone_radial(cone, point).length() - station * cone.half_angle.tan()).abs() <= tolerance
}

fn edge_is_cone_ruling(cone: ConeCarrier, a: Vec3, b: Vec3, tolerance: f64) -> bool {
    if !point_on_cone(cone, a, tolerance) || !point_on_cone(cone, b, tolerance) {
        return false;
    }
    let ra = cone_radial(cone, a);
    let rb = cone_radial(cone, b);
    let (Ok(ua), Ok(ub), Ok(direction)) = (ra.normalized(), rb.normalized(), b.sub(a).normalized())
    else {
        return false;
    };
    let expected = cone
        .axis
        .add(ua.scale(cone.half_angle.tan()))
        .normalized()
        .unwrap_or(cone.axis);
    ua.cross(ub).length() <= 0.01
        && direction
            .cross(expected)
            .length()
            .min(direction.cross(expected.scale(-1.0)).length())
            <= 0.01
}

fn cone_plane_edge_supported(
    cone: ConeCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let da = a.sub(plane.origin).dot(normal).abs();
    let db = b.sub(plane.origin).dot(normal).abs();
    if da > tolerance || db > tolerance {
        return false;
    }
    let ra = cone_radial(cone, a);
    let rb = cone_radial(cone, b);
    let looks_like_ruling = (cone_station(cone, a) - cone_station(cone, b)).abs() > tolerance
        && ra
            .normalized()
            .ok()
            .zip(rb.normalized().ok())
            .is_some_and(|(ua, ub)| ua.cross(ub).length() <= 0.05);
    if looks_like_ruling {
        let apex_in_plane = cone.apex.sub(plane.origin).dot(normal).abs() <= tolerance * 4.0;
        if apex_in_plane && normal.dot(b.sub(a)).abs() <= tolerance {
            return true;
        }
    }
    let on_ring = (cone_station(cone, a) - cone_station(cone, b)).abs() <= tolerance * 4.0;
    if on_ring && normal.cross(cone.axis).length() <= 0.01 {
        return true;
    }
    // A finite oblique trim is represented later as the exact projective map
    // of only its reference-circle arc. This also covers local hyperbolic and
    // parabolic branches without the mixed weights of a full-circle map.
    true
}

fn cylinder_cone_edge_supported(
    cylinder: CylinderCarrier,
    cone: ConeCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    if cylinder.axis.cross(cone.axis).length() > 0.01 {
        return false;
    }
    let cylinder_station_spread = (station(cylinder, a) - station(cylinder, b)).abs();
    let cone_station_spread = (cone_station(cone, a) - cone_station(cone, b)).abs();
    cylinder_station_spread <= tolerance * 4.0
        && cone_station_spread <= tolerance * 4.0
        && (radial(cylinder, a).length() - cylinder.radius).abs() <= tolerance * 4.0
        && (radial(cylinder, b).length() - cylinder.radius).abs() <= tolerance * 4.0
        && point_on_cone(cone, a, tolerance * 4.0)
        && point_on_cone(cone, b, tolerance * 4.0)
}

/// Diagnostic stage trace, enabled by `BREP_DEBUG_HYBRID`.
fn stage_trace(stage: &str) {
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        eprintln!("[hybrid] stage: {stage}");
    }
}

/// Diagnostic trace of one demotion decision, enabled by `BREP_DEBUG_HYBRID`.
fn demote_trace(candidate: u32, neighbor: u32, why: &str, a: Vec3, b: Vec3) {
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        eprintln!(
            "[hybrid] demote region {candidate} (neighbour {neighbor}): {why}; edge ({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4})",
            a.x, a.y, a.z, b.x, b.y, b.z
        );
    }
}

fn facet_plane(triangle: [usize; 3], vertices: &[Vec3]) -> Result<PlaneInfo, String> {
    let a = vertices[triangle[0]];
    let b = vertices[triangle[1]];
    let c = vertices[triangle[2]];
    let normal = b.sub(a).cross(c.sub(a)).normalized().map_err(|_| {
        "hybrid BREP: cannot create a planar face from a degenerate source triangle".to_owned()
    })?;
    Ok(PlaneInfo { origin: a, normal })
}

fn facets_are_coplanar(
    first: [usize; 3],
    second: [usize; 3],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<bool, String> {
    let first_plane = facet_plane(first, vertices)?;
    let second_plane = facet_plane(second, vertices)?;
    // Do not turn a shallow crease into a plane merely because a distance
    // tolerance happens to cover its short triangles. The distance checks
    // below cover f32 STL coordinate noise; this angular check remains
    // deliberately close to exact coplanarity.
    if first_plane.normal.dot(second_plane.normal) <= 0.0
        || first_plane.normal.cross(second_plane.normal).length() > 1.0e-8
    {
        return Ok(false);
    }
    Ok(first.iter().chain(second.iter()).all(|&vertex| {
        let point = vertices[vertex];
        point.sub(first_plane.origin).dot(first_plane.normal).abs() <= tolerance
            && point
                .sub(second_plane.origin)
                .dot(second_plane.normal)
                .abs()
                <= tolerance
    }))
}

/// Replace unsupported regions with safely merged coplanar patches and
/// recursively demote revolved regions that cannot share exact edges with them.
fn local_regions(
    regions: &[MeshRegion],
    triangle_regions: &[u32],
    triangles: &[[usize; 3]],
    vertices: &[Vec3],
    incidence: &BTreeMap<EdgeKey, Vec<usize>>,
    tolerance: f64,
    forced_demotions: &BTreeSet<u32>,
) -> Result<LocalRegions, String> {
    let mut original_cylinders = HashMap::<u32, CylinderCarrier>::new();
    let mut original_cones = HashMap::<u32, ConeCarrier>::new();
    let mut original_spheres = HashMap::<u32, SphereCarrier>::new();
    let mut original_tori = HashMap::<u32, TorusCarrier>::new();
    let mut original_planes = HashMap::<u32, PlaneInfo>::new();
    let mut retained = BTreeSet::<u32>::new();
    for region in regions {
        if let RegionCarrier::Plane { origin, normal } = region.carrier {
            original_planes.insert(region.id, PlaneInfo { origin, normal });
        }
        if let Some(cylinder) = carrier_cylinder(region)? {
            original_cylinders.insert(region.id, cylinder);
            if !forced_demotions.contains(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(cone) = carrier_cone(region)? {
            original_cones.insert(region.id, cone);
            if !forced_demotions.contains(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(sphere) = carrier_sphere(region)? {
            original_spheres.insert(region.id, sphere);
            if !forced_demotions.contains(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(torus) = carrier_torus(region)? {
            original_tori.insert(region.id, torus);
            if !forced_demotions.contains(&region.id) {
                retained.insert(region.id);
            }
        }
    }
    #[derive(Clone, Copy)]
    enum Revolve {
        Cylinder(CylinderCarrier),
        Cone(ConeCarrier),
        Sphere(SphereCarrier),
        Torus(TorusCarrier),
    }
    let revolve_of = |id: u32| -> Option<Revolve> {
        original_cylinders
            .get(&id)
            .map(|&cylinder| Revolve::Cylinder(cylinder))
            .or_else(|| original_cones.get(&id).map(|&cone| Revolve::Cone(cone)))
            .or_else(|| original_spheres.get(&id).map(|&sphere| Revolve::Sphere(sphere)))
            .or_else(|| original_tori.get(&id).map(|&torus| Revolve::Torus(torus)))
    };
    // A sphere ring stays exact against a coaxial revolve even when that
    // revolve was itself demoted for one of its other trims.
    let sphere_ring_with = |sphere: SphereCarrier, other: Revolve, a: Vec3, b: Vec3| -> bool {
        match other {
            Revolve::Cylinder(cylinder) => {
                sphere_cylinder_edge_supported(sphere, cylinder, a, b, tolerance)
            }
            Revolve::Cone(cone) => sphere_cone_edge_supported(sphere, cone, a, b, tolerance),
            Revolve::Sphere(_) | Revolve::Torus(_) => false,
        }
    };
    // A torus ring stays exact against a coaxial cylinder, cone or torus in
    // the same way.
    let torus_ring_with = |torus: TorusCarrier, other: Revolve, a: Vec3, b: Vec3| -> bool {
        match other {
            Revolve::Cylinder(cylinder) => torus_revolve_edge_supported(
                torus,
                cylinder.origin,
                cylinder.axis,
                |point| {
                    (radial(cylinder, point).length() - cylinder.radius).abs()
                        <= accepted_radial_bar(tolerance * 4.0, cylinder.deviation)
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Cone(cone) => torus_revolve_edge_supported(
                torus,
                cone.apex,
                cone.axis,
                |point| {
                    point_on_cone(cone, point, accepted_radial_bar(tolerance * 4.0, cone.deviation))
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Torus(other) => torus_revolve_edge_supported(
                torus,
                other.center,
                other.axis,
                |point| {
                    torus_radial_error(other, point)
                        <= accepted_radial_bar(tolerance * 4.0, other.deviation)
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Sphere(_) => false,
        }
    };

    loop {
        let mut demote = BTreeSet::new();
        for (&(a, b), incident) in incidence {
            let first = triangle_regions[incident[0]];
            let second = triangle_regions[incident[1]];
            if first == second {
                continue;
            }
            for (candidate, neighbor) in [(first, second), (second, first)] {
                if !retained.contains(&candidate) {
                    continue;
                }
                let Some(own) = revolve_of(candidate) else {
                    continue;
                };
                if retained.contains(&neighbor) {
                    let (supported, why) = match (own, revolve_of(neighbor)) {
                        (Revolve::Cylinder(cylinder), Some(Revolve::Cone(cone)))
                        | (Revolve::Cone(cone), Some(Revolve::Cylinder(cylinder))) => (
                            cylinder_cone_edge_supported(
                                cylinder,
                                cone,
                                vertices[a],
                                vertices[b],
                                tolerance,
                            ),
                            "cylinder/cone edge unsupported",
                        ),
                        // Two cylinders meet exactly along a shared axial
                        // ruling: a tangent chain of arcs on parallel axes
                        // (a rounded outline, a fillet running out into a
                        // boss). Any other cylinder/cylinder curve is not
                        // yet proven exact.
                        // Two cylinders of one radius whose axes cross meet
                        // in planar ellipses (crossing bores).
                        (Revolve::Cylinder(first), Some(Revolve::Cylinder(second))) => (
                            (edge_is_cylinder_ruling(first, vertices[a], vertices[b], tolerance)
                                && edge_is_cylinder_ruling(
                                    second,
                                    vertices[a],
                                    vertices[b],
                                    tolerance,
                                ))
                                || cylinder_pair_section_normal(
                                    first,
                                    second,
                                    &[vertices[a], vertices[b]],
                                    tolerance,
                                )
                                .is_some(),
                            "cylinder/cylinder edge is neither a shared ruling nor an equal-radius crossing",
                        ),
                        (Revolve::Sphere(sphere), Some(other))
                        | (other, Some(Revolve::Sphere(sphere))) => (
                            sphere_ring_with(sphere, other, vertices[a], vertices[b]),
                            "sphere edge is not a coaxial ring",
                        ),
                        (Revolve::Torus(torus), Some(other))
                        | (other, Some(Revolve::Torus(torus))) => (
                            torus_ring_with(torus, other, vertices[a], vertices[b]),
                            "torus edge is not a coaxial ring",
                        ),
                        // Cone/cone intersections are not yet proven exact.
                        _ => (false, "same-kind revolve adjacency"),
                    };
                    if !supported {
                        demote_trace(candidate, neighbor, why, vertices[a], vertices[b]);
                        demote.insert(candidate);
                        demote.insert(neighbor);
                    }
                    continue;
                }
                // A revolve may have been locally demoted because one of its
                // other trims is unsupported. Its shared ring with a coaxial
                // neighbor remains exact, so that neighbor need not be
                // demoted with it.
                match (own, revolve_of(neighbor)) {
                    (Revolve::Cylinder(cylinder), Some(Revolve::Cone(cone))) => {
                        if cylinder_cone_edge_supported(
                            cylinder,
                            cone,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ) {
                            continue;
                        }
                    }
                    (Revolve::Sphere(sphere), Some(other)) => {
                        if sphere_ring_with(sphere, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    (other, Some(Revolve::Sphere(sphere))) => {
                        if sphere_ring_with(sphere, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    (Revolve::Torus(torus), Some(other)) | (other, Some(Revolve::Torus(torus))) => {
                        if torus_ring_with(torus, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    _ => {}
                }
                if let Some(&plane) = original_planes.get(&neighbor) {
                    let supported = match own {
                        Revolve::Cylinder(cylinder) => cylinder_plane_edge_supported(
                            cylinder,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                        Revolve::Cone(cone) => cone_plane_edge_supported(
                            cone,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                        Revolve::Sphere(sphere) => sphere_plane_edge_supported(
                            sphere,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                        Revolve::Torus(torus) => torus_plane_edge_supported(
                            torus,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                    };
                    if !supported {
                        demote_trace(candidate, neighbor, "edge with plane neighbour unsupported", vertices[a], vertices[b]);
                        demote.insert(candidate);
                    }
                    continue;
                }
                // Only a ruling can be shared exactly with a facet; a
                // sphere has none.
                let supported = match own {
                    Revolve::Cylinder(cylinder) => {
                        edge_is_cylinder_ruling(cylinder, vertices[a], vertices[b], tolerance)
                    }
                    Revolve::Cone(cone) => {
                        edge_is_cone_ruling(cone, vertices[a], vertices[b], tolerance)
                    }
                    Revolve::Sphere(_) | Revolve::Torus(_) => false,
                };
                if !supported {
                    demote_trace(candidate, neighbor, "edge with non-plane neighbour is not a ruling", vertices[a], vertices[b]);
                    demote.insert(candidate);
                }
            }
        }
        if demote.is_empty() {
            break;
        }
        for region in demote {
            retained.remove(&region);
        }
    }

    let demoted = original_cylinders
        .keys()
        .chain(original_cones.keys())
        .chain(original_spheres.keys())
        .chain(original_tori.keys())
        .copied()
        .filter(|id| !retained.contains(id))
        .collect::<BTreeSet<_>>();
    let mut planes = HashMap::new();
    let mut cylinders = HashMap::new();
    let mut cones = HashMap::new();
    let mut spheres = HashMap::new();
    let mut tori = HashMap::new();
    let mut original_to_local = HashMap::<u32, u32>::new();
    let mut local_to_original = HashMap::<u32, u32>::new();
    let mut next = 0_u32;
    for region in regions {
        match region.carrier {
            RegionCarrier::Plane { origin, normal } => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                planes.insert(next, PlaneInfo { origin, normal });
                next += 1;
            }
            RegionCarrier::Cylinder { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                cylinders.insert(next, original_cylinders[&region.id]);
                next += 1;
            }
            RegionCarrier::Cone { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                cones.insert(next, original_cones[&region.id]);
                next += 1;
            }
            RegionCarrier::Sphere { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                spheres.insert(next, original_spheres[&region.id]);
                next += 1;
            }
            RegionCarrier::Torus { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                tori.insert(next, original_tori[&region.id]);
                next += 1;
            }
            _ => {}
        }
    }

    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        for (&local, cylinder) in &cylinders {
            eprintln!(
                "[hybrid] region {local} (original {}): cylinder r={:.4} axis=({:+.4},{:+.4},{:+.4}) origin=({:.3},{:.3},{:.3}) sense={}",
                local_to_original[&local],
                cylinder.radius,
                cylinder.axis.x, cylinder.axis.y, cylinder.axis.z,
                cylinder.origin.x, cylinder.origin.y, cylinder.origin.z,
                cylinder.sense
            );
        }
    }
    let fallback = triangle_regions
        .iter()
        .map(|original| !original_to_local.contains_key(original))
        .collect::<Vec<_>>();
    let mut neighbors = vec![Vec::<usize>::new(); triangles.len()];
    for incident in incidence.values() {
        let first = incident[0];
        let second = incident[1];
        if fallback[first] && fallback[second] {
            neighbors[first].push(second);
            neighbors[second].push(first);
        }
    }

    // Check every member against one fixed seed carrier. Pairwise-only
    // transitive merging could otherwise accumulate a long sequence of tiny
    // creases into a patch that is not actually planar.
    let mut fallback_component = vec![None; triangles.len()];
    let mut fallback_components = 0_usize;
    for seed in 0..triangles.len() {
        if !fallback[seed] || fallback_component[seed].is_some() {
            continue;
        }
        let component = fallback_components;
        fallback_components += 1;
        fallback_component[seed] = Some(component);
        let mut queue = VecDeque::from([seed]);
        while let Some(current) = queue.pop_front() {
            for &neighbor in &neighbors[current] {
                if fallback_component[neighbor].is_none()
                    && facets_are_coplanar(
                        triangles[seed],
                        triangles[neighbor],
                        vertices,
                        tolerance,
                    )?
                {
                    fallback_component[neighbor] = Some(component);
                    queue.push_back(neighbor);
                }
            }
        }
    }

    let mut fallback_to_local = HashMap::<usize, u32>::new();
    let mut local_triangle_regions = Vec::with_capacity(triangles.len());
    for (triangle_id, &original) in triangle_regions.iter().enumerate() {
        if let Some(&local) = original_to_local.get(&original) {
            local_triangle_regions.push(local);
        } else {
            let component = fallback_component[triangle_id]
                .ok_or_else(|| "hybrid BREP: fallback component was not assigned".to_owned())?;
            let local = if let Some(&local) = fallback_to_local.get(&component) {
                local
            } else {
                let local = next;
                next += 1;
                planes.insert(local, facet_plane(triangles[triangle_id], vertices)?);
                fallback_to_local.insert(component, local);
                local
            };
            local_triangle_regions.push(local);
        }
    }
    let analytic_plane_triangles = triangle_regions
        .iter()
        .filter(|region| original_planes.contains_key(region))
        .count();
    let analytic_cylinder_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_cylinders.contains_key(region))
        .count();
    let analytic_cone_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_cones.contains_key(region))
        .count();
    let analytic_sphere_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_spheres.contains_key(region))
        .count();
    let analytic_torus_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_tori.contains_key(region))
        .count();
    let demoted_triangles = triangle_regions
        .iter()
        .filter(|region| demoted.contains(region))
        .count();
    let faceted_triangles = triangles.len()
        - analytic_plane_triangles
        - analytic_cylinder_triangles
        - analytic_cone_triangles
        - analytic_sphere_triangles
        - analytic_torus_triangles;
    let stats = HybridBrepStats {
        total_faces: next as usize,
        analytic_plane_faces: original_planes.len(),
        analytic_plane_triangles,
        analytic_cylinder_faces: original_cylinders
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_cylinder_triangles,
        analytic_cone_faces: original_cones
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_cone_triangles,
        analytic_sphere_faces: original_spheres
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_sphere_triangles,
        analytic_torus_faces: original_tori
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_torus_triangles,
        faceted_faces: fallback_to_local.len(),
        faceted_triangles,
        demoted_regions: demoted.len(),
        demoted_triangles,
        segmentation_plane_regions: original_planes.len(),
        segmentation_cylinder_regions: original_cylinders.len(),
        segmentation_cone_regions: original_cones.len(),
        segmentation_sphere_regions: original_spheres.len(),
        segmentation_torus_regions: original_tori.len(),
        segmentation_unsupported_regions: regions.len()
            - original_planes.len()
            - original_cylinders.len()
            - original_cones.len()
            - original_spheres.len()
            - original_tori.len(),
    };
    Ok(LocalRegions {
        triangle_regions: local_triangle_regions,
        planes,
        cylinders,
        cones,
        spheres,
        tori,
        local_to_original,
        count: next as usize,
        stats,
    })
}

fn region_boundaries(
    triangles: &[[usize; 3]],
    regions: &[u32],
    region_count: usize,
    incidence: &BTreeMap<EdgeKey, Vec<usize>>,
) -> Result<(Vec<Chain>, Vec<RegionBoundary>), String> {
    let mut chains = Vec::<Chain>::new();
    let mut chain_by_edges = BTreeMap::<Vec<EdgeKey>, usize>::new();
    let mut boundaries = Vec::with_capacity(region_count);
    for region in 0..region_count as u32 {
        let mut outgoing = BTreeMap::<usize, (usize, u32)>::new();
        for (triangle_id, triangle) in triangles.iter().enumerate() {
            if regions[triangle_id] != region {
                continue;
            }
            for corner in 0..3 {
                let a = triangle[corner];
                let b = triangle[(corner + 1) % 3];
                let incident = &incidence[&edge_key(a, b)];
                let other = incident.iter().copied().find(|&id| id != triangle_id);
                let neighbor = other.map(|id| regions[id]).unwrap_or(u32::MAX);
                if neighbor != region && outgoing.insert(a, (b, neighbor)).is_some() {
                    return Err(format!(
                        "hybrid BREP: region {region} boundary pinches at vertex {a}"
                    ));
                }
            }
        }
        let starts = outgoing.keys().copied().collect::<Vec<_>>();
        let mut visited = BTreeSet::new();
        let mut cycles = Vec::new();
        for start in starts {
            if visited.contains(&start) {
                continue;
            }
            let mut steps = Vec::<(usize, usize, u32)>::new();
            let mut current = start;
            loop {
                if !visited.insert(current) {
                    return Err(format!(
                        "hybrid BREP: region {region} boundary self-crosses"
                    ));
                }
                let &(next, neighbor) = outgoing
                    .get(&current)
                    .ok_or_else(|| format!("hybrid BREP: region {region} boundary walk is open"))?;
                steps.push((current, next, neighbor));
                current = next;
                if current == start {
                    break;
                }
            }
            let all_one_neighbor = steps.iter().all(|step| step.2 == steps[0].2);
            let mut groups = Vec::<Vec<(usize, usize, u32)>>::new();
            if all_one_neighbor {
                groups.push(steps);
            } else {
                let cut = (0..steps.len())
                    .find(|&index| {
                        steps[index].2 != steps[(index + steps.len() - 1) % steps.len()].2
                    })
                    .unwrap();
                steps.rotate_left(cut);
                let mut begin = 0;
                while begin < steps.len() {
                    let neighbor = steps[begin].2;
                    let mut end = begin + 1;
                    while end < steps.len() && steps[end].2 == neighbor {
                        end += 1;
                    }
                    groups.push(steps[begin..end].to_vec());
                    begin = end;
                }
            }
            let mut traversals = Vec::new();
            for group in groups {
                // `steps` is one boundary cycle. It may be only one of several
                // cycles owned by this region, so comparing against the
                // region-wide outgoing-edge count would misclassify a hole ring.
                let closed = all_one_neighbor;
                let mut chain_vertices = vec![group[0].0];
                chain_vertices.extend(group.iter().map(|step| step.1));
                if closed {
                    chain_vertices.pop();
                }
                let mut key = group
                    .iter()
                    .map(|step| edge_key(step.0, step.1))
                    .collect::<Vec<_>>();
                key.sort_unstable();
                let chain_id = if let Some(&existing) = chain_by_edges.get(&key) {
                    existing
                } else {
                    let id = chains.len();
                    let adjacent = (region.min(group[0].2), region.max(group[0].2));
                    chains.push(Chain {
                        vertices: chain_vertices.clone(),
                        closed,
                        adjacent,
                    });
                    chain_by_edges.insert(key, id);
                    id
                };
                let stored = &chains[chain_id];
                let a = chain_vertices[0];
                let b = chain_vertices[1];
                let position = stored
                    .vertices
                    .iter()
                    .position(|&vertex| vertex == a)
                    .ok_or("hybrid BREP: shared chain start is missing")?;
                let forward = if stored.closed {
                    stored.vertices[(position + 1) % stored.vertices.len()] == b
                } else if position + 1 < stored.vertices.len() && stored.vertices[position + 1] == b
                {
                    true
                } else if position > 0 && stored.vertices[position - 1] == b {
                    false
                } else {
                    return Err("hybrid BREP: shared chain directions disagree".into());
                };
                traversals.push(Traversal {
                    chain: chain_id,
                    forward,
                });
            }
            cycles.push(traversals);
        }
        boundaries.push(RegionBoundary { cycles });
    }
    Ok((chains, boundaries))
}

/// Two planar regions meet along a line, so their chain is one straight
/// edge — unless one of them is a leftover facet whose two edges into the
/// plane bend at a shared vertex. Such a chain becomes one straight edge
/// per bend, and every traversal of it in the region cycles is rewritten in
/// order.
fn split_bent_planar_chains(
    chains: Vec<Chain>,
    boundaries: &mut [RegionBoundary],
    vertices: &[Vec3],
    tolerance: f64,
    is_revolve: impl Fn(u32) -> bool,
) -> Vec<Chain> {
    let mut result = Vec::with_capacity(chains.len());
    let mut replacement: Vec<Vec<usize>> = Vec::with_capacity(chains.len());
    for chain in chains {
        let planar = !is_revolve(chain.adjacent.0) && !is_revolve(chain.adjacent.1);
        let points = chain_points(&chain, vertices);
        if !planar || chain.closed || points.len() < 3 || max_line_deviation(&points) <= tolerance {
            replacement.push(vec![result.len()]);
            result.push(chain);
            continue;
        }
        // Greedy runs of collinear mesh edges.
        let mut pieces = Vec::new();
        let mut start = 0;
        for end in 2..chain.vertices.len() {
            if max_line_deviation(&points[start..=end]) > tolerance {
                pieces.push(result.len());
                result.push(Chain {
                    vertices: chain.vertices[start..end].to_vec(),
                    closed: false,
                    adjacent: chain.adjacent,
                });
                start = end - 1;
            }
        }
        pieces.push(result.len());
        result.push(Chain {
            vertices: chain.vertices[start..].to_vec(),
            closed: false,
            adjacent: chain.adjacent,
        });
        replacement.push(pieces);
    }
    for boundary in boundaries.iter_mut() {
        for cycle in &mut boundary.cycles {
            let mut rewritten = Vec::with_capacity(cycle.len());
            for traversal in cycle.iter() {
                let pieces = &replacement[traversal.chain];
                if traversal.forward {
                    rewritten.extend(pieces.iter().map(|&chain| Traversal {
                        chain,
                        forward: true,
                    }));
                } else {
                    rewritten.extend(pieces.iter().rev().map(|&chain| Traversal {
                        chain,
                        forward: false,
                    }));
                }
            }
            *cycle = rewritten;
        }
    }
    result
}

fn chain_points(chain: &Chain, vertices: &[Vec3]) -> Vec<Vec3> {
    chain.vertices.iter().map(|&id| vertices[id]).collect()
}

fn snap_cylinder_axes(
    cylinders: &mut HashMap<u32, CylinderCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<(), (u32, String)> {
    for (&region, cylinder) in cylinders.iter_mut() {
        let mut candidates = Vec::new();
        for chain in chains.iter().filter(|chain| {
            chain.closed && (chain.adjacent.0 == region || chain.adjacent.1 == region)
        }) {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let Some(plane) = planes.get(&neighbor) else {
                continue;
            };
            if plane.normal.cross(cylinder.axis).length() > 0.01 {
                continue;
            }
            let points = chain_points(chain, vertices);
            let spread = points
                .iter()
                .map(|point| point.sub(plane.origin).dot(plane.normal).abs())
                .fold(0.0_f64, f64::max);
            if spread <= tolerance {
                let axis = if plane.normal.dot(cylinder.axis) >= 0.0 {
                    plane.normal
                } else {
                    plane.normal.scale(-1.0)
                };
                candidates.push(axis.normalized().map_err(|error| (region, error))?);
            }
        }
        if let Some(&axis) = candidates.first() {
            // Independent plane fits to rounded STL rings need not have
            // bit-identical normals. Bound their disagreement by the
            // positional error it induces around this cylinder's rim,
            // using the same tolerance as the shared topology checks.
            if candidates
                .iter()
                .any(|other| cylinder.radius * other.cross(axis).length() > tolerance)
            {
                return Err((
                    region,
                    format!("hybrid BREP: cylinder {region} has inconsistent ring normals"),
                ));
            }
            cylinder.axis = axis;
        }
    }
    Ok(())
}

fn snap_cone_carriers(
    cones: &mut HashMap<u32, ConeCarrier>,
    cylinders: &HashMap<u32, CylinderCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<(), (u32, String)> {
    for (&region, cone) in cones.iter_mut() {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut snapped = None::<(Vec3, Vec3)>;

        // A cylinder/cone join is a shared circle. The cylinder fixes the
        // coaxial frame and the cone apex then follows from its opening angle.
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let Some(cylinder) = cylinders.get(&neighbor).copied() else {
                continue;
            };
            let mut axis = cylinder.axis;
            if axis.dot(cone.axis) < 0.0 {
                axis = axis.scale(-1.0);
            }
            let points = chain_points(chain, vertices);
            let center_station = points
                .iter()
                .map(|&point| point.sub(cylinder.origin).dot(axis))
                .sum::<f64>()
                / points.len() as f64;
            let center = cylinder.origin.add(axis.scale(center_station));

            let mut rings = Vec::<(f64, f64)>::new();
            for boundary_chain in &relevant {
                let boundary_points = chain_points(boundary_chain, vertices);
                let stations = boundary_points
                    .iter()
                    .map(|&point| point.sub(cylinder.origin).dot(axis))
                    .collect::<Vec<_>>();
                let low_station = stations.iter().copied().fold(f64::INFINITY, f64::min);
                let high_station = stations.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if high_station - low_station > tolerance * 4.0 {
                    continue;
                }
                let station = stations.iter().sum::<f64>() / stations.len() as f64;
                let neighbor = if boundary_chain.adjacent.0 == region {
                    boundary_chain.adjacent.1
                } else {
                    boundary_chain.adjacent.0
                };
                let radius = if cylinders.contains_key(&neighbor) {
                    cylinder.radius
                } else {
                    boundary_points
                        .iter()
                        .map(|&point| {
                            let delta = point.sub(cylinder.origin);
                            delta.sub(axis.scale(delta.dot(axis))).length()
                        })
                        .sum::<f64>()
                        / boundary_points.len() as f64
                };
                rings.push((station, radius));
            }
            rings.sort_by(|a, b| a.0.total_cmp(&b.0));
            if let (Some(&(s0, r0)), Some(&(s1, r1))) = (rings.first(), rings.last()) {
                let slope = (r1 - r0) / (s1 - s0);
                if slope.is_finite() && slope > 1.0e-6 {
                    let apex_station = s0 - r0 / slope;
                    cone.apex = cylinder.origin.add(axis.scale(apex_station));
                    cone.axis = axis;
                    cone.half_angle = slope.atan();
                    snapped = Some((cone.apex, axis));
                    break;
                }
            }

            // Short cone patches fit their apex poorly from tessellated
            // normals. Their two side planes, however, contain the exact cone
            // apex. Intersect those planes with the coaxial line fixed by the
            // adjacent cylinder, then derive the half-angle from the common
            // cylinder/cone ring.
            let mut apex_stations = Vec::new();
            for side_chain in &relevant {
                let side_neighbor = if side_chain.adjacent.0 == region {
                    side_chain.adjacent.1
                } else {
                    side_chain.adjacent.0
                };
                let Some(side_plane) = planes.get(&side_neighbor) else {
                    continue;
                };
                let side_points = chain_points(side_chain, vertices);
                let side_spread = side_points
                    .iter()
                    .map(|&point| point.sub(cylinder.origin).dot(axis))
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
                        (low.min(value), high.max(value))
                    });
                if side_spread.1 - side_spread.0 <= tolerance * 4.0 {
                    continue;
                }
                let normal = side_plane
                    .normal
                    .normalized()
                    .map_err(|error| (region, error))?;
                let denominator = normal.dot(axis);
                if denominator.abs() <= 1.0e-8 {
                    continue;
                }
                apex_stations
                    .push(normal.dot(side_plane.origin.sub(cylinder.origin)) / denominator);
            }
            let side_solution = if !apex_stations.is_empty() {
                let apex_station = apex_stations.iter().sum::<f64>() / apex_stations.len() as f64;
                let axial_radius = center_station - apex_station;
                if axial_radius <= tolerance {
                    None
                } else {
                    Some((
                        cylinder.origin.add(axis.scale(apex_station)),
                        (cylinder.radius / axial_radius).atan(),
                    ))
                }
            } else {
                None
            };
            // Accept a side-plane apex only when it corroborates the fitted
            // opening angle. Otherwise those planes cut the cone in genuine
            // oblique conics and do not contain its apex.
            let (apex, half_angle) = match side_solution {
                Some((apex, angle)) if (angle - cone.half_angle).abs() <= 0.01 => (apex, angle),
                _ => (
                    center.sub(axis.scale(cylinder.radius / cone.half_angle.tan())),
                    cone.half_angle,
                ),
            };
            cone.half_angle = half_angle;
            snapped = Some((apex, axis));
            break;
        }

        // Otherwise a ring in an axis-normal plane supplies an equally exact
        // frame. This covers truncated cones without an adjacent cylinder.
        if snapped.is_none() {
            for chain in relevant.iter().filter(|chain| chain.closed) {
                let neighbor = if chain.adjacent.0 == region {
                    chain.adjacent.1
                } else {
                    chain.adjacent.0
                };
                let Some(plane) = planes.get(&neighbor).copied() else {
                    continue;
                };
                let mut axis = plane.normal.normalized().map_err(|error| (region, error))?;
                if axis.dot(cone.axis) < 0.0 {
                    axis = axis.scale(-1.0);
                }
                if axis.cross(cone.axis).length() > 0.01 {
                    continue;
                }
                let points = chain_points(chain, vertices);
                let mean = points
                    .iter()
                    .copied()
                    .fold(Vec3::new(0.0, 0.0, 0.0), Vec3::add)
                    .scale(1.0 / points.len() as f64);
                let center = mean.sub(axis.scale(mean.sub(plane.origin).dot(axis)));
                let radius = points
                    .iter()
                    .map(|&point| {
                        let delta = point.sub(center);
                        delta.sub(axis.scale(delta.dot(axis))).length()
                    })
                    .sum::<f64>()
                    / points.len() as f64;
                if radius > tolerance {
                    let apex = center.sub(axis.scale(radius / cone.half_angle.tan()));
                    snapped = Some((apex, axis));
                    break;
                }
            }
        }

        if let Some((apex, axis)) = snapped {
            cone.apex = apex;
            cone.axis = axis;
        }
    }
    Ok(())
}

fn station(carrier: CylinderCarrier, point: Vec3) -> f64 {
    point.sub(carrier.origin).dot(carrier.axis)
}

fn radial(carrier: CylinderCarrier, point: Vec3) -> Vec3 {
    let delta = point.sub(carrier.origin);
    delta.sub(carrier.axis.scale(delta.dot(carrier.axis)))
}

fn build_cylinder_surfaces(
    carriers: &HashMap<u32, CylinderCarrier>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<HashMap<u32, CylinderInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in &relevant {
            for point in chain_points(chain, vertices) {
                let value = station(carrier, point);
                low = low.min(value);
                high = high.max(value);
            }
        }
        if !low.is_finite() || high - low <= tolerance {
            return Err((
                region,
                format!("hybrid BREP: cylinder {region} has no finite axial span"),
            ));
        }
        let mut seam = None;
        for chain in &relevant {
            if chain.closed {
                continue;
            }
            let points = chain_points(chain, vertices);
            let s0 = station(carrier, points[0]);
            let s1 = station(carrier, *points.last().unwrap());
            let r0 = radial(carrier, points[0]);
            let r1 = radial(carrier, *points.last().unwrap());
            if (s1 - s0).abs() > tolerance && r0.cross(r1).length() <= tolerance * carrier.radius {
                seam = Some(r0.normalized().map_err(|error| (region, error))?);
                break;
            }
        }
        // Without a ruling, a crossing-bore ellipse arc puts the seam at its
        // end, so the arcs that chain around the wall meet it only at vertices.
        if seam.is_none() {
            for chain in &relevant {
                let neighbor = if chain.adjacent.0 == region {
                    chain.adjacent.1
                } else {
                    chain.adjacent.0
                };
                let Some(&other) = carriers.get(&neighbor) else {
                    continue;
                };
                let points = chain_points(chain, vertices);
                if !chain.closed
                    && cylinder_pair_section_normal(carrier, other, &points, tolerance).is_some()
                {
                    seam = Some(
                        radial(carrier, points[0])
                            .normalized()
                            .map_err(|error| (region, error))?,
                    );
                    break;
                }
            }
        }
        let x_axis = match seam {
            Some(axis) => axis,
            None => carrier
                .axis
                .perpendicular()
                .map_err(|error| (region, error))?,
        };
        let y_axis = carrier
            .axis
            .cross(x_axis)
            .normalized()
            .map_err(|error| (region, error))?;
        let base = carrier.origin.add(carrier.axis.scale(low));
        let profile = make_line(
            base.add(x_axis.scale(carrier.radius)),
            base.add(carrier.axis.scale(high - low))
                .add(x_axis.scale(carrier.radius)),
        )
        .map_err(|error| (region, error))?;
        let surface =
            make_revolution(base, carrier.axis, &profile, TAU).map_err(|error| (region, error))?;
        result.insert(
            region,
            CylinderInfo {
                carrier,
                base_station: low,
                height: high - low,
                x_axis,
                y_axis,
                surface,
            },
        );
    }
    Ok(result)
}

/// Cone regions that reach their apex: the region owns a mesh vertex at the
/// carrier's apex and is bounded by exactly one closed ring away from it.
fn pointed_cone_regions(
    carriers: &HashMap<u32, ConeCarrier>,
    triangle_regions: &[u32],
    triangles: &[[usize; 3]],
    vertices: &[Vec3],
    chains: &[Chain],
    tolerance: f64,
) -> BTreeSet<u32> {
    let mut pointed = BTreeSet::new();
    for (&region, &cone) in carriers {
        let bar = accepted_radial_bar(tolerance * 4.0, cone.deviation);
        let owns_apex = triangle_regions
            .iter()
            .zip(triangles)
            .filter(|(&owner, _)| owner == region)
            .flat_map(|(_, triangle)| triangle.iter())
            .any(|&vertex| vertices[vertex].sub(cone.apex).length() <= bar);
        if !owns_apex {
            continue;
        }
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let [ring] = relevant.as_slice() else {
            continue;
        };
        if !ring.closed {
            continue;
        }
        let clear_of_apex = chain_points(ring, vertices)
            .iter()
            .all(|&point| cone_station(cone, point) * cone.half_angle.tan() > bar);
        if clear_of_apex {
            pointed.insert(region);
        }
    }
    pointed
}

fn build_cone_surfaces(
    carriers: &HashMap<u32, ConeCarrier>,
    pointed_cones: &BTreeSet<u32>,
    cylinders: &HashMap<u32, CylinderInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<HashMap<u32, ConeInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in &relevant {
            for point in chain_points(chain, vertices) {
                let value = cone_station(carrier, point);
                low = low.min(value);
                high = high.max(value);
            }
        }
        let pointed = pointed_cones.contains(&region);
        if pointed {
            // The surface starts at the apex; the ring is its only trim.
            low = 0.0;
        }
        // Any other cone reaching its apex would need a pole this builder
        // only synthesizes for a region closed by one ring.
        if !low.is_finite()
            || (!pointed && low * carrier.half_angle.tan() <= tolerance)
            || high - low <= tolerance
        {
            return Err((
                region,
                format!("hybrid BREP: cone {region} is pointed or has no finite axial span"),
            ));
        }
        // Keep fitted STL boundary samples away from the NURBS domain ends.
        // The trim loops still define the face; this small carrier extension
        // merely permits exact plane/cone intersections through quantized end
        // vertices without evaluating outside the surface domain.
        let margin = ((high - low) * 1.0e-4).max(tolerance * 2.0);
        if !pointed {
            low = (low - margin).max(tolerance / carrier.half_angle.tan());
        }
        high += margin;
        let mut seam = relevant.iter().find_map(|chain| {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            cylinders.get(&neighbor).map(|cylinder| cylinder.x_axis)
        });
        for chain in &relevant {
            if chain.closed {
                continue;
            }
            let points = chain_points(chain, vertices);
            let s0 = cone_station(carrier, points[0]);
            let s1 = cone_station(carrier, *points.last().unwrap());
            let r0 = cone_radial(carrier, points[0]);
            let r1 = cone_radial(carrier, *points.last().unwrap());
            if (s1 - s0).abs() > tolerance
                && r0.cross(r1).length() <= tolerance * r0.length().max(r1.length())
            {
                seam = Some(r0.normalized().map_err(|error| (region, error))?);
                break;
            }
        }
        // A pointed cone's seam runs from its ring's first vertex to the
        // apex, so the ring edge starts on it.
        if seam.is_none() && pointed {
            if let Some(ring) = relevant.first() {
                seam = cone_radial(carrier, vertices[ring.vertices[0]]).normalized().ok();
            }
        }
        let x_axis = match seam {
            Some(axis) => axis,
            None => carrier
                .axis
                .perpendicular()
                .map_err(|error| (region, error))?,
        };
        let y_axis = carrier
            .axis
            .cross(x_axis)
            .normalized()
            .map_err(|error| (region, error))?;
        let radius_low = low * carrier.half_angle.tan();
        let radius_high = high * carrier.half_angle.tan();
        let base = carrier.apex.add(carrier.axis.scale(low));
        let profile = make_line(
            base.add(x_axis.scale(radius_low)),
            base.add(carrier.axis.scale(high - low))
                .add(x_axis.scale(radius_high)),
        )
        .map_err(|error| (region, error))?;
        let surface =
            make_revolution(base, carrier.axis, &profile, TAU).map_err(|error| (region, error))?;
        result.insert(
            region,
            ConeInfo {
                carrier,
                pointed,
                base_station: low,
                height: high - low,
                x_axis,
                y_axis,
                surface,
            },
        );
    }
    Ok(result)
}

/// Frame each sphere as a revolution about the common normal of the ring
/// planes that bound it, pointing into the region so a single-ring cap
/// holds the north pole; the seam passes through the first ring's first
/// source vertex.
///
/// A sphere whose rings are NOT all coaxial (a sphere clipped by a box: six
/// flats on three axes) is framed about one ring axis whose two poles both
/// fall in removed caps of coaxial rings, with its seam meridian turned clear
/// of every other ring. Those tilted rings are exact plane/sphere circles
/// whose sphere pcurves are fitted, not isoparametric; see
/// [`tilted_sphere_ring_edge`].
#[allow(clippy::too_many_arguments)]
fn build_sphere_surfaces(
    carriers: &HashMap<u32, SphereCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    region_centroids: &HashMap<u32, Vec3>,
    region_samples: &HashMap<u32, Vec<Vec3>>,
    tolerance: f64,
) -> Result<HashMap<u32, SphereInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        if relevant.is_empty() {
            return Err((
                region,
                format!("hybrid BREP: sphere {region} is not bounded by closed rings"),
            ));
        }
        if relevant.iter().any(|chain| !chain.closed) {
            let samples = region_samples.get(&region).map(Vec::as_slice).unwrap_or(&[]);
            let info = sphere_patch_info(
                region, carrier, &relevant, cylinders, cones, vertices, samples, tolerance,
            )
            .map_err(|message| (region, message))?;
            result.insert(region, info);
            continue;
        }
        let mut axis: Option<Vec3> = None;
        // A ring shared with a coaxial revolve is one edge whose pcurve on
        // each face starts at that face's seam, so the seams must coincide.
        let mut seam: Option<Vec3> = None;
        let angular = tolerance / carrier.radius;
        let mut rings = Vec::with_capacity(relevant.len());
        let mut coaxial = true;
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let (direction, seam_hint) = if let Some(plane) = planes.get(&neighbor) {
                (plane.normal.normalized().map_err(|error| (region, error))?, None)
            } else if let Some(cylinder) = cylinders.get(&neighbor) {
                (cylinder.carrier.axis, Some(cylinder.x_axis))
            } else if let Some(cone) = cones.get(&neighbor) {
                (cone.carrier.axis, Some(cone.x_axis))
            } else {
                return Err((
                    region,
                    format!("hybrid BREP: sphere {region} ring neighbor is not a plane or coaxial revolve"),
                ));
            };
            let points = chain_points(chain, vertices);
            let station = points
                .iter()
                .map(|point| point.sub(carrier.center).dot(direction))
                .sum::<f64>()
                / points.len() as f64;
            rings.push(SphereRing {
                direction,
                station,
                plane: planes.contains_key(&neighbor),
                seam_hint,
                first_point: points[0],
            });
            match axis {
                None => axis = Some(direction),
                Some(current) if current.cross(direction).length() <= angular => {}
                Some(_) => coaxial = false,
            }
        }
        let (mut axis, seam, north_pole_inside) = if coaxial {
            for ring in &rings {
                if let Some(hint) = ring.seam_hint {
                    seam.get_or_insert(hint);
                }
            }
            (axis.unwrap(), seam, relevant.len() == 1)
        } else {
            let samples = region_samples.get(&region).map(Vec::as_slice).unwrap_or(&[]);
            let (axis, seam) = tilted_sphere_frame(carrier, &mut rings, samples, tolerance)
                .map_err(|message| {
                    (
                        region,
                        format!("hybrid BREP: sphere {region} rings are not coaxial and {message}"),
                    )
                })?;
            (axis, Some(seam), false)
        };
        // Point the axis into the region: a cap then surrounds the north
        // pole. A tilted frame has both poles in removed caps, so either
        // sense serves and the one chosen stays.
        if coaxial {
            if let Some(&centroid) = region_centroids.get(&region) {
                if centroid.sub(carrier.center).dot(axis) < 0.0 {
                    axis = axis.scale(-1.0);
                }
            }
        }
        let seam = seam.unwrap_or_else(|| rings[0].first_point.sub(carrier.center));
        let surface = make_sphere_surface_framed(carrier.center, carrier.radius, axis, Some(seam))
            .map_err(|error| (region, error))?;
        let equator = surface.evaluate(0.0, 0.5).map_err(|error| (region, error))?;
        let x_axis = equator
            .sub(carrier.center)
            .normalized()
            .map_err(|error| (region, error))?;
        let y_axis = axis.cross(x_axis).normalized().map_err(|error| (region, error))?;
        result.insert(
            region,
            SphereInfo {
                carrier,
                axis,
                x_axis,
                y_axis,
                surface,
                north_pole_inside,
            },
        );
    }
    Ok(result)
}

/// A sphere patch bounded by open arcs, such as the corner octant of a box
/// rounded on every edge: one loop of three quarter-arcs, each shared with an
/// edge blend on a different axis, so no polar axis makes them latitudes.
///
/// Every arc must be shared with a coaxial cylinder or cone, whose arc is
/// the edge's curve; the sphere side is a fitted pcurve (see
/// [`sphere_patch_pcurve`]). The frame keeps the whole patch in the open
/// hemisphere around its mean direction `m`: the polar axis is square to `m`
/// and the seam meridian passes through `-m`, so both poles and the seam are
/// at least `90° - (widest angle from m)` away from every point of the patch,
/// and that clearance must exceed the tilted lane's bar.
#[allow(clippy::too_many_arguments)]
fn sphere_patch_info(
    region: u32,
    carrier: SphereCarrier,
    relevant: &[&Chain],
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    vertices: &[Vec3],
    samples: &[Vec3],
    tolerance: f64,
) -> Result<SphereInfo, String> {
    for chain in relevant {
        let neighbor = if chain.adjacent.0 == region {
            chain.adjacent.1
        } else {
            chain.adjacent.0
        };
        if !cylinders.contains_key(&neighbor) && !cones.contains_key(&neighbor) {
            return Err(format!(
                "hybrid BREP: sphere {region} is bounded by open arcs and one is not shared with a cylinder or cone"
            ));
        }
    }
    let directions = relevant
        .iter()
        .flat_map(|chain| chain_points(chain, vertices))
        .chain(samples.iter().copied())
        .map(|point| point.sub(carrier.center).normalized())
        .collect::<Result<Vec<_>, String>>()?;
    let mean = directions
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, direction| sum.add(*direction))
        .normalized()
        .map_err(|_| format!("hybrid BREP: sphere {region} patch has no mean direction"))?;
    let widest = directions
        .iter()
        .map(|direction| direction.dot(mean).clamp(-1.0, 1.0).acos())
        .fold(0.0, f64::max);
    let clearance = std::f64::consts::FRAC_PI_2 - widest;
    if clearance * carrier.radius <= 16.0 * tolerance {
        return Err(format!(
            "hybrid BREP: sphere {region} patch does not fit in a hemisphere clear of a pole and seam (widest {widest:.4} rad)"
        ));
    }
    let axis = mean.perpendicular()?;
    let seam = mean.scale(-1.0);
    let surface = make_sphere_surface_framed(carrier.center, carrier.radius, axis, Some(seam))?;
    let equator = surface.evaluate(0.0, 0.5)?;
    let x_axis = equator.sub(carrier.center).normalized()?;
    let y_axis = axis.cross(x_axis).normalized()?;
    Ok(SphereInfo {
        carrier,
        axis,
        x_axis,
        y_axis,
        surface,
        north_pole_inside: false,
    })
}

/// The pcurve on a patch sphere (see [`sphere_patch_info`]) of an open arc
/// whose curve the neighbouring revolve built, oriented along the curve and
/// checked against it at the exact-topology bar. The patch frame keeps the
/// seam clear of the arc, so the pcurve never wraps.
fn sphere_patch_pcurve(
    chain_index: usize,
    sphere: &SphereInfo,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let pcurve = build_pcurve_on_surface_range(
        &sphere.surface,
        curve,
        t0,
        t1,
        true,
        (tolerance * 0.05).max(1.0e-9),
    )?;
    let domain = pcurve.domain()?;
    let mut previous_u: Option<f64> = None;
    for sample in 0..=64 {
        let fraction = sample as f64 / 64.0;
        let t = t0 + (t1 - t0) * fraction;
        let q = domain[0] + (domain[1] - domain[0]) * fraction;
        let uv = pcurve.evaluate(q)?;
        let surface_point = sphere.surface.evaluate(uv.x, uv.y)?;
        if curve.evaluate(t)?.sub(surface_point).length() > tolerance * 0.1
            || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25)
        {
            return Err(format!(
                "hybrid BREP: sphere patch arc {chain_index} pcurve failed agreement"
            ));
        }
        previous_u = Some(uv.x);
    }
    Ok(pcurve)
}

/// One closed ring bounding a sphere region: the plane `(p - center) ·
/// direction = station` it lies in, and what the neighbor across it demands.
struct SphereRing {
    direction: Vec3,
    station: f64,
    plane: bool,
    seam_hint: Option<Vec3>,
    first_point: Vec3,
}

/// Polar axis and seam direction for a sphere region bounded by rings on
/// several axes, or why there is none.
///
/// The region must be the sphere less DISJOINT caps, one per ring: every
/// sample of the region lies on the kept side of every ring. The polar axis
/// is a ring axis whose two poles both lie strictly inside removed caps of
/// rings coaxial with it, so those rings stay latitudes and no pole touches
/// the face. Every other ring must be a plane circle, and the seam meridian
/// is turned to clear each such ring's removed cap by a margin, so its
/// fitted pcurve is a closed loop inside the parameter square.
fn tilted_sphere_frame(
    carrier: SphereCarrier,
    rings: &mut [SphereRing],
    samples: &[Vec3],
    tolerance: f64,
) -> Result<(Vec3, Vec3), String> {
    let radius = carrier.radius;
    let angular = tolerance / radius;
    // Orient each ring so `(p - center) · direction >= station` is the KEPT
    // side: the removed cap is then centred on `-direction`.
    for ring in rings.iter_mut() {
        let side = samples
            .iter()
            .map(|sample| sample.sub(carrier.center).dot(ring.direction) - ring.station)
            .max_by(|a, b| a.abs().total_cmp(&b.abs()))
            .ok_or_else(|| "the region has no samples to side its rings".to_owned())?;
        if side < 0.0 {
            ring.direction = ring.direction.scale(-1.0);
            ring.station = -ring.station;
        }
        if ring.station.abs() >= radius - tolerance {
            return Err("a ring is degenerate".to_owned());
        }
    }
    let slack = tolerance * 4.0;
    if samples.iter().any(|sample| {
        rings
            .iter()
            .any(|ring| sample.sub(carrier.center).dot(ring.direction) < ring.station - slack)
    }) {
        return Err("the region is not the sphere less one cap per ring".to_owned());
    }
    // Removed caps must be disjoint: the angle between two cap centres must
    // exceed the sum of their angular radii. A cap centred on `-direction`
    // holds the points with `(p - center) · -direction > -station`.
    let cap_radius = |ring: &SphereRing| (-ring.station / radius).clamp(-1.0, 1.0).acos();
    for (index, first) in rings.iter().enumerate() {
        for second in &rings[index + 1..] {
            let between = first.direction.dot(second.direction).clamp(-1.0, 1.0).acos();
            if between <= cap_radius(first) + cap_radius(second) + angular {
                return Err("two removed caps touch".to_owned());
            }
        }
    }
    // Strictly inside a removed cap, clear of its rim.
    let in_removed_cap = |ring: &SphereRing, pole: Vec3| {
        pole.dot(ring.direction) * radius < ring.station - slack
    };
    for candidate in rings.iter() {
        let axis = candidate.direction;
        let parallel = |ring: &SphereRing| axis.cross(ring.direction).length() <= angular;
        let north_capped = rings
            .iter()
            .any(|ring| parallel(ring) && in_removed_cap(ring, axis));
        let south_capped = rings
            .iter()
            .any(|ring| parallel(ring) && in_removed_cap(ring, axis.scale(-1.0)));
        if !north_capped || !south_capped {
            continue;
        }
        let tilted = rings.iter().filter(|ring| !parallel(ring)).collect::<Vec<_>>();
        if tilted.iter().any(|ring| !ring.plane) {
            continue;
        }
        let Ok(x0) = axis.perpendicular() else {
            continue;
        };
        let Ok(y0) = axis.cross(x0).normalized() else {
            continue;
        };
        // How far the meridian at azimuth `phi` stays outside every tilted
        // cap, as an angle. The meridian's closest approach to a direction
        // `w` has cosine `max(hypot(a, b), |b|)` for `a` the component of
        // `w` along the meridian's equator direction and `b` along the axis.
        let clearance = |phi: f64| {
            let toward = x0.scale(phi.cos()).add(y0.scale(phi.sin()));
            tilted
                .iter()
                .map(|ring| {
                    let cap = ring.direction.scale(-1.0);
                    let a = cap.dot(toward);
                    let b = cap.dot(axis);
                    let closest = if a >= 0.0 { a.hypot(b) } else { b.abs() };
                    closest.clamp(-1.0, 1.0).acos() - cap_radius(ring)
                })
                .fold(f64::INFINITY, f64::min)
        };
        let forced = rings
            .iter()
            .filter(|ring| parallel(ring))
            .find_map(|ring| ring.seam_hint);
        let phi = match forced {
            Some(hint) => hint.dot(y0).atan2(hint.dot(x0)),
            None => (0..720)
                .map(|step| step as f64 * TAU / 720.0)
                .max_by(|a, b| clearance(*a).total_cmp(&clearance(*b)))
                .unwrap(),
        };
        if clearance(phi) * radius <= 16.0 * tolerance {
            continue;
        }
        return Ok((axis, x0.scale(phi.cos()).add(y0.scale(phi.sin()))));
    }
    Err("no ring axis has both poles in removed coaxial caps with a seam clear of the other rings".to_owned())
}

fn sphere_station(info: &SphereInfo, point: Vec3) -> f64 {
    point.sub(info.carrier.center).dot(info.axis)
}

fn sphere_coordinates(info: &SphereInfo, point: Vec3) -> [f64; 2] {
    let d = point.sub(info.carrier.center);
    let angle = d.dot(info.y_axis).atan2(d.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    // The meridian is a half circle from the south pole: two rational
    // quadratic spans over the polar angle.
    let polar = (sphere_station(info, point) / info.carrier.radius)
        .clamp(-1.0, 1.0)
        .asin();
    let v = circle_angle_to_parameter(2, std::f64::consts::PI, polar + std::f64::consts::FRAC_PI_2);
    [u, v]
}

fn max_line_deviation(points: &[Vec3]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let start = points[0];
    let chord = points[points.len() - 1].sub(start);
    let denominator = chord.dot(chord);
    points[1..points.len() - 1]
        .iter()
        .map(|point| {
            let delta = point.sub(start);
            let t = if denominator > 0.0 {
                (delta.dot(chord) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            delta.sub(chord.scale(t)).length()
        })
        .fold(0.0, f64::max)
}

fn trim_curve(curve: &NurbsCurve, start: f64, end: f64) -> Result<NurbsCurve, String> {
    let domain = curve.domain()?;
    // `split` refuses a parameter within the kernel's knot identity
    // tolerance of either end, so a trim that close to an end is the whole
    // curve: a station a fraction of a nanometre above a region's own
    // minimum (the fitted axis is not exactly a coordinate direction, so
    // two vertices on one rim differ in the last bits) is that region's
    // base, not a cut.
    let mut trimmed = if end < domain[1] - KNOT_IDENTITY_TOL {
        curve.split(end)?.0
    } else {
        curve.clone()
    };
    if start > domain[0] + KNOT_IDENTITY_TOL {
        trimmed = trimmed.split(start)?.1;
    }
    Ok(trimmed)
}

fn cylinder_coordinates(info: &CylinderInfo, point: Vec3) -> [f64; 2] {
    let r = radial(info.carrier, point);
    let angle = r.dot(info.y_axis).atan2(r.dot(info.x_axis)).rem_euclid(TAU);
    // A rational quadratic circle is not angle-linear inside a knot span.
    // Use the public exact angle-to-NURBS mapping used by the kernel carrier.
    let u = circle_angle_to_parameter(4, TAU, angle);
    let v = (station(info.carrier, point) - info.base_station) / info.height;
    [u, v]
}

fn cone_coordinates(info: &ConeInfo, point: Vec3) -> [f64; 2] {
    let r = cone_radial(info.carrier, point);
    let angle = r.dot(info.y_axis).atan2(r.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    let v = (cone_station(info.carrier, point) - info.base_station) / info.height;
    [u, v]
}

/// The exact circle where a plane cuts a sphere, as the edge of a closed
/// chain that is NOT a latitude of the sphere's frame, with its fitted pcurve
/// on the sphere (oriented along the curve).
///
/// The circle starts at the source ring's first vertex, so the edge's one
/// vertex is where the chain begins. The pcurve is checked against the
/// circle at the exact-topology bar; the frame's seam was turned clear of
/// this ring, so the pcurve is a closed loop that never wraps.
fn tilted_sphere_ring_edge(
    chain_index: usize,
    sphere: &SphereInfo,
    plane: PlaneInfo,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(NurbsCurve, f64, f64, bool, NurbsCurve), String> {
    let normal = plane.normal.normalized()?;
    let center = sphere.carrier.center;
    let offset = plane.origin.sub(center).dot(normal);
    let radius_sq = sphere.carrier.radius * sphere.carrier.radius - offset * offset;
    if radius_sq <= tolerance * tolerance {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} misses its sphere"
        ));
    }
    let ring_radius = radius_sq.sqrt();
    let ring_center = center.add(normal.scale(offset));
    let radial_bar = accepted_radial_bar(tolerance, sphere.carrier.deviation);
    for &point in points {
        let delta = point.sub(ring_center);
        let height = delta.dot(normal);
        let in_plane = delta.sub(normal.scale(height)).length();
        if height.abs() > tolerance || (in_plane - ring_radius).abs() > radial_bar {
            return Err(format!(
                "hybrid BREP: tilted sphere ring {chain_index} source leaves its circle (height {height:.3e}, radial {:.3e}, bar {radial_bar:.3e})",
                in_plane - ring_radius
            ));
        }
    }
    let first = points[0].sub(ring_center);
    let x = first.sub(normal.scale(first.dot(normal))).normalized()?;
    let y = normal.cross(x).normalized()?;
    // The chain winds one way about the normal; the circle runs the same way
    // when its second point turns positively.
    let angles = points
        .iter()
        .chain(std::iter::once(&points[0]))
        .map(|point| {
            let delta = point.sub(ring_center);
            delta.dot(y).atan2(delta.dot(x)) / TAU
        })
        .collect::<Vec<_>>();
    let mut unwrapped = angles.clone();
    unwrap(&mut unwrapped);
    let sweep = unwrapped.last().unwrap() - unwrapped[0];
    if (sweep.abs() - 1.0).abs() > 0.02 || !monotone_parameter_walk(&unwrapped, 1.0e-5) {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} does not wind once (sweep {sweep:.6})"
        ));
    }
    let curve = make_arc(ring_center, x, y, ring_radius, 0.0, TAU)?;
    let domain = curve.domain()?;
    let pcurve = build_pcurve_on_surface_range(
        &sphere.surface,
        &curve,
        domain[0],
        domain[1],
        true,
        (tolerance * 0.05).max(1.0e-9),
    )?;
    let pcurve_domain = pcurve.domain()?;
    let mut previous_u: Option<f64> = None;
    for sample in 0..=64 {
        let fraction = sample as f64 / 64.0;
        let t = domain[0] + (domain[1] - domain[0]) * fraction;
        let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
        let uv = pcurve.evaluate(q)?;
        let surface_point = sphere.surface.evaluate(uv.x, uv.y)?;
        if curve.evaluate(t)?.sub(surface_point).length() > tolerance * 0.1
            || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25)
        {
            return Err(format!(
                "hybrid BREP: tilted sphere ring {chain_index} pcurve failed agreement"
            ));
        }
        previous_u = Some(uv.x);
    }
    let start = pcurve.evaluate(pcurve_domain[0])?;
    let end = pcurve.evaluate(pcurve_domain[1])?;
    if (start.x - end.x).abs() > 1.0e-9 || (start.y - end.y).abs() > 1.0e-9 {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} pcurve does not close"
        ));
    }
    Ok((curve, domain[0], domain[1], sweep > 0.0, pcurve))
}

/// The exact ellipse arc two crossing cylinders of one radius share, with
/// its fitted pcurve on each (oriented along the curve).
///
/// In its section plane through the axes' crossing point the ellipse has
/// the common radius as its semi-minor axis, along the common normal of the
/// axes, and `r / sqrt(1 - (e · axis)^2)` along the in-plane direction `e`
/// normal to that, the same for both walls. Both pcurves are checked against
/// the arc, and each must stay inside its wall's parameter square without
/// wrapping: the wall's seam was put at the end of one of these arcs.
#[allow(clippy::type_complexity)]
fn cylinder_pair_section_edge(
    chain_index: usize,
    walls: [(u32, &CylinderInfo); 2],
    points: &[Vec3],
    tolerance: f64,
) -> Result<(NurbsCurve, f64, f64, bool, Vec<(u32, NurbsCurve)>), String> {
    let [(_, first), (_, second)] = walls;
    let (center, normal) =
        cylinder_pair_section_normal(first.carrier, second.carrier, points, tolerance)
            .ok_or_else(|| {
                format!(
                    "hybrid BREP: cylinder/cylinder chain {chain_index} is not an equal-radius crossing section"
                )
            })?;
    let radius = 0.5 * (first.carrier.radius + second.carrier.radius);
    let minor = first.carrier.axis.cross(second.carrier.axis).normalized()?;
    let major = normal.cross(minor).normalized()?;
    let along = major.dot(first.carrier.axis.normalized()?);
    let major_radius = radius / (1.0 - along * along).sqrt();
    let mut angles = points
        .iter()
        .map(|point| {
            let delta = point.sub(center);
            (delta.dot(minor) / radius).atan2(delta.dot(major) / major_radius)
        })
        .collect::<Vec<_>>();
    for index in 1..angles.len() {
        while angles[index] - angles[index - 1] > std::f64::consts::PI {
            angles[index] -= TAU;
        }
        while angles[index] - angles[index - 1] < -std::f64::consts::PI {
            angles[index] += TAU;
        }
    }
    if !monotone_parameter_walk(&angles, 1.0e-5) {
        return Err(format!(
            "hybrid BREP: cylinder/cylinder chain {chain_index} backtracks along its ellipse"
        ));
    }
    let (a0, a1) = (angles[0], *angles.last().unwrap());
    let (low, high) = (a0.min(a1), a0.max(a1));
    if high - low >= TAU - 1.0e-6 {
        return Err(format!(
            "hybrid BREP: cylinder/cylinder chain {chain_index} spans its whole ellipse"
        ));
    }
    // A unit arc in the xy plane, mapped affinely onto the ellipse: exact.
    let unit = make_arc(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.0,
        low,
        high,
    )?;
    let controls = unit
        .control_points
        .iter()
        .map(|control| {
            let point = center
                .add(major.scale(major_radius * control.x / control.w))
                .add(minor.scale(radius * control.y / control.w));
            Vec4::from_point(point, control.w)
        })
        .collect::<Vec<_>>();
    let curve = NurbsCurve::new(unit.degree, unit.knots.clone(), controls)?;
    let domain = curve.domain()?;
    let bar = accepted_radial_bar(
        tolerance,
        first.carrier.deviation.max(second.carrier.deviation),
    );
    for &point in points {
        let distance = project_point_to_curve(&curve, point)?.distance;
        if distance > bar {
            return Err(format!(
                "hybrid BREP: cylinder/cylinder chain {chain_index} source leaves its ellipse by {distance:.3e} (bar {bar:.3e})"
            ));
        }
    }
    let mut pcurves = Vec::with_capacity(2);
    for (owner, wall) in walls {
        let pcurve = build_pcurve_on_surface_range(
            &wall.surface,
            &curve,
            domain[0],
            domain[1],
            true,
            (tolerance * 0.05).max(1.0e-9),
        )?;
        let pcurve_domain = pcurve.domain()?;
        let mut previous_u: Option<f64> = None;
        for sample in 0..=32 {
            let fraction = sample as f64 / 32.0;
            let t = domain[0] + (domain[1] - domain[0]) * fraction;
            let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
            let uv = pcurve.evaluate(q)?;
            let surface_point = wall.surface.evaluate(uv.x, uv.y)?;
            let miss = curve.evaluate(t)?.sub(surface_point).length();
            if miss > bar || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25) {
                return Err(format!(
                    "hybrid BREP: cylinder/cylinder chain {chain_index} pcurve on region {owner} failed agreement (miss {miss:.3e}, bar {bar:.3e})"
                ));
            }
            previous_u = Some(uv.x);
        }
        pcurves.push((owner, pcurve));
    }
    Ok((curve, domain[0], domain[1], a1 > a0, pcurves))
}

fn exact_local_cone_plane_conic(
    info: &ConeInfo,
    plane: PlaneInfo,
    points: &[Vec3],
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let mut angles = points
        .iter()
        .map(|&point| {
            let radial = cone_radial(info.carrier, point);
            radial.dot(info.y_axis).atan2(radial.dot(info.x_axis))
        })
        .collect::<Vec<_>>();
    for index in 1..angles.len() {
        while angles[index] - angles[index - 1] > std::f64::consts::PI {
            angles[index] -= TAU;
        }
        while angles[index] - angles[index - 1] < -std::f64::consts::PI {
            angles[index] += TAU;
        }
    }
    if !monotone_parameter_walk(&angles, 1.0e-5) {
        return Err("hybrid BREP: cone/plane source angles are not monotone".into());
    }
    let low = angles[0].min(*angles.last().unwrap());
    let high = angles[0].max(*angles.last().unwrap());
    if high - low <= 1.0e-10 || high - low > TAU + 1.0e-10 {
        return Err("hybrid BREP: cone/plane source angle span is invalid".into());
    }

    // Map a finite exact reference-circle arc projectively through the cone
    // apex onto the cutting plane. Unlike mapping a full circle, this keeps
    // hyperbolic/parabolic branches one-signed over the actual trim interval.
    let reference_station = info.base_station + 0.5 * info.height;
    let reference_radius = reference_station * info.carrier.half_angle.tan();
    let center = info
        .carrier
        .apex
        .add(info.carrier.axis.scale(reference_station));
    let circle = make_arc(
        center,
        info.x_axis,
        info.y_axis,
        reference_radius,
        low,
        high,
    )?;
    let normal = plane.normal.normalized()?;
    let apex = info.carrier.apex;
    let scale = plane.origin.sub(apex).dot(normal);
    if scale.abs() <= tolerance {
        return Err("hybrid BREP: cone/plane section passes through the apex".into());
    }
    let mut controls = circle
        .control_points
        .iter()
        .map(|point| {
            let relative = Vec3::new(
                point.x - point.w * apex.x,
                point.y - point.w * apex.y,
                point.z - point.w * apex.z,
            );
            let weight = relative.dot(normal);
            let scaled = relative.scale(scale);
            Vec4 {
                x: apex.x * weight + scaled.x,
                y: apex.y * weight + scaled.y,
                z: apex.z * weight + scaled.z,
                w: weight,
            }
        })
        .collect::<Vec<_>>();
    if controls.iter().any(|point| point.w.abs() <= 1.0e-12) {
        return Err("hybrid BREP: cone/plane local conic crosses an asymptote".into());
    }
    let sign = controls[0].w.signum();
    if controls.iter().any(|point| point.w.signum() != sign) {
        return Err("hybrid BREP: cone/plane local conic has mixed weight signs".into());
    }
    if sign < 0.0 {
        for point in &mut controls {
            point.x = -point.x;
            point.y = -point.y;
            point.z = -point.z;
            point.w = -point.w;
        }
    }
    NurbsCurve::new(circle.degree, circle.knots, controls)
}

fn unwrap(values: &mut [f64]) {
    for index in 1..values.len() {
        while values[index] - values[index - 1] > 0.5 {
            values[index] -= 1.0;
        }
        while values[index] - values[index - 1] < -0.5 {
            values[index] += 1.0;
        }
    }
}

fn monotone_parameter_walk(values: &[f64], tolerance: f64) -> bool {
    let net = values.last().unwrap() - values[0];
    if net.abs() <= tolerance {
        return false;
    }
    let sign = net.signum();
    let variation = values
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .try_fold(0.0, |sum, step| {
            (step * sign >= -tolerance).then_some(sum + step.abs())
        });
    variation.is_some_and(|total| (total - net.abs()).abs() <= tolerance * values.len() as f64)
}

/// A boolean cut through a polygonal cylinder puts its ruling on a chord,
/// not on the circle. Radially projecting each arc end then leaves the
/// adjoining plane/plane edge behind. Reconstruct the ruling on BOTH its
/// carriers first, so every chain uses the same corrected corner.
fn snap_cylinder_ruling_vertices(
    vertices: &mut [Vec3],
    chains: &[Chain],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderCarrier>,
    tolerance: f64,
) {
    let mut incident = HashMap::<usize, BTreeSet<u32>>::new();
    for chain in chains {
        for &vertex in &chain.vertices {
            incident
                .entry(vertex)
                .or_default()
                .extend([chain.adjacent.0, chain.adjacent.1]);
        }
    }
    let source = vertices.to_vec();
    let mut proposals = BTreeMap::<usize, Vec<Vec3>>::new();
    for chain in chains.iter().filter(|chain| !chain.closed) {
        let (cylinder, plane) = match (
            cylinders.get(&chain.adjacent.0),
            planes.get(&chain.adjacent.1),
            cylinders.get(&chain.adjacent.1),
            planes.get(&chain.adjacent.0),
        ) {
            (Some(cylinder), Some(plane), _, _) | (_, _, Some(cylinder), Some(plane)) => {
                (cylinder, plane)
            }
            _ => continue,
        };
        let points = chain_points(chain, &source);
        let span = points.last().unwrap().sub(points[0]);
        // Only axial rulings; rings and oblique sections keep their own lane.
        if span.dot(cylinder.axis).abs() <= tolerance
            || span.cross(cylinder.axis).length() > tolerance
            || max_line_deviation(&points) > tolerance
        {
            continue;
        }
        let normal = plane.normal;
        let radial_normal = normal.sub(cylinder.axis.scale(normal.dot(cylinder.axis)));
        let Ok(x) = radial_normal.normalized() else {
            continue;
        };
        if normal.dot(cylinder.axis).abs() * span.length() > tolerance {
            continue;
        }
        let y = cylinder.axis.cross(x);
        let bar = accepted_radial_bar(tolerance, cylinder.deviation);
        for (&vertex, point) in chain.vertices.iter().zip(points) {
            let center = cylinder.origin.add(
                cylinder
                    .axis
                    .scale(point.sub(cylinder.origin).dot(cylinder.axis)),
            );
            let offset = plane.origin.sub(center).dot(normal) / radial_normal.length();
            let height_squared = cylinder.radius * cylinder.radius - offset * offset;
            if height_squared <= 0.0 {
                continue;
            }
            let height = height_squared.sqrt();
            let base = center.add(x.scale(offset));
            let a = base.add(y.scale(height));
            let b = base.sub(y.scale(height));
            let exact = if a.sub(point).length() < b.sub(point).length() {
                a
            } else {
                b
            };
            if !exact.sub(point).length().is_finite() || exact.sub(point).length() > bar {
                continue;
            }
            // Do not move a corner off any other incident carrier, including
            // a facet produced by a previous local demotion. Unsupported
            // neighbors keep their original vertices.
            let compatible = incident[&vertex].iter().all(|region| {
                if let Some(plane) = planes.get(region) {
                    exact.sub(plane.origin).dot(plane.normal).abs() <= tolerance
                } else if let Some(cylinder) = cylinders.get(region) {
                    let delta = exact.sub(cylinder.origin);
                    (delta
                        .sub(cylinder.axis.scale(delta.dot(cylinder.axis)))
                        .length()
                        - cylinder.radius)
                        .abs()
                        <= tolerance
                } else {
                    false
                }
            });
            if compatible {
                proposals.entry(vertex).or_default().push(exact);
            }
        }
    }
    for (vertex, candidates) in proposals {
        if candidates
            .iter()
            .all(|p| p.sub(candidates[0]).length() <= tolerance)
        {
            vertices[vertex] = candidates[0];
        }
    }
}

fn exact_arena_vertex(
    mesh_vertex: usize,
    exact_point: Vec3,
    tolerance: f64,
    arena: &mut TopologyArena,
    arena_vertices: &mut HashMap<usize, brep_kernel::VertexId>,
    ids: &mut Ids,
) -> Result<brep_kernel::VertexId, String> {
    if let Some(&vertex) = arena_vertices.get(&mesh_vertex) {
        let existing = arena
            .vertices
            .get(vertex)
            .ok_or_else(|| "hybrid BREP: reused arena vertex disappeared".to_owned())?;
        let disagreement = existing.point.sub(exact_point).length();
        if !disagreement.is_finite() || disagreement > tolerance {
            return Err(format!(
                "hybrid BREP: exact curves disagree by {disagreement:.6e} (bar {tolerance:.6e}) at shared mesh vertex {mesh_vertex}: ({:.6},{:.6},{:.6}) vs ({:.6},{:.6},{:.6})",
                existing.point.x,
                existing.point.y,
                existing.point.z,
                exact_point.x,
                exact_point.y,
                exact_point.z,
            ));
        }
        return Ok(vertex);
    }
    let vertex = arena.vertices.insert(ArenaVertex {
        wire_id: ids.next(),
        point: exact_point,
    });
    arena_vertices.insert(mesh_vertex, vertex);
    Ok(vertex)
}

#[derive(Clone, Copy)]
enum RevolveInfoRef<'a> {
    Cylinder(&'a CylinderInfo),
    Cone(&'a ConeInfo),
    Sphere(&'a SphereInfo),
    Torus(&'a TorusInfo),
}

impl<'a> RevolveInfoRef<'a> {
    fn surface(self) -> &'a NurbsSurface {
        match self {
            Self::Cylinder(info) => &info.surface,
            Self::Cone(info) => &info.surface,
            Self::Sphere(info) => &info.surface,
            Self::Torus(info) => &info.surface,
        }
    }

    fn coordinates(self, point: Vec3) -> [f64; 2] {
        match self {
            Self::Cylinder(info) => cylinder_coordinates(info, point),
            Self::Cone(info) => cone_coordinates(info, point),
            Self::Sphere(info) => sphere_coordinates(info, point),
            Self::Torus(info) => torus_coordinates(info, point),
        }
    }

    fn station(self, point: Vec3) -> f64 {
        match self {
            Self::Cylinder(info) => station(info.carrier, point),
            Self::Cone(info) => cone_station(info.carrier, point),
            Self::Sphere(info) => sphere_station(info, point),
            Self::Torus(info) => point.sub(info.carrier.center).dot(info.axis),
        }
    }

    /// The exact topology tolerance expressed along the angular parameter:
    /// an arc that starts at the seam ruling reaches it through a second
    /// chain whose fitted radial direction differs by the carrier noise.
    fn seam_tolerance(self, tolerance: f64) -> f64 {
        match self {
            Self::Cylinder(info) => (tolerance / (TAU * info.carrier.radius)).max(1.0e-8),
            Self::Cone(_) => 1.0e-8,
            Self::Sphere(info) => (tolerance / (TAU * info.carrier.radius)).max(1.0e-8),
            Self::Torus(info) => (tolerance / (TAU * info.carrier.major_radius)).max(1.0e-8),
        }
    }

    /// Radial bar for a source vertex on a ring or arc chain: the exact
    /// topology tolerance, or the deviation the segmenter already accepted
    /// for this carrier when that is wider. A vertex the tessellator left
    /// on a rim chord is still on the ring the exact circle replaces.
    fn radial_bar(self, tolerance: f64) -> f64 {
        match self {
            Self::Cylinder(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Cone(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Sphere(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Torus(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
        }
    }

    fn radial_error(self, point: Vec3) -> f64 {
        match self {
            Self::Cylinder(info) => {
                (radial(info.carrier, point).length() - info.carrier.radius).abs()
            }
            Self::Cone(info) => {
                let s = cone_station(info.carrier, point);
                (cone_radial(info.carrier, point).length() - s * info.carrier.half_angle.tan())
                    .abs()
                    * info.carrier.half_angle.cos()
            }
            Self::Sphere(info) => sphere_radial_error(info.carrier, point),
            Self::Torus(info) => torus_radial_error(info.carrier, point),
        }
    }
}

fn revolve_info<'a>(
    region: u32,
    cylinders: &'a HashMap<u32, CylinderInfo>,
    cones: &'a HashMap<u32, ConeInfo>,
    spheres: &'a HashMap<u32, SphereInfo>,
    tori: &'a HashMap<u32, TorusInfo>,
) -> Option<RevolveInfoRef<'a>> {
    cylinders
        .get(&region)
        .map(RevolveInfoRef::Cylinder)
        .or_else(|| cones.get(&region).map(RevolveInfoRef::Cone))
        .or_else(|| spheres.get(&region).map(RevolveInfoRef::Sphere))
        .or_else(|| tori.get(&region).map(RevolveInfoRef::Torus))
}

#[allow(clippy::too_many_arguments)]
fn build_chain_edge(
    chain_index: usize,
    chain: &Chain,
    vertices: &[Vec3],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    spheres: &HashMap<u32, SphereInfo>,
    tori: &HashMap<u32, TorusInfo>,
    tolerance: f64,
    arena: &mut TopologyArena,
    arena_vertices: &mut HashMap<usize, brep_kernel::VertexId>,
    ids: &mut Ids,
) -> Result<BuiltEdge, String> {
    let points = chain_points(chain, vertices);
    let mut custom_revolve_pcurves = HashMap::new();
    let revolve_ids = [chain.adjacent.0, chain.adjacent.1]
        .into_iter()
        .filter(|id| {
            cylinders.contains_key(id)
                || cones.contains_key(id)
                || spheres.contains_key(id)
                || tori.contains_key(id)
        })
        .collect::<Vec<_>>();
    if revolve_ids.len() > 2 {
        return Err(format!(
            "hybrid BREP: chain {chain_index} has too many revolved neighbors"
        ));
    }
    // An open arc on a sphere patch takes its curve from the other revolve;
    // the sphere side is fitted to it below.
    let mut revolve_ids = revolve_ids;
    if !chain.closed && revolve_ids.len() == 2 && spheres.contains_key(&revolve_ids[0]) {
        revolve_ids.swap(0, 1);
    }
    let (curve, t0, t1, curve_along_chain, primary_uv) = if let Some(&region) = revolve_ids.first()
    {
        let info = revolve_info(region, cylinders, cones, spheres, tori).unwrap();
        let stations = points.iter().map(|&p| info.station(p)).collect::<Vec<_>>();
        let station_spread = stations.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - stations.iter().copied().fold(f64::INFINITY, f64::min);
        let radius_error = points
            .iter()
            .map(|&point| info.radial_error(point))
            .fold(0.0, f64::max);
        let mut coordinates = points
            .iter()
            .map(|&p| info.coordinates(p))
            .collect::<Vec<_>>();
        let mut us = coordinates.iter().map(|uv| uv[0]).collect::<Vec<_>>();
        unwrap(&mut us);
        for (uv, u) in coordinates.iter_mut().zip(&us) {
            uv[0] = *u;
        }
        let tilted_plane = match info {
            RevolveInfoRef::Sphere(sphere) if chain.closed && station_spread > tolerance => {
                [chain.adjacent.0, chain.adjacent.1]
                    .into_iter()
                    .find_map(|neighbor| planes.get(&neighbor).map(|plane| (sphere, *plane)))
            }
            _ => None,
        };
        if let Some((sphere, plane)) = tilted_plane {
            let (curve, t0, t1, curve_along_chain, pcurve) =
                tilted_sphere_ring_edge(chain_index, sphere, plane, &points, tolerance)?;
            let domain = pcurve.domain()?;
            let start = pcurve.evaluate(domain[0])?;
            let end = pcurve.evaluate(domain[1])?;
            custom_revolve_pcurves.insert(region, pcurve);
            (
                curve,
                t0,
                t1,
                curve_along_chain,
                Some(ChainUv {
                    region,
                    start: [start.x, start.y],
                    end: [end.x, end.y],
                }),
            )
        } else if chain.closed {
            let first = points[0];
            let mut ring_points = points.clone();
            ring_points.push(first);
            let mut ring_us = ring_points
                .iter()
                .map(|&p| info.coordinates(p)[0])
                .collect::<Vec<_>>();
            unwrap(&mut ring_us);
            let sweep = ring_us[ring_us.len() - 1] - ring_us[0];
            if station_spread > tolerance
                || radius_error > info.radial_bar(tolerance)
                || (sweep.abs() - 1.0).abs() > 0.02
                || !monotone_parameter_walk(&ring_us, 1.0e-5)
            {
                return Err(format!(
                    "hybrid BREP: closed chain {chain_index} (region {region}, adjacent {:?}) is not a revolve ring: station spread {station_spread:.3e}, radial error {radius_error:.3e} (bar {:.3e}), sweep {sweep:.6}, monotone {}",
                    chain.adjacent,
                    info.radial_bar(tolerance),
                    monotone_parameter_walk(&ring_us, 1.0e-5)
                ));
            }
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            let curve = info.surface().iso_curve_v(v)?;
            (
                curve,
                0.0,
                1.0,
                sweep > 0.0,
                Some(ChainUv {
                    region,
                    start: [0.0, v],
                    end: [if sweep > 0.0 { 1.0 } else { -1.0 }, v],
                }),
            )
        } else if station_spread <= tolerance && radius_error <= info.radial_bar(tolerance) {
            if !monotone_parameter_walk(&us, 1.0e-5) {
                return Err(format!(
                    "hybrid BREP: cylinder arc chain {chain_index} backtracks"
                ));
            }
            let mut a = us[0];
            let mut b = *us.last().unwrap();
            let seam_tolerance = info.seam_tolerance(tolerance);
            let shift = (-2..=2)
                .map(f64::from)
                .find(|shift| {
                    a + shift >= -seam_tolerance
                        && b + shift >= -seam_tolerance
                        && a + shift <= 1.0 + seam_tolerance
                        && b + shift <= 1.0 + seam_tolerance
                })
                .ok_or_else(|| {
                    format!(
                        "hybrid BREP: cylinder arc chain {chain_index} (region {region}, adjacent {:?}) crosses the chosen seam: u {a:.6} -> {b:.6} over {} points",
                        chain.adjacent,
                        points.len()
                    )
                })?;
            a += shift;
            b += shift;
            if a.abs() <= seam_tolerance {
                a = 0.0;
            } else if (a - 1.0).abs() <= seam_tolerance {
                a = 1.0;
            }
            if b.abs() <= seam_tolerance {
                b = 0.0;
            } else if (b - 1.0).abs() <= seam_tolerance {
                b = 1.0;
            }
            if (b - a).abs() <= 1.0e-9 || (b - a).abs() >= 1.0 - 1.0e-9 {
                return Err(format!(
                    "hybrid BREP: cylinder arc chain {chain_index} has invalid sweep"
                ));
            }
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            let low = a.min(b);
            let high = a.max(b);
            let curve = trim_curve(&info.surface().iso_curve_v(v)?, low, high)?;
            let domain = curve.domain()?;
            (
                curve,
                domain[0],
                domain[1],
                b > a,
                Some(ChainUv {
                    region,
                    start: [a, v],
                    end: [b, v],
                }),
            )
        } else if !matches!(info, RevolveInfoRef::Sphere(_) | RevolveInfoRef::Torus(_))
            && (max_line_deviation(&points) <= tolerance
                || us.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                    - us.iter().copied().fold(f64::INFINITY, f64::min)
                    <= 1.0e-4)
        {
            let a = coordinates[0];
            let mut b = *coordinates.last().unwrap();
            while b[0] - a[0] > 0.5 {
                b[0] -= 1.0;
            }
            while b[0] - a[0] < -0.5 {
                b[0] += 1.0;
            }
            if (b[0] - a[0]).abs() > 1.0e-4 {
                return Err(format!(
                    "hybrid BREP: chain {chain_index} is not an axial ruling"
                ));
            }
            let u = (a[0] + b[0]) * 0.5;
            let surface = info.surface();
            // An isoparametric ruling sits at the mean of its endpoint
            // angles, which moves each endpoint tangentially by half their
            // angular spread.  The arc chains meeting this ruling place
            // those corner vertices at their own angle, and
            // `exact_arena_vertex` compares the two placements at the
            // exact-topology bar; a tessellation whose ruling endpoints
            // differ by a fraction of a degree would lose its whole
            // cylinder to that disagreement.  Half the bar leaves the rest
            // of it to the offsets the arc chain itself carries (its mean
            // station, its seam snap).
            let parametric_offset = [a, b]
                .into_iter()
                .map(|uv| {
                    Ok(surface
                        .evaluate(u.rem_euclid(1.0), uv[1])?
                        .sub(surface.evaluate(uv[0].rem_euclid(1.0), uv[1])?)
                        .length())
                })
                .collect::<Result<Vec<f64>, String>>()?
                .into_iter()
                .fold(0.0, f64::max);
            if parametric_offset > tolerance * 0.5 {
                // Build the segment between the endpoints' own positions on
                // the carrier instead.  It is the chord of a helix spanning
                // at most 1.0e-4 of a turn, so it leaves the surface by
                // radius * (pi * 1.0e-4)^2 / 2 -- under a nanometre at any
                // scale a mesh import carries -- while both corners stay
                // exactly where the arcs that share them put them.
                if max_line_deviation(&points) > tolerance {
                    return Err(format!(
                        "hybrid BREP: chain {chain_index} is neither an axial ruling nor straight"
                    ));
                }
                (
                    make_line(
                        surface.evaluate(a[0].rem_euclid(1.0), a[1])?,
                        surface.evaluate(b[0].rem_euclid(1.0), b[1])?,
                    )?,
                    0.0,
                    1.0,
                    true,
                    Some(ChainUv {
                        region,
                        start: a,
                        end: b,
                    }),
                )
            } else {
                let low = a[1].min(b[1]);
                let high = a[1].max(b[1]);
                let curve = trim_curve(&surface.iso_curve_u(u.rem_euclid(1.0))?, low, high)?;
                let domain = curve.domain()?;
                (
                    curve,
                    domain[0],
                    domain[1],
                    b[1] > a[1],
                    Some(ChainUv {
                        region,
                        start: [u, a[1]],
                        end: [u, b[1]],
                    }),
                )
            }
        } else if let (Some(cone), Some(plane)) = (
            cones.get(&region),
            [chain.adjacent.0, chain.adjacent.1]
                .into_iter()
                .find_map(|neighbor| planes.get(&neighbor)),
        ) {
            let normal = plane.normal.normalized()?;
            let x = normal.perpendicular()?;
            let y = normal.cross(x).normalized()?;
            let extent = points
                .iter()
                .map(|point| point.sub(plane.origin).length())
                .fold(1.0_f64, f64::max)
                * 4.0;
            let plane_surface = brep_kernel::make_plane(
                plane.origin.sub(x.scale(extent)).sub(y.scale(extent)),
                x,
                y,
                extent * 2.0,
                extent * 2.0,
            )?;
            let branches = intersect_analytic_pair(&plane_surface, &cone.surface, tolerance)
                .filter(|branches| !branches.is_empty())
                .unwrap_or_else(|| {
                    exact_local_cone_plane_conic(cone, *plane, &points, tolerance)
                        .into_iter()
                        .collect()
                });
            let mut candidates = branches
                .into_iter()
                .map(|curve| {
                    let projections = points
                        .iter()
                        .map(|&point| project_point_to_curve(&curve, point))
                        .collect::<Result<Vec<_>, String>>()?;
                    let score: f64 = projections
                        .iter()
                        .map(|projection| projection.distance)
                        .sum();
                    Ok((score, curve, projections))
                })
                .collect::<Result<Vec<_>, String>>()?;
            candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
            let (_, curve, projections) = candidates.into_iter().next().ok_or_else(|| {
                format!("hybrid BREP: exact cone/plane conic is empty for chain {chain_index}")
            })?;
            let source_error = projections
                .iter()
                .map(|projection| projection.distance)
                .fold(0.0_f64, f64::max);
            if source_error > tolerance * 2.0 {
                return Err(format!(
                    "hybrid BREP: exact cone/plane conic chain {chain_index} misses source by {source_error:.6e}"
                ));
            }
            let p0 = projections[0];
            let p1 = *projections.last().unwrap();
            let low = p0.u.min(p1.u);
            let high = p0.u.max(p1.u);
            let direction = (p1.u - p0.u).signum();
            let monotone = direction != 0.0
                && projections.windows(2).all(|pair| {
                    (pair[1].u - pair[0].u) * direction >= -1.0e-8
                        && pair[1].u >= low - 1.0e-8
                        && pair[1].u <= high + 1.0e-8
                });
            if !monotone || high - low <= 1.0e-10 {
                return Err(format!(
                    "hybrid BREP: exact cone/plane conic chain {chain_index} crosses its branch seam"
                ));
            }
            let pcurve = build_pcurve_on_surface_range(
                &cone.surface,
                &curve,
                low,
                high,
                true,
                (tolerance * 0.05).max(1.0e-9),
            )?;
            let pcurve_domain = pcurve.domain()?;
            for sample in 0..=16 {
                let fraction = sample as f64 / 16.0;
                let t = low + (high - low) * fraction;
                let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
                let edge_point = curve.evaluate(t)?;
                let uv = pcurve.evaluate(q)?;
                let surface_point = cone.surface.evaluate(uv.x, uv.y)?;
                if edge_point.sub(surface_point).length() > tolerance * 0.1
                    || edge_point.sub(plane.origin).dot(normal).abs() > tolerance * 0.1
                {
                    return Err(format!(
                        "hybrid BREP: cone/plane conic chain {chain_index} pcurve failed agreement"
                    ));
                }
            }
            let edge_uv_start = pcurve.evaluate(pcurve_domain[0])?;
            let edge_uv_end = pcurve.evaluate(pcurve_domain[1])?;
            let curve_along_chain = p1.u > p0.u;
            let (chain_uv_start, chain_uv_end) = if curve_along_chain {
                (edge_uv_start, edge_uv_end)
            } else {
                (edge_uv_end, edge_uv_start)
            };
            custom_revolve_pcurves.insert(region, pcurve);
            (
                curve,
                low,
                high,
                curve_along_chain,
                Some(ChainUv {
                    region,
                    start: [chain_uv_start.x, chain_uv_start.y],
                    end: [chain_uv_end.x, chain_uv_end.y],
                }),
            )
        } else if let (Some(first), Some(second)) = (
            cylinders.get(&chain.adjacent.0).filter(|_| !chain.closed),
            cylinders.get(&chain.adjacent.1),
        ) {
            let (curve, t0, t1, curve_along_chain, pcurves) = cylinder_pair_section_edge(
                chain_index,
                [(chain.adjacent.0, first), (chain.adjacent.1, second)],
                &points,
                tolerance,
            )?;
            let mut primary = None;
            for (owner, pcurve) in pcurves {
                let domain = pcurve.domain()?;
                let (start, end) = (pcurve.evaluate(domain[0])?, pcurve.evaluate(domain[1])?);
                let (start, end) = if curve_along_chain { (start, end) } else { (end, start) };
                if owner == region {
                    primary = Some(ChainUv {
                        region,
                        start: [start.x, start.y],
                        end: [end.x, end.y],
                    });
                }
                custom_revolve_pcurves.insert(owner, pcurve);
            }
            (curve, t0, t1, curve_along_chain, primary)
        } else {
            return Err(format!(
                "hybrid BREP: revolved chain {chain_index} (region {region}, adjacent {:?}, closed={}, station spread {station_spread:.6e}, radial error {radius_error:.6e}, line deviation {:.6e}, uv {:?}->{:?}, points {:?}) is neither arc, ring, nor ruling",
                chain.adjacent,
                chain.closed,
                max_line_deviation(&points),
                coordinates.first(),
                coordinates.last(),
                points,
                // Source samples are included to make conservative demotions
                // diagnosable in focused regression tests.
            ));
        }
    } else {
        if chain.closed || max_line_deviation(&points) > tolerance {
            return Err(format!(
                "hybrid BREP: non-cylinder chain {chain_index} is not a line"
            ));
        }
        (
            make_line(points[0], *points.last().unwrap())?,
            0.0,
            1.0,
            true,
            None,
        )
    };

    let first = chain.vertices[0];
    let last = if chain.closed {
        first
    } else {
        *chain.vertices.last().unwrap()
    };
    let (start_mesh, end_mesh) = if curve_along_chain {
        (first, last)
    } else {
        (last, first)
    };
    let exact_start = curve.evaluate(t0)?;
    let exact_end = curve.evaluate(t1)?;
    let start = exact_arena_vertex(
        start_mesh,
        exact_start,
        tolerance,
        arena,
        arena_vertices,
        ids,
    )?;
    let end = exact_arena_vertex(end_mesh, exact_end, tolerance, arena, arena_vertices, ids)?;
    let edge = arena.edges.insert(ArenaEdge {
        wire_id: ids.next(),
        curve,
        t0,
        t1,
        start,
        end,
        degenerate: false,
        name: None,
    });
    if !chain.closed {
        if let Some(sphere) = revolve_ids.get(1).and_then(|secondary| spheres.get(secondary)) {
            let pcurve = sphere_patch_pcurve(
                chain_index,
                sphere,
                &arena.edges[edge].curve,
                t0,
                t1,
                tolerance,
            )?;
            custom_revolve_pcurves.insert(revolve_ids[1], pcurve);
        }
    }
    let mut revolve_uvs = primary_uv.into_iter().collect::<Vec<_>>();
    if let Some(pcurve) = revolve_ids
        .get(1)
        .and_then(|secondary| custom_revolve_pcurves.get(secondary))
    {
        let domain = pcurve.domain()?;
        let (start, end) = (pcurve.evaluate(domain[0])?, pcurve.evaluate(domain[1])?);
        let (start, end) = if curve_along_chain { (start, end) } else { (end, start) };
        revolve_uvs.push(ChainUv {
            region: revolve_ids[1],
            start: [start.x, start.y],
            end: [end.x, end.y],
        });
    } else if revolve_ids.len() == 2 {
        let secondary_region = revolve_ids[1];
        let secondary = revolve_info(secondary_region, cylinders, cones, spheres, tori).unwrap();
        let mut coordinates = points
            .iter()
            .map(|&point| secondary.coordinates(point))
            .collect::<Vec<_>>();
        let mut us = coordinates.iter().map(|uv| uv[0]).collect::<Vec<_>>();
        unwrap(&mut us);
        for (uv, u) in coordinates.iter_mut().zip(&us) {
            uv[0] = *u;
        }
        let uv = if chain.closed {
            let mut ring_points = points.clone();
            ring_points.push(points[0]);
            let mut ring_us = ring_points
                .iter()
                .map(|&point| secondary.coordinates(point)[0])
                .collect::<Vec<_>>();
            unwrap(&mut ring_us);
            let sweep = ring_us.last().unwrap() - ring_us[0];
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            ChainUv {
                region: secondary_region,
                start: [0.0, v],
                end: [if sweep > 0.0 { 1.0 } else { -1.0 }, v],
            }
        } else {
            let start = coordinates[0];
            let mut end = *coordinates.last().unwrap();
            if (start[1] - end[1]).abs() <= 1.0e-5 {
                // A common ring arc: constant station on both revolves.
                ChainUv {
                    region: secondary_region,
                    start,
                    end,
                }
            } else {
                // A shared axial ruling: constant angle on the secondary
                // cylinder, which may sit on its parameter seam.
                while end[0] - start[0] > 0.5 {
                    end[0] -= 1.0;
                }
                while end[0] - start[0] < -0.5 {
                    end[0] += 1.0;
                }
                if (end[0] - start[0]).abs() > 1.0e-4 {
                    return Err(format!(
                        "hybrid BREP: shared revolve chain {chain_index} is neither a common ring arc nor a common ruling"
                    ));
                }
                let u = ((start[0] + end[0]) * 0.5).rem_euclid(1.0);
                ChainUv {
                    region: secondary_region,
                    start: [u, start[1]],
                    end: [u, end[1]],
                }
            }
        };
        revolve_uvs.push(uv);
    }
    Ok(BuiltEdge {
        edge,
        curve_along_chain,
        revolve_uvs,
        custom_revolve_pcurves,
    })
}

fn plane_frame(
    plane: PlaneInfo,
    boundary: &RegionBoundary,
    chains: &[Chain],
    vertices: &[Vec3],
) -> Result<(Vec3, Vec3, Vec3, f64, f64), String> {
    let normal = plane.normal.normalized()?;
    let x = normal.perpendicular()?;
    let y = normal.cross(x).normalized()?;
    let mut min_u = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut min_v = f64::INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for traversal in boundary.cycles.iter().flatten() {
        for &vertex in &chains[traversal.chain].vertices {
            let delta = vertices[vertex].sub(plane.origin);
            let u = delta.dot(x);
            let v = delta.dot(y);
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
    }
    if !min_u.is_finite() || max_u - min_u <= 1.0e-12 || max_v - min_v <= 1.0e-12 {
        return Err("hybrid BREP: degenerate planar face bounds".into());
    }
    Ok((
        plane.origin.add(x.scale(min_u)).add(y.scale(min_v)),
        x,
        y,
        max_u - min_u,
        max_v - min_v,
    ))
}

fn affine_pcurve(curve: &NurbsCurve, origin: Vec3, x: Vec3, y: Vec3) -> Result<NurbsCurve, String> {
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            if !control.w.is_finite() || control.w.abs() <= f64::EPSILON {
                return Err("hybrid BREP: pcurve source has an invalid homogeneous weight".into());
            }
            let point = Vec3::new(
                control.x / control.w,
                control.y / control.w,
                control.z / control.w,
            );
            let delta = point.sub(origin);
            Ok(Vec4 {
                x: delta.dot(x) * control.w,
                y: delta.dot(y) * control.w,
                z: 0.0,
                w: control.w,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
}

#[allow(clippy::too_many_arguments)]
fn build_plane_face(
    _region: u32,
    plane: PlaneInfo,
    boundary: &RegionBoundary,
    chains: &[Chain],
    built_edges: &[BuiltEdge],
    vertices: &[Vec3],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let (origin, x, y, u_extent, v_extent) = plane_frame(plane, boundary, chains, vertices)?;
    let surface = brep_kernel::make_plane(origin, x, y, u_extent, v_extent)?;
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let edge = &arena.edges[built.edge];
            let mut pcurve = affine_pcurve(&edge.curve, origin, x, y)?;
            let forward = traversal.forward == built.curve_along_chain;
            if !forward {
                pcurve = pcurve.reversed()?;
            }
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface,
        same_sense: true,
        loops,
        name: None,
    }))
}

fn parameter_line(start: [f64; 2], end: [f64; 2]) -> Result<NurbsCurve, String> {
    make_line(
        Vec3::new(start[0], start[1], 0.0),
        Vec3::new(end[0], end[1], 0.0),
    )
}

fn build_cylinder_face(
    region: u32,
    cylinder: &CylinderInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: cylinder {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let mut pcurve = built
                .custom_revolve_pcurves
                .get(&region)
                .cloned()
                .unwrap_or(parameter_line(start, end)?);
            if !forward && built.custom_revolve_pcurves.contains_key(&region) {
                pcurve = pcurve.reversed()?;
            }
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: cylinder.surface.clone(),
        same_sense: cylinder.carrier.sense == 1,
        loops,
        name: None,
    }))
}

/// A sphere face from its ring loops.  A cap around the north pole also
/// carries the surface seam from the ring to the pole and the degenerate
/// pole edge, the way the kernel's own sphere solid is built, so its
/// parameter-space loop closes.
fn build_sphere_face(
    region: u32,
    sphere: &SphereInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        let mut ring: Option<(brep_kernel::VertexId, [f64; 2], [f64; 2])> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: sphere {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let edge = &arena.edges[built.edge];
            ring = Some((edge.start, start, end));
            // A tilted ring carries its fitted loop, oriented along its curve.
            let pcurve = match built.custom_revolve_pcurves.get(&region) {
                Some(pcurve) if forward => pcurve.clone(),
                Some(pcurve) => pcurve.reversed()?,
                None => parameter_line(start, end)?,
            };
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        if sphere.north_pole_inside {
            let Some((ring_vertex, ring_start, ring_end)) = ring.filter(|_| cycle.len() == 1) else {
                return Err(format!(
                    "hybrid BREP: sphere {region} cap boundary is not a single ring"
                ));
            };
            let v = ring_start[1];
            let seam = trim_curve(&sphere.surface.iso_curve_u(0.0)?, v, 1.0)?;
            let domain = seam.domain()?;
            let pole_point = sphere.surface.evaluate(0.0, 1.0)?;
            let pole = arena.vertices.insert(ArenaVertex {
                wire_id: ids.next(),
                point: pole_point,
            });
            let seam_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: seam,
                t0: domain[0],
                t1: domain[1],
                start: ring_vertex,
                end: pole,
                degenerate: false,
                name: None,
            });
            let pole_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: make_line(pole_point, pole_point)?,
                t0: 0.0,
                t1: 1.0,
                start: pole,
                end: pole,
                degenerate: true,
                name: None,
            });
            // Up the seam on the ring's far side, across the pole, down the
            // seam on the ring's near side: the loop closes in parameter
            // space with the cap on its left.
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], v], [ring_end[0], 1.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: pole_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], 1.0], [ring_start[0], 1.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: false,
                pcurve: parameter_line([ring_start[0], 1.0], [ring_start[0], v])?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: sphere.surface.clone(),
        same_sense: sphere.carrier.sense == 1,
        loops,
        name: None,
    }))
}

/// A revolve face whose loops are all parameter lines of its surface (a
/// torus band between two latitude rings, as a full cylinder wall is
/// between two circles): each loop's pcurves are the chains' own uv lines.
#[allow(clippy::too_many_arguments)]
fn build_revolve_band_face(
    region: u32,
    kind: &str,
    surface: &NurbsSurface,
    same_sense: bool,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            if built.custom_revolve_pcurves.contains_key(&region) {
                return Err(format!("hybrid BREP: {kind} {region} has a fitted-pcurve edge"));
            }
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: {kind} {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward: traversal.forward == built.curve_along_chain,
                pcurve: parameter_line(start, end)?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: surface.clone(),
        same_sense,
        loops,
        name: None,
    }))
}

/// Tube angle of a point about a torus frame, unwrapped around the band's
/// middle: in [0, 2π] for every point of a band clear of the tube seam.
fn torus_tube_angle(center: Vec3, axis: Vec3, major_radius: f64, middle: f64, point: Vec3) -> f64 {
    let d = point.sub(center);
    let z = d.dot(axis);
    let rho = d.sub(axis.scale(z)).length();
    let raw = z.atan2(rho - major_radius);
    let delta = (raw - middle + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
    middle + delta
}

fn torus_coordinates(info: &TorusInfo, point: Vec3) -> [f64; 2] {
    let d = point.sub(info.carrier.center);
    let radial = d.sub(info.axis.scale(d.dot(info.axis)));
    let angle = radial.dot(info.y_axis).atan2(radial.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    let tube = torus_tube_angle(
        info.carrier.center,
        info.axis,
        info.carrier.major_radius,
        info.band_middle,
        point,
    )
    .clamp(0.0, TAU);
    [u, circle_angle_to_parameter(4, TAU, tube)]
}

/// Frame each torus band on the kernel's torus template. The band must be
/// bounded by closed rings only; its axis snaps onto a coaxial cylinder or
/// cone neighbour's and takes that neighbour's seam meridian (a shared ring
/// is then one parameter line on both faces), and it is turned so that its
/// tube angles stay clear of the template's tube seam at the outer equator.
#[allow(clippy::too_many_arguments)]
fn build_torus_surfaces(
    carriers: &HashMap<u32, TorusCarrier>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    region_samples: &HashMap<u32, Vec<Vec3>>,
    tolerance: f64,
) -> Result<HashMap<u32, TorusInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &original) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        if relevant.is_empty() || relevant.iter().any(|chain| !chain.closed) {
            return Err((
                region,
                format!("hybrid BREP: torus {region} is not bounded by closed rings"),
            ));
        }
        let mut carrier = original;
        let mut seam = None;
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let frame = cylinders
                .get(&neighbor)
                .map(|info| (info.carrier.origin, info.carrier.axis, info.x_axis))
                .or_else(|| {
                    cones
                        .get(&neighbor)
                        .map(|info| (info.carrier.apex, info.carrier.axis, info.x_axis))
                });
            if let Some((origin, axis, x_axis)) = frame {
                let axis = if axis.dot(carrier.axis) < 0.0 {
                    axis.scale(-1.0)
                } else {
                    axis
                };
                carrier.axis = axis;
                carrier.center = origin.add(axis.scale(carrier.center.sub(origin).dot(axis)));
                seam = Some(x_axis);
                break;
            }
        }
        // The band's tube angles: its samples and its ring points.
        let mut points = region_samples.get(&region).cloned().unwrap_or_default();
        for chain in &relevant {
            points.extend(chain_points(chain, vertices));
        }
        if points.is_empty() {
            return Err((region, format!("hybrid BREP: torus {region} has no samples")));
        }
        let mut frame = None;
        for sign in [1.0, -1.0] {
            let axis = carrier.axis.scale(sign);
            let (sin, cos) = points.iter().fold((0.0, 0.0), |(s, c), &point| {
                let raw =
                    torus_tube_angle(carrier.center, axis, carrier.major_radius, 0.0, point);
                (s + raw.sin(), c + raw.cos())
            });
            let middle = sin.atan2(cos).rem_euclid(TAU);
            let angular = tolerance / carrier.minor_radius;
            let clear = points.iter().all(|&point| {
                let angle =
                    torus_tube_angle(carrier.center, axis, carrier.major_radius, middle, point);
                angle >= -angular && angle <= TAU + angular
            });
            if clear {
                frame = Some((axis, middle));
                break;
            }
        }
        let Some((axis, band_middle)) = frame else {
            return Err((
                region,
                format!("hybrid BREP: torus {region} band crosses the tube seam either way"),
            ));
        };
        let x_axis = seam
            .map(|x| x.sub(axis.scale(x.dot(axis))))
            .unwrap_or_else(|| axis.perpendicular().unwrap_or_default())
            .normalized()
            .map_err(|error| (region, error))?;
        let y_axis = axis.cross(x_axis).normalized().map_err(|error| (region, error))?;
        let tube = make_arc(
            carrier.center.add(x_axis.scale(carrier.major_radius)),
            x_axis,
            axis,
            carrier.minor_radius,
            0.0,
            TAU,
        )
        .map_err(|error| (region, error))?;
        let surface = make_revolution(carrier.center, axis, &tube, TAU)
            .map_err(|error| (region, error))?;
        result.insert(
            region,
            TorusInfo {
                carrier,
                axis,
                x_axis,
                y_axis,
                band_middle,
                surface,
            },
        );
    }
    Ok(result)
}

fn build_cone_face(
    region: u32,
    cone: &ConeInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        let mut ring: Option<(brep_kernel::VertexId, [f64; 2], [f64; 2])> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: cone {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let mut pcurve = built
                .custom_revolve_pcurves
                .get(&region)
                .cloned()
                .unwrap_or(parameter_line(start, end)?);
            if !forward && built.custom_revolve_pcurves.contains_key(&region) {
                pcurve = pcurve.reversed()?;
            }
            ring = Some((arena.edges[built.edge].start, start, end));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        if cone.pointed {
            let Some((ring_vertex, ring_start, ring_end)) = ring.filter(|_| cycle.len() == 1) else {
                return Err(format!(
                    "hybrid BREP: pointed cone {region} boundary is not a single ring"
                ));
            };
            let v = ring_start[1];
            let u = ring_start[0].rem_euclid(1.0);
            // Seam from the apex up to the ring vertex, and the apex itself
            // as a degenerate edge along v = 0.
            let seam = trim_curve(&cone.surface.iso_curve_u(u)?, 0.0, v)?;
            let domain = seam.domain()?;
            let apex_point = cone.surface.evaluate(u, 0.0)?;
            let apex = arena.vertices.insert(ArenaVertex {
                wire_id: ids.next(),
                point: apex_point,
            });
            let seam_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: seam,
                t0: domain[0],
                t1: domain[1],
                start: apex,
                end: ring_vertex,
                degenerate: false,
                name: None,
            });
            let apex_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: make_line(apex_point, apex_point)?,
                t0: 0.0,
                t1: 1.0,
                start: apex,
                end: apex,
                degenerate: true,
                name: None,
            });
            // Down the seam on the ring's far side, across the apex, up the
            // seam on the ring's near side.
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: false,
                pcurve: parameter_line([ring_end[0], v], [ring_end[0], 0.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: apex_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], 0.0], [ring_start[0], 0.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: true,
                pcurve: parameter_line([ring_start[0], 0.0], [ring_start[0], v])?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: cone.surface.clone(),
        same_sense: cone.carrier.sense == 1,
        loops,
        name: None,
    }))
}

fn sort_region_cycles(
    boundaries: &mut [RegionBoundary],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    spheres: &HashMap<u32, SphereInfo>,
    tori: &HashMap<u32, TorusInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
) {
    for (region, boundary) in boundaries.iter_mut().enumerate() {
        let mut scored = boundary
            .cycles
            .drain(..)
            .map(|cycle| {
                let mut points = Vec::new();
                for traversal in &cycle {
                    let chain = &chains[traversal.chain];
                    let iter: Box<dyn Iterator<Item = &usize>> = if traversal.forward {
                        Box::new(chain.vertices.iter())
                    } else {
                        Box::new(chain.vertices.iter().rev())
                    };
                    for vertex in iter {
                        points.push(vertices[*vertex]);
                    }
                }
                let area = if let Some(plane) = planes.get(&(region as u32)) {
                    let normal = plane.normal.normalized().unwrap_or(plane.normal);
                    let x = normal.perpendicular().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
                    let y = normal.cross(x);
                    polygon_area(
                        &points
                            .iter()
                            .map(|p| {
                                let d = p.sub(plane.origin);
                                [d.dot(x), d.dot(y)]
                            })
                            .collect::<Vec<_>>(),
                    )
                } else if let Some(cylinder) = cylinders.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| cylinder_coordinates(cylinder, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(cone) = cones.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| cone_coordinates(cone, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(sphere) = spheres.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| sphere_coordinates(sphere, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(torus) = tori.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| torus_coordinates(torus, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else {
                    0.0
                };
                let same = planes.contains_key(&(region as u32))
                    || cylinders
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || cones
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || spheres
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || tori
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1);
                let score = if same { area } else { -area };
                (score, cycle)
            })
            .collect::<Vec<_>>();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0));
        boundary.cycles = scored.into_iter().map(|(_, cycle)| cycle).collect();
    }
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    (0..points.len())
        .map(|i| {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        * 0.5
}

