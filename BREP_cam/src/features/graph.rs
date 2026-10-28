//! The face-adjacency graph of a mesh snapshot: per face the outward normal,
//! Z range and area; per pair of faces the shared edge length split into
//! convex and concave parts, plus the shared edge polylines. Triangle edges
//! are matched by exact vertex coordinates (the watertight tessellation
//! shares them bit for bit).

use crate::mesh::MeshSnapshot;
use crate::model::Point3;
use crate::numeric::Tolerance;
use std::collections::BTreeMap;

/// Aggregate of one face's triangles.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceStats {
    pub face: u32,
    pub area: f64,
    /// Area-weighted mean outward normal, normalised (`None` for a folded face).
    pub normal: Option<Point3>,
    pub z_min: f64,
    pub z_max: f64,
    pub centroid: Point3,
    pub triangles: u64,
}

/// How two faces meet along their shared edges.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Adjacency {
    pub convex_length: f64,
    pub concave_length: f64,
    /// Shared edge segments (from the lower face index's side), unordered.
    pub segments: Vec<(Point3, Point3)>,
}

impl Adjacency {
    pub fn length(&self) -> f64 {
        self.convex_length + self.concave_length
    }
    /// Mostly concave (a wall rising from a floor, a pocket corner).
    pub fn concave(&self) -> bool {
        self.concave_length > self.convex_length
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FaceGraph {
    pub faces: Vec<FaceStats>,
    /// Keyed by `(lower face index, higher face index)`.
    pub adjacency: BTreeMap<(u32, u32), Adjacency>,
}

fn key(p: Point3) -> (u64, u64, u64) {
    (p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
}

impl FaceGraph {
    pub fn build(mesh: &MeshSnapshot, tol: &Tolerance) -> FaceGraph {
        let n = mesh.faces.len();
        let mut area = vec![0.0; n];
        let mut normal_sum = vec![Point3::ZERO; n];
        let mut z_min = vec![f64::INFINITY; n];
        let mut z_max = vec![f64::NEG_INFINITY; n];
        let mut centroid = vec![Point3::ZERO; n];
        let mut count = vec![0u64; n];
        // Directed edges with their triangle and its third vertex.
        let mut edges: BTreeMap<((u64, u64, u64), (u64, u64, u64)), (u32, u32, Point3)> = BTreeMap::new();
        for t in &mesh.triangles {
            let f = t.face as usize;
            let a = t.area();
            area[f] += a;
            normal_sum[f] = normal_sum[f].add(t.normal.scale(a));
            z_min[f] = z_min[f].min(t.min_z());
            z_max[f] = z_max[f].max(t.max_z());
            centroid[f] = centroid[f].add(t.centroid().scale(a));
            count[f] += 1;
            let v = t.vertices();
            for k in 0..3 {
                edges.insert((key(v[k]), key(v[(k + 1) % 3])), (t.face, t.id, v[(k + 2) % 3]));
            }
        }
        let faces = (0..n)
            .map(|f| FaceStats {
                face: f as u32,
                area: area[f],
                normal: normal_sum[f].normalized(),
                z_min: z_min[f],
                z_max: z_max[f],
                centroid: if area[f] > 0.0 { centroid[f].scale(1.0 / area[f]) } else { Point3::ZERO },
                triangles: count[f],
            })
            .collect();
        let mut adjacency: BTreeMap<(u32, u32), Adjacency> = BTreeMap::new();
        for (&(ka, kb), &(face_a, tri_a, _)) in &edges {
            // The opposite directed edge belongs to the neighbouring triangle.
            let Some(&(face_b, _tri_b, third_b)) = edges.get(&(kb, ka)) else { continue };
            if face_a == face_b || face_a > face_b {
                continue;
            }
            let ta = mesh.triangle(tri_a);
            let v = ta.vertices();
            let (pa, pb) = {
                let i = (0..3).find(|&i| key(v[i]) == ka).unwrap();
                (v[i], v[(i + 1) % 3])
            };
            let len = pa.distance(pb);
            // Convex when the neighbour's far vertex lies behind this triangle's plane.
            let side = ta.normal.dot(third_b.sub(pa));
            let entry = adjacency.entry((face_a, face_b)).or_default();
            if side < -tol.distance_eps {
                entry.convex_length += len;
            } else if side > tol.distance_eps {
                entry.concave_length += len;
            } else {
                // Coplanar neighbours (a face split across a seam): neither.
            }
            entry.segments.push((pa, pb));
        }
        FaceGraph { faces, adjacency }
    }

    /// Neighbours of `face` with their adjacency, ascending face index.
    pub fn neighbours(&self, face: u32) -> Vec<(u32, &Adjacency)> {
        self.adjacency
            .iter()
            .filter_map(|(&(a, b), adj)| if a == face { Some((b, adj)) } else if b == face { Some((a, adj)) } else { None })
            .collect()
    }

    pub fn adjacency(&self, a: u32, b: u32) -> Option<&Adjacency> {
        self.adjacency.get(&(a.min(b), a.max(b)))
    }
}
