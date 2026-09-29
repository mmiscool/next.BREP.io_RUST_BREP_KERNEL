use super::*;

/// §5.8 CLOSED loft: the sections form a RING (the last flows back into the
/// first), producing a capless genus-1 solid. Every interpolation column runs
/// through `interpolate_curve_closed`, so the ring is C² across the closure —
/// not a welded seam. Topology per skin is the cylinder-wall rectangle turned
/// on its side: u runs along the section curve (open), v through the sections
/// (closed); the section-0 curves themselves serve as the doubled v-seam
/// edges and the corner rings (one closed curve through every section at each
/// section corner) are shared between adjacent skins.
pub fn loft_profile_brep_closed(
    input_sections: &[Vec<NurbsCurve>],
) -> Result<BrepSolid, KernelRefusal> {
    loft_profile_brep_closed_shifted(input_sections, 0)
}

/// The largest number of stations one interpolation column of a SHIFTED closed
/// loft may carry, laps included — the loft's own station cap, because the
/// column solve is dense and `O(stations³)`.
const MAX_SHIFTED_COLUMN_STATIONS: usize = 1024;

/// [`loft_profile_brep_closed`] for a ring whose sections come back SHIFTED:
/// after the last section the cycle continues into section 0 with curve `j`
/// flowing into curve `(j + shift) mod m`, not into curve `j`.
///
/// That is a twisted sweep's closure when the section is symmetric. A square
/// rolled a quarter turn over one lap lands each side on the NEXT side's place,
/// so the wall leaving side `j` must arrive on side `j + 1`. The faces stay ONE
/// PER SECTION CURVE, each spanning one lap: face `j` runs from curve `j` of
/// section 0 (its `v = 0` seam) to curve `j + shift` of section 0 (its `v = 1`
/// seam). Its corner rings run from vertex `j` to vertex `j + shift`. The seam
/// edges are still the section-0 curves, each used by one face at `v = 0` and one
/// at `v = 1`, so `V − E + F = m − 2m + m = 0`.
///
/// C² ACROSS THE SEAM, not welded. The curve indices form cycles of
/// `laps = m / gcd(m, shift)` under `j → j + shift`, and a control column's data
/// runs through all of them before it repeats. So each column is interpolated
/// PERIODICALLY over `laps × S` stations, at parameters `l + p_k` for lap `l`, and
/// split at the whole laps into one piece per face. Each piece is the lap's own
/// v-parameterization, `[0, 1]` on the same knots every face uses. At `shift = 0`
/// every cycle is one lap and this is exactly [`loft_profile_brep_closed`].
///
/// Curves `j` and `j + shift` of section 0 must share a representation (degree,
/// knots, weights). The seam edge at `v = 1` is the other curve, parameterized
/// as this face's own `u`.
pub(crate) fn loft_profile_brep_closed_shifted(
    input_sections: &[Vec<NurbsCurve>],
    shift: usize,
) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = 1e-6;
    let section_count = input_sections.len();
    if section_count < 4 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "section_count",
            "loftSolid: a closed loft needs at least 4 sections",
        ));
    }
    let sections = input_sections.to_vec();
    let curve_count = validate_sections(&sections, tolerance, "loftSolid", false)?;
    if shift >= curve_count {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "seam_shift",
            format!(
                "loftSolid: a closed loft's seam shift {shift} must be below its {curve_count} curves"
            ),
        ));
    }
    // The curve each one flows into across the seam.
    let landing = |curve_index: usize| (curve_index + shift) % curve_count;
    for curve_index in 0..curve_count {
        let (from, to) = (&sections[0][curve_index], &sections[0][landing(curve_index)]);
        if from.degree != to.degree
            || from.control_points.len() != to.control_points.len()
            || from.knots.len() != to.knots.len()
            || from.knots.iter().zip(&to.knots).any(|(a, b)| (a - b).abs() > 1e-9)
            || from
                .control_points
                .iter()
                .zip(&to.control_points)
                .any(|(a, b)| (a.w - b.w).abs() > 1e-12 * a.w.abs().max(1.0))
        {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "seam_compatibility",
                format!(
                    "loftSolid: curve {curve_index} flows across the seam into curve {}, and the two \
                     are not representation-compatible",
                    landing(curve_index)
                ),
            ));
        }
    }

    // Cyclic chord parameters over S stations plus the wrap span back to
    // station 0 — the closed analogue of the open loft's averaging.
    let mut accumulated = vec![0.0; section_count + 1];
    let mut columns = 0usize;
    for curve_index in 0..curve_count {
        for control_index in 0..sections[0][curve_index].control_points.len() {
            let mut chords = vec![0.0; section_count + 1];
            let mut total = 0.0;
            for station in 1..=section_count {
                let previous = sections[station - 1][curve_index].control_points[control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                let across = if station == section_count {
                    landing(curve_index)
                } else {
                    curve_index
                };
                let current = sections[station % section_count][across].control_points
                    [control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                total += current.sub(previous).length();
                chords[station] = total;
            }
            if total <= tolerance {
                continue;
            }
            for station in 0..=section_count {
                accumulated[station] += chords[station] / total;
            }
            columns += 1;
        }
    }
    if columns == 0 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "coincident_sections",
            "loftSolid: sections coincide",
        ));
    }
    let mut parameters = vec![0.0; section_count + 1];
    for station in 0..=section_count {
        parameters[station] = accumulated[station] / columns as f64;
    }
    parameters[0] = 0.0;
    parameters[section_count] = 1.0;
    if parameters.windows(2).any(|pair| pair[1] <= pair[0] + 1e-9) {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "section_order",
            "loftSolid: closed sections are not strictly ordered",
        ));
    }

    let mut skins = Vec::with_capacity(curve_count);
    if shift == 0 {
        for curve_index in 0..curve_count {
            let reference = &sections[0][curve_index];
            let mut grid = Vec::with_capacity(reference.control_points.len());
            let mut knots_v = Vec::new();
            for control_index in 0..reference.control_points.len() {
                let points = sections
                    .iter()
                    .map(|section| section[curve_index].control_points[control_index].point())
                    .collect::<Result<Vec<_>, String>>()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                let interpolated = crate::interpolate_curve_closed(&points, &parameters)
                    .or_refuse(KernelStage::Refine, "interpolate_closed")?;
                knots_v = interpolated.knots.clone();
                let weight = reference.control_points[control_index].w;
                grid.push(
                    interpolated
                        .control_points
                        .iter()
                        .map(|point| Vec4::from_point(point.point().unwrap(), weight))
                        .collect(),
                );
            }
            skins.push(NurbsSurface::new(
                reference.degree,
                3,
                reference.knots.clone(),
                knots_v,
                grid,
            )
            .or_refuse(KernelStage::Fragment, "skin")?);
        }
    } else {
        skins = shifted_skins(&sections, &parameters, curve_count, shift)?;
    }

    // Topology: c corner vertices (section-0 corners), c corner-ring edges
    // (closed v-curves at each corner), c seam edges (the section-0 curves),
    // c skin faces. V − E + F = c − 2c + c = 0 → genus 1.
    let base_points = closed_points(&sections[0], tolerance)?;
    let mut vertices = Vec::with_capacity(curve_count);
    for (index, point) in base_points.iter().enumerate() {
        vertices.push(VertexRecord {
            id: index as u64 + 1,
            point: *point,
        });
    }
    let mut edges = Vec::with_capacity(2 * curve_count);
    let mut ring_edge_ids = Vec::with_capacity(curve_count);
    let mut seam_edge_ids = Vec::with_capacity(curve_count);
    for index in 0..curve_count {
        let [u_start, u_end] = sections[0][index]
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        let _ = u_end;
        let ring = skins[index]
            .iso_curve_u(u_start)
            .or_refuse(KernelStage::Fragment, "iso_curve")?;
        let [ring_start, ring_end] = ring
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        let ring_id = 100 + index as u64;
        ring_edge_ids.push(ring_id);
        edges.push(EdgeRecord {
            id: ring_id,
            curve: ring,
            t0: ring_start,
            t1: ring_end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: landing(index) as u64 + 1,
            degenerate: false,
            name: None,
        });
        let seam = sections[0][index].clone();
        let [seam_start, seam_end] = seam
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        let seam_id = 100 + curve_count as u64 + index as u64;
        seam_edge_ids.push(seam_id);
        edges.push(EdgeRecord {
            id: seam_id,
            curve: seam,
            t0: seam_start,
            t1: seam_end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: ((index + 1) % curve_count) as u64 + 1,
            degenerate: false,
            name: None,
        });
    }

    let mut next_id = 1000u64;
    let mut faces = Vec::with_capacity(curve_count);
    for index in 0..curve_count {
        let [u_start, u_end] = sections[0][index]
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        // Cylinder-wall rectangle: seam at v=0 forward, next corner ring up,
        // seam at v=1 backward, own corner ring down. On a SHIFTED ring the seam
        // at v=1 is the curve this one lands on.
        let coedges = vec![
            CoedgeRecord {
                id: next_id,
                edge_id: seam_edge_ids[index],
                forward: true,
                pcurve: parameter_line(u_start, 0.0, u_end, 0.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 1,
                edge_id: ring_edge_ids[(index + 1) % curve_count],
                forward: true,
                pcurve: parameter_line(u_end, 0.0, u_end, 1.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 2,
                edge_id: seam_edge_ids[landing(index)],
                forward: false,
                pcurve: parameter_line(u_end, 1.0, u_start, 1.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 3,
                edge_id: ring_edge_ids[index],
                forward: false,
                pcurve: parameter_line(u_start, 1.0, u_start, 0.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
        ];
        next_id += 4;
        let loop_id = next_id;
        next_id += 1;
        let face_id = next_id;
        next_id += 1;
        faces.push(FaceRecord {
            id: face_id,
            surface: skins[index].clone(),
            same_sense: true,
            loops: vec![LoopRecord {
                id: loop_id,
                coedges,
            }],
            name: None,
        });
    }

    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: next_id + 1,
            faces,
        }],
        genus: 1,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!(
                "loftSolid: closed loft failed validation: {issues:?}"
            ),
        ));
    }
    // The ring's outward side depends on the sections' winding; the signed
    // volume is the arbiter.
    if crate::solid_signed_volume(&solid).or_refuse(KernelStage::Validate, "signed_volume")? < 0.0
    {
        crate::offset_shell::flip_all_faces(&mut solid)
            ?;
    }
    // A ring carried round a path too tight for its own profile closes through
    // itself, and neither the validation above nor the signed volume can see
    // it: the enclosed space still measures. Same acceptance as the open loft.
    crate::accept_sound(solid, "loftSolid")
}

/// The skins of a SHIFTED closed loft ([`loft_profile_brep_closed_shifted`]): each
/// control column interpolated periodically over every lap of its curve cycle,
/// then split at the whole laps, one piece per face.
fn shifted_skins(
    sections: &[Vec<NurbsCurve>],
    parameters: &[f64],
    curve_count: usize,
    shift: usize,
) -> Result<Vec<NurbsSurface>, KernelRefusal> {
    let section_count = sections.len();
    let gcd = |mut a: usize, mut b: usize| {
        while b != 0 {
            (a, b) = (b, a % b);
        }
        a
    };
    let laps = curve_count / gcd(curve_count, shift);
    if laps * section_count > MAX_SHIFTED_COLUMN_STATIONS {
        // Refused before any solve runs — a size the lane declines by name, not
        // a budget that ran out, so it is a deferral rather than a non-convergence.
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "station_cap",
            format!(
                "loftSolid: a ring whose curves land {shift} along after one lap runs each skin \
                 through {laps} laps of {section_count} stations, {} in all, past the \
                 {MAX_SHIFTED_COLUMN_STATIONS}-station cap the loft's dense column solve can carry",
                laps * section_count
            ),
        ));
    }
    // The knots every face shares in v: one lap's periodic interpolation, clamped.
    let mut lap_knots = vec![parameters[0]; 4];
    lap_knots.extend_from_slice(&parameters[1..section_count]);
    lap_knots.extend(std::iter::repeat_n(parameters[section_count], 4));
    let mut lap_parameters = Vec::with_capacity(laps * section_count + 1);
    for lap in 0..laps {
        for station in 0..section_count {
            lap_parameters.push(lap as f64 + parameters[station]);
        }
    }
    lap_parameters.push(laps as f64);

    let mut grids: Vec<Vec<Vec<Vec4>>> = sections[0]
        .iter()
        .map(|curve| vec![Vec::new(); curve.control_points.len()])
        .collect();
    let mut visited = vec![false; curve_count];
    for start in 0..curve_count {
        if visited[start] {
            continue;
        }
        let cycle: Vec<usize> = (0..laps).map(|lap| (start + lap * shift) % curve_count).collect();
        for &curve_index in &cycle {
            visited[curve_index] = true;
        }
        for control_index in 0..sections[0][start].control_points.len() {
            let mut points = Vec::with_capacity(laps * section_count);
            for &curve_index in &cycle {
                for section in sections {
                    points.push(
                        section[curve_index].control_points[control_index]
                            .point()
                            .or_refuse(KernelStage::Refine, "control_point")?,
                    );
                }
            }
            let mut rest = crate::interpolate_curve_closed(&points, &lap_parameters)
                .or_refuse(KernelStage::Refine, "interpolate_closed")?;
            for (lap, &curve_index) in cycle.iter().enumerate() {
                let piece = if lap + 1 < laps {
                    let (piece, remainder) = rest
                        .split((lap + 1) as f64)
                        .or_refuse(KernelStage::Refine, "split")?;
                    rest = remainder;
                    piece
                } else {
                    rest.clone()
                };
                if piece.knots.len() != lap_knots.len()
                    || piece
                        .knots
                        .iter()
                        .zip(&lap_knots)
                        .any(|(knot, own)| (knot - lap as f64 - own).abs() > 1e-9)
                {
                    return Err(KernelRefusal::internal(
                        KernelStage::Refine,
                        "lap_split",
                        format!(
                            "loftSolid: lap {lap} of a shifted ring's column did not split onto the \
                             lap's own knots"
                        ),
                    ));
                }
                let weight = sections[0][curve_index].control_points[control_index].w;
                grids[curve_index][control_index] = piece
                    .control_points
                    .iter()
                    .map(|point| Ok(Vec4::from_point(point.point()?, weight)))
                    .collect::<Result<Vec<_>, String>>()
                    .or_refuse(KernelStage::Refine, "control_point")?;
            }
        }
    }
    sections[0]
        .iter()
        .zip(grids)
        .map(|(reference, grid)| {
            NurbsSurface::new(reference.degree, 3, reference.knots.clone(), lap_knots.clone(), grid)
        })
        .collect::<Result<Vec<_>, String>>()
        .or_refuse(KernelStage::Fragment, "skin")
}
