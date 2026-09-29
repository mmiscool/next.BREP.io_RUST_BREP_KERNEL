use super::*;

mod trim;
mod rim;
use rim::{rotate_rim_to_seam, complete_rim_seam};
mod biperiodic;
mod wrap_wavy;
mod band;
mod seam_wall;

use trim::seam_meridian_samples;

pub(super) use band::{close_periodic_band, close_periodic_spiral_annulus};
pub(super) use biperiodic::{close_biperiodic_seam_band, close_biperiodic_straddle_band};
pub(super) use seam_wall::{
    close_periodic_recut_wall, polygon_is_simple, seam_straddle_wall_polygons,
    unwrap_seam_crossing_trim,
};
pub(super) use trim::{close_periodic_trim, tessellate_full_biperiodic_face};
pub(super) use wrap_wavy::{close_periodic_wavy_band, close_periodic_wrap_wall};
