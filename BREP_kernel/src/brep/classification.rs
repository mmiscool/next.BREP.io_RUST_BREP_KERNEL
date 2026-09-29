use crate::arrangement::Vec2;
use crate::spatial::{Aabb, Bvh};
use crate::topology::{BrepSolid, FaceRecord};
use crate::{intersect_curve_surface, make_line, project_point_to_surface, NurbsCurve, Vec3};
use serde::Serialize;

#[path = "classification/face_containment.rs"]
mod face_containment;
#[path = "classification/solid_classifier.rs"]
mod solid_classifier;

pub use face_containment::{
    containment_lane, horizon_cross_frame, parameter_point_in_face, parameter_point_in_face_scan,
    trim_sample_count, trim_station, ContainmentLane, PolygonClass, COVERING_RIM_STRIP_SWITCH,
    HORIZON_CONTAINMENT_SWITCH, HORIZON_CROSS_FRAME_SWITCH, HORIZON_SINGLE_LOOP_SWITCH,
    NO_SPHERE_CHART_TRIM_SWITCH, SEAM_BAND_MERGED_SWITCH,
};
pub(crate) use face_containment::forget_prepared_faces;
pub use solid_classifier::{classify_point, PointClass, PointClassification, SolidClassifier};
pub(crate) use solid_classifier::{classify_profile_begin, classify_profile_report, surface_uv_band};
