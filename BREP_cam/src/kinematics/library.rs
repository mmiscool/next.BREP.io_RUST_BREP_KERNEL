//! The built-in machine library (lane C17c): typical machines as data —
//! kinematics, travel and rotary limits, a default post and collision solids
//! on the chain's links — that a setup picks by id. A custom machine starts
//! as a copy of one of these under a document id.
//!
//! Conventions: table pivots sit at the machine origin's X/Y (so a work
//! offset at the origin gives the same chain whether or not the caller
//! shifts pivots by it); tool-side solids start well above the tip (a
//! 100 mm gauge length), so a vertical tool at the top of a stock never
//! collides; part-side solids are the table or platter below z = 0.

use crate::model::{
    FeedLimits, Kinematics, Link, LinkShape, LinkSolid, Machine, MachineKind, MachineModel, Point3, RotaryAxis, SpindleLimits,
    TravelLimits,
};

fn axis(name: &str, direction: Point3, origin: Point3, limits: Option<[f64; 2]>) -> RotaryAxis {
    RotaryAxis { name: name.into(), direction, origin, limits_deg: limits }
}

fn cylinder(name: &str, link: Link, base: Point3, axis: Point3, radius: f64, height: f64) -> LinkSolid {
    LinkSolid { name: name.into(), link, shape: LinkShape::Cylinder { base, axis, radius, height } }
}

fn boxed(name: &str, link: Link, min: Point3, max: Point3) -> LinkSolid {
    LinkSolid { name: name.into(), link, shape: LinkShape::Box { min, max } }
}

/// The spindle nose and housing on the tool link: 100 mm above the tip.
fn spindle(radius: f64) -> Vec<LinkSolid> {
    vec![
        cylinder("spindle nose", Link::Tool, Point3::new(0.0, 0.0, 100.0), Point3::Z, 0.35 * radius, 40.0),
        cylinder("spindle housing", Link::Tool, Point3::new(0.0, 0.0, 140.0), Point3::Z, radius, 260.0),
    ]
}

fn machine(id: &str, name: &str, kind: MachineKind, post: &str, travel: (Point3, Point3), rpm: f64, collision: Vec<LinkSolid>) -> Machine {
    Machine {
        id: id.into(),
        name: name.into(),
        kind,
        travel: Some(TravelLimits { min: travel.0, max: travel.1 }),
        spindle: SpindleLimits { min_rpm: 0.0, max_rpm: rpm },
        feed: FeedLimits { max_cutting: 10_000.0, rapid: 30_000.0, acceleration: Some(2_000.0) },
        post: Some(post.into()),
        tool_change_seconds: 5.0,
        collision: MachineModel { solids: collision },
    }
}

/// Every built-in machine, in picker order.
pub fn builtin_machines() -> Vec<Machine> {
    let p = Point3::new;
    vec![
        machine(
            "vmc-3axis",
            "3-axis vertical machining centre",
            MachineKind::Mill3,
            "fanuc",
            (p(-400.0, -250.0, -300.0), p(400.0, 250.0, 300.0)),
            12_000.0,
            [spindle(70.0), vec![boxed("table", Link::Base, p(-450.0, -250.0, -80.0), p(450.0, 250.0, -40.0))]].concat(),
        ),
        machine(
            "vmc-4axis-a",
            "4-axis VMC with an A rotary table (about X)",
            MachineKind::Mill4 { rotary: axis("A", Point3::X, Point3::ZERO, None) },
            "haas",
            (p(-400.0, -250.0, -300.0), p(400.0, 250.0, 300.0)),
            12_000.0,
            [spindle(70.0), vec![cylinder("rotary faceplate", Link::Part, p(-160.0, 0.0, 0.0), Point3::X, 90.0, 30.0)]].concat(),
        ),
        machine(
            "trunnion-ac",
            "5-axis trunnion A/C (A tilts the C platter)",
            MachineKind::Mill5 {
                kinematics: Kinematics::TableTable {
                    primary: axis("A", Point3::X, Point3::new(0.0, 0.0, -50.0), Some([-120.0, 30.0])),
                    secondary: axis("C", Point3::Z, Point3::ZERO, None),
                },
            },
            "fanuc",
            (p(-350.0, -300.0, -350.0), p(350.0, 300.0, 300.0)),
            15_000.0,
            [spindle(65.0), vec![cylinder("C platter", Link::Part, p(0.0, 0.0, -45.0), Point3::Z, 110.0, 40.0)]].concat(),
        ),
        machine(
            "trunnion-bc",
            "5-axis trunnion B/C (B tilts the C platter)",
            MachineKind::Mill5 {
                kinematics: Kinematics::TableTable {
                    primary: axis("B", Point3::Y, Point3::new(0.0, 0.0, -50.0), Some([-110.0, 110.0])),
                    secondary: axis("C", Point3::Z, Point3::ZERO, None),
                },
            },
            "heidenhain",
            (p(-350.0, -300.0, -350.0), p(350.0, 300.0, 300.0)),
            15_000.0,
            [spindle(65.0), vec![cylinder("C platter", Link::Part, p(0.0, 0.0, -45.0), Point3::Z, 110.0, 40.0)]].concat(),
        ),
        machine(
            "head-head-bc",
            "5-axis head-head (C carries the B fork)",
            MachineKind::Mill5 {
                kinematics: Kinematics::HeadHead {
                    primary: axis("C", Point3::Z, Point3::new(0.0, 0.0, 250.0), Some([-360.0, 360.0])),
                    secondary: axis("B", Point3::Y, Point3::new(0.0, 0.0, 150.0), Some([-110.0, 110.0])),
                },
            },
            "siemens-840d",
            (p(-1000.0, -600.0, -400.0), p(1000.0, 600.0, 400.0)),
            18_000.0,
            [
                spindle(55.0),
                vec![
                    boxed("table", Link::Base, p(-1000.0, -600.0, -80.0), p(1000.0, 600.0, -40.0)),
                    cylinder("C housing", Link::Joint { index: 0 }, p(0.0, 0.0, 300.0), Point3::Z, 120.0, 200.0),
                ],
            ]
            .concat(),
        ),
        machine(
            "head-table-bc",
            "5-axis head-table (B head, C table)",
            MachineKind::Mill5 {
                kinematics: Kinematics::HeadTable {
                    head: axis("B", Point3::Y, Point3::new(0.0, 0.0, 150.0), Some([-110.0, 110.0])),
                    table: axis("C", Point3::Z, Point3::ZERO, None),
                },
            },
            "heidenhain",
            (p(-600.0, -400.0, -400.0), p(600.0, 400.0, 400.0)),
            18_000.0,
            [spindle(60.0), vec![cylinder("C table", Link::Part, p(0.0, 0.0, -50.0), Point3::Z, 250.0, 40.0)]].concat(),
        ),
        machine("lathe-2axis", "2-axis lathe (X/Z)", MachineKind::Lathe, "fanuc-lathe", (p(-150.0, 0.0, -400.0), p(150.0, 0.0, 50.0)), 4_000.0, Vec::new()),
        machine(
            "mill-turn-cy",
            "Mill-turn lathe with C and Y",
            MachineKind::MillTurn { c_axis: axis("C", Point3::Z, Point3::new(0.0, 0.0, -100.0), None), y_axis: true },
            "mazak",
            (p(-150.0, -60.0, -400.0), p(150.0, 60.0, 50.0)),
            5_000.0,
            [spindle(45.0), vec![cylinder("chuck", Link::Part, p(0.0, 0.0, -100.0), Point3::Z, 90.0, 60.0)]].concat(),
        ),
    ]
}

/// A built-in machine by id.
pub fn builtin(id: &str) -> Option<Machine> {
    builtin_machines().into_iter().find(|m| m.id.as_str() == id)
}

/// The built-in ids, in picker order.
pub fn builtin_ids() -> Vec<String> {
    builtin_machines().into_iter().map(|m| m.id.as_str().to_string()).collect()
}

