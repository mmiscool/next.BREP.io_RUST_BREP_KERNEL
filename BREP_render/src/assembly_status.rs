//! The ONE assembly-constraint status → label/color map. Both the Assembly
//! Constraints panel's row status labels AND the viewport overlay labels (lane G)
//! read THIS module — the status vocabulary and its colors are never re-declared
//! anywhere else (the UI-consistency directive).
//!
//! Colors are the captured palette: grey `#8e8e93` (disabled), green `#30d158`
//! (satisfied), yellow `#ffd60a` (adjusting / everything indeterminate — also
//! the default), red `#ff3b30` (blocked / duplicate / error / failed). Exposed
//! as plain sRGB byte triples so this stays egui-free (reusable by any painter).

/// The human label for a constraint `persistentData.status` word. Unknown /
/// empty statuses read as "Pending" (a constraint that has never run).
pub fn status_label(status: &str) -> &'static str {
    match status {
        "disabled" => "Disabled",
        "satisfied" => "Satisfied",
        "adjusted" | "adjusting" => "Adjusting",
        "blocked" => "Blocked",
        "pending" | "" => "Pending",
        "duplicate" => "Duplicate",
        "error" => "Error",
        "incomplete" => "Incomplete",
        "invalid-selection" => "Invalid selection",
        "unsupported-selection" | "unimplemented" => "Unimplemented",
        "pending-component" => "Pending component",
        "fixed" => "Locked",
        "noop" => "No change",
        "apply-failed" => "Failed",
        _ => "Pending",
    }
}

/// The status color as sRGB bytes. Grey for disabled, green for satisfied,
/// red for the error family (blocked / duplicate / error / apply-failed),
/// yellow (`#ffd60a`) for adjusting and every indeterminate status — the
/// captured default.
pub fn status_color_rgb(status: &str) -> [u8; 3] {
    match status {
        "disabled" => [0x8e, 0x8e, 0x93],
        "satisfied" => [0x30, 0xd1, 0x58],
        "blocked" | "duplicate" | "error" | "invalid-selection" | "apply-failed" => {
            [0xff, 0x3b, 0x30]
        }
        "unsupported-selection" | "unimplemented" => [0xff, 0x9f, 0x0a],
        _ => [0xff, 0xd6, 0x0a],
    }
}

/// The status color as a CSS hex string (`#rrggbb`) — the overlay/label lanes
/// that speak CSS-hex use this so the byte triple and the hex can never drift.
pub fn status_color_hex(status: &str) -> String {
    let [r, g, b] = status_color_rgb(status);
    format!("#{r:02x}{g:02x}{b:02x}")
}

/// Rank a status for the structure tree's per-component ROLLUP dot: the WORST
/// status across a component's constraints wins. Higher = worse. Order:
/// error family (3) > indeterminate/adjusting (2) > satisfied (1) >
/// disabled (0).
pub fn status_severity(status: &str) -> u8 {
    match status {
        "disabled" => 0,
        "satisfied" => 1,
        "blocked" | "duplicate" | "error" | "apply-failed" => 3,
        _ => 2,
    }
}

