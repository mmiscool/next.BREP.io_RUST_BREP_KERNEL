//! Setup sheet (lane C4, row 7): one HTML page per program, from the same
//! stream the post reads. Deterministic: no dates, no random ids.

use super::{Program, Units};
use crate::model::{Coolant, Stock, ToolKind};

fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

fn num(v: f64) -> String {
    let s = format!("{v:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

fn time(seconds: f64) -> String {
    let total = seconds.round().max(0.0) as u64;
    let (h, m, s) = (total / 3600, (total % 3600) / 60, total % 60);
    if h > 0 {
        format!("{h}h {m:02}m {s:02}s")
    } else {
        format!("{m}m {s:02}s")
    }
}

fn coolant(c: Coolant) -> &'static str {
    match c {
        Coolant::Off => "off",
        Coolant::Flood => "flood",
        Coolant::Mist => "mist",
        Coolant::ThroughTool => "through tool",
        Coolant::Air => "air",
        Coolant::FloodAndThroughTool => "flood + through tool",
        Coolant::FloodAndMist => "flood + mist",
    }
}

fn stock(s: &Stock, units: Units) -> String {
    let u = units.as_str();
    match s {
        Stock::BoundsOffset { x, y, top, bottom } => format!("model bounds + {} {u} (X), {} {u} (Y), {} {u} top, {} {u} bottom", num(*x), num(*y), num(*top), num(*bottom)),
        Stock::Box { min, max } => format!("box {} x {} x {} {u}", num(max.x - min.x), num(max.y - min.y), num(max.z - min.z)),
        Stock::Cylinder { radius, height, .. } => format!("cylinder d{} x {} {u}", num(2.0 * radius), num(*height)),
        Stock::FromSolid { solid } => format!("body {}", esc(solid)),
        Stock::FromRest { setup } => format!("rest material of setup {}", esc(setup.as_str())),
    }
}

/// Work offset name as controllers print it.
pub fn wcs_name(number: u32) -> String {
    match number {
        1..=6 => format!("G{}", 53 + number),
        n => format!("G54.1 P{}", n - 6),
    }
}

/// Render the setup sheet.
pub fn render(program: &Program) -> String {
    let u = program.units.as_str();
    let mut h = String::new();
    h.push_str("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n");
    h.push_str(&format!("<title>Setup sheet: {}</title>\n", esc(&program.name)));
    h.push_str("<style>\nbody{font-family:system-ui,sans-serif;margin:2em;color:#222}\nh1{font-size:1.6em}\nh2{font-size:1.2em;margin-top:1.5em;border-bottom:1px solid #ccc}\ntable{border-collapse:collapse;margin:.5em 0;font-size:.95em}\nth,td{border:1px solid #ccc;padding:.3em .6em;text-align:left;vertical-align:top}\nth{background:#f3f3f3}\ntd.n{text-align:right;font-variant-numeric:tabular-nums}\n.warn{color:#9a6700}\n.ok{color:#1a7f37}\n</style>\n</head>\n<body>\n");
    h.push_str(&format!("<h1>Setup sheet: {}</h1>\n", esc(&program.name)));
    h.push_str("<table>\n");
    if let Some(n) = program.number {
        h.push_str(&format!("<tr><th>Program number</th><td>O{n}</td></tr>\n"));
    }
    if !program.comment.is_empty() {
        h.push_str(&format!("<tr><th>Comment</th><td>{}</td></tr>\n", esc(&program.comment)));
    }
    h.push_str(&format!("<tr><th>Units</th><td>{u}</td></tr>\n"));
    if let Some(m) = &program.machine {
        h.push_str(&format!("<tr><th>Machine</th><td>{}</td></tr>\n", esc(&m.name)));
        if let Some(post) = &m.post {
            h.push_str(&format!("<tr><th>Post</th><td>{}</td></tr>\n", esc(post)));
        }
    }
    h.push_str(&format!("<tr><th>Operations</th><td>{}</td></tr>\n", program.sections.len()));
    h.push_str(&format!("<tr><th>Tool changes</th><td>{}</td></tr>\n", program.summary.tool_changes));
    h.push_str(&format!("<tr><th>Cutting length</th><td>{} {u}</td></tr>\n", num(program.summary.cutting_length)));
    h.push_str(&format!("<tr><th>Estimated time</th><td>{}</td></tr>\n", time(program.summary.estimated_seconds)));
    h.push_str("</table>\n");

    h.push_str("<h2>Setups</h2>\n<table>\n<tr><th>#</th><th>Name</th><th>WCS</th><th>Stock</th><th>Clearance</th><th>Retract</th><th>Feed height</th><th>Protected bodies</th></tr>\n");
    for (i, s) in program.setups.iter().enumerate() {
        h.push_str(&format!(
            "<tr><td class=\"n\">{}</td><td>{}</td><td>{}</td><td>{}</td><td class=\"n\">{}</td><td class=\"n\">{}</td><td class=\"n\">{}</td><td>{}</td></tr>\n",
            i + 1,
            esc(&s.name),
            wcs_name(s.wcs.number),
            stock(&s.stock, program.units),
            num(s.heights.clearance),
            num(s.heights.retract),
            num(s.heights.feed),
            esc(&s.protected.iter().map(|p| p.solid.clone()).collect::<Vec<_>>().join(", "))
        ));
    }
    h.push_str("</table>\n");

    h.push_str("<h2>Tools</h2>\n<table>\n<tr><th>T</th><th>Name</th><th>Type</th><th>Diameter</th><th>Corner radius</th><th>Flute length</th><th>Flutes</th><th>Holder</th><th>Spindle</th><th>Feed</th><th>Plunge</th><th>Coolant</th></tr>\n");
    for t in &program.tools {
        let kind = serde_json::to_value(t.kind).ok().and_then(|v| v.as_str().map(String::from)).unwrap_or_else(|| format!("{:?}", ToolKind::Flat));
        h.push_str(&format!(
            "<tr><td class=\"n\">T{}</td><td>{}</td><td>{}</td><td class=\"n\">{}</td><td class=\"n\">{}</td><td class=\"n\">{}</td><td class=\"n\">{}</td><td>{}</td><td class=\"n\">{} rpm</td><td class=\"n\">{} {u}/min</td><td class=\"n\">{} {u}/min</td><td>{}</td></tr>\n",
            t.number,
            esc(&t.name),
            kind,
            num(t.geometry.diameter),
            t.geometry.corner_radius.map(num).unwrap_or_default(),
            num(t.geometry.cutting_length),
            t.geometry.flute_count.map(|n| n.to_string()).unwrap_or_default(),
            esc(&t.holder.as_ref().map(|h| h.name.clone()).unwrap_or_default()),
            num(t.feeds.spindle_rpm),
            num(t.feeds.cutting_feed),
            num(t.feeds.plunge_feed),
            coolant(t.coolant)
        ));
    }
    h.push_str("</table>\n");

    h.push_str("<h2>Operations</h2>\n<table>\n<tr><th>#</th><th>Name</th><th>Strategy</th><th>Setup</th><th>WCS</th><th>Tool</th><th>Spindle</th><th>Coolant</th><th>Tolerance</th><th>Stock to leave</th><th>Cutting length</th><th>Time</th><th>Status</th></tr>\n");
    for s in &program.sections {
        let status = match s.report.status {
            crate::model::OperationStatus::Ok => "<span class=\"ok\">ok</span>".to_string(),
            crate::model::OperationStatus::Warning => format!(
                "<span class=\"warn\">{} warning{}</span>",
                s.report.warnings.len(),
                if s.report.warnings.len() == 1 { "" } else { "s" }
            ),
            other => format!("{other:?}").to_lowercase(),
        };
        h.push_str(&format!(
            "<tr><td class=\"n\">{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td class=\"n\">T{}</td><td class=\"n\">{} rpm</td><td>{}</td><td class=\"n\">{}</td><td class=\"n\">{} / {}</td><td class=\"n\">{} {u}</td><td class=\"n\">{}</td><td>{}</td></tr>\n",
            s.index + 1,
            esc(&s.name),
            esc(&s.strategy),
            esc(&s.setup_name),
            wcs_name(s.wcs.number),
            s.tool_number,
            num(s.spindle_rpm),
            coolant(s.coolant),
            num(s.tolerance),
            num(s.stock_to_leave.radial),
            num(s.stock_to_leave.axial),
            num(s.cutting_length),
            time(s.estimated_seconds),
            status
        ));
    }
    h.push_str("</table>\n");
    let warnings: Vec<String> = program
        .sections
        .iter()
        .flat_map(|s| s.report.warnings.iter().map(move |w| format!("<li><b>{}</b>: {} <code>{}</code></li>", esc(&s.name), esc(&w.message), esc(&w.code))))
        .collect();
    if !warnings.is_empty() {
        h.push_str("<h2>Warnings</h2>\n<ul>\n");
        for w in warnings {
            h.push_str(&w);
            h.push('\n');
        }
        h.push_str("</ul>\n");
    }
    h.push_str("</body>\n</html>\n");
    h
}
