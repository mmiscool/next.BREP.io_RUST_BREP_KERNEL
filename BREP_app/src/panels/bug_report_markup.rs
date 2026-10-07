//! Screenshot annotations in source-image pixels. The same geometry paints the
//! live drag and rasterizes the PNG, so resizing the preview cannot move marks.
use eframe::egui;

#[derive(Clone, Copy, Default, PartialEq)]
pub(super) enum Tool {
    #[default]
    Pen,
    Arrow,
    Rectangle,
}

struct Mark {
    tool: Tool,
    points: Vec<egui::Pos2>,
    color: egui::Color32,
    width: f32,
}

impl Mark {
    fn segments(&self) -> Vec<[egui::Pos2; 2]> {
        let a = self.points[0];
        let b = *self.points.last().unwrap();
        match self.tool {
            Tool::Pen => {
                if self.points.len() == 1 {
                    vec![[a, a]]
                } else {
                    self.points.windows(2).map(|p| [p[0], p[1]]).collect()
                }
            }
            Tool::Rectangle => {
                let c = egui::pos2(b.x, a.y);
                let d = egui::pos2(a.x, b.y);
                vec![[a, c], [c, b], [b, d], [d, a]]
            }
            Tool::Arrow => {
                let direction = (b - a).normalized();
                let side = egui::vec2(-direction.y, direction.x);
                let length = (self.width * 5.0).min(a.distance(b) * 0.5);
                vec![
                    [a, b],
                    [b, b - direction * length + side * length * 0.5],
                    [b, b - direction * length - side * length * 0.5],
                ]
            }
        }
    }
}

pub(super) struct Markup {
    original: egui::ColorImage,
    marks: Vec<Mark>,
    active: Option<Mark>,
    tool: Tool,
    color: egui::Color32,
}

impl Markup {
    pub fn new(original: egui::ColorImage) -> Self {
        Self {
            original,
            marks: Vec::new(),
            active: None,
            tool: Tool::Pen,
            color: egui::Color32::RED,
        }
    }

    /// Returns a new flattened image only when a completed edit changes it.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        texture: &egui::TextureHandle,
        hits: &mut std::collections::HashMap<String, egui::Rect>,
    ) -> Option<egui::ColorImage> {
        let mut changed = false;
        ui.horizontal_wrapped(|ui| {
            for (tool, label) in [
                (Tool::Pen, "Pen"),
                (Tool::Arrow, "Arrow"),
                (Tool::Rectangle, "Rectangle"),
            ] {
                let response = ui.selectable_value(&mut self.tool, tool, label);
                hits.insert(
                    format!("markup:{}", label.to_lowercase()),
                    response.interact_rect,
                );
            }
            ui.label("Color:");
            ui.color_edit_button_srgba(&mut self.color);
            let undo = ui.add_enabled(!self.marks.is_empty(), egui::Button::new("Undo"));
            hits.insert("markup:undo".into(), undo.interact_rect);
            if undo.clicked() {
                self.marks.pop();
                changed = true;
            }
            let clear = ui.add_enabled(!self.marks.is_empty(), egui::Button::new("Clear"));
            hits.insert("markup:clear".into(), clear.interact_rect);
            if clear.clicked() {
                self.marks.clear();
                changed = true;
            }
        });
        ui.weak("Drag on the image to highlight the problem.");
        let size = texture.size_vec2();
        let scale = (ui.available_width() / size.x)
            .min(ui.available_height() / size.y)
            .min(1.0)
            .max(f32::EPSILON);
        let response = ui.add(
            egui::Image::new((texture.id(), size * scale))
                .sense(egui::Sense::click_and_drag())
                .bg_fill(egui::Color32::from_gray(20)),
        );
        let response = response.on_hover_cursor(egui::CursorIcon::Crosshair);
        hits.insert("markup:image".into(), response.interact_rect);
        {
            let point = response.interact_pointer_pos().map(|p| {
                let p = response.rect.clamp(p);
                let p = (p - response.rect.min) / scale;
                egui::pos2(p.x.min(size.x - 1.0), p.y.min(size.y - 1.0))
            });
            if response.drag_started() || response.clicked() {
                // Use the press origin, rather than where the drag threshold was
                // crossed, so arrow tails and the start of pen strokes are exact.
                let start = ui
                    .input(|i| i.pointer.press_origin())
                    .map(|p| {
                        let p = (response.rect.clamp(p) - response.rect.min) / scale;
                        egui::pos2(p.x.min(size.x - 1.0), p.y.min(size.y - 1.0))
                    })
                    .or(point);
                if let Some(start) = start {
                    self.active = Some(Mark {
                        tool: self.tool,
                        points: vec![start],
                        color: self.color,
                        width: 3.0 / scale,
                    });
                }
            }
            if let (Some(mark), Some(point)) = (&mut self.active, point) {
                if mark.tool != Tool::Pen && mark.points.len() > 1 {
                    mark.points[1] = point;
                } else if mark.points.last() != Some(&point) {
                    mark.points.push(point);
                }
            }
        }
        if response.drag_stopped() || response.clicked() {
            if let Some(mark) = self.active.take() {
                self.marks.push(mark);
                changed = true;
            }
        }
        // Draw newly completed edits immediately; the caller updates the
        // texture for the following frame. Live drags don't re-encode a PNG.
        if changed {
            ui.ctx().request_repaint();
            return Some(self.image());
        }
        if let Some(mark) = &self.active {
            let painter = ui
                .painter()
                .with_clip_rect(response.rect.intersect(ui.clip_rect()));
            for [a, b] in mark.segments() {
                painter.line_segment(
                    [
                        response.rect.min + a.to_vec2() * scale,
                        response.rect.min + b.to_vec2() * scale,
                    ],
                    egui::Stroke::new(mark.width * scale, mark.color),
                );
            }
        }
        None
    }

    fn image(&self) -> egui::ColorImage {
        let mut image = self.original.clone();
        for mark in &self.marks {
            for [a, b] in mark.segments() {
                paint_segment(&mut image, a, b, mark.width, mark.color);
            }
        }
        image
    }

    pub fn count(&self) -> usize {
        self.marks.len()
    }
}

/// Rasterize a round-ended stroke, blending antialiased edges into the source.
fn paint_segment(
    image: &mut egui::ColorImage,
    a: egui::Pos2,
    b: egui::Pos2,
    width: f32,
    color: egui::Color32,
) {
    let radius = width / 2.0;
    let min = a.min(b) - egui::Vec2::splat(radius + 1.0);
    let max = a.max(b) + egui::Vec2::splat(radius + 1.0);
    let delta = b - a;
    let length_sq = delta.length_sq();
    let [r, g, blue, alpha] = color.to_srgba_unmultiplied();
    for y in
        (min.y.floor().max(0.0) as usize)..=(max.y.ceil().max(0.0) as usize).min(image.size[1] - 1)
    {
        for x in (min.x.floor().max(0.0) as usize)
            ..=(max.x.ceil().max(0.0) as usize).min(image.size[0] - 1)
        {
            let p = egui::pos2(x as f32 + 0.5, y as f32 + 0.5);
            let t = if length_sq > 0.0 {
                ((p - a).dot(delta) / length_sq).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let coverage = (radius + 0.5 - p.distance(a + delta * t)).clamp(0.0, 1.0);
            let opacity = coverage * alpha as f32 / 255.0;
            if opacity > 0.0 {
                let pixel = &mut image.pixels[y * image.size[0] + x];
                let [br, bg, bb, ba] = pixel.to_srgba_unmultiplied();
                let back_alpha = ba as f32 / 255.0;
                let out_alpha = opacity + back_alpha * (1.0 - opacity);
                let blend = |front: u8, back: u8| {
                    ((front as f32 * opacity + back as f32 * back_alpha * (1.0 - opacity))
                        / out_alpha)
                        .round() as u8
                };
                *pixel = egui::Color32::from_rgba_unmultiplied(
                    blend(r, br),
                    blend(g, bg),
                    blend(blue, bb),
                    (out_alpha * 255.0).round() as u8,
                );
            }
        }
    }
}
