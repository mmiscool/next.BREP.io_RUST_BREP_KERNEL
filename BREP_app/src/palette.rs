//! A generic, reusable searchable **command-palette** modal — engine-agnostic.
//!
//! The caller drives it with a flat list of [`PaletteItem`]s (each an opaque
//! `id`, a displayed `label`, and extra `keywords` to match on) and reads back
//! the chosen `id`. The dialog itself knows NOTHING about features, the engine,
//! or the scene — so every "pick one of N named things" call site (add-feature
//! here; later: insert-datum, apply-appearance, jump-to-solid, …) reuses it
//! verbatim.
//!
//! # Public API shape — `show(...) -> Option<String>`
//!
//! The user asked for "a callback executed upon selection". In egui's immediate
//! mode a stored `Box<dyn FnMut(&str)>` would have to be invoked from inside the
//! per-frame draw while the caller ALSO holds `&mut EngineState` (to act on the
//! pick) — the closure would need to capture that same `&mut`, which the borrow
//! checker rejects. So the idiomatic form is inverted: [`Palette::show`] RETURNS
//! `Some(id)` on the frame an item is chosen (and closes itself), and the caller
//! acts on the id with its own `&mut EngineState` right there. Same effect as a
//! callback, no borrow fight, and the palette stays free of caller state.
//!
//! # Behaviour
//!
//! * A centred modal [`egui::Modal`] (backdrop dims + blocks the rest of the UI).
//! * A single-line text input **focused by default**, then scrollable results.
//! * A display dropdown: large icons (default), medium or small icons, or
//!   compact lists in one or multiple columns. The caller persists the choice.
//! * The list is **alphabetical by label** and **filtered live** (case-insensitive
//!   substring, with a light subsequence fuzzy fallback) against `label` +
//!   `keywords` — see [`filter_items`] (pure + unit-tested).
//! * **Click** an item → selects it. **Enter** → selects the current TOP of the
//!   filtered list. **Esc** / backdrop click → cancels.

use crate::automation::hit_keys::HitKeyDoc;
use eframe::egui;
use std::collections::HashMap;

/// One selectable entry. `id` is the opaque token returned on selection; `label`
/// is shown (and is the alphabetical sort key); `keywords` are extra strings the
/// query also matches against (short codes / aliases) but which aren't displayed.
#[derive(Clone, Debug)]
pub struct PaletteItem {
    pub id: String,
    pub label: String,
    pub keywords: Vec<String>,
}

impl PaletteItem {
    pub fn new(id: impl Into<String>, label: impl Into<String>, keywords: Vec<String>) -> Self {
        Self {
            id: id.into(),
            label: label.into(),
            keywords,
        }
    }

    /// The row text as shown. For features the label already carries the leading
    /// glyph (prepended by `feature_long_name`); the palette sorts/searches on a
    /// glyph-stripped key ([`sort_key`]) so ordering stays alphabetical by name.
    fn display(&self) -> String {
        self.label.clone()
    }
}

/// Application-wide presentation preference; independent of model contents.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PaletteDisplay {
    CompactSingle,
    CompactMulti,
    #[default]
    LargeIcons,
    MediumIcons,
    SmallIcons,
}

impl PaletteDisplay {
    const ALL: [Self; 5] = [
        Self::CompactSingle,
        Self::CompactMulti,
        Self::LargeIcons,
        Self::MediumIcons,
        Self::SmallIcons,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::CompactSingle => "Compact list — 1 column",
            Self::CompactMulti => "Compact list — multi column",
            Self::LargeIcons => "Icons large",
            Self::MediumIcons => "Icons medium",
            Self::SmallIcons => "Icons small",
        }
    }

    fn columns(self, width: f32, gap: f32) -> usize {
        let minimum = match self {
            Self::CompactSingle => return 1,
            Self::CompactMulti => 230.0,
            Self::LargeIcons => 140.0,
            Self::MediumIcons => 110.0,
            Self::SmallIcons => 88.0,
        };
        ((width + gap) / (minimum + gap)).floor().max(1.0) as usize
    }

    /// The tile geometry of an icon layout — `None` for the compact lists,
    /// which draw ordinary rows instead of tiles. This is the ONE place the
    /// three sizes are listed, so a new size is a single row here plus its
    /// `label`/`columns` entries.
    fn tile(self) -> Option<TileMetrics> {
        let (height, side, caption_top) = match self {
            Self::CompactSingle | Self::CompactMulti => return None,
            Self::LargeIcons => (120.0, 56.0, 76.0),
            Self::MediumIcons => (96.0, 36.0, 54.0),
            Self::SmallIcons => (80.0, 24.0, 40.0),
        };
        Some(TileMetrics {
            height,
            side,
            caption_top,
        })
    }
}

/// One icon layout's tile geometry: the tile's height, the longest side of its
/// artwork, and the top edge its caption is painted at — all measured from the
/// tile's top. The caption sits at a FIXED offset rather than being laid out
/// under the artwork, so these three must agree: `icon_tiles_fit_their_captions`
/// checks that the artwork clears the caption and that two caption rows still
/// fit inside the tile.
#[derive(Clone, Copy, Debug)]
struct TileMetrics {
    height: f32,
    side: f32,
    caption_top: f32,
}

/// The gap between a tile's top edge and its artwork, in points — shared by
/// every icon size, so [`TileMetrics::caption_top`] can be checked against it.
const ICON_TOP: f32 = 10.0;

/// Draw a fixed-size tile with catalogue artwork above its wrapping caption.
fn icon_tile(
    ui: &mut egui::Ui,
    label: &str,
    top: bool,
    width: f32,
    tile: TileMetrics,
) -> egui::Response {
    let response = ui.add_sized([width, tile.height], egui::Button::new("").selected(top));
    let (icon, caption) = crate::icon_text::split_caption(label);
    if ui.is_rect_visible(response.rect) {
        let color = ui.style().interact_selectable(&response, top).text_color();
        if let Some(icon) = icon {
            egui_extras::install_image_loaders(ui.ctx());
            let side = tile.side;
            let art_height = side.min((width - 16.0) / icon.artwork_aspect.max(1.0));
            let rect = egui::Rect::from_center_size(
                egui::pos2(
                    response.rect.center().x,
                    response.rect.top() + ICON_TOP + side / 2.0,
                ),
                egui::vec2(art_height * icon.artwork_aspect, art_height),
            );
            let mut art = egui::Image::new(egui::ImageSource::Bytes {
                uri: format!("{}-artwork.svg", icon.uri).into(),
                bytes: egui::load::Bytes::Static(icon.artwork_svg.as_bytes()),
            })
            .fit_to_exact_size(rect.size());
            if icon.mono {
                art = art.tint(color);
            }
            art.paint_at(ui, rect);
        }
        let font = egui::TextStyle::Button.resolve(ui.style());
        let mut job = egui::text::LayoutJob::simple(caption.to_owned(), font, color, width - 12.0);
        job.halign = egui::Align::Center;
        job.wrap.max_rows = 2;
        let galley = ui.fonts_mut(|fonts| fonts.layout_job(job));
        ui.painter().galley(
            egui::pos2(
                response.rect.center().x,
                response.rect.top() + tile.caption_top,
            ),
            galley,
            color,
        );
    }
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), caption)
    });
    response.on_hover_text(caption)
}

/// The palette's transient state. The caller owns ONE, calls [`Palette::open`]
/// to populate + show it, and [`Palette::show`] every frame; the return of
/// `show` is the selection outcome.
#[derive(Default)]
pub struct Palette {
    open: bool,
    pub display: PaletteDisplay,
    display_changed: bool,
    /// The live query text (bound to the text input).
    query: String,
    /// Heading text (empty → no heading).
    title: String,
    /// Text-input hint.
    placeholder: String,
    /// The items, pre-sorted alphabetical by label at [`open`](Palette::open).
    items: Vec<PaletteItem>,
    /// Set on open (and after a no-op Enter) so the text input grabs focus.
    want_focus: bool,
    /// Per-frame widget rects (text input + visible rows) for the headed
    /// verifier. Keys include `"input"`, `"display"`, `"top"`, and `"item:<id>"`.
    /// Rebuilt each show; visible dropdown options use `"display:<mode>"`.
    hits: HashMap<String, egui::Rect>,
}

impl Palette {
    pub fn new() -> Self {
        Self::default()
    }

    /// Populate + open the palette. Items are sorted alphabetical by label; the
    /// query is reset and the text input will focus on the next frame.
    pub fn open(
        &mut self,
        mut items: Vec<PaletteItem>,
        title: impl Into<String>,
        placeholder: impl Into<String>,
    ) {
        sort_by_label(&mut items);
        self.items = items;
        self.title = title.into();
        self.placeholder = placeholder.into();
        self.query.clear();
        self.open = true;
        self.want_focus = true;
    }

    pub fn take_display_change(&mut self) -> bool {
        std::mem::take(&mut self.display_changed)
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    pub fn close(&mut self) {
        self.open = false;
        self.query.clear();
        self.items.clear();
        self.hits.clear();
    }

    /// The per-frame widget rects (text input + visible rows) for the headed
    /// verifier — empty when the palette is closed. They are SCREEN points: the
    /// palette is a centred [`egui::Modal`] over the whole window, which is why
    /// the shell publishes them as their own panel rather than folding them
    /// into the pane whose button opened it (see [`HIT_KEYS`]).
    pub fn hits(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    /// Draw the palette (if open) and return the chosen item id on the frame a
    /// selection is made (the palette closes itself). `None` while it stays open,
    /// while nothing is chosen, and on cancel (Esc / backdrop click, which also
    /// close it). Idempotent when closed.
    pub fn show(&mut self, ctx: &egui::Context) -> Option<String> {
        if !self.open {
            return None;
        }
        self.hits.clear();

        let mut selected: Option<String> = None;
        let mut enter_no_match = false;

        let modal = egui::Modal::new(egui::Id::new("brep-command-palette")).show(ctx, |ui| {
            let desired_width: f32 = if self.display == PaletteDisplay::CompactSingle {
                360.0
            } else {
                620.0
            };
            ui.set_width(desired_width.min((ctx.content_rect().width() - 40.0).max(160.0)));

            if !self.title.is_empty() {
                ui.heading(&self.title);
                ui.add_space(4.0);
            }

            ui.horizontal(|ui| {
                ui.label("Display");
                let combo = egui::ComboBox::from_id_salt("palette-display")
                    .selected_text(self.display.label())
                    .show_ui(ui, |ui| {
                        for mode in PaletteDisplay::ALL {
                            let option = ui.selectable_value(&mut self.display, mode, mode.label());
                            self.hits.insert(format!("display:{mode:?}"), option.rect);
                            if option.changed() {
                                self.display_changed = true;
                                self.want_focus = true;
                            }
                        }
                    });
                self.hits.insert("display".into(), combo.response.rect);
            });
            ui.add_space(4.0);

            // --- search input (focused by default) ----------------------------
            let input = ui.add(
                egui::TextEdit::singleline(&mut self.query)
                    .hint_text(&self.placeholder)
                    .desired_width(f32::INFINITY),
            );
            self.hits.insert("input".into(), input.rect);
            if self.want_focus {
                input.request_focus();
                self.want_focus = false;
            }
            // While focused, keep Esc for the modal (its `should_close` consumes
            // it to cancel) instead of letting egui surrender focus — which would
            // otherwise let the app's global Esc handler eat it a frame early.
            if input.has_focus() {
                ui.memory_mut(|m| {
                    m.set_focus_lock_filter(
                        input.id,
                        egui::EventFilter {
                            escape: true,
                            horizontal_arrows: true,
                            ..Default::default()
                        },
                    )
                });
            }
            // Enter commits the current TOP of the filtered list.
            let enter = input.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));

            ui.add_space(4.0);
            ui.separator();

            // --- filtered, ranked list ---------------------------------------
            let filtered = filter_items(&self.items, &self.query);
            if enter {
                match filtered.first() {
                    Some(top) => selected = Some(top.id.clone()),
                    None => enter_no_match = true,
                }
            }

            // The list keeps its full width but SHRINKS to the height its rows
            // actually need, so the modal is exactly as tall as the icons it
            // shows. Its ceiling is what the screen has left under the chrome
            // drawn above (title, display row, search field) — measured from
            // `min_rect`, not guessed, and independent of where the centred
            // modal ends up, so there is no size/position feedback. Only when
            // the rows outgrow that ceiling does a scroll bar appear.
            let chrome = ui.min_rect().height();
            let max_height = (ctx.content_rect().height() - chrome - 48.0).max(120.0);
            // A modal's ui is only as tall as the modal was LAST frame (an
            // `Area`'s max_rect is its own previous rect), so a list that sized
            // itself from `available_height` could never grow past the dialog it
            // is already inside. Hand it a child ui whose max rect IS the
            // screen-derived budget: the list shrinks to its rows within that,
            // and the modal follows the list.
            let budget = egui::Rect::from_min_size(
                ui.cursor().min,
                egui::vec2(ui.available_width(), max_height),
            );
            ui.scope_builder(egui::UiBuilder::new().max_rect(budget), |ui| {
                egui::ScrollArea::vertical()
                    .max_height(max_height)
                    .auto_shrink([false, true])
                    .show(ui, |ui| {
                        if filtered.is_empty() {
                            ui.weak("No matches");
                        }
                        let gap = ui.spacing().item_spacing.x;
                        let columns = self.display.columns(ui.available_width(), gap);
                        let width = ((ui.available_width() - gap * (columns - 1) as f32)
                            / columns as f32)
                            .max(1.0);
                        for (row_index, items) in filtered.chunks(columns).enumerate() {
                            ui.horizontal(|ui| {
                                for (column, item) in items.iter().enumerate() {
                                    let is_top = row_index == 0 && column == 0;
                                    let row = ui
                                        .push_id(&item.id, |ui| match self.display.tile() {
                                            // Every icon size draws the same tile;
                                            // only its geometry differs.
                                            Some(tile) => {
                                                icon_tile(ui, &item.label, is_top, width, tile)
                                            }
                                            None if self.display
                                                == PaletteDisplay::CompactSingle =>
                                            {
                                                crate::icon_text::selectable_icon_label(
                                                    ui,
                                                    is_top,
                                                    &item.display(),
                                                )
                                            }
                                            // CompactMulti.
                                            None => {
                                                let (icon, caption) =
                                                    crate::icon_text::split_caption(&item.label);
                                                let button = if let Some(icon) = icon {
                                                    egui_extras::install_image_loaders(ui.ctx());
                                                    egui::Button::selectable(
                                                        is_top,
                                                        (
                                                            crate::icon_text::image(
                                                                icon,
                                                                ui.text_style_height(
                                                                    &egui::TextStyle::Body,
                                                                ),
                                                            ),
                                                            caption,
                                                        ),
                                                    )
                                                    .image_tint_follows_text_color(icon.mono)
                                                } else {
                                                    egui::Button::selectable(is_top, caption)
                                                };
                                                ui.add_sized([width, 24.0], button.truncate())
                                                    .on_hover_text(caption)
                                            }
                                        })
                                        .inner;
                                    if ui.is_rect_visible(row.rect) {
                                        let visible = row.rect.intersect(ui.clip_rect());
                                        self.hits.insert(format!("item:{}", item.id), visible);
                                        if is_top {
                                            self.hits.insert("top".into(), visible);
                                        }
                                    }
                                    if row.clicked() {
                                        selected = Some(item.id.clone());
                                    }
                                }
                            });
                        }
                    });
            });
        });

        // A pick closes + reports; a cancel (Esc / backdrop) just closes.
        if let Some(id) = selected {
            self.close();
            return Some(id);
        }
        if modal.should_close() {
            self.close();
        } else if enter_no_match {
            // Enter with an empty result surrendered the input's focus; grab it
            // back so the user can keep typing.
            self.want_focus = true;
        }
        None
    }
}

/// Sort/tie-break key: the label lower-cased with any leading non-alphanumeric
/// glyph (a feature icon prepended by `feature_long_name`) and spaces stripped,
/// so the resting order stays alphabetical by NAME despite an icon prefix. A
/// glyph-free label is unaffected.
fn sort_key(label: &str) -> String {
    label
        .trim_start_matches(|c: char| !c.is_ascii_alphanumeric())
        .to_lowercase()
}

/// Sort items alphabetical (case-insensitive) by label — the resting list order.
pub(crate) fn sort_by_label(items: &mut [PaletteItem]) {
    items.sort_by(|a, b| sort_key(&a.label).cmp(&sort_key(&b.label)));
}

/// Filter + rank `items` against `query` (case-insensitive). Returns references
/// to the matching items, best match first, ties broken alphabetically by label.
/// An empty query returns ALL items in their existing (alphabetical) order.
///
/// Matching is substring-first (an earlier, longer hit ranks higher) with a
/// light subsequence fuzzy fallback, evaluated against the label (slightly
/// preferred) and every keyword.
pub fn filter_items<'a>(items: &'a [PaletteItem], query: &str) -> Vec<&'a PaletteItem> {
    let needle = query.trim().to_lowercase();
    if needle.is_empty() {
        return items.iter().collect();
    }
    let mut scored: Vec<(i32, &PaletteItem)> = items
        .iter()
        .filter_map(|item| item_score(item, &needle).map(|s| (s, item)))
        .collect();
    // Higher score first; alphabetical by label on ties (stable, predictable top).
    scored.sort_by(|a, b| {
        b.0.cmp(&a.0)
            .then_with(|| sort_key(&a.1.label).cmp(&sort_key(&b.1.label)))
    });
    scored.into_iter().map(|(_, item)| item).collect()
}

/// Best match score of an item against a (lowercased) needle, or `None` if it
/// matches neither the label nor any keyword. The label carries a small bonus so
/// a label hit outranks an equal keyword hit.
fn item_score(item: &PaletteItem, needle: &str) -> Option<i32> {
    const LABEL_BONUS: i32 = 10;
    let mut best: Option<i32> = None;
    let mut consider = |s: Option<i32>| {
        if let Some(s) = s {
            best = Some(best.map_or(s, |b| b.max(s)));
        }
    };
    consider(fuzzy_score(&item.label, needle).map(|s| s + LABEL_BONUS));
    for kw in &item.keywords {
        consider(fuzzy_score(kw, needle));
    }
    best
}

/// Score one `haystack` against a lowercased `needle`: a substring hit scores
/// `1000 - start` (earlier is better); otherwise a subsequence hit scores
/// `400 - last_index` (tighter is better); no match → `None`.
fn fuzzy_score(haystack: &str, needle: &str) -> Option<i32> {
    if needle.is_empty() {
        return Some(0);
    }
    let hay = haystack.to_lowercase();
    if let Some(pos) = hay.find(needle) {
        return Some(1000 - pos as i32);
    }
    // Subsequence fallback: every needle char appears in order.
    let mut chars = needle.chars().peekable();
    let mut last = 0i32;
    for (i, hc) in hay.chars().enumerate() {
        match chars.peek() {
            Some(&nc) if hc == nc => {
                chars.next();
                last = i as i32;
            }
            Some(_) => {}
            None => break,
        }
    }
    chars.peek().is_none().then_some(400 - last)
}

/// The hit keys this modal publishes (see `automation::hit_keys`). It is its
/// OWN panel: a centred modal is not a row of the pane whose button opened it,
/// and it publishes no `panel:clip` because nothing about it scrolls.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "palette", prefix: "input", meaning: "the palette's search box (focused on open)", command: None },
    HitKeyDoc { panel: "palette", prefix: "top", meaning: "the current top match — the entry Enter picks", command: None },
    HitKeyDoc { panel: "palette", prefix: "item:", meaning: "one offered entry, by its opaque id (item:P.CU for the add-feature palette's Cube)", command: None },
    HitKeyDoc { panel: "palette", prefix: "display", meaning: "the layout dropdown's header", command: None },
    HitKeyDoc { panel: "palette", prefix: "display:", meaning: "a layout in the open dropdown (display:LargeIcons, :MediumIcons, :SmallIcons, :CompactMulti, :CompactSingle)", command: None },
];

