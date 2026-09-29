//! Authenticated, versioned previews shared by the CAD catalog and workspace.
//! A browser owns its cache, so changing sessions drops pictures and requests.
use super::{client::PlmClient, PlmFuture};
use eframe::egui;
use std::{collections::HashMap, io::Cursor, rc::Rc, task::{Context, Poll, Waker}};

const MAX_ENTRIES: usize = 128;
const MAX_PENDING: usize = 4;
const MAX_BYTES: usize = 1024 * 1024;
pub const SIDE: f32 = 32.0;

/// Only the server's versioned thumbnail routes are accepted, never an arbitrary
/// URL from catalog data. Credentials remain on this client's server.
fn version(url: &str) -> Option<&str> {
    let (path, hash) = url.split_once("?v=")?;
    if hash.len() != 64 || !hash.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let segments: Vec<_> = path.split('/').collect();
    let id = |s: &str| !s.is_empty() && s.bytes().all(|b| b.is_ascii_alphanumeric() || b"-_".contains(&b));
    match segments.as_slice() {
        ["", "api", "parts", part, "thumbnail"] if id(part) => Some(hash),
        ["", "api", "parts", part, "revisions", revision, "thumbnail"] if id(part) && id(revision) => Some(hash),
        _ => None,
    }
}


pub fn fetch(client: Rc<PlmClient>, url: String) -> PlmFuture<Option<Vec<u8>>> {
    Box::pin(async move {
        let expected = version(&url).ok_or("invalid thumbnail URL")?;
        let response = client.call("GET", &url, None).await.map_err(|e| e.to_string())?;
        if response.status == 204 {
            return Ok(None);
        }
        // A document can change between listing and GET. The route then serves
        // its NEW picture: never keep those bytes under the old version's key.
        let etag = response.headers.iter().find(|(name, _)| name.eq_ignore_ascii_case("etag"));
        if etag.map(|(_, value)| value.trim_matches('"')) != Some(expected) {
            return Ok(None);
        }
        if response.body.len() > MAX_BYTES {
            return Err("thumbnail exceeds the size limit".into());
        }
        Ok(Some(response.body))
    })
}

fn decode(bytes: &[u8]) -> Option<egui::ColorImage> {
    if bytes.len() > MAX_BYTES {
        return None;
    }
    let mut reader = image::ImageReader::with_format(Cursor::new(bytes), image::ImageFormat::Png);
    let mut limits = image::Limits::default();
    limits.max_image_width = Some(1024);
    limits.max_image_height = Some(1024);
    limits.max_alloc = Some(16 * 1024 * 1024);
    reader.limits(limits);
    let image = reader.decode().ok()?.thumbnail(64, 64).to_rgba8();
    Some(egui::ColorImage::from_rgba_unmultiplied(
        [image.width() as usize, image.height() as usize], image.as_raw(),
    ))
}

struct Entry {
    pending: Option<PlmFuture<Option<Vec<u8>>>>,
    image: Option<egui::ColorImage>,
    texture: Option<egui::TextureHandle>,
    used: u64,
}

#[derive(Default)]
pub struct ThumbnailCache {
    entries: HashMap<String, Entry>,
    clock: u64,
}

impl ThumbnailCache {
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    pub fn busy(&self) -> bool {
        self.entries.values().any(|e| e.pending.is_some())
    }

    pub fn ready(&self, url: &str) -> bool {
        self.entries.get(url).is_some_and(|e| e.image.is_some() || e.texture.is_some())
    }

    /// Poll even when the panel is hidden; textures are allocated only on draw.
    pub fn poll(&mut self, waker: &Waker) {
        for entry in self.entries.values_mut() {
            if let Some(future) = &mut entry.pending {
                if let Poll::Ready(answer) = future.as_mut().poll(&mut Context::from_waker(waker)) {
                    entry.pending = None;
                    entry.image = answer.ok().flatten().and_then(|bytes| decode(&bytes));
                }
            }
        }
    }

    fn request(&mut self, url: &str, fetch: impl FnOnce(&str) -> PlmFuture<Option<Vec<u8>>>) {
        self.clock = self.clock.wrapping_add(1);
        if let Some(entry) = self.entries.get_mut(url) {
            entry.used = self.clock;
            return;
        }
        if version(url).is_none() || self.entries.values().filter(|e| e.pending.is_some()).count() >= MAX_PENDING {
            return;
        }
        if self.entries.len() >= MAX_ENTRIES {
            let oldest = self.entries.iter().filter(|(_, e)| e.pending.is_none())
                .min_by_key(|(_, e)| e.used).map(|(url, _)| url.clone());
            let Some(oldest) = oldest else { return };
            self.entries.remove(&oldest);
        }
        self.entries.insert(url.to_string(), Entry {
            pending: Some(fetch(url)), image: None, texture: None, used: self.clock,
        });
    }

    /// Reserve the same space for every row. Offscreen rows make no requests.
    pub fn show(&mut self, ui: &mut egui::Ui, url: &str, fetch: impl FnOnce(&str) -> PlmFuture<Option<Vec<u8>>>) -> egui::Response {
        let (rect, response) = ui.allocate_exact_size(egui::Vec2::splat(SIDE), egui::Sense::click());
        if !ui.is_rect_visible(rect) {
            return response;
        }
        if !url.is_empty() {
            self.request(url, fetch);
        }
        if let Some(entry) = self.entries.get_mut(url) {
            if let Some(image) = entry.image.take() {
                entry.texture = Some(ui.ctx().load_texture(format!("plm-thumbnail:{url}"), image, egui::TextureOptions::LINEAR));
            }
            if let Some(texture) = &entry.texture {
                let size = texture.size_vec2();
                let size = size * (SIDE / size.x.max(size.y));
                ui.painter().image(texture.id(), egui::Rect::from_center_size(rect.center(), size),
                    egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
                return response;
            }
        }
        ui.painter().rect_filled(rect.shrink(2.0), 3.0, ui.visuals().faint_bg_color);
        ui.painter().rect_stroke(rect.shrink(2.0), 3.0, ui.visuals().widgets.noninteractive.bg_stroke, egui::StrokeKind::Inside);
        if self.busy() {
            ui.ctx().request_repaint();
        }
        response
    }
}
