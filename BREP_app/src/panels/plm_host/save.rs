//! Save-time checkout consent, remembered only by this running CAD session.
use super::*;
use crate::panels::{file::FileDialog, plm::Verb};

#[derive(Default)]
pub(super) struct SaveCheckout {
    pub document: Option<u64>,
    answer: Option<bool>,
    remember: bool,
    checking_out: bool,
}

impl SaveCheckout {
    fn answer(&mut self, yes: bool) {
        if self.remember { self.answer = Some(yes); }
        if yes { self.checking_out = true; } else { self.document = None; }
    }
}

impl PlmHost {
    pub(crate) fn save_prompt(&mut self, ctx: &egui::Context, docs: &mut Documents, file: &mut FileDialog, store: &dyn ModelStore) {
        self.hits.retain(|key, _| !key.starts_with("save:"));
        let Some(client) = self.client.clone() else { return };
        if let Some(id) = file.take_plm_save_request() {
            if self.save.document.is_none() {
                self.save.document = Some(id);
                self.save.remember = false;
                self.save.checking_out = false;
                if let Some(panel) = self.lifecycle.get_mut(&id) { panel.refresh(&client); }
            }
        }
        let Some(id) = self.save.document else { return };
        // A queued save must never write whichever tab happens to be active later.
        if docs.active_id() != id {
            self.save.document = None;
            file.status = "Save cancelled because the active document changed".into();
            return;
        }
        let Some(panel) = self.lifecycle.get_mut(&id) else {
            ctx.request_repaint();
            return;
        };
        if panel.busy() { ctx.request_repaint(); return; }
        if panel.revision().is_some_and(|rev| rev.locked_by_me && rev.editable) {
            self.save.document = None;
            file.finish_plm_save(docs, store);
            return;
        }
        if self.save.checking_out || !panel.offered().contains(&Verb::CheckOut) {
            self.save.document = None;
            file.status = format!("Save cancelled: {}", panel.message().unwrap_or("this revision cannot be checked out; see the PLM panel"));
            docs.engine_mut().push_notice(&file.status);
            return;
        }
        let mut answer = self.save.answer;
        if answer.is_none() {
            let label = docs.active().title();
            let shown = egui::Modal::new(egui::Id::new("plm-save-checkout")).show(ctx, |ui| {
                ui.heading("Check out before saving?");
                ui.label(format!("{label} is not checked out. Check it out and save?"));
                let remember = ui.checkbox(&mut self.save.remember, "Use this answer for all saves in this CAD session");
                self.hits.insert("save:remember".into(), remember.rect);
                ui.horizontal(|ui| {
                    let yes = ui.button("Check out and save");
                    self.hits.insert("save:yes".into(), yes.rect);
                    if yes.clicked() { answer = Some(true); }
                    let no = ui.button("Don't save");
                    self.hits.insert("save:no".into(), no.rect);
                    if no.clicked() { answer = Some(false); }
                });
            });
            // Escape/dismiss cancels this save without remembering a decision.
            if shown.should_close() && answer.is_none() {
                self.save.document = None;
                file.status = "Save cancelled".into();
                return;
            }
        }
        if let Some(yes) = answer {
            self.save.answer(yes);
            if yes {
                panel.press(&client, Verb::CheckOut);
                file.status = "Checking out before saving…".into();
                ctx.request_repaint();
            } else {
                file.status = "Save cancelled; the part was not checked out".into();
            }
        }
    }
}

