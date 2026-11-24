//! The Inspector's "Pin functions" table for a placed part: which of each
//! pin's functions ([`brep_ecad_core::Pin::alternates`]) the sheet prints for
//! it. Documentary only; nets, ERC and the netlist read the pin's number and
//! main name whatever is chosen here.
use crate::Editor;
use brep_ecad_core::Component;

impl Editor {
    /// The table for `c`, one dropdown per pin that serves more than one
    /// function, keyed `inspector:pin_function:<number>` (and
    /// `…#<function>` for each choice). Nothing for a part whose pins each
    /// have one name.
    pub(crate) fn pin_functions_table(&mut self, ui: &mut egui::Ui, c: &Component) {
        let pins: Vec<_> = c.symbol.pins.iter().filter(|p| !p.alternates.is_empty()).collect();
        if pins.is_empty() {
            return;
        }
        ui.separator();
        ui.label("Pin functions");
        ui.small("The name the sheet shows for each pin. Nothing is remapped or checked.");
        let mut chosen = vec![];
        egui::Grid::new("pin_functions").num_columns(2).striped(true).show(ui, |ui| {
            for pin in pins {
                ui.label(&pin.number);
                let choices: Vec<(String, &str, &str)> = std::iter::once(pin.name.as_str())
                    .chain(pin.alternates.iter().map(String::as_str))
                    .map(|name| (name.to_owned(), name, name))
                    .collect();
                let before = c.pin_label(pin).to_owned();
                let mut current = before.clone();
                self.properties_hits.enum_dropdown(
                    ui,
                    &format!("pin_function:{}", pin.number),
                    &choices,
                    &mut current,
                );
                if current != before {
                    chosen.push((pin.number.clone(), current));
                }
                ui.end_row();
            }
        });
        let id = c.id;
        for (number, function) in chosen {
            self.transaction(|d| {
                let _ = d.set_pin_function(id, &number, Some(&function));
            });
        }
    }
}
