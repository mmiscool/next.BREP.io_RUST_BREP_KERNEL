//! A pin's alternate functions ([`Pin::alternates`]) and the one each placed
//! part shows ([`Component::pin_functions`]). Documentary only: nets, ERC and
//! the netlist bind by number and read the main name and type.
use crate::{Component, Document, Pin};
use uuid::Uuid;

impl Component {
    /// The name this component's `pin` goes by: the function chosen for it
    /// ([`Component::pin_functions`]) while its symbol still offers that
    /// function, else the pin's main name.
    pub fn pin_label<'a>(&'a self, pin: &'a Pin) -> &'a str {
        self.pin_functions
            .get(&pin.number)
            .filter(|chosen| pin.alternates.contains(chosen))
            .map_or(pin.name.as_str(), String::as_str)
    }
}

impl Document {
    /// Choose which function component `id`'s pin `number` goes by: one of the
    /// pin's [`Pin::alternates`], or its main name (`None`, or the main name
    /// itself), which clears the choice. Documentary only; see
    /// [`Pin::alternates`]. Returns whether anything changed.
    pub fn set_pin_function(&mut self, id: Uuid, number: &str, function: Option<&str>) -> Result<bool, String> {
        let c = self
            .components
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or("Component no longer exists")?;
        let pin = c
            .symbol
            .pins
            .iter()
            .find(|p| p.number == number)
            .ok_or_else(|| format!("{} has no pin {number}", c.reference))?;
        let chosen = match function {
            None => None,
            Some(name) if name == pin.name => None,
            Some(name) if pin.alternates.iter().any(|a| a == name) => Some(name.to_owned()),
            Some(name) => {
                return Err(format!(
                    "Pin {number} of {} has no function {name}; it offers {}",
                    c.reference,
                    std::iter::once(pin.name.as_str())
                        .chain(pin.alternates.iter().map(String::as_str))
                        .collect::<Vec<_>>()
                        .join(", ")
                ));
            }
        };
        let before = c.pin_functions.get(number).cloned();
        match &chosen {
            Some(name) => {
                c.pin_functions.insert(number.to_owned(), name.clone());
            }
            None => {
                c.pin_functions.remove(number);
            }
        }
        Ok(before != chosen)
    }
}
