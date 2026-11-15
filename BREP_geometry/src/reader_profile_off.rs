//! No reader bookkeeping, clocks or caller propagation in ordinary builds.
pub const CURVE: u8 = 0;
pub const SURFACE: u8 = 1;
pub const PCURVE: u8 = 2;
pub const BASIS: u8 = 1;
pub const ACCUMULATE: u8 = 2;
pub struct Point;
impl Point {
    #[inline(always)]
    pub fn open(_: u8, _: &'static std::panic::Location<'static>) -> Option<Self> { None }
}
pub struct Part;
impl Part {
    #[inline(always)]
    pub fn open(_: u8) -> Option<Self> { None }
}
