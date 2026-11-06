//! No reader bookkeeping, clocks or caller propagation in ordinary builds.
pub(crate) const CURVE: u8 = 0;
pub(crate) const SURFACE: u8 = 1;
pub(crate) const PCURVE: u8 = 2;
pub(crate) const BASIS: u8 = 1;
pub(crate) const ACCUMULATE: u8 = 2;
pub(crate) struct Point;
impl Point {
    #[inline(always)]
    pub(crate) fn open(_: u8, _: &'static std::panic::Location<'static>) -> Option<Self> { None }
}
pub(crate) struct Part;
impl Part {
    #[inline(always)]
    pub(crate) fn open(_: u8) -> Option<Self> { None }
}
