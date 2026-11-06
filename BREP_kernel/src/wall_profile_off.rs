//! Compile away diagnostic scopes in ordinary builds, including the allocator hook.
pub(crate) const FIT: usize = 1;
pub(crate) const POINT: usize = 2;
pub(crate) const SEEDED: usize = 3;
pub(crate) const LOOKUP: usize = 4;
pub(crate) const MARCH: usize = 6;
pub(crate) const CLASSIFY: usize = 7;
pub(crate) struct Scope;
impl Scope {
    #[inline(always)]
    pub(crate) fn enter(_: usize) -> Option<Self> { None }
}
pub(crate) struct Stage;
impl Stage {
    #[inline(always)]
    pub(crate) fn open(_: &'static str) -> Option<Self> { None }
}
