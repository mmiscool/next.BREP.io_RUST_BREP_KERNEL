//! Compile away diagnostic scopes in ordinary builds, including the allocator hook.
pub const FIT: usize = 1;
pub const POINT: usize = 2;
pub const SEEDED: usize = 3;
pub const LOOKUP: usize = 4;
pub const MARCH: usize = 6;
pub const CLASSIFY: usize = 7;
pub struct Scope;
impl Scope {
    #[inline(always)]
    pub fn enter(_: usize) -> Option<Self> { None }
}
pub struct Stage;
impl Stage {
    #[inline(always)]
    pub fn open(_: &'static str) -> Option<Self> { None }
}
