//! Ids of assembly pattern instances (`APAT<n>.<seed>.i<k>`).


/// `APAT<digits>`: the prefix every instance id carries.
pub fn is_pattern_id(id: &str) -> bool {
    id.strip_prefix("APAT")
        .is_some_and(|digits| !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()))
}

/// The instance id of place `n` of `seed` in pattern `pattern_id`.
pub fn instance_id(pattern_id: &str, seed: &str, n: i64) -> String {
    format!("{pattern_id}.{seed}.i{n}")
}

/// `(pattern id, seed id, n)` of an instance id.
pub fn parse_instance_id(id: &str) -> Option<(&str, &str, i64)> {
    let mut parts = id.split('.');
    let (pattern_id, seed, key) = (parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() || !is_pattern_id(pattern_id) {
        return None;
    }
    Some((pattern_id, seed, key.strip_prefix('i')?.parse().ok()?))
}
