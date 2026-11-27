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

/// `(pattern id, seed id, n)` of an instance id. The seed may itself be an
/// instance of an earlier pattern (a pattern of a pattern):
/// `APAT2.APAT1.ACOMP3.i2.i3` is place 3 of `APAT1.ACOMP3.i2` in `APAT2`. The
/// pattern is the first segment and the place the last, so a one-level id
/// reads as before.
pub fn parse_instance_id(id: &str) -> Option<(&str, &str, i64)> {
    let (pattern_id, rest) = id.split_once('.')?;
    let (seed, key) = rest.rsplit_once('.')?;
    if seed.is_empty() || !is_pattern_id(pattern_id) {
        return None;
    }
    if seed.contains('.') && parse_instance_id(seed).is_none() {
        return None;
    }
    Some((pattern_id, seed, key.strip_prefix('i')?.parse().ok()?))
}
