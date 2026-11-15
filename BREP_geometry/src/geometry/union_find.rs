/// Deterministic union-find (smallest index wins as representative).
pub struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    pub fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    pub fn find(&mut self, x: usize) -> usize {
        let mut root = x;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        let mut cur = x;
        while self.parent[cur] != root {
            let next = self.parent[cur];
            self.parent[cur] = root;
            cur = next;
        }
        root
    }

    pub fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            self.parent[hi] = lo;
        }
    }
}

/// Path-halving over an existing parent slice; preserves the caller's root policy.
#[inline]
pub fn find_root(parent: &mut [usize], mut index: usize) -> usize {
    while parent[index] != index {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    index
}

/// Full compression for sparse identifiers, preserving the caller's chosen representative.
pub fn find_keyed_root<K, S>(parent: &mut std::collections::HashMap<K, K, S>, key: K) -> K
where
    K: Copy + Eq + std::hash::Hash,
    S: std::hash::BuildHasher,
{
    let next = *parent.entry(key).or_insert(key);
    if next == key { return key; }
    let root = find_keyed_root(parent, next);
    parent.insert(key, root);
    root
}
