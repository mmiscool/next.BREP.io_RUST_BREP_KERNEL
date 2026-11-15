//! Conservative indexing of immutable trim-ring boxes. The leaf test is the
//! original box-separation expression; unions use only min/max, so rejecting
//! a node rejects every box below it without changing a geometric predicate.

#[derive(Clone)]
struct Node {
    bounds: [f64; 4],
    range: std::ops::Range<usize>,
    children: Option<(usize, usize)>,
}

pub(super) struct BoxIndex {
    boxes: Vec<[f64; 4]>,
    order: Vec<usize>,
    nodes: Vec<Node>,
    overflow: Vec<usize>,
}

fn apart(a: [f64; 4], b: [f64; 4], margin: f64) -> bool {
    a[1] + margin < b[0] || b[1] + margin < a[0] || a[3] + margin < b[2] || b[3] + margin < a[2]
}

impl BoxIndex {
    pub(super) fn new(boxes: &[[f64; 4]]) -> Self {
        let (order, overflow) = (0..boxes.len()).partition(|&i| {
            boxes[i].iter().all(|x| x.is_finite())
                && boxes[i][0] <= boxes[i][1] && boxes[i][2] <= boxes[i][3]
        });
        let mut result = Self { boxes: boxes.to_vec(), order, overflow, nodes: Vec::new() };
        if !result.order.is_empty() {
            result.nodes.reserve(result.order.len() * 2);
            result.build(0..result.order.len());
        }
        result
    }

    fn build(&mut self, range: std::ops::Range<usize>) -> usize {
        let mut bounds = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
        for &i in &self.order[range.clone()] {
            let b = self.boxes[i];
            bounds = [bounds[0].min(b[0]), bounds[1].max(b[1]), bounds[2].min(b[2]), bounds[3].max(b[3])];
        }
        let index = self.nodes.len();
        self.nodes.push(Node { bounds, range: range.clone(), children: None });
        if range.len() > 8 {
            let axis = if bounds[1] - bounds[0] >= bounds[3] - bounds[2] { 0 } else { 2 };
            let middle = range.start + range.len() / 2;
            let boxes = &self.boxes;
            self.order[range.clone()].select_nth_unstable_by(range.len() / 2, |&a, &b| {
                let center = |i: usize| boxes[i][axis] * 0.5 + boxes[i][axis + 1] * 0.5;
                center(a).total_cmp(&center(b)).then_with(|| a.cmp(&b))
            });
            let left = self.build(range.start..middle);
            let right = self.build(middle..range.end);
            self.nodes[index].children = Some((left, right));
        }
        index
    }

    /// Visit exactly the boxes for which the original separation test is
    /// false. Stop as soon as a visitor returns true. Traversal order is
    /// intentionally unspecified; consumers needing authored order sort ids.
    pub(super) fn visit(&self, query: [f64; 4], margin: f64, mut visitor: impl FnMut(usize) -> bool) -> bool {
        if !self.nodes.is_empty() && self.visit_node(0, query, margin, &mut visitor) {
            return true;
        }
        self.overflow.iter().any(|&i| !apart(query, self.boxes[i], margin) && visitor(i))
    }

    fn visit_node(&self, index: usize, query: [f64; 4], margin: f64, visitor: &mut impl FnMut(usize) -> bool) -> bool {
        let node = &self.nodes[index];
        if apart(query, node.bounds, margin) {
            return false;
        }
        if let Some((left, right)) = node.children {
            self.visit_node(left, query, margin, visitor) || self.visit_node(right, query, margin, visitor)
        } else {
            self.order[node.range.clone()].iter().any(|&i| !apart(query, self.boxes[i], margin) && visitor(i))
        }
    }
}
