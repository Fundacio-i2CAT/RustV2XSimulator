//! 2D Spatial Indexing using KD-Tree for O(log N) candidate receiver culling.
//!
//! Restricts link evaluation to nodes within communication range ($d \le 800\,\text{m}$).
//! Also includes an all-pairs $\mathcal{O}(N^2)$ mode for ablation testing ($A3$).

use crate::geometry::segment::Point2D;
use crate::scheduler::event::ActorId;

#[derive(Debug, Clone)]
pub struct NodeEntry {
    pub id: ActorId,
    pub point: Point2D,
}

impl NodeEntry {
    pub fn new(id: ActorId, point: Point2D) -> Self {
        Self { id, point }
    }
}

#[derive(Debug, Clone)]
enum KdNode {
    Leaf(Vec<NodeEntry>),
    Internal {
        split_axis: usize, // 0 for X, 1 for Y
        split_val: f64,
        left: Box<KdNode>,
        right: Box<KdNode>,
    },
}

#[derive(Debug, Clone, Default)]
pub struct SpatialIndex {
    root: Option<KdNode>,
    count: usize,
    use_all_pairs: bool,
    all_entries: Vec<NodeEntry>,
}

const LEAF_CAPACITY: usize = 16;

impl SpatialIndex {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_all_pairs(use_all_pairs: bool) -> Self {
        Self {
            root: None,
            count: 0,
            use_all_pairs,
            all_entries: Vec::new(),
        }
    }

    pub fn set_all_pairs_mode(&mut self, enabled: bool) {
        self.use_all_pairs = enabled;
    }

    /// Rebuild the spatial index with current node locations.
    pub fn rebuild(&mut self, mut entries: Vec<NodeEntry>) {
        self.count = entries.len();
        if self.use_all_pairs {
            self.all_entries = entries;
            self.root = None;
        } else {
            self.all_entries = entries.clone();
            self.root = Some(Self::build_recursive(&mut entries, 0));
        }
    }

    fn build_recursive(entries: &mut [NodeEntry], depth: usize) -> KdNode {
        if entries.len() <= LEAF_CAPACITY {
            return KdNode::Leaf(entries.to_vec());
        }

        let axis = depth % 2;
        if axis == 0 {
            entries.sort_by(|a, b| {
                a.point
                    .x
                    .partial_cmp(&b.point.x)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        } else {
            entries.sort_by(|a, b| {
                a.point
                    .y
                    .partial_cmp(&b.point.y)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        let mid = entries.len() / 2;
        let split_val = if axis == 0 {
            entries[mid].point.x
        } else {
            entries[mid].point.y
        };

        let (left_slice, right_slice) = entries.split_at_mut(mid);
        let left_child = Self::build_recursive(left_slice, depth + 1);
        let right_child = Self::build_recursive(right_slice, depth + 1);

        KdNode::Internal {
            split_axis: axis,
            split_val,
            left: Box::new(left_child),
            right: Box::new(right_child),
        }
    }

    /// Find all candidate receiver nodes within `radius_m` of `center`, excluding `sender_id`.
    pub fn query_radius(
        &self,
        center: Point2D,
        radius_m: f64,
        exclude_id: ActorId,
    ) -> Vec<(ActorId, Point2D, f64)> {
        let r2 = radius_m * radius_m;
        let mut results = Vec::new();

        if self.use_all_pairs {
            // Ablation A3: O(N) linear scan over all nodes
            for entry in &self.all_entries {
                if entry.id != exclude_id {
                    let d2 = center.distance_squared(&entry.point);
                    if d2 <= r2 {
                        results.push((entry.id, entry.point, d2.sqrt()));
                    }
                }
            }
        } else if let Some(ref root) = self.root {
            Self::query_recursive(root, &center, radius_m, r2, exclude_id, &mut results);
        }

        results
    }

    fn query_recursive(
        node: &KdNode,
        center: &Point2D,
        radius: f64,
        radius_sq: f64,
        exclude_id: ActorId,
        results: &mut Vec<(ActorId, Point2D, f64)>,
    ) {
        match node {
            KdNode::Leaf(entries) => {
                for entry in entries {
                    if entry.id != exclude_id {
                        let d2 = center.distance_squared(&entry.point);
                        if d2 <= radius_sq {
                            results.push((entry.id, entry.point, d2.sqrt()));
                        }
                    }
                }
            }
            KdNode::Internal {
                split_axis,
                split_val,
                left,
                right,
            } => {
                let (center_coord, _) = if *split_axis == 0 {
                    (center.x, center.y)
                } else {
                    (center.y, center.x)
                };

                let dist_to_plane = center_coord - *split_val;

                if dist_to_plane <= 0.0 {
                    // Search left first
                    Self::query_recursive(left, center, radius, radius_sq, exclude_id, results);
                    if dist_to_plane.abs() <= radius {
                        Self::query_recursive(
                            right, center, radius, radius_sq, exclude_id, results,
                        );
                    }
                } else {
                    // Search right first
                    Self::query_recursive(right, center, radius, radius_sq, exclude_id, results);
                    if dist_to_plane.abs() <= radius {
                        Self::query_recursive(left, center, radius, radius_sq, exclude_id, results);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kd_tree_radius_search() {
        let mut index = SpatialIndex::new();
        let entries = vec![
            NodeEntry::new(1, Point2D::new(0.0, 0.0)),
            NodeEntry::new(2, Point2D::new(100.0, 0.0)),
            NodeEntry::new(3, Point2D::new(500.0, 0.0)),
            NodeEntry::new(4, Point2D::new(900.0, 0.0)),
        ];
        index.rebuild(entries);

        let res = index.query_radius(Point2D::new(0.0, 0.0), 800.0, 1);
        let ids: Vec<ActorId> = res.iter().map(|(id, _, _)| *id).collect();

        assert_eq!(ids.len(), 2);
        assert!(ids.contains(&2));
        assert!(ids.contains(&3));
        assert!(!ids.contains(&1)); // Sender excluded
        assert!(!ids.contains(&4)); // Out of 800m range
    }
}
