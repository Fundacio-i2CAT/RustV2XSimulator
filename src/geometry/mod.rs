//! 2D Computational geometry, AABB broad-phase culling, polygon building parsing, and KD-Tree spatial indexing.

pub mod aabb;
pub mod kd_tree;
pub mod polygon;
pub mod segment;

pub use aabb::AABB;
pub use kd_tree::{NodeEntry, SpatialIndex};
pub use polygon::{BuildingMap, BuildingPolygon, ObstructionInfo};
pub use segment::{Point2D, Segment2D};
