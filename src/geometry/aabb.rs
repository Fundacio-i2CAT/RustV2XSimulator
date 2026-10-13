//! Axis-Aligned Bounding Box (AABB) for broad-phase spatial culling.

use crate::geometry::segment::{Point2D, Segment2D};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AABB {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl AABB {
    pub const fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    pub fn from_points(points: &[Point2D]) -> Self {
        if points.is_empty() {
            return Self::new(0.0, 0.0, 0.0, 0.0);
        }
        let mut min_x = points[0].x;
        let mut max_x = points[0].x;
        let mut min_y = points[0].y;
        let mut max_y = points[0].y;

        for p in &points[1..] {
            if p.x < min_x {
                min_x = p.x;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.y > max_y {
                max_y = p.y;
            }
        }

        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[inline]
    pub fn contains_point(&self, p: &Point2D) -> bool {
        p.x >= self.min_x && p.x <= self.max_x && p.y >= self.min_y && p.y <= self.max_y
    }

    #[inline]
    pub fn overlaps(&self, other: &AABB) -> bool {
        self.max_x >= other.min_x
            && self.min_x <= other.max_x
            && self.max_y >= other.min_y
            && self.min_y <= other.max_y
    }

    /// Fast slab-method segment intersection test against this AABB.
    pub fn intersects_segment(&self, seg: &Segment2D) -> bool {
        let (s_min_x, s_max_x) = if seg.p1.x < seg.p2.x {
            (seg.p1.x, seg.p2.x)
        } else {
            (seg.p2.x, seg.p1.x)
        };
        let (s_min_y, s_max_y) = if seg.p1.y < seg.p2.y {
            (seg.p1.y, seg.p2.y)
        } else {
            (seg.p2.y, seg.p1.y)
        };

        if s_max_x < self.min_x
            || s_min_x > self.max_x
            || s_max_y < self.min_y
            || s_min_y > self.max_y
        {
            return false;
        }

        let dx = seg.p2.x - seg.p1.x;
        let dy = seg.p2.y - seg.p1.y;

        let mut t_min = 0.0f64;
        let mut t_max = 1.0f64;

        // X slab
        if dx.abs() > 1e-12 {
            let tx1 = (self.min_x - seg.p1.x) / dx;
            let tx2 = (self.max_x - seg.p1.x) / dx;
            let (t_near_x, t_far_x) = if tx1 < tx2 { (tx1, tx2) } else { (tx2, tx1) };
            t_min = t_min.max(t_near_x);
            t_max = t_max.min(t_far_x);
            if t_min > t_max {
                return false;
            }
        } else if seg.p1.x < self.min_x || seg.p1.x > self.max_x {
            return false;
        }

        // Y slab
        if dy.abs() > 1e-12 {
            let ty1 = (self.min_y - seg.p1.y) / dy;
            let ty2 = (self.max_y - seg.p1.y) / dy;
            let (t_near_y, t_far_y) = if ty1 < ty2 { (ty1, ty2) } else { (ty2, ty1) };
            t_min = t_min.max(t_near_y);
            t_max = t_max.min(t_far_y);
            if t_min > t_max {
                return false;
            }
        } else if seg.p1.y < self.min_y || seg.p1.y > self.max_y {
            return false;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_segment_intersection() {
        let box1 = AABB::new(10.0, 10.0, 20.0, 20.0);
        let s_hit = Segment2D::new(Point2D::new(0.0, 15.0), Point2D::new(30.0, 15.0));
        let s_miss = Segment2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 5.0));

        assert!(box1.intersects_segment(&s_hit));
        assert!(!box1.intersects_segment(&s_miss));
    }
}
