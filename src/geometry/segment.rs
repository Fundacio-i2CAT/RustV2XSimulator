//! 2D Computational Geometry: Points and Line Segments.

use serde::{Deserialize, Serialize};

/// 2D point with f64 precision.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn distance_to(&self, other: &Point2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    #[inline]
    pub fn distance_squared(&self, other: &Point2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }
}

/// 2D line segment between start point `p1` and end point `p2`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Segment2D {
    pub p1: Point2D,
    pub p2: Point2D,
}

impl Segment2D {
    #[inline]
    pub const fn new(p1: Point2D, p2: Point2D) -> Self {
        Self { p1, p2 }
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.p1.distance_to(&self.p2)
    }

    /// Check if this segment intersects another segment `other`.
    pub fn intersects(&self, other: &Segment2D) -> bool {
        // Fast bounding box rejection test
        let (s_min_x, s_max_x) = if self.p1.x < self.p2.x {
            (self.p1.x, self.p2.x)
        } else {
            (self.p2.x, self.p1.x)
        };
        let (s_min_y, s_max_y) = if self.p1.y < self.p2.y {
            (self.p1.y, self.p2.y)
        } else {
            (self.p2.y, self.p1.y)
        };

        let (o_min_x, o_max_x) = if other.p1.x < other.p2.x {
            (other.p1.x, other.p2.x)
        } else {
            (other.p2.x, other.p1.x)
        };
        let (o_min_y, o_max_y) = if other.p1.y < other.p2.y {
            (other.p1.y, other.p2.y)
        } else {
            (other.p2.y, other.p1.y)
        };

        if s_max_x < o_min_x || o_max_x < s_min_x || s_max_y < o_min_y || o_max_y < s_min_y {
            return false;
        }

        let d1 = cross_product_direction(&other.p1, &other.p2, &self.p1);
        let d2 = cross_product_direction(&other.p1, &other.p2, &self.p2);
        let d3 = cross_product_direction(&self.p1, &self.p2, &other.p1);
        let d4 = cross_product_direction(&self.p1, &self.p2, &other.p2);

        if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
            && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
        {
            return true;
        }

        // Collinear / endpoint cases
        if d1.abs() < 1e-9 && on_segment(&other.p1, &self.p1, &other.p2) {
            return true;
        }
        if d2.abs() < 1e-9 && on_segment(&other.p1, &self.p2, &other.p2) {
            return true;
        }
        if d3.abs() < 1e-9 && on_segment(&self.p1, &other.p1, &self.p2) {
            return true;
        }
        if d4.abs() < 1e-9 && on_segment(&self.p1, &other.p2, &self.p2) {
            return true;
        }

        false
    }

    /// Compute the intersection point between two segments if they intersect.
    pub fn intersection_point(&self, other: &Segment2D) -> Option<Point2D> {
        let x1 = self.p1.x;
        let y1 = self.p1.y;
        let x2 = self.p2.x;
        let y2 = self.p2.y;

        let x3 = other.p1.x;
        let y3 = other.p1.y;
        let x4 = other.p2.x;
        let y4 = other.p2.y;

        let denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
        if denom.abs() < 1e-12 {
            return None; // Collinear or parallel
        }

        let ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        let ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

        if (0.0..=1.0).contains(&ua) && (0.0..=1.0).contains(&ub) {
            Some(Point2D::new(x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)))
        } else {
            None
        }
    }

    /// Distance from point `p` to this segment.
    pub fn distance_to_point(&self, p: &Point2D) -> f64 {
        let l2 = self.p1.distance_squared(&self.p2);
        if l2 < 1e-12 {
            return self.p1.distance_to(p);
        }
        let t = ((p.x - self.p1.x) * (self.p2.x - self.p1.x)
            + (p.y - self.p1.y) * (self.p2.y - self.p1.y))
            / l2;
        let t_clamped = t.clamp(0.0, 1.0);
        let projection = Point2D::new(
            self.p1.x + t_clamped * (self.p2.x - self.p1.x),
            self.p1.y + t_clamped * (self.p2.y - self.p1.y),
        );
        p.distance_to(&projection)
    }
}

#[inline]
fn cross_product_direction(pi: &Point2D, pj: &Point2D, pk: &Point2D) -> f64 {
    (pk.x - pi.x) * (pj.y - pi.y) - (pj.x - pi.x) * (pk.y - pi.y)
}

#[inline]
fn on_segment(pi: &Point2D, pk: &Point2D, pj: &Point2D) -> bool {
    let min_x = pi.x.min(pj.x);
    let max_x = pi.x.max(pj.x);
    let min_y = pi.y.min(pj.y);
    let max_y = pi.y.max(pj.y);
    (min_x..=max_x).contains(&pk.x) && (min_y..=max_y).contains(&pk.y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segment_intersection() {
        let s1 = Segment2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        let s2 = Segment2D::new(Point2D::new(0.0, 10.0), Point2D::new(10.0, 0.0));
        assert!(s1.intersects(&s2));

        let pt = s1.intersection_point(&s2).expect("intersection point");
        assert!((pt.x - 5.0).abs() < 1e-9);
        assert!((pt.y - 5.0).abs() < 1e-9);

        let s3 = Segment2D::new(Point2D::new(20.0, 20.0), Point2D::new(30.0, 30.0));
        assert!(!s1.intersects(&s3));
    }
}
