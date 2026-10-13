//! Building polygon parsing and obstacle ray-casting.
//!
//! Parses OpenStreetMap/SUMO polygon files (`osm.poly.xml`) and performs
//! multi-segment intersection, wall penetration distance calculation,
//! concrete building penetration attenuation ($0.85\,\text{dB/m}$ capped at $45\,\text{dB}$),
//! and occluding vertex extraction for knife-edge diffraction.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use quick_xml::events::Event as XmlEvent;
use quick_xml::reader::Reader as XmlReader;
use serde::{Deserialize, Serialize};

use crate::common::{SimError, SimResult};
use crate::geometry::aabb::AABB;
use crate::geometry::segment::{Point2D, Segment2D};

/// A building polygon representing an urban obstacle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildingPolygon {
    pub id: String,
    pub vertices: Vec<Point2D>,
    pub segments: Vec<Segment2D>,
    pub aabb: AABB,
}

impl BuildingPolygon {
    pub fn new(id: String, vertices: Vec<Point2D>) -> Self {
        let aabb = AABB::from_points(&vertices);
        let mut segments = Vec::with_capacity(vertices.len());
        if vertices.len() >= 3 {
            for i in 0..vertices.len() {
                let next_idx = (i + 1) % vertices.len();
                segments.push(Segment2D::new(vertices[i], vertices[next_idx]));
            }
        }
        Self {
            id,
            vertices,
            segments,
            aabb,
        }
    }

    /// Check whether a line-of-sight segment intersects this building.
    #[inline]
    pub fn intersects_segment(&self, ray: &Segment2D) -> bool {
        if !self.aabb.intersects_segment(ray) {
            return false;
        }
        for seg in &self.segments {
            if seg.intersects(ray) {
                return true;
            }
        }
        false
    }

    /// Find all intersection points between the ray and the building perimeter.
    pub fn intersection_points(&self, ray: &Segment2D) -> Vec<Point2D> {
        if !self.aabb.intersects_segment(ray) {
            return Vec::new();
        }
        let mut points = Vec::new();
        for seg in &self.segments {
            if let Some(pt) = seg.intersection_point(ray) {
                points.push(pt);
            }
        }
        // Sort intersection points by distance from ray.p1
        points.sort_by(|a, b| {
            let d_a = ray.p1.distance_squared(a);
            let d_b = ray.p1.distance_squared(b);
            d_a.partial_cmp(&d_b).unwrap_or(std::cmp::Ordering::Equal)
        });
        points
    }

    /// Compute the penetration distance (meters) through this building.
    pub fn penetration_distance(&self, ray: &Segment2D) -> f64 {
        let pts = self.intersection_points(ray);
        if pts.len() < 2 {
            0.0
        } else {
            let mut total_dist = 0.0;
            // Pair up entry and exit points
            for chunk in pts.chunks(2) {
                if chunk.len() == 2 {
                    total_dist += chunk[0].distance_to(&chunk[1]);
                }
            }
            total_dist
        }
    }

    /// Point-in-polygon containment test using ray-casting algorithm.
    pub fn contains_point(&self, p: &Point2D) -> bool {
        if !self.aabb.contains_point(p) {
            return false;
        }
        let mut inside = false;
        let n = self.vertices.len();
        for i in 0..n {
            let j = (i + n - 1) % n;
            let vi = &self.vertices[i];
            let vj = &self.vertices[j];

            if ((vi.y > p.y) != (vj.y > p.y))
                && (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x)
            {
                inside = !inside;
            }
        }
        inside
    }

    /// Find the closest vertex of this building to the line of sight ray
    /// for knife-edge diffraction calculation.
    pub fn closest_vertex_to_ray(&self, ray: &Segment2D) -> (Point2D, f64) {
        let mut min_dist = f64::MAX;
        let mut best_v = self.vertices[0];

        for v in &self.vertices {
            let d = ray.distance_to_point(v);
            if d < min_dist {
                min_dist = d;
                best_v = *v;
            }
        }
        (best_v, min_dist)
    }
}

/// Map containing all building obstacles in a scenario.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BuildingMap {
    pub buildings: Vec<BuildingPolygon>,
    pub overall_aabb: AABB,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstructionInfo {
    pub is_los: bool,
    pub wall_crossings: usize,
    pub penetration_distance_m: f64,
    pub penetration_attenuation_db: f64,
    pub closest_corner_dist_m: f64,
    pub closest_corner_point: Option<Point2D>,
}

impl BuildingMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_buildings(buildings: Vec<BuildingPolygon>) -> Self {
        let all_points: Vec<Point2D> = buildings
            .iter()
            .flat_map(|b| b.vertices.iter().copied())
            .collect();
        let overall_aabb = AABB::from_points(&all_points);
        Self {
            buildings,
            overall_aabb,
        }
    }

    /// Parse buildings from SUMO/OSM `osm.poly.xml` file.
    pub fn from_osm_poly_xml<P: AsRef<Path>>(path: P) -> SimResult<Self> {
        let file = File::open(path.as_ref())
            .map_err(|e| SimError::IoError(format!("Failed to open polygon file: {}", e)))?;
        let mut reader = XmlReader::from_reader(BufReader::new(file));
        reader.config_mut().trim_text(true);

        let mut buildings = Vec::new();
        let mut buf = Vec::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(XmlEvent::Start(ref e)) | Ok(XmlEvent::Empty(ref e)) => {
                    if e.name().as_ref() == b"poly" {
                        let mut poly_id = String::new();
                        let mut poly_type = String::new();
                        let mut shape_str = String::new();

                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"id" => {
                                    poly_id = String::from_utf8_lossy(&attr.value).into_owned();
                                }
                                b"type" => {
                                    poly_type = String::from_utf8_lossy(&attr.value).into_owned();
                                }
                                b"shape" => {
                                    shape_str = String::from_utf8_lossy(&attr.value).into_owned();
                                }
                                _ => {}
                            }
                        }

                        // Only ingest building obstacles
                        if poly_type.starts_with("building") || poly_type.contains("building") {
                            let mut vertices = Vec::new();
                            for coord_pair in shape_str.split_whitespace() {
                                let mut parts = coord_pair.split(',');
                                if let (Some(x_str), Some(y_str)) = (parts.next(), parts.next()) {
                                    if let (Ok(x), Ok(y)) =
                                        (x_str.parse::<f64>(), y_str.parse::<f64>())
                                    {
                                        vertices.push(Point2D::new(x, y));
                                    }
                                }
                            }
                            if vertices.len() >= 3 {
                                buildings.push(BuildingPolygon::new(poly_id, vertices));
                            }
                        }
                    }
                }
                Ok(XmlEvent::Eof) => break,
                Err(e) => return Err(SimError::XmlParseError(format!("XML error: {}", e))),
                _ => {}
            }
            buf.clear();
        }

        Ok(Self::with_buildings(buildings))
    }

    /// Fast Line-of-Sight check between two 2D points.
    pub fn is_los(&self, p1: &Point2D, p2: &Point2D) -> bool {
        let ray = Segment2D::new(*p1, *p2);
        if !self.overall_aabb.intersects_segment(&ray) {
            return true;
        }
        for b in &self.buildings {
            if b.intersects_segment(&ray) {
                return false;
            }
        }
        true
    }

    /// Calculate comprehensive obstruction parameters between two points.
    pub fn evaluate_obstruction(
        &self,
        p1: &Point2D,
        p2: &Point2D,
        db_per_m: f64,
        max_attenuation_db: f64,
    ) -> ObstructionInfo {
        let ray = Segment2D::new(*p1, *p2);
        if !self.overall_aabb.intersects_segment(&ray) {
            return ObstructionInfo {
                is_los: true,
                wall_crossings: 0,
                penetration_distance_m: 0.0,
                penetration_attenuation_db: 0.0,
                closest_corner_dist_m: f64::MAX,
                closest_corner_point: None,
            };
        }

        let mut wall_crossings = 0;
        let mut total_pen_dist = 0.0;
        let mut min_corner_dist = f64::MAX;
        let mut closest_corner = None;

        for b in &self.buildings {
            if b.aabb.intersects_segment(&ray) {
                let pts = b.intersection_points(&ray);
                if !pts.is_empty() {
                    wall_crossings += pts.len();
                    total_pen_dist += b.penetration_distance(&ray);

                    let (v, d) = b.closest_vertex_to_ray(&ray);
                    if d < min_corner_dist {
                        min_corner_dist = d;
                        closest_corner = Some(v);
                    }
                }
            }
        }

        let is_los = wall_crossings == 0;
        let penetration_attenuation_db = (total_pen_dist * db_per_m).min(max_attenuation_db);

        ObstructionInfo {
            is_los,
            wall_crossings,
            penetration_distance_m: total_pen_dist,
            penetration_attenuation_db,
            closest_corner_dist_m: min_corner_dist,
            closest_corner_point: closest_corner,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_building_polygon_intersection() {
        let vertices = vec![
            Point2D::new(10.0, 10.0),
            Point2D::new(20.0, 10.0),
            Point2D::new(20.0, 20.0),
            Point2D::new(10.0, 20.0),
        ];
        let building = BuildingPolygon::new("b1".to_string(), vertices);
        let ray = Segment2D::new(Point2D::new(0.0, 15.0), Point2D::new(30.0, 15.0));

        assert!(building.intersects_segment(&ray));
        let pen_dist = building.penetration_distance(&ray);
        assert!((pen_dist - 10.0).abs() < 1e-6);
    }
}
