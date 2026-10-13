//! Geodetic and spatial coordinate transformations (SUMO Cartesian <-> UTM <-> WGS-84).
//!
//! Conforms to ETSI EN 302 637-2 microdegree coordinates (1e-7 degrees).

use serde::{Deserialize, Serialize};

/// Earth radius in meters (WGS-84 mean).
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Geodetic coordinates in WGS-84 (latitude, longitude, elevation).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Wgs84Coordinate {
    /// Latitude in decimal degrees [-90.0, 90.0].
    pub latitude: f64,
    /// Longitude in decimal degrees [-180.0, 180.0].
    pub longitude: f64,
    /// Altitude/elevation in meters.
    pub altitude: f64,
}

impl Wgs84Coordinate {
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    /// Convert latitude to ETSI C-ITS integer microdegrees (1e-7 deg).
    #[inline]
    pub fn etsi_latitude(&self) -> i32 {
        (self.latitude * 10_000_000.0).round() as i32
    }

    /// Convert longitude to ETSI C-ITS integer microdegrees (1e-7 deg).
    #[inline]
    pub fn etsi_longitude(&self) -> i32 {
        (self.longitude * 10_000_000.0).round() as i32
    }

    /// Construct from ETSI C-ITS integer microdegrees.
    pub fn from_etsi_microdegrees(lat_micro: i32, lon_micro: i32, altitude: f64) -> Self {
        Self {
            latitude: lat_micro as f64 * 1e-7,
            longitude: lon_micro as f64 * 1e-7,
            altitude,
        }
    }

    /// Compute Haversine great-circle distance to another point in meters.
    pub fn haversine_distance(&self, other: &Wgs84Coordinate) -> f64 {
        let lat1_rad = self.latitude.to_radians();
        let lon1_rad = self.longitude.to_radians();
        let lat2_rad = other.latitude.to_radians();
        let lon2_rad = other.longitude.to_radians();

        let dlat = lat2_rad - lat1_rad;
        let dlon = lon2_rad - lon1_rad;

        let a = (dlat / 2.0).sin().powi(2)
            + lat1_rad.cos() * lat2_rad.cos() * (dlon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        EARTH_RADIUS_M * c
    }
}

/// Geodetic transform mapping between SUMO 2D coordinates and WGS-84/UTM.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeodeticTransform {
    pub utm_zone: u8,
    pub northern_hemisphere: bool,
    pub net_offset_x: f64,
    pub net_offset_y: f64,
    pub orig_lat: f64,
    pub orig_lon: f64,
}

impl Default for GeodeticTransform {
    fn default() -> Self {
        // Default Barcelona (Diagonal) UTM Zone 31N
        Self {
            utm_zone: 31,
            northern_hemisphere: true,
            net_offset_x: -425117.14,
            net_offset_y: -4580733.51,
            orig_lat: 41.38,
            orig_lon: 2.12,
        }
    }
}

impl GeodeticTransform {
    pub fn new(
        utm_zone: u8,
        northern_hemisphere: bool,
        net_offset_x: f64,
        net_offset_y: f64,
        orig_lat: f64,
        orig_lon: f64,
    ) -> Self {
        Self {
            utm_zone,
            northern_hemisphere,
            net_offset_x,
            net_offset_y,
            orig_lat,
            orig_lon,
        }
    }

    /// Convert SUMO cartesian coordinates (x, y) to WGS-84 lat/lon.
    pub fn sumo_to_wgs84(&self, x: f64, y: f64, z: f64) -> Wgs84Coordinate {
        let easting = x - self.net_offset_x;
        let northing = y - self.net_offset_y;
        let zone_letter = if self.northern_hemisphere { 'N' } else { 'S' };

        match utm::wsg84_utm_to_lat_lon(easting, northing, self.utm_zone, zone_letter) {
            Ok((lat, lon)) => Wgs84Coordinate::new(lat, lon, z),
            Err(_) => {
                // Fallback: local tangent plane projection around origin
                let lat = self.orig_lat + (y / 111_139.0);
                let lon = self.orig_lon + (x / (111_139.0 * self.orig_lat.to_radians().cos()));
                Wgs84Coordinate::new(lat, lon, z)
            }
        }
    }

    /// Convert WGS-84 coordinate to SUMO cartesian (x, y).
    pub fn wgs84_to_sumo(&self, coord: &Wgs84Coordinate) -> (f64, f64) {
        let (easting, northing, _) =
            utm::to_utm_wgs84(coord.latitude, coord.longitude, self.utm_zone);
        let x = easting + self.net_offset_x;
        let y = northing + self.net_offset_y;
        (x, y)
    }
}

/// 2D Euclidean distance in meters.
#[inline]
pub fn euclidean_distance_2d(p1: (f64, f64), p2: (f64, f64)) -> f64 {
    let dx = p1.0 - p2.0;
    let dy = p1.1 - p2.1;
    (dx * dx + dy * dy).sqrt()
}

/// 3D Euclidean distance in meters.
#[inline]
pub fn euclidean_distance_3d(p1: (f64, f64, f64), p2: (f64, f64, f64)) -> f64 {
    let dx = p1.0 - p2.0;
    let dy = p1.1 - p2.1;
    let dz = p1.2 - p2.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_etsi_microdegrees() {
        let coord = Wgs84Coordinate::new(41.385064, 2.1734035, 10.0);
        assert_eq!(coord.etsi_latitude(), 413_850_640);
        assert_eq!(coord.etsi_longitude(), 21_734_035);

        let roundtrip = Wgs84Coordinate::from_etsi_microdegrees(
            coord.etsi_latitude(),
            coord.etsi_longitude(),
            10.0,
        );
        assert!((roundtrip.latitude - coord.latitude).abs() < 1e-6);
        assert!((roundtrip.longitude - coord.longitude).abs() < 1e-6);
    }

    #[test]
    fn test_haversine_distance() {
        let p1 = Wgs84Coordinate::new(41.3851, 2.1734, 0.0);
        let p2 = Wgs84Coordinate::new(41.3860, 2.1734, 0.0);
        let dist = p1.haversine_distance(&p2);
        assert!((dist - 100.0).abs() < 5.0);
    }
}
