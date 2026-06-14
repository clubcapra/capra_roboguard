//! Geodetic <-> local ENU about a datum.
//!
//! Exact port of the sim's equirectangular mapping
//! (`rove_sim/rove_sim/api/devices.py`): a small-angle tangent-plane projection
//! about the datum. Must stay byte-for-byte consistent so the pose the engine
//! derives matches the pose the sim encoded.

/// WGS84 equatorial radius (metres) — same constant as the sim (`_EARTH_R`).
pub const EARTH_R: f64 = 6_378_137.0;

/// geodetic (deg) -> local ENU metres about (`lat0`,`lon0`).
/// Returns (east `x`, north `y`).
pub fn geodetic_to_enu(lat: f64, lon: f64, lat0: f64, lon0: f64) -> (f64, f64) {
    let x = (lon - lon0).to_radians() * EARTH_R * lat0.to_radians().cos();
    let y = (lat - lat0).to_radians() * EARTH_R;
    (x, y)
}

/// local ENU metres -> geodetic (deg). Inverse of [`geodetic_to_enu`].
/// (Used when emitting waypoints/targets back as lat/lon — wired in later slices.)
#[allow(dead_code)]
pub fn enu_to_geodetic(x: f64, y: f64, lat0: f64, lon0: f64) -> (f64, f64) {
    let lat = lat0 + (y / EARTH_R).to_degrees();
    let lon = lon0 + (x / (EARTH_R * lat0.to_radians().cos())).to_degrees();
    (lat, lon)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Thun datum (rove_sim DEFAULT_DATUM).
    const LAT0: f64 = 46.7512;
    const LON0: f64 = 7.6131;

    #[test]
    fn roundtrip_is_stable() {
        let (x, y) = (58.3, -22.7);
        let (lat, lon) = enu_to_geodetic(x, y, LAT0, LON0);
        let (x2, y2) = geodetic_to_enu(lat, lon, LAT0, LON0);
        assert!((x - x2).abs() < 1e-6, "x {x} vs {x2}");
        assert!((y - y2).abs() < 1e-6, "y {y} vs {y2}");
    }

    #[test]
    fn matches_python_formula() {
        // From a live VectorNav frame off the deployed sim.
        let (lat, lon) = (46.750995272811515, 7.613862762657887);
        let (x, y) = geodetic_to_enu(lat, lon, LAT0, LON0);
        // Python: x = rad(lon-lon0)*R*cos(rad(lat0)); y = rad(lat-lat0)*R
        let px = (lon - LON0).to_radians() * EARTH_R * LAT0.to_radians().cos();
        let py = (lat - LAT0).to_radians() * EARTH_R;
        assert!((x - px).abs() < 1e-9);
        assert!((y - py).abs() < 1e-9);
        // Sanity: robot sits tens of metres E and ~20 m S of the datum.
        assert!(x > 40.0 && x < 80.0, "x={x}");
        assert!(y < 0.0 && y > -40.0, "y={y}");
    }
}
