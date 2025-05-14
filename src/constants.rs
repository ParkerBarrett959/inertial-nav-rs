//! Constants and Parameters

/// Pi
pub const PI: f64 = std::f64::consts::PI;

/// Earth rotation rate (rad/s)
pub const OMEGA_EARTH: f64 = 7.2921159 * 1.0e-5;

/// Earth gravitational constant
pub const KM: f64 = 3.986005 * 1.0e14;

/// Normal gravity at equation (m/s^2)
pub const GEQ: f64 = 9.7803267715;

/// Degrees to radians
pub const DEG_TO_RAD: f64 = PI / 180.0;

/// Radians to degrees
pub const RAD_TO_DEG: f64 = 180.0 / PI;

/// WGS84 ellipsoid semi-major axis (m)
pub const A: f64 = 6378137.0;

/// WGS84 ellipsoid semi-minor axis (m)
pub const B: f64 = 6356752.3141;

/// WGS84 ellipsoid flattening term
pub const F: f64 = 1.0 / 298.257222101;
