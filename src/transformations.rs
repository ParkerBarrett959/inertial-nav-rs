//! Coordinate transformation functions
use crate::constants::{A, B, DEG_TO_RAD, F, PI, RAD_TO_DEG};
use crate::math::fmod;
use nalgebra::{Matrix3, Vector3};

/// Get the transformation matrix from ECEF to NED
///
/// # Arguments
///
/// * `lla` - The latitude/longitude/altitude in deg/deg/m above WGS84-ellipsoid
///
/// # Returns
///
/// Returns a 3x3 matrix re_presenting the transformation from the ECEF to NED frames.
pub fn ecef_to_ned_transformation(lla: &Vector3<f64>) -> Matrix3<f64> {
    // Define helpful quantities
    let slat: f64 = (lla[0] * DEG_TO_RAD).sin();
    let clat: f64 = (lla[0] * DEG_TO_RAD).cos();
    let slon: f64 = (lla[1] * DEG_TO_RAD).sin();
    let clon: f64 = (lla[2] * DEG_TO_RAD).cos();

    // Define rotation
    Matrix3::<f64>::new(
        -slat * clon,
        -slat * slon,
        clat,
        -slon,
        clon,
        0.0,
        -clat * clon,
        -clat * slon,
        -slat,
    )
}

/// Get the transformation matrix from NED to ECEF
///
/// # Arguments
///
/// * `lla` - The latitude/longitude/altitude in deg/deg/m above WGS84-ellipsoid
///
/// # Returns
///
/// Returns a 3x3 matrix re_presenting the transformation from the NED to ECEF frames.
pub fn ned_to_ecef_transformation(lla: &Vector3<f64>) -> Matrix3<f64> {
    ecef_to_ned_transformation(&lla).transpose()
}

/// Transform an LLA position to ECEF
///
/// # Arguments
///
/// * `lla` - The latitude/longitude/altitude in deg/deg/m above WGS84-ellipsoid
///
/// # Returns
///
/// Returns a 3x1 ECEF frame position vector.
pub fn lla_to_ecef(lla: &Vector3<f64>) -> Vector3<f64> {
    // Get latitude and longitude in radians
    let lat_rad: f64 = lla[0] * DEG_TO_RAD;
    let lon_rad: f64 = lla[1] * DEG_TO_RAD;

    // Calculate Earth Eccentricity Squared
    let e_2: f64 = (2.0 * F) - (F * F);

    // Compute Prime Vertical Radius of Curvature
    let n: f64 = A / (1.0 - (e_2 * lat_rad.sin() * lat_rad.sin())).sqrt();

    // Compute ECEF Position
    let r_x: f64 = (n + lla[2]) * lat_rad.cos() * lon_rad.cos();
    let r_y: f64 = (n + lla[2]) * lat_rad.cos() * lon_rad.sin();
    let r_z: f64 = (((1.0 - e_2) * n) + lla[2]) * lat_rad.sin();
    Vector3::<f64>::new(r_x, r_y, r_z)
}

/// Transform an ECEF position to LLA
///
/// # Arguments
///
/// * `ecef` - The 3x1 ECEF frame position vector (m)
///
/// # Returns
///
/// Returns a 3x1 vector containing Latitude/Longitude/Altitude (deg/deg/m above WGS84 ellipsoid)
pub fn ecef_to_lla(ecef: &Vector3<f64>) -> Vector3<f64> {
    // Calculate Earth Quantities
    let e_2: f64 = (2.0 * F) - (F * F);
    let e_p: f64 = ((A * A - B * B) / (B * B)).sqrt();
    let p: f64 = (ecef[0] * ecef[0] + ecef[1] * ecef[1]).sqrt();
    let th: f64 = (A * ecef[2]).atan2(B * p);

    // Compute Latitude in Radians
    let lat_y: f64 = ecef[2] + (e_p * e_p * B * th.sin() * th.sin() * th.sin());
    let lat_x = p - (e_2 * A * th.cos() * th.cos() * th.cos());
    let lat_rad = lat_y.atan2(lat_x);

    // Compute Altitude
    let n: f64 = A / (1.0 - e_2 * lat_rad.sin() * lat_rad.sin()).sqrt();
    let alt: f64 = (p / lat_rad.cos()) - n;

    // Compute Longitude in radians
    let mut lon_rad: f64 = (ecef[1]).atan2(ecef[0]);

    // Check Longitude Range [-pi, pi]
    lon_rad = fmod(lon_rad, 2.0 * PI);
    if lon_rad > PI {
        lon_rad -= 2.0 * PI;
    }
    Vector3::<f64>::new(lat_rad * RAD_TO_DEG, lon_rad * RAD_TO_DEG, alt)
}
