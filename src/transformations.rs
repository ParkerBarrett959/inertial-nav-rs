//! Coordinate transformation functions
use crate::constants::DEG_TO_RAD;
use nalgebra::{Matrix3, Vector3};

/// Get the transformation matrix from ECEF to NED
///
/// # Arguments
///
/// * `lla` - The latitude/longitude/altitude in deg/deg/m above WGS84-ellipsoid
///
/// # Returns
///
/// Returns a 3x3 matrix representing the transformation from the ECEF to NED frames.
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
/// Returns a 3x3 matrix representing the transformation from the NED to ECEF frames.
pub fn ned_to_ecef_transformation(lla: &Vector3<f64>) -> Matrix3<f64> {
    ecef_to_ned_transformation(&lla).transpose()
}
