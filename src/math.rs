//! Math Utility functions
use nalgebra::{Matrix3, Vector3};

/// Get the skew-symmetric form of a vector
///
/// # Arguments
///
/// * `vec3` - A 3x1 vector
///
/// # Returns
///
/// Returns a 3x3 skew symmetric matrix from the elemnts of the vector.
pub fn skew(vec3: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::<f64>::new(
        0.0, -vec3[2], vec3[1], vec3[2], 0.0, -vec3[0], -vec3[1], vec3[0], 0.0,
    )
}

/// Floating point remainder of division
///
/// # Arguments
///
/// * `num` - Numerator
/// * `den` - Denominator
///
/// # Returns
///
/// Returns the floating pointer remainder after division.
pub fn fmod(num: f64, den: f64) -> f64 {
    num % den
}
