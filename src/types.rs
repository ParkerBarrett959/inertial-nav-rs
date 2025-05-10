//! Navigation types
use nalgebra::{Matrix3, Vector3, Vector6};

/// IMU measurement
///
/// This struct contains a timestamped IMU measurement.
///
/// # Fields
///
/// * `t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
/// * `d_v` - 3-axis change in velocity measurement (m/s)
/// * `d_theta` - 3-axis change in orientation measurement (rad)
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuMeasurement {
    pub t: f64,
    pub d_v: Vector3<f64>,
    pub d_theta: Vector3<f64>,
}

/// Gyroscope error model
///
/// This struct contains a definition of gyroscope errors. The error model definition
/// is d_theta_corrected = (I + diag(scale_factor)) * (I + misalignment) * (d_theta - bias*dt)
///
/// # Fields
///
/// * `scale_factor` - 3-axis scale factor error (ppm)
/// * `misalignment` - 6-element misalignment (off-diagonals row major)
/// * `bias` - 3-axis bias (rad/s)
#[derive(Debug, Clone, Copy, Default)]
pub struct GyroscopeError {
    pub scale_factor: Vector3<f64>,
    pub misalignment: Vector6<f64>,
    pub bias: Vector3<f64>,
}

/// Accelerometer error model
///
/// This struct contains a definition of accelerometer errors. The error model definition
/// is d_v_corrected = (I + diag(scale_factor)) * (I + misalignment) * (d_v - bias*dt)
///
/// # Fields
///
/// * `scale_factor` - 3-axis scale factor error (ppm)
/// * `misalignment` - 6-element misalignment (off-diagonals)
/// * `bias` - 3-axis bias (m/s^2)
#[derive(Debug, Clone, Copy, Default)]
pub struct AccelerometerError {
    pub scale_factor: Vector3<f64>,
    pub misalignment: Vector6<f64>,
    pub bias: Vector3<f64>,
}

/// IMU error model
///
/// This struct contains the combination of gyroscope and accelerometer errors.
///
/// # Fields
///
/// * `accelerometer_error` - Accelerometer error model
/// * `gyroscope_error` - Gyroscope error model
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuError {
    pub accelerometer_error: AccelerometerError,
    pub gyroscope_error: GyroscopeError,
}

/// 6-DOF navigation state
///
/// This struct contains the position/velocity/attitude of an object with respect to
/// the ECEF reference frame.
///
/// # Fields
///
/// * `position` - 3 dimensional ECEF position vector (m)
/// * `velocity` - 3 dimensional ECEF velocity vector (m/s)
/// * `body_to_ecef` - 3x3 DCM relating the IMU body frame to ECEF
#[derive(Debug, Clone, Copy, Default)]
pub struct NavState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub body_to_ecef: Matrix3<f64>,
}
