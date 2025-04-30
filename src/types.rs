use nalgebra::{Vector3, Vector6};

/// IMU Measurement
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    pub t: f64,                // Seconds since UTC epoch (Jan 1, 1970)
    pub d_v: Vector3<f64>,     // 3-axis change in velocity measurement (m/s)
    pub d_theta: Vector3<f64>, // 3-axis change in orientation measurement (rad)
}

/// Gyroscope Error Model
#[derive(Debug, Clone, Copy)]
pub struct GyroscopeError {
    pub scale_factor: Vector3<f64>, // 3-axis scale factor error (ppm)
    pub misalignment: Vector6<f64>, // 6-element misalignment (off-diagonals)
    pub bias: Vector3<f64>,         // 3-axis bias (rad/s)
}

/// Accelerometer Error Model
#[derive(Debug, Clone, Copy)]
pub struct AccelerometerError {
    pub scale_factor: Vector3<f64>, // 3-axis scale factor error (ppm)
    pub misalignment: Vector6<f64>, // 6-element misalignment (off-diagonals)
    pub bias: Vector3<f64>,         // 3-axis bias (m/s^2)
}

/// IMU Error Model
#[derive(Debug, Clone, Copy)]
pub struct ImuError {
    pub accelerometer_error: AccelerometerError,
    pub gyroscope_error: GyroscopeError,
}
