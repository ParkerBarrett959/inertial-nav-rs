use nalgebra::Vector3;

/// IMU Measurement
pub struct ImuMeasurement {
    pub t: f64,                // Seconds since UTC epoch (Jan 1, 1970)
    pub d_v: Vector3<f64>,     // 3-axis change in velocity measurement (m/s)
    pub d_theta: Vector3<f64>, // 3-axis change in orientation measurement (rad)
}
