use crate::types::{ImuError, NavState};

/// Strapdown Integrator Struct
pub struct Strapdown {
    pub t: f64,              // Timestamp of the current process
    pub nav_state: NavState, // Strapdown position/velocity/attitude
    pub imu_error: ImuError, // IMU error model used to compensate measurements
}
