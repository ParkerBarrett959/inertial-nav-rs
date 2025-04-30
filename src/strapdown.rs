use nalgebra::{Vector3};
use crate::types::{ImuMeasurement, ImuError};

/// Strapdown Integrator Struct
pub struct Strapdown {
    pub t: f64,              // Timestamp of the current process
    // TODO: Add a navigation state (PVA)
    pub imu_error: ImuError, // IMU error model used to compensate measurements
}
