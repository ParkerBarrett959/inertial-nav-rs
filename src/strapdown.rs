use crate::types::{ImuError, NavState};

/// Strapdown Integrator Struct
#[derive(Default)]
pub struct Strapdown {
    pub t: f64,              // Timestamp of the current process
    pub nav_state: NavState, // Strapdown position/velocity/attitude
    pub imu_error: ImuError, // IMU error model used to compensate measurements
}

/// Implementation of Strapdown
impl Strapdown {
    // Strapdown initialization with default error model
    pub fn new(init_t: f64, init_state: NavState) -> Self {
        Strapdown {
            t: init_t,
            nav_state: init_state,
            ..Default::default()
        }
    }

    // Strapdown initialization with error model specified
    pub fn with_imu_error(init_t: f64, init_state: NavState, init_imu_error: ImuError) -> Self {
        Strapdown {
            t: init_t,
            nav_state: init_state,
            imu_error: init_imu_error,
        }
    }
}
