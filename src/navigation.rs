//! The top level navigation object
use crate::strapdown::Strapdown;
use crate::types::{ImuError, ImuMeasurement};
use crate::utilities::correct_imu;

/// Navigation struct definition
///
/// The navigation struct contains the strapdown position/velocity/attitude state,
/// a navigation filter for predicting strapdown errors and making corrections with
/// external measurements, and an error model struct which ocntains the current best
/// estimate of IMU errors. The corrections are applied as measurements arrive.
///
/// # Fields
///
/// * `t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
/// * `strapdown` - Strapdown object holding the navigation state
/// * `imu_error` - IMU error model parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct Navigation {
    pub t: f64,
    pub strapdown: Strapdown,
    // pub filter: Filter,     // TODO
    pub imu_error: ImuError,
}

impl Navigation {
    /// Initialization with default error model
    ///
    /// # Arguments
    ///
    /// * `init_t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
    /// * `init_strapdown` - Initial strapdown object containing position/velocity/attitude
    ///
    /// # Returns
    ///
    /// Returns the initialized navigation object.
    pub fn new(init_t: f64, init_strapdown: Strapdown) -> Self {
        Navigation {
            t: init_t,
            strapdown: init_strapdown,
            ..Default::default()
        }
    }

    /// Initialization with imu error model
    ///
    /// # Arguments
    ///
    /// * `init_t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
    /// * `init_strapdown` - Initial strapdown object containing position/velocity/attitude
    /// * `init_imu_error` - Initial imu error model (gyro and accelerometers)
    ///
    /// # Returns
    ///
    /// Returns the initialized navigation object.
    pub fn with_imu_error(
        init_t: f64,
        init_strapdown: Strapdown,
        init_imu_error: ImuError,
    ) -> Self {
        Navigation {
            t: init_t,
            strapdown: init_strapdown,
            imu_error: init_imu_error,
        }
    }

    /// Integrate the current state given an IMU measurement
    ///
    /// This function first applies the IMU error model correction, then uses the
    /// compensated measurement to integrate the strapdown and navigation filter.
    ///
    /// # Arguments
    ///
    /// * `imu` - A reference to a timestamped IMU measurement
    ///
    /// # Returns
    ///
    /// Returns the integrated navigation object
    pub fn integrate(&mut self, imu: &ImuMeasurement) {
        // Compute timestep
        let dt: f64 = imu.t - self.t;

        // Compensate the incoming measurement for errors
        let imu_corrected: ImuMeasurement = correct_imu(&imu, &self.imu_error, dt);

        // Integrate the strapdown solution

        // Integrate the navigation filter
    }
}
