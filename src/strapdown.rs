//! The strapdown integrator
use crate::constants::OMEGA_EARTH;
use crate::math::skew;
use crate::types::{ImuMeasurement, NavState};
use nalgebra::{Matrix3, Vector3};

/// Strapdown struct definition
///
/// Defines the strapdown state which is a navigation state at a given time.
///
/// # Fields
///
/// * `t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
/// * `nav_state` - The ECEF frame position/velocity/attitude
#[derive(Debug, Clone, Copy, Default)]
pub struct Strapdown {
    pub t: f64,
    pub nav_state: NavState,
}

impl Strapdown {
    /// Initialization of the strapdown state
    ///
    /// # Arguments
    ///
    /// * `init_t` - Unix timestamp (seconds) since UTC epoch (January 1st, 1970)
    /// * `init_state` - Initial ECEF position/velocity/attitude
    ///
    /// # Returns
    ///
    /// Returns the initialized strapdown object.
    pub fn new(init_t: f64, init_state: NavState) -> Self {
        Strapdown {
            t: init_t,
            nav_state: init_state,
        }
    }

    /// Integrate the strapdown state with IMU data
    ///
    /// # Arguments
    ///
    /// * `imu` - The current IMU measurement
    ///
    /// # Returns
    ///
    /// Returns the integrated strapdown object.
    pub fn integrate(&mut self, imu: &ImuMeasurement) {
        // Compute timestep
        let dt: f64 = imu.t - self.t;

        // Compute approximate angular rate of ECEF wrt inertial on the ECEF frame
        let wei_e = Vector3::<f64>::new(0.0, 0.0, OMEGA_EARTH);

        // Compute angular rate of body wrt inertial in the body frame
        let wbi_b: Vector3<f64> = imu.d_theta / dt;

        // Compute angular rate of body wrt ECEF in the body frame
        let wbe_b: Vector3<f64> = wbi_b - self.nav_state.body_to_ecef.transpose() * wei_e;

        // Get skew symmetric matrix
        let omegabe_b: Matrix3<f64> = skew(&wbe_b);

        // Integrate attitude using matrix exponential approach
        self.nav_state.body_to_ecef = self.nav_state.body_to_ecef * (omegabe_b * dt).exp();

        // Integrate velocity

        // Integrate position
    }
}
