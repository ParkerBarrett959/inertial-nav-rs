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
/// * `nav_state_prev` - The ECEF frame position/velocity/attitude from the previous step
/// * `v_dot_prev` - Time rate of change of the ECEF velocity at the previous timestep
#[derive(Debug, Clone, Copy, Default)]
pub struct Strapdown {
    pub t: f64,
    pub nav_state: NavState,
    pub nav_state_prev: NavState,
    pub v_dot_prev: Vector3<f64>,
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
            nav_state_prev: init_state,
            v_dot_prev: Vector3::<f64>::zeros(),
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
        // Compute timestep and set previous state to current state
        let dt: f64 = imu.t - self.t;
        self.nav_state_prev = self.nav_state;
        self.t = imu.t;

        // Compute approximate angular rate of ECEF wrt inertial on the ECEF frame
        let wei_e = Vector3::<f64>::new(0.0, 0.0, OMEGA_EARTH);
        let omegaei_e: Matrix3<f64> = skew(&wei_e);

        // Compute angular rate of body wrt inertial in the body frame
        let wbi_b: Vector3<f64> = imu.d_theta / dt;

        // Compute angular rate of body wrt ECEF in the body frame
        let wbe_b: Vector3<f64> = wbi_b - self.nav_state_prev.body_to_ecef.transpose() * wei_e;

        // Get skew symmetric matrix
        let omegabe_b: Matrix3<f64> = skew(&wbe_b);

        // Integrate attitude using matrix exponential approach
        let body_curr_to_body_prev: Matrix3<f64> = (omegabe_b * dt).exp();
        self.nav_state.body_to_ecef = self.nav_state_prev.body_to_ecef * body_curr_to_body_prev;

        // Compute approximate current rate of change of velocity
        let v_kp1_approx: Vector3<f64> = self.nav_state_prev.velocity + self.v_dot_prev * dt;
        let coriolis: Vector3<f64> = -2.0 * omegaei_e * v_kp1_approx;
        let centrifugal: Vector3<f64> =
            -omegaei_e * omegaei_e * 0.5 * dt * (self.nav_state_prev.velocity + v_kp1_approx);
        let specific_force: Vector3<f64> = self.nav_state.body_to_ecef * (imu.d_v / dt);
        let gravity: Vector3<f64> = Vector3::<f64>::zeros(); // TODO
        let v_dot_curr = coriolis + centrifugal + specific_force + gravity;

        // Integrate velocity and position using a trapezoidal integration scheme
        self.nav_state.velocity =
            self.nav_state.velocity + 0.5 * dt * (self.v_dot_prev + v_dot_curr);
        self.v_dot_prev = v_dot_curr;
        self.nav_state.position = self.nav_state.position
            + 0.5 * dt * (self.nav_state_prev.velocity + self.nav_state.velocity);
    }
}
