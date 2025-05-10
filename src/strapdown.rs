//! The strapdown integrator
use crate::types::NavState;

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
}
