//! Utility functions
use crate::types::{ImuError, ImuMeasurement};
use nalgebra::Matrix3;

/// Correct an iMU measurement given an error model
///
/// # Arguments
///
/// * `meas` - A reference to a timestamped IMU measurement
/// * `error` - A reference to an IMU error model
/// * `dt` - The timestep to apply the correction (s)
///
/// # Returns
///
/// Returns a corrected IMU measurement
pub fn correct_imu(meas: &ImuMeasurement, error: &ImuError, dt: f64) -> ImuMeasurement {
    // Define gyroscope scale factor matrix. Convert ppm to decimal.
    let gyro_sf: Matrix3<f64> = Matrix3::<f64>::identity()
        + Matrix3::<f64>::from_diagonal(
            &error.gyroscope_error.scale_factor.map(|x| x / 1_000_000.0),
        );

    // Define gyroscope misalignment matrix
    let gyro_mis: Matrix3<f64> = Matrix3::<f64>::new(
        1.0,
        error.gyroscope_error.misalignment[0],
        error.gyroscope_error.misalignment[1],
        error.gyroscope_error.misalignment[2],
        1.0,
        error.gyroscope_error.misalignment[3],
        error.gyroscope_error.misalignment[4],
        error.gyroscope_error.misalignment[5],
        1.0,
    );

    // Define accelerometer scale factor matrix. Convert ppm to decimal.
    let accel_sf: Matrix3<f64> = Matrix3::<f64>::identity()
        + Matrix3::<f64>::from_diagonal(
            &error
                .accelerometer_error
                .scale_factor
                .map(|x| x / 1_000_000.0),
        );

    // Define accelerometer misalignment matrix
    let accel_mis: Matrix3<f64> = Matrix3::<f64>::new(
        1.0,
        error.accelerometer_error.misalignment[0],
        error.accelerometer_error.misalignment[1],
        error.accelerometer_error.misalignment[2],
        1.0,
        error.accelerometer_error.misalignment[3],
        error.accelerometer_error.misalignment[4],
        error.accelerometer_error.misalignment[5],
        1.0,
    );

    // Correct the measurement
    ImuMeasurement {
        t: meas.t,
        d_v: accel_sf * accel_mis * (meas.d_v - error.accelerometer_error.bias * dt),
        d_theta: gyro_sf * gyro_mis * (meas.d_theta - error.gyroscope_error.bias * dt),
    }
}
