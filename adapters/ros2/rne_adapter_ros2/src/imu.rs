//! IMU mapping helpers.

use crate::messages::{RosHeader, RosImu, RosQuaternion, RosVector3};
use rne_core::SimTime;
use rne_data::ImuSample;
use rne_math::Vec3;

/// Converts an RNE IMU sample to a ROS `Imu` message.
///
/// The sample carries no orientation estimate, so the orientation is identity
/// and `orientation_covariance[0]` is `-1`, the ROS marker for "not provided".
/// The angular velocity and linear acceleration covariances are left at zero
/// ("unknown"), as most IMU drivers publish them.
pub fn to_ros_imu(sample: &ImuSample, sim_time: SimTime, frame_id: &str) -> RosImu {
    let mut orientation_covariance = [0.0; 9];
    orientation_covariance[0] = -1.0;
    RosImu {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        orientation: RosQuaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance,
        angular_velocity: to_ros_vector3(sample.angular_velocity_rad_s),
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: to_ros_vector3(sample.linear_acceleration_m_s2),
        linear_acceleration_covariance: [0.0; 9],
    }
}

fn to_ros_vector3(value: Vec3) -> RosVector3 {
    RosVector3 {
        x: value.x,
        y: value.y,
        z: value.z,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_sample_maps_rates_and_marks_orientation_unknown() {
        let sample = ImuSample {
            angular_velocity_rad_s: Vec3::new(0.1, -0.2, 0.3),
            linear_acceleration_m_s2: Vec3::new(0.0, 0.0, 9.81),
        };
        let message = to_ros_imu(&sample, SimTime::from_ticks(2_250_000_000), "imu");
        assert_eq!(message.header.frame_id, "imu");
        assert_eq!(message.header.stamp.sec, 2);
        assert_eq!(message.header.stamp.nanosec, 250_000_000);
        assert_eq!(message.angular_velocity.y, -0.2);
        assert_eq!(message.linear_acceleration.z, 9.81);
        assert_eq!(message.orientation_covariance[0], -1.0);
        assert_eq!(message.orientation.w, 1.0);
    }
}
