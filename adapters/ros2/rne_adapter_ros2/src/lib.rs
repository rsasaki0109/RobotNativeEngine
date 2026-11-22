//! ROS2 adapter mappings for Robot Native Engine.

#![deny(missing_docs)]

pub mod camera_info;
pub mod cdr;
pub mod clock;
pub mod image;
pub mod imu;
pub mod jointstate;
pub mod laserscan;
pub mod mcap;
pub mod messages;
pub mod nav_map;
pub mod nav_odometry;
pub mod pointcloud;
pub mod tf;

pub use camera_info::{camera_info, pinhole_camera_info};
pub use cdr::{
    encode_camera_info, encode_image, encode_imu, encode_pointcloud2, encode_tf_message, CdrWriter,
};
pub use clock::{to_ros_clock, to_ros_time};
pub use image::{to_ros_image, to_ros_mono8_image};
pub use imu::to_ros_imu;
pub use jointstate::to_ros_joint_state;
pub use laserscan::pointcloud_to_laserscan;
pub use mcap::{
    McapChannelId, McapWriter, Ros2MessageSchema, CAMERA_INFO_SCHEMA, IMAGE_SCHEMA, IMU_SCHEMA,
    POINTCLOUD2_SCHEMA, TF_MESSAGE_SCHEMA,
};
pub use messages::{
    RosCameraInfo, RosClock, RosHeader, RosImage, RosImu, RosJointState, RosLaserScan,
    RosMapMetaData, RosOccupancyGrid, RosOdometry, RosPath, RosPoint, RosPointCloud2,
    RosPointField, RosPose, RosPoseStamped, RosPoseWithCovariance, RosQuaternion,
    RosRegionOfInterest, RosTfMessage, RosTime, RosTransform, RosTransformStamped, RosTwist,
    RosTwistWithCovariance, RosVector3,
};
pub use nav_map::{to_ros_occupancy_grid, to_ros_path, to_ros_pose_stamped};
pub use nav_odometry::{
    from_ros_twist, to_ros_odometry, to_ros_pose, to_ros_twist, yaw_from_ros_quaternion,
    yaw_to_pose,
};
pub use pointcloud::{decode_xyz_points, to_ros_pointcloud2};
pub use tf::{
    to_ros_tf_message, to_ros_tf_message_from_buffer, to_ros_transform,
    to_ros_transform_from_matrix, to_ros_transform_stamped,
};

/// Adapter boundary marker for future ROS2 node integration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ros2Adapter;

impl Ros2Adapter {
    /// Returns true when the optional `ros2` feature is enabled.
    pub const fn ros2_runtime_enabled() -> bool {
        cfg!(feature = "ros2")
    }
}
