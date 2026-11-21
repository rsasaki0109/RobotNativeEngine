//! ROS 2 CDR (XCDR1, little-endian) serialization of adapter messages.
//!
//! These encoders produce the exact payload bytes a ROS 2 publisher puts on the
//! wire and `ros2 bag record` stores: a 4-byte encapsulation header followed by
//! the message fields, each aligned to its own size relative to the end of the
//! encapsulation header. No ROS 2 installation is needed.
//!
//! Only the message types the bag writer records are covered: `sensor_msgs/Imu`,
//! `sensor_msgs/PointCloud2` and `tf2_msgs/TFMessage`.

use crate::messages::{
    RosHeader, RosImu, RosPointCloud2, RosQuaternion, RosTfMessage, RosTransformStamped, RosVector3,
};

/// CDR little-endian encapsulation header (`CDR_LE`, no options).
const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Minimal ROS 2 CDR writer.
#[derive(Debug)]
pub struct CdrWriter {
    buffer: Vec<u8>,
}

impl Default for CdrWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl CdrWriter {
    /// Starts a little-endian CDR payload.
    pub fn new() -> Self {
        Self {
            buffer: CDR_LE_HEADER.to_vec(),
        }
    }

    fn align(&mut self, alignment: usize) {
        let offset = self.buffer.len() - CDR_LE_HEADER.len();
        let padding = (alignment - offset % alignment) % alignment;
        self.buffer.extend(std::iter::repeat_n(0_u8, padding));
    }

    /// Writes a `uint8` (also used for `bool` and `byte`).
    pub fn write_u8(&mut self, value: u8) {
        self.buffer.push(value);
    }

    /// Writes a `bool`.
    pub fn write_bool(&mut self, value: bool) {
        self.write_u8(u8::from(value));
    }

    /// Writes an aligned `int32`.
    pub fn write_i32(&mut self, value: i32) {
        self.align(4);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    /// Writes an aligned `uint32`.
    pub fn write_u32(&mut self, value: u32) {
        self.align(4);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    /// Writes an aligned `float64`.
    pub fn write_f64(&mut self, value: f64) {
        self.align(8);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    /// Writes a `string`: a `uint32` length that counts the trailing NUL, the
    /// UTF-8 bytes, and the NUL.
    pub fn write_string(&mut self, value: &str) {
        self.write_u32(sequence_length(value.len() + 1));
        self.buffer.extend_from_slice(value.as_bytes());
        self.buffer.push(0);
    }

    /// Writes a `uint8[]` sequence.
    pub fn write_bytes(&mut self, value: &[u8]) {
        self.write_u32(sequence_length(value.len()));
        self.buffer.extend_from_slice(value);
    }

    /// Returns the finished payload.
    pub fn finish(self) -> Vec<u8> {
        self.buffer
    }

    fn write_header(&mut self, header: &RosHeader) {
        self.write_i32(header.stamp.sec);
        self.write_u32(header.stamp.nanosec);
        self.write_string(&header.frame_id);
    }

    fn write_vector3(&mut self, value: &RosVector3) {
        self.write_f64(value.x);
        self.write_f64(value.y);
        self.write_f64(value.z);
    }

    fn write_quaternion(&mut self, value: &RosQuaternion) {
        self.write_f64(value.x);
        self.write_f64(value.y);
        self.write_f64(value.z);
        self.write_f64(value.w);
    }

    fn write_covariance(&mut self, value: &[f64; 9]) {
        for item in value {
            self.write_f64(*item);
        }
    }

    fn write_transform_stamped(&mut self, value: &RosTransformStamped) {
        self.write_header(&value.header);
        self.write_string(&value.child_frame_id);
        self.write_vector3(&value.transform.translation);
        self.write_quaternion(&value.transform.rotation);
    }
}

fn sequence_length(length: usize) -> u32 {
    u32::try_from(length).expect("CDR sequences are limited to u32::MAX elements")
}

/// Serializes a `sensor_msgs/msg/Imu`.
pub fn encode_imu(message: &RosImu) -> Vec<u8> {
    let mut writer = CdrWriter::new();
    writer.write_header(&message.header);
    writer.write_quaternion(&message.orientation);
    writer.write_covariance(&message.orientation_covariance);
    writer.write_vector3(&message.angular_velocity);
    writer.write_covariance(&message.angular_velocity_covariance);
    writer.write_vector3(&message.linear_acceleration);
    writer.write_covariance(&message.linear_acceleration_covariance);
    writer.finish()
}

/// Serializes a little-endian `sensor_msgs/msg/PointCloud2`.
///
/// Field names are read up to their first NUL byte, as stored in
/// [`crate::RosPointField::name`].
pub fn encode_pointcloud2(message: &RosPointCloud2) -> Vec<u8> {
    let mut writer = CdrWriter::new();
    writer.write_header(&message.header);
    writer.write_u32(message.height);
    writer.write_u32(message.width);
    writer.write_u32(sequence_length(message.fields.len()));
    for field in &message.fields {
        let name_len = field
            .name
            .iter()
            .position(|byte| *byte == 0)
            .unwrap_or(field.name.len());
        let name = String::from_utf8_lossy(&field.name[..name_len]);
        writer.write_string(&name);
        writer.write_u32(field.offset);
        writer.write_u8(field.datatype);
        writer.write_u32(field.count);
    }
    writer.write_bool(false); // is_bigendian
    writer.write_u32(message.point_step);
    writer.write_u32(message.row_step);
    writer.write_bytes(&message.data);
    writer.write_bool(message.is_dense);
    writer.finish()
}

/// Serializes a `tf2_msgs/msg/TFMessage` (used for both `/tf` and `/tf_static`).
pub fn encode_tf_message(message: &RosTfMessage) -> Vec<u8> {
    let mut writer = CdrWriter::new();
    writer.write_u32(sequence_length(message.transforms.len()));
    for transform in &message.transforms {
        writer.write_transform_stamped(transform);
    }
    writer.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{RosPointField, RosTime, RosTransform};

    fn header(frame_id: &str) -> RosHeader {
        RosHeader {
            stamp: RosTime { sec: 3, nanosec: 7 },
            frame_id: frame_id.to_string(),
        }
    }

    #[test]
    fn strings_count_the_nul_and_doubles_align_after_the_encapsulation_header() {
        let mut writer = CdrWriter::new();
        writer.write_string("ab");
        writer.write_f64(1.0);
        let bytes = writer.finish();
        // header(4) + len(4) + "ab\0"(3) + pad(1) + f64(8)
        assert_eq!(bytes.len(), 20);
        assert_eq!(&bytes[4..8], &3_u32.to_le_bytes());
        assert_eq!(&bytes[8..11], b"ab\0");
        assert_eq!(bytes[11], 0);
        assert_eq!(&bytes[12..20], &1.0_f64.to_le_bytes());
    }

    #[test]
    fn imu_payload_has_the_fixed_ros2_layout() {
        let message = RosImu {
            header: header("imu"),
            orientation: RosQuaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            angular_velocity: RosVector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: RosVector3 {
                x: 0.0,
                y: 0.0,
                z: 9.81,
            },
            linear_acceleration_covariance: [0.0; 9],
        };
        let bytes = encode_imu(&message);
        // header(4) + stamp(8) + "imu\0" string(8) + 37 doubles
        assert_eq!(bytes.len(), 4 + 8 + 8 + 37 * 8);
        let gyro_x_offset = 4 + 8 + 8 + 4 * 8 + 9 * 8;
        assert_eq!(
            &bytes[gyro_x_offset..gyro_x_offset + 8],
            &0.5_f64.to_le_bytes()
        );
        assert_eq!(
            &bytes[bytes.len() - 80..bytes.len() - 72],
            &9.81_f64.to_le_bytes()
        );
    }

    #[test]
    fn pointcloud2_payload_ends_with_data_and_dense_flag() {
        let mut name = [0_u8; 32];
        name[0] = b'x';
        let message = RosPointCloud2 {
            header: header("lidar"),
            height: 1,
            width: 1,
            fields: vec![RosPointField {
                name,
                offset: 0,
                datatype: 7,
                count: 1,
            }],
            point_step: 4,
            row_step: 4,
            data: 2.5_f32.to_le_bytes().to_vec(),
            is_dense: true,
        };
        let bytes = encode_pointcloud2(&message);
        assert_eq!(bytes[bytes.len() - 1], 1);
        assert_eq!(
            &bytes[bytes.len() - 5..bytes.len() - 1],
            &2.5_f32.to_le_bytes()
        );
        assert_eq!(
            &bytes[bytes.len() - 9..bytes.len() - 5],
            &4_u32.to_le_bytes()
        );
        // The field name is written without the padding of the fixed-size array.
        let field_name_at = bytes
            .windows(6)
            .position(|window| window == [2, 0, 0, 0, b'x', 0])
            .expect("field name");
        assert!(field_name_at > 0);
    }

    #[test]
    fn tf_message_serializes_each_transform() {
        let message = RosTfMessage {
            transforms: vec![RosTransformStamped {
                header: header("imu"),
                child_frame_id: "lidar".to_string(),
                transform: RosTransform {
                    translation: RosVector3 {
                        x: 0.1,
                        y: 0.2,
                        z: 0.3,
                    },
                    rotation: RosQuaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
            }],
        };
        let bytes = encode_tf_message(&message);
        assert_eq!(&bytes[4..8], &1_u32.to_le_bytes());
        assert_eq!(&bytes[bytes.len() - 8..], &1.0_f64.to_le_bytes());
    }
}
