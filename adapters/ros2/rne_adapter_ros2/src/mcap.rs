//! A dependency-free MCAP writer for ROS 2 bags.
//!
//! MCAP is the default `ros2 bag` storage format since ROS 2 Iron. The writer
//! emits the `ros2` profile: `ros2msg` schemas and `cdr` message encoding, so
//! the file opens in ROS 2 tooling and in readers that need no ROS install
//! (for example Calibrex, which audits sensor calibration from bags).
//!
//! The layout is the simplest valid MCAP: a header, then schema, channel and
//! message records in write order, a data-end record and a footer. Chunking,
//! compression, indexes and the summary section are optional in the format and
//! are not written; readers fall back to a linear scan. Messages should be
//! written in non-decreasing log time, as a recorder would.
//!
//! Message payloads come from [`crate::cdr`].

use std::collections::BTreeMap;
use std::io::{self, Write};

const MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";
const OP_HEADER: u8 = 0x01;
const OP_FOOTER: u8 = 0x02;
const OP_SCHEMA: u8 = 0x03;
const OP_CHANNEL: u8 = 0x04;
const OP_MESSAGE: u8 = 0x05;
const OP_DATA_END: u8 = 0x0F;

/// `offered_qos_profiles` of a transient-local (latched) topic such as
/// `/tf_static`, as `ros2 bag record` stores it.
pub const LATCHED_QOS_PROFILE: &str = "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 2147483647\n    nsec: 4294967295\n  lifespan:\n    sec: 2147483647\n    nsec: 4294967295\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 2147483647\n    nsec: 4294967295\n  avoid_ros_namespace_conventions: false\n";

/// A ROS 2 message type with its `ros2msg` definition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ros2MessageSchema {
    /// Fully qualified type name, e.g. `sensor_msgs/msg/Imu`.
    pub name: &'static str,
    /// The `.msg` text followed by every dependency, separated as `ros2 bag` does.
    pub definition: &'static str,
}

#[cfg(test)]
const SEPARATOR: &str =
    "================================================================================\n";

macro_rules! schema_text {
    ($main:expr $(, $dep_name:expr => $dep_text:expr)* $(,)?) => {
        concat!(
            $main
            $(, "================================================================================\n",
            "MSG: ", $dep_name, "\n", $dep_text)*
        )
    };
}

/// `sensor_msgs/msg/Imu`.
pub const IMU_SCHEMA: Ros2MessageSchema = Ros2MessageSchema {
    name: "sensor_msgs/msg/Imu",
    definition: schema_text!(
        "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n",
        "std_msgs/Header" => "builtin_interfaces/Time stamp\nstring frame_id\n",
        "builtin_interfaces/Time" => "int32 sec\nuint32 nanosec\n",
        "geometry_msgs/Quaternion" => "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n",
        "geometry_msgs/Vector3" => "float64 x\nfloat64 y\nfloat64 z\n",
    ),
};

/// `sensor_msgs/msg/PointCloud2`.
pub const POINTCLOUD2_SCHEMA: Ros2MessageSchema = Ros2MessageSchema {
    name: "sensor_msgs/msg/PointCloud2",
    definition: schema_text!(
        "std_msgs/Header header\nuint32 height\nuint32 width\nPointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense\n",
        "std_msgs/Header" => "builtin_interfaces/Time stamp\nstring frame_id\n",
        "builtin_interfaces/Time" => "int32 sec\nuint32 nanosec\n",
        "sensor_msgs/PointField" => "uint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n",
    ),
};

/// `sensor_msgs/msg/Image`.
pub const IMAGE_SCHEMA: Ros2MessageSchema = Ros2MessageSchema {
    name: "sensor_msgs/msg/Image",
    definition: schema_text!(
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\n",
        "std_msgs/Header" => "builtin_interfaces/Time stamp\nstring frame_id\n",
        "builtin_interfaces/Time" => "int32 sec\nuint32 nanosec\n",
    ),
};

/// `sensor_msgs/msg/CameraInfo`.
pub const CAMERA_INFO_SCHEMA: Ros2MessageSchema = Ros2MessageSchema {
    name: "sensor_msgs/msg/CameraInfo",
    definition: schema_text!(
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring distortion_model\nfloat64[] d\nfloat64[9] k\nfloat64[9] r\nfloat64[12] p\nuint32 binning_x\nuint32 binning_y\nsensor_msgs/RegionOfInterest roi\n",
        "std_msgs/Header" => "builtin_interfaces/Time stamp\nstring frame_id\n",
        "builtin_interfaces/Time" => "int32 sec\nuint32 nanosec\n",
        "sensor_msgs/RegionOfInterest" => "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify\n",
    ),
};

/// `tf2_msgs/msg/TFMessage`, the type of `/tf` and `/tf_static`.
pub const TF_MESSAGE_SCHEMA: Ros2MessageSchema = Ros2MessageSchema {
    name: "tf2_msgs/msg/TFMessage",
    definition: schema_text!(
        "geometry_msgs/TransformStamped[] transforms\n",
        "geometry_msgs/TransformStamped" => "std_msgs/Header header\nstring child_frame_id\nTransform transform\n",
        "std_msgs/Header" => "builtin_interfaces/Time stamp\nstring frame_id\n",
        "builtin_interfaces/Time" => "int32 sec\nuint32 nanosec\n",
        "geometry_msgs/Transform" => "Vector3 translation\nQuaternion rotation\n",
        "geometry_msgs/Vector3" => "float64 x\nfloat64 y\nfloat64 z\n",
        "geometry_msgs/Quaternion" => "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n",
    ),
};

/// Identifier of a channel (topic) returned by [`McapWriter::add_channel`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct McapChannelId(u16);

/// Streaming MCAP writer with the ROS 2 profile.
#[derive(Debug)]
pub struct McapWriter<W: Write> {
    out: W,
    schemas: BTreeMap<&'static str, u16>,
    channel_count: u16,
    sequences: Vec<u32>,
}

impl<W: Write> McapWriter<W> {
    /// Writes the magic and the header record.
    pub fn new(mut out: W) -> io::Result<Self> {
        out.write_all(MAGIC)?;
        let mut content = Vec::new();
        put_string(&mut content, "ros2");
        put_string(&mut content, "rne_adapter_ros2");
        write_record(&mut out, OP_HEADER, &content)?;
        Ok(Self {
            out,
            schemas: BTreeMap::new(),
            channel_count: 0,
            sequences: Vec::new(),
        })
    }

    /// Declares a topic; its schema record is written once per message type.
    ///
    /// `latched` records the transient-local QoS profile that `ros2 bag play`
    /// needs to republish a topic such as `/tf_static` to late subscribers.
    pub fn add_channel(
        &mut self,
        topic: &str,
        schema: Ros2MessageSchema,
        latched: bool,
    ) -> io::Result<McapChannelId> {
        let schema_id = match self.schemas.get(schema.name) {
            Some(id) => *id,
            None => {
                let id = u16::try_from(self.schemas.len() + 1).map_err(too_many)?;
                let mut content = Vec::new();
                content.extend_from_slice(&id.to_le_bytes());
                put_string(&mut content, schema.name);
                put_string(&mut content, "ros2msg");
                put_bytes(&mut content, schema.definition.as_bytes());
                write_record(&mut self.out, OP_SCHEMA, &content)?;
                self.schemas.insert(schema.name, id);
                id
            }
        };
        let channel_id = self.channel_count;
        self.channel_count = channel_id.checked_add(1).ok_or_else(|| too_many(()))?;
        let mut metadata = Vec::new();
        if latched {
            put_string(&mut metadata, "offered_qos_profiles");
            put_string(&mut metadata, LATCHED_QOS_PROFILE);
        }
        let mut content = Vec::new();
        content.extend_from_slice(&channel_id.to_le_bytes());
        content.extend_from_slice(&schema_id.to_le_bytes());
        put_string(&mut content, topic);
        put_string(&mut content, "cdr");
        put_bytes(&mut content, &metadata);
        write_record(&mut self.out, OP_CHANNEL, &content)?;
        self.sequences.push(0);
        Ok(McapChannelId(channel_id))
    }

    /// Writes one CDR payload; `log_time_ns` is the bag (receive) time.
    pub fn write_message(
        &mut self,
        channel: McapChannelId,
        log_time_ns: u64,
        publish_time_ns: u64,
        payload: &[u8],
    ) -> io::Result<()> {
        let sequence = self
            .sequences
            .get_mut(usize::from(channel.0))
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "unknown MCAP channel"))?;
        let mut content = Vec::with_capacity(22 + payload.len());
        content.extend_from_slice(&channel.0.to_le_bytes());
        content.extend_from_slice(&sequence.to_le_bytes());
        content.extend_from_slice(&log_time_ns.to_le_bytes());
        content.extend_from_slice(&publish_time_ns.to_le_bytes());
        content.extend_from_slice(payload);
        *sequence = sequence.wrapping_add(1);
        write_record(&mut self.out, OP_MESSAGE, &content)
    }

    /// Writes the data-end record, the footer and the closing magic.
    pub fn finish(mut self) -> io::Result<W> {
        // A zero data-section CRC means "not computed".
        write_record(&mut self.out, OP_DATA_END, &0_u32.to_le_bytes())?;
        // No summary section: summary_start, summary_offset_start and summary_crc are zero.
        write_record(&mut self.out, OP_FOOTER, &[0_u8; 20])?;
        self.out.write_all(MAGIC)?;
        self.out.flush()?;
        Ok(self.out)
    }
}

fn too_many<T>(_: T) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        "too many MCAP channels or schemas",
    )
}

fn write_record<W: Write>(out: &mut W, opcode: u8, content: &[u8]) -> io::Result<()> {
    out.write_all(&[opcode])?;
    out.write_all(&(content.len() as u64).to_le_bytes())?;
    out.write_all(content)
}

fn put_string(buffer: &mut Vec<u8>, value: &str) {
    put_bytes(buffer, value.as_bytes());
}

fn put_bytes(buffer: &mut Vec<u8>, value: &[u8]) {
    let length = u32::try_from(value.len()).expect("MCAP fields are limited to u32::MAX bytes");
    buffer.extend_from_slice(&length.to_le_bytes());
    buffer.extend_from_slice(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Splits the records between the leading and trailing magic.
    fn records(bytes: &[u8]) -> Vec<(u8, Vec<u8>)> {
        assert_eq!(&bytes[..8], MAGIC);
        assert_eq!(&bytes[bytes.len() - 8..], MAGIC);
        let body = &bytes[8..bytes.len() - 8];
        let mut offset = 0;
        let mut out = Vec::new();
        while offset < body.len() {
            let opcode = body[offset];
            let length =
                u64::from_le_bytes(body[offset + 1..offset + 9].try_into().unwrap()) as usize;
            out.push((opcode, body[offset + 9..offset + 9 + length].to_vec()));
            offset += 9 + length;
        }
        assert_eq!(offset, body.len());
        out
    }

    #[test]
    fn writes_header_schema_channels_messages_and_footer() {
        let mut writer = McapWriter::new(Vec::new()).unwrap();
        let imu = writer.add_channel("/imu", IMU_SCHEMA, false).unwrap();
        let imu2 = writer.add_channel("/imu2", IMU_SCHEMA, false).unwrap();
        let tf = writer
            .add_channel("/tf_static", TF_MESSAGE_SCHEMA, true)
            .unwrap();
        writer.write_message(tf, 5, 5, &[1, 2]).unwrap();
        writer.write_message(imu, 10, 9, &[3]).unwrap();
        writer.write_message(imu, 20, 19, &[4]).unwrap();
        writer.write_message(imu2, 20, 19, &[5]).unwrap();
        let bytes = writer.finish().unwrap();

        let opcodes: Vec<u8> = records(&bytes).iter().map(|(op, _)| *op).collect();
        // One schema per message type, one channel per topic.
        assert_eq!(
            opcodes,
            vec![
                OP_HEADER,
                OP_SCHEMA,
                OP_CHANNEL,
                OP_CHANNEL,
                OP_SCHEMA,
                OP_CHANNEL,
                OP_MESSAGE,
                OP_MESSAGE,
                OP_MESSAGE,
                OP_MESSAGE,
                OP_DATA_END,
                OP_FOOTER
            ]
        );
        let all = records(&bytes);
        // The second /imu message: channel 0, sequence 1, log time 20, payload [4].
        let (_, message) = &all[8];
        assert_eq!(&message[..2], &0_u16.to_le_bytes());
        assert_eq!(&message[2..6], &1_u32.to_le_bytes());
        assert_eq!(&message[6..14], &20_u64.to_le_bytes());
        assert_eq!(&message[22..], &[4]);
        // The latched channel carries its QoS profile.
        let (_, tf_channel) = &all[5];
        let text = String::from_utf8_lossy(tf_channel);
        assert!(text.contains("/tf_static"));
        assert!(text.contains("offered_qos_profiles"));
        assert!(text.contains("durability: 1"));
    }

    #[test]
    fn schema_definitions_list_their_dependencies() {
        assert!(IMU_SCHEMA
            .definition
            .contains("MSG: geometry_msgs/Vector3\n"));
        assert!(POINTCLOUD2_SCHEMA
            .definition
            .contains("MSG: sensor_msgs/PointField\n"));
        assert!(TF_MESSAGE_SCHEMA
            .definition
            .contains("MSG: geometry_msgs/Transform\n"));
        assert!(TF_MESSAGE_SCHEMA.definition.contains(SEPARATOR));
    }

    #[test]
    fn unknown_channel_is_rejected() {
        let mut writer = McapWriter::new(Vec::new()).unwrap();
        let error = writer
            .write_message(McapChannelId(3), 0, 0, &[])
            .unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
    }
}
