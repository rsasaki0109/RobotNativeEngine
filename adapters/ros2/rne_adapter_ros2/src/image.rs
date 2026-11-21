//! ROS2 `sensor_msgs/Image` mapping.

use rne_core::SimTime;
use rne_data::ImageRgb8;

use crate::clock::to_ros_time;
use crate::messages::{RosHeader, RosImage};

/// Maps a `DataBus` [`ImageRgb8`] payload to `sensor_msgs/Image`.
pub fn to_ros_image(image: &ImageRgb8, sim_time: SimTime, frame_id: &str) -> RosImage {
    let width = image.width;
    let height = image.height;
    let step = 4 * width;
    RosImage {
        header: RosHeader {
            stamp: to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height,
        width,
        encoding: "rgba8".to_string(),
        is_bigendian: false,
        step,
        data: image.rgba8.clone(),
    }
}

/// Maps a `DataBus` [`ImageRgb8`] payload to a `mono8` `sensor_msgs/Image`.
///
/// Each pixel becomes its Rec. 601 luma, `0.299 R + 0.587 G + 0.114 B`, rounded;
/// alpha is dropped. Calibration tools that track features read grayscale, and
/// the message is a quarter the size of `rgba8`.
pub fn to_ros_mono8_image(image: &ImageRgb8, sim_time: SimTime, frame_id: &str) -> RosImage {
    let data = image
        .rgba8
        .chunks_exact(4)
        .map(|pixel| {
            let luma = 0.299 * f64::from(pixel[0])
                + 0.587 * f64::from(pixel[1])
                + 0.114 * f64::from(pixel[2]);
            luma.round().clamp(0.0, 255.0) as u8
        })
        .collect();
    RosImage {
        header: RosHeader {
            stamp: to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height: image.height,
        width: image.width,
        encoding: "mono8".to_string(),
        is_bigendian: false,
        step: image.width,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn image_preserves_dimensions_and_payload() {
        let image = ImageRgb8::from_rgba8(8, 6, vec![1; 8 * 6 * 4]);
        let ros = to_ros_image(&image, SimTime::from_ticks(1_000_000_000), "wrist_camera");
        assert_eq!(ros.width, 8);
        assert_eq!(ros.height, 6);
        assert_eq!(ros.step, 32);
        assert_eq!(ros.encoding, "rgba8");
        assert_eq!(ros.data.len(), 8 * 6 * 4);
        assert_eq!(ros.header.frame_id, "wrist_camera");
    }

    #[test]
    fn mono8_image_uses_rec601_luma() {
        let image =
            ImageRgb8::from_rgba8(3, 1, vec![255, 0, 0, 255, 0, 255, 0, 255, 255, 255, 255, 0]);
        let ros = to_ros_mono8_image(&image, SimTime::from_ticks(0), "camera");
        assert_eq!(ros.encoding, "mono8");
        assert_eq!(ros.step, 3);
        assert_eq!(ros.data, vec![76, 150, 255]);
    }
}
