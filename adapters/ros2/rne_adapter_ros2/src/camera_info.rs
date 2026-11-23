//! `sensor_msgs/CameraInfo` mapping for RNE's pinhole camera, with or without lens
//! distortion.

use crate::clock::to_ros_time;
use crate::messages::{RosCameraInfo, RosHeader, RosRegionOfInterest};
use rne_core::SimTime;
use rne_sensor::CameraDistortion;

/// `CameraInfo` of an undistorted pinhole camera with RNE's render geometry.
///
/// RNE renders square pixels with a vertical field of view `fov_y_rad` and the
/// principal point at the image center. With ROS's convention that pixel
/// centers sit at integer coordinates, that gives
/// `fx = fy = (height / 2) / tan(fov_y / 2)`, `cx = width / 2 - 0.5` and
/// `cy = height / 2 - 0.5`. The distortion model is `plumb_bob` with zero
/// coefficients, so it only matches frames rendered without lens distortion; see
/// [`camera_info`] for a distorted camera.
pub fn pinhole_camera_info(
    width: u32,
    height: u32,
    fov_y_rad: f64,
    sim_time: SimTime,
    frame_id: &str,
) -> RosCameraInfo {
    camera_info(
        width,
        height,
        fov_y_rad,
        &CameraDistortion::default(),
        sim_time,
        frame_id,
    )
}

/// `CameraInfo` of an RNE camera with lens distortion.
///
/// The intrinsics are those of [`pinhole_camera_info`]; `D` is the `plumb_bob`
/// vector `[k1, k2, p1, p2, k3]` of [`CameraDistortion`], whose coefficients have
/// OpenCV's and ROS's meaning, so the message describes the frames RNE renders.
pub fn camera_info(
    width: u32,
    height: u32,
    fov_y_rad: f64,
    distortion: &CameraDistortion,
    sim_time: SimTime,
    frame_id: &str,
) -> RosCameraInfo {
    let f = (f64::from(height) / 2.0) / (fov_y_rad / 2.0).tan();
    let cx = f64::from(width) / 2.0 - 0.5;
    let cy = f64::from(height) / 2.0 - 0.5;
    RosCameraInfo {
        header: RosHeader {
            stamp: to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height,
        width,
        distortion_model: "plumb_bob".to_string(),
        d: vec![
            distortion.k1,
            distortion.k2,
            distortion.p1,
            distortion.p2,
            distortion.k3,
        ],
        k: [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: RosRegionOfInterest::default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ninety_degree_fov_puts_the_focal_length_at_half_the_height() {
        let info = pinhole_camera_info(
            640,
            480,
            std::f64::consts::FRAC_PI_2,
            SimTime::from_ticks(0),
            "camera",
        );
        assert!((info.k[0] - 240.0).abs() < 1e-9);
        assert_eq!(info.k[0], info.k[4]);
        assert_eq!(info.k[2], 319.5);
        assert_eq!(info.k[5], 239.5);
        assert_eq!(info.p[0], info.k[0]);
        assert_eq!(info.d, vec![0.0; 5]);
    }

    #[test]
    fn a_distorted_camera_carries_its_plumb_bob_coefficients() {
        let distortion = CameraDistortion {
            k1: -0.12,
            k2: 0.03,
            k3: -0.004,
            p1: 0.001,
            p2: -0.002,
        };
        let info = camera_info(
            480,
            360,
            std::f64::consts::FRAC_PI_2,
            &distortion,
            SimTime::from_ticks(0),
            "camera",
        );
        assert_eq!(info.distortion_model, "plumb_bob");
        assert_eq!(info.d, vec![-0.12, 0.03, 0.001, -0.002, -0.004]);
        assert_eq!(info.k[2], 239.5);
    }
}
