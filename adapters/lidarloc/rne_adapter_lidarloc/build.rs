//! Links lidar_localization_core (lidar_localization_ros2's registration,
//! built without ROS 2) when one is available.
//!
//! The core is found from `RNE_LIDARLOC_DIR` (an install prefix with
//! `lib/liblidar_localization_core.a` and `include/lidar_localization/`), or
//! from `target/lidarloc/install` as written by `scripts/build_lidarloc.sh`.
//! Without it the crate still builds and `LidarLoc::new` returns
//! `LidarLocError::Unavailable`, so the workspace never requires PCL.
//! `RNE_LIDARLOC_DISABLE=1` forces that stub build.

use std::env;
use std::path::PathBuf;

/// The PCL components the core's registration uses.
const PCL: [&str; 8] = [
    "pcl_registration",
    "pcl_features",
    "pcl_filters",
    "pcl_sample_consensus",
    "pcl_search",
    "pcl_kdtree",
    "pcl_octree",
    "pcl_common",
];

fn main() {
    println!("cargo::rustc-check-cfg=cfg(lidarloc)");
    println!("cargo:rerun-if-env-changed=RNE_LIDARLOC_DIR");
    println!("cargo:rerun-if-env-changed=RNE_LIDARLOC_DISABLE");
    println!("cargo:rerun-if-env-changed=PCL_LIBRARY_DIR");
    if env::var_os("RNE_LIDARLOC_DISABLE").is_some_and(|v| v != "0") {
        return;
    }
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let prefix = env::var_os("RNE_LIDARLOC_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| manifest.join("../../../target/lidarloc/install"));
    let library = prefix.join("lib/liblidar_localization_core.a");
    println!("cargo:rerun-if-changed={}", library.display());
    if !library.exists() {
        return;
    }
    println!(
        "cargo:rustc-link-search=native={}",
        prefix.join("lib").display()
    );
    println!("cargo:rustc-link-lib=static=lidar_localization_core");
    if let Some(dir) = env::var_os("PCL_LIBRARY_DIR") {
        println!(
            "cargo:rustc-link-search=native={}",
            PathBuf::from(dir).display()
        );
    }
    for lib in PCL {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    for lib in ["flann_cpp", "lz4", "gomp", "stdc++"] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    println!("cargo:rustc-cfg=lidarloc");
}
