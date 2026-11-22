//! Builds the C++ shim against a libgnss++ build when one is available.
//!
//! libgnss++ is found from `RNE_GNSSPP_DIR` (a source checkout; its CMake
//! build directory is `RNE_GNSSPP_BUILD_DIR`, default `<dir>/build`), or from
//! `target/gnsspp/src` as written by `scripts/build_gnsspp.sh`. Without it,
//! the crate still builds and every solver reports `GnssppError::Unavailable`,
//! so the workspace never requires a C++ toolchain. `RNE_GNSSPP_DISABLE=1`
//! forces that stub build.

use std::env;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo::rustc-check-cfg=cfg(gnsspp)");
    println!("cargo:rerun-if-env-changed=RNE_GNSSPP_DIR");
    println!("cargo:rerun-if-env-changed=RNE_GNSSPP_BUILD_DIR");
    println!("cargo:rerun-if-env-changed=EIGEN3_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=RNE_GNSSPP_DISABLE");
    if env::var_os("RNE_GNSSPP_DISABLE").is_some_and(|v| v != "0") {
        return;
    }
    println!("cargo:rerun-if-changed=cpp/rne_gnsspp.cpp");
    println!("cargo:rerun-if-changed=cpp/rne_gnsspp.h");

    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let workspace_default = manifest.join("../../../target/gnsspp/src");
    let source = env::var_os("RNE_GNSSPP_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            workspace_default
                .join("include/libgnss++/gnss.hpp")
                .exists()
                .then_some(workspace_default)
        });
    let Some(source) = source else {
        return;
    };
    let build = env::var_os("RNE_GNSSPP_BUILD_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| source.join("build"));
    let libraries = [
        "libgnss_rtk_runtime.a",
        "libgnss_lib.a",
        "third_party/sofa/libsofa.a",
        "third_party/ginan-iers2010/libginan_iers2010.a",
    ];
    if !libraries.iter().all(|lib| build.join(lib).exists()) {
        println!(
            "cargo:warning=libgnss++ sources found at {} but its libraries are not built in {}; run scripts/build_gnsspp.sh",
            source.display(),
            build.display()
        );
        return;
    }
    let eigen = env::var_os("EIGEN3_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/usr/include/eigen3"));

    cc::Build::new()
        .cpp(true)
        .std("c++20")
        .file("cpp/rne_gnsspp.cpp")
        .include(source.join("include"))
        .include(&eigen)
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-deprecated-declarations")
        .warnings(false)
        .compile("rne_gnsspp_shim");

    link_static(&build, "gnss_rtk_runtime");
    link_static(&build, "gnss_lib");
    link_static(&build.join("third_party/sofa"), "sofa");
    link_static(&build.join("third_party/ginan-iers2010"), "ginan_iers2010");
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-cfg=gnsspp");
}

fn link_static(dir: &Path, name: &str) {
    println!("cargo:rustc-link-search=native={}", dir.display());
    println!("cargo:rustc-link-lib=static={name}");
}
