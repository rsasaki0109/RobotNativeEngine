# rne_adapter_gnsspp

[libgnss++](https://github.com/rsasaki0109/gnssplusplus-library) solvers for
Robot Native Engine: single-point positioning, kinematic RTK against a base
station, and moving-baseline RTK between two antennas (heading), on the
ray-traced observations of `rne_gnss`.

```bash
scripts/build_gnsspp.sh          # pinned libgnss++ into target/gnsspp/src
cargo test --release -p rne_adapter_gnsspp
```

`build.rs` compiles `cpp/rne_gnsspp.cpp` (a C ABI shim) against the libgnss++
build in `target/gnsspp/src/build`, or in `RNE_GNSSPP_DIR` /
`RNE_GNSSPP_BUILD_DIR`, with Eigen from `EIGEN3_INCLUDE_DIR` (default
`/usr/include/eigen3`). Without a build, or with `RNE_GNSSPP_DISABLE=1`, the
crate compiles as a stub whose constructors return
`GnssppError::Unavailable`, and its tests skip.

```rust
use rne_adapter_gnsspp::{Navigation, Rtk, RtkConfig, RtkMode};
use rne_gnss::NavigationMessage;

let navigation = Navigation::new(&NavigationMessage::from_constellation(
    &constellation, config.epoch, 0.0, config.atmosphere.klobuchar,
))?;
let mut rtk = Rtk::with_config(&RtkConfig {
    preset: "low-cost".into(),
    ..RtkConfig::new(RtkMode::Kinematic)
})?;
rtk.set_base_position(base_ecef)?;
let solution = rtk.process(&rover_epoch, None, &base_epoch, Some(base_ecef), &navigation)?;
```

See [GNSS](../../../docs/GNSS.md) and example 151.
