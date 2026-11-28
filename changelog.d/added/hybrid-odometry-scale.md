- `rne_slam::HybridLocalizerConfig::odometry_scale_calibration` (off by default) and
  `HybridLocalizer::odometry_scale`: wherever RTK fixed fixes or LiDAR matches hold the
  estimate within 0.15 m, every 20 m of wheel travel is compared with how far the
  estimate moved, and the calibrated scale is applied to the odometry. Example 152
  turns it on (its wheels read 1 % long): through the 300 m tunnel with
  `--lidarloc` the error drops from 0.83 m RMSE (2.82 m max) to 0.04 m (0.27 m).
