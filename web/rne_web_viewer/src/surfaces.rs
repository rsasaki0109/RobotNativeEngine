//! Deterministic, tileable concrete material maps generated once at scene load.
use rne_render::{ImageFrame, PbrMaterial};
use std::sync::Arc;

const SIZE: u32 = 128;

#[derive(Debug)]
pub(crate) struct Concrete {
    pub(crate) color: Arc<ImageFrame>,
    pub(crate) material: PbrMaterial,
}

// Periodic value noise keeps both the color and height gradients continuous
// across texture edges. The integer hash is a fixed seed, not runtime randomness.
fn noise(x: f64, y: f64, cells: u32) -> f64 {
    let hash = |x: u32, y: u32| {
        let mut n = (x % cells).wrapping_mul(0x1f123bb5)
            ^ (y % cells).wrapping_mul(0x5f356495)
            ^ 0x725e1a93;
        n ^= n >> 16;
        n = n.wrapping_mul(0x45d9f3b);
        f64::from(n ^ (n >> 16)) / f64::from(u32::MAX)
    };
    let x = x.rem_euclid(1.0) * f64::from(cells);
    let y = y.rem_euclid(1.0) * f64::from(cells);
    let ix = x.floor() as u32;
    let iy = y.floor() as u32;
    let smooth = |v: f64| v * v * (3.0 - 2.0 * v);
    let u = smooth(x.fract());
    let v = smooth(y.fract());
    let a = hash(ix, iy) * (1.0 - u) + hash(ix + 1, iy) * u;
    let b = hash(ix, iy + 1) * (1.0 - u) + hash(ix + 1, iy + 1) * u;
    a * (1.0 - v) + b * v
}

fn height(x: f64, y: f64) -> f64 {
    0.55 * noise(x, y, 4) + 0.30 * noise(x, y, 16) + 0.15 * noise(x, y, 48)
}

pub(crate) fn concrete() -> Concrete {
    let mut color = Vec::with_capacity((SIZE * SIZE * 4) as usize);
    let mut normal = Vec::with_capacity(color.capacity());
    let mut roughness = Vec::with_capacity(color.capacity());
    let step = 1.0 / f64::from(SIZE);
    for y in 0..SIZE {
        for x in 0..SIZE {
            let (u, v) = (f64::from(x) * step, f64::from(y) * step);
            let h = height(u, v);
            let grain = noise(u, v, 64);
            let shade = (170.0 + 27.0 * h + 5.0 * grain) as u8;
            color.extend_from_slice(&[shade, shade - 2, shade - 6, 255]);
            let dx = (height(u + step, v) - height(u - step, v)) * 2.0;
            let dy = (height(u, v + step) - height(u, v - step)) * 2.0;
            let length = (dx * dx + dy * dy + 1.0).sqrt();
            let encode = |value: f64| ((value / length * 0.5 + 0.5) * 255.0).round() as u8;
            normal.extend_from_slice(&[encode(-dx), encode(-dy), encode(1.0), 255]);
            let rough = (192.0 + h * 42.0) as u8;
            roughness.extend_from_slice(&[rough, rough, rough, 255]);
        }
    }
    Concrete {
        color: Arc::new(ImageFrame::from_rgba8(SIZE, SIZE, color)),
        material: PbrMaterial::default()
            .with_texture_maps(
                Some(Arc::new(ImageFrame::from_rgba8(SIZE, SIZE, normal))),
                Some(Arc::new(ImageFrame::from_rgba8(SIZE, SIZE, roughness))),
            )
            .with_normal_strength(0.45),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn concrete_maps_are_repeatable_periodic_and_physically_bounded() {
        let a = concrete();
        let b = concrete();
        assert_eq!(a.color, b.color);
        assert_eq!(a.material, b.material);
        assert_ne!(&a.color.rgba8[..4], &a.color.rgba8[4..8]);
        for cells in [4, 16, 48, 64] {
            assert!((noise(0.0, 0.37, cells) - noise(1.0, 0.37, cells)).abs() < 1e-12);
            assert!((noise(0.29, 0.0, cells) - noise(0.29, 1.0, cells)).abs() < 1e-12);
        }
        for pixel in a
            .material
            .normal_texture
            .as_ref()
            .unwrap()
            .rgba8
            .chunks_exact(4)
        {
            let xyz = [pixel[0], pixel[1], pixel[2]].map(|v| f64::from(v) / 255.0 * 2.0 - 1.0);
            let length = xyz.into_iter().map(|v| v * v).sum::<f64>().sqrt();
            assert!((length - 1.0).abs() < 0.015);
            assert!(xyz[2] > 0.9, "floor relief must stay shallow");
        }
        assert!(a
            .material
            .roughness_texture
            .as_ref()
            .unwrap()
            .rgba8
            .chunks_exact(4)
            .all(|p| (192..=234).contains(&p[0])));
    }
}
