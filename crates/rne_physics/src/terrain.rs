//! Seeded fractal terrain for [`ColliderShape::HeightField`].
//!
//! RaiSim ships a terrain generator that turns a handful of fractal-noise
//! parameters into a height map, and legged-locomotion training leans on it to
//! randomize the ground every episode. [`FractalTerrain`] is the
//! backend-neutral equivalent: fractional Brownian motion over 2D gradient
//! (Perlin) noise, optionally quantized into steps, sampled onto the grid that
//! [`ColliderShape::HeightField`] already describes. The same spec and seed
//! always produce the same heights, so a terrain is reproducible from two small
//! values instead of a stored height map.
//!
//! [`height_field_surface`] samples a height field at a point and returns the
//! surface height and normal, which is what a contact model needs to place
//! point contacts on the terrain without a physics backend.

use crate::backend::PhysicsError;
use crate::components::ColliderShape;
use rne_math::Vec3;
use std::sync::Arc;

/// Fractal noise terrain specification.
///
/// The field spans `size_x_m` by `size_z_m` on the collider's local XZ plane,
/// centered on its origin, with `columns` samples along X and `rows` samples
/// along Z, matching [`ColliderShape::HeightField`]. Each sample is
///
/// ```text
/// h = height_offset_m + amplitude_m * Σₖ gainᵏ · noise(frequency · lacunarityᵏ · p) / Σₖ gainᵏ
/// ```
///
/// rounded to a multiple of `step_m` when it is positive, which produces
/// RaiSim-style stair terrain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FractalTerrain {
    /// Extent of the field along local X, in meters.
    pub size_x_m: f64,
    /// Extent of the field along local Z, in meters.
    pub size_z_m: f64,
    /// Number of samples along X (at least 2).
    pub columns: u32,
    /// Number of samples along Z (at least 2).
    pub rows: u32,
    /// Spatial frequency of the first octave, in cycles per meter.
    pub frequency_per_m: f64,
    /// Peak height of the normalized noise, in meters.
    pub amplitude_m: f64,
    /// Number of noise octaves (at least 1).
    pub octaves: u32,
    /// Frequency multiplier between successive octaves.
    pub lacunarity: f64,
    /// Amplitude multiplier between successive octaves.
    pub gain: f64,
    /// Height quantization step in meters; zero keeps the surface smooth.
    pub step_m: f64,
    /// Constant height added to every sample, in meters.
    pub height_offset_m: f64,
}

impl Default for FractalTerrain {
    fn default() -> Self {
        Self {
            size_x_m: 10.0,
            size_z_m: 10.0,
            columns: 101,
            rows: 101,
            frequency_per_m: 0.3,
            amplitude_m: 0.15,
            octaves: 4,
            lacunarity: 2.0,
            gain: 0.5,
            step_m: 0.0,
            height_offset_m: 0.0,
        }
    }
}

impl FractalTerrain {
    /// Generates the row-major heights, in meters, for `seed`.
    ///
    /// Row `r` lies at `z = -size_z_m / 2 + r * size_z_m / (rows - 1)` and
    /// column `c` at the matching `x`. Pass a seed derived from the scene's
    /// world random source so terrain stays reproducible with the episode.
    pub fn heights_m(&self, seed: u64) -> Result<Vec<f64>, PhysicsError> {
        self.validate()?;
        let noise = GradientNoise::new(seed);
        let mut amplitude_sum = 0.0;
        let mut amplitude = 1.0;
        for _ in 0..self.octaves {
            amplitude_sum += amplitude;
            amplitude *= self.gain;
        }
        let dx = self.size_x_m / f64::from(self.columns - 1);
        let dz = self.size_z_m / f64::from(self.rows - 1);
        let mut heights = Vec::with_capacity(self.rows as usize * self.columns as usize);
        for row in 0..self.rows {
            let z = -0.5 * self.size_z_m + f64::from(row) * dz;
            for column in 0..self.columns {
                let x = -0.5 * self.size_x_m + f64::from(column) * dx;
                let mut value = 0.0;
                let mut amplitude = 1.0;
                let mut frequency = self.frequency_per_m;
                for _ in 0..self.octaves {
                    value += amplitude * noise.sample(x * frequency, z * frequency);
                    amplitude *= self.gain;
                    frequency *= self.lacunarity;
                }
                let mut height = self.amplitude_m * value / amplitude_sum;
                if self.step_m > 0.0 {
                    height = (height / self.step_m).round() * self.step_m;
                }
                heights.push(self.height_offset_m + height);
            }
        }
        Ok(heights)
    }

    /// Generates a [`ColliderShape::HeightField`] for `seed`.
    pub fn height_field(&self, seed: u64) -> Result<ColliderShape, PhysicsError> {
        let heights = self.heights_m(seed)?;
        Ok(ColliderShape::HeightField {
            nrows: self.rows,
            ncols: self.columns,
            heights_m: Arc::from(heights),
            scale: Vec3::new(self.size_x_m, 1.0, self.size_z_m),
        })
    }

    fn validate(&self) -> Result<(), PhysicsError> {
        let invalid = |reason| Err(PhysicsError::InvalidColliderShape { reason });
        if self.rows < 2 || self.columns < 2 {
            return invalid("fractal terrain requires at least 2 rows and 2 columns");
        }
        if !(self.size_x_m.is_finite()
            && self.size_z_m.is_finite()
            && self.size_x_m > 0.0
            && self.size_z_m > 0.0)
        {
            return invalid("fractal terrain size must be finite and positive");
        }
        if self.octaves == 0 {
            return invalid("fractal terrain requires at least one octave");
        }
        let finite = [
            self.frequency_per_m,
            self.amplitude_m,
            self.lacunarity,
            self.gain,
            self.step_m,
            self.height_offset_m,
        ]
        .iter()
        .all(|value| value.is_finite());
        if !finite || self.gain <= 0.0 || self.lacunarity <= 0.0 || self.step_m < 0.0 {
            return invalid(
                "fractal terrain parameters must be finite, gain and lacunarity positive, and step non-negative",
            );
        }
        Ok(())
    }
}

/// Height and normal of a height-field surface at one point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeightFieldSurface {
    /// Surface height along local Y, in meters.
    pub height_m: f64,
    /// Unit surface normal in the collider's local frame.
    pub normal: Vec3,
}

/// Samples a [`ColliderShape::HeightField`] at local `(x, z)`.
///
/// The height is bilinear within each grid cell, and the normal is the
/// normalized bilinear gradient. Returns `None` for other shapes, malformed
/// fields, or points outside the field's footprint. Physics backends that
/// triangulate each cell may differ from this surface by at most the cell's
/// bilinear twist.
pub fn height_field_surface(
    shape: &ColliderShape,
    x_m: f64,
    z_m: f64,
) -> Option<HeightFieldSurface> {
    let ColliderShape::HeightField {
        nrows,
        ncols,
        heights_m,
        scale,
    } = shape
    else {
        return None;
    };
    let (rows, columns) = (*nrows as usize, *ncols as usize);
    if rows < 2 || columns < 2 || heights_m.len() != rows * columns {
        return None;
    }
    if !(x_m.is_finite() && z_m.is_finite()) {
        return None;
    }
    let dx = scale.x / (columns - 1) as f64;
    let dz = scale.z / (rows - 1) as f64;
    let u = (x_m + 0.5 * scale.x) / dx;
    let v = (z_m + 0.5 * scale.z) / dz;
    if u < 0.0 || v < 0.0 || u > (columns - 1) as f64 || v > (rows - 1) as f64 {
        return None;
    }
    let column = (u.floor() as usize).min(columns - 2);
    let row = (v.floor() as usize).min(rows - 2);
    let (fu, fv) = (u - column as f64, v - row as f64);
    let at = |r: usize, c: usize| heights_m[r * columns + c] * scale.y;
    let (h00, h01) = (at(row, column), at(row, column + 1));
    let (h10, h11) = (at(row + 1, column), at(row + 1, column + 1));
    let height_m = h00 * (1.0 - fu) * (1.0 - fv)
        + h01 * fu * (1.0 - fv)
        + h10 * (1.0 - fu) * fv
        + h11 * fu * fv;
    let slope_x = ((h01 - h00) * (1.0 - fv) + (h11 - h10) * fv) / dx;
    let slope_z = ((h10 - h00) * (1.0 - fu) + (h11 - h01) * fu) / dz;
    Some(HeightFieldSurface {
        height_m,
        normal: Vec3::new(-slope_x, 1.0, -slope_z).normalize(),
    })
}

/// Seeded 2D gradient noise with a 256-entry permutation table.
struct GradientNoise {
    permutation: [u8; 512],
}

impl GradientNoise {
    fn new(seed: u64) -> Self {
        let mut table: [u8; 256] = std::array::from_fn(|index| index as u8);
        let mut state = seed;
        for index in (1..256).rev() {
            let pick = (splitmix64(&mut state) % (index as u64 + 1)) as usize;
            table.swap(index, pick);
        }
        let permutation = std::array::from_fn(|index| table[index & 255]);
        Self { permutation }
    }

    /// Noise value in approximately `[-1, 1]`, zero at every lattice point.
    fn sample(&self, x: f64, z: f64) -> f64 {
        let (x0, z0) = (x.floor(), z.floor());
        let (fx, fz) = (x - x0, z - z0);
        let xi = (x0.rem_euclid(256.0)) as usize;
        let zi = (z0.rem_euclid(256.0)) as usize;
        let p = &self.permutation;
        let hash = |cx: usize, cz: usize| p[p[cx] as usize + cz];
        let n00 = gradient(hash(xi, zi), fx, fz);
        let n10 = gradient(hash(xi + 1, zi), fx - 1.0, fz);
        let n01 = gradient(hash(xi, zi + 1), fx, fz - 1.0);
        let n11 = gradient(hash(xi + 1, zi + 1), fx - 1.0, fz - 1.0);
        let (u, w) = (fade(fx), fade(fz));
        let near = n00 + u * (n10 - n00);
        let far = n01 + u * (n11 - n01);
        // Scale so the extreme of 2D gradient noise reaches about ±1.
        std::f64::consts::SQRT_2 * (near + w * (far - near))
    }
}

fn fade(t: f64) -> f64 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

fn gradient(hash: u8, x: f64, z: f64) -> f64 {
    // Eight unit directions spaced every 45 degrees.
    const DIAGONAL: f64 = std::f64::consts::FRAC_1_SQRT_2;
    match hash & 7 {
        0 => x,
        1 => -x,
        2 => z,
        3 => -z,
        4 => DIAGONAL * (x + z),
        5 => DIAGONAL * (x - z),
        6 => DIAGONAL * (-x + z),
        _ => DIAGONAL * (-x - z),
    }
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_reproduces_the_terrain_and_new_seeds_change_it() {
        let spec = FractalTerrain::default();
        let first = spec.heights_m(42).expect("terrain");
        assert_eq!(first, spec.heights_m(42).expect("terrain"));
        assert_ne!(first, spec.heights_m(43).expect("terrain"));
        assert_eq!(first.len(), 101 * 101);
    }

    #[test]
    fn heights_stay_within_the_amplitude_and_vary() {
        let spec = FractalTerrain {
            height_offset_m: 0.3,
            ..FractalTerrain::default()
        };
        let heights = spec.heights_m(7).expect("terrain");
        let min = heights.iter().copied().fold(f64::INFINITY, f64::min);
        let max = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(min >= 0.3 - spec.amplitude_m - 1.0e-12, "min {min}");
        assert!(max <= 0.3 + spec.amplitude_m + 1.0e-12, "max {max}");
        assert!(
            max - min > 0.2 * spec.amplitude_m,
            "flat terrain: {min}..{max}"
        );
    }

    #[test]
    fn stepped_terrain_is_quantized() {
        let spec = FractalTerrain {
            step_m: 0.05,
            ..FractalTerrain::default()
        };
        for height in spec.heights_m(3).expect("terrain") {
            let steps = height / 0.05;
            assert!((steps - steps.round()).abs() < 1.0e-9, "height {height}");
        }
    }

    #[test]
    fn invalid_specs_are_rejected() {
        for spec in [
            FractalTerrain {
                rows: 1,
                ..FractalTerrain::default()
            },
            FractalTerrain {
                octaves: 0,
                ..FractalTerrain::default()
            },
            FractalTerrain {
                size_x_m: -1.0,
                ..FractalTerrain::default()
            },
            FractalTerrain {
                gain: f64::NAN,
                ..FractalTerrain::default()
            },
        ] {
            assert!(matches!(
                spec.heights_m(0),
                Err(PhysicsError::InvalidColliderShape { .. })
            ));
        }
    }

    #[test]
    fn surface_sampling_recovers_a_plane_and_its_normal() {
        // h = 0.1 x - 0.2 z sampled on a 3 x 4 grid is reproduced exactly.
        let (rows, columns) = (4_u32, 3_u32);
        let (size_x, size_z) = (2.0, 3.0);
        let mut heights = Vec::new();
        for row in 0..rows {
            let z = -0.5 * size_z + f64::from(row) * size_z / f64::from(rows - 1);
            for column in 0..columns {
                let x = -0.5 * size_x + f64::from(column) * size_x / f64::from(columns - 1);
                heights.push(0.1 * x - 0.2 * z);
            }
        }
        let shape = ColliderShape::HeightField {
            nrows: rows,
            ncols: columns,
            heights_m: Arc::from(heights),
            scale: Vec3::new(size_x, 1.0, size_z),
        };
        let surface = height_field_surface(&shape, 0.37, -0.81).expect("inside");
        assert!((surface.height_m - (0.1 * 0.37 + 0.2 * 0.81)).abs() < 1.0e-12);
        let expected = Vec3::new(-0.1, 1.0, 0.2).normalize();
        assert!((surface.normal - expected).length() < 1.0e-12);
        assert!(height_field_surface(&shape, 1.5, 0.0).is_none());
        assert!(height_field_surface(&ColliderShape::Sphere { radius_m: 1.0 }, 0.0, 0.0).is_none());
    }

    #[test]
    fn generated_field_matches_sampled_heights_at_grid_points() {
        let spec = FractalTerrain {
            columns: 11,
            rows: 9,
            ..FractalTerrain::default()
        };
        let shape = spec.height_field(5).expect("field");
        let heights = spec.heights_m(5).expect("heights");
        let x = -0.5 * spec.size_x_m + 3.0 * spec.size_x_m / 10.0;
        let z = -0.5 * spec.size_z_m + 4.0 * spec.size_z_m / 8.0;
        let surface = height_field_surface(&shape, x, z).expect("inside");
        assert!((surface.height_m - heights[4 * 11 + 3]).abs() < 1.0e-12);
    }
}
