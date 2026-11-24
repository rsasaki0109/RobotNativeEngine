//! Movable Gaussian splat actors inside a splat background.
//!
//! An actor is a small splat (a car, a bus, a pedestrian) authored in its own frame. It shares
//! the background's GPU buffer, so the viewer sorts its Gaussians together with the scene's and
//! occlusion against the scene is correct in both directions. Moving an actor rewrites only its
//! range of that buffer.
//!
//! Rigid motion with uniform scale only. View-dependent colour (spherical harmonics beyond the
//! base colour) is dropped for actors, because the harmonics would have to be rotated with them.

use glam::{Quat as GQuat, Vec3 as GVec3};
use rne_math::Transform3;
use wgpu_3dgs_viewer::Gaussian;

/// Where hidden actors are parked in the background's frame: far below any scene.
const HIDDEN_OFFSET_M: f32 = -1.0e5;

/// An actor's Gaussians in its own frame, with view-dependent colour removed.
#[derive(Clone, Debug, PartialEq)]
pub struct SplatActorAsset {
    gaussians: Vec<Gaussian>,
}

impl SplatActorAsset {
    /// Wraps actor-frame Gaussians. Spherical-harmonic coefficients are cleared.
    #[must_use]
    pub fn new(mut gaussians: Vec<Gaussian>) -> Self {
        for gaussian in &mut gaussians {
            gaussian.sh = [GVec3::ZERO; 15];
        }
        Self { gaussians }
    }

    /// Number of Gaussians.
    #[must_use]
    pub fn len(&self) -> usize {
        self.gaussians.len()
    }

    /// Whether the asset has no Gaussians.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.gaussians.is_empty()
    }

    /// Actor-frame Gaussians.
    #[must_use]
    pub fn gaussians(&self) -> &[Gaussian] {
        &self.gaussians
    }
}

/// The actor's Gaussians placed by `frame_from_actor` (rigid, uniform scale from `scale.x`).
#[must_use]
pub fn place_actor_gaussians(
    asset: &SplatActorAsset,
    frame_from_actor: &Transform3,
) -> Vec<Gaussian> {
    let rotation = GQuat::from_xyzw(
        frame_from_actor.rotation.x as f32,
        frame_from_actor.rotation.y as f32,
        frame_from_actor.rotation.z as f32,
        frame_from_actor.rotation.w as f32,
    )
    .normalize();
    let translation = GVec3::new(
        frame_from_actor.translation.x as f32,
        frame_from_actor.translation.y as f32,
        frame_from_actor.translation.z as f32,
    );
    let scale = frame_from_actor.scale.x as f32;
    asset
        .gaussians
        .iter()
        .map(|g| Gaussian {
            rotation: (rotation * g.rotation).normalize(),
            pos: rotation * (g.pos * scale) + translation,
            scale: g.scale * scale,
            ..*g
        })
        .collect()
}

/// The actor's Gaussians moved out of sight (same count, so the buffer range is rewritten).
#[must_use]
pub fn hidden_actor_gaussians(asset: &SplatActorAsset) -> Vec<Gaussian> {
    asset
        .gaussians
        .iter()
        .map(|g| Gaussian {
            pos: g.pos + GVec3::new(0.0, 0.0, HIDDEN_OFFSET_M),
            ..*g
        })
        .collect()
}

/// Where each actor lives in the shared buffer: after the background, in insertion order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct ActorLayout {
    ranges: Vec<(usize, usize)>,
}

impl ActorLayout {
    pub(crate) fn new(background_len: usize, actor_lens: &[usize]) -> Self {
        let mut start = background_len;
        let ranges = actor_lens
            .iter()
            .map(|&len| {
                let range = (start, len);
                start += len;
                range
            })
            .collect();
        Self { ranges }
    }

    /// `(first index, count)` of actor `index`.
    pub(crate) fn range(&self, index: usize) -> Option<(usize, usize)> {
        self.ranges.get(index).copied()
    }

    pub(crate) fn len(&self) -> usize {
        self.ranges.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::U8Vec4;
    use rne_math::{Quat, Vec3};

    fn gaussian(pos: GVec3, sh: f32) -> Gaussian {
        Gaussian {
            rotation: GQuat::IDENTITY,
            pos,
            color: U8Vec4::new(200, 30, 20, 255),
            sh: [GVec3::splat(sh); 15],
            scale: GVec3::new(0.1, 0.2, 0.3),
        }
    }

    #[test]
    fn asset_drops_view_dependent_colour() {
        let asset = SplatActorAsset::new(vec![gaussian(GVec3::ZERO, 0.7)]);
        assert_eq!(asset.gaussians()[0].sh, [GVec3::ZERO; 15]);
        assert_eq!(asset.gaussians()[0].color, U8Vec4::new(200, 30, 20, 255));
        assert_eq!(asset.len(), 1);
    }

    #[test]
    fn placing_rotates_translates_and_scales() {
        let asset = SplatActorAsset::new(vec![gaussian(GVec3::new(1.0, 0.0, 0.0), 0.0)]);
        let pose = Transform3 {
            translation: Vec3::new(10.0, 0.0, -2.0),
            rotation: Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: Vec3::splat(2.0),
        };
        let placed = place_actor_gaussians(&asset, &pose);
        // +X rotated 90 deg about +Y is -Z; scaled by 2, then translated.
        assert!((placed[0].pos - GVec3::new(10.0, 0.0, -4.0)).length() < 1.0e-5);
        assert!((placed[0].scale - GVec3::new(0.2, 0.4, 0.6)).length() < 1.0e-6);
        let expected = GQuat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        assert!(placed[0].rotation.abs_diff_eq(expected, 1.0e-6));
    }

    #[test]
    fn hidden_actor_keeps_its_count_far_away() {
        let asset = SplatActorAsset::new(vec![gaussian(GVec3::ZERO, 0.0); 3]);
        let hidden = hidden_actor_gaussians(&asset);
        assert_eq!(hidden.len(), 3);
        assert!(hidden.iter().all(|g| g.pos.z < -1.0e4));
    }

    #[test]
    fn layout_puts_actors_after_the_background() {
        let layout = ActorLayout::new(100, &[5, 7]);
        assert_eq!(layout.range(0), Some((100, 5)));
        assert_eq!(layout.range(1), Some((105, 7)));
        assert_eq!(layout.range(2), None);
        assert_eq!(layout.len(), 2);
    }
}
