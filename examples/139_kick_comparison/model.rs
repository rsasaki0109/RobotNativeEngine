//! Physical-model checks for the dedicated Rapier kick scenes.

use rne_ai::UrdfSceneSim;
use rne_math::Vec3;
use rne_physics::{
    Collider, CompoundCollider, FixedJointDesc, MultibodyLink, RigidBody, RigidBodyInertia,
    RigidBodyType,
};
use rne_robot::{Joint, JointKind, Link};
use rne_world::{world_transform_of, Transform3};
use serde_json::{json, Value};
use std::{error::Error, io};

type ModelResult<T> = Result<T, Box<dyn Error>>;

const FIXED_POSITION_TOLERANCE_M: f64 = 1.0e-4;
const FIXED_ROTATION_TOLERANCE_RAD: f64 = 5.0e-4;
const MASS_TOLERANCE_KG: f64 = 1.0e-8;

fn require(condition: bool, message: impl Into<String>) -> ModelResult<()> {
    if condition {
        Ok(())
    } else {
        Err(io::Error::other(message.into()).into())
    }
}

fn validate_transform(transform: Transform3, name: &str) -> ModelResult<()> {
    require(
        transform.translation.is_finite()
            && transform.rotation.is_finite()
            && (transform.rotation.length_squared() - 1.0).abs() <= 1.0e-5
            && transform.scale.is_finite()
            && (transform.scale - Vec3::ONE).length() <= 1.0e-6,
        format!("link {name} has a non-finite, non-unit, or scaled physical pose"),
    )
}

/// Returns robot COM position, COM velocity, and total mass in link-name order.
///
/// Only dynamic bodies carrying a robot `Link` contribute; ground, scene objects,
/// and a possible impactor are excluded. Every contribution requires exact
/// declared inertia. This helper is specific to the example's Rapier plant:
/// `sync_to_ecs` copies Rapier `linvel`, which is already the body COM velocity.
/// Adding `angular_velocity.cross(rotated_local_com)` would count the offset
/// twice. The position still includes each link's rotated declared COM offset.
pub(super) fn center_of_mass(sim: &UrdfSceneSim) -> ModelResult<(Vec3, Vec3, f64)> {
    let mut links: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            let link = entity.get::<Link>()?;
            let body = entity.get::<RigidBody>()?;
            (body.body_type == RigidBodyType::Dynamic).then_some((link.name.as_str(), entity.id()))
        })
        .collect();
    links.sort_unstable_by(|a, b| a.0.cmp(b.0));
    require(!links.is_empty(), "the plant has no dynamic robot links")?;
    require(
        links.windows(2).all(|pair| pair[0].0 != pair[1].0),
        "the plant has duplicate robot link names",
    )?;

    let mut weighted_position_m_kg = Vec3::ZERO;
    let mut weighted_velocity_kg_m_s = Vec3::ZERO;
    let mut total_mass_kg = 0.0;
    for (name, entity) in links {
        let body = sim
            .world()
            .get::<RigidBody>(entity)
            .expect("collected body");
        require(
            body.mass_kg.is_finite()
                && body.mass_kg > 0.0
                && (body.mass_kg as f32).is_finite()
                && (body.mass_kg as f32) > 0.0
                && body.linear_velocity_m_s.is_finite()
                && body.angular_velocity_rad_s.is_finite(),
            format!("link {name} has invalid mass or velocity"),
        )?;
        let inertia = sim.world().get::<RigidBodyInertia>(entity).ok_or_else(|| {
            io::Error::other(format!("link {name} has no exact declared inertia"))
        })?;
        require(
            inertia.is_valid(),
            format!("link {name} has invalid declared inertia"),
        )?;
        let transform = world_transform_of(sim.world(), entity);
        validate_transform(transform, name)?;
        let position_m =
            transform.translation + transform.rotation * inertia.center_of_mass_local_m;
        weighted_position_m_kg += position_m * body.mass_kg;
        weighted_velocity_kg_m_s += body.linear_velocity_m_s * body.mass_kg;
        total_mass_kg += body.mass_kg;
    }
    require(
        total_mass_kg.is_finite()
            && total_mass_kg > 0.0
            && weighted_position_m_kg.is_finite()
            && weighted_velocity_kg_m_s.is_finite(),
        "the robot COM accumulation is not finite",
    )?;
    Ok((
        weighted_position_m_kg / total_mass_kg,
        weighted_velocity_kg_m_s / total_mass_kg,
        total_mass_kg,
    ))
}

/// Returns the largest fixed-anchor separation and relative-rotation error.
///
/// Every authored fixed joint must retain its physical attachment. All fixed
/// descriptors on robot links are then checked in link-name order, including
/// attachments whose children have no collider.
pub(super) fn fixed_joint_error(sim: &UrdfSceneSim) -> ModelResult<(f64, f64)> {
    let world = sim.world();
    let mut authored: Vec<_> = world
        .iter_entities()
        .filter_map(|entity| {
            let joint = entity.get::<Joint>()?;
            (joint.kind == JointKind::Fixed).then_some(joint)
        })
        .collect();
    authored.sort_unstable_by_key(|joint| joint.child_link);
    for joint in authored {
        let link = world
            .get::<Link>(joint.child_link)
            .ok_or_else(|| io::Error::other("an authored fixed joint has no named child"))?;
        let desc = world
            .get::<FixedJointDesc>(joint.child_link)
            .ok_or_else(|| {
                io::Error::other(format!(
                    "fixed child {} has no physical attachment",
                    link.name
                ))
            })?;
        require(
            desc.parent == joint.parent_link,
            format!("fixed child {} is attached to the wrong parent", link.name),
        )?;
    }

    let mut fixed: Vec<_> = world
        .iter_entities()
        .filter_map(|entity| {
            let link = entity.get::<Link>()?;
            let desc = entity.get::<FixedJointDesc>()?;
            Some((link.name.as_str(), entity.id(), desc))
        })
        .collect();
    fixed.sort_unstable_by(|a, b| a.0.cmp(b.0));
    require(
        !fixed.is_empty(),
        "the robot has no physical fixed attachments",
    )?;
    let mut max_position_error_m = 0.0_f64;
    let mut max_rotation_error_rad = 0.0_f64;
    for (name, entity, desc) in fixed {
        require(
            world.get::<Link>(desc.parent).is_some()
                && world.get::<RigidBody>(desc.parent).is_some()
                && desc.anchor_parent_m.is_finite()
                && desc.anchor_child_m.is_finite()
                && desc.relative_rotation.is_finite()
                && (desc.relative_rotation.length_squared() - 1.0).abs() <= 1.0e-5,
            format!("fixed child {name} has invalid parent or attachment geometry"),
        )?;
        let parent = world_transform_of(world, desc.parent);
        let child = world_transform_of(world, entity);
        validate_transform(parent, name)?;
        validate_transform(child, name)?;
        let parent_anchor_m = parent.translation + parent.rotation * desc.anchor_parent_m;
        let child_anchor_m = child.translation + child.rotation * desc.anchor_child_m;
        let position_error_m = (parent_anchor_m - child_anchor_m).length();
        let expected_rotation = (parent.rotation * desc.relative_rotation).normalize();
        let rotation_dot = expected_rotation
            .dot(child.rotation.normalize())
            .abs()
            .clamp(0.0, 1.0);
        let rotation_error_rad = 2.0 * rotation_dot.acos();
        require(
            position_error_m.is_finite() && rotation_error_rad.is_finite(),
            format!("fixed child {name} has a non-finite attachment error"),
        )?;
        max_position_error_m = max_position_error_m.max(position_error_m);
        max_rotation_error_rad = max_rotation_error_rad.max(rotation_error_rad);
    }
    Ok((max_position_error_m, max_rotation_error_rad))
}

/// Rejects legacy mass approximations, detached links, and altered foot geometry.
pub(super) fn validate_plant(
    sim: &UrdfSceneSim,
    expected_mass_kg: f64,
    expected_foot_parts: usize,
    feet: &[&str],
) -> ModelResult<Value> {
    require(
        expected_mass_kg.is_finite()
            && expected_mass_kg > 0.0
            && expected_foot_parts > 0
            && !feet.is_empty(),
        "the expected physical-model contract is invalid",
    )?;
    let (com_m, com_velocity_m_s, total_mass_kg) = center_of_mass(sim)?;
    require(
        (total_mass_kg - expected_mass_kg).abs() <= MASS_TOLERANCE_KG,
        format!(
            "robot mass {total_mass_kg:.12} kg differs from declared total {expected_mass_kg:.12} kg"
        ),
    )?;
    let mut links: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<Link>()
                .map(|link| (link.name.as_str(), entity.id()))
        })
        .collect();
    links.sort_unstable_by(|a, b| a.0.cmp(b.0));
    for (name, entity) in &links {
        require(
            sim.world()
                .get::<RigidBody>(*entity)
                .is_some_and(|body| body.body_type == RigidBodyType::Dynamic)
                && sim.world().get::<MultibodyLink>(*entity).is_some(),
            format!("link {name} is not a dynamic member of the robot multibody"),
        )?;
    }
    let mut foot_geometry = Vec::new();
    for (index, name) in feet.iter().enumerate() {
        require(
            !feet[..index].contains(name),
            format!("support foot {name} is listed more than once"),
        )?;
        let (_, entity) = links
            .iter()
            .find(|(link_name, _)| link_name == name)
            .ok_or_else(|| io::Error::other(format!("support foot {name} is missing")))?;
        let collider = sim
            .world()
            .get::<Collider>(*entity)
            .ok_or_else(|| io::Error::other(format!("support foot {name} has no collider")))?;
        require(
            !collider.sensor
                && collider.material.friction.is_finite()
                && collider.material.friction >= 0.0
                && collider.material.restitution.is_finite()
                && (0.0..=1.0).contains(&collider.material.restitution),
            format!("support foot {name} has invalid contact material or is a sensor"),
        )?;
        validate_transform(collider.local_offset, name)?;
        let parts = if let Some(compound) = sim.world().get::<CompoundCollider>(*entity) {
            require(
                compound.is_valid(),
                format!("support foot {name} has invalid compound geometry"),
            )?;
            compound.parts.len()
        } else {
            let single = CompoundCollider {
                parts: vec![rne_physics::ColliderPart {
                    shape: collider.shape.clone(),
                    local_offset: collider.local_offset,
                }],
            };
            require(
                single.is_valid(),
                format!("support foot {name} has invalid primitive geometry"),
            )?;
            1
        };
        require(
            parts == expected_foot_parts,
            format!("support foot {name} has {parts} parts, expected {expected_foot_parts}"),
        )?;
        foot_geometry.push(json!({"link":name,"collision_parts":parts}));
    }
    let (position_error_m, rotation_error_rad) = fixed_joint_error(sim)?;
    require(
        position_error_m <= FIXED_POSITION_TOLERANCE_M
            && rotation_error_rad <= FIXED_ROTATION_TOLERANCE_RAD,
        format!("fixed attachment error is {position_error_m:.9} m / {rotation_error_rad:.9} rad"),
    )?;
    Ok(json!({
        "total_mass_kg":total_mass_kg,
        "expected_mass_kg":expected_mass_kg,
        "exact_inertia_links":links.len(),
        "all_links_in_multibody":true,
        "center_of_mass_m":com_m.to_array(),
        "center_of_mass_velocity_m_s":com_velocity_m_s.to_array(),
        "velocity_contract":"Rapier body linear velocity is already at each body COM",
        "feet":foot_geometry,
        "max_fixed_position_error_m":position_error_m,
        "max_fixed_rotation_error_rad":rotation_error_rad,
        "fixed_position_tolerance_m":FIXED_POSITION_TOLERANCE_M,
        "fixed_rotation_tolerance_rad":FIXED_ROTATION_TOLERANCE_RAD
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    const G1_MASS_KG: f64 = 34.133_857_284;
    const G1_FEET: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];

    fn g1() -> UrdfSceneSim {
        UrdfSceneSim::from_scene_path(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("g1.rne.scene.toml"),
        )
        .expect("dedicated physical G1 scene")
    }

    #[test]
    fn dedicated_models_preserve_mass_foot_geometry_and_fixed_attachments() {
        validate_plant(&g1(), G1_MASS_KG, 4, &G1_FEET).expect("physical G1");
        let go2 = UrdfSceneSim::from_scene_path(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("go2.rne.scene.toml"),
        )
        .expect("dedicated physical Go2 scene");
        validate_plant(
            &go2,
            16.087_000_011,
            1,
            &["FL_foot", "FR_foot", "RL_foot", "RR_foot"],
        )
        .expect("physical Go2");
    }

    #[test]
    fn legacy_scenes_cannot_qualify_as_declared_physical_models() {
        for (path, mass, parts, feet) in [
            (
                UrdfSceneSim::unitree_g1_dynamic_scene_path(),
                G1_MASS_KG,
                4,
                &G1_FEET[..],
            ),
            (
                UrdfSceneSim::unitree_go2_dynamic_scene_path(),
                16.087_000_011,
                1,
                &["FL_foot", "FR_foot", "RL_foot", "RR_foot"][..],
            ),
        ] {
            let legacy = UrdfSceneSim::from_scene_path(&path).expect("legacy scene");
            let error =
                validate_plant(&legacy, mass, parts, feet).expect_err("reject legacy plant");
            assert!(error.to_string().contains("no exact declared inertia"));
        }
    }

    #[test]
    fn mass_or_foot_shape_changes_fail_the_declared_model_gate() {
        let mut sim = g1();
        let pelvis = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Link>()
                    .is_some_and(|link| link.name == "pelvis")
            })
            .expect("pelvis")
            .id();
        sim.world_mut()
            .get_mut::<RigidBody>(pelvis)
            .expect("pelvis body")
            .mass_kg += 0.5;
        assert!(validate_plant(&sim, G1_MASS_KG, 4, &G1_FEET)
            .expect_err("reject altered mass")
            .to_string()
            .contains("differs from declared total"));

        let mut sim = g1();
        let foot = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Link>()
                    .is_some_and(|link| link.name == G1_FEET[0])
            })
            .expect("foot")
            .id();
        sim.world_mut()
            .entity_mut(foot)
            .remove::<CompoundCollider>();
        assert!(validate_plant(&sim, G1_MASS_KG, 4, &G1_FEET)
            .expect_err("reject bounding-box replacement")
            .to_string()
            .contains("has 1 parts, expected 4"));
    }

    #[test]
    fn a_detached_fixed_child_and_broken_anchor_cannot_pass() {
        let mut sim = g1();
        let child = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Link>()
                    .is_some_and(|link| link.name == "head_link")
            })
            .expect("head")
            .id();
        let desc = *sim
            .world()
            .get::<FixedJointDesc>(child)
            .expect("physical head attachment");
        sim.world_mut().entity_mut(child).remove::<FixedJointDesc>();
        assert!(fixed_joint_error(&sim)
            .expect_err("reject detached child")
            .to_string()
            .contains("has no physical attachment"));
        sim.world_mut().entity_mut(child).insert(FixedJointDesc {
            anchor_parent_m: desc.anchor_parent_m + Vec3::X * 0.02,
            ..desc
        });
        assert!(validate_plant(&sim, G1_MASS_KG, 4, &G1_FEET)
            .expect_err("reject broken anchor")
            .to_string()
            .contains("fixed attachment error"));
    }

    #[test]
    fn rapier_com_velocity_does_not_add_the_local_offset_twice() {
        let mut sim = g1();
        let links: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|entity| entity.get::<Link>().is_some())
            .map(|entity| entity.id())
            .collect();
        let velocity_m_s = Vec3::new(1.25, 0.5, -0.75);
        for entity in links {
            let mut body = sim
                .world_mut()
                .get_mut::<RigidBody>(entity)
                .expect("robot body");
            body.linear_velocity_m_s = velocity_m_s;
            body.angular_velocity_rad_s = Vec3::new(2.0, -3.0, 1.0);
        }
        let (_, measured_m_s, mass_kg) = center_of_mass(&sim).expect("declared COM");
        assert!((measured_m_s - velocity_m_s).length() < 1.0e-12);
        assert!((mass_kg - G1_MASS_KG).abs() < MASS_TOLERANCE_KG);
    }

    #[test]
    fn non_finite_link_velocity_fails_before_com_feedback() {
        let mut sim = g1();
        let entity = sim
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Link>().is_some())
            .expect("robot link")
            .id();
        sim.world_mut()
            .get_mut::<RigidBody>(entity)
            .expect("robot body")
            .linear_velocity_m_s = Vec3::new(f64::NAN, 0.0, 0.0);
        assert!(center_of_mass(&sim)
            .expect_err("reject NaN state")
            .to_string()
            .contains("invalid mass or velocity"));
    }
}
