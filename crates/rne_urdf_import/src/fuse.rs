//! Fusing URDF links that hang from the articulated tree only by fixed joints.

use std::collections::{HashMap, HashSet};

use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderPart, CompoundCollider, RigidBody, RigidBodyInertia};
use rne_world::Transform3;

use crate::schema::{UrdfJointType, UrdfRobot};
use crate::spawn::SpawnedUrdfRobot;

/// Moves every fixed-only URDF link's collision geometry, mass and inertia
/// into the link it hangs from, as MuJoCo's URDF importer fuses static
/// bodies, and returns how many links were fused.
///
/// A link is fixed-only when neither it nor any descendant takes part in a
/// non-fixed joint: a foot, a bumper or a sensor mount welded to a leg. Fusing
/// keeps the articulation at one body per moving part, instead of adding a
/// zero-degree-of-freedom multibody link (and its share of every mass-matrix
/// rebuild and forward-kinematics pass) for each such link, as
/// `weld_fixed_children` does.
///
/// Call it after spawning and before attaching the articulation, which must be
/// a multibody one without `weld_fixed_children`: the fused links keep their
/// entities, transforms and visuals, but lose their [`RigidBody`],
/// [`RigidBodyInertia`], [`Collider`] and [`CompoundCollider`], so the
/// articulation leaves them out. The parent's collider becomes a
/// [`CompoundCollider`] of its own and the fused parts, posed through the
/// fixed joints, and keeps the parent's material and settings. A fused link
/// that declares an inertial ([`RigidBodyInertia`], from
/// `use_declared_inertial_masses`) adds its mass to the parent's, and the two
/// inertia tensors combine about their joint centre of mass by the
/// parallel-axis theorem. A link that declares none is massless, as the URDF
/// specification has it, rather than carrying the spawner's default mass.
pub fn fuse_urdf_fixed_children(
    world: &mut World,
    urdf: &UrdfRobot,
    spawned: &SpawnedUrdfRobot,
) -> usize {
    let parent_of: HashMap<&str, &str> = urdf
        .joints
        .iter()
        .map(|joint| (joint.child.as_str(), joint.parent.as_str()))
        .collect();
    let fixed_child: HashSet<&str> = urdf
        .joints
        .iter()
        .filter(|joint| joint.joint_type == UrdfJointType::Fixed)
        .map(|joint| joint.child.as_str())
        .collect();
    // Links of moving joints, the base and all their ancestors stay bodies.
    let mut articulated: HashSet<&str> = urdf
        .joints
        .iter()
        .filter(|joint| joint.joint_type != UrdfJointType::Fixed)
        .flat_map(|joint| [joint.parent.as_str(), joint.child.as_str()])
        .collect();
    if let Some((name, _)) = spawned
        .links
        .iter()
        .find(|(_, entity)| **entity == spawned.base_link)
    {
        articulated.insert(name.as_str());
    }
    let mut stack: Vec<&str> = articulated.iter().copied().collect();
    while let Some(name) = stack.pop() {
        if let Some(parent) = parent_of.get(name) {
            if articulated.insert(parent) {
                stack.push(parent);
            }
        }
    }
    let depth = |name: &str| {
        let mut depth = 0;
        let mut current = name;
        while let Some(parent) = parent_of.get(current) {
            depth += 1;
            current = parent;
        }
        depth
    };
    let mut candidates: Vec<&str> = urdf
        .links
        .iter()
        .map(|link| link.name.as_str())
        .filter(|name| fixed_child.contains(name) && !articulated.contains(name))
        .collect();
    // Deepest first, so a chain of fixed links folds into its articulated
    // ancestor one joint at a time; names break ties deterministically.
    candidates.sort_by_key(|name| (std::cmp::Reverse(depth(name)), *name));

    let mut fused = 0;
    for name in candidates {
        let (Some(&child), Some(&parent)) = (
            spawned.links.get(name),
            parent_of
                .get(name)
                .and_then(|parent| spawned.links.get(*parent)),
        ) else {
            continue;
        };
        let joint = world.get::<Transform3>(child).copied().unwrap_or_default();
        fuse_colliders(world, parent, child, &joint);
        fuse_mass(world, parent, child, &joint);
        world
            .entity_mut(child)
            .remove::<(RigidBody, RigidBodyInertia, Collider, CompoundCollider)>();
        fused += 1;
    }
    fused
}

/// An entity's collision parts relative to itself.
fn collision_parts(world: &World, entity: Entity) -> Vec<ColliderPart> {
    if let Some(compound) = world.get::<CompoundCollider>(entity) {
        return compound.parts.clone();
    }
    world
        .get::<Collider>(entity)
        .map(|collider| {
            vec![ColliderPart {
                shape: collider.shape.clone(),
                local_offset: collider.local_offset,
            }]
        })
        .unwrap_or_default()
}

fn fuse_colliders(world: &mut World, parent: Entity, child: Entity, joint: &Transform3) {
    let child_parts: Vec<ColliderPart> = collision_parts(world, child)
        .into_iter()
        .map(|part| ColliderPart {
            shape: part.shape,
            local_offset: joint.mul_transform(&part.local_offset),
        })
        .collect();
    let Some(first) = child_parts.first().cloned() else {
        return;
    };
    if world.get::<Collider>(parent).is_none() {
        // The parent had no geometry: it takes the child's settings.
        let mut collider = world
            .get::<Collider>(child)
            .cloned()
            .expect("a link with collision parts has a collider");
        collider.shape = first.shape;
        collider.local_offset = first.local_offset;
        world.entity_mut(parent).insert(collider);
        if child_parts.len() > 1 {
            world
                .entity_mut(parent)
                .insert(CompoundCollider { parts: child_parts });
        }
        return;
    }
    let mut parts = collision_parts(world, parent);
    parts.extend(child_parts);
    world.entity_mut(parent).insert(CompoundCollider { parts });
}

type Matrix3 = [[f64; 3]; 3];

fn tensor(inertia: &RigidBodyInertia) -> Matrix3 {
    [
        [inertia.ixx_kg_m2, inertia.ixy_kg_m2, inertia.ixz_kg_m2],
        [inertia.ixy_kg_m2, inertia.iyy_kg_m2, inertia.iyz_kg_m2],
        [inertia.ixz_kg_m2, inertia.iyz_kg_m2, inertia.izz_kg_m2],
    ]
}

/// `R I Rᵀ` for the rotation `rotation`.
fn rotated(inertia: &Matrix3, rotation: Quat) -> Matrix3 {
    let columns = [rotation * Vec3::X, rotation * Vec3::Y, rotation * Vec3::Z];
    let r = |i: usize, j: usize| columns[j].to_array()[i];
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            *value = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| r(i, k) * inertia[k][l] * r(j, l))
                .sum();
        }
    }
    out
}

/// `m (|d|² E − d dᵀ)`: a point mass `m` at offset `d`.
fn shifted(mass_kg: f64, offset: Vec3) -> Matrix3 {
    let d = offset.to_array();
    let squared = offset.length_squared();
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            *value = mass_kg * (if i == j { squared } else { 0.0 } - d[i] * d[j]);
        }
    }
    out
}

fn fuse_mass(world: &mut World, parent: Entity, child: Entity, joint: &Transform3) {
    let (Some(parent_body), Some(child_body), Some(child_inertia)) = (
        world.get::<RigidBody>(parent).cloned(),
        world.get::<RigidBody>(child).cloned(),
        world.get::<RigidBodyInertia>(child).copied(),
    ) else {
        // No declared inertial: a massless link.
        return;
    };
    let total_kg = parent_body.mass_kg + child_body.mass_kg;
    if let Some(parent_inertia) = world.get::<RigidBodyInertia>(parent).copied() {
        if total_kg > 0.0 {
            let child_com =
                joint.translation + joint.rotation * child_inertia.center_of_mass_local_m;
            let com = (parent_inertia.center_of_mass_local_m * parent_body.mass_kg
                + child_com * child_body.mass_kg)
                / total_kg;
            let parts = [
                tensor(&parent_inertia),
                shifted(
                    parent_body.mass_kg,
                    parent_inertia.center_of_mass_local_m - com,
                ),
                rotated(&tensor(&child_inertia), joint.rotation),
                shifted(child_body.mass_kg, child_com - com),
            ];
            let sum = |i: usize, j: usize| parts.iter().map(|part| part[i][j]).sum::<f64>();
            world.entity_mut(parent).insert(RigidBodyInertia {
                center_of_mass_local_m: com,
                ixx_kg_m2: sum(0, 0),
                ixy_kg_m2: sum(0, 1),
                ixz_kg_m2: sum(0, 2),
                iyy_kg_m2: sum(1, 1),
                iyz_kg_m2: sum(1, 2),
                izz_kg_m2: sum(2, 2),
            });
        }
    }
    if let Some(mut body) = world.get_mut::<RigidBody>(parent) {
        body.mass_kg = total_kg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};

    const ARM: &str = r#"<robot name="arm">
      <link name="base">
        <inertial><mass value="2"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
        <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
      </link>
      <link name="arm">
        <inertial><origin xyz="0 0 -0.1"/><mass value="1"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.004"/></inertial>
        <collision><geometry><cylinder radius="0.02" length="0.2"/></geometry></collision>
      </link>
      <link name="tip">
        <inertial><origin xyz="0.1 0 0"/><mass value="0.5"/>
          <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.003"/></inertial>
        <collision><geometry><sphere radius="0.05"/></geometry></collision>
      </link>
      <link name="pad">
        <collision><origin xyz="0 0 -0.01"/><geometry><sphere radius="0.01"/></geometry></collision>
      </link>
      <joint name="shoulder" type="revolute">
        <parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
      <joint name="tip_mount" type="fixed">
        <parent link="arm"/><child link="tip"/><origin xyz="0 0 -0.2" rpy="0 0 1.5707963267948966"/>
      </joint>
      <joint name="pad_mount" type="fixed">
        <parent link="tip"/><child link="pad"/><origin xyz="0.05 0 0"/>
      </joint>
    </robot>"#;

    fn matrix(inertia: &RigidBodyInertia) -> Matrix3 {
        tensor(inertia)
    }

    #[test]
    fn fixed_children_fold_into_the_moving_link_with_their_geometry_and_mass() {
        let document = parse_urdf_document(ARM).unwrap();
        let mut world = World::new();
        let config = UrdfSpawnConfig {
            use_declared_inertial_masses: true,
            ..UrdfSpawnConfig::default()
        };
        let spawned = spawn_urdf_document_with_config(&mut world, &document, config).unwrap();
        let link = |name: &str| spawned.links[name];
        let before: Vec<(f64, RigidBodyInertia)> = ["arm", "tip"]
            .iter()
            .map(|name| {
                (
                    world.get::<RigidBody>(link(name)).unwrap().mass_kg,
                    *world.get::<RigidBodyInertia>(link(name)).unwrap(),
                )
            })
            .collect();
        // The pad declares no inertial: the spawner's default mass, which
        // fusing drops, as the URDF specification makes such a link massless.
        assert!(world.get::<RigidBodyInertia>(link("pad")).is_none());

        assert_eq!(
            fuse_urdf_fixed_children(&mut world, &document.robot, &spawned),
            2
        );
        for name in ["tip", "pad"] {
            let entity = world.entity(link(name));
            assert!(entity.get::<RigidBody>().is_none() && entity.get::<Collider>().is_none());
            assert!(
                entity.get::<Transform3>().is_some(),
                "{name} keeps its frame"
            );
        }
        // The base and the arm stay bodies; the arm now carries three parts.
        assert!(world.get::<CompoundCollider>(link("base")).is_none());
        let parts = &world.get::<CompoundCollider>(link("arm")).unwrap().parts;
        assert_eq!(parts.len(), 3);
        let mount = Transform3::from_translation_rotation(
            Vec3::new(0.0, 0.0, -0.2),
            Quat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        );
        assert!((parts[1].local_offset.translation - mount.translation).length() < 1e-12);
        assert!(parts[1].local_offset.rotation.angle_between(mount.rotation) < 1e-12);
        // The pad hangs 0.05 m along the tip's x, which the mount turns to y,
        // and 0.01 m below its own frame.
        let pad = Vec3::new(0.0, 0.05, -0.21);
        assert!((parts[2].local_offset.translation - pad).length() < 1e-12);

        let arm_body = world.get::<RigidBody>(link("arm")).unwrap();
        let arm = *world.get::<RigidBodyInertia>(link("arm")).unwrap();
        let (arm_kg, tip_kg) = (before[0].0, before[1].0);
        assert!((arm_body.mass_kg - (arm_kg + tip_kg)).abs() < 1e-12);
        // Centres of mass in the arm frame.
        let tip_com = Vec3::new(0.0, 0.1, -0.2);
        let com =
            (before[0].1.center_of_mass_local_m * arm_kg + tip_com * tip_kg) / (arm_kg + tip_kg);
        assert!((arm.center_of_mass_local_m - com).length() < 1e-12);
        // Parallel axis about the arm origin: the fused tensor shifted there
        // equals the parts' tensors shifted there. The tip's tensor turns by
        // a quarter turn about z, swapping its x and y moments.
        let tip_tensor = [[0.002, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.003]];
        let arm_tensor = matrix(&before[0].1);
        let mut expected = [[0.0; 3]; 3];
        let mut fused = matrix(&arm);
        let fused_shift = shifted(arm_kg + tip_kg, arm.center_of_mass_local_m);
        let arm_shift = shifted(arm_kg, before[0].1.center_of_mass_local_m);
        let tip_shift = shifted(tip_kg, tip_com);
        for i in 0..3 {
            for j in 0..3 {
                expected[i][j] =
                    arm_tensor[i][j] + arm_shift[i][j] + tip_tensor[i][j] + tip_shift[i][j];
                fused[i][j] += fused_shift[i][j];
                assert!((fused[i][j] - expected[i][j]).abs() < 1e-12, "{i}{j}");
            }
        }
    }
}
