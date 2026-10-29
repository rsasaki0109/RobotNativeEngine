//! Idealized passive spherical rollers; not calibrated LeKiwi hardware geometry.
use anyhow::{Context, Result};
use rne_math::Vec3;
use rne_urdf_import::{
    rpy_to_quat, UrdfDocument, UrdfGeometry, UrdfGeometryElement, UrdfInertial, UrdfJoint,
    UrdfJointType, UrdfLink,
};

pub(crate) const PREFIX: &str = "arena_roller_";
const COUNT: usize = 16;
const RADIUS_M: f64 = 0.008;

/// Replace each solid wheel collision with a ring of free tangent-axis rollers.
pub(crate) fn add_rollers(document: &mut UrdfDocument) -> Result<()> {
    // Experimental chassis envelope replaces overlapping CAD base-part AABBs.
    // Arm geometry/inertials remain authored. These are declared model parameters,
    // not a calibration of the physical platform.
    let mut base_links = std::collections::BTreeSet::from(["base_link".to_owned()]);
    loop {
        let previous = base_links.len();
        for j in &document.robot.joints {
            if base_links.contains(&j.parent) && j.name != "so101_mount_joint" {
                base_links.insert(j.child.clone());
            }
        }
        if previous == base_links.len() {
            break;
        }
    }
    for link in &mut document.robot.links {
        if !base_links.contains(&link.name) {
            continue;
        }
        link.collisions.clear();
        let mass = if link.name == "base_link" {
            1.5
        } else if link.name.starts_with("omni_wheel_mount") || link.name.starts_with("4-Omni") {
            0.04
        } else {
            0.001
        };
        link.inertial_mass_kg = Some(mass);
        let moment = if link.name == "base_link" {
            0.01
        } else {
            mass * 0.0005
        };
        document.inertials.insert(
            link.name.clone(),
            UrdfInertial {
                origin_xyz_m: Vec3::ZERO,
                origin_rpy_rad: Vec3::ZERO,
                mass_kg: mass,
                ixx_kg_m2: moment,
                iyy_kg_m2: moment,
                izz_kg_m2: moment,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyz_kg_m2: 0.0,
            },
        );
        if link.name == "base_link" {
            link.collisions.push(UrdfGeometryElement {
                origin_xyz: Vec3::new(0.0, 0.0, 0.04),
                origin_rpy: Vec3::ZERO,
                material_rgba: None,
                geometry: UrdfGeometry::Box {
                    size_m: Vec3::new(0.20, 0.18, 0.03),
                },
            });
        }
    }
    for (index, suffix) in ["-2", "-1", ""].iter().enumerate() {
        let hub = format!("omni_wheel_mount-v5{suffix}");
        let wheel = format!("4-Omni-Directional-Wheel_Single_Body-v1{suffix}");
        let drive = document
            .robot
            .joints
            .iter()
            .find(|j| j.child == hub)
            .context("wheel drive missing")?;
        let axis = drive.axis.normalize();
        let fixed = document
            .robot
            .joints
            .iter()
            .find(|j| j.child == wheel)
            .context("wheel mount missing")?;
        let shape = document
            .robot
            .links
            .iter()
            .find(|l| l.name == wheel)
            .and_then(|l| l.visuals.first())
            .context("wheel geometry missing")?;
        // Center the idealized ring on the actual drive axle; retain axial offset.
        let old_center = fixed.origin_xyz + rpy_to_quat(fixed.origin_rpy) * shape.origin_xyz;
        let center = axis * old_center.dot(axis);
        let reference = if axis.x.abs() < 0.8 { Vec3::X } else { Vec3::Y };
        let u = axis.cross(reference).normalize();
        let v = axis.cross(u);
        for link in &mut document.robot.links {
            if link.name == hub || link.name == wheel {
                link.collisions.clear();
            }
        }
        for roller in 0..COUNT {
            let angle = std::f64::consts::TAU * roller as f64 / COUNT as f64;
            let radial = u * angle.cos() + v * angle.sin();
            let name = format!("{PREFIX}{index}_{roller}");
            let mass = 0.005;
            let inertia = 0.4 * mass * RADIUS_M * RADIUS_M;
            let geometry = UrdfGeometryElement {
                origin_xyz: Vec3::ZERO,
                origin_rpy: Vec3::ZERO,
                material_rgba: Some([0.2, 0.25, 0.3, 1.0]),
                geometry: UrdfGeometry::Sphere { radius_m: RADIUS_M },
            };
            document.robot.links.push(UrdfLink {
                name: name.clone(),
                inertial_mass_kg: Some(mass),
                collisions: vec![geometry.clone()],
                visuals: vec![geometry],
            });
            document.inertials.insert(
                name.clone(),
                UrdfInertial {
                    origin_xyz_m: Vec3::ZERO,
                    origin_rpy_rad: Vec3::ZERO,
                    mass_kg: mass,
                    ixx_kg_m2: inertia,
                    iyy_kg_m2: inertia,
                    izz_kg_m2: inertia,
                    ixy_kg_m2: 0.0,
                    ixz_kg_m2: 0.0,
                    iyz_kg_m2: 0.0,
                },
            );
            document.robot.joints.push(UrdfJoint {
                name: name.clone(),
                joint_type: UrdfJointType::Continuous,
                parent: hub.clone(),
                child: name,
                origin_xyz: center + radial * (0.0508 - RADIUS_M),
                origin_rpy: Vec3::ZERO,
                axis: axis.cross(radial),
                limit: None,
                mimic: None,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rolling_model_preserves_arm_and_adds_tangent_passive_joints() {
        let mut document = crate::presets::preset("lekiwi_grasp")
            .unwrap()
            .robot
            .document()
            .unwrap();
        let arm = document.inertials["upper_arm_link"];
        add_rollers(&mut document).unwrap();
        assert_eq!(document.inertials["upper_arm_link"], arm);
        let rollers = document
            .robot
            .joints
            .iter()
            .filter(|j| j.name.starts_with(PREFIX))
            .collect::<Vec<_>>();
        assert_eq!(rollers.len(), 3 * COUNT);
        for joint in rollers {
            let drive = document
                .robot
                .joints
                .iter()
                .find(|j| j.child == joint.parent)
                .unwrap();
            let axis = drive.axis.normalize();
            assert!(joint.axis.dot(axis).abs() < 1e-12);
            let radial = joint.origin_xyz - axis * joint.origin_xyz.dot(axis);
            assert!(joint.axis.dot(radial).abs() < 1e-12);
            assert!((radial.length() + RADIUS_M - 0.0508).abs() < 1e-12);
        }
    }
}
