//! Bundled workbench projects, also usable by headless tests.
use crate::project::{Project, RobotFormat, RobotSource};
use anyhow::Result;

pub(crate) fn preset(name: &str) -> Result<Project> {
    let source = match name {
        "so101" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../assets/robots/so101/so101.urdf").into(),
        },
        "lekiwi_grasp" | "lekiwi_pick_place" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../assets/robots/lekiwi_so101/lekiwi_so101_grasp.urdf")
                .replace("filename=\"../", "filename=\""),
        },
        "lekiwi_so101" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../assets/robots/lekiwi_so101/lekiwi_so101_workbench.urdf")
                .replace("filename=\"../", "filename=\""),
        },
        "slider" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
                .into(),
        },
        "arm" => RobotSource {
            format: RobotFormat::Mjcf,
            xml: include_str!("../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").into(),
        },
        _ => anyhow::bail!("unknown robot preset"),
    };
    let mut project = Project::new(source)?;
    if matches!(
        name,
        "so101" | "lekiwi_so101" | "lekiwi_grasp" | "lekiwi_pick_place"
    ) {
        project.robot_rotation_rpy_rad[0] = -std::f64::consts::FRAC_PI_2;
    }
    if matches!(name, "lekiwi_so101" | "lekiwi_grasp" | "lekiwi_pick_place") {
        project.schema_version = 2;
        project.use_declared_inertias = true;
        project.planar_base = Some(crate::project::PlanarBase { height_m: 0.06 });
        let names = project
            .joint_catalog()?
            .into_iter()
            .map(|j| j.name)
            .collect::<std::collections::BTreeSet<_>>();
        project.targets.retain(|name, _| names.contains(name));
        project.objects[0].position_m[1] = -0.05;
        project.objects.push(crate::project::SceneObject {
            dynamics: None,
            name: "pickup-table".into(),
            position_m: [0.65, 0.12, 0.0],
            size_m: [0.3, 0.24, 0.4],
            color_rgba: [0.55, 0.36, 0.2, 1.0],
        });
        project.objects.push(crate::project::SceneObject {
            dynamics: None,
            name: "target-block".into(),
            position_m: [0.65, 0.265, 0.0],
            size_m: [0.05; 3],
            color_rgba: [0.95, 0.55, 0.15, 1.0],
        });
    }
    project.objects.push(crate::project::SceneObject {
        dynamics: None,
        name: "inspection-wall".into(),
        position_m: [1.3, 0.5, -0.4],
        size_m: [0.15, 1.0, 1.2],
        color_rgba: [0.2, 0.55, 0.65, 1.0],
    });
    if matches!(name, "lekiwi_grasp" | "lekiwi_pick_place") {
        project.schema_version = 3;
        let table = project
            .objects
            .iter_mut()
            .find(|o| o.name == "pickup-table")
            .unwrap();
        table.position_m = [0.60, 0.1845, -0.046];
        table.size_m = [0.16, 0.369, 0.08];
        let block = project
            .objects
            .iter_mut()
            .find(|o| o.name == "target-block")
            .unwrap();
        block.position_m = [0.60, 0.38, -0.054];
        block.size_m = [0.02; 3];
        block.dynamics = Some(crate::project::ObjectDynamics {
            mass_kg: 0.02,
            friction: 1.0,
        });
    }
    if name == "lekiwi_pick_place" {
        project.objects.push(crate::project::SceneObject {
            dynamics: None,
            name: "place-table".into(),
            position_m: [0.60, 0.1845, 0.154],
            size_m: [0.16, 0.369, 0.08],
            color_rgba: [0.2, 0.65, 0.45, 1.0],
        });
    }
    project.validate()?;
    Ok(project)
}
