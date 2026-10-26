//! Local application commands and GPU sensor presentation.

use crate::drive::DriveCommand;
use crate::presets::preset;
use crate::{
    project::{JointPosition, Project, RobotFormat, RobotSource},
    simulation::Simulation,
};
use anyhow::{ensure, Context, Result};
use rne_math::Vec3;
use rne_render::{Camera, MeshRenderCache, RenderBackend};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Deserialize;
use serde_json::{json, Value};
use std::{collections::BTreeMap, io::Cursor, path::PathBuf};

#[derive(Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
pub(crate) enum Command {
    GraspDemo,
    TaskStart,
    TaskAdvance {
        to_boundary: bool,
    },
    TaskCancel,
    PickPlaceDemo,
    Replay {
        recording: crate::replay::Recording,
    },
    Drive {
        velocity: DriveCommand,
    },
    Step {
        steps: u32,
    },
    Targets {
        targets: BTreeMap<String, JointPosition>,
    },
    SavePose {
        name: String,
    },
    ApplyPose {
        name: String,
    },
    ReplaceProject {
        project: Project,
    },
    Import {
        format: RobotFormat,
        xml: String,
    },
    Preset {
        name: String,
    },
    Reset,
    View {
        yaw_rad: f64,
        pitch_rad: f64,
        distance_m: f64,
    },
}

pub(crate) struct Host {
    simulation: Simulation,
    task: Option<crate::task::Task>,
    renderer: WgpuRenderBackend,
    meshes: MeshRenderCache,
    asset_root: PathBuf,
    orbit: CameraOrbit,
    grasp_report: Option<crate::grasp::GraspReport>,
    pick_place_report: Option<crate::grasp::PickPlaceReport>,
}

impl std::fmt::Debug for Host {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Host")
            .field("simulation", &self.simulation)
            .field("asset_root", &self.asset_root)
            .finish_non_exhaustive()
    }
}

impl Host {
    pub(crate) fn new(project: Project, asset_root: PathBuf) -> Result<Self> {
        let simulation = Simulation::new(project, Some(&asset_root))?;
        let (focus, distance_m) = simulation.camera_fit();
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut simulation.render_scene(), &[&asset_root])?;
        Ok(Self {
            simulation,
            task: None,
            grasp_report: None,
            pick_place_report: None,
            renderer: WgpuRenderBackend::new().context("initialize GPU renderer")?,
            meshes,
            asset_root,
            orbit: CameraOrbit {
                yaw_rad: 0.7,
                pitch_rad: 1.0,
                distance_m,
                focus,
            },
        })
    }

    pub(crate) fn command(&mut self, command: Command) -> Result<()> {
        ensure!(
            self.task.is_none()
                || !matches!(
                    &command,
                    Command::Step { .. }
                        | Command::Drive { .. }
                        | Command::Targets { .. }
                        | Command::ApplyPose { .. }
                ),
            "cancel the task before manual control"
        );
        match command {
            Command::TaskStart => self.start_task()?,
            Command::TaskCancel => {
                if self.task.is_some() {
                    self.simulation.set_drive(DriveCommand::default())?;
                }
                self.task = None;
            }
            Command::TaskAdvance { to_boundary } => {
                let task = self.task.as_mut().context("no active task")?;
                ensure!(
                    task.status == crate::task::Status::Running,
                    "task is terminal; reset or cancel it"
                );
                let phase = task.phase;
                loop {
                    task.advance(&mut self.simulation)?;
                    if !to_boundary
                        || task.phase != phase
                        || task.status != crate::task::Status::Running
                    {
                        break;
                    }
                }
            }
            Command::GraspDemo => self.demo(false)?,
            Command::PickPlaceDemo => self.demo(true)?,
            Command::Replay { recording } => {
                let candidate = recording.verify(Some(&self.asset_root))?;
                let mut meshes = MeshRenderCache::new();
                meshes.resolve_scene(&mut candidate.render_scene(), &[&self.asset_root])?;
                let (focus, distance_m) = candidate.camera_fit();
                self.grasp_report = None;
                self.task = None;
                self.pick_place_report = None;
                self.simulation = candidate;
                self.meshes = meshes;
                self.orbit.focus = focus;
                self.orbit.distance_m = distance_m;
                self.renderer.reset_taa_history();
            }
            Command::Drive { velocity } => self.simulation.set_drive(velocity)?,
            Command::Step { steps } => self.simulation.step(steps)?,
            Command::Targets { targets } => self.simulation.set_targets(targets)?,
            Command::SavePose { name } => self.simulation.project.save_pose(&name)?,
            Command::ApplyPose { name } => {
                let pose = self
                    .simulation
                    .project
                    .poses
                    .get(&name)
                    .context("unknown saved pose")?
                    .clone();
                self.simulation.project.apply_pose(&pose)?;
            }
            Command::ReplaceProject { project } => self.replace(project)?,
            Command::Import { format, xml } => {
                self.replace(Project::new(RobotSource { format, xml })?)?
            }
            Command::Preset { name } => self.replace(preset(&name)?)?,
            Command::Reset => self.replace(self.simulation.project.clone())?,
            Command::View {
                yaw_rad,
                pitch_rad,
                distance_m,
            } => {
                ensure!(
                    yaw_rad.is_finite()
                        && yaw_rad.abs() <= 100.0
                        && pitch_rad.is_finite()
                        && (0.15..=1.45).contains(&pitch_rad)
                        && distance_m.is_finite()
                        && (0.3..=30.0).contains(&distance_m),
                    "invalid camera orbit"
                );
                self.orbit.yaw_rad = yaw_rad;
                self.orbit.pitch_rad = pitch_rad;
                self.orbit.distance_m = distance_m;
            }
        }
        Ok(())
    }

    fn start_task(&mut self) -> Result<()> {
        let mut project = self.simulation.project.clone();
        for joint in project.joint_catalog()? {
            let value = 0.0_f64.clamp(joint.lower, joint.upper);
            project.targets.insert(
                joint.name,
                if joint.linear {
                    JointPosition::Prismatic { position_m: value }
                } else {
                    JointPosition::Revolute {
                        position_rad: value,
                    }
                },
            );
        }
        let candidate = Simulation::new(project, Some(&self.asset_root))?;
        let task = crate::task::Task::new(&candidate)?;
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut candidate.render_scene(), &[&self.asset_root])?;
        self.simulation = candidate;
        self.meshes = meshes;
        self.task = Some(task);
        self.grasp_report = None;
        self.pick_place_report = None;
        self.renderer.reset_taa_history();
        Ok(())
    }

    fn demo(&mut self, place: bool) -> Result<()> {
        let name = if place {
            "lekiwi_pick_place"
        } else {
            "lekiwi_grasp"
        };
        let mut candidate = Simulation::new(preset(name)?, Some(&self.asset_root))?;
        let (grasp_report, pick_place_report) = if place {
            (None, Some(crate::grasp::pick_place(&mut candidate)?))
        } else {
            (Some(crate::grasp::run(&mut candidate)?), None)
        };
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut candidate.render_scene(), &[&self.asset_root])?;
        self.simulation = candidate;
        self.meshes = meshes;
        self.orbit.focus = Vec3::new(0.4, 0.3, 0.08);
        self.orbit.distance_m = 1.3;
        self.task = None;
        self.grasp_report = grasp_report;
        self.pick_place_report = pick_place_report;
        self.renderer.reset_taa_history();
        Ok(())
    }

    fn replace(&mut self, project: Project) -> Result<()> {
        let refit = project.robot != self.simulation.project.robot
            || project.robot_rotation_rpy_rad != self.simulation.project.robot_rotation_rpy_rad;
        let candidate = Simulation::new(project, Some(&self.asset_root))?;
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut candidate.render_scene(), &[&self.asset_root])?;
        if refit {
            let (focus, distance_m) = candidate.camera_fit();
            self.orbit.focus = focus;
            self.orbit.distance_m = distance_m;
        }
        self.simulation = candidate;
        self.grasp_report = None;
        self.task = None;
        self.pick_place_report = None;
        self.meshes = meshes;
        self.renderer.reset_taa_history();
        Ok(())
    }

    pub(crate) fn recording(&self) -> Result<Vec<u8>> {
        Ok(serde_json::to_vec(self.simulation.recording())?)
    }

    pub(crate) fn snapshot(&mut self) -> Result<Value> {
        let mut scene = self.simulation.render_scene();
        self.meshes.resolve_scene(&mut scene, &[&self.asset_root])?;
        let mut camera = Camera::new(640, 400, 0.9);
        camera.near_m = 0.02;
        camera.far_m = 30.0;
        let view = self.renderer.render_scene_camera(
            &camera,
            &self.orbit.camera_transform(),
            &scene,
            [0.025, 0.04, 0.06, 1.0],
        )?;
        let image = image::RgbaImage::from_raw(640, 400, view.color.rgba8)
            .context("invalid rendered image")?;
        let mut png = Cursor::new(Vec::new());
        image::DynamicImage::ImageRgba8(image).write_to(&mut png, image::ImageFormat::Png)?;
        let mut sensor_camera = Camera::new(160, 120, 1.0);
        sensor_camera.near_m = 0.02;
        sensor_camera.far_m = 10.0;
        let sensor_mount = CameraOrbit {
            yaw_rad: -0.5,
            pitch_rad: 1.1,
            distance_m: 2.0,
            focus: Vec3::new(0.0, 0.35, 0.0),
        }
        .camera_transform();
        let sensor = self.renderer.render_scene_camera(
            &sensor_camera,
            &sensor_mount,
            &scene,
            [0.025, 0.04, 0.06, 1.0],
        )?;
        let depth = sensor
            .depth
            .depth_m
            .iter()
            .map(|v| {
                if v.is_finite() && *v >= 0.0 && *v < sensor_camera.far_m as f32 {
                    Some(*v)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        Ok(json!({
            "project": self.simulation.project,
            "grasp_report": self.grasp_report,
            "pick_place_report": self.pick_place_report.as_ref().or_else(|| self.task.as_ref().and_then(|task| task.report.as_ref())),
            "task": self.task,
            "transfer_preview": match crate::transfer::plan(&self.simulation.project) {
                Ok(plan) => json!({"plan": plan, "error": null}),
                Err(error) => json!({"plan": null, "error": error.to_string()}),
            },
            "base": { "pose_x_z_yaw": self.simulation.base_reading().0,
                "command": self.simulation.base_reading().1,
                "wheel_targets_rad_s": self.simulation.base_reading().2 },
            "sim_time_ticks": self.simulation.sim_time_ticks().to_string(),
            "state_hash": format!("{:016x}", self.simulation.state_hash()),
            "joints": self.simulation.readings()?,
            "objects": self.simulation.object_readings()?,
            "view_png_base64": base64::encode(png.into_inner()),
            "camera": { "sim_time_ticks": self.simulation.sim_time_ticks().to_string(),
                "width": 160, "height": 120, "rgba8_base64": base64::encode(sensor.color.rgba8),
                "depth_m": depth, "near_m": sensor_camera.near_m, "far_m": sensor_camera.far_m },
            "lidar": self.simulation.lidar()?,
            "orbit": { "yaw_rad": self.orbit.yaw_rad, "pitch_rad": self.orbit.pitch_rad,
                "distance_m": self.orbit.distance_m }
        }))
    }
}
