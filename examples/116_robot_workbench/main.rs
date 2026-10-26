//! Local robot workbench application.

mod drive;
mod grasp;
#[cfg(feature = "gpu")]
mod host;
mod presets;
mod project;
mod replay;
#[cfg(feature = "gpu")]
mod server;
mod simulation;
mod task;
mod transfer;

use anyhow::{bail, ensure, Context, Result};
use project::{JointPosition, Project, RobotFormat, RobotSource};
use std::io::Read;
#[cfg(feature = "gpu")]
use std::path::PathBuf;

fn read_bounded(path: &str) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(4 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(bytes.len() <= 4 * 1024 * 1024, "file exceeds 4 MiB");
    Ok(bytes)
}

fn replay_file(path: &str, root: Option<&std::path::Path>) -> Result<()> {
    let recording = replay::Recording::from_json(&read_bounded(path)?)?;
    let simulation = recording.verify(root)?;
    println!(
        "replay verified: {} frames, {} ticks, state={:016x}",
        recording.frames.len(),
        simulation.sim_time_ticks(),
        simulation.state_hash()
    );
    Ok(())
}

fn smoke_pick_place() -> Result<()> {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
    let mut sim = simulation::Simulation::new(presets::preset("lekiwi_pick_place")?, Some(&root))?;
    let report = grasp::pick_place(&mut sim)?;
    println!("{}", serde_json::to_string(&report)?);
    sim.recording().verify(Some(&root))?;
    println!("pick/place and replay verified");
    Ok(())
}

fn smoke_grasp() -> Result<()> {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
    let mut sim = simulation::Simulation::new(presets::preset("lekiwi_grasp")?, Some(&root))?;
    let report = grasp::run(&mut sim)?;
    println!("{}", serde_json::to_string(&report)?);
    sim.recording().verify(Some(&root))?;
    println!("contact grasp and replay verified");
    Ok(())
}

fn smoke_lekiwi() -> Result<()> {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
    let mut simulation =
        simulation::Simulation::new(presets::preset("lekiwi_so101")?, Some(&root))?;
    let mut targets = simulation.project.targets.clone();
    targets.insert(
        "shoulder_pan".into(),
        JointPosition::Revolute { position_rad: 0.2 },
    );
    simulation.set_targets(targets)?;
    simulation.set_drive(drive::DriveCommand {
        vx_m_s: 0.05,
        vz_m_s: 0.02,
        yaw_rad_s: 0.1,
    })?;
    simulation.step(120)?;
    let pose = simulation.base_reading().0;
    ensure!(
        pose[0] > 0.02 && pose[1] > 0.005 && pose[2] > 0.04,
        "base did not move"
    );
    ensure!(
        simulation.readings()?.len() == 6,
        "SO-101 must expose six joints"
    );
    simulation.set_drive(drive::DriveCommand::default())?;
    simulation.step(120)?;
    ensure!(simulation.base_reading().0 == pose, "base did not stop");
    ensure!(
        simulation
            .readings()?
            .iter()
            .any(|joint| joint.info.name == "shoulder_pan" && (joint.measured - 0.2).abs() < 0.05),
        "SO-101 shoulder did not move: {:?}",
        simulation.readings()?
    );
    let recording = replay::Recording::from_json(&serde_json::to_vec(simulation.recording())?)?;
    recording.verify(Some(&root))?;
    println!("LeKiwi + SO-101 smoke passed: planar drive, stop, six arm joints, exact replay; state={:016x}", simulation.state_hash());
    Ok(())
}

fn smoke() -> Result<()> {
    let mut project = Project::new(RobotSource {
        format: RobotFormat::Urdf,
        xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
            .into(),
    })?;
    project.targets.insert(
        "slider_joint".into(),
        JointPosition::Prismatic { position_m: 0.1 },
    );
    project.save_pose("extended")?;
    let saved = project.poses["extended"].clone();
    project.apply_pose(&saved)?;
    let roundtrip = Project::from_json(&serde_json::to_vec(&project)?)?;
    let mut simulation = simulation::Simulation::new(roundtrip, None)?;
    simulation.set_targets(project.targets.clone())?;
    simulation.step(240)?;
    ensure!(
        simulation.readings()?[0].measured > 0.06,
        "native slider did not reach its target"
    );
    ensure!(
        !simulation.render_scene().items.is_empty(),
        "missing robot visuals"
    );
    ensure!(
        simulation.camera_fit().1.is_finite(),
        "invalid camera bounds"
    );
    ensure!(
        simulation.lidar()?.ranges_m.len() == 180,
        "incomplete LiDAR scan"
    );
    let mut arm = Project::new(RobotSource {
        format: RobotFormat::Mjcf,
        xml: include_str!("../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").into(),
    })?;
    arm.targets.insert(
        "shoulder".into(),
        JointPosition::Revolute { position_rad: 0.3 },
    );
    let mut arm_simulation = simulation::Simulation::new(arm, None)?;
    arm_simulation.step(240)?;
    ensure!(
        arm_simulation
            .readings()?
            .iter()
            .any(|joint| joint.info.name == "shoulder" && joint.measured > 0.1),
        "MJCF arm did not move"
    );
    ensure!(
        arm_simulation.readings()?.len() == 2,
        "MJCF joint import failed"
    );
    println!("workbench smoke passed: URDF servo, MJCF articulation, pose/project roundtrip, visuals, LiDAR; state={:016x}", simulation.state_hash());
    Ok(())
}

fn main() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    match args.as_slice() {
        [command, path] if command == "--check-project" => {
            let bytes = read_bounded(path).context("read workbench project")?;
            let project = Project::from_json(&bytes)?;
            let joints = project.joint_catalog()?;
            let _simulation = simulation::Simulation::new(project.clone(), None)?;
            println!(
                "project valid: {} joints, {} scene objects, {} saved poses",
                joints.len(),
                project.objects.len(),
                project.poses.len()
            );
            Ok(())
        }
        [command, path] if command == "--replay" => replay_file(path, None),
        [command, path, flag, root] if command == "--replay" && flag == "--asset-root" => {
            replay_file(path, Some(std::path::Path::new(root)))
        }
        [command] if command == "--smoke" => smoke(),
        [command] if command == "--smoke-pick-place" => smoke_pick_place(),
        [command] if command == "--smoke-grasp" => smoke_grasp(),
        [command] if command == "--smoke-lekiwi" => smoke_lekiwi(),
        [command] if command == "--help" => {
            println!("rne-workbench [--port PORT] [--preset so101|lekiwi_so101|lekiwi_grasp|lekiwi_pick_place|arm|slider] [--robot MODEL.urdf|MODEL.xml] [--asset-root DIRECTORY]\nrne-workbench --check-project PROJECT.json\nrne-workbench --smoke\nrne-workbench --smoke-lekiwi\nrne-workbench --smoke-grasp\nrne-workbench --smoke-pick-place\nrne-workbench --replay RECORDING.json [--asset-root DIRECTORY]");
            Ok(())
        }
        _ => serve(&args),
    }
}

#[cfg(feature = "gpu")]
fn serve(args: &[String]) -> Result<()> {
    let mut port = 8765;
    let mut robot = None;
    let mut preset = "so101".to_string();
    let mut asset_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
    let mut args = args.iter();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--port" => port = args.next().context("--port requires a number")?.parse()?,
            "--preset" => preset = args.next().context("--preset requires a name")?.clone(),
            "--robot" => robot = Some(args.next().context("--robot requires a file")?.clone()),
            "--asset-root" => {
                asset_root =
                    PathBuf::from(args.next().context("--asset-root requires a directory")?)
            }
            _ => bail!("unknown argument {argument}; see --help"),
        }
    }
    let project = if let Some(path) = robot {
        let xml = String::from_utf8(read_bounded(&path)?)?;
        let format = if xml.contains("<mujoco") {
            RobotFormat::Mjcf
        } else {
            RobotFormat::Urdf
        };
        Project::new(RobotSource { format, xml })?
    } else {
        presets::preset(&preset)?
    };
    server::serve(host::Host::new(project, asset_root.canonicalize()?)?, port)
}

#[cfg(not(feature = "gpu"))]
fn serve(_args: &[String]) -> Result<()> {
    bail!("interactive workbench requires the gpu feature; headless --smoke and --check-project remain available")
}
