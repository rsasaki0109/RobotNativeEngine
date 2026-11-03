//! A pushed block on four small spheres: Rapier's contact against Coulomb's
//! law and MuJoCo.
//!
//! Example 144 traced the trials where RNE and MuJoCo disagree to a foot
//! sliding and landing under a large horizontal push. This example takes the
//! foot alone: a 10 kg block standing on four 5 mm spheres, like the G1's foot,
//! is pushed horizontally for 0.1 s, or dropped tilted while moving, on a floor
//! of friction 1 at the walking loop's 2 ms step. `mujoco_contact_slide.py`
//! runs the same scenarios in MuJoCo from this example's report and prints the
//! two next to the closed-form answers where Coulomb friction gives one.
//!
//! ```text
//! cargo run --release -p contact_slide --example 145_contact_slide
//! cargo run --release -p contact_slide --example 145_contact_slide -- --solver-iterations 16
//! cargo run --release -p contact_slide --example 145_contact_slide -- --contact-softness 60,5
//! ```
//!
//! Everything reported is in a Z-up frame (x forward, y left, z up), as in
//! MuJoCo; pitch is the rotation about +y, positive nose down.

mod scenario;

use std::path::PathBuf;

use serde_json::json;

use scenario::{run, Settings, SCENARIOS};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {
    let mut settings = Settings::default();
    let mut output = PathBuf::from("target/contact-slide");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--output" => output = PathBuf::from(value()?),
            "--solver-iterations" => settings.solver_iterations = value()?.parse()?,
            "--contact-softness" => {
                let text = value()?;
                let (hz, ratio) = text
                    .split_once(',')
                    .ok_or("--contact-softness takes HZ,DAMPING_RATIO")?;
                settings.contact_softness = Some((hz.parse()?, ratio.parse()?));
            }
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    let mut results = Vec::new();
    for scenario in SCENARIOS {
        let result = run(scenario, settings)?;
        println!("{}", scenario::summary_line(scenario, &result));
        results.push(json!({
            "scenario": scenario.to_json(),
            "summary": result.summary,
            "samples": result.samples,
        }));
    }
    std::fs::create_dir_all(&output)?;
    let path = output.join("contact-slide.json");
    let report = json!({
        "schema_version": 1,
        "engine": "RNE Rapier",
        "settings": settings.to_json(),
        "block": scenario::block_json(),
        "results": results,
    });
    std::fs::write(&path, serde_json::to_string_pretty(&report)? + "\n")?;
    println!("wrote {}", path.display());
    Ok(())
}
