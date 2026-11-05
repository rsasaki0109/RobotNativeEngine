//! Contact cases with closed-form answers: Rapier against them and MuJoCo.
//!
//! Example 145 checked a pushed block against Coulomb friction. This example
//! takes the other textbook contact cases at the walking loop's 2 ms step: a
//! box launched across the floor in three directions, a box on two inclines, a
//! ball rolling down an incline with and without slipping, a ball bounced with
//! restitution 0 and 0.5, and a 100 kg box resting on a 1 kg box. Every case
//! has a closed-form answer. `mujoco_contact_closed_form.py` runs the same
//! cases in MuJoCo from this example's report and prints the closed form, RNE
//! and MuJoCo side by side.
//!
//! ```text
//! cargo run --release -p contact_closed_form --example 146_contact_closed_form
//! cargo run --release -p contact_closed_form --example 146_contact_closed_form -- --solver-iterations 16
//! ```
//!
//! Everything reported is in a Z-up frame (x forward, y left, z up), as in
//! MuJoCo. An incline tilts gravity toward +x rather than the floor.

mod scenario;

use std::path::PathBuf;

use serde_json::json;

use scenario::{run, Settings, SCENARIOS};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {
    let mut settings = Settings::default();
    let mut output = PathBuf::from("target/contact-closed-form");
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
        for line in scenario::summary_lines(&scenario, &result.summary) {
            println!("{line}");
        }
        results.push(json!({
            "scenario": scenario.to_json(),
            "closed_form": scenario.closed_form(),
            "summary": result.summary,
            "samples": result.samples,
        }));
    }
    std::fs::create_dir_all(&output)?;
    let path = output.join("contact-closed-form.json");
    let report = json!({
        "schema_version": 1,
        "engine": "RNE Rapier",
        "settings": settings.to_json(),
        "results": results,
    });
    std::fs::write(&path, serde_json::to_string_pretty(&report)? + "\n")?;
    println!("wrote {}", path.display());
    Ok(())
}
