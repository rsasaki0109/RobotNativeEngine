//! Pure CLI, claim/framing and opaque-byte checks. No Fixture or pipeline call.
use super::*;
use flate2::read::GzDecoder;
use std::io::Read;
use std::sync::atomic::{AtomicUsize, Ordering};

fn directory() -> PathBuf {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("pure-test-artifacts");
    std::fs::create_dir_all(&root).expect("create task-owned pure directory");
    for ordinal in 0..10000 {
        let path = root.join(format!("pid{}-case{}", std::process::id(), ordinal));
        match std::fs::create_dir(&path) {
            Ok(()) => return path,
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => panic!("pure directory claim failed: {error}"),
        }
    }
    panic!("pure directory claim limit reached")
}

fn options(path: &Path) -> Options {
    Options {
        case: CASES[0],
        variant: Variant::Baseline,
        observer_enabled: true,
        label: "SYNTHETIC_NO_NATIVE_WORLD".to_owned(),
        output: path.join("synthetic.jsonl.gz"),
    }
}

fn records(path: &Path) -> Vec<serde_json::Value> {
    let file = File::open(path).expect("retained pure gzip");
    let mut decoder = GzDecoder::new(file);
    let mut raw = String::new();
    decoder
        .read_to_string(&mut raw)
        .expect("complete gzip footer and CRC");
    raw.lines()
        .map(|line| serde_json::from_str(line).expect("synthetic JSON"))
        .collect()
}

#[test]
fn fixed_cli_accepts_twenty_cases_and_two_variant_labels_without_tuning() {
    for case in CASES {
        for variant in ["baseline", "candidate"] {
            let parsed = Options::parse(
                [
                    "--case",
                    case.case_id,
                    "--variant",
                    variant,
                    "--observer",
                    "off",
                    "--label",
                    "pure",
                    "--output",
                    "/workspace/unused-pure.jsonl.gz",
                ]
                .map(str::to_owned),
            )
            .expect("fixed CLI parse only");
            assert_eq!(parsed.case.case_id, case.case_id);
            assert_eq!(
                parsed.case.target.position_rad().to_bits(),
                case.target.position_rad().to_bits()
            );
            assert!(!parsed.observer_enabled);
            assert_eq!(
                serde_json::to_value(parsed.variant).expect("variant label"),
                variant
            );
        }
    }
    for args in [
        vec![],
        vec!["--case", "old143"],
        vec!["--variant", "auto"],
        vec!["--case", CASES[0].case_id, "--case", CASES[1].case_id],
        vec!["--observer", "maybe"],
        vec!["--label", "a/b"],
        vec!["--output", "relative.gz"],
        vec!["--gain", "300"],
        vec!["--target", "negative"],
        vec!["--mass", "0.5"],
        vec!["--timestep", "0.002"],
        vec!["--cap", "2"],
        vec!["--output"],
    ] {
        assert!(Options::parse(args.into_iter().map(str::to_owned)).is_err());
    }
}

#[test]
fn input_table_exact_order_words_and_zero_controls_are_source_bound() {
    let expected = [
        (
            "m05_h05_neg_cap2",
            0x3f000000_u32,
            0x3a03126f_u32,
            0xba83126f_u32,
            0x40000000_u32,
        ),
        (
            "m05_h05_neg_cap001",
            0x3f000000_u32,
            0x3a03126f_u32,
            0xba83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m05_h05_pos_cap2",
            0x3f000000_u32,
            0x3a03126f_u32,
            0x3a83126f_u32,
            0x40000000_u32,
        ),
        (
            "m05_h05_pos_cap001",
            0x3f000000_u32,
            0x3a03126f_u32,
            0x3a83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m05_h05_zero_cap2",
            0x3f000000_u32,
            0x3a03126f_u32,
            0x00000000_u32,
            0x40000000_u32,
        ),
        (
            "m05_h2_neg_cap2",
            0x3f000000_u32,
            0x3b03126f_u32,
            0xba83126f_u32,
            0x40000000_u32,
        ),
        (
            "m05_h2_neg_cap001",
            0x3f000000_u32,
            0x3b03126f_u32,
            0xba83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m05_h2_pos_cap2",
            0x3f000000_u32,
            0x3b03126f_u32,
            0x3a83126f_u32,
            0x40000000_u32,
        ),
        (
            "m05_h2_pos_cap001",
            0x3f000000_u32,
            0x3b03126f_u32,
            0x3a83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m05_h2_zero_cap2",
            0x3f000000_u32,
            0x3b03126f_u32,
            0x00000000_u32,
            0x40000000_u32,
        ),
        (
            "m2_h05_neg_cap2",
            0x40000000_u32,
            0x3a03126f_u32,
            0xba83126f_u32,
            0x40000000_u32,
        ),
        (
            "m2_h05_neg_cap001",
            0x40000000_u32,
            0x3a03126f_u32,
            0xba83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m2_h05_pos_cap2",
            0x40000000_u32,
            0x3a03126f_u32,
            0x3a83126f_u32,
            0x40000000_u32,
        ),
        (
            "m2_h05_pos_cap001",
            0x40000000_u32,
            0x3a03126f_u32,
            0x3a83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m2_h05_zero_cap2",
            0x40000000_u32,
            0x3a03126f_u32,
            0x00000000_u32,
            0x40000000_u32,
        ),
        (
            "m2_h2_neg_cap2",
            0x40000000_u32,
            0x3b03126f_u32,
            0xba83126f_u32,
            0x40000000_u32,
        ),
        (
            "m2_h2_neg_cap001",
            0x40000000_u32,
            0x3b03126f_u32,
            0xba83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m2_h2_pos_cap2",
            0x40000000_u32,
            0x3b03126f_u32,
            0x3a83126f_u32,
            0x40000000_u32,
        ),
        (
            "m2_h2_pos_cap001",
            0x40000000_u32,
            0x3b03126f_u32,
            0x3a83126f_u32,
            0x3c23d70a_u32,
        ),
        (
            "m2_h2_zero_cap2",
            0x40000000_u32,
            0x3b03126f_u32,
            0x00000000_u32,
            0x40000000_u32,
        ),
    ];
    let mut ids = std::collections::BTreeSet::new();
    let mut combinations = std::collections::BTreeSet::new();
    let mut zeros = 0;
    for (case, (id, mass, h, target, cap)) in CASES.into_iter().zip(expected) {
        assert!(ids.insert(case.case_id));
        assert_eq!(case.case_id, id);
        assert_eq!(
            (
                case.mass_kg.to_bits(),
                case.outer_dt_s.to_bits(),
                case.target.position_rad().to_bits(),
                case.ceiling_nm.to_bits()
            ),
            (mass, h, target, cap)
        );
        assert!(combinations.insert((mass, h, target, cap)));
        let condition = Conditions::fixed(case);
        assert_eq!(condition.sphere_radius_m_word, 0x3dcccccd);
        assert_eq!(condition.stiffness_nm_per_rad_word, 0x43960000);
        assert_eq!(condition.damping_nm_s_per_rad_word, 0x41400000);
        assert_eq!(
            (
                condition.small_steps,
                condition.pgs_passes,
                condition.stabilization_passes
            ),
            (1, 1, 2)
        );
        assert_eq!(condition.initial_coordinate_rad_word, 0);
        assert_eq!(condition.target_velocity_rad_s_word, 0);
        if target == 0 {
            zeros += 1;
            assert_eq!(cap, 0x40000000);
            assert_eq!(case.target, Target::Zero);
        }
    }
    assert_eq!(ids.len(), 20);
    assert_eq!(combinations.len(), 20);
    assert_eq!(zeros, 4);
}

#[test]
fn missing_or_repeated_mandatory_flags_never_reach_a_callback() {
    let valid = [
        "--case",
        CASES[0].case_id,
        "--variant",
        "baseline",
        "--observer",
        "on",
        "--label",
        "pure",
        "--output",
        "/workspace/unused-pure.jsonl.gz",
    ];
    for omitted in 0..5 {
        let args = valid
            .chunks_exact(2)
            .enumerate()
            .filter(|(index, _)| *index != omitted)
            .flat_map(|(_, pair)| pair.iter().map(|s| (*s).to_owned()))
            .collect::<Vec<_>>();
        assert!(Options::parse(args).is_err());
    }
    for pair in valid.chunks_exact(2) {
        let mut args = valid.map(str::to_owned).to_vec();
        args.extend(pair.iter().map(|s| (*s).to_owned()));
        assert!(Options::parse(args).is_err());
    }
}

#[test]
fn synthetic_zero_header_keeps_new_kind_case_and_literal_variant_without_fixture() {
    let dir = directory();
    let mut opt = options(&dir);
    opt.case = declared_case("m05_h05_zero_cap2").expect("declared zero");
    opt.variant = Variant::Candidate;
    assert!(run_claimed(&opt, |_, _| Err(
        "SYNTHETIC stop after header; no Fixture".into()
    ))
    .is_err());
    let saved = records(&opt.output);
    assert_eq!(saved[0]["study_kind"], STUDY_KIND);
    assert_eq!(saved[0]["case_id"], "m05_h05_zero_cap2");
    assert_eq!(saved[0]["variant"], "candidate");
    assert_eq!(saved[0]["target"], "zero");
    assert_eq!(saved[0]["target_position_rad_word"], 0);
    assert_eq!(saved[0]["declared_conditions"]["small_steps"], 1);
    assert_eq!(saved[1]["progress"]["fixture_construction_started"], false);
    assert_eq!(saved[1]["status"], "failed");
}

#[test]
fn existing_claim_blocks_before_callback_without_overwrite() {
    let dir = directory();
    let opt = options(&dir);
    std::fs::write(&opt.output, b"SYNTHETIC EXISTING CLAIM").expect("pure claim");
    let called = AtomicUsize::new(0);
    let result = run_claimed(&opt, |_, _| {
        called.fetch_add(1, Ordering::SeqCst);
        Ok(())
    });
    assert!(result.is_err());
    assert_eq!(called.load(Ordering::SeqCst), 0);
    assert_eq!(
        std::fs::read(&opt.output).expect("read pure claim"),
        b"SYNTHETIC EXISTING CLAIM"
    );
}

#[test]
fn exclusive_header_precedes_stub_callback_and_keeps_error_terminal() {
    let dir = directory();
    let opt = options(&dir);
    let result = run_claimed(&opt, |writer, progress| {
        assert!(opt.output.is_file());
        assert!(std::fs::metadata(&opt.output)?.len() > 10);
        writer.record(&serde_json::json!({"kind":"synthetic_partial","ieee_words":[0,2147483648_u32,2143289345_u32]}))?;
        progress.partial_records_written += 1;
        Err("SYNTHETIC deliberate collector error; no native fixture".into())
    });
    assert!(result.is_err());
    let saved = records(&opt.output);
    assert_eq!(saved.len(), 3);
    assert_eq!(saved[0]["kind"], "run_start");
    assert_eq!(saved[2]["status"], "failed");
    assert_eq!(saved[2]["progress"]["fixture_constructed"], false);
    assert_eq!(saved[2]["progress"]["partial_records_written"], 1);
}

#[test]
fn stub_panic_is_retained_and_empty_success_is_not_collected() {
    let dir = directory();
    let opt = options(&dir);
    assert!(run_claimed(&opt, |_, _| -> Result<()> {
        panic!("SYNTHETIC stub panic; no engine")
    })
    .is_err());
    let saved = records(&opt.output);
    assert_eq!(saved[1]["status"], "failed");
    assert!(saved[1]["error"]
        .as_str()
        .expect("failure text")
        .contains("SYNTHETIC stub panic"));
    let dir = directory();
    let opt = options(&dir);
    assert!(run_claimed(&opt, |_, _| Ok(())).is_err());
    assert_eq!(records(&opt.output)[1]["status"], "failed");
}

#[test]
fn synthetic_completed_framing_is_deterministic() {
    fn pure(path: &Path) -> Vec<u8> {
        let opt = options(path);
        run_claimed(&opt, |writer, progress| {
            writer.record(
                &serde_json::json!({"kind":"synthetic_initial","bits":[0,2147483648_u32]}),
            )?;
            writer.record(
                &serde_json::json!({"kind":"synthetic_complete","bits":[2143289345_u32]}),
            )?;
            // Only invented framing counters, never a Fixture/pipeline call.
            progress.pipeline_step_returned = true;
            progress.completed_outer_intervals = 1;
            progress.initial_records_written = 1;
            progress.completed_records_written = 1;
            Ok(())
        })
        .expect("pure framing only");
        let saved = records(&opt.output);
        assert_eq!(saved.len(), 4);
        assert_eq!(saved[3]["status"], "collected");
        std::fs::read(opt.output).expect("pure gzip bytes")
    }
    let a = pure(&directory());
    let b = pure(&directory());
    assert_eq!(a, b);
    assert_eq!(&a[0..4], &[0x1f, 0x8b, 8, 0]);
    assert_eq!(&a[4..8], &[0; 4]);
    assert_eq!(a[9], 255);
}

#[test]
fn postcard_opaque_f32_words_preserve_zero_nan_and_infinity() {
    #[derive(Debug, Serialize, Deserialize)]
    struct OpaqueProbe {
        values: Vec<f32>,
    }
    let expected = [0, 0x80000000, 0x7fc00001, 0xffc00002, 0x7f800000, 1];
    let probe = OpaqueProbe {
        values: expected.map(f32::from_bits).to_vec(),
    };
    let bytes = exact_native_bytes(&probe).expect("opaque synthetic words");
    let (decoded, remainder): (OpaqueProbe, _) =
        postcard::take_from_bytes(&bytes).expect("exact opaque decode");
    assert!(remainder.is_empty());
    assert_eq!(
        decoded
            .values
            .iter()
            .map(|x| x.to_bits())
            .collect::<Vec<_>>(),
        expected
    );
    let encoded = serde_json::to_vec(&ObjectBytes {
        kind: ObjectKind::Body,
        handle_words: [123, 7],
        postcard_bytes: bytes.clone(),
    })
    .expect("byte vector JSON");
    let value: serde_json::Value = serde_json::from_slice(&encoded).expect("integer-only wrapper");
    assert_eq!(
        value["postcard_bytes"],
        serde_json::to_value(bytes).expect("literal bytes")
    );
}

#[test]
fn impulse_promoted_f64_words_keep_signed_zero_and_absence() {
    let observed = impulse_value(MultibodyMotorImpulseValue::Revolute {
        applied_impulse_nm_s: f64::from_bits(0x8000000000000000),
        effective_effort_ceiling_nm: None,
        rounded_impulse_bound_nm_s: Some(f64::from_bits(1)),
    });
    assert_eq!(observed.applied_impulse_word, 0x8000000000000000);
    assert_eq!(observed.effort_ceiling_word, None);
    assert_eq!(observed.rounded_bound_word, Some(1));
}
