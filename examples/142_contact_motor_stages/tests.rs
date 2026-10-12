use super::*;
use flate2::read::GzDecoder;
use serde_json::Value;
use std::io::{self, Read};
use std::sync::{Arc, Mutex};

#[derive(Clone, Debug, Default)]
struct SharedBytes(Arc<Mutex<Vec<u8>>>);

impl Write for SharedBytes {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn decode_records(bytes: &[u8]) -> Vec<Value> {
    let mut text = String::new();
    GzDecoder::new(bytes).read_to_string(&mut text).unwrap();
    text.lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

fn args() -> Vec<String> {
    [
        "--mass-kg",
        "5",
        "--observer",
        "on",
        "--label",
        "synthetic",
        "--output",
        "synthetic.jsonl.gz",
    ]
    .map(String::from)
    .to_vec()
}

#[test]
fn strict_cli_rejects_undeclared_repeated_and_missing_values() {
    assert!(Options::parse(args()).unwrap().is_some());
    assert!(Options::parse(vec!["--help".into()]).unwrap().is_none());
    for invalid in ["0", "5.0", "05", "10", "NaN", "inf"] {
        let mut invalid_args = args();
        invalid_args[1] = invalid.into();
        assert!(Options::parse(invalid_args).is_err());
    }
    for invalid in ["yes", "ON", "0"] {
        let mut invalid_args = args();
        invalid_args[3] = invalid.into();
        assert!(Options::parse(invalid_args).is_err());
    }
    for flag in ["--mass-kg", "--observer", "--label", "--output"] {
        let mut repeated = args();
        repeated.extend([flag.into(), "5".into()]);
        assert!(Options::parse(repeated).is_err());
    }
    for pair in 0..4 {
        let mut missing = args();
        missing.drain(2 * pair..2 * pair + 2);
        assert!(Options::parse(missing).is_err());
    }
    assert!(validate_label("../unsafe").is_err());
    assert!(validate_label("label\nline").is_err());
    assert!(validate_label("").is_err());
}

fn synthetic_encoded_run() -> (Vec<u8>, Vec<u64>) {
    let sink = SharedBytes::default();
    let expected = vec![
        u64::from(0.0_f32.to_bits()),
        u64::from((-0.0_f32).to_bits()),
        u64::from(f32::INFINITY.to_bits()),
        u64::from(f32::from_bits(0x7fc0_0042).to_bits()),
        (-0.0_f64).to_bits(),
        u64::MAX,
    ];
    let result = write_run(5.0, true, "synthetic", sink.clone(), |stream| {
        stream.progress.completed_outer_steps = 1;
        stream.frame(&serde_json::json!({
            "kind": "completed_step", "schema_version": 1,
            "completed_outer_step": 1,
            "synthetic_native_words": expected,
            "synthetic_unavailable": null,
        }))?;
        Err("synthetic data only; no world or acceptance result".into())
    });
    assert!(result.is_err());
    let encoded = sink.0.lock().unwrap().clone();
    (encoded, expected)
}

#[test]
fn gzip_roundtrip_preserves_ieee_words_and_unavailable_presence() {
    let (encoded, words) = synthetic_encoded_run();
    let records = decode_records(&encoded);
    assert_eq!(records.len(), 3);
    let actual: Vec<u64> = records[1]["synthetic_native_words"]
        .as_array()
        .unwrap()
        .iter()
        .map(|word| word.as_u64().unwrap())
        .collect();
    assert_eq!(actual, words);
    assert_ne!(actual[0], actual[1]);
    assert!(records[1]["synthetic_unavailable"].is_null());
    assert_eq!(records[2]["status"], "failed");
}

#[test]
fn gzip_encoding_has_fixed_header_and_literal_fresh_replay() {
    let (first, _) = synthetic_encoded_run();
    let (fresh, _) = synthetic_encoded_run();
    assert_eq!(first, fresh);
    assert_eq!(&first[4..8], &[0, 0, 0, 0]);
    assert_eq!(first[9], 255);
    assert_eq!(first[3], 0);
}

#[test]
fn collection_error_retains_complete_frames_and_failed_terminal() {
    let (encoded, _) = synthetic_encoded_run();
    let records = decode_records(&encoded);
    assert_eq!(records[0]["kind"], "run_start");
    assert_eq!(records[0]["expected_completed_outer_steps"], 1250);
    assert_eq!(records[2]["kind"], "run_end");
    assert_eq!(records[2]["completed_outer_steps"], 1);
    assert_eq!(records[2]["frames_written"], 1);
    assert!(records[2]["physical_report"].is_null());
    assert!(records[2]["error"]
        .as_str()
        .unwrap()
        .contains("synthetic data only"));
}

#[test]
fn unwind_panic_retains_failed_terminal_and_valid_gzip_footer() {
    let sink = SharedBytes::default();
    let result = write_run(5.0, false, "synthetic", sink.clone(), |_stream| {
        panic!("synthetic collection panic; no native world")
    });
    assert!(result
        .unwrap_err()
        .to_string()
        .contains("synthetic collection panic"));
    let records = decode_records(&sink.0.lock().unwrap());
    assert_eq!(records.len(), 2);
    assert_eq!(records[1]["status"], "failed");
    assert_eq!(records[1]["frames_written"], 0);
}

#[derive(Debug)]
struct FlushFailure;

impl Write for FlushFailure {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Err(io::Error::other("synthetic sink flush failure"))
    }
}

#[test]
fn gzip_finish_and_sink_flush_errors_are_surfaced() {
    let result = write_run(5.0, false, "synthetic", FlushFailure, |_stream| {
        Err("synthetic collection error".into())
    });
    let error = result.unwrap_err().to_string();
    assert!(error.contains("collection error: synthetic collection error"));
    assert!(error.contains("gzip finish/flush error: synthetic sink flush failure"));
}

#[derive(Debug)]
struct FailingRecord;

impl Serialize for FailingRecord {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> std::result::Result<S::Ok, S::Error> {
        let mut record = serializer.serialize_struct("FailingRecord", 1)?;
        serde::ser::SerializeStruct::serialize_field(&mut record, "partial", &1_u8)?;
        Err(serde::ser::Error::custom("synthetic serialization error"))
    }
}

#[test]
fn serialization_failure_does_not_insert_half_a_json_line() {
    let sink = SharedBytes::default();
    let result = write_run(5.0, false, "synthetic", sink.clone(), |stream| {
        stream.frame(&FailingRecord)?;
        unreachable!("synthetic serializer must fail before a completed record")
    });
    assert!(result
        .unwrap_err()
        .to_string()
        .contains("synthetic serialization error"));
    let records = decode_records(&sink.0.lock().unwrap());
    assert_eq!(records.len(), 2);
    assert_eq!(records[1]["status"], "failed");
    assert_eq!(records[1]["frames_written"], 0);
}

#[test]
fn an_incomplete_success_cannot_emit_a_collected_terminal() {
    let sink = SharedBytes::default();
    let result = write_run(5.0, false, "synthetic", sink.clone(), |_stream| {
        Ok(PhysicalReport {
            cube_ecs_pose_ieee_words: vec![],
            physical_ieee_words: [0; 4],
            physical_stdout_row: "synthetic report; no native world".into(),
        })
    });
    assert!(result
        .unwrap_err()
        .to_string()
        .contains("incomplete collection"));
    let records = decode_records(&sink.0.lock().unwrap());
    assert_eq!(records[1]["status"], "failed");
    assert!(records[1]["physical_report"].is_null());
}
