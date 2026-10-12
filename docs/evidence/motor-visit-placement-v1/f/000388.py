#!/usr/bin/python3
"""Prepare an external source copy without constructing any native world."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/workspace/RobotNativeEngine')
OWN = Path('/workspace/.rne-onboarding/recovery-20261012/next-motor-visit-placement-preparation')
COLLECTOR = OWN / 'collector'
OWN.mkdir()
COLLECTOR.mkdir()
shutil.copytree(ROOT / 'third_party/rapier3d', OWN / 'rapier3d', ignore=shutil.ignore_patterns('target', 'Cargo.lock'))
for name in ('lib.rs', 'main.rs', 'tests.rs'):
    shutil.copyfile(ROOT / 'examples/143_motor_construction' / name, COLLECTOR / name)
shutil.copyfile(ROOT / 'clippy.toml', COLLECTOR / 'clippy.toml')

path = COLLECTOR / 'lib.rs'
original = path.read_text()
s = original.replace('motor-construction collector', 'motor visitation-placement collector')
s = s.replace('isolated motor construction collection', 'motor visitation-placement collection')
s = s.replace('--target negative|positive', '--profile split_1_2|early_1_0|early_3_0 --target negative|positive')
marker = '#[derive(Debug)]\nstruct Options {'
profile = '''#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Profile {
    #[serde(rename = "split_1_2")]
    Split1_2,
    #[serde(rename = "early_1_0")]
    Early1_0,
    #[serde(rename = "early_3_0")]
    Early3_0,
}

impl Profile {
    fn passes(self) -> (usize, usize) {
        match self {
            Self::Split1_2 => (1, 2),
            Self::Early1_0 => (1, 0),
            Self::Early3_0 => (3, 0),
        }
    }
}

'''
assert s.count(marker) == 1
s = s.replace(marker, profile + marker)
s = s.replace('struct Options {\n', 'struct Options {\n    profile: Profile,\n', 1)
s = s.replace('(mut target, mut observer, mut label, mut output) = (None, None, None, None)', '(mut profile, mut target, mut observer, mut label, mut output) = (None, None, None, None, None)')
s = s.replace('match arg.as_str() {\n', '''match arg.as_str() {
                "--profile" if profile.is_none() => {
                    profile = Some(match value.as_str() {
                        "split_1_2" => Profile::Split1_2,
                        "early_1_0" => Profile::Early1_0,
                        "early_3_0" => Profile::Early3_0,
                        _ => return Err("profile must be split_1_2, early_1_0 or early_3_0".into()),
                    });
                }
''', 1)
s = s.replace('target: target.ok_or', 'profile: profile.ok_or("--profile is required")?,\n            target: target.ok_or', 1)
s = s.replace("struct RunStart<'a> {\n", "struct RunStart<'a> {\n    study_id: &'static str,\n    profile: Profile,\n", 1)
s = s.replace('fn fixed() -> Self {\n', 'fn fixed(profile: Profile) -> Self {\n        let (pgs_passes, stabilization_passes) = profile.passes();\n', 1)
s = s.replace('pgs_passes: 1,\n            stabilization_passes: 2,', 'pgs_passes,\n            stabilization_passes,', 1)
s = s.replace('writer.record(&RunStart {\n', 'writer.record(&RunStart {\n            study_id: "motor_visit_placement_v1",\n            profile: options.profile,\n', 1)
s = s.replace('Conditions::fixed()', 'Conditions::fixed(options.profile)', 1)
s = s.replace('fn new(target: Target, observer_enabled: bool)', 'fn new(profile: Profile, target: Target, observer_enabled: bool)', 1)
s = s.replace('num_internal_pgs_iterations: 1,\n                num_internal_stabilization_iterations: 2,', 'num_internal_pgs_iterations: profile.passes().0,\n                num_internal_stabilization_iterations: profile.passes().1,', 1)
s = s.replace('Fixture::new(options.target, options.observer_enabled)', 'Fixture::new(options.profile, options.target, options.observer_enabled)', 1)
path.write_text(s)

tests = COLLECTOR / 'tests.rs'
original_tests = tests.read_text()
t = original_tests.replace('Options {\n        target:', 'Options {\n        profile: Profile::Split1_2,\n        target:', 1)
t = t.replace('"--target",\n                target,', '"--profile",\n                "split_1_2",\n                "--target",\n                target,', 1)
t += '''
#[test]
fn profile_parser_and_headers_use_only_registered_counts() {
    for (name, expected) in [("split_1_2", (1, 2)), ("early_1_0", (1, 0)), ("early_3_0", (3, 0))] {
        let parsed = Options::parse(["--profile", name, "--target", "positive", "--observer", "on", "--label", "pure", "--output", "/workspace/unused-pure.gz"].map(str::to_owned)).expect("pure parse");
        assert_eq!(parsed.profile.passes(), expected);
        let conditions = Conditions::fixed(parsed.profile);
        assert_eq!((conditions.pgs_passes, conditions.stabilization_passes), expected);
        assert_eq!(conditions.small_steps, 32);
        assert_eq!(serde_json::to_value(parsed.profile).expect("profile encoding"), name);
    }
    for name in ["", "1", "early_2_0", "pgs1_s2"] {
        assert!(Options::parse(["--profile", name].map(str::to_owned)).is_err());
    }
}
'''
tests.write_text(t)
main = COLLECTOR / 'main.rs'
main.write_text(main.read_text().replace('isolated_motor_construction_collector', 'motor_visit_placement_collector').replace('isolated motor construction collection', 'motor visitation-placement collection'))

manifest = '''[workspace]

[package]
name = "motor_visit_placement_collector"
version = "0.1.0"
edition = "2021"
rust-version = "1.88.0"
license = "MIT OR Apache-2.0"
publish = false

[lib]
path = "lib.rs"

[[bin]]
name = "motor_visit_placement"
path = "main.rs"

[dependencies]
flate2 = "=1.1.9"
postcard = { version = "=1.1.3", default-features = false, features = ["alloc"] }
rapier3d = { path = "../rapier3d", features = ["serde-serialize"] }
serde = { version = "=1.0.228", features = ["derive", "rc"] }
serde_json = { version = "=1.0.150", features = ["float_roundtrip", "raw_value"] }

[lints.rust]
unreachable_pub = "warn"
missing_debug_implementations = "warn"
unused_qualifications = "warn"

[lints.clippy]
todo = "deny"
unimplemented = "deny"
dbg_macro = "deny"
too_many_lines = "warn"
'''
(COLLECTOR / 'Cargo.toml').write_text(manifest)
shutil.copyfile(ROOT / 'Cargo.lock', COLLECTOR / 'Cargo.lock')
diff = list(difflib.unified_diff(original.splitlines(True), s.splitlines(True), fromfile='current143/lib.rs', tofile='collector/lib.rs'))
diff += list(difflib.unified_diff(original_tests.splitlines(True), t.splitlines(True), fromfile='current143/tests.rs', tofile='collector/tests.rs'))
(OWN / 'collector-source-delta.patch').write_text(''.join(diff))
sources = {}
for p in sorted((OWN / 'rapier3d').rglob('*')):
    if p.is_file():
        rel = p.relative_to(OWN / 'rapier3d')
        baseline = ROOT / 'third_party/rapier3d' / rel
        assert p.read_bytes() == baseline.read_bytes()
        sources[str(rel)] = {'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'bytes': p.stat().st_size, 'copied_path': str(p), 'baseline_path': str(baseline)}
(OWN / 'unchanged-vendor-copy.json').write_text(json.dumps({'head': '5e8e3627cd91df767ae41e8364f9401a8cd4e2d7', 'files': sources, 'exclusions': ['Cargo.lock', 'target']}, indent=2) + '\n')
print(json.dumps({'collector': str(COLLECTOR), 'vendor_files': len(sources), 'native_worlds_executed': 0}))
