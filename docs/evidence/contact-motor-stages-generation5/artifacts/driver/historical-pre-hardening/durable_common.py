"""Durable file primitives for the separate generation-5 cohort.

Every new artifact uses exclusive creation and an fsync of its parent directory.
No helper deletes, replaces, repairs, or resumes an existing collection artifact.
"""

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re


GENERATION_ROOT = Path(
    "/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-5"
)
BUILDS = ("baseline", "rejected_inverse_w_plus_g")
MASSES = (5, 20, 50)
MODES = ("on", "fresh_on", "off")
INDEPENDENCE = "NEW_INDEPENDENT_COHORT_NOT_RESTORATION_OF_LOST_GENERATION4_TAR"
SPEC_STATUS = "FROZEN_BEFORE_GENERATION5_COLLECTION"
BLOCK = 1 << 20


def now():
    return datetime.now(timezone.utc).isoformat()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def literal_json(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def strict_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key: " + key)
            result[key] = value
        return result

    def invalid(value):
        raise ValueError("nonfinite JSON token: " + value)

    return json.loads(Path(path).read_bytes(), object_pairs_hook=pairs, parse_constant=invalid)


def persistent_path(path, *, beneath=None):
    value = Path(path)
    require(value.is_absolute(), "artifact path is not absolute")
    resolved = value.resolve()
    require(str(value) == str(resolved), "artifact path is not canonical or traverses a symlink")
    require(resolved.is_relative_to(Path("/workspace")), "artifact is outside persistent /workspace")
    if beneath is not None:
        require(resolved.is_relative_to(Path(beneath).resolve()), "artifact is outside its cohort root")
    return resolved


def fsync_directory(path):
    descriptor = os.open(Path(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def mkdir_new(path):
    path = persistent_path(path)
    path.mkdir(exist_ok=False)
    fsync_directory(path.parent)


def open_new(path):
    path = persistent_path(path)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    fsync_directory(path.parent)
    return os.fdopen(descriptor, "wb")


def sync_stream(stream):
    stream.flush()
    os.fsync(stream.fileno())


def write_new_bytes(path, content):
    with open_new(path) as stream:
        stream.write(content)
        sync_stream(stream)
    fsync_directory(Path(path).parent)
    return artifact(path)


def write_new_json(path, value):
    return write_new_bytes(path, literal_json(value))


def stream_hash(path):
    digest = hashlib.sha256()
    count = 0
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(BLOCK), b""):
            digest.update(chunk)
            count += len(chunk)
    return count, digest.hexdigest()


def artifact(path):
    path = persistent_path(path)
    require(path.is_file(), "missing regular artifact: " + str(path))
    count, digest = stream_hash(path)
    require(path.stat().st_size == count, "artifact size changed while hashing")
    return {"path": str(path), "bytes": count, "sha256": digest}


def validate_artifact(record, *, beneath=None):
    require(type(record) is dict and set(record) == {"path", "bytes", "sha256"}, "invalid artifact descriptor")
    require(type(record["bytes"]) is int and record["bytes"] >= 0, "invalid artifact byte count")
    require(type(record["sha256"]) is str and re.fullmatch("[0-9a-f]{64}", record["sha256"]), "invalid SHA256")
    persistent_path(record["path"], beneath=beneath)
    require(artifact(record["path"]) == record, "artifact binding mismatch: " + record["path"])
    return record


def fixed_cells():
    return [
        {"build": build, "mass_kg": mass, "mode": mode, "run_label": f"{build}-m{mass}-{mode}"}
        for build in BUILDS for mass in MASSES for mode in MODES
    ]


def source_snapshot(repository, manifest):
    """Hash exactly the preregistered sorted input paths, not a dependency closure."""
    repository = persistent_path(repository)
    require(type(manifest.get("scope")) is str and "non-exhaustive" in manifest["scope"].lower(),
            "source guard must declare its non-exhaustive scope")
    files = manifest.get("files")
    require(type(files) is dict and files and list(files) == sorted(files), "source input map is not sorted/nonempty")
    observed = {}
    for relative, digest in files.items():
        require(type(relative) is str and Path(relative).as_posix() == relative and not Path(relative).is_absolute()
                and all(part not in (".", "..", "") for part in relative.split("/")), "invalid source path")
        require(type(digest) is str and re.fullmatch("[0-9a-f]{64}", digest), "invalid source digest")
        path = persistent_path(repository / relative, beneath=repository)
        observed[relative] = stream_hash(path)[1]
    return {"scope": manifest["scope"], "files": observed}


def validate_spec(spec_record):
    """Fail before any claim or launch unless all frozen artifacts still bind."""
    validate_artifact(spec_record, beneath=GENERATION_ROOT)
    spec = strict_json(spec_record["path"])
    require(spec.get("schema_version") == 1 and spec.get("generation") == 5, "wrong driver spec schema/generation")
    require(spec.get("status") == SPEC_STATUS and spec.get("independence") == INDEPENDENCE,
            "driver spec is not a frozen independent generation-5 cohort")
    require(type(spec.get("cohort_id")) is str and bool(spec["cohort_id"]), "missing cohort identity")
    require(spec.get("authoritative_worlds_of_this_cohort_before_freeze") == 0, "cohort has already executed worlds")
    require(spec.get("fixed_cells") == fixed_cells() and spec.get("world_limit") == 18,
            "fixed 18-cell matrix/order changed")
    require(spec.get("preflight_gates") == {"reader_pure_tests": "PASS", "independent_readonly_review": "PASS"},
            "pre-world reader/review gates have not passed")
    persistent_path(spec["cohort_root"], beneath=GENERATION_ROOT)
    persistent_path(spec["repository_root"])
    for role in ("protocol", "reader_pure_test_receipt", "independent_readonly_review", "source_guard_manifest"):
        validate_artifact(spec[role], beneath=GENERATION_ROOT)
    protocol = strict_json(spec["protocol"]["path"])
    require(protocol.get("fixed_cells") == fixed_cells(), "protocol matrix/order differs")
    require(protocol.get("authoritative_coupled_worlds") == 18, "protocol does not register exactly 18 worlds")
    require(type(spec.get("reader_artifacts")) is list and spec["reader_artifacts"], "reader artifacts are not frozen")
    for record in spec["reader_artifacts"]:
        validate_artifact(record, beneath=GENERATION_ROOT)
    require(type(spec.get("driver_artifacts")) is list and spec["driver_artifacts"], "driver artifacts are not frozen")
    for record in spec["driver_artifacts"]:
        validate_artifact(record, beneath=GENERATION_ROOT / "driver")
    module_records = {record["path"]: record for record in spec["driver_artifacts"]}
    for name in ("durable_common.py", "collect_once.py", "prepare_archive.py"):
        path = Path(__file__).resolve().parent / name
        require(module_records.get(str(path)) == artifact(path), "driver module omitted from freeze: " + name)
    require(type(spec.get("bindings")) is dict and set(spec["bindings"]) == set(BUILDS), "wrong build bindings")
    bindings = {}
    for build in BUILDS:
        record = spec["bindings"][build]
        validate_artifact(record, beneath=GENERATION_ROOT)
        binding = strict_json(record["path"])
        require(binding.get("build") == build and binding.get("cohort_id") == spec["cohort_id"], "wrong binding identity")
        require(binding.get("protocol_sha256") == spec["protocol"]["sha256"]
                and binding.get("authoritative_worlds_of_this_cohort_before_freeze") == 0,
                "binding is not frozen before this independent cohort")
        require(binding.get("status") == "FROZEN_BEFORE_GENERATION5_COHORT_WORLDS", "unfrozen source/binary binding")
        validate_artifact(binding["binary"], beneath=GENERATION_ROOT)
        require(os.access(binding["binary"]["path"], os.X_OK), "frozen binary is not executable")
        validate_artifact(binding["source_manifest"], beneath=GENERATION_ROOT)
        bindings[build] = binding
    manifest = strict_json(spec["source_guard_manifest"]["path"])
    expected = {"scope": manifest["scope"], "files": manifest["files"]}
    require(source_snapshot(spec["repository_root"], manifest) == expected, "restored source input guard differs")
    return spec, bindings, expected
