#!/usr/bin/env python3
"""Pure exact source reconstruction and forbidden source-delta checks."""

import json
from pathlib import Path

from audit_coupled_stages import UNIT_SOURCE, check_candidate_delta, record_for


def main():
    directory = Path(__file__).parent
    original = json.loads((directory / "baseline-source-manifest.json").read_bytes())["files"]
    rejected = json.loads((directory / "rejected-source-manifest.json").read_bytes())["files"]
    common = {"compiler": "synthetic source-map check only", "cargo": "synthetic", "target": "synthetic",
              "resolved_features": record_for(directory / "common142-resolved-features.txt"), "rustflags": [], "native_float_mode": {}}
    first, second = dict(common, repo_source_sha256=original), dict(common, repo_source_sha256=rejected)
    check_candidate_delta(first, second)
    rejected_cases = []
    def fails(name, altered):
        try:
            check_candidate_delta(first, dict(second, repo_source_sha256=altered))
        except ValueError as error:
            rejected_cases.append({"case": name, "actual_error": str(error)})
        else:
            raise AssertionError("forbidden source delta admitted")
    extra = dict(rejected); other = next(name for name in original if name != UNIT_SOURCE); extra[other] = "0" * 64
    fails("second candidate source path", extra)
    wrong = dict(rejected); wrong[UNIT_SOURCE] = "1" * 64
    fails("wrong reconstructed runtime source", wrong)
    missing = dict(rejected); missing.pop(other)
    fails("removed candidate source path", missing)
    fails("candidate equals baseline instead of historical hunk", dict(original))
    print(json.dumps({"status": "PASS", "native_worlds": 0, "scope": "pure source-byte reconstruction, no binary/world",
                      "motor_context_is_unique_and_limit_builder_unchanged": True, "rejected": rejected_cases}, indent=2))


if __name__ == "__main__":
    main()
