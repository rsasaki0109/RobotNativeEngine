#!/usr/bin/env python3
"""Pure metadata/hash/provenance checks; no complete native trace reread."""

import copy
import datetime
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile

from audit_coupled_stages import check_repair, record_for


def main():
    here = Path(__file__).parent
    original = here.parent
    base = json.loads((here / "repair-manifest-candidate-for-pure-tests.json").read_bytes())
    protocol = record_for(original / "frozen-protocol.json")
    collection = record_for(original / "actual-coupled-collection-index.json")
    rejected = []
    with tempfile.TemporaryDirectory(prefix="rne-pure-repair-provenance-", dir="/tmp") as directory:
        directory = Path(directory)
        def arguments(document):
            target = directory / (str(len(list(directory.iterdir()))) + ".json")
            target.write_text(json.dumps(document, indent=2) + "\n")
            return SimpleNamespace(repair_manifest=str(target), repair_manifest_sha256=record_for(target)["sha256"])
        good = arguments(base)
        check_repair(good, protocol, collection)
        def fails(name, document, wrong_hash=False):
            args = arguments(document)
            if wrong_hash:
                args.repair_manifest_sha256 = "0" * 64
            try:
                check_repair(args, protocol, collection)
            except ValueError as error:
                rejected.append({"case": name, "actual_error": str(error)})
            else:
                raise AssertionError("repair provenance tamper admitted: " + name)
        fails("literal manifest hash", base, True)
        before_data = copy.deepcopy(base)
        before_data["registered_utc"] = "2026-01-01T00:00:00+00:00"
        fails("false pre-data repair timestamp", before_data)
        relabeled = copy.deepcopy(base)
        relabeled["original_protocol"]["sha256"] = "0" * 64
        fails("original protocol relabel", relabeled)
        duplicate = copy.deepcopy(base)
        duplicate["repaired_reader_closure"].append(duplicate["repaired_reader_closure"][0])
        fails("duplicate new closure", duplicate)
        criteria = copy.deepcopy(base)
        criteria["numerical_formulas_and_scientific_gates_unchanged"] = False
        fails("criteria changes", criteria)
        expanded = copy.deepcopy(base)
        expanded["repair_scope"] = "also change solver tolerances"
        fails("scope expansion", expanded)
        rerun = copy.deepcopy(base)
        rerun["authoritative_worlds_rerun"] = 1
        fails("new worlds claimed", rerun)
    print(json.dumps({"status": "PASS", "scope": "pure post-collection provenance checks, original metadata/hashes only",
                      "native_worlds": 0, "complete_raw_measurement_rereads": 0,
                      "actual_original_failed_audit_retained": True, "checked_original_closure": 4, "checked_repaired_closure": 4,
                      "rejected": rejected}, indent=2))


if __name__ == "__main__":
    main()
