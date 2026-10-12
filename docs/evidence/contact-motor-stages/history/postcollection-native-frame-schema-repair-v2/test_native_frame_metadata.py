#!/usr/bin/env python3
"""Pure strict palm native-axis/handle/link/model gates, no trace reread."""

import copy
import json

from stage_analysis import Analysis, analyze_telemetry
from stage_schema import parse_ecs
from test_stage_reader import fixture


def main():
    protocol, frame = fixture()
    entities = parse_ecs(frame["ecs_physics_ieee_words"], protocol["resource_ceilings"])
    good = Analysis("baseline", lambda value: None)
    analyze_telemetry(frame["stage_telemetry"], frame["native_checkpoint"], entities, 1, protocol, good)
    assert good.summary()["complete_native_replay"]
    rejected = []
    def fails(name, change):
        telemetry = copy.deepcopy(frame["stage_telemetry"])
        for event in telemetry["events"]:
            for row in event["rows"]:
                change(row)
        try:
            analyze_telemetry(telemetry, frame["native_checkpoint"], entities, 1, protocol, Analysis("baseline", lambda value: None))
        except ValueError as error:
            rejected.append({"name": name, "actual_error": str(error)})
        else:
            raise AssertionError("fixed palm native metadata gate relaxed: " + name)
    fails("world LinY label rejected as native frame", lambda row: row.__setitem__("axis", "LinY"))
    fails("wrong native angular axis", lambda row: row.__setitem__("axis", "AngX"))
    fails("wrong motor gain model", lambda row: row.__setitem__("gain_model", "AccelerationBased"))
    fails("wrong bound child link", lambda row: row.__setitem__("link_id", 2))
    fails("finger handle cannot replace palm", lambda row: row.__setitem__("joint_handle", {"index": 4, "generation": 0}))
    fails("handle generation remains exact", lambda row: row.__setitem__("joint_handle", {"index": 3, "generation": 1}))
    print(json.dumps({"status": "PASS", "native_worlds": 0, "complete_raw_measurement_rereads": 0,
                      "scope": "synthetic exact native LinX palm gate plus forbidden identity/model/frame transitions",
                      "native_LinX_accepted_without_other_gate_changes": True, "rejected": rejected}, indent=2))


if __name__ == "__main__":
    main()
