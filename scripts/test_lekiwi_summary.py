"""Summary checks without running physics."""
import json
import copy
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from test_lekiwi_evaluation import main, markdown_summary, save_summary


class SummaryTests(unittest.TestCase):
    def test_grasp_changes_match_names_and_show_decreases_and_improvements(self):
        from test_lekiwi_evaluation import BASELINE, metric_delta_table
        baseline = json.loads(BASELINE.read_text())
        current = copy.deepcopy(baseline)
        current["cases"].reverse()
        nominal = next(case for case in current["cases"] if case["name"] == "nominal")
        nominal["placement"]["grasp"]["lift_m"] += 0.002
        nominal["placement"]["grasp"]["minimum_hold_lift_m"] -= 0.002
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "baseline.json").write_text(json.dumps(baseline))
            (output / "evaluation" / "report.json").write_text(json.dumps(current))
            text = "\n".join(metric_delta_table(output, grasp=True))
            self.assertIn("nominal | 88.581 | 90.581 | +2.000 | 88.581 | 86.581 | -2.000", text)
            self.assertIn("nominal / minimum hold lift**, **-1.000 mm** (exceeds allowance)", text)
            self.assertNotIn("| missed-block |", text)

    def test_grasp_margin_uses_lower_unrounded_boundary_for_both_heights(self):
        from test_lekiwi_evaluation import BASELINE, metric_delta_table
        baseline = json.loads(BASELINE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "baseline.json").write_text(json.dumps(baseline))
            for key, label in [("lift_m", "grasp lift"), ("minimum_hold_lift_m", "minimum hold lift")]:
                for extra, status in [(0, "within allowance"), (1e-9, "exceeds allowance")]:
                    with self.subTest(key=key, extra=extra):
                        current = copy.deepcopy(baseline)
                        grasp = current["cases"][0]["placement"]["grasp"]
                        grasp[key] = grasp[key] - 0.001 - extra
                        (output / "evaluation" / "report.json").write_text(json.dumps(current))
                        text = "\n".join(metric_delta_table(output, grasp=True))
                        self.assertIn(f"nominal / {label}", text)
                        self.assertIn(status, text)

    def test_invalid_grasp_evidence_does_not_hide_placement_table_or_claim_margin(self):
        from test_lekiwi_evaluation import BASELINE
        baseline = json.loads(BASELINE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "baseline.json").write_text(json.dumps(baseline))
            for evidence in [None, {}, {"lift_m": -1, "minimum_hold_lift_m": 0},
                             {"lift_m": float("nan"), "minimum_hold_lift_m": 0}]:
                with self.subTest(evidence=evidence):
                    current = copy.deepcopy(baseline)
                    current["cases"][0]["placement"]["grasp"] = evidence
                    (output / "evaluation" / "report.json").write_text(json.dumps(current))
                    text = markdown_summary(output, {"passed": False})
                    self.assertIn("Grasp changes unavailable", text)
                    self.assertNotIn("Least remaining grasp margin", text)
                    self.assertIn("### Metric changes (mm)", text)
                    self.assertIn("CI gate: **FAIL**", text)

    def test_offline_summary_preserves_evidence_and_does_not_run_evaluation(self):
        from test_lekiwi_evaluation import BASELINE, fingerprints
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "evaluation" / "report.json").write_bytes(BASELINE.read_bytes())
            (output / "baseline.json").write_bytes(BASELINE.read_bytes())
            step = output / "step.md"
            step.write_text("existing summary")
            for passed in (True, False):
                with self.subTest(passed=passed):
                    (output / "ci-summary.json").write_text(json.dumps({"passed": passed, "error": "recorded failure"}))
                    before = fingerprints(output), fingerprints(output / "evaluation")
                    stream = io.StringIO()
                    with patch("test_lekiwi_evaluation.check") as check, \
                            patch("test_lekiwi_evaluation.subprocess.run") as run, \
                            patch.dict(os.environ, {"GITHUB_STEP_SUMMARY": str(step)}), \
                            contextlib.redirect_stdout(stream):
                        main(["--summarize", str(output)])
                    check.assert_not_called()
                    run.assert_not_called()
                    self.assertEqual(before, (fingerprints(output), fingerprints(output / "evaluation")))
                    self.assertIn(f"CI gate: **{'PASS' if passed else 'FAIL'}**", stream.getvalue())
                    self.assertIn("missed-block | failed | close | 100", stream.getvalue())

    def test_offline_summary_requires_an_explicit_boolean_verdict(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            for content in (None, "{broken", "[]", '{}', '{"passed": "false"}', '{"passed": 1}'):
                with self.subTest(content=content):
                    if content is not None:
                        (output / "ci-summary.json").write_text(content)
                    stdout, stderr = io.StringIO(), io.StringIO()
                    with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                        with self.assertRaises(SystemExit) as error:
                            main(["--summarize", str(output)])
                    self.assertEqual(error.exception.code, 2)
                    self.assertEqual(stdout.getvalue(), "")
                    self.assertIn("cannot read retained CI verdict", stderr.getvalue())

    def test_offline_setup_failure_can_render_without_physical_report(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "ci-summary.json").write_text(json.dumps({"passed": False, "error": "setup failed"}))
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                main(["--summarize", str(output)])
            self.assertIn("CI gate: **FAIL**", stream.getvalue())
            self.assertIn("setup failed", stream.getvalue())
            self.assertIn("unavailable or incomplete", stream.getvalue())

    def test_diagnostics_show_recorded_grasp_failure_and_preflight_rejections(self):
        from test_lekiwi_evaluation import BASELINE
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "evaluation" / "report.json").write_bytes(BASELINE.read_bytes())
            text = markdown_summary(output, {"passed": True})
            details = text.split("### Failure and rejection diagnostics")[1]
            self.assertIn("missed-block | failed | close | 100 | Close: block must contact both grasp pads", details)
            self.assertIn("unequal-height | rejected | not executed | 0 | destination not supported", details)
            self.assertNotIn("| nominal |", details)
            self.assertIn("CI gate: **PASS**", text)

    def test_completed_placement_with_replay_failure_has_separate_diagnostic(self):
        from test_lekiwi_evaluation import BASELINE
        report = json.loads(BASELINE.read_text())
        case = report["cases"][0]
        case["replay_verified"] = False
        case["replay_error"] = "hash | mismatch <step>\nnext"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "evaluation" / "report.json").write_text(json.dumps(report))
            text = markdown_summary(output, {"passed": False, "error": "replay failed"})
            self.assertIn("nominal | completed | yes | FAIL", text)
            self.assertIn("nominal | completed | not reported | 330 | not reported | hash &#124; mismatch &lt;step&gt; next", text)

    def test_diagnostics_preserve_unexpected_outcomes_without_inventing_evidence(self):
        from test_lekiwi_evaluation import BASELINE
        report = json.loads(BASELINE.read_text())
        report["cases"] = [{"name": "unexpected", "outcome": "error", "expectation_met": False,
                            "replay_verified": None, "reason": "setup <failed> | retry\nlog"},
                           {"name": "unexpected-success", "outcome": "completed", "expectation_met": False,
                            "replay_verified": True, "expected": "failed"}]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "evaluation" / "report.json").write_text(json.dumps(report))
            text = markdown_summary(output, {"passed": False})
            self.assertIn("unexpected | not reported | not reported | not reported | setup &lt;failed&gt; &#124; retry log", text)
            self.assertIn("unexpected-success | failed | not reported", text)

    def test_expected_failure_is_not_physical_success(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            report = {"completed_cases": 1, "executed_cases": 2, "rejected_cases": 1,
                      "error_cases": 0, "cases": [
                          {"name": "nominal", "outcome": "completed", "expectation_met": True,
                           "replay_verified": True, "placement": {"placement_error_x_z_m": [0.003, 0.004]}},
                          {"name": "missed|<block>\n", "outcome": "failed", "expectation_met": True,
                           "replay_verified": True},
                          {"name": "height", "outcome": "rejected", "expectation_met": True,
                           "replay_verified": None}]}
            (output / "evaluation" / "report.json").write_text(json.dumps(report))
            text = markdown_summary(output, {"passed": True})
            self.assertIn("Physical completion: 1/2", text)
            self.assertIn("5.000", text)
            self.assertIn("missed&#124;&lt;block&gt;  | failed | yes | pass", text)
            self.assertIn("height | rejected | yes | not executed", text)

    def test_failure_without_report_is_retained_and_summary_appends(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            step = output / "step.md"
            step.write_text("Earlier summary\n")
            with patch.dict(os.environ, {"GITHUB_STEP_SUMMARY": str(step)}):
                save_summary(output, {"passed": False, "error": "setup <failed>"})
            text = step.read_text()
            self.assertTrue(text.startswith("Earlier summary\n"))
            self.assertIn("**FAIL**", text)
            self.assertIn("setup &lt;failed&gt;", text)
            self.assertIn("unavailable or incomplete", text)
            self.assertFalse(json.loads((output / "ci-summary.json").read_text())["passed"])
            self.assertTrue((output / "summary.md").exists())

    def test_malformed_report_does_not_hide_gate_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            (output / "evaluation" / "report.json").write_text("{broken")
            text = markdown_summary(output, {"passed": False, "error": "bad report"})
            self.assertIn("**FAIL**", text)
            self.assertIn("unavailable or incomplete", text)

    def test_baseline_regression_remains_visible_with_physical_report_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "baseline.stdout.log").write_text(json.dumps({
                "passed": False, "regressions": ["nominal: placement error increased <limit>"]}))
            text = markdown_summary(output, {"passed": False, "error": "baseline comparison failed"})
            self.assertIn("Reviewed baseline: **FAIL**", text)
            self.assertIn("placement error increased &lt;limit&gt;", text)
            self.assertIn("unavailable or incomplete", text)

    def test_setup_failure_retains_the_reference_without_modifying_it(self):
        from test_lekiwi_evaluation import BASELINE, check
        before = BASELINE.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with patch("test_lekiwi_evaluation.run", return_value=1):
                with self.assertRaisesRegex(RuntimeError, "evaluation failed"):
                    check(output)
            self.assertEqual((output / "baseline.json").read_bytes(), before)
        self.assertEqual(BASELINE.read_bytes(), before)

    def test_metric_changes_match_cases_by_name_and_show_signed_millimeters(self):
        from test_lekiwi_evaluation import metric_delta_table
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            def case(name, error, drift, outcome="completed"):
                return {"name": name, "outcome": outcome, "placement": {
                    "placement_error_x_z_m": error, "maximum_settled_drift_m": drift, "planned_destination_m": [0, 0, 0]}}
            baseline = {"schema_version": 1, "catalog": "v1", "cases": [case("better", [0.003, 0.004], 0.002),
                                  case("worse", [0.0, 0.001], 0.0),
                                  case("failed", [0.0, 0.0], 0.0, "failed")]}
            current = {"schema_version": 1, "catalog": "v1", "cases": [case("worse", [0.0, 0.003], 0.001),
                                 case("better", [0.003, 0.0], 0.001),
                                 case("failed", [0.0, 0.0], 0.0, "failed")]}
            (output / "baseline.json").write_text(json.dumps(baseline))
            (output / "evaluation" / "report.json").write_text(json.dumps(current))
            text = "\n".join(metric_delta_table(output))
            self.assertIn("worse | 1.000 | 3.000 | +2.000 | 0.000 | 1.000 | +1.000", text)
            self.assertIn("better | 5.000 | 3.000 | -2.000 | 2.000 | 1.000 | -1.000", text)
            self.assertNotIn("| failed |", text)

    def test_metric_changes_with_no_baseline_do_not_claim_zero_change(self):
        from test_lekiwi_evaluation import metric_delta_table
        with tempfile.TemporaryDirectory() as directory:
            text = "\n".join(metric_delta_table(Path(directory)))
            self.assertIn("unavailable", text)
            self.assertNotIn("+0.000", text)

    def test_metric_changes_reject_changed_destination_or_catalog(self):
        from test_lekiwi_evaluation import metric_delta_table
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            report = {"schema_version": 1, "catalog": "v1", "cases": [
                {"name": "nominal", "outcome": "completed", "placement": {
                    "planned_destination_m": [0, 0, 0], "placement_error_x_z_m": [0.001, 0],
                    "maximum_settled_drift_m": 0.0}}]}
            (output / "baseline.json").write_text(json.dumps(report))
            report["cases"][0]["placement"]["planned_destination_m"][0] = 1
            (output / "evaluation" / "report.json").write_text(json.dumps(report))
            self.assertIn("changed destination", "\n".join(metric_delta_table(output)))
            report["catalog"] = "v2"
            (output / "evaluation" / "report.json").write_text(json.dumps(report))
            self.assertIn("catalog changed", "\n".join(metric_delta_table(output)))

    def test_smallest_margin_uses_unrounded_limit_and_includes_drift(self):
        from test_lekiwi_evaluation import metric_delta_table
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            report = {"schema_version": 1, "catalog": "v1", "cases": [
                {"name": "nominal", "outcome": "completed", "placement": {
                    "planned_destination_m": [0, 0, 0], "placement_error_x_z_m": [0, 0],
                    "maximum_settled_drift_m": 0.0}}]}
            (output / "baseline.json").write_text(json.dumps(report))
            for drift, label in [(0.001, 'within allowance'), (0.0010001, 'exceeds allowance')]:
                report['cases'][0]['placement']['maximum_settled_drift_m'] = drift
                (output / "evaluation" / "report.json").write_text(json.dumps(report))
                text = "\n".join(metric_delta_table(output))
                self.assertIn('nominal / settled drift', text)
                self.assertIn(label, text)
                self.assertIn('placement metrics only', text)

    def test_smallest_margin_selects_worst_case_independent_of_order(self):
        from test_lekiwi_evaluation import metric_delta_table
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "evaluation").mkdir()
            def case(name, error):
                return {"name": name, "outcome": "completed", "placement": {
                    "planned_destination_m": [0, 0, 0], "placement_error_x_z_m": [error, 0],
                    "maximum_settled_drift_m": 0.0}}
            baseline = {"schema_version": 1, "catalog": "v1", "cases": [case('first', 0), case('second', 0)]}
            (output / 'baseline.json').write_text(json.dumps(baseline))
            current = {**baseline, 'cases': [case('second', 0.0008), case('first', 0.0002)]}
            for cases in [current['cases'], list(reversed(current['cases']))]:
                (output / 'evaluation' / 'report.json').write_text(json.dumps({**current, 'cases': cases}))
                text = '\n'.join(metric_delta_table(output))
                self.assertIn('second / placement error**, **+0.200 mm**', text)
