"""Summary checks without running physics."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from test_lekiwi_evaluation import markdown_summary, save_summary


class SummaryTests(unittest.TestCase):
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
