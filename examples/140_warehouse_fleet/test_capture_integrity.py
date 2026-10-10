#!/usr/bin/env python3
"""Regression checks for stale inputs and tampered pixels under optimized Python."""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from PIL import Image

from encode import ENGINE_INPUTS, ROOT, sha256


def current_provenance() -> dict:
    workspace = ROOT.parents[1]
    return {
        'format_version': 1,
        'simulation_source_sha256': sha256(ROOT / 'main.rs'),
        'package_manifest_sha256': sha256(ROOT / 'Cargo.toml'),
        'workspace_manifest_sha256': sha256(workspace / 'Cargo.toml'),
        'workspace_lock_sha256': sha256(workspace / 'Cargo.lock'),
        'engine_sources_sha256': {
            name: sha256(workspace / name) for name in ENGINE_INPUTS
        },
    }


class CaptureIntegrityTests(unittest.TestCase):
    """Exercise real subprocess entry points with deliberately unqualified input."""

    def run_cli(self, script: str, trace: dict, expected_error: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'trace.json'
            path.write_text(json.dumps(trace))
            environment = os.environ.copy()
            environment['PYTHONDONTWRITEBYTECODE'] = '1'
            for optimization in ([], ['-O']):
                result = subprocess.run(
                    [sys.executable, *optimization, str(ROOT / script),
                     '--trace', str(path), '--output', str(root / 'rejected')],
                    env=environment, capture_output=True, text=True, check=False,
                )
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn(expected_error, result.stderr)
                self.assertFalse((root / 'rejected').exists())

    def test_failed_simulation_stays_rejected_with_assertions_disabled(self) -> None:
        trace = {
            'source_provenance': current_provenance(),
            'schema_version': 1,
            'robot_count': 40,
            'validation': {'passed': False},
        }
        self.run_cli('encode.py', trace, 'Simulation acceptance gates did not pass')
        self.run_cli('check_recording.py', trace, "validation['passed']")

    def test_stale_example_and_engine_inputs_stay_rejected(self) -> None:
        trace = {'source_provenance': current_provenance()}
        stale = copy.deepcopy(trace)
        stale['source_provenance']['simulation_source_sha256'] = '0' * 64
        self.run_cli('encode.py', stale, 'differs from current main.rs')
        stale = copy.deepcopy(trace)
        stale['source_provenance']['engine_sources_sha256'][ENGINE_INPUTS[0]] = '0' * 64
        self.run_cli('check_recording.py', stale, 'engine input hashes differ')

    def test_pixel_tampering_stays_rejected_with_assertions_disabled(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trace = root / 'trace.json'
            provenance = current_provenance()
            trace.write_text(json.dumps({'source_provenance': provenance}))
            raw = root / 'raw'
            raw.mkdir()
            pixels = Image.new('RGBA', (1024, 656), (0, 0, 0, 255))
            digest = hashlib.sha256(pixels.tobytes()).hexdigest()
            pixels.save(raw / 'frame-0000.png')
            metadata = {
                'trace_validated': True,
                'diagnostic_only': False,
                'fixture_only': False,
                'trace_sha256': sha256(trace),
                'renderer_source_sha256': sha256(ROOT / 'render.rs'),
                'renderer_manifest_sha256': sha256(ROOT / 'Cargo.toml'),
                'simulation_source_provenance_verified': True,
                'simulation_source_sha256': provenance['simulation_source_sha256'],
                'source_provenance': provenance,
                'width': 1024,
                'height': 656,
                'frames': [{'index': 0, 'file': 'frame-0000.png', 'rgba_sha256': digest}],
            }
            (raw / 'render-metadata.json').write_text(json.dumps(metadata))
            code = (
                'import sys; from pathlib import Path; '
                'from encode import verify_render; '
                'verify_render(Path(sys.argv[1]),Path(sys.argv[2]),[0])'
            )
            for optimization in ([], ['-O']):
                command = [sys.executable, *optimization, '-c', code, str(trace), str(raw)]
                valid = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                       check=False)
                self.assertEqual(valid.returncode, 0, valid.stderr)
                pixels.putpixel((0, 0), (255, 0, 0, 255))
                pixels.save(raw / 'frame-0000.png')
                tampered = subprocess.run(command, cwd=ROOT, capture_output=True,
                                          text=True, check=False)
                self.assertNotEqual(tampered.returncode, 0)
                self.assertIn('pixels differ from capture', tampered.stderr)
                pixels.putpixel((0, 0), (0, 0, 0, 255))
                pixels.save(raw / 'frame-0000.png')


if __name__ == '__main__':
    unittest.main()
