"""Execute required release examples and compare all non-timing stdout."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

REPO = Path('/workspace/RobotNativeEngine')
ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
BASE = Path('/workspace/.rne-kick-demo/g1-motor-torque-diagnostics/baseline/examples')
OUT = ROOT / 'next-experiment' / 'vendor-lint-qualification' / 'release-regressions'
EXAMPLES = [('go2_contact_terrain', '133_go2_contact_terrain'),
            ('go2_terrain_trot', '134_go2_terrain_trot'),
            ('go2_feedback_trot', '135_go2_feedback_trot'),
            ('go2_parallel_rl', '136_go2_parallel_rl'),
            ('go2_native_backend', '137_go2_native_backend'),
            ('heavy_grasp', '138_heavy_grasp')]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def physical(text):
    text = re.sub(r',\s*[0-9.]+ us per step', '', text)
    text = re.sub(r'\d+ env steps/s \(\d+ sim steps/s\)', '<throughput>', text)
    return re.sub(r'parallel speedup [0-9.]+x', 'parallel speedup <timing>', text)


OUT.mkdir(exist_ok=False)
rows = []
for package, example in EXAMPLES:
    command = ['cargo', 'run', '--locked', '--release', '-p', package, '--example', example]
    start = time.monotonic()
    with (OUT / (example + '.stdout')).open('wb') as stdout, (OUT / (example + '.stderr')).open('wb') as stderr:
        process = subprocess.Popen(command, cwd=REPO, stdout=stdout, stderr=stderr)
        (OUT / (example + '-process.json')).write_text(json.dumps({'pid': process.pid, 'command': command}) + '\n')
        code = process.wait()
    new = physical((OUT / (example + '.stdout')).read_text())
    old = physical((BASE / (example + '.stdout')).read_text())
    (OUT / (example + '.physical.txt')).write_text(new)
    row = {'example': example, 'command': command, 'exit_code': code,
           'elapsed_s': time.monotonic() - start,
           'ended_utc': datetime.now(timezone.utc).isoformat(),
           'has_ok': ': ok' in new, 'physical_stdout_matches_baseline': new == old,
           'baseline_stdout_sha256': digest(BASE / (example + '.stdout')),
           'physical_stdout_sha256': digest(OUT / (example + '.physical.txt'))}
    rows.append(row)
    (OUT / 'results.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(row), flush=True)
    if code or not row['has_ok'] or not row['physical_stdout_matches_baseline']:
        raise SystemExit(1)
raise SystemExit(0)
