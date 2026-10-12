"""Read-only guards for the frozen required release-133--138 comparison."""
from datetime import datetime, timezone
import ast
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
REPO = Path('/workspace/RobotNativeEngine')
ROOT = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration')
BASE = Path('/workspace/.rne-kick-demo/g1-motor-torque-diagnostics/baseline/examples')
EXAMPLES = [('go2_contact_terrain', '133_go2_contact_terrain'),
            ('go2_terrain_trot', '134_go2_terrain_trot'),
            ('go2_feedback_trot', '135_go2_feedback_trot'),
            ('go2_parallel_rl', '136_go2_parallel_rl'),
            ('go2_native_backend', '137_go2_native_backend'),
            ('heavy_grasp', '138_heavy_grasp')]
EXPRESSIONS = [(r',\s*[0-9.]+ us per step', ''),
               (r'\d+ env steps/s \(\d+ sim steps/s\)', '<throughput>'),
               (r'parallel speedup [0-9.]+x', 'parallel speedup <timing>')]


def load(path):
    return json.loads(Path(path).read_bytes())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def ref(path):
    path = Path(path)
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': sha(path)}


def physical(raw):
    text = raw.decode('utf-8')
    for pattern, replacement in EXPRESSIONS:
        text = re.sub(pattern, replacement, text)
    return text.encode('utf-8')


def expected_argv(package, example):
    return ['cargo', 'run', '--locked', '--release', '-p', package, '--example', example]


def guard_tools(expected_freeze_sha):
    freeze_path = HERE / 'tool-freeze.json'
    assert sha(freeze_path) == expected_freeze_sha, 'tool freeze changed'
    freeze = load(freeze_path)
    for entry in freeze['frozen_files']:
        assert sha(entry['path']) == entry['sha256'], entry['path']
    criteria = load(HERE / 'criteria.json')
    assert criteria['timing_only_exclusions'] == [
        {'regex': a, 'replacement': b} for a, b in EXPRESSIONS]
    assert criteria['examples'] == [list(x) for x in EXAMPLES]
    old = Path(criteria['historical_runner']['path']).read_bytes()
    new = (HERE / 'run_release_regressions.py').read_bytes()
    assert old.replace(criteria['historical_out_line'].encode(),
                       criteria['current_out_line'].encode()) == new
    tree = ast.parse(new)
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'physical')
    substitutions = [(ast.literal_eval(n.args[0]), ast.literal_eval(n.args[1]))
                     for n in ast.walk(function) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == 'sub']
    assert substitutions == EXPRESSIONS
    for item in criteria['baseline_files']:
        assert sha(item['path']) == item['sha256'], item['path']
    return criteria


def source_files():
    # Explicitly selected source scope, not a compiler/dependency closure claim.
    paths = {REPO / name for name in ['AGENTS.md', 'Cargo.toml', 'Cargo.lock',
                                    'rust-toolchain.toml', '.cargo/config.toml']
             if (REPO / name).is_file()}
    for name in ['crates', 'examples', 'third_party', 'adapters', 'xtask', 'tests', 'tools']:
        root = REPO / name
        if not root.exists():
            continue
        for path in root.rglob('*'):
            relative = path.relative_to(REPO)
            if any(x in {'target', '.git', '.venv', '__pycache__'} for x in relative.parts):
                continue
            if path.is_file() and path.suffix in {'.rs', '.toml', '.lock'}:
                paths.add(path)
    return {str(path.relative_to(REPO)): sha(path) for path in sorted(paths)}


def save_new(path, value):
    path = Path(path)
    with path.open('x') as output:
        output.write(json.dumps(value, indent=2, allow_nan=False) + '\n')


def now():
    return datetime.now(timezone.utc).isoformat()
