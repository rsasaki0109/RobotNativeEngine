"""Controlled 50 ms contact admission experiment; not a quality/cost benchmark."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import mujoco
import box
import contact_constraint_audit as constraints

PROTOCOL = 'contact_admission_v1'
MARGINS = [0.0, 0.0001, 0.02]
MJ_SETTINGS = [dict(solref=[.02, 1], margin_m=m, gap_m=g)
               for m, g in [(0.0, 0.0), (0.0, .0001), (.0001, .0001), (.0001, 0.0)]]
SOURCES = ['contact_admission.py', 'contact_constraint_audit.py',
           'force_friction_matched.py', 'force_friction.py', 'box.py']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_native(binary, margin):
    result = json.loads(subprocess.run(
        [str(binary.resolve()), '2000000', '3', '1.2', '0', '200', 'audit',
         '.5', 'wrench', 'diagnostics', str(margin)],
        check=True, capture_output=True, text=True, timeout=120).stdout)
    # Wall-clock duration is not part of the deterministic evidence.
    del result['elapsed_s']
    return result


def summarize_native(audit, margin):
    x = np.asarray(audit['trace'])
    rows = audit['contact_diagnostics']
    if (x.shape != (26, 15) or not np.isfinite(x).all() or len(rows) != 25
            or audit['debug_assertions'] or audit['load_mode'] != 'external_body_wrench'
            or audit['settings']['contact_margin_m'] != margin
            or audit['settings']['regularization'] != 0
            or abs(audit['settings']['equivalent_force_n'] - 17.658) > 1e-12):
        raise ValueError('Native fixture differs')
    if not np.allclose(x[:, 0], np.arange(26)*.002, rtol=0, atol=1e-12):
        raise ValueError('clock differs')
    counts = []; positive_loaded = []; gaps = []
    for i, row in enumerate(rows):
        if row['before'][1:14] != audit['trace'][i][1:14]:
            raise ValueError('pre-step state differs')
        c = row['pipeline']
        if not c['generated'] >= c['deduplicated'] >= c['retained'] == len(row['points']):
            raise ValueError('pipeline differs')
        counts.append(c['retained'])
        for point in row['points']:
            gap = point['gap_m']
            if not np.isfinite(gap) or gap > margin:
                raise ValueError('point outside admission distance')
            gaps.append(gap)
            if gap > 0 and point['normal_impulse_n_s'] > 1e-10:
                positive_loaded.append(gap)
    spin = np.linalg.norm(x[:, 11:14], axis=1)
    crossed = np.flatnonzero(spin > .01)
    return dict(retained_counts=sorted(set(counts)),
                first_spin_gate_time_s=float(x[crossed[0], 0]) if len(crossed) else None,
                max_spin_rad_s=float(spin.max()),
                max_penetration_m=max(0.0, float(-box.corners(x)[:, :, 1].min())),
                positive_gap_loaded_samples=len(positive_loaded),
                max_loaded_positive_gap_m=max(positive_loaded, default=0.0),
                max_candidate_gap_m=max(gaps, default=None))


def admission_probe(setting):
    """Measure compiled contact semantics at 50 micrometres separation."""
    xml = constraints.fixture.model_xml(constraints.CASE, setting['solref']).replace(
        'margin="0" gap="0"',
        f'margin="{setting["margin_m"]}" gap="{setting["gap_m"]}"')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    data.qpos[1] += 5e-5  # This fixture uses world Y as vertical.
    mujoco.mj_forward(model, data)
    return [dict(distance_m=float(c.dist), include_margin_m=float(c.includemargin),
                 constraint_active=bool(c.efc_address >= 0)) for c in data.contact]


def verify(directory, binary):
    report = json.loads((directory/'report.json').read_text())
    if (report['protocol'] != PROTOCOL or report['mujoco_version'] != '3.9.0'
            or report['case'] != constraints.CASE
            or report['binary_sha256'] != digest(binary)
            or report['source_sha256'] != {n: digest(Path(__file__).with_name(n)) for n in SOURCES}
            or [r['margin_m'] for r in report['native']] != MARGINS
            or [r['setting'] for r in report['mujoco']] != MJ_SETTINGS):
        raise ValueError('provenance/settings differ')
    for row in report['native']:
        if (summarize_native(row['audit'], row['margin_m']) != row['summary']
                or run_native(binary, row['margin_m']) != row['audit']):
            raise ValueError('Native metrics/replay differ')
    for i, row in enumerate(report['mujoco']):
        if row['admission_probe'] != admission_probe(row['setting']):
            raise ValueError('compiled admission semantics differ')
        path = directory/f'admission-{i}.npz'
        if row['file'] != path.name or digest(path) != row['sha256']:
            raise ValueError('MuJoCo artifact differs')
        with np.load(path, allow_pickle=False) as arrays:
            if constraints.mj_summary(arrays['steps'], arrays['points']) != row['summary']:
                raise ValueError('MuJoCo metrics differ')
            replay = constraints.run_mujoco(**dict(ref=row['setting']['solref'],
                        margin_m=row['setting']['margin_m'], gap_m=row['setting']['gap_m']))
            if not all(np.array_equal(arrays[key], value) for key, value in zip(['steps', 'points'], replay)):
                raise ValueError('MuJoCo replay differs')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        verify(args.directory, args.binary)
        print('Verified admission experiment including fresh replays'); return
    if args.directory.exists() or mujoco.__version__ != '3.9.0':
        parser.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    report = dict(protocol=PROTOCOL, mujoco_version=mujoco.__version__, case=constraints.CASE,
                  binary_sha256=digest(args.binary),
                  source_sha256={n: digest(Path(__file__).with_name(n)) for n in SOURCES},
                  native=[], mujoco=[])
    for margin in MARGINS:
        audit = run_native(args.binary, margin)
        summary = summarize_native(audit, margin)
        report['native'].append(dict(margin_m=margin, audit=audit, summary=summary))
        print(json.dumps(dict(native_margin_m=margin, summary=summary)), flush=True)
    for i, setting in enumerate(MJ_SETTINGS):
        steps, points = constraints.run_mujoco(setting['solref'], setting['margin_m'], setting['gap_m'])
        path = args.directory/f'admission-{i}.npz'
        np.savez_compressed(path, steps=steps, points=points)
        summary = constraints.mj_summary(steps, points)
        report['mujoco'].append(dict(setting=setting, admission_probe=admission_probe(setting), file=path.name, sha256=digest(path), summary=summary))
        print(json.dumps(dict(mujoco=setting, summary=summary)), flush=True)
    (args.directory/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    verify(args.directory, args.binary)


if __name__ == '__main__':
    main()
