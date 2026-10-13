"""Prepare a new isolated physical study without launching native worlds."""
from pathlib import Path
import hashlib
import json
import shutil
import struct
import subprocess

ROOT = Path('/workspace/RobotNativeEngine')
OWN = Path(__file__).resolve().parent
OLD = OWN.parent / 'next-motor-visit-placement-preparation'

def describe(path):
    data = path.read_bytes()
    return {'path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}

def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')

def word(value):
    return f'{struct.unpack("<I", struct.pack("<f", value))[0]:08x}'

def main():
    paths = subprocess.check_output(['git', 'ls-files', '-z', 'third_party/rapier3d'], cwd=ROOT).split(b'\0')
    paths = [Path(p.decode()) for p in paths if p]
    assert len(paths) == 123, len(paths)
    bindings = []
    for name in ('vendor_base', 'vendor_candidate'):
        dest = OWN / name
        dest.mkdir()
        for rel in paths:
            target = dest / rel.relative_to('third_party/rapier3d')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / rel, target)
            assert target.read_bytes() == (ROOT / rel).read_bytes()
            bindings.append({'variant': name, 'root': describe(ROOT / rel), 'copy': describe(target)})
    rel = Path('src/dynamics/joint/multibody_joint/unit_multibody_joint.rs')
    target = OWN / 'vendor_candidate' / rel
    data = target.read_bytes()
    start = data.index(b'pub(crate) fn unit_joint_motor_constraint_with_construction_inputs(')
    prefix, suffix = data[:start], data[start:]
    before = b'        inv_lhs: crate::utils::inv(lhs),\r\n'
    after = (b'        inv_lhs: match motor.model {\r\n'
             b'            crate::dynamics::MotorModel::ForceBased => {\r\n'
             b'                crate::utils::inv(lhs + motor_params.cfm_gain)\r\n'
             b'            }\r\n'
             b'            crate::dynamics::MotorModel::AccelerationBased => crate::utils::inv(lhs),\r\n'
             b'        },\r\n')
    assert suffix.count(before) == 1
    target.write_bytes(prefix + suffix.replace(before, after, 1))
    changed = []
    for path in paths:
        short = path.relative_to('third_party/rapier3d')
        if (OWN / 'vendor_base' / short).read_bytes() != (OWN / 'vendor_candidate' / short).read_bytes():
            changed.append(str(short))
    assert changed == [str(rel)]
    save(OWN / 'source-copy-binding.json', {'kind': 'NEW_PHYSICAL_SOURCE_ONLY_PREPARATION', 'schema_version': 1,
         'root_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
         'native_processes_launched': 0, 'copies_before_candidate_patch': bindings,
         'candidate_delta': {'baseline': describe(OWN / 'vendor_base' / rel), 'candidate': describe(target),
             'scope': 'ForceBased internal motor normalization only; contact freedom admitted by isolated plant, not ndofs.',
             'production_adoption': False, 'other_changed_paths': []}})
    collector = OWN / 'collector'
    collector.mkdir(exist_ok=True)
    manifest = (OLD / 'collector' / 'Cargo.toml').read_text()
    manifest = manifest.replace('motor_visit_placement_collector', 'motor_physical_collector')
    manifest = manifest.replace('name = "motor_visit_placement"', 'name = "motor_physical"')
    manifest = manifest.replace('rapier3d = { path = "../rapier3d",', 'rapier3d = { version = "=0.22.0",')
    with (collector / 'Cargo.toml').open('x') as stream:
        stream.write(manifest)
    oldlock = (OLD / 'collector' / 'Cargo.lock').read_bytes()
    assert oldlock.count(b'name = "motor_visit_placement_collector"') == 1
    with (collector / 'Cargo.lock').open('xb') as stream:
        stream.write(oldlock.replace(b'name = "motor_visit_placement_collector"', b'name = "motor_physical_collector"'))
    for variant, vendor in (('baseline', 'vendor_base'), ('candidate', 'vendor_candidate')):
        with (OWN / f'{variant}-cargo-config.toml').open('x') as stream:
            stream.write(f'[patch.crates-io]\nrapier3d = {{ path = "{OWN / vendor}" }}\n')
    cases = []
    for mass_tag, mass in (('m05', .5), ('m2', 2.0)):
        for time_tag, dt in (('h05', .0005), ('h2', .002)):
            for sign_tag, target_pos in (('neg', -.001), ('pos', .001)):
                for cap_tag, cap in (('cap2', 2.0), ('cap001', .01)):
                    cases.append({'case_id': f'{mass_tag}_{time_tag}_{sign_tag}_{cap_tag}',
                        'mass_kg': mass, 'outer_dt_s': dt, 'target_pos_rad': target_pos, 'max_torque_nm': cap})
            cases.append({'case_id': f'{mass_tag}_{time_tag}_zero_cap2', 'mass_kg': mass,
                'outer_dt_s': dt, 'target_pos_rad': 0.0, 'max_torque_nm': 2.0})
    assert len(cases) == 20 and len({c['case_id'] for c in cases}) == 20
    for case in cases:
        case.update(radius_m=.1, stiffness_nm_per_rad=300.0, damping_nm_s_per_rad=12.0,
                    num_solver_iterations=1, num_internal_pgs_iterations=1,
                    num_internal_stabilization_iterations=2, outer_intervals=1)
        case['input_f32_words'] = {key: word(value) for key, value in case.items() if isinstance(value, float)}
    order = []
    for variant in ('baseline', 'candidate'):
        for case in cases:
            for observer_label, observer in (('on', 'on'), ('fresh_on', 'on'), ('off', 'off')):
                order.append({'slot': len(order), 'variant': variant, 'case_id': case['case_id'],
                    'observer': observer, 'label': observer_label})
    save(OWN / 'protocol-source-template.json', {'kind': 'NEW_PHYSICAL_STUDY_PRE_DATA_TEMPLATE', 'schema_version': 1,
        'status': 'NOT_ADMITTED_NO_WORLDS', 'cases': cases, 'process_order': order,
        'budget': {'fresh_native_processes': 120, 'scientific_reader_invocations': 1, 'pilot_processes': 0,
                   'retries': 0, 'replacement_processes': 0, 'old_reader_invocations': 0},
        'candidate': 'Diagnostic outside copy ForceBased D=inv(w+g); previous global rejection remains unchanged.',
        'physical_reference': 'Independent SI clipped backward Euler using I=(2/5)mR^2; integration velocity distinct from final writeback.',
        'promotion_gates': ['historical four primary accuracy criteria unchanged', 'literal physical stdout examples133-138 unchanged',
                            'both heavy grasp tests', 'full required local and GitHub CI'],
        'claims_excluded': ['production adoption', 'hardware validation', 'coupled contact cause', 'maturity increase'],
        'pre_data_required': ['exact source/compiler/features/flags/binaries and environment', 'pure tests',
                              'frozen once-only driver and independent reader', 'independent admission']})
    print(json.dumps({'vendor_files_each': len(paths), 'candidate_changed_paths': changed,
                      'cases': len(cases), 'budget': len(order), 'native_processes_launched': 0}))

if __name__ == '__main__':
    main()
