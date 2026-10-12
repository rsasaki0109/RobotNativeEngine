"""Read-only source/receipt audit, without importing the measurement reader."""
import ast
import copy
import datetime
import difflib
import hashlib
import json
from pathlib import Path
import stat

Q = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-4/current-stage-qualification')
R = Q / 'postcollection-handle-schema-repair-v1'
OUT = Path(__file__).parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path)
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def read(path):
    return json.loads(Path(path).read_bytes())


def check(value):
    assert sha(value['path']) == value['sha256'], value['path']
    assert Path(value['path']).stat().st_size == value['bytes'], value['path']


def save(name, value):
    path = OUT / name
    assert not path.exists(), path
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    return ref(path)


def tree(path):
    return ast.parse(Path(path).read_bytes())


def same(left, right):
    assert ast.dump(left, include_attributes=False) == ast.dump(right, include_attributes=False)


def functions(value):
    return {n.name: n for n in value.body if isinstance(n, ast.FunctionDef)}


class HandleToTuple(ast.NodeTransformer):
    def visit_Call(self, node):
        self.generic_visit(node)
        if isinstance(node.func, ast.Name) and node.func.id == 'native_joint_handle_words':
            node.func.id = 'tuple'
        return node


class ProtocolParentReverse(ast.NodeTransformer):
    def visit_Call(self, node):
        self.generic_visit(node)
        if isinstance(node.func, ast.Name) and node.func.id == 'Path' and node.args:
            arg = node.args[0]
            if isinstance(arg, ast.Subscript) and isinstance(arg.value, ast.Name) and arg.value.id == 'protocol_record':
                assert isinstance(arg.slice, ast.Constant) and arg.slice.value == 'path'
                node.args[0] = ast.Name(id='__file__', ctx=ast.Load())
        return node


class FixtureReverse(ast.NodeTransformer):
    def visit_Attribute(self, node):
        if isinstance(node.value, ast.Name) and node.value.id == 'HERE' and node.attr == 'parent':
            return ast.Name(id='HERE', ctx=ast.Load())
        return self.generic_visit(node)

    def visit_Dict(self, node):
        self.generic_visit(node)
        for i, key in enumerate(node.keys):
            if isinstance(key, ast.Constant) and key.value == 'joint_handle':
                assert isinstance(node.values[i], ast.Dict)
                node.values[i] = ast.List(elts=[ast.Constant(value=3), ast.Constant(value=0)], ctx=ast.Load())
        return node


def main():
    manifest_path = R / 'frozen-repair-manifest.json'
    manifest = read(manifest_path)
    assert sha(manifest_path) == 'b640876ff36d3e1ff9821be56e45a0463cd006a8ded04f0a10f9ea36a5c8a366'
    refs = [manifest[k] for k in ['first_frame_diagnosis', 'narrow_diff', 'original_collection', 'original_failed_audit', 'original_failed_command', 'original_protocol', 'pure_tests']]
    refs += manifest['original_reader_closure'] + manifest['repaired_reader_closure'] + manifest['rust_schema_sources']
    for value in refs:
        check(value)
    assert sha(R / 'narrow-handle-schema-repair.patch') == '011b88b8e5b3478ed1f4e21456fb3d3cf71cbe9e948165cfd798c4f049394955'
    old_closure = {Path(v['path']).name: v for v in manifest['original_reader_closure']}
    new_closure = {Path(v['path']).name: v for v in manifest['repaired_reader_closure']}
    assert set(old_closure) == set(new_closure) == {'audit_coupled_stages.py', 'stage_analysis.py', 'stage_schema.py', 'stage_numeric.py'}
    assert (Q / 'stage_numeric.py').read_bytes() == (R / 'stage_numeric.py').read_bytes()
    copied, added = [], []
    order = ['audit_coupled_stages.py', 'stage_analysis.py', 'stage_schema.py', 'test_stage_reader.py']
    for path in sorted(R.glob('*.py')):
        original = Q / path.name
        if original.exists():
            copied.append({'name': path.name, 'original': ref(original), 'repaired': ref(path), 'literal_equal': original.read_bytes() == path.read_bytes()})
        else:
            added.append({'name': path.name, 'source': ref(path), 'scope': 'New pure metadata/shape/provenance tests, not measurement analysis'})
    diff = ''.join(''.join(difflib.unified_diff((Q / name).read_text().splitlines(keepends=True), (R / name).read_text().splitlines(keepends=True), fromfile='original/' + name, tofile='postcollection-handle-schema-repair-v1/' + name)) for name in order)
    assert diff.encode() == (R / 'narrow-handle-schema-repair.patch').read_bytes()
    assert len(copied) == 6 and len(added) == 2
    assert {x['name'] for x in copied if not x['literal_equal']} == set(order)
    old_analysis, new_analysis = tree(Q / 'stage_analysis.py'), tree(R / 'stage_analysis.py')
    for node in new_analysis.body:
        if isinstance(node, ast.ImportFrom) and node.module == 'stage_schema':
            node.names = [a for a in node.names if a.name != 'native_joint_handle_words']
    same(old_analysis, HandleToTuple().visit(new_analysis))
    old_schema, new_schema = tree(Q / 'stage_schema.py'), tree(R / 'stage_schema.py')
    new_schema.body = [n for n in new_schema.body if not (isinstance(n, ast.FunctionDef) and n.name == 'native_joint_handle_words')]
    for node in new_schema.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'validate_row':
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name) and sub.func.id == 'native_joint_handle_words':
                    sub.func.id = 'words'
                    sub.args.append(ast.Constant(value=32))
                    sub.keywords = [ast.keyword(arg='expected', value=ast.Constant(value=2))]
        else:
            HandleToTuple().visit(node)
    same(old_schema, new_schema)
    old_audit, new_audit = functions(tree(Q / 'audit_coupled_stages.py')), functions(tree(R / 'audit_coupled_stages.py'))
    assert set(new_audit) - set(old_audit) == {'check_repair'} and not (set(old_audit) - set(new_audit))
    unchanged = sorted(set(old_audit) - {'check_protocol', 'audit', 'main'})
    for name in unchanged:
        same(old_audit[name], new_audit[name])
    same(old_audit['check_protocol'], ProtocolParentReverse().visit(copy.deepcopy(new_audit['check_protocol'])))
    audit = copy.deepcopy(new_audit['audit'])
    removed = [s for s in audit.body if isinstance(s, ast.Assign) and len(s.targets) == 1 and isinstance(s.targets[0], ast.Name) and s.targets[0].id == 'repair_record']
    assert len(removed) == 1 and isinstance(removed[0].value, ast.Call) and removed[0].value.func.id == 'check_repair'
    audit.body = [s for s in audit.body if s not in removed]
    extras = {'post_collection_handle_schema_repair', 'repaired_reader_existed_before_data', 'original_predata_coupled_audit_status', 'numerical_formulas_and_scientific_stage_gates_unchanged'}
    for node in ast.walk(audit):
        if isinstance(node, ast.Dict):
            pairs = [(k, v) for k, v in zip(node.keys, node.values) if not (isinstance(k, ast.Constant) and k.value in extras)]
            node.keys, node.values = [k for k, v in pairs], [v for k, v in pairs]
    same(old_audit['audit'], audit)
    main_node = copy.deepcopy(new_audit['main'])
    for node in ast.walk(main_node):
        if isinstance(node, ast.List) and node.elts and all(isinstance(x, ast.Constant) for x in node.elts) and any(x.value == 'repair-manifest' for x in node.elts):
            assert [x.value for x in node.elts][-2:] == ['repair-manifest', 'repair-manifest-sha256']
            node.elts = node.elts[:-2]
        if isinstance(node, ast.Dict):
            pairs = [(k, v) for k, v in zip(node.keys, node.values) if not (isinstance(k, ast.Constant) and k.value in {'repair_manifest_sha256_argument', 'reader_is_post_collection_schema_repair'})]
            node.keys, node.values = [k for k, v in pairs], [v for k, v in pairs]
    same(old_audit['main'], main_node)
    same(tree(Q / 'test_stage_reader.py'), FixtureReverse().visit(tree(R / 'test_stage_reader.py')))
    qualification = read(manifest['pure_tests']['path'])
    assert qualification['status'] == 'ACTUAL_PURE_POSTCOLLECTION_SHAPE_AND_PROVENANCE_TESTS_PASS'
    assert type(qualification['native_worlds']) is int and qualification['native_worlds'] == 0 and qualification['complete_raw_measurement_rereads'] == 0
    outputs = {}
    for result in qualification['tests']:
        assert result['actual_exit_code'] == 0
        check(result['stdout']); check(result['stderr'])
        assert Path(result['stderr']['path']).read_bytes() == b''
        outputs[result['name']] = read(result['stdout']['path'])
    for value in qualification['test_source_artifacts']:
        check(value)
    check(qualification['prior_shape_tests'])
    assert all(v['status'] == 'PASS' and v['native_worlds'] == 0 for v in outputs.values())
    assert len(outputs['test_native_handle_shape.py']['checked']) == 6 and len(outputs['test_native_handle_shape.py']['rejected']) == 14
    assert len(outputs['test_stage_reader.py']['passed']) == 11 and len(outputs['test_stage_reader.py']['rejected']) == 28
    assert len(outputs['test_repair_provenance.py']['rejected']) == 7
    collection, failure = read(manifest['original_collection']['path']), read(manifest['original_failed_audit']['path'])
    assert failure['status'] == 'FAIL_RETAINED_WITHOUT_CRITERIA_CHANGE' and failure['error'] == 'word vector must be a list'
    assert datetime.datetime.fromisoformat(collection['ended_utc']) < datetime.datetime.fromisoformat(manifest['registered_utc'])
    assert type(manifest['authoritative_worlds_rerun']) is int and manifest['authoritative_worlds_rerun'] == 0
    original_modes = []
    required_readonly = {v['path'] for v in manifest['original_reader_closure']} | {manifest['original_protocol']['path']}
    for value in manifest['original_reader_closure'] + [manifest[k] for k in ['original_protocol', 'original_collection', 'original_failed_audit', 'original_failed_command']]:
        mode = stat.S_IMODE(Path(value['path']).stat().st_mode)
        if value['path'] in required_readonly:
            assert mode == 0o444, (value['path'], mode)
        original_modes.append({'path': value['path'], 'mode_octal': oct(mode), 'exact_recorded_SHA_verified': True, 'scope': 'Original source/protocol readonly; failed-audit/command/index metadata independently SHA-bound, actual mode reported literally'})
    raw_stats = []
    for row in collection['runs']:
        value = row['trace']; path = Path(value['path']); mode = stat.S_IMODE(path.stat().st_mode)
        assert path.stat().st_size == value['bytes']
        raw_stats.append({'path': value['path'], 'original_recorded_sha256_not_reread': value['sha256'], 'original_recorded_bytes_actual_stat_exact': value['bytes'], 'mode_octal': oct(mode)})
    assert len(raw_stats) == 18
    candidate = read(R / 'repair-manifest-candidate-for-pure-tests.json')
    changed = sorted(k for k in manifest if candidate.get(k) != manifest[k])
    assert changed == ['pure_tests', 'registered_utc'], changed
    for value in refs:
        check(value)
    comparison = save('source-comparison.json', {'original_copy_sources': copied, 'new_pure_test_sources': added, 'narrow_diff_exact_recomputed': True,
                      'unchanged_arithmetic_module': ref(R / 'stage_numeric.py'), 'AST_scientific_analysis_exact_after_reversing_only_handle_access': True,
                      'AST_schema_exact_after_reversing_only_handle_shape_identity': True, 'unchanged_audit_function_AST_names': unchanged,
                      'AST_original_audit_and_main_exact_after_reversing_only_late_provenance_and_CLI': True,
                      'AST_synthetic_stage_fixture_exact_after_only_handle_shape_and_parent_relocation': True,
                      'pure_candidate_to_final_manifest_changed_keys': changed})
    result = {'schema_version': 1, 'status': 'PASS_STATIC_POST_COLLECTION_HANDLE_SCHEMA_REPAIR_SCOPE_REVIEW', 'audited_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'auditor': ref(__file__), 'frozen_repair_manifest': ref(manifest_path), 'narrow_patch': ref(R / 'narrow-handle-schema-repair.patch'), 'source_comparison': comparison,
              'metadata_correction': 'Rust MultibodyJointHandle is a serde newtype around named Index{index:u32,generation:u32}; exact keys/bounded integers, raw rows unnormalized. No array/tuple fallback or bool/noninteger/extra-key admission.',
              'arithmetic_stage_gates_thresholds_domains_and_coverage_unchanged': True,
              'late_provenance': 'Repair registered after collection end. Revised reader explicitly states repaired_reader_existed_before_data=false; original failed pre-data reader attempt remains literal retained failure.',
              'original_failed_audit': manifest['original_failed_audit'], 'original_failed_command': manifest['original_failed_command'], 'original_protocol': manifest['original_protocol'], 'original_collection': manifest['original_collection'],
              'actual_existing_pure_qualification': manifest['pure_tests'], 'actual_pure_case_counts': {'handle_positive': 6, 'handle_negative': 14, 'stage_positive': 11, 'stage_negative': 28, 'provenance_negative': 7},
              'original_artifact_SHA_and_actual_modes': original_modes, 'original_18_raw_stream_metadata_stat_only': raw_stats,
              'raw_integrity_scope': 'Original stream sizes/modes and recorded SHA bindings retained; compressed/decompressed measurement bytes were not reread or rehashed by this source review.',
              'original_separate_scalar72_scope': 'Separate original scalar qualification remains separate from coupled18; no new90 worlds or numerical outcomes inferred.',
              'review_guard_correction': 'Initial external review guards incorrectly promoted parent blanket0444 wording into a metadata/raw mode requirement. Parent clarified creation modes are allowed and hash-based artifact binding is the actual preservation gate. Metadata/raw modes are recorded literally, without chmod or new scientific requirements. Original reader/protocol frozen modes were0444. An external helper syntax typo was also corrected before execution; no original or repaired measurement reader/source was edited.',
              'material_blockers': [], 'no_original_or_repaired_source_edits': True,
              'no_Rust_build_tests_worlds_complete_data_reader_GitHub_or_CI_polling': True,
              'auditor_operations': 'Static source/reference SHA, AST/diff comparisons, prior pure-test receipt reads, raw-file stat only; existing tests not rerun.',
              'postcollection_review_not_predata_qualification': True, 'complete_coupled_reread_result_not_claimed': True}
    proof = save('independent-repair-scope-review.json', result)
    print(json.dumps({'proof': proof, 'source_comparison': comparison, 'material_blockers': 0}))


if __name__ == '__main__':
    main()
