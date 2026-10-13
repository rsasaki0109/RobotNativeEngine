"""NEW120 whole framing/header/completeness guards; no source or SI arithmetic."""
from pathlib import Path
from fixed_contract import CASES,PROCESS_ORDER,STUDY_KIND
from spec_contract import folder_for,native_argv
from strict_io import bound,canonical,descriptor,gzip_records,integer,keys,loads,need

STATE_SCOPE='sorted individual native rigid bodies, collider, unique multibody, exact integration parameters and public exposed IEEE words; excludes full-set metadata, broad/narrow phase, private pipeline/solver/contact caches and observer buffers'
START_KEYS='study_kind case_id variant kind schema_version run_label target target_position_rad_word observer_enabled expected_outer_intervals native_scalar_bits state_codec state_scope declared_conditions'


def same(a,b,message):need(canonical(a)==canonical(b),message)


def conditions(case):
    w=case['input_f32_words']
    return dict(model='ForceBased',initial_coordinate_rad_word=0,body_linear_damping_word=0,body_angular_damping_word=0,
        limits_enabled=False,additional_solver_iterations=0,max_ccd_substeps=1,
        sphere_radius_m_word=int(w['radius_m'],16),sphere_mass_kg_word=int(w['mass_kg'],16),outer_dt_s_word=int(w['outer_dt_s'],16),
        stiffness_nm_per_rad_word=int(w['stiffness_nm_per_rad'],16),damping_nm_s_per_rad_word=int(w['damping_nm_s_per_rad'],16),
        ceiling_nm_word=int(w['max_torque_nm'],16),target_velocity_rad_s_word=0,gravity_words=[0,0,0],initial_generalized_velocity_word=0,
        passive_damping_word=0,armature_word=0,small_steps=1,pgs_passes=1,stabilization_passes=2,
        contacts_enabled=False,sleep_enabled=False,ccd_enabled=False,identity_coincident_x_frames=True)


def header(path,process):
    records,_,framing=gzip_records(path)
    need(len(records)==4,'complete success requires exactlyfour native records')
    start,initial,done,end=records
    keys(start,START_KEYS)
    case=next(c for c in CASES if c['case_id']==process['case_id'])
    target='negative' if '_neg_' in case['case_id'] else 'positive' if '_pos_' in case['case_id'] else 'zero'
    expected=dict(study_kind=STUDY_KIND,case_id=process['case_id'],variant=process['variant'],kind='run_start',schema_version=1,
        run_label=process['label'],target=target,target_position_rad_word=int(case['input_f32_words']['target_pos_rad'],16),
        observer_enabled=process['observer']=='on',expected_outer_intervals=1,native_scalar_bits=32,
        state_codec='postcard-1.1.3-raw-exact-consumption',state_scope=STATE_SCOPE,declared_conditions=conditions(case))
    same(start,expected,'native start/header/input words differ')
    keys(initial,'kind schema_version checkpoint');keys(done,'kind schema_version completed_outer_interval checkpoint construction_companion stage_v1 impulse_v1_ieee_words')
    for record in (initial,done,end):integer(record['schema_version']);need(record['schema_version']==1,'native record schema differs')
    integer(done['completed_outer_interval']);need(initial['kind']=='initial_state' and done['kind']=='completed_interval' and done['completed_outer_interval']==1,'native interval incomplete')
    for checkpoint in (initial['checkpoint'],done['checkpoint']):
        need(type(checkpoint) is dict,'checkpoint unavailable')
        integer(checkpoint['schema_version']);integer(checkpoint['native_scalar_bits'])
        need(checkpoint['schema_version']==1 and checkpoint['native_scalar_bits']==32 and checkpoint['state_scope']==STATE_SCOPE,'checkpoint scope/width/version differ')
    telemetry=('construction_companion','stage_v1','impulse_v1_ieee_words')
    if process['observer']=='off':need(all(done[k] is None for k in telemetry),'OFF observer payload must be absent')
    else:need(all(type(done[k]) is dict for k in telemetry),'ON observer payload missing')
    keys(end,'kind schema_version status progress records_written_before_terminal error')
    integer(end['records_written_before_terminal'])
    expected_progress=dict(fixture_construction_started=True,fixture_constructed=True,pipeline_step_started=True,pipeline_step_returned=True,
        completed_outer_intervals=1,initial_records_written=1,completed_records_written=1,partial_records_written=0)
    same(end['progress'],expected_progress,'native completed progress differs')
    need(end['kind']=='run_end' and end['status']=='collected' and end['error'] is None and end['records_written_before_terminal']==3,'native terminal failed/incomplete')
    return dict(process=process,framing=framing,whole_raw=descriptor(path))


def complete_cohort(spec):
    root=Path(spec['output_root']);terminal=loads((root/'driver-terminal.json').read_bytes())
    need(terminal['status']=='all_120_processes_completed' and terminal['actual_processes_started']==120 and terminal['original_process_terminals']==120,'coordinator incomplete')
    need(terminal['native_processes_started_count_is_complete'] and terminal['prelaunch_directory_barriers_complete'] and terminal['source_inputs_unchanged'],'coordinator unknown/source guard/durability failure')
    need(terminal['error'] is None and terminal['retries']==0 and terminal['resumed'] is False and terminal['unlaunched_case_labels']==terminal['launch_outcome_unknown_case_labels']==[],'coordinator failed/retried/partial')
    need(len(terminal['cases'])==len(terminal['case_launch_accounting'])==120,'missing native original receipts')
    entries=[]
    for index,process in enumerate(PROCESS_ORDER):
        folder=folder_for(spec,process);r=loads((folder/'process-terminal.json').read_bytes())
        same(r,terminal['cases'][index],'original receipt differs from memory terminal');same(r['case'],process,'native process identity differs')
        same(r['argv'],native_argv(spec,process),'native original argv differs')
        integer(r['actual_exit_code']);integer(r['attempt_index']);integer(r['pid'])
        need(r['actual_exit_code']==0 and r['attempt_index']==index and r['native_process_started'] is True and r['timed_out'] is False and r['error'] is None,'native original process failed')
        for key in ('stdout','stderr','raw_output'):bound(r[key])
        need(r['raw_output']['path']==str(folder/'raw.jsonl.gz'),'substituted raw path')
        a=terminal['case_launch_accounting'][index]
        need(a['status']=='known_started_exit_observed' and a['actual_exit_code']==0 and a['process_terminal_storage']=='stored_fsynced' and a['storage_error'] is None,'incomplete native launch/storage accounting')
        entries.append(header(folder/'raw.jsonl.gz',process))
    return dict(kind='physical20_prereader_byte_header_completion_guard',native_original_processes=120,whole_files=entries,
                source_arithmetic_or_SI_reanalysis=False,scientific_reader_invocations=0)
