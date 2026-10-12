from pathlib import Path
root=Path(__file__).parent/'source-copy/third_party/rapier3d/src'
def edit(name,transform):
    p=root/name; raw=p.read_bytes(); t=raw.decode().replace('\r\n','\n'); u=transform(t)
    if u==t: raise RuntimeError('no change '+name)
    p.write_bytes(u.replace('\n','\r\n').encode() if b'\r\n' in raw else u.encode())
def rep(t,a,b,count=1):
    if t.count(a)!=count:raise RuntimeError(('match',t.count(a),a[:100]))
    return t.replace(a,b)

def module(t):
    t=rep(t,'pub use collision_pipeline::CollisionPipeline;','pub use collision_pipeline::CollisionPipeline;\npub(crate) use multibody_motor_construction::{InternalMotorConstructionRecorder, InternalMotorConstructionRowRecorder};\npub use multibody_motor_construction::{MultibodyMotorConstructionSource, MultibodyMotorConstructionPhase, MultibodyMotorConstructionAvailability, MultibodyMotorConstructionRow, MultibodyMotorConstructionEndpoint, MultibodyMotorConstructionEvent, MultibodyMotorConstructionTelemetry};')
    return rep(t,'mod multibody_motor_impulse;','mod multibody_motor_impulse;\nmod multibody_motor_construction;')
edit('pipeline/mod.rs',module)

def unit(t):
    t=rep(t,'use na::DVector;','use na::DVector;\nuse crate::pipeline::InternalMotorConstructionRowRecorder;')
    start=t.index('pub fn unit_joint_motor_constraint(');body=t.index('    let inv_dt = params.inv_dt();',start)
    header=t[start:body]
    wrapper=header+'    unit_joint_motor_constraint_with_construction_inputs(params, multibody, link, motor, curr_pos, limits, dof_id, j_id, jacobians, constraints, insert_at, None);\n}\n\n'
    helper=header.replace('pub fn unit_joint_motor_constraint(', 'pub(crate) fn unit_joint_motor_constraint_with_construction_inputs(').replace('    insert_at: &mut usize,\n','    insert_at: &mut usize,\n    recorder: Option<InternalMotorConstructionRowRecorder<\'_>>,\n')
    t=t[:start]+wrapper+helper+t[body:]
    last=t.rfind('    constraints[*insert_at] = constraint;')
    capture='''    if let Some(recorder) = recorder {
        recorder.capture(crate::dynamics::MultibodyJointHandle(link.rigid_body_handle().0), motor, &motor_params, curr_pos, params.dt, inv_dt, limits, target_vel, &constraint, jacobians);
    }

'''
    return t[:last]+capture+t[last:]
edit('dynamics/joint/multibody_joint/unit_multibody_joint.rs',unit)

def joint(t):
    t=rep(t,'use na::{DVector, DVectorViewMut};','use na::{DVector, DVectorViewMut};\nuse crate::pipeline::{InternalMotorConstructionRecorder, InternalMotorConstructionRowRecorder};\nuse crate::dynamics::JointAxis;')
    start=t.index('    pub fn velocity_constraints(');body=t.index('        let j_id = &mut j_id;',start);header=t[start:body]
    wrapper=header+'        self.velocity_constraints_with_construction_inputs(params, multibody, link, j_id, jacobians, constraints, None)\n    }\n\n'
    wrapper=wrapper.replace('        mut j_id: usize,','        j_id: usize,')
    helper=header.replace('    pub fn velocity_constraints(', '    pub(crate) fn velocity_constraints_with_construction_inputs(').replace('        constraints: &mut [JointGenericOneBodyConstraint],\n','        constraints: &mut [JointGenericOneBodyConstraint],\n        mut recorder: Option<InternalMotorConstructionRecorder<\'_>>,\n')
    t=t[:start]+wrapper+helper+t[body:]
    t=rep(t,'        let j_id = &mut j_id;','''        let axes = [JointAxis::LinX, JointAxis::LinY, #[cfg(feature="dim3")] JointAxis::LinZ, JointAxis::AngX, #[cfg(feature="dim3")] JointAxis::AngY, #[cfg(feature="dim3")] JointAxis::AngZ];
        let j_id = &mut j_id;''')
    old='''                    joint::unit_joint_motor_constraint(
'''
    replacement='''                    let row_recorder = recorder.as_mut().map(|recorder| InternalMotorConstructionRowRecorder {
                        rows: &mut *recorder.rows,
                        constraint_index: recorder.constraint_base + num_constraints,
                        link_id: recorder.link_id,
                        axis: axes[i],
                    });
                    joint::unit_joint_motor_constraint_with_construction_inputs(
'''
    t=rep(t,old,replacement,2)
    a='''                        &mut num_constraints,
                    );
'''
    # Exactly the two motor call blocks, not limits.
    pos=0
    while True:
        call=t.find('joint::unit_joint_motor_constraint_with_construction_inputs(',pos)
        if call<0:break
        end=t.index(a,call)
        t=t[:end]+t[end:].replace(a,'''                        &mut num_constraints,
                        row_recorder,
                    );
''',1);pos=end+len(a)
    return t
edit('dynamics/joint/multibody_joint/multibody_joint.rs',joint)

def builder(t):
    t=rep(t,'use crate::utils;','use crate::utils;\nuse crate::pipeline::{InternalMotorConstructionRecorder, MultibodyMotorConstructionRow};')
    # Enum update keeps the old API; only Internal passes the extra recorder.
    start=t.index('    pub fn update(',t.index('pub(crate) fn append_internal_motor_stage_bindings'))
    body=t.index('        match self {',start);header=t[start:body]
    wrapper=header+'        self.update_with_construction_inputs(params, multibodies, bodies, jacobians, out, None);\n    }\n\n'
    helper=header.replace('    pub fn update(', '    pub(crate) fn update_with_construction_inputs(').replace('        out: &mut [JointGenericOneBodyConstraint],\n','        out: &mut [JointGenericOneBodyConstraint],\n        recorder: Option<&mut Vec<MultibodyMotorConstructionRow>>,\n')
    t=t[:start]+wrapper+helper+t[body:]
    t=rep(t,'            Self::Internal(builder) => builder.update(params, multibodies, jacobians, out),','            Self::Internal(builder) => builder.update_with_construction_inputs(params, multibodies, jacobians, out, recorder),')
    start=t.index('    pub fn update(',t.index('impl JointGenericVelocityOneBodyInternalConstraintBuilder'))
    body=t.index('        let mb = ',start);header=t[start:body]
    wrapper=header+'        self.update_with_construction_inputs(params, multibodies, jacobians, out, None);\n    }\n\n'
    helper=header.replace('    pub fn update(', '    fn update_with_construction_inputs(').replace('        out: &mut [JointGenericOneBodyConstraint],\n','        out: &mut [JointGenericOneBodyConstraint],\n        recorder: Option<&mut Vec<MultibodyMotorConstructionRow>>,\n')
    t=t[:start]+wrapper+helper+t[body:]
    t=rep(t,'        link.joint().velocity_constraints(\n','''        let recorder = if link.joint().ndofs() == 1 {
            recorder.map(|rows| InternalMotorConstructionRecorder {rows, constraint_base:self.constraint_id, link_id:self.link.id})
        } else {None};
        link.joint().velocity_constraints_with_construction_inputs(
''')
    t=t[:start]+t[start:].replace('            &mut out[self.constraint_id..],\n        );','            &mut out[self.constraint_id..],\n            recorder,\n        );',1)
    return t
edit('dynamics/solver/joint_constraint/joint_generic_constraint_builder.rs',builder)

def set_(t):
    t=rep(t,'use crate::utils;', 'use crate::utils;\nuse crate::pipeline::MultibodyMotorConstructionRow;') if 'use crate::utils;' in t else 'use crate::pipeline::MultibodyMotorConstructionRow;\n'+t
    start=t.index('    pub fn update(');body=t.index('        for builder in &mut self.generic_velocity_constraints_builder',start);header=t[start:body]
    wrapper=header+'        self.update_with_construction_inputs(params, multibodies, solver_bodies, None);\n    }\n\n'
    helper=header.replace('    pub fn update(', '    pub(crate) fn update_with_construction_inputs(').replace('        solver_bodies: &[SolverBody],\n','        solver_bodies: &[SolverBody],\n        mut recorder: Option<&mut Vec<MultibodyMotorConstructionRow>>,\n')
    t=t[:start]+wrapper+helper+t[body:]
    old='''        for builder in &mut self.generic_velocity_one_body_constraints_builder {
            builder.update(
                params,
                multibodies,
                solver_bodies,
                &mut self.generic_jacobians,
                &mut self.generic_velocity_one_body_constraints,
            );
        }
'''
    new=old.replace('builder.update(', 'builder.update_with_construction_inputs(').replace('                &mut self.generic_velocity_one_body_constraints,\n','                &mut self.generic_velocity_one_body_constraints,\n                recorder.as_deref_mut(),\n')
    return rep(t,old,new)
edit('dynamics/solver/joint_constraint/joint_constraints_set.rs',set_)

def pipeline(t):
    t=rep(t,'    PhysicsHooks, QueryPipeline,','    PhysicsHooks, QueryPipeline, MultibodyMotorConstructionTelemetry,')
    t=rep(t,'    completed_motor_stage_telemetry: Option<MultibodyMotorStageTelemetry>,','''    completed_motor_stage_telemetry: Option<MultibodyMotorStageTelemetry>,
    motor_construction_telemetry_enabled: bool,
    pending_motor_construction_telemetry: Option<MultibodyMotorConstructionTelemetry>,
    completed_motor_construction_telemetry: Option<MultibodyMotorConstructionTelemetry>,''')
    t=rep(t,'            completed_motor_stage_telemetry: None,','''            completed_motor_stage_telemetry: None,
            motor_construction_telemetry_enabled: false,
            pending_motor_construction_telemetry: None,
            completed_motor_construction_telemetry: None,''')
    t=rep(t,'    fn clear_modified_colliders(','''    /// Enables the independent read-only construction/coordinate companion.
    ///
    /// Disabled by default and independent of version-one stage observation.
    /// Toggling clears this observer's pending/completed reports only.
    pub fn set_multibody_motor_construction_telemetry_enabled(&mut self, enabled: bool) {
        if self.motor_construction_telemetry_enabled != enabled {
            self.motor_construction_telemetry_enabled = enabled;
            self.pending_motor_construction_telemetry = None;
            self.completed_motor_construction_telemetry = None;
        }
    }

    /// Returns the completed construction companion, or None when unavailable.
    ///
    /// None includes disabled, pre-step, toggled and interrupted states. Empty
    /// completed events do not mean a measured zero. Runtime neutrality and
    /// physical calibration require separately registered qualification.
    pub fn completed_multibody_motor_constructions(&self) -> Option<&MultibodyMotorConstructionTelemetry> {
        self.completed_motor_construction_telemetry.as_ref()
    }

    fn clear_modified_colliders(''')
    t=rep(t,'        for (island, solver) in self.solvers[..islands.num_islands()].iter_mut().enumerate() {','''        let construction_partition = self.pending_motor_construction_telemetry.as_ref()
            .map(|report| report.ccd_partition_duration_words.len() - 1).unwrap_or(0);
        for (island, solver) in self.solvers[..islands.num_islands()].iter_mut().enumerate() {''')
    t=rep(t,'                stage_partition,\n                island,\n            );','''                stage_partition,
                island,
            );
            solver.prepare_motor_construction_telemetry(self.motor_construction_telemetry_enabled, construction_partition, island);''')
    t=rep(t,'        // Generate contact force events if needed.','''        if let Some(report) = &mut self.pending_motor_construction_telemetry {
            for solver in &mut self.solvers[..islands.num_islands()] {
                report.events.extend(solver.take_motor_construction_events());
            }
        }

        // Generate contact force events if needed.''')
    t=rep(t,'        // Clear every cached island, including islands inactive this step.','''        self.completed_motor_construction_telemetry = None;
        self.pending_motor_construction_telemetry = self.motor_construction_telemetry_enabled
            .then(|| MultibodyMotorConstructionTelemetry::new(integration_parameters.dt));
        // Clear every cached island, including islands inactive this step.''')
    t=rep(t,'            solver.prepare_motor_stage_telemetry(false, 0, 0);','            solver.prepare_motor_stage_telemetry(false, 0, 0);\n            solver.prepare_motor_construction_telemetry(false, 0, 0);')
    t=rep(t,'            self.build_islands_and_solve_velocity_constraints(','            if let Some(report) = &mut self.pending_motor_construction_telemetry {\n                report.ccd_partition_duration_words.push(motor_stage_word(integration_parameters.dt));\n            }\n            self.build_islands_and_solve_velocity_constraints(')
    t=rep(t,'        self.completed_motor_stage_telemetry = self.pending_motor_stage_telemetry.take();','        self.completed_motor_stage_telemetry = self.pending_motor_stage_telemetry.take();\n        self.completed_motor_construction_telemetry = self.pending_motor_construction_telemetry.take();')
    return t
edit('pipeline/physics_pipeline.rs',pipeline)

def island(t):
    t=rep(t,'use crate::pipeline::{InternalMotorImpulseSample, MultibodyMotorStageEvent};','use crate::pipeline::{InternalMotorImpulseSample, MultibodyMotorStageEvent, MultibodyMotorConstructionEvent};')
    t=rep(t,'    pub fn init_and_solve(','''    pub(crate) fn prepare_motor_construction_telemetry(&mut self, enabled: bool, partition: usize, island: usize) {
        self.velocity_solver.motor_construction_events = enabled.then(Vec::new);
        self.velocity_solver.motor_construction_partition_index = partition;
        self.velocity_solver.motor_construction_island_index = island;
        self.velocity_solver.motor_construction_final_context = None;
        self.velocity_solver.motor_construction_final_bindings.clear();
    }

    pub(crate) fn take_motor_construction_events(&mut self) -> Vec<MultibodyMotorConstructionEvent> {
        self.velocity_solver.motor_construction_events.take().unwrap_or_default()
    }

    pub fn init_and_solve(''')
    return rep(t,'        counters.solver.velocity_writeback_time.pause();','''        self.velocity_solver.record_motor_construction_writeback(multibodies, &self.joint_constraints);
        counters.solver.velocity_writeback_time.pause();''')
edit('dynamics/solver/island_solver.rs',island)
