function [ddot_q_control, qd_nullSpace] = control(par, st, desired, inverseJ_primary)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CONTROL Computes the nominal PD control law with null-space damping.
% This function implements a Cartesian PD controller in the joint space using 
% a provided task-space pseudoinverse. It also includes an optional 
% null-space damping term to stabilize the internal motion.
%
% Inputs:
%   par              - Configuration structure.
%   st               - Current SimulatorState object.
%   desired          - Reference trajectory structure at time t.
%   inverseJ_primary - Pre-computed primary task pseudoinverse (nxm).
%
% Outputs:
%   ddot_q_control   - Computed nominal control acceleration (nx1).
%   qd_nullSpace     - Current joint velocity projected into the task null-space.
%
% SEE ALSO: CORE_CONTROL, NOMINAL.CALCENTITIES


    Kp = par.control.PropGain;
    Kd = par.control.DerivGain;
    
    % Nominal PD
    ddot_q_control = inverseJ_primary * ( ...
        (desired.desired_acceleration(par.task.taskSpaceRows,:) + ...
        Kp * (st.ent.errors.estimated.pose) + ...
        Kd * (st.ent.errors.estimated.vel)) ...
        - st.ent.geometryEstimated.Jdot_task * st.sys_observer_qdot_hat);

    % Null-Space damping
    qd_nullSpace = st.ent.geometryEstimated.P_task * st.ent.geometryEstimated.qdot;
    ddot_q_control = ddot_q_control - par.control.weightNullspaceDampingCoeff * diag(par.control.weightNullspaceDamping)*qd_nullSpace;
end
