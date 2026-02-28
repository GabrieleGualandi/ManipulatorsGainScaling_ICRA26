function log = core_log(par, log, st, t, tStart, isSimulation, isJacobian, data)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORE_LOG Records simulation variables into the log structure.
% This function populates the pre-allocated log structure with current state 
% values and intermediate control/attack data. It supports minimal logging 
% for numerical Jacobian calculations and performance-optimized lookaheads.
%
% Inputs:
%   par          - Configuration structure.
%   log          - Existing log structure.
%   st           - Current SimulatorState object.
%   t            - Current global time step.
%   tStart       - Start time of the current simulation segment.
%   isSimulation - Boolean flag for lookahead simulations.
%   isJacobian   - Boolean flag for Jacobian differentiated mode (forces minimal logging).
%   data         - Structure containing auxiliary signals from control/attack dispatchers.
%
% Outputs:
%   log          - Updated log structure.
%
% SEE ALSO: CORE_INIT, CORE_CONTROL, CORE_ATTACK, CORESIMULATOR


    if isSimulation && ~isJacobian
        return;
    end
    
    idx = t - tStart + 1;
    
    if isJacobian
        % Minimal logging for Jacobian calculation (attack.perturbationG)
        % Strictly necessary variables: Hand Position & Velocity
        if isfield(st.ent.geometryReal, 'handPosition_task')
            log.handPosition_task(:, idx) = st.ent.geometryReal.handPosition_task;
        end
        if isfield(st.ent.geometryReal, 'handGeneralizedVelocity_task')
            log.handGeneralizedVelocity_task(:, idx) = st.ent.geometryReal.handGeneralizedVelocity_task;
        end
        return;
    end

    % State Variables
    log.sys_real_q(:, idx) = st.sys_real_q;
    log.sys_real_qdot(:, idx) = st.sys_real_qdot;
    log.sys_observer_q_hat(:, idx) = st.sys_observer_q_hat;
    log.sys_observer_qdot_hat(:, idx) = st.sys_observer_qdot_hat;
    log.sys_sensingFreeProjected_q(:, idx) = st.sys_sensingFreeProjected_q;
    log.sys_sensingFreeProjected_qdot(:, idx) = st.sys_sensingFreeProjected_qdot;

    if isfield(par.def.active.scheme, 'usePDgainsScaling') && par.def.active.scheme.usePDgainsScaling
        log.gainScaling_q(:, idx) = st.sys_gainScaling_q;
    end

    if par.def.passive.scheme.useChiSquared
        log.ADS_z(:, idx) = data.ADS_z;
        log.ADS_r(:, idx) = data.ADS_r;
    end

    log.y_prev_total_attack(:, idx) = st.y_prev_total_attack;
    log.distance2attackGoal(:, idx) = data.distance2attackGoal;
    log.yAttack(:, idx) = data.yAttack;
    log.p_goal(:, idx) = data.p_goal;
    log.delta_y_optimal(:, idx) = data.delta_y_optimal;
    log.handOrientation(:, idx) = st.ent.geometryReal.handOrientation(:);

    if log.logFlags.multiJacobianIndexes_kstar
        log.multiJacobianIndexes_kstar(:, idx) = data.multiJacobianIndexes_kstar; 
    end
    
    if log.logFlags.zProj
        log.zProj(:, idx) = data.zProj; 
    end

    if log.logFlags.ddot_q_control
        log.ddot_q_control(:, idx) = data.ddot_q_control; 
    end

    % Errors & Geometry
    log.error_real_pose(:, idx) = st.ent.errors.real.pose;
    log.error_real_vel(:, idx) = st.ent.errors.real.vel;
    log.error_estimated_pose(:, idx) = st.ent.errors.estimated.pose;
    log.error_estimated_vel(:, idx) = st.ent.errors.estimated.vel;
    log.error_sensingFreeProjected_pose(:, idx) = st.ent.errors.sensingFreeProjected.pose;
    log.error_sensingFreeProjected_vel(:, idx) = st.ent.errors.sensingFreeProjected.vel;

    log.handPosition_task(:, idx)       = st.ent.geometryReal.handPosition_task;
    log.handGeneralizedVelocity_task(:, idx) = st.ent.geometryReal.handGeneralizedVelocity_task;
    log.ddot_q_final(:, idx) = st.ent.ddot_q_final;
    log.ddot_q_final_unsaturated(:, idx) = data.ddot_q_final_unsaturated;

end
