function [st] = core_updateState(par, st, ddot_q_final, y_tilde, isSimulation, isJacobian)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORE_UPDATESTATE Propagates the system's dynamics to the next time step.
% This function evolves the Real System (with optional process noise), 
% the Sensing-Free Projector, the State Observer (Kalman/Luenberger),
% and all auxiliary smoother dynamics for defence tasks.
%
% Inputs:
%   par          - Configuration structure.
%   st           - Current SimulatorState object.
%   ddot_q_final - Total control command (acceleration/torque) applied.
%   y_tilde      - Corrupted/Noisy sensor measurement vector.
%   isSimulation - Boolean flag for internal lookahead runs.
%   isJacobian   - Boolean flag for numerical differentiation mode.
%
% Outputs:
%   st           - Updated SimulatorState object (st.next populated with future states).
%
% NOTE: The actual state transition (st.next -> st) happens in CORESIMULATOR 
% via the st.update() call.
%
% SEE ALSO: SIMULATORSTATE.UPDATE, CORESIMULATOR, CORE_CONTROL

    

    
    %% Real System
    sys_real_state = [st.sys_real_q; st.sys_real_qdot];
    sys_real_state_next = par.models.sys_real.A * sys_real_state + par.models.sys_real.B * ddot_q_final;
    n_half_sys_real = numel(sys_real_state_next) / 2;

    if isSimulation || isJacobian || not(par.sim.modelProcessNoise)
        st.next.sys_real_q = sys_real_state_next(1:n_half_sys_real);
        st.next.sys_real_qdot = sys_real_state_next(n_half_sys_real + 1 : end);
    else
        Q = par.sys.processCov;
        m = size(Q,1);
        w = mvnrnd(zeros(1,m), (Q+Q.')/2, 1).';
        st.next.sys_real_q = sys_real_state_next(1:n_half_sys_real) + w(1:n_half_sys_real);
        st.next.sys_real_qdot = sys_real_state_next(n_half_sys_real + 1 : end) + w(n_half_sys_real + 1 : end);
    end

    %% Sensing-Free Projected System
    if par.def.passive.isProjectedStateIdeal
        st.next.sys_sensingFreeProjected_q = st.next.sys_real_q;
        st.next.sys_sensingFreeProjected_qdot = st.next.sys_real_qdot;
    else
        sys_sensingFreeProjected_state = [st.sys_sensingFreeProjected_q; st.sys_sensingFreeProjected_qdot];
        sys_sensingFreeProjected_state_next = par.models.sys_sensingFreeProjected.A * sys_sensingFreeProjected_state + par.models.sys_sensingFreeProjected.B * ddot_q_final;
        n_half_sys_sensingFreeProjected = numel(sys_sensingFreeProjected_state_next) / 2;
        st.next.sys_sensingFreeProjected_q = sys_sensingFreeProjected_state_next(1:n_half_sys_sensingFreeProjected);
        st.next.sys_sensingFreeProjected_qdot = sys_sensingFreeProjected_state_next(n_half_sys_sensingFreeProjected + 1 : end);
    end

    %% State Estimator
    sys_observer_x_hat_current = [st.sys_observer_q_hat; st.sys_observer_qdot_hat];
    sys_observer_y_hat_at_k = par.models.sys_observer.C * sys_observer_x_hat_current + par.models.sys_observer.D * ddot_q_final;
    sys_observer_r_k = y_tilde - sys_observer_y_hat_at_k;
    
    sys_observer_x_hat_next_value = par.models.sys_observer.A * sys_observer_x_hat_current + ...
                                    par.models.sys_observer.B * ddot_q_final + ...
                                    par.models.sys_observer.L * sys_observer_r_k;
    
    n_observer = numel(st.sys_observer_q_hat);
    st.next.sys_observer_q_hat = sys_observer_x_hat_next_value(1:n_observer);
    st.next.sys_observer_qdot_hat = sys_observer_x_hat_next_value(n_observer + 1 : end);

    st.next.current_Pz = propagate_covariance_onestep(par.def.scheme.propagator, st.current_Pz);

    %% Gain Scaling States
    if isfield(par.def.active.scheme, 'usePDgainsScaling') && par.def.active.scheme.usePDgainsScaling
         gainScaling_state = [st.sys_gainScaling_q; st.sys_gainScaling_qdot];
         gainScaling_state_next = par.models.sys_gainScaling.A * gainScaling_state + par.models.sys_gainScaling.B * st.target_gainScaling;
         n_half = numel(gainScaling_state_next) / 2;
         st.next.sys_gainScaling_q = gainScaling_state_next(1:n_half);
         st.next.sys_gainScaling_qdot = gainScaling_state_next(n_half + 1 : end);
    end

    %% Attack Accumulator
    st.next.y_prev_total_attack = st.y_prev_total_attack;

end
