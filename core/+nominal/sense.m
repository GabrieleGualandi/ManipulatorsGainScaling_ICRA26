function [y_tilde] = sense(par, st, yAttack)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% SENSE Simulates robot sensor measurements and adversarial injection.
% This function translates the real robot state into a sensor output vector 
% and adds the attack signal (yAttack) provided by the attacker plugins.
%
% Inputs:
%   par     - Configuration structure.
%   st      - Current SimulatorState object.
%   yAttack - Adversarial vector to be injected into the measurement.
%
% Outputs:
%   y_tilde - Final corrupted measurement vector (the only signal visible to the controller).
%
% SEE ALSO: CORE_ATTACK, CORE_UPDATESTATE


    %% 1. Measurement Simulation
    sys_real_state_current = [st.sys_real_q; st.sys_real_qdot];
    n_sys_real = numel(st.sys_real_q); 
    ddot_q_default = zeros(n_sys_real, 1); 
    y = par.models.sys_real.C * sys_real_state_current + par.models.sys_real.D * ddot_q_default;
    y_tilde = y + yAttack; 
end
