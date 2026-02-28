function [ddot_q_control, st] = PDScaling(par, st, ddot_q_control, zProj)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% PDSCALING Applies PD Gain Scaling based on anomaly detection.
% This function scales the nominal control acceleration by a gain smoother 
% value (st.sys_gainScaling_q). The target gain is updated each step based 
% on the anomaly metric (zProj) using unified decreasing logic.
%
% Inputs:
%   par            - Configuration structure.
%   st             - Current SimulatorState object.
%   ddot_q_control - Nominal primary control acceleration.
%   zProj          - Normalized anomaly metric (Mahalanobis distance).
%
% Outputs:
%   ddot_q_control - Scaled control acceleration.
%   st             - Updated SimulatorState object (target_gainScaling populated).
%
% SEE ALSO: DEFENCE.UNIFIEDDECREASING, CORE_CONTROL


    % Apply scaling
    ddot_q_control = st.sys_gainScaling_q * ddot_q_control;
    
    % Update target for next step
    [st.target_gainScaling] = defence.unifiedDecreasing(zProj, par.def.active.PDgainScaling, 'pin', 'exponential');
end
