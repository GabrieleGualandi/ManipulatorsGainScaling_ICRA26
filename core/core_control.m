function [st, ctrlData] = core_control(par, st, desired)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORE_CONTROL Control Orchestrator for nominal control law.
% This function calculates the total control torque/acceleration by combining:
% - Nominal PD control with secondary task projections.
%
% Inputs:
%   par          - Configuration structure.
%   st           - Current SimulatorState object.
%   desired      - Reference trajectory structure. cost data).
%   ctrlData     - Structure containing intermediate control signals for logging.
%
% SEE ALSO: NOMINAL.CONTROL, DEFENCE.MANIPULABILITYSHAPE, DEFENCE.PDSCALING, DEFENCE.ACTIVEDISSIPATION


    nJoints = par.robot.nJoints;
    
    %% 1. Geometric Entities
    st.ent = nominal.calcEntities(par, st, desired);

    %% 2. Primary Task Pseudoinverse
    inverseJ_primary = st.ent.geometryEstimated.pseudoinverse_task;

    %% 3. Full State Residual & Covariance (Projections)
    % Compute the residual between the estimated state and the sensing-free projected state.
    x_est = [st.ent.geometryEstimated.q;st.ent.geometryEstimated.qdot];
    x_proj = [st.ent.geometrySensingFreeProjected.q;st.ent.geometrySensingFreeProjected.qdot];
    rProj_full = x_est - x_proj;

    indexHalf = (nJoints*2)+1;
    current_Pr = st.current_Pz( indexHalf : end, indexHalf : end );
    covRproj_inv = invRobust(current_Pr,'iterative');
    zProj = rProj_full' * covRproj_inv * rProj_full;

    %% 4. Nominal PD Control
    [ddot_q_control, qd_nullSpace] = nominal.control(par, st, desired, inverseJ_primary);

    %% 5. Active PD Scaling
    if isfield(par.def.active.scheme, 'usePDgainsScaling') && par.def.active.scheme.usePDgainsScaling
        [ddot_q_control, st] = defence.PDScaling(par, st, ddot_q_control, zProj);
    end

    %% 4. Saturation
    % Smooth saturation to enable Jacobian calculation
    [st.ent.ddot_q_final, st.ent.hasSaturated, st.ent.saturExcess] = saturateVector(ddot_q_control, par.robot.qdd_max, 'inf-smoothstepDifferentiable',50); 
    
    %% 7. Pack data for logging
    ctrlData.ddot_q_control = ddot_q_control;
    ctrlData.ddot_without_damping_sat = ddot_q_control;
    ctrlData.ddot_q_final_unsaturated = ddot_q_control;
    ctrlData.qd_nullSpace = qd_nullSpace;
    ctrlData.zProj = zProj;
end
