function [s, info] = computeInitialState(par)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% COMPUTEINITIALSTATE Computes the initial state structure for the simulator.
% This function determines which state variables are required based on active 
% feature flags (defence, attack, smoothing) and populates their initial values.
%
% Inputs:
%   par  - Simulation configuration structure.
%
% Outputs:
%   s    - Struct containing initial values for selected SimulatorState properties.
%   info - Struct containing human-readable descriptions for each state field.
%
% SEE ALSO: SIMULATORSTATE, CORE_INIT, EXPPARMS

%

    s = struct();
    info = struct();

    % Feature flags (read once for clarity)
    usePDgainsScaling      = par.def.active.scheme.usePDgainsScaling;
    useActiveDissipation   = par.def.active.scheme.useActiveDissipation;

    %% --- Always present: core robot state ---

    % Real feedback linearized system (state of the robot)
    s.sys_real_q        = par.task.q0;
    info.sys_real_q     = "Real robot joint positions (nx1) [rad]";

    s.sys_real_qdot     = pinv(par.funcs.J_func_task(par.task.q0)) * par.desired.velocity(par.task.taskSpaceRows, 1);
    info.sys_real_qdot  = "Real robot joint velocities (nx1) [rad/s]";

    % System observer
    s.sys_observer_q_hat        = s.sys_real_q;
    info.sys_observer_q_hat     = "Observer estimated joint positions (nx1) [rad]";

    s.sys_observer_qdot_hat     = s.sys_real_qdot;
    info.sys_observer_qdot_hat  = "Observer estimated joint velocities (nx1) [rad/s]";

    % Simulated state (Sensing-Free Projected)
    s.sys_sensingFreeProjected_q        = s.sys_observer_q_hat;
    info.sys_sensingFreeProjected_q     = "Sensing-free projected state joint positions (nx1) [rad]";

    s.sys_sensingFreeProjected_qdot     = s.sys_observer_qdot_hat;
    info.sys_sensingFreeProjected_qdot  = "Sensing-free projected state joint velocities (nx1) [rad/s]";

    % Previous total attack signal (used by core_attack regardless of scheme)
    s.y_prev_total_attack       = zeros(par.robot.nJoints, 1);
    info.y_prev_total_attack    = "Previous total attack signal (nx1)";

    % Projected state covariance (always used in core_control for zProj)
    s.current_Pz        = par.def.scheme.propagator.Pz0;
    info.current_Pz     = "Current projected state covariance matrix";

    %% --- Gain Scaling smoother: needed by PDScaling or ActiveDissipation ---
    % sys_gainScaling_q drives gamma in activeDissipation and scales ddot_q in PDScaling.

    if usePDgainsScaling || useActiveDissipation
        s.sys_gainScaling_q         = 1; % Start at full gain (not 0)
        info.sys_gainScaling_q      = "State for Gain Scaling smoother (position)";
        s.sys_gainScaling_qdot      = 0;
        info.sys_gainScaling_qdot   = "State for Gain Scaling smoother (velocity)";
        s.target_gainScaling        = 1;
        info.target_gainScaling     = "Target gain for Gain Scaling smoother";
    end



    %% --- ActiveDissipation: smooth damping state ---

    if useActiveDissipation
        s.sys_smootDamp_q       = zeros(par.robot.nJoints, 1);
        info.sys_smootDamp_q    = "State for Smoothing active dissipation (position) (nx1)";
        s.sys_smootDamp_qdot    = zeros(par.robot.nJoints, 1);
        info.sys_smootDamp_qdot = "State for Smoothing active dissipation (velocity) (nx1)";
        s.target_smootDamp      = 0;
        info.target_smootDamp   = "Target value for Smoothing active dissipation";
    end

end
