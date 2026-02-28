function [st, log] = core_init(par, stIn, Nsteps, self)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORE_INIT Initialize simulation state and logging structures.
% This function prepares the SimulatorState object 'st' and pre-allocates
% the 'log' structure for data recording. It handles memory initialization
% for standard and optional logged variables based on feature flags.
%
% Inputs:
%   par      - Simulation configuration structure.
%   stIn     - Initial state data (SimulatorState or struct).
%   Nsteps   - Number of simulation steps to pre-allocate.
%   self     - Lookahead configuration structure (isSimulation, isJacobian, etc.).
%
% Outputs:
%   st       - Initialized SimulatorState object.
%   log      - Memory-allocated logging structure (prefilled with NaNs).
%
% SEE ALSO: CORE_LOG, SIMULATORSTATE, COMPUTEINITIALSTATE


    % Flags
    isSimulation = self.isSimulation;
    isJacobian = false;
    if isfield(self,'isJacobian'); isJacobian = self.isJacobian; end
    
    % Initialize state structure 'st'
    if isa(stIn, 'SimulatorState')
        st = stIn;
    else
        st = struct(); 
        
        % Core system states
        st.sys_real_q           = stIn.sys_real_q;
        st.sys_real_qdot        = stIn.sys_real_qdot;
        st.sys_observer_q_hat   = stIn.sys_observer_q_hat;
        st.sys_observer_qdot_hat= stIn.sys_observer_qdot_hat;
        st.sys_sensingFreeProjected_q            = stIn.sys_sensingFreeProjected_q;
        st.sys_sensingFreeProjected_qdot         = stIn.sys_sensingFreeProjected_qdot;
        st.current_Pz           = stIn.current_Pz;
        
        
        % Attack history
        st.y_prev_total_attack = stIn.y_prev_total_attack;
    end
    
    % Get Task Dimension
    nTask = numel(par.task.taskSpaceRows); 

    %% Log Allocation
    log = struct();
    
    if isJacobian
        % Minimal logging for Jacobian (prevent dynamic resizing)
        log.handPosition_task = nan(numel(par.task.taskP), Nsteps); 
        log.handGeneralizedVelocity_task = nan(nTask, Nsteps);
    elseif ~isSimulation
        % Preallocate arrays for logging (using NaN)
        nJoints = par.robot.nJoints;
        
        log.sys_real_q              = nan(size(stIn.sys_real_q, 1), Nsteps);
        log.sys_real_qdot           = nan(size(stIn.sys_real_qdot, 1), Nsteps);
        log.sys_observer_q_hat      = nan(size(stIn.sys_observer_q_hat, 1), Nsteps);
        log.sys_observer_qdot_hat   = nan(size(stIn.sys_observer_qdot_hat, 1), Nsteps);
        log.sys_sensingFreeProjected_q               = nan(size(stIn.sys_sensingFreeProjected_q, 1), Nsteps);
        log.sys_sensingFreeProjected_qdot            = nan(size(stIn.sys_sensingFreeProjected_qdot, 1), Nsteps);

        if isfield(par.def.active.scheme, 'usePDgainsScaling') && par.def.active.scheme.usePDgainsScaling
            log.gainScaling_q = nan(size(stIn.sys_gainScaling_q, 1), Nsteps);
        end

        log.y_prev_total_attack = nan(size(stIn.y_prev_total_attack, 1), Nsteps);
        log.handOrientation             = nan(9, Nsteps);
        % Optional Logs via par.logging
        % We iterate through known optional variables and check if they are enabled in par.logging
        if isfield(par, 'logging')
            lev = par.logging;
            
            if isfield(lev, 'multiJacobianIndexes_kstar') && lev.multiJacobianIndexes_kstar
                log.multiJacobianIndexes_kstar = nan(1, Nsteps);
            end
            if isfield(lev, 'zProj') && lev.zProj
                log.zProj = nan(1, Nsteps);
            end
            if isfield(lev, 'ddot_q_control') && lev.ddot_q_control
                log.ddot_q_control = nan(nJoints, Nsteps);
            end
        end
        % Preallocate remaining fields
        log.handPosition_task       = nan(numel(par.task.taskP), Nsteps);
        log.handGeneralizedVelocity_task = nan(nTask, Nsteps);
        log.ddot_q_final            = nan(nJoints, Nsteps);
        log.ddot_q_final_unsaturated = nan(nJoints, Nsteps);
        
        % Preallocate error fields
        % Note: Dimensions depend on task space size. 
        % numel(par.task.taskP) + numel(par.task.taskO (max 3?))
        nPos = numel(par.task.taskP);
        nOri = numel(par.task.taskO); % taskO contains indices, so numel is correct count
        nPoseError = nPos + nOri;
        nVelError = numel(par.task.taskSpaceRows); 
        
        log.error_real_pose = nan(nPoseError, Nsteps);
        log.error_real_vel  = nan(nVelError, Nsteps);
        log.error_estimated_pose = nan(nPoseError, Nsteps);
        log.error_estimated_vel  = nan(nVelError, Nsteps);
        log.error_sensingFreeProjected_pose = nan(nPoseError, Nsteps);
        log.error_sensingFreeProjected_vel  = nan(nVelError, Nsteps);

        % Setup logging flags to avoid isfield in the loop
        log.logFlags = struct();
        lev = par.logging;
        log.logFlags.multiJacobianIndexes_kstar = isfield(lev, 'multiJacobianIndexes_kstar') && lev.multiJacobianIndexes_kstar;
        log.logFlags.zProj = isfield(lev, 'zProj') && lev.zProj;
        log.logFlags.ddot_q_control = isfield(lev, 'ddot_q_control') && lev.ddot_q_control;
    end
end
