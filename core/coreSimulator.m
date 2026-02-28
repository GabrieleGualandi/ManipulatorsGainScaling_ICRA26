function [log] = coreSimulator(par, stIn, Nsteps, self)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORESIMULATOR Main simulation loop for the Secure Robot Control System.
% This function serves as the central orchestrator for simulating the robot's
% dynamics, control systems (nominal and active defence), attack scenarios,
% and state estimation. It iterates through time steps, invoking specialized
% sub-functions for each phase of the control loop.
%
% Inputs:
%   par     - Struct containing all simulation parameters.
%   stIn    - Struct containing the initial state of the system.
%   Nsteps  - Number of simulation steps to execute.
%   self    - Struct for recursion/lookahead management. Key fields:
%             - isSimulation (bool): If true, indicates an internal lookahead simulation.
%             - isJacobian (bool):   If true, indicates the simulation is for numerical Jacobian calculation.
%
% Outputs:
%   log     - Struct containing time-series data of all simulated variables.
%
% SEE ALSO: CORE_INIT, NOMINAL.GETDESIRED, CORE_CONTROL, CORE_ATTACK, NOMINAL.SENSE, CORE_UPDATESTATE, CORE_LOG

%
% Workflow:
%   1. Initialization (core_init)
%   2. Main Loop (t = tStart to tStart+Nsteps-1):
%      a. State Update: Propagate 'next' states to 'current' states (updateCurrentFromNext).
%      b. Trajectory Retrieval: Get desired position/velocity/acceleration (nominal.getDesired).
%      c. Control Calculation: Compute nominal PD, Manipulability Shape, and Active Dissipation control laws (core_control).
%      d. Attack Generation: Simulate attacker behavior and compute attack vector (core_attack).
%      e. Sensing & Detection: Simulate sensor measurements, add attack/noise, and run Anomaly Detection System (nominal.sense).
%      f. Dynamics Propagation: Evolve system state using computed controls and measurements (core_updateState).
%      g. Logging: Record variables for post-processing (core_log).

    %% 0. Initialization & Configuration
    % specific configurations for recursive calls (e.g., lookahead for attack optimization)
    if isfield(self,'tNow') 
        tStart = self.tNow;
    else
        tStart = 1;
    end
    
    isSimulation = self.isSimulation;
    isJacobian = false;
    if isfield(self,'isJacobian'); isJacobian = self.isJacobian; end
    hasAlarmFired = false;

    
    % Initialize Simulation State (st) and Logging Structure (log)
    [st, log] = core_init(par, stIn, Nsteps, self);

    % Pre-populate 'next' states in 'st' with initial values to bootstrap the loop.
    % The loop pattern updates 'current' from 'next', so 'next' must be valid at t=1.
    st.next = st.toStruct();
    
    
    %% MAIN SIMULATION LOOP
    for t = tStart : tStart+Nsteps - 1 
        
        %% 1. State Transition
        st.update();

        %% 2. Trajectory Generation
        desired = nominal.getDesired(par, t);
        
        %% 3. Control Calculation
        [st, ctrlData] = core_control(par, st, desired);

        %% 4. Attack Simulation
        % Compute the attack vector (yAttack) based on the attacker's strategy (e.g., Greedy PD).
        % Also computes optimal attack increments and lookahead metrics.
        
        attackValueOverride = [];
        if isfield(self, 'attackValue'); attackValueOverride = self.attackValue; end
        
        [yAttack, distance2attackGoal, delta_y_optimal, p_goal, st, multiJacobianIndexes_kstar] = ...
            core_attack(par, st, t, tStart, Nsteps, isSimulation, attackValueOverride);
        
        %% 5. Sensing & Detection
        % 5a. Measurement (Sensor Simulation)
        [y_tilde] = nominal.sense(par, st, yAttack);
        
        % 5b. Passive ADS (Constraints Check, Stealthiness)
        [ADS_z, ADS_r, hasAlarmFired] = defence.ADS_passive(par, st, y_tilde, isSimulation, hasAlarmFired);
        

        
        %% 6. System Dynamics Propagation
        % Evolve the system state (Real, Observer, Paranoid) to the next time step (t+1).
        % These values are stored in 'st.next' and will be applied at the start of the next iteration.
        st = core_updateState(par, st, st.ent.ddot_q_final, y_tilde, isSimulation, isJacobian);

        %% 7. Data Logging
        % Record simulation data into the 'log' structure. 
        % Logic:
        % - Generally skipped for lookahead simulations (isSimulation=true) to save performance.
        % - However, if computing Jacobian (isJacobian=true), we MUST log minimal data (e.g., hand pos/vel) 
        %   needed by the perturbation function (attack.perturbationG), even during lookahead.
        if ~isSimulation || isJacobian
            if isJacobian
                logData = struct(); % Minimal logging doesn't use external data
             else
                % Pack control and attack data for the logger
                logData = ctrlData; 
                logData.yAttack = yAttack;
                logData.distance2attackGoal = distance2attackGoal;
                logData.delta_y_optimal = delta_y_optimal;
                logData.p_goal = p_goal;
                logData.ADS_z = ADS_z;
                logData.ADS_r = ADS_r;
                logData.multiJacobianIndexes_kstar = multiJacobianIndexes_kstar;
             end

            log = core_log(par, log, st, t, tStart, isSimulation, isJacobian, logData);
        end
        
        % Reset geometric entities to clear memory for the next iteration
        st.ent = []; 
    end

    % Clean up internal logging flags before returning
    if ~isSimulation && isfield(log, 'logFlags')
        log = rmfield(log, 'logFlags');
    end
end
