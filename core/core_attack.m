function [yAttack, distance2attackGoal, delta_y_optimal, p_goal, st, multiJacobianIndexes_kstar] = core_attack(par, st, t, tStart, Nsteps, isSimulation, attackValueOverride)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% CORE_ATTACK Dispatcher for attack plugins.
% This function dispatches execution to enabled attack plugins found in 
% par.attack.scheme. It handles global lookahead overrides used during 
% internal simulations (e.g., optimization).
%
% Inputs:
%   par                  - Configuration structure.
%   st                   - Current SimulatorState object.
%   t                    - Current simulation time step.
%   tStart               - Global start time of the simulation.
%   Nsteps               - Number of simulation steps.
%   isSimulation         - Boolean flag for lookahead simulations.
%   attackValueOverride  - (Optional) matrix of pre-computed attack values.
%
% Outputs:
%   yAttack              - Computed attack vector at time t.
%   distance2attackGoal  - Scalar distance to the attacker's target.
%   delta_y_optimal      - Optimal attack increment (for greedy strategies).
%   p_goal               - Current attacker target position.
%   st                   - Updated SimulatorState object (if modified by plugin).
%   multiJacobianIndexes_kstar - Index of the selected Jacobian configuration.
%
% SEE ALSO: CORE_CONTROL, CORESIMULATOR


    % Defaults for outputs
    nJoints = par.robot.nJoints;
    yAttack = zeros(nJoints, 1);
    distance2attackGoal = 0;
    delta_y_optimal = zeros(nJoints, 1);
    p_goal = zeros(numel(par.attack.attackedPosComponents), 1);
    multiJacobianIndexes_kstar = nan;

    % 1. Simulation Override (Global Lookahead Logic)
    if isSimulation
        % In lookahead, we rely on pre-computed or overridden attack values
        if t-tStart+1 > 3
             error('core_attack:InvalidLookahead', 'A simulation has a maximum step of size 3');
        else
            if nargin >= 7 && ~isempty(attackValueOverride)
                yAttack = attackValueOverride(:,t-tStart+1); 
            else
                yAttack = par.attack.value(:,t-tStart+1); 
            end
        end
        return; 
    end
    
    % 2. Dispatch to Plugins
    if isempty(coder.target)
         % Execute pre-validated plugin handles (stored in par.attack.activePlugins at init)
         for i = 1:numel(par.attack.activePlugins)
             runFunc = par.attack.activePlugins{i};
             
             % Call the plugin
             % Signature: [yAttack, st, info, ...] = runFunc(...)
             % Signature varies by plugin, we can extract it or pass everything
             try
                 [yAttack, st, info] = runFunc(par, st, t, tStart, Nsteps, isSimulation, yAttack);
             catch
                 % for useNoise which now has a smaller signature
                 [yAttack, st, info] = runFunc(par, st, isSimulation, yAttack);
             end
             
             % Merge standard metrics from plugin info into outputs
             if isfield(info, 'distance2attackGoal'); distance2attackGoal = info.distance2attackGoal; end
             if isfield(info, 'delta_y_optimal'); delta_y_optimal = info.delta_y_optimal; end
             if isfield(info, 'p_goal'); p_goal = info.p_goal; end
             if isfield(info, 'multiJacobianIndexes_kstar'); multiJacobianIndexes_kstar = info.multiJacobianIndexes_kstar; end
         end
         
    else
        error('Invalid execution path: core_attack plugins not supported in codegen target yet');
    end

end
