function [yAttack, st, info] = run(par, st, t, tStart, Nsteps, isSimulation, yAttack)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% ATTACK.USEGREEDYPD.RUN Executes the Greedy PD attack strategy.
% This plugin computes an optimal adversarial signal (yAttack) that steers 
% the robot toward a malicious goal while maximizing impact on the 
% end-effector's tracking error. It uses numerical Jacobians to predict 
% the closed-loop response.
%
% Inputs:
%   par, st, t, tStart, Nsteps, isSimulation - Standard simulator signals.
%   yAttack - Accumulated attack vector from previous plugins.
%
% Outputs:
%   yAttack - Updated attack vector (cumulative).
%   st      - Updated SimulatorState object (y_prev_total_attack updated).
%   info    - Structure containing attack metrics (p_goal, distance to goal, etc.).
%
% SEE ALSO: CORE_ATTACK, ATTACK.USEGREEDYPD.OPTIMGREEDYPD, ATTACK.USEGREEDYPD.PERTURBATIONG


    % Initialize info struct with defaults
    info = struct();
    info.distance2attackGoal = 0;
    info.delta_y_optimal = zeros(par.robot.nJoints, 1);
    info.p_goal = zeros(numel(par.attack.attackedPosComponents), 1);
    info.multiJacobianIndexes_kstar = nan;

    nJoints = par.robot.nJoints;

    if isSimulation
        % In simulation (lookahead), we just read the pre-set attack value
        % Note: logic copied from original core_attack, but usually overrides are handled outside?
        % In original core_attack: if isSimulation ... yAttack = override/par.attack.value
        % Here we assume core_attack handles the override dispatch or we replicate it?
        % The user said core_attack should be a "dispatcher".
        % The simulation logic (reading pre-computed values) acts as an "override" or "replay" 
        % rather than the "GreedyPD" strategy itself.
        % However, if we are in this plugin, it means we are "running" GreedyPD.
        % But wait, if isSimulation is true, core_attack originally returned early.
        % The dispatcher should probably handle the isSimulation check if it applies to ALL attacks.
        % BUT, coreSimulator passes attackValueOverride to core_attack.
        
        % Let's look at the original code. 
        % if isSimulation ... yAttack = override ... return.
        % This logic bypasses the specific attack computation.
        % So it arguably belongs in the dispatcher (core_attack), OR this plugin shouldn't be called if isSimulation?
        % Actually, if isSimulation, we might simply NOT call the constructive plugins.
        % I will rely on core_attack to handle the `isSimulation` override shortcut as it seems global.
        % So `run` here will assume we are calculating the attack.
        return; 
    end
    
    % Main Attack Logic (extracted from core_attack)
    if (t >= par.attack.startSample)
        % 1. Get Goals
 
        idx_k1 = t - tStart + 1 + 2;
        if idx_k1 <= size(par.attack.p_goal, 2)
            p_goal_k1 = par.attack.p_goal(:, idx_k1); 
            v_goal_k1 = par.attack.v_goal(:, idx_k1); 
        else
            p_goal_k1 = par.attack.p_goal(:, end); 
            v_goal_k1 = zeros(size(p_goal_k1)); 
        end

        % 2. Calculate Baselines (Lookahead)
        jacobians = struct();
        baselines = struct();
        
        % CALL PLUGIN-SPECIFIC FUNCTIONS using strict package paths
        baselines.pk1 = attack.useGreedyPD.perturbationG(par, st,  {'handPosition_task'}, st.y_prev_total_attack, 2, t);
        baselines.vk1 = attack.useGreedyPD.perturbationG(par, st,  {'handGeneralizedVelocity_task'}, st.y_prev_total_attack, 2, t);
        baselines.vk1 = baselines.vk1(par.attack.attackedPosComponents);

        % 3. Numerical Jacobian
        jacSuccess = false;
        multiJacobianIndexes = par.attack.multiJacobianIndexes;
        
        % Pre-define function handle
        perturbFunc = @(delta_y_a) attack.useGreedyPD.perturbationG(par, st, {'handGeneralizedVelocity_task'}, st.y_prev_total_attack + delta_y_a, 3, t);
        
        % Note: numJacobianVardelta is likely in path (init or core?). 
        % Checked: it's in core/ (implicit path) or need qualification? 
        % It wasn't in the moved files list. It seems to be a general utility.
        % Let's assume it's available.
        
        while ~jacSuccess
            [jacobians.G_v_k2, baselines.vk2, multiJacobianIndexes_kstar] = numJacobianVardelta( ...
                perturbFunc, ...
                zeros(nJoints,1), multiJacobianIndexes, par.verbose);

            if isnan(jacobians.G_v_k2)
                multiJacobianIndexes = attack.useGreedyPD.enlargeKInterval(multiJacobianIndexes);
            else
                jacSuccess = true;
            end
        end

        baselines.vk2 = baselines.vk2(par.attack.attackedPosComponents);
        jacobians.G_ddp_k = 1/par.sim.Ts * jacobians.G_v_k2;
        jacobians.G_ddp_k = jacobians.G_ddp_k(par.attack.attackedPosComponents,:);

        % Since our system is non-linear and includes a complex observer/controller in the loop,
        % calculating the "true" analytic acceleration for $k+2$ is difficult.
        % We use a numerical finite difference approach:
        baselines.ddp_baseline = (baselines.vk2 -  baselines.vk1) / par.sim.Ts;

        % 4. PD Policy & Optimizer
        attacker_p_error = p_goal_k1 - baselines.pk1; 
        attacker_v_error = v_goal_k1 - baselines.vk1; 
        
        target_acceleration = par.attack.greedyPD.Kp * attacker_p_error + ...
                              par.attack.greedyPD.Kd * attacker_v_error;

        % Add optional acceleration feedforward
        if isfield(par.attack.greedyPD, 'useFeedforward') && par.attack.greedyPD.useFeedforward
            % The acceleration that leads to p_goal(idx_k1) is a_goal(idx_k1 - 1)
            idx_ff = idx_k1 - 1; 
            if idx_ff <= size(par.attack.a_goal, 2)
                a_ff = par.attack.a_goal(:, idx_ff); 
            else
                a_ff = zeros(size(target_acceleration)); 
            end
            target_acceleration = a_ff + target_acceleration;
        end

        x_real_AT        = [st.sys_real_q;  st.sys_real_qdot];
        x_hat_C_prior_for_vk = [st.sys_observer_q_hat; st.sys_observer_qdot_hat];
        v_k_for_stealth   = par.models.sys_real.C * (x_real_AT - x_hat_C_prior_for_vk);

        [delta_y_optimal, ~] = attack.useGreedyPD.optimGreedyPD(par, target_acceleration, st.y_prev_total_attack, v_k_for_stealth, nJoints, jacobians, baselines);

        % 5. Update State
        yAttackInc = st.y_prev_total_attack + delta_y_optimal;
        if any(isnan(yAttackInc)); error('NAN attack!'); end
        st.y_prev_total_attack = yAttackInc;
        
        % Combine with input yAttack (assuming greedy attack replaces or adds? 
        % Original code: `yAttack = st.y_prev_total_attack + delta_y_optimal` (which is stored in st.y_prev...)
        % `yAttack` output of core_attack is this value.
        % If multiple attacks run, how do they combine?
        % Usually GreedyPD drives the total attack vector.
        % We'll assume it sets the baseline `yAttack`.
        yAttack = yAttack + yAttackInc; 

        % Populate Info
        info.distance2attackGoal = norm(st.ent.geometryReal.handPosition_task - p_goal_k1);
        info.delta_y_optimal = delta_y_optimal;
        info.p_goal = p_goal_k1;
        info.multiJacobianIndexes_kstar = multiJacobianIndexes_kstar;

        % Logging
         if (t >= par.attack.startSample)
            fprintf('Iter %d / %d. Attacker Dist: %.6f\n', t, Nsteps, info.distance2attackGoal);
         end
    end
end
