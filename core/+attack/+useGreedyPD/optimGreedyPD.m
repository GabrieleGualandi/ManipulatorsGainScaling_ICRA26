function [delta_y_optimal, solved_successfully, verification_metrics] = ...
    optimGreedyPD(par, target_acceleration, y_prev_total_attack, v_k_for_stealth, numSensors, jacobians_accel, baselines_accel)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% OPTIMGREEDYPD Solves for an incremental attack delta_y_k^a to match target acceleration.
% This function solves a Quadratic Constrained Quadratic Programming (QCQP) problem
% to find the optimal adversarial increment that steers the robot toward the 
% attacker's goal while remaining stealthy against a passive detector.
%
% Inputs:
%   par                 - System and attack parameters structure.
%   target_acceleration - Desired average hand acceleration vector.
%   y_prev_total_attack - Total accumulated attack vector from the previous step.
%   v_k_for_stealth     - Innovation residual (innovation) used for the stealth constraint.
%   numSensors          - Number of sensor channels available for attack (dy).
%   jacobians_accel     - Struct containing the numerical Jacobian G_ddp_k.
%   baselines_accel     - Struct containing the baseline acceleration ddp_baseline.
%
% Outputs:
%   delta_y_optimal      - Optimal incremental attack vector for the current step.
%   solved_successfully  - Boolean flag indicating if the optimization converged.
%   verification_metrics - Struct containing expected impact and stealth budget diagnostics.
%
% SEE ALSO: ATTACK.USEGREEDYPD.RUN, OPTIM.GUROBI.SOLVEQCQP

    dy = numSensors;
    Sigma_r_inv       = par.attack.Sigma_prime; 

    G_ddp_k = jacobians_accel.G_ddp_k; 
    ddp_baseline = baselines_accel.ddp_baseline(par.attack.attackedPosComponents); 
    delta_ddp_target = target_acceleration - ddp_baseline;

    Q_regularization = par.attack.greedyPD.lambda_regularization * eye(dy);
    Q_obj_k = (G_ddp_k' * G_ddp_k) + Q_regularization;
    f_obj_k = -G_ddp_k' * delta_ddp_target;

    %--- Build Gurobi model struct ---
    model              = struct();
    model.modelsense   = 'min';

    if dy > 0
        if isfield(par.attack, 'use_delta_y_max_abs_constraint') && par.attack.use_delta_y_max_abs_constraint
            model.lb = -par.attack.delta_y_max_abs_constraint * ones(dy,1); 
            model.ub =  par.attack.delta_y_max_abs_constraint * ones(dy,1);
        else
            model.lb = -inf(dy,1);
            model.ub =  inf(dy,1);
        end
    end
    
    model.Q   = sparse(Q_obj_k); 
    model.obj = f_obj_k;         

    % Quadratic (stealth) constraint
    if par.attack.use_stealth_constraint
        tau_prime       = par.attack.tau_prime;
        c_offset        = y_prev_total_attack + v_k_for_stealth;
        Q_c             = Sigma_r_inv;
        f_c             = 2 * Sigma_r_inv * c_offset;
        s_c             = c_offset' * Sigma_r_inv * c_offset - tau_prime;
        Q_c_sym         = 0.5 * (Q_c + Q_c'); 

        qc_stealth = struct();
        qc_stealth.Qc    = sparse(Q_c_sym);
        qc_stealth.q     = f_c'; 
        qc_stealth.rhs   = -s_c; 
        qc_stealth.sense = '<';
        qc_stealth.name  = 'Stealth_Residual';
        model.quadcon = qc_stealth;
    end
    
    model.A     = sparse(0, dy);
    model.rhs   = [];
    model.sense = '';

    %--- Solver Options ---
    solver_params = struct();
    solver_params.OutputFlag = 0; 
    if isfield(par.optim, 'gurobi')
        gurobi_params_fields = fieldnames(par.optim.gurobi);
        for i = 1:length(gurobi_params_fields)
            solver_params.(gurobi_params_fields{i}) = par.optim.gurobi.(gurobi_params_fields{i});
        end
        if isfield(par.sim, 'verboseLevel') && par.sim.verboseLevel < 2 
             solver_params.OutputFlag = 0;
        elseif ~isfield(solver_params, 'OutputFlag') 
             solver_params.OutputFlag = 0;
        end
    else 
        warning('optimGreedyPD:DefaultParams', 'Setting default gurobi parameters');
        solver_params.NumericFocus = 0;
        solver_params.FeasibilityTol = 1e-6;
        solver_params.OptimalityTol = 1e-6;
    end

    %--- Solve with Decoupled Solver ---
    [delta_y_optimal, solved_successfully, ~] = optim.gurobi.solveQCQP(model, solver_params, par.optim);
    
    if ~solved_successfully
         fprintf('[optimGreedyPD] Optimization failed. Returning zero delta_y.\n');
    end

    if nargout > 2
        % --- Optional Populate Verification Metrics ---
        verification_metrics = struct();
        verification_metrics.delta_y_optimal = delta_y_optimal;
        
        % Expected Anomaly Score (Stealth Constraint Check)
        if par.attack.use_stealth_constraint
            total_attack_k_optimal = y_prev_total_attack + delta_y_optimal;
            residual_predicted_optimal = total_attack_k_optimal + v_k_for_stealth;
            verification_metrics.expected_anomaly_score = residual_predicted_optimal' * Sigma_r_inv * residual_predicted_optimal;
            verification_metrics.stealth_budget_tau_prime = tau_prime;
        end
    
        % Expected achieved average acceleration
        predicted_ddp_avg_achieved = ddp_baseline + G_ddp_k * delta_y_optimal;
        verification_metrics.predicted_ddp_avg_achieved = predicted_ddp_avg_achieved;
        verification_metrics.target_acceleration = target_acceleration;
        verification_metrics.error_to_target_acceleration_norm_sq = norm(target_acceleration - predicted_ddp_avg_achieved)^2;
    end
end
