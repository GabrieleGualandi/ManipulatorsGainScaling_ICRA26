function [x, solved, result] = solveQCQP(model, params, optim_par)
% core.optim.gurobi.solveQCQP: Helper to call Gurobi for a QCQP problem.
% Encapsulates Gurobi call, result status handling, and regularization loops.
%
% Inputs:
%   model     - Gurobi model struct (Q, obj, A, rhs, quadcon, lb, ub, etc.)
%   params    - Gurobi params struct (OutputFlag, etc.)
%   optim_par - Struct with optimization settings (max_obj_reg_attempts, epsilon_reg_Qobj, etc.)
%
% Outputs:
%   x       - Optimal solution vector (zeros if failed)
%   solved  - Boolean indicating success
%   result  - Full result struct from Gurobi

    x = zeros(size(model.obj));
    solved = false;
    result = struct('status', 'GUROBI_NOT_FOUND');
    
    % Check if Gurobi is visible in the MATLAB path
    if exist('gurobi', 'file') ~= 3 && exist('gurobi', 'file') ~= 2
        warning('core:optim:GurobiNotFound', ...
            'Gurobi solver was not found on the MATLAB path.\n' + ...
            'Please ensure Gurobi is installed and its MATLAB interface is added to your path (e.g., via gurobi_setup).\n' + ...
            'Skipping optimization.');
        return;
    end
    done = 0;
    wasObjRegularized = 0;
    max_obj_reg_attempts = 1;
    if isfield(optim_par, 'max_obj_reg_attempts'); max_obj_reg_attempts = optim_par.max_obj_reg_attempts; end
    attempt_count = 0;
    
    current_Q_obj_for_solver = model.Q; 

    while ~done && attempt_count <= max_obj_reg_attempts
        try
            result = gurobi(model, params);
            switch result.status
                case 'OPTIMAL'
                    x = result.x;
                    solved = true;
                    done = 1;
                case 'INFEASIBLE'
                    fprintf('[solveQCQP] Gurobi: Problem INFEASIBLE.\n');
                    done = 1;
                case {'NUMERIC', 'SUBOPTIMAL'} 
                     warning('[solveQCQP] Gurobi: Numerical issues (status: %s).\n', result.status);
                     if ~wasObjRegularized && attempt_count < max_obj_reg_attempts
                         fprintf('[solveQCQP] Attempting to regularize Q_obj.\n');
                         Q_obj_sym_check = 0.5 * (current_Q_obj_for_solver + current_Q_obj_for_solver');
                         epsilon_reg_Qobj = 1e-8;
                         if isfield(optim_par, 'epsilon_reg_Qobj'); epsilon_reg_Qobj = optim_par.epsilon_reg_Qobj; end
                         
                         min_eig_Qobj = min(eig(full(Q_obj_sym_check))); 
                         shift_val = max(0, epsilon_reg_Qobj - min_eig_Qobj);

                         if shift_val > 0
                            model.Q = sparse(Q_obj_sym_check + shift_val * speye(size(Q_obj_sym_check)));
                            fprintf('[solveQCQP] Regularized model.Q with shift = %e\n', shift_val);
                         else
                            fprintf('[solveQCQP] model.Q already sufficiently PSD, but solver failed numerically.\n');
                            done = 1; 
                         end
                         wasObjRegularized = 1;
                    else
                        fprintf('[solveQCQP] Failed after regularization or max attempts.\n');
                        done = 1;
                    end
                otherwise
                    fprintf('[solveQCQP] Gurobi: Solver failed (%s).\n', result.status);
                    done = 1;
            end
        catch ME
            if strcmp(ME.identifier, 'MATLAB:UndefinedFunction') && contains(ME.message, 'gurobi')
                fprintf('[solveQCQP] CRITICAL ERROR: Gurobi function is undefined. This usually means Gurobi is not on the MATLAB path.\n');
            else
                fprintf('[solveQCQP] ERROR during Gurobi call: %s\n', ME.message);
            end
            done = 1;
        end
        attempt_count = attempt_count + 1;
    end
end
