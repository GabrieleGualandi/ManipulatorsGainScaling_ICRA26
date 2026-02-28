function [x, status, final_A] = robustInverseSolve(A, b, method, options)
%ROBUSTINVERSESOLVE Solves A*x = b robustly, especially for ill-conditioned A.
%
%   [x, status, final_A] = robustInverseSolve(A, b, method, options)
%
%   Inputs:
%       A       - The system matrix (square).
%       b       - The right-hand side vector.
%       method  - Solution method:
%                 'lambdaInc': Start with small damping (lambda*eye) and
%                              increase lambda until rcond is acceptable
%                              or MATLAB's '\' stops warning.
%                 'SVD':       Use Singular Value Decomposition to compute
%                              a pseudoinverse, thresholding small singular
%                              values. Damping is applied *before* SVD if
%                              options.initial_lambda > 0.
%                 'iterative': Use MINRES iterative solver. ***Requires A + lambda*I
%                              to be symmetric.*** Damping is applied *before* solving.
%       options - (Optional) Struct with method-specific parameters:
%                 For 'lambdaInc':
%                   .initial_lambda (default: 1e-9) - Starting damping factor.
%                   .lambda_factor  (default: 10)   - Multiplicative factor to increase lambda.
%                   .max_lambda     (default: 1e-2) - Maximum allowed lambda.
%                   .max_tries      (default: 8)    - Max attempts with increasing lambda.
%                   .rcond_thresh   (default: 1e-14)- RCOND threshold below which matrix
%                                                     is considered ill-conditioned.
%                 For 'SVD':
%                   .initial_lambda (default: 1e-9) - Damping factor added before SVD.
%                   .svd_tol_mode   (default: 'relative') - 'relative' or 'absolute'.
%                   .svd_tol        (default: eps) - Tolerance for thresholding singular values.
%                                                    If 'relative', tol is max(size(A))*eps(max(s))*svd_tol.
%                                                    If 'absolute', tol is svd_tol.
%                 For 'iterative':
%                   .initial_lambda (default: 1e-9) - Damping factor added before solve.
%                   .iter_tol       (default: 1e-10) - Tolerance for MINRES.
%                   .max_iter       (default: size(A,1)) - Max iterations for MINRES.
%                   .symmetry_tol   (default: 1e-12) - Absolute tolerance for checking matrix symmetry
%                                                      using norm(A - A', 'fro').
%
%   Outputs:
%       x       - The solution vector. NaN if method failed before erroring.
%       status  - Struct containing information about the solution process:
%                 % ... (rest of outputs documentation unchanged) ...
%       final_A - The matrix actually used for the final solve step (A + lambda*I).
%
%   Example Usage (symmetric case for iterative):
%       J = randn(6, 7);
%       JJt = J * J'; % JJt is symmetric
%       d = randn(6, 1);
%       opts_iter = struct('initial_lambda', 1e-8);
%       [x_iter, status_iter, A_iter] = robustInverseSolve(JJt, d, 'iterative', opts_iter);
%       disp(status_iter);
%
%   Example Usage (non-symmetric case for iterative - WILL ERROR):
%       A_nonsymm = [1 2; 3 4];
%       b_nonsymm = [1; 1];
%       try
%           [x_err, status_err, A_err] = robustInverseSolve(A_nonsymm, b_nonsymm, 'iterative');
%       catch ME
%           fprintf('Caught expected error: %s\n', ME.message);
%       end

% --- Input Validation and Option Defaults ---
narginchk(3, 4);
if ~ismatrix(A) || size(A, 1) ~= size(A, 2)
    % Use the identifier expected by the test
    error('MATLAB:expectedSquare', 'Matrix A must be square.');
end
if ~isvector(b) || size(A, 1) ~= length(b)
     % Use the identifier expected by the test
    error('MATLAB:dimagree', 'Dimensions of A and b are incompatible.');
end
m = size(A, 1);
b = b(:); % Ensure b is a column vector

% Default options (can be overridden by user input)
default_opts = struct(...
    'initial_lambda', 1e-12, ...
    'lambda_factor', 10, ...
    'max_lambda', 1e-2, ...
    'max_tries', 20, ...
    'rcond_thresh', 1e-14, ...
    'svd_tol_mode', 'relative', ...
    'svd_tol', eps, ...
    'iter_tol', 1e-5, ... % 1e-10,
    'max_iter', 100, ...
    'symmetry_tol', 1e-12 ... % Tolerance for manual symmetry check
);

% Merge user options with defaults
if ~isempty(fieldnames(options))
    fields = fieldnames(default_opts);
    for i = 1:length(fields)
        if ~isfield(options, fields{i})
            options.(fields{i}) = default_opts.(fields{i});
        end
    end
else
    options = default_opts;
end

% Initialize outputs
x = nan(m, 1);
status = struct('method', method, 'success', false, 'message', 'Method not executed');
final_A = A; % Start with original A

% --- Method Selection ---
Id = eye(m); % Identity matrix

switch lower(method)
    case 'lambdainc'

        lambda = options.initial_lambda;
        status.iterations = 0;
        status.final_lambda = lambda;
        status.rcond = NaN;

        for i = 1:options.max_tries
            status.iterations = i;
            final_A = A + lambda * Id;
            status.final_lambda = lambda;

            % Check condition number estimate first
            current_rcond = rcond(final_A);
            status.rcond = current_rcond;

            if isnan(current_rcond) || current_rcond < options.rcond_thresh
                 status.message = sprintf('Attempt %d: RCOND %.2e < threshold %.2e or NaN. Lambda=%.2e', ...
                                          i, current_rcond, options.rcond_thresh, lambda);
                % warning('robustInverseSolve:lowRCOND', status.message); % Issue a MATLAB warning

                 if lambda >= options.max_lambda || i == options.max_tries
                      status.success = false;
                      status.message = sprintf('Failed after %d tries. Max lambda %.2e reached or exceeded. Final RCOND=%.2e.', i, options.max_lambda, current_rcond);
                      return; % Exit function
                 end
                 lambda = lambda * options.lambda_factor; % Increase lambda
                 continue; % Try next iteration
            end

            % RCOND seems ok, now try solving and check for specific MATLAB warnings
            lastwarn(''); % Clear last warning
            try
                x = final_A \ b;
                [warnMsg, warnId] = lastwarn;
                % Check if the *specific* singularity warning occurred
                if isempty(warnMsg) || ~(contains(warnId, 'MATLAB:singularMatrix') || contains(warnId, 'MATLAB:nearlySingularMatrix'))
                    status.success = true;
                    status.message = sprintf('Success on attempt %d with lambda=%.2e, RCOND=%.2e.', i, lambda, current_rcond);
                    return; % Solution found!
                else
                    % Got the singularity warning despite RCOND check
                    status.message = sprintf('Attempt %d: Backslash warning ''%s'' (ID: %s). Lambda=%.2e, RCOND=%.2e', ...
                                             i, warnMsg, warnId, lambda, current_rcond);
                     warning('robustInverseSolve:backslashWarning', status.message);

                     if lambda >= options.max_lambda || i == options.max_tries
                         status.success = false;
                         status.message = sprintf('robustInverseSolve: Failed after %d tries due to backslash warning. Max lambda %.2e reached or exceeded.', i, options.max_lambda);
                         x = nan(m,1); % Ensure x is NaN
                         return; % Exit function
                    end
                    lambda = lambda * options.lambda_factor; % Increase lambda
                    % Continue to next iteration
                end
            catch ME % Catch potential errors during backslash (less common)
                 status.success = false;
                 status.message = sprintf('Attempt %d: Error during solve: %s. Lambda=%.2e', i, ME.message, lambda);
                 warning('robustInverseSolve:solveError', status.message);
                 x = nan(m,1); % Ensure x is NaN
                 return; % Exit function
            end
        end % End for loop

        % If loop finishes without returning, it failed
        status.success = false;
        status.message = sprintf('Failed to find stable solution within %d tries and max_lambda %.2e.', options.max_tries, options.max_lambda);


    case 'svd'
        % ... (code for SVD is unchanged) ...
        lambda = options.initial_lambda;
        final_A = A + lambda * Id;
        status.final_lambda = lambda;

        try
            [U, S, V] = svd(final_A, 'econ'); % Use economy SVD
            s_diag = diag(S);

            % Determine tolerance
            if strcmpi(options.svd_tol_mode, 'relative')
                 max_s = s_diag(1); % Max singular value
                 if max_s == 0, max_s = 1; end % Avoid eps(0) = 0 if matrix is zero
                 tol = max(m, size(S,2)) * eps(max_s) * options.svd_tol;
            elseif strcmpi(options.svd_tol_mode, 'absolute')
                tol = options.svd_tol;
            else
                error('MATLAB:unrecognizedStringChoice', 'Invalid svd_tol_mode. Use ''relative'' or ''absolute''.');
            end

            % Invert singular values, thresholding small ones
            s_inv_diag = zeros(size(s_diag));
            valid_s_idx = s_diag > tol;
            s_inv_diag(valid_s_idx) = 1 ./ s_diag(valid_s_idx);
            status.svd_rank = sum(valid_s_idx); % Number of singular values kept

            % Reconstruct pseudoinverse * b directly
            x = V(:, 1:length(s_inv_diag)) * (diag(s_inv_diag) * (U' * b));

            status.success = true;
            status.message = sprintf('SVD successful. Lambda=%.2e, Rank=%d based on tol=%.2e (%s).', ...
                                     lambda, status.svd_rank, tol, options.svd_tol_mode);

        catch ME
            status.success = false;
            status.message = sprintf('Error during SVD: %s. Lambda=%.2e', ME.message, lambda);
            warning('robustInverseSolve:svdError', status.message);
            x = nan(m, 1);
        end


    case 'iterative'
        lambda = options.initial_lambda;
        final_A = A + lambda * Id;
        status.final_lambda = lambda;

        % --- *** MODIFIED SYMMETRY CHECK *** ---
        % Check symmetry manually using Frobenius norm and absolute tolerance
        diff_norm = norm(final_A - final_A', 'fro');
        if diff_norm > options.symmetry_tol
             error('robustInverseSolve:nonSymmetricMatrix', ...
                   'Iterative method (MINRES) requires the matrix A + lambda*I to be symmetric. norm(A+lambda*I - (A+lambda*I)'', ''fro'') = %.3e, which exceeds tolerance %.3e.', ...
                   diff_norm, options.symmetry_tol);
        end
        % --- *** END MODIFIED SYMMETRY CHECK *** ---

        % Proceed with MINRES only if the matrix is symmetric within tolerance
        solver_name = 'MINRES';
        try
            [x, flag, relres, iter, resvec] = minres(final_A, b, options.iter_tol, options.max_iter);
            status.iter_resvec = resvec; % Store residual vector history
        catch ME
             status.success = false;
             status.message = sprintf('Error during MINRES execution: %s. Lambda=%.2e', ME.message, lambda);
             warning('robustInverseSolve:minresError', status.message);
             x = nan(m, 1);
             flag = -99; % Indicate error
             relres = NaN;
             iter = 0;
        end

        status.iter_flag = flag;
        status.iter_relres = relres;
        status.iterations = iter; % Actual iterations performed

        if flag == 0
            status.success = true;
            status.message = sprintf('%s converged successfully in %d iterations. Lambda=%.2e, RelRes=%.2e.', ...
                                     solver_name, iter, lambda, relres);
        elseif flag > 0
             status.success = false; % Convergence tolerance not met
             status.message = sprintf('%s finished with flag %d (no convergence within max_iter) in %d iterations. Lambda=%.2e, RelRes=%.2e. Check flag meaning.', ...
                                     solver_name, flag, iter, lambda, relres);
             warning('robustInverseSolve:iterWarning', status.message);
        elseif flag < 0 % Includes error case -99
             status.success = false; % Solver breakdown or error
             status.message = sprintf('%s failed with flag %d (breakdown/error) in %d iterations. Lambda=%.2e. Check flag meaning.', ...
                                      solver_name, flag, iter, lambda);
              warning('robustInverseSolve:iterWarning', status.message);
        end


    otherwise
        error('MATLAB:unrecognizedStringChoice','Invalid method specified. Choose ''lambdaInc'', ''SVD'', or ''iterative''.');

end % End switch statement

end % End function