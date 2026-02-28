%==========================================================================
% Author: Gabriele Gualandi     Date: 2025     Ver: 1.0
%==========================================================================
function [A_inv_robust, status_all] = invRobust(A, method, options)
%COMPUTEROBUSTINVERSE Computes a robust inverse of A using robustInverseSolve.
%
%   [A_inv_robust, status_all] = invRobust(A, method, options)
%
%   Solves A * X = I column by column using robustInverseSolve.
%
%   Inputs:
%       A       - The matrix to invert.
%       method  - Method for robustInverseSolve ('lambdaInc', 'SVD', 'iterative').
%       options - (Optional) Options struct for robustInverseSolve.
%
%   Outputs:
%       A_inv_robust - The robustly computed (pseudo)inverse matrix.
%       status_all   - Cell array containing the status struct from each
%                      column solve.

    n = size(A, 1);
    if size(A, 2) ~= n
        error('Matrix A must be square.');
    end

    if nargin < 3
        options = struct();
    end

    I = eye(n);
    A_inv_robust = zeros(n, n);
    status_all = cell(1, n);
    all_success = true;

    %fprintf('Computing robust inverse column by column using method: %s\n', method);
    for i = 1:n
        e_i = I(:, i); % i-th column of identity matrix

        try
            [x_i, status_i, ~] = robustInverseSolve(A, e_i, method, options);

            status_all{i} = status_i;
            if ~status_i.success
                warning('invRobust: robustInverseSolve failed for column %d. Status: %s', i, status_i.message);
                A_inv_robust(:, i) = NaN; % Mark column as failed
                all_success = false;
            else
                A_inv_robust(:, i) = x_i;
            end
        catch ME
             warning('invRobust: Error solving for column %d: %s', i, ME.message);
             status_all{i} = struct('success', false, 'message', ME.message);
             A_inv_robust(:, i) = NaN;
             all_success = false;
        end

        if mod(i, 10) == 0 || i == n
             %fprintf('  Processed column %d of %d\n', i, n);
        end
    end

    if ~all_success
        warning('invRobust:FailedSolve', 'One or more columns failed to solve. Resulting matrix contains NaN.');
    else
         %fprintf('Robust inverse computation completed.\n');
    end

end