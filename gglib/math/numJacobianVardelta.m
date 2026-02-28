function [J, f0, kstar, Jcells, Djcells, hcells] = numJacobianVardelta(f, q0, kInterval, verbose)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% NUMJACOBIANVARDELTA Compute the Jacobian of a vector-valued function f numerically.
% This function evaluates the optimal finite-difference step size over a provided range
% to minimize numerical error, returning both the Jacobian and the optimal step index.
%
% Inputs:
%   f         - Function handle evaluating to a column vector.
%   q0        - Operating point vector (nx1).
%   kInterval - Vector of exponents k defining candidate step sizes.
%   verbose   - Boolean flag to enable warnings.
%
% Outputs:
%   J         - Numerical Jacobian matrix evaluated at q0.
%   f0        - Function evaluation at q0.
%   kstar     - Most frequently selected index in kInterval for the step size.
%   Jcells    - (Optional) Cell array of evaluated Jacobians for each candidate k.
%   Djcells   - (Optional) Cell array of central differences per coordinate.
%   hcells    - (Optional) Cell array of candidate step sizes used per coordinate.

% Ensure q0 is a column vector
q0 = q0(:);
n  = numel(q0);

% Evaluate f at the operating point once
f0 = f(q0);
m  = numel(f0);

% Preallocate Jacobian
J = zeros(m, n);

% Compute base step sizes and form array of candidate h values
epsmach  = eps;
baseStep = epsmach^(1/2) * (abs(q0) + 1);

K      = kInterval(:).';
nK     = numel(K);
h_vals = baseStep .* (10.^K);

% Preallocate central difference estimates
D = zeros(m, nK, n);

% Store chosen indices per coordinate
kChosen = zeros(n, 1);

for j = 1:n
    ej = zeros(n, 1); ej(j) = 1;

    % Evaluate all candidate central differences for coordinate j
    for idx = 1:nK
        hj      = h_vals(j, idx);
        f_plus  = f(q0 + hj * ej);
        f_minus = f(q0 - hj * ej);
        D(:, idx, j) = (f_plus - f_minus) / (2 * hj);
    end

    if nK == 1
        % Single step: no search, just take it
        J(:, j)    = D(:, 1, j);
        kChosen(j) = 1;
    else
        % ----- Robust selection of k for column j -----
        % 1) Per-candidate magnitudes (to detect "all-zero" artefacts)
        Nk = zeros(1,nK);
        for kk = 1:nK
            Nk(kk) = norm(D(:,kk,j), Inf);
        end

        % Scale-aware thresholds
        absTol = 10*eps;                          % absolute floor
        relTol = 1e-8;                            % relative floor
        Nk_s = Nk;
        Nk_s(~isfinite(Nk_s)) = 0;
        scaleF = max([norm(f0,Inf), max(Nk_s), 1]);        
        tau    = max(absTol, relTol*scaleF);      % magnitude threshold

        % 2) Successive differences (convergence curve)
        E = zeros(1,nK-1);
        for kk = 1:(nK-1)
            E(kk) = norm(D(:,kk+1,j) - D(:,kk,j), Inf);
        end

        % 3) Ignore pairs where both sides are numerically zero
        bothSmall = (Nk(1:end-1) < tau) & (Nk(2:end) < tau);
        E(bothSmall) = Inf;

        % 4) Choose the best pair; if none, fall back to the largest nonzero Nk
        [Emn, kpair] = min(E);
        if isinf(Emn)
            [~, kbest] = max(Nk);                 % widest nonzero signal
        else
            % Prefer the member of the pair with larger magnitude (avoid zero choice)
            if Nk(kpair+1) >= Nk(kpair)
                kbest = kpair + 1;
            else
                kbest = kpair;
            end
        end

        J(:,j)    = reshape(D(:,kbest,j), m, 1);  % assign chosen column
        kChosen(j)= kbest;

        if verbose
            if kbest >= nK-1
                warning('numJacobianVardelta: k* near upper bound (j=%d).', j);
            elseif kbest <= 2 && nK>2
                warning('numJacobianVardelta: k* near lower bound (j=%d).', j);
            end
        end
    end

    if nargout > 3 % requesting all Jacobians
        diagOut.J_all    = zeros(m,n,nK);
        for k = 1:nK, diagOut.J_all(:,:,k) = squeeze(D(:,k,:)); end
    end
end

% Collapse to single scalar kstar
if nK == 1
    kstar = 1;  % single available step
else
    kstar = mode(kChosen);  % most frequently chosen index
end





%%
if nargout > 3
    % Safe 2-D slice from 3-D array, preserving (m×n)
    slice2D = @(A, rows, mid, cols, m, n) reshape(A(rows, mid, cols), m, n);

    if nargout >= 4
        % Build cells robustly
        Jcells = cell(1,nK);
        for k = 1:nK
            Jcells{k} = slice2D(D, 1:m, k, 1:n, m, n);
        end
    end

    if nargout >= 5
        Djcells = cell(1,n);
        for j = 1:n
            Djcells{j} = reshape(D(:, :, j), m, nK);
        end
    end

    if nargout >= 6
        % Actual step sizes used per coordinate, as cells (for readability)
        % hcells{j} is a 1×nK row of the hj values tried for variable j
        hcells = cell(1, n);
        for j = 1:n
            hcells{j} = h_vals(j, :);
        end
    end
end

%% Final important check
if all(all(J == 0))
    f0 = nan;
    J = nan;
    return
end
end
