function Jdls = dampedPseudoRedundant(J)
% DAMPEDPSEUDOREDUNDANT   Compute a damped pseudoinverse for a redundant Jacobian.
%
%   Jdls = DAMPEDPSEUDOREDUNDANT(J) returns the n×m damped pseudoinverse
%   of an m×n Jacobian J, where n>m (redundant robot).  The function:
%     1) Estimates the largest and smallest singular values of J by
%        computing the eigenvalues of (J*J').
%     2) Chooses a scalar damping lambda automatically:
%           alpha = 1e-3 * sigma_max
%           if sigma_min >= alpha
%               lambda = 0
%           else
%               lambda = alpha - sigma_min
%           end
%        so that no damping is applied when cond(J)<1e3, and otherwise
%        lambda grows as sigma_min->0.
%     3) Forms A = J*J' + lambda^2 * I_m (an m×m positive‐definite matrix).
%     4) Computes its Cholesky factor R (so R'*R = A), then solves
%        A * Z = J  (Z is m×n) via two back‐substitutions:
%           Z = R \ ( R' \ J ).
%     5) Returns Jdls = Z'  (which is n×m and equals J' * A^{-1}).
%
%   This is more efficient than forming (J'*J + lambda^2 I_n) when n>>m,
%   since you only invert an m×m matrix instead of n×n.
%
%   Input:
%     J – an m×n Jacobian matrix with n > m.
%
%   Output:
%     Jdls – the n×m Tikhonov‐damped pseudoinverse: 
%              Jdls = J' * (J*J' + lambda^2 I_m)^(-1).
%
%   Example:
%       % For a 6×7 Jacobian J of a 7‐DOF arm:
%       J = randn(6,7);
%       Jdls = dampedPseudoRedundant(J);
%
%   Notes:
%     • If lambda = 0 (i.e. cond(J) <= 1e3), this computes the minimum‐norm
%       right inverse J' * (J*J')^{-1}, which is valid when J has full row rank.
%     • If J loses rank (exactly singular), lambda will be positive and
%       regularize the system.  
%     • Because (J*J') is only m×m, eigenvalue‐ or Cholesky‐based methods
%       on that matrix are much cheaper than an SVD on an m×n (m<n) matrix.
%
%   See also CHOL, EIG.

    % Dimensions
    [m, n] = size(J);
    if m >= n
        error('dampedPseudoRedundant:InvalidSize', ...
              'This routine assumes n > m (i.e. more columns than rows).');
    end


    % 1) Compute J*J' (m×m) and its eigenvalues
    JJt = J * J.';               % size m×m
    eigenvals = eig(JJt);        % compute all m eigenvalues

    % 2) Extract sigma_max and sigma_min
    %    Since JJt is symmetric positive semidefinite, eigenvals >= 0
    sigma_sq_max = max(eigenvals);
    sigma_sq_min = min(eigenvals);
    sigma_max    = sqrt(sigma_sq_max);
    sigma_min    = sqrt( max(sigma_sq_min, 0) );  

    % 3) Choose damping parameter lambda automatically
    alpha = 1e-3 * sigma_max;  
    if sigma_min >= alpha
        lambda = 0;
    else
        lambda = alpha - sigma_min;
    end

    % 4) Form A = J*J' + lambda^2 * I_m
    A = JJt;
    if lambda > 0
        A = A + (lambda^2) * eye(m);
    end

    % 5) Cholesky factorization: A = R' * R (R is upper‐triangular)
    R = chol(A, 'upper');

    % 6) Solve A * Z = J   <=>  R'*(R*Z) = J
    %    First:  W = R' \ J   solves R' * W = J   (m×n right‐hand side)
    %    Second: Z = R  \ W   solves R  * Z = W
    W = R' \ J;  
    Z = R  \ W;              

    % 7) The damped pseudoinverse is Jdls = Z'  (n×m)
    Jdls = Z.';
end
