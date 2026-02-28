function [s_abs, l1_per_t, l1_total, l1_mean, l1_max] = calcStatsNorm1(s)
% calcStatsNorm1
%
% Input:
%   s : matrix in R^{n x T}
%       Row index j ∈ {1,...,n}, column (time) index k ∈ {1,...,T}.
%
% Output:
%   s_abs     : |s| elementwise.
%   l1_per_t  : 1×T vector, per-time-step ℓ1 norm.
%   l1_total  : scalar, accumulated ℓ1 norm over all time steps.
%   l1_mean   : scalar, temporal mean of the per-time-step ℓ1 norm.
%   l1_max    : scalar, temporal maximum of the per-time-step ℓ1 norm.

% Elementwise absolute value
% LaTeX:
%   s_{\text{abs}}(j,k) = \lvert s(j,k) \rvert,
%   \quad j = 1,\ldots,n,\; k = 1,\ldots,T.
s_abs = abs(s);

% Per-column (per time step) ℓ1 norm
% LaTeX:
%   \ell_1(k) = \sum_{j=1}^{n} \lvert s(j,k) \rvert,
%   \quad k = 1,\ldots,T.
l1_per_t = sum(s_abs, 1);

% Total accumulated ℓ1 norm over all time steps
% LaTeX:
%   L_{1,\text{total}}
%   = \sum_{k=1}^{T} \ell_1(k)
%   = \sum_{k=1}^{T} \sum_{j=1}^{n} \lvert s(j,k) \rvert.
l1_total = sum(l1_per_t);

% Mean ℓ1 norm over time
% LaTeX:
%   L_{1,\text{mean}}
%   = \frac{1}{T} \sum_{k=1}^{T} \ell_1(k).
l1_mean = mean(l1_per_t);

% Maximum ℓ1 norm over time
% LaTeX:
%   L_{1,\text{max}}
%   = \max_{k \in \{1,\ldots,T\}} \ell_1(k).
l1_max = max(l1_per_t);

end
