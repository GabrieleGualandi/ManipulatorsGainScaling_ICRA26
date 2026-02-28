function [l2_per_t, l2_total, l2_mean, l2_max] = calcStatsNorm2(s)
% calcStatsNorm2
%
% Input:
%   s : matrix in R^{n x T}
%       Row index j ∈ {1,...,n}, column (time) index k ∈ {1,...,T}.
%       The k-th column s(:,k) is the n-dimensional signal at time k.
%
% Output:
%   l2_per_t : 1×T vector. l2_per_t(k) = ||s(:,k)||_2.
%   l2_total : scalar. Sum over time of ||s(:,k)||_2.
%   l2_mean  : scalar. Time-average of ||s(:,k)||_2.
%   l2_max   : scalar. Maximum over time of ||s(:,k)||_2.
%
% LaTeX notation:
% Let s(j,k) denote the element in row j, column k of s.
% Let \bm{s}_k ∈ ℝ^n denote the k-th column of s:
%   \bm{s}_k = [ s(1,k), s(2,k), ..., s(n,k) ]^T.
%
% Per-time-step Euclidean norm:
%   \ell_2(k) = \lVert \bm{s}_k \rVert_2
%             = \sqrt{\sum_{j=1}^{n} ( s(j,k) )^2},
%   \quad k = 1,\ldots,T.
%
% Total accumulated Euclidean norm over all time steps:
%   L_{2,\text{total}}
%   = \sum_{k=1}^{T} \ell_2(k).
%
% Mean Euclidean norm over time:
%   L_{2,\text{mean}}
%   = \frac{1}{T} \sum_{k=1}^{T} \ell_2(k).
%
% Maximum Euclidean norm over time:
%   L_{2,\text{max}}
%   = \max_{k \in \{1,\ldots,T\}} \ell_2(k).

% Per-time-step Euclidean norm of each column:
% In MATLAB: vecnorm(s,2,1) gives [ ||s(:,1)||_2, ..., ||s(:,T)||_2 ]
l2_per_t = max(vecnorm(s, 2, 1), 0);

% Total accumulated L2 magnitude over time:
% L_{2,\text{total}} = sum_k \ell_2(k)
l2_total = sum(l2_per_t);

% Time-average of the L2 norm:
% L_{2,\text{mean}} = (1/T) * sum_k \ell_2(k)
l2_mean = mean(l2_per_t);

% Maximum L2 norm across time:
% L_{2,\text{max}} = max_k \ell_2(k)
l2_max = max(l2_per_t);

end
