function [rmsError, maxError] = computeTrajectoryMetrics(p_ref, p_act)
%computeTrajectoryMetrics Compare reference and actual 3D trajectories.
%
%   INPUTS:
%     p_ref : 3 x N_ref matrix, reference trajectory (columns = samples).
%     p_act : 3 x N_act matrix, actual trajectory   (columns = samples).
%
%   OUTPUTS:
%     rmsError        : RMS position error over time (Euclidean), only
%                       meaningful if N_ref == N_act and same time grid.
%     maxError        : Maximum position error over time.
%
%   NOTES:
%   - p_ref and p_act are treated as polygonal chains in R^3 (columns are
%     vertices in order).
%   - rmsError and maxError assume that p_ref and p_act are sampled on the
%     same time grid; if not, they are set to NaN.

    % --- Basic input checks -----------------------------------------------
    if size(p_ref,1) ~= size(p_act,1)
        error('p_ref and p_act must have the same number of rows (spatial dimension).');
    end

    % Ensure double precision
    p_ref = double(p_ref);
    p_act = double(p_act);

    N_ref = size(p_ref, 2);
    N_act = size(p_act, 2);

    % --- 1) Time-domain tracking error ------------------------------------
    if N_ref == N_act
        diff = p_ref - p_act;                 % 3 x N
        distPerSample = sqrt(sum(diff.^2, 1));% 1 x N

        rmsError = sqrt(mean(distPerSample.^2));
        maxError = max(distPerSample);
    else
        % Time grids incompatible (or not aligned) -> no meaningful RMS/max
        rmsError = NaN;
        maxError = NaN;
    end
end
