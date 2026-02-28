function log = addTrajectoryMetrics(log, prefix, p_ref, p_act)
% addTrajectoryMetrics Calculates trajectory metrics and adds them to log struct
%
% Inputs:
%   log    - The log structure to update
%   prefix - String prefix for the field names (e.g., 'hand' or 'att')
%   p_ref  - Reference trajectory
%   p_act  - Actual trajectory
%
% Outputs:
%   log    - Updated log structure with new fields:
%            [prefix '_rmsError']
%            [prefix '_maxError']

    [rmsError, maxError] = computeTrajectoryMetrics(p_ref, p_act);
        
    log.([prefix '_rmsError'])        = rmsError;
    log.([prefix '_maxError'])        = maxError;
end
