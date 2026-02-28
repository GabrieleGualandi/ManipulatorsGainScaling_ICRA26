function log = addNorm2Stats(log, prefix, signal)
% addNorm2Stats Calculates Norm2 stats and adds them to log struct
%
% Inputs:
%   log    - The log structure to update
%   prefix - String prefix for the field names (e.g., 'deviation_HandP')
%   signal - The signal matrix/vector to analyze
%
% Outputs:
%   log    - Updated log structure with new fields:
%            [prefix '_l2']
%            [prefix '_l2_total']
%            [prefix '_l2_mean']
%            [prefix '_l2_max']

    [l2, l2_tot, l2_avg, l2_max] = calcStatsNorm2(signal);
    
    log.([prefix '_l2'])     = l2;
    log.([prefix '_l2_tot']) = l2_tot;
    log.([prefix '_l2_avg']) = l2_avg;
    log.([prefix '_l2_max']) = l2_max;
end
