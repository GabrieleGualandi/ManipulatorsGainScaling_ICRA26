function log = addNorm1Stats(log, prefix, signal)
% addNorm1Stats Calculates Norm1 stats and adds them to log struct
%
% Inputs:
%   log    - The log structure to update
%   prefix - String prefix for the field names (e.g., 'power_final')
%   signal - The signal matrix/vector to analyze
%
% Outputs:
%   log    - Updated log structure with new fields:
%            [prefix '_s_abs']
%            [prefix '_l1_per_t']
%            [prefix '_l1_total']
%            [prefix '_l1_mean']
%            [prefix '_l1_max']

    [s_abs, l1, l1_tot, l1_avg, l1_max] = calcStatsNorm1(signal);
    
    log.([prefix '_s_abs'])   = s_abs;
    log.([prefix '_l1'])      = l1;
    log.([prefix '_l1_tot'])  = l1_tot;
    log.([prefix '_l1_avg'])  = l1_avg;
    log.([prefix '_l1_max'])  = l1_max;
end
