function [yAttack, st, info] = run(par, st, isSimulation, yAttack)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% RUN Executes the Gaussian Noise attack strategy.
% This plugin injects additive zero-mean multivariate normal noise into the 
% measurement channel, using the robot's sensor covariance (senseCov).
%
% Inputs:
%   par, st, t, tStart, Nsteps, isSimulation - Standard simulator signals.
%   yAttack - Accumulated attack vector from previous plugins.
%
% Outputs:
%   yAttack - Updated attack vector (cumulative noise).
%   st      - Updated SimulatorState object (unchanged by this plugin).
%   info    - Empty structure (no specific metrics for noise attack).
%
% SEE ALSO: CORE_ATTACK

    info = struct();
    
    if isSimulation
        return;
    end

    % Add Noise
    % Logic extracted from core_attack.m
    R = par.sys.senseCov;
    m = size(R,1);
    v = mvnrnd(zeros(1,m), (R+R.')/2, 1).'; 
    yAttack = yAttack + v;

end
