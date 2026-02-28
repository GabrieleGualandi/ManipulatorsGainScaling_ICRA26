% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% MAIN_BATCH_EXECUTION Batch entry point for ICRA26 workspace.
% Executes all simulations, exports all data, and plot images.

% Ensure that your solver is visible e.g.,
% addpath('/Library/gurobi1203/macos_universal2/matlab')

clc; clear; close all;
% Add all subfolders to the MATLAB path
addpath(genpath(pwd));

%% 1. Configuration
workspaceID = 'ICRA26';

%% 2. Batch Execution
% Run ALL simulations defined in config/workspaceID/
% (Each run saves its results to .mat in results/workspaceID/)
fprintf('Starting batch execution for %s...\n', workspaceID);
auto_runAllSimulations(workspaceID);

%% 3. Visualization
% Call aggregate plotting script (expects all results to be on disk)
ICRA26plotResults;

