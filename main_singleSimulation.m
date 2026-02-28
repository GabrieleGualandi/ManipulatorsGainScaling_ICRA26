% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% MAIN_SINGLE_SIMULATION Main entry point for a single simulation run.
% This script allows selecting and running a single simulation scenario,
% saving its results, and visualizing the output.
%
% Ensure that your solver is visible e.g.,
% addpath('/Library/gurobi1203/macos_universal2/matlab')

clc; clear; close all;
addpath(genpath(pwd));


%% 1. Configuration
workspaceID = 'ICRA26';

% Discover available simulations and allow selection
simDB = getAllSimulationsNames('params', workspaceID);
simulations = simDB.(workspaceID);

% --- USER SELECTION ---
% Change the index or ID to run a different simulation
simulationID = simulations{1}; 
fprintf('Selected Simulation: %s\n', simulationID);
% ----------------------

%% 2. Initialization & Execution
% Step 1: Initialize (load pre-generated parameters)
parFile = fullfile('params', workspaceID, [simulationID, '.mat']);
if ~exist(parFile, 'file')
    error('Pre-generated parameter file not found: %s', parFile);
end
loadedData = load(parFile);
par = loadedData.par;

%% Step 2: Run Simulation
fprintf('Running simulation: %s...\n', simulationID);
simulation = main_2_runSimulation(par);

%% Step 3: Post-processing
simulation = main_3_postProcess(simulation);

%% 3. Save Results
% Save simulation to .mat 
resDir = fullfile('results', workspaceID);
if ~exist(resDir, 'dir'), mkdir(resDir); end
fprintf('Saving simulation to %s/%s.mat...\n', resDir, simulationID);
save(fullfile(resDir, simulationID), 'simulation');

