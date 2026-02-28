function simulation = main_2_runSimulation(par)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% main_2_runSimulation   Runs the core simulation loop.
% This function initializes the SimulatorState and invokes the coreSimulator
% to execute the robot control and attack scenario.
%
% Inputs:
%   par        - Simulation configuration structure.
% 
% Outputs:
%   simulation - Structure containing the final 'log' and 'par'.
%
% SEE ALSO: CORESIMULATOR, SIMULATORSTATE, MAIN_1_INITSIMULATION

    % Ensure generated functions are on path if needed
    if isfield(par, 'sim') && isfield(par.sim, 'functionHandleMethod') && ...
       (strcmp(par.sim.functionHandleMethod, 'file') || strcmp(par.sim.functionHandleMethod, 'mex'))
        genDir = fullfile(pwd, 'genFunctions');
        if exist(genDir, 'dir')
            addpath(genDir);
        end
    end

    % Display simulation info if available
    if isfield(par, 'info') && isfield(par.info, 'simulationInfo')
        fprintf('\n--- STARTING SIMULATION ---\n%s\n---------------------------\n', par.info.simulationInfo);
    end
    
    st = SimulatorState();
    st.initialize(par.simulatorStateInit);
    
    % Execution
    rng(3) 
    tic
    log = coreSimulator(par, st, par.sim.Nsteps, struct('isSimulation',false, 'isJacobian',false, 'tStart',1));
    compuTime = toc;
    fprintf('Simulation completed in %.4f seconds.\n', compuTime);
    
    simulation = struct('par', par, 'log', log); 
end
