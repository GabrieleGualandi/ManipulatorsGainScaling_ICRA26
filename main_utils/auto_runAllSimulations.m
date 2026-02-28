function results = auto_runAllSimulations(workspaceID, varargin)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% AUTO_RUNALLSIMULATIONS  Batch-run all simulations for a given workspace context.
% Sequentially executes main_2_runSimulation → main_3_postProcess
% for every simulation associated with workspaceID. Simulation names 
% are discovered automatically from the config folder.
%
% Inputs:
%   workspaceID     - Paper context string (e.g., 'ICRA26' or 'IFAC26').
%   varargin     - Name-Value pairs:
%                  'Simulations' - Cell array of simulation IDs to run.
%                  'SaveResults' - Logical, export CSV/MAT (default: true).
%
% Outputs:
%   results      - Struct array containing execution status and simulation data.
%
% SEE ALSO: MAIN_2_RUNSIMULATION, MAIN_3_POSTPROCESS


    arguments
        workspaceID {mustBeTextScalar, mustBeNonempty}
    end
    arguments (Repeating)
        varargin
    end

    % --- parse optional NV pairs ---
    [simulationsOverride, saveResults] = parseOpts(varargin);

    % --- simulation list per paper (discovered from config/ on disk) ---
    allSimulations = getSimulationsForPaper(workspaceID);

    if ~isempty(simulationsOverride)
        % Validate user-supplied subset
        unknown = setdiff(simulationsOverride, allSimulations);
        if ~isempty(unknown)
            error('auto_runAllSimulations:UnknownSimulation', ...
                'Unknown simulation(s) for %s: %s\nAvailable: %s', ...
                workspaceID, strjoin(unknown, ', '), strjoin(allSimulations, ', '));
        end
        simulations = simulationsOverride;
    else
        simulations = allSimulations;
    end

    nSim = numel(simulations);
    fprintf('\n========================================================\n');
    fprintf('  auto_runAllSimulations  |  Paper: %s\n', workspaceID);
    fprintf('  %d simulation(s) queued\n', nSim);
    fprintf('========================================================\n\n');

    % Pre-allocate output struct array
    results = repmat(struct('simulationID','','ok',false,'errorMsg','','simulation',[],'wallTime',nan), nSim, 1);

    totalT0 = tic;

    for i = 1:nSim
        simulationID = simulations{i};
        results(i).simulationID = simulationID;

        fprintf('[%d/%d] ── %s ──────────────────────────────\n', i, nSim, simulationID);
        scenT0 = tic;

        %% Step 1 – Initialise
        fprintf('  [1/3] Initialising (loading from params)...\n');
        parFile = fullfile('params', workspaceID, [simulationID, '.mat']);
        if ~exist(parFile, 'file')
            error('Pre-generated parameter file not found: %s', parFile);
        end
        loadedData = load(parFile);
        par = loadedData.par;

        %% Step 2 – Simulate
        fprintf('  [2/3] Simulating...\n');
        simulation = main_2_runSimulation(par);

        %% Step 3 – Post-process
        fprintf('  [3/3] Post-processing...\n');
        simulation = main_3_postProcess(simulation);

        %% Optional – save to disk
        if saveResults
            % Save simulation to MAT file
            resDir = fullfile('results', workspaceID);
            if ~exist(resDir, 'dir'), mkdir(resDir); end
            fprintf('  [SAVE] Saving simulation to %s/%s.mat...\n', resDir, simulationID);
            save(fullfile(resDir, simulationID), 'simulation');
        end

        elapsed = toc(scenT0);
        results(i).ok         = true;
        results(i).simulation = simulation;
        results(i).wallTime   = elapsed;
        fprintf('  ✓  Done in %.1f s\n\n', elapsed);


    end

    totalElapsed = toc(totalT0);

    %% Summary table
    nOk   = sum([results.ok]);

    fprintf('========================================================\n');
    fprintf('  Summary  |  %d/%d succeeded  |  total %.1f s\n', nOk, nSim, totalElapsed);
    fprintf('========================================================\n');
    for i = 1:nSim
        if results(i).ok
            tag = sprintf('OK   (%.1f s)', results(i).wallTime);
        else
            tag = sprintf('FAIL – %s', results(i).errorMsg);
        end
        fprintf('  %-40s  %s\n', results(i).simulationID, tag);
    end
    fprintf('========================================================\n\n');
end

% =========================================================================
%  HELPERS
% =========================================================================

function simulations = getSimulationsForPaper(workspaceID)
% getSimulationsForPaper  Return simulation IDs for a paper by reading the
% params folder on disk via getAllSimulationsNames.
    infoDB = getAllSimulationsNames('params', workspaceID);
    fieldName = matlab.lang.makeValidName(workspaceID);
    if ~isfield(infoDB, fieldName)
        available = strjoin(fieldnames(infoDB), ', ');
        error('auto_runAllSimulations:UnknownPaper', ...
            'No simulations found for workspaceID="%s".\nAvailable papers: %s', ...
            workspaceID, available);
    end
    simulations = infoDB.(fieldName);
end

function [simulationsOverride, saveResults] = parseOpts(args)
% Parse optional name-value pairs.
    simulationsOverride = {};
    saveResults       = true;

    i = 1;
    while i <= numel(args)
        name = args{i};
        if ~ischar(name) && ~isstring(name)
            error('auto_runAllSimulations:BadOpt', 'Expected a string option name at position %d.', i);
        end
        val = args{i+1};
        switch lower(char(name))
            case 'simulations'
                if ischar(val),   val = {val}; end
                if isstring(val), val = cellstr(val); end
                simulationsOverride = val;
            case 'saveresults'
                saveResults = logical(val);
            otherwise
                error('auto_runAllSimulations:UnknownOpt', 'Unknown option: "%s".', name);
        end
        i = i + 2;
    end
end
