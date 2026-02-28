function clearResults()
% clearResults  Removes all simulation result files from the results folder for ICRA26.
%
%   Usage:
%     clearResults()
%
%   There are no inputs required. It automatically targets the 'ICRA26' workspace.

    workspaceID = 'ICRA26';

    % Locate project root
    projectRoot = fileparts(mfilename('fullpath'));
    [projectRoot, folderName] = fileparts(projectRoot);
    if ~strcmp(folderName, 'main_utils')
        projectRoot = fileparts(mfilename('fullpath'));
    end

    resultsDir = fullfile(projectRoot, 'results', workspaceID);
    
    if ~exist(resultsDir, 'dir')
        fprintf('Results directory not found: %s\n', resultsDir);
        return;
    end
    
    fprintf('--------------------------------------------------\n');
    fprintf('Clearing results for workspace: %s\n', workspaceID);
    fprintf('Path: %s\n', resultsDir);
    fprintf('--------------------------------------------------\n');
    
    % Clear everything in the folder
    items = dir(fullfile(resultsDir, '*.mat'));
    if isempty(items)
        fprintf('No result files (.mat) found in %s\n', workspaceID);
    else
        delete(fullfile(resultsDir, '*.mat'));
        fprintf('  [FILE] Removed all %d .mat result files.\n', numel(items));
    end
    fprintf('--------------------------------------------------\n');
end
