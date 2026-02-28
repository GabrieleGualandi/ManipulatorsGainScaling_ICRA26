function simDB = getAllSimulationsNames(baseDir, targetPaper)
% getAllSimulationsNames  Discover all paper contexts and their simulations
%                        by scanning the params directory on disk.
%
% simDB = getAllSimulationsNames()
% simDB = getAllSimulationsNames('params')   % explicit base dir (default)
% simDB = getAllSimulationsNames('params', 'ICRA26') % log only for ICRA26
%
% Returns a struct with one field per paper context found, each containing
% a cell array of simulation IDs (filename without extension):
%
%   simDB.ICRA26 = {'NoDefence_NoADS_YesAttack', 'NoDefence_YesAttack', ...}
%   simDB.IFAC26 = {'A1_NoDef_circle', 'A2_damping_circle', ...}
%
% Discovery rules (intentionally simple):
%   1. A "paper folder" is any direct subdirectory of baseDir that
%      - does NOT start with '_'  (excludes _old_experiments, etc.)
%   2. Simulations are the basenames of *.mat files that sit directly inside
%      <paper>/ — files inside any sub-subfolder are ignored.
%      'par.mat' is explicitly excluded.
%
% Usage examples
%   simDB    = getAllSimulationsNames();
%   papers   = fieldnames(simDB);          % {'ICRA26', 'IFAC26'}
%   simIDs   = simDB.ICRA26;              % cell array of simulation names

    if nargin < 1 || isempty(baseDir)
        baseDir = 'params';
    end
    if nargin < 2, targetPaper = ''; end
    baseDir = char(baseDir);

    if ~isfolder(baseDir)
        error('getAllSimulationsNames:MissingBaseDir', ...
            'Params base directory not found: %s', baseDir);
    end

    simDB = struct();

    % ── 1. List direct subdirectories of baseDir ──────────────────────────
    items = dir(baseDir);
    items = items([items.isdir]);                    % keep directories only
    items = items(~ismember({items.name}, {'.','..'}));  % drop . and ..

    for i = 1:numel(items)
        paperName = items(i).name;

        % Skip hidden / legacy folders (names starting with '_' or '.')
        if paperName(1) == '_' || paperName(1) == '.'
            continue;
        end

        simDir = fullfile(baseDir, paperName);
        if ~isfolder(simDir)
            continue;   % not a valid paper folder
        end

        % ── 2. List *.mat files directly in paper dir (no recursion) ──
        matFiles = dir(fullfile(simDir, '*.mat'));
        % dir() includes files only (not subdirs) when given a glob pattern,
        % so we just strip the extension to get the simulation IDs.
        simIDs = erase({matFiles.name}, '.mat');
        simIDs = simIDs(:);   % ensure column cell
        
        % exclude 'par' as it is a generic generated param file
        simIDs = simIDs(~strcmp(simIDs, 'par'));

        if isempty(simIDs)
            warning('getAllSimulationsNames:EmptyPaper', ...
                'No *.mat simulations found in: %s', simDir);
            continue;
        end

        % Sanitise paper name into a valid MATLAB field name
        fieldName = matlab.lang.makeValidName(paperName);
        simDB.(fieldName) = simIDs;

        % Log finding ONLY if it matches our target paper or if no target is specified
        if isempty(targetPaper) || strcmpi(paperName, targetPaper)
            fprintf('  Found paper "%s"  (%d simulation(s)): %s\n', ...
                paperName, numel(simIDs), strjoin(simIDs, ', '));
        end
    end

    if isempty(fieldnames(simDB))
        warning('getAllSimulationsNames:NoPapersFound', ...
            'No valid paper folders found under: %s', baseDir);
    end
end
