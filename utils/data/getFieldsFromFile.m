function out = getFieldsFromFile(matFilename, varPath, fields)
%GETFIELDSFROMFILE Load selected fields from a (sub)struct in a MAT-file.
%
%   out = GETFIELDSFROMFILE(matFilename, varPath, fields)
%
%   Inputs:
%     matFilename : char or string
%         Path to the .mat file.
%
%     varPath     : char or string
%         Name of a struct variable or a dot-separated path to a sub-struct
%         inside it. Examples:
%             'simulation'
%             'simulation.log'
%             'simulation.results.inner'
%
%     fields      : (optional) char, string array, or cellstr
%         Names of the fields to keep from the final struct.
%         If empty or omitted:
%           - if varPath ends at a struct, the entire struct is returned;
%           - if varPath ends at a non-struct (e.g. a scalar), that value
%             is returned as-is.
%
%   Output:
%     out : struct (if selecting fields from a struct) or any MATLAB value
%           (if varPath ends at a non-struct and fields is empty).

    if nargin < 3
        fields = [];
    end

    matFilename = i_toCharScalar(matFilename, 'matFilename');
    varPath     = i_toCharScalar(varPath, 'varPath');

    if ~isfile(matFilename)
        error('getFieldsFromFile:FileNotFound', ...
            'MAT-file not found: "%s".', matFilename);
    end

    if isempty(strtrim(varPath))
        error('getFieldsFromFile:EmptyPath', ...
            'varPath is empty. Provide a variable name or a dot-separated path.');
    end

    % Parse varPath: first token is the root variable in the MAT-file
    parts   = strsplit(varPath, '.');
    parts   = parts(~cellfun(@isempty, parts)); % guard against accidental '..'
    if isempty(parts)
        error('getFieldsFromFile:InvalidPath', ...
            'Invalid varPath "%s".', varPath);
    end

    rootVar = parts{1};
    subPath = parts(2:end);

    % Inspect MAT-file contents (for better diagnostics)
    try
        fileVarsInfo = whos('-file', matFilename);
    catch ME
        error('getFieldsFromFile:CannotInspectFile', ...
            'Cannot inspect MAT-file "%s". Original error: %s', ...
            matFilename, ME.message);
    end

    fileVarNames = {fileVarsInfo.name};

    % Load only the root variable
    try
        data = load(matFilename, rootVar);
    catch ME
        error('getFieldsFromFile:LoadFailed', ...
            'Failed to load variable "%s" from MAT-file "%s". Original error: %s', ...
            rootVar, matFilename, ME.message);
    end

    % --- Compatibility Logic: experiment <-> simulation fuzzy matching ---
    if ~isfield(data, rootVar)
        % If we wanted 'simulation' but file has 'experiment', or vice-versa, load the alias instead.
        alias = '';
        if strcmp(rootVar, 'simulation'), alias = 'experiment';
        elseif strcmp(rootVar, 'experiment'), alias = 'simulation'; 
        end
        
        if ~isempty(alias) && ismember(alias, fileVarNames)
            warning('getFieldsFromFile:LegacyTerminology', ...
                'Variable "%s" not found in "%s", but found legacy variable "%s". Loading alias.', ...
                rootVar, matFilename, alias);
            rootVar = alias;
            data = load(matFilename, rootVar);
        end
    end
    % ---------------------------------------------------------------------

    if ~isfield(data, rootVar)
        if isempty(fileVarNames)
            availMsg = '(the MAT-file appears to contain no variables)';
        else
            availMsg = sprintf('Available variables: %s', i_joinList(fileVarNames, 12));
        end

        error('getFieldsFromFile:VarNotFound', ...
            ['Root variable "%s" not found in MAT-file "%s".\n' ...
             '%s'], ...
            rootVar, matFilename, availMsg);
    end

    var = data.(rootVar);
    currentPath = rootVar;

    % Follow the subPath, if any
    for k = 1:numel(subPath)
        level = subPath{k};

        if ~isstruct(var)
            error('getFieldsFromFile:NotStructAlongPath', ...
                ['Cannot continue path "%s" at level "%s".\n' ...
                 'Resolved path so far: "%s"\n' ...
                 'The value at "%s" is %s, not a struct.'], ...
                varPath, level, currentPath, currentPath, i_describeValue(var));
        end

        % Optional: clarify unsupported indexing case (struct arrays)
        if ~isscalar(var)
            error('getFieldsFromFile:StructArrayAlongPath', ...
                ['Path traversal reached "%s", which is a struct array of size %s.\n' ...
                 'Dot-only paths are ambiguous on struct arrays. Index explicitly before saving, ' ...
                 'or store a scalar struct in the MAT-file.'], ...
                currentPath, mat2str(size(var)));
        end

        if ~isfield(var, level)
            availFields = fieldnames(var);
            if isempty(availFields)
                availMsg = '(the struct has no fields)';
            else
                availMsg = sprintf('Available fields at "%s": %s', ...
                    currentPath, i_joinList(availFields, 15));
            end

            error('getFieldsFromFile:MissingLevel', ...
                ['Field "%s" not found while following path "%s".\n' ...
                 'Failure occurred at struct "%s".\n' ...
                 '%s'], ...
                level, varPath, currentPath, availMsg);
        end

        var = var.(level);
        currentPath = [currentPath '.' level];
    end

    % If no specific fields requested, return whatever we reached
    if isempty(fields)
        out = var;
        return;
    end

    % From here on, we expect var to be a struct
    if ~isstruct(var)
        error('getFieldsFromFile:TargetNotStruct', ...
            ['Target "%s" is %s, not a struct.\n' ...
             'Field selection is only valid when varPath resolves to a struct.'], ...
            currentPath, i_describeValue(var));
    end

    % Normalize "fields" to a cell array of char
    fields = i_normalizeFieldList(fields);

    % Optional: remove duplicates while preserving order
    fields = unique(fields, 'stable');

    allFields = fieldnames(var);
    missing   = setdiff(fields, allFields);

    if ~isempty(missing)
        if isempty(allFields)
            availMsg = '(the target struct has no fields)';
        else
            availMsg = sprintf('Available fields at "%s": %s', ...
                currentPath, i_joinList(allFields, 20));
        end

        error('getFieldsFromFile:MissingField', ...
            ['Requested field(s) not present at "%s": %s\n' ...
             '%s'], ...
            currentPath, i_joinList(missing, 20), availMsg);
    end

    % Keep only the requested fields
    toRemove = setdiff(allFields, fields);

    if isempty(toRemove)
        out = var;
    else
        out = rmfield(var, toRemove);
    end
end


% ---------- Local helpers ----------

function s = i_toCharScalar(x, argName)
    if isstring(x)
        if ~isscalar(x)
            error('getFieldsFromFile:InvalidInputType', ...
                '%s must be a char vector or a string scalar.', argName);
        end
        s = char(x);
    elseif ischar(x)
        s = x;
    else
        error('getFieldsFromFile:InvalidInputType', ...
            '%s must be a char vector or a string scalar.', argName);
    end
end

function fields = i_normalizeFieldList(fields)
    if ischar(fields)
        fields = {fields};
    elseif isstring(fields)
        fields = cellstr(fields(:));  % string array -> cellstr column
    elseif iscell(fields)
        % accept cell array of char/string scalars
        ok = true(size(fields));
        for ii = 1:numel(fields)
            if isstring(fields{ii}) && isscalar(fields{ii})
                fields{ii} = char(fields{ii});
            elseif ~ischar(fields{ii})
                ok(ii) = false;
            end
        end
        if ~all(ok)
            error('getFieldsFromFile:InvalidFields', ...
                'fields must be char, string, or a cell array of char/string scalars.');
        end
        fields = fields(:).';
    else
        error('getFieldsFromFile:InvalidFields', ...
            'fields must be char, string, or a cell array of char/string scalars.');
    end

    % Validate non-empty names
    if any(cellfun(@isempty, fields))
        error('getFieldsFromFile:InvalidFields', ...
            'fields contains an empty field name.');
    end
end

function txt = i_describeValue(v)
    cls = class(v);
    sz  = size(v);
    txt = sprintf('a %s of size %s', cls, mat2str(sz));
end

function txt = i_joinList(c, maxShow)
    % Join a list of names, truncating if necessary
    if nargin < 2
        maxShow = inf;
    end
    c = c(:).';
    n = numel(c);

    if n == 0
        txt = '(none)';
        return;
    end

    if n <= maxShow
        txt = strjoin(c, ', ');
    else
        head = c(1:maxShow);
        txt = sprintf('%s, ... (%d more)', strjoin(head, ', '), n - maxShow);
    end
end