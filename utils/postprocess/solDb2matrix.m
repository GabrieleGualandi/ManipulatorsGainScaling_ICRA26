function [matOut, colNames] = solDb2matrix(solDb, options)
% SOLDB2MATRIX Converts a structure of time-series data into a concatenated matrix.
%
% This function iterates through all fields of the input structure 'solDb'
% and assembles them into a single numeric matrix where each row corresponds 
% to a time sample and each column corresponds to a specific signal component.
%
% Inputs:
%   solDb    - A structure where each field contains a numeric array of size [nR x N].
%              N is the number of time samples (must be consistent across fields).
%              nR is the number of signal components (e.g., number of joints).
%   options  - (Name-Value pairs)
%              'ignoreEmpty'  (bool) - Skip empty fields if true. Default: false.
%              'ignoreNan'    (bool) - Skip fields with NaN if true. Default: false.
%              'setNanToZero' (bool) - Replace NaNs with 0 if true. Default: false.
%
% Outputs:
%   matOut   - A numeric matrix of size [N x TotalComponents].
%              Note: Input signals are transposed so time is along the first dimension.
%   colNames - A cell array of strings containing the name of each column.
%              Vector fields are expanded with suffixes (e.g., 'field_1', 'field_2').
    arguments
        solDb (1,1) struct
        options.ignoreEmpty (1,1) logical = false
        options.ignoreNan (1,1) logical = false
        options.setNanToZero (1,1) logical = false
    end

    ignoreEmpty = options.ignoreEmpty;
    ignoreNan = options.ignoreNan;
    setNanToZero = options.setNanToZero;

    matOut = [];
    fNames = fieldnames(solDb);
    colNames = {};
    
    % --- First Pass: Determine signal length N ---
    N = 1; % Default to 1 (purely scalar struct)
    for i = 1:numel(fNames)
        fNow = solDb.(fNames{i});
        if ~isempty(fNow)
            N = max(N, size(fNow, 2));
        end
    end

    % --- Second Pass: Process fields ---
    for i = 1:numel(fNames)
        fnameNow = fNames{i};
        fNow = solDb.(fnameNow);

        % Handle empty fields
        if isempty(fNow)
            if ignoreEmpty
                continue; 
            else
                error('solDb2matrix:EmptyField', 'Field "%s" is empty (Size: %s). Set ignoreEmpty=true to skip.', ...
                    fnameNow, mat2str(size(fNow)));
            end
        end

        % Handle NaN values
        if any(isnan(fNow(:)))
            if setNanToZero
                fNow(isnan(fNow)) = 0; 
            elseif ~ignoreNan
                error('solDb2matrix:NaNDetected', 'Field "%s" contains NaN values. Set ignoreNan=true or setNanToZero=true.', fnameNow);
            else
                continue; 
            end
        end

        % Check for sample count consistency
        [nR, nC] = size(fNow);
        
        if nC == 1 && N > 1
            % Automatic repetition of scalar values to match signal length
            fNow = repmat(fNow, 1, N);
        elseif nC ~= N
            error('solDb2matrix:DimensionMismatch', ...
                ['Dimension mismatch for field "%s". \n' ...
                 'It has %d samples, while the structure contains signals with %d samples.\n' ...
                 'Sorted structures require pre-calculating the simulation length.'], ...
                fnameNow, nC, N);
        end

        if nR == 1 % scalar over time
            try
                colNames{end+1} = fnameNow; %#ok<AGROW>
                matOut(:, end+1) = fNow(1, :).'; %#ok<AGROW>
            catch ME
                 error('solDb2matrix:ConcatenationError', 'Error adding scalar field "%s" to output: %s', fnameNow, ME.message);
            end
        else % vector over time
            for j = 1:nR
                try
                    colNames{end+1} = sprintf('%s_%d', fnameNow, j); %#ok<AGROW>
                    matOut(:, end+1) = fNow(j, :).'; %#ok<AGROW>
                catch ME
                    error('solDb2matrix:ConcatenationError', 'Error adding component %d of field "%s" to output: %s', j, fnameNow, ME.message);
                end
            end
        end
    end
end
