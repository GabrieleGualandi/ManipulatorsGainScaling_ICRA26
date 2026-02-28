function [numResults, stringResults] = addFieldsToStructResults(numResults, stringResults, dataList, nd_1, nd_2)
% addFieldsToStructResults Batch adds fields to numResults and stringResults
%
% Inputs:
%   numResults    - Numeric results struct
%   stringResults - String results struct
%   dataList      - Cell array: {FieldName, Value, FormatCode (1 or 2)}
%   nd_1          - Decimal places for format code 1
%   nd_2          - Decimal places for format code 2
% 
% Output:
%   numResults    - Updated numeric results
%   stringResults - Updated string results

    for i = 1:size(dataList, 1)
        name = dataList{i, 1};
        val  = dataList{i, 2};
        fmt  = dataList{i, 3};
        
        decimals = nd_1;
        if fmt == 2
            decimals = nd_2;
        elseif fmt == 0
             decimals = 0;
        end
        
        [numResults, stringResults] = addFieldAsNumAndString(numResults, stringResults, name, val, decimals);
    end
end
