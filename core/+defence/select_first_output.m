%
% SELECT_FIRST_OUTPUT Helper function to capture only the first output of a function.
% Useful for capturing the first output of a multi-output function in anonymous 
% function handles or arrayfun/cellfun operations.
function first = select_first_output(varargin)
    first = varargin{1};
end
