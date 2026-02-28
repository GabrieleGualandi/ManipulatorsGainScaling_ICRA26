function idx = findUpCrossings(values, threshold)
%FINDUPCROSSINGS Indices where a signal crosses upward over a threshold.
%
%   idx = FINDUPCROSSINGS(values, threshold)
%
%   Inputs:
%     values    : vector (row or column) of numeric values.
%     threshold : scalar. Crossing level (e.g., 1).
%
%   Output:
%     idx       : column vector of indices k such that
%                 values(k-1) < threshold and values(k) >= threshold.
%                 (i.e., the first index at/above the threshold
%                  after being below it).
%
%   Example:
%     v = [0.2 0.5 0.9 1.1 0.8 1.0 1.2 0.9];
%     idx = findUpCrossings(v, 1);
%     % idx = [4; 6]
%     % Explanation: 0.9 -> 1.1 (3->4), and 0.8 -> 1.0 (5->6).

    if ~isvector(values)
        error('findUpCrossings:InputNotVector', ...
              'Input "values" must be a vector.');
    end

    values = values(:);  % work in column form internally

    % Logical condition for up-crossings:
    % previous sample < threshold, current sample >= threshold
    crossings = (values(1:end-1) < threshold) & (values(2:end) >= threshold);

    % Indices of the *second* sample in each crossing pair
    idx = find(crossings) + 1;
end
