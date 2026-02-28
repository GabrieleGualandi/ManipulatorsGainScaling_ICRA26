function HT_positions = jointFramesOverTime(INq, INfuns, components)
% jointFramesOverTime - Tracks the selected components of joint positions over time.
%
% Inputs:
%   INq        : [n x T] matrix, where each column is a joint configuration at time t.
%   INfuns     : {1 x n} cell array of function handles. INfuns{i}(q) returns a 4x4 HT matrix for joint i.
%   components : (optional) vector of indices (e.g., [1,2]) selecting which position components to extract.
%                Default is [1,2,3] (all components).
%
% Output:
%   HT_positions : {1 x n} cell array. Each cell contains a [length(components) x T] matrix
%                  with the selected components of the joint i-th frame over time.

    if nargin < 3 || isempty(components)
        components = 1:3;
    end

    [n, T] = size(INq);
    HT_positions = cell(1, n);

    for i = 1:n
        HT_positions{i} = zeros(length(components), T);
    end

    for t = 1:T
        qt = INq(:, t); % Joint configuration at time t
        for i = 1:n
            HT_i = INfuns{i}(qt);              % 4x4 HT matrix
            pos_i = HT_i(1:3, 4);              % Full 3D position
            HT_positions{i}(:, t) = pos_i(components);
        end
    end
end