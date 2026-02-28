function K2 = enlargeKInterval(K, varargin)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% ENLARGEKINTERVAL Enlarge an exponent grid K by dilating endpoints and densifying.
% This utility is used to adapt the search range for numerical Jacobians when 
% the current range is insufficient to find a valid numerical gradient.
%
%   K2 = enlargeKInterval(K) makes the min 50% smaller and the max 50% larger,
%   then increases the number of samples. If K is scalar c, the new bounds are
%   [0.5*c, 1.5*c]. By default, negative exponents are removed (clamped to 0).
%
% Inputs:
%   K        - Vector of exponents in h = baseStep .* 10.^K.
%   varargin - Name-Value pairs:
%              'LowerScale' (0.5)      - Multiplier for current min.
%              'UpperScale' (1.5)      - Multiplier for current max.
%              'GrowCount' (1.5)       - Multiplier for #points.
%              'MinCount' (5)          - Minimum #points after growth.
%              'SnapToInteger' (true)  - Only if original had >=2 integer points.
%              'NonNegative' (true)    - Enforce K2 >= 0.
%
% Outputs:
%   K2       - Vector of enlarged and densified exponents.
%
% SEE ALSO: ATTACK.USEGREEDYPD.RUN, NUMJACOBIANVARDELTA

    p = inputParser;
    p.addRequired('K', @(x)isnumeric(x) && isvector(x) && all(isfinite(x)));
    p.addParameter('LowerScale',    0.5, @(x)isnumeric(x) && isscalar(x) && x>0);
    p.addParameter('UpperScale',    1.5, @(x)isnumeric(x) && isscalar(x) && x>0);
    p.addParameter('GrowCount',     1.5, @(x)isnumeric(x) && isscalar(x) && x>1);
    p.addParameter('MinCount',      5,   @(x)isnumeric(x) && isscalar(x) && x>=2);
    p.addParameter('SnapToInteger', false, @(x)islogical(x) && isscalar(x));
    p.addParameter('NonNegative',   true, @(x)islogical(x) && isscalar(x));
    p.parse(K, varargin{:});

    a  = min(K(:));                 % current min
    b  = max(K(:));                 % current max
    Na = numel(K);

    % --- Dilate endpoints multiplicatively ---
    a2 = a * p.Results.LowerScale;
    b2 = b * p.Results.UpperScale;

    % Ensure a2 <= b2
    if a2 > b2
        tmp = a2; a2 = b2; b2 = tmp;
    end

    % --- Enforce non-negativity (if requested) ---
    if p.Results.NonNegative
        a2 = max(0, a2);
        b2 = max(0, b2);
        if a2 == b2
            % If clamping collapsed the interval, give it a small width
            if b2 == 0
                b2 = 1; % 1 decade by default
            else
                b2 = b2 * 1.001;
            end
        end
    end

    % --- Decide number of points ---
    if Na >= 2
        Nnew = max(p.Results.MinCount, 1 + ceil((Na-1)*p.Results.GrowCount));
    else
        % Scalar: grow to at least MinCount
        Nnew = p.Results.MinCount;
    end

    % --- Build continuous grid ---
    K2 = linspace(a2, b2, Nnew);

    % --- Integer snapping only if input looked like an integer grid with >=2 pts ---
    if p.Results.SnapToInteger && Na >= 2
        isIntGrid = all(abs(K(:) - round(K(:))) < 1e-12);
        if isIntGrid
            K2 = round(K2);
            % Ensure uniqueness and monotonicity; refill to MinCount if collapsed
            K2 = unique(K2(:).', 'stable');
            if numel(K2) < p.Results.MinCount
                if numel(K2) >= 2
                    K2 = K2(1):K2(end);
                else
                    % Rebuild integer grid from lower to upper
                    K2 = ceil(a2):floor(b2);
                    if numel(K2) < p.Results.MinCount
                        if isempty(K2), K2 = 0:(p.Results.MinCount-1);
                        else
                            step = 1;
                            while numel(K2) < p.Results.MinCount
                                K2 = [K2, K2(end)+step]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
    end

    % Final tidy
    K2 = K2(:).';  % row vector
end
