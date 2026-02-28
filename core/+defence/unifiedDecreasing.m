function f = unifiedDecreasing(Z, parIn, mode, phiType)
%UNIFIEDDECREASING Computes a smooth, strictly decreasing gain function.
%
%   f = unifiedDecreasing(Z, parIn, mode, phiType) calculates a gain factor
%   'f' based on an input 'Z' (anomaly score) and various design parameters.
%   The function 'f' is designed to be 1 at Z=0 and smoothly decrease
%   towards a terminal value 'L' as Z approaches infinity.
%
%   Inputs:
%   Z       : Scalar or vector, representing the anomaly score(s). Must be Z >= 0.
%   parIn   : Structure containing design parameters:
%             .L             : Terminal value (limit as Z->inf). Must be L < 1.
%             .z_x           : Design point for Z. Must be z_x > 0.
%             .z_y           : Desired value of f(z_x).
%             .expSteepness  : Shape exponent 'p' for 'exponential' phiType.
%                              Must be p > 0. (Equivalent to gamma in some contexts).
%   mode    : String, specifies how z_y is interpreted:
%             'pin'      : f(z_x) is exactly z_y. (z_y must be in (L, 1)).
%             'tolerance': f(z_x) is >= z_y. (z_y must be in (L, 1]).
%                          (Note: Current implementation for 'tolerance'
%                          calculates zs as if f(z_x) = z_y. For a true
%                          "at least" tolerance, a different zs calculation
%                          or a numerical optimization might be needed,
%                          or this mode implies z_y is the *minimum* value
%                          at z_x that is then "pinned" to).
%   phiType : String, specifies the mathematical form of the decreasing function:
%             'rational' : Uses a rational function form.
%             'exponential': Uses an exponential function form (as in your paper).
%
%   Output:
%   f       : Scalar or vector, the calculated gain factor(s) corresponding
%             to each element in Z. f will be in (L, 1].
%
%   Examples:
%     % Exponential type, pinned to 0.3 at z=5, terminal value 0, steepness 2
%     par.L = 0; par.z_x = 5; par.z_y = 0.3; par.expSteepness = 2;
%     Z_values = 0:0.1:10;
%     f_exp = unifiedDecreasing(Z_values, par, 'pin', 'exponential');
%     plot(Z_values, f_exp); title('Exponential Gain');
%
%     % Rational type, pinned to 0.5 at z=2, terminal value -0.1
%     par.L = -0.1; par.z_x = 2; par.z_y = 0.5; par.expSteepness = 1; % steepness not used for rational
%     f_rat = unifiedDecreasing(Z_values, par, 'pin', 'rational');
%     hold on; plot(Z_values, f_rat, '--'); title('Rational vs Exponential Gain');
%
%   See also: EXP, LOG

% --- Input Parsing and Validation ---
% Validate Z
if ~isnumeric(Z) || any(Z < 0)
    error('unifiedDecreasing:InvalidZ', 'Input Z must be a non-negative numeric scalar or vector.');
end

% Validate parIn structure fields
if ~isstruct(parIn)
    error('unifiedDecreasing:InvalidParIn', 'parIn must be a structure.');
end
if ~isfield(parIn, 'L') || ~isscalar(parIn.L) || ~isnumeric(parIn.L) || parIn.L >= 1
    error('unifiedDecreasing:InvalidL', 'parIn.L (terminal value) must be a numeric scalar < 1.');
end
if ~isfield(parIn, 'z_x') || ~isscalar(parIn.z_x) || ~isnumeric(parIn.z_x) || parIn.z_x <= 0
    error('unifiedDecreasing:InvalidZx', 'parIn.z_x (design point) must be a numeric scalar > 0.');
end
if ~isfield(parIn, 'z_y') || ~isscalar(parIn.z_y) || ~isnumeric(parIn.z_y)
    error('unifiedDecreasing:InvalidZy', 'parIn.z_y (value at z_x) must be a numeric scalar.');
end
if ~isfield(parIn, 'expSteepness') || ~isscalar(parIn.expSteepness) || ~isnumeric(parIn.expSteepness) || parIn.expSteepness <= 0
    error('unifiedDecreasing:InvalidExpSteepness', 'parIn.expSteepness (p) must be a numeric scalar > 0.');
end

% Validate mode
validModes = {'pin', 'tolerance'};
if ~ischar(mode) || ~ismember(lower(mode), validModes)
    error('unifiedDecreasing:InvalidMode', 'Mode must be ''pin'' or ''tolerance''.');
end
mode = lower(mode); % Convert to lowercase for case-insensitive comparison

% Validate phiType
validPhiTypes = {'rational', 'exponential'};
if ~ischar(phiType) || ~ismember(lower(phiType), validPhiTypes)
    error('unifiedDecreasing:InvalidPhiType', 'phiType must be ''rational'' or ''exponential''.');
end
phiType = lower(phiType); % Convert to lowercase

% Assign parameters for brevity
L = parIn.L;
z_x = parIn.z_x;
z_y = parIn.z_y;
p = parIn.expSteepness;

% --- Further validation based on mode and z_y ---
switch mode
    case 'pin'
        if ~(z_y > L && z_y < 1)
            error('unifiedDecreasing:PinZyRange', ...
                  'For ''pin'' mode, z_y must be strictly between L and 1 (i.e., L < z_y < 1).');
        end
    case 'tolerance'
        % The current implementation for 'tolerance' effectively pins f(z_x) = z_y.
        % If z_y is intended as a lower bound, the calculation might need revision.
        if ~(z_y > L && z_y <= 1)
            error('unifiedDecreasing:ToleranceZyRange', ...
                  'For ''tolerance'' mode, z_y must be between L and 1 (i.e., L < z_y <= 1).');
        end
end

% --- Compute zs (scaling parameter) from the chosen design constraint ---
% We need to find zs such that f(z_x) = z_y.
% Recall f(Z) = 1 + (L - 1) * phi(Z).
% So, z_y = 1 + (L - 1) * phi(z_x).
% This means phi(z_x) = (z_y - 1) / (L - 1).

switch phiType
    case 'rational'
        % phi(Z) = Z ./ (Z + zs)
        % At Z = z_x, phi(z_x) = z_x / (z_x + zs)
        % So, (z_y - 1) / (L - 1) = z_x / (z_x + zs)
        % Solving for zs:
        % zs = z_x * (L - z_y) / (z_y - 1)
        zs = z_x * (L - z_y) / (z_y - 1); % This matches your original derivation's form
                                           % which used 'z_y' as 'x' in z_x * (x - L) / (1 - x)
    case 'exponential'
        % phi(Z) = 1 - exp(-(Z./zs).^p)
        % At Z = z_x, phi(z_x) = 1 - exp(-(z_x./zs).^p)
        % So, (z_y - 1) / (L - 1) = 1 - exp(-(z_x./zs).^p)
        % Solving for zs:
        % zs = z_x / (-log((L - z_y) / (L - 1)))^(1/p)
        zs = z_x / (-log((L - z_y) / (L - 1)))^(1/p); % This matches your original derivation's form
                                                        % which used 'z_y' as 'x' in z_x / (-log((x - L)/(1 - L)))^(1/p)
end

% --- Evaluate phi(Z) ---
switch phiType
    case 'rational'
        phi = Z ./ (Z + zs);
    case 'exponential'
        phi = 1 - exp(-(Z./zs).^p);
end

% --- Final gain law f(Z) ---
% The unified form: f(Z) = 1 + (L - 1) * phi(Z)
f = 1 + (L - 1) .* phi;

end