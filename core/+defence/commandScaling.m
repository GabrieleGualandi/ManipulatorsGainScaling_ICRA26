function f = commandScaling(Z, parIn, mode, phiType)
%COMMANDSCALING Computes a smooth, strictly decreasing command scaling gain.
%
%   f = commandScaling(Z, parIn, mode, phiType) calculates a gain factor 'f'
%   based on an anomaly score 'Z'.
%
%   Inputs:
%   Z       : Scalar or vector, representing the anomaly score(s). Must be Z >= 0.
%   parIn   : Structure containing design parameters:
%             .z_x           : Design point for Z. Must be z_x > 0.
%             .z_y           : Desired value of f(z_x). Must be (0 < z_y < 1).
%             .expSteepness  : Shape exponent (\gamma or 'p'). Must be p > 0.
%             (.L is ignored).
%   mode    : Ignored (forced 'pin' behavior conceptually).
%   phiType : Ignored (forced 'exponential' behavior conceptually).
%
%   Output:
%   f       : Scalar or vector, the calculated gain factor(s) corresponding
%             to each element in Z. f will be in (0, 1].

if ~isnumeric(Z) || any(Z < 0)
    error('commandScaling:InvalidZ', 'Input Z must be a non-negative numeric scalar or vector.');
end

z_x = parIn.z_x;
z_y = parIn.z_y;
p = parIn.expSteepness;

if ~(z_y > 0 && z_y < 1)
    error('commandScaling:ZyRange', 'z_y must be strictly between 0 and 1 (i.e., 0 < z_y < 1).');
end

% Explicit scaling parameter z_scale derived from f(z_x) = z_y
z_scale = z_x / (-log(z_y))^(1/p);

% Final explicit formulation: f(Z) = exp(-(Z / z_scale)^p)
f = exp(-(Z ./ z_scale).^p);

end
