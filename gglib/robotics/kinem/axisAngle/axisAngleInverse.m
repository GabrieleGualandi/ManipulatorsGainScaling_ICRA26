%==========================================================================
% Author: Gabriele Gualandi     Date: 2024     Ver: 1.11
% Mälardalens University - Course in Industrial Robotics (DVA400)
%==========================================================================
function [axis1, angle1, axis2, angle2] = axisAngleInverse(R)
%AXISANGLEINVERSE Robust computation of axis–angle representations from a rotation matrix.
%
%   [axis1, angle1, axis2, angle2] = axisAngleInverse(R)
%
%   Inputs:
%     R       : 3x3 rotation matrix (orthogonal with det = +1)
%
%   Outputs:
%     axis1, angle1 : primary axis–angle pair (theta in [0, pi])
%     axis2, angle2 : secondary equivalent pair (theta in [-pi, 0])
%
%   Notes:
%     - Automatically handles cases where angle ~ 0 or pi.
%     - axis1 and axis2 are unit vectors.

    % Parameters
    tol = 1e-8;

    % Check input
    assert(all(size(R) == [3 3]), 'R must be 3x3.');
    assert(abs(det(R) - 1) < 1e-6, 'Input R must have det(R) = +1.');

    % Compute skew-symmetric component norm
    skewPart = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    argument1 = norm(skewPart);
    traceR = R(1,1)+R(2,2)+R(3,3); % trace(R) but more efficient
    argument2 = traceR - 1;

    % Compute angles using atan2 formulation
    angle1 = atan2(argument1, argument2); % in [0, pi]
    angle2 = atan2(-argument1, argument2); % in [-pi, 0]

    % Handle angle near 0
    if abs(angle1) < tol
        angle1 = 0;
        angle2 = 0;
        axis1 = [1; 0; 0]; % arbitrary
        axis2 = axis1;
        return;
    end

    % Handle angle near pi
    if abs(angle1 - pi) < tol
        angle1 = pi;
        angle2 = -pi;

        % Extract axis from (R + I)/2
        K = (R + eye(3)) / 2;
        % Take square roots of diagonal
        k_sq = max(0, diag(K)); % clamp to avoid complex sqrt
        [~, idx] = max(k_sq);
        axis1 = zeros(3,1);
        axis1(idx) = sqrt(k_sq(idx));

        % Recover other components from off-diagonal signs
        if idx == 1
            axis1(2) = signOrOne(R(1,2)) * sqrt(k_sq(2));
            axis1(3) = signOrOne(R(1,3)) * sqrt(k_sq(3));
        elseif idx == 2
            axis1(1) = signOrOne(R(1,2)) * sqrt(k_sq(1));
            axis1(3) = signOrOne(R(2,3)) * sqrt(k_sq(3));
        else
            axis1(1) = signOrOne(R(1,3)) * sqrt(k_sq(1));
            axis1(2) = signOrOne(R(2,3)) * sqrt(k_sq(2));
        end
        axis1 = axis1 / norm(axis1);
        axis2 = axis1;
        return;
    end

    % General case: compute axis
    axis1 = skewPart / (2*sin(angle1));
    axis1 = axis1 / norm(axis1);
    axis2 = -axis1;

end

function s = signOrOne(v)
    % returns sign of v (but 1 if v is zero)
    if v >= 0
        s = 1;
    else
        s = -1;
    end
end

