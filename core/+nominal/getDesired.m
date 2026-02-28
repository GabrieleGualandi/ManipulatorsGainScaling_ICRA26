function [desired] = getDesired(par, t)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% GETDESIRED Retrieves the reference trajectory for the current time step.
% This function extracts position, orientation, velocity, and acceleration
% from the pre-computed par.desired structure. It handles both static 
% setpoints and time-varying trajectories.
%
% Inputs:
%   par     - Configuration structure.
%   t       - Current global time step index.
%
% Outputs:
%   desired - Structure containing target pose and higher-order derivatives.
%
% SEE ALSO: NOMINAL.CONTROL, CORESIMULATOR


    desired = struct();
    try
        if size(par.desired.posePosit,2) > 1 
            desired.desired_position = par.desired.posePosit(:,t);
        else 
            desired.desired_position = par.desired.posePosit;
        end
        
        if numel(par.desired.poseOri) > 1 
            desired.desired_orientation = par.desired.poseOri{t};
        else
            desired.desired_orientation = par.desired.poseOri{1};
        end
        
        if size(par.desired.velocity,2) > 1 
            desired.desired_spatial_velocity = par.desired.velocity(:,t);
        else
            desired.desired_spatial_velocity = par.desired.velocity;
        end
        
        if size(par.desired.acceleration,2) > 1 
            desired.desired_acceleration = par.desired.acceleration(:,t);
        else
            desired.desired_acceleration = par.desired.acceleration;
        end
    catch 
        % Hold last value if out of bounds
        desired.desired_position = par.desired.posePosit(:,end);
        desired.desired_orientation = par.desired.poseOri{end};
        desired.desired_spatial_velocity = par.desired.velocity(:,end);
        desired.desired_acceleration = par.desired.acceleration(:,end);
    end
end
