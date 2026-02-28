function varargout = perturbationG(par, st, outvars, y_a_k, timeVars, tNow)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% PERTURBATIONG Run a short lookahead simulation to evaluate future quantities.
% This function executes a 3-step lookahead using coreSimulator to predict 
% how a specific attack increment affects simulated outputs (e.g., pose, velocity).
% It assumes the attacker has full access to the current state.
%
% Inputs:
%   par      - Structure containing project parameters.
%   st       - Current SimulatorState object (cloned internally to avoid side effects).
%   outvars  - Cell array of log field names to extract (e.g., {'handPosition_task'}).
%   y_a_k    - Total attack vector to inject at the current step.
%   timeVars - Integer offsets (1..3) defining which lookahead step to extract for each outvar.
%   tNow     - Current global simulation time index.
%
% Outputs:
%   varargout - Predicted values for each requested output variable at specified offsets.
%
% Note: This function simplifies the attacker's knowledge by passing the true 
% state 'st' directly into the lookahead simulation.
%
% SEE ALSO: CORESIMULATOR, ATTACK.USEGREEDYPD.RUN

Nsteps = 3;
% Construct attack value for override (replaces par.attack.value modification)
% We assume a greedy strategy where we only optimize the attack at the current step (k).
% Future attack values (at k+1, k+2...) in the lookahead horizon are set to zero
% (they are uninfluentual given the considered dynamics unfolding).
yAttackSequence = [ 
    y_a_k, ...
    zeros(size(y_a_k,1), Nsteps-1)...
    ]; 

self = struct('isSimulation',true,'isJacobian',true,'tNow',tNow);
self.attackValue = yAttackSequence;

% Clone state to avoid side effects
st_clone = st.clone();
log = coreSimulator(par,st_clone,Nsteps,self);

varargout = cell(1,numel(outvars));
for i = 1:numel(outvars)
    if timeVars(i) > 3
        error('This function only envisages 3 steps ahead!');
    end
    outDataAll = log.(outvars{i});
    varargout{i} = outDataAll(:,timeVars(i));
end

end
