function Pz_next = propagate_covariance_onestep(propagator_data, Pz_current)
% PROPAGATE_COVARIANCE_ONESTEP
%   Performs a single-step update of the augmented state covariance matrix
%   using the discrete-time Lyapunov equation.
%
% INPUTS:
%   propagator_data : Structure from initialize_covariance_step_propagator.
%   Pz_current      : The current (k) augmented covariance matrix [2n x 2n].
%
% OUTPUTS:
%   Pz_next         : The next (k+1) augmented covariance matrix [2n x 2n].

% Unpack the constant matrices
F = propagator_data.F;
Pi = propagator_data.Pi;

% Apply the one-step Lyapunov update rule
Pz_next = F * Pz_current * F' + Pi;

end