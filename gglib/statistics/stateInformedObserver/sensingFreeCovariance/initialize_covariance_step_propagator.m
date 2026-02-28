
function propagator_data = initialize_covariance_step_propagator(A_d, C_d, Q_d, R, P_inf)
% INITIALIZE_COVARIANCE_STEP_PROPAGATOR
%   Pre-computes the constant matrices (F, Pi) and the initial state (Pz0)
%   for a one-step covariance propagation.
%
% INPUTS:
%   A_d     : Discrete-time state matrix.
%   C_d     : Discrete-time output matrix.
%   Q_d     : Process noise covariance matrix.
%   R       : Measurement noise covariance matrix.
%   P_inf   : Steady-state Kalman filter error covariance P.
%
% OUTPUTS:
%   propagator_data : Structure containing the matrices F, Pi, Pz0, and state dimension n.

% --- Step 1: Compute Kalman Gain (L) and dimensions ---
n = size(A_d, 1);
L = (A_d * P_inf * C_d') / (C_d * P_inf * C_d' + R);

% --- Step 2: Define Augmented System Matrices F and G ---
F = [A_d - L * C_d,   zeros(n, n); 
     L * C_d,         A_d];
G = [eye(n),   -L; 
     zeros(n),  L];

% --- Step 3: Define Constant Noise Term Pi = G * Q_noise * G' ---
Pi = G * blkdiag(Q_d, R) * G';

% --- Step 4: Define Initial Covariance Pz0 ---
Pz0 = blkdiag(P_inf, zeros(n, n));

% --- Step 5: Package and return data ---
propagator_data = struct('n', n, 'F', F, 'Pi', Pi, 'Pz0', Pz0);
end