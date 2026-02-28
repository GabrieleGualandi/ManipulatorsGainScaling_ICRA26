function sysOut = secondOrderGainSmoother(params, q0, qdot0, Ts, nComp)
%secondOrderGainSmoother Creates a (possibly multi-channel) 2nd-order smoother.
%
%   SYNOPSIS:
%       sysOut = secondOrderGainSmoother(params, q0, qdot0, Ts)
%       sysOut = secondOrderGainSmoother(params, q0, qdot0, Ts, nComp)
%
%   DESCRIPTION:
%       Constructs a discrete-time, linear, second-order system modeling a
%       mass–spring–damper, to act as a low-pass "gain smoother". The filter
%       is normalized to have STATIC GAIN = 1, i.e. steady-state output
%       equals steady-state input.
%
%       The system can be replicated across multiple independent channels.
%       Each channel is dynamically identical and uncoupled. This is useful
%       when smoothing multiple gain signals (one per joint, etc.).
%
%   INPUTS:
%       params   (struct)
%           Physical parameters for the mass–spring–damper model.
%           Must contain at least:
%               'mass', 'spring', 'damp' (or equivalent fields expected by
%               plantMassSpringDamp).
%
%       q0       (scalar or vector [nComp x 1])
%           Initial position(s) of the filter output(s).
%           If scalar and nComp>1, it will be replicated.
%
%       qdot0    (scalar or vector [nComp x 1])
%           Initial rate(s) of change of the filter output(s).
%           If scalar and nComp>1, it will be replicated.
%
%       Ts       (scalar)
%           Sampling time [s].
%
%       nComp    (scalar, optional, default = 1)
%           Number of independent channels to smooth. Each channel is a
%           copy of the same critically damped 2nd-order cell.
%
%   OUTPUT:
%       sysOut
%           Simulator object returned by secondOrderLinearSimulator, built
%           from the block-diagonal lifted system (size 2*nComp states).
%
%   NOTE ON DAMPING:
%       Critical damping is achieved when
%           damp = 2 * sqrt(mass * spring).
%       This yields fastest settling without overshoot.
%
%   See also: ss, stepinfo, plantMassSpringDamp,
%             secondOrderLinearSimulator
%
%   ---------------------------------------------------------------------

    % ----- Defaults -----------------------------------------------------
    if nargin < 5 || isempty(nComp)
        nComp = 1;
    end

    % ----- Build single-channel continuous & discrete model ------------
    gaintracker = struct();
    gaintracker.sys.meta = params;
    gaintracker = plantMassSpringDamp(gaintracker);

    % Extract base (single-channel) discrete-time matrices
    A1 = gaintracker.sys.dis.A;    % 2x2
    B1 = gaintracker.sys.dis.B;    % 2x1
    C1 = gaintracker.sys.dis.C;    % 1x2
    D1 = gaintracker.sys.dis.D;    % 1x1

    % Enforce unit static gain: scale input channel(s)
    staticGain = gaintracker.sys.info.cont.staticGain;
    B1 = B1 * (1/staticGain);      % still 2x1

    % ----- Replicate to nComp independent channels ---------------------
    % State dimension per channel = 2 (q, qdot).
    % Total state dimension        = 2*nComp.
    %
    % A_big: (2n x 2n) block-diagonal of A1
    % B_big: (2n x n)  block-diagonal of B1
    % C_big: (n  x 2n) block-diagonal of C1
    % D_big: (n  x n)  block-diagonal of D1
    %
    I_n   = eye(nComp);
    A_big = kron(I_n, A1);
    B_big = kron(I_n, B1);
    C_big = kron(I_n, C1);
    D_big = kron(I_n, D1);

    % ----- Initial conditions ------------------------------------------
    % q0, qdot0 must be nComp-by-1 vectors. If scalars are provided,
    % replicate them.
    if isscalar(q0)
        q0_vec = repmat(q0,    nComp, 1);
    else
        q0_vec = q0(:);
        if numel(q0_vec) ~= nComp
            error('q0 must be scalar or length nComp (%d).', nComp);
        end
    end

    if isscalar(qdot0)
        qdot0_vec = repmat(qdot0, nComp, 1);
    else
        qdot0_vec = qdot0(:);
        if numel(qdot0_vec) ~= nComp
            error('qdot0 must be scalar or length nComp (%d).', nComp);
        end
    end

    % ----- Build simulator object --------------------------------------
    % Assumption: secondOrderLinearSimulator accepts vector ICs for q and qdot
    % and constructs the stacked state x0 = [q0_vec; qdot0_vec].
    sysOut = defence.secondOrderLinearSimulator( ...
        A_big, B_big, C_big, D_big, ...
        q0_vec, qdot0_vec);

    % ----- Informative settling-time report (single-channel dynamics) ---
    % We report the settling time of a single channel, since all channels
    % are identical copies.
    sysSingle = ss(A1, B1, C1, D1, Ts);
    info      = stepinfo(sysSingle);
    settlingTime = info.SettlingTime;

    fprintf('Settling time (per channel): %.2f s (%.0f samples)\n', ...
        settlingTime, settlingTime/Ts);
end
