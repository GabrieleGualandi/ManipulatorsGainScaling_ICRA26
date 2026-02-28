function [v_sat, hasSaturated, saturExcess] = saturateVector(v, maxNorm, normType, sharpness, epsMargin)
%SATURATEVECTOR Saturate a vector under (possibly anisotropic) magnitude limits.
%
%   [v_sat, hasSaturated, saturExcess] = saturateVector(v, maxNorm)
%   [v_sat, hasSaturated, saturExcess] = saturateVector(v, maxNorm, normType)
%   [v_sat, hasSaturated, saturExcess] = saturateVector(v, maxNorm, normType, sharpness)
%   [v_sat, hasSaturated, saturExcess] = saturateVector(v, maxNorm, normType, sharpness, epsMargin)
%
%   PURPOSE
%   Enforce amplitude limits on a command vector v.
%   Supports smooth (C^1) saturation laws suitable for differentiation.
%
%   INPUTS
%     v         : numeric vector (row or column).
%
%     maxNorm   : either
%                 - scalar M>0  (isotropic limit), or
%                 - vector M_i>0 of same length as v (anisotropic limits).
%               Requirement after saturation: |v_sat(i)| <= maxNorm(i).
%
%     normType  : one of
%
%        'infinity','inf'
%            Hard L_inf scaling (direction-preserving).
%            Compute r = max_i |v_i| / M_i. If r>1, scale by 1/r.
%
%        'euclidean','2'
%            Hard L2 saturation. Requires scalar maxNorm.
%            If ||v||_2 > maxNorm, scale by maxNorm/||v||_2.
%
%        'inf-sigmoid'
%            Component-wise smooth clamp using tanh.
%            v_sat(i) = M_i * tanh( sharpness * v_i / M_i ).
%            (Not direction-preserving.)
%
%        'inf-smoothstep'
%            Direction-preserving smooth L_inf-type clamp.
%            Let r = max_i |v_i|/M_i.
%            - If r <= a     : g=1.
%            - If r >= 1     : g=1/r (exact boundary).
%            - Otherwise     : cubic Hermite blend with C^1 transition,
%                              zero slope at r=1.
%            Here a = 1 - 1/(1+sharpness) in (0,1).
%
%        'inf-smoothstepDifferentiable'
%            Same as 'inf-smoothstep' but with a strict safety margin.
%            The vector is smoothly steered toward at most
%                  (1 - epsMargin)*maxNorm(i)
%            instead of exactly maxNorm(i).
%            Thus the actuator never mathematically reaches its true limit.
%            This avoids ever commanding the physical bound itself.
%
%     sharpness : (>0, default 1).
%        Controls how wide the linear (identity) region is for the smooth
%        modes. Larger sharpness ⇒ larger linear core before blending.
%
%     epsMargin : (optional, scalar, default 1e-3).
%        Safety margin for 'inf-smoothstepDifferentiable' mode.
%        Must be a small value in the range (0, 1).
%
%   OUTPUTS
%     v_sat         : saturated vector, same shape (row/col) as v.
%     hasSaturated  : boolean, true if any limiting/blending was applied.
%     saturExcess   : scalar diagnostic of "how far beyond" the admissible
%                     (or virtual-admissible) set the original v was.
%
%   NOTES
%   - Direction of v is preserved in all modes except 'inf-sigmoid'.
%   - For 'inf-smoothstepDifferentiable', the optional argument `epsMargin`
%     is used. The output never exactly equals the physical limit maxNorm, removing
%     hard contact with the actuator bound in closed-loop simulation.

    % ---------------- validation ----------------
    if nargin < 2
        error('Need at least v and maxNorm.');
    end
    if ~isvector(v) || ~isnumeric(v)
        error('v must be a numeric vector.');
    end
    if nargin < 3 || isempty(normType)
        normType = 'infinity';
    end
    if nargin < 4 || isempty(sharpness)
        sharpness = 1;
    end
    if ~isscalar(sharpness) || ~isnumeric(sharpness) || ~(sharpness > 0)
        error('sharpness must be a positive scalar.');
    end
    if nargin < 5 || isempty(epsMargin)
        epsMargin = 1e-3;
    end
    if ~isscalar(epsMargin) || ~isnumeric(epsMargin) || ~(epsMargin > 0 && epsMargin < 1)
        error('epsMargin must be a scalar in the range (0, 1).');
    end

    % remember original orientation for output
    v_is_row = isrow(v);
    v = v(:);
    n = numel(v);

    % interpret maxNorm
    if isscalar(maxNorm)
        Mvec = repmat(maxNorm, n, 1);
        isScalarBound = true;
    else
        Mvec = maxNorm(:);
        if numel(Mvec) ~= n
            error('If maxNorm is a vector, it must match length(v).');
        end
        isScalarBound = false;
    end

    % precompute usage ratios w.r.t. anisotropic bounds
    ratios = abs(v) ./ Mvec;
    r_rel  = max(ratios);  % worst-case relative usage (>=0)

    % ---------------- saturation modes ----------------
    switch lower(normType)

        % --- Hard L2 saturation (direction-preserving). ------------------
        case {'euclidean','2'}
            if ~isScalarBound
                error('For "euclidean", maxNorm must be scalar.');
            end
            nrm2 = norm(v,2);
            if nrm2 <= maxNorm || nrm2 == 0
                v_sat = v;
                hasSaturated = false;
                saturExcess  = 0;
            else
                g = maxNorm / nrm2;
                v_sat = g * v;
                hasSaturated = true;
                saturExcess  = nrm2 - maxNorm;
            end

        % --- Hard L_inf scaling (direction-preserving). ------------------
        case {'infinity','inf'}
            if r_rel <= 1 || r_rel == 0
                g = 1.0;
            else
                g = 1 / r_rel;
            end
            v_sat = g * v;
            hasSaturated = (r_rel > 1);
            if isScalarBound
                saturExcess = max(0, norm(v,inf) - maxNorm);
            else
                saturExcess = max(0, r_rel - 1);
            end

        % --- Component-wise smooth tanh clamp. ---------------------------
        case 'inf-sigmoid'
            s = sharpness;
            v_sat = Mvec .* tanh( s * (v ./ Mvec) );
            % diagnostics
            hasSaturated = any(abs(v) > Mvec + eps);
            if isScalarBound
                saturExcess = max(0, norm(v,inf) - maxNorm);
            else
                saturExcess = max(0, r_rel - 1);
            end

        % --- C^1 smoothstep toward exact boundary. -----------------------
        case 'inf-smoothstep'
            g = local_gainSmoothstep(r_rel, sharpness);
            v_sat = g * v;

            hasSaturated = (r_rel >= 1);
            if isScalarBound
                saturExcess = max(0, norm(v,inf) - maxNorm);
            else
                saturExcess = max(0, r_rel - 1);
            end

        % --- C^1 smoothstep toward an interior "virtual" boundary. -------
        %     Never exactly reaches |v_i| = M_i, but approaches
        %     (1 - epsMargin)*M_i.
        case 'inf-smoothstepdifferentiable'
            % epsMargin is now a function argument with a default
            g = local_gainSmoothstepEps(r_rel, sharpness, epsMargin);
            v_sat = g * v;

            % "saturation" here means we had to invoke the margin
            r_virtual = (1 - epsMargin);
            hasSaturated = (r_rel >= r_virtual - 1e-12);

            if isScalarBound
                virtualBound = (1 - epsMargin) * maxNorm;
                saturExcess  = max(0, norm(v,inf) - virtualBound);
            else
                saturExcess  = max(0, r_rel - r_virtual);
            end

        otherwise
            error('Unsupported normType "%s".', normType);
    end

    % restore row shape if needed
    if v_is_row
        v_sat = v_sat.';
    end
end


% ====== Local helper: classic smoothstep radial gain =====================
function g = local_gainSmoothstep(r_rel, sharpness)
% C^1 direction-preserving limiter to the TRUE boundary.
%
% r_rel = max_i |v_i|/M_i  (>=0)
% a     = linear-core radius in (0,1).
%
% Piecewise:
%   if r <= a:   g = 1                          (identity region)
%   if r >= 1:   g = 1/r                        (exactly on boundary)
%   else:        cubic Hermite blend on [a,1]
%                y(a)=a, y'(a)=1,
%                y(1)=1, y'(1)=0,
%                g = y(r)/r.

    if r_rel == 0
        g = 1.0;
        return;
    end

    s = sharpness;
    a = 1 - 1/(1 + s);                 % in (0,1)
    a = min(max(a, 1e-12), 1 - 1e-12); % numerical safety

    r = r_rel;

    if r <= a
        g = 1.0;

    elseif r >= 1
        g = 1.0 / r;

    else
        % Hermite interpolation y(r) on [a,1]
        t   = (r - a) / (1 - a);           % t in [0,1]
        H00 =  2*t^3 - 3*t^2 + 1;
        H01 = -2*t^3 + 3*t^2;
        H10 =      t^3 - 2*t^2 + t;
        m0  = (1 - a);                     % slope at r=a is 1
        y   = H00*a + H01*1 + H10*m0;      % y(a)=a, y(1)=1, y'(1)=0
        g   = y / r;
    end
end


% ====== Local helper: smoothstep with interior margin ====================
function g = local_gainSmoothstepEps(r_rel, sharpness, epsMargin)
% Same construction as local_gainSmoothstep, but we never allow y(1)=1.
% Instead:
%   y(1) = 1 - epsMargin   (<1),
% giving an interior "virtual" boundary. For r>=1 we scale to
% (1 - epsMargin)/r instead of 1/r.
%
% Result: the actuator command never exactly equals its physical bound.

    if r_rel == 0
        g = 1.0;
        return;
    end

    s  = sharpness;
    a  = 1 - 1/(1 + s);                 % in (0,1)
    a  = min(max(a, 1e-12), 1 - 1e-12); % numerical safety
    y1 = 1 - epsMargin;                 % virtual boundary (<1)

    r = r_rel;

    if r <= a
        g = 1.0;

    elseif r >= 1
        g = y1 / r;                     % saturate to strictly inside bound

    else
        % Hermite interpolation y(r) on [a,1], with:
        %   y(a)=a, y'(a)=1,
        %   y(1)=y1 (<1), y'(1)=0.
        t   = (r - a) / (1 - a);        % t in [0,1]
        H00 =  2*t^3 - 3*t^2 + 1;
        H01 = -2*t^3 + 3*t^2;
        H10 =      t^3 - 2*t^2 + t;
        m0  = (1 - a);                  % slope at r=a is 1
        y   = H00*a + H01*y1 + H10*m0;  % y smoothly transitions to y1
        g   = y / r;
    end
end