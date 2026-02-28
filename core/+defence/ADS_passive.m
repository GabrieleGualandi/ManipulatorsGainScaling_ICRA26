function [ADS_z, ADS_r, hasAlarmFired] = ADS_passive(par, st, y_tilde, isSimulation, hasAlarmFired)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% ADS_PASSIVE Simulates the Passive Anomaly Detection System (ADS).
% This function calculates the residuals between corrupted measurements 
% and observer estimates, computing a chi-squared anomaly metric (ADS_z).
% It also monitors stealthiness constraints from the attacker's perspective.
%
% Inputs:
%   par           - Configuration structure.
%   st            - Current SimulatorState object.
%   y_tilde       - Corrupted measurement vector at time k.
%   isSimulation  - Boolean flag for lookahead runs (disables warnings).
%   hasAlarmFired - Current boolean state of the detection alarm.
%
% Outputs:
%   ADS_z         - Computed anomaly metric (Squared Mahalanobis distance).
%   ADS_r         - Residual vector (raw difference).
%   hasAlarmFired - Updated boolean state of the alarm.
%
% SEE ALSO: ADS_ACTIVE, DEFENCE.PDSCALING, DEFENCE.ACTIVEDISSIPATION


    useChiSquared = par.def.passive.scheme.useChiSquared;
    checkStealthiness = par.attack.use_stealth_constraint;

    ADS_z = nan;
    ADS_r = nan(numel(st.sys_real_q),1);
    
    if useChiSquared
        if par.def.passive.scheme.useChiSquared
            ADS_y_hat = par.models.sys_observer.C * [st.sys_observer_q_hat; st.sys_observer_qdot_hat];
            ADS_r = y_tilde - ADS_y_hat;
            
            if ~isSimulation && checkStealthiness
                ADS_z = ADS_r' * par.def.Sigma_prime * ADS_r;
                if (ADS_z - par.attack.tau_prime > 10e-05) && ~hasAlarmFired
                    warning('Stealthiness constraint violated: %s', num2str(ADS_z - par.attack.tau_prime));
                    hasAlarmFired = true;
                end
            end
        end
    end
end
