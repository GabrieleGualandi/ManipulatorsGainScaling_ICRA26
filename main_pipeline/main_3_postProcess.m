function simulation = main_3_postProcess(simulation)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% main_3_postProcess   Calculates statistics and metrics from simulation logs.
% This function processes the simulation data to compute energy, power,
% trajectory errors, and other performance metrics for evaluation.
%
% Inputs:
%   simulation - Structure containing simulation 'log' and 'par'.
%
% Outputs:
%   simulation - The input structure augmented with calculated metrics.
%
% SEE ALSO: MAIN_2_RUNSIMULATION, MAIN_4_PLOTGRAPHS



    par = simulation.par;
    
    % Samples indexing
    simulation.log.samples = 1:size(simulation.log.sys_real_q,2);

    %% 1. Saturating joint analysis
    A = simulation.log.ddot_q_final;
    [m, n] = size(A);
    saturIndexes = zeros(m, n);        % preallocate
    for j = 1:n
        saturIndexes(:,j) = abs(A(:,j)) >= simulation.par.robot.UMAX_afterFeedbackLin *0.95;   % fun: R^m -> R
    end
    
    rowMask = any(saturIndexes, 2);      % logical vector: true for rows containing at least one 1
    saturatingIdx  = find(rowMask);      % numeric indices of those rows
    if ~(isempty(saturatingIdx))
        fprintf('In the simulation, you have near saturation in joints %s\n', strjoin(string(saturatingIdx.'), ', ') );
    end

    %% 2. Basic Norms
    simulation.log = addNorm2Stats(simulation.log, 'hand', simulation.log.handGeneralizedVelocity_task);

    % Hand deviation from setpoint squared
    simulation.log.deviation_HandP = par.desired.posePosit(par.task.taskP,:) - simulation.log.handPosition_task;
    simulation.log = addNorm2Stats(simulation.log, 'deviation_HandP', simulation.log.deviation_HandP);

    simulation.log = addNorm2Stats(simulation.log, 'sys_real_qdot', simulation.log.sys_real_qdot);



    %% 3. Maximum Impact & Actuation
    % Max Attack impact
    X_att = simulation.log.handPosition_task;
    d_inf = vecnorm(X_att - X_att(:,1), Inf, 1);   % 1×n, with d_inf(1)=0
    simulation.log.maxAttackImpact = max(d_inf);

    % Max actuation
    X_act = simulation.log.ddot_q_final;
    d_inf_act = vecnorm(X_act - X_act(:,1), Inf, 1);   % 1×n, with d_inf(1)=0
    simulation.log.maxActuationDiff = max(d_inf_act);

    %% 4. Energy Analysis
    % Kinetic energy
    simulation.log.kinNrg = 0.5 * (simulation.log.sys_real_qdot.^2);
    simulation.log.kinNrgTot = sum(simulation.log.kinNrg,1);
    simulation.log.kinNrgTotAvg = mean(simulation.log.kinNrgTot);
    simulation.log.kinNrgTotMax = max(simulation.log.kinNrgTot);

    %% 5. Power Analysis (Norm 1 Stats)
    % Final Power (Control + External)
    simulation.log = addNorm1Stats(simulation.log, 'power_final', simulation.log.ddot_q_final .* simulation.log.sys_real_qdot);

    % Gain Scaling analysis
    if isfield(par.def.active.scheme, 'usePDgainsScaling') && par.def.active.scheme.usePDgainsScaling
        simulation.log.gs_min = min(simulation.log.gainScaling_q);
        simulation.log.gs_avg = mean(simulation.log.gainScaling_q);
        simulation.log.gs_max = max(simulation.log.gainScaling_q);
    end

    %% 6. Trajectory Metrics (Hand Precision)
    % Defender task hand error
    simulation.log = addTrajectoryMetrics(simulation.log, 'hand', par.desired.posePosit(par.task.taskP,:), simulation.log.handPosition_task);

    % Attacker task hand error (if applicable)
    if simulation.par.attack.scheme.useGreedyPD
        simulation.log = addTrajectoryMetrics(simulation.log, 'att', par.attack.p_goal, simulation.log.handPosition_task);
    end

    %% 7. Deviation Analysis (Norm 2 Stats)
    % Sensing-Free Projected vs Real, Previously called deviation_xParaxReal
    simulation.log.deviation_xSFPxReal = [simulation.log.sys_real_q; simulation.log.sys_real_qdot] - [simulation.log.sys_sensingFreeProjected_q; simulation.log.sys_sensingFreeProjected_qdot];
    simulation.log = addNorm2Stats(simulation.log, 'deviation_xSFPxReal', simulation.log.deviation_xSFPxReal);

    % Estimated vs Real
    simulation.log.deviation_xEstxReal = [simulation.log.sys_real_q; simulation.log.sys_real_qdot] - [simulation.log.sys_observer_q_hat; simulation.log.sys_observer_qdot_hat];
    simulation.log = addNorm2Stats(simulation.log, 'deviation_xEstxReal', simulation.log.deviation_xEstxReal);

    % Hand precision (position) wrt authentic task
    % (Norm 2 stats already added in section 2)

    % Hand precision (position) wrt attacker task
    if par.attack.scheme.useGreedyPD
        simulation.log.attackerDeviation_HandP = par.attack.p_goal - simulation.log.handPosition_task;
        simulation.log = addNorm2Stats(simulation.log, 'attackerDeviation_HandP', simulation.log.attackerDeviation_HandP);
    end


    %% 10. Joint Frame Positions Over Time
    % Reconstruct the [x,y] position of each joint frame at every timestep.
    % Produces fields frameNPos (2 x Nsteps) in simulation.log, matching the
    % naming convention expected by the robot-trajectory LaTeX image.
    if isfield(par, 'funcs') && isfield(par.funcs, 'HT_funcs') && ~isempty(par.funcs.HT_funcs)
        framesPos = jointFramesOverTime(simulation.log.sys_real_q, par.funcs.HT_funcs, par.task.taskP);
        for nn = 1:numel(framesPos)
            simulation.log.(['frame', num2str(nn), 'Pos']) = framesPos{nn};
        end
    end


    % Standard plotting variables
    simulation.log.time = (0 : size(simulation.log.sys_real_q, 2) - 1) * par.sim.Ts;
    simulation.log.q_realMINUSq_hat = simulation.log.sys_real_q - simulation.log.sys_observer_q_hat;
    simulation.log.qd_realMINUSq_hat = simulation.log.sys_real_qdot - simulation.log.sys_observer_qdot_hat;
    simulation.log.positionError = simulation.log.p_goal - simulation.log.handPosition_task;
    % previously called pgoalMINUSp
    simulation.log = addNorm2Stats(simulation.log, 'positionError', simulation.log.positionError);

    %% 11. Final Values & Path Length
    simulation.log = addNorm2Stats(simulation.log, 'u_final', simulation.log.ddot_q_final);
    simulation.log.FinalNorm_sys_real_qdot = norm(simulation.log.sys_real_qdot(:,end));
    simulation.log.FinalNorm_HandVelocity = norm(simulation.log.handGeneralizedVelocity_task(:,end));
    simulation.log.FinalNorm_u_max = simulation.log.u_final_l2_max;


    speed = simulation.log.hand_l2;                           % 1 x N
    L_trapz = sum( 0.5 * (speed(1:end-1) + speed(2:end)) ) * par.sim.Ts;
    simulation.log.handPathLength = L_trapz;

end
