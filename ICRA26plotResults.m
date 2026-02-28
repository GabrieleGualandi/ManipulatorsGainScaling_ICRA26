function ICRA26plotResults(resultsDir)
% ICRAplot  Images for the paper ICRA.
%
%   ICRAplot(resultsDir) loads data from the specified folder (default: results/ICRA26)
%   and generates the images for the paper.

    if nargin < 1
        resultsDir = fullfile('results', 'ICRA26');
    end

    fileU   = fullfile(resultsDir, 'NoDefence_NoADS_YesAttack.mat');
    filePO  = fullfile(resultsDir, 'NoDefence_YesAttack.mat');
    fileD   = fullfile(resultsDir, 'YesDefence_YesAttack.mat');

    logU  = loadLog(fileU);
    logPO = loadLog(filePO);
    logD  = loadLog(fileD);

    % c1=red, c2=blue, c3=green, c4=purple, c5=orange, c6=goldenrod
    C.U   = [77,175,74]  / 255;
    C.PO  = [228,26,28]  / 255;
    C.D   = [55,126,184] / 255;
    C.j   = {[228,26,28]/255, [55,126,184]/255, [77,175,74]/255, ...
             [152,78,163]/255, [255,127,0]/255, [218,165,32]/255};

    lw = 1.5;

    %% FIGURE 1 – img1
    figImg1 = figure('Name','ICRA26 – img1','NumberTitle','off','Color','w');

    k_U  = logU.samples(:);
    k_PO = logPO.samples(:);
    k_D  = logD.samples(:);

    sPO = load(filePO);
    rootPO = getRootStruct(sPO);
    % Standard path: par.def.passive.tau
    tau_prime = rootPO.par.def.passive.tau / rootPO.par.def.passive.chiSquared.detectionWindow;

    sD = load(fileD);
    rootD = getRootStruct(sD);
    z_x = rootD.par.def.active.PDgainScaling.z_x;

    % Subplot 1: Position error
    ax1 = subplot(4,1,1);
    h1 = [];
    h1(1) = semilogy(k_U,  logU.positionError_l2(:),  'Color',C.U,  'LineWidth',lw); hold on;
    h1(2) = semilogy(k_PO, logPO.positionError_l2(:), 'Color',C.PO, 'LineWidth',lw);
    h1(3) = semilogy(k_D,  logD.positionError_l2(:),  'Color',C.D,  'LineWidth',lw, ...
             'Marker','s','MarkerIndices',1:400:length(k_D),'MarkerFaceColor','w','MarkerEdgeColor',C.D);

    ylabel('$\|\bar{\mathbf{p}}^{\mathrm{A}}_k - \mathbf{p}_k\|_2$','Interpreter','latex');
    ylim([1e-4, 100]);
    grid on; box on; set(ax1, 'YMinorGrid', 'on');
    legend(h1, {'U: $\|\bar{\mathbf{p}}^{\mathrm{A}}_k - \mathbf{p}_k\|_2$', ...
                'PO: $\|\bar{\mathbf{p}}^{\mathrm{A}}_k - \mathbf{p}_k\|_2$', ...
                'D: $\|\bar{\mathbf{p}}^{\mathrm{A}}_k - \mathbf{p}_k\|_2$'}, ...
           'Interpreter','latex','Location','northwest','NumColumns',2,'FontSize',8);
    xlim([-50, 2020]);
    xticks([0 500 1000 1500 2000]);

    % Subplot 2: ADS
    subplot(4,1,2);
    h2 = []; l2 = {};
    if isfield(logPO,'ADS_z')
        h2(end+1) = plot(k_PO, logPO.ADS_z(:), 'Color',C.PO, 'LineWidth',lw); hold on;
        l2{end+1} = 'PO: $z_k$';
    end
    if isfield(logD,'ADS_z')
        h2(end+1) = plot(k_D, logD.ADS_z(:), 'Color',C.D, 'LineWidth',lw, ...
             'Marker','s','MarkerIndices',1:300:length(k_D),'MarkerFaceColor','w','MarkerEdgeColor',C.D); hold on;
        l2{end+1} = 'D: $z_k$';
    end

    h2(end+1) = yline(tau_prime,'k--','$\tau''$','Interpreter','latex','LabelVerticalAlignment','bottom');
    l2{end+1} = '$\tau''$';

    ylabel('$z_k$','Interpreter','latex');
    ylim([0, 11]);
    yticks([0 2 4]);
    grid on; box on;
    legend(h2, l2, 'Interpreter','latex','Location','northwest');
    xlim([-50, 2020]);
    xticks([0 500 1000 1500 2000]);

    % Subplot 3: Projected z
    subplot(4,1,3);
    h3 = []; l3 = {};
    if isfield(logD,'zProj')
        h3(end+1) = plot(k_D, logD.zProj(:), 'Color',C.D, 'LineWidth',lw, ...
             'Marker','s','MarkerIndices',1:300:length(k_D),'MarkerFaceColor','w','MarkerEdgeColor',C.D); hold on;
        l3{end+1} = 'D: $\tilde{z}_k$';
    end

    h3(end+1) = yline(z_x,'k--','$z_{\mathrm{x}}$','Interpreter','latex','LabelVerticalAlignment','bottom');
    l3{end+1} = '$z_{\mathrm{x}}$';

    ylabel('$\tilde{z}_k$','Interpreter','latex');
    ylim([0, 350]);
    yticks([100 200 350]);
    grid on; box on;
    legend(h3, l3, 'Interpreter','latex', 'Location','northwest');
    xlim([-50, 2020]);
    xticks([0 500 1000 1500 2000]);

    % Subplot 4: Gain scaling
    subplot(4,1,4);

    if isfield(logD,'gainScaling_q')
        plot(k_D, logD.gainScaling_q(:), 'Color',C.D, 'LineWidth',lw, ...
             'Marker','s','MarkerIndices',1:300:length(k_D),'MarkerFaceColor','w','MarkerEdgeColor',C.D);
    end

    xlabel('$k$','Interpreter','latex');
    ylabel('$f(\tilde{z}_k)$','Interpreter','latex');
    grid on; box on;
    legend({'D: $f(\tilde{z}_k)$'},'Interpreter','latex','Location','northeast');
    xlim([-50, 2020]);
    xticks([0 500 1000 1500 2000]);

    sgtitle('img1 – Position Error, ADS, z-Projection, Gain Scaling', ...
            'Interpreter','none','FontWeight','bold');

    %% FIGURE 2 – img2
    figImg2 = figure('Name','ICRA26 – img2','NumberTitle','off','Color','w');

    jointLabels = {'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'};

    % Compute shared y-limits for each row (same ymin/ymax for PO and D panels)
    ylim_delta = sharedYlim(logPO.delta_y_optimal, logD.delta_y_optimal);
    ylim_atk   = sharedYlim(logPO.yAttack, logD.yAttack);
    ylim_u     = sharedYlim(logPO.ddot_q_final_unsaturated, logD.ddot_q_final_unsaturated);

    % Row 1: delta_y_optimal -- PO (left) and D (right)
    subplot(3,2,1);
    plotJoints(k_PO, logPO.delta_y_optimal, C.j, lw, false);
    ylabel('PO: $\mathbf{\Delta}^{\star}_{k}$','Interpreter','latex');
    title('\textbf{Passive Defence Only (PO)}','Interpreter','latex');
    ylim(ylim_delta); grid on;
    legend(jointLabels,'Location','northeast','FontSize',7,'NumColumns',2);
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    subplot(3,2,2);
    plotJoints(k_D, logD.delta_y_optimal, C.j, lw, true);
    ylabel('D: $\mathbf{\Delta}^{\star}_{k}$','Interpreter','latex');
    title('\textbf{Active Defence (D)}','Interpreter','latex');
    ylim(ylim_delta); grid on;
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    % Row 2: yAttack -- PO and D
    subplot(3,2,3);
    plotJoints(k_PO, logPO.yAttack, C.j, lw, false);
    ylabel('PO: $\mathbf{a}_{k}$','Interpreter','latex');
    ylim(ylim_atk); grid on;
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    subplot(3,2,4);
    plotJoints(k_D, logD.yAttack, C.j, lw, true);
    ylabel('D: $\mathbf{a}_{k}$','Interpreter','latex');
    ylim(ylim_atk); grid on;
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    % Row 3: ddot_q_final_unsaturated (= u_k) -- PO and D
    subplot(3,2,5);
    plotJoints(k_PO, logPO.ddot_q_final_unsaturated, C.j, lw, false);
    ylabel('PO: $\mathbf{u}_{k}$','Interpreter','latex');
    xlabel('$k$','Interpreter','latex');
    ylim(ylim_u); grid on; box on;
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    subplot(3,2,6);
    plotJoints(k_D, logD.ddot_q_final_unsaturated, C.j, lw, true);
    ylabel('D: $\mathbf{u}_{k}$','Interpreter','latex');
    xlabel('$k$','Interpreter','latex');
    ylim(ylim_u); grid on; box on;
    xlim([-20, 2020]); xticks([0 500 1000 1500 2000]);

    sgtitle('img2 – Attack perturbations, attack signal, control input', ...
            'Interpreter','none','FontWeight','bold');

    %% FIGURE 3 – imgRobot
    figRobot = figure('Name','ICRA26 – imgRobot','NumberTitle','off','Color','w');


    subplot(2,1,1);
    plotRobotTrajectory(logPO, C.PO, 'PO');
    title('Passive Defence Only (PO)','FontWeight','bold');
    box on;

    subplot(2,1,2);
    plotRobotTrajectory(logD, C.D, 'D');
    title('Active Defence (D)','FontWeight','bold');
    box on;

    sgtitle('imgRobot – End-Effector Trajectory', ...
            'Interpreter','none','FontWeight','bold');

    figs = [figImg1, figImg2, figRobot];
    for i = 1:length(figs)
        set(figs(i), 'Color', 'w');
        
        % Axes
        axList = findall(figs(i), 'type', 'axes');
        set(axList, 'Color', 'w');
        set(axList, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
        for j = 1:length(axList)
            set(axList(j).Title, 'Color', 'k');
            set(axList(j).XLabel, 'Color', 'k');
            set(axList(j).YLabel, 'Color', 'k');
        end
        
        % Legends
        legList = findall(figs(i), 'type', 'legend');
        set(legList, 'Color', 'w', 'TextColor', 'k', 'EdgeColor', 'k');
        
        % Supertitle
        sg = findall(figs(i), 'Tag', 'sgtitle');
        if ~isempty(sg), set(sg, 'Color', 'k'); end
    end
end

%% =========================================================================
%% Helpers


function log = loadLog(filepath)
% loadLog  Load the .log field from a simulation .mat file.
%   Handles both root keys 'simulation' and 'experiment'.
    s = load(filepath);
    root = getRootStruct(s);
    log = root.log;
end

function root = getRootStruct(s)
% getRootStruct  Retrieve the root structure (simulation or experiment).
    keys = fieldnames(s);
    if isscalar(keys)
        root = s.(keys{1});
    else
        % Prefer 'simulation' or 'experiment' if present
        if isfield(s, 'simulation')
            root = s.simulation;
        elseif isfield(s, 'experiment')
            root = s.experiment;
        else
            root = s.(keys{1});
        end
    end
end

function yl = sharedYlim(dataND, dataD)
% Compute symmetric y-limits spanning both datasets.
    allData = [dataND(:); dataD(:)];
    allData = allData(isfinite(allData));
    ymin = min(allData);
    ymax = max(allData);
    margin = 0.05 * (ymax - ymin);
    yl = [ymin - margin, ymax + margin];
end

function plotJoints(k, data, C, lw, useMark)
% plotJoints  Plot 6 joint signals (rows of data) with per-joint colours.
%   k       – sample vector (column)
%   data    – [6 x N] matrix
%   C       – cell array of 6 colours
%   lw      – line width
%   useMark – if true add square markers (for D scenario)
    nJoints = size(data, 1);
    hold on;
    for j = 1:nJoints
        markStep = 300 + (j-1)*100;   % stagger marker positions strictly matching TikZ
        if useMark
            plot(k, data(j,:)', 'Color',C{j}, 'LineWidth',lw, ...
                 'Marker','s', ...
                 'MarkerIndices', 1:markStep:length(k), ...
                 'MarkerFaceColor','w', 'MarkerEdgeColor',C{j});
        else
            plot(k, data(j,:)', 'Color',C{j}, 'LineWidth',lw);
        end
    end
end

function plotRobotTrajectory(log, colorHand, scenarioLabel)
    hold on; axis equal; grid on;

    frameNames = {'frame1Pos','frame2Pos','frame3Pos','frame4Pos','frame5Pos','frame6Pos'};
    nFrames = numel(frameNames);
    decim = 50; 

    % Collect decimated frame positions
    allFrames = cell(1, nFrames);
    for fi = 1:nFrames
        if isfield(log, frameNames{fi})
            fp = log.(frameNames{fi});        % [2 x N]
            allFrames{fi} = fp(:, 1:decim:end);
        end
    end

    % Plot joint positions (small dots)
    for fi = 1:nFrames
        if ~isempty(allFrames{fi})
            plot(allFrames{fi}(1,:), allFrames{fi}(2,:), '.k', 'MarkerSize', 3, ...
                 'HandleVisibility','off');
        end
    end

    % Draw robot
    if ~isempty(allFrames{1})
        nSnaps = size(allFrames{1}, 2);
        skipSnap = max(1, round(nSnaps / 8));   % draw ~8 skeleton poses
        for si = 1:skipSnap:nSnaps
            prevPt = [0; 0];     % robot base at origin
            for fi = 1:nFrames
                if ~isempty(allFrames{fi}) && size(allFrames{fi},2) >= si
                    currPt = allFrames{fi}(:, si);
                    plot([prevPt(1), currPt(1)], [prevPt(2), currPt(2)], 'k-', 'LineWidth',0.8, ...
                         'HandleVisibility','off');
                    prevPt = currPt;
                end
            end
        end
    end

    % Robot base 
    plot(0, 0, 'ks', 'MarkerSize', 6, 'MarkerFaceColor','k', 'HandleVisibility','off');

    % ---------- Attacker's goal trajectory  -------------------
    if isfield(log, 'p_goal')
        pg = log.p_goal;    % [2 x N]
        plot(pg(1,:), pg(2,:), 'Color',[152,78,163]/255, 'LineWidth',1.5, 'LineStyle',':', ...
             'DisplayName','$\bar{\mathbf{p}}^{\mathrm{A}}$');
    end

    % ---------- Actual hand trajectory (frame 6 = end-effector) --------------
    if isfield(log, 'handPosition_task')
        hp = log.handPosition_task;   % [2 x N]
        plot(hp(1,:), hp(2,:), 'Color', colorHand, 'LineWidth', 1.2, ...
             'DisplayName', sprintf('%s: $\\mathbf{p}$', scenarioLabel));
    end

    % labels and legend
    xlabel('$x$','Interpreter','latex');
    ylabel('$y$','Interpreter','latex');
    legend('Interpreter','latex','Location','northwest');
    xlim([-2.3, 2]);
    ylim([-0.5, 2]);
    yticks([-1 0 1 2]);
end
