function results = runSuperboxScaleComplexityExperiment(seed)
%RUNSUPERBOXSCALECOMPLEXITYEXPERIMENT Superbox scaling for N x type count.
% Tests N = 20, 40, 80 cartons and K = 4, 8 carton types without
% modifying boxes.xlsx. All cases use the same pallet and stability rules.
if nargin < 1
    seed = 20260727;
end

projectDir = fileparts(mfilename('fullpath'));
cfg = palletTaskConfig();
boxCounts = [20 40 80];
typeCounts = [4 8];
outDir = fullfile(projectDir,'superbox_scale_complexity_experiment');
if ~isfolder(outDir)
    mkdir(outDir);
end

summary = table();
plans = cell(numel(boxCounts),numel(typeCounts));
caseData = cell(numel(boxCounts),numel(typeCounts));

for i = 1:numel(boxCounts)
    for j = 1:numel(typeCounts)
        n = boxCounts(i);
        kTypes = typeCounts(j);
        caseSeed = seed + 100*i + j;
        b = makeBoxData(n,kTypes,caseSeed);
        caseData{i,j} = b;
        fprintf('START N=%d K=%d\n',n,kTypes);
        drawnow;

        succeeded = false;
        failureReason = "";
        plan = struct([]);
        execution = struct([]);
        requiredBuffer = NaN;
        tic;
        try
            plan = buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,n, ...
                cfg.palletMargin,cfg.boxGap,false);
            planningTime = toc;

            cfgAnalysis = cfg;
            cfgAnalysis.bufferCapacity = n;
            cfgAnalysis.bufferGrid = [ceil(n/2) 2];
            execution = buildBufferExecutionPlan(plan,cfgAnalysis);
            requiredBuffer = peakBuffer(execution);
            row = computeLayoutComparisonMetrics("Superbox",plan,execution,cfg, ...
                planningTime,NaN);
            succeeded = true;
        catch ME
            planningTime = toc;
            failureReason = string(ME.message);
            warning('N=%d, K=%d failed: %s',n,kTypes,ME.message);
            row = failedMetricRow(planningTime);
        end

        row.BoxCount = n;
        row.TypeCount = kTypes;
        row.PlanningSucceeded = succeeded;
        row.FailureReason = failureReason;
        row.RequiredBufferCapacity = requiredBuffer;
        row.FeasibleWith8SlotBuffer = succeeded && ...
            requiredBuffer <= cfg.bufferCapacity;
        row.Seed = caseSeed;
        summary = [summary; row]; %#ok<AGROW>
        plans{i,j} = plan;

        inputTable = table((1:n)',b.length,b.width,b.height,b.weight,b.type, ...
            'VariableNames',{'ArrivalIndex','L','W','H','Mass','Type'});
        writetable(inputTable,fullfile(outDir, ...
            sprintf('input_N%d_K%d.csv',n,kTypes)));
        writetable(summary,fullfile(outDir,'checkpoint_summary.csv'));
        fprintf('DONE N=%d K=%d success=%d planningTime=%.6f s\n', ...
            n,kTypes,succeeded,planningTime);
        drawnow;
    end
end

summary = movevars(summary,{'BoxCount','TypeCount'},'Before',1);
writetable(summary,fullfile(outDir,'superbox_scale_complexity_summary.csv'));
save(fullfile(outDir,'superbox_scale_complexity_experiment.mat'), ...
    'summary','plans','caseData','cfg','boxCounts','typeCounts','seed');
for i = 1:numel(boxCounts)
    for j = 1:numel(typeCounts)
        if ~isempty(plans{i,j})
            plotOne(plans{i,j},cfg,boxCounts(i),typeCounts(j), ...
                fullfile(outDir,sprintf('layout_N%d_K%d.png', ...
                boxCounts(i),typeCounts(j))));
        end
    end
end
plotSummary(summary,boxCounts,typeCounts,fullfile(outDir, ...
    'superbox_scale_complexity_metrics.png'));

disp(summary(:,{'BoxCount','TypeCount','PlanningSucceeded', ...
    'MeanPlanningTime_s','Layers','StackHeight_m', ...
    'PalletVolumeUtilization','MinSupportRatio', ...
    'MinCOMSupportMargin_m','RequiredBufferCapacity'}));
results = struct('summary',summary,'plans',{plans}, ...
    'outputDirectory',outDir,'seed',seed);
assignin('base','superboxScaleComplexityExperiment',results);
end

function b = makeBoxData(n,kTypes,seed)
% The four-type set exactly matches boxes.xlsx. The eight-type set retains
% those four extremes and adds four intermediate footprints. Every carton
% remains upright and L-by-W is its largest face.
types4 = [ ...
    0.30 0.24 0.15 3.2; ...
    0.27 0.22 0.15 2.7; ...
    0.23 0.20 0.15 2.2; ...
    0.20 0.18 0.15 1.6];
types8 = [ ...
    0.30 0.24 0.15 3.2; ...
    0.29 0.20 0.15 2.9; ...
    0.27 0.22 0.15 2.7; ...
    0.26 0.18 0.15 2.4; ...
    0.23 0.20 0.15 2.2; ...
    0.22 0.19 0.15 2.0; ...
    0.20 0.18 0.15 1.6; ...
    0.21 0.18 0.15 1.7];
if kTypes == 4
    catalogue = types4;
else
    catalogue = types8;
end

% Use a balanced cyclic sequence to isolate N and K. Arrival-order
% sensitivity is measured separately by runArrivalPatternExperiment40.
ids = repmat(1:kTypes,1,ceil(n/kTypes));
ids = ids(1:n);
b.length = catalogue(ids,1);
b.width = catalogue(ids,2);
b.height = catalogue(ids,3);
b.weight = catalogue(ids,4);
labels = string(cellstr(char(double('A') + (0:kTypes-1))'));
b.type = labels(ids);
b.sourceID = (1:n)';
end

function row = failedMetricRow(planningTime)
names = {'Method','PlacedBoxes','MeanPlanningTime_s','PlanningTimeStd_s', ...
    'Layers','StackHeight_m','BoxVolume_m3','PalletVolumeUtilization', ...
    'UsableEnvelopeVolumeUtilization','MeanLayerAreaUtilization', ...
    'FirstLayerAreaUtilization','TopLayerAreaUtilization', ...
    'MinimumLayerAreaUtilization','LayerUtilizationStd','MinSupportRatio', ...
    'MinCOMSupportMargin_m','UnstableBoxCount','COMOffset_m', ...
    'NormalizedCOMOffset','COMHeight_m','MinStaticTippingAngle_deg', ...
    'ToBufferActions','FromBufferActions','PeakBufferOccupancy', ...
    'RobotMoves','TransferDistance_m','PayloadLiftEnergyProxy_J','BoxesPerLayer'};
values = cell(1,numel(names));
values{1} = "Superbox";
for k = 2:numel(names)-1
    values{k} = NaN;
end
values{3} = planningTime;
values{end} = "";
row = cell2table(values,'VariableNames',names);
end

function peak = peakBuffer(execution)
occupancy = 0;
peak = 0;
for k = 1:numel(execution)
    action = string(execution(k).action);
    if action == "conveyor_to_buffer"
        occupancy = occupancy + 1;
    elseif action == "buffer_to_pallet"
        occupancy = occupancy - 1;
    end
    peak = max(peak,occupancy);
end
end

function plotOne(plan,cfg,n,kTypes,outFile)
f = figure('Visible','off','Color','w','Position',[100 80 900 700]);
ax = axes(f);
hold(ax,'on');
axis(ax,'equal');
grid(ax,'on');
view(ax,35,25);
cuboid(ax,cfg.palletLoc-[cfg.palletDim(1:2)/2 0], ...
    cfg.palletDim,[.7 .7 .7],.25);
colors = lines(kTypes);
for q = 1:numel(plan)
    if ~plan(q).isPlaced
        continue
    end
    typeIndex = max(1,min(kTypes,double(upper(char(plan(q).type))-'A'+1)));
    cuboid(ax,plan(q).pos-[plan(q).L plan(q).W plan(q).H]/2, ...
        [plan(q).L plan(q).W plan(q).H],colors(typeIndex,:),.88);
end
xlabel(ax,'X (m)');
ylabel(ax,'Y (m)');
zlabel(ax,'Z (m)');
title(ax,sprintf('Superbox: N = %d, carton types = %d',n,kTypes));
exportgraphics(f,outFile,'Resolution',180);
close(f);
end

function plotSummary(S,boxCounts,typeCounts,outFile)
f = figure('Visible','off','Color','w','Position',[80 80 1450 760]);
tl = tiledlayout(f,2,2,'TileSpacing','compact');
metrics = {'MeanPlanningTime_s','PalletVolumeUtilization', ...
    'StackHeight_m','RequiredBufferCapacity'};
titles = {'Planning time (s)','Pallet volume utilization', ...
    'Stack height (m)','Required buffer capacity'};
for q = 1:4
    ax = nexttile(tl);
    values = NaN(numel(boxCounts),numel(typeCounts));
    for i = 1:numel(boxCounts)
        for j = 1:numel(typeCounts)
            hit = S.BoxCount == boxCounts(i) & S.TypeCount == typeCounts(j);
            values(i,j) = S.(metrics{q})(hit);
        end
    end
    bar(ax,values);
    grid(ax,'on');
    xticks(ax,1:numel(boxCounts));
    xticklabels(ax,string(boxCounts));
    xlabel(ax,'Number of cartons');
    title(ax,titles{q});
    if q == 1
        legend(ax,string(typeCounts) + " types",'Location','best');
    end
end
exportgraphics(f,outFile,'Resolution',180);
close(f);
end

function cuboid(ax,o,d,c,a)
x=o(1); y=o(2); z=o(3); L=d(1); W=d(2); H=d(3);
V=[x y z;x+L y z;x+L y+W z;x y+W z; ...
   x y z+H;x+L y z+H;x+L y+W z+H;x y+W z+H];
F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch(ax,'Vertices',V,'Faces',F,'FaceColor',c, ...
    'FaceAlpha',a,'EdgeColor',[.1 .1 .1]);
end
