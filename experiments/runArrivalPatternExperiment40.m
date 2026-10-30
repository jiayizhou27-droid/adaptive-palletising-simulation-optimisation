function results = runArrivalPatternExperiment40(seed)
%RUNARRIVALPATTERNEXPERIMENT40 Compare three planners under five arrival patterns.
% The physical cartons, pallet geometry, layout constraints, and metric
% definitions are unchanged. Only the conveyor arrival order is varied.
if nargin < 1
    seed = 20260727;
end

projectDir = fileparts(mfilename('fullpath'));
cfg = palletTaskConfig();
T0 = readtable(fullfile(projectDir,'boxes.xlsx'));
assert(height(T0) >= 40,'boxes.xlsx must contain at least 40 cartons.');
T0 = T0(1:40,:);

patterns = ["CompletelyRandom";"LargeBoxFirst";"SmallBoxFirst"; ...
    "TypeClustered";"AlternatingLargeSmall"];
methods = ["Shelf";"RollingHorizon3";"Superbox40"];
orders = makeArrivalOrders(T0,seed);

outDir = fullfile(projectDir,'arrival_pattern_experiment40');
if ~isfolder(outDir)
    mkdir(outDir);
end

allSummary = table();
plansByPattern = cell(numel(patterns),numel(methods));
executionsByPattern = cell(numel(patterns),numel(methods));

for s = 1:numel(patterns)
    permutation = orders{s};
    T = T0(permutation,:);
    T.ArrivalIndex = (1:40)';
    T.OriginalRow = permutation(:);
    writetable(T,fullfile(outDir,patterns(s) + "_arrival_order.csv"));

    b.length = T.L;
    b.width = T.W;
    b.height = T.H;
    b.weight = T.Mass;
    if ismember('Type',T.Properties.VariableNames)
        b.type = string(T.Type);
    end
    b.sourceID = (1:40)';

    scenarioSummary = table();
    for m = 1:numel(methods)
        try
            tic;
            switch methods(m)
                case "Shelf"
                    plan = buildLayoutPlanShelfBaseline(b,cfg.palletDim,cfg.palletLoc, ...
                        cfg.palletMargin,cfg.boxGap);
                case "RollingHorizon3"
                    plan = buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,3, ...
                        cfg.palletMargin,cfg.boxGap,false);
                otherwise
                    plan = buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,40, ...
                        cfg.palletMargin,cfg.boxGap,false);
            end
            planningTime = toc;

            % Build the unconstrained execution schedule to measure how much
            % buffer the layout would require; feasibility is then checked
            % against the unchanged physical eight-slot buffer.
            cfgAnalysis = cfg;
            cfgAnalysis.bufferCapacity = 40;
            cfgAnalysis.bufferGrid = [20 2];
            execution = buildBufferExecutionPlan(plan,cfgAnalysis);
            requiredBuffer = peakBuffer(execution);
            row = computeLayoutComparisonMetrics(methods(m),plan,execution,cfg, ...
                planningTime,NaN);
            row.PlanningSucceeded = true;
            row.FailureReason = "";
        catch ME
            planningTime = toc;
            warning('%s failed for %s: %s',methods(m),patterns(s),ME.message);
            plan = struct([]);
            execution = struct([]);
            requiredBuffer = NaN;
            assert(~isempty(scenarioSummary), ...
                'The first method failed, so a metric row template is unavailable.');
            row = scenarioSummary(1,:);
            numericVars = varfun(@isnumeric,row,'OutputFormat','uniform');
            row{1,numericVars} = NaN;
            logicalVars = varfun(@islogical,row,'OutputFormat','uniform');
            row{1,logicalVars} = false;
            row.Method = methods(m);
            row.MeanPlanningTime_s = planningTime;
            row.PlanningSucceeded = false;
            row.FailureReason = string(ME.message);
        end
        row.ArrivalPattern = patterns(s);
        row.Seed = seed;
        row.RequiredBufferCapacity = requiredBuffer;
        row.FeasibleWith8SlotBuffer = row.PlanningSucceeded && ...
            requiredBuffer <= cfg.bufferCapacity;
        scenarioSummary = [scenarioSummary; row]; %#ok<AGROW>

        plansByPattern{s,m} = plan;
        executionsByPattern{s,m} = execution;
    end

    allSummary = [allSummary; scenarioSummary]; %#ok<AGROW>
    plotScenario(plansByPattern(s,:),methods,cfg,patterns(s), ...
        fullfile(outDir,patterns(s) + "_layout_comparison.png"));
    writetable(scenarioSummary, ...
        fullfile(outDir,patterns(s) + "_performance.csv"));
end

allSummary = movevars(allSummary,{'ArrivalPattern','Method'},'Before',1);
writetable(allSummary,fullfile(outDir,'arrival_pattern_performance_all.csv'));
save(fullfile(outDir,'arrival_pattern_experiment40.mat'), ...
    'allSummary','plansByPattern','executionsByPattern','orders', ...
    'patterns','methods','cfg','T0','seed');

plotMetricSummary(allSummary,patterns,methods, ...
    fullfile(outDir,'arrival_pattern_key_metrics.png'));
disp(allSummary(:,{'ArrivalPattern','Method','Layers','StackHeight_m', ...
    'PalletVolumeUtilization','MinSupportRatio','MinCOMSupportMargin_m', ...
    'MeanPlanningTime_s','RequiredBufferCapacity','FeasibleWith8SlotBuffer'}));

results = struct('summary',allSummary,'orders',{orders}, ...
    'patterns',patterns,'methods',methods,'outputDirectory',outDir,'seed',seed);
assignin('base','arrivalPatternExperiment40',results);
end

function orders = makeArrivalOrders(T,seed)
n = height(T);
volume = T.L .* T.W .* T.H;
rng(seed,'twister');
orders = cell(5,1);
orders{1} = randperm(n);

tie = rand(n,1);
[~,orders{2}] = sortrows([-volume tie],[1 2]);
[~,orders{3}] = sortrows([ volume tie],[1 2]);

type = string(T.Type);
uniqueTypes = unique(type,'stable');
clustered = zeros(n,1);
cursor = 1;
for k = 1:numel(uniqueTypes)
    ids = find(type == uniqueTypes(k));
    ids = ids(randperm(numel(ids)));
    clustered(cursor:cursor+numel(ids)-1) = ids;
    cursor = cursor + numel(ids);
end
orders{4} = clustered(:)';

[~,sorted] = sortrows([-volume tie],[1 2]);
alternating = zeros(1,n);
lo = 1;
hi = n;
for k = 1:n
    if mod(k,2) == 1
        alternating(k) = sorted(lo);
        lo = lo + 1;
    else
        alternating(k) = sorted(hi);
        hi = hi - 1;
    end
end
orders{5} = alternating;
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

function plotScenario(plans,methods,cfg,pattern,outFile)
f = figure('Visible','off','Color','w','Position',[50 80 1650 560]);
tl = tiledlayout(f,1,3,'TileSpacing','compact');
for m = 1:3
    ax = nexttile(tl);
    hold(ax,'on');
    axis(ax,'equal');
    grid(ax,'on');
    view(ax,35,25);
    cuboid(ax,cfg.palletLoc-[cfg.palletDim(1:2)/2 0], ...
        cfg.palletDim,[.7 .7 .7],.25);
        plan = plans{m};
        if isempty(plan)
            text(ax,0.5,0.5,0.5,'No feasible layout', ...
                'HorizontalAlignment','center','Color',[.75 .1 .1], ...
                'FontWeight','bold');
            title(ax,methods(m),'Interpreter','none');
            continue
        end
    for k = 1:numel(plan)
        if ~plan(k).isPlaced
            continue
        end
        cuboid(ax,plan(k).pos-[plan(k).L plan(k).W plan(k).H]/2, ...
            [plan(k).L plan(k).W plan(k).H],typeColor(plan(k).type),.88);
    end
    title(ax,methods(m),'Interpreter','none');
    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');
    zlabel(ax,'Z (m)');
end
title(tl,strrep(pattern,'_',' '),'Interpreter','none');
exportgraphics(f,outFile,'Resolution',180);
close(f);
end

function plotMetricSummary(S,patterns,methods,outFile)
f = figure('Visible','off','Color','w','Position',[80 80 1500 820]);
tl = tiledlayout(f,2,2,'TileSpacing','compact');
metricNames = {'PalletVolumeUtilization','StackHeight_m', ...
    'RequiredBufferCapacity','MeanPlanningTime_s'};
titles = {'Pallet volume utilization','Stack height (m)', ...
    'Required buffer capacity','Planning time (s)'};
for q = 1:4
    ax = nexttile(tl);
    values = zeros(numel(patterns),numel(methods));
    for s = 1:numel(patterns)
        for m = 1:numel(methods)
            hit = S.ArrivalPattern == patterns(s) & S.Method == methods(m);
            values(s,m) = S.(metricNames{q})(hit);
        end
    end
    bar(ax,values);
    grid(ax,'on');
    title(ax,titles{q});
    xticks(ax,1:numel(patterns));
    xticklabels(ax,["Random","Large first","Small first","Clustered","Alternating"]);
    xtickangle(ax,18);
    if q == 1
        legend(ax,methods,'Location','best');
    end
end
exportgraphics(f,outFile,'Resolution',180);
close(f);
end

function c = typeColor(t)
switch upper(char(t))
    case 'A', c = [.86 .47 .12];
    case 'B', c = [.22 .55 .82];
    case 'C', c = [.30 .68 .38];
    otherwise, c = [.65 .55 .78];
end
end

function cuboid(ax,o,d,c,a)
x=o(1); y=o(2); z=o(3); L=d(1); W=d(2); H=d(3);
V=[x y z;x+L y z;x+L y+W z;x y+W z; ...
   x y z+H;x+L y z+H;x+L y+W z+H;x y+W z+H];
F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch(ax,'Vertices',V,'Faces',F,'FaceColor',c, ...
    'FaceAlpha',a,'EdgeColor',[.1 .1 .1]);
end
