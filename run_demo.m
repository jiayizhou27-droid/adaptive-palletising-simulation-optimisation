function results = run_demo(options)
%RUN_DEMO Run the adaptive palletising pipeline from sample data.
%
% results = run_demo()
% results = run_demo(Method="csga", NumCartons=10, PlanTrajectory=true)
%
% Method choices are "constructive", "csga", and "cssa". The default
% configuration is intentionally small enough for a reviewer to run as a
% demonstration. Use NumCartons=40 and MaxEvaluations=120 to reproduce the
% larger illustrative integration configuration described in the report.

arguments
    options.Method (1,1) string {mustBeMember(options.Method,["constructive","csga","cssa"])} = "constructive"
    options.NumCartons (1,1) double {mustBeInteger,mustBePositive} = 10
    options.MaxEvaluations (1,1) double {mustBeInteger,mustBePositive} = 25
    options.PlanTrajectory (1,1) logical = true
    options.OpenModel (1,1) logical = false
    options.Seed (1,1) double {mustBeInteger} = 20260824
end

projectRoot = fileparts(mfilename("fullpath"));
addpath(projectRoot);
dataFile = fullfile(projectRoot,"data","sample","boxes.xlsx");
assert(isfile(dataFile),"Sample data file not found: %s",dataFile);

cfg = palletTaskConfig();
boxTable = readtable(dataFile);
boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
if ismember("Type",string(boxTable.Properties.VariableNames))
    boxDataAll.type = string(boxTable.Type);
end

boxData = sliceBoxData(boxDataAll,cfg.firstExcelID);
n = min(options.NumCartons,numel(boxData.length));
fields = fieldnames(boxData);
for k = 1:numel(fields)
    value = boxData.(fields{k});
    if numel(value) >= n
        boxData.(fields{k}) = value(1:n);
    end
end

switch options.Method
    case "constructive"
        layoutPlan = buildLayoutPlanBatch3(boxData,cfg.palletDim, ...
            cfg.palletLoc,n,cfg.palletMargin,cfg.boxGap);
        optimizerInfo = struct("method","Constructive", ...
            "evaluations",1,"complete",all([layoutPlan.isPlaced]));
    case "csga"
        [layoutPlan,optimizerInfo] = buildLayoutPlanCSGA(boxData,cfg, ...
            struct("seed",options.Seed,"populationSize",8, ...
            "maxEvaluations",options.MaxEvaluations));
    case "cssa"
        [layoutPlan,optimizerInfo] = buildLayoutPlanCSSA(boxData,cfg, ...
            struct("seed",options.Seed,"maxEvaluations", ...
            options.MaxEvaluations));
end

executionPlan = buildBufferExecutionPlan(layoutPlan,cfg);
top = max([layoutPlan.z] + [layoutPlan.H]/2);
base = cfg.palletLoc(3) + cfg.palletDim(3)/2;
stackHeight = top - base;
cartonVolume = sum([layoutPlan.L].*[layoutPlan.W].*[layoutPlan.H]);
utilization = cartonVolume/(prod(cfg.palletDim(1:2))*stackHeight);

results = struct();
results.method = options.Method;
results.layoutPlan = layoutPlan;
results.optimizerInfo = optimizerInfo;
results.executionPlan = executionPlan;
results.stackHeight = stackHeight;
results.envelopeUtilization = utilization;
results.layerCount = max([layoutPlan.layer]);
results.trajectory = [];
results.trajectoryReport = [];

if options.PlanTrajectory
    trajectoryOptions = struct( ...
        "pickPosition",cfg.pickPosition, ...
        "pickYaw",cfg.pickYaw, ...
        "approachHeight",cfg.approachHeight, ...
        "topClearance",cfg.topClearance, ...
        "samplesPerBox",80, ...
        "sampleTime",0.05, ...
        "useRRTFallback",true, ...
        "visualize",false, ...
        "executionPlan",executionPlan);
    [results.trajectory,results.trajectoryReport] = ...
        planPalletizingTrajectory(layoutPlan,cfg.palletDim, ...
        cfg.palletLoc,trajectoryOptions);
end

assignin("base","layoutPlan",layoutPlan);
assignin("base","executionPlan",executionPlan);
assignin("base","boxData",boxData);
assignin("base","palletDim",cfg.palletDim);
assignin("base","palletLoc",cfg.palletLoc);
if options.PlanTrajectory
    assignin("base","traj",results.trajectory);
    assignin("base","trajectoryReport",results.trajectoryReport);
end

fprintf("Method: %s\n",optimizerInfo.method);
fprintf("Cartons placed: %d/%d\n",sum([layoutPlan.isPlaced]),n);
fprintf("Layers: %d\n",results.layerCount);
fprintf("Stack height: %.3f m\n",stackHeight);
fprintf("Envelope utilization: %.3f %%\n",100*utilization);
fprintf("Execution actions: %d\n",numel(executionPlan));

if usejava("desktop")
    plotLayoutPlan3D(layoutPlan,cfg.palletDim,cfg.palletLoc);
end
if options.OpenModel
    open_system(fullfile(projectRoot,"PalletizeBoxesUsingCobot.slx"));
end
end
