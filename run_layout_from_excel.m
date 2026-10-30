clear; clc; close all;

% ============================================================
% 1. Read box data from Excel
% ============================================================
projectDir = fileparts(mfilename("fullpath"));
excelPath = fullfile(projectDir, "data", "sample", "boxes.xlsx");
cfg = palletTaskConfig();

boxTable = readtable(excelPath);

boxDataAll.length = boxTable.L;
boxDataAll.width  = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
if ismember("Type", string(boxTable.Properties.VariableNames))
    boxDataAll.type = string(boxTable.Type);
end
layoutBoxOffset = cfg.layoutBoxOffset;
firstExcelID = cfg.firstExcelID;
boxData = sliceBoxData(boxDataAll, firstExcelID);
nLayoutBoxes = min(cfg.maxLayoutBoxes, numel(boxData.length));
boxData.length = boxData.length(1:nLayoutBoxes);
boxData.width = boxData.width(1:nLayoutBoxes);
boxData.height = boxData.height(1:nLayoutBoxes);
boxData.weight = boxData.weight(1:nLayoutBoxes);
if isfield(boxData,'type')
    boxData.type = boxData.type(1:nLayoutBoxes);
end

% ============================================================
% 2. Define pallet size and location
%    Use your existing palletDim and palletLoc if they already exist.
% ============================================================
if ~exist('palletDim','var')
    palletDim = cfg.palletDim;
end

if ~exist('palletLoc','var')
    palletLoc = cfg.palletLoc;
end
if ~exist('palletMargin','var')
    palletMargin = cfg.palletMargin;
end
if ~exist('boxGap','var')
    boxGap = cfg.boxGap;
end

% ============================================================
% 3. Build layout plan
% ============================================================
if ~exist('batchSize','var')
    batchSize = cfg.batchSize;
end
[layoutPlan,csgaInfo] = buildLayoutPlanCSGA(boxData,cfg,struct( ...
    'seed',20260824,'populationSize',12,'maxEvaluations',120));
executionPlan = buildBufferExecutionPlan(layoutPlan, cfg);

% ============================================================
% 4. Convert layoutPlan to Simulink variables
% ============================================================
layoutPos = reshape([layoutPlan.pos],3,[])';
layoutYaw = reshape([layoutPlan.yaw],1,[])';
layoutN   = size(layoutPos,1);

% ============================================================
% 5. Plan robot trajectory from the optimized layout
% ============================================================
if ~exist('enableRRTFallback','var')
    enableRRTFallback = true;
end
if ~exist('enableTrajectoryPlot','var')
    enableTrajectoryPlot = usejava('desktop');
end
if ~exist('samplesPerBox','var')
    samplesPerBox = 80;
end

trajOpt = struct();
trajOpt.pickPosition = cfg.pickPosition;
trajOpt.pickYaw = cfg.pickYaw;
trajOpt.approachHeight = cfg.approachHeight;
trajOpt.topClearance = cfg.topClearance;
trajOpt.samplesPerBox = samplesPerBox;
trajOpt.sampleTime = 0.05;
trajOpt.useRRTFallback = enableRRTFallback;
trajOpt.visualize = false;
trajOpt.executionPlan = executionPlan;

[traj, trajectoryReport] = planPalletizingTrajectory( ...
    layoutPlan, palletDim, palletLoc, trajOpt);

qTraj = traj.q;
qdTraj = traj.qd;
qddTraj = traj.qdd;
qTrajTs = timeseries(qTraj.', traj.time);

assignin('base','boxData',boxData);
assignin('base','boxDataAll',boxDataAll);
assignin('base','layoutBoxOffset',layoutBoxOffset);
assignin('base','firstExcelID',firstExcelID);
assignin('base','layoutPlan',layoutPlan);
assignin('base','executionPlan',executionPlan);
assignin('base','csgaInfo',csgaInfo);
assignin('base','layoutPos',layoutPos);
assignin('base','layoutYaw',layoutYaw);
assignin('base','layoutN',layoutN);
assignin('base','batchSize',batchSize);
assignin('base','palletDim',palletDim);
assignin('base','palletLoc',palletLoc);
assignin('base','palletMargin',palletMargin);
assignin('base','boxGap',boxGap);
assignin('base','traj',traj);
assignin('base','trajectoryReport',trajectoryReport);
assignin('base','qTraj',qTraj);
assignin('base','qdTraj',qdTraj);
assignin('base','qddTraj',qddTraj);
assignin('base','qTrajTs',qTrajTs);

save(fullfile(projectDir, "plannedTrajectory.mat"), ...
    "traj", "trajectoryReport", "qTraj", "qdTraj", "qddTraj", ...
    "qTrajTs", "layoutPlan", "executionPlan", "palletDim", "palletLoc", ...
    "layoutBoxOffset", "firstExcelID", "boxData", "boxDataAll");

% ============================================================
% 6. Display and plot
% ============================================================
disp("Generated layout positions:");
disp(layoutPos);
disp("Generated trajectory size [joints x samples]:");
disp(size(qTraj));
disp("Per-box trajectory report:");
disp(struct2table(trajectoryReport));

if enableTrajectoryPlot
    plotLayoutPlan3D(layoutPlan, palletDim, palletLoc);
end
