clear; clc; close all;

% ============================================================
% Step 5: trajectory planning from the optimized pallet layout
% ============================================================
projectDir = fileparts(mfilename("fullpath"));
excelPath = fullfile(projectDir, "data", "sample", "boxes.xlsx");
cfg = palletTaskConfig();

% 1. Load external box data
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

% 2. Build the adaptive layout. Keep these values consistent with Simulink.
palletDim = cfg.palletDim;
palletLoc = cfg.palletLoc;
palletMargin = cfg.palletMargin;
boxGap = cfg.boxGap;
batchSize = cfg.batchSize;

if exist("layoutOptimizerOpt", "var")
    warning("layoutOptimizerOpt is ignored: Simulink uses rolling-window mixed-layer planning.");
end
layoutPlan = buildLayoutPlanBatch3(boxData, palletDim, palletLoc, batchSize, palletMargin, boxGap);
executionPlan = buildBufferExecutionPlan(layoutPlan, cfg);

% 3. Plan robot motion for every placed box.
if ~exist("enableRRTFallback", "var")
    enableRRTFallback = true;
end
if ~exist("enableTrajectoryPlot", "var")
    enableTrajectoryPlot = usejava("desktop");
end
if ~exist("samplesPerBox", "var")
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

% 4. Export Simulink-friendly variables to the base workspace.
layoutPos = reshape([layoutPlan.pos], 3, []).';
layoutYaw = reshape([layoutPlan.yaw], 1, []).';
layoutN = size(layoutPos, 1);

qTraj = traj.q;
qdTraj = traj.qd;
qddTraj = traj.qdd;
qTrajTs = timeseries(qTraj.', traj.time);

assignin("base", "boxData", boxData);
assignin("base", "boxDataAll", boxDataAll);
assignin("base", "layoutBoxOffset", layoutBoxOffset);
assignin("base", "firstExcelID", firstExcelID);
assignin("base", "layoutPlan", layoutPlan);
assignin("base", "executionPlan", executionPlan);
assignin("base", "layoutPos", layoutPos);
assignin("base", "layoutYaw", layoutYaw);
assignin("base", "layoutN", layoutN);
assignin("base", "palletDim", palletDim);
assignin("base", "palletLoc", palletLoc);
assignin("base", "palletMargin", palletMargin);
assignin("base", "boxGap", boxGap);
assignin("base", "batchSize", batchSize);
assignin("base", "traj", traj);
assignin("base", "trajectoryReport", trajectoryReport);
assignin("base", "qTraj", qTraj);
assignin("base", "qdTraj", qdTraj);
assignin("base", "qddTraj", qddTraj);
assignin("base", "qTrajTs", qTrajTs);

save(fullfile(projectDir, "plannedTrajectory.mat"), ...
    "traj", "trajectoryReport", "qTraj", "qdTraj", "qddTraj", ...
    "qTrajTs", "layoutPlan", "executionPlan", "palletDim", "palletLoc", ...
    "layoutBoxOffset", "firstExcelID", "boxData", "boxDataAll");

disp("Trajectory planning complete.");
disp("qTraj size [joints x samples]:");
disp(size(qTraj));
disp("Per-box trajectory report:");
disp(struct2table(trajectoryReport));

if enableTrajectoryPlot
    plotLayoutPlan3D(layoutPlan, palletDim, palletLoc);
end
