%% ===== Init Script for Pallet Layout =====

% --- Shared task parameters ---
cfg = palletTaskConfig();
palletDim = cfg.palletDim;
palletLoc = cfg.palletLoc;
palletMargin = cfg.palletMargin;
boxGap = cfg.boxGap;

% --- Load Excel ---
projectDir = fileparts(mfilename("fullpath"));
boxTable = readtable(fullfile(projectDir,"data","sample","boxes.xlsx"));

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

% --- Build layout using the same batch-3 optimizer as Simulink ---
batchSize = cfg.batchSize;
layoutPlan = buildLayoutPlanBatch3(boxData, palletDim, palletLoc, batchSize, palletMargin, boxGap);
executionPlan = buildBufferExecutionPlan(layoutPlan, cfg);

% --- Convert to Simulink-friendly format ---
layoutPos = reshape([layoutPlan.pos], 3, []).';
layoutYaw = reshape([layoutPlan.yaw], 1, []).';
layoutN   = size(layoutPos,1);

% --- Dummy variable (for Simulink compatibility) ---
boxDim = [boxData.length(1); boxData.width(1); boxData.height(1)];
boxPosition = cfg.sourceBoxPosition;
pickPosition = cfg.pickPosition;

disp("Initialization complete.");
