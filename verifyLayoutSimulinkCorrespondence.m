function verifyLayoutSimulinkCorrespondence()
% verifyLayoutSimulinkCorrespondence Check plotted targets match Simulink IDs.

projectDir = fileparts(mfilename("fullpath"));
boxTable = readtable(fullfile(projectDir, "boxes.xlsx"));
cfg = palletTaskConfig();

boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;

layoutBoxOffset = cfg.layoutBoxOffset;
firstExcelID = cfg.firstExcelID;
boxData = sliceBoxData(boxDataAll, firstExcelID);

palletDim = cfg.palletDim;
palletLoc = cfg.palletLoc;
palletMargin = cfg.palletMargin;
boxGap = cfg.boxGap;
batchSize = cfg.batchSize;

layoutPlan = buildLayoutPlanBatch3(boxData, palletDim, palletLoc, batchSize, palletMargin, boxGap);
layoutPos = reshape([layoutPlan.pos], 3, []).';
layoutYaw = reshape([layoutPlan.yaw], 1, []).';
layoutN = size(layoutPos, 1);

assert(layoutN == height(boxTable) - layoutBoxOffset, ...
    "Layout count does not match Excel rows after skipped boxes.");
assert(all([layoutPlan.isPlaced]), "Not every included Excel box was placed.");
assert(isequal([layoutPlan.sourceID], firstExcelID:height(boxTable)), ...
    "sourceID is not a one-to-one mapping to Excel rows.");

assignin("base", "boxData", boxData);
assignin("base", "boxDataAll", boxDataAll);
assignin("base", "layoutBoxOffset", layoutBoxOffset);
assignin("base", "firstExcelID", firstExcelID);
assignin("base", "layoutPlan", layoutPlan);
assignin("base", "layoutPos", layoutPos);
assignin("base", "layoutYaw", layoutYaw);
assignin("base", "layoutN", layoutN);
assignin("base", "palletDim", palletDim);
assignin("base", "palletLoc", palletLoc);
assignin("base", "palletMargin", palletMargin);
assignin("base", "boxGap", boxGap);

simulinkGoalPos = zeros(layoutN, 3);
simulinkBoxDim = zeros(layoutN, 3);
for id = 0:layoutN-1
    row = id + 1;
    simulinkGoalPos(row,:) = getBatch3GoalFromLayout(id);
    simulinkBoxDim(row,:) = selectBoxDim(id).';
end

layoutDims = [[layoutPlan.L].' [layoutPlan.W].' [layoutPlan.H].'];
posErr = max(abs(simulinkGoalPos - layoutPos), [], "all");
dimErr = max(abs(simulinkBoxDim - layoutDims), [], "all");

assert(posErr == 0, "Simulink goal positions do not match layoutPos.");
assert(dimErr == 0, "Simulink box dimensions do not match layoutPlan dimensions.");

fprintf("LAYOUT_SIMULINK_CORRESPONDENCE_OK\n");
fprintf("Excel rows included: %d:%d (%d boxes)\n", firstExcelID, height(boxTable), layoutN);
fprintf("Max position error: %.12g\n", posErr);
fprintf("Max dimension error: %.12g\n", dimErr);
disp(table((0:layoutN-1).', [layoutPlan.sourceID].', layoutPos(:,1), ...
    layoutPos(:,2), layoutPos(:,3), layoutDims(:,1), layoutDims(:,2), layoutDims(:,3), ...
    'VariableNames', ["SimulinkID","ExcelRow","X","Y","Z","L","W","H"]));
end
