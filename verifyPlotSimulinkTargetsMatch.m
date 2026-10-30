function verifyPlotSimulinkTargetsMatch()
%VERIFYPLOTSIMULINKTARGETSMATCH Ensure plot and Simulink use same targets.

model = "PalletizeBoxesUsingCobot";
projectDir = fileparts(mfilename("fullpath"));

cfg = palletTaskConfig();
boxTable = readtable(fullfile(projectDir, "boxes.xlsx"));
boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
plotBoxData = sliceBoxData(boxDataAll, cfg.firstExcelID);
plotPlan = buildLayoutPlanBatch3( ...
    plotBoxData, cfg.palletDim, cfg.palletLoc, cfg.batchSize, ...
    cfg.palletMargin, cfg.boxGap);
plotPos = reshape([plotPlan.pos], 3, []).';
plotDims = [[plotPlan.L].' [plotPlan.W].' [plotPlan.H].'];

load_system(model);
initFcn = get_param(model, "InitFcn");
evalin("base", "clear boxData boxDataAll boxTable layoutPlan layoutPos layoutYaw layoutH layoutN boxDim boxWeight boxPosition pickPosition boxToEEOffset suctionCenterOffsetCheck maxNumOfBoxes");
evalin("base", initFcn);

simPlan = evalin("base", "layoutPlan");
simPos = evalin("base", "layoutPos");
simDims = [[simPlan.L].' [simPlan.W].' [simPlan.H].'];

assert(numel(plotPlan) == numel(simPlan), ...
    "Plot and Simulink layoutPlan lengths differ.");
assert(size(plotPos, 1) == size(simPos, 1), ...
    "Plot and Simulink layoutPos lengths differ.");

posErr = max(abs(plotPos - simPos), [], "all");
dimErr = max(abs(plotDims - simDims), [], "all");
idMatch = isequal([plotPlan.sourceID].', [simPlan.sourceID].');

assert(posErr == 0, "Plot and Simulink target positions differ.");
assert(dimErr == 0, "Plot and Simulink target dimensions differ.");
assert(idMatch, "Plot and Simulink source IDs differ.");

fprintf("PLOT_SIMULINK_TARGETS_MATCH_OK\n");
fprintf("Boxes checked: %d\n", numel(plotPlan));
fprintf("Max position error: %.12g\n", posErr);
fprintf("Max dimension error: %.12g\n", dimErr);

visibleStart = cfg.plotStartBox;
visibleEnd = min(visibleStart + 9, numel(plotPlan));
visibleIDs = (visibleStart:visibleEnd).';
actorIDs = visibleIDs - 1;
visiblePos = plotPos(visibleIDs,:);
visibleSim3D = zeros(numel(visibleIDs), 3);
for k = 1:numel(visibleIDs)
    visibleSim3D(k,:) = layoutToSim3DPosition(visiblePos(k,:));
end

fprintf("Visible Simulink actor to 3D-plot target mapping:\n");
disp(table(actorIDs, visibleIDs, [plotPlan(visibleIDs).sourceID].', ...
    visiblePos(:,1), visiblePos(:,2), visiblePos(:,3), ...
    visibleSim3D(:,1), visibleSim3D(:,2), visibleSim3D(:,3), ...
    plotDims(visibleIDs,1), plotDims(visibleIDs,2), plotDims(visibleIDs,3), ...
    'VariableNames', ["ActorID","PlotBoxID","ExcelRow", ...
    "WorldX","WorldY","WorldZ","Sim3DX","Sim3DY","Sim3DZ","L","W","H"]));
end
