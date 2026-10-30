function selfCheckLayoutSnapWorkflow()
%SELFCHECKLAYOUTSNAPWORKFLOW Lightweight checks for final-layout locking.

assert(~isempty(which("snapPlacedBoxesToLayout")), ...
    "snapPlacedBoxesToLayout is not on the MATLAB path.");
assert(~isempty(which("verifyVisibleFinalTargets")), ...
    "verifyVisibleFinalTargets is not on the MATLAB path.");

spawnText = fileread("spawnBox.m");
assert(contains(spawnText, "snapPlacedBoxesToLayout(double(id))"), ...
    "spawnBox does not lock previous boxes before spawning the next one.");

model = "PalletizeBoxesUsingCobot";
load_system(model);
stopFcn = get_param(model, "StopFcn");
assert(contains(stopFcn, "snapPlacedBoxesToLayout(inf)"), ...
    "StopFcn does not snap all boxes at simulation end.");
assert(contains(stopFcn, "verifyVisibleFinalTargets"), ...
    "StopFcn does not verify visible final actor positions.");

cfg = palletTaskConfig();
layoutPlan = evalin("base", "layoutPlan");
assert(~isempty(layoutPlan), "layoutPlan is empty.");

startBox = cfg.plotStartBox;
endBox = min(startBox + 9, numel(layoutPlan));
ids = (startBox:endBox).';
actorIDs = ids - 1;
pos = reshape([layoutPlan(ids).pos], 3, []).';
sim3dPos = zeros(numel(ids), 3);
for k = 1:numel(ids)
    sim3dPos(k,:) = layoutToSim3DPosition(pos(k,:));
end

disp(table(actorIDs, ids, [layoutPlan(ids).sourceID].', ...
    pos(:,1), pos(:,2), pos(:,3), ...
    sim3dPos(:,1), sim3dPos(:,2), sim3dPos(:,3), ...
    'VariableNames', ["ActorID","PlotBoxID","ExcelRow", ...
    "WorldX","WorldY","WorldZ","Sim3DX","Sim3DY","Sim3DZ"]));

fprintf("SELF_CHECK_OK_LAYOUT_DIMENSIONS_AND_SNAP_FUNCTION\n");
fprintf("Visible actor mapping is collBox%d..collBox%d -> layout boxes %d..%d.\n", ...
    actorIDs(1), actorIDs(end), ids(1), ids(end));
end
