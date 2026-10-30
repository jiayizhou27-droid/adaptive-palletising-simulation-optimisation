function report = verifyVisibleFinalTargets(n, tol)
%VERIFYVISIBLEFINALTARGETS Verify visible Sim3D box actors against layout.
%
% Run after a simulation step or at the end of simulation. The expected
% mapping is:
%   collBox2 -> layoutPlan(3)
%   collBox3 -> layoutPlan(4)
% because the first two warm-up boxes are hidden.

if nargin < 1 || isempty(n)
    n = 10;
end
if nargin < 2 || isempty(tol)
    tol = 1e-4;
end

cfg = palletTaskConfig();
layoutPlan = evalin("base", "layoutPlan");
if isempty(layoutPlan)
    error("layoutPlan is empty. Run model initialization first.");
end

startBox = cfg.plotStartBox;
endBox = min(startBox + n - 1, numel(layoutPlan));
plotIDs = (startBox:endBox).';
actorIDs = plotIDs - 1;

expectedWorld = reshape([layoutPlan(plotIDs).pos], 3, []).';
expectedSim3D = zeros(numel(plotIDs), 3);
actualSim3D = nan(numel(plotIDs), 3);
actualWorld = nan(numel(plotIDs), 3);
posErr = nan(numel(plotIDs), 1);
actorFound = false(numel(plotIDs), 1);

world = sim3d.World.getWorld(bdroot);
for k = 1:numel(plotIDs)
    expectedSim3D(k,:) = layoutToSim3DPosition(expectedWorld(k,:));
    actor = world.Root.findBy("ActorName", "collBox" + actorIDs(k), "first");
    if isempty(actor)
        continue;
    end

    actorFound(k) = true;
    actualSim3D(k,:) = double(actor.Translation) + sim3dActorLocalCenter(actor);
    actualWorld(k,:) = [actualSim3D(k,1), -actualSim3D(k,2), actualSim3D(k,3)];
    posErr(k) = max(abs(actualSim3D(k,:) - expectedSim3D(k,:)));
end

report = table(actorIDs, plotIDs, [layoutPlan(plotIDs).sourceID].', ...
    actorFound, ...
    expectedWorld(:,1), expectedWorld(:,2), expectedWorld(:,3), ...
    actualWorld(:,1), actualWorld(:,2), actualWorld(:,3), posErr, ...
    'VariableNames', ["ActorID","PlotBoxID","ExcelRow","ActorFound", ...
    "ExpectedX","ExpectedY","ExpectedZ","ActualX","ActualY","ActualZ","MaxErr"]);

disp(report);
missing = ~actorFound;
if any(missing)
    error("Missing visible actors: %s", mat2str(actorIDs(missing).'));
end
bad = posErr > tol;
if any(bad)
    error("Visible actor final target mismatch. Bad ActorID(s): %s", ...
        mat2str(actorIDs(bad).'));
end

fprintf("VISIBLE_FINAL_TARGETS_MATCH_OK\n");
fprintf("Checked visible boxes %d through %d with tolerance %.3g m.\n", ...
    startBox, endBox, tol);
end
