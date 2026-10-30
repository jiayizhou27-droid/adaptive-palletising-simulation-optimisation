function metrics = showCurrentOptimizationMetrics(nVisible)
%SHOWCURRENTOPTIMIZATIONMETRICS Print layout and robot metrics for comparison.
%
% The metrics are intended for comparing alternative pallet layout and
% trajectory planning methods in the final report.

if nargin < 1 || isempty(nVisible)
    nVisible = 10;
end

cfg = palletTaskConfig();
projectDir = fileparts(mfilename("fullpath"));
boxTable = readtable(fullfile(projectDir, "boxes.xlsx"));

boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
boxData = sliceBoxData(boxDataAll, cfg.firstExcelID);

tStart = tic;
layoutPlan = buildLayoutPlanBatch3( ...
    boxData, cfg.palletDim, cfg.palletLoc, cfg.batchSize, ...
    cfg.palletMargin, cfg.boxGap);
layoutComputeTime = toc(tStart);

visibleStart = cfg.plotStartBox;
visibleEnd = min(visibleStart + nVisible - 1, numel(layoutPlan));
visibleIDs = visibleStart:visibleEnd;
visiblePlan = layoutPlan(visibleIDs);

palletArea = cfg.palletDim(1) * cfg.palletDim(2);
safeArea = (cfg.palletDim(1) - 2*cfg.palletMargin) * ...
           (cfg.palletDim(2) - 2*cfg.palletMargin);
boxFootprintAreas = [visiblePlan.L] .* [visiblePlan.W];
boxVolumes = boxFootprintAreas .* [visiblePlan.H];
visiblePos = reshape([visiblePlan.pos], 3, []).';
bottomZ = visiblePos(:,3).' - [visiblePlan.H]/2;
topZ = visiblePos(:,3).' + [visiblePlan.H]/2;
stackHeight = max(topZ) - (cfg.palletLoc(3) + cfg.palletDim(3)/2);
boundingVolume = palletArea * max(stackHeight, eps);

edgeClearances = zeros(numel(visiblePlan), 1);
for i = 1:numel(visiblePlan)
    b = visiblePlan(i);
    x0 = b.pos(1) - b.L/2;
    x1 = b.pos(1) + b.L/2;
    y0 = b.pos(2) - b.W/2;
    y1 = b.pos(2) + b.W/2;
    edgeClearances(i) = min([ ...
        x0 - (cfg.palletLoc(1) - cfg.palletDim(1)/2), ...
        (cfg.palletLoc(1) + cfg.palletDim(1)/2) - x1, ...
        y0 - (cfg.palletLoc(2) - cfg.palletDim(2)/2), ...
        (cfg.palletLoc(2) + cfg.palletDim(2)/2) - y1]);
end

minGap = computeMinimumPlanarGap(visiblePlan);
layerTol = 1e-5;
layerBottoms = unique(round(bottomZ / layerTol) * layerTol);
rowCount = numel(unique(round([visiblePlan.pos] / 0.02) * 0.02));
rowCentersY = clusterValues(reshape([visiblePlan.pos], 3, []).', 2, 0.08);
rowAlignmentError = computeRowAlignmentError(visiblePlan, rowCentersY);
supportRatios = computeSupportRatios(visiblePlan, cfg);

pickHeights = cfg.sourceSurfaceZ + [visiblePlan.H]/2 + cfg.sourceSpawnClearance;
eeOffsetZ = [visiblePlan.H]/2 + 0.11;

metrics.Layout = table( ...
    string("Shelf heuristic"), ...
    numel(visiblePlan), ...
    layoutComputeTime, ...
    sum(boxFootprintAreas) / palletArea, ...
    sum(boxFootprintAreas) / safeArea, ...
    sum(boxVolumes) / boundingVolume, ...
    numel(layerBottoms), ...
    stackHeight, ...
    min(edgeClearances), ...
    mean(edgeClearances), ...
    minGap, ...
    numel(rowCentersY), ...
    rowAlignmentError, ...
    min(supportRatios), ...
    'VariableNames', [ ...
    "Method","VisibleBoxes","LayoutComputeTime_s", ...
    "PalletAreaUtilisation","SafeAreaUtilisation", ...
    "BoundingVolumeUtilisation","LayerCount","StackHeight_m", ...
    "MinEdgeClearance_m","MeanEdgeClearance_m","MinBoxGap_m", ...
    "EstimatedRowCount","RowAlignmentError_m","MinSupportRatio"]);

metrics.Robot = table( ...
    cfg.approachHeight, ...
    cfg.transferClearance, ...
    cfg.minTransferHeight, ...
    min(pickHeights), max(pickHeights), ...
    -0.2, min(eeOffsetZ), max(eeOffsetZ), ...
    0.06, 8.0, ...
    'VariableNames', [ ...
    "ApproachHeight_m","TransferClearance_m","MinTransferHeight_m", ...
    "MinPickCenterZ_m","MaxPickCenterZ_m", ...
    "EEOffsetX_m","MinEEOffsetZ_m","MaxEEOffsetZ_m", ...
    "StateTolerance_rad","StateTimeout_s"]);

metrics.VisibleBoxes = table(visibleIDs.', [visiblePlan.sourceID].', ...
    reshape([visiblePlan.pos], 3, []).', ...
    [visiblePlan.L].', [visiblePlan.W].', [visiblePlan.H].', ...
    [visiblePlan.yaw].', ...
    pickHeights.', eeOffsetZ.', ...
    'VariableNames', [ ...
    "LayoutID","ExcelRow","TargetPosition_world","L","W","H","Yaw", ...
    "PickCenterZ_m","EEOffsetZ_m"]);

fprintf("\n=== Layout Comparison Metrics ===\n");
disp(metrics.Layout);
fprintf("\n=== Robot / Trajectory Parameters ===\n");
disp(metrics.Robot);
fprintf("\n=== Visible Box Target Parameters ===\n");
disp(metrics.VisibleBoxes);

fprintf("\nRecommended comparison features:\n");
fprintf("1. Layout compute time, pallet area utilisation, safe-area utilisation.\n");
fprintf("2. Layer count, stack height, minimum edge clearance, minimum box gap.\n");
fprintf("3. Row alignment error and support ratio for layout regularity/stability.\n");
fprintf("4. Robot cycle time, joint travel, end-effector path length, peak velocity.\n");
fprintf("5. Collision count, minimum clearance, state stalls, final actor error.\n\n");
end

function minGap = computeMinimumPlanarGap(plan)
minGap = inf;
for i = 1:numel(plan)
    for j = i+1:numel(plan)
        bottomI = plan(i).pos(3) - plan(i).H/2;
        bottomJ = plan(j).pos(3) - plan(j).H/2;
        if abs(bottomI - bottomJ) > 1e-6
            continue;
        end
        ax0 = plan(i).pos(1) - plan(i).L/2;
        ax1 = plan(i).pos(1) + plan(i).L/2;
        ay0 = plan(i).pos(2) - plan(i).W/2;
        ay1 = plan(i).pos(2) + plan(i).W/2;
        bx0 = plan(j).pos(1) - plan(j).L/2;
        bx1 = plan(j).pos(1) + plan(j).L/2;
        by0 = plan(j).pos(2) - plan(j).W/2;
        by1 = plan(j).pos(2) + plan(j).W/2;
        dx = max([bx0-ax1, ax0-bx1, 0]);
        dy = max([by0-ay1, ay0-by1, 0]);
        if dx == 0 || dy == 0
            gap = max(dx, dy);
        else
            gap = hypot(dx, dy);
        end
        minGap = min(minGap, gap);
    end
end
if isinf(minGap)
    minGap = NaN;
end
end

function centers = clusterValues(pos, dim, tol)
vals = sort(pos(:,dim));
centers = [];
for i = 1:numel(vals)
    if isempty(centers) || abs(vals(i) - centers(end)) > tol
        centers(end+1) = vals(i); %#ok<AGROW>
    else
        centers(end) = 0.5 * (centers(end) + vals(i));
    end
end
end

function err = computeRowAlignmentError(plan, rowCentersY)
pos = reshape([plan.pos], 3, []).';
errVals = zeros(size(pos,1), 1);
for i = 1:size(pos,1)
    errVals(i) = min(abs(pos(i,2) - rowCentersY));
end
err = mean(errVals);
end

function ratios = computeSupportRatios(plan, cfg)
zTop = cfg.palletLoc(3) + cfg.palletDim(3)/2;
ratios = ones(numel(plan), 1);
for i = 1:numel(plan)
    b = plan(i);
    bBottom = b.pos(3) - b.H/2;
    if abs(bBottom - zTop) < 1e-6
        ratios(i) = 1;
        continue;
    end
    area = 0;
    bx0 = b.pos(1) - b.L/2;
    bx1 = b.pos(1) + b.L/2;
    by0 = b.pos(2) - b.W/2;
    by1 = b.pos(2) + b.W/2;
    for j = 1:numel(plan)
        a = plan(j);
        aTop = a.pos(3) + a.H/2;
        if j == i || abs(aTop - bBottom) > 1e-6
            continue;
        end
        ax0 = a.pos(1) - a.L/2;
        ax1 = a.pos(1) + a.L/2;
        ay0 = a.pos(2) - a.W/2;
        ay1 = a.pos(2) + a.W/2;
        ox = max(0, min(bx1, ax1) - max(bx0, ax0));
        oy = max(0, min(by1, ay1) - max(by0, ay0));
        area = area + ox * oy;
    end
    ratios(i) = area / (b.L * b.W);
end
end
