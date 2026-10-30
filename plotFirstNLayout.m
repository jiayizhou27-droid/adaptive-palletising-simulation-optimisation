function [layoutPlan, outPath] = plotFirstNLayout(n, outPath, startIndex)
%PLOTFIRSTNLAYOUT Plot the first N optimized boxes for Simulink comparison.

if nargin < 1 || isempty(n)
    n = 10;
end
if nargin < 2
    outPath = "";
end
if nargin < 3 || isempty(startIndex)
    cfg = palletTaskConfig();
    startIndex = cfg.plotStartBox;
end

cfg = palletTaskConfig();
boxTable = readtable("boxes.xlsx");

boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
boxData = sliceBoxData(boxDataAll, cfg.firstExcelID);

layoutPlan = buildLayoutPlanBatch3( ...
    boxData, cfg.palletDim, cfg.palletLoc, cfg.batchSize, ...
    cfg.palletMargin, cfg.boxGap);

startIndex = max(1, min(double(startIndex), numel(layoutPlan)));
endIndex = min(startIndex + n - 1, numel(layoutPlan));
plotIDs = startIndex:endIndex;
n = numel(plotIDs);

visibleMode = "on";
if strlength(string(outPath)) > 0
    visibleMode = "off";
end
fig = figure("Color", "w", "Visible", visibleMode);
ax = axes(fig);
hold(ax, "on");
axis(ax, "equal");
grid(ax, "on");
view(ax, 35, 25);
xlabel(ax, "X");
ylabel(ax, "Y");
zlabel(ax, "Z");
title(ax, sprintf("Layout Boxes %d-%d Target", plotIDs(1), plotIDs(end)));

palletOrigin = cfg.palletLoc - cfg.palletDim/2;
drawCuboid(ax, palletOrigin, cfg.palletDim, [0.75 0.75 0.75], 0.25);

safeOrigin = [ ...
    cfg.palletLoc(1) - cfg.palletDim(1)/2 + cfg.palletMargin, ...
    cfg.palletLoc(2) - cfg.palletDim(2)/2 + cfg.palletMargin, ...
    cfg.palletLoc(3) + cfg.palletDim(3)/2 + 0.004];
safeSize = [ ...
    cfg.palletDim(1) - 2*cfg.palletMargin, ...
    cfg.palletDim(2) - 2*cfg.palletMargin];
plot3(ax, ...
    [safeOrigin(1), safeOrigin(1)+safeSize(1), safeOrigin(1)+safeSize(1), safeOrigin(1), safeOrigin(1)], ...
    [safeOrigin(2), safeOrigin(2), safeOrigin(2)+safeSize(2), safeOrigin(2)+safeSize(2), safeOrigin(2)], ...
    safeOrigin(3)*ones(1,5), "--", "Color", [0 0.35 0.9], "LineWidth", 1.8);

colors = lines(max(n, 1));
for localIdx = 1:n
    i = plotIDs(localIdx);
    b = layoutPlan(i);
    if ~b.isPlaced
        continue;
    end

    origin = b.pos - [b.L b.W b.H]/2;
    drawCuboid(ax, origin, [b.L b.W b.H], colors(localIdx,:), 0.82);
    text(ax, b.pos(1), b.pos(2), b.pos(3)+b.H/2+0.025, ...
        sprintf("%d", i), "HorizontalAlignment", "center", ...
        "FontWeight", "bold", "FontSize", 10, "Color", [0 0 0]);
end

pad = 0.2;
xlim(ax, [cfg.palletLoc(1)-cfg.palletDim(1)/2-pad, cfg.palletLoc(1)+cfg.palletDim(1)/2+pad]);
ylim(ax, [cfg.palletLoc(2)-cfg.palletDim(2)/2-pad, cfg.palletLoc(2)+cfg.palletDim(2)/2+pad]);
zlim(ax, [0, max([layoutPlan(plotIDs).z] + [layoutPlan(plotIDs).H]/2) + 0.25]);

fprintf("Layout targets %d through %d:\n", plotIDs(1), plotIDs(end));
disp(table(plotIDs.', [layoutPlan(plotIDs).sourceID].', ...
    reshape([layoutPlan(plotIDs).pos], 3, []).', ...
    [layoutPlan(plotIDs).L].', [layoutPlan(plotIDs).W].', [layoutPlan(plotIDs).H].', ...
    [layoutPlan(plotIDs).yaw].', ...
    'VariableNames', ["BoxID","ExcelRow","Position","L","W","H","Yaw"]));

if strlength(string(outPath)) > 0
    exportgraphics(fig, outPath, "Resolution", 180);
end
end

function drawCuboid(ax, origin, dims, faceColor, faceAlpha)
x = origin(1); y = origin(2); z = origin(3);
L = dims(1); W = dims(2); H = dims(3);

V = [ ...
    x   y   z;
    x+L y   z;
    x+L y+W z;
    x   y+W z;
    x   y   z+H;
    x+L y   z+H;
    x+L y+W z+H;
    x   y+W z+H];

F = [ ...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];

patch(ax, "Vertices", V, "Faces", F, ...
    "FaceColor", faceColor, "FaceAlpha", faceAlpha, ...
    "EdgeColor", "k", "LineWidth", 1.0);
end
