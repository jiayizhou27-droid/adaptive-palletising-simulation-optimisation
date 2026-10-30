function verifyLayoutWithinPallet()
%VERIFYLAYOUTWITHINPALLET Check every plotted box lies inside pallet bounds.

cfg = palletTaskConfig();
boxTable = readtable("boxes.xlsx");

boxDataAll.length = boxTable.L;
boxDataAll.width = boxTable.W;
boxDataAll.height = boxTable.H;
boxDataAll.weight = boxTable.Mass;
boxData = sliceBoxData(boxDataAll, cfg.firstExcelID);
layoutPlan = buildLayoutPlanBatch3(boxData, cfg.palletDim, cfg.palletLoc, cfg.batchSize, cfg.palletMargin, cfg.boxGap);

xMin = cfg.palletLoc(1) - cfg.palletDim(1)/2 + cfg.palletMargin;
xMax = cfg.palletLoc(1) + cfg.palletDim(1)/2 - cfg.palletMargin;
yMin = cfg.palletLoc(2) - cfg.palletDim(2)/2 + cfg.palletMargin;
yMax = cfg.palletLoc(2) + cfg.palletDim(2)/2 - cfg.palletMargin;
zTop = cfg.palletLoc(3) + cfg.palletDim(3)/2;

for i = 1:numel(layoutPlan)
    b = layoutPlan(i);
    bxMin = b.pos(1) - b.L/2;
    bxMax = b.pos(1) + b.L/2;
    byMin = b.pos(2) - b.W/2;
    byMax = b.pos(2) + b.W/2;
    bzMin = b.pos(3) - b.H/2;

    assert(bxMin >= xMin - 1e-9 && bxMax <= xMax + 1e-9, ...
        "Box %d is outside pallet X bounds.", b.ID);
    assert(byMin >= yMin - 1e-9 && byMax <= yMax + 1e-9, ...
        "Box %d is outside pallet Y bounds.", b.ID);
    assert(bzMin >= zTop - 1e-9, ...
        "Box %d is below the pallet top.", b.ID);
end

fprintf("LAYOUT_WITHIN_PALLET_OK\n");
fprintf("Checked %d boxes against pallet bounds.\n", numel(layoutPlan));
end
