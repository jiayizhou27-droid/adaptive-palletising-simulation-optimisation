function verifyLiftTransferGeometry()
% verifyLiftTransferGeometry Verify deterministic lift-transfer-place motion.

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

placed = struct('ID',{},'sourceID',{},'xMin',{},'xMax',{}, ...
    'yMin',{},'yMax',{},'zMin',{},'zMax',{});
clearance = cfg.topClearance;

for i = 1:numel(layoutPlan)
    box = layoutPlan(i);
    assert(box.isPlaced, "Layout box %d was not placed.", i);

    moving = boxAabb(box);
    placedTop = palletLoc(3) + palletDim(3)/2;
    if ~isempty(placed)
        placedTop = max(placedTop, max([placed.zMax]));
    end

    safeBottomZ = placedTop + clearance;
    safeCenterZ = safeBottomZ + box.H/2;

    assert(safeCenterZ >= box.z, ...
        "Safe transfer height is below target center for layout box %d.", box.ID);

    for j = 1:numel(placed)
        p = placed(j);

        horizontalOverlap = intervalsOverlap(moving.xMin, moving.xMax, p.xMin, p.xMax) && ...
            intervalsOverlap(moving.yMin, moving.yMax, p.yMin, p.yMax);

        assert(~horizontalOverlap || safeBottomZ >= p.zMax + clearance - 1e-9, ...
            "Lift-transfer path is too low over Excel row %d while placing Excel row %d.", ...
            p.sourceID, box.sourceID);

        finalBottom = moving.zMin;
        assert(~horizontalOverlap || finalBottom >= p.zMax - 1e-9, ...
            "Final descent penetrates Excel row %d while placing Excel row %d.", ...
            p.sourceID, box.sourceID);
    end

    placed(end+1) = moving; %#ok<AGROW>
end

fprintf("LIFT_TRANSFER_GEOMETRY_OK\n");
fprintf("Checked Excel rows %d:%d (%d boxes), clearance %.3f m.\n", ...
    firstExcelID, height(boxTable), numel(layoutPlan), clearance);
end

function a = boxAabb(box)
a.ID = box.ID;
a.sourceID = box.sourceID;
% layoutPlan stores x/y/z as center coordinates. The packing helper uses
% corner coordinates internally, but writes cx/cy/cz back to layoutPlan.
a.xMin = box.x - box.L/2;
a.xMax = box.x + box.L/2;
a.yMin = box.y - box.W/2;
a.yMax = box.y + box.W/2;
a.zMin = box.z - box.H/2;
a.zMax = box.z + box.H/2;
end

function tf = intervalsOverlap(aMin, aMax, bMin, bMax)
tf = aMin < bMax - 1e-9 && bMin < aMax - 1e-9;
end
