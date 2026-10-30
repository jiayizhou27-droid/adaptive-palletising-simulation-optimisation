function collisionArraySubset = exampleHelperGetCollisionEnvironment(ID,graspState,boxDim,boxPosition,maxNumOfBoxes,baseLoc,baseDim,benchDim,benchLoc,palletDim,palletLoc)
%exampleHelperGetCollisionEnvironment Set up the collision environment.

persistent collisionArray
coder.varsize('collisionArray',[1,50]);

boxDims = getBoxDimsForID(ID, boxDim);

if isempty(collisionArray)
    collisionArray = cell(1,0);

    for i = coder.unroll(1:50)
        if i <= maxNumOfBoxes + 3
            collisionArray{end+1} = collisionBox( ...
                boxDims(1), boxDims(2), boxDims(3), ...
                Pose=trvec2tform([3,3,0]));
        end
    end

    collisionArray{1}.X = baseDim(1);
    collisionArray{1}.Y = baseDim(2);
    collisionArray{1}.Z = baseDim(3);
    collisionArray{1}.Pose = trvec2tform(baseLoc);

    collisionArray{2}.X = benchDim(1);
    collisionArray{2}.Y = benchDim(2);
    collisionArray{2}.Z = benchDim(3);
    collisionArray{2}.Pose = trvec2tform(benchLoc);

    collisionArray{3}.X = max(palletDim(1) - 0.04, 0.01);
    collisionArray{3}.Y = max(palletDim(2) - 0.04, 0.01);
    collisionArray{3}.Z = palletDim(3);
    collisionArray{3}.Pose = trvec2tform(palletLoc);

    collisionArray{4}.X = boxDims(1);
    collisionArray{4}.Y = boxDims(2);
    collisionArray{4}.Z = boxDims(3);
    collisionArray{4}.Pose = trvec2tform(boxPosition);
end

if graspState == false
    if ID < maxNumOfBoxes
        nextDims = getBoxDimsForID(ID+1, boxDim);
        collisionArray{ID+4}.X = nextDims(1);
        collisionArray{ID+4}.Y = nextDims(2);
        collisionArray{ID+4}.Z = nextDims(3);
        collisionArray{ID+4}.Pose = trvec2tform(boxPosition);
    end

    if ID >= 1
        droppedDims = getBoxDimsForID(ID, boxDim);
        droppedYaw = getLayoutYawFromBase(ID-1);
        droppedBoxPoseUpdate = exampleHelperPalletArrangement( ...
            ID-1,palletDim,palletLoc,droppedDims);

        collisionArray{ID+3}.X = max(droppedDims(1) - 0.02, 0.01);
        collisionArray{ID+3}.Y = max(droppedDims(2) - 0.02, 0.01);
        collisionArray{ID+3}.Z = max(droppedDims(3) - 0.02, 0.01);
        collisionArray{ID+3}.Pose = ...
            trvec2tform(droppedBoxPoseUpdate) * axang2tform([0 0 1 droppedYaw]);
    end
end

if graspState == true
    if ID < maxNumOfBoxes
        collisionArray{ID+4}.Pose = trvec2tform([3,3,0]);
    end
end

collisionArraySubset = cell(1,0);
for i = coder.unroll(1:50)
    if i <= ID+4
        collisionArraySubset{end+1} = collisionArray{i};
    end
end

end

function dims = getBoxDimsForID(ID, fallbackDim)
dims = normalizeBoxDims(fallbackDim);
try
    boxData = evalin('base','boxData');
    n = numel(boxData.length);
    if n > 0
        idx = mod(double(ID), n) + 1;
        layoutDims = getLayoutDims(idx);
        if any(isfinite(layoutDims))
            dims = layoutDims;
        else
            dims = double([boxData.length(idx) boxData.width(idx) boxData.height(idx)]);
        end
    end
catch
end
end

function dims = getLayoutDims(idx)
dims = [NaN NaN NaN];
try
    layoutPlan = evalin('base','layoutPlan');
    if idx >= 1 && idx <= numel(layoutPlan)
        dims = double([layoutPlan(idx).L layoutPlan(idx).W layoutPlan(idx).H]);
    end
catch
end
end

function yaw = getLayoutYawFromBase(id)
yaw = 0;
try
    layoutPlan = evalin('base','layoutPlan');
    n = numel(layoutPlan);
    if n > 0
        idx = mod(double(id), n) + 1;
        yaw = double(layoutPlan(idx).yaw);
    end
catch
end
end

function dims = normalizeBoxDims(boxDim)
if isscalar(boxDim)
    dims = double([boxDim boxDim boxDim]);
else
    dims = double(boxDim(:)).';
    if numel(dims) < 3
        dims = [dims repmat(dims(end), 1, 3-numel(dims))];
    end
    dims = dims(1:3);
end
end
