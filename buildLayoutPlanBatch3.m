function layoutPlan = buildLayoutPlanBatch3(boxData, palletDim, palletLoc, batchSize, margin, boxGap)
%BUILDLAYOUTPLANBATCH3 Rolling-window mixed-layer pallet optimizer.
% The legacy entry point is retained for Simulink compatibility. Boxes are
% never tipped: the only admissible orientations are [L W H] and [W L H].

if nargin < 4 || isempty(batchSize), batchSize = 6; end
if nargin < 5 || isempty(margin), margin = 0; end
if nargin < 6 || isempty(boxGap), boxGap = 0; end

n = numel(boxData.length);
assert(numel(boxData.width) == n && numel(boxData.height) == n, ...
    "Box dimension arrays must have equal lengths.");

layoutPlan = repmat(emptyPlanRecord(), n, 1);
for i = 1:n
    dims = double([boxData.length(i), boxData.width(i), boxData.height(i)]);
    validateUprightCuboid(dims, i);
    layoutPlan(i).ID = i;
    layoutPlan(i).sourceID = getSourceID(boxData, i);
    layoutPlan(i).L = dims(1);
    layoutPlan(i).W = dims(2);
    layoutPlan(i).H = dims(3);
    layoutPlan(i).weight = double(boxData.weight(i));
    layoutPlan(i).type = getBoxType(boxData, i, dims);
end

Lp = double(palletDim(1));
Wp = double(palletDim(2));
Hp = double(palletDim(3));
xMin = double(palletLoc(1)) - Lp/2 + margin;
yMin = double(palletLoc(2)) - Wp/2 + margin;
usableL = Lp - 2*margin;
usableW = Wp - 2*margin;
assert(usableL > 0 && usableW > 0, "Pallet margin leaves no usable area.");
zTop = double(palletLoc(3)) + Hp/2;

placed = emptyPlaced();
layer = 1;
layerBaseZ = zTop;
layerHeight = NaN;
freeRects = newLayerRect(xMin, yMin, usableL, usableW);
placementRank = 0;
try
    taskCfg = palletTaskConfig();
    maxBoxesPerLayer = taskCfg.targetBoxesPerLayer;
catch
    maxBoxesPerLayer = inf;
end
boxesInLayer = 0;

% Each fixed rolling window may be resequenced by the adjacent buffer. The
% next window is not opened until every item in the current one is placed.
for windowStart = 1:batchSize:n
    windowIDs = windowStart:min(windowStart + batchSize - 1, n);
    pending = windowIDs;
    while ~isempty(pending)
        if boxesInLayer >= maxBoxesPerLayer
            layer = layer + 1;
            layerBaseZ = max([placed.z] + [placed.H]);
            layerHeight = NaN;
            freeRects = newLayerRect(xMin, yMin, usableL, usableW);
            boxesInLayer = 0;
        end
        [ok, candidate] = chooseBestCandidate(pending, boxData, freeRects, ...
            placed, layerBaseZ, layerHeight, layer, boxGap, palletLoc);
        if ~ok
            if isempty(placed)
                error("buildLayoutPlanBatch3:NoFit", ...
                    "No box in the first rolling window fits the pallet.");
            end
            layer = layer + 1;
            layerBaseZ = max([placed.z] + [placed.H]);
            layerHeight = NaN;
            freeRects = newLayerRect(xMin, yMin, usableL, usableW);
            boxesInLayer = 0;
            [ok, candidate] = chooseBestCandidate(pending, boxData, freeRects, ...
                placed, layerBaseZ, layerHeight, layer, boxGap, palletLoc);
            if ~ok
                error("buildLayoutPlanBatch3:UnsupportedLayer", ...
                    "Box %d cannot be placed with at least 80%% base support.", pending(1));
            end
        end

        placementRank = placementRank + 1;
        candidate.rec.placementRank = placementRank;
        placed(end+1) = candidate.rec; %#ok<AGROW>
        boxesInLayer = boxesInLayer + 1;
        freeRects = splitAndCleanFreeRects(freeRects, candidate.freeIndex, ...
            candidate.rec, boxGap);
        if isnan(layerHeight), layerHeight = candidate.rec.H; end

        id = candidate.rec.ID;
        layoutPlan(id) = planFromPlaced(layoutPlan(id), candidate.rec);
        pending(pending == id) = [];
    end
end

assert(all([layoutPlan.isPlaced]), "Not every carton was placed.");
try
    cfg = palletTaskConfig();
    executionPlan = buildBufferExecutionPlan(layoutPlan, cfg);
    for i = 1:n
        layoutPlan(i).wasBuffered = any([executionPlan.boxID] == i & ...
            strcmp({executionPlan.action}, 'conveyor_to_buffer'));
        hit = find([executionPlan.boxID] == i & [executionPlan.bufferSlot] > 0, 1);
        if ~isempty(hit), layoutPlan(i).bufferSlot = executionPlan(hit).bufferSlot; end
    end
    assignin('base','executionPlan',executionPlan);
catch ME
    warning("PalletizeBoxes:BufferPlan", "Buffer plan unavailable: %s", ME.message);
end
assignin('base','layoutPlan',layoutPlan);
end

function [ok, best] = chooseBestCandidate(ids, boxData, freeRects, placed, ...
        layerBaseZ, layerHeight, layer, boxGap, palletLoc)
ok = false;
best.score = inf;
best.freeIndex = 0;
best.rec = emptyPlacedRecord();
for ii = 1:numel(ids)
    id = ids(ii);
    base = double([boxData.length(id), boxData.width(id), boxData.height(id)]);
    for rot = 0:1
        if rot == 0
            L = base(1); W = base(2); yaw = 0;
        else
            L = base(2); W = base(1); yaw = pi/2;
        end
        H = base(3); % Height axis is invariant by design.
        if ~isnan(layerHeight) && abs(H-layerHeight) > 1e-8, continue; end
        for f = 1:numel(freeRects)
            fr = freeRects(f);
            reqL = L + boxGap;
            reqW = W + boxGap;
            if reqL > fr.L + 1e-9 || reqW > fr.W + 1e-9, continue; end
            x = fr.x; y = fr.y;
            support = supportRatio(x, y, L, W, layerBaseZ, placed);
            if support < 0.80 - 1e-9, continue; end
            waste = fr.L*fr.W - reqL*reqW;
            shortWaste = min(fr.L-reqL, fr.W-reqW);
            centerDistance = hypot(x+L/2-palletLoc(1), y+W/2-palletLoc(2));
            edgeFit = min(abs(fr.L-reqL), abs(fr.W-reqW));
            score = 20*waste + 2*shortWaste + 0.08*centerDistance + 0.5*edgeFit;
            if score < best.score - 1e-12
                rec = emptyPlacedRecord();
                rec.ID = id; rec.x = x; rec.y = y; rec.z = layerBaseZ;
                rec.L = L; rec.W = W; rec.H = H; rec.yaw = yaw;
                rec.cx = x + L/2; rec.cy = y + W/2; rec.cz = layerBaseZ + H/2;
                rec.layer = layer; rec.supportRatio = support;
                best.score = score; best.freeIndex = f; best.rec = rec; ok = true;
            end
        end
    end
end
end

function ratio = supportRatio(x, y, L, W, z, placed)
if isempty(placed)
    ratio = 1;
    return;
end
tol = 1e-8;
if z <= min([placed.z]) + tol
    ratio = 1;
    return;
end
area = 0;
for i = 1:numel(placed)
    p = placed(i);
    if abs((p.z+p.H)-z) > tol, continue; end
    dx = max(0, min(x+L,p.x+p.L)-max(x,p.x));
    dy = max(0, min(y+W,p.y+p.W)-max(y,p.y));
    area = area + dx*dy;
end
ratio = min(1, area/(L*W));
end

function rects = splitAndCleanFreeRects(rects, idx, rec, gap)
fr = rects(idx);
rects(idx) = [];
usedL = rec.L + gap; usedW = rec.W + gap;
rightWide = fr.L-usedL;
topHigh = fr.W-usedW;
if rightWide >= topHigh
    a = makeRect(fr.x+usedL, fr.y, rightWide, fr.W);
    b = makeRect(fr.x, fr.y+usedW, usedL, topHigh);
else
    a = makeRect(fr.x+usedL, fr.y, rightWide, usedW);
    b = makeRect(fr.x, fr.y+usedW, fr.L, topHigh);
end
if a.L > 1e-8 && a.W > 1e-8, rects(end+1) = a; end %#ok<AGROW>
if b.L > 1e-8 && b.W > 1e-8, rects(end+1) = b; end %#ok<AGROW>
rects = removeContainedRects(rects);
end

function rects = removeContainedRects(rects)
keep = true(1,numel(rects));
for i = 1:numel(rects)
    for j = 1:numel(rects)
        if i == j || ~keep(i), continue; end
        if rects(i).x >= rects(j).x-1e-9 && rects(i).y >= rects(j).y-1e-9 && ...
           rects(i).x+rects(i).L <= rects(j).x+rects(j).L+1e-9 && ...
           rects(i).y+rects(i).W <= rects(j).y+rects(j).W+1e-9
            keep(i) = false;
        end
    end
end
rects = rects(keep);
end

function validateUprightCuboid(dims, id)
assert(all(isfinite(dims)) && all(dims > 0), "Box %d has invalid dimensions.", id);
assert(max(dims)-min(dims) > 1e-6, "Box %d is a cube; rectangular cuboids are required.", id);
faces = [dims(1)*dims(2), dims(1)*dims(3), dims(2)*dims(3)];
[~,order] = sort(faces,'descend');
bottomRank = find(order == 1, 1);
assert(bottomRank <= 2, ...
    "Box %d violates the upright rule: L-by-W is the smallest face.", id);
assert(min(dims(1:2)) >= 0.18-1e-9, ...
    "Box %d footprint is too small for the pallet gaps.", id);
end

function out = planFromPlaced(out, p)
out.x=p.cx; out.y=p.cy; out.z=p.cz; out.yaw=p.yaw;
out.L=p.L; out.W=p.W; out.H=p.H; out.isPlaced=true;
out.pos=[p.cx p.cy p.cz]; out.layer=p.layer;
out.placementRank=p.placementRank; out.supportRatio=p.supportRatio;
end

function sourceID = getSourceID(boxData, idx)
if isfield(boxData,'sourceID') && numel(boxData.sourceID)>=idx
    sourceID=double(boxData.sourceID(idx));
else
    sourceID=idx;
end
end

function type = getBoxType(boxData, idx, dims)
if isfield(boxData,'type') && numel(boxData.type)>=idx
    type=string(boxData.type(idx));
else
    type=sprintf("%.0fx%.0fx%.0f",1000*dims(1),1000*dims(2),1000*dims(3));
end
end

function r = newLayerRect(x,y,L,W), r=makeRect(x,y,L,W); end
function r = makeRect(x,y,L,W), r=struct('x',x,'y',y,'L',L,'W',W); end
function p = emptyPlaced(), p=repmat(emptyPlacedRecord(),0,1); end
function r = emptyPlacedRecord()
r=struct('ID',0,'x',0,'y',0,'z',0,'L',0,'W',0,'H',0,'yaw',0, ...
    'cx',0,'cy',0,'cz',0,'layer',0,'placementRank',0,'supportRatio',0);
end
function r = emptyPlanRecord()
r=struct('ID',0,'x',0,'y',0,'z',0,'yaw',0,'L',0,'W',0,'H',0, ...
    'weight',0,'isPlaced',false,'pos',[0 0 0],'layer',0,'sourceID',0, ...
    'type',"",'placementRank',0,'supportRatio',0,'wasBuffered',false,'bufferSlot',0);
end
