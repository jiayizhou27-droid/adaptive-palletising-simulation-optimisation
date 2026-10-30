function layoutPlan = buildLayoutPlanBatch3(boxData, palletDim, palletLoc, batchSize, margin, boxGap, buildExecution, allowPartial, geneHints)
%BUILDLAYOUTPLANBATCH3 Rolling-window mixed-layer pallet optimizer.
% The legacy entry point is retained for Simulink compatibility. Boxes are
% never tipped: the only admissible orientations are [L W H] and [W L H].

if nargin < 4 || isempty(batchSize), batchSize = 6; end
if nargin < 5 || isempty(margin), margin = 0; end
if nargin < 6 || isempty(boxGap), boxGap = 0; end
if nargin < 7 || isempty(buildExecution), buildExecution = true; end
if nargin < 8 || isempty(allowPartial), allowPartial = false; end
if nargin < 9 || isempty(geneHints), geneHints = struct(); end

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
        if shouldAdvanceLayer(pending(1),geneHints,boxesInLayer,placed, ...
                layerBaseZ,usableL*usableW)
            layer = layer + 1;
            layerBaseZ = max([placed.z] + [placed.H]);
            layerHeight = NaN;
            freeRects = newLayerRect(xMin, yMin, usableL, usableW);
            boxesInLayer = 0;
        end
        if boxesInLayer >= maxBoxesPerLayer
            layer = layer + 1;
            layerBaseZ = max([placed.z] + [placed.H]);
            layerHeight = NaN;
            freeRects = newLayerRect(xMin, yMin, usableL, usableW);
            boxesInLayer = 0;
        end
        [ok, candidate] = chooseBestCandidate(pending, boxData, freeRects, ...
            placed, layerBaseZ, layerHeight, layer, boxGap, palletLoc,geneHints);
        if ~ok
            if isempty(placed)
                if allowPartial
                    assignin('base','layoutPlan',layoutPlan);
                    return
                end
                error("buildLayoutPlanBatch3:NoFit", ...
                    "No box in the first rolling window fits the pallet.");
            end
            layer = layer + 1;
            layerBaseZ = max([placed.z] + [placed.H]);
            layerHeight = NaN;
            freeRects = newLayerRect(xMin, yMin, usableL, usableW);
            boxesInLayer = 0;
            [ok, candidate] = chooseBestCandidate(pending, boxData, freeRects, ...
                placed, layerBaseZ, layerHeight, layer, boxGap, palletLoc,geneHints);
            if ~ok
                if allowPartial
                    assignin('base','layoutPlan',layoutPlan);
                    return
                end
                error("buildLayoutPlanBatch3:UnsupportedLayer", ...
                    "Box %d cannot be placed with at least 95%% base support and 10 mm COM margin.", pending(1));
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

if ~allowPartial
    assert(all([layoutPlan.isPlaced]), "Not every carton was placed.");
end
if buildExecution
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
end
assignin('base','layoutPlan',layoutPlan);
end

function [ok, best] = chooseBestCandidate(ids, boxData, freeRects, placed, ...
        layerBaseZ, layerHeight, layer, boxGap, palletLoc,geneHints)
ok = false;
best.score = inf;
best.freeIndex = 0;
best.rec = emptyPlacedRecord();
for ii = 1:numel(ids)
    id = ids(ii);
    base = double([boxData.length(id), boxData.width(id), boxData.height(id)]);
    rotations=allowedRotations(id,geneHints);
    for rot = rotations
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
            [xCandidates,yCandidates]=candidateCoordinates(fr,L,W,layerBaseZ,placed);
            for xi=1:numel(xCandidates)
            for yi=1:numel(yCandidates)
            x=xCandidates(xi); y=yCandidates(yi);
            if x<fr.x-1e-9 || y<fr.y-1e-9 || ...
                    x+reqL>fr.x+fr.L+1e-9 || y+reqW>fr.y+fr.W+1e-9
                continue
            end
            support = supportRatio(x, y, L, W, layerBaseZ, placed);
            % Strict practical rule: at most five percent overhang.
            if support < 0.95 - 1e-9, continue; end
            stabilityMargin = supportCOMMargin(x,y,L,W,layerBaseZ,placed);
            if stabilityMargin < 0.010 - 1e-9, continue; end
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
end
end

function rotations=allowedRotations(id,geneHints)
rotations=0:1;
if isfield(geneHints,'orientation') && numel(geneHints.orientation)>=id
    value=double(geneHints.orientation(id));
    if isfinite(value) && any(round(value)==[0 1])
        rotations=round(value);
    end
end
end

function advance=shouldAdvanceLayer(id,geneHints,boxesInLayer,placed,layerBaseZ,palletArea)
% A high layer-preference gene may close a sufficiently occupied layer.
% It can never create an unsupported placement: the normal 95% support and
% 10 mm COM-margin tests still decide whether the next layer is feasible.
advance=false;
if boxesInLayer<2 || isempty(placed) || ...
        ~isfield(geneHints,'layerPreference') || ...
        numel(geneHints.layerPreference)<id
    return
end
g=max(0,min(1,double(geneHints.layerPreference(id))));
if g<0.80,return;end
onLayer=abs([placed.z]-layerBaseZ)<1e-8;
coverage=sum([placed(onLayer).L].*[placed(onLayer).W])/palletArea;
advance=coverage >= 0.55+0.30*(1-g);
end

function [xs,ys]=candidateCoordinates(fr,L,W,z,placed)
xs=fr.x; ys=fr.y;
if isempty(placed) || z<=min([placed.z])+1e-8, return; end
for i=1:numel(placed)
    p=placed(i);
    if abs((p.z+p.H)-z)>1e-8, continue; end
    xs=[xs,p.x,p.x+p.L-L]; %#ok<AGROW>
    ys=[ys,p.y,p.y+p.W-W]; %#ok<AGROW>
end
xs=unique(xs); ys=unique(ys);
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

function margin=supportCOMMargin(x,y,L,W,z,placed)
% Signed distance from footprint centre to the union-support boundary.
if isempty(placed) || z<=min([placed.z])+1e-8
    margin=inf;
    return
end
shape=polyshape();
for i=1:numel(placed)
    p=placed(i);
    if abs((p.z+p.H)-z)>1e-8, continue; end
    x1=max(x,p.x); x2=min(x+L,p.x+p.L);
    y1=max(y,p.y); y2=min(y+W,p.y+p.W);
    if x2-x1>1e-9 && y2-y1>1e-9
        shape=union(shape,polyshape([x1 x2 x2 x1],[y1 y1 y2 y2]));
    end
end
if isempty(shape.Vertices), margin=-inf; return; end
cx=x+L/2; cy=y+W/2;
[vx,vy]=boundary(shape);
distance=pointBoundaryDistance(cx,cy,vx,vy);
if isinterior(shape,cx,cy), margin=distance; else, margin=-distance; end
end

function d=pointBoundaryDistance(px,py,x,y)
d=inf; breaks=[0;find(isnan(x));numel(x)+1];
for b=1:numel(breaks)-1
    ids=(breaks(b)+1):(breaks(b+1)-1);
    if numel(ids)<2, continue; end
    xx=x(ids); yy=y(ids);
    if xx(1)~=xx(end)||yy(1)~=yy(end),xx(end+1)=xx(1);yy(end+1)=yy(1);end %#ok<AGROW>
    for k=1:numel(xx)-1
        dx=xx(k+1)-xx(k);dy=yy(k+1)-yy(k);
        t=max(0,min(1,((px-xx(k))*dx+(py-yy(k))*dy)/(dx*dx+dy*dy)));
        d=min(d,hypot(px-(xx(k)+t*dx),py-(yy(k)+t*dy)));
    end
end
end

function rects = splitAndCleanFreeRects(rects, idx, rec, gap)
% General MaxRects-style split, valid even when placement is not at the
% lower-left corner of a free rectangle. idx is retained for compatibility.
newRects=repmat(makeRect(0,0,0,0),0,1); %#ok<NASGU>
ux1=rec.x; uy1=rec.y; ux2=rec.x+rec.L+gap; uy2=rec.y+rec.W+gap;
for k=1:numel(rects)
    fr=rects(k); fx1=fr.x; fy1=fr.y; fx2=fr.x+fr.L; fy2=fr.y+fr.W;
    if ux2<=fx1+1e-9 || ux1>=fx2-1e-9 || uy2<=fy1+1e-9 || uy1>=fy2-1e-9
        newRects(end+1)=fr; %#ok<AGROW>
        continue
    end
    candidates=[makeRect(fx1,fy1,max(0,ux1-fx1),fr.W), ...
        makeRect(min(ux2,fx2),fy1,max(0,fx2-ux2),fr.W), ...
        makeRect(fx1,fy1,fr.L,max(0,uy1-fy1)), ...
        makeRect(fx1,min(uy2,fy2),fr.L,max(0,fy2-uy2))];
    for c=1:numel(candidates)
        if candidates(c).L>1e-8 && candidates(c).W>1e-8
            newRects(end+1)=candidates(c); %#ok<AGROW>
        end
    end
end
rects=newRects;
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
