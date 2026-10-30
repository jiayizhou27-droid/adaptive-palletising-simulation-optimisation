function recovery = replanRemainingBySupport(layoutPlan,acceptedIDs,faultBoxID,cfg)
%REPLANREMAININGBYSUPPORT Recompute a feasible order with placed boxes fixed.
% The global target coordinates are retained. The remaining execution order
% is rebuilt from actual support dependencies, so no accepted carton moves.
if nargin < 4 || isempty(cfg)
    cfg = palletTaskConfig();
end
n = numel(layoutPlan);
accepted = false(1,n);
accepted(acceptedIDs) = true;
remaining = find(~accepted);

% placementRank is the support-certified order produced by the global
% planner. Recompute the remaining subsequence after removing every target
% already confirmed by vision. This preserves the fixed geometry and never
% asks a higher carton to precede the cartons that supported it.
[~,idx] = sort([layoutPlan(remaining).placementRank]);
order = remaining(idx);
valid = all([layoutPlan(order).isPlaced]) && ...
    all([layoutPlan(order).supportRatio] >= 0.95-1e-9);
if ~valid || ~ismember(faultBoxID,order)
    recovery = struct('succeeded',false,'order',order, ...
        'bufferedBoxID',faultBoxID,'bufferSlot',NaN, ...
        'reason',"invalid_support_certified_order");
    return
end
recovery = struct('succeeded',true,'order',order, ...
    'bufferedBoxID',faultBoxID,'bufferSlot',1, ...
    'reason',"support_certified_remaining_order");
end

function ok = targetFeasible(id,plan,accepted,cfg)
b = plan(id);
tol = 1e-8;
bottom = b.pos(3)-b.H/2;
palletTop = cfg.palletLoc(3)+cfg.palletDim(3)/2;
if abs(bottom-palletTop) <= tol
    ok = true;
    return
end

x1=b.pos(1)-b.L/2; x2=b.pos(1)+b.L/2;
y1=b.pos(2)-b.W/2; y2=b.pos(2)+b.W/2;
supportShape=polyshape();
for j=find(accepted)
    p=plan(j);
    if abs((p.pos(3)+p.H/2)-bottom)>tol
        continue
    end
    ox1=max(x1,p.pos(1)-p.L/2); ox2=min(x2,p.pos(1)+p.L/2);
    oy1=max(y1,p.pos(2)-p.W/2); oy2=min(y2,p.pos(2)+p.W/2);
    if ox2-ox1>tol && oy2-oy1>tol
        supportShape=union(supportShape, ...
            polyshape([ox1 ox2 ox2 ox1],[oy1 oy1 oy2 oy2]));
    end
end
if isempty(supportShape.Vertices)
    ok=false;
    return
end
supportRatio=area(supportShape)/(b.L*b.W);
[vx,vy]=boundary(supportShape);
margin=pointBoundaryDistance(b.pos(1),b.pos(2),vx,vy);
inside=isinterior(supportShape,b.pos(1),b.pos(2));
ok=supportRatio>=0.95-1e-9 && inside && margin>=0.010-1e-9;
end

function d=pointBoundaryDistance(px,py,x,y)
d=inf;
breaks=[0;find(isnan(x));numel(x)+1];
for b=1:numel(breaks)-1
    ids=(breaks(b)+1):(breaks(b+1)-1);
    if numel(ids)<2, continue; end
    xx=x(ids); yy=y(ids);
    if xx(1)~=xx(end)||yy(1)~=yy(end)
        xx(end+1)=xx(1); yy(end+1)=yy(1); %#ok<AGROW>
    end
    for k=1:numel(xx)-1
        dx=xx(k+1)-xx(k); dy=yy(k+1)-yy(k);
        denom=dx*dx+dy*dy;
        if denom<eps, continue; end
        t=max(0,min(1,((px-xx(k))*dx+(py-yy(k))*dy)/denom));
        d=min(d,hypot(px-(xx(k)+t*dx),py-(yy(k)+t*dy)));
    end
end
end
