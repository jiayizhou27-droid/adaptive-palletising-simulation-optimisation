function executionPlan = buildBufferExecutionPlan(layoutPlan, cfg)
%BUILDBUFFEREXECUTIONPLAN Buffer only cartons not currently placeable.
% A target is feasible when it is collision-free in the final layout and
% has at least 95 percent footprint support and 10 mm COM margin.
if nargin < 2 || isempty(cfg), cfg=palletTaskConfig(); end
n=numel(layoutPlan);
placed=false(1,n);
bufferIDs=zeros(1,cfg.bufferCapacity);
executionPlan=repmat(emptyAction(),0,1);

% The first Simulink cycle is an intentionally empty warm-up.  It follows
% the normal pick/place trajectory using carton 1 dimensions, but its actor
% is invisible and collision-free and it does not alter placement state.
warmup=makeAction('hidden_warmup',1,0, ...
    sourcePickPosition(cfg,layoutPlan(1).H),layoutPlan(1).pos,layoutPlan);
executionPlan(end+1)=warmup;

for arriving=1:n
    if targetIsFeasible(arriving,layoutPlan,placed,cfg)
        executionPlan(end+1)=makeAction('conveyor_to_pallet',arriving,0, ...
            sourcePickPosition(cfg,layoutPlan(arriving).H),layoutPlan(arriving).pos,layoutPlan); %#ok<AGROW>
        placed(arriving)=true;
        [executionPlan,placed,bufferIDs]=drainFeasibleBuffer( ...
            executionPlan,placed,bufferIDs,layoutPlan,cfg);
    else
        slot=find(bufferIDs==0,1);
        assert(~isempty(slot),'No free buffer slot for physically infeasible box %d.',arriving);
        executionPlan(end+1)=makeAction('conveyor_to_buffer',arriving,slot, ...
            sourcePickPosition(cfg,layoutPlan(arriving).H),bufferCenter(cfg,slot,layoutPlan(arriving).H),layoutPlan); %#ok<AGROW>
        bufferIDs(slot)=arriving;
    end
end

[executionPlan,placed,bufferIDs]=drainFeasibleBuffer( ...
    executionPlan,placed,bufferIDs,layoutPlan,cfg);
assert(all(placed),'Buffered cartons remain physically infeasible after all arrivals.');
assert(~any(bufferIDs),'Buffer is not empty at the end of the plan.');
end

function [actions,placed,bufferIDs]=drainFeasibleBuffer(actions,placed,bufferIDs,plan,cfg)
while true
    candidates=bufferIDs(bufferIDs>0);
    if isempty(candidates), return; end
    ranks=[plan(candidates).placementRank];
    [~,order]=sort(ranks);
    moved=false;
    for k=1:numel(order)
        id=candidates(order(k));
        if ~targetIsFeasible(id,plan,placed,cfg), continue; end
        slot=find(bufferIDs==id,1);
        actions(end+1)=makeAction('buffer_to_pallet',id,slot, ...
            bufferPickPosition(cfg,slot,plan(id).H),plan(id).pos,plan); %#ok<AGROW>
        bufferIDs(slot)=0; placed(id)=true; moved=true; break;
    end
    if ~moved, return; end
end
end

function ok=targetIsFeasible(id,plan,placed,cfg)
b=plan(id); tol=1e-8;
bottom=b.pos(3)-b.H/2;
palletTop=cfg.palletLoc(3)+cfg.palletDim(3)/2;
if abs(bottom-palletTop)<=tol
    support=1;
    stabilityMargin=inf;
else
    supportArea=0;
    supportShape=polyshape();
    x1=b.pos(1)-b.L/2; x2=b.pos(1)+b.L/2;
    y1=b.pos(2)-b.W/2; y2=b.pos(2)+b.W/2;
    ids=find(placed);
    for j=ids
        p=plan(j);
        if abs((p.pos(3)+p.H/2)-bottom)>tol, continue; end
        px1=p.pos(1)-p.L/2; px2=p.pos(1)+p.L/2;
        py1=p.pos(2)-p.W/2; py2=p.pos(2)+p.W/2;
        ox1=max(x1,px1); ox2=min(x2,px2);
        oy1=max(y1,py1); oy2=min(y2,py2);
        supportArea=supportArea+max(0,ox2-ox1)*max(0,oy2-oy1);
        if ox2-ox1>tol && oy2-oy1>tol
            supportShape=union(supportShape,polyshape([ox1 ox2 ox2 ox1],[oy1 oy1 oy2 oy2]));
        end
    end
    support=min(1,supportArea/(b.L*b.W));
    if isempty(supportShape.Vertices)
        stabilityMargin=-inf;
    else
        [vx,vy]=boundary(supportShape);
        distance=pointBoundaryDistance(b.pos(1),b.pos(2),vx,vy);
        if isinterior(supportShape,b.pos(1),b.pos(2))
            stabilityMargin=distance;
        else
            stabilityMargin=-distance;
        end
    end
end
ok=support>=0.95-1e-9 && stabilityMargin>=0.010-1e-9;
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

function p=sourcePickPosition(cfg,H)
p=[cfg.sourceBoxXY cfg.sourceSurfaceZ+H/2+cfg.sourceSpawnClearance];
end
function p=bufferCenter(cfg,slot,H)
col=mod(slot-1,cfg.bufferGrid(2)); row=floor((slot-1)/cfg.bufferGrid(2));
p=[cfg.bufferOriginXY(1)+col*cfg.bufferPitch(1), ...
   cfg.bufferOriginXY(2)-row*cfg.bufferPitch(2),cfg.bufferSurfaceZ+H/2];
end
function p=bufferPickPosition(cfg,slot,H)
p=bufferCenter(cfg,slot,H);
end
function a=makeAction(action,id,slot,fromPos,toPos,plan)
a=emptyAction(); a.action=action; a.boxID=id; a.sourceID=plan(id).sourceID;
a.bufferSlot=slot; a.fromPos=fromPos; a.toPos=toPos; a.targetYaw=plan(id).yaw;
end
function a=emptyAction()
a=struct('action','','boxID',0,'sourceID',0,'bufferSlot',0, ...
    'fromPos',[0 0 0],'toPos',[0 0 0],'targetYaw',0);
end
