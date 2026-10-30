function M=computeExtendedOfflineMetrics(plan,exec,cfg,planningTime)
%COMPUTEEXTENDEDOFFLINEMETRICS Thesis-grade layout and execution metrics.
% Cycle time and full robot energy are intentionally not estimated here:
% those are measured later with the unchanged Simulink handling model.
base=computeLayoutComparisonMetrics("offline",plan,exec,cfg,planningTime,NaN);
p=plan([plan.isPlaced]);
palletArea=prod(cfg.palletDim(1:2));
envelopeVolume=palletArea*base.StackHeight_m;

M=base;
M.UnusedEnvelopeVolume_m3=envelopeVolume-base.BoxVolume_m3;
M.InternalVoidRatio=max(0,M.UnusedEnvelopeVolume_m3/envelopeVolume);
M.MaximumOverhang_m=maximumOverhang(p,cfg);
M.MeanSupportRatio=meanNonBaseSupport(p,cfg);
M.SurfaceFlatness_m=surfaceFlatness(p);
M.FootprintCompactness=footprintCompactness(p);
M.MixedTypeAdjacencyScore=mixedTypeScore(p,cfg);
M.LayerWeightMonotonicity=weightMonotonicity(p,cfg);
M.BufferRetrievals=M.FromBufferActions;
end

function value=maximumOverhang(p,cfg)
x1=cfg.palletLoc(1)-cfg.palletDim(1)/2+cfg.palletMargin;
x2=cfg.palletLoc(1)+cfg.palletDim(1)/2-cfg.palletMargin;
y1=cfg.palletLoc(2)-cfg.palletDim(2)/2+cfg.palletMargin;
y2=cfg.palletLoc(2)+cfg.palletDim(2)/2-cfg.palletMargin;
value=0;
for i=1:numel(p)
    value=max(value,max([x1-(p(i).x-p(i).L/2), ...
        (p(i).x+p(i).L/2)-x2,y1-(p(i).y-p(i).W/2), ...
        (p(i).y+p(i).W/2)-y2,0]));
end
end

function value=meanNonBaseSupport(p,cfg)
palletTop=cfg.palletLoc(3)+cfg.palletDim(3)/2;
isUpper=arrayfun(@(q)q.z-q.H/2>palletTop+1e-8,p);
if any(isUpper),value=mean([p(isUpper).supportRatio]);else,value=1;end
end

function value=surfaceFlatness(p)
% Area-weighted standard deviation of the exposed top surface height.
xs=unique([[p.x]-[p.L]/2,[p.x]+[p.L]/2]);
ys=unique([[p.y]-[p.W]/2,[p.y]+[p.W]/2]);
sumA=0;sumH=0;sumH2=0;
for ix=1:numel(xs)-1
for iy=1:numel(ys)-1
    cx=(xs(ix)+xs(ix+1))/2;cy=(ys(iy)+ys(iy+1))/2;
    cover=arrayfun(@(q)abs(cx-q.x)<q.L/2-1e-10 && ...
        abs(cy-q.y)<q.W/2-1e-10,p);
    if ~any(cover),continue;end
    h=max([p(cover).z]+[p(cover).H]/2);
    a=(xs(ix+1)-xs(ix))*(ys(iy+1)-ys(iy));
    sumA=sumA+a;sumH=sumH+a*h;sumH2=sumH2+a*h*h;
end
end
value=sqrt(max(0,sumH2/sumA-(sumH/sumA)^2));
end

function value=footprintCompactness(p)
layers=unique([p.layer]);scores=nan(size(layers));
for k=1:numel(layers)
    q=p([p.layer]==layers(k));shape=polyshape();
    for i=1:numel(q)
        x=q(i).x+q(i).L/2*[-1 1 1 -1];
        y=q(i).y+q(i).W/2*[-1 -1 1 1];
        shape=union(shape,polyshape(x,y));
    end
    [bx,by]=boundingbox(shape);
    scores(k)=area(shape)/prod([bx(2)-bx(1),by(2)-by(1)]);
end
value=mean(scores,'omitnan');
end

function value=mixedTypeScore(p,cfg)
different=0;total=0;
gapTolerance=cfg.boxGap+1e-8;
for i=1:numel(p)-1
for j=i+1:numel(p)
    if p(i).layer~=p(j).layer,continue;end
    xOverlap=min(p(i).x+p(i).L/2,p(j).x+p(j).L/2)- ...
        max(p(i).x-p(i).L/2,p(j).x-p(j).L/2);
    yOverlap=min(p(i).y+p(i).W/2,p(j).y+p(j).W/2)- ...
        max(p(i).y-p(i).W/2,p(j).y-p(j).W/2);
    xGap=max(0,-xOverlap);yGap=max(0,-yOverlap);shared=0;
    if xGap<=gapTolerance && yOverlap>1e-8,shared=yOverlap;end
    if yGap<=gapTolerance && xOverlap>1e-8,shared=max(shared,xOverlap);end
    if shared>0
        total=total+shared;
        if string(p(i).type)~=string(p(j).type),different=different+shared;end
    end
end
end
if total==0,value=NaN;else,value=different/total;end
end

function value=weightMonotonicity(p,cfg)
palletTop=cfg.palletLoc(3)+cfg.palletDim(3)/2;good=0;total=0;
for i=1:numel(p)
    bottom=p(i).z-p(i).H/2;
    if abs(bottom-palletTop)<1e-8,continue;end
    for j=1:numel(p)
        if abs((p(j).z+p(j).H/2)-bottom)>1e-8,continue;end
        ox=min(p(i).x+p(i).L/2,p(j).x+p(j).L/2)- ...
            max(p(i).x-p(i).L/2,p(j).x-p(j).L/2);
        oy=min(p(i).y+p(i).W/2,p(j).y+p(j).W/2)- ...
            max(p(i).y-p(i).W/2,p(j).y-p(j).W/2);
        if ox>1e-9&&oy>1e-9
            total=total+ox*oy;
            if p(j).weight>=p(i).weight-1e-9,good=good+ox*oy;end
        end
    end
end
if total==0,value=1;else,value=good/total;end
end
