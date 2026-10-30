function row=computeLayoutComparisonMetrics(method,plan,exec,cfg,planningTime,planningStd)
%COMPUTELAYOUTCOMPARISONMETRICS Common offline and execution-plan metrics.
placed=[plan.isPlaced]; p=plan(placed); n=numel(p);
if nargin<6, planningStd=NaN; end
layers=max([p.layer]);
palletArea=prod(cfg.palletDim(1:2));
usableArea=prod(cfg.palletDim(1:2)-2*cfg.palletMargin);
areaByLayer=zeros(1,layers); countByLayer=zeros(1,layers);
for k=1:n
    areaByLayer(p(k).layer)=areaByLayer(p(k).layer)+p(k).L*p(k).W;
    countByLayer(p(k).layer)=countByLayer(p(k).layer)+1;
end
boxVolume=sum([p.L].*[p.W].*[p.H]);
stackHeight=max([p.z]+[p.H]/2)-(cfg.palletLoc(3)+cfg.palletDim(3)/2);
actions=string({exec.action}); realActions=actions~="hidden_warmup";
dist=0; payloadWork=0;
for k=find(realActions)
    d=exec(k).toPos-exec(k).fromPos; dist=dist+norm(d);
    id=exec(k).boxID; payloadWork=payloadWork+plan(id).weight*9.81*max(d(3),0);
end
peak=peakBufferOccupancy(exec);
mass=sum([p.weight]);
com=[sum([p.weight].*[p.x])/mass,sum([p.weight].*[p.y])/mass,sum([p.weight].*[p.z])/mass];
comOffset=hypot(com(1)-cfg.palletLoc(1),com(2)-cfg.palletLoc(2));
[minMargin,unstable]=supportCOMMargins(p,cfg);
halfL=cfg.palletDim(1)/2-cfg.palletMargin; halfW=cfg.palletDim(2)/2-cfg.palletMargin;
edgeDistances=[halfL-abs(com(1)-cfg.palletLoc(1)),halfW-abs(com(2)-cfg.palletLoc(2))];
comHeight=com(3)-(cfg.palletLoc(3)+cfg.palletDim(3)/2);
tipAngle=atan2d(min(edgeDistances),comHeight);
row=table(string(method),n,planningTime,planningStd,layers,stackHeight,boxVolume, ...
    boxVolume/(palletArea*stackHeight),boxVolume/(usableArea*stackHeight), ...
    mean(areaByLayer/palletArea),areaByLayer(1)/palletArea, ...
    areaByLayer(end)/palletArea,min(areaByLayer/palletArea), ...
    std(areaByLayer/palletArea),min([p.supportRatio]),minMargin,unstable, ...
    comOffset,comOffset/min(halfL,halfW),comHeight,tipAngle, ...
    sum(actions=="conveyor_to_buffer"),sum(actions=="buffer_to_pallet"),peak, ...
    sum(realActions),dist,payloadWork,string(mat2str(countByLayer)), ...
 'VariableNames',{'Method','PlacedBoxes','MeanPlanningTime_s','PlanningTimeStd_s','Layers','StackHeight_m', ...
 'BoxVolume_m3','PalletVolumeUtilization','UsableEnvelopeVolumeUtilization', ...
 'MeanLayerAreaUtilization','FirstLayerAreaUtilization','TopLayerAreaUtilization', ...
 'MinimumLayerAreaUtilization','LayerUtilizationStd', ...
 'MinSupportRatio','MinCOMSupportMargin_m','UnstableBoxCount','COMOffset_m', ...
 'NormalizedCOMOffset','COMHeight_m','MinStaticTippingAngle_deg', ...
 'ToBufferActions','FromBufferActions','PeakBufferOccupancy', ...
 'RobotMoves','TransferDistance_m','PayloadLiftEnergyProxy_J','BoxesPerLayer'});
end

function [minMargin,unstable]=supportCOMMargins(p,cfg)
% Signed distance from each carton COM projection to its actual support.
palletTop=cfg.palletLoc(3)+cfg.palletDim(3)/2; margins=inf(1,numel(p));
for i=1:numel(p)
    bottom=p(i).z-p(i).H/2;
    if abs(bottom-palletTop)<1e-8
        x1=cfg.palletLoc(1)-cfg.palletDim(1)/2+cfg.palletMargin;
        x2=cfg.palletLoc(1)+cfg.palletDim(1)/2-cfg.palletMargin;
        y1=cfg.palletLoc(2)-cfg.palletDim(2)/2+cfg.palletMargin;
        y2=cfg.palletLoc(2)+cfg.palletDim(2)/2-cfg.palletMargin;
        margins(i)=min([p(i).x-x1,x2-p(i).x,p(i).y-y1,y2-p(i).y]); continue
    end
    supports=[];
    for j=1:numel(p)
        if abs((p(j).z+p(j).H/2)-bottom)>1e-8, continue; end
        overlapX=min(p(i).x+p(i).L/2,p(j).x+p(j).L/2)-max(p(i).x-p(i).L/2,p(j).x-p(j).L/2);
        overlapY=min(p(i).y+p(i).W/2,p(j).y+p(j).W/2)-max(p(i).y-p(i).W/2,p(j).y-p(j).W/2);
        if overlapX>1e-9 && overlapY>1e-9
            supports(end+1)=j; %#ok<AGROW>
        end
    end
    if isempty(supports), margins(i)=-inf; continue; end
    supportShape=polyshape();
    bx1=p(i).x-p(i).L/2; bx2=p(i).x+p(i).L/2;
    by1=p(i).y-p(i).W/2; by2=p(i).y+p(i).W/2;
    for k=1:numel(supports)
        s=p(supports(k));
        x1=max(bx1,s.x-s.L/2); x2=min(bx2,s.x+s.L/2);
        y1=max(by1,s.y-s.W/2); y2=min(by2,s.y+s.W/2);
        supportShape=union(supportShape,polyshape([x1 x2 x2 x1],[y1 y1 y2 y2]));
    end
    [vx,vy]=boundary(supportShape);
    distance=pointBoundaryDistance(p(i).x,p(i).y,vx,vy);
    if isinterior(supportShape,p(i).x,p(i).y)
        margins(i)=distance;
    else
        margins(i)=-distance;
    end
end
minMargin=min(margins); unstable=sum(margins<0.010-1e-9);
end

function d=pointBoundaryDistance(px,py,x,y)
d=inf;
breaks=[0;find(isnan(x));numel(x)+1];
for b=1:numel(breaks)-1
    ids=(breaks(b)+1):(breaks(b+1)-1);
    if numel(ids)<2, continue; end
    xx=x(ids); yy=y(ids);
    if xx(1)~=xx(end) || yy(1)~=yy(end), xx(end+1)=xx(1); yy(end+1)=yy(1); end %#ok<AGROW>
    for k=1:numel(xx)-1
        ax=xx(k); ay=yy(k); dx=xx(k+1)-ax; dy=yy(k+1)-ay;
        t=max(0,min(1,((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)));
        d=min(d,hypot(px-(ax+t*dx),py-(ay+t*dy)));
    end
end
end
function peak=peakBufferOccupancy(exec)
occ=0; peak=0;
for k=1:numel(exec)
    a=string(exec(k).action);
    if a=="conveyor_to_buffer", occ=occ+1; end
    if a=="buffer_to_pallet", occ=occ-1; end
    peak=max(peak,occ);
end
end
