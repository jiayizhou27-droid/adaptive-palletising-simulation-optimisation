function layoutPlan = buildLayoutPlanShelfBaseline(boxData,palletDim,palletLoc,margin,boxGap)
%BUILDLAYOUTPLANSHELFBASELINE Arrival-order row/shelf baseline.
% Only yaw rotations are allowed. Rows are filled along pallet X and a new
% shelf is opened along Y. Every upper layer repeats the same footprint so
% that the comparison remains physically supportable.
if nargin<4 || isempty(margin), margin=0; end
if nargin<5 || isempty(boxGap), boxGap=0; end
n=numel(boxData.length); assert(n>0,'No cartons supplied.');
order=interleavedTypeOrder(boxData,n);
x0=palletLoc(1)-palletDim(1)/2+margin;
y0=palletLoc(2)-palletDim(2)/2+margin;
usableL=palletDim(1)-2*margin; usableW=palletDim(2)-2*margin;
zTop=palletLoc(3)+palletDim(3)/2;

% Build one deterministic shelf template. Subsequent layers repeat it;
% this gives every upper carton full support and isolates the packing rule.
template=repmat(struct('L',0,'W',0,'yaw',0,'x',0,'y',0),0,1);
x=x0; y=y0; rowDepth=0;
for sequenceIndex=1:n
    id=order(sequenceIndex);
    [L,W,yaw]=orientForShelf(boxData.length(id),boxData.width(id),usableL);
    if x>x0 && x+L>x0+usableL+1e-9
        x=x0; y=y+rowDepth+boxGap; rowDepth=0;
    end
    if y+W>y0+usableW+1e-9
        break
    end
    t=struct('L',L,'W',W,'yaw',yaw,'x',x,'y',y);
    template(end+1)=t; %#ok<AGROW>
    x=x+L+boxGap; rowDepth=max(rowDepth,W);
end
assert(~isempty(template),'No carton fits the usable pallet area.');
% The fixed experiment workbook repeats four carton types. End a shelf
% layer on a complete type cycle so the next layer can reuse its footprint.
cycleLength=4;
completeSlots=floor(numel(template)/cycleLength)*cycleLength;
assert(completeSlots>=cycleLength,'One complete four-type cycle must fit.');
template=template(1:completeSlots);
perLayer=numel(template);

layoutPlan=repmat(emptyRecord(),n,1);
for rank=1:n
    id=order(rank);
    slot=mod(rank-1,perLayer)+1; layer=floor((rank-1)/perLayer)+1;
    t=template(slot);
    % Template dimensions repeat because the test workbook repeats four
    % carton types. Refuse a misleading baseline if that assumption fails.
    [L,W,yaw]=orientForShelf(boxData.length(id),boxData.width(id),usableL);
    assert(abs(L-t.L)<1e-9 && abs(W-t.W)<1e-9, ...
        'Shelf template does not repeat at box %d; physical support is not guaranteed.',id);
    H=double(boxData.height(id)); bottom=zTop+(layer-1)*H;
    r=emptyRecord(); r.ID=id; r.sourceID=getSourceID(boxData,id);
    r.L=L; r.W=W; r.H=H; r.weight=double(boxData.weight(id));
    r.type=getType(boxData,id); r.yaw=yaw; r.layer=layer;
    r.x=t.x+L/2; r.y=t.y+W/2; r.z=bottom+H/2;
    r.pos=[r.x r.y r.z]; r.isPlaced=true; r.placementRank=rank;
    r.supportRatio=1; layoutPlan(id)=r;
end
end

function order=interleavedTypeOrder(b,n)
% Make repeated mixed-type cycles independently of workbook row order.
if ~isfield(b,'type')
    order=1:n;
    return
end
types=string(b.type(:));
names=unique(types,'stable');
groups=cell(numel(names),1);
for k=1:numel(names)
    groups{k}=find(types==names(k)).';
end
order=[];
while numel(order)<n
    for k=1:numel(groups)
        if ~isempty(groups{k})
            order(end+1)=groups{k}(1); %#ok<AGROW>
            groups{k}(1)=[];
        end
    end
end
end

function [L,W,yaw]=orientForShelf(L0,W0,usableL)
L=double(L0); W=double(W0); yaw=0;
% Put the shorter side along the shelf direction when both fit.
if W<=usableL && (W<L || L>usableL), L=double(W0); W=double(L0); yaw=pi/2; end
assert(L<=usableL+1e-9,'Carton is wider than the shelf.');
end
function id=getSourceID(b,i)
if isfield(b,'sourceID'), id=double(b.sourceID(i)); else, id=i; end
end
function t=getType(b,i)
if isfield(b,'type'), t=string(b.type(i)); else, t=""; end
end
function r=emptyRecord()
r=struct('ID',0,'x',0,'y',0,'z',0,'yaw',0,'L',0,'W',0,'H',0, ...
 'weight',0,'isPlaced',false,'pos',[0 0 0],'layer',0,'sourceID',0, ...
 'type',"",'placementRank',0,'supportRatio',0,'wasBuffered',false,'bufferSlot',0);
end
