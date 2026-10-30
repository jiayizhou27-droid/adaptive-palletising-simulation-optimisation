function [bestPlan,info]=optimizeConstructiveSeededPhysical(method,boxData,cfg,opt)
%OPTIMIZECONSTRUCTIVESEEDEDPHYSICAL CS-GA/CS-SA chromosome search.
% Chromosome = box order + fixed upright orientation + layer preference.
% All candidates use the shared hard-physics decoder.
if ~isfield(opt,'seed'),opt.seed=1;end
if ~isfield(opt,'maxEvaluations'),opt.maxEvaluations=50;end
if ~isfield(opt,'populationSize'),opt.populationSize=8;end
if ~isfield(opt,'initialPlan')||isempty(opt.initialPlan)
    opt.initialPlan=buildLayoutPlanBatch3(boxData,cfg.palletDim,cfg.palletLoc, ...
        numel(boxData.length),cfg.palletMargin,cfg.boxGap,false);
end
method=upper(string(method));n=numel(boxData.length);rng(opt.seed,'twister');
[~,ix]=sort([opt.initialPlan.placementRank]);
seedOrder=[opt.initialPlan(ix).sourceID];
seedOrientation=zeros(1,n);seedLayer=zeros(1,n);
maxLayer=max([opt.initialPlan.layer]);
for i=1:n
    seedOrientation(i)=abs(opt.initialPlan(i).yaw-pi/2)<1e-8;
    seedLayer(i)=0.70*(opt.initialPlan(i).layer-1)/max(1,maxLayer-1);
end
cache=containers.Map('KeyType','char','ValueType','any');evaluations=0;

    function R=evaluate(C)
        key=sprintf('%d,',C.order)+"|"+sprintf('%d,',C.orientation)+"|"+ ...
            sprintf('%.2f,',C.layerPreference);
        key=char(key);
        if isKey(cache,key),R=cache(key);return;end
        rb=reorderData(boxData,C.order);
        hints.orientation=C.orientation(C.order);
        hints.layerPreference=C.layerPreference(C.order);
        raw=buildLayoutPlanBatch3(rb,cfg.palletDim,cfg.palletLoc,1, ...
            cfg.palletMargin,cfg.boxGap,false,true,hints);
        plan=remap(raw,C.order,boxData);
        placed=[plan.isPlaced];placedN=sum(placed);
        if placedN
            p=plan(placed);layers=max([p.layer]);
            height=max([p.z]+[p.H]/2)-(cfg.palletLoc(3)+cfg.palletDim(3)/2);
            vol=sum([p.L].*[p.W].*[p.H]);
            util=vol/(prod(cfg.palletDim(1:2))*height);
            meanSupport=mean([p.supportRatio]);
        else
            layers=inf;height=inf;util=0;meanSupport=0;
        end
        score=1e8*placedN-1e6*layers+1e5*util+1e3*meanSupport;
        R=struct('chromosome',C,'plan',plan,'placedN',placedN, ...
            'layers',layers,'height',height,'utilization',util,'score',score);
        cache(key)=R;evaluations=evaluations+1;
    end

seed=struct('order',seedOrder,'orientation',seedOrientation, ...
    'layerPreference',seedLayer);
if method=="CS-GA"
    pop=repmat(struct('chromosome',seed,'result',[]),opt.populationSize,1);
    pop(1).chromosome=seed;
    for i=2:opt.populationSize,pop(i).chromosome=randomized(seed);end
    for i=1:opt.populationSize,pop(i).result=evaluate(pop(i).chromosome);end
    while evaluations<opt.maxEvaluations
        [~,ord]=sort(arrayfun(@(x)x.result.score,pop),'descend');pop=pop(ord);
        next=pop(1:min(2,numel(pop)));
        while numel(next)<opt.populationSize&&evaluations<opt.maxEvaluations
            a=tournament(pop).chromosome;b=tournament(pop).chromosome;
            child=crossover(a,b);child=mutate(child);
            next(end+1)=struct('chromosome',child,'result',evaluate(child)); %#ok<AGROW>
        end
        while numel(next)<opt.populationSize,next(end+1)=pop(numel(next)+1);end %#ok<AGROW>
        pop=next;
    end
    [~,j]=max(arrayfun(@(x)x.result.score,pop));best=pop(j).result;
elseif method=="CS-SA"
    current=evaluate(seed);best=current;T0=2e5;
    while evaluations<opt.maxEvaluations
        candidate=evaluate(mutate(current.chromosome));
        temp=max(1,T0*(1-evaluations/opt.maxEvaluations)^2);
        delta=candidate.score-current.score;
        if delta>=0||rand<exp(delta/temp),current=candidate;end
        if candidate.score>best.score,best=candidate;end
    end
else
    error('Method must be CS-GA or CS-SA.');
end
bestPlan=best.plan;
info=rmfield(best,'plan');info.method=method;info.evaluations=evaluations;
info.complete=best.placedN==n;
if ~info.complete,bestPlan=struct([]);end
end

function C=randomized(seed)
C=seed;n=numel(C.order);
for k=1:max(2,round(n/5)),C=mutate(C);end
end
function p=tournament(pop)
ids=randi(numel(pop),1,3);[~,j]=max(arrayfun(@(i)pop(i).result.score,ids));p=pop(ids(j));
end
function c=crossover(a,b)
n=numel(a.order);cuts=sort(randperm(n,2));c=a;
c.order(:)=0;c.order(cuts(1):cuts(2))=a.order(cuts(1):cuts(2));
rem=b.order(~ismember(b.order,c.order));c.order(c.order==0)=rem;
mask=rand(1,n)<0.5;c.orientation(mask)=b.orientation(mask);
c.layerPreference(mask)=b.layerPreference(mask);
end
function C=mutate(C)
n=numel(C.order);choice=randi(3);
if choice==1
    ids=randperm(n,2);C.order(ids)=C.order(fliplr(ids));
elseif choice==2
    id=randi(n);C.orientation(id)=1-C.orientation(id);
else
    id=randi(n);C.layerPreference(id)=min(1,max(0,C.layerPreference(id)+0.35*randn));
end
end
function d=reorderData(b,order)
fields=fieldnames(b);d=struct();
for i=1:numel(fields)
    v=b.(fields{i});
    if numel(v)==numel(order),d.(fields{i})=v(order);else,d.(fields{i})=v;end
end
d.sourceID=order(:);
end
function plan=remap(raw,order,b)
n=numel(order);plan=repmat(emptyRecord(),n,1);
for j=1:n
    original=order(j);r=raw(j);r.ID=original;r.sourceID=original;
    r.type=string(b.type(original));r.weight=double(b.weight(original));plan(original)=r;
end
end
function r=emptyRecord()
r=struct('ID',0,'x',0,'y',0,'z',0,'yaw',0,'L',0,'W',0,'H',0, ...
 'weight',0,'isPlaced',false,'pos',[0 0 0],'layer',0,'sourceID',0, ...
 'type',"",'placementRank',0,'supportRatio',0,'wasBuffered',false,'bufferSlot',0);
end
