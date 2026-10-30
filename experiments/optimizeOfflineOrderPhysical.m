function [bestPlan,info]=optimizeOfflineOrderPhysical(method,boxData,cfg,opt)
%OPTIMIZEOFFLINEORDERPHYSICAL GA/SA permutation search with hard physics.
% Every candidate is decoded by buildLayoutPlanBatch3 with a one-box
% decision window. The shared decoder enforces pallet bounds, no overlap,
% upright-only yaw rotation, >=95% support and >=10 mm COM support margin.
% Partial candidates may be scored during search but can never be returned
% as a feasible final layout.
if nargin<4 || isempty(opt),opt=struct();end
if ~isfield(opt,'seed'),opt.seed=1;end
if ~isfield(opt,'maxEvaluations'),opt.maxEvaluations=80;end
if ~isfield(opt,'populationSize'),opt.populationSize=10;end
if ~isfield(opt,'initialOrder'),opt.initialOrder=1:numel(boxData.length);end
method=string(method);
method=upper(method);
n=numel(boxData.length);
rng(opt.seed,'twister');
cache=containers.Map('KeyType','char','ValueType','any');
evaluations=0;

    function result=evaluate(order)
        key=sprintf('%d,',order);
        if isKey(cache,key), result=cache(key); return; end
        reordered=reorderData(boxData,order);
        raw=buildLayoutPlanBatch3(reordered,cfg.palletDim,cfg.palletLoc,1, ...
            cfg.palletMargin,cfg.boxGap,false,true);
        plan=remapPlan(raw,order,boxData);
        placed=[plan.isPlaced]; placedN=sum(placed);
        if placedN==0
            layers=inf; areaScore=0;
        else
            p=plan(placed); layers=max([p.layer]);
            area=zeros(1,layers);
            for z=1:numel(p),area(p(z).layer)=area(p(z).layer)+p(z).L*p(z).W;end
            areaScore=mean(area/prod(cfg.palletDim(1:2)));
        end
        % Lexicographic intent: complete placement dominates all quality.
        score=1e6*placedN-1e4*layers+1e3*areaScore;
        result=struct('order',order,'plan',plan,'placedN',placedN, ...
            'layers',layers,'areaScore',areaScore,'score',score);
        cache(key)=result; evaluations=evaluations+1;
    end

if method=="GA"
    pop=repmat(struct('order',[],'result',[]),opt.populationSize,1);
    pop(1).order=opt.initialOrder;
    for i=2:opt.populationSize,pop(i).order=randperm(n);end
    for i=1:opt.populationSize,pop(i).result=evaluate(pop(i).order);end
    while evaluations<opt.maxEvaluations
        scores=arrayfun(@(x)x.result.score,pop);
        [~,idx]=sort(scores,'descend'); pop=pop(idx);
        next=pop(1:min(2,numel(pop)));
        while numel(next)<opt.populationSize && evaluations<opt.maxEvaluations
            p1=tournament(pop); p2=tournament(pop);
            child=orderCrossover(p1.order,p2.order);
            child=mutateOrder(child);
            c=struct('order',child,'result',evaluate(child));
            next(end+1)=c; %#ok<AGROW>
        end
        while numel(next)<opt.populationSize,next(end+1)=pop(numel(next)+1);end %#ok<AGROW>
        pop=next;
    end
    scores=arrayfun(@(x)x.result.score,pop); [~,i]=max(scores); best=pop(i).result;
elseif method=="SA"
    current=evaluate(opt.initialOrder); best=current;
    T0=max(1,0.02*abs(current.score));
    while evaluations<opt.maxEvaluations
        order=current.order;
        if rand<0.5
            ids=randperm(n,2); order(ids)=order(fliplr(ids));
        else
            ids=sort(randperm(n,2)); order(ids(1):ids(2))=order(ids(2):-1:ids(1));
        end
        candidate=evaluate(order);
        temperature=max(1e-6,T0*(1-evaluations/opt.maxEvaluations)^2);
        delta=candidate.score-current.score;
        if delta>=0 || rand<exp(delta/temperature),current=candidate;end
        if candidate.score>best.score,best=candidate;end
    end
else
    error('Method must be "GA" or "SA".');
end

bestPlan=best.plan;
info=rmfield(best,'plan');
info.method=method; info.evaluations=evaluations;
info.complete=best.placedN==n;
if ~info.complete
    bestPlan=struct([]);
end
end

function d=reorderData(b,order)
fields=fieldnames(b); d=struct();
for i=1:numel(fields)
    v=b.(fields{i});
    if numel(v)==numel(order),d.(fields{i})=v(order);else,d.(fields{i})=v;end
end
d.sourceID=order(:);
end

function plan=remapPlan(raw,order,b)
n=numel(order); plan=repmat(emptyRecord(),n,1);
for j=1:n
    original=order(j); r=raw(j); r.ID=original; r.sourceID=original;
    if isfield(b,'type'),r.type=string(b.type(original));end
    r.weight=double(b.weight(original)); plan(original)=r;
end
end

function p=tournament(pop)
ids=randi(numel(pop),1,3);
[~,j]=max(arrayfun(@(i)pop(i).result.score,ids)); p=pop(ids(j));
end

function child=orderCrossover(a,b)
n=numel(a); cuts=sort(randperm(n,2)); child=zeros(1,n);
child(cuts(1):cuts(2))=a(cuts(1):cuts(2));
remaining=b(~ismember(b,child));
child(child==0)=remaining;
end

function out=mutateOrder(in)
out=in; n=numel(in);
if rand<0.7
    ids=randperm(n,2);out(ids)=out(fliplr(ids));
else
    ids=sort(randperm(n,2));out(ids(1):ids(2))=out(ids(2):-1:ids(1));
end
end

function r=emptyRecord()
r=struct('ID',0,'x',0,'y',0,'z',0,'yaw',0,'L',0,'W',0,'H',0, ...
 'weight',0,'isPlaced',false,'pos',[0 0 0],'layer',0,'sourceID',0, ...
 'type',"",'placementRank',0,'supportRatio',0,'wasBuffered',false,'bufferSlot',0);
end
