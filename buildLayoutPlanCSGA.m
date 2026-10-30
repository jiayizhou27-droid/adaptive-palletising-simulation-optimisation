function [bestPlan,info] = buildLayoutPlanCSGA(boxData,cfg,opt)
%BUILDLAYOUTPLANCSGA Constructive-seeded GA using the restored decoder.
% Only the offline arrival/placement order is optimized.  Every chromosome
% is decoded by the proven 2026-07-19 physical layout rules, then checked by
% the restored buffer execution scheduler.  Robot trajectory generation is
% deliberately outside this function and remains unchanged.
if nargin<3, opt=struct(); end
if ~isfield(opt,'seed'), opt.seed=20260824; end
if ~isfield(opt,'populationSize'), opt.populationSize=12; end
if ~isfield(opt,'maxEvaluations'), opt.maxEvaluations=120; end
rng(opt.seed,'twister');
n=numel(boxData.length);

seedPlan=buildLayoutPlanBatch3(boxData,cfg.palletDim,cfg.palletLoc,n, ...
    cfg.palletMargin,cfg.boxGap);
assert(all([seedPlan.isPlaced]),'Constructive seed is incomplete.');
[~,rankIndex]=sort([seedPlan.placementRank]);
seedOrder=[seedPlan(rankIndex).ID];

population=repmat(seedOrder,opt.populationSize,1);
for i=2:opt.populationSize
    population(i,:)=mutateOrder(seedOrder,max(2,round(n/6)));
end
evaluations=0; bestScore=-inf; bestPlan=struct([]); bestOrder=[];
while evaluations<opt.maxEvaluations
    scores=-inf(opt.populationSize,1);
    plans=cell(opt.populationSize,1);
    for i=1:opt.populationSize
        if evaluations>=opt.maxEvaluations, break; end
        [scores(i),plans{i}]=evaluateOrder(population(i,:),boxData,cfg);
        evaluations=evaluations+1;
        if scores(i)>bestScore
            bestScore=scores(i); bestPlan=plans{i}; bestOrder=population(i,:);
        end
    end
    [~,ord]=sort(scores,'descend');
    elite=population(ord(1:min(2,opt.populationSize)),:);
    next=elite;
    while size(next,1)<opt.populationSize
        parent=population(ord(randi(max(1,ceil(opt.populationSize/2)))) ,:);
        next(end+1,:)=mutateOrder(parent,1+randi(3)); %#ok<AGROW>
    end
    population=next(1:opt.populationSize,:);
end
assert(~isempty(bestPlan),'CS-GA produced no complete 12-slot-buffer-feasible layout.');
info=struct('method',"CS-GA",'seed',opt.seed,'evaluations',evaluations, ...
    'score',bestScore,'order',bestOrder,'complete',all([bestPlan.isPlaced]));
end

function [score,plan]=evaluateOrder(order,b,cfg)
rb=reorderData(b,order);
try
    % A three-carton decoding window makes the chromosome order effective
    % while retaining the restored decoder's support and upright rules.
    raw=buildLayoutPlanBatch3(rb,cfg.palletDim,cfg.palletLoc,3, ...
        cfg.palletMargin,cfg.boxGap);
catch
    score=-inf; plan=struct([]); return
end
plan=remapPlan(raw,order,b);
if ~all([plan.isPlaced]), score=-inf; return; end
try
    executionPlan=buildBufferExecutionPlan(plan,cfg); %#ok<NASGU>
catch
    score=-inf; return
end
layers=max([plan.layer]);
top=max([plan.z]+[plan.H]/2);
base=cfg.palletLoc(3)+cfg.palletDim(3)/2;
height=max(top-base,eps);
volume=sum([plan.L].*[plan.W].*[plan.H]);
utilization=volume/(prod(cfg.palletDim(1:2))*height);
support=mean([plan.supportRatio]);
score=1e8*numel(plan)-1e6*layers+1e5*utilization+1e3*support;
end

function order=mutateOrder(order,count)
for k=1:count
    ids=sort(randperm(numel(order),2));
    if rand<0.5
        order(ids)=order(fliplr(ids));
    else
        order(ids(1):ids(2))=fliplr(order(ids(1):ids(2)));
    end
end
end

function d=reorderData(b,order)
d=struct(); fields=fieldnames(b);
for i=1:numel(fields)
    value=b.(fields{i});
    if numel(value)==numel(order), d.(fields{i})=value(order);
    else, d.(fields{i})=value; end
end
d.sourceID=order(:);
end

function plan=remapPlan(raw,order,b)
n=numel(order); plan=repmat(raw(1),n,1);
for j=1:n
    original=order(j); record=raw(j);
    record.ID=original; record.sourceID=original;
    if isfield(b,'type'), record.type=string(b.type(original)); end
    record.weight=double(b.weight(original));
    plan(original)=record;
end
end
