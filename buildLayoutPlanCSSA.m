function [bestPlan,info] = buildLayoutPlanCSSA(boxData,cfg,opt)
%BUILDLAYOUTPLANCSSA Constructive-seeded simulated annealing.
% Optimises only the offline carton order.  The restored physical decoder,
% buffer scheduler, robot trajectory and Sim3D execution rules are reused.
if nargin<3, opt=struct(); end
if ~isfield(opt,'seed'),opt.seed=20261824;end
if ~isfield(opt,'maxEvaluations'),opt.maxEvaluations=120;end
if ~isfield(opt,'initialTemperature'),opt.initialTemperature=2e5;end
rng(opt.seed,'twister');
n=numel(boxData.length);
seedPlan=buildLayoutPlanBatch3(boxData,cfg.palletDim,cfg.palletLoc,n, ...
    cfg.palletMargin,cfg.boxGap);
[~,rankIndex]=sort([seedPlan.placementRank]);
seedOrder=[seedPlan(rankIndex).ID];

evaluations=0;
[currentScore,currentPlan]=evaluateOrder(seedOrder,boxData,cfg);
evaluations=evaluations+1;
currentOrder=seedOrder;
% Locate a feasible neighbour if the three-carton decoding window cannot
% reproduce the full-window seed exactly.
while ~isfinite(currentScore) && evaluations<min(30,opt.maxEvaluations)
    currentOrder=mutateOrder(seedOrder,1+randi(3));
    [currentScore,currentPlan]=evaluateOrder(currentOrder,boxData,cfg);
    evaluations=evaluations+1;
end
assert(isfinite(currentScore),'CS-SA could not construct a feasible initial state.');
bestScore=currentScore;bestPlan=currentPlan;bestOrder=currentOrder;

while evaluations<opt.maxEvaluations
    candidateOrder=mutateOrder(currentOrder,1+randi(3));
    [candidateScore,candidatePlan]=evaluateOrder(candidateOrder,boxData,cfg);
    evaluations=evaluations+1;
    progress=evaluations/opt.maxEvaluations;
    temperature=max(1,opt.initialTemperature*(1-progress)^2);
    if isfinite(candidateScore)
        delta=candidateScore-currentScore;
        if delta>=0 || rand<exp(delta/temperature)
            currentOrder=candidateOrder;
            currentScore=candidateScore;
            currentPlan=candidatePlan;
        end
        if candidateScore>bestScore
            bestScore=candidateScore;bestPlan=candidatePlan;
            bestOrder=candidateOrder;
        end
    end
end
info=struct('method',"CS-SA",'seed',opt.seed,'evaluations',evaluations, ...
    'score',bestScore,'order',bestOrder,'complete',all([bestPlan.isPlaced]));
end

function [score,plan]=evaluateOrder(order,b,cfg)
rb=reorderData(b,order);
try
    raw=buildLayoutPlanBatch3(rb,cfg.palletDim,cfg.palletLoc,3, ...
        cfg.palletMargin,cfg.boxGap);
catch
    score=-inf;plan=struct([]);return
end
plan=remapPlan(raw,order,b);
if ~all([plan.isPlaced]),score=-inf;return;end
try
    buildBufferExecutionPlan(plan,cfg);
catch
    score=-inf;return
end
layers=max([plan.layer]);
height=max([plan.z]+[plan.H]/2)- ...
    (cfg.palletLoc(3)+cfg.palletDim(3)/2);
volume=sum([plan.L].*[plan.W].*[plan.H]);
utilization=volume/(prod(cfg.palletDim(1:2))*max(height,eps));
support=mean([plan.supportRatio]);
score=1e8*numel(plan)-1e6*layers+1e5*utilization+1e3*support;
end

function order=mutateOrder(order,count)
for k=1:count
    ids=sort(randperm(numel(order),2));
    if rand<0.5,order(ids)=order(fliplr(ids));
    else,order(ids(1):ids(2))=fliplr(order(ids(1):ids(2)));end
end
end

function d=reorderData(b,order)
d=struct();fields=fieldnames(b);
for i=1:numel(fields)
    value=b.(fields{i});
    if numel(value)==numel(order),d.(fields{i})=value(order);
    else,d.(fields{i})=value;end
end
d.sourceID=order(:);
end

function plan=remapPlan(raw,order,b)
n=numel(order);plan=repmat(raw(1),n,1);
for j=1:n
    original=order(j);record=raw(j);
    record.ID=original;record.sourceID=original;
    if isfield(b,'type'),record.type=string(b.type(original));end
    record.weight=double(b.weight(original));plan(original)=record;
end
end
