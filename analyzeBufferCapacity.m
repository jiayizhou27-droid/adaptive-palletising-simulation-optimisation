function report = analyzeBufferCapacity(boxCount, capacities)
%ANALYZEBUFFERCAPACITY Study execution feasibility without changing layout.
% The global layout and conveyor order are fixed once.  Only physical
% buffer capacity varies, so the experiment does not reward a different
% layout or an artificially reordered conveyor stream.
if nargin<1 || isempty(boxCount), boxCount=40; end
if nargin<2 || isempty(capacities), capacities=[0 2 4 6 8]; end

cfg=palletTaskConfig();
T=readtable('boxes.xlsx');
assert(boxCount<=height(T),'boxes.xlsx contains only %d cartons.',height(T));
d.length=T.L(1:boxCount); d.width=T.W(1:boxCount);
d.height=T.H(1:boxCount); d.weight=T.Mass(1:boxCount);
if ismember('Type',T.Properties.VariableNames), d.type=string(T.Type(1:boxCount)); end
layoutPlan=buildLayoutPlanBatch3(d,cfg.palletDim,cfg.palletLoc, ...
    cfg.batchSize,cfg.palletMargin,cfg.boxGap);

% A generous logical buffer reveals the minimum peak occupancy required by
% this fixed layout/order pair.  Slot coordinates are irrelevant here.
wide=cfg; wide.bufferCapacity=boxCount;
wide.bufferGrid=[ceil(boxCount/2) 2];
executionPlan=buildBufferExecutionPlan(layoutPlan,wide);
actions=string({executionPlan.action});
toBuffer=sum(actions=='conveyor_to_buffer');
fromBuffer=sum(actions=='buffer_to_pallet');
requiredPeak=max([executionPlan.bufferSlot]);

capacities=double(capacities(:));
feasible=capacities>=requiredPeak;
report=table(repmat(boxCount,numel(capacities),1),capacities,feasible, ...
    repmat(requiredPeak,numel(capacities),1),repmat(toBuffer,numel(capacities),1), ...
    repmat(fromBuffer,numel(capacities),1),repmat(max([layoutPlan.layer]),numel(capacities),1), ...
    'VariableNames',{'BoxCount','BufferCapacity','ExecutionFeasible', ...
    'RequiredPeakCapacity','ToBufferActions','FromBufferActions','LayoutLayers'});
disp(report);
writetable(report,sprintf('buffer_capacity_study_%dboxes.csv',boxCount));
end
