function initializeSavedSimulinkValidationCase(planFile)
%INITIALIZESAVEDSIMULINKVALIDATIONCASE Load an exact saved 3-D layout.
% This function is used as a temporary model InitFcn by the validation
% runner.  It does not recompute or alter the offline target layout.

S=load(planFile,'plan');
assert(isfield(S,'plan') && ~isempty(S.plan),'Saved plan is missing: %s',planFile);
layoutPlan=S.plan; cfg=palletTaskConfig();
assert(all([layoutPlan.isPlaced]),'Saved layout contains unplaced cartons.');

boxData.length=reshape([layoutPlan.L],[],1);
boxData.width=reshape([layoutPlan.W],[],1);
boxData.height=reshape([layoutPlan.H],[],1);
boxData.weight=reshape([layoutPlan.weight],[],1);
boxData.type=strings(numel(layoutPlan),1);
for i=1:numel(layoutPlan)
    try, boxData.type(i)=string(layoutPlan(i).type); catch, boxData.type(i)="Unknown"; end
end
boxDataAll=boxData;
executionPlan=buildBufferExecutionPlan(layoutPlan,cfg);
layoutN=numel(layoutPlan);
layoutPos=reshape([layoutPlan.pos],3,[])';
layoutYaw=reshape([layoutPlan.yaw],[],1);
layoutH=reshape([layoutPlan.H],[],1);
boxDim=[boxData.length(1);boxData.width(1);boxData.height(1)];
boxWeight=boxData.weight(1);
boxPosition=cfg.sourceBoxPosition; pickPosition=cfg.pickPosition;
boxToEEOffset=[-0.2 0 boxDim(3)/2+0.11];
suctionCenterOffsetCheck=boxToEEOffset+[0.2 0 0];
maxNumOfBoxes=numel(executionPlan); executionActionLog=zeros(0,4);
executionActorRegistry=cell(1,layoutN); EnableGeospatial=false;
enableSimulinkRRT=false; rbt=exampleHelperSetUpCobot();

names={'cfg','boxData','boxDataAll','layoutPlan','executionPlan','layoutN', ...
    'layoutPos','layoutYaw','layoutH','boxDim','boxWeight','boxPosition', ...
    'pickPosition','boxToEEOffset','suctionCenterOffsetCheck','maxNumOfBoxes', ...
    'executionActionLog','executionActorRegistry','EnableGeospatial', ...
    'enableSimulinkRRT','rbt'};
for i=1:numel(names), assignin('base',names{i},eval(names{i})); end
fprintf('Validation case loaded: %s | %d cartons | %d actions\n', ...
    planFile,layoutN,maxNumOfBoxes);
end
