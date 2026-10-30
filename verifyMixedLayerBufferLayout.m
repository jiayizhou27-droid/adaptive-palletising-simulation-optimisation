function report = verifyMixedLayerBufferLayout(makePlot)
%VERIFYMIXEDLAYERBUFFERLAYOUT End-to-end constraint verification.
if nargin < 1, makePlot = true; end
cfg = palletTaskConfig();
T = readtable('boxes.xlsx');
allData.length=T.L; allData.width=T.W; allData.height=T.H; allData.weight=T.Mass;
if ismember("Type",string(T.Properties.VariableNames)), allData.type=string(T.Type); end
boxData=sliceBoxData(allData,cfg.firstExcelID);
layoutPlan=buildLayoutPlanBatch3(boxData,cfg.palletDim,cfg.palletLoc, ...
    cfg.batchSize,cfg.palletMargin,cfg.boxGap);
executionPlan=buildBufferExecutionPlan(layoutPlan,cfg);

dims=[[layoutPlan.L].' [layoutPlan.W].' [layoutPlan.H].'];
faceAreas=[dims(:,1).*dims(:,2),dims(:,1).*dims(:,3),dims(:,2).*dims(:,3)];
[~,order]=sort(faceAreas,2,'descend');
bottomRank=zeros(size(dims,1),1);
for i=1:size(dims,1), bottomRank(i)=find(order(i,:)==1,1); end
layers=[layoutPlan.layer].';
types=string({layoutPlan.type}).';
layerIDs=unique(layers).';
mixed=true; layerCounts=zeros(numel(layerIDs),1); layerArea=zeros(numel(layerIDs),1);
for k=1:numel(layerIDs)
    ids=layers==layerIDs(k); layerCounts(k)=sum(ids);
    layerArea(k)=sum(dims(ids,1).*dims(ids,2));
    mixed=mixed && numel(unique(types(ids)))>=2;
end
bufferDepth=0; maxBufferDepth=0;
for i=1:numel(executionPlan)
    if strcmp(executionPlan(i).action,'conveyor_to_buffer'), bufferDepth=bufferDepth+1; end
    if strcmp(executionPlan(i).action,'buffer_to_pallet'), bufferDepth=bufferDepth-1; end
    maxBufferDepth=max(maxBufferDepth,bufferDepth);
end

assert(numel(layoutPlan)==20,'Expected exactly 20 visible pallet boxes.');
assert(numel(layerIDs)==2 && all(layerCounts==10),'Expected two layers of ten boxes.');
assert(all(bottomRank<=2),'A box uses its smallest face as footprint.');
assert(all(abs(mod([layoutPlan.yaw],pi/2))<1e-10),'Non-planar rotation detected.');
assert(all([layoutPlan.supportRatio]>=0.80-1e-9),'Support ratio below 80%%.');
assert(mixed,'Every layer must contain mixed carton types.');
assert(maxBufferDepth<=cfg.bufferCapacity,'Buffer capacity exceeded.');

bufferBounds=bounds2D(cfg.bufferPlatformCenterXY,cfg.bufferPlatformDim(1:2));
conveyorBounds=bounds2D(cfg.conveyorCenterXY,cfg.conveyorDimXY);
baseBounds=bounds2D(cfg.robotBaseCenterXY,cfg.robotBaseDimXY);
palletBounds=bounds2D(cfg.palletLoc(1:2),cfg.palletDim(1:2));
assert(abs(bufferBounds(1)-conveyorBounds(2))<1e-9, ...
    'Buffer platform is not flush with the conveyor edge.');
assert(rectOverlapArea(bufferBounds,conveyorBounds)<1e-12, ...
    'Buffer platform overlaps the conveyor.');
assert(rectOverlapArea(bufferBounds,baseBounds)<1e-12, ...
    'Buffer platform overlaps the robot base.');
assert(rectOverlapArea(bufferBounds,palletBounds)<1e-12, ...
    'Buffer platform overlaps the pallet.');

report=table(layerIDs.',layerCounts,layerArea, ...
    layerArea/(cfg.palletDim(1)*cfg.palletDim(2)), ...
    'VariableNames',{'Layer','BoxCount','BoxArea','PalletAreaFraction'});
disp(report);
fprintf('Max buffer occupancy: %d/%d slots\n',maxBufferDepth,cfg.bufferCapacity);
fprintf('Minimum support ratio: %.1f%%%%\n',100*min([layoutPlan.supportRatio]));
if makePlot
    plotLayoutPlan3D(layoutPlan,cfg.palletDim,cfg.palletLoc);
    exportgraphics(gcf,'mixed_layer_buffer_layout.png','Resolution',180);
    writetable(struct2table(executionPlan),'buffer_execution_plan.csv');
end
assignin('base','layoutPlan',layoutPlan);
assignin('base','executionPlan',executionPlan);
end

function b=bounds2D(center,dims)
b=[center(1)-dims(1)/2,center(1)+dims(1)/2, ...
   center(2)-dims(2)/2,center(2)+dims(2)/2];
end
function a=rectOverlapArea(aBounds,bBounds)
dx=max(0,min(aBounds(2),bBounds(2))-max(aBounds(1),bBounds(1)));
dy=max(0,min(aBounds(4),bBounds(4))-max(aBounds(3),bBounds(3)));
a=dx*dy;
end
