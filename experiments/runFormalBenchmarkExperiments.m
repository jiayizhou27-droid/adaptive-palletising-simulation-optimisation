function results = runFormalBenchmarkExperiments(mode)
%RUNFORMALBENCHMARKEXPERIMENTS Formal Superbox benchmark with checkpoints.
% mode="smoke": six validation cases. mode="full": 190 instances.
if nargin<1, mode="full"; end
mode=string(mode);
projectDir=fileparts(mfilename('fullpath'));
cfg=palletTaskConfig();
outDir=fullfile(projectDir,'formal_benchmark_experiments');
if ~isfolder(outDir), mkdir(outDir); end
instanceDir=fullfile(outDir,'instances');
if ~isfolder(instanceDir), mkdir(instanceDir); end
checkpointFile=fullfile(outDir,'formal_benchmark_checkpoint.csv');

design=buildDesign(mode);
if isfile(checkpointFile)
    existing=readtable(checkpointFile,'TextType','string');
else
    existing=table();
end
rows=cell(height(design),1);

for q=1:height(design)
    id=design.CaseID(q);
    if ~isempty(existing) && any(existing.CaseID==id)
        fprintf('SKIP completed %s\n',id); continue
    end
    fprintf('START %d/%d %s\n',q,height(design),id); drawnow;
    [b,meta,typeTable]=generatePalletBenchmarkInstance( ...
        design.N(q),design.K(q),design.InstanceClass(q),design.Seed(q));
    writetable(typeTable,fullfile(instanceDir,id+"_types.csv"));
    inputTable=table((1:meta.N)',b.length,b.width,b.height,b.weight,b.type, ...
        'VariableNames',{'ArrivalIndex','L','W','H','Mass','Type'});
    writetable(inputTable,fullfile(instanceDir,id+"_boxes.csv"));

    success=false; failureReason=""; plan=struct([]); execution=struct([]);
    requiredBuffer=NaN; planningTime=NaN;
    tic;
    try
        plan=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,meta.N, ...
            cfg.palletMargin,cfg.boxGap,false);
        planningTime=toc;
        cfgWide=cfg; cfgWide.bufferCapacity=meta.N;
        cfgWide.bufferGrid=[ceil(meta.N/2) 2];
        execution=buildBufferExecutionPlan(plan,cfgWide);
        requiredBuffer=peakBuffer(execution);
        metric=computeLayoutComparisonMetrics("Superbox",plan,execution,cfg, ...
            planningTime,NaN);
        success=true;
    catch ME
        planningTime=toc;
        failureReason=string(ME.message);
        metric=emptyMetric(planningTime);
    end
    timedOut=planningTime>design.TimeLimit_s(q);
    row=[design(q,:) table(meta.VolumeCV,meta.FootprintCV,meta.LengthCV, ...
        meta.WidthCV,meta.TypeEntropy,meta.MeanVolume_m3, ...
        meta.MeanFootprint_m2,meta.TotalMass_kg,success,timedOut, ...
        failureReason,requiredBuffer,requiredBuffer<=cfg.bufferCapacity, ...
        'VariableNames',{'VolumeCV','FootprintCV','LengthCV','WidthCV', ...
        'TypeEntropy','MeanVolume_m3','MeanFootprint_m2','TotalMass_kg', ...
        'PlanningSucceeded','ExceededTimeLimit','FailureReason', ...
        'RequiredBufferCapacity','FeasibleWith8SlotBuffer'}) metric];
    rows{q}=row;
    if isempty(existing), existing=row; else, existing=[existing;row]; end %#ok<AGROW>
    writetable(existing,checkpointFile);
    save(fullfile(outDir,'latest_case.mat'),'plan','execution','b','meta','row');
    fprintf('DONE %s success=%d time=%.3f s\n',id,success,planningTime); drawnow;
end

summary=existing;
if mode=="full"
    writetable(summary,fullfile(outDir,'formal_benchmark_raw_results.csv'));
    aggregate=aggregateResults(summary);
    writetable(aggregate,fullfile(outDir,'formal_benchmark_aggregate.csv'));
    save(fullfile(outDir,'formal_benchmark_results.mat'), ...
        'summary','aggregate','design','cfg');
else
    aggregate=table();
end
results=struct('raw',summary,'aggregate',aggregate,'design',design, ...
    'outputDirectory',outDir);
assignin('base','formalBenchmarkResults',results);
disp(summary(:,{'Study','N','K','InstanceClass','Seed', ...
    'PlanningSucceeded','MeanPlanningTime_s','Layers', ...
    'PalletVolumeUtilization','RequiredBufferCapacity'}));
end

function D=buildDesign(mode)
baseSeeds=(20260801:20260810)';
study=strings(0,1); N=[]; K=[]; cls=strings(0,1); seed=[];
if mode=="smoke"
    specs={ ...
        "Scale",10,4,"Balanced",20260801; ...
        "Scale",20,4,"Balanced",20260802; ...
        "TypeCount",40,1,"TypeCount",20260801; ...
        "TypeCount",40,8,"TypeCount",20260802; ...
        "SizeClass",40,4,"LowDiversity",20260801; ...
        "SizeClass",40,4,"Complementary",20260802};
    for i=1:size(specs,1)
        study(end+1)=specs{i,1}; N(end+1)=specs{i,2}; K(end+1)=specs{i,3}; %#ok<AGROW>
        cls(end+1)=specs{i,4}; seed(end+1)=specs{i,5}; %#ok<AGROW>
    end
else
    for n=10:10:80
        for s=baseSeeds'
            study(end+1)="Scale"; N(end+1)=n; K(end+1)=4; %#ok<AGROW>
            cls(end+1)="Balanced"; seed(end+1)=s; %#ok<AGROW>
        end
    end
    for k=[1 2 4 5 8]
        for s=baseSeeds'
            study(end+1)="TypeCount"; N(end+1)=40; K(end+1)=k; %#ok<AGROW>
            cls(end+1)="TypeCount"; seed(end+1)=s; %#ok<AGROW>
        end
    end
    classes=["LowDiversity","Balanced","HighDiversity", ...
        "SmallDominated","LargeDominated","Complementary"];
    for c=classes
        for s=baseSeeds'
            study(end+1)="SizeClass"; N(end+1)=40; K(end+1)=4; %#ok<AGROW>
            cls(end+1)=c; seed(end+1)=s; %#ok<AGROW>
        end
    end
end
study=study(:); N=N(:); K=K(:); cls=cls(:); seed=seed(:);
limit=120*ones(size(N)); limit(N>=40)=300; limit(N>=60)=600;
caseID=compose("%s_N%d_K%d_%s_S%d",study,N,K,cls,seed);
D=table(caseID,study,N,K,cls,seed,limit, ...
    'VariableNames',{'CaseID','Study','N','K','InstanceClass','Seed','TimeLimit_s'});
end

function A=aggregateResults(T)
keys=unique(T(:,{'Study','N','K','InstanceClass'}),'rows','stable');
A=table();
for i=1:height(keys)
    hit=T.Study==keys.Study(i)&T.N==keys.N(i)&T.K==keys.K(i)& ...
        T.InstanceClass==keys.InstanceClass(i);
    X=T(hit,:);
    row=[keys(i,:) table(height(X),mean(X.PlanningSucceeded), ...
        mean(X.MeanPlanningTime_s,'omitnan'),std(X.MeanPlanningTime_s,'omitnan'), ...
        mean(X.PalletVolumeUtilization,'omitnan'), ...
        std(X.PalletVolumeUtilization,'omitnan'), ...
        mean(X.StackHeight_m,'omitnan'), ...
        mean(X.RequiredBufferCapacity,'omitnan'), ...
        mean(X.FeasibleWith8SlotBuffer), ...
        'VariableNames',{'Instances','SuccessRate','MeanPlanningTime_s', ...
        'StdPlanningTime_s','MeanPalletVolumeUtilization', ...
        'StdPalletVolumeUtilization','MeanStackHeight_m', ...
        'MeanRequiredBuffer','EightSlotFeasibilityRate'})];
    A=[A;row]; %#ok<AGROW>
end
end

function peak=peakBuffer(exec)
occ=0; peak=0;
for k=1:numel(exec)
    a=string(exec(k).action);
    if a=="conveyor_to_buffer",occ=occ+1;
    elseif a=="buffer_to_pallet",occ=occ-1;end
    peak=max(peak,occ);
end
end

function row=emptyMetric(t)
names={'Method','PlacedBoxes','MeanPlanningTime_s','PlanningTimeStd_s', ...
    'Layers','StackHeight_m','BoxVolume_m3','PalletVolumeUtilization', ...
    'UsableEnvelopeVolumeUtilization','MeanLayerAreaUtilization', ...
    'FirstLayerAreaUtilization','TopLayerAreaUtilization', ...
    'MinimumLayerAreaUtilization','LayerUtilizationStd','MinSupportRatio', ...
    'MinCOMSupportMargin_m','UnstableBoxCount','COMOffset_m', ...
    'NormalizedCOMOffset','COMHeight_m','MinStaticTippingAngle_deg', ...
    'ToBufferActions','FromBufferActions','PeakBufferOccupancy','RobotMoves', ...
    'TransferDistance_m','PayloadLiftEnergyProxy_J','BoxesPerLayer'};
v=cell(1,numel(names)); v{1}="Superbox";
for i=2:numel(names)-1,v{i}=NaN;end
v{3}=t; v{end}="";
row=cell2table(v,'VariableNames',names);
end
