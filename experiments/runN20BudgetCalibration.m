function results=runN20BudgetCalibration()
%RUNN20BUDGETCALIBRATION Budget calibration for Constructive/CS-GA/CS-SA.
% Ten independent geometry instances; arrival order is saved separately
% and is never used by the layout optimizers.
root=fileparts(mfilename('fullpath'));
outDir=fullfile(root,'n20_budget_calibration');
instanceDir=fullfile(outDir,'instances');
if ~isfolder(outDir),mkdir(outDir);end
if ~isfolder(instanceDir),mkdir(instanceDir);end
checkpoint=fullfile(outDir,'n20_budget_checkpoint_v2.csv');
if isfile(checkpoint),T=readtable(checkpoint,'TextType','string');else,T=table();end
cfg=palletTaskConfig();N=20;K=4;
geometrySeeds=41001:41010;
budgets=[12 25 50 100];
repeats=3;populationSize=8;

% Unrecorded JIT warm-up for all three code paths.
fprintf('WARMUP START\n');drawnow;
[wb,~,~,~]=generateIndependentGeometryInstance(12,4,40999);
wc=buildLayoutPlanBatch3(wb,cfg.palletDim,cfg.palletLoc,12, ...
    cfg.palletMargin,cfg.boxGap,false);
wo=struct('seed',1,'maxEvaluations',12,'populationSize',6,'initialPlan',wc);
optimizeConstructiveSeededPhysical("CS-GA",wb,cfg,wo);
optimizeConstructiveSeededPhysical("CS-SA",wb,cfg,wo);
fprintf('WARMUP COMPLETE\n');drawnow;

for q=1:numel(geometrySeeds)
    geometrySeed=geometrySeeds(q);
    caseID=sprintf("N20_G%05d",geometrySeed);
    [b,meta,typeTable,arrivalOrder]=generateIndependentGeometryInstance(N,K,geometrySeed);
    writetable(typeTable,fullfile(instanceDir,caseID+"_geometry.csv"));
    writetable(table((1:N)',arrivalOrder,'VariableNames',{'ArrivalRank','BoxID'}), ...
        fullfile(instanceDir,caseID+"_arrival_order_buffer_only.csv"));
    save(fullfile(instanceDir,caseID+"_instance.mat"),'b','meta','typeTable','arrivalOrder');

    cKey=caseID+"|Constructive|B0|R1";
    hit=~isempty(T)&&any(T.Key==cKey);
    if hit
        cRow=T(T.Key==cKey,:);constructiveTime=cRow.SeedTime_s(1);
        constructive=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,N, ...
            cfg.palletMargin,cfg.boxGap,false);
        baseUtil=cRow.VolumeUtilization(1);baseHeight=cRow.StackHeight_m(1);
    else
        tt=tic;
        constructive=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,N, ...
            cfg.palletMargin,cfg.boxGap,false);
        constructiveTime=toc(tt);
        M=layoutMetrics(constructive,cfg,constructiveTime);
        baseUtil=M.PalletVolumeUtilization;baseHeight=M.StackHeight_m;
        row=packRow(cKey,caseID,geometrySeed,"Constructive",0,1,true,"", ...
            constructiveTime,0,constructiveTime,1,M,0,0);
        T=appendRow(T,row);writetable(T,checkpoint);
        fprintf('DONE %s time=%.2f\n',cKey,constructiveTime);drawnow;
    end

    for budget=budgets
    for method=["CS-GA","CS-SA"]
    for repeat=1:repeats
        key=caseID+"|"+method+"|B"+budget+"|R"+repeat;
        if ~isempty(T)&&any(T.Key==key),fprintf('SKIP %s\n',key);continue;end
        fprintf('START %s\n',key);drawnow;
        success=false;reason="";M=[];info=struct();searchTime=NaN;
        try
            opt=struct('seed',geometrySeed+100000*(method=="CS-SA")+1000*budget+repeat, ...
                'maxEvaluations',budget,'populationSize',populationSize, ...
                'initialPlan',constructive);
            tt=tic;
            [plan,info]=optimizeConstructiveSeededPhysical(method,b,cfg,opt);
            searchTime=toc(tt);endToEnd=constructiveTime+searchTime;
            assert(~isempty(plan)&&all([plan.isPlaced]));
            M=layoutMetrics(plan,cfg,endToEnd);
            assert(M.UnstableBoxCount==0&&M.MaximumOverhang_m<=1e-9);
            assert(M.MinSupportRatio>=0.95-1e-9);
            assert(M.MinCOMSupportMargin_m>=0.010-1e-9);
            success=true;
        catch ME
            reason=string(ME.message);endToEnd=constructiveTime+searchTime;
        end
        evals=NaN;if isfield(info,'evaluations'),evals=info.evaluations;end
        if success
            utilImprovement=M.PalletVolumeUtilization-baseUtil;
            heightImprovement=baseHeight-M.StackHeight_m;
        else
            utilImprovement=NaN;heightImprovement=NaN;
        end
        row=packRow(key,caseID,geometrySeed,method,budget,repeat,success, ...
            reason,constructiveTime,searchTime,endToEnd,evals,M, ...
            utilImprovement,heightImprovement);
        T=appendRow(T,row);writetable(T,checkpoint);
        fprintf('DONE %s success=%d E2E=%.2f dUtil=%g\n', ...
            key,success,endToEnd,utilImprovement);drawnow;
    end
    end
    end
end
writetable(T,fullfile(outDir,'n20_budget_all_runs.csv'));
A=aggregateBudget(T);
writetable(A,fullfile(outDir,'n20_budget_summary.csv'));
makePlots(A,outDir);
save(fullfile(outDir,'n20_budget_results.mat'),'T','A','budgets', ...
    'geometrySeeds','repeats','cfg');
results=struct('raw',T,'summary',A,'outputDirectory',outDir);
assignin('base','n20BudgetCalibrationResults',results);
fprintf('N20_BUDGET_CALIBRATION_COMPLETE\n');
end

function M=layoutMetrics(plan,cfg,time)
e=struct('action',{},'boxID',{},'sourceID',{},'bufferSlot',{}, ...
    'fromPos',{},'toPos',{},'targetYaw',{});
M=computeExtendedOfflineMetrics(plan,e,cfg,time);
end
function T=appendRow(T,row)
if isempty(T),T=row;else,T=[T;row];end
end
function row=packRow(key,caseID,seed,method,budget,repeat,ok,reason, ...
        seedTime,searchTime,endTime,evals,M,dUtil,dHeight)
names={'Key','CaseID','GeometrySeed','Method','Budget','Repeat', ...
 'PlanningSucceeded','FailureReason','SeedTime_s','SearchTime_s', ...
 'EndToEndTime_s','Evaluations','Layers','StackHeight_m', ...
 'VolumeUtilization','InternalVoidRatio','MeanLayerAreaUtilization', ...
 'MinSupportRatio','MeanSupportRatio','MinCOMMargin_m','COMOffset_m', ...
 'TippingAngle_deg','SurfaceFlatness_m','FootprintCompactness', ...
 'MixedTypeScore','WeightMonotonicity','UnstableBoxCount', ...
 'MaximumOverhang_m','VolumeUtilizationImprovement', ...
 'StackHeightImprovement_m'};
v={key,caseID,seed,method,budget,repeat,ok,reason,seedTime,searchTime, ...
    endTime,evals};v=[v repmat({NaN},1,numel(names)-numel(v))];
if ok
    source={'Layers','StackHeight_m','PalletVolumeUtilization', ...
        'InternalVoidRatio','MeanLayerAreaUtilization','MinSupportRatio', ...
        'MeanSupportRatio','MinCOMSupportMargin_m','COMOffset_m', ...
        'MinStaticTippingAngle_deg','SurfaceFlatness_m', ...
        'FootprintCompactness','MixedTypeAdjacencyScore', ...
        'LayerWeightMonotonicity','UnstableBoxCount','MaximumOverhang_m'};
    for i=1:numel(source),v{12+i}=M.(source{i});end
    v{29}=dUtil;v{30}=dHeight;
end
row=cell2table(v,'VariableNames',names);
end
function A=aggregateBudget(T)
X=T(T.Method~="Constructive",:);
keys=unique(X(:,{'Method','Budget'}),'rows','stable');A=table();
for i=1:height(keys)
    q=X(X.Method==keys.Method(i)&X.Budget==keys.Budget(i),:);
    row=[keys(i,:) table(height(q),numel(unique(q.CaseID)), ...
        mean(q.PlanningSucceeded),mean(q.EndToEndTime_s,'omitnan'), ...
        std(q.EndToEndTime_s,'omitnan'), ...
        mean(q.VolumeUtilization,'omitnan'),std(q.VolumeUtilization,'omitnan'), ...
        mean(q.VolumeUtilizationImprovement,'omitnan'), ...
        max(q.VolumeUtilizationImprovement,[],'omitnan'), ...
        mean(q.StackHeightImprovement_m,'omitnan'), ...
        mean(q.VolumeUtilizationImprovement>1e-9,'omitnan'), ...
        min(q.MinSupportRatio,[],'omitnan'), ...
        min(q.MinCOMMargin_m,[],'omitnan'),sum(q.UnstableBoxCount,'omitnan'), ...
        'VariableNames',{'Runs','IndependentInstances','FeasibilityRate', ...
        'MeanEndToEndTime_s','StdEndToEndTime_s','MeanVolumeUtilization', ...
        'StdVolumeUtilization','MeanUtilizationImprovement', ...
        'BestUtilizationImprovement','MeanHeightImprovement_m', ...
        'ImprovementRate','MinimumSupportRatio','MinimumCOMMargin_m', ...
        'TotalUnstableBoxes'})];
    A=[A;row]; %#ok<AGROW>
end
end
function makePlots(A,outDir)
methods=["CS-GA","CS-SA"];colors=[.20 .41 .63;.18 .56 .51];
f=figure('Visible','off','Color','w','Position',[80 80 1300 850]);
tl=tiledlayout(f,2,2,'TileSpacing','compact');
fields={'MeanVolumeUtilization','MeanUtilizationImprovement', ...
    'MeanEndToEndTime_s','ImprovementRate'};
titles={'Mean volume utilization','Improvement over Constructive', ...
    'End-to-end planning time (s)','Fraction improving Constructive'};
for p=1:4
    ax=nexttile(tl);hold(ax,'on');
    for m=1:2
        q=A(A.Method==methods(m),:);q=sortrows(q,'Budget');
        plot(ax,q.Budget,q.(fields{p}),'-o','LineWidth',1.8, ...
            'Color',colors(m,:),'DisplayName',methods(m));
    end
    grid(ax,'on');xlabel(ax,'Candidate-evaluation budget');title(ax,titles{p});
end
legend(nexttile(tl,1),'Location','best');
title(tl,'N=20 budget calibration: 10 independent geometry instances');
exportgraphics(f,fullfile(outDir,'n20_budget_calibration_overview.png'),'Resolution',200);
close(f);
end
