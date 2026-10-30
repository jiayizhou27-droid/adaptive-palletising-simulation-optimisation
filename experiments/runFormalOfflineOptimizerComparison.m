function results=runFormalOfflineOptimizerComparison()
%RUNFORMALOFFLINEOPTIMIZERCOMPARISON Repeated GA/SA comparison.
% Six matched instances; Constructive once, GA and SA ten independent runs.
projectDir=fileparts(mfilename('fullpath'));
outDir=fullfile(projectDir,'formal_offline_optimizer_comparison');
if ~isfolder(outDir),mkdir(outDir);end
checkpoint=fullfile(outDir,'formal_optimizer_eval12_checkpoint.csv');
cfg=palletTaskConfig();
Ns=[20 40 60];instanceSeeds=[20260801 20260802];
methods=["ConstructiveSuperbox","GeneticAlgorithm","SimulatedAnnealing"];
repeats=[1 10 10];evalBudget=12;populationSize=6;
if isfile(checkpoint),T=readtable(checkpoint,'TextType','string');else,T=table();end

for n=Ns
for instanceSeed=instanceSeeds
    caseID=sprintf("N%d_S%d",n,instanceSeed);
    [b,~,~]=generatePalletBenchmarkInstance(n,4,"Balanced",instanceSeed);
    tc=tic;
    constructive=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,n, ...
        cfg.palletMargin,cfg.boxGap,false);
    constructiveTime=toc(tc);
    [~,ix]=sort([constructive.placementRank]);
    initialOrder=[constructive(ix).sourceID];
    for mi=1:numel(methods)
    for run=1:repeats(mi)
        method=methods(mi);key=caseID+"|"+method+"|"+run;
        if ~isempty(T)&&any(T.Key==key),fprintf('SKIP %s\n',key);continue;end
        fprintf('START %s\n',key);drawnow;
        success=false;reason="";metric=[];info=struct();required=NaN;
        tt=tic;
        try
            if method=="ConstructiveSuperbox"
                plan=constructive;elapsed=constructiveTime;info.evaluations=1;
            else
                algorithmSeed=instanceSeed+100000*mi+run;
                opt=struct('seed',algorithmSeed,'maxEvaluations',evalBudget, ...
                    'populationSize',populationSize,'initialOrder',initialOrder);
                if method=="GeneticAlgorithm",name="GA";else,name="SA";end
                [plan,info]=optimizeOfflineOrderPhysical(name,b,cfg,opt);
                elapsed=toc(tt);
            end
            assert(~isempty(plan)&&all([plan.isPlaced]));
            wide=cfg;wide.bufferCapacity=n;wide.bufferGrid=[ceil(n/2) 2];
            exec=buildBufferExecutionPlan(plan,wide);
            metric=computeExtendedOfflineMetrics(plan,exec,cfg,elapsed);
            required=metric.PeakBufferOccupancy;
            assert(metric.UnstableBoxCount==0);
            assert(metric.MinSupportRatio>=0.95-1e-9);
            assert(metric.MinCOMSupportMargin_m>=0.010-1e-9);
            assert(metric.MaximumOverhang_m<=1e-9);
            success=true;
        catch ME
            elapsed=toc(tt);reason=string(ME.message);
        end
        row=makeRow(key,caseID,n,instanceSeed,method,run,success,reason, ...
            elapsed,info,required,metric,cfg.bufferCapacity);
        if isempty(T),T=row;else,T=[T;row];end %#ok<AGROW>
        writetable(T,checkpoint);
        fprintf('DONE %s success=%d time=%.2f\n',key,success,elapsed);drawnow;
    end
    end
end
end
writetable(T,fullfile(outDir,'formal_optimizer_eval12_all_runs.csv'));
A=aggregateRuns(T);
writetable(A,fullfile(outDir,'formal_optimizer_eval12_summary.csv'));
save(fullfile(outDir,'formal_optimizer_eval12_results.mat'),'T','A','cfg','Ns', ...
    'instanceSeeds','evalBudget','populationSize');
results=struct('raw',T,'summary',A,'outputDirectory',outDir);
assignin('base','formalOfflineOptimizerResults',results);
fprintf('FORMAL_OFFLINE_OPTIMIZER_COMPLETE\n');
end

function row=makeRow(key,caseID,n,seed,method,run,ok,reason,time,info,required,M,cap)
evals=NaN;if isfield(info,'evaluations'),evals=info.evaluations;end
names={'Key','CaseID','N','InstanceSeed','Method','Run','PlanningSucceeded', ...
 'FailureReason','PlanningTime_s','Evaluations','Layers','StackHeight_m', ...
 'PalletVolumeUtilization','MeanLayerAreaUtilization','UnusedEnvelopeVolume_m3', ...
 'InternalVoidRatio','MinSupportRatio','MeanSupportRatio', ...
 'MinCOMSupportMargin_m','UnstableBoxCount','COMOffset_m', ...
 'MinStaticTippingAngle_deg','MaximumOverhang_m','SurfaceFlatness_m', ...
 'FootprintCompactness','MixedTypeAdjacencyScore','LayerWeightMonotonicity', ...
 'RequiredBufferCapacity','FeasibleWith8SlotBuffer','ToBufferActions', ...
 'BufferRetrievals','RobotMoves','TransferDistance_m','PayloadLiftEnergyProxy_J'};
v={key,caseID,n,seed,method,run,ok,reason,time,evals};
v=[v repmat({NaN},1,numel(names)-numel(v))];
if ok
    fields=names(11:end);
    for i=1:numel(fields)
        f=fields{i};
        if strcmp(f,'RequiredBufferCapacity'),v{10+i}=required;
        elseif strcmp(f,'FeasibleWith8SlotBuffer'),v{10+i}=required<=cap;
        elseif ismember(f,M.Properties.VariableNames),v{10+i}=M.(f);
        end
    end
end
row=cell2table(v,'VariableNames',names);
end

function A=aggregateRuns(T)
methods=unique(T.Method,'stable');A=table();
metrics={'PlanningTime_s','PalletVolumeUtilization','MeanLayerAreaUtilization', ...
 'StackHeight_m','InternalVoidRatio','MinSupportRatio','MeanSupportRatio', ...
 'MinCOMSupportMargin_m','COMOffset_m','MinStaticTippingAngle_deg', ...
 'SurfaceFlatness_m','FootprintCompactness','MixedTypeAdjacencyScore', ...
 'LayerWeightMonotonicity','RequiredBufferCapacity','RobotMoves', ...
 'TransferDistance_m','PayloadLiftEnergyProxy_J'};
for m=methods'
    X=T(T.Method==m,:);
    base=table(m,height(X),mean(X.PlanningSucceeded), ...
        sum(X.UnstableBoxCount,'omitnan'),max(X.MaximumOverhang_m,[],'omitnan'), ...
        'VariableNames',{'Method','Runs','FeasibilityRate', ...
        'TotalUnstableBoxes','MaximumOverhang_m'});
    for i=1:numel(metrics)
        z=X.(metrics{i});
        base.("Mean_"+metrics{i})=mean(z,'omitnan');
        base.("Std_"+metrics{i})=std(z,'omitnan');
        if metrics{i}=="PlanningTime_s"||metrics{i}=="StackHeight_m"|| ...
                metrics{i}=="InternalVoidRatio"||metrics{i}=="COMOffset_m"|| ...
                metrics{i}=="SurfaceFlatness_m"||metrics{i}=="RequiredBufferCapacity"|| ...
                metrics{i}=="RobotMoves"||metrics{i}=="TransferDistance_m"|| ...
                metrics{i}=="PayloadLiftEnergyProxy_J"
            base.("Best_"+metrics{i})=min(z,[],'omitnan');
        else
            base.("Best_"+metrics{i})=max(z,[],'omitnan');
        end
    end
    A=[A;base]; %#ok<AGROW>
end
end
