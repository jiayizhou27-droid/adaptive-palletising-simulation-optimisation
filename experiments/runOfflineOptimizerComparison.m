function results=runOfflineOptimizerComparison()
%RUNOFFLINEOPTIMIZERCOMPARISON Constructive vs GA vs SA with hard physics.
% Six matched scale instances are used: N={20,40,60}, two fixed seeds each.
% GA and SA receive the same constructive feasible incumbent and exactly
% the same candidate-evaluation budget. No Simulink execution is run.
projectDir=fileparts(mfilename('fullpath'));
outDir=fullfile(projectDir,'offline_optimizer_comparison');
if ~isfolder(outDir),mkdir(outDir);end
checkpoint=fullfile(outDir,'offline_optimizer_checkpoint.csv');
cfg=palletTaskConfig();
methods=["ConstructiveSuperbox","GeneticAlgorithm","SimulatedAnnealing"];
Ns=[20 40 60]; seeds=[20260801 20260802];
evalBudget=12; populationSize=6;
if isfile(checkpoint),T=readtable(checkpoint,'TextType','string');else,T=emptyRows();end

caseCounter=0;
for n=Ns
for seed=seeds
    caseCounter=caseCounter+1;
    caseID=sprintf("N%d_S%d",n,seed);
    [b,~,~]=generatePalletBenchmarkInstance(n,4,"Balanced",seed);

    fprintf('CONSTRUCT %s\n',caseID);drawnow;
    t=tic;
    constructive=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,n, ...
        cfg.palletMargin,cfg.boxGap,false);
    constructiveTime=toc(t);
    [~,idx]=sort([constructive.placementRank]);
    initialOrder=[constructive(idx).sourceID];

    for m=1:numel(methods)
        method=methods(m);
        if any(T.CaseID==caseID&T.Method==method)
            fprintf('SKIP %s %s\n',caseID,method);continue
        end
        success=false;reason="";plan=struct([]);searchInfo=struct();
        t=tic;
        try
            if method=="ConstructiveSuperbox"
                plan=constructive; elapsed=constructiveTime;
                searchInfo.evaluations=1; searchInfo.complete=true;
            else
                opt=struct('seed',seed+1000*m,'maxEvaluations',evalBudget, ...
                    'populationSize',populationSize,'initialOrder',initialOrder);
                if method=="GeneticAlgorithm",key="GA";else,key="SA";end
                [plan,searchInfo]=optimizeOfflineOrderPhysical(key,b,cfg,opt);
                elapsed=toc(t);
            end
            assert(~isempty(plan)&&all([plan.isPlaced]),'Optimizer did not place every box.');
            wide=cfg;wide.bufferCapacity=n;wide.bufferGrid=[ceil(n/2) 2];
            execution=buildBufferExecutionPlan(plan,wide);
            metric=computeLayoutComparisonMetrics(method,plan,execution,cfg,elapsed,NaN);
            required=peakBuffer(execution);
            assert(metric.UnstableBoxCount==0,'Physical stability verification failed.');
            assert(metric.MinSupportRatio>=0.95-1e-9,'Support ratio below 95%%.');
            assert(metric.MinCOMSupportMargin_m>=0.010-1e-9,'COM margin below 10 mm.');
            success=true;
        catch ME
            elapsed=toc(t);reason=string(ME.message);required=NaN;
            metric=[];
        end
        row=pack(caseID,n,seed,method,success,reason,elapsed,required, ...
            searchInfo,metric,cfg.bufferCapacity);
        T=[T;row]; %#ok<AGROW>
        writetable(T,checkpoint);
        fprintf('DONE %s %s success=%d time=%.2f eval=%g\n', ...
            caseID,method,success,elapsed,row.Evaluations);drawnow;
    end
end
end
assert(height(T)==numel(Ns)*numel(seeds)*numel(methods));
writetable(T,fullfile(outDir,'offline_optimizer_raw_results.csv'));
A=aggregate(T);
writetable(A,fullfile(outDir,'offline_optimizer_aggregate.csv'));
makePlots(T,A,outDir);
save(fullfile(outDir,'offline_optimizer_results.mat'),'T','A','cfg','Ns','seeds');
results=struct('raw',T,'aggregate',A,'outputDirectory',outDir);
assignin('base','offlineOptimizerResults',results);
end

function row=pack(caseID,n,seed,method,ok,reason,time,required,info,metric,cap)
evals=NaN;if isfield(info,'evaluations'),evals=info.evaluations;end
vals={string(caseID),n,seed,string(method),ok,string(reason),time,evals, ...
    NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,required,ok&&required<=cap,NaN,NaN,NaN};
if ok
    vals(9:16)={metric.Layers,metric.StackHeight_m,metric.PalletVolumeUtilization, ...
        metric.MeanLayerAreaUtilization,metric.MinSupportRatio, ...
        metric.MinCOMSupportMargin_m,metric.UnstableBoxCount, ...
        metric.MinStaticTippingAngle_deg};
    vals(19:21)={metric.ToBufferActions,metric.RobotMoves,metric.TransferDistance_m};
end
row=cell2table(vals,'VariableNames',names());
end

function T=emptyRows()
types=["string","double","double","string","logical","string", ...
    repmat("double",1,11),"logical",repmat("double",1,3)];
T=table('Size',[0 numel(names())],'VariableTypes',cellstr(types),'VariableNames',names());
end
function n=names()
n={'CaseID','N','Seed','Method','PlanningSucceeded','FailureReason', ...
    'PlanningTime_s','Evaluations','Layers','StackHeight_m', ...
    'PalletVolumeUtilization','MeanLayerAreaUtilization','MinSupportRatio', ...
    'MinCOMSupportMargin_m','UnstableBoxCount','MinStaticTippingAngle_deg', ...
    'RequiredBufferCapacity','FeasibleWith8SlotBuffer','ToBufferActions', ...
    'RobotMoves','TransferDistance_m'};
end
function A=aggregate(T)
methods=unique(T.Method,'stable');A=table();
for m=methods'
    X=T(T.Method==m,:);
    row=table(m,height(X),mean(X.PlanningSucceeded), ...
        mean(X.PlanningTime_s,'omitnan'),std(X.PlanningTime_s,'omitnan'), ...
        mean(X.PalletVolumeUtilization,'omitnan'), ...
        mean(X.Layers,'omitnan'),mean(X.RequiredBufferCapacity,'omitnan'), ...
        mean(X.FeasibleWith8SlotBuffer),min(X.MinSupportRatio,[],'omitnan'), ...
        min(X.MinCOMSupportMargin_m,[],'omitnan'), ...
        sum(X.UnstableBoxCount,'omitnan'), ...
        'VariableNames',{'Method','Instances','SuccessRate','MeanPlanningTime_s', ...
        'StdPlanningTime_s','MeanVolumeUtilization','MeanLayers', ...
        'MeanRequiredBuffer','EightSlotFeasibilityRate','MinSupportRatio', ...
        'MinCOMSupportMargin_m','UnstableBoxCount'});
    A=[A;row]; %#ok<AGROW>
end
end
function makePlots(T,A,outDir)
methods=["ConstructiveSuperbox","GeneticAlgorithm","SimulatedAnnealing"];
labels=["Constructive Superbox","Genetic Algorithm","Simulated Annealing"];
colors=[.78 .48 .10;.18 .39 .63;.18 .55 .52];
f=figure('Visible','off','Color','w','Position',[80 80 1500 760]);
tl=tiledlayout(f,2,2,'TileSpacing','compact');
metrics={'SuccessRate','MeanPlanningTime_s','MeanVolumeUtilization','MeanRequiredBuffer'};
titles={'Planning success rate','Planning time (s)','Pallet volume utilization','Required buffer'};
for q=1:4
    ax=nexttile(tl);vals=zeros(1,3);
    for m=1:3,vals(m)=A.(metrics{q})(A.Method==methods(m));end
    h=bar(ax,vals,'FaceColor','flat');h.CData=colors;
    ax.XTick=1:3;ax.XTickLabel=labels;xtickangle(ax,15);grid(ax,'on');title(ax,titles{q});
end
title(tl,'Offline global optimizer comparison with identical hard physics');
exportgraphics(f,fullfile(outDir,'offline_optimizer_overall.png'),'Resolution',180);close(f);

f=figure('Visible','off','Color','w','Position',[80 80 1400 600]);
tl=tiledlayout(f,1,2,'TileSpacing','compact');
scaleLevels=[20 40 60];
for q=1:2
    ax=nexttile(tl);hold(ax,'on');
    metric={'PalletVolumeUtilization','PlanningTime_s'};ttl={'Volume utilization','Planning time (s)'};
    for m=1:3
        Y=zeros(1,3);
        for j=1:3
            x=T(T.Method==methods(m)&T.N==scaleLevels(j),:);
            Y(j)=mean(x.(metric{q}),'omitnan');
        end
        plot(ax,scaleLevels,Y,'-o','LineWidth',1.8,'Color',colors(m,:), ...
            'DisplayName',labels(m));
    end
    grid(ax,'on');xlabel(ax,'Number of boxes, N');title(ax,ttl{q});
end
legend(nexttile(tl,1),'Location','best');
exportgraphics(f,fullfile(outDir,'offline_optimizer_by_scale.png'),'Resolution',180);close(f);
end
function peak=peakBuffer(exec)
occ=0;peak=0;
for k=1:numel(exec)
    a=string(exec(k).action);
    if a=="conveyor_to_buffer",occ=occ+1;elseif a=="buffer_to_pallet",occ=occ-1;end
    peak=max(peak,occ);
end
end
