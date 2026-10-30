function results=runScaleTypeBufferStudy(shard)
%RUNSCALETYPEBUFFERSTUDY Scale, type complexity, and buffer sensitivity.
if nargin<1,shard="all";end
shard=string(shard);
root=fileparts(mfilename('fullpath'));
switch shard
    case "low", folder='scale_type_buffer_study_B25';
    case "mid", folder='scale_type_buffer_study_B25_mid';
    case "high", folder='scale_type_buffer_study_B25_high';
    case "type", folder='scale_type_buffer_study_B25_type';
    otherwise, folder='scale_type_buffer_study_B25_all';
end
outDir=fullfile(root,folder);
planDir=fullfile(outDir,'plans');
instanceDir=fullfile(outDir,'instances');
if ~isfolder(outDir),mkdir(outDir);end
if ~isfolder(planDir),mkdir(planDir);end
if ~isfolder(instanceDir),mkdir(instanceDir);end
layoutFile=fullfile(outDir,'layout_checkpoint.csv');
bufferFile=fullfile(outDir,'buffer_checkpoint.csv');
if isfile(layoutFile),L=readtable(layoutFile,'TextType','string');else,L=table();end
if isfile(bufferFile),B=readtable(bufferFile,'TextType','string');else,B=table();end
cfg=palletTaskConfig();methods=["Constructive","CS-GA","CS-SA"];
evaluationBudget=25;populationSize=8;
design=buildDesign();
switch shard
    case "low", design=design(design.Study=="Scale"&design.N<=50,:);
    case "mid", design=design(design.Study=="Scale"&design.N>=60&design.N<=80,:);
    case "high",design=design(design.Study=="Scale"&design.N>=90,:);
    case "type",design=design(design.Study=="TypeComplexity",:);
end

% Unrecorded MATLAB/JIT warm-up.
[wb,~,~,~]=generateIndependentGeometryInstance(10,4,50999);
wc=buildLayoutPlanBatch3(wb,cfg.palletDim,cfg.palletLoc,10, ...
    cfg.palletMargin,cfg.boxGap,false);
wo=struct('seed',1,'maxEvaluations',12,'populationSize',6,'initialPlan',wc);
optimizeConstructiveSeededPhysical("CS-GA",wb,cfg,wo);
optimizeConstructiveSeededPhysical("CS-SA",wb,cfg,wo);
fprintf('FULL STUDY WARMUP COMPLETE\n');drawnow;

for q=1:height(design)
    spec=design(q,:);caseID=spec.CaseID;
    [b,meta,typeTable,arrivalRandom]=generateIndependentGeometryInstance( ...
        spec.N,spec.K,spec.GeometrySeed);
    if ~isfile(fullfile(instanceDir,caseID+"_instance.mat"))
        save(fullfile(instanceDir,caseID+"_instance.mat"), ...
            'b','meta','typeTable','arrivalRandom','spec');
    end
    cPlanFile=fullfile(planDir,caseID+"_Constructive.mat");
    if isfile(cPlanFile)
        S=load(cPlanFile,'plan','planningTime');constructive=S.plan;
        constructiveTime=S.planningTime;
    else
        tt=tic;
        constructive=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc, ...
            spec.N,cfg.palletMargin,cfg.boxGap,false);
        constructiveTime=toc(tt);plan=constructive;planningTime=constructiveTime; %#ok<NASGU>
        save(cPlanFile,'plan','planningTime');
    end

    for mi=1:numel(methods)
        method=methods(mi);key=caseID+"|"+method;
        planFile=fullfile(planDir,keySafe(key)+".mat");
        if ~isempty(L)&&any(L.Key==key)&&isfile(planFile)
            fprintf('SKIP LAYOUT %s\n',key);
            S=load(planFile,'plan');plan=S.plan;
        else
            fprintf('START LAYOUT %s\n',key);drawnow;
            ok=false;reason="";searchTime=0;info=struct('evaluations',1);
            try
                if method=="Constructive"
                    plan=constructive;endTime=constructiveTime;
                else
                    opt=struct('seed',spec.GeometrySeed+100000*mi, ...
                        'maxEvaluations',evaluationBudget, ...
                        'populationSize',populationSize,'initialPlan',constructive);
                    tt=tic;
                    [plan,info]=optimizeConstructiveSeededPhysical(method,b,cfg,opt);
                    searchTime=toc(tt);endTime=constructiveTime+searchTime;
                end
                assert(~isempty(plan)&&all([plan.isPlaced]));
                M=layoutMetrics(plan,cfg,endTime);
                assert(M.UnstableBoxCount==0&&M.MaximumOverhang_m<=1e-9);
                assert(M.MinSupportRatio>=0.95-1e-9&&M.MinCOMSupportMargin_m>=.010-1e-9);
                ok=true;
                planningTime=endTime; %#ok<NASGU>
                save(planFile,'plan','planningTime','searchTime','info');
            catch ME
                reason=string(ME.message);endTime=constructiveTime+searchTime;M=[];
                plan=struct([]);
            end
            row=layoutRow(spec,method,ok,reason,constructiveTime,searchTime, ...
                endTime,info,M,evaluationBudget);
            L=append(L,row);writetable(L,layoutFile);
            fprintf('DONE LAYOUT %s success=%d E2E=%.2f\n',key,ok,endTime);drawnow;
        end

        % Buffer study is deliberately separated from layout generation.
        if spec.Study=="TypeComplexity" && spec.N==40 && any(spec.K==[4 8]) ...
                && ~isempty(plan)
            [orders,patterns]=arrivalOrders(b,spec.GeometrySeed);
            capacities=[0 2 4 8 12 16 24 inf];
            for pi=1:numel(patterns)
                bKey=key+"|"+patterns(pi);
                if ~isempty(B)&&any(B.BufferKey==bKey),continue;end
                arrivalPlan=plan(orders{pi});
                wide=cfg;wide.bufferCapacity=spec.N;wide.bufferGrid=[ceil(spec.N/2) 2];
                exec=buildBufferExecutionPlan(arrivalPlan,wide);
                required=peakBuffer(exec);
                actions=string({exec.action});real=actions~="hidden_warmup";
                distance=0;
                for ai=find(real)
                    distance=distance+norm(exec(ai).toPos-exec(ai).fromPos);
                end
                for capacity=capacities
                    row=table(bKey,caseID,spec.N,spec.K,spec.GeometrySeed, ...
                        method,patterns(pi),capacity,required,required<=capacity, ...
                        sum(actions=="conveyor_to_buffer"), ...
                        sum(actions=="buffer_to_pallet"),sum(real),distance, ...
                        'VariableNames',{'BufferKey','CaseID','N','K','GeometrySeed', ...
                        'Method','ArrivalPattern','BufferCapacity', ...
                        'RequiredBufferCapacity','ExecutionFeasible', ...
                        'ToBufferActions','FromBufferActions','RobotMoves', ...
                        'TransferDistance_m'});
                    B=append(B,row);
                end
                writetable(B,bufferFile);
            end
        end
    end
end
writetable(L,fullfile(outDir,'layout_results_all.csv'));
writetable(B,fullfile(outDir,'buffer_results_all.csv'));
LA=aggregateLayouts(L);BA=aggregateBuffer(B);
writetable(LA,fullfile(outDir,'layout_results_aggregate.csv'));
writetable(BA,fullfile(outDir,'buffer_results_aggregate.csv'));
makePlots(LA,BA,outDir);
save(fullfile(outDir,'scale_type_buffer_results.mat'),'L','B','LA','BA', ...
    'design','evaluationBudget','cfg');
results=struct('layout',L,'buffer',B,'layoutAggregate',LA, ...
    'bufferAggregate',BA,'outputDirectory',outDir);
assignin('base','scaleTypeBufferResults',results);
fprintf('SCALE_TYPE_BUFFER_STUDY_COMPLETE\n');
end

function design=buildDesign()
rows={};cursor=0;
for N=10:10:100
for rep=1:10
    cursor=cursor+1;seed=600000+1000*N+rep;
    rows(cursor,:)={"Scale",N,4,rep,seed,sprintf("Scale_N%03d_K04_R%02d",N,rep)}; %#ok<AGROW>
end
end
for K=[2 4 6 8 10]
for rep=1:10
    cursor=cursor+1;seed=700000+1000*K+rep;
    rows(cursor,:)={"TypeComplexity",40,K,rep,seed,sprintf("Type_N040_K%02d_R%02d",K,rep)}; %#ok<AGROW>
end
end
design=cell2table(rows,'VariableNames', ...
    {'Study','N','K','Instance','GeometrySeed','CaseID'});
design.Study=string(design.Study);design.CaseID=string(design.CaseID);
end
function M=layoutMetrics(plan,cfg,time)
e=struct('action',{},'boxID',{},'sourceID',{},'bufferSlot',{}, ...
    'fromPos',{},'toPos',{},'targetYaw',{});
M=computeExtendedOfflineMetrics(plan,e,cfg,time);
end
function row=layoutRow(s,method,ok,reason,seedTime,searchTime,endTime,info,M,budget)
evals=NaN;if isfield(info,'evaluations'),evals=info.evaluations;end
names={'Key','CaseID','Study','N','K','Instance','GeometrySeed','Method', ...
    'EvaluationBudget','PlanningSucceeded','FailureReason','SeedTime_s', ...
    'SearchTime_s','EndToEndTime_s','Evaluations','Layers','StackHeight_m', ...
    'VolumeUtilization','InternalVoidRatio','MeanLayerAreaUtilization', ...
    'MinSupportRatio','MeanSupportRatio','MinCOMMargin_m','COMOffset_m', ...
    'TippingAngle_deg','SurfaceFlatness_m','FootprintCompactness', ...
    'MixedTypeScore','WeightMonotonicity','UnstableBoxCount','MaximumOverhang_m'};
key=s.CaseID+"|"+method;
v={key,s.CaseID,s.Study,s.N,s.K,s.Instance,s.GeometrySeed,method,budget, ...
    ok,reason,seedTime,searchTime,endTime,evals};
v=[v repmat({NaN},1,numel(names)-numel(v))];
if ok
    fields={'Layers','StackHeight_m','PalletVolumeUtilization','InternalVoidRatio', ...
        'MeanLayerAreaUtilization','MinSupportRatio','MeanSupportRatio', ...
        'MinCOMSupportMargin_m','COMOffset_m','MinStaticTippingAngle_deg', ...
        'SurfaceFlatness_m','FootprintCompactness','MixedTypeAdjacencyScore', ...
        'LayerWeightMonotonicity','UnstableBoxCount','MaximumOverhang_m'};
    for i=1:numel(fields),v{15+i}=M.(fields{i});end
end
row=cell2table(v,'VariableNames',names);
end
function A=aggregateLayouts(L)
keys=unique(L(:,{'Study','N','K','Method'}),'rows','stable');A=table();
for i=1:height(keys)
    x=L(L.Study==keys.Study(i)&L.N==keys.N(i)&L.K==keys.K(i)& ...
        L.Method==keys.Method(i),:);
    r=[keys(i,:) table(height(x),mean(x.PlanningSucceeded), ...
        mean(x.VolumeUtilization,'omitnan'),std(x.VolumeUtilization,'omitnan'), ...
        mean(x.StackHeight_m,'omitnan'),mean(x.EndToEndTime_s,'omitnan'), ...
        std(x.EndToEndTime_s,'omitnan'),min(x.MinSupportRatio,[],'omitnan'), ...
        min(x.MinCOMMargin_m,[],'omitnan'),sum(x.UnstableBoxCount,'omitnan'), ...
        mean(x.SurfaceFlatness_m,'omitnan'),mean(x.FootprintCompactness,'omitnan'), ...
        'VariableNames',{'IndependentInstances','FeasibilityRate', ...
        'MeanVolumeUtilization','StdVolumeUtilization','MeanStackHeight_m', ...
        'MeanEndToEndTime_s','StdEndToEndTime_s','MinimumSupportRatio', ...
        'MinimumCOMMargin_m','TotalUnstableBoxes','MeanSurfaceFlatness_m', ...
        'MeanFootprintCompactness'})];
    A=[A;r]; %#ok<AGROW>
end
end
function A=aggregateBuffer(B)
if isempty(B),A=table();return;end
keys=unique(B(:,{'K','Method','ArrivalPattern','BufferCapacity'}),'rows','stable');A=table();
for i=1:height(keys)
    x=B(B.K==keys.K(i)&B.Method==keys.Method(i)& ...
        B.ArrivalPattern==keys.ArrivalPattern(i)& ...
        B.BufferCapacity==keys.BufferCapacity(i),:);
    r=[keys(i,:) table(height(x),mean(x.ExecutionFeasible), ...
        mean(x.RequiredBufferCapacity),mean(x.RobotMoves), ...
        mean(x.TransferDistance_m), ...
        'VariableNames',{'IndependentInstances','ExecutionFeasibilityRate', ...
        'MeanRequiredBuffer','MeanRobotMoves','MeanTransferDistance_m'})];
    A=[A;r]; %#ok<AGROW>
end
end
function [orders,patterns]=arrivalOrders(b,seed)
n=numel(b.length);volume=b.length.*b.width.*b.height;rng(seed,'twister');
patterns=["Random","LargeFirst","SmallFirst","TypeClustered","Alternating"];
orders=cell(5,1);orders{1}=randperm(n);[~,orders{2}]=sort(volume,'descend');
[~,orders{3}]=sort(volume,'ascend');
types=unique(b.type,'stable');o=[];
for t=types',o=[o;find(b.type==t)];end %#ok<AGROW>
orders{4}=o';sorted=orders{2};alt=zeros(1,n);lo=1;hi=n;
for i=1:n
    if mod(i,2),alt(i)=sorted(lo);lo=lo+1;else,alt(i)=sorted(hi);hi=hi-1;end
end
orders{5}=alt;
end
function peak=peakBuffer(exec)
occ=0;peak=0;
for i=1:numel(exec)
    a=string(exec(i).action);
    if a=="conveyor_to_buffer",occ=occ+1;elseif a=="buffer_to_pallet",occ=occ-1;end
    peak=max(peak,occ);
end
end
function makePlots(LA,BA,outDir)
methods=["Constructive","CS-GA","CS-SA"];colors=[.78 .48 .12;.20 .41 .63;.18 .56 .51];
f=figure('Visible','off','Color','w','Position',[50 50 1400 850]);
tl=tiledlayout(f,2,2,'TileSpacing','compact');
fields={'MeanVolumeUtilization','MeanStackHeight_m','MeanEndToEndTime_s','FeasibilityRate'};
titles={'Volume utilization','Stack height (m)','End-to-end time (s)','Layout feasibility'};
for p=1:4
    ax=nexttile(tl);hold(ax,'on');
    for m=1:3
        x=LA(LA.Study=="Scale"&LA.Method==methods(m),:);x=sortrows(x,'N');
        plot(ax,x.N,x.(fields{p}),'-o','LineWidth',1.7,'Color',colors(m,:), ...
            'DisplayName',methods(m));
    end
    grid(ax,'on');xlabel(ax,'Number of boxes, N');title(ax,titles{p});
end
legend(nexttile(tl,1),'Location','best');title(tl,'Scale study: 10 independent instances per N');
exportgraphics(f,fullfile(outDir,'scale_study_overview.png'),'Resolution',200);close(f);

f=figure('Visible','off','Color','w','Position',[50 50 1400 650]);
tl=tiledlayout(f,1,2,'TileSpacing','compact');
for p=1:2
    ax=nexttile(tl);hold(ax,'on');field={'MeanVolumeUtilization','MeanEndToEndTime_s'};
    ttl={'Volume utilization','End-to-end time (s)'};
    for m=1:3
        x=LA(LA.Study=="TypeComplexity"&LA.Method==methods(m),:);x=sortrows(x,'K');
        plot(ax,x.K,x.(field{p}),'-o','LineWidth',1.7,'Color',colors(m,:), ...
            'DisplayName',methods(m));
    end
    grid(ax,'on');xlabel(ax,'Number of box types, K');title(ax,ttl{p});
end
legend(nexttile(tl,1),'Location','best');title(tl,'Box-type complexity: N=40');
exportgraphics(f,fullfile(outDir,'type_complexity_overview.png'),'Resolution',200);close(f);

if isempty(BA),return;end
f=figure('Visible','off','Color','w','Position',[50 50 1400 700]);
tl=tiledlayout(f,1,2,'TileSpacing','compact');
for ki=1:2
    K=[4 8];ax=nexttile(tl);hold(ax,'on');
    for m=1:3
        x=BA(BA.K==K(ki)&BA.Method==methods(m)&BA.ArrivalPattern=="Random",:);
        x=x(isfinite(x.BufferCapacity),:);x=sortrows(x,'BufferCapacity');
        plot(ax,x.BufferCapacity,x.ExecutionFeasibilityRate,'-o','LineWidth',1.7, ...
            'Color',colors(m,:),'DisplayName',methods(m));
    end
    grid(ax,'on');xlabel(ax,'Buffer capacity');ylabel(ax,'Execution feasibility');
    title(ax,sprintf('K=%d, random arrival',K(ki)));
end
legend(nexttile(tl,1),'Location','best');title(tl,'Buffer-capacity sensitivity');
exportgraphics(f,fullfile(outDir,'buffer_capacity_overview.png'),'Resolution',200);close(f);
end
function T=append(T,row)
if isempty(T),T=row;else,T=[T;row];end
end
function s=keySafe(s),s=replace(s,["|","/","\"],"_");end
