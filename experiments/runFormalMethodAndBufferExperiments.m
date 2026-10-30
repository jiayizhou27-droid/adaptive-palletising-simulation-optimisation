function results=runFormalMethodAndBufferExperiments(mode)
%RUNFORMALMETHODANDBUFFEREXPERIMENTS Fair three-method and buffer study.
% All methods receive identical generated cartons, arrival order, pallet
% geometry, upright-only orientations, gap and physical stability rules.
% Buffer sensitivity never changes a layout; it only tests whether the
% already generated execution schedule fits each candidate capacity.
if nargin<1, mode="full"; end
mode=string(mode);
projectDir=fileparts(mfilename('fullpath'));
cfg=palletTaskConfig();
formalDir=fullfile(projectDir,'formal_benchmark_experiments');
outDir=fullfile(projectDir,'method_buffer_experiments');
if ~isfolder(outDir), mkdir(outDir); end
checkpointFile=fullfile(outDir,'method_comparison_checkpoint.csv');
formalFile=fullfile(formalDir,'formal_benchmark_checkpoint.csv');
assert(isfile(formalFile),'The completed 190-instance Superbox file is missing.');
formal=readtable(formalFile,'TextType','string');
assert(height(formal)==190 && numel(unique(formal.CaseID))==190, ...
    'Formal benchmark must contain exactly 190 unique cases.');
design=formal(:,{'CaseID','Study','N','K','InstanceClass','Seed'});
if mode=="smoke", design=design([1 11],:); end
methods=["Shelf","RollingHorizon3","Superbox"];
if isfile(checkpointFile)
    existing=readtable(checkpointFile,'TextType','string');
else
    existing=emptyResultTable();
end

total=height(design)*numel(methods);
counter=0;
for q=1:height(design)
    [b,~,~]=generatePalletBenchmarkInstance(design.N(q),design.K(q), ...
        design.InstanceClass(q),design.Seed(q));
    for m=1:numel(methods)
        counter=counter+1;
        method=methods(m);
        hit=existing.CaseID==design.CaseID(q) & existing.Method==method;
        if any(hit)
            fprintf('SKIP %d/%d %s %s\n',counter,total,design.CaseID(q),method);
            continue
        end
        fprintf('START %d/%d %s %s\n',counter,total,design.CaseID(q),method);
        drawnow;
        if method=="Superbox"
            src=formal(formal.CaseID==design.CaseID(q),:);
            row=fromFormalRow(src);
        else
            row=runOne(design(q,:),method,b,cfg);
        end
        existing=[existing;row]; %#ok<AGROW>
        writetable(existing,checkpointFile);
        fprintf('DONE %s %s success=%d time=%.3f buffer=%.0f\n', ...
            design.CaseID(q),method,row.PlanningSucceeded, ...
            row.PlanningTime_s,row.RequiredBufferCapacity);
        drawnow;
    end
end

if mode=="full"
    assert(height(existing)==570,'Expected 570 method-case rows, got %d.',height(existing));
    assert(numel(unique(existing.CaseID+"|"+existing.Method))==570, ...
        'Method comparison contains duplicate method-case keys.');
    writetable(existing,fullfile(outDir,'method_comparison_all_570.csv'));
    aggregate=aggregateComparison(existing);
    writetable(aggregate,fullfile(outDir,'method_comparison_aggregate.csv'));
    buffer=buildBufferSensitivity(existing);
    writetable(buffer,fullfile(outDir,'buffer_capacity_sensitivity_all.csv'));
    bufferAgg=aggregateBuffer(buffer);
    writetable(bufferAgg,fullfile(outDir,'buffer_capacity_sensitivity_aggregate.csv'));
    makePlots(existing,aggregate,bufferAgg,outDir);
    save(fullfile(outDir,'method_buffer_results.mat'), ...
        'existing','aggregate','buffer','bufferAgg','design','methods','cfg');
else
    aggregate=table(); buffer=table(); bufferAgg=table();
end
results=struct('raw',existing,'aggregate',aggregate,'buffer',buffer, ...
    'bufferAggregate',bufferAgg,'outputDirectory',outDir);
assignin('base','methodBufferResults',results);
end

function row=runOne(spec,method,b,cfg)
t=tic; success=false; reason=""; required=NaN; metric=[];
try
    if method=="Shelf"
        % Physical online shelf baseline: one visible arrival at a time.
        % Using the same placement kernel keeps geometry, support and COM
        % constraints identical; only the decision horizon is changed.
        plan=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,1, ...
            cfg.palletMargin,cfg.boxGap,false);
    else
        plan=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,3, ...
            cfg.palletMargin,cfg.boxGap,false);
    end
    planningTime=toc(t);
    wide=cfg; wide.bufferCapacity=spec.N; wide.bufferGrid=[ceil(spec.N/2) 2];
    execution=buildBufferExecutionPlan(plan,wide);
    required=peakBuffer(execution);
    metric=computeLayoutComparisonMetrics(method,plan,execution,cfg, ...
        planningTime,NaN);
    success=true;
catch ME
    planningTime=toc(t); reason=string(ME.message);
end
row=packRow(spec,method,success,reason,planningTime,required,metric,cfg.bufferCapacity);
end

function row=fromFormalRow(src)
row=table(src.CaseID,src.Study,src.N,src.K,src.InstanceClass,src.Seed, ...
    "Superbox",logical(src.PlanningSucceeded),string(src.FailureReason), ...
    src.MeanPlanningTime_s,src.Layers,src.StackHeight_m, ...
    src.PalletVolumeUtilization,src.MeanLayerAreaUtilization, ...
    src.FirstLayerAreaUtilization,src.TopLayerAreaUtilization, ...
    src.MinSupportRatio,src.MinCOMSupportMargin_m,src.UnstableBoxCount, ...
    src.COMOffset_m,src.MinStaticTippingAngle_deg, ...
    src.RequiredBufferCapacity,logical(src.FeasibleWith8SlotBuffer), ...
    src.ToBufferActions,src.FromBufferActions,src.RobotMoves, ...
    src.TransferDistance_m,src.PayloadLiftEnergyProxy_J, ...
    'VariableNames',resultNames());
end

function row=packRow(spec,method,success,reason,t,required,metric,capacity)
vals=cell(1,numel(resultNames()));
vals(1:9)={spec.CaseID,spec.Study,spec.N,spec.K,spec.InstanceClass, ...
    spec.Seed,method,success,reason};
vals{10}=t;
for j=11:numel(vals), vals{j}=NaN; end
vals{22}=required; vals{23}=success && required<=capacity;
if success
    source={'Layers','StackHeight_m','PalletVolumeUtilization', ...
        'MeanLayerAreaUtilization','FirstLayerAreaUtilization', ...
        'TopLayerAreaUtilization','MinSupportRatio','MinCOMSupportMargin_m', ...
        'UnstableBoxCount','COMOffset_m','MinStaticTippingAngle_deg'};
    for k=1:numel(source)
        vals{10+k}=metric.(source{k});
    end
    vals{22}=required; vals{23}=required<=capacity;
    vals{24}=metric.ToBufferActions; vals{25}=metric.FromBufferActions;
    vals{26}=metric.RobotMoves; vals{27}=metric.TransferDistance_m;
    vals{28}=metric.PayloadLiftEnergyProxy_J;
end
row=cell2table(vals,'VariableNames',resultNames());
end

function names=resultNames()
names={'CaseID','Study','N','K','InstanceClass','Seed','Method', ...
    'PlanningSucceeded','FailureReason','PlanningTime_s','Layers', ...
    'StackHeight_m','PalletVolumeUtilization','MeanLayerAreaUtilization', ...
    'FirstLayerAreaUtilization','TopLayerAreaUtilization','MinSupportRatio', ...
    'MinCOMSupportMargin_m','UnstableBoxCount','COMOffset_m', ...
    'MinStaticTippingAngle_deg','RequiredBufferCapacity', ...
    'FeasibleWith8SlotBuffer','ToBufferActions','FromBufferActions', ...
    'RobotMoves','TransferDistance_m','PayloadLiftEnergyProxy_J'};
end

function T=emptyResultTable()
names=resultNames();
types=["string","string","double","double","string","double","string", ...
    "logical","string",repmat("double",1,13),"logical",repmat("double",1,5)];
T=table('Size',[0 numel(names)],'VariableTypes',cellstr(types), ...
    'VariableNames',names);
end

function A=aggregateComparison(T)
keys=unique(T(:,{'Study','N','K','InstanceClass','Method'}),'rows','stable');
A=table();
for i=1:height(keys)
    hit=T.Study==keys.Study(i)&T.N==keys.N(i)&T.K==keys.K(i)& ...
        T.InstanceClass==keys.InstanceClass(i)&T.Method==keys.Method(i);
    X=T(hit,:);
    row=[keys(i,:) table(height(X),mean(X.PlanningSucceeded), ...
        mean(X.PlanningTime_s,'omitnan'),std(X.PlanningTime_s,'omitnan'), ...
        mean(X.PalletVolumeUtilization,'omitnan'), ...
        std(X.PalletVolumeUtilization,'omitnan'),mean(X.Layers,'omitnan'), ...
        mean(X.MeanLayerAreaUtilization,'omitnan'), ...
        min(X.MinSupportRatio,[],'omitnan'), ...
        min(X.MinCOMSupportMargin_m,[],'omitnan'), ...
        sum(X.UnstableBoxCount,'omitnan'), ...
        mean(X.RequiredBufferCapacity,'omitnan'), ...
        max(X.RequiredBufferCapacity,[],'omitnan'), ...
        mean(X.FeasibleWith8SlotBuffer), ...
        'VariableNames',{'Instances','SuccessRate','MeanPlanningTime_s', ...
        'StdPlanningTime_s','MeanVolumeUtilization','StdVolumeUtilization', ...
        'MeanLayers','MeanLayerAreaUtilization','MinSupportRatio', ...
        'MinCOMSupportMargin_m','UnstableBoxCount','MeanRequiredBuffer', ...
        'MaxRequiredBuffer','EightSlotFeasibilityRate'})];
    A=[A;row]; %#ok<AGROW>
end
end

function B=buildBufferSensitivity(T)
capacities=[0 2 4 6 8 10 12 16 20 24 28 32]';
B=table();
for i=1:height(T)
    n=numel(capacities);
    x=table(repmat(T.CaseID(i),n,1),repmat(T.Study(i),n,1), ...
        repmat(T.N(i),n,1),repmat(T.K(i),n,1), ...
        repmat(T.InstanceClass(i),n,1),repmat(T.Method(i),n,1), ...
        capacities,repmat(T.RequiredBufferCapacity(i),n,1), ...
        T.PlanningSucceeded(i)&capacities>=T.RequiredBufferCapacity(i), ...
        repmat(T.ToBufferActions(i),n,1),repmat(T.FromBufferActions(i),n,1), ...
        'VariableNames',{'CaseID','Study','N','K','InstanceClass','Method', ...
        'BufferCapacity','RequiredBufferCapacity','ExecutionFeasible', ...
        'ToBufferActions','FromBufferActions'});
    B=[B;x]; %#ok<AGROW>
end
end

function A=aggregateBuffer(B)
keys=unique(B(:,{'Method','BufferCapacity'}),'rows','stable'); A=table();
for i=1:height(keys)
    hit=B.Method==keys.Method(i)&B.BufferCapacity==keys.BufferCapacity(i);
    X=B(hit,:);
    row=[keys(i,:) table(height(X),mean(X.ExecutionFeasible), ...
        mean(X.RequiredBufferCapacity,'omitnan'), ...
        mean(X.ToBufferActions+X.FromBufferActions,'omitnan'), ...
        'VariableNames',{'Instances','ExecutionFeasibilityRate', ...
        'MeanRequiredBuffer','MeanExtraBufferMoves'})];
    A=[A;row]; %#ok<AGROW>
end
end

function makePlots(raw,aggregate,bufferAgg,outDir)
methods=["Shelf","RollingHorizon3","Superbox"];
colors=[0.18 .39 .63; .18 .55 .52; .78 .48 .10];

S=aggregate(aggregate.Study=="Scale",:);
f=figure('Visible','off','Color','w','Position',[80 80 1500 850]);
tl=tiledlayout(f,2,2,'TileSpacing','compact');
metrics={'MeanVolumeUtilization','MeanPlanningTime_s', ...
    'MeanRequiredBuffer','EightSlotFeasibilityRate'};
titles={'Pallet volume utilization','Planning time (s)', ...
    'Required buffer capacity','Feasible with 8-slot buffer'};
for q=1:4
    ax=nexttile(tl); hold(ax,'on');
    for m=1:numel(methods)
        X=sortrows(S(S.Method==methods(m),:),'N');
        plot(ax,X.N,X.(metrics{q}),'-o','LineWidth',1.8, ...
            'Color',colors(m,:),'DisplayName',methods(m));
    end
    grid(ax,'on'); title(ax,titles{q}); xlabel(ax,'Number of boxes, N');
    if q==1||q==4, ylim(ax,[0 max(1,max(S.(metrics{q}))*1.1)]); end
end
legend(nexttile(tl,1), 'Location','best');
title(tl,'Fair method comparison on identical scale instances');
exportgraphics(f,fullfile(outDir,'method_comparison_scale.png'),'Resolution',180);
close(f);

f=figure('Visible','off','Color','w','Position',[80 80 1350 560]);
tl=tiledlayout(f,1,2,'TileSpacing','compact');
rawMetrics={'PalletVolumeUtilization','RequiredBufferCapacity'};
rawTitles={'Pallet volume utilization','Required buffer capacity'};
for q=1:2
    ax=nexttile(tl); hold(ax,'on');
    metric=rawMetrics{q};
    vals=zeros(numel(methods),1);
    for m=1:numel(methods)
        vals(m)=mean(raw.(metric)(raw.Method==methods(m)),'omitnan');
    end
    bar(ax,vals,'FaceColor','flat');
    ax.Children.CData=colors; ax.XTick=1:3; ax.XTickLabel=methods;
    grid(ax,'on'); title(ax,rawTitles{q}); xtickangle(ax,15);
end
title(tl,'Overall comparison across 190 matched instances');
exportgraphics(f,fullfile(outDir,'method_comparison_overall.png'),'Resolution',180);
close(f);

f=figure('Visible','off','Color','w','Position',[80 80 1000 620]);
ax=axes(f); hold(ax,'on');
for m=1:numel(methods)
    X=sortrows(bufferAgg(bufferAgg.Method==methods(m),:),'BufferCapacity');
    plot(ax,X.BufferCapacity,100*X.ExecutionFeasibilityRate,'-o', ...
        'LineWidth',2,'Color',colors(m,:),'DisplayName',methods(m));
end
grid(ax,'on'); xlabel(ax,'Buffer capacity (slots)');
ylabel(ax,'Execution feasibility (%)'); ylim(ax,[0 105]);
title(ax,'Buffer-capacity sensitivity on 190 matched instances');
legend(ax,'Location','southeast');
exportgraphics(f,fullfile(outDir,'buffer_capacity_sensitivity.png'),'Resolution',180);
close(f);
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
