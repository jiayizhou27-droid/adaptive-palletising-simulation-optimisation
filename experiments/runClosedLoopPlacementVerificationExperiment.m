function results = runClosedLoopPlacementVerificationExperiment()
%RUNCLOSEDLOOPPLACEMENTVERIFICATIONEXPERIMENT Software vision fault study.
% Injects translation, rotation, missing-box, and grasp faults after a
% nominal placement, then tests detection, pause, buffer recovery, and
% support-aware replanning with already accepted cartons fixed.
projectDir=fileparts(mfilename('fullpath'));
outDir=fullfile(projectDir,'closed_loop_placement_verification');
if ~isfolder(outDir), mkdir(outDir); end
visionCfg=closedLoopVisionConfig();
palletCfg=palletTaskConfig();

layoutPlan=loadSuperboxPlan(projectDir,palletCfg);
[~,placementOrder]=sort([layoutPlan.placementRank]);
faultOrderIndex=min(12,numel(placementOrder));
faultBoxID=placementOrder(faultOrderIndex);
acceptedIDs=placementOrder(1:faultOrderIndex-1);
targetPos=layoutPlan(faultBoxID).pos;
targetYaw=layoutPlan(faultBoxID).yaw;

scenarios=buildScenarios();
report=table();
recoveryOrders=cell(height(scenarios),1);
for i=1:height(scenarios)
    measurement=struct( ...
        'position',targetPos+scenarios.TranslationVector_m(i,:), ...
        'yawDeg',rad2deg(targetYaw)+scenarios.YawError_deg(i), ...
        'actorVisible',~scenarios.MissingBox(i), ...
        'graspSucceeded',~scenarios.GraspFailure(i));
    verdict=verifyClosedLoopPlacement(targetPos,targetYaw,measurement,visionCfg);

    if verdict.detected
        recovery=replanRemainingBySupport(layoutPlan,acceptedIDs, ...
            faultBoxID,palletCfg);
        recoveryOrders{i}=recovery.order;
        % A missing carton has no physical object to buffer. Its replacement
        % is requested and inserted at the head of the recovery schedule.
        sentToBuffer=recovery.succeeded && ~scenarios.MissingBox(i);
        replanSucceeded=recovery.succeeded;
        resumed=recovery.succeeded;
    else
        recovery=struct('succeeded',false,'order',[],'bufferSlot',NaN);
        recoveryOrders{i}=[];
        sentToBuffer=false;
        replanSucceeded=false;
        resumed=true;
    end

    expectedFault=scenarios.ExpectedFault(i);
    correctlyClassified=(expectedFault && verdict.detected) || ...
        (~expectedFault && ~verdict.detected);
    row=table(scenarios.Scenario(i),expectedFault, ...
        verdict.translationError_m,verdict.yawError_deg, ...
        verdict.detected,string(verdict.reason),correctlyClassified, ...
        verdict.pauseRequested,sentToBuffer,replanSucceeded,resumed, ...
        recovery.bufferSlot,faultBoxID, ...
        'VariableNames',{'Scenario','ExpectedFault','TranslationError_m', ...
        'YawError_deg','Detected','DetectionReason','CorrectlyClassified', ...
        'PauseRequested','SentToBuffer','ReplanSucceeded','ExecutionResumed', ...
        'RecoveryBufferSlot','FaultBoxID'});
    report=[report;row]; %#ok<AGROW>
end

detectionRate=sum(report.Detected & report.ExpectedFault)/sum(report.ExpectedFault);
falsePositiveRate=sum(report.Detected & ~report.ExpectedFault)/sum(~report.ExpectedFault);
recoveryRate=sum(report.ReplanSucceeded & report.ExpectedFault)/sum(report.ExpectedFault);
summary=table(height(report),sum(report.ExpectedFault),detectionRate, ...
    falsePositiveRate,recoveryRate,visionCfg.translationTolerance_m, ...
    visionCfg.yawTolerance_deg, ...
    'VariableNames',{'TestCases','FaultCases','DetectionRate', ...
    'FalsePositiveRate','RecoverySuccessRate','TranslationTolerance_m', ...
    'YawTolerance_deg'});

writetable(report,fullfile(outDir,'closed_loop_fault_test_results.csv'));
writetable(summary,fullfile(outDir,'closed_loop_fault_test_summary.csv'));
save(fullfile(outDir,'closed_loop_fault_test_results.mat'), ...
    'report','summary','recoveryOrders','layoutPlan','scenarios', ...
    'visionCfg','palletCfg','faultBoxID','acceptedIDs');
plotResults(report,fullfile(outDir,'closed_loop_fault_test_results.png'));
disp(report);
disp(summary);
results=struct('report',report,'summary',summary, ...
    'outputDirectory',outDir,'faultBoxID',faultBoxID);
assignin('base','closedLoopPlacementVerificationResults',results);
end

function plan=loadSuperboxPlan(projectDir,cfg)
strictFile=fullfile(projectDir,'arrival_pattern_experiment40', ...
    'arrival_pattern_experiment40.mat');
if isfile(strictFile)
    S=load(strictFile,'plansByPattern','patterns','methods');
    scenarioIdx=find(string(S.patterns)=="LargeBoxFirst",1);
    methodIdx=find(string(S.methods)=="Superbox40",1);
    if ~isempty(scenarioIdx) && ~isempty(methodIdx)
        candidate=S.plansByPattern{scenarioIdx,methodIdx};
        if min([candidate.supportRatio])>=0.95-1e-9
            plan=candidate;
            return
        end
    end
end
comparisonFile=fullfile(projectDir,'method_comparison_results', ...
    'method_comparison_40.mat');
if isfile(comparisonFile)
    S=load(comparisonFile,'plans','methods');
    idx=find(string(S.methods)=="Superbox40",1);
    if ~isempty(idx)
        plan=S.plans{idx};
        return
    end
end
T=readtable(fullfile(projectDir,'boxes.xlsx'));
T=T(1:40,:);
b.length=T.L; b.width=T.W; b.height=T.H; b.weight=T.Mass;
b.type=string(T.Type); b.sourceID=(1:40)';
plan=buildLayoutPlanBatch3(b,cfg.palletDim,cfg.palletLoc,40, ...
    cfg.palletMargin,cfg.boxGap,false);
end

function scenarios=buildScenarios()
names="Nominal";
translation=[0 0 0];
yaw=0;
missing=false;
grasp=false;
expected=false;
for magnitude=[0.002 0.005 0.010]
    names(end+1)=sprintf('Translation +%d mm',round(1000*magnitude)); %#ok<AGROW>
    translation(end+1,:)=[magnitude 0 0]; yaw(end+1)=0; %#ok<AGROW>
    missing(end+1)=false; grasp(end+1)=false; expected(end+1)=true; %#ok<AGROW>
    names(end+1)=sprintf('Translation -%d mm',round(1000*magnitude)); %#ok<AGROW>
    translation(end+1,:)=[-magnitude 0 0]; yaw(end+1)=0; %#ok<AGROW>
    missing(end+1)=false; grasp(end+1)=false; expected(end+1)=true; %#ok<AGROW>
end
for magnitude=[1 3 5]
    names(end+1)=sprintf('Rotation +%d deg',magnitude); %#ok<AGROW>
    translation(end+1,:)=[0 0 0]; yaw(end+1)=magnitude; %#ok<AGROW>
    missing(end+1)=false; grasp(end+1)=false; expected(end+1)=true; %#ok<AGROW>
    names(end+1)=sprintf('Rotation -%d deg',magnitude); %#ok<AGROW>
    translation(end+1,:)=[0 0 0]; yaw(end+1)=-magnitude; %#ok<AGROW>
    missing(end+1)=false; grasp(end+1)=false; expected(end+1)=true; %#ok<AGROW>
end
names(end+1)="Missing box";
translation(end+1,:)=[0 0 0]; yaw(end+1)=0;
missing(end+1)=true; grasp(end+1)=false; expected(end+1)=true;
names(end+1)="Grasp failure";
translation(end+1,:)=[0 0 0]; yaw(end+1)=0;
missing(end+1)=false; grasp(end+1)=true; expected(end+1)=true;
scenarios=table(names',translation,yaw',missing',grasp',expected', ...
    'VariableNames',{'Scenario','TranslationVector_m','YawError_deg', ...
    'MissingBox','GraspFailure','ExpectedFault'});
end

function plotResults(report,outFile)
f=figure('Visible','off','Color','w','Position',[80 80 1450 700]);
tl=tiledlayout(f,2,1,'TileSpacing','compact');
ax=nexttile(tl);
bar(ax,1000*report.TranslationError_m);
ylabel(ax,'Translation error (mm)');
grid(ax,'on');
xticks(ax,1:height(report)); xticklabels(ax,report.Scenario);
xtickangle(ax,35);
ax=nexttile(tl);
bar(ax,[report.Detected report.PauseRequested report.SentToBuffer ...
    report.ReplanSucceeded]);
ylim(ax,[0 1.15]); grid(ax,'on');
ylabel(ax,'Controller response');
xticks(ax,1:height(report)); xticklabels(ax,report.Scenario);
xtickangle(ax,35);
legend(ax,{'Detected','Pause','Buffer','Replan'},'Location','northwest');
exportgraphics(f,outFile,'Resolution',180);
close(f);
end
