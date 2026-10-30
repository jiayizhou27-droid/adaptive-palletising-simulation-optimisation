function results = runThirtySimulinkValidation(runIDs,options)
%RUNTHIRTYSIMULINKVALIDATION Execute checkpointed full Simulink/Sim3D runs.
%
% Commands:
%   buildSimulinkValidationManifest
%   runThirtySimulinkValidation(1)       % smoke test
%   runThirtySimulinkValidation(1:3)     % paired three-method case
%   runThirtySimulinkValidation          % all unfinished runs
%
% StopTime remains Inf.  Each run is actively stopped after all expected
% actions have been logged or after the wall-clock timeout.

if nargin<2, options=struct(); end
if ~isfield(options,'pollSeconds'),options.pollSeconds=1;end
if ~isfield(options,'finalActionGraceSeconds'),options.finalActionGraceSeconds=12;end
if ~isfield(options,'maxWallTimePerAction'),options.maxWallTimePerAction=50;end
if ~isfield(options,'minimumTimeoutSeconds'),options.minimumTimeoutSeconds=300;end

projectDir=fileparts(mfilename('fullpath'));
outDir=fullfile(projectDir,'simulink_validation_30');
manifestFile=fullfile(outDir,'simulink_validation_manifest.mat');
if ~isfile(manifestFile),manifest=buildSimulinkValidationManifest();else,S=load(manifestFile);manifest=S.manifest;end
if nargin<1 || isempty(runIDs),runIDs=manifest.RunID';end
runIDs=unique(double(runIDs(:)'));
assert(all(ismember(runIDs,manifest.RunID)),'runIDs must be between 1 and %d.',height(manifest));

resultFile=fullfile(outDir,'simulink_validation_results.csv');
if isfile(resultFile),results=readtable(resultFile,'TextType','string');else,results=emptyResults();end
setupAdaptiveSimulinkCallbacks();
model='PalletizeBoxesUsingCobot';load_system(model);

for runID=runIDs
    if ~isempty(results)&&any(results.RunID==runID & results.Completed),fprintf('SKIP completed RunID %d\n',runID);continue;end
    row=manifest(manifest.RunID==runID,:);
    fprintf('\n=== Sim3D validation %d/30: %s | %s ===\n',runID,row.CaseID,row.Method);
    result=runOne(model,row,options);
    if ~isempty(results),results(results.RunID==runID,:)=[];end
    results=[results;result]; %#ok<AGROW>
    writetable(results,resultFile);
    save(fullfile(outDir,'simulink_validation_results.mat'),'results','manifest');
end
disp(results);
end

function result=runOne(model,row,opt)
oldInit=get_param(model,'InitFcn'); oldStop=get_param(model,'StopTime');
cleanup=onCleanup(@()restoreModel(model,oldInit,oldStop)); %#ok<NASGU>
planFile=char(row.PlanFile);escaped=strrep(planFile,'''','''''');
set_param(model,'InitFcn',sprintf('initializeSavedSimulinkValidationCase(''%s'');',escaped));
set_param(model,'StopTime','inf');
initializeSavedSimulinkValidationCase(planFile);
plan=evalin('base','layoutPlan');execution=evalin('base','executionPlan');
actions=string({execution.action});
% The intentionally invisible warm-up cycle is not a physical carton move
% and may not create an action-log row.  Completion is therefore judged
% against real conveyor/buffer/pallet actions only.
expected=sum(actions~="hidden_warmup");
timeout=max(opt.minimumTimeoutSeconds,opt.maxWallTimePerAction*expected);
direct=sum(actions=="conveyor_to_pallet");toBuffer=sum(actions=="conveyor_to_buffer");
fromBuffer=sum(actions=="buffer_to_pallet");peak=peakBuffer(execution);

wall=tic;status="Running";message="";simTime=0;
try
    set_param(model,'SimulationCommand','start');
    graceStarted=false;graceTimer=[];
    while toc(wall)<timeout
        pause(opt.pollSeconds);drawnow;
        statusNow=string(get_param(model,'SimulationStatus'));
        try,log=evalin('base','executionActionLog');catch,log=zeros(0,4);end
        if size(log,1)>=max(0,expected-1) && ~graceStarted
            graceStarted=true;graceTimer=tic;
        end
        if graceStarted && toc(graceTimer)>=opt.finalActionGraceSeconds,break;end
        if statusNow=="stopped",break;end
    end
    try
        rawSimTime=get_param(model,'SimulationTime');
        if isnumeric(rawSimTime),simTime=double(rawSimTime);else,simTime=str2double(rawSimTime);end
    catch,simTime=NaN;end
    if ~strcmp(get_param(model,'SimulationStatus'),'stopped')
        set_param(model,'SimulationCommand','stop');pause(1);drawnow;
    end
catch ME
    status="RuntimeError";message=string(ME.message);
    try,set_param(model,'SimulationCommand','stop');catch,end
end
wallSeconds=toc(wall);
try,log=evalin('base','executionActionLog');catch,log=zeros(0,4);end
logged=size(log,1);completed=logged>=expected;
if status=="Running"
    if completed,status="Completed";else,status="TimeoutOrIncomplete";end
end

[transferDistance,jointPath,cycleEstimate,energy,peakPower,peakSpeed,peakAccel,trajOK,trajMessage]=trajectoryMetrics(plan,execution);
cfg=palletTaskConfig();
layoutMetric=computeLayoutComparisonMetrics(row.Method,plan,execution,cfg,NaN,NaN);
result=table(row.RunID,row.CaseID,row.Study,row.N,row.K,row.Replicate,row.Method, ...
    row.PlanFile,row.RequiredBufferCapacity,row.PhysicalBufferCapacity,expected,logged, ...
    completed,status,message,wallSeconds,simTime,direct,toBuffer,fromBuffer,peak, ...
    transferDistance,jointPath,cycleEstimate,energy,peakPower,peakSpeed,peakAccel, ...
    trajOK,trajMessage,layoutMetric.Layers,layoutMetric.StackHeight_m, ...
    layoutMetric.PalletVolumeUtilization,layoutMetric.MinSupportRatio, ...
    layoutMetric.MinCOMSupportMargin_m,layoutMetric.UnstableBoxCount, ...
    layoutMetric.COMOffset_m,layoutMetric.MinStaticTippingAngle_deg, ...
    'VariableNames',{'RunID','CaseID','Study','N','K','Replicate','Method','PlanFile', ...
    'RequiredBufferCapacity','PhysicalBufferCapacity','ExpectedActions','LoggedActions', ...
    'Completed','Status','FailureMessage','WallTime_s','FinalSimulationTime_s', ...
    'DirectPlacements','BufferInActions','BufferOutActions','PeakBufferOccupancy', ...
    'TransferDistance_m','JointPathLength_rad','EstimatedCycleTime_s', ...
    'JointMechanicalEnergy_J','PeakMechanicalPower_W','PeakJointSpeed_rad_s', ...
    'PeakJointAcceleration_rad_s2','TrajectoryFeasible','TrajectoryMessage', ...
    'Layers','StackHeight_m','PalletVolumeUtilization','MinimumSupportRatio', ...
    'MinimumCOMMargin_m','UnstableBoxCount','COMOffset_m', ...
    'MinimumStaticTippingAngle_deg'});
end

function [distance,jointPath,duration,energy,peakPower,peakSpeed,peakAccel,ok,msg]=trajectoryMetrics(plan,execution)
distance=0;for i=2:numel(execution),distance=distance+norm(execution(i).toPos-execution(i).fromPos);end
jointPath=NaN;duration=NaN;energy=NaN;peakPower=NaN;peakSpeed=NaN;peakAccel=NaN;ok=false;msg="";
try
    cfg=palletTaskConfig();o=struct('pickPosition',cfg.pickPosition,'pickYaw',cfg.pickYaw, ...
        'approachHeight',cfg.approachHeight,'topClearance',cfg.topClearance, ...
        'samplesPerBox',80,'sampleTime',0.05,'useRRTFallback',true, ...
        'visualize',false,'executionPlan',execution);
    [traj,~]=planPalletizingTrajectory(plan,cfg.palletDim,cfg.palletLoc,o);
    duration=traj.time(end)-traj.time(1);jointPath=sum(abs(diff(traj.q,1,2)),'all');
    peakSpeed=max(abs(traj.qd),[],'all');peakAccel=max(abs(traj.qdd),[],'all');
    robot=exampleHelperSetUpCobot();t=double(traj.time(:));power=zeros(numel(t),1);
    for k=1:numel(t)
        tau=inverseDynamics(robot,traj.q(:,k)',traj.qd(:,k)',traj.qdd(:,k)');
        power(k)=sum(abs(tau(:).*traj.qd(:,k)));
    end
    energy=trapz(t,power);peakPower=max(power);ok=true;
catch ME,msg=string(ME.message);end
end

function p=peakBuffer(execution)
q=0;p=0;for i=1:numel(execution),if strcmp(execution(i).action,'conveyor_to_buffer'),q=q+1;elseif strcmp(execution(i).action,'buffer_to_pallet'),q=q-1;end;p=max(p,q);end
end

function restoreModel(model,initFcn,stopTime)
try,if ~strcmp(get_param(model,'SimulationStatus'),'stopped'),set_param(model,'SimulationCommand','stop');end;catch,end
try,set_param(model,'InitFcn',initFcn);set_param(model,'StopTime',stopTime);catch,end
end

function T=emptyResults()
T=table('Size',[0 38],'VariableTypes',{'double','string','string','double','double','double','string','string','double','double','double','double','logical','string','string','double','double','double','double','double','double','double','double','double','double','double','double','double','logical','string','double','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'RunID','CaseID','Study','N','K','Replicate','Method','PlanFile','RequiredBufferCapacity','PhysicalBufferCapacity','ExpectedActions','LoggedActions','Completed','Status','FailureMessage','WallTime_s','FinalSimulationTime_s','DirectPlacements','BufferInActions','BufferOutActions','PeakBufferOccupancy','TransferDistance_m','JointPathLength_rad','EstimatedCycleTime_s','JointMechanicalEnergy_J','PeakMechanicalPower_W','PeakJointSpeed_rad_s','PeakJointAcceleration_rad_s2','TrajectoryFeasible','TrajectoryMessage','Layers','StackHeight_m','PalletVolumeUtilization','MinimumSupportRatio','MinimumCOMMargin_m','UnstableBoxCount','COMOffset_m','MinimumStaticTippingAngle_deg'});
end
