function [traj, report] = planPalletizingTrajectory(layoutPlan, palletDim, palletLoc, opt)
% planPalletizingTrajectory Build robot joint trajectories from a layout plan.
%
% INPUTS
%   layoutPlan  Struct array produced by buildLayoutPlanGA or rolling layout.
%   palletDim   [L W H] pallet size in metres.
%   palletLoc   [x y z] pallet centre in world coordinates.
%   opt         Optional settings. Important fields:
%                 pickPosition, approachHeight, samplesPerBox,
%                 useRRTFallback, visualize
%
% OUTPUTS
%   traj.q      6-by-N joint trajectory, ready for Simulink From Workspace.
%   traj.qd     6-by-N joint velocities.
%   traj.qdd    6-by-N joint accelerations.
%   traj.time   1-by-N time vector.
%   report      Per-box IK/collision status for verification.

if nargin < 4 || isempty(opt)
    opt = struct();
end
opt = fillDefaultTrajectoryOptions(opt);

robot = exampleHelperSetUpCobot();
eeName = selectEndEffector(robot, opt.endEffector);

ik = inverseKinematics("RigidBodyTree", robot);
ikWeights = [0.2 0.2 0.2 1 1 1];

qCurrent = homeConfiguration(robot);
if isstruct(qCurrent)
    qCurrent = [qCurrent.JointPosition];
end
qCurrent = qCurrent(:).';

placedBoxes = struct('ID',{},'L',{},'W',{},'H',{},'pos',{},'yaw',{});
qAll = [];
qdAll = [];
qddAll = [];

tasks = buildTrajectoryTasks(layoutPlan, opt);
report = repmat(emptyReport(), numel(tasks), 1);

for k = 1:numel(tasks)
    idx = tasks(k).boxID;
    box = layoutPlan(idx);

    [poses, poseNames] = buildPickPlacePoses(box, opt, ...
        tasks(k).fromPos, tasks(k).toPos, tasks(k).targetYaw);
    qWaypoints = zeros(numel(poseNames)+1, numel(qCurrent));
    qWaypoints(1,:) = qCurrent;

    ikOK = true;
    ikStatus = strings(numel(poseNames), 1);
    qGuess = qCurrent;

    for p = 1:numel(poseNames)
        [qSol, solInfo] = ik(eeName, poses(:,:,p), ikWeights, qGuess);
        qWaypoints(p+1,:) = qSol;
        qGuess = qSol;

        ikStatus(p) = string(solInfo.Status);
        if solInfo.ExitFlag <= 0
            ikOK = false;
        end
    end

    [qBox, qdBox, qddBox] = trapveltraj(qWaypoints.', opt.samplesPerBox);

    env = {};
    collision = emptyCollision();
    usedRRT = false;

    if opt.enableCollisionCheck
        env = buildStaticEnvironment(palletDim, palletLoc, placedBoxes, opt);
        collision = scanTrajectoryCollision(robot, qBox, env, opt);

        if collision.hasCollision && opt.useRRTFallback
            [qBox, qdBox, qddBox, usedRRT] = replanWithRRT(robot, qWaypoints, env, opt);
            collision = scanTrajectoryCollision(robot, qBox, env, opt);
        end
    end

    if opt.visualize
        visualizeTrajectory(robot, qBox, env, sprintf("Box %d trajectory", box.ID));
    end

    qAll = [qAll qBox]; %#ok<AGROW>
    qdAll = [qdAll qdBox]; %#ok<AGROW>
    qddAll = [qddAll qddBox]; %#ok<AGROW>

    report(k).ID = box.ID;
    report(k).ikOK = ikOK;
    report(k).ikStatus = ikStatus;
    report(k).collisionFree = ~collision.hasCollision;
    report(k).collisionSamples = collision.samples;
    report(k).usedRRTFallback = usedRRT;
    report(k).numSamples = size(qBox, 2);
    report(k).targetPosition = box.pos;
    report(k).targetPosition = tasks(k).toPos;
    report(k).targetYaw = tasks(k).targetYaw;
    report(k).action = string(tasks(k).action);

    if collision.hasCollision
        msg = sprintf("Collision remains in trajectory for layout box %d after planning.", box.ID);
        if opt.failOnCollision
            error("planPalletizingTrajectory:CollisionRemaining", "%s", msg);
        else
            warning("planPalletizingTrajectory:CollisionRemaining", ...
                "%s Continuing and recording collisionSamples in trajectoryReport.", msg);
        end
    end

    qCurrent = qBox(:,end).';
    if contains(string(tasks(k).action), "_to_pallet")
        placedBoxes(end+1) = struct( ... %#ok<AGROW>
            'ID', box.ID, 'L', box.L, 'W', box.W, 'H', box.H, ...
            'pos', box.pos, 'yaw', box.yaw);
    end
end

traj.q = qAll;
traj.qd = qdAll;
traj.qdd = qddAll;
traj.time = (0:size(qAll,2)-1) * opt.sampleTime;
traj.sampleTime = opt.sampleTime;
traj.endEffector = eeName;
traj.pickPosition = opt.pickPosition;

end

function opt = fillDefaultTrajectoryOptions(opt)
try
    cfg = palletTaskConfig();
catch
    cfg = struct();
end
if ~isfield(opt,'pickPosition')
    if isfield(cfg,'pickPosition')
        opt.pickPosition = cfg.pickPosition;
    else
        opt.pickPosition = [-0.65 0.65 0.82];
    end
end
if ~isfield(opt,'pickYaw'), opt.pickYaw = 0; end
if ~isfield(opt,'approachHeight')
    if isfield(cfg,'approachHeight')
        opt.approachHeight = cfg.approachHeight;
    else
        opt.approachHeight = 0.30;
    end
end
if ~isfield(opt,'topClearance')
    if isfield(cfg,'topClearance')
        opt.topClearance = cfg.topClearance;
    else
        opt.topClearance = 0.08;
    end
end
if ~isfield(opt,'samplesPerBox'), opt.samplesPerBox = 180; end
if ~isfield(opt,'sampleTime'), opt.sampleTime = 0.05; end
if ~isfield(opt,'endEffector'), opt.endEffector = "4cup_assembly"; end
if ~isfield(opt,'useRRTFallback'), opt.useRRTFallback = true; end
if ~isfield(opt,'enableCollisionCheck'), opt.enableCollisionCheck = false; end
if ~isfield(opt,'rrtInterpPerSegment'), opt.rrtInterpPerSegment = 35; end
if ~isfield(opt,'visualize'), opt.visualize = false; end
if ~isfield(opt,'collisionStride'), opt.collisionStride = 5; end
if ~isfield(opt,'ignoreSelfCollision'), opt.ignoreSelfCollision = false; end
if ~isfield(opt,'includePalletCollision'), opt.includePalletCollision = false; end
if ~isfield(opt,'failOnCollision'), opt.failOnCollision = false; end
if ~isfield(opt,'palletInflation'), opt.palletInflation = [0.02 0.02 0.02]; end
if ~isfield(opt,'boxInflation'), opt.boxInflation = [0.01 0.01 0.01]; end
end

function eeName = selectEndEffector(robot, preferredName)
bodyNames = string(robot.BodyNames);
preferredName = string(preferredName);
if any(bodyNames == preferredName)
    eeName = char(preferredName);
else
    eeName = char(bodyNames(end));
end
end

function [poses, names] = buildPickPlacePoses(box, opt, pickPosition, targetPosition, targetYaw)
names = ["prePick","pick","lift","prePlace","place","retreat"];

pickTop = pickPosition;
placeTop = targetPosition;
placeTop(3) = targetPosition(3) + box.H/2 + opt.topClearance;

pickPose = topDownPose(pickTop, opt.pickYaw);
prePickPose = pickPose;
prePickPose(3,4) = prePickPose(3,4) + opt.approachHeight;

liftPose = prePickPose;

placePose = topDownPose(placeTop, targetYaw);
prePlacePose = placePose;
prePlacePose(3,4) = prePlacePose(3,4) + opt.approachHeight;

retreatPose = prePlacePose;

poses = zeros(4,4,numel(names));
poses(:,:,1) = prePickPose;
poses(:,:,2) = pickPose;
poses(:,:,3) = liftPose;
poses(:,:,4) = prePlacePose;
poses(:,:,5) = placePose;
poses(:,:,6) = retreatPose;
end

function tasks = buildTrajectoryTasks(layoutPlan,opt)
if isfield(opt,'executionPlan') && ~isempty(opt.executionPlan)
    tasks=opt.executionPlan;
    return;
end
ids=find([layoutPlan.isPlaced]);
tasks=repmat(struct('action','conveyor_to_pallet','boxID',0,'sourceID',0, ...
    'bufferSlot',0,'fromPos',[0 0 0],'toPos',[0 0 0],'targetYaw',0),numel(ids),1);
for k=1:numel(ids)
    id=ids(k); tasks(k).boxID=id; tasks(k).sourceID=layoutPlan(id).sourceID;
    tasks(k).fromPos=opt.pickPosition; tasks(k).toPos=layoutPlan(id).pos;
    tasks(k).targetYaw=layoutPlan(id).yaw;
end
end

function pose = topDownPose(position, yaw)
% ZYX Euler: yaw around world Z, then pitch the tool downward.
pose = trvec2tform(position) * eul2tform([yaw pi 0], "ZYX");
end

function env = buildStaticEnvironment(palletDim, palletLoc, placedBoxes, opt)
env = cell(1, 0);

if opt.includePalletCollision
    palletBox = collisionBox( ...
        palletDim(1) + opt.palletInflation(1), ...
        palletDim(2) + opt.palletInflation(2), ...
        palletDim(3) + opt.palletInflation(3));
    palletBox.Pose = trvec2tform(palletLoc);
    env{end+1} = palletBox;
end

for i = 1:numel(placedBoxes)
    b = placedBoxes(i);
    cBox = collisionBox( ...
        b.L + opt.boxInflation(1), ...
        b.W + opt.boxInflation(2), ...
        b.H + opt.boxInflation(3));
    cBox.Pose = trvec2tform(b.pos) * axang2tform([0 0 1 b.yaw]);
    env{end+1} = cBox; %#ok<AGROW>
end
end

function collision = scanTrajectoryCollision(robot, qTraj, env, opt)
collision.hasCollision = false;
collision.samples = [];
ignoreSelfCollision = boolToOnOff(opt.ignoreSelfCollision);

if isempty(env)
    return;
end

for s = 1:opt.collisionStride:size(qTraj,2)
    q = qTraj(:,s).';
    try
        inCollision = checkCollision(robot, q, env, ...
            "IgnoreSelfCollision", ignoreSelfCollision, ...
            "SkippedSelfCollisions", "Parent");
    catch
        inCollision = checkCollision(robot, q, env);
    end

    if any(inCollision)
        collision.hasCollision = true;
        collision.samples(end+1) = s; %#ok<AGROW>
    end
end
end

function [qOut, qdOut, qddOut, usedRRT] = replanWithRRT(robot, qWaypoints, env, opt)
usedRRT = false;
qPath = qWaypoints(1,:);

planner = manipulatorRRT(robot, env);
planner.IgnoreSelfCollision = opt.ignoreSelfCollision;
planner.SkippedSelfCollisions = "Parent";
planner.MaxConnectionDistance = 0.25;
planner.ValidationDistance = 0.04;

prevSeed = rng(0);
for i = 2:size(qWaypoints,1)
    qStart = qPath(end,:);
    qGoal = qWaypoints(i,:);
    segment = [];

    try
        planOut = plan(planner, qStart, qGoal);
        if ~isempty(planOut)
            planOut = shorten(planner, planOut, 5);
            segment = interpolate(planner, planOut, opt.rrtInterpPerSegment);
        end
    catch
        segment = [];
    end

    if isempty(segment)
        segment = [qStart; qGoal];
    else
        usedRRT = true;
    end

    qPath = [qPath; segment(2:end,:)]; %#ok<AGROW>
end
rng(prevSeed);

numSamples = max(opt.samplesPerBox, size(qPath,1));
[qOut, qdOut, qddOut] = trapveltraj(qPath.', numSamples);
end

function value = boolToOnOff(tf)
if tf
    value = "on";
else
    value = "off";
end
end

function collision = emptyCollision()
collision.hasCollision = false;
collision.samples = [];
end

function visualizeTrajectory(robot, qTraj, env, figTitle)
figure("Name", figTitle);
ax = axes;
hold(ax, "on");
show(robot, qTraj(:,1).', "Parent", ax, "PreservePlot", false);
for i = 1:numel(env)
    show(env{i}, "Parent", ax);
end
axis(ax, "equal");
view(ax, 135, 25);
grid(ax, "on");
title(ax, figTitle);

for s = 1:5:size(qTraj,2)
    show(robot, qTraj(:,s).', "Parent", ax, "PreservePlot", false);
    drawnow;
end
end

function r = emptyReport()
r = struct( ...
    'ID', 0, ...
    'ikOK', false, ...
    'ikStatus', strings(0,1), ...
    'collisionFree', false, ...
    'collisionSamples', [], ...
    'usedRRTFallback', false, ...
    'numSamples', 0, ...
    'targetPosition', [0 0 0], ...
    'targetYaw', 0, ...
    'action', "");
end
