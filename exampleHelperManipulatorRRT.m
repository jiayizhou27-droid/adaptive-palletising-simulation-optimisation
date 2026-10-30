function [fixedDimInterpPlan,numComputedSamples] = exampleHelperManipulatorRRT(graspState, startConfig, goalConfig, ID, boxPosition, boxDim, maxNumOfBoxes)
%EXAMPLEHELPERMANIPULATORRRT Cartesian lift-transfer planner for Simulink.
%
% The Simulink block expects a fixed 6-by-200 joint path. Build that path
% from end-effector waypoints instead of adding arbitrary offsets to joint
% angles; this keeps the carried box moving up, across, and down in world
% space instead of drawing large arcs in the air.

NUMDOF = 6;
NUMPATHSAMPLES = 200;

q0 = reshape(double(startConfig), 1, NUMDOF);
q1 = reshape(double(goalConfig), 1, NUMDOF);
q1 = unwrapNear(q1, q0);

try
    fixedDimInterpPlan = buildCartesianLiftTransfer( ...
        q0, q1, graspState, ID, boxPosition, boxDim, maxNumOfBoxes, NUMPATHSAMPLES);
catch
    fixedDimInterpPlan = buildJointFallback(q0, q1, NUMPATHSAMPLES);
end

numComputedSamples = NUMPATHSAMPLES;
end

function qPath = buildCartesianLiftTransfer(q0, q1, graspState, ID, boxPosition, boxDim, maxNumOfBoxes, numSamples)
unused = double(ID) + double(maxNumOfBoxes);
if unused ~= unused
    q0 = q0;
end

[robot, ik, eeName] = getPlannerObjects();

startT = getTransform(robot, q0, eeName);
goalT = getTransform(robot, q1, eeName);

transferZ = computeTransferHeight(startT, goalT, graspState, boxPosition, boxDim);

startHighT = startT;
startHighT(3,4) = transferZ;
goalHighT = goalT;
goalHighT(3,4) = transferZ;

liftSamples = 45;
moveSamples = 110;
dropSamples = numSamples - liftSamples - moveSamples;

poses = cat(3, ...
    segmentPoses(startT, startHighT, liftSamples, true), ...
    segmentPoses(startHighT, goalHighT, moveSamples, false), ...
    segmentPoses(goalHighT, goalT, dropSamples, false));

qPath = zeros(6, numSamples);
qGuess = q0;
weights = [0.25 0.25 0.25 1 1 1];

for k = 1:numSamples
    if k == 1
        qPath(:,k) = q0(:);
        qGuess = q0;
        continue;
    end
    [qSol, solInfo] = ik(eeName, poses(:,:,k), weights, qGuess);
    if solInfo.ExitFlag <= 0
        error("exampleHelperManipulatorRRT:IKFailed", ...
            "IK failed while building Cartesian transfer path.");
    end
    qSol = unwrapNear(qSol, qGuess);
    if max(abs(qSol - qGuess)) > 0.45
        error("exampleHelperManipulatorRRT:IKJump", ...
            "IK changed branches while building Cartesian transfer path.");
    end
    qPath(:,k) = qSol(:);
    qGuess = qSol;
end

qPath(:,1) = q0(:);
qPath(:,end) = unwrapNear(q1, qPath(:,end-1)).';
end

function poses = segmentPoses(t0, t1, n, includeFirst)
if includeFirst
    s = linspace(0, 1, n);
else
    s = linspace(0, 1, n + 1);
    s = s(2:end);
end

poses = zeros(4, 4, numel(s));
try
    [poses, ~, ~] = transformtraj(t0, t1, [0 1], s);
catch
    for i = 1:numel(s)
        poses(:,:,i) = t0;
        poses(1:3,4,i) = (1-s(i))*t0(1:3,4) + s(i)*t1(1:3,4);
        if s(i) > 0.5
            poses(1:3,1:3,i) = t1(1:3,1:3);
        end
    end
end
end

function transferZ = computeTransferHeight(startT, goalT, graspState, boxPosition, boxDim)
try
    cfg = palletTaskConfig();
    clearance = cfg.transferClearance;
    minHeight = cfg.minTransferHeight;
catch
    clearance = 0.25;
    minHeight = 0.85;
end

boxPosition = double(boxPosition(:)).';
boxDim = double(boxDim(:)).';
if numel(boxDim) < 3
    boxDim = [boxDim repmat(boxDim(end), 1, 3-numel(boxDim))];
end

sourceTop = boxPosition(3) + 0.5 * boxDim(3);
if double(graspState) ~= 0
    carriedClearance = clearance;
else
    carriedClearance = 0.6 * clearance;
end

transferZ = max([ ...
    startT(3,4), ...
    goalT(3,4), ...
    sourceTop + carriedClearance, ...
    minHeight]);
end

function qPath = buildJointFallback(q0, q1, numSamples)
qPath = zeros(6, numSamples);
q1 = unwrapNear(q1, q0);
for k = 1:numSamples
    s = (k-1) / (numSamples-1);
    blend = 3*s*s - 2*s*s*s;
    qPath(:,k) = ((1-blend)*q0 + blend*q1).';
end
end

function q = unwrapNear(q, qRef)
q = double(q(:)).';
qRef = double(qRef(:)).';
for i = 1:numel(q)
    while q(i) - qRef(i) > pi
        q(i) = q(i) - 2*pi;
    end
    while q(i) - qRef(i) < -pi
        q(i) = q(i) + 2*pi;
    end
end
end

function [robot, ik, eeName] = getPlannerObjects()
persistent cachedRobot cachedIk cachedEeName
if isempty(cachedRobot)
    cachedRobot = exampleHelperSetUpCobot();
    cachedEeName = selectEndEffector(cachedRobot, "4cup_assembly");
    cachedIk = inverseKinematics("RigidBodyTree", cachedRobot);
    try
        cachedIk.SolverParameters.AllowRandomRestart = false;
    catch
    end
end

robot = cachedRobot;
ik = cachedIk;
eeName = cachedEeName;
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
