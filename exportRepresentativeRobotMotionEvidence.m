function evidence = exportRepresentativeRobotMotionEvidence()
%EXPORTREPRESENTATIVEROBOTMOTIONEVIDENCE Export one representative transfer.
% Uses the currently prepared 40-carton trajectory when available. It does
% not modify the Simulink model or its execution logic.

projectDir = fileparts(mfilename("fullpath"));
outDir = fullfile(projectDir,"chapter6_evidence","representative_motion");
if ~exist(outDir,"dir"), mkdir(outDir); end

[traj,report] = loadTrajectoryEvidence(projectDir);
assert(~isempty(report) && isfield(report,"numSamples"), ...
    "Trajectory report is unavailable or incomplete.");

actions = string({report.action});
valid = find(actions=="conveyor_to_pallet" & [report.ikOK]);
if isempty(valid)
    valid = find([report.ikOK]);
end
assert(~isempty(valid),"No successful physical transfer is available.");
selectedReportIndex = valid(ceil(numel(valid)/2));

counts = [report.numSamples];
firstSample = 1 + sum(counts(1:selectedReportIndex-1));
lastSample = firstSample + counts(selectedReportIndex)-1;
q = traj.q(:,firstSample:lastSample);
qd = traj.qd(:,firstSample:lastSample);
qdd = traj.qdd(:,firstSample:lastSample);
t = traj.time(firstSample:lastSample);
t = t-t(1);

robot = exampleHelperSetUpCobot();
eeName = traj.endEffector;
p = zeros(3,numel(t));
for k=1:numel(t)
    T = getTransform(robot,q(:,k).',eeName);
    p(:,k) = T(1:3,4);
end

phaseNames = ["Start","Pre-pick","Pick","Lift","Pre-place","Place","Retreat"];
phaseIdx = unique(round(linspace(1,numel(t),numel(phaseNames))));
phaseNames = phaseNames(1:numel(phaseIdx));

jointPathLength = sum(vecnorm(diff(q,1,2),2,1));
cartesianPathLength = sum(vecnorm(diff(p,1,2),2,1));
peakJointSpeed = max(abs(qd),[],"all");
peakJointAcceleration = max(abs(qdd),[],"all");

actionLabel = replace(actions(selectedReportIndex),"_"," ");
waypointLabels = "W"+(0:numel(phaseIdx)-1);

% Figure 1: Cartesian path and every planning waypoint.
fPath = figure("Color","w","Position",[80 80 1450 650]);
ax1 = axes(fPath);
ax1.Position = [0.06 0.18 0.88 0.72];
hold(ax1,"on");
segmentNames = ["Home to pre-pick","Pick descent","Lift", ...
    "Transfer","Place descent","Retreat"];
segmentColors = lines(numel(segmentNames));
for k=1:numel(segmentNames)
    ids = phaseIdx(k):phaseIdx(k+1);
    plot3(ax1,p(1,ids),p(2,ids),p(3,ids),"LineWidth",3, ...
        "Color",segmentColors(k,:),"DisplayName",segmentNames(k));
end
scatter3(ax1,p(1,phaseIdx),p(2,phaseIdx),p(3,phaseIdx),42, ...
    1:numel(phaseIdx),"filled","HandleVisibility","off");
labelOffset = [0.02 0.02 0.02 -0.07 0.02 0.02 -0.07; ...
               0.00 0.00 0.00  0.02 0.00 0.00  0.02; ...
               0.02 0.05 -0.05 0.08 0.05 -0.05 0.08];
for k=1:numel(phaseIdx)
    text(ax1,p(1,phaseIdx(k))+labelOffset(1,k), ...
        p(2,phaseIdx(k))+labelOffset(2,k), ...
        p(3,phaseIdx(k))+labelOffset(3,k),waypointLabels(k), ...
        "FontSize",12,"FontWeight","bold","Color",[0.05 0.05 0.05]);
end
grid(ax1,"on"); axis(ax1,"equal"); view(ax1,38,25);
xlabel(ax1,"World X (m)"); ylabel(ax1,"World Y (m)"); zlabel(ax1,"World Z (m)");
title(ax1,sprintf("End-effector path: carton %d, %s", ...
    report(selectedReportIndex).ID,actionLabel));
legend(ax1,"Location","southoutside","NumColumns",3);
annotation(fPath,"textbox",[0.75 0.61 0.20 0.27],"String", ...
    sprintf("W0 Start\nW1 Pre-pick\nW2 Pick\nW3 Lift\nW4 Pre-place\nW5 Place\nW6 Retreat"), ...
    "FitBoxToText","on","BackgroundColor","white","EdgeColor",[0.2 0.2 0.2], ...
    "FontSize",11,"FontWeight","bold");
exportgraphics(fPath,fullfile(outDir,"representative_ee_path_waypoints.png"),"Resolution",300);
savefig(fPath,fullfile(outDir,"representative_ee_path_waypoints.fig"));
close(fPath);

% Figure 2: joint trajectory with the same planning waypoints.
fJoint = figure("Color","w","Position",[80 80 1200 720]);
ax2 = axes(fJoint);
plot(ax2,t,q.',"LineWidth",1.7); hold(ax2,"on");
jointColors = colororder(ax2);
for k=1:numel(phaseIdx)
    scatter(ax2,repmat(t(phaseIdx(k)),size(q,1),1),q(:,phaseIdx(k)),32, ...
        jointColors(1:size(q,1),:),"filled","HandleVisibility","off");
end
for k=2:numel(phaseIdx)-1
    xline(ax2,t(phaseIdx(k)),":",waypointLabels(k),"Color",[0.35 0.35 0.35], ...
        "LabelVerticalAlignment","top","LabelOrientation","horizontal", ...
        "HandleVisibility","off");
end
grid(ax2,"on"); xlabel(ax2,"Time (s)"); ylabel(ax2,"Joint angle (rad)");
title(ax2,{sprintf("Continuous six-joint trajectory: carton %d",report(selectedReportIndex).ID), ...
    "W0 Start | W1 Pre-pick | W2 Pick | W3 Lift | W4 Pre-place | W5 Place | W6 Retreat"});
legend(ax2,"q_1","q_2","q_3","q_4","q_5","q_6", ...
    "Location","southoutside","NumColumns",6);
exportgraphics(fJoint,fullfile(outDir,"representative_joint_trajectory_waypoints.png"),"Resolution",300);
savefig(fJoint,fullfile(outDir,"representative_joint_trajectory_waypoints.fig"));
close(fJoint);

jointNames = "q"+(1:size(q,1));
motionTable = array2table([t(:),q.',p.'], ...
    "VariableNames",["Time_s",jointNames,"EE_X_m","EE_Y_m","EE_Z_m"]);
writetable(motionTable,fullfile(outDir,"representative_robot_motion.csv"));

evidence = table(string(datetime("now","Format","yyyyMMdd_HHmmss")), ...
    report(selectedReportIndex).ID,actions(selectedReportIndex),numel(t),t(end), ...
    jointPathLength,cartesianPathLength,peakJointSpeed,peakJointAcceleration, ...
    logical(report(selectedReportIndex).ikOK),false,"Not evaluated", ...
    'VariableNames',["RunID","CartonID","Action","Samples","Duration_s", ...
    "JointPathLength_rad","CartesianPathLength_m","PeakJointSpeed_rad_s", ...
    "PeakJointAcceleration_rad_s2","IKSuccess","CollisionCheckEnabled", ...
    "CollisionStatus"]);
writetable(evidence,fullfile(outDir,"representative_robot_motion_summary.csv"));
save(fullfile(outDir,"representative_robot_motion.mat"), ...
    "q","qd","qdd","t","p","phaseIdx","phaseNames","evidence");

disp(evidence);
fprintf("Representative motion evidence saved in: %s\n",outDir);
end

function [traj,report] = loadTrajectoryEvidence(projectDir)
if evalin("base","exist('traj','var') && exist('trajectoryReport','var')")
    traj = evalin("base","traj");
    report = evalin("base","trajectoryReport");
    return;
end
matPath = fullfile(projectDir,"plannedTrajectory.mat");
assert(exist(matPath,"file")==2, ...
    "No trajectory data found. Run the 40-carton preparation first.");
s = load(matPath,"traj","trajectoryReport");
traj = s.traj;
report = s.trajectoryReport;
end
