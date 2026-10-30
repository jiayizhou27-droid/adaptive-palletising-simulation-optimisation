function finalizeExecutionAction(actionID)
plan=evalin('base','executionPlan');
idx=mod(double(actionID),numel(plan))+1;
world=sim3d.World.getWorld(bdroot);
actorID=getExecutionActorID(actionID);
actor=[];
if strcmp(plan(idx).action,'conveyor_to_buffer') || ...
        strcmp(plan(idx).action,'buffer_to_pallet')
    try
        registry=evalin('base','executionActorRegistry');
        if numel(registry)>=plan(idx).boxID
            actor=registry{plan(idx).boxID};
        end
    catch
    end
end
if isempty(actor)
    actor=world.Root.findBy('ActorName',['collBox' num2str(actorID)],'first');
end
if isempty(actor), return; end
try, actor.Mobility=sim3d.utils.MobilityTypes.Movable; catch, end
try, actor.Collisions=false; catch, end
setSim3DActorCenter(actor,worldToSim3DPosition(plan(idx).toPos));
if strcmp(plan(idx).action,'hidden_warmup') || ...
        strcmp(plan(idx).action,'terminal_flush')
    try, actor.Visible=false; catch, end
    try, actor.Opacity=0; catch, end
    try, actor.Transparency=1; catch, end
end
% Dimensions were already swapped for yaw=90 deg, so no extra Sim3D
% rotation is needed. Keeping zero also avoids world/Sim3D yaw sign drift.
try, actor.Rotation=[0 0 0]; catch, end
try, actor.PreciseContacts=false; catch, end
try, actor.Mobility=sim3d.utils.MobilityTypes.Static; catch, end
if strcmp(plan(idx).action,'buffer_to_pallet')
    try
        registry=evalin('base','executionActorRegistry');
        registry{plan(idx).boxID}=[];
        assignin('base','executionActorRegistry',registry);
    catch
    end
end
try
    log=evalin('base','executionActionLog');
catch
    log=zeros(0,4);
end
actionCode=-strcmp(plan(idx).action,'hidden_warmup')+ ...
    -2*strcmp(plan(idx).action,'terminal_flush')+ ...
    strcmp(plan(idx).action,'conveyor_to_buffer')+ ...
    2*strcmp(plan(idx).action,'buffer_to_pallet');
log(end+1,:)=[double(actionID),double(plan(idx).boxID), ...
    double(plan(idx).bufferSlot),double(actionCode)];
assignin('base','executionActionLog',log);
end
