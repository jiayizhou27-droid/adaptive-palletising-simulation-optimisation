function spawnExecutionActionBox(actionID,boxPosition,boxDim)
%SPAWNEXECUTIONACTIONBOX Create source box or re-expose buffered box.
plan=evalin('base','executionPlan');
idx=mod(double(actionID),numel(plan))+1;
action=plan(idx);
% Finalize the previous actor only after the robot block has detached it.
if double(actionID)>0
    finalizeExecutionAction(double(actionID)-1);
end
if strcmp(action.action,'hidden_warmup') || ...
        strcmp(action.action,'terminal_flush')
    spawnBox(actionID,boxPosition,boxDim);
    world=sim3d.World.getWorld(bdroot);
    actor=world.Root.findBy('ActorName',['collBox' num2str(double(actionID))],'first');
    if ~isempty(actor)
        try, actor.Visible=false; catch, end
        try, actor.Opacity=0; catch, end
        try, actor.Transparency=1; catch, end
        try, actor.Collisions=false; catch, end
    end
elseif strcmp(action.action,'buffer_to_pallet')
    world=sim3d.World.getWorld(bdroot);
    try, registry=evalin('base','executionActorRegistry'); catch, registry={}; end
    assert(numel(registry)>=action.boxID && ~isempty(registry{action.boxID}), ...
        'Buffered actor handle for box %d is missing.',action.boxID);
    actor=registry{action.boxID};
    try, actor.Mobility=sim3d.utils.MobilityTypes.Movable; catch, end
    setSim3DActorCenter(actor,worldToSim3DPosition(action.fromPos));
    try, actor.Collisions=true; catch, end
else
    spawnBox(actionID,boxPosition,boxDim);
    if strcmp(action.action,'conveyor_to_buffer')
        world=sim3d.World.getWorld(bdroot);
        actor=world.Root.findBy('ActorName',['collBox' num2str(double(actionID))],'first');
        assert(~isempty(actor),'New buffer actor collBox%d is missing.',double(actionID));
        try, registry=evalin('base','executionActorRegistry'); catch, registry={}; end
        if numel(registry)<action.boxID, registry{action.boxID}=[]; end
        registry{action.boxID}=actor;
        assignin('base','executionActorRegistry',registry);
    end
end
end
