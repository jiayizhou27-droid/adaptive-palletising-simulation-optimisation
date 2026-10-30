function spawnBox(id, boxPosition, boxDim)
%SPAWNBOX Create or replace a variable-size box actor in the Sim3D world.
%
if nargin < 3 || isempty(boxDim)
    boxDim = 0.02;
end

if isscalar(boxDim)
    dims = double([boxDim boxDim boxDim]);
else
    dims = double(boxDim(:)).';
    if numel(dims) < 3
        dims = [dims repmat(dims(end), 1, 3-numel(dims))];
    end
    dims = dims(1:3);
end

if nargin < 2 || isempty(boxPosition)
    boxPosition = [0 0 dims(3)/2];
end
boxPosition = double(boxPosition(:)).';
cfg = palletTaskConfig();
boxPosition(1:2) = cfg.sourceBoxXY;
boxPosition(3) = cfg.sourceSurfaceZ + dims(3)/2 + cfg.sourceSpawnClearance;
boxPosSim3D = worldToSim3DPosition(boxPosition);
isHiddenWarmupBox = double(id) < cfg.hiddenInitialBoxes;
try
    executionPlan = evalin('base','executionPlan');
    if ~isempty(executionPlan)
        actionIdx=mod(double(id),numel(executionPlan))+1;
        isHiddenWarmupBox=strcmp(executionPlan(actionIdx).action,'hidden_warmup');
    end
catch
end

world = sim3d.World.getWorld(bdroot);
name = ['collBox' num2str(double(id))];

oldActor = world.Root.findBy('ActorName', name, 'first');
if isempty(oldActor)
    box = sim3d.Actor('ActorName', name, ...
        'Mobility', sim3d.utils.MobilityTypes.Movable);
    add(world, box);
    createShape(box, 'box', dims);
else
    box = oldActor;
    try
        box.Mobility = sim3d.utils.MobilityTypes.Movable;
    catch
    end
end
setSim3DActorCenter(box, boxPosSim3D);
box.Rotation = [0 0 0];
box.Color = [200 100 16]/256;
box.Collisions = ~isHiddenWarmupBox;
box.Parent = world.Root;

if isHiddenWarmupBox
    hideActor(box);
else
    showActor(box);
end

end

function hideActor(actor)
try, actor.Visible = false; catch, end
try, actor.Opacity = 0; catch, end
try, actor.Transparency = 1; catch, end
try, actor.Color = [200 100 16]/256; catch, end
end

function showActor(actor)
try, actor.Visible = true; catch, end
try, actor.Opacity = 1; catch, end
try, actor.Transparency = 0; catch, end
end
