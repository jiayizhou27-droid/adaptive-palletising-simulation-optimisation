function snapPlacedBoxesToLayout(nextID)
%SNAPPLACEDBOXESTOLAYOUT Align dropped Sim3D boxes to optimizer targets.
%
% nextID is the incoming Simulink zero-based box ID. All boxes with IDs
% smaller than nextID are already considered placed, so their visual actors
% are snapped to layoutPlan. This function never deletes actors during a
% simulation step because the Simulation 3D Robot block may hold references
% to them.

if nargin < 1
    nextID = inf;
end

layoutPlan = evalin("base", "layoutPlan");
if isempty(layoutPlan)
    return;
end
cfg = palletTaskConfig();

lastID = min(double(nextID) - 1, numel(layoutPlan) - 1);
if lastID < 0
    return;
end

world = sim3d.World.getWorld(bdroot);

for id = 0:lastID
    idx = id + 1;
    if idx > numel(layoutPlan) || ~layoutPlan(idx).isPlaced
        continue;
    end

    moveLayoutBox(world, id, layoutPlan(idx), id < cfg.hiddenInitialBoxes);
end
end

function moveLayoutBox(world, id, layoutBox, hideBox)
name = ['collBox' num2str(id)];
actor = world.Root.findBy('ActorName', name, 'first');

pos = double(layoutBox.pos);
dims = double([layoutBox.L layoutBox.W layoutBox.H]);
if numel(pos) ~= 3 || numel(dims) ~= 3
    return;
end

if isempty(actor)
    warning("PalletizeBoxes:MissingPlacedActor", ...
        "Skip snap for missing actor %s. New actors are only created by spawnBox.", name);
    return;
end

try
    actor.Mobility = sim3d.utils.MobilityTypes.Movable;
catch
end
actor.Collisions = false;
setSim3DActorCenter(actor, layoutToSim3DPosition(pos));
actor.Rotation = [0 0 0];
actor.Collisions = false;
if hideBox
    hideActor(actor);
else
    showActor(actor);
end
try
    actor.PreciseContacts = false;
catch
end
try
    actor.Mobility = sim3d.utils.MobilityTypes.Static;
catch
end
end

function hideActor(actor)
try, actor.Visible = false; catch, end
try, actor.Opacity = 0; catch, end
try, actor.Transparency = 1; catch, end
end

function showActor(actor)
try, actor.Visible = true; catch, end
try, actor.Opacity = 1; catch, end
try, actor.Transparency = 0; catch, end
end
