function sim3dPosition = layoutToSim3DPosition(worldPosition)
%LAYOUTTOSIM3DPOSITION Map 3D-plot pallet targets to Sim3D pallet targets.
%
% This is an equal-scale pallet-to-pallet mapping:
%   source centre: cfg.palletLoc used by the MATLAB 3D plot and planner
%   target centre: the actual Sim3D collPallet mesh centre, when available
%   axes: X same direction, Y opposite direction, Z same direction
%
% Keeping the mapping relative to the two pallet centres prevents the box
% layout from drifting toward the viewer if the Sim3D pallet actor has a
% different mesh origin from the Robotics world convention.

worldPosition = double(worldPosition(:)).';
cfg = palletTaskConfig();

simPalletCenter = cfg.scenePalletCenterSim3D;
try
    world = sim3d.World.getWorld(bdroot);
    actor = world.Root.findBy('ActorName', 'collPallet', 'first');
    if ~isempty(actor)
        maxVertices = max(double(actor.Vertices));
        minVertices = min(double(actor.Vertices));
        meshCenter = (maxVertices + minVertices) / 2;
        simPalletCenter = double(actor.Translation) + meshCenter;
    end
catch
end

delta = worldPosition - cfg.palletLoc;
sim3dPosition = simPalletCenter + [delta(1), -delta(2), delta(3)] + cfg.sim3DLayoutBoxOffset;
end
