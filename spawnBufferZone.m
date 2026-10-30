function spawnBufferZone()
%SPAWNBUFFERZONE Add one continuous buffer platform flush with the conveyor.
cfg=palletTaskConfig();
world=sim3d.World.getWorld(bdroot);

name='BufferPlatformContinuous';
actor=world.Root.findBy('ActorName',name,'first');
if isempty(actor)
    actor=sim3d.Actor('ActorName',name, ...
        'Mobility',sim3d.utils.MobilityTypes.Movable);
    add(world,actor);
    createShape(actor,'box',cfg.bufferPlatformDim);
end

% The platform top is exactly coplanar with the conveyor/source surface.
p=[cfg.bufferPlatformCenterXY, ...
   cfg.bufferSurfaceZ-cfg.bufferPlatformDim(3)/2];
setSim3DActorCenter(actor,worldToSim3DPosition(p));
actor.Color=[45 115 184]/255;
actor.Collisions=true;
actor.PreciseContacts=true;
actor.Parent=world.Root;
try, actor.Mobility=sim3d.utils.MobilityTypes.Static; catch, end
end
