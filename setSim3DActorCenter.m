function setSim3DActorCenter(actor, desiredCenter)
%SETSIM3DACTORCENTER Move a Sim3D actor by its visual/collision centre.
%
% Sim3D primitive actors are not guaranteed to have their local origin at
% the geometric centre. For boxes, using Translation directly as the centre
% causes size-dependent placement drift. This helper writes Translation so
% that Translation + localMeshCentre equals the requested centre.

desiredCenter = double(desiredCenter(:)).';
actor.Translation = desiredCenter - sim3dActorLocalCenter(actor);
end
