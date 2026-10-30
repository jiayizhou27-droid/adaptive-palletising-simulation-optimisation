function center = sim3dActorLocalCenter(actor)
%SIM3DACTORLOCALCENTER Return an actor's local mesh centre.

center = [0 0 0];
try
    vertices = double(actor.Vertices);
    if ~isempty(vertices)
        center = (max(vertices) + min(vertices)) / 2;
    end
catch
end
end
