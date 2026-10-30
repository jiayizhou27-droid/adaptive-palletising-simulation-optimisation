function positionArray = exampleHelperGetBoxPoses(graspState,ID,maxNumOfBoxes)
%EXAMPLEHELPERGETBOXPOSES This helper queries the sim3D environment to get
%the pose of all the boxes currently on the pallet

%   Copyright 2024 The MathWorks, Inc.

positionArray = NaN(maxNumOfBoxes,3);

W = sim3d.World.getWorld(bdroot);
actorList = W.Actors;
actorNames = fieldnames(actorList);
 
% Extract the collision environment
collIdxArray = find(contains(actorNames,'coll'));

% If graspState is true, remove collBox(ID) from the list
if graspState == true
    idx = find(contains(actorNames,['collBox',num2str(ID)]));
    collIdxArray = collIdxArray(collIdxArray~=idx);
end


for i=1:numel(collIdxArray)
    actor = actorList.(actorNames{collIdxArray(i)});
    maxVertices = max(double(actor.Vertices));
    minVertices = min(double(actor.Vertices));
    
    boxCenterLoc = (maxVertices + minVertices)/2;
    objLoc = double(actor.Translation) + boxCenterLoc;
    
    % Convert to SL frame
    boxPose = [objLoc(1) -objLoc(2) objLoc(3)];
    positionArray(i,3) = boxPose;
end

end

