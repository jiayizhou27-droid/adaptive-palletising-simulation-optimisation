function [baseLoc,baseDim,benchLoc,benchDim,palletLoc,palletDim] = exampleHelperGetInitActorProp()
%HELPERGETINITACTORPROP Queries the sim3d world for workspace actor properties

%   Copyright 2024 The MathWorks, Inc.

W = sim3d.World.getWorld(bdroot);
actorList = W.Actors;

% Extract the collision environment
actor1 = actorList.('collBench');
maxVertices = max(double(actor1.Vertices));
minVertices = min(double(actor1.Vertices));

boxCenterLoc = (maxVertices + minVertices)/2;
benchDim = maxVertices - minVertices;
objLoc1 = double(actor1.Translation) + boxCenterLoc;
% Convert to SL frame
benchLoc = [objLoc1(1) -objLoc1(2) objLoc1(3)];

% Extract the collision environment
actor2 = actorList.('collBase');
maxVertices = max(double(actor2.Vertices));
minVertices = min(double(actor2.Vertices));
baseDim = maxVertices - minVertices;
objLoc2 = double(actor2.Translation);
    
% Convert to SL frame
baseLoc = [objLoc2(1) -objLoc2(2) objLoc2(3)];

actor3 = actorList.('collPallet');
maxVertices = max(double(actor3.Vertices));
minVertices = min(double(actor3.Vertices));

boxCenterLoc = (maxVertices + minVertices)/2;
palletDim = maxVertices - minVertices;
objLoc3 = double(actor3.Translation) + boxCenterLoc;
    
% Convert to SL frame
palletLoc = [objLoc3(1) -objLoc3(2) objLoc3(3)];


end

