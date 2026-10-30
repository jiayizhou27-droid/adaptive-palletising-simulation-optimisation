function cfg = palletTaskConfig()
%PALLETTASKCONFIG Shared task geometry for layout, spawning, and planning.
%
% All positions are expressed in the Robotics/Simulink world frame. Sim3D
% uses the opposite Y convention; spawnBox handles that conversion only at
% the visualization boundary.

cfg.palletDim = [0.8 1.2 0.144];
cfg.palletStlCenterOffsetSim3D = [0.4 0.6 cfg.palletDim(3)/2];
% Shift the pallet 0.20 m in X and 0.20 m in world Y toward the robot and
% conveyor. The resulting world centre is [-0.60 -0.30].
cfg.scenePalletSim3DTranslation = [-1.0 -0.3 0];
cfg.scenePalletCenterSim3D = cfg.scenePalletSim3DTranslation + cfg.palletStlCenterOffsetSim3D;
cfg.scenePalletCenter = [ ...
    cfg.scenePalletCenterSim3D(1), ...
    -cfg.scenePalletCenterSim3D(2), ...
    cfg.scenePalletCenterSim3D(3)];
cfg.scenePalletLoc = cfg.scenePalletCenter;
cfg.placementOffset = [0 0 0];
cfg.palletLoc = cfg.scenePalletLoc + cfg.placementOffset;
cfg.sim3DLayoutBoxOffset = [0 0 0];
cfg.palletMargin = 0.025;
cfg.boxGap = 0.008;
cfg.palletSim3DTranslation = cfg.scenePalletSim3DTranslation;

cfg.layoutBoxOffset = 0;
cfg.firstExcelID = cfg.layoutBoxOffset + 1;
cfg.hiddenInitialBoxes = 1;
cfg.plotStartBox = cfg.hiddenInitialBoxes + 1;
cfg.maxLayoutBoxes = 40;
% Optimize the complete 40-carton set independently of conveyor arrival
% order.  The execution scheduler uses the physical buffer when required.
cfg.batchSize = 40;
cfg.targetBoxesPerLayer = inf;

% Common twelve-slot buffer used by Constructive, CS-GA and CS-SA.
% The original continuous platform convention and robot-side clearance are
% retained; only one extra column is added away from the conveyor.
cfg.bufferCapacity = 12;
cfg.bufferGrid = [4 3];
cfg.bufferOriginXY = [-0.16 1.00];
cfg.bufferPitch = [0.27 0.20];
cfg.bufferPlatformDim = [0.82 0.80 0.04];
cfg.bufferPlatformCenterXY = [0.11 0.70];

% Scene geometry used to guarantee contact without intersection.
cfg.conveyorCenterXY = [-0.70 0.70];
cfg.conveyorDimXY = [0.80 0.80];
cfg.robotBaseCenterXY = [0 0];
cfg.robotBaseDimXY = [0.25 0.25];

cfg.sourceSurfaceZ = 0.50;
cfg.bufferSurfaceZ = cfg.sourceSurfaceZ;
cfg.sourceSpawnClearance = 0.015;
cfg.sourceBoxXY = [-0.65 0.65];
cfg.sourceBoxPosition = [cfg.sourceBoxXY, cfg.sourceSurfaceZ + 0.25/2 + cfg.sourceSpawnClearance];
cfg.sourceBoxSim3DPosition = worldToSim3DPosition(cfg.sourceBoxPosition);
cfg.pickPosition = cfg.sourceBoxPosition + [0 0 0.22];

cfg.approachHeight = 0.28;
cfg.topClearance = 0.08;
cfg.transferClearance = 0.32;
cfg.minTransferHeight = 0.88;
cfg.pickYaw = 0;
end
