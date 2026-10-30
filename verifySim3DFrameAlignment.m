function verifySim3DFrameAlignment()
%VERIFYSIM3DFRAMEALIGNMENT Check layout world coordinates against Sim3D.

cfg = palletTaskConfig();

expectedPalletSim3D = cfg.palletSim3DTranslation;
expectedSourceBoxSim3D = cfg.sourceBoxSim3DPosition;
palletCenterSim3D = cfg.scenePalletSim3DTranslation + cfg.palletStlCenterOffsetSim3D;
palletCenterWorld = [palletCenterSim3D(1), -palletCenterSim3D(2), palletCenterSim3D(3)];

fprintf("Scene pallet center: [%.3f %.3f %.3f]\n", cfg.scenePalletLoc);
fprintf("Calibrated layout center: [%.3f %.3f %.3f]\n", cfg.palletLoc);
fprintf("Placement offset: [%.3f %.3f %.3f]\n", cfg.placementOffset);
fprintf("Expected Sim3D pallet translation: [%.3f %.3f %.3f]\n", expectedPalletSim3D);
fprintf("Expected Sim3D pallet center: [%.3f %.3f %.3f]\n", palletCenterSim3D);
fprintf("Robotics source box center: [%.3f %.3f %.3f]\n", cfg.sourceBoxPosition);
fprintf("Expected Sim3D source box center: [%.3f %.3f %.3f]\n", expectedSourceBoxSim3D);

assert(max(abs(palletCenterWorld - cfg.scenePalletLoc)) < 1e-12, ...
    "Scene pallet actor center is inconsistent with layout center.");
assert(abs(expectedSourceBoxSim3D(2) + cfg.sourceBoxPosition(2)) < 1e-12, ...
    "Source box Y-axis conversion is inconsistent.");

fprintf("SIM3D_FRAME_ALIGNMENT_OK\n");
end
