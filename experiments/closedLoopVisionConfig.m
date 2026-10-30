function cfg = closedLoopVisionConfig()
%CLOSEDLOOPVISIONCONFIG Thresholds for software placement verification.
cfg.translationTolerance_m = 0.0015;
cfg.yawTolerance_deg = 0.5;
cfg.requireActorVisible = true;
cfg.requireGraspSuccess = true;
cfg.recoveryBufferSlot = 1;
cfg.pauseOnFault = true;
cfg.replanOnFault = true;
end
