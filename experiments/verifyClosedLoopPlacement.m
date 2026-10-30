function verdict = verifyClosedLoopPlacement(targetPos,targetYaw,measurement,cfg)
%VERIFYCLOSEDLOOPPLACEMENT Compare a vision measurement with its target.
if nargin < 4 || isempty(cfg)
    cfg = closedLoopVisionConfig();
end
positionError = double(measurement.position) - double(targetPos);
translationError = norm(positionError);
yawError = abs(wrapTo180Local(double(measurement.yawDeg) - ...
    rad2deg(double(targetYaw))));
missing = ~logical(measurement.actorVisible);
graspFailure = ~logical(measurement.graspSucceeded);
translationFault = translationError > cfg.translationTolerance_m;
rotationFault = yawError > cfg.yawTolerance_deg;
detected = missing || graspFailure || translationFault || rotationFault;

reason = "none";
if missing
    reason = "missing_box";
elseif graspFailure
    reason = "grasp_failure";
elseif translationFault && rotationFault
    reason = "translation_and_rotation";
elseif translationFault
    reason = "translation_error";
elseif rotationFault
    reason = "rotation_error";
end
verdict = struct( ...
    'detected',detected, ...
    'reason',reason, ...
    'translationError_m',translationError, ...
    'positionError_m',positionError, ...
    'yawError_deg',yawError, ...
    'pauseRequested',detected && cfg.pauseOnFault, ...
    'sendToBuffer',detected, ...
    'replanRequested',detected && cfg.replanOnFault);
end

function angle = wrapTo180Local(angle)
angle = mod(angle + 180,360) - 180;
end
