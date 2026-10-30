function goal=getExecutionPlaceLocation(actionID)
plan=evalin('base','executionPlan');
idx=mod(double(actionID),numel(plan))+1;
goal=double(plan(idx).toPos);
end
