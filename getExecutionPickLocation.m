function pick=getExecutionPickLocation(actionID)
plan=evalin('base','executionPlan');
idx=mod(double(actionID),numel(plan))+1;
pick=double(plan(idx).fromPos);
end
