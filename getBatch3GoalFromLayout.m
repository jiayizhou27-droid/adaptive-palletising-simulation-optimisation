function goalLoc = getBatch3GoalFromLayout(id)
% getBatch3GoalFromLayout Return the current 3-box batch optimized target.
%
% Simulink uses 0-based IDs. layoutPlan uses MATLAB 1-based indexing.

layoutPos = [];
try
    layoutPos = evalin('base','layoutPos');
catch
end

if ~isempty(layoutPos)
    maxID = size(layoutPos, 1);
    idx = mod(double(id), maxID) + 1;
    goalLoc = double(layoutPos(idx,:));
    return;
end

layoutPlan = evalin('base','layoutPlan');
maxID = numel(layoutPlan);
idx = mod(double(id), maxID) + 1;
goalLoc = double(layoutPlan(idx).pos);
end
