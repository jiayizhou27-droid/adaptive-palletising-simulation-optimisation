function goalLoc = getGoalFromLayout(id)

layoutPlan = evalin('base','layoutPlan');

maxID = numel(layoutPlan);
idx = mod(double(id), maxID) + 1;

goalLoc = layoutPlan(idx).pos;

end
