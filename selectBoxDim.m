function [boxDim, boxWeight] = selectBoxDim(id)

boxData = evalin("base","boxData");

maxID = length(boxData.length);
idx = mod(double(id), maxID) + 1;
try
    executionPlan = evalin("base","executionPlan");
    if ~isempty(executionPlan)
        actionIdx = mod(double(id),numel(executionPlan))+1;
        idx = double(executionPlan(actionIdx).boxID);
    end
catch
end

boxDim = getLayoutDim(idx, boxData);
boxWeight = boxData.weight(idx);

end

function boxDim = getLayoutDim(idx, boxData)
boxDim = [
    boxData.length(idx)
    boxData.width(idx)
    boxData.height(idx)
];

try
    layoutPlan = evalin("base","layoutPlan");
    if idx >= 1 && idx <= numel(layoutPlan) && layoutPlan(idx).isPlaced
        boxDim = [
            layoutPlan(idx).L
            layoutPlan(idx).W
            layoutPlan(idx).H
        ];
    end
catch
end
end
