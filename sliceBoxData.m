function boxData = sliceBoxData(boxDataAll, firstExcelID)
% sliceBoxData Keep boxes from the requested Excel row number onward.

if nargin < 2 || isempty(firstExcelID)
    firstExcelID = 1;
end

n = numel(boxDataAll.length);
firstExcelID = max(1, min(double(firstExcelID), n + 1));
idx = firstExcelID:n;

boxData.length = boxDataAll.length(idx);
boxData.width = boxDataAll.width(idx);
boxData.height = boxDataAll.height(idx);
boxData.weight = boxDataAll.weight(idx);
if isfield(boxDataAll, 'type')
    boxData.type = boxDataAll.type(idx);
end
boxData.sourceID = idx(:);
boxData.skippedExcelIDs = (1:firstExcelID-1).';
end
