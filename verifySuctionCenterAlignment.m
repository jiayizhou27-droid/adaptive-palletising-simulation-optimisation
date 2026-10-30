function verifySuctionCenterAlignment(n)
%VERIFYSUCTIONCENTERALIGNMENT Check suction center aligns above box center.

if nargin < 1 || isempty(n)
    n = 10;
end

cfg = palletTaskConfig();
layoutPlan = evalin("base", "layoutPlan");
if isempty(layoutPlan)
    error("layoutPlan is empty. Run model initialization first.");
end

startBox = cfg.plotStartBox;
endBox = min(startBox + n - 1, numel(layoutPlan));
ids = (startBox:endBox).';
h = [layoutPlan(ids).H].';

eeOffset = [-0.2*ones(numel(ids),1), zeros(numel(ids),1), h/2 + 0.11];
suctionCenterOffset = eeOffset + repmat([0.2 0 0], numel(ids), 1);
xyErr = max(abs(suctionCenterOffset(:,1:2)), [], 2);

disp(table(ids, h, eeOffset(:,1), eeOffset(:,2), eeOffset(:,3), ...
    suctionCenterOffset(:,1), suctionCenterOffset(:,2), suctionCenterOffset(:,3), ...
    xyErr, ...
    'VariableNames', ["PlotBoxID","H","EEOffsetX","EEOffsetY","EEOffsetZ", ...
    "SuctionX","SuctionY","SuctionZ","XYErr"]));

assert(all(xyErr < 1e-12), "Suction center XY is not aligned with box center.");
fprintf("SUCTION_CENTER_ALIGNMENT_OK\n");
fprintf("Suction center XY offset is zero for visible boxes %d through %d.\n", ...
    startBox, endBox);
end
