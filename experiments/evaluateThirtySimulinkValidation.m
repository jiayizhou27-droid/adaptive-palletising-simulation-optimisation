function [evaluated,summary] = evaluateThirtySimulinkValidation()
%EVALUATETHIRTYSIMULINKVALIDATION Apply preregistered pass/fail criteria.
projectDir=fileparts(mfilename('fullpath'));
outDir=fullfile(projectDir,'simulink_validation_30');
file=fullfile(outDir,'simulink_validation_results.csv');
assert(isfile(file),'Run runThirtySimulinkValidation first.');
evaluated=readtable(file,'TextType','string');

evaluated.ActionLogPass=evaluated.Completed & ...
    evaluated.LoggedActions>=evaluated.ExpectedActions;
evaluated.BufferPass=evaluated.PeakBufferOccupancy<=evaluated.PhysicalBufferCapacity & ...
    evaluated.RequiredBufferCapacity<=evaluated.PhysicalBufferCapacity;
evaluated.SupportPass=evaluated.MinimumSupportRatio>=0.95-1e-9;
evaluated.COMStabilityPass=evaluated.MinimumCOMMargin_m>=0.010-1e-9 & ...
    evaluated.UnstableBoxCount==0;
evaluated.TrajectoryPass=evaluated.TrajectoryFeasible & ...
    isfinite(evaluated.EstimatedCycleTime_s);
evaluated.FullValidationPass=evaluated.ActionLogPass & evaluated.BufferPass & ...
    evaluated.SupportPass & evaluated.COMStabilityPass & evaluated.TrajectoryPass;
evaluated.CycleTimePerBox_s=evaluated.EstimatedCycleTime_s./evaluated.N;
evaluated.EnergyPerBox_J=evaluated.JointMechanicalEnergy_J./evaluated.N;
evaluated.MovesPerBox=(evaluated.DirectPlacements+evaluated.BufferInActions+ ...
    evaluated.BufferOutActions)./evaluated.N;
writetable(evaluated,fullfile(outDir,'simulink_validation_evaluated.csv'));

methods=unique(evaluated.Method,'stable');rows=cell(numel(methods),1);
for i=1:numel(methods)
    X=evaluated(evaluated.Method==methods(i),:);
    rows{i}=table(methods(i),height(X),mean(X.FullValidationPass), ...
        mean(X.CycleTimePerBox_s,'omitnan'),mean(X.EnergyPerBox_J,'omitnan'), ...
        mean(X.MovesPerBox,'omitnan'),mean(X.PeakBufferOccupancy,'omitnan'), ...
        mean(X.PalletVolumeUtilization,'omitnan'),mean(X.StackHeight_m,'omitnan'), ...
        'VariableNames',{'Method','Runs','ValidationPassRate', ...
        'MeanCycleTimePerBox_s','MeanEnergyPerBox_J','MeanMovesPerBox', ...
        'MeanPeakBufferOccupancy','MeanPalletVolumeUtilization','MeanStackHeight_m'});
end
summary=vertcat(rows{:});
writetable(summary,fullfile(outDir,'simulink_validation_method_summary.csv'));
disp(summary);
fprintf('\nHard acceptance criteria:\n');
fprintf('  Completed action log: LoggedActions >= ExpectedActions\n');
fprintf('  Physical buffer: peak and required capacity <= physical capacity\n');
fprintf('  Support ratio >= 0.95\n');
fprintf('  COM support margin >= 0.010 m and unstable cartons = 0\n');
fprintf('  A finite collision-aware robot trajectory exists\n');
end
