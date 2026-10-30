function outputs = buildThesisEvidenceChain()
%BUILDTHESISEVIDENCECHAIN Build a reproducible thesis evidence package.
%   This function reuses completed experiments; it does not rerun any
%   optimizer or Simulink model. Run it from any working directory:
%
%       outputs = buildThesisEvidenceChain;
%
%   The generated CSV, PNG and TXT files are written to
%   thesis_evidence_chain beside this file.

rootDir = fileparts(mfilename('fullpath'));
sourceDir = fullfile(rootDir, 'data');
outDir = fullfile(rootDir, 'generated_evidence');
figDir = fullfile(outDir, 'figures');
if ~isfolder(outDir), mkdir(outDir); end
if ~isfolder(figDir), mkdir(figDir); end

scale = readtable(fullfile(sourceDir, 'merged_scale_results.csv'), ...
    'TextType', 'string', 'VariableNamingRule', 'preserve');
typeData = readtable(fullfile(sourceDir, 'merged_type_complexity_results.csv'), ...
    'TextType', 'string', 'VariableNamingRule', 'preserve');
buffer = readtable(fullfile(sourceDir, 'merged_buffer_results.csv'), ...
    'TextType', 'string', 'VariableNamingRule', 'preserve');

methods = ["Constructive", "CS-GA", "CS-SA"];
requiredLayout = ["CaseID","Method","PlanningSucceeded", ...
    "VolumeUtilization","StackHeight_m","EndToEndTime_s", ...
    "MinSupportRatio","UnstableBoxCount"];
requiredBuffer = ["CaseID","Method","ArrivalPattern","BufferCapacity", ...
    "RequiredBufferCapacity","ExecutionFeasible","RobotMoves", ...
    "TransferDistance_m"];
assertColumns(scale, requiredLayout, 'scale data');
assertColumns(typeData, requiredLayout, 'type-complexity data');
assertColumns(buffer, requiredBuffer, 'buffer data');

% Evidence layer 1: data provenance and pairing audit.
audit = [auditStudy(scale, "Scale", methods); ...
         auditStudy(typeData, "TypeComplexity", methods)];
writetable(audit, fullfile(outDir, '01_data_audit.csv'));
if any(~audit.CompletePairing)
    warning('Some cases do not contain exactly one result for every method.');
end

layout = [addStudy(scale, "Scale"); addStudy(typeData, "TypeComplexity")];
overall = summarizeMethods(layout, methods);
writetable(overall, fullfile(outDir, '02_overall_method_summary.csv'));

% Evidence layer 2: paired comparisons against the common Constructive seed.
pairedScale = pairedEvidence(scale, "Scale", methods(2:end));
pairedType = pairedEvidence(typeData, "TypeComplexity", methods(2:end));
paired = [pairedScale; pairedType];
writetable(paired, fullfile(outDir, '03_paired_method_evidence.csv'));

% Evidence layer 3: robustness over N and K.
scaleSummary = groupedMethodSummary(scale, 'N', methods);
typeSummary = groupedMethodSummary(typeData, 'K', methods);
writetable(scaleSummary, fullfile(outDir, '04_scale_summary.csv'));
writetable(typeSummary, fullfile(outDir, '05_type_complexity_summary.csv'));

% Evidence layer 4: buffer sensitivity and arrival-pattern robustness.
bufferSummary = summarizeBuffer(buffer);
writetable(bufferSummary, fullfile(outDir, '06_buffer_summary.csv'));

makeQualityTimeFigure(overall, methods, fullfile(figDir, '01_quality_time_tradeoff.png'));
makePairedFigure(scale, typeData, methods, fullfile(figDir, '02_paired_utilization_gain.png'));
makeRobustnessFigure(scaleSummary, typeSummary, methods, ...
    fullfile(figDir, '03_scale_and_type_robustness.png'));
makeBufferFigure(bufferSummary, fullfile(figDir, '04_buffer_sensitivity.png'));

writeNarrative(fullfile(outDir, '07_evidence_chain_summary.txt'), ...
    audit, overall, paired, bufferSummary);

outputs = struct('outputDirectory', outDir, 'audit', audit, ...
    'overall', overall, 'paired', paired, 'scale', scaleSummary, ...
    'typeComplexity', typeSummary, 'buffer', bufferSummary);
fprintf('Thesis evidence package written to:\n%s\n', outDir);
end

function T = addStudy(T, study)
T.StudyEvidence = repmat(string(study), height(T), 1);
end

function assertColumns(T, required, label)
missing = setdiff(required, string(T.Properties.VariableNames));
if ~isempty(missing)
    error('EvidenceChain:MissingColumns', '%s missing columns: %s', ...
        label, strjoin(missing, ', '));
end
end

function A = auditStudy(T, study, methods)
cases = unique(string(T.CaseID));
complete = true(numel(cases),1);
duplicates = false(numel(cases),1);
for i = 1:numel(cases)
    rows = T(string(T.CaseID) == cases(i),:);
    counts = arrayfun(@(m) sum(string(rows.Method) == m), methods);
    complete(i) = all(counts == 1);
    duplicates(i) = any(counts > 1);
end
A = table(string(study), height(T), numel(cases), sum(complete), ...
    all(complete), any(duplicates), sum(T.PlanningSucceeded == 0), ...
    'VariableNames', {'Study','Rows','UniqueCases','CompleteCases', ...
    'CompletePairing','HasDuplicateMethodRows','PlanningFailures'});
end

function S = summarizeMethods(T, methods)
S = table;
for m = methods
    X = T(string(T.Method) == m,:);
    row = table(m, height(X), mean(X.PlanningSucceeded), ...
        mean(X.VolumeUtilization,'omitnan'), std(X.VolumeUtilization,'omitnan'), ...
        mean(X.StackHeight_m,'omitnan'), mean(X.EndToEndTime_s,'omitnan'), ...
        median(X.EndToEndTime_s,'omitnan'), mean(X.MinSupportRatio,'omitnan'), ...
        sum(X.UnstableBoxCount), ...
        'VariableNames', {'Method','Cases','SuccessRate','MeanUtilization', ...
        'SDUtilization','MeanHeight_m','MeanTime_s','MedianTime_s', ...
        'MeanMinSupport','TotalUnstableBoxes'});
    S = [S; row]; %#ok<AGROW>
end
end

function P = pairedEvidence(T, study, challengers)
P = table;
for challenger = challengers
    base = T(string(T.Method)=="Constructive",:);
    alt = T(string(T.Method)==challenger,:);
    [common, ib, ia] = intersect(string(base.CaseID), string(alt.CaseID), 'stable');
    du = alt.VolumeUtilization(ia) - base.VolumeUtilization(ib);
    dh = alt.StackHeight_m(ia) - base.StackHeight_m(ib);
    dt = alt.EndToEndTime_s(ia) - base.EndToEndTime_s(ib);
    [ciLow, ciHigh] = normalApproxMeanCI(du);
    pSign = exactSignP(du);
    tol = 1e-12;
    row = table(string(study), challenger, numel(common), ...
        mean(du,'omitnan'), median(du,'omitnan'), ciLow, ciHigh, ...
        sum(du>tol), sum(abs(du)<=tol), sum(du<-tol), pSign, ...
        mean(dh,'omitnan'), mean(dt,'omitnan'), ...
        'VariableNames', {'Study','Challenger','Pairs', ...
        'MeanDeltaUtilization','MedianDeltaUtilization','NormalApproxMeanCI95Low', ...
        'NormalApproxMeanCI95High','Wins','Ties','Losses','ExactSignP', ...
        'MeanDeltaHeight_m','MeanDeltaTime_s'});
    P = [P; row]; %#ok<AGROW>
end
end

function [lo, hi] = normalApproxMeanCI(x)
x = x(isfinite(x));
n = numel(x);
if n < 2
    lo = NaN; hi = NaN; return
end
halfWidth = 1.96 * std(x) / sqrt(n);
lo = mean(x) - halfWidth;
hi = mean(x) + halfWidth;
end

function p = exactSignP(delta)
% Two-sided exact sign test, excluding ties; no Statistics Toolbox needed.
delta = delta(isfinite(delta) & delta ~= 0);
n = numel(delta);
if n == 0, p = 1; return; end
k = min(sum(delta > 0), sum(delta < 0));
p = min(1, 2 * sum(arrayfun(@(j) nchoosek(n,j), 0:k)) / 2^n);
end

function S = groupedMethodSummary(T, groupName, methods)
groups = unique(T.(groupName));
S = table;
for g = reshape(groups,1,[])
    for m = methods
        X = T(T.(groupName)==g & string(T.Method)==m,:);
        row = table(g, m, height(X), mean(X.VolumeUtilization,'omitnan'), ...
            std(X.VolumeUtilization,'omitnan'), mean(X.StackHeight_m,'omitnan'), ...
            mean(X.EndToEndTime_s,'omitnan'), ...
            'VariableNames', {groupName,'Method','Cases','MeanUtilization', ...
            'SDUtilization','MeanHeight_m','MeanTime_s'});
        S = [S; row]; %#ok<AGROW>
    end
end
end

function S = summarizeBuffer(B)
patterns = unique(string(B.ArrivalPattern), 'stable');
capacities = unique(B.BufferCapacity);
S = table;
for p = reshape(patterns,1,[])
    for c = reshape(capacities,1,[])
        X = B(string(B.ArrivalPattern)==p & B.BufferCapacity==c,:);
        if isempty(X), continue; end
        row = table(p, c, height(X), mean(X.ExecutionFeasible,'omitnan'), ...
            mean(X.RequiredBufferCapacity,'omitnan'), ...
            median(X.RequiredBufferCapacity,'omitnan'), ...
            mean(X.RobotMoves,'omitnan'), mean(X.TransferDistance_m,'omitnan'), ...
            'VariableNames', {'ArrivalPattern','BufferCapacity','Cases', ...
            'FeasibilityRate','MeanRequiredCapacity','MedianRequiredCapacity', ...
            'MeanRobotMoves','MeanTransferDistance_m'});
        S = [S; row]; %#ok<AGROW>
    end
end
end

function makeQualityTimeFigure(S, ~, file)
f = figure('Visible','off','Color','w','Position',[100 100 850 520]);
scatter(S.MeanTime_s, 100*S.MeanUtilization, 90, 'filled'); grid on
for i=1:height(S)
    text(S.MeanTime_s(i),100*S.MeanUtilization(i),"  "+S.Method(i), ...
        'FontSize',10,'VerticalAlignment','middle');
end
xlabel('Mean end-to-end planning time (s)'); ylabel('Mean volume utilization (%)');
title('Evidence 1: solution quality versus computational cost');
exportgraphics(f,file,'Resolution',200); close(f)
end

function makePairedFigure(scale, typeData, methods, file)
f = figure('Visible','off','Color','w','Position',[100 100 950 500]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
datasets = {scale,typeData}; titles = {'Scale study','Type-complexity study'};
for d=1:2
    nexttile; hold on; yline(0,'k--');
    base = datasets{d}(string(datasets{d}.Method)=="Constructive",:);
    for j=2:numel(methods)
        alt = datasets{d}(string(datasets{d}.Method)==methods(j),:);
        [~,ib,ia] = intersect(string(base.CaseID),string(alt.CaseID),'stable');
        delta = 100*(alt.VolumeUtilization(ia)-base.VolumeUtilization(ib));
        scatter(repmat(j-1,numel(delta),1),delta,18,'filled','MarkerFaceAlpha',0.35);
        plot(j-1,mean(delta),'kd','MarkerFaceColor','y','MarkerSize',8);
    end
    xlim([0.5 2.5]); xticks([1 2]); xticklabels(methods(2:end));
    ylabel('Paired utilization gain (percentage points)'); title(titles{d}); grid on
end
exportgraphics(f,file,'Resolution',200); close(f)
end

function makeRobustnessFigure(scaleS, typeS, methods, file)
f = figure('Visible','off','Color','w','Position',[100 100 1000 500]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on
for m=methods
    X=scaleS(string(scaleS.Method)==m,:); plot(X.N,100*X.MeanUtilization,'-o','DisplayName',m);
end
xlabel('Number of boxes, N'); ylabel('Mean utilization (%)'); title('Robustness over problem scale'); grid on; legend('Location','best')
nexttile; hold on
for m=methods
    X=typeS(string(typeS.Method)==m,:); plot(X.K,100*X.MeanUtilization,'-o','DisplayName',m);
end
xlabel('Number of box types, K'); ylabel('Mean utilization (%)'); title('Robustness over type complexity'); grid on; legend('Location','best')
exportgraphics(f,file,'Resolution',200); close(f)
end

function makeBufferFigure(S, file)
f = figure('Visible','off','Color','w','Position',[100 100 900 520]); hold on
patterns=unique(S.ArrivalPattern,'stable');
for p=reshape(patterns,1,[])
    X=S(S.ArrivalPattern==p,:); [~,idx]=sort(X.BufferCapacity); X=X(idx,:);
    plot(X.BufferCapacity,100*X.FeasibilityRate,'-o','DisplayName',p);
end
xlabel('Available buffer capacity'); ylabel('Execution feasibility (%)');
title('Evidence 4: buffer-capacity sensitivity by arrival pattern');
grid on; legend('Location','bestoutside'); ylim([-2 102]);
exportgraphics(f,file,'Resolution',200); close(f)
end

function writeNarrative(file, audit, overall, paired, bufferSummary)
fid=fopen(file,'w','n','UTF-8');
assert(fid>=0,'Could not create narrative output.'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'THESIS EVIDENCE CHAIN\n');
fprintf(fid,'Generated: %s\n\n',char(datetime('now')));
fprintf(fid,'Claim 1 - Experimental comparability\n');
fprintf(fid,'The scale and type-complexity studies contain %d unique paired cases and %d planning failures.\n', ...
    sum(audit.UniqueCases),sum(audit.PlanningFailures));
fprintf(fid,'Complete pairing across Constructive, CS-GA and CS-SA: %s.\n\n', ...
    string(all(audit.CompletePairing)));
fprintf(fid,'Claim 2 - Quality/time trade-off\n');
for i=1:height(overall)
    fprintf(fid,'%s: utilization %.3f%%, height %.3f m, mean time %.3f s, success %.1f%%.\n', ...
        overall.Method(i),100*overall.MeanUtilization(i),overall.MeanHeight_m(i), ...
        overall.MeanTime_s(i),100*overall.SuccessRate(i));
end
fprintf(fid,'\nClaim 3 - Search contribution relative to the common Constructive baseline\n');
for i=1:height(paired)
    fprintf(fid,'%s / %s: mean utilization delta %.3f percentage points; wins/ties/losses %d/%d/%d; exact sign p=%.4g; mean time delta %.3f s.\n', ...
        paired.Study(i),paired.Challenger(i),100*paired.MeanDeltaUtilization(i), ...
        paired.Wins(i),paired.Ties(i),paired.Losses(i),paired.ExactSignP(i),paired.MeanDeltaTime_s(i));
end
fprintf(fid,'\nClaim 4 - Buffer sensitivity\n');
patterns=unique(bufferSummary.ArrivalPattern,'stable');
for p=reshape(patterns,1,[])
    X=bufferSummary(bufferSummary.ArrivalPattern==p,:);
    feasible=X(X.FeasibilityRate>=0.999,:);
    if isempty(feasible), threshold='not reached'; else, threshold=string(min(feasible.BufferCapacity)); end
    fprintf(fid,'%s: smallest tested capacity with 100%% feasibility = %s.\n',p,threshold);
end
fprintf(fid,'\nDefensible conclusion\n');
fprintf(fid,['The completed experiments support a quality-time trade-off claim, not a universal superiority claim. ', ...
    'Constructive is the fast feasible baseline; CS-GA and CS-SA are assessed by paired gains on identical CaseIDs; ', ...
    'scale and type studies establish robustness; buffer curves quantify when resequencing becomes executable.\n']);
end
