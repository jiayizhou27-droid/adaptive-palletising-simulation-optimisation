function manifest = buildSimulinkValidationManifest()
%BUILDSIMULINKVALIDATIONMANIFEST Select 30 paired full-Sim3D validation cases.
% Five scale levels and five carton-type levels are represented.  For each
% level the same replicate is used by Constructive, CS-GA and CS-SA.  The
% first replicate for which all three methods fit the physical buffer is
% selected, so method comparisons remain paired and physically executable.

projectDir = fileparts(mfilename('fullpath'));
cfg = palletTaskConfig();
methods = ["Constructive","CS-GA","CS-SA"];
rows = cell(0,1);

for N = [20 40 60 80 100]
    [rep,files,required] = findPairedReplicate(projectDir,"Scale",N,4,methods,cfg);
    for m = 1:numel(methods)
        caseID = sprintf('Scale_N%03d_K04_R%02d',N,rep);
        rows{end+1,1} = table(string(caseID),"Scale",N,4,rep,methods(m), ...
            string(files(m)),required(m),cfg.bufferCapacity, ...
            'VariableNames',{'CaseID','Study','N','K','Replicate','Method', ...
            'PlanFile','RequiredBufferCapacity','PhysicalBufferCapacity'}); %#ok<AGROW>
    end
end

for K = [2 4 6 8 10]
    [rep,files,required] = findPairedReplicate(projectDir,"Type",40,K,methods,cfg);
    for m = 1:numel(methods)
        caseID = sprintf('Type_N040_K%02d_R%02d',K,rep);
        rows{end+1,1} = table(string(caseID),"TypeComplexity",40,K,rep,methods(m), ...
            string(files(m)),required(m),cfg.bufferCapacity, ...
            'VariableNames',{'CaseID','Study','N','K','Replicate','Method', ...
            'PlanFile','RequiredBufferCapacity','PhysicalBufferCapacity'}); %#ok<AGROW>
    end
end

manifest = vertcat(rows{:});
manifest.RunID = (1:height(manifest))';
manifest = movevars(manifest,'RunID','Before','CaseID');
outDir = fullfile(projectDir,'simulink_validation_30');
if ~isfolder(outDir), mkdir(outDir); end
writetable(manifest,fullfile(outDir,'simulink_validation_manifest.csv'));
save(fullfile(outDir,'simulink_validation_manifest.mat'),'manifest');
disp(manifest);
end

function [selectedRep,files,required] = findPairedReplicate(projectDir,prefix,N,K,methods,cfg)
selectedRep = 0; files = strings(1,numel(methods)); required = nan(1,numel(methods));
for rep = 1:10
    candidateFiles = strings(1,numel(methods)); candidateRequired = nan(1,numel(methods));
    valid = true;
    for m = 1:numel(methods)
        stem = sprintf('%s_N%03d_K%02d_R%02d_%s.mat',prefix,N,K,rep,methods(m));
        hit = dir(fullfile(projectDir,'scale_type_buffer_study_B25*','plans',stem));
        if isempty(hit), valid=false; break; end
        candidateFiles(m) = string(fullfile(hit(1).folder,hit(1).name));
        S = load(candidateFiles(m),'plan');
        if ~isfield(S,'plan') || isempty(S.plan), valid=false; break; end
        wide = cfg; wide.bufferCapacity=max(N,cfg.bufferCapacity);
        wide.bufferGrid=[ceil(wide.bufferCapacity/3) 3];
        try
            execution = buildBufferExecutionPlan(S.plan,wide);
            candidateRequired(m) = peakBuffer(execution);
        catch
            valid=false; break;
        end
        if candidateRequired(m)>cfg.bufferCapacity, valid=false; break; end
    end
    if valid
        selectedRep=rep; files=candidateFiles; required=candidateRequired; return;
    end
end
error('No paired %s N=%d K=%d replicate fits the %d-slot physical buffer.', ...
    prefix,N,K,cfg.bufferCapacity);
end

function peak = peakBuffer(execution)
occupied = 0; peak = 0;
for i=1:numel(execution)
    if strcmp(execution(i).action,'conveyor_to_buffer'), occupied=occupied+1; end
    if strcmp(execution(i).action,'buffer_to_pallet'), occupied=occupied-1; end
    peak=max(peak,occupied);
end
end
