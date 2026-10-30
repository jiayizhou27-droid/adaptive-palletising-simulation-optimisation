function [b,meta,typeTable,arrivalOrder]=generateIndependentGeometryInstance(N,K,seed)
%GENERATEINDEPENDENTGEOMETRYINSTANCE Geometry and arrival are independent.
% The seed changes type dimensions and type-count proportions. The returned
% box data is in a deterministic geometry order; arrivalOrder is separate
% and is used only by buffer/execution experiments.
arguments
    N (1,1) double {mustBeInteger,mustBePositive}
    K (1,1) double {mustBeInteger,mustBePositive}
    seed (1,1) double {mustBeInteger}
end
assert(N>=K,'N must be at least K.');
rng(seed,'twister');
H=0.15;
% Stratified random footprints preserve practical carton limits while
% making every seed a genuinely different geometric instance.
largeToSmall=linspace(1,0,K)';
L=0.205+0.105*largeToSmall+0.018*(rand(K,1)-0.5);
W=0.182+0.060*largeToSmall+0.014*(rand(K,1)-0.5);
L=max(0.20,min(0.32,L)); W=max(0.18,min(0.25,W));
swap=L<W; tmp=L(swap);L(swap)=W(swap);W(swap)=tmp;
catalogue=[round(L,3) round(W,3) repmat(H,K,1)];

weights=-log(max(rand(K,1),eps));
counts=ones(K,1);
remaining=N-K;
raw=remaining*weights/sum(weights);
counts=counts+floor(raw);
left=N-sum(counts);
[~,ix]=sort(raw-floor(raw),'descend');
counts(ix(1:left))=counts(ix(1:left))+1;

% Canonical layout order is size-based, not conveyor arrival order.
typeIDs=repelem((1:K)',counts);
[~,ord]=sort(prod(catalogue(typeIDs,1:2),2),'descend');
typeIDs=typeIDs(ord);
density=300;
b.length=catalogue(typeIDs,1);
b.width=catalogue(typeIDs,2);
b.height=catalogue(typeIDs,3);
b.weight=density*prod(catalogue(typeIDs,:),2);
labels="T"+string((1:K)');
b.type=labels(typeIDs);
b.sourceID=(1:N)';
arrivalOrder=randperm(N)';

areas=b.length.*b.width;volumes=areas.*b.height;
p=counts/N;
meta=struct('N',N,'K',K,'Seed',seed,'FootprintCV',std(areas)/mean(areas), ...
    'VolumeCV',std(volumes)/mean(volumes),'TypeEntropy',-sum(p.*log(p)), ...
    'LargestToSmallestFootprint',max(areas)/min(areas), ...
    'ArrivalOrderSeparated',true);
typeTable=table(labels,catalogue(:,1),catalogue(:,2),catalogue(:,3), ...
    density*prod(catalogue,2),counts, ...
    'VariableNames',{'Type','L','W','H','Mass','Count'});
end
