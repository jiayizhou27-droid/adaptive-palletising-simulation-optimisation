function [b,meta,typeTable] = generatePalletBenchmarkInstance(N,K,instanceClass,seed)
%GENERATEPALLETBENCHMARKINSTANCE Reproducible Martello-style pallet instance.
% Every generated carton is an upright cuboid, has footprint sides >=0.18 m,
% fixed height 0.15 m, and mass proportional to volume.
arguments
    N (1,1) double {mustBeInteger,mustBePositive}
    K (1,1) double {mustBeInteger,mustBePositive}
    instanceClass (1,1) string
    seed (1,1) double {mustBeInteger}
end

instanceClass = string(instanceClass);
[catalogue,proportions] = classCatalogue(K,instanceClass);
assert(size(catalogue,1)==K,'Catalogue does not contain K types.');
counts = allocateCounts(N,proportions);
typeIDs = repelem((1:K)',counts);
assert(numel(typeIDs)==N);
rng(seed,'twister');
typeIDs = typeIDs(randperm(N));

density = 300; % kg/m^3, calibrated to the current cardboard-carton table.
b.length = catalogue(typeIDs,1);
b.width = catalogue(typeIDs,2);
b.height = catalogue(typeIDs,3);
b.weight = density*prod(catalogue(typeIDs,1:3),2);
labels = string(cellstr(char(double('A')+(0:K-1))'));
b.type = labels(typeIDs);
b.length=b.length(:); b.width=b.width(:); b.height=b.height(:);
b.weight=b.weight(:); b.type=b.type(:);
b.sourceID = (1:N)';

volumes = b.length.*b.width.*b.height;
areas = b.length.*b.width;
p = counts/N;
positiveP = p(p>0);
meta = struct( ...
    'N',N,'K',K,'InstanceClass',instanceClass,'Seed',seed, ...
    'VolumeCV',std(volumes)/mean(volumes), ...
    'FootprintCV',std(areas)/mean(areas), ...
    'LengthCV',std(b.length)/mean(b.length), ...
    'WidthCV',std(b.width)/mean(b.width), ...
    'TypeEntropy',-sum(positiveP.*log(positiveP)), ...
    'MeanVolume_m3',mean(volumes), ...
    'MeanFootprint_m2',mean(areas), ...
    'TotalMass_kg',sum(b.weight));
typeTable = table(labels,catalogue(:,1),catalogue(:,2),catalogue(:,3), ...
    density*prod(catalogue(:,1:3),2),counts, ...
    'VariableNames',{'Type','L','W','H','Mass','Count'});
end

function [catalogue,p] = classCatalogue(K,c)
H=0.15;
switch lower(char(c))
    case {'balanced','balancedmixed'}
        base=[.30 .24;.27 .22;.23 .20;.20 .18];
        catalogue=resampleCatalogue(base,K,H);
        p=ones(K,1)/K;
    case {'typecount','mediumdiversity'}
        % Same end points and approximately the same mean footprint for each K.
        base=[linspace(.30,.20,K)' linspace(.24,.18,K)'];
        if K==1, base=[.25 .21]; end
        catalogue=[base repmat(H,K,1)];
        p=ones(K,1)/K;
    case 'lowdiversity'
        requireK4(K,c);
        catalogue=[.27 .22 H;.26 .21 H;.25 .20 H;.24 .19 H];
        p=ones(4,1)/4;
    case 'highdiversity'
        requireK4(K,c);
        catalogue=[.30 .24 H;.28 .21 H;.23 .19 H;.20 .18 H];
        p=ones(4,1)/4;
    case 'smalldominated'
        requireK4(K,c);
        catalogue=[.30 .24 H;.27 .22 H;.23 .20 H;.20 .18 H];
        p=[.10;.20;.30;.40];
    case 'largedominated'
        requireK4(K,c);
        catalogue=[.30 .24 H;.27 .22 H;.23 .20 H;.20 .18 H];
        p=[.40;.30;.20;.10];
    case 'complementary'
        requireK4(K,c);
        catalogue=[.30 .24 H;.30 .18 H;.20 .24 H;.20 .18 H];
        p=ones(4,1)/4;
    case 'noncomplementary'
        requireK4(K,c);
        catalogue=[.29 .23 H;.27 .21 H;.23 .19 H;.21 .18 H];
        p=ones(4,1)/4;
    otherwise
        error('Unknown instance class: %s',c);
end
assert(all(catalogue(:,1)>=.18) && all(catalogue(:,2)>=.18), ...
    'Generated footprint violates the 0.18 m minimum side.');
end

function catalogue=resampleCatalogue(base,K,H)
if K==size(base,1)
    xy=base;
elseif K==1
    xy=mean(base,1);
else
    t=linspace(1,size(base,1),K);
    xy=[interp1(1:size(base,1),base(:,1),t)' ...
        interp1(1:size(base,1),base(:,2),t)'];
end
catalogue=[xy repmat(H,K,1)];
end

function requireK4(K,c)
assert(K==4,'%s is defined for K=4.',c);
end

function counts=allocateCounts(N,p)
raw=N*p(:);
counts=floor(raw);
remainder=N-sum(counts);
[~,order]=sort(raw-counts,'descend');
counts(order(1:remainder))=counts(order(1:remainder))+1;
end
