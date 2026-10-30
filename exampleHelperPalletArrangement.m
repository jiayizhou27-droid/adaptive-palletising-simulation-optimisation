function goalLoc = exampleHelperPalletArrangement(id,palletDim,palletLoc,boxDim)
%EXAMPLEHELPERPALLETARRANGEMENT returns the adaptive layout target.
%
% The Simulink model still calls this helper from the RRT collision
% environment. Prefer the optimizer output in layoutPlan, and keep the
% original fixed-grid calculation as a fallback.

goalLoc = getLayoutPositionFromBase(id);
if any(isfinite(goalLoc))
    return;
end

boxDims = normalizeBoxDims(boxDim);
boxDimxy = 1.1*max(boxDims(1), boxDims(2));
nRows = max(1, floor(palletDim(1)/boxDimxy));
nCols = max(1, floor(palletDim(2)/boxDimxy));

cornerLocation = zeros(1,3);
cornerLocation(1) = palletLoc(1) + palletDim(1)/2;
cornerLocation(2) = palletLoc(2) + palletDim(2)/2;
cornerLocation(3) = palletLoc(3) + palletDim(3)/2;

nBoxesPerLayer = nRows*nCols;
layerID = ceil((id+1)/nBoxesPerLayer);
idInLayer = (id+1) - nBoxesPerLayer*(layerID - 1);

rowID =  ceil(idInLayer/nCols);
colID =  mod(idInLayer-1,nCols)+1;

goalLoc = zeros(1,3);
goalLoc(1) = cornerLocation(1) - (rowID-1)*boxDimxy - boxDimxy/2;
goalLoc(2) = cornerLocation(2) - (colID-1)*boxDimxy - boxDimxy/2;
goalLoc(3) = cornerLocation(3) + (layerID-1)*boxDims(3) + boxDims(3)/2;
goalLoc = double(goalLoc);
end

function pos = getLayoutPositionFromBase(id)
pos = [NaN NaN NaN];
try
    layoutPlan = evalin('base','layoutPlan');
    maxID = numel(layoutPlan);
    if maxID < 1
        return;
    end
    idx = mod(double(id), maxID) + 1;
    pos = double(layoutPlan(idx).pos);
catch
end
end

function dims = normalizeBoxDims(boxDim)
if isscalar(boxDim)
    dims = double([boxDim boxDim boxDim]);
else
    dims = double(boxDim(:)).';
    if numel(dims) < 3
        dims = [dims repmat(dims(end), 1, 3-numel(dims))];
    end
    dims = dims(1:3);
end
end
