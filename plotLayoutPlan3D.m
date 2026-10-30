function plotLayoutPlan3D(layoutPlan, palletDim, palletLoc)

figure; clf; hold on; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Mixed-Layer Pallet Layout with Conveyor Buffer');

Lp = palletDim(1); Wp = palletDim(2); Hp = palletDim(3);

px = palletLoc(1) - Lp/2;
py = palletLoc(2) - Wp/2;
pz = palletLoc(3) - Hp/2;

drawCuboid([px py pz],[Lp Wp Hp],[0.7 0.7 0.7],0.3);

if exist('palletMargin','var') && palletMargin > 0
    sx = palletLoc(1) - Lp/2 + palletMargin;
    sy = palletLoc(2) - Wp/2 + palletMargin;
    sz = palletLoc(3) + Hp/2 + 0.002;
    plot3([sx sx+Lp-2*palletMargin sx+Lp-2*palletMargin sx sx], ...
        [sy sy sy+Wp-2*palletMargin sy+Wp-2*palletMargin sy], ...
        [sz sz sz sz sz], '--', 'Color',[0 0.45 0.9], 'LineWidth',1.5);
end

for i = 1:numel(layoutPlan)
    if ~layoutPlan(i).isPlaced
        continue;
    end

    cx = layoutPlan(i).x;
    cy = layoutPlan(i).y;
    cz = layoutPlan(i).z;
    L  = layoutPlan(i).L;
    W  = layoutPlan(i).W;
    H  = layoutPlan(i).H;

    x = cx - L/2;
    y = cy - W/2;
    z = cz - H/2;

    faceColor = colorForBox(layoutPlan(i));
    drawCuboid([x y z],[L W H],faceColor,0.88);
    if isfield(layoutPlan, 'sourceID') && layoutPlan(i).sourceID > 0
        labelID = layoutPlan(i).sourceID;
    else
        labelID = layoutPlan(i).ID;
    end
    text(cx,cy,cz+H/2+0.01,sprintf('%d',labelID), ...
        'HorizontalAlignment','center','FontSize',9);
end

cfg = palletTaskConfig();
bp=cfg.bufferPlatformCenterXY-[cfg.bufferPlatformDim(1:2)/2];
drawCuboid([bp cfg.bufferSurfaceZ-cfg.bufferPlatformDim(3)], ...
    cfg.bufferPlatformDim,[0.18 0.45 0.72],0.35);
for slot = 1:cfg.bufferCapacity
    col = mod(slot-1,cfg.bufferGrid(2));
    row = floor((slot-1)/cfg.bufferGrid(2));
    bx = cfg.bufferOriginXY(1)+col*cfg.bufferPitch(1);
    by = cfg.bufferOriginXY(2)-row*cfg.bufferPitch(2);
    text(bx,by,cfg.bufferSurfaceZ+0.02,sprintf('B%d',slot), ...
        'HorizontalAlignment','center','Color',[0.05 0.2 0.45]);
end
xAll = [palletLoc(1)-Lp/2-0.1, palletLoc(1)+Lp/2+0.1, ...
    bp(1)-0.1, bp(1)+cfg.bufferPlatformDim(1)+0.1];
yAll = [palletLoc(2)-Wp/2-0.1, palletLoc(2)+Wp/2+0.1, ...
    bp(2)-0.1, bp(2)+cfg.bufferPlatformDim(2)+0.1];
xlim([min(xAll) max(xAll)]);
ylim([min(yAll) max(yAll)]);
zmax = max([layoutPlan.z] + [layoutPlan.H]/2);
zlim([0, max(zmax,cfg.bufferSurfaceZ)+0.25]);

end

function c = colorForBox(box)
if isfield(box,'type')
    switch upper(char(box.type))
        case 'A', c=[0.86 0.47 0.12];
        case 'B', c=[0.22 0.55 0.82];
        case 'C', c=[0.30 0.68 0.38];
        otherwise, c=[0.65 0.55 0.78];
    end
else
    c=[0.70 0.55 0.25];
end
end

function drawCuboid(origin,dims,faceColor,faceAlpha)
x = origin(1); y = origin(2); z = origin(3);
L = dims(1);   W = dims(2);   H = dims(3);

V = [ ...
    x   y   z;
    x+L y   z;
    x+L y+W z;
    x   y+W z;
    x   y   z+H;
    x+L y   z+H;
    x+L y+W z+H;
    x   y+W z+H];

F = [ ...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];

patch('Vertices',V,'Faces',F, ...
      'FaceColor',faceColor, ...
      'FaceAlpha',faceAlpha, ...
      'EdgeColor','k','LineWidth',1.0);
end
