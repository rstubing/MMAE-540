function [d xb yb] = p_poly_dist(x, y, xv, yv)

xv = xv(:);
yv = yv(:);
Nv = length(xv);
if ((xv(1) ~= xv(Nv)) || (yv(1) ~= yv(Nv)))
    xv = [xv ; xv(1)];
    yv = [yv ; yv(1)];
    Nv = Nv + 1;
end

A = -diff(yv);
B = diff(xv);
C = yv(2:end).*xv(1:end-1) - xv(2:end).*yv(1:end-1);

AB = 1./(A.^2 + B.^2);
vv = (A*x + B*y + C);
xp = x - (A.*AB).*vv;
yp = y - (B.*AB).*vv;

idx_x = (((xp >= xv(1:end-1)) & (xp <= xv(2:end))) | ((xp >= xv(2:end)) & (xp <= xv(1:end-1))));
idx_y = (((yp >= yv(1:end-1)) & (yp <= yv(2:end))) | ((yp >= yv(2:end)) & (yp <= yv(1:end-1))));
idx = idx_x & idx_y;

dv = sqrt((xv(1:end-1)-x).^2 + (yv(1:end-1)-y).^2);

if(~any(idx))
    [d II] = min(dv);
    xb = xv(II);
    yb = yv(II);
else
    xx = xp(idx);
    yy = yp(idx);
    dp = sqrt((xp(idx)-x).^2 + (yp(idx)-y).^2);
    ds = [dv; dp];
    [d II] = min(ds);
    if II > length(dv)
        xb = xx(II - length(dv));
        yb = yy(II - length(dv));
    elseif II < length(dv);
        xb = xv(II);
        yb = yv(II);
    end
end

if(inpolygon(x, y, xv, yv))
    d = -d;
end
