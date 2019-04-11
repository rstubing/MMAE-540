function [F] = mmae540_ffrep(rho_o, eta, O, xobst_n)

[d, xb, yb] = p_poly_dist(O(1,1), O(2,1), xobst_n(1,:), xobst_n(2,:));

if d < rho_o
    F = eta*((1/d) - (1/rho_o))*(1/(d^2)).*((O - [xb; yb])./(d));
elseif d >= rho_o
        F = zeros(2,1);
end
