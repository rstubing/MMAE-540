function [F] = mmae540_fatt(d, zeta, O)
%cutoff parameter, d
%weighting parameter, zeta
%Oi(q) - Oi(qf), O (column vector)
%ex: F = ffatt(d, zeta, O)

if norm(O) < d
    F = -zeta*O;
else
    F = (-d*zeta*O)/norm(O);
end