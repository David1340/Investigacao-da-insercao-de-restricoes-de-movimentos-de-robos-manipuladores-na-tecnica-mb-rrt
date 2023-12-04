function [p_proj] = proj_ponto_plano(n,p0,p)
%distancias2 Summary of this function goes here
%   Detailed explanation goes here
d = -n'*p0; %produto escalar
alpha = (-d - n'*p)/(n(1)^2 + n(2)^2 + n(3)^2);
p_proj = p + alpha*n;
end