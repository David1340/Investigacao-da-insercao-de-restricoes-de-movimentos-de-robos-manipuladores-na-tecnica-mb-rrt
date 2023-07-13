function [dists] = distancias2(A,B,V)
%distancias2 Summary of this function goes here
%   Detailed explanation goes here
dists = zeros(1,size(B,2));
for i = 1:length(dists)
  B_projetado = proj_ponto_plano(V(:,i),A,B(:,i));
  dists(i) = sqrt(sum((A - B_projetado).^2));
end

end

