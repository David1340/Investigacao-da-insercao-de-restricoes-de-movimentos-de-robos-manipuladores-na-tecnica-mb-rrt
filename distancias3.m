function [dists] = distancias3(A,B,V,d)
%Calcula a distância euclidiana entre um ponto B e um conjunto de pontos
%B (projetados no planos definidos pelos vetores em V e pontos e em A)

dists = zeros(1,size(B,2));
for i = 1:length(dists)
  B_projetado = proj_ponto_plano(V(:,i),A,B(:,i));
  v = B_projetado - A;
  v = v/norm(v);
  B_projetado = A + d*v;
  dists(i) = sqrt(sum((B(:,i) - B_projetado).^2));
end

end

