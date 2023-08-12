function [dists] = distancias2(A,B,V)
%Calcula a distância euclidiana entre um ponto A e um conjunto de pontos
%B (projetados no planos definidos pleo vetores em V e pontos e em A)

dists = zeros(1,size(B,2));
for i = 1:length(dists)
  B_projetado = proj_ponto_plano(V(:,i),A,B(:,i));
  dists(i) = sqrt(sum((A - B_projetado).^2));
end

end

