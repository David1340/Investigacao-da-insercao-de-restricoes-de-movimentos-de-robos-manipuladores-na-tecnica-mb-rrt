function [d] = distancias(A,B)
% Calcula a distância euclidiana entre um ponto A e um conjunto de pontos
%B
d = zeros(1,size(B,2));
for i = 1:length(d)
  d(i) = sqrt(sum((A - B(:,i)).^2));
end

end

