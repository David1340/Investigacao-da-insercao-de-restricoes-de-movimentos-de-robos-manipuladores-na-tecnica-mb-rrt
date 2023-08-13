function [q] = angulos2(p)
% Cálculo os ângulos a partir da especificação da posição das juntas
% para o manipulador 6dod
% Atualmente tem problema com pontos colineares
n = size(p,2)-1;
q = zeros(n,1);
y = [0;1;0];
z = [0;0;1];
alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
%% cálculo dos vetores de referência
V = zeros(3,n);
V(1:3,1) = z;
for i =2:n
  v = (p(1:3,i) - p(1:3,i-1))/norm(p(1:3,i) - p(1:3,i-1));
  V(1:3,i) = rotacionar_vetor(V(1:3,i-1),-v,alpha(i-1));
end

%% Cálculos dos vetores de ref e ângulos 
for i = 1:n
  %% Vref
  if(i == 1)
    vref = y;   
  else
    vref = p(:,i) - p(:,i-1);
    vref = vref/norm(vref);    
  end
  
  v = rotacionar_vetor(vref,V(:,i),pi/2);
  
  %% ângulo
  vaux = p(:,i+1) - p(:,i);
  vaux = vaux/norm(vaux);
  q(i) = acos(vaux'*vref);
  if(vaux'*v < 0)
    q(i) = -q(i);
  end
  
end
% [p2,juntas] = cinematica_direta(q);
% p2 = [juntas(1:3,:) p2(1:3)];
% if(sqrt((p2(:,end) - p(:,end)).^2) > 10^-4)
%   q(end) = -q(end);
% end
