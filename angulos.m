function [q] = angulos(p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% Atualmente tem problema com pontos colineares
n = size(p,2)-1;
q = zeros(n,1);
x = [1;0;0];

%% Eixos de atuação
V = zeros(3,n);
for i = 1:n
  if(i == 1 || i == 3 || i == 5)
    v = p(:,i+1) - p(:,i);
    V(:,i) = v/norm(v);    
  else
    v1 = p(:,i+1) - p(:,i);
    v1 = v1/norm(v1);
    v2 = p(:,i-1) - p(:,i);
    v2 = v2/norm(v2);
    V(:,i) = cross(v1,v2);
    V(:,i) = V(:,i)/norm(V(:,i));
  end
end
%% Cálculos dos vetores de ref e ângulos 
for i = 1:n
  %% Vref
  if(i == 1)
    vref = x;   
  elseif(i == n)
    vref = p(:,i) - p(:,i-1);
    vref = vref/norm(vref);    
  else
    vref = V(:,i-1);
    vref = vref/norm(vref);
  end
  
  v = rotacionar_vetor(vref,V(:,i),pi/2);
  
  %% ângulo
  if(i == n)
    vaux = p(:,i+1) - p(:,i);
    vaux = vaux/norm(vaux);
    q(i) = acos(vaux'*vref);
    if(vaux'*v < 0) 
      q(i) = -q(i);
    end
  elseif(i == n-1)
    vaux = p(:,i+1) - p(:,i);
    vaux = vaux/norm(vaux);
    q(i) = acos(vaux'*vref);
    if(vaux'*v < 0)
      q(i) = -q(i);
    end
  else
    q(i) = acos(V(:,i+1)'*vref);
    if(V(:,i+1)'*v < 0)
      q(i) = -q(i);
    end
  end
  
end

