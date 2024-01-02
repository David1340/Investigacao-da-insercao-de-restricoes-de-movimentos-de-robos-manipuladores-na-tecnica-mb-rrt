%% MB-RRT for 4DOF manipulator
clc
clear
close all
% rng(2);
%% Par�metros do rob�
alpha = [pi/2 ,-pi/2, pi/2, 0];
d = [0.075,0.075,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;0;0]; %x,y,z,eixo de rota��o, �ndice da cadeira, n� pai 
n = 4; %n�mero de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunfer�ncias em volta do destino
r = [r(end-1:-1:1),0];
%% configura��o do experimento
repeticoes = 1000;
K = 1000; %n�mero m�ximo de itera��o
ks = [];
for rep = 1:repeticoes
  rep
  q = -pi + pi*rand(4,1);
  [posD,juntas] = cinematica_direta2(q); %posi��o desejada
  posD = posD(1:3);
  Xgoal = posD;
  %% Inicialza��o da MB-RRT
  erro = Inf; %erro inicial
  P = root; %�rvore
  
  for k = 1:K
    for i = 1:n
      th = 2*pi*rand(1,1);
      fi = 2*pi*rand(1,1);
      if(i == 1 | i == 3) %Se for Hinge
        raio = r(i+1); % comprimento da cadeia entre a junta superior at� o efetuador
        Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
          raio*sin(2*pi*fi)*sin(2*pi*th);
          raio*cos(2*pi*fi)]; %ponto aleat�rio na esfera
        Xrand = Xrand + posD; %desloca o centro da esfera
        pos = find(P(7,:) == i-1); %encontra as colunas dos n�s de �ndice i-1
        p = P(1:6,pos);
        [valor pos2] = min(abs(distancias3(Xrand,p(1:3,:),p(4:6,:),d(i)) - d(i+1))); %n�s mais pr�ximo
%         [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - d(i) - d(i+1))); %n�s mais pr�ximo

        normal = p(4:6,pos2);
        p = p(1:3,pos2);    
        %% Projetanto ponto no plano
        Xrand2 = proj_ponto_plano(normal,p,Xrand);
        %% MB-RRT padr�o
        v = (Xrand2 - p)/norm(Xrand2 - p);
        Xnew = d(i)*v + p;
        p2 = p;
        %% Vnew
        Vnew = rotacionar_vetor(normal,-v,alpha(i)); %alpha1
        P = [P, [Xnew;Vnew;i;size(P,2)]];
        %%
        p = Xnew;
        normal = Vnew;
        v = (Xrand - p)/norm(Xrand - p);

        Vnew = rotacionar_vetor(normal,-v,alpha(i+1));
        Xnew = d(i+1)*v + p;
        P = [P, [Xnew;Vnew;i+1;size(P,2)]];
      end
      drawnow
      if(i == n)
        erro = sqrt(sum((posD -P(1:3,end)).^2));
      end
    end
    if(erro < 0.001)
      ks = [ks k];
      break
    end
  end
%   k
%   erro
end
convergencia = length(ks)/repeticoes * 100
media_de_iteracao = mean(ks)