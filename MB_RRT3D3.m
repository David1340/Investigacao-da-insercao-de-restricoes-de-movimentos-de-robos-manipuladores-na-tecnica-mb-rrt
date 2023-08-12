%% MB-RRT for 4DOF manipulator
%a cada duas juntas
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
posD = [0.03;0.03;0.1]; %posi��o desejada

Xgoal = posD;
q = zeros(n,1); %configura��o inicial
K = 1000; %n�mero m�ximo de itera��o

%% Inicializa��o do plot
figure()
plot3(root(1),root(2),root(3),'d'); %plot da base
hold on
grid on
plot3(posD(1),posD(2),posD(3),'*','linewidth',5); %plot da posi��o desejada
legend('root','destino','AutoUpdate','off')
color = ['b','g','r','c','m','y','k']; %diferentes cores

%% Inicialza��o da MB-RRT
erro = Inf; %erro inicial
P = root; %�rvore

%% MB-RRT
% plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(1));
% axis equal
for k = 1:K
  for i = 1:n
     th = rand(1,1);
     fi = rand(1,1);
     if(i == 1 | i == 3) %Se for Hinge
       raio = r(i+1); % comprimento da cadeia entre a junta superior at� o efetuador
       Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
         raio*sin(2*pi*fi)*sin(2*pi*th);
         raio*cos(2*pi*fi)]; %ponto aleat�rio na esfera
       Xrand = Xrand + posD; %desloca o centro da esfera
       pos = find(P(7,:) == i-1); %encontra as colunas dos n�s de �ndice i-1
       p = P(1:6,pos);
       [valor pos2] = min(abs(distancias3(Xrand,p(1:3,:),p(4:6,:)) - d(i))); %n�s mais pr�ximo
%        [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - d(i) - d(i+1))); %n�s mais pr�ximo

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
%        scatter3(Xnew(1),Xnew(2),Xnew(3));
     end
    drawnow
    if(i == n)
      erro = sqrt(sum((posD -P(1:3,end)).^2));
    end
  end
    if(erro < 0.001)
      convergiu = 1;
      break
    end
end
k
erro

