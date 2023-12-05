%% MB-RRT for 7DOF manipulator
clc
clear
close all
rng(2);
%% Parâmetros do robô

d = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;1;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 7; %número de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunferências em volta do destino
r = [r(end-1:-1:1),0];
%% configuração do experimento
posD = 2*[0.075;0.02;0.075]; %posição desejada

Xgoal = posD;
q = zeros(n,1); %configuração inicial
K = 1000; %número máximo de iteração

%% Inicialização do plot
% figure()
% plot3(root(1),root(2),root(3),'d'); %plot da base
% hold on
% grid on
% plot3(posD(1),posD(2),posD(3),'*','linewidth',5); %plot da posição desejada
% legend('root','destino','AutoUpdate','off')
color = ['b','g','r','c','m','y','k']; %diferentes cores

%% Inicialzação da MB-RRT
erro = Inf; %erro inicial
P = root; %árvore
Xnew = d(1)*P(4:6,1) + P(1:3,1);
P = [P, [Xnew ; zeros(3,1);2;1]];

%% MB-RRT
% plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(1));
% axis equal
for k = 1:K
  for i = 2:n
     th = rand(1,1);
     fi = rand(1,1);
    if(i == 2 | i == 4) %Se for Hinge 
     raio = r(i+1); % comprimento da cadeia entre a junta superior até o efetuador
     Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
       raio*sin(2*pi*fi)*sin(2*pi*th);
       raio*cos(2*pi*fi)]; %ponto aleatório na esfera
     Xrand = Xrand + posD; %desloca o centro da esfera
     pos = find(P(7,:) == i); %encontra as colunas dos nós de índice i
     p = P(1:3,pos);
     [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - (d(i)+d(i+1)))); %nós mais próximo
     p = p(1:3,pos2);
     v = (Xrand - p)/norm(Xrand - p);
     Xnew1 = d(i)*v + p;
     Xnew2 = d(i+1)*v + Xnew1;
     P = [P, [Xnew1;v;i+1;pos(pos2)]];
%      plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i));
     P = [P, [Xnew2;zeros(3,1);i+2;size(P,2)]];
%      plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i+1));
    elseif(i == 6)
      
     raio = r(i); % comprimento da cadeia entre a junta superior até o efetuador
     Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
       raio*sin(2*pi*fi)*sin(2*pi*th);
       raio*cos(2*pi*fi)]; %ponto aleatório na esfera
     Xrand = Xrand + posD; %desloca o centro da esfera

     pos = find(P(7,:) == i); %encontra as colunas dos nós de índice i
     p = P(1:3,pos);
     [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - d(i)));

     p = p(:,pos2);
%      n = n(:,pos2);
%      Xrand = proj_ponto_plano(n,p,Xrand);

     v = (Xrand - p)/norm(Xrand - p);
     Xnew = d(i)*v + p;
     P = [P, [Xnew;zeros(3,1);i+1;pos(pos2)]];
%      plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i));
    
    elseif(i ==7)
      if(k == 63)
        oi = 1;
      end
      Xrand = posD;
      %% Projetanto ponto no plano
      A = P(:,P(8,end));
      B = P(:,A(8));
      v1 = P(1:3,end) - A(1:3);
      v1 = v1/norm(v1);
      v2 = B(1:3) - A(1:3);
      v2 = v2/norm(v2);
      v3 = cross(v2,v1); 
      v3 = v3/norm(v3);
      normal = rotacionar_vetor(v3,v1,pi/2); %v3 em torno de v1
      normal = normal/norm(normal);
      %d_plano = -normal'*P(1:3,end);
      %alpha = (-d_plano - normal'*Xrand);
      %Xrand = Xrand + alpha*normal;
      Xrand = proj_ponto_plano(normal,P(1:3,end),Xrand);
      %% MB-RRT padrão
      p = P(1:3,end);
      v = (Xrand - p)/norm(Xrand - p);
      Xnew = 0.075*v + p;
      P = [P, [Xnew;zeros(3,1);i+1;size(P,2)]];
%       plot3([P(1,end-1) P(1,end)],[P(2,end-1) P(2,end)],[P(3,end-1) P(3,end)],color(i)); 
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

figure()
plot3(root(1),root(2),root(3),'d'); %plot da base
hold on
grid on
plot3(posD(1),posD(2),posD(3),'*','linewidth',5); %plot da posição desejada

p0 = P(:,end);
P2 = p0;

for i = 1:n
  p0 = P(:,p0(8));
  P2 = [P2 p0];
  plot3([P2(1,end-1) P2(1,end)],[P2(2,end-1) P2(2,end)],[P2(3,end-1) P2(3,end)],color(i));
end
axis equal
xlabel('x')
ylabel('y')

P2 = P2(1:3,end:-1:1);
q = angulos(P2);
% q(end) = -q(end)
[p,juntas] = cinematica_direta(q);
P = [juntas(1:3,:) p(1:3)]
P2
for i = 2:n+1
  plot3([P(1,i-1) P(1,i)],[P(2,i-1) P(2,i)],[P(3,i-1) P(3,i)],color(i-1));
end

%% plot do plano
v1 = P2(1:3,end-1) - P2(1:3,end-2);
v1 = v1/norm(v1);
v2 = P2(1:3,end-3) - P2(1:3,end-2);
v2 = v2/norm(v2);
v3 = cross(v2,v1);
v3 = v3/norm(v3);
normal = rotacionar_vetor(v3,v1,pi/2); %v3 em torno de v1
normal = normal/norm(normal);
d_plano = -normal'*P2(1:3,end-1);

[x y] = meshgrid(-0.05:0.005:0.05); % Generate x and y data
x = P2(1,end-1) +x;
y = P2(2,end-1) +y;
z = -1/normal(3)*(normal(1)*x + normal(2)*y + d_plano); % Solve for z data
surf(x,y,z) %Plot the surface
% testes2