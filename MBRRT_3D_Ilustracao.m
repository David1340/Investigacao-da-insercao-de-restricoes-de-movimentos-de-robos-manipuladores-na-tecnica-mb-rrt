%% MB-RRT Ilustração para discutir com os professores
clc
clear
close all
rng(2);
%% Parâmetros do robô
raio_junta = 0.025;
d = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;1;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 7; %número de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunferências em volta do destino
r = [r(end-1:-1:1),0];
%% configuração do experimento
posD = 2*[0.075;0.02;0.1]; %posição desejada
Xgoal = posD;
q = zeros(n,1); %configuração inicial
K = 1000; %número máximo de iteração
Xrands = [];
%% Inicilização da MB-RRT
erro = Inf; %erro inicial
P = root; %árvore
Xnew = d(1)*P(4:6,1) + P(1:3,1);
P = [P, [Xnew ; zeros(3,1);2;1]];

%% MB-RRT
k = 1
for i = 2:n
  th = rand(1,1);
  fi = rand(1,1);
  if(i == 2 | i == 4) %Se for Hinge
    raio = r(i+1); % comprimento da cadeia entre a junta superior até o efetuador
    Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
      raio*sin(2*pi*fi)*sin(2*pi*th);
      raio*cos(2*pi*fi)]; %ponto aleatório na esfera
    Xrand = Xrand + posD; %desloca o centro da esfera
    Xrands = [Xrands Xrand];

    pos = find(P(7,:) == i); %encontra as colunas dos nós de índice i
    p = P(1:3,pos);
    [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - (d(i)+d(i+1)))); %nós mais próximo
    p = p(1:3,pos2);
    v = (Xrand - p)/norm(Xrand - p);
    Xnew1 = d(i)*v + p;
    Xnew2 = d(i+1)*v + Xnew1;
     
    P = [P, [Xnew1;v;i+1;pos(pos2)]];
    
    P = [P, [Xnew2;zeros(3,1);i+2;size(P,2)]];
  elseif(i == 6)
    
    raio = r(i); % comprimento da cadeia entre a junta superior até o efetuador
    Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
      raio*sin(2*pi*fi)*sin(2*pi*th);
      raio*cos(2*pi*fi)]; %ponto aleatório na esfera
    Xrand = Xrand + posD; %desloca o centro da esfera
    Xrands = [Xrands Xrand];
    pos = find(P(7,:) == i); %encontra as colunas dos nós de índice i
    p = P(1:3,pos);
    [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - d(i)));
    
    p = p(:,pos2);
    
    v = (Xrand - p)/norm(Xrand - p);
    Xnew = d(i)*v + p;
    P = [P, [Xnew;zeros(3,1);i+1;pos(pos2)]];
    
  elseif(i ==7)
    Xrand = posD;
    Xrands = [Xrands Xrand];
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
    Xrand = proj_ponto_plano(normal,P(1:3,end),Xrand);
    %% MB-RRT padrão
    p = P(1:3,end);
    v = (Xrand - p)/norm(Xrand - p);
    Xnew = 0.075*v + p;
    P = [P, [Xnew;zeros(3,1);i+1;size(P,2)]];
  end
  
  if(i == n)
    erro = sqrt(sum((posD -P(1:3,end)).^2));
  end
end

k
erro

p0 = P(:,end);
P2 = p0;

for i = 1:n
  p0 = P(:,p0(8));
  P2 = [P2 p0];
end

P2 = P2(1:3,end:-1:1);
q = angulos2(P2);
[p,juntas] = cinematica_direta(q);
P = [juntas(1:3,:) p(1:3)];

%% Inicialização do plot
figure()
pd = plot3(posD(1),posD(2),posD(3),'*','linewidth',1); %plot da posição desejada
hold on
grid on
title('MB-RRT (1ª iteração)')
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([-0.4 0.6])

%plot da base
pb = plot_junta_revolucao(eye(4),root(1:3),'z',0.025,raio_junta);
% axis equal
xlabel('x')
ylabel('y')
zlabel('Z')
color = ['b','g','r','c','m','y','k']; %diferentes cores
legend([pd pb],'destino','base')
pause()
[DH_d,DH_a,DH_alpha,DH_theta,DH_p_n] = getDH_paramaters(q);
%% Junta 1
T1 = matriz_homogenea(DH_d(1),DH_a(1),DH_alpha(1),DH_theta(1));
p1_1 = [0;-0.05;0;1];
p1 = plot_junta_revolucao(T1,p1_1,'y',0.05,raio_junta,color(1),-0.025*0.5);
legend([pd pb p1],'destino','base','j1','autoupdate','off')
pause()
%% esfera1
esf = plot_esfera(posD,r(3),'r');
pause()
xrand = scatter3(Xrands(1,1),Xrands(2,1),Xrands(3,1),'k','filled','linewidth',1);
pause()
delete(esf)
%% Junta 2
A2 = matriz_homogenea(DH_d(2),DH_a(2),DH_alpha(2),DH_theta(2));
T2 = T1*A2;
p2_2 = [0;0;0;1];
p2 = plot_junta_revolucao(T2,p2_2,'y',0.025,raio_junta,color(2));
legend([pd pb p1 p2],'destino','base','j1','j2','autoupdate','off')
pause()
%% Junta 3
A3 = matriz_homogenea(DH_d(3),DH_a(3),DH_alpha(3),DH_theta(3));
T3 = T2*A3;
p3_3 = [0;-0.075;0;1];
p3 = plot_junta_revolucao(T3,p3_3,'y',0.05,raio_junta,color(3));
legend([pd pb p1 p2 p3],'destino','base','j1','j2','j3','autoupdate','off')
pause()
%% esfera 2
delete(xrand)
esf = plot_esfera(posD,r(5),'r');
pause()
xrand = scatter3(Xrands(1,2),Xrands(2,2),Xrands(3,2),'k','filled','linewidth',1);
pause()
delete(esf)
%% Junta 4
A4 = matriz_homogenea(DH_d(4),DH_a(4),DH_alpha(4),DH_theta(4));
T4 = T3*A4;
p4_4 = [0;0;0;1];
p4 = plot_junta_revolucao(T4,p4_4,'y',0.025,raio_junta,color(4));
legend([pd pd p1 p2 p3 p4],'destino','base','j1','j2','j3','j4','autoupdate','off')
pause()
%% Junta 5
A5 = matriz_homogenea(DH_d(5),DH_a(5),DH_alpha(5),DH_theta(5));
T5 = T4*A5;
p5_5 = [0;-0.075;0;1];
p5 = plot_junta_revolucao(T5,p5_5,'y',0.05,raio_junta,color(5));
legend([pd pb p1 p2 p3 p4 p5],...
  'destino','base','j1','j2','j3','j4','j5','autoupdate','off')
pause()
%% esfera 3
delete(xrand)
esf = plot_esfera(posD,r(6),'r');
pause()
xrand = scatter3(Xrands(1,3),Xrands(2,3),Xrands(3,3),'k','filled','linewidth',1);
pause()
delete(esf)
%% Junta 6
A6 = matriz_homogenea(DH_d(6),DH_a(6),DH_alpha(6),DH_theta(6));
T6 = T5*A6;
p6_6 = [-0.075;0;0;1];
p6a = plot_junta_revolucao(T6,p6_6,'y',0.025,raio_junta,color(6));
p6b = plot_junta_revolucao(T6,p6_6,'x',0.5*0.025,raio_junta,color(6),1.5*0.025);
legend([pd pb p1 p2 p3 p4 p5 p6a],...
  'destino','base','j1','j2','j3','j4','j5','j6','autoupdate','off')
pause()
%% Junta 7
A7 = matriz_homogenea(DH_d(7),DH_a(7),DH_alpha(7),DH_theta(7));
T7 = T6*A7;
p7_7 = [0;0;0;1];
p7 = plot_junta_revolucao(T7,p7_7,'y',0.025,raio_junta,color(7));
legend([pd pb p1 p2 p3 p4 p5 p6a p7],...
  'destino','base','j1','j2','j3','j4','j5','j6','j7','autoupdate','off')
pause()

%% Esfera 4 (o próximo destino)
delete(xrand)
pause()

%% plot do plano
p7_0 = T7*p7_7;
p7_0 = p7_0(1:3);
p6_0 = T6*p6_6;
p6_0 = p6_0(1:3); 
p5_0 = T5*p5_5;
p5_0 = p5_0(1:3); 

v1 = p7_0 - p6_0;
v1 = v1/norm(v1);
v2 = p5_0 - p6_0;
v2 = v2/norm(v2);
v3 = cross(v2,v1);
v3 = v3/norm(v3);
normal = rotacionar_vetor(v3,v1,pi/2); %v3 em torno de v1
normal = normal/norm(normal);
d_plano = -normal'*P2(1:3,end-1);

[x y] = meshgrid(-0.2:0.05:0.2); % Generate x and y data
x = P2(1,end-1) +x;
y = P2(2,end-1) +y;
z = -1/normal(3)*(normal(1)*x + normal(2)*y + d_plano); % Solve for z data
plano = surf(x,y,z); %Plot the surface
pause()
%% Efetuador

% Definição das dimensões
w = 0.05; % largura do efetuador
h = 0.025; % altura do efetuador
L = 0.075; % comprimento do elo que liga a junta 7 ao efetuador

% Pontos do efetuador em forma de 'u'
% Topo esquerdo do u
e1_7 = [0; -w/2; L + h; 1];
% Base esquerda do u
e2_7 = [0; -w/2; L; 1];
% Base direita do u
e3_7 = [0; w/2; L; 1];
% Topo direito do u
e4_7 = [0; w/2; L + h; 1];

% Calculando os pontos no sistema de coordenadas 0
e1_0 = T7 * e1_7;
e2_0 = T7 * e2_7;
e3_0 = T7 * e3_7;
e4_0 = T7 * e4_7;

% Criando um vetor com os valores x dos pontos
x_vals = [e1_0(1,1), e2_0(1,1), e3_0(1,1), e4_0(1,1), e3_0(1,1)];

% Criando um vetor com os valores y dos pontos
y_vals = [e1_0(2,1), e2_0(2,1), e3_0(2,1), e4_0(2,1), e3_0(2,1)];

% Criando um vetor com os valores z dos pontos
z_vals = [e1_0(3,1), e2_0(3,1), e3_0(3,1), e4_0(3,1), e3_0(3,1)];

% Plotando o gráfico 3D
ef = plot_junta_revolucao(T7,p7_7,'z',0.04,raio_junta,color(2),0.025);
plot3(x_vals, y_vals, z_vals, color(2));
legend([pd pb p1 p2 p3 p4 p5 p6a p7 ef],...
  'destino','base','j1','j2','j3','j4','j5','j6','j7','efetuador')
pause()

delete(plano)
pause()