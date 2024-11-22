clc
clear
close all

%% Configs
h = 0.025;
L = 0.075;
L2 = 0.75*L;
radius = 0.005;
color = [.1 .1 .1];
a = 0.005;
%% centros
c0 = [0;0;0]; %J_{i-2}
th = pi/4;
c1 = [L*cos(th);L*sin(th);0];%
th2 = pi/2;
c2 = [L*cos(th2);L*sin(th2);0];%
%% J_{i-2}
A2 = zeros(4);
A2(1:3,end) = c0;
A2(end,end) = 1;
A2(1:3,1:3) = axang2rotm([1,1,0,0]);

j2 = plot_junta_revolucao(A2,[0;0;-h/2],'z',h,radius,color);
hold on
t2 = text(c0(1)+a,c0(2)+a,c0(3)+a,'J_{i-2}','FontSize',18);

th = 0:0.1:2.1*pi;
x = L*cos(th);
y = L*sin(th);
z = 0*th;
pontos = [x;y;z;ones(1,length(x))];
pontos = A2*pontos;
% plot3(pontos(1,:),pontos(2,:),pontos(3,:),'.','color','k');
% elo 1
ponto = pontos(:,30);
e2 = plot3([c0(1),c1(1)],[c0(2),c1(2)],...
  [c0(3),c1(3)],'color',color,'LineWidth',3);
e3 = plot3([c0(1),c2(1)],[c0(2),c2(2)],...
  [c0(3),c2(3)],'color',color,'LineWidth',3);
c = c0;
c(3) = c(3)-0.02;
plot_esfera(c,radius/2,color);
c(3) = c(3)-0.015;
plot_esfera(c,radius/2,color);
c(3) = c(3)-0.015;
plot_esfera(c,radius/2,color);
%% J_{i-1}^a

A = zeros(4);
A(1:3,end) = c1;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([1,1,0,pi/2]);

j1 = plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal

t1 = text(c1(1)+a,c1(2)+a,c1(3)+a,'J_{i-1}^a','FontSize',18);

x = L2*cos(th);
y = L2*sin(th);
z = 0*th;
pontos = [x;y;z;ones(1,length(x))];
pontos = A*pontos;
ponto = pontos(:,30);
e1 = plot3([c1(1),ponto(1)],[c1(2),ponto(2)],...
  [c1(3),ponto(3)],'color',color,'LineWidth',3);
scatter3(ponto(1),ponto(2),ponto(3),'k','filled')
plot3(pontos(1,:),pontos(2,:),pontos(3,:),'.','color','k');
%% junta J_{i-1}^b
A = zeros(4);
A(1:3,end) = c2;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([0,1,0,pi/2]);

j1 = plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal

t1 = text(c2(1)+a,c2(2)+a,c2(3)+a,'J_{i-1}^b','FontSize',18);

x = L2*cos(th);
y = L2*sin(th);
z = 0*th;
pontos = [x;y;z;ones(1,length(x))];
pontos = A*pontos;
ponto = pontos(:,30);
e1 = plot3([c2(1),ponto(1)],[c2(2),ponto(2)],...
  [c2(3),ponto(3)],'color',color,'LineWidth',3);
scatter3(ponto(1),ponto(2),ponto(3),'k','filled')
plot3(pontos(1,:),pontos(2,:),pontos(3,:),'.','color','k');
%% Xrand
posD = [0.08;0.08;0.15];
p_Xgoal = scatter3(posD(1),posD(2),posD(3),'b','filled');
text(posD(1)-2*a,posD(2)-2*a,posD(3)+2*a,'x_{goal}','FontSize',14,...
  'FontWeight', 'bold','color','blue')

radius2 = 0.025;
color2 = [.5 .5 .5];
esfera = plot_esfera(posD,radius2,color2,0.3);

th = 2*pi*rand(1,1);
fi = 2*pi*rand(1,1);
Xrand  = [radius2*sin(fi)*cos(th);
  radius2*sin(fi)*sin(th);
  radius2*cos(fi)]; %ponto aleatório na esfera
Xrand = Xrand + posD; %desloca o centro da esfera
p_Xrand = scatter3(Xrand(1),Xrand(2),Xrand(3),'r','filled');
text(Xrand(1)-a,Xrand(2)-a,Xrand(3)+2*a,'x_{rand}','FontSize',14,...
  'FontWeight', 'bold','color','red')
% pause()
%% Xnew
% delete(esfera)
% v = (Xrand - center)/(norm(Xrand - center));
% center2 = center + v*L;


% j2 = plot_esfera(center2,radius,color);
% e1 = plot3([center(1),center2(1)],[center(2),center2(2)],...
%   [center(3),center2(3)],'color',color,'LineWidth',3);
% reta = plot3([center(1),Xrand(1)],[center(2),Xrand(2)],...
%   [center(3),Xrand(3)],'-.','color',color,'LineWidth',2);
% t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J_{i}','FontSize',18);

% esfera = plot_esfera(Xrand,radius2,[0.1,0.1,0.1],0.1);