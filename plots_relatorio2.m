clc
clear
close all

%% junta 1
h = 0.025;
L = 0.075;
center = zeros(3,1);
radius = 0.005;
color = [.1 .1 .1];
A = zeros(4);
A(1:3,end) = center;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([1,1,0,pi/2]);

j1 = plot_junta_revolucao(A,zeros(3,1),'z',h,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal

a = 0.005;
t1 = text(center(1)+a,center(2)+a,center(3)+a,'J_{i-1}','FontSize',18);
th = 0:0.1:2.1*pi;
x = L*cos(th);
y = L*sin(th);
z = 0*th;
pontos = [x;y;z;ones(1,length(x))];
pontos = A*pontos;
plot3(pontos(1,:),pontos(2,:),pontos(3,:),'.','color','k');
%% junta 2
center2 = [-0.08;-0.04;0];

A2 = zeros(4);
A2(1:3,end) = center2;
A2(end,end) = 1;
A2(1:3,1:3) = axang2rotm([1,1,0,0]);

j2 = plot_junta_revolucao(A2,zeros(3,1),'z',h,radius,color);
t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J_{i-1}','FontSize',18);

x = L*cos(th);
y = L*sin(th);
z = 0*th;
pontos = [x;y;z;ones(1,length(x))];
pontos = A2*pontos;
plot3(pontos(1,:),pontos(2,:),pontos(3,:),'.','color','k');
%% elo 1
% e1 = plot3([center(1),center2(1)],[center(2),center2(2)],...
%   [center(3),center2(3)],'color',color,'LineWidth',3);

%% Xrand
posD = [0.08;0.08;0.2];
p_Xgoal = scatter3(posD(1),posD(2),posD(3),'b','filled');
text(posD(1)-2*a,posD(2)-2*a,posD(3)+2*a,'x_{goal}','FontSize',14,...
  'FontWeight', 'bold','color','blue')

radius2 = 0.05;
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
pause()
%% Xnew
delete(esfera)
v = (Xrand - center)/(norm(Xrand - center));
center2 = center + v*L;


j2 = plot_esfera(center2,radius,color);
e1 = plot3([center(1),center2(1)],[center(2),center2(2)],...
  [center(3),center2(3)],'color',color,'LineWidth',3);
reta = plot3([center(1),Xrand(1)],[center(2),Xrand(2)],...
  [center(3),Xrand(3)],'-.','color',color,'LineWidth',2);
t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J_{i}','FontSize',18);

esfera = plot_esfera(Xrand,radius2,[0.1,0.1,0.1],0.1);