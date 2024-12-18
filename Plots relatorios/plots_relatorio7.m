clc
clear
close all
rng(5)

%% J^a_{i-1} e J^a_{i}
h = 0.025;
L = 0.075;
center = zeros(3,1);
radius = 0.005;
color = [.1 .1 .1];
A = zeros(4);
A(1:3,end) = center;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([1,0,0,pi/4]);

p_aux = A*[0;0;h/2;1];

j1 = plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal

a = 0.005;
t1 = text(center(1)+a,center(2)+a,center(3)+a,'J^a_{i-1}','FontSize',18);

v1 = A(1:3,3);
Xnew_a = center + v1*0.075;
t2 = text(Xnew_a(1)+a,Xnew_a(2)+a,Xnew_a(3)+a,'J^a_{i}','FontSize',18);
es2 = plot_esfera(Xnew_a,1.5*radius,color,1);

plot3([center(1),Xnew_a(1)],[center(2),Xnew_a(2)],...
  [center(3),Xnew_a(3)],'color',color,'LineWidth',3);
%% J^b_{i-1} e J^b_{i}
center2 = [-0.1;0;0];

A2 = zeros(4);
A2(1:3,end) = center2;
A2(end,end) = 1;
A2(1:3,1:3) = axang2rotm([1,1,0,0]);

j2 = plot_junta_revolucao(A2,[0;0;-h/2],'z',h,radius,color);
t3 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J^b_{i-1}','FontSize',18);

v1 = A2(1:3,3);
Xnew_b = center2 + v1*0.075;
t4 = text(Xnew_b(1)+a,Xnew_b(2)+a,Xnew_b(3)+a,'J^b_{i}','FontSize',18);

J3 = plot_esfera(Xnew_b,1.5*radius,color,1);
plot3([center2(1),Xnew_b(1)],[center2(2),Xnew_b(2)],...
  [center2(3),Xnew_b(3)],'color',color,'LineWidth',3);
%% junta 3
center3 = [-0.1;0.04;0];

J3 = plot_esfera(center3,1.5*radius,color,1);
plot3([center3(1),center2(1)],[center3(2),center2(2)],...
  [center3(3),center2(3)],'color',color,'LineWidth',3);

J4 = plot_esfera(center3,1.5*radius,color,1);
plot3([center3(1),center(1)],[center3(2),center(2)],...
  [center3(3),center(3)],'color',color,'LineWidth',3);
%% Xrand
posD = [0.08;0.08;0.2];
p_Xgoal = scatter3(posD(1),posD(2),posD(3),'b','filled');
t5 = text(posD(1)-2*a,posD(2)-2*a,posD(3)+2*a,'x_{goal}','FontSize',14,...
  'FontWeight', 'bold','color','blue');

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
delete(p_Xgoal)
delete(es2)
delete(t5)
vy = (Xrand - Xnew_a)/norm((Xrand - Xnew_a));
Xnew_a2 = Xnew_a + vy*0.075;

vx = (center  - Xnew_a)/norm((center - Xnew_a));
vz = cross(vx,vy);

A3 = zeros(4,4);
A3(1:3,end) = Xnew_a;
A3(end,end) = 1;
A3(1:3,1:3) = [vx';vy';vz'];
plot_junta_revolucao(A3,[0;0;-h/2],'z',h,radius,color);
delete(J3);

plot_esfera(Xnew_a2,1.5*radius,color,1);
plot3([Xnew_a(1),Xnew_a2(1)],[Xnew_a(2),Xnew_a2(2)],...
  [Xnew_a(3),Xnew_a2(3)],'color',color,'LineWidth',3);
esfera = plot_esfera(Xrand,radius2,color2,0.3);
text(Xnew_a2(1)+a,Xnew_a2(2)+a,Xnew_a2(3)+a,'J^a_{i+1}','FontSize',18);
plot3([Xnew_a(1) Xrand(1)],[Xnew_a(2) Xrand(2)],[Xnew_a(3) Xrand(3)],...
  '--','color','k','LineWidth',2);