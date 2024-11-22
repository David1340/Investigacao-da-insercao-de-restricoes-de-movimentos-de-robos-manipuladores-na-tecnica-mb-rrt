clc
clear
close all
rng(5)

%% Setup
h = 0.025;
L = 0.075;
radius = 0.005;
color = [.1 .1 .1];
a = 0.005;
%% J_{i-2}
c0 = [0;0;0];
R1 = axang2rotm([0,0,1,pi/2]);
R2 = axang2rotm([1,1,0,pi/2])*axang2rotm([0,0,1,pi/4]);
c1 = R1*[L;0;0];
c2 = R2*[L;0;0];
J0 = plot_esfera(c0,1.5*radius,color,1);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal
xlabel("x")
ylabel("y")
zlabel("z")
t0 = text(c0(1)+a,c0(2)+a,c0(3)+a,'J_{i-2}','FontSize',18);

A1 = eye(4);
z1 = (c1 - c0)/norm(c1 - c0);
y1 = [1;0;0];
x1 = cross(y1,z1);
A1(1:3,1:3) = [x1,y1,z1];
A1(1:3,4) = c1;
J1 = plot_junta_revolucao(A1,[0;0;-h/2],'z',h,radius,color);
t1 = text(c1(1)+a,c1(2)+a,c1(3)+a,'J_{i-1}^a','FontSize',18);

A2 = eye(4);
z2 = (c2 - c0)/norm(c2 - c0);
y2 = [1;0;0];
x2 = cross(y2,z2);
A2(1:3,1:3) = [x2,y2,z2];
A2(1:3,4) = c2;
J2 = plot_junta_revolucao(A2,[0;0;-h/2],'z',h,radius,color);
t2 = text(c2(1)+a,c2(2)+a,c2(3)+a,'J_{i-1}^b','FontSize',18);



plot3([c0(1),c1(1)],[c0(2),c1(2)],...
  [c0(3),c1(3)],'color',color,'LineWidth',3);

plot3([c0(1),c2(1)],[c0(2),c2(2)],...
  [c0(3),c2(3)],'color',color,'LineWidth',3);

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
%% J^a_{i-1} e J^a_{i}

center = zeros(3,1);

A = zeros(4);
A(1:3,end) = center;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([1,0,0,-pi/2]);

p_aux = A*[0;0;h/2;1];

j1 = plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);



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