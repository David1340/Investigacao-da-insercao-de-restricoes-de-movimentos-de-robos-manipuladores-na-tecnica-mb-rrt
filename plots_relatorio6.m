clc
clear
close all
rng(5)

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

p_aux = A*[0;0;h/2;1];

j1 = plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.02    0.300])
axis equal

a = 0.005;
t1 = text(center(1)+a,center(2)+a,center(3)+a,'J^a_{i-1}','FontSize',18);

normal = A(1:3,3);
d_plano = -normal'*center;

[x z] = meshgrid(-0.05:0.005:0.25); % Generate x and y data
y = -1/normal(2)*(normal(1)*x + normal(3)*z + d_plano); % Solve for z data
plan1 = surf(x,y,z,'FaceColor',[0.1,0.1,0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

%% junta 2
% center2 = [-0.08;-0.04;0];
center2 = [-0.1;-0.04;0];

A2 = zeros(4);
A2(1:3,end) = center2;
A2(end,end) = 1;
A2(1:3,1:3) = axang2rotm([1,1,0,0]);

j2 = plot_junta_revolucao(A2,[0;0;-h/2],'z',h,radius,color);
t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J^b_{i-1}','FontSize',18);

normal = A2(1:3,3);
d_plano = -normal'*center2;

[x y] = meshgrid(-0.05:0.005:0.2); % Generate x and y data
x = x + center2(1);
y = y + center2(2);
z = -1/normal(3)*(normal(1)*x + normal(2)*5 + d_plano); % Solve for z data
z = z + center2(3);
plan2 = surf(x,y,z,'FaceColor',[0.1,0.1,0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

%% Xrand
posD = [0.08;0.08;0.2];
p_Xgoal = scatter3(posD(1),posD(2),posD(3),'b','filled');
t3 = text(posD(1)-2*a,posD(2)-2*a,posD(3)+2*a,'x_{goal}','FontSize',14,...
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

%% projeções

delete(esfera)
delete(p_Xgoal)
delete(t3);

normal = A(1:3,3);
Xrand_a = proj_ponto_plano(normal,center,Xrand);
x_rand_a = scatter3(Xrand_a(1),Xrand_a(2),Xrand_a(3),'k','filled');
t3 = text(Xrand_a(1)-2*a,Xrand_a(2)-2*a,Xrand_a(3)+2*a,'x^a_{rand}','FontSize',14,...
  'FontWeight', 'bold','color','k');

normal = A2(1:3,3);
Xrand_b = proj_ponto_plano(normal,center2,Xrand);
x_rand_b = scatter3(Xrand_b(1),Xrand_b(2),Xrand_b(3),'k','filled');
t4 = text(Xrand_b(1)-2*a,Xrand_b(2)-2*a,Xrand_b(3)+2*a,'x^b_{rand}','FontSize',14,...
  'FontWeight', 'bold','color','k');
pause()
%% RRT com as projeções
xlim([-0.15    0.25])
ylim([-0.15    0.25])
zlim([-0.05    0.3])
delete(plan1);
delete(plan2);

v1 = (Xrand_a - center)/norm((Xrand_a - center));
Xnew_a = center + v1*0.075;
A3 = zeros(4,4);
A3(1:3,end) = Xnew_a;
A3(end,end) = 1;
A3(1:3,1:3) = axang2rotm([v1',pi/2])*A(1:3,1:3);
j3 = plot_junta_revolucao(A3,[0;0;-h/2],'z',h,radius,color);
e1 = plot3([center(1),Xnew_a(1)],[center(2),Xnew_a(2)],...
  [center(3),Xnew_a(3)],'color',color,'LineWidth',3);
trac1 = plot3([center(1) Xrand_a(1)],[center(2) Xrand_a(2)],[center(3) Xrand_a(3)],...
  '--','color','k','LineWidth',2);
t5 = text(Xnew_a(1)-2*a,Xnew_a(2)-2*a,Xnew_a(3)+2*a,'J^a_{i}','FontSize',18);

v1 = (Xrand_b - center2)/norm((Xrand_b - center2));
Xnew_b = center2 + v1*0.075;
A4 = zeros(4,4);
A4(1:3,end) = Xnew_b;
A4(end,end) = 1;
A4(1:3,1:3) = axang2rotm([v1',pi/2])*A2(1:3,1:3);
j3 = plot_junta_revolucao(A4,[0;0;-h/2],'z',h,radius,color);
e2 = plot3([center2(1),Xnew_b(1)],[center2(2),Xnew_b(2)],...
  [center2(3),Xnew_b(3)],'color',color,'LineWidth',3);
trac2 = plot3([center2(1) Xrand_b(1)],[center2(2) Xrand_b(2)],[center2(3) Xrand_b(3)],...
  '--','color','k','LineWidth',2);
t6 = text(Xnew_b(1)-2*a,Xnew_b(2)-2*a,Xnew_b(3)+2*a,'J^b_{i}','FontSize',18);
pause()
%% Xnew
delete(t3)
delete(t4)
delete(x_rand_a)
delete(x_rand_b)
delete(trac1)
delete(trac2)

v1 = (Xrand - Xnew_a)/norm((Xrand - Xnew_a));
Xnew_a2 = Xnew_a + v1*0.075;
plot_esfera(Xnew_a2,1.5*radius,color,1);
plot3([Xnew_a(1),Xnew_a2(1)],[Xnew_a(2),Xnew_a2(2)],...
  [Xnew_a(3),Xnew_a2(3)],'color',color,'LineWidth',3);
esfera = plot_esfera(Xrand,radius2,color2,0.3);
text(Xnew_a2(1)+a,Xnew_a2(2)+a,Xnew_a2(3)+a,'J^a_{i+1}','FontSize',18);
plot3([Xnew_a(1) Xrand(1)],[Xnew_a(2) Xrand(2)],[Xnew_a(3) Xrand(3)],...
  '--','color','k','LineWidth',2);