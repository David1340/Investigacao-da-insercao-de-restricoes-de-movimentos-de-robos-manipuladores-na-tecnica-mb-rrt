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
A(1:3,1:3) = axang2rotm([1,1,0,0]);

j1 = plot_junta_revolucao(A,zeros(3,1),'z',h,radius,color);
hold on
xlim([-0.05    0.1])
ylim([-0.05    0.1])
zlim([-0.05    0.1])
axis equal

a = 0.005;
t1 = text(center(1)+a,center(2)+a,center(3)+a,'J_{i-1}','FontSize',18);

%% junta 2
center2 = [0;0;0.075];
radius = 0.005;
color = [.1 .1 .1];
A = zeros(4);
A(1:3,end) = center2 + [-h/2;0;0];
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([0,1,0,pi/2]);

j2 = plot_junta_revolucao(A,zeros(3,1),'z',h,radius,color);

a = 0.005;
t2 = text(center2(1)+a,center2(2)+a,center2(3)+3*a,'J_{i}','FontSize',18);

%% Elo 1
e1 = plot3([center(1),center2(1)],[center(2),center2(2)],...
  [center(3),center2(3)],'color',color,'LineWidth',3);

%% junta 3
center3 = [0,0.05,0.075];
radius = 0.005;
color = [.1 .1 .1];
A = zeros(4);
A(1:3,end) = center2;
A(end,end) = 1;
A(1:3,1:3) = axang2rotm([0,0,1,0]);

j3 = plot_esfera(center3,radius,color,1);

a = 0.005;
t3 = text(center3(1)+a,center3(2)+a,center3(3)+a,'J_{i+1}','FontSize',18);
e1 = plot3([center2(1),center3(1)],[center2(2),center3(2)],...
  [center2(3),center3(3)],'color',color,'LineWidth',3);


%%
plot_esfera(center2,0.05,[0.1,0.1,0.1],0.1);