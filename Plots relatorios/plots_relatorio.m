clc
clear
close all
rng(5)
%% junta 1
center = zeros(3,1);
center(3) = 0.1;
radius = 0.005;
color = [.1 .1 .1];

j1 = plot_esfera(center,radius,color);
hold on
xlim([-0.1600    0.1600])
ylim([-0.1600    0.1600])
zlim([-0.07    0.250])


a = 0.005;
t1 = text(center(1)+a,center(2)+a,center(3)+a,'J_{i-1}^a','FontSize',18);

%% junta 2
center2 = [-0.04;0;0.09];
% 
j2 = plot_esfera(center2,radius,color);
t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J_{i-1}^b','FontSize',18);
%% J_{i-2}
center3 = zeros(3,1);
center3(1) = center(1);
center3(3) = ((center(1) - center2(1))^2 + center2(3)^2 - center(3)^2)/...
  (2*(center2(3) - center(3)));
j2 = plot_esfera(center3,radius,color);
t2 = text(center3(1)+a,center3(2)+a,center3(3)+a,'J_{i-2}','FontSize',18);

e1 = plot3([center(1),center3(1)],[center(2),center3(2)],...
  [center(3),center3(3)],'color',color,'LineWidth',3);

e2 = plot3([center2(1),center3(1)],[center2(2),center3(2)],...
  [center2(3),center3(3)],'color',color,'LineWidth',3);

center3(3) = center3(3)-0.015;
plot_esfera(center3,radius/2,color);
center3(3) = center3(3)-0.015;
plot_esfera(center3,radius/2,color);
center3(3) = center3(3)-0.015;
plot_esfera(center3,radius/2,color);
%% Xrand
posD = [0.1;0.1;0.15];
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
% delete(j2);
% delete(e1);
% delete(t2)
delete(esfera)
v = (Xrand - center)/(norm(Xrand - center));
center2 = center + v*0.075;


j2 = plot_esfera(center2,radius,color);
e1 = plot3([center(1),center2(1)],[center(2),center2(2)],...
  [center(3),center2(3)],'color',color,'LineWidth',3);
reta = plot3([center(1),Xrand(1)],[center(2),Xrand(2)],...
  [center(3),Xrand(3)],'-.','color',color,'LineWidth',2);
t2 = text(center2(1)+a,center2(2)+a,center2(3)+a,'J_{i}^a','FontSize',18);

esfera = plot_esfera(Xrand,radius2,[0.1,0.1,0.1],0.1);