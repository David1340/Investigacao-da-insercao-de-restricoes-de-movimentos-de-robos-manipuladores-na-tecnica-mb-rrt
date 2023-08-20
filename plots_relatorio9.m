clc
clear
close all

%% parâmetros
h = 0.025;
L = 0.075;
radius = 2*0.005;
color = [.1,.1,.1];
a = 0.005;
%% junta1
A = eye(4);
A(1:3,end) = [0;0;0*0.075];
A(1:3,1:3) = axang2rotm([0,1,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,0*0.075+2*a,'J_1','FontSize',14,...
  'FontWeight', 'bold','color','red')
hold on
%% junta2
A(1:3,end) = [0;0;0.075];
A(1:3,1:3) = axang2rotm([1,0,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,0.075+2*a,'J_2','FontSize',14,...
  'FontWeight', 'bold','color','red')
%% junta3
A(1:3,end) = [0;0;2*0.075];
A(1:3,1:3) = axang2rotm([0,1,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,2*0.075+2*a,'J_3','FontSize',14,...
  'FontWeight', 'bold','color','red')


%% junta4
A(1:3,end) = [0;0;3*0.075];
A(1:3,1:3) = axang2rotm([1,0,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,3*0.075+2*a,'J_4','FontSize',14,...
  'FontWeight', 'bold','color','red')
%% junta5
A(1:3,end) = [0;0;4*0.075];
A(1:3,1:3) = axang2rotm([0,1,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,4*0.075+2*a,'J_5','FontSize',14,...
  'FontWeight', 'bold','color','red')
%% junta6
A(1:3,end) = [0;0;5*0.075];
A(1:3,1:3) = axang2rotm([1,0,0,pi/2]);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,color);
text(0-a,0-a,5*0.075+2*a,'J_6','FontSize',14,...
  'FontWeight', 'bold','color','red')
%% efetuador
plot_esfera([0;0;6*0.075],1.5*radius,color,1);
%% elos
plot3([0,0],[0,0],[0,6*0.075],'color',color,'LineWidth',3);
text(0-a,0-a,6*0.075+2*a,'Efetuador','FontSize',14,...
  'FontWeight', 'bold','color','red')
a = 0.15;
xlim([-a a])
ylim([-a a])