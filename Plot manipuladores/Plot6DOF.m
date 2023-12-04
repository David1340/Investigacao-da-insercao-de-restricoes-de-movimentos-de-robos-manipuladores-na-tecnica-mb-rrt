clc
clear
close all

%% comprimentos dos elos
L = [0.075,0.075,0.075,0.075,0.075,0.075];
%% Variáveis do manipulador
q = zeros(6,1);
q(6) = pi/2;
%% parâmetros de DH
a = [-L(1),-L(2),-L(3),-L(4),-L(5),-L(6)];
alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
d = [0,0,0,0,0,0];
theta = [-pi/2 + q(1),q(2),q(3),q(4),q(5),q(6)];

% Matriz de transformações
A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
A3 = matriz_homogenea(d(3),a(3),alpha(3),theta(3));
A4 = matriz_homogenea(d(4),a(4),alpha(4),theta(4));
A5 = matriz_homogenea(d(5),a(5),alpha(5),theta(5));
A6 = matriz_homogenea(d(6),a(6),alpha(6),theta(6));

T1 = A1;
T2 = T1*A2;
T3 = T2*A3;
T4 = T3*A4;
T5 = T4*A5;
T6 = T5*A6;

% pontos de interesse
p0_0 = [0;0;0;1];
p1_1 = [0;0;0;1];
p2_2 = [0;0;0;1];
p3_3 = [0;0;0;1];
p4_4 = [0;0;0;1];
p5_5 = [0;0;0;1];
p6_6 = [0;0;0;1];


p1_0 = T1*p1_1;
p2_0 = T2*p2_2;
p3_0 = T3*p3_3;
p4_0 = T4*p4_4;
p5_0 = T5*p5_5;
p6_0 = T6*p6_6;


%Plot do manipulador
x = 3;
y = 1;
z = 2;
plot3([p0_0(x),p1_0(x),p2_0(x),p3_0(x),...
  p4_0(x),p5_0(x),p6_0(x)]...
  ,[p0_0(y),p1_0(y),p2_0(y),p3_0(y),...
  p4_0(y),p5_0(y),p6_0(y)]...
  ,[p0_0(z),p1_0(z),p2_0(z),p3_0(z),...
  p4_0(z),p5_0(z),p6_0(z)],'-*','markersize',5);


grid on
xlabel('x')
ylabel('y')
zlabel('z')
lim = 0.5;
xlim([-lim,lim])
ylim([-lim,lim])
zlim([-lim,lim])