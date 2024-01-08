% Configuração A*

clc
clear 
close all

q = zeros(2,1);
L = [2,1];
gama = 0.5;%rand(1)*pi/2;
theta = [pi/2 + q(1); pi/2 + q(2)];
a = [0;L(2)];
d = [L(1);0];
alpha = [pi/2 + gama; pi];

A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
T1 = A1;
T2 = A1*A2;

% pontos de interesse
p0_0 = [0;0;0;1];
p1_1 = [0;0;0;1];
p2_2 = [0;0;0;1];


p1_0 = T1*p1_1;
p2_0 = T2*p2_2;

%Plot do manipulador
x = 1;
y = 2;
z = 3;
plot3([p0_0(x),p1_0(x),p2_0(x)]...
  ,[p0_0(y),p1_0(y),p2_0(y)]...
  ,[p0_0(z),p1_0(z),p2_0(z)],'-*','markersize',5);

hold on
grid on
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
xlim([-1,1])
ylim([-1,1])



%% plot eixos

% x = 0.2*[1;0;0;1];
% y = 0.2*[0;1;0;1];
% z = 0.2*[0;0;1;1];
% o = [0;0;0;1];
% plot3([o(1) x(1)],[o(2) x(2)],[o(3) x(3)],'r','linewidth',3);
% plot3([o(1) y(1)],[o(2) y(2)],[o(3) y(3)],'g','linewidth',3);
% plot3([o(1) z(1)],[o(2) z(2)],[o(3) z(3)],'b','linewidth',3);

%% movimento da junta 1
p = [];
for th1 = 0:0.1:2*pi
  for th2 = 0:0.1:2*pi
    q(1) = th1;
    q(2) = th2;
    theta = [pi/2 + q(1); pi/2 + q(2)];
    a = [0;L(2)];
    d = [L(1);0];
    alpha = [pi/2 + gama; pi];
    
    A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
    A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
    T1 = A1;
    T2 = A1*A2;
    p2_0 = T2*p2_2;
    p = [p p2_0];
  end
end

plot3(p(1,:),p(2,:),p(3,:),"k*");

%% comentários:

%è a revolução de um anel, o qual varia entre uma casca de uma esfera
%e um pedaço dela