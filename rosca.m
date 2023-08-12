clc
clear
close all

%% comprimentos dos elos
L = [0.075,0.025];
%% Variáveis do manipulador
for q1 = 0:0.1:2*pi
  for q2 = 0:0.1:2*pi
    q = zeros(2,1);
    q(1) = q1;
    q(2) = q2;
    %% parâmetros de DH
    a = [-L(1),-L(2)];
    alpha = [pi/2,-pi/2];
    d = [0,0];
    theta = [-pi/2 + q(1),q(2)];
    
    % Matriz de transformações
    A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
    A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
    
    T1 = A1;
    T2 = T1*A2;
    
    % pontos de interesse
    p0_0 = [0;0;0;1];
    p1_1 = [0;0;0;1];
    p2_2 = [0;0;0;1];
    
    
    p1_0 = T1*p1_1;
    p2_0 = T2*p2_2;
    
    %Plot do manipulador
    x = 3;
    y = 1;
    z = 2;
    plot3(p2_0(x),p2_0(y),p2_0(z),'k.');
    %   scatter3(p2_0(x),p2_0(y),p2_0(z),'k','filled');
    hold on
  end
end


grid on
xlabel('x')
ylabel('y')
zlabel('z')
lim = 0.5;
xlim([-lim,lim])
ylim([-lim,lim])
zlim([-lim,lim])