function [d,a,alpha,theta,p_n] = getDH_paramaters3(q)
%% Parâmetros de DH do robô 6DOF

elos = [0.075,0.075,0.075,0.075,0.075,0.075];

d = [0,0,0,0,0,0];
a = [-elos(1),-elos(2),-elos(3),-elos(4),-elos(5),-elos(6)];
alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
theta = [-pi/2 + q(1),q(2),q(3),q(4),q(5),q(6)];

%ponto de atuação do manipulador no sistema de coordenadas on xn yn zn
p_n = [0;0;0;1];
end

