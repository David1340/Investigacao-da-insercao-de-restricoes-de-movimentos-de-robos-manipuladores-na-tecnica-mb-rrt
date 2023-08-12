function [d,a,alpha,theta,p_n] = getDH_paramaters(q)
%% Parâmetros de DH do pioneer7dof

elos = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075];

d = [elos(1),0,elos(2) + elos(3),0,elos(4)+elos(5),0,0];
a = [0,0,0,0,0,elos(6),0];
alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,pi/2,pi/2];
theta = [pi/2 + q(1),q(2),q(3),q(4),q(5),pi/2 + q(6),pi/2 + q(7)];
%distância da ultima junta a extremidade do efetuador
L = 0.075;
%ponto de atuação do manipulador no sistema de coordenadas on xn yn zn
p_n = [0;0;L;1];
end

