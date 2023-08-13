function [d,a,alpha,theta,p_n] = getDH_paramaters2(q)
%% Par�metros de DH do rob� 4DOF

elos = [0.075,0.075,0.075,0.075];

d = [0,0,0,0];
a = [-elos(1),-elos(2),-elos(3),-elos(4)];
alpha = [pi/2,-pi/2,pi/2,0];
theta = [-pi/2 + q(1),q(2),q(3),q(4)];

%ponto de atua��o do manipulador no sistema de coordenadas on xn yn zn
p_n = [0;0;0;1];
end

