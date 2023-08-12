function [p,p2] = cinematica_direta2(q)
% Calcula a cinem�tica direta do manipulador 4dof
[d,a,alpha,theta,p_n] = getDH_paramaters2(q);
elos = [0.075,0.075,0.075,0.075];
o = [0;0;0;1];
p1_1 = o;
p2_2 = o;
p3_3 = o;
p4_4 = o;

A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
A3 = matriz_homogenea(d(3),a(3),alpha(3),theta(3));
A4 = matriz_homogenea(d(4),a(4),alpha(4),theta(4));

T1 = A1;
T2 = T1*A2;
T3 = T2*A3;
T4 = T3*A4;


p1_0 = T1*p1_1;
p2_0 = T2*p2_2;
p3_0 = T3*p3_3;
p4_0 = T4*p4_4;

p = p4_0;
p2 = [o,p1_0,p2_0,p3_0];
end
