function [p,p2] = cinematica_direta3(q)
% Calcula a cinemática direta do manipulador 6dof
[d,a,alpha,theta,p_n] = getDH_paramaters3(q);
elos = [0.075,0.075,0.075,0.075,0.075,0.075];
o = [0;0;0;1];
p1_1 = o;
p2_2 = o;
p3_3 = o;
p4_4 = o;
p5_5 = o;
p6_6 = o;


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

p1_0 = T1*p1_1;
p2_0 = T2*p2_2;
p3_0 = T3*p3_3;
p4_0 = T4*p4_4;
p5_0 = T5*p5_5;
p6_0 = T6*p6_6;

p = p6_0;
p2 = [o,p1_0,p2_0,p3_0,p4_0,p5_0];
end

