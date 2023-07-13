function [q_r] = rotacionar_vetor(p,v,th)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%gira p em torno de v em th rad
a = cos(th/2);
b = v(1)*sin(th/2);
c = v(2)*sin(th/2);
d = v(3)*sin(th/2);
p_aumentado = [0;p];
h = [a;b;c;d];
hx = [a,-b,-c,-d];
p_r = multiplicacao_quaternios(h,p_aumentado);
q_r = multiplicacao_quaternios(p_r,hx);
q_r = q_r(2:end);
end

