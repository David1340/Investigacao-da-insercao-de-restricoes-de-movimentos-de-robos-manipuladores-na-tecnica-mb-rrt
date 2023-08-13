clc
clear
close all

q = -pi + pi*rand(6,1);
[p2 p] = cinematica_direta3(q);
p = [p p2];
p = p(1:3,:)
[q, angulos2(p)]