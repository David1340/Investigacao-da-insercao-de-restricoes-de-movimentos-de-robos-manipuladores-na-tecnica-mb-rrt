clc
clear 
close all
A = eye(4);
r = 1;
th = 0:0.8:2*pi +0.12;
z = linspace(0,10,length(th));
[z, th] = meshgrid(z,th);

x = (r*cos(th))*A(1,1) + (r*sin(th))*A(1,2) + (r*cos(th))*A(1,3)+... 
ones(size(z))*A(1,4);
y = (r*cos(th))*A(2,1) + (r*sin(th))*A(2,2) + (r*cos(th))*A(2,3)+... 
ones(size(z))*A(2,4);
z = (r*cos(th))*A(3,1) + (r*sin(th))*A(3,2) + (z)*A(3,3)+... 
ones(size(z))*A(3,4);
surface(x,y,z,'FaceColor','b');
hold on
grid on