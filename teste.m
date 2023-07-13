clc
clear 
close all
q = -1*ones(7,1);
q = -pi + 2*pi*rand(7,1);
[p,p2] = cinematica_direta(q);
p = p(1:3,1);
q(end)
P = [p2(1:3,:) p]
q = angulos2(P)';

[p,p2] = cinematica_direta(q);
p = p(1:3,1);
q(end)
P2 = [p2(1:3,:) p]

% n = 7;
% figure
% color = ['b','g','r','c','m','y','k']; %diferentes cores
% for i = 2:n+1
%   plot3([P(1,i-1) P(1,i)],[P(2,i-1) P(2,i)],[P(3,i-1) P(3,i)],color(i-1));
%   hold on
% end
% 
% for i = 2:n+1
%   plot3([P2(1,i-1) P2(1,i)],[P2(2,i-1) P2(2,i)],[P2(3,i-1) P2(3,i)],color(i-1));
% end
% grid on