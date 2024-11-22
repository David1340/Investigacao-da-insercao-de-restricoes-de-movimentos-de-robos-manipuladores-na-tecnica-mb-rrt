%% ambiente
raio_esferas = 0.05;
e1 = struct('raio',raio_esferas,'centro',[0;-0.08;0.1]);
e2 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0.1]);
e3 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.1]);

e4 = struct('raio',raio_esferas,'centro',[0;-0.08;0.26]);
e5 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0.26]);
e6 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.26]);

e7 = struct('raio',raio_esferas,'centro',[0;-0.08;0.18]);
e8 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.18]);

esferas = [e1,e2,e3,e4,e5,e6,e7,e8];

% for e = esferas
%   plot_esfera(e.centro,e.raio,[0,0.5,0.5],0.1);
% end

plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.05])
plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.21])
plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.05])
plot_box(0.16,0.08,0.06,[-0.12,-0.12,0.15])