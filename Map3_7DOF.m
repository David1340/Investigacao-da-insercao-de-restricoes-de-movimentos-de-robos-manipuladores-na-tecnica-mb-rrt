%% ambiente
raio_esferas = 0.025;
e1 = struct('raio',raio_esferas,'centro',[0.05;-0.25;0.25]);
e2 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0.15]);
e3 = struct('raio',raio_esferas,'centro',[-0.05;-0.25;0.25]);
e4 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.15]);

e5 = struct('raio',raio_esferas,'centro',[0.08;0;0.25]);
e6 = struct('raio',raio_esferas,'centro',[-0.08;0;0.25]);
e7 = struct('raio',raio_esferas,'centro',[0;-0.16;0.2]);
e8 = struct('raio',raio_esferas,'centro',[0;-0.08;0.25]);

% e9 = struct('raio',raio_esferas,'centro',[0.08;0.08;0.35]);
% e10 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0.35]);
% e11 = struct('raio',raio_esferas,'centro',[-0.08;0.08;0.35]);
% e12 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.35]);

% e7 = struct('raio',raio_esferas,'centro',[0;-0.08;0.18]);
% e8 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.18]);

esferas = [e1,e2,e3,e4,e5,e6,e7,e8];%,e9,e10,e11,e12];

for e = esferas
  plot_esfera(e.centro,e.raio,[0.59, 0.29, 0.0],0.7);
end

% plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.05])
% plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.21])
% plot_box(0.24,0.08,0.1,[-0.12,-0.12,0.05])
% plot_box(0.16,0.08,0.06,[-0.12,-0.12,0.15])