%% ambiente
raio_esferas = 0.05;
e1 = struct('raio',raio_esferas,'centro',[0;-0.08;0]);
e2 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0]);
e3 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0]);

e4 = struct('raio',raio_esferas,'centro',[0;-0.08;0.16]);
e5 = struct('raio',raio_esferas,'centro',[0.08;-0.08;0.16]);
e6 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.16]);

e7 = struct('raio',raio_esferas,'centro',[0;-0.08;0.08]);
e8 = struct('raio',raio_esferas,'centro',[-0.08;-0.08;0.08]);

esferas = [e1,e2,e3,e4,e5,e6,e7,e8];

for e = esferas
  plot_esfera(e.centro,e.raio,[0,0.5,0.5],0.3);
end