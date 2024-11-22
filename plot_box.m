function a = plot_box(Lx,Ly,Lz,offset)
% Coordenadas dos v�rtices do paralelep�pedo
vertices = [
    0, 0, 0;  % V�rtice 1
    Lx, 0, 0; % V�rtice 2
    Lx, Ly, 0; % V�rtice 3
    0, Ly, 0; % V�rtice 4
    0, 0, Lz; % V�rtice 5
    Lx, 0, Lz; % V�rtice 6
    Lx, Ly, Lz; % V�rtice 7
    0, Ly, Lz  % V�rtice 8
];

% Aplicando o offset
vertices = vertices + offset;

% Defini��o das faces usando os �ndices dos v�rtices
faces = [
    1, 2, 6, 5; % Face inferior
    2, 3, 7, 6; % Face lateral
    3, 4, 8, 7; % Face superior
    4, 1, 5, 8; % Face lateral
    1, 2, 3, 4; % Base inferior
    5, 6, 7, 8  % Base superior
];

% Plotando o paralelep�pedo
patch('Vertices', vertices, 'Faces', faces, ...
      'FaceColor', [0.59, 0.29, 0.0], 'EdgeColor', 'None', 'FaceAlpha', 0.7);
end

