function a = plot_box(Lx,Ly,Lz,offset)
% Coordenadas dos vértices do paralelepípedo
vertices = [
    0, 0, 0;  % Vértice 1
    Lx, 0, 0; % Vértice 2
    Lx, Ly, 0; % Vértice 3
    0, Ly, 0; % Vértice 4
    0, 0, Lz; % Vértice 5
    Lx, 0, Lz; % Vértice 6
    Lx, Ly, Lz; % Vértice 7
    0, Ly, Lz  % Vértice 8
];

% Aplicando o offset
vertices = vertices + offset;

% Definição das faces usando os índices dos vértices
faces = [
    1, 2, 6, 5; % Face inferior
    2, 3, 7, 6; % Face lateral
    3, 4, 8, 7; % Face superior
    4, 1, 5, 8; % Face lateral
    1, 2, 3, 4; % Base inferior
    5, 6, 7, 8  % Base superior
];

% Plotando o paralelepípedo
patch('Vertices', vertices, 'Faces', faces, ...
      'FaceColor', [0.59, 0.29, 0.0], 'EdgeColor', 'None', 'FaceAlpha', 0.7);
end

