function [saida] = plot_junta_revolucao(A, p, c, h, r, cor, offset)
    % A matriz de Rotação, p origem da junta no seu sistema de coordenadas, c eixo do

    if nargin < 7
        offset = 0;
    end

    if nargin < 6
        cor = 'k';
    end

    theta = 0:0.8:2*pi+ 0.12; % theta de 0 a 2pi com passos de 0.8
    if c == 'z'
        z = linspace(0, h, numel(theta)) + offset; % z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        [z, theta] = meshgrid(z, theta); % transforma em vetores 2D por causa do plot de superficie
        
        % [x, y, z].T = A * ([x, y, z].T + p)
        x = (r * cos(theta) + p(1, 1)) * A(1, 1) + (r * sin(theta) + p(2, 1)) * A(1, 2) + (z + p(3, 1)) * A(1, 3) + ones(size(z)) * A(1, 4);
        y = (r * cos(theta) + p(1, 1)) * A(2, 1) + (r * sin(theta) + p(2, 1)) * A(2, 2) + (z + p(3, 1)) * A(2, 3) + ones(size(z)) * A(2, 4);
        z = (r * cos(theta) + p(1, 1)) * A(3, 1) + (r * sin(theta) + p(2, 1)) * A(3, 2) + (z + p(3, 1)) * A(3, 3) + ones(size(z)) * A(3, 4);
        saida = surf(x, y, z, 'FaceColor',cor, 'EdgeColor', 'none', 'FaceAlpha', 1);
        
    elseif c == 'y'
        y = linspace(-h, h, numel(theta)) + offset; % y de -h a h com o numero de elementos iguais ao de theta
        [y, theta] = meshgrid(y, theta); % transforma em vetores 2D por causa do plot de superficie
        
        % [x, y, z].T = A * ([x, y, z].T + p)
        x = (r * cos(theta) + p(1, 1)) * A(1, 1) + (r * sin(theta) + p(3, 1)) * A(1, 3) + (y + p(2, 1)) * A(1, 2) + ones(size(y)) * A(1, 4);
        z = (r * cos(theta) + p(1, 1)) * A(3, 1) + (r * sin(theta) + p(3, 1)) * A(3, 3) + (y + p(2, 1)) * A(3, 2) + ones(size(y)) * A(3, 4);
        y = (r * cos(theta) + p(1, 1)) * A(2, 1) + (r * sin(theta) + p(3, 1)) * A(2, 3) + (y + p(2, 1)) * A(2, 2) + ones(size(y)) * A(2, 4);
        saida = surf(x, y, z, 'FaceColor', cor, 'EdgeColor', 'none', 'FaceAlpha', 1);
        
    elseif c == 'x'
        x = linspace(-h, h, numel(theta)) + offset; % x de -h a h com o numero de elementos iguais ao de theta
        [x, theta] = meshgrid(x, theta); % transforma em vetores 2D por causa do plot de superficie
        
        % [x, y, z].T = A * ([x, y, z].T + p)
        y = (r * cos(theta) + p(3, 1)) * A(2, 3) + (r * sin(theta) + p(2, 1)) * A(2, 2) + (x + p(1, 1)) * A(2, 1) + ones(size(x)) * A(2, 4);
        z = (r * cos(theta) + p(3, 1)) * A(3, 3) + (r * sin(theta) + p(2, 1)) * A(3, 2) + (x + p(1, 1)) * A(3, 1) + ones(size(x)) * A(3, 4);
        x = (r * cos(theta) + p(3, 1)) * A(1, 3) + (r * sin(theta) + p(2, 1)) * A(1, 2) + (x + p(1, 1)) * A(1, 1) + ones(size(x)) * A(1, 4);
        saida = surf(x, y, z, 'FaceColor', cor, 'EdgeColor', 'none', 'FaceAlpha', 1);
    end
end
