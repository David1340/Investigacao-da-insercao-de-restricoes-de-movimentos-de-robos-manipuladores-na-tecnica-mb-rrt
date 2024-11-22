function saida = plot_esfera(c,r, cor, FaceAlpha)
    % A matriz de Rotação, p origem da junta no seu sistema de coordenadas, c eixo do
    
    if(nargin < 4)
      FaceAlpha = 0.8;
    end
    
    theta = 0:0.1:2*pi; % theta de 0 a 2pi com passos de 0.8
    phi = 0:0.1:3*pi;    % Ângulo phi varia de 0 a 2*pi
    [theta, phi] = meshgrid(theta, phi);
    x = r*sin(theta).*cos(phi) + c(1);
    y = r*sin(theta).*sin(phi) + c(2);
    z = r*cos(theta) + c(3);
    saida = surf(x, y, z,'FaceColor', cor, 'EdgeColor', 'none', 'FaceAlpha', FaceAlpha);


end
