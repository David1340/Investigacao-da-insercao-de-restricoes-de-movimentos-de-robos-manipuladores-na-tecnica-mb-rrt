function resultado = deteccao_de_colisao(pr1, pr2, p, raio)
    v = pr2 - pr1;
    u = p - pr1;

    % Seja w a proje��o de u em v ent�o w = k*v
    % em que w = dot(u, v)/norm(v)^2 * v
    N_v = norm(v); % norma de v
    k = dot(u, v)/(N_v^2);

    if(k > N_v || k < 0) % Se a proje��o de u n�o est� no segmento de reta pr1->pr2
        if(norm(pr1 - p) <= raio)
            resultado = true;
            return;
        end

        if(norm(pr2 - p) <= raio)
            resultado = true;
            return;
        end

        resultado = false;
        return;
    end

    % d = |u x v|/|v|^2 dist�ncia entre ponto e reta
    d = sqrt(sum(cross(u, v).^2))/(N_v^2);

    if(d <= raio)
        resultado = true;
    else
        resultado = false;
    end
end