classdef myF
    methods(Static)
      %% Calcula a distância euclidiana entre um ponto A e um conjunto de pontos B
        function d = distancias(A,B)
            d = zeros(1,size(B,2));
            for i = 1:length(d)
              d(i) = sqrt(sum((A - B(:,i)).^2));
            end
        end
        
        %% Calcula a distância euclidiana entre um ponto A e um conjunto de pontos
        %B (projetados no planos definidos pelos vetores em V e pontos em A)
        function dists = distancias2(A,B,V)
            dists = zeros(1,size(B,2));
            for i = 1:length(dists)
              B_projetado = myF.proj_ponto_plano(V(:,i),A,B(:,i));
              dists(i) = sqrt(sum((A - B_projetado).^2));
            end
        end
 
        %% Calcula a distância euclidiana entre um ponto B e um conjunto de pontos
        %B (projetados no planos definidos pelos vetores em V e pontos e em A)
        function dists = distancias3(A,B,V,d)
          dists = zeros(1,size(B,2));
          for i = 1:length(dists)
            B_projetado = myF.proj_ponto_plano(V(:,i),A,B(:,i));
            v = B_projetado - A;
            v = v/norm(v);
            B_projetado = A + d*v;
            dists(i) = sqrt(sum((B(:,i) - B_projetado).^2));
          end
        end
              
        %% Calcula a cinemática direta do manipulador 6dof
        function [p,p2] = CD_6DOF(q)
          
          [d,a,alpha,theta,p_n] = myF.DH_6DOF(q);
          elos = [0.075,0.075,0.075,0.075,0.075,0.075];
          o = [0;0;0;1];
          p1_1 = o;
          p2_2 = o;
          p3_3 = o;
          p4_4 = o;
          p5_5 = o;
          p6_6 = o;
          
          
          A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
          A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
          A3 = matriz_homogenea(d(3),a(3),alpha(3),theta(3));
          A4 = matriz_homogenea(d(4),a(4),alpha(4),theta(4));
          A5 = matriz_homogenea(d(5),a(5),alpha(5),theta(5));
          A6 = matriz_homogenea(d(6),a(6),alpha(6),theta(6));
          
          T1 = A1;
          T2 = T1*A2;
          T3 = T2*A3;
          T4 = T3*A4;
          T5 = T4*A5;
          T6 = T5*A6;
          
          p1_0 = T1*p1_1;
          p2_0 = T2*p2_2;
          p3_0 = T3*p3_3;
          p4_0 = T4*p4_4;
          p5_0 = T5*p5_5;
          p6_0 = T6*p6_6;
          
          p = p6_0;
          p2 = [o,p1_0,p2_0,p3_0,p4_0,p5_0];
        end
        %% Calcul a cinemática direta do manipulador 7dof
        function [efetuador,juntas,eixos] = CD_7DOF(q)
          [d,a,alpha,theta,p_n] = myF.DH_7DOF(q);
          elos = [0.05,0.075,0.075,0.0725,0.0725,0.075,0.075];
          o = [0;0;0;1];
          p1_1 = [0;-elos(1)-elos(2);0;1];
          p2_2 = o;
          p3_3 = [0;-elos(3);0;1];
          p4_4 = o;
          p5_5 = [0;-elos(5);0;1];
          p6_6 = [-elos(6);0;0;1];
          p7_7 = o;
          A1 = matriz_homogenea(d(1),a(1),alpha(1),theta(1));
          A2 = matriz_homogenea(d(2),a(2),alpha(2),theta(2));
          A3 = matriz_homogenea(d(3),a(3),alpha(3),theta(3));
          A4 = matriz_homogenea(d(4),a(4),alpha(4),theta(4));
          A5 = matriz_homogenea(d(5),a(5),alpha(5),theta(5));
          A6 = matriz_homogenea(d(6),a(6),alpha(6),theta(6));
          A7 = matriz_homogenea(d(7),a(7),alpha(7),theta(7));
          T1 = A1;
          T2 = T1*A2;
          T3 = T2*A3;
          T4 = T3*A4;
          T5 = T4*A5;
          T6 = T5*A6;
          T7 = T6*A7;
          
          p1_0 = T1*p1_1;
          p2_0 = T2*p2_2;
          p3_0 = T3*p3_3;
          p4_0 = T4*p4_4;
          p5_0 = T5*p5_5;
          p6_0 = T6*p6_6;
          p7_0 = T7*p7_7;
          
          efetuador = T7*p_n;
          juntas = [p1_0,p2_0,p3_0,p4_0,p5_0,p6_0,p7_0];
          eixos = [T1(1:3,2),T2(1:3,2),T3(1:3,2),T4(1:3,2),T5(1:3,2),T6(1:3,2),T7(1:3,2)];
        end
      
        %% Parâmetros de DH do robô 6DOF
        function [d,a,alpha,theta,p_n] = DH_6DOF(q)      
          elos = [0.075,0.075,0.075,0.075,0.075,0.075];
          
          d = [0,0,0,0,0,0];
          a = [-elos(1),-elos(2),-elos(3),-elos(4),-elos(5),-elos(6)];
          alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
          theta = [-pi/2 + q(1),q(2),q(3),q(4),q(5),q(6)];
          
          %ponto de atuação do manipulador no sistema de coordenadas on xn yn zn
          p_n = [0;0;0;1];
        end
        
        %% Parâmetros de DH do robô 7DOF
        function [d,a,alpha,theta,p_n] = DH_7DOF(q)        
          elos = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075];
          
          d = [elos(1),0,elos(2) + elos(3),0,elos(4)+elos(5),0,0];
          a = [0,0,0,0,0,elos(6),0];
          alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,pi/2,pi/2];
          theta = [pi/2 + q(1),q(2),q(3),q(4),q(5),pi/2 + q(6),pi/2 + q(7)];
          %distância da ultima junta a extremidade do efetuador
          L = 0.075;
          %ponto de atuação do manipulador no sistema de coordenadas on xn yn zn
          p_n = [0;0;L;1];
        end
        
        %% checa se houve colisão entre um cilindro e uma esfera
        function resultado = colisao(pr1, pr2, p, raio)
          v = pr2 - pr1;
          u = p - pr1;
          
          % Seja w a projeção de u em v então w = k*v
          % em que w = dot(u, v)/norm(v)^2 * v
          N_v = norm(v); % norma de v
          k = dot(u, v)/(N_v^2);
          
          if(k > N_v || k < 0) % Se a projeção de u não está no segmento de reta pr1->pr2
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
          
          % d = |u x v|/|v|^2 distância entre ponto e reta
          d = sqrt(sum(cross(u, v).^2))/(N_v^2);
          
          if(d <= raio)
            resultado = true;
          else
            resultado = false;
          end
        end
        
        %% rotaciona p em torno de v em th rad
        function [q_r] = rot_vetor(p,v,th)
          a = cos(th/2);
          b = v(1)*sin(th/2);
          c = v(2)*sin(th/2);
          d = v(3)*sin(th/2);
          p_aumentado = [0 p'];
          h = [a,b,c,d];
          hx = [a,-b,-c,-d];
          p_r = quatmultiply(h,p_aumentado);
          q_r = quatmultiply(p_r,hx)';
          q_r = q_r(2:end);
        end
        %% Cálculo os ângulos a partir da especificação da posição das juntas
          % para o manipulador 6dod
          % Atualmente tem problema com pontos colineares
        function [q] = angulos_6DOF(p)
          
          n = size(p,2)-1;
          q = zeros(n,1);
          y = [0;1;0];
          z = [0;0;1];
          alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
          % cálculo dos vetores de referência
          V = zeros(3,n);
          V(1:3,1) = z;
          for i =2:n
            v = (p(1:3,i) - p(1:3,i-1))/norm(p(1:3,i) - p(1:3,i-1));
            V(1:3,i) = myF.rot_vetor(V(1:3,i-1),-v,alpha(i-1));
          end
          
          %% Cálculos dos vetores de ref e ângulos
          for i = 1:n
            %% Vref
            if(i == 1)
              vref = y;
            else
              vref = p(:,i) - p(:,i-1);
              vref = vref/norm(vref);
            end
            
            v = myF.rot_vetor(vref,V(:,i),pi/2);
            
            % ângulo
            vaux = p(:,i+1) - p(:,i);
            vaux = vaux/norm(vaux);
            q(i) = acos(vaux'*vref);
            if(vaux'*v < 0)
              q(i) = -q(i);
            end
          end
        end
        
          %% Cálculo os ângulos a partir da especificação da posição das juntas
          % para o pioneer7DOF
          % Atualmente tem problema com pontos colineares        
        function [q] = angulos_7DOF(p)

          n = size(p,2)-1;
          q = zeros(n,1);
          x = [1;0;0];
          
          %% Eixos de atuação
          V = zeros(3,n);
          for i = 1:n
            if(i == 1 || i == 3 || i == 5)
              v = p(:,i+1) - p(:,i);
              V(:,i) = v/norm(v);
            else
              v1 = p(:,i+1) - p(:,i);
              v1 = v1/norm(v1);
              v2 = p(:,i-1) - p(:,i);
              v2 = v2/norm(v2);
              V(:,i) = cross(v1,v2);
              V(:,i) = V(:,i)/norm(V(:,i));
            end
          end
          %% Cálculos dos vetores de ref e ângulos
          for i = 1:n
            %% Vref
            if(i == 1)
              vref = x;
            elseif(i == n)
              vref = p(:,i) - p(:,i-1);
              vref = vref/norm(vref);
            else
              vref = V(:,i-1);
              vref = vref/norm(vref);
            end
            
            v = myF.rot_vetor(vref,V(:,i),pi/2);
            
            %% ângulo
            if(i == n)
              vaux = p(:,i+1) - p(:,i);
              vaux = vaux/norm(vaux);
              q(i) = acos(vaux'*vref);
              if(vaux'*v < 0)
                q(i) = -q(i);
              end
            elseif(i == n-1)
              vaux = p(:,i+1) - p(:,i);
              vaux = vaux/norm(vaux);
              q(i) = acos(vaux'*vref);
              if(vaux'*v < 0)
                q(i) = -q(i);
              end
            else
              q(i) = acos(V(:,i+1)'*vref);
              if(V(:,i+1)'*v < 0)
                q(i) = -q(i);
              end
            end
            
          end
          [p2,juntas] = myF.CD_7DOF(q);
          p2 = [juntas(1:3,:) p2(1:3)];
          erro_angulos = sqrt(sum((p2(:,end) - p(:,end)).^2));
          if(erro_angulos > 10^-4)
            q(end) = -q(end);
          end
        end
        
        function [p_proj] = proj_ponto_plano(n,p0,p)
          d = -n'*p0; %produto escalar
          alpha = (-d - n'*p)/(n(1)^2 + n(2)^2 + n(3)^2);
          p_proj = p + alpha*n;
        end
    end
end