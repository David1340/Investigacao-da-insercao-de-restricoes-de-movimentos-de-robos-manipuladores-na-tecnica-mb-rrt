%% MB-RRT for 7DOF manipulator 
%configuração A
%Experimentos
clc
clear
close all
rng(1)
%% Parâmetros do robô
limites_superior = [pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,(3/4)*pi];
limites_inferior = -[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,(3/4)*pi];
radius = 0.005; %ráio dos elos do robô
h = 0.025;
color = [0.1 0.1 0.1]; %cor do robô 
d = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;1;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 7; %número de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunferências em volta do destino
r = [r(end-1:-1:1),0];

%% configuração do experimento
erro_min = sum(d)/100;
K = 1000; %número máximo de iteração

taxa_convergiu = [];
media_iteracao = [];
Exps = 1000
for Exp = 1:Exps
  repeticao = Exp
  %% Inicialzação da MB-RRT
  erro = Inf; %erro inicial
  G = root; %árvore
  
  qrand = limites_inferior + (limites_superior - limites_inferior).*rand(7,1);
  [posD,juntas,eixos] = myF.CD_7DOF(qrand);
  posD = posD(1:3); %posição desejada
%   posD = [0;-0.2;0.2];
  Xgoal = posD;
  
  P_new = d(1)*G(4:6,1) + G(1:3,1); %primeira junta é fixa devido a estrutura do manipulador
  G = [G, [P_new ; zeros(3,1);2;1]]; %ainda não sei o eixo de atuação
  
  %% plot inicial
  A = eye(4);
  
  Map1 %vazio
  close all
  P_parent = G(1:3,1);
  
  %% MB-RRT
  for k = 1:K
    for i = 2:n
      th = rand(1,1);
      fi = rand(1,1);
      if(i == 2 | i == 4) %Se for Hinge
        
        raio = r(i+1); % comprimento da cadeia entre a junta superior até o efetuador
        P_rand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
          raio*sin(2*pi*fi)*sin(2*pi*th);
          raio*cos(2*pi*fi)]; %ponto aleatório na esfera
        
        P_rand = P_rand + posD; %desloca o centro da esfera
        
        idcs_parents = find(G(7,:) == i); %encontra os nós de índice i
        
        if(isempty(idcs_parents))
          continue
        end
        
        g = G(1:3,idcs_parents);
        [valor idc_parent] = min(abs(myF.distancias(P_rand,g) - (d(i)+d(i+1)))); %nós mais próximo
        idc_parent = idcs_parents(idc_parent);
        P_parent = G(1:3,idc_parent);
        V_new = (P_rand - P_parent)/norm(P_rand - P_parent);
        P_new = d(i)*V_new + P_parent;
        P_new2 = d(i+1)*V_new + P_new;
        %% checagem de colisão 1
        colidiu = 0;
        for e = esferas
          if(deteccao_de_colisao(P_parent,P_new,e.centro,raio_esferas + h))
            colidiu = 1;
            break;
          end
        end
        if(colidiu)
          continue
        end
        %% checagem de colisão 2
        colidiu = 0;
        for e = esferas
          if(deteccao_de_colisao(P_new,P_new2,e.centro,raio_esferas + h))
            colidiu = 1;
            break;
          end
        end
        if(colidiu)
          continue
        end
        %% Checagem do ângulo 1
        idc_grandparent = G(8,idc_parent);
        P_grandparent = G(1:3,idc_grandparent);
        v2 = P_grandparent - P_parent;
        v2 = v2/norm(v2);
        v1 = P_new - P_parent;
        v1 = v1/norm(v1);
        V_parent = cross(v1,v2);
        V_parent = V_parent/norm(V_parent);
        V_grandparent = G(4:6,idc_grandparent);
        
        if(i == 2)
          V_ref = [1;0;0]; %x
        else
          
          idc_greatgrandparent = G(8,idc_grandparent);
          P_greatgrandparent = G(1:3,idc_greatgrandparent);
          
          idc_great_greatgrandparent = G(8,idc_greatgrandparent);
          P_great_greatgrandparent = G(1:3,idc_great_greatgrandparent);
          v2 = P_great_greatgrandparent - P_greatgrandparent;
          v2 = v2/norm(v2);
          v1 = P_grandparent - P_greatgrandparent;
          v1 = v1/norm(v1);
          V_greatgrandparent = cross(v1,v2);
          V_greatgrandparent = V_greatgrandparent/norm(V_greatgrandparent);
          V_ref = V_greatgrandparent;
        end
        v = myF.rot_vetor(V_ref,V_grandparent,pi/2);
        q = acos(V_parent'*V_ref);
        if(V_parent'*v < 0)
          q = -q;
        end
        if(q > limites_superior(i-1) | q < limites_inferior(i-1))
          continue
        end
        
        %% Checagem do ângulo 2
        V_ref = V_grandparent;
        V_aux = P_new - P_parent;
        V_aux = V_aux/norm(V_aux);
        
        v = myF.rot_vetor(V_ref,V_parent,pi/2);
        
        q = acos(V_aux'*V_ref);
        if(V_aux'*v < 0)
          q = -q;
        end
        if(q > limites_superior(i) | q < limites_inferior(i))
          continue
        end
        %% Adição do primeiro nó da árvore
        G = [G, [P_new;V_new;i+1;idc_parent]];
        %% plot do primeiro ponto
        y = V_new;
        idc_grandparent = G(8,idc_parent);
        P_grandparent = G(1:3,idc_grandparent);
        v1 = P_grandparent - P_parent;
        v1 = v1/norm(v1);
        v2 = P_new - P_parent;
        v2 = v2/norm(v2);
        v = cross(v1,v2);
        v = v/norm(v);
        x = v;
        z = cross(x,y);
        A = eye(4);
        A(1:3,1:3) = [x y z];
        A(1:3,end) = P_new;
        %       plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        %         ,'Color',color,'linewidth',2)
        %       plot_junta_revolucao(A,[0;-h/2;0],'y',h,radius,color);
        %% Adição do segundo nó na árvore
        G = [G, [P_new2;zeros(3,1);i+2;size(G,2)]];
        %% plot do segundo nó
        %       plot3([P_new(1) P_new2(1)],[P_new(2) P_new2(2)],[P_new(3) P_new2(3)]...
        %         ,'Color',color,'linewidth',2)
        %       plot_esfera(P_new2,2*radius,color,1);
        %       pause(0.1)
      elseif(i == 6)
        
        raio = r(i); % comprimento da cadeia entre a junta superior até o efetuador
        P_rand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
          raio*sin(2*pi*fi)*sin(2*pi*th);
          raio*cos(2*pi*fi)]; %ponto aleatório na esfera
        P_rand = P_rand + posD; %desloca o centro da esfera
        
        idcs_parents = find(G(7,:) == i); %encontra as colunas dos nós de índice i
        
        if(isempty(idcs_parents))
          continue
        end
        
        g = G(1:3,idcs_parents);
        
        [valor idc_parent] = min(abs(myF.distancias(P_rand,g) - d(i)));
        idc_parent = idcs_parents(idc_parent);
        P_parent = G(1:3,idc_parent);
        
        v = (P_rand - P_parent)/norm(P_rand - P_parent);
        P_new = d(i)*v + P_parent;
        %% checagem de colisão
        colidiu = 0;
        for e = esferas
          if(deteccao_de_colisao(P_parent,P_new,e.centro,raio_esferas + h))
            colidiu = 1;
            break;
          end
        end
        if(colidiu)
          continue
        end
        
        %% Checagem do ângulo 1
        idc_grandparent = G(8,idc_parent);
        P_grandparent = G(1:3,idc_grandparent);
        v2 = P_grandparent - P_parent;
        v2 = v2/norm(v2);
        v1 = P_new - P_parent;
        v1 = v1/norm(v1);
        V_parent = cross(v1,v2);
        V_parent = V_parent/norm(V_parent);
        V_grandparent = G(4:6,idc_grandparent);
        
        idc_greatgrandparent = G(8,idc_grandparent);
        P_greatgrandparent = G(1:3,idc_greatgrandparent);
        
        idc_great_greatgrandparent = G(8,idc_greatgrandparent);
        P_great_greatgrandparent = G(1:3,idc_great_greatgrandparent);
        v2 = P_great_greatgrandparent - P_greatgrandparent;
        v2 = v2/norm(v2);
        v1 = P_grandparent - P_greatgrandparent;
        v1 = v1/norm(v1);
        V_greatgrandparent = cross(v1,v2);
        V_greatgrandparent = V_greatgrandparent/norm(V_greatgrandparent);
        V_ref = V_greatgrandparent;
        
        v = myF.rot_vetor(V_ref,V_grandparent,pi/2);
        q = acos(V_parent'*V_ref);
        if(V_parent'*v < 0)
          q = -q;
        end
        if(q > limites_superior(i-1) | q < limites_inferior(i-1))
          continue
        end
        
        %% Checagem do ângulo 2
        V_ref = V_grandparent;
        V_aux = P_new - P_parent;
        V_aux = V_aux/norm(V_aux);
        
        v = myF.rot_vetor(V_ref,V_parent,pi/2);
        
        q = acos(V_aux'*V_ref);
        if(V_aux'*v < 0)
          q = -q;
        end
        if(q > limites_superior(i) | q < limites_inferior(i))
          continue
        end
        
        %% Adicionando nó a árvore
        G = [G, [P_new;zeros(3,1);i+1;idc_parent]];
        %% plot do nó na árvore
        %       plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        %         ,'Color',color,'linewidth',2)
        %       plot_esfera(P_new,2*radius,color,1);
        %       pause(0.1)
      elseif(i == 7)
        if(~(G(7,end) == i))
          continue
        end
        P_rand = posD;
        %% Projetanto ponto no plano
        P_grandparent = G(:,G(8,end));
        P_greatgrandparent = G(:,P_grandparent(8));
        
        v1 = G(1:3,end) - P_grandparent(1:3);
        v1 = v1/norm(v1);
        v2 = P_greatgrandparent(1:3) - P_grandparent(1:3);
        v2 = v2/norm(v2);
        V_grandparent = cross(v2,v1);
        V_grandparent = V_grandparent/norm(V_grandparent);
        
        V_parent = myF.rot_vetor(V_grandparent,v1,pi/2); %v3 em torno de v1
        V_parent = V_parent/norm(V_parent);
        P_rand = myF.proj_ponto_plano(V_parent,G(1:3,end),P_rand);
        %% MB-RRT padrão
        P_parent = G(1:3,end);
        v = (P_rand - P_parent)/norm(P_rand - P_parent);
        P_new = d(end)*v + P_parent;
        %% checagem de colisão
        colidiu = 0;
        for e = esferas
          if(deteccao_de_colisao(P_parent,P_new,e.centro,raio_esferas + h))
            colidiu = 1;
            break;
          end
        end
        if(colidiu)
          continue
        end
        %% checagem de ângulo
        idc_grandparent = G(8,idc_parent);
        P_grandparent = G(1:3,idc_grandparent);
        
        V_ref = P_parent - P_grandparent;
        V_ref = V_ref/norm(V_ref);
        
        v = myF.rot_vetor(V_ref,V_grandparent,pi/2);
        q = acos(V_parent'*V_ref);
        if(V_parent'*v < 0)
          q = -q;
        end
        if(q > limites_superior(i) | q < limites_inferior(i))
          continue
        end
        %% Adição do nó na árvore
        G = [G, [P_new;zeros(3,1);i+1;size(G,2)]];
        %% plot do nó na árvore
        %       plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        %         ,'Color',color,'linewidth',2)
        %       plot_esfera(P_new,2*radius,color,1);
        %       pause(0.1)
      end
      
      if(i == n)
        erro = sqrt(sum((posD -G(1:3,end)).^2));
      end
    end
    if(erro < erro_min)
      erro_final = erro;
      break
    end
  end
  
  if (erro < erro_min)
    
    p0 = G(:,end);
    P2 = p0;
    
    for i = 1:n
      p0 = G(:,p0(8));
      P2 = [P2 p0];
    end
    
    P2 = P2(1:3,end:-1:1);
    q = myF.angulos_7DOF(P2);
    [p,juntas,eixos] = myF.CD_7DOF(q);
    G = [juntas(1:3,:) p(1:3)];
    %% plot
    for i = 1:7
      %     plot3([G(1,i) G(1,i+1)],[G(2,i) G(2,i+1)],[G(3,i) G(3,i+1)]...
      %       ,'b','linewidth',2)
      
    end
    
    %% se quiser testar
    angulos_checagem = q*(180/pi);
    
    erro = sqrt(sum((posD -G(1:3,end)).^2));
    if(erro < erro_min)
      convergiu = 1;
    else
      convergiu = 0;
    end
  else
    convergiu = 0;
  end
  taxa_convergiu = [taxa_convergiu convergiu];
  media_iteracao = [media_iteracao k];
end
media_iteracao = media_iteracao(find(taxa_convergiu));
desvio_iteracao = std(media_iteracao)
media_iteracao = mean(media_iteracao) 

taxa_convergiu = sum(taxa_convergiu)/Exps * 100