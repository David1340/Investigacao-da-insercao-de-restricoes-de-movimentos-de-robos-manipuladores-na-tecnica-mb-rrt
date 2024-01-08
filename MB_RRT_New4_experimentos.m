%% MB-RRT for 6DOF manipulator
% Configuração B
clc
clear
close all

%% Parâmetros do robô
limites_superior = [pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];
limites_inferior = -[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];
radius = 0.005; %ráio dos elos do robô
h = 0.025;
color = [0.1 0.1 0.1]; %cor do robô 
alpha = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0]; %parâmetro alpha-DH
d = [0.075,0.075,0.075,0.075,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;0;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 6; %número de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunferências em volta do destino
r = [r(end-1:-1:1),0];
%% configuração do experimento
erro_min = sum(d)/100;
K = 1000; %número máximo de iteração

%% Inicialzação da MB-RRT
erro = Inf; %erro inicial
G = root; %árvore
%% Plot inicial
A = eye(4);
% plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,'b');
% xlim([-0.2,0.6])
% xlabel('x')
% ylim([-0.2,0.6])
% ylabel('y')
% zlim([-0.4,0.4])
% axis equal
% zlabel('z')
% title("Manipulator-Based Rapidly Random Tree")
% hold on
% scatter3(Xgoal(1),Xgoal(2),Xgoal(3),'r','filled','linewidth',3)

ambiente
esferas = [];
close all
% legend("Solution","Destino","Obstáculos",'AutoUpdate','off')
%% Loop da MB-RRT
taxa_convergiu = [];
media_iteracao = [];
Exps = 10000;
for Exp = 1:Exps
  erro = Inf; %erro inicial
  G = root; %árvore
  
  qrand = limites_inferior + (limites_superior - limites_inferior).*rand(7,1);
  
  [posD,juntas] = cinematica_direta3(qrand); %posição desejada
  posD = posD(1:3);
%   posD = [0;-0.2;0.1];
  Xgoal = posD;
  
  repeticao = Exp
  for k = 1:K
    for i = 1:n
      if(i < n)
        %% Geração de p_rand
        th = 2*pi*rand(1,1);
        fi = 2*pi*rand(1,1);
        raio = r(i+1); % comprimento da cadeia entre a junta superior até o efetuador
        
        P_rand  = [raio*sin(fi)*cos(th);
          raio*sin(fi)*sin(th);
          raio*cos(fi)]; %ponto aleatório na esfera
        
        P_rand = P_rand + posD; %desloca o centro da esfera
        
        idcs_parents = find(G(7,:) == i-1); %encontra os nós de índice i-1
        if(isempty(idcs_parents))
          continue
        end
        g = G(1:6,idcs_parents); %nós de índice i-1
        
        [valor idc_parent] = min(abs(distancias3(P_rand,g(1:3,:),g(4:6,:),d(i)) - d(i+1))); %nós mais próximo
        idc_parent = idcs_parents(idc_parent); %em G
        V_parent = G(4:6,idc_parent); %vetor
        P_parent = G(1:3,idc_parent);
        %% Projetanto ponto no plano
        P_rand2 = proj_ponto_plano(V_parent,P_parent,P_rand);
        %% MB-RRT padrão
        v = (P_rand2 - P_parent)/norm(P_rand2 - P_parent);
        P_new = d(i)*v + P_parent;
        p2 = P_parent;
        V_new = rotacionar_vetor(V_parent,-v,alpha(i)); %alpha1
        
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
        %% cálculo do segundo ponto
        p = P_new;
        normal = V_new;
        v = (P_rand - p)/norm(P_rand - p);
        
        V_new2 = rotacionar_vetor(normal,-v,alpha(i+1));
        P_new2 = d(i+1)*v + p;
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
        %% checagem de ângulo 1
        if(i == 1)
          V_ref = [0;1;0]; %y
        else
          idc_grandparent = G(8,idc_parent);
          V_grandparent = G(4:6,idc_grandparent);
          P_grandparent = G(1:3,idc_grandparent);
          V_ref = P_parent - P_grandparent;
          V_ref = V_ref/norm(V_ref);
        end
        v = rotacionar_vetor(V_ref,V_parent,pi/2); %vetor ortogonal a V_new
        V_aux = P_new - P_parent;
        V_aux = V_aux/norm(V_aux);
        q_i = acos(V_aux'*V_ref);
        if(V_aux'*v < 0)
          q_i = -q_i;
        end
        if(q_i > limites_superior(i) | q_i < limites_inferior(i))
          continue
        end
        %% checagem de ângulo 2
        
        V_ref = P_new - P_parent;
        V_ref = V_ref/norm(V_ref);
        
        v = rotacionar_vetor(V_ref,V_new,pi/2); %vetor ortogonal a V_new
        V_aux = P_new2 - P_new;
        V_aux = V_aux/norm(V_aux);
        q_i = acos(V_aux'*V_ref);
        if(V_aux'*v < 0)
          q_i = -q_i;
        end
        if(q_i > limites_superior(i+1) | q_i < limites_inferior(i+1))
          continue
        end
        %% adição do nó 1 na árvore
        G = [G, [P_new;V_new;i;idc_parent]];
        %% Plot do primeiro ponto
        y = V_new;
        x = G(4:6,idc_parent);
        z = cross(x,y);
        
        P_parent = G(1:3,idc_parent);
        
        A(1:3,end) = P_new;
        A(end,end) = 1;
        A(1:3,1:3) = [x y z];
        
        %       %pause(0.1)
%         plot_junta_revolucao(A,[0;-h/2;0],'y',h,radius,color);
%         plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
%           ,'Color',color,'linewidth',2)
        
        %% adição do nó 2 na árvore
        G = [G, [P_new2;V_new2;i+1;size(G,2)]];
        
        %% Plot do segundo ponto
        y = V_new2;
        x = G(4:6,G(end,end));
        z = cross(x,y);
        
        P_parent = G(1:3,G(end,end));
        
        A(1:3,end) = P_new2;
        A(end,end) = 1;
        A(1:3,1:3) = [x y z];
        
        %pause(0.1)
        if(i == n-1)
%           scatter3(P_new2(1),P_new2(2),P_new2(3),'k','filled','linewidth',3)
        else
%           plot_junta_revolucao(A,[0;-h/2;0],'y',h,radius,color);
        end
%         plot3([P_new2(1) P_new(1)],[P_new2(2) P_new(2)],[P_new2(3) P_new(3)]...
%           ,'Color',color,'linewidth',2);
      end
      if(i == n && G(7,end) == n)
        erro = sqrt(sum((posD -G(1:3,end)).^2));
      end
      
    end
    if(erro < erro_min)
      break
    end
    
  end
  
  if(erro < erro_min)
    
    p0 = G(:,end);
    P2 = p0;
    
    for i = 1:n
      p0 = G(:,p0(8));
      P2 = [P2 p0];
    end
    V2 = P2(4:6,end:-1:1);
    P2 = P2(1:3,end:-1:1);
    q = angulos2(P2);
    
    %% Plot
    A = eye(4);
    P_parent = P2(:,1);
    p_new = P2(:,2);
%     plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,'b');
%     plot3([p_new(1) P_parent(1)],[p_new(2) P_parent(2)],[p_new(3) P_parent(3)]...
%       ,'b','linewidth',5)
    
    for i = 2:n
      y = V2(:,i);
      x = V2(:,i-1);
      z = cross(x,y);
      
      p_new = P2(:,i);
      P_parent = P2(:,i-1);
      
      A(1:3,end) = p_new;
      A(end,end) = 1;
      A(1:3,1:3) = [x y z];
%       plot3([p_new(1) P_parent(1)],[p_new(2) P_parent(2)],[p_new(3) P_parent(3)]...
%         ,'Color',[0,0,1],'linewidth',5)
%       plot_junta_revolucao(A,[0;-h/2;0],'y',h,radius,[0,0,1]);
    end
    P_parent = P2(:,end-1);
    p_new = P2(:,end);
%     plot3([p_new(1) P_parent(1)],[p_new(2) P_parent(2)],[p_new(3) P_parent(3)]...
%       ,'b','linewidth',3)
    if(i < n)
%       scatter3(p_new(1),p_new(2),p_new(3),'b','filled','linewidth',3)
    end
    %% se quiser testar
    [p,juntas] = cinematica_direta3(q);
    G = [juntas(1:3,:) p(1:3)];
    erro = sqrt(sum((posD -G(1:3,end)).^2));
    if(erro < erro_min)
      convergiu = 1;
    else
      conerviu = 0;
    end
%     k
%     erro
%     angulos = q*(180/pi)
  end
  taxa_convergiu = [taxa_convergiu convergiu];
  media_iteracao = [media_iteracao k];
end

media_iteracao = media_iteracao(find(taxa_convergiu));
desvio_iteracao = std(media_iteracao)
media_iteracao = mean(media_iteracao) 

taxa_convergiu = sum(taxa_convergiu)/Exps * 100