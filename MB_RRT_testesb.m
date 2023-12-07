%% MB-RRT for 7DOF manipulator
clc
clear
close all

%% Parâmetros do robô
limites_superior = [pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];
limites_inferior = -[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];
radius = 0.005; %ráio dos elos do robô
h = 0.025;
color = [0.1 0.1 0.1]; %cor do robô 
d = [0.075+0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]; %comprimento do elos
root = [0;0;0;0;0;1;1;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 7; %número de juntas do manipulador
r = cumsum(d(end:-1:1)); %raio das circunferências em volta do destino
r = [r(end-1:-1:1),0];

%% configuração do experimento
erro_min = 0.001;
K = 1000; %número máximo de iteração

qrand = -pi + 2*pi*rand(7,1);
[posD,juntas,eixos] = cinematica_direta(qrand);
posD = posD(1:3); %posição desejada
Xgoal = posD;


%% Inicialzação da MB-RRT
erro = Inf; %erro inicial
G = root; %árvore
P_new = d(1)*G(4:6,1) + G(1:3,1); %primeira junta é fixa devido a estrutura do manipulador
G = [G, [P_new ; zeros(3,1);2;1]]; %ainda não sei o eixo de atuação

%% Plot inicial
A = eye(4);
plot_junta_revolucao(A,[0;0;-h/2],'z',h,radius,'g');
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

title("Manipulator-Based Rapidly Random Tree")
hold on
scatter3(Xgoal(1),Xgoal(2),Xgoal(3),'r','filled','linewidth',3)

ambiente
legend("Base","Destino","Obstáculos",'AutoUpdate','off')

plot_esfera(P_new,2*radius,color,1);
P_parent = G(1:3,1);
plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        ,'Color',color,'linewidth',2);

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
      g = G(1:3,idcs_parents);
      [valor idc_parent] = min(abs(distancias(P_rand,g) - (d(i)+d(i+1)))); %nós mais próximo
      idc_parent = idcs_parents(idc_parent);
      P_parent = G(1:3,idc_parent);
      V_new = (P_rand - P_parent)/norm(P_rand - P_parent);
      P_new = d(i)*V_new + P_parent;
      P_new2 = d(i+1)*V_new + P_new;
      %% Adição do primeiro nó da árvore
      G = [G, [P_new;V_new;i+1;idc_parent]];
      %% Plot do primeiro ponto
      y = V_new;
      idc_grandparent = G(8,idc_parent);
      P_grandparent = G(1:3,idc_grandparent);
      v1 = P_grandparent - P_parent;
      v2 = P_new - P_parent;
      v = cross(v1,v2);
      v = v/norm(v);
      x = v;
      z = cross(x,y);
      A = eye(4);
      A(1:3,1:3) = [x y z];
      A(1:3,end) = P_new;
      plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        ,'Color',color,'linewidth',2)
      plot_junta_revolucao(A,[0;-h/2;0],'y',h,radius,color);
      %% Adição do segundo nó na árvore
      G = [G, [P_new2;zeros(3,1);i+2;size(G,2)]];
      %% Plot do segundo nó
      plot3([P_new(1) P_new2(1)],[P_new(2) P_new2(2)],[P_new(3) P_new2(3)]...
        ,'Color',color,'linewidth',2)
      plot_esfera(P_new2,2*radius,color,1);
      pause(0.1)
    elseif(i == 6)
      
      raio = r(i); % comprimento da cadeia entre a junta superior até o efetuador
      P_rand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
        raio*sin(2*pi*fi)*sin(2*pi*th);
        raio*cos(2*pi*fi)]; %ponto aleatório na esfera
      P_rand = P_rand + posD; %desloca o centro da esfera
      
      idcs_parents = find(G(7,:) == i); %encontra as colunas dos nós de índice i
      g = G(1:3,idcs_parents);
      [valor idc_parent] = min(abs(distancias(P_rand,g) - d(i)));
      idc_parent = idcs_parents(idc_parent);
      P_parent = G(1:3,idc_parent);
      
      v = (P_rand - P_parent)/norm(P_rand - P_parent);
      P_new = d(i)*v + P_parent;
      %% Adicionando nó a árvore
      G = [G, [P_new;zeros(3,1);i+1;idc_parent]];
      %% Plot do nó na árvore
      plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        ,'Color',color,'linewidth',2)
      plot_esfera(P_new,2*radius,color,1);
      pause(0.1)
    elseif(i == 7)
      P_rand = posD;
      %% Projetanto ponto no plano
      A = G(:,G(8,end));
      B = G(:,A(8));
      v1 = G(1:3,end) - A(1:3);
      v1 = v1/norm(v1);
      v2 = B(1:3) - A(1:3);
      v2 = v2/norm(v2);
      v3 = cross(v2,v1);
      v3 = v3/norm(v3);
      normal = rotacionar_vetor(v3,v1,pi/2); %v3 em torno de v1
      normal = normal/norm(normal);
      P_rand = proj_ponto_plano(normal,G(1:3,end),P_rand);
      %% MB-RRT padrão
      P_parent = G(1:3,end);
      v = (P_rand - P_parent)/norm(P_rand - P_parent);
      P_new = 0.075*v + P_parent;
      %% Adição do nó na árvore
      G = [G, [P_new;zeros(3,1);i+1;size(G,2)]];
      %% Plot do nó na árvore
      plot3([P_new(1) P_parent(1)],[P_new(2) P_parent(2)],[P_new(3) P_parent(3)]...
        ,'Color',color,'linewidth',2)
      plot_esfera(P_new,2*radius,color,1);
      pause(0.1)
    end
    
    if(i == n)
      erro = sqrt(sum((posD -G(1:3,end)).^2));
    end
  end
  if(erro < erro_min)
    erro_final = erro
    break 
  end
end

p0 = G(:,end);
P2 = p0;

for i = 1:n
  p0 = G(:,p0(8));
  P2 = [P2 p0];
end

P2 = P2(1:3,end:-1:1);
q = angulos(P2);
[p,juntas,eixos] = cinematica_direta(q);
G = [juntas(1:3,:) p(1:3)];
%% Plot
for i = 1:7
  plot3([G(1,i) G(1,i+1)],[G(2,i) G(2,i+1)],[G(3,i) G(3,i+1)]...
        ,'g','linewidth',2)
  
end
%% se quiser testar
angulos = q*(180/pi)

erro = sqrt(sum((posD -G(1:3,end)).^2))
if(erro < erro_min)
  convergiu = 1;
end

