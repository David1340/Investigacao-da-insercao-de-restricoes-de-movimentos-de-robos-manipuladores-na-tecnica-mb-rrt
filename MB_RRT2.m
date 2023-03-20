clc
clear
close all

d = [10,10,10,10]; %comprimento do elos
root = [0;0;0;0;0;1;0;0]; %x,y,z,eixo de rotação, índice da cadeira, nó pai 
n = 4; %número de juntas do manipulador
r = (n -1)*10:-10:0; %raio das circunferências em volta do destino

%configuração do experimento
posD = [15;0;15]; %posição desejada

Xgoal = posD;
q = zeros(n,1); %configuração inicial
K = 500; %número máximo de iteração
figure()
plot3(root(1),root(2),root(3),'d'); %plot da base
hold on
grid on
plot3(posD(1),posD(2),posD(3),'*','linewidth',5); %plot da posição desejada
legend('root','destino')

erro = Inf; %erro inicial
P = root; %árvore
color = ['b','g','r','c','m','y','k']; %diferentes cores

Xnew = d(1)*P(4:6,1) + P(1:3,1);
P = [P, [Xnew ; zeros(3,1);1;1]]; 
plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(1));

for k = 1:K
  for i = 2:n
     th = rand(1,1);
     fi = rand(1,1);
    if(i == 2)
     raio = 10;
     Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
       raio*sin(2*pi*fi)*sin(2*pi*th);
       raio*cos(2*pi*fi)];
     Xrand = Xrand + posD;
     pos = find(P(7,:) == i -1); %encontra as colunas dos nós de índice i -1
     p = P(1:3,pos);
     [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - raio)); %nós mais próximo
     p = p(1:3,pos2);
     v = (Xrand - p)/norm(Xrand - p);
     Xnew1 = d(i)*v + p;
     Xnew2 = d(i+1)*v + Xnew1;
     P = [P, [Xnew1;zeros(3,1);i;pos(pos2)]];
     plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i));
     P = [P, [Xnew2;v;i+1;size(P,2)]];
     plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i+1));
    elseif(i==4)
      raio = r(i);
      Xrand  = [raio*sin(2*pi*fi)*cos(2*pi*th);
       raio*sin(2*pi*fi)*sin(2*pi*th);
       raio*cos(2*pi*fi)];
     Xrand = Xrand + posD;
     pos = find(P(7,:) == i -1); %encontra as colunas dos nós de índice i -1
     p = P(1:3,pos);
     [valor pos2] = min(abs(distancias(Xrand,p(1:3,:)) - 10)); %nós mais próximo
     p = p(1:3,pos2);
     v = (Xrand - p)/norm(Xrand - p);
     Xnew = d(i)*v + p;
     P = [P, [Xnew;zeros(3,1);i;pos(pos2)]];
     plot3([P(1,P(8,end)) P(1,end)],[P(2,P(8,end)) P(2,end)],[P(3,P(8,end)) P(3,end)],color(i));
    end

    drawnow
      if(i == n)
        erro = sqrt(sum((posD -Xnew).^2));
      end
  end
    if(erro < n*10*0.001)
      convergiu = 1;
      break
    end
end
k
erro

% r = 10;
% [x,y,z] = sphere();
% x = r*x;
% y = r*y;
% z = r*z;
% 
% surf(x+posD(1),y+posD(2),z+posD(3));
% axis equal