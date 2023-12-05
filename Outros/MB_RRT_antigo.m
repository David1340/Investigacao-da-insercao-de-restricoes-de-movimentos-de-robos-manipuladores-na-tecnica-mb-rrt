clc
clear
close all

%parâmetros do manipulador
DH = [0,10,0,0]; %d , a, alpha, theta
n = 3; %número de juntas do manipulador
root = [0;0;0;0]; %x,y, índice da cadeira, nó pai 
r = (n-1)*10:-10:0; %raio das circunferências em volta do destino

%configuração do experimento
posD = [15;15];

Xgoal = posD;
q = -pi + 2*pi*rand(n,1);
q = zeros(n,1);
K = 1000;
figure()
plot(root(1),root(2),'*');
hold on
plot(posD(1),posD(2),'*','linewidth',5);
legend('root','destino')
erro = Inf;
menor_erro = Inf;
P = root;
color = ['b','g','r','c','m','y','k'];
vezes = 0;
U = 10000;
kk = [];
for u = 1:U
  convergiu = 0;
  th = rand(1,1);
  posD = 30*rand(1,1) * [cos(th);sin(th)];
  for k = 1:K
    for i = 1:n
      %     r = 40;
      %     Xrand  = [r*rand(1,1)*cos(2*pi*rand(1,1));r*rand(1,1)*sin(pi*rand(1,1))];
      th = rand(1,1);
      Xrand  = [r(i)*cos(2*pi*th);r(i)*sin(2*pi*th)];
      Xrand = Xrand + posD;
      pos = find(P(3,:) == i -1);
      p = P(1:2,pos);
      [valor pos2] = min(abs(distancias(Xrand,p) - DH(2)));
      p = p(:,pos2);
      Xnew = DH(2)*(Xrand - p)/norm(Xrand - p) + p;
      P = [P, [Xnew;i;pos(pos2)]];
%       plot([P(1,P(4,end)) P(1,end)],[P(2,P(4,end)) P(2,end)],color(i));
%       drawnow
      if(i == n)
        erro = sqrt(sum((posD -Xnew).^2));
      end
    end
    if(erro < n*10*0.01)
      convergiu = 1;
      break
    end
  end
  if(convergiu == 1)
    vezes = vezes + 1;
    kk = [kk  k];
  end
  P = root;
  erro = Inf;
end
mean(kk)
vezes / U * 100