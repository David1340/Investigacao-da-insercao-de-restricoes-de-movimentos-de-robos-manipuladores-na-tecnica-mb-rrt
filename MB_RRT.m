clc
clear
close all

%par�metros do manipulador
DH = [0,10,0,0]; %d , a, alpha, theta
n = 4; %n�mero de juntas do manipulador
root = [0;0;0;0]; %x,y, �ndice da cadeira, n� pai 
r = (n-1)*10:-10:0; %raio das circunfer�ncias em volta do destino

%configura��o do experimento
posD = [20;30]; %posi��o desejada

Xgoal = posD;
%q = -pi + 2*pi*rand(n,1);
q = zeros(n,1); %configura��o inicial
K = 1000; %n�mero m�ximo de itera��o
figure()
plot(root(1),root(2),'d'); %plot da base
hold on
plot(posD(1),posD(2),'*','linewidth',5); %plot da posi��o desejada
legend('root','destino')
erro = Inf; %erro inicial
P = root; %�rvore
color = ['b','g','r','c','m','y','k']; %diferentes cores

for k = 1:K
  for i = 1:n
    th = rand(1,1);
    Xrand  = [r(i)*cos(2*pi*th);r(i)*sin(2*pi*th)];
    Xrand = Xrand + posD;
    pos = find(P(3,:) == i -1); %encontra as colunas dos n�s de �ndice i -1
    p = P(1:2,pos);
    [valor pos2] = min(abs(distancias(Xrand,p) - DH(2))); %n�s mais pr�ximo
    p = p(:,pos2);
    Xnew = DH(2)*(Xrand - p)/norm(Xrand - p) + p;
    P = [P, [Xnew;i;pos(pos2)]];
    plot([P(1,P(4,end)) P(1,end)],[P(2,P(4,end)) P(2,end)],color(i));
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
