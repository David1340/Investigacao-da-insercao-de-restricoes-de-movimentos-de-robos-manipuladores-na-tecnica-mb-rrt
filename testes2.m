q(end) = 0;
[p,juntas] = cinematica_direta(q);
P3 = [juntas(1:3,:) p(1:3)];
for i = 2:n+1
  plot3([P3(1,i-1) P3(1,i)],[P3(2,i-1) P3(2,i)],[P3(3,i-1) P3(3,i)],color(i-1));
end

figure()
plot3([P(1,end-2) P(1,end-1)],[P(2,end-2) P(2,end-1)],[P(3,end-2) P(3,end-1)]);
hold on
grid on
plot3([P2(1,end-2) P2(1,end-3)],[P2(2,end-2) P2(2,end-3)],[P2(3,end-2) P2(3,end-3)])

plot3([P(1,end) P(1,end-1)],[P(2,end) P(2,end-1)],[P(3,end) P(3,end-1)])
plot3([P3(1,end) P3(1,end-1)],[P3(2,end) P3(2,end-1)],[P3(3,end) P3(3,end-1)])
legend('v1','v2','v4','v5')
v1 = P2(:,end-1) - P2(:,end-2);
v1 = v1/norm(v1);
v2 = P2(:,end-3) - P2(:,end-2);
v2 = v2/norm(v2);
v3 = cross(v2,v1);
normal = rotacionar_vetor(v3,v1,pi/2); 
normal = normal/norm(normal)

v4 = P(:,end) - P(:,end-1);
v4 = v4/norm(v4);
v5 = P3(:,end) - P3(:,end-1);
v5 = v5/norm(v5);
normal = cross(v5,v4);
normal = normal/norm(normal)