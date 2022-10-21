clc

q=load('Run01');
Q=q.Q;

plot3(Q(1,:),Q(2,:),Q(3,:),'k*')
axis equal
xlim([-1500 1500])
ylim([-1500 1500])
zlim([-1000 1800])
xlabel('X')
ylabel('Y')
zlabel('Z')