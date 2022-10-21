clc

q=load('Section1D');
Qs=q.Q;

plot3(Qs(1,:),Qs(2,:),Qs(3,:),'k*')    % plot a section of 3D workspace
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal