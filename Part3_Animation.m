DATA=load('Run04');
p01=DATA.p01;
p03=DATA.p03;
p04=DATA.p04;
p1_a=DATA.p1_a;
p2_a=DATA.p2_a;
p2_b=DATA.p2_b;
p3_b=DATA.p3_b;
p4_b=DATA.p4_b;

for k=1:3
    for i=1:30
        
        plot3([0 p01(1,i,k)],[0 p01(2,i,k)],[170 p01(3,i,k)],'k','LineWidth',1.8);                                    %Link1
        hold on
        plot3([p01(1,i,k) p1_a(1,i,k)],[p01(2,i,k) p1_a(2,i,k)],[p01(3,i,k) p1_a(3,i,k)],'k','LineWidth',1.8);
        
        plot3([p1_a(1,i,k) p2_a(1,i,k)],[p1_a(2,i,k) p2_a(2,i,k)],[p1_a(3,i,k) p2_a(3,i,k)],'b','LineWidth',1.8);   % Link2
        plot3([p2_a(1,i,k) p2_b(1,i,k)],[p2_a(2,i,k) p2_b(2,i,k)],[p2_a(3,i,k) p2_b(3,i,k)],'b','LineWidth',1.8);
        
        plot3([p2_b(1,i,k) p03(1,i,k)],[p2_b(2,i,k) p03(2,i,k)],[p2_b(3,i,k) p03(3,i,k)],'r','LineWidth',1.8);      % Link3   
        plot3([p03(1,i,k) p3_b(1,i,k)],[p03(2,i,k) p3_b(2,i,k)],[p03(3,i,k) p3_b(3,i,k)],'r','LineWidth',1.8);
        
        plot3([p3_b(1,i,k) p04(1,i,k)],[p3_b(2,i,k) p04(2,i,k)],[p3_b(3,i,k) p04(3,i,k)],'g','LineWidth',1.8);      % Link4
        plot3([p04(1,i,k) p4_b(1,i,k)],[p04(2,i,k) p4_b(2,i,k)],[p04(3,i,k) p4_b(3,i,k)],'y','LineWidth',1.8);      % Link5
        
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        axis equal
        xlim([-1200 1200])
        ylim([-1000 1000])
        zlim([-1100 2000])
        grid on
        pause(0.05)
        hold off

    end
    pause;
end

