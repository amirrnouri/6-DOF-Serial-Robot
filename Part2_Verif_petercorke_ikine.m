clc

%% DH table and Calculation
L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;
dh=[ 
    0 L0 0 pi/2
    0 0 L3 0
    pi/2 0 0 pi/2
    0 L5+L6 0 -pi/2
    0 0 0 pi/2
    0 0 0 0];
r=SerialLink(dh);
r.name='Kawasaki RS013N';

%  coornitades and orientation of end effector:
T0e=[1 0 0 -30
    0 1 0 -320
    0 0 1 50
    0 0 0 1]
%  T0e(1:3,1:3)=T06(1:3,1:3)
 
q=r.ikine(T0e);
 t1=q(1);
 t2=q(2)-pi/2;
 t3=q(3);
 t4=q(4);
 t5=q(5);
 t6=q(6);
 Q=vpa([t1 t2 t3 t4 t5 t6]',4)
%% Robot Animating Plot
T1=linspace(0,t1,30);
T2=linspace(0,t2,30);
T3=linspace(0,t3,30);
T4=linspace(0,t4,30);
T5=linspace(0,t5,30);
T6=linspace(0,t6,30);

for i=1:30
    r.plot([T1(i) T2(i)+pi/2 T3(i) T4(i) T5(i) T6(i)])
    pause(0.1)
end