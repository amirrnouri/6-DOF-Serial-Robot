%% FORWARD KINEMATICS
clc
clear

Offset=170;
L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;

syms t1 t2 t3 t4 t5 t6 real

% t1=0;
% t2=0;
% t3=0;
% t4=0;
% t5=0;
% t6=0;

% Transformation Matrices
T01=[cos(t1) -sin(t1) 0 0
     sin(t1)  cos(t1) 0 0
     0 0 1 L0
     0 0 0 1];

T12=[-sin(t2) -cos(t2) 0 0
     0 0 -1 0
     cos(t2)  -sin(t2) 0 0
     0 0 0 1];

T23=[cos(t3) -sin(t3) 0 L3
     sin(t3)  cos(t3) 0 0
     0 0 1 0
     0 0 0 1];

T34=[cos(t4) -sin(t4) 0 0
     0 0 -1 -L5-L6
     sin(t4)  cos(t4) 0 0
     0 0 0 1];

T45=[cos(t5)  -sin(t5) 0 0
     0 0 1 0
     -sin(t5) -cos(t5) 0 0
     0 0 0 1];

T56=[cos(t6) -sin(t6) 0 0
     0 0 -1 0
     sin(t6)  cos(t6) 0 0
     0 0 0 1];
 
P01=T01(1:3,end);
T02=T01*T12;
P02=T02(1:3,end);
T03=T01*T12*T23;
P03=T03(1:3,end);
T04=T01*T12*T23*T34;
P04=T04(1:3,end);
T05=T01*T12*T23*T34*T45;
P05=T05(1:3,end);

T06=T01*T12*T23*T34*T45*T56;
R06=T06(1:3,1:3);               % Positin of End-effector
P06=T06(1:3,end);               % Rotation Matrix of End-Effector