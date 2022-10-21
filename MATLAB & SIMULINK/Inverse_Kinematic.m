
%% INVERSE KINEMATICS

Offset=170;
L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;
 
%  coordinates of end effector:
x=500;
y=500;
z=500;
%  orientation of end effector (Re0):
r=[1 0 0;0 1 0;0 0 1];

a2=L3;
a3=0;
d3=0;
d4=L5+L6;

K=(x^2+y^2+z^2-(a2^2+a3^2+d3^2+d4^2))/(2*a2);

theta1(1)=atan2(y,x);
theta1(2)=atan2(y,x)+pi;
theta3(1)=asin(((L0-z)^2/2-L3^2/2-(L5+L6)^2/2+x^2/2+y^2/2)/(L3*(L5+L6)));
theta3(2)=pi-asin(((L0-z)^2/2-L3^2/2-(L5+L6)^2/2+x^2/2+y^2/2)/(L3*(L5+L6)));

theta23=zeros(2);
theta2=zeros(2);
theta4=zeros(2);
theta5=zeros(2);
theta6=zeros(2);

for i=1:2
    for j=1:2
        c23(i,j)=(L3*((z-L0)*cos(theta3(j))-(-x*cos(theta1(i))-y*sin(theta1(i)))*sin(theta3(j)))-(-x*cos(theta1(i))-y*sin(theta1(i)))*(L5+L6))/((z-L0)^2+(-x*cos(theta1(i))-y*sin(theta1(i)))^2);%#ok
        s23(i,j)=(L3*cos(theta3(j))-(z-L0)*c23(i,j))/(-x*cos(theta1(i))-y*sin(theta1(i)));%#ok
        theta23(i,j)=atan2(s23(i,j),c23(i,j));
        
        theta2(i,j)=theta23(i,j)-theta3(j);
        
        theta4(i,j)=atan2(r(1,3)*sin(theta1(i))-r(2,3)*cos(theta1(i)),...
            r(3,3)*cos(theta23(i,j))-r(1,3)*sin(theta23(i,j))*cos(theta1(i))-r(2,3)*sin(theta23(i,j))*sin(theta1(i)));
        
        s5(i,j)=r(3,3)*cos(theta23(i,j))*cos(theta4(i,j))-r(1,3)*(cos(theta1(i))*cos(theta2(i,j))*cos(theta4(i,j))*sin(theta3(j))-sin(theta1(i))*sin(theta4(i,j))+cos(theta1(i))*cos(theta3(j))*cos(theta4(i,j))*sin(theta2(i,j)))...
            -r(2,3)*(cos(theta1(i))*sin(theta4(i,j))+cos(theta2(i,j))*cos(theta4(i,j))*sin(theta1(i))*sin(theta3(i))+cos(theta3(j))*cos(theta4(i,j))*sin(theta1(i))*sin(theta2(i,j)));%#ok
        c5(i,j)=r(3,3)*sin(theta23(i,j))+r(1,3)*cos(theta23(i,j))*cos(theta1(i))+r(2,3)*cos(theta23(i,j))*sin(theta1(i));%#ok
        theta5(i,j)=atan2(s5(i,j),c5(i,j));
        
        s6(i,j)=-r(1,1)*(cos(theta1(i))*cos(theta2(i,j))*cos(theta3(j))*sin(theta5(i,j))-cos(theta5(i,j))*sin(theta1(i))*sin(theta4(i,j))-cos(theta1(i))*sin(theta2(i,j))*sin(theta3(j))*sin(theta5(i,j))+...
            cos(theta1(i))*cos(theta2(i,j))*cos(theta4(i,j))*cos(theta5(i,j))*sin(theta3(j))+cos(theta1(i))*cos(theta3(j))*cos(theta4(i,j))*cos(theta5(i,j))*sin(theta2(i,j)))-...
            r(2,1)*(cos(theta1(i))*cos(theta5(i,j))*sin(theta4(i,j))+cos(theta2(i,j))*cos(theta3(j))*sin(theta1(i))*sin(theta5(i,j))-sin(theta1(i))*sin(theta2(i,j))*sin(theta3(j))*sin(theta5(i,j))+...
            cos(theta2(i,j))*cos(theta4(i,j))*cos(theta5(i,j))*sin(theta1(i))*sin(theta3(j))+cos(theta3(j))*cos(theta4(i,j))*cos(theta5(i,j))*sin(theta1(i))*sin(theta2(i,j)))-r(3,1)*(cos(theta2(i,j))*sin(theta3(j))*sin(theta5(i,j))+...
            cos(theta3(j))*sin(theta2(i,j))*sin(theta5(i,j))-cos(theta2(i,j))*cos(theta3(j))*cos(theta4(i,j))*cos(theta5(i,j))+cos(theta4(i,j))*cos(theta5(i,j))*sin(theta2(i,j))*sin(theta3(j)));%#ok
        c6(i,j)=r(1,1)*(cos(theta4(i,j))*sin(theta1(i))+cos(theta1(i))*cos(theta2(i,j))*sin(theta3(j))*sin(theta4(i,j))+cos(theta1(i))*cos(theta3(j))*sin(theta2(i,j))*sin(theta4(i,j)))+...
            r(2,1)*(cos(theta2(i,j))*sin(theta1(i))*sin(theta3(j))*sin(theta4(i,j))-cos(theta1(i))*cos(theta4(i,j))+cos(theta3(j))*sin(theta1(i))*sin(theta2(i,j))*sin(theta4(i,j)))-r(3,1)*cos(theta2(i,j)+theta3(j))*sin(theta4(i,j));%#ok
        theta6(i,j)=atan2(c6(i,j),s6(i,j));
        
    end
end
% all 8 set of answers :
theta4prime=theta4-[pi pi; pi pi];
theta5prime=-theta5;
theta6prime=theta6-[pi pi; pi pi];
ans1=[theta1(1)
theta2(1)
theta3(1)
theta4(1)
theta5(1)
theta6(1)];

ans2=[theta1(2)
theta2(2)
theta3(1)
theta4(2)
theta5(2)
theta6(2)];

ans3=[theta1(1)
theta2(3)
theta3(2)
theta4(3)
theta5(3)
theta6(3)];

ans4=[theta1(1)
theta2(4)
theta3(2)
theta4(4)
theta5(4)
theta6(4)];

ans5=[theta1(1)
theta2(1)
theta3(1)
theta4prime(1)
theta5prime(1)
theta6prime(1)];

ans6=[theta1(2)
theta2(2)
theta3(1)
theta4prime(2)
theta5prime(2)
theta6prime(2)];

ans7=[theta1(1)
theta2(3)
theta3(2)
theta4prime(3)
theta5prime(3)
theta6prime(3)];

ans8=[theta1(1)
theta2(4)
theta3(2)
theta4prime(4)
theta5prime(4)
theta6prime(4)];

all8ans=[ ans1 ans2 ans3 ans4 ans5 ans6 ans7 ans8]
%% Inverse Kineniatic Verification

L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;

t1=all8ans(1,2);
t2=all8ans(2,2);
t3=all8ans(3,2);
t4=all8ans(4,2);
t5=all8ans(5,2);
t6=all8ans(6,2);

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

T16=T12*T23*T34*T45*T56;
T06=T01*T12*T23*T34*T45*T56;
R06=T06(1:3,1:3);               % Position of End-effector
P06=T06(1:3,end);               % Rotation Matrix of End-Effector

P06
R06