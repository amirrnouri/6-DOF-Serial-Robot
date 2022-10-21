
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

%% Newton-Euler Dynamic Iteration
syms thetadot [1 6]
syms thetaddot [1 6]
syms w [3 6]
syms wdot [3 6]
syms vdot [3 6]
syms F [3 6]
syms f [3 6]
syms N [3 6]
syms n [3 6]
syms g
% manual adding links' angular velocity and acceleration:
% thetadot=[1 1 1 1 1 1];
% thetaddot=[0 0 0 0 0 0];

%end effector forces and torques:
f(1:3,7)=[ 0; 0;0];
n(1:3,7)=[0 ;0; 0];
% mass of links:
m=[ 21.879 16.483 9.291 6.618 0.535 0.019] ;
% position oc center of mass of links in their own frame: (computed by
% CAD softwares)
Pc=[ [0 34.7066 -116.3371]' [279.892 0 89.869]' [0 0 -72.154]' [0 0 -350.286]' [0 -10.939 0]' [0 0 78]' ]/1000;
% Moment of inertia matrices of links about their center of mass:(computed by
% CAD softwares)
I(1:3,1:3)=[ 0.432 0 0
    0 0.386 -0.027
    0 -0.027 0.203];
I(1:3,4:6)=[ 0.064 0 0.061
    0 1.194 0
    0.061 0 1.206];
I(1:3,7:9)=[ 0.06 0 -0.001
     0 0.068 0
     -0.001 0 0.065];
I(1:3,10:12)= [ 0.267 0 -0.001
      0 0.269 0
      -0.001 0 0.013];
I(1:3,13:15)= [ 0.001 0 0
       0 0.001 0
        0 0 0.001];
I(1:3,16:18)= [ 0.0001 0 0
       0 0.0001 0
        0 0 0.0001];

R_i=[ transpose(T01(1:3,1:3)) transpose(T12(1:3,1:3)) transpose(T23(1:3,1:3)) transpose(T34(1:3,1:3)) transpose(T45(1:3,1:3)) transpose(T56(1:3,1:3)) [1 0 0;0 1 0; 0 0 1]];
P_i=[T01(1:3,4) T12(1:3,4) T23(1:3,4) T34(1:3,4) T45(1:3,4) T56(1:3,4) T56(1:3,4)]/1000;
w(1:3,1)=[ 0 ;0; thetadot(1)];
wdot(1:3,1)=[0 ;0 ;thetaddot(1)];
vdot(1:3,1)=[0 ;0 ;g];
vcdot(1:3,1)=cross(wdot(1:3,1),Pc(1:3,1))+cross(w(1:3,1),cross(w(1:3,1),Pc(1:3,1)))+vdot(1:3,1);
F(1:3,1)=m(1)*vcdot(1:3,1);
N(1:3,1)=I(1:3,1:3)*wdot(1:3,1)+cross(w(1:3,1),I(1:3,1:3)*w(1:3,1));
 for i=2:6
     w(1:3,i)=R_i(:,i*3-2:i*3)*w(1:3,i-1)+[0 ;0 ;thetadot(i)];
     wdot(1:3,i)=R_i(:,i*3-2:i*3)*wdot(1:3,i-1)+cross(R_i(:,i*3-2:i*3)*w(1:3,i-1),[0 ;0 ;thetadot(i)])+[0; 0 ;thetaddot(i)];
     vdot(1:3,i)=R_i(:,i*3-2:i*3)*(cross(wdot(1:3,i-1),P_i(1:3,i))+cross(w(1:3,i-1),cross(w(1:3,i-1),P_i(1:3,i)))+vdot(1:3,i-1));
     vcdot(1:3,i)=cross(wdot(1:3,i),Pc(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),Pc(1:3,i)))+vdot(1:3,i);
     F(1:3,i)=m(i)*vcdot(1:3,i);
     N(1:3,i)=I(1:3,i*3-2:i*3)*wdot(1:3,i)+cross(w(1:3,i),I(1:3,i*3-2:i*3)*w(1:3,i)); 
 end
 
for i=6:-1:1
    f(1:3,i)=transpose(R_i(:,(i+1)*3-2:(i+1)*3))*f(1:3,i+1)+F(1:3,i);
    n(1:3,i)=N(1:3,i)+transpose(R_i(:,(i+1)*3-2:(i+1)*3))*n(1:3,i+1)+cross(Pc(1:3,i),F(1:3,i))+cross(P_i(1:3,i+1),transpose(R_i(:,(i+1)*3-2:(i+1)*3))*f(1:3,i+1)); 
   
end
  tau=n(3,1:6);

 %% Inertia Matrix 
 syms M [6 6]
 for i=1:6
     for j=1:6
        [mm mx]=coeffs(tau(j),thetaddot(i));
        s=size(mx);
        if s(2)~=2
            M(j,i)=0;
        else
       
        M(j,i)=vpa(mm(1));

        end
     end
 end
 
%% Gravity
 syms G [6 1]
 for i=1:6
     [gg gx]=coeffs(tau(i),g);
     s=size(gx);
     if s(2)~=2
            G(i,1)=0;
     else 
     G(i,1)=vpa(gg(1))*g;
     end
 end
 %% Coriolis and Centrifugal Matrix
syms C [6 6]
for i=1:6
     for j=1:6
        [cc cx]=coeffs(tau(j),thetadot(i));
         s=size(cx);
        if s(2)==3;
            C(j,i)=cc(2)+thetadot(i)*cc(1);
        end
        if s(2)==2
            if cx(1)==thetadot2^2
               C(j,i)=thetadot(i)*cc(1); 
            else
             C(j,i)=cc(1);
            end
        end
        if s(2)~=2 && s(2)~=3
            C(j,i)=0
        end
        
     end
end
%% Jacobian of CoM of all links
syms z1 z2 z3 z4 z5 z6 o1 o2 o3 o4 o5 o6
syms jw1 jw2 jw3 jw4 jw5 jw6 
syms jv1 jv2 jv3 jv4 jv5 jv6 
syms Tc11 [4 4]
syms Tc22 [4 4]
syms Tc33 [4 4]
syms Tc44 [4 4]
syms Tc55 [4 4]
syms Tc66 [4 4]

Tc11=subs(Tc11,Tc11,eye(4,4));
Tc22=subs(Tc22,Tc22,eye(4,4));
Tc33=subs(Tc33,Tc33,eye(4,4));
Tc44=subs(Tc44,Tc44,eye(4,4));
Tc55=subs(Tc55,Tc55,eye(4,4));
Tc66=subs(Tc66,Tc66,eye(4,4));

Tc11(1:3,4)=Pc(1:3,1)*1000;
Tc22(1:3,4)=Pc(1:3,2)*1000;
Tc33(1:3,4)=Pc(1:3,3)*1000;
Tc44(1:3,4)=Pc(1:3,4)*1000;
Tc55(1:3,4)=Pc(1:3,5)*1000;
Tc66(1:3,4)=Pc(1:3,6)*1000;

T0c1=T01*Tc11;
T0c2=T01*T12*Tc22;
T0c3=T01*T12*T23*Tc33;
T0c4=T01*T12*T23*T34*Tc44;
T0c5=T01*T12*T23*T34*T45*Tc55;
T0c6=T01*T12*T23*T34*T45*T56*Tc66;               
               

z1=T01(1:3,3);
z2=T02(1:3,3);
z3=T03(1:3,3);
z4=T04(1:3,3);
z5=T05(1:3,3);
z6=T06(1:3,3);
o1=T0c1(1:3,4)/1000;
o2=T0c2(1:3,4)/1000;
o3=T0c3(1:3,4)/1000;
o4=T0c4(1:3,4)/1000;
o5=T0c5(1:3,4)/1000;
o6=T0c6(1:3,4)/1000;

% link 1
jv1(1:3,1:6)=zeros(3,6);
jv1(1:3,1)=cross(z1,o1);
jw1(1:3,1:6)=zeros(3,6);
jw1(3,1)=1;
% link 2
jv2(1:3,1:6)=zeros(3,6);
jv2(1:3,1)=cross(z1,o2-o1);
jv2(1:3,2)=cross(z2,o2-T02(1:3,4)/1000);
jw2(1:3,1:6)=zeros(3,6);
jw2(1:3,1)=z1;
jw2(1:3,2)=z2;
% link 3
jv3(1:3,1:6)=zeros(3,6);
jv3(1:3,1)=cross(z1,o3-o1);
jv3(1:3,2)=cross(z2,o3-o2);
jv3(1:3,3)=cross(z3,o3-T03(1:3,4)/1000);
jw3(1:3,1:6)=zeros(3,6);  
jw3(1:3,1)=z1;
jw3(1:3,2)=z2;
jw3(1:3,3)=z3;
% link 4
jv4(1:3,1:6)=zeros(3,6);
jv4(1:3,1)=cross(z1,o4-o1);
jv4(1:3,2)=cross(z2,o4-o2);
jv4(1:3,3)=cross(z3,o4-o3);
jv4(1:3,4)=cross(z4,o4-T04(1:3,4)/1000);
jw4(1:3,1:6)=zeros(3,6);
jw4(1:3,1)=z1;
jw4(1:3,2)=z2;
jw4(1:3,3)=z3;
jw4(1:3,4)=z4;
% Link 5
jv5(1:3,1:6)=zeros(3,6);
jv5(1:3,1)=cross(z1,o5-o1);
jv5(1:3,2)=cross(z2,o5-o2);
jv5(1:3,3)=cross(z3,o5-o3);
jv5(1:3,4)=cross(z4,o5-o4);
jv5(1:3,5)=cross(z5,o5-T05(1:3,4)/1000);
jw5(1:3,1:6)=zeros(3,6);
jw5(1:3,1)=z1;
jw5(1:3,2)=z2;
jw5(1:3,3)=z3;
jw5(1:3,4)=z4;
jw5(1:3,5)=z5;
% Link 6
jv6(1:3,1:6)=zeros(3,6);
jv6(1:3,1)=cross(z1,o6-o1);
jv6(1:3,2)=cross(z2,o6-o2);
jv6(1:3,3)=cross(z3,o6-o3);
jv6(1:3,4)=cross(z4,o6-o4);
jv6(1:3,5)=cross(z5,o6-o5);
jv6(1:3,6)=cross(z6,o6-T06(1:3,4)/1000);
jw6(1:3,1:6)=zeros(3,6);
jw6(1:3,1)=z1;
jw6(1:3,2)=z2;
jw6(1:3,3)=z3;
jw6(1:3,4)=z4;
jw6(1:3,5)=z5;
jw6(1:3,6)=z6;

%% Lagrangian Dynamics
%% Inertia Matrix
syms ML [6 6]
ML=m(1)*transpose(jv1)*jv1+transpose(jw1)*T01(1:3,1:3)*I(1:3,1:3)*transpose(T01(1:3,1:3))*jw1;
ML=ML+m(2)*transpose(jv2)*jv2+transpose(jw2)*T02(1:3,1:3)*I(1:3,4:6)*transpose(T02(1:3,1:3))*jw2;
ML=ML+m(3)*transpose(jv3)*jv3+transpose(jw3)*T03(1:3,1:3)*I(1:3,7:9)*transpose(T03(1:3,1:3))*jw3;
ML=ML+m(4)*transpose(jv4)*jv4+transpose(jw4)*T04(1:3,1:3)*I(1:3,10:12)*transpose(T04(1:3,1:3))*jw4;
ML=ML+m(5)*transpose(jv5)*jv5+transpose(jw5)*T05(1:3,1:3)*I(1:3,13:15)*transpose(T05(1:3,1:3))*jw5;
ML=ML+m(6)*transpose(jv6)*jv6+transpose(jw6)*T06(1:3,1:3)*I(1:3,16:18)*transpose(T06(1:3,1:3))*jw6;


%% cheristopher coeffs:
t=[t1 t2 t3 t4 t5 t6];

syms c1 c2 c3 c4 c5 c6
c1(1:6,1:6)=zeros(6,6);
c2(1:6,1:6)=zeros(6,6);
c3(1:6,1:6)=zeros(6,6);
c4(1:6,1:6)=zeros(6,6);
c5(1:6,1:6)=zeros(6,6);
c6(1:6,1:6)=zeros(6,6);

k=1;
for i=1:6
    for j=1:6
        
        c1(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end

k=2;
for i=1:6
    for j=1:6
        
        c2(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end
k=3;
for i=1:6
    for j=1:6
        
        c3(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end
k=4;
for i=1:6
    for j=1:6
        
        c4(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end
k=5;
for i=1:6
    for j=1:6
        
        c5(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end
k=6;
for i=1:6
    for j=1:6
        
        c6(i,j)=0.5*(diff(ML(k,j),t(i))+diff(ML(k,i),t(j))+diff(ML(i,j),t(k)));
    end
end
%% Coriolis and Centrifugal Matrix
syms CL [ 6 6]
for j=1:6
CL(1,j)=thetadot(1)*c1(j,1)+thetadot(2)*c1(j,2)+thetadot(3)*c1(j,3)+thetadot(4)*c1(j,4)+thetadot(5)*c1(j,5)+thetadot(6)*c1(j,6);
end
    for j=1:6
CL(2,j)=thetadot(1)*c2(j,1)+thetadot(2)*c2(j,2)+thetadot(3)*c2(j,3)+thetadot(4)*c2(j,4)+thetadot(5)*c2(j,5)+thetadot(6)*c2(j,6);
    end
    for j=1:6
CL(3,j)=thetadot(1)*c3(j,1)+thetadot(2)*c3(j,2)+thetadot(3)*c3(j,3)+thetadot(4)*c3(j,4)+thetadot(5)*c3(j,5)+thetadot(6)*c3(j,6);
    end
    for j=1:6
CL(4,j)=thetadot(1)*c4(j,1)+thetadot(2)*c4(j,2)+thetadot(3)*c4(j,3)+thetadot(4)*c4(j,4)+thetadot(5)*c4(j,5)+thetadot(6)*c4(j,6);
    end
    for j=1:6
CL(5,j)=thetadot(1)*c5(j,1)+thetadot(2)*c5(j,2)+thetadot(3)*c5(j,3)+thetadot(4)*c5(j,4)+thetadot(5)*c5(j,5)+thetadot(6)*c5(j,6);
    end
    for j=1:6
CL(6,j)=thetadot(1)*c6(j,1)+thetadot(2)*c6(j,2)+thetadot(3)*c6(j,3)+thetadot(4)*c6(j,4)+thetadot(5)*c6(j,5)+thetadot(6)*c6(j,6);
    end

%% Gravity
syms P GL
P=m(1)*[0 0 g]*T0c1(1:3,4);
P=P+m(2)*[0 0 g]*T0c2(1:3,4)/1000;
P=P+m(3)*[0 0 g]*T0c3(1:3,4)/1000;
P=P+m(4)*[0 0 g]*T0c4(1:3,4)/1000;
P=P+m(5)*[0 0 g]*T0c5(1:3,4)/1000;
P=P+m(6)*[0 0 g]*T0c6(1:3,4)/1000;
for i=1:6
    GL(i,1)=diff(P,t(i));
end
