

%% FOREWARD KINEMATICS
Offset=170;
L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;

syms   t1 t2 t3 t4 t5 t6 real
% t1=pi/4;
% t2=pi/3;
% t3=pi/4;
% t4=pi/6;
% t5=pi/5;
% t6=pi/4;


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
r=P06;


%% Jacobian (Alternative Approach Method) Method1


R60=transpose(T06(1:3,1:3));
Re0=R60;

J0v1_1 = cross(T01(1:3,3) , T06(1:3,4)-T01(1:3,4));
J0v2_1 = cross(T02(1:3,3) , T06(1:3,4)-T02(1:3,4));
J0v3_1 = cross(T03(1:3,3) , T06(1:3,4)-T03(1:3,4));
J0v4_1 = cross(T04(1:3,3) , T06(1:3,4)-T04(1:3,4));
J0v5_1 = cross(T05(1:3,3) , T06(1:3,4)-T05(1:3,4));
J0v6_1 = cross(T06(1:3,3) , T06(1:3,4)-T06(1:3,4));
J0v_1 = [J0v1_1 J0v2_1 J0v3_1 J0v4_1 J0v5_1 J0v6_1];
Jev_1 = R60*J0v_1;

J0w1_1 = T01(1:3,3);
J0w2_1 = T02(1:3,3);
J0w3_1 = T03(1:3,3);
J0w4_1 = T04(1:3,3);
J0w5_1 = T05(1:3,3);
J0w6_1 = T06(1:3,3);
J0w_1 = [J0w1_1 J0w2_1 J0w3_1 J0w4_1 J0w5_1 J0w6_1];
Jew_1 = R60*J0w_1;

J0e_1 = [J0v_1 ; J0w_1]
Jee_1 = [Jev_1 ; Jew_1]

% J0e_1 = simplify([J0v_1 ; J0w_1]) 
% Jee_1 = simplify([Jev_1 ; Jew_1])




%% Jacobian (Velocity Propagation Method) Method2
R60=transpose(T06(1:3,1:3));
Re0=R60;

R10=transpose(T01(1:3,1:3));
R21=transpose(T12(1:3,1:3));
R32=transpose(T23(1:3,1:3));
R43=transpose(T34(1:3,1:3));
R54=transpose(T45(1:3,1:3));
R65=transpose(T56(1:3,1:3));
Re5=R65;
R0e=R06;

P12=T12(1:3,end);
P23=T23(1:3,end);
P34=T34(1:3,end);
P45=T45(1:3,end);
P56=T56(1:3,end);
P5e=P56;

syms tdot1 tdot2 tdot3 tdot4 tdot5 tdot6 real

w00_2=[0;0;0];
w11_2=R10*w00_2+tdot1*[0;0;1];
w22_2=R21*w11_2+tdot2*[0;0;1];
w33_2=R32*w22_2+tdot3*[0;0;1];
w44_2=R43*w33_2+tdot4*[0;0;1];
w55_2=R54*w44_2+tdot5*[0;0;1];
w66_2=R65*w55_2+tdot6*[0;0;1];
wee_2=w66_2;
w0e_2=R0e*wee_2;

v00_2=[0;0;0];
v11_2=R10*(v00_2+cross(w00_2 ,P01));
v22_2=R21*(v11_2+cross(w11_2 ,P12));
v33_2=R32*(v22_2+cross(w22_2 ,P23));
v44_2=R43*(v33_2+cross(w33_2 ,P34));
v55_2=R54*(v44_2+cross(w44_2 ,P45));
v66_2=R65*(v55_2+cross(w55_2 ,P56));
vee_2=v66_2;
v0e_2=R0e*vee_2;

V0e_2=[v0e_2 ; w0e_2];

J0e_2_11= simplify(subs(coeffs(V0e_2(1,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_12= simplify(subs(coeffs(V0e_2(1,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_13= simplify(subs(coeffs(V0e_2(1,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_14= simplify(subs(coeffs(V0e_2(1,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_15= simplify(subs(coeffs(V0e_2(1,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_16= simplify(subs(coeffs(V0e_2(1,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));
J0e_2_21= simplify(subs(coeffs(V0e_2(2,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_22= simplify(subs(coeffs(V0e_2(2,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_23= simplify(subs(coeffs(V0e_2(2,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_24= simplify(subs(coeffs(V0e_2(2,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_25= simplify(subs(coeffs(V0e_2(2,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_26= simplify(subs(coeffs(V0e_2(2,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));
J0e_2_31= simplify(subs(coeffs(V0e_2(3,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_32= simplify(subs(coeffs(V0e_2(3,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_33= simplify(subs(coeffs(V0e_2(3,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_34= simplify(subs(coeffs(V0e_2(3,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_35= simplify(subs(coeffs(V0e_2(3,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_36= simplify(subs(coeffs(V0e_2(3,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));
J0e_2_41= simplify(subs(coeffs(V0e_2(4,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_42= simplify(subs(coeffs(V0e_2(4,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_43= simplify(subs(coeffs(V0e_2(4,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_44= simplify(subs(coeffs(V0e_2(4,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_45= simplify(subs(coeffs(V0e_2(4,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_46= simplify(subs(coeffs(V0e_2(4,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));
J0e_2_51= simplify(subs(coeffs(V0e_2(5,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_52= simplify(subs(coeffs(V0e_2(5,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_53= simplify(subs(coeffs(V0e_2(5,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_54= simplify(subs(coeffs(V0e_2(5,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_55= simplify(subs(coeffs(V0e_2(5,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_56= simplify(subs(coeffs(V0e_2(5,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));
J0e_2_61= simplify(subs(coeffs(V0e_2(6,1) ,tdot1),{tdot2 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_62= simplify(subs(coeffs(V0e_2(6,1) ,tdot2),{tdot1 ,tdot3, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_63= simplify(subs(coeffs(V0e_2(6,1) ,tdot3),{tdot1 ,tdot2, tdot4 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_64= simplify(subs(coeffs(V0e_2(6,1) ,tdot4),{tdot1 ,tdot2, tdot3 ,tdot5 ,tdot6},{0,0,0,0,0}));
J0e_2_65= simplify(subs(coeffs(V0e_2(6,1) ,tdot5),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot6},{0,0,0,0,0}));
J0e_2_66= simplify(subs(coeffs(V0e_2(6,1) ,tdot6),{tdot1 ,tdot2, tdot3 ,tdot4 ,tdot5},{0,0,0,0,0}));

J0e_2 = [J0e_2_11(end) J0e_2_12(end) J0e_2_13(end) J0e_2_14(end) J0e_2_15(end) J0e_2_16(end)
         J0e_2_21(end) J0e_2_22(end) J0e_2_23(end) J0e_2_24(end) J0e_2_25(end) J0e_2_26(end)
         J0e_2_31(end) J0e_2_32(end) J0e_2_33(end) J0e_2_34(end) J0e_2_35(end) J0e_2_36(end)
         J0e_2_41(end) J0e_2_42(end) J0e_2_43(end) J0e_2_44(end) J0e_2_45(end) J0e_2_46(end)
         J0e_2_51(end) J0e_2_52(end) J0e_2_53(end) J0e_2_54(end) J0e_2_55(end) J0e_2_56(end)
         J0e_2_61(end) J0e_2_62(end) J0e_2_63(end) J0e_2_64(end) J0e_2_65(end) J0e_2_66(end)]
     
Jev_2 = (Re0*J0e_2(1:3,1:6));
Jew_2 = (Re0*J0e_2(4:6,1:6));
Jee_2 = simplify([Jev_2 ; Jew_2])

% double(J0e_2)
% double(Jee_2)

%% Singularity
%Robot is Decoupled Manipulator
%J0e = [J11 0 ;J21 J22];
%det(J0e) = det(J11)*det(J22)
% J11 = J0e_1(1:3,1:3);
% J22 = J0e_1(4:6,4:6);
%  simplify(det(J11))
%  simplify(det(J22))

          