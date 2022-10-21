L0=505;
L1=73;
L2=37;
L3=700;
L4=110;
L5=110;
L6=650;
L7=67;

t1=pi/3;
t2=0;
t3=pi/2;
t4=pi/3;
t5=pi/3;
t6=pi/3;

dh=[ 
    0 L0 0 pi/2
    0 0 L3 0
    pi/2 0 0 pi/2
    0 L5+L6 0 -pi/2
    0 0 0 pi/2
    0 0 0 0];
r=SerialLink(dh);
r.name='Kawasaki RS013N';
j0=r.jacob0([t1 t2+pi/2 t3 t4 t5 t6]);
je=r.jacobe([t1 t2+pi/2 t3 t4 t5 t6]);
