%State Space System

%Parameters
g = 9.80
m = 1.568
I_xx = 0.0090
I_yy = 0.0090
I_zz = 0.017

%Control Gains
kpx = 1;
kdx = -1.7;
kix = 0;

kpy = 1;
kdy = 1.7;
kiy = 0;

kpz = 4.5;
kdz = 1.7;
kiz = 2;

kpr = 2.5;
kdr = 0.345;
kir = 0;

kpp = 2.5;
kdp = 0.345;
kip = 0;

kpyaw = 1;
kdyaw = 0.06;
kiyaw = 0;

%State Space Model
A = [0 0 0 0 0 0 1 0 0 0 0 0 ; 
     0 0 0 0 0 0 0 1 0 0 0 0 ;
     0 0 0 0 0 0 0 0 1 0 0 0 ;
     0 0 0 0 0 0 0 0 0 1 0 0 ;
     0 0 0 0 0 0 0 0 0 0 1 0 ;
     0 0 0 0 0 0 0 0 0 0 0 1 ;
     0 0 0 0 g 0 0 0 0 0 0 0 ;
     0 0 0 -g 0 0 0 0 0 0 0 0 ;
     0 0 0 0 0 0 0 0 0 0 0 0 ;
     0 0 0 0 0 0 0 0 0 0 0 0 ;
     0 0 0 0 0 0 0 0 0 0 0 0 ;
     0 0 0 0 0 0 0 0 0 0 0 0 ;]

B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 1/I_xx 0 0;
     0 0 1/I_yy 0;
     0 0 0 1/I_zz]

C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0]

D = [0 0 0 0 ; 
     0 0 0 0 ;
     0 0 0 0 ;
     0 0 0 0 ;
     0 0 0 0 ;
     0 0 0 0 ]



%Transfer Functions
sys = ss(A,B,C,D)
sys_tf = tf(sys)
T1 = sys_tf(1, 3)
T2 = sys_tf(2, 2)
T3 = sys_tf(3, 1)
T4 = sys_tf(4, 2)
T5 = sys_tf(5, 3)
T6 = sys_tf(6, 4)

C1 = tf([kdx, kpx, kix], [0 1 0])
C2 = tf([kdy, kpy, kiy], [0 1 0])
C3 = tf([kdz, kpz, kiz], [0 1 0])
C4 = tf([kdr, kpr, kir], [0 1 0])
C5 = tf([kdp, kpp, kip], [0 1 0])
C6 = tf([kdyaw, kpyaw, kiyaw], [0 1 0])


%Stability Analysis


olsys1 = C1*T1
olsys2 = C2*T2
olsys3 = C3*T3
olsys4 = C4*T4
olsys5 = C5*T5
olsys6 = C6*T6

clsys1 = C1*T1/(1+C1*T1)
clsys2 = C2*T2/(1+C2*T2)
clsys3 = C3*T3/(1+C3*T3)
clsys4 = C4*T4/(1+C4*T4)
clsys5 = C5*T5/(1+C5*T5)
clsys6 = C6*T6/(1+C6*T6)

%{
figure(1)
rlocus(clsys1)
figure(2)
rlocus(clsys2)
figure(3)
rlocus(clsys3)
figure(4)
rlocus(clsys4)
figure(5)
rlocus(clsys5)
figure(6)
rlocus(clsys6)
%}

figure(1);
margin(olsys1)
grid



figure(2);
margin(olsys2)
grid

figure(3);
margin(olsys3)
grid

figure(4);
margin(olsys4)
grid


figure(5);
margin(olsys5)
grid


figure(6);
margin(olsys6)
grid
