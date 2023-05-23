%% Task 2 Assignment 4 DynSys solved using LQR
A = [0 1; 0 0];
B = [0; 7];
C = [1 0; 0 1];
D = 0;

Rii = 1/(0.22^2)
X1i = 1/(560^2) * 10000
X2i = (1/sqrt(2 * 7 * 0.22 * 560)) / 10

mu = 1;
Q1 = [X1i 0; 0 X2i];
R1 = Rii;

[K1,P1,e1] = lqr(A,B,Q1,R1)

A_cl1 = A - B*K1;

x0 = [1; 0];

sys_cl1 = ss(A_cl1, B, C, D);
figure
initial (sys_cl1, x0);
pole (sys_cl1)
