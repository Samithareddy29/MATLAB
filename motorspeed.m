clc;clear;
close all;
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
%transfer function
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2);
%step response with p controller
Kp = 100;
C = pid(Kp);
sys_cl = feedback(C*P_motor,1);
t = 0:0.01:5;
figure(1);
step(sys_cl,t)
grid
title('Step Response with Proportional Control')
%step response with pid controller
Kp = 75;
Ki = 1;
Kd = 1;
C = pid(Kp,Ki,Kd);
figure(2);
sys_cl = feedback(C*P_motor,1);
step(sys_cl,[0:1:200])

title('PID Control with Small Ki and Small Kd')
%tuning gains
 Kp = 100;
 Ki = 200;
 Kd = 1;
 C = pid(Kp,Ki,Kd);
 sys_cl = feedback(C*P_motor,1);
 figure(3);
 step(sys_cl, 0:0.01:4)
 grid
 title('PID Control with Large Ki and Small Kd')
 %increasing kd
 Kp = 100;
 Ki = 200;
 Kd = 10;
 C = pid(Kp,Ki,Kd);
 sys_cl = feedback(C*P_motor,1);
 figure(4);
 step(sys_cl, 0:0.01:4)
 grid
 title('PID Control with Large Ki and Large Kd')
