clc;clear;
close all;
Kp = 1;
Ki = 1;
Kd = 1;

s = tf('s');
C = Kp + Ki/s + Kd*s;
% p controller
m = 1000;
b = 50;
r = 10;
%transfer function
s = tf('s');
P_cruise = 1/(m*s + b);

Kp = 100;
C = pid(Kp);

T = feedback(C*P_cruise,1);

t = 0:0.1:20;
figure(1);
step(r*T,t)
axis([0 20 0 10])
%to reduce the steady state error and rise time
Kp = 5000;
C = pid(Kp);
T = feedback(C*P_cruise,1);
figure(2);
step(r*T,t)
axis([0 20 0 10])
%pi controller
Kp = 600;
Ki = 1;
C = pid(Kp,Ki);

T = feedback(C*P_cruise,1);
figure(3);
step(r*T,t)
axis([0 20 0 10])
%adjusting kp ki to get desired response
Kp = 800;
Ki = 40;
C = pid(Kp,Ki);

T = feedback(C*P_cruise,1);
figure(4);
step(r*T,t)
axis([0 20 0 10])
%PID controller-not needed derivative controller for implementing cruise
%control
Kp = 1;
Ki = 1;
Kd = 1;
C = pid(Kp,Ki,Kd);

T = feedback(C*P_cruise,1);