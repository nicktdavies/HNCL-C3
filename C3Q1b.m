
clear all; clc; close all;
 
%% PD controller
 
dt = 0.01;                           % Time step of simulation
Time = 0:dt:30;                      % Create time vector for simulation length
P = 20;                              % Define P and D gains for controller
D = 1;
mu = 10;                             % Viscous coefficient
DesiredTraj = sin(Time);             % Fabricate signal to follow
 
%State(:,1) = [1;0];                  % Create state which is a vector composed of [position; velocity]
friction(1) = 0;
   x(1)=1;
   v(1)=0;
   e(1)=0;
for i=1:size(Time,2)-1               % Simulate physical system with for loop
   
    x(i+1)=x(i)+v(i)*dt;
    e(i+1)=DesiredTraj(i+1)-x(i+1); 
    edot(i+1)=(e(i+1)-e(i))/dt;
    
    u(i) = (P*e(i))+(D*edot(i)-(friction(i)));
    v(i+1)=v(i)+u(i)*dt;             % Enter your code here
    friction(i+1) = mu * v(i+1);
    State(:,i) = [x(i);v(i)];
end

subplot (2,1,1),plot(Time, DesiredTraj, 'r');
hold on;
subplot(2,1,1),plot(Time, x, 'b');
xlabel 'Time (s)';
ylabel 'Trajectory Position (m)';
title 'Trajectory through PD controller with damping where mu=10, shown with the Desired Trajectory';
legend('Desired', 'PD Controller');

subplot(2,1,2),plot(v,friction);
xlabel 'Velocity';
ylabel 'Friction';
title 'Friction vs velocity';


