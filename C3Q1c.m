
clear all; clc; close all;
 
%% PD controller
 
dt = 0.01;                           % Time step of simulation
Time = 0:dt:30;                      % Create time vector for simulation length
P = 20;                              % Define P and D gains for controller
D = 1;
mu = 10;                             % Viscous coefficient
DesiredTraj = sin(Time);             % Fabricate signal to follow
 
%State(:,1) = [1;0];                 % Create state which is a vector composed of [position; velocity]
   friction(1) = 0;                  % Initiailise nes values for the loop
   x(1)=0;
   v(1)=0;
   e(1)=0;
   upd(1)=0;
   uff(1)=0;
   gamma = 0.005;
   p(1) = 0;
   frictiono(1) = 0;
   xo(1)=0;
   vo(1)=0;
   eo(1)=0;
for i=1:size(Time,2)-1               % Simulate physical system with for loop
   
    x(i+1)=x(i)+v(i)*dt;             % Calc Compensated Values used for the Compensated PD Controller
    e(i+1)=DesiredTraj(i+1)-x(i+1); 
    edot(i+1)=(e(i+1)-e(i))/dt;
   
    p(i+1) = p(i) + (gamma * v(i) * upd(i));
    U(i) = upd(i) + uff(i);
    v(i+1) = v(i)+U(i)*dt;          
    friction(i+1) = p(i+1) * v(i+1);
    State(:,i) = [x(i);v(i)];
    uff(i+1) = v(i+1) * p(i+1);
    upd(i+1) = (P*e(i+1))+(D*edot(i+1)-(friction(i+1)));
    
    xo(i+1)=xo(i)+vo(i)*dt;          % Calc Uncompensated Values used for the Uncompensated PD Controller
    eo(i+1)=DesiredTraj(i+1)-xo(i+1); 
    edoto(i+1)=(eo(i+1)-eo(i))/dt;
    uo(i) = (P*eo(i))+(D*edoto(i)-(frictiono(i)));
    vo(i+1)=vo(i)+uo(i)*dt;          
    frictiono(i+1) = mu * vo(i+1);
    Stateo(:,i) = [xo(i);vo(i)];
end

subplot (3,1,1),plot(Time, x, 'r')  % Plot Comp and Uncomp Traj against time
hold on;
subplot(3,1,1),plot(Time, xo, 'b')
subplot (3,1,1), plot(Time, DesiredTraj, 'k')
xlabel 'Time (s)';
ylabel 'Trajectory Position (m)';
title 'Plot of Trajectory Position of a Uncompensated PD Controller with Friction and Compensated PD Controller with Friction Over Time';
legend('Comp PD', 'Uncomp PD', 'Desired Tajectory');

subplot(3,1,2),plot(Time, p, 'r');  % Plot Calc and Actual values of coeff friction against time
hold on
subplot(3,1,2),plot(Time, mu, 'b');
xlabel 'Time (s)';
ylabel 'Coefficient of Friction';
title 'Plot of p and mu changing over time';
legend('p', 'mu');
axis([0 30 -2 11])

subplot(3,1,3),plot(Time, upd, 'r'); % Plot Upd and uff against time
hold on
subplot(3,1,3),plot(Time, uff, 'b');
xlabel 'Time (s)';
ylabel 'Spring Like Force (N)';
title 'Plot of PD Controller spring like force and feedforward Spring like force over time';
legend('uPD', 'uFF');




