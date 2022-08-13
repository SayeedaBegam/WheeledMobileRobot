%% kinematic simulation of a land-based mobile robot
close all; clear all; clc;

%% simulation parameters 
dt = 0.1; % step size
ts = 10; % simulation time 
t = 0:dt:ts; % time span 

%% Initial conditions 
x0 = 0;
y0 = 0;
psi0 = pi/4;

eta0 = [x0 ; y0 ; psi0];

eta(:,1) = eta0;

%% loop starts here
for i = 1:length(t)
    psi = eta(3,i); % current orientation in rad
   % jacobian matrix 
    J_psi = [cos(psi),-sin(psi),0;
             sin(psi), cos(psi),0;
             0,0,1];
    u = 0.1;
    v = 0.05;
    r = 0;
    zeta(:,i) = [u,v,r];


    eta_dot(:,i) = J_psi * zeta(:,i);


    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); % Euler method  
end

%% plotting function 

plot(t,eta(1,1:i),'-r');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('x,[m]');


plot(t,eta(2,1:i),'-b');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('y,[m]');



plot(t,eta(3,1:i),'-g');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('/psi,[rad]');


%% animation (mobile robot motion animation )
l = 0.6; % lenght of the mobile robot 
w = 0.4; %widht of the mobile robot
% Mobile robot coordinates 
mr_co =  [-1/2,1/2,-1/2,-1/2;
           -w/2,-w/2,w/2,-w/2];
figure 
for i = 1:length(t) %animation starts here
    psi =  eta(3:i);
    R_psi = [cos(psi),-sin(psi);
             sin(psi), cos(psi);]; %rotation matrix 
    v_pos = R_psi * mr_co;
    fill(v_pos(1,:) +eta(1,i),v_pos(2,:)+eta(2,i),'g')
    hold on, grid on 
    axis([-1 3 -1 3]), axis square
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('Mr','Path')
    set(gca,'fontsize',24)
    xlabel('x,[m]'); ylabel ('y,[m]');
    pause(0.1);
    hold off 
end % animation ends here









