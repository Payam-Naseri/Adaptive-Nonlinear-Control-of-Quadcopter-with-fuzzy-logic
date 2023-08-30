clc
clear
%% Planner

Tfinal = 10;

% slctr = randi([1 2],1,1); % Selects shape of reference signal
slctr = 1;
w = [1 1 0.5];
% X0 = (rand(1,3)*2 + 0.1).*(-1*ones(1,3)).^(randi([1 2],1,3)); % Reference position in slctr = 2 mode
X0 = [5 5 10];


%% Parameters of Drone
g = 9.8; 
mass = 0.65;  % Mass of drone (kg)

% Moment of inertia
Ix = 7.5e-3;  
Iy = 7.5e-3;
Iz = 1.3e-2;

Jr = 6e-5; % rotor inertia
l = 0.23; % arm length

b = 3.13e-5; % trust coefficient
d = 7.5e-7; % drag coefficient

initpos = [0 0 0]; % Drone's initial position
initvb = [0 0 0]; % Drone's initial velocity in body frame
initeuler = [0 0 0]; % Drone's initial euler angels
initangRates = [0 0 0]; % Drone's initial angular volocity in body frame
inertia_mat = [Ix 0 0
               0 Iy 0
               0 0 Iz];

%% NTSMC parameters

% position controller parameters (from GA optimization)
kp_z = -15; kp_y = -2.6224; kp_x = -2.9805;
kd_z = -20; kd_y = -2.0093; kd_x = -2.6701;

% Atitude controller parameters
kk2 = 15; kk3 = 15; beta = 2; % s_dot = -kk2 * s - kk3 * |s|^ro * sign(s)

k_phi1 = kk2;
k_phi2 = kk3;

k_theta1 = kk2;
k_theta2 = kk3;

k_psi1 = 8;
k_psi2 = 8;

a1 = (Iy-Iz)/Ix;
a2 = Jr/Ix;
a3 = (Iz-Ix)/Iy;
a4 = Jr/Iy;
a5 = (Ix-Iy)/Iz;

Omega2U = [b b b b
           0 -b 0 b
          -b 0 b 0
          -d d -d d];
U2Omega = inv(Omega2U);

gama = 1.5; % 1 < gama < 2
ro = 0.3; % 0 < ro < 1
beta_phi = beta; beta_theta = beta; beta_psi = beta;

%% NTSMC Adaptive Gain Scheduling with Fuzzy Inference System
% Define FIS
    fis_theta = sugfis();
    fis_theta=addInput(fis_theta,[-10 10],'Name','s_theta');
    fis_theta=addInput(fis_theta,[-3 3],'Name','s_theta_dot');
    fis_theta=addInput(fis_theta,[0 pi],'Name','theta_abs');
    fis_theta=addInput(fis_theta,[0 pi],'Name','phi_abs');
    fis_theta = addOutput(fis_theta,[0 80],'Name',"k_theta");
    
    fis_phi = sugfis();
    fis_phi=addInput(fis_phi,[-10 10],'Name','s_phi');
    fis_phi=addInput(fis_phi,[-3 3],'Name','s_phi_dot');
    fis_phi=addInput(fis_phi,[0 pi],'Name','phi_abs');
    fis_phi=addInput(fis_phi,[0 pi],'Name','theta_abs');
    fis_phi = addOutput(fis_phi,[0 50],'Name',"k_phi");

% Add Membership functions
    % FIS theta
    fis_theta = addMF(fis_theta,"s_theta","trapmf",[-20 -20 -0.2 -0.2]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[-0.2 -0.1 0]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[-0.1 0 0.1]);
    fis_theta = addMF(fis_theta,"s_theta","trimf",[0 0.1 0.2]);
    fis_theta = addMF(fis_theta,"s_theta","trapmf",[0.1 0.2 20 20]);

    fis_theta = addMF(fis_theta,"s_theta_dot","trapmf",[-5 -5 -0.1 0]);
    fis_theta = addMF(fis_theta,"s_theta_dot","trimf",[-0.1 0 0.1]);
    fis_theta = addMF(fis_theta,"s_theta_dot","trapmf",[0 0.1 5 5]);

    fis_theta = addMF(fis_theta,"theta_abs","trapmf",[0 0 0.05 0.2]);
    fis_theta = addMF(fis_theta,"theta_abs","trimf",[0.05 0.2 0.5]);
    fis_theta = addMF(fis_theta,"theta_abs","trapmf",[0.2 0.5 pi pi]);

    fis_theta = addMF(fis_theta,"phi_abs","trapmf",[0 0 0.05 0.2]);
    fis_theta = addMF(fis_theta,"phi_abs","trimf",[0.05 0.2 0.5]);
    fis_theta = addMF(fis_theta,"phi_abs","trapmf",[0.2 0.5 pi pi]);

    % FIS phi
    fis_phi = addMF(fis_phi,"s_phi","trapmf",[-20 -20 -0.2 -0.1]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[-0.2 -0.1 0]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[-0.1 0 0.1]);
    fis_phi = addMF(fis_phi,"s_phi","trimf",[0 0.1 0.2]);
    fis_phi = addMF(fis_phi,"s_phi","trapmf",[0.1 0.2 20 20]);

    fis_phi = addMF(fis_phi,"s_phi_dot","trapmf",[-5 -5 -0.1 0]);
    fis_phi = addMF(fis_phi,"s_phi_dot","trimf",[-0.1 0 0.1]);
    fis_phi = addMF(fis_phi,"s_phi_dot","trapmf",[0 0.1 5 5]);

    fis_phi = addMF(fis_phi,"phi_abs","trapmf",[0 0 0.05 0.2]);
    fis_phi = addMF(fis_phi,"phi_abs","trimf",[0.05 0.2 0.5]);
    fis_phi = addMF(fis_phi,"phi_abs","trapmf",[0.2 0.5 pi pi]);

    fis_phi = addMF(fis_phi,"theta_abs","trapmf",[0 0 0.05 0.2]);
    fis_phi = addMF(fis_phi,"theta_abs","trimf",[0.05 0.2 0.5]);
    fis_phi = addMF(fis_phi,"theta_abs","trapmf",[0.2 0.5 pi pi]);

% MF's for output(u)
    % FIS theta
    fis_theta = addMF(fis_theta,"k_theta","constant",[5]);  % Rule #1
    fis_theta = addMF(fis_theta,"k_theta","constant",[10]); % Rule #2
    fis_theta = addMF(fis_theta,"k_theta","constant",[15]); % Rule #3
    fis_theta = addMF(fis_theta,"k_theta","constant",[20]); % Rule #4
    fis_theta = addMF(fis_theta,"k_theta","constant",[26]); % Rule #5
    fis_theta = addMF(fis_theta,"k_theta","constant",[28]); % Rule #6
    fis_theta = addMF(fis_theta,"k_theta","constant",[30]); % Rule #7
    fis_theta = addMF(fis_theta,"k_theta","constant",[35]); % Rule #8
    fis_theta = addMF(fis_theta,"k_theta","constant",[40]); % Rule #9
    fis_theta = addMF(fis_theta,"k_theta","constant",[50]); % Rule #10
    fis_theta = addMF(fis_theta,"k_theta","constant",[55]); % Rule #11
    fis_theta = addMF(fis_theta,"k_theta","constant",[60]); % Rule #12

    % FIS phi
    fis_phi = addMF(fis_phi,"k_phi","constant",[5]);  % Rule #1
    fis_phi = addMF(fis_phi,"k_phi","constant",[10]); % Rule #2
    fis_phi = addMF(fis_phi,"k_phi","constant",[15]); % Rule #3
    fis_phi = addMF(fis_phi,"k_phi","constant",[20]); % Rule #4
    fis_phi = addMF(fis_phi,"k_phi","constant",[26]); % Rule #5
    fis_phi = addMF(fis_phi,"k_phi","constant",[28]); % Rule #6
    fis_phi = addMF(fis_phi,"k_phi","constant",[30]); % Rule #7
    fis_phi = addMF(fis_phi,"k_phi","constant",[35]); % Rule #8
    fis_phi = addMF(fis_phi,"k_phi","constant",[40]); % Rule #9
    fis_phi = addMF(fis_phi,"k_phi","constant",[50]); % Rule #10
    fis_phi = addMF(fis_phi,"k_phi","constant",[55]); % Rule #11
    fis_phi = addMF(fis_phi,"k_phi","constant",[60]); % Rule #12

% Rules of FIS
    rules = [1 1 1 1 10 1 1
             1 1 2 1 10 1 1
             1 1 2 2 11 1 1
             1 1 1 2 11 1 1
             1 1 3 1 12 1 1
             1 1 3 2 12 1 1
             1 1 3 3 12 1 1
             1 1 1 3 12 1 1
             1 1 2 3 12 1 1
             1 2 1 1 8 1 1
             1 2 2 1 8 1 1
             1 2 2 2 9 1 1
             1 2 1 2 9 1 1
             1 2 3 1 10 1 1
             1 2 3 2 10 1 1
             1 2 3 3 10 1 1
             1 2 1 3 10 1 1
             1 2 2 3 10 1 1
             1 3 1 1 9 1 1
             1 3 2 1 9 1 1
             1 3 2 2 9 1 1
             1 3 1 2 9 1 1
             1 3 3 1 10 1 1
             1 3 3 2 10 1 1
             1 3 3 3 10 1 1
             1 3 1 3 10 1 1
             1 3 2 3 10 1 1
             2 1 1 1 8 1 1
             2 1 2 1 9 1 1
             2 1 2 2 9 1 1
             2 1 1 2 9 1 1
             2 1 3 1 10 1 1
             2 1 3 2 10 1 1
             2 1 3 3 10 1 1
             2 1 1 3 10 1 1
             2 1 2 3 10 1 1
             2 2 1 1 5 1 1
             2 2 2 1 6 1 1
             2 2 2 2 6 1 1
             2 2 1 2 6 1 1
             2 2 3 1 7 1 1
             2 2 3 2 7 1 1
             2 2 3 3 7 1 1
             2 2 1 3 7 1 1
             2 2 2 3 7 1 1
             2 3 1 1 4 1 1
             2 3 2 1 5 1 1
             2 3 2 2 5 1 1
             2 3 1 2 5 1 1
             2 3 3 1 6 1 1
             2 3 3 2 6 1 1
             2 3 3 3 6 1 1
             2 3 1 3 6 1 1
             2 3 2 3 6 1 1
             3 1 1 1 5 1 1
             3 1 2 1 6 1 1
             3 1 2 2 6 1 1
             3 1 1 2 6 1 1
             3 1 3 1 7 1 1
             3 1 3 2 7 1 1
             3 1 3 3 7 1 1
             3 1 1 3 7 1 1
             3 1 2 3 7 1 1
             3 2 1 1 1 1 1
             3 2 2 1 2 1 1
             3 2 2 2 2 1 1
             3 2 1 2 2 1 1
             3 2 3 1 3 1 1
             3 2 3 2 3 1 1
             3 2 3 3 3 1 1
             3 2 1 3 3 1 1
             3 2 2 3 3 1 1
             3 3 1 1 5 1 1
             3 3 2 1 6 1 1
             3 3 2 2 6 1 1
             3 3 1 2 6 1 1
             3 3 3 1 7 1 1
             3 3 3 2 7 1 1
             3 3 3 3 7 1 1
             3 3 1 3 7 1 1
             3 3 2 3 7 1 1
             4 1 1 1 8 1 1
             4 3 2 1 9 1 1
             4 3 2 2 9 1 1
             4 3 1 2 9 1 1
             4 3 3 1 10 1 1
             4 3 3 2 10 1 1
             4 3 3 3 10 1 1
             4 3 1 3 10 1 1
             4 3 2 3 10 1 1
             4 2 1 1 5 1 1
             4 2 2 1 6 1 1
             4 2 2 2 6 1 1
             4 2 1 2 6 1 1
             4 2 3 1 7 1 1
             4 2 3 2 7 1 1
             4 2 3 3 7 1 1
             4 2 1 3 7 1 1
             4 2 2 3 7 1 1
             4 1 1 1 4 1 1
             4 1 2 1 5 1 1
             4 1 2 2 5 1 1
             4 1 1 2 5 1 1
             4 1 3 1 6 1 1
             4 1 3 2 6 1 1
             4 1 3 3 6 1 1
             4 1 1 3 6 1 1
             4 1 2 3 6 1 1
             5 3 1 1 10 1 1
             5 3 2 1 10 1 1
             5 3 2 2 11 1 1
             5 3 1 2 11 1 1
             5 3 3 1 12 1 1
             5 3 3 2 12 1 1
             5 3 3 3 12 1 1
             5 3 1 3 12 1 1
             5 3 2 3 12 1 1
             5 2 1 1 10 1 1
             5 2 2 1 10 1 1
             5 2 2 2 10 1 1
             5 2 1 2 10 1 1
             5 2 3 1 10 1 1
             5 2 3 2 10 1 1
             5 2 3 3 10 1 1
             5 2 1 3 10 1 1
             5 2 2 3 10 1 1
             5 1 1 1 8 1 1
             5 1 2 1 8 1 1
             5 1 2 2 8 1 1
             5 1 1 2 8 1 1
             5 1 3 1 8 1 1
             5 1 3 2 8 1 1
             5 1 3 3 8 1 1
             5 1 1 3 8 1 1
             5 1 2 3 8 1 1
             ];

fis_theta = addRule(fis_theta,rules);
fis_phi = addRule(fis_phi,rules);

%% Results
% Altitude 
x_ref=out.ref.Data(:,1);
y_ref=out.ref.Data(:,2);
z_ref=out.ref.Data(:,3);

x=out.pos.Data(:,1);
y=out.pos.Data(:,2);
z=out.pos.Data(:,3);

time = out.tout;

figure; subplot(3,1,1)
plot(time,x_ref,'--');hold on
plot(time,x); title('X Psition of Drone'); legend('X position','reference')

subplot(3,1,2)
plot(time,y_ref,'--');hold on
plot(time,y); title('Y Psition of Drone');legend('Y position','reference')

subplot(3,1,3)
plot(time,z_ref,'--');hold on
plot(time,z); title('Z Psition of Drone');legend('Z position','reference')

%% plot 3d position

time = out.tout;

x_ref=out.ref.Data(:,1);
y_ref=out.ref.Data(:,2);
z_ref=out.ref.Data(:,3);

x=out.pos.Data(:,1);
y=out.pos.Data(:,2);
z=out.pos.Data(:,3);
plot3(x_ref,y_ref,z_ref);hold on
plot3(x,y,z);title('Reference Tracking')
legend('Reference Position','Quadcopter trajectory')
% for i = 1:length(x)
%     plot3(x(i),y(i),z(i),'*')
%     drawnow
% end

%% Results with presence of disturbance
time = out.tout;
phi = out.phi.Data(:,1); phi_ref = out.phi.Data(:,2);
theta = out.theta.Data(:,1); theta_ref = out.theta.Data(:,2);
psi = out.psi.Data;

d_phi = out.disturb.Data(:,1);
d_theta = out.disturb.Data(:,2);

figure; subplot(3,1,1)
plot(time,phi);hold on
plot(time,phi_ref); hold off; title('roll angel')
legend('roll','reference roll')

subplot(3,1,2)
plot(time,theta);hold on
plot(time,theta_ref); hold off; title('pitch angel')
legend('pitch','reference pitch')

subplot(3,1,3)
plot(time,psi); title('yaw angel')


U2 = out.U2U3.Data(:,1);
U3 = out.U2U3.Data(:,2);

%% plot for Disturbance
k_phi = out.k_adptv_phi.Data;
k_theta = out.k_adptv_theta.Data;

figure; subplot(3,1,1)
plot(time,phi);hold on
plot(time,phi_ref); hold off; title('roll angel')
legend('roll','reference roll')

subplot(3,1,2)
plot(time,d_phi); title('Disturbance in roll direction')

subplot(3,1,3)
plot(time,k_phi); title('NTSMC Adaptive Gain')


figure; subplot(3,1,1)
plot(time,theta);hold on
plot(time,theta_ref); hold off; title('pitch angel')
legend('roll','reference roll')

subplot(3,1,2)
plot(time,d_theta); title('Disturbance in pitch direction')

subplot(3,1,3)
plot(time,k_theta); title('NTSMC Adaptive Gain')
