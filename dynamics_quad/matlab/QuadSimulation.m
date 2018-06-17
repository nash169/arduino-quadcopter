%% Main simualtion
clear; close all; clc;
% Simulation times, in seconds.
T = 20;
dt = .01;
t = 0 : dt : T;

% Number fo points in the simulation.
N = numel(t);

% Model data
dat = GetData('drone');
[A, B, C, D] = GetLinearSys(dat);

% Initial simulation state q = [x, theta, v, omega] (12x1).
q = zeros(12,1);
qdot = q;

% Define initial positions
q(1:3) = [0, 0, 5]'; % x
q(4:6) = pi/8*rand(3,1); % theta

% Define initial velocities
q(7:9) = [0, 0, 0]'; % v
qdot(1:3) = q(7:9); % xdot
deviation = 10; % [rad/s] Disturbances in angular velocity
qdot(4:6) = deg2rad(2 * deviation * rand(3,1) - deviation); % thetadot
q(10:12) = ThetaDot2Omega(q)\qdot(4:6); % omega

% Optimal Control Gain Matrix
rho = 0.1;
Q = 0.01*eye(12); Q(4:6,4:6) = eye(3); Q(10:12,10:12) = eye(3); Q(3,3) = 0.1; Q(9,9) = 0.1;
R = eye(4);

% Kalman Filter Gain Matrix
theta = 1;
QN = eye(4);
RN = eye(6);

[K, L, sys] = OptimalGain(rho,Q,R,theta,QN,RN,A,B,C,D);

% Initialize Controller
z_ref = 1;
u  = AttitudeController(q,K) + AltitudeController(q, z_ref, K);

% Plot frequency
framerate = 10;

% Initialize story log
Qlog = zeros(length(t), length(q));
Qlog(1,:) = q';
Ulog = zeros(length(t), length(u));
Ulog(1,:) = u';

% Simualtion Loop
for i = 1:length(t)
    if mod(i,framerate) == 0
       PlotDrone(Qlog, i, dat); 
    end
    
    % Unwrap q and qdot vectors for cleares code
    x = q(1:3); theta = q(4:6); v = q(7:9); omega = q(10:12);
    xdot = qdot(1:3); thetadot = qdot(4:6); vdot = qdot(7:9); omegadot = qdot(10:12);
    
    % Compute rotation matrices
    R = RotMatrix(q);
    W = Omega2ThetaDot(q);
    
    % Get control forces
    u  = AttitudeController(q,K) + AltitudeController(q, z_ref, K);
    
    % Thrust, Drag and Gravity
    T = [0; 0; dat.k * sum(u)];
    T = R * T;
    Fd = -dat.r * xdot;
    g = [0; 0; -dat.g];
    
    % Torques
    tau = Torques(u, dat);
    
    % Calculate xdot and thetadot
    qdot(1:3) = q(7:9); % xdot = v
    qdot(4:6) = W*q(10:12); % thetadot = W*omega
    
    % Calculate accelerations
    qdot(7:9) = g + 1 / dat.m * T + Fd; % vdot = F/m
    qdot(10:12) = dat.I \ (tau - cross(q(10:12),...
        dat.I * q(10:12))); % omegadot = J\(tau - omega x J*omega)
    
    % Calculate next step state and memoriza data
    q = q + dt*qdot;
    Qlog(i+1,:) = q';
    Ulog(i+1,:) = u';
end
save('data/StateLog', 'Qlog');
save('data/ControlLog', 'Ulog');