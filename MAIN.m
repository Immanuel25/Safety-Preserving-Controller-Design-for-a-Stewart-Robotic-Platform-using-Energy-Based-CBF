clear all;
close all;
clear; clc;

%%  PARAMETERS
%   Steward Platform
q0 = [0;0;0.32;deg2rad(0);deg2rad(0);deg2rad(0)]; % x,y,z,phi,theta,psi
dq0 = [0;0;0;0;0;0];
Q0 = [q0; dq0];

q_des = [0;0;0.6;deg2rad(0);deg2rad(0);deg2rad(0)];
dq_des = [0;0;0;0;0;0];
Gains.Q_des = [q_des; dq_des];

%   LQR
Gains.Q = 100*[eye(6) zeros(6); zeros(6) eye(6)]; % Q matrix from Riccati Equation
Gains.R = 1*eye(6); % R matrix from Riccati Equation 
K_LQR();

%   CBF
Gains.alpha = 1;
Gains.alpha_e = 1;
Gains.q_max = [0.025;0.025;0.5;deg2rad(10);deg2rad(10);deg2rad(10)];
Gains.dq_max = [0.005;0.005;0.1;deg2rad(2);deg2rad(2);deg2rad(2)];

%   Simulation
global Sims
Sims.Plot = 'true';
Sims.CBF = 'true';  % 'true', 'false'
Sims.time = [];
Sims.F = [];
Sims.Q = [];
Sims.P = [];
Sims.h = [];
Sims.hD = [];

%%  Simulation

Sims.tspan = [0 20];
options = odeset('OutputFcn', @Logger);  % Set the logging function
% options = odeset('OutputFcn', @Logger, 'MaxStep', 0.01);  % Set the logging function
[~,~] = ode15s(@(t,Q) StewartPlatform(t,Q,Gains,Sims), Sims.tspan, Q0, options);
SimsLQR = jsondecode(jsonencode(Sims));

% Sims.time = [];
% Sims.F_lqr = [];
% Sims.Q = [];
% Sims.Plot = 'true';
% Sims.CBF = 'true';  % 'true', 'false'
% [~,~] = ode15s(@(t,Q) StewartPlatform(t,Q,Gains,Sims), Sims.tspan, x0, options);
% SimsCBF = Sims;

save(['Datas/data.mat']);
% ComparePlot();
% Plot();
Animation();