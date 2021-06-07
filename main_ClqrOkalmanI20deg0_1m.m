%% EGH445 Written Report Assignment
% Chiran Walisundara - N10454012
% close all; clear; clc
% format short

% get initial values
initialise;

% equilibrium points
equilibrium;

% get associated equilibrium points' linearized system dynamics
% infinite EP's while x3 = position = 0, therefore
% EPa - RA angle which has most influence on TO displacement
% EPb - RA angle which has least influence on TO displacement
EP = "a";

% get equilibrium point system dynamics
epsysdyn;

% stability check
stabilitycheck;

% controllability check
controlcheck;

% controller switch
controller_enabled = 1;

% lqr controller gains 'K'
% penalising states and control input
Qs1 = 1e3*0.25/1e3; % RA Angle
Qs2 = 1e2*0.25/1e3; % RA Angular Velocity
Qs3 = 1e6*0.25/1e3; % TO Position
Qs4 = 1e4*0.75/1e3; % TO Linear Velocity
Ru = 1e1/1e3; % Input Torque

% cross functions
Nin1 = 1e3/1e3*0;
Nin2 = 1e3/1e3*0;
Nin3 = 1e3/1e3*0;
Nin4 = 1e2/1e3*0;

% linear quadratic regulator gains 'K'
% outputs control gains Klqr
klqr;

% state feedback controller : u = -K * x(t)
K = Klqr;

% closed loop system with controller
consys = (A - B*K);
conpoles = eig(consys);

% observer switch
use_state_estimates = 1;

% RA Angular Velocity & TO Linear Velocity cannot be observed
% Assigning output matrix to be only RA Angle & TO Position
Cnew = [ 1 0 0 0;
         0 0 1 0 ];

Dnew = zeros(size(Cnew,1), size(B,2));

% observability check
observecheck;

% kalman filter
G = eye(4); % process disturbance
H = ones(2,1) * 0; % measurement noise
% kalman filter covariance matrices
% process disturbance covariance
Qcov = diag(ones(1,4));
Qcov(1,1) = Qcov(1,1) * 500;
Qcov(2,2) = Qcov(2,2) * 500;
Qcov(3,3) = Qcov(3,3) * 200;
Qcov(4,4) = Qcov(4,4) * 800;
% measurement noise covariance
Rcov = diag(0.01 * ones(1,2));
N = zeros(size(Qcov,1), size(Rcov,2)); % cross
BF = [B G 0*B]; % system input, process disturbance, measurement noise
midZeros = zeros(size(Dnew,1),size(G,2));
DF = [Dnew midZeros H]; % system feed-through, process disturbance, measurement noise
syskf = ss(A, BF, Cnew, DF);
[kest,Lkalman,PP] = kalman(syskf,Qcov,Rcov,N);
% Lkalman
L = Lkalman;

% closed loop system with observer
obssys = (A - L*Cnew);
obspoles = eig(obssys);

x_bar_obs = [x_bar(1); x_bar(3)];

wholeSys = [ (A - B*K)      B*K;
             zeros(4,4)  (A - L*Cnew) ];
wholeSysPoles = eig(wholeSys); % eigenvalues match independantly!

% simulink model and ode solver parameters
h = 0.01;
stoptime = 5;

% simulation and plotting
simandplotobserve;
