%% EGH445 Written Report Assignment
close all; clear; clc
format bank;

% get initial values
initialise;

% equilibrium points
equilibrium;

% get associated equilibrium points' linearized system dynamics
% infinite EP's while x3 = position = 0, therefore
% EPa - RA angle which has most influence on TORA displacement
% EPb - RA angle which has least influence on TORA displacement
EP = "a";
epsysdyn;

% stability check
stabilitycheck;

% controllability check
controlcheck;

% controller switch
controller_enabled = 1;

% for equilibrium point "a" (controllable EP)
% from the 4 poles, designing for 2 poles
% so that is it a 2nd order system
% making 2 poles to be more than 5x far away
% dominant pole theory!
% designing for overshoot 3% and settling time 1s
% overshoot(%)
PO = 3;
% settling time
Ts = 3;
% non-dominant (fast poles) distances from dominant (slow poles)
% if ndpl1 ~= ndpl2 -> 'place' will be used
ndpl1 = 10;
ndpl2 = 10;

% normal controller gains 'K'
% outputs control gains Knormal
knormal;

% lqr controller gains 'K'
% penalising states and control input
Qs1 = 1e3; % RA Angle
Qs2 = 1e2; % RA Angular Velocity
Qs3 = 1e6; % TO Position
Qs4 = 1e4; % TO Linear Velocity
Ru = 1; % Input Torque

% linear quadratic controller gains 'K'
% outputs control gains Klqr
klqr;

% state feedback controller : u = -K * x(t)
% either Knormal or Klqr
K = Klqr;

% closed loop system with controller
newsys = (A - B*K);
CLpoles = eig(newsys);

% Cart_Pendulum_Animation(NL.t,NL.x1,NL.x2,x_bar(1),x_bar(2))

% RA Angular Velocity & TO Linear Velocity cannot be observed
% Assigning output matrix to be only RA Angle & TO Position
C = [ 1 0 0 0;
      0 0 1 0 ];

% observability check
observecheck;

% observer switch
use_state_estimates = 0;
  
% full-order luenberger observer
% gain L
lambda5 = -64;
lambda6 = -65;
lambda7 = -66;
lambda8 = -67;

observeDE = [ lambda5 lambda6 lambda7 lambda8 ];
Lnormal = place(A', C', observeDE)';

% kalman filter
% augment system with disturbances and noise
% disturbances covariance
Vd = 1 * eye(4);
% measurement noise covariance
Vn = eye(size(C,1)) * 0.1;
% Vn = 1 * 0.0001;
% augment input with disturbance and noise
BF = [B Vd 0*B];

% build kalman filter
% [L,P,E] = lqe(A,G,C,Q,R,N)
[Lkalman,P,E] = lqe(A,Vd,C,Vd,Vn);
% DF = [0 0 0 0 0 Vn];
% DF = [ DF;
%        DF ];
% sysC = ss(A, BF, C, DF);

%  [KEST,L2,P2] = kalman(sysC,Vd,Vn,0);

% Lnormal or Lkalman or L2
L = Lkalman;


x_bar_obs = [x_bar(1); x_bar(3)];

% simulink model and ode solver parameters
h = 0.01;
stoptime = 30;

% simulation and plotting
simandplot;

