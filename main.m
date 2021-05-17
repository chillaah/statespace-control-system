%% EGH445 Written Report Assignment
close all; clear; clc

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
knormal;

% lqr controller gains 'K'
% penalising states and control input
Qs1 = 1; % RA Angle
Qs2 = 1; % RA Angular Velocity
Qs3 = 1; % TO Position
Qs4 = 1; % TO Linear Velocity
Ru = 1; % Input Torque

% linear quadratic controller gains 'K'
klqr;

% state feedback controller : u = -K * x(t)
% either Knormal or Klqr
K = Knormal;

% closed loop system with controller
newsys = (A - B*K);
CLpoles = eig(newsys);

% Cart_Pendulum_Animation(NL.t,NL.x1,NL.x2,x_bar(1),x_bar(2))

% RA Angular Velocity & TO Linear Velocity cannot be observed
% Assigning output vector to RA Angle & TO Position
C = [ 1 0 0 0;
      % 0 0 0 0;
      0 0 1 0 ];
      % 0 0 0 0 ];

% observability check
observecheck;
  
% full-order luenberger observer
% gain L
lamda5 = -63;
lamda6 = -64;
lamda7 = -65;
lamda8 = -66;

observeDE = [ lamda5 lamda6 lamda7 lamda8 ];
Lnormal = place(A', C', observeDE)';

L = Lnormal;

x_bar_obs = [x_bar(1); x_bar(3)];

% simulation and plotting
simandplot;

