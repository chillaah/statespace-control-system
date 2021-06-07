%% EGH445 - Written Report
close all; clear; clc

% simulink model and ode solver parameters
h = 0.01;
stoptime = 5;

% Initial Values
M = 1.3608; % mass of TO
m = 0.096; % mass of RA
J = 0.0002175; % applied intertia
k = 186.3; % spring constant
l = 1; % distance from RA axis

% solving for equilibrium points
syms x1 x3
u = 0;
x2 = 0;
x4 = 0;

eq1Num = (m + M)*u - m*l*cos(x1) * (m*l*x2^2*sin(x1) - k*x3);
eq2Num = -m*l*u*cos(x1) + (J + m*l^2) * (m*l*x2^2*sin(x1) - k*x3);
delTheta = (J + m*l^2)*(m + M) - m^2*l^2*cos(x1)^2;

eq1 = eq1Num/delTheta;
eq2 = eq2Num/delTheta;

[x1, x3] = solve(eq1 == 0, eq2 == 0);
x1 = double(x1); x3 = double(x3);

xa_bar = [ x1(2); x2; x3(2); x4 ];
xb_bar = [ x1(1); x2; x3(1); x4 ];

clear x1 x2 x3 x4

% get associated equilibrium points' linearized system dynamics
% infinite EP's while x3 = position = 0, therefore
% EPa - RA angle which has most influence on TO displacement
% EPb - RA angle which has least influence on TO displacement
EP = "a";

% EPa has the dynamics which has most influence on TORA displacement
if EP == "a"
    
    ALPHAa = m*k*l*cos(xa_bar(1)) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xa_bar(1))^2);
    
    BETAa = -k*(J + m*l^2) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xa_bar(1))^2);
    
    A = [ 0 1      0 0;
          0 0 ALPHAa 0;
          0 0      0 1;
          0 0  BETAa 0 ];
    
    GAMMAa = (m + M) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xa_bar(1))^2);
    
    ETAa = -m*l*cos(xa_bar(1)) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xa_bar(1))^2);
    
    B = [     0;
         GAMMAa;
              0;
           ETAa ];
    
    C = [ 0 0 0 0;
          0 1 0 0;
          0 0 0 0;
          0 0 0 1 ];

    D = zeros(size(C, 1), size(B, 2));
    
    x0 = [ wrapTo180(20)*pi/180; 0; 0.1; 0 ];
    
    x_bar = xa_bar;
    
    x0_linear = x0 - x_bar;
    
% EPb has the dynamics which has least influence on TORA displacement
elseif EP == "b"
    
    ALPHAb = m*k*l*cos(xb_bar(1)) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xb_bar(1))^2);
    
    BETAb = -k*(J + m*l^2) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xb_bar(1))^2);
    
    A = [ 0 1      0 0;
          0 0 ALPHAb 0;
          0 0      0 1;
          0 0  BETAb 0 ];
    
    GAMMAb = (m + M) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xb_bar(1))^2);
    
    ETAb = -m*l*cos(xb_bar(1)) / ((J + m*l^2)*(m + M) - m^2*l^2*cos(xb_bar(1))^2);
    
    B = [     0;
         GAMMAb;
              0;
           ETAb ];
    
    C = [ 0 0 0 0;
          0 1 0 0;
          0 0 0 0;
          0 0 0 1 ];
    
    D = zeros(size(C, 1), size(B, 2));
    
    x0 = [ wrapTo180(110)*pi/180; 0; 0.1; 0 ];
    
    x_bar = xb_bar;
    
    x0_linear = x0 - x_bar;
    
% invalid EP
else
    
    disp("invalid EP. please enter 'a' or 'b' only.");
    return
    
end

% lyapunov's "indirect" method criterion for non-linear systems'
% stability checking
lambda = eig(A);

if (sum(lambda > 0) >= 1)
    
    fprintf('Equilibrium Point "%s" is Unstable \n\n', EP);
    
elseif (sum(lambda == 0) >= 1)
    
    fprintf('Equilibrium Point "%s" is Lyapunov''s "Indirect" Method Inconclusive \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is Stable \n\n', EP);
    
end

% controller switch
controller_enabled = 1;

% controllability matrix check
Cab = ctrb(A, B);

if (rank(Cab) == length(A))
    
    fprintf('Equilibrium Point "%s" is Completely Controllable \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Controllable \n\n', EP);
    
end

%% Plot 1

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

% state sapce model of system
SYS = ss(A, B, C, D);

% state and control input covariance matrices Q & R
Qcon = eye(length(x_bar));

% penalising states
Qcon(1,1) = Qs1 * Qcon(1,1);
Qcon(2,2) = Qs2 * Qcon(2,2);
Qcon(3,3) = Qs3 * Qcon(3,3);
Qcon(4,4) = Qs4 * Qcon(4,4);

% penalising control input
Rcon = Ru * eye(size(u));

Ncon = [Nin1, Nin2, Nin3, Nin4]';

% lqr
[Klqr, S, CLP] = lqr(SYS, Qcon, Rcon, Ncon);

% state feedback controller : u = -K * x(t)
% either Knormal or Klqr
K = Klqr;

% closed loop system with controller
consys = (A - B*K);
conpoles = eig(consys);

% observer switch
use_state_estimates = 0;

% RA Angular Velocity & TO Linear Velocity cannot be observed
% Assigning output matrix to be only RA Angle & TO Position
Cnew = [ 1 0 0 0;
         0 0 1 0 ];

Dnew = zeros(size(Cnew,1), size(B,2));

% observability matrix check
Oac = obsv(A, Cnew);

if (rank(Oac) == length(A))
    
    fprintf('Equilibrium Point "%s" is Completely Observable \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Observable \n\n', EP);
    
end

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
BF = [B G]; % system input, process disturbance, measurement noise
midZeros = zeros(size(Dnew,1),size(G,2));
DF = [Dnew midZeros]; % system feed-through, process disturbance, measurement noise
syskf = ss(A, BF, Cnew, DF);
[kest,Lkalman,PP] = kalman(syskf,Qcov,Rcov,N);
% either Lnormal or Lkalman
L = Lkalman;

% closed loop system with observer
obssys = (A - L*Cnew);
obspoles = eig(obssys);

x_bar_obs = [x_bar(1); x_bar(3)];

% simulation and plotting
simandplotcontrol;


%% Plot 2

% observer switch
use_state_estimates = 1;

% simulation and plotting
simandplotobserve;


%% Plot 3

x0 = [ wrapTo180(180)*pi/180; 0; 0; 0 ];

x0_linear = x0 - x_bar;

% observer switch
use_state_estimates = 0;

% simulink simulation
Lin = sim('TORALinearized', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
NL = sim('TORANonlinear', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');

% plotting
figure();
NL_data = [NL.x, NL.T];
Lin_data = [Lin.x, Lin.T];
r2d = [180/pi, 180/pi, 1, 1];

% angle
subplot(2,1,1);
plot(NL.t,r2d(1)*NL.x(:,1),'b-', Lin.t,r2d(1)*Lin.x(:,1),'r--', 'LineWidth', 2);
grid on
box on
xlabel('time [s]', 'FontSize', 20, 'interpreter','latex');
ylabel('$x_{1} [deg]$', 'FontSize', 15, 'interpreter','latex');
title('Rotational Actuator''s Angle', 'FontSize', 15, 'interpreter','latex');
legend('$x_{1} Nonlinear$','$x_{1} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');


%% Plot 4 (3 and 4 were plotted on the same figure)

x0 = [ wrapTo180(0)*pi/180; 0; 0.3; 0 ];

x0_linear = x0 - x_bar;

% simulink simulation
Lin = sim('TORALinearized', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
NL = sim('TORANonlinear', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');

% plotting
NL_data = [NL.x, NL.T];
Lin_data = [Lin.x, Lin.T];
r2d = [180/pi, 180/pi, 1, 1];

% position
subplot(2,1,2);
plot(NL.t,r2d(3)*NL.x(:,3),'b-', Lin.t,r2d(3)*Lin.x(:,3),'r--', 'LineWidth', 2);
grid on
box on
xlabel('time [s]', 'FontSize', 20, 'interpreter','latex');
ylabel('$x_{3} [m]$', 'FontSize', 15, 'interpreter','latex');
title('Translational Oscillator''s Position', 'FontSize', 15, 'interpreter','latex');
legend('$x_{3} Nonlinear$','$x_{3} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');


%% Plot 5

x0 = [ wrapTo180(20)*pi/180; 0; 0.1; 0 ];

x0_linear = x0 - x_bar;

% for equilibrium point "a" (controllable EP)
% from the 4 poles, designing for 2 poles
% so that is it a 2nd order system
% making 2 poles to be more than 5x far away
% dominant pole theory!
% designing for overshoot 5% and settling time 2s
% overshoot(%)
PO = 5;
% settling time
Ts = 2;
% non-dominant (fast poles) distances from dominant (slow poles)
% if ndpl1 ~= ndpl2 -> 'place' will be used
ndpl1 = 10;
ndpl2 = 10;

% damping ratio, natural frequency & roots
zeta = -log(PO/100)/sqrt(pi^2 + log(PO/100)^2);
wn = 4 / (Ts * zeta);

% 2nd order TF form
r = roots([1, 2*zeta*wn, wn^2]);

% 2 slow poles and 2 fast poles
lambda1 = r(1); 
lambda2 = r(2);
lambda3 = ndpl1 * lambda1;
lambda4 = ndpl2 * lambda2;

% controller array
controlDE = [lambda1 lambda2 lambda3 lambda4];

% 'acker' vs 'place' function logic
if numel(unique(real(controlDE))) == 4
    Knormal = place(A, B, controlDE);
else
    Knormal = acker(A, B, controlDE);
end

% state feedback controller : u = -K * x(t)
% either Knormal or Klqr
K = Knormal;

% closed loop system with controller
consys = (A - B*K);
conpoles = eig(consys);

% simulation and plotting
simandplotcontrol;


%% Plot 6

% state feedback controller : u = -K * x(t)
% either Knormal or Klqr
K = Klqr;

% closed loop system with controller
consys = (A - B*K);
conpoles = eig(consys);

% observer switch
use_state_estimates = 1;

% full-order luenberger observer
% gain L
% observer poles are 10 to 20 times further than
% regulator poles
lambda5 = 20 * real(lambda1) + imag(lambda1)*1i;
lambda6 = 20 * real(lambda2) + imag(lambda2)*1i;
lambda7 = 10 * real(lambda3) + imag(lambda3)*1i;
lambda8 = 10 * real(lambda4) + imag(lambda4)*1i;

observeDE = [ lambda5 lambda6 lambda7 lambda8 ];
Lnormal = place(A', Cnew', observeDE)';

% either Lnormal or Lkalman
L = Lnormal;

% closed loop system with observer
obssys = (A - L*Cnew);
obspoles = eig(obssys);

% simulation and plotting
simandplotobserve;





