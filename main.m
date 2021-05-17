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
PO = 5;
% settling time
Ts = 5;
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

% simulink model and ode solver parameters
h = 0.02; 
stoptime = 10;

NL = sim('TORANonlinear', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
L = sim('TORALinearized', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
figure();
NL_data = [NL.x, NL.T];
L_data = [L.x, L.T];
labels = {'x_1 [rad]', 'x_2 [rad/s]', 'x_3 [m]', 'x_4 [m/s]', 'T [N*m]'};
titles = {'Rotational Actuator''s Angle',...
          'Rotational Actuator''s Angular Velocity',...
          'Translational Oscillator''s Position',...
          'Translational Oscillator''s Linear Velocity',...
          'Input Torque'};

if EP == "a"
    sgtitle('Design Using Linearisation about Equilibrium Point : A');
else
    sgtitle('Design Using Linearisation about Equilibrium Point : B');
end

for ii = 1:5 
    subplot(5,1,ii);
    plot(NL.t, NL_data(:,ii), 'b', L.t, L_data(:,ii), 'r--', 'LineWidth', 2);
    grid on
    box on
    xlabel('time [s]', 'FontSize', 15);
    ylabel(labels{ii});
    title(titles{ii}, 'FontSize', 15);
    legend('Non-Linear', 'Linearized', 'FontSize', 15,'Location', 'best');
end

% Cart_Pendulum_Animation(NL.t,NL.x1,NL.x2,x_bar(1),x_bar(2))
% observability check
observecheck;

