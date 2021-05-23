%% EGH445 Simulation & Plotting

% simulink simulation
Lin = sim('TORALinearized', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
NL = sim('TORANonlinear', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');


% plotting
figure();
NL_data = [NL.x, NL.T];
L_data = [Lin.x, Lin.T];
labels = {'x_1 [deg]', 'x_2 [deg/s]', 'x_3 [m]', 'x_4 [m/s]', 'T [N*m]'};
titles = {'Rotational Actuator''s Angle',...
          'Rotational Actuator''s Angular Velocity',...
          'Translational Oscillator''s Position',...
          'Translational Oscillator''s Linear Velocity',...
          'Control Torque'};
% radians to degrees
r2d = [180/pi, 180/pi, 1, 1];
if EP == "a"
    sgtitle('Design Using Linearisation About Equilibrium Point : A',...
            'FontSize', 20, 'interpreter','latex');
else
    sgtitle('Design Using Linearisation About Equilibrium Point : B',...
            'FontSize', 20, 'interpreter','latex');
end

% for ii = 1:4
%     subplot(5,1,ii);
%     plot(NL.t,r2d(ii)*NL.x(:,ii),'b-',... % NL.t,r2d(ii)*NL.x_hat(:,ii),'g-',...
%          Lin.t,r2d(ii)*Lin.x(:,ii),'r--',... % Lin.t,r2d(ii)*Lin.x_hat(:,ii),'k--',...
%          'LineWidth', 2);
%     grid on
%     box on
%     xlabel('time [s]', 'FontSize', 15, 'interpreter','latex');
%     ylabel('$' + labels{ii} + '$', 'FontSize', 15, 'interpreter','latex');
%     title(titles{ii}, 'FontSize', 15, 'interpreter','latex');
%     legend('$x_{1}$ Nonlinear',...
%            '$x_{2}$ hat Nonlinear',...
%            '$x_{3}$ Linearised',...
%            '$x_{4}$ hat Linearised',...
%            'FontSize', 15, 'Location', 'best', 'interpreter','latex');
% end

% angle
subplot(5,1,1);
plot(NL.t,r2d(1)*NL.x(:,1),'b-', Lin.t,r2d(1)*Lin.x(:,1),'r--', 'LineWidth', 2);
grid on
box on
% xlabel('time [s]', 'FontSize', 15, 'interpreter','latex');
ylabel('$x_{1} [deg]$', 'FontSize', 15, 'interpreter','latex');
title(titles{1}, 'FontSize', 15, 'interpreter','latex');
legend('$x_{1} Nonlinear$','$x_{1} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');

% anglular velocity
subplot(5,1,2);
plot(NL.t,r2d(2)*NL.x(:,2),'b-', Lin.t,r2d(2)*Lin.x(:,2),'r--', 'LineWidth', 2);
grid on
box on
% xlabel('time [s]', 'FontSize', 15, 'interpreter','latex');
ylabel('$x_{2} [deg/s]$', 'FontSize', 15, 'interpreter','latex');
title(titles{2}, 'FontSize', 15, 'interpreter','latex');
legend('$x_{2} Nonlinear$','$x_{2} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');

% anglular velocity
subplot(5,1,3);
plot(NL.t,r2d(3)*NL.x(:,3),'b-', Lin.t,r2d(3)*Lin.x(:,3),'r--', 'LineWidth', 2);
grid on
box on
% xlabel('time [s]', 'FontSize', 15, 'interpreter','latex');
ylabel('$x_{3} [m]$', 'FontSize', 15, 'interpreter','latex');
title(titles{3}, 'FontSize', 15, 'interpreter','latex');
legend('$x_{3} Nonlinear$','$x_{3} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');

% position
subplot(5,1,4);
plot(NL.t,r2d(4)*NL.x(:,4),'b-', Lin.t,r2d(4)*Lin.x(:,4),'r--', 'LineWidth', 2);
grid on
box on
% xlabel('time [s]', 'FontSize', 15, 'interpreter','latex');
ylabel('$x_{4} [m/s]$', 'FontSize', 15, 'interpreter','latex');
title(titles{4}, 'FontSize', 15, 'interpreter','latex');
legend('$x_{4} Nonlinear$','$x_{4} Linearized$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');

% input torque
subplot(5,1,5);
plot(NL.t,NL.T,'b-',Lin.t,Lin.T,'r--', 'LineWidth', 2);
grid on
box on
xlabel('time [s]', 'FontSize', 20, 'interpreter','latex');
ylabel('$\tau [N\cdot m]$', 'FontSize', 15, 'interpreter','latex');
title('Input Torque', 'FontSize', 15, 'interpreter','latex');
legend('$\tau Nonlinear$','$\tau Linearised$', 'FontSize', 15, 'Location', 'best', 'interpreter','latex');
