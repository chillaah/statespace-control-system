%% EGH445 Simulation & Plotting

% simulink simulation
Lin = sim('TORALinearized', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');
NL = sim('TORANonlinear', 'Solver', 'ode4', 'FixedStep', 'h', 'StopTime', 'stoptime');


% plotting
figure();
NL_data = [NL.x, NL.T];
L_data = [Lin.x, Lin.T];
labels = {'x_1 [rad]', 'x_2 [rad/s]', 'x_3 [m]', 'x_4 [m/s]', 'T [N*m]'};
titles = {'Rotational Actuator''s Angle',...
          'Rotational Actuator''s Angular Velocity',...
          'Translational Oscillator''s Position',...
          'Translational Oscillator''s Linear Velocity',...
          'Control Torque'};
% radians to degrees
r2d = [180/pi, 180/pi, 1, 1];
if EP == "a"
    sgtitle('Design Using Linearisation about Equilibrium Point : A');
else
    sgtitle('Design Using Linearisation about Equilibrium Point : B');
end

for ii = 1:4 
    subplot(5,1,ii);
    plot(NL.t,r2d(ii)*NL.x(:,ii),'b-',NL.t,r2d(ii)*NL.x_hat(:,ii),'g-',...
         Lin.t,r2d(ii)*Lin.x(:,ii),'r--',Lin.t,r2d(ii)*Lin.x_hat(:,ii),'k--',...
         'LineWidth', 1);
    grid on
    box on
    xlabel('time [s]', 'FontSize', 15);
    ylabel(labels{ii}, 'FontSize', 15);
    title(titles{ii}, 'FontSize', 15);
    legend(strcat('x_',num2str(ii),' Nonlinear'),...
           strcat('x_',num2str(ii),' hat Nonlinear'),...
           strcat('x_',num2str(ii),' Linearised'),...
           strcat('x_',num2str(ii),' hat Linearised'), 'FontSize', 15,...
           'Location', 'east');
end

% input torque plot
subplot(5,1,5);
plot(NL.t,NL.T,'b-',Lin.t,Lin.T,'r--', 'LineWidth', 1);
grid on
box on
xlabel('time [s]', 'FontSize', 15);
ylabel(labels{5}, 'FontSize', 15);
legend('T Nonlinear','T Linearised', 'FontSize', 15, 'Location', 'east');
