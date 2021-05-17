%% EGH445 Equilibrium Points

% solving for equilibrium points
syms x1 x3
u = 0;
x2 = 0;
x4 = 0;

eq1Num = (m + M)*u - m*L*cos(x1) * (m*L*x2^2*sin(x1) - k*x3);
eq2Num = -m*L*u*cos(x1) + (J + m*L^2) * (m*L*x2^2*sin(x1) - k*x3);
delTheta = (J + m*L^2)*(m + M) - m^2*L^2*cos(x1)^2;

eq1 = eq1Num/delTheta;
eq2 = eq2Num/delTheta;

[x1, x3] = solve(eq1 == 0, eq2 == 0);
x1 = double(x1); x3 = double(x3);

xa_bar = [ x1(2); x2; x3(2); x4 ];
xb_bar = [ x1(1); x2; x3(1); x4 ];

clear x1 x2 x3 x4
