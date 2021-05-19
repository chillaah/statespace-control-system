%% EGH445 Linear Quadratic Regulator (LQR) Controller

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
N = 0;

% lqr
[Klqr, S, CLP] = lqr(SYS, Qcon, Rcon, N);
