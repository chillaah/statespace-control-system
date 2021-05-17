%% EGH445 Linear Quadratic Regulator (LQR) Controller

% state sapce model of system
SYS = ss(A, B, C, D);

% state and control input covariance matrices Q & R
Q = eye(length(x_bar));

% penalising states
Q(1,1) = Qs1 * Q(1,1);
Q(2,2) = Qs2 * Q(2,2);
Q(3,3) = Qs3 * Q(3,3);
Q(4,4) = Qs4 * Q(4,4);

% penalising control input
R = Ru * eye(size(u));
N = 0;

% lqr
[Klqr, S, CLP] = lqr(SYS, Q, R, N);
