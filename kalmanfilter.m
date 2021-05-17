% augment system with disturbances and noise
% disturbances covariance
Vd = 0.1 * eye(4);
% noise covariance
Vn = 1;

% augment input with disturbance and noise
BF = [B Vd 0*B];

% build state space system
C = [1 0 0 0];
sysC = ss(A, BF, C, [0 0 0 0 0 Vn]);

% system with full state output, disturbance, no noise
sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));

% build kalman filter
[Kf,P,E] = lqe(A,Vd,C,Vd,Vn);

% system with kalman filter
sysKF = ss(A-Kf*C,[B Kf],eye(4),0*[B Kf]);

dt = 0.01;
t = dt:dt:50;

uDIST = randn(4,size(t,2));
uNOISE = randn(size(t));
u = 0*t;
u(100:120) = 100;
u(1500:1520) = -100;

uAUG = [u; Vd*Vd*uDIST; uNOISE];

[y,t] = lsim(sysC,uAUG,t);
plot(t,y);
hold on
[xtrue,t] = lsim(sysFullOutput,uAUG,t);
plot(xtrue,y);

