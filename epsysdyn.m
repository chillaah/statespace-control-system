%% EGH445 EP & Linearized System Dynamics

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
    
    x0 = [ 20*pi/180; 0; 0; 0 ];
    
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
    
    x0 = [ 100*pi/180; 0; 0; 0 ];
    
    x_bar = xb_bar;
    
    x0_linear = x0 - x_bar;
    
% invalid EP
else
    
    disp("invalid EP. Please enter 'a' or 'b' only.");
    return
    
end
