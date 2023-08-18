%% Parameters
m = 0.468;
g = 9.81;

Ixx = 4.856*10^-3;
Iyy = 4.856*10^-3;
Izz = 8.801*10^-3;

J = [ Ixx  0  0
      0  Iyy  0
      0  0  Izz] ;

% Gain values for position error Z-axis
Kz_P = -1.50;
Kz_D = -2.5;
% Gain values for proportional Moment error
K_phi_P = 6;
K_theta_P = 6;
K_psi_P = 6;
% Gain values for Diff Moment error
K_phi_D = 1.75;
K_theta_D = 1.75;
K_psi_D = 1.75;

% Aerodynamic Drag Coefficient
Ax = 0.5;
Ay = 0.5;
Az = 0.5;

Attitude = Vari_States(7:9); 
phi = Attitude(1);
theta = Attitude(2);
psi =Attitude(3);

R1 =  [ cos(psi)   sin(psi)   0
       -sin(psi)   cos(psi)   0
       0         0         1] ;
R2 = [ cos(theta)   0   -sin(theta)
       0            1             0
       sin(theta)   0   cos(theta)] ;
R3 =  [ 1               0          0
        0        cos(phi)   sin(phi)
        0       -sin(phi)   cos(phi)] ;

Force_Drag = [Ax 0 0
              0 Ay 0
              0 0 Ay];

% f1 = (m*g)/4 -1;
% f2 = (m*g)/4 -1;
% f3 = (m*g)/4 +1;
% f4 = (m*g)/4 +1;
% l = 2;
