function Inputs = Stabilisation(Vari_States,diff_states)

% Input variables to function are inherited from previous loop iteration of
% main program

Param % cointains the value of the parameters for the program

% Body states at instant of time extracted from Input to function
Position = Vari_States(1:3);
Pz = Position(3);
Attitude = Vari_States(7:9);
phi = Attitude(1);
theta = Attitude(2);
psi = Attitude(3);

% Body differential states at instant of time extracted from Input to 
% function 
Position_diff = diff_states(1:3);
Pz_diff = Position_diff(3);
Attitude_diff = diff_states(7:9);
phi_diff = Attitude_diff(1);
theta_diff = Attitude_diff(2);
psi_diff = Attitude_diff(3);

% Fixed Desired initial states of a quadcopter
Position_d = [0;0;-1] ; % 
Pz_d = Position_d(3);
Attitude_d = [deg2rad(0);deg2rad(0);deg2rad(0)] ;
phi_d = Attitude_d(1);
theta_d = Attitude_d(2);
psi_d =Attitude_d(3);

% Desired initial values differential states of quadcopter
Position_d_diff = [0;0;0] ;
Pz_d_diff = Position_d_diff(3);
Attitude_d_diff = [0;0;0] ;
phi_d_diff = Attitude_d_diff(1);
theta_d_diff = Attitude_d_diff(2);
psi_d_diff =Attitude_d_diff(3);


% Errors
e_z = Pz_d - Pz ;
e_phi = phi_d - phi ;
e_theta = theta_d - theta ;
e_psi = psi_d - psi ;

% Differential Errors
e_z_diff = Pz_d_diff - Pz_diff ;
e_phi_diff = phi_d_diff - phi_diff ;
e_theta_diff = theta_d_diff - theta_diff ;
e_psi_diff = psi_d_diff - psi_diff ;

%Forces and Moments PID calculations 
Fz = (g +(Kz_D*e_z_diff) + (Kz_P*e_z)) * (m/(cos(phi)*cos(theta))) ;

M_Roll = ( K_phi_D*(e_phi_diff)+ K_phi_P*(e_phi) )*Ixx ;
M_Pitch = ( K_theta_D*(e_theta_diff)+ K_theta_P*(e_theta) )*Iyy ;
M_Yaw = ( K_psi_D*(e_psi_diff)+ K_psi_P*(e_psi) )*Izz ;

%Forces and Moments calculated from PID 
Forces = [0;0;Fz] ;
Moments = [M_Roll;M_Pitch;M_Yaw] ;

Inputs = [Forces;Moments] ;
end
