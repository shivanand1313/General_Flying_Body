close all;clear;

dt =0.01 ;

% Initial states of the quadcopter
Position = [0;0;-1] ; % Inertial frame
Velocity = [0;0;0] ; % Body frame
Attitude = [deg2rad(30);deg2rad(0);deg2rad(0)] ; % Respective psi-theta-phi frames
Angularvelocity = [0;0;0] ; % Body frame
%Combined Vector of all States
Vari_States = [Position ; Velocity ; Attitude ; Angularvelocity] ;

% Initial values differential states of quadcopter
Position_diff = [0;0;0] ; % Inertial frame
Velocity_diff = [0;0;0] ; % Body frame
Attitude_diff = [0;0;0] ; % Body frame
Angularvelocity_diff = [0;0;0] ; % Body frame
diff_states = [ Position_diff ; Velocity_diff ; Attitude_diff ; Angularvelocity_diff ] ;

% Fixed Desired initial states of a quadcopter
Position_d = [0;0;-1] ; % 
Attitude_d = [deg2rad(0);deg2rad(0);deg2rad(0)] ;
Desired_states = [ Position_d ; Attitude_d ];

% Desired initial values differential states of quadcopter
Position_d_diff = [0;0;0] ;
Attitude_d_diff = [0;0;0] ;
Desired_states_diff = [Position_d_diff ; Attitude_d_diff ];

% Empty array for Recording data
state = [];
d_states = [];
Rec_Forces = [];
Rec_Ang = [];
Rec_Mom = [];

for t = 0:dt:10
    state = [state; Vari_States'];
    Param;
    Inputs = Stabilisation(Vari_States,diff_states);
   
% Variables for storing Recording data
    Attitude = Vari_States(7:9);
    Rec_Ang = [Rec_Ang; Attitude'];
    Moments = Inputs(4:6);
    Rec_Mom = [Rec_Mom; Moments'];
    F = Inputs(1:3);
    Rec_Forces = [Rec_Forces; F'];
    d_states = [d_states; diff_states'];

    diff_states = differential_states(Vari_States,Inputs);
    Vari_States = Vari_States +  diff_states*dt;

end

%% Plotting
time = 0:dt:10;

% Extracting the values of Position & Velocity along each axis
Px = state(:,1);
Vx = state(:,4);
Py = state(:,2);
Vy = state(:,5);
Pz = state(:,3);
Vz = state(:,6);

% Pz_offset = Position_d(3);
% Pzm = (1.*Pz_offset)+(-Pz-1);

% figure(12)
% quiver3(Px,Py,Pz,Vx,Vy,Vz);

% Recorded Forces in flight in body frame
RFx= Rec_Forces(:,1);
RFy= Rec_Forces(:,2);
RFz= Rec_Forces(:,3);

% Recorded Moments in flight in body frame
RMx= Rec_Mom(:,1);
RMy= Rec_Mom(:,2);
RMz= Rec_Mom(:,3);

%figure(1)
Rphi = Rec_Ang(:,1);
Rtheta = Rec_Ang(:,2);
Rpsi = Rec_Ang(:,3);

% L = sqrt((Rphi.^2) + (Rtheta.^2) + (Rpsi.^2));
% u_phi = Rphi./L;
% u_theta = Rtheta./L;
% u_psi = Rpsi./L;

% quiver3(Px,Py,Pz,u_phi,u_theta,u_psi,"off")
% set(gca,'XLim',[-2 2],'YLim',[0 1],'ZLim',[0.9 1])
% axis normal
  
% figure(14)
% nx= -sin(Rtheta);
% ny= sin(Rphi).*cos(Rtheta);
% nz= -cos(Rphi).*cos(Rtheta);
  
% nx = sin(Rtheta).*cos(Rphi);
% ny = sin(Rtheta).*sin(Rphi);
% nz = cos(Rtheta);

%figure(14)
% nx = sin(u_theta).*cos(u_phi);
% ny = sin(u_theta).*sin(u_phi);
% nz = cos(u_theta);

% set(gca,'XLim',[-2 2],'YLim',[0 1],'ZLim',[-0.5 0])
% view(58,23);
% for i=1:length(time)
%     hold on
%     quiver3(Px(i),Py(i),Pz(i),nx(i),ny(i),nz(i))
%     hold off
%     drawnow
% %   quiver3(Px,Py,Pz,nx,ny,nz)
% %   axis equal
% end

% figure(10)
% quiver3(Px,Py,Pz,RFx,RFy,RFz)
 
% figure(11)
% quiver3(Px,Py,Pz,RMx,RMy,RMz)

figure(13)
% Real time plot
    curve = animatedline('LineWidth',2);
    %set(gca,'XLim',[-0.5 0.5],'YLim',[0 1],'ZLim',[-1.1 -0.9])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    title('Realtime 3D plot for stabilisation trajectory')
    view(58,23);
    for i=1:length(time)
        
        addpoints(curve,Px(i),Py(i),Pz(i))
        drawnow
        axis normal                
    end

% figure(2)
% plot(time,Vz)

% figure(3)
% plot(time,Rec_diff(:,10))
% figure(4)
% plot(time,Rec_diff(:,11))
% figure(5)
% plot(time,Rec_diff(:,12))

figure(4)
plot(time,Rec_Forces(:,1),time,Rec_Forces(:,2),time,Rec_Forces(:,3))
legend('Fx','Fy','Fz')
grid on

figure(5)
plot(time,Px,"-",time,Py,"-",time,(Pz),"--")
legend('Px','Py','Pz')

figure(6)
plot(time,Vx,"-",time,Vy,"-",time,(Vz),"--")
legend('Vx','Vy','Vz')

figure(7)
plot(time,rad2deg(state(:,7)),"-",time,rad2deg(state(:,8)),"-",time,rad2deg(state(:,9)),"-")%,time,Vz,"--")
legend('\phi','\theta','\psi')
% 
figure(8)
plot(time,Rec_Mom(:,1),time,Rec_Mom(:,2),time,Rec_Mom(:,3))
legend('M Roll','M Pitch','M Yaw')


%figure(9)
% plot(time,rad2deg(Rec_Ang(:,1)))

% Real time plot
%     curve = animatedline('LineWidth',2);
%     set(gca,'XLim',[-0.5 0.1],'YLim',[0 1],'ZLim',[-1.1 -1])
%     view(-110,24);
%     for i=1:length(time)
%         addpoints(curve,Px(i),Py(i),Pz(i))
%         axis normal
%         drawnow
%     end