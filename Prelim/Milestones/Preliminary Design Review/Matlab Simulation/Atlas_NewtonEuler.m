function [dot_b] = Atlas_NewtonEuler(b, F)
% Returns derivative of state for the Atlas System when
% a current state and Torque-Force matrix is supplied.
%
% Inputs:
%   b: state of Atlas system (m, rad, m/s, rad/s)
%   F: Torque-Force matrix    (Nm, N)
%
% Outputs:
%   dot_b: (m/s, rad/s, m/s/s, rad/s/s)
%
% Example:
%   [out01, out02] = functionName(in01, in02);
% 
% Description:
%   This function is to be used as a dependant of Atlas_NewtonEulerSolver.m
%   for simulation across time via a Runge-Kutta integrator.
%
% required m-files:
%   RecursiveFK.m
%     for recursive forward kinematics calculation
%   rotx.m
%     DCM rotation about x axis
%   roty.m
%     DCM rotation about y axis
%   rotz.m
%     DCM rotation about z axis
%   skew.m
%     for calculating the skew symmetric matrix of a given matrix
%
% Subfunctions:
%   None
%
% required MAT-files:
%   None
%
% Author: Ian Adelman
% Email: Adelmani@my.erau.edu
% Maintainer: Ian Adelman
% Maintainer Email: Adelmani@my.erau.edu
% Created: createdDate
% Revised: revisedDate
%
% Previous Ver #: 0.0
%  Current Ver #: 1.0 : 10/27/2023
% Most Recent Version Notes:
%   started Everything
%

% translational position of main body
delta_x = b(1);
delta_y = b(2);
delta_z = b(3);
% pose angles of main body
phi = b(4);
theta = b(5);
psi = b(6);
% front left leg
A_theta_1 = b(7);
A_theta_2 = b(8);
A_theta_3 = b(9);
% front right leg
B_theta_1 = b(10);
B_theta_2 = b(11);
B_theta_3 = b(12);
% back left leg
C_theta_1 = b(13);
C_theta_2 = b(14);
C_theta_3 = b(15);
% back right leg
D_theta_1 = b(16);
D_theta_2 = b(17);
D_theta_3 = b(18);


% translational velocities of main body
dot_delta_x = b(19);
dot_delta_y = b(20);
dot_delta_z = b(21);
% angular velocities of main body
dot_phi = b(22);
dot_theta = b(23);
dot_psi = b(24);
% front left leg
dot_A_theta_1 = b(25);
dot_A_theta_2 = b(26);
dot_A_theta_3 = b(27);
% front right leg
dot_B_theta_1 = b(28);
dot_B_theta_2 = b(29);
dot_B_theta_3 = b(30);
% back left leg
dot_C_theta_1 = b(31);
dot_C_theta_2 = b(32);
dot_C_theta_3 = b(33);
% back right leg
dot_D_theta_1 = b(34);
dot_D_theta_2 = b(35);
dot_D_theta_3 = b(36);


gamma = b(1:18);
dot_gamma = b(18:36);


% joint positions
r_IIrF = zeros(3,1);

r_FFrA = zeros(3,1);
r_A_AAr1 = zeros(3,1);
r_A_11r2 = zeros(3,1);

r_FFrB = zeros(3,1);
r_B_BBr1 = zeros(3,1);
r_B_11r2 = zeros(3,1);

r_FFrC = zeros(3,1);
r_C_CCr1 = zeros(3,1);
r_C_11r2 = zeros(3,1);

r_FFrD = zeros(3,1);
r_D_DDr1 = zeros(3,1);
r_D_11r2 = zeros(3,1);



% mass parameters




% Moments of inertia (negative tensor form)


% jacobian for Frame frame
xyz_KDE = [cos(phi)*cos(theta), sin(phi), 0; -cos(theta)*sin(phi), cos(phi), 0; sin(theta), 0, 1];

Jac_F = [zeros(3, 3),     xyz_KDE, zeros(3, 8);...
           eye(3, 3), zeros(3, 3), zeros(3, 8)];
    

dot_Jac_F = zeros(6, size(b, 1)/2);
dot_Jac_(1, 4) = -sin(psi)*dot_psi*cos(theta) - sin(theta)*dot_theta*cos(psi);
dot_Jac_F(1, 5) = dot_psi*cos(psi);
dot_Jac_F(2, 4) = sin(psi)*sin(theta)*dot_theta - cos(psi)*cos(theta)*dot_psi;
dot_Jac_F(2, 5) = -dot_psi*sin(psi);
dot_Jac_F(3, 4) = dot_theta*cos(theta);


% set up I_at matricies (mapping of angular velocity for rotatinoal joints)
% I_hat_F is unused
I_hat_A = zeros(3, size(b, 1)/2);
I_hat_A_1 = zeros(3, size(b, 1)/2);
I_hat_A_2 = zeros(3, size(b, 1)/2);

I_hat_B = zeros(3, size(b, 1)/2);
I_hat_B_1 = zeros(3, size(b, 1)/2);
I_hat_B_2 = zeros(3, size(b, 1)/2);

I_hat_C = zeros(3, size(b, 1)/2);
I_hat_C_1 = zeros(3, size(b, 1)/2);
I_hat_C_2 = zeros(3, size(b, 1)/2);

I_hat_D = zeros(3, size(b, 1)/2);
I_hat_D_1 = zeros(3, size(b, 1)/2);
I_hat_D_2 = zeros(3, size(b, 1)/2);


% I_hat_F is unused
error("defined by geometry")


% set up I_tilde matricies (mapping of linear velocity for translational
% joints)
% I_tilde_F is unused
I_tilde_A = zeros(3, size(b, 1)/2);  
I_tilde_A_1 = zeros(3, size(b, 1)/2);
I_tilde_A_2 = zeros(3, size(b, 1)/2);

I_tilde_B = zeros(3, size(b, 1)/2);  
I_tilde_B_1 = zeros(3, size(b, 1)/2);
I_tilde_B_2 = zeros(3, size(b, 1)/2);
I_tilde_C = zeros(3, size(b, 1)/2);
I_tilde_C_1 = zeros(3, size(b, 1)/2);
I_tilde_C_2 = zeros(3, size(b, 1)/2);

I_tilde_D = zeros(3, size(b, 1)/2);
I_tilde_D_1 = zeros(3, size(b, 1)/2);
I_tilde_D_2 = zeros(3, size(b, 1)/2);



% Direction Cosine Matrices
T_ITF = rotx(phi)*roty(theta)*rotz(psi);

T_FTA = rotx(A_theta_1)
T_A_AT1 = rotx(A_theta_2)
T_A_1T2 = rotx(A_theta_3)

T_FTB = rotx(B_theta_1)
T_B_BT1 = rotx(B_theta_2)
T_B_1T2 = rotx(B_theta_3)

T_FTC = rotx(C_theta_1)
T_C_CT1 = rotx(C_theta_2)
T_C_1T2 = rotx(C_theta_3)

T_FTD = rotx(D_theta_1)
T_D_DT1
T_D_1T

end