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
phi = b(4);   % rotation about the forward (x) axis
theta = b(5); % rotation about the y axis
psi = b(6);   % rotation about the z axis
% front left leg
theta_A1 = b(7);
theta_A2 = b(8);
theta_A3 = b(9);
% front right leg
theta_B1 = b(10);
theta_B2 = b(11);
theta_B3 = b(12);
% back left leg
theta_C1 = b(13);
theta_C2 = b(14);
theta_C3 = b(15);
% back right leg
theta_D1 = b(16);
theta_D2 = b(17);
theta_D3 = b(18);


% translational velocities of main body
dot_delta_x = b(19);
dot_delta_y = b(20);
dot_delta_z = b(21);
% angular velocities of main body
dot_phi = b(22);
dot_theta = b(23);
dot_psi = b(24);
% front left leg
dot_theta_A1 = b(25);
dot_theta_A2 = b(26);
dot_theta_A3 = b(27);
% front right leg
dot_theta_B1 = b(28);
dot_theta_B2 = b(29);
dot_theta_B3 = b(30);
% back left leg
dot_theta_C1 = b(31);
dot_theta_C2 = b(32);
dot_theta_C3 = b(33);
% back right leg
dot_theta_D1 = b(34);
dot_theta_D2 = b(35);
dot_theta_D3 = b(36);


gamma = b(1:18);
dot_gamma = b(18:36);


% joint positions
r_I_I_r_F = zeros(3,1);

r_F_F_r_A1 = zeros(3,1);
r_A1_A1_r_A2 = zeros(3,1);
r_A2_A2_r_A3 = zeros(3,1);

r_F_F_r_B1 = zeros(3,1);
r_B1_B1_r_B2 = zeros(3,1);
r_B2_B2_r_B3 = zeros(3,1);

r_F_F_r_C1 = zeros(3,1);
r_C1_C1_r_C2 = zeros(3,1);
r_C2_C2_r_C3 = zeros(3,1);

r_F_F_r_D1 = zeros(3,1);
r_D1_D1_r_D2 = zeros(3,1);
r_D2_D2_r_D3 = zeros(3,1);


% mass parameters
m_F

m_A1
m_A2
m_A3

m_B1
m_B2
m_B3

m_C1
m_C2
m_C3

m_D1
m_D2
m_D3


% Moments of inertia (negative tensor form)
J_FFJ

J_A1_A1_J
J_A2_A2_J
J_A3_A3_J

J_B1_B1_J
J_B2_B2_J
J_B3_B3_J

J_C1_C1_J
J_C2_C2_J
J_C3_C3_J

J_D1_D1_J
J_D2_D2_J
J_D3_D3_J



% jacobian for Frame frame
xyz_KDE = [cos(phi)*cos(theta), sin(phi), 0; -cos(theta)*sin(phi), cos(phi), 0; sin(theta), 0, 1];

Jac_F = [zeros(3, 3),     xyz_KDE, zeros(3, 8);...
           eye(3, 3), zeros(3, 3), zeros(3, 8)];
    

dot_Jac_F = zeros(6, size(b, 1)/2);
dot_Jac_F(1, 4) = -sin(psi)*dot_psi*cos(theta) - sin(theta)*dot_theta*cos(psi);
dot_Jac_F(1, 5) = dot_psi*cos(psi);
dot_Jac_F(2, 4) = sin(psi)*sin(theta)*dot_theta - cos(psi)*cos(theta)*dot_psi;
dot_Jac_F(2, 5) = -dot_psi*sin(psi);
dot_Jac_F(3, 4) = dot_theta*cos(theta);


% set up I_at matricies (mapping of angular velocity for rotatinoal joints)
% I_hat_F is unused
I_hat_A1 = zeros(3, size(b, 1)/2);
I_hat_A2 = zeros(3, size(b, 1)/2);
I_hat_A3 = zeros(3, size(b, 1)/2);

I_hat_B1 = zeros(3, size(b, 1)/2);
I_hat_B2 = zeros(3, size(b, 1)/2);
I_hat_B3 = zeros(3, size(b, 1)/2);

I_hat_C1 = zeros(3, size(b, 1)/2);
I_hat_C2 = zeros(3, size(b, 1)/2);
I_hat_C3 = zeros(3, size(b, 1)/2);

I_hat_D1 = zeros(3, size(b, 1)/2);
I_hat_D2 = zeros(3, size(b, 1)/2);
I_hat_D3 = zeros(3, size(b, 1)/2);


% I_hat_F is unused
error("defined by geometry")


% set up I_tilde matricies (mapping of linear velocity for translational
% joints)
% I_tilde_F is unused
I_tilde_A1 = zeros(3, size(b, 1)/2);  
I_tilde_A2 = zeros(3, size(b, 1)/2);
I_tilde_A3 = zeros(3, size(b, 1)/2);

I_tilde_B1 = zeros(3, size(b, 1)/2);  
I_tilde_B2 = zeros(3, size(b, 1)/2);
I_tilde_B3 = zeros(3, size(b, 1)/2);

I_tilde_C1 = zeros(3, size(b, 1)/2);
I_tilde_C2 = zeros(3, size(b, 1)/2);
I_tilde_C3 = zeros(3, size(b, 1)/2);

I_tilde_D1 = zeros(3, size(b, 1)/2);
I_tilde_D2 = zeros(3, size(b, 1)/2);
I_tilde_D3 = zeros(3, size(b, 1)/2);



% Direction Cosine Matrices
T_I_T_F = rotx(phi)*roty(theta)*rotz(psi);

T_F_T_A1 = roty(theta_A1);
T_A1_T_A2 = roty(theta_A2);
T_A2_T_A3 = roty(theta_A3);

T_F_T_B1 = roty(theta_B1);
T_B1_T_B2 = roty(theta_B2);
T_B2_T_B3 = roty(theta_B3);

T_F_T_C1 = roty(theta_C1);
T_C1_T_C2 = roty(theta_C2);
T_C2_T_C3 = roty(theta_C3);

T_F_T_D1 = roty(theta_D1);
T_D1_T_D2 = roty(theta_D2);
T_D2_T_D3 = roty(theta_D3);

%% Recursive FK

w_FFwI = [dot_phi; dot_theta; dot_psi];

[T_I_T_F, w_FFwI, Jac_F, dot_Jac_F] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);

[T_ITA, w_AAwI, Jac_A, dot_Jac_A] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_A_IT1, w_A_11wI, Jac_A_1, dot_Jac_A_1] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_A_IT2, w_A_22wI, Jac_A_2, dot_Jac_A_2] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);

[T_ITB, w_BBwI, Jac_B, dot_Jac_B] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_B_IT1, w_B_11wI, Jac_B_1, dot_Jac_B_1] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_B_IT2, w_B_22wI, Jac_B_2, dot_Jac_B_2] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);

[T_ITC, w_CCwI, Jac_C, dot_Jac_C] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_C_IT1, w_C_11wI, Jac_C_1, dot_Jac_C_1] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_C_IT2, w_C_22wI, Jac_C_2, dot_Jac_C_2] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);

[T_ITD, w_DDwI, Jac_D, dot_Jac_D] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_D_IT1, w_D_11wI, Jac_D_1, dot_Jac_D_1] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);
[T_D_IT2, w_D_22wI, Jac_D_2, dot_Jac_D_2] = RecursiveFK(T_I_T_F, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_FFrA, I_hat_A, I_tilde_A, dot_gamma);

%% 
 
H =   Jac_F' * [J_FFJ, skew(G_FFG)*T_I_T_F'; -T_I_T_F*skew(G_FFG), m_F*eye(3)] * Jac_F + ...
      Jac_A' * [J_AAJ, skew(G_AAG)*T_ITA'; -T_ITA*skew(G_AAG), m_A*eye(3)] * Jac_A +...
    Jac_A_1' * [J_A_11J, skew(G_A_11G)*T_A_IT1'; -T_A_IT1*skew(G_A_11G), m_A_1*eye(3)] * Jac_A1 +...
    Jac_A_2' * [J_A_22J, skew(G_22G)*T_IT2'; -T_IT2*skew(G_22G), m_2*eye(3)] * Jac_2 +...
      Jac_B' * [J_BBJ, skew(G_33G)*T_IT3'; -T_IT3*skew(G_33G), m_3*eye(3)] * Jac_3 +...
    Jac_B_1' * [J_B_11J, skew(G_44G)*T_IT4'; -T_IT4*skew(G_44G), m_4*eye(3)] * Jac_4 +...
    Jac_B_2' * [J_B_22J, skew(G_55G)*T_IT5'; -T_IT5*skew(G_55G), m_5*eye(3)] * Jac_5 +...
      Jac_C' * [J_CCJ, skew(G_66G)*T_IT6'; -T_IT6*skew(G_66G), m_6*eye(3)] * Jac_6 +...
    Jac_C_1' * [J_C_11J, skew(G_77G)*T_IT7'; -T_IT7*skew(G_77G), m_7*eye(3)] * Jac_7 +...
    Jac_C_2' * [J_C_22J, skew(G_SSG)*T_ITS'; -T_ITS*skew(G_SSG), m_S*eye(3)] * Jac_S;
    Jac_C_D' * [J_DDJ, skew(G_SSG)*T_ITS'; -T_ITS*skew(G_SSG), m_S*eye(3)] * Jac_S;
    Jac_D_1' * [J_D_11J, skew(G_SSG)*T_ITS'; -T_ITS*skew(G_SSG), m_S*eye(3)] * Jac_S;
    Jac_D_2' * [J_D_22J, skew(G_SSG)*T_ITS'; -T_ITS*skew(G_SSG), m_S*eye(3)] * Jac_S;











end