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
T_ITF = rotx(phi)*roty(theta)*rotz(psi);

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

[T_ITF, w_FFwI, Jac_F, dot_Jac_F] = RecursiveFK(T_ITF, T_F_T_A1, Jac_F(1:3, :)*dot_gamma, Jac_F, dot_Jac_F, r_F_F_r_A1, I_hat_A1, I_tilde_A1, dot_gamma);

[T_ITA1, w_A1_A1_w_I, Jac_A1, dot_Jac_A1] = RecursiveFK(T_ITF,  T_F_T_A1,  Jac_F(1:3, :)*dot_gamma, Jac_F,  dot_Jac_F,  r_F_F_r_A1,   I_hat_A1, I_tilde_A1, dot_gamma);
[T_ITA2, w_A2_A2_w_I, Jac_A2, dot_Jac_A2] = RecursiveFK(T_ITA1, T_A1_T_A2, Jac_F(1:3, :)*dot_gamma, Jac_A1, dot_Jac_A2, r_A1_A1_r_A2, I_hat_A2, I_tilde_A2, dot_gamma);
[T_ITA3, w_A2_A2_w_I, Jac_A3, dot_Jac_A3] = RecursiveFK(T_ITA2, T_A2_T_A3, Jac_F(1:3, :)*dot_gamma, Jac_A2, dot_Jac_A2, r_A2_A2_r_A3, I_hat_A3, I_tilde_A3, dot_gamma);

[T_ITB1, w_B1_B1_w_I, Jac_B1, dot_Jac_B1] = RecursiveFK(T_ITF,  T_F_T_B1,  Jac_F(1:3, :)*dot_gamma, Jac_F,  dot_Jac_F,  r_F_F_r_B1,   I_hat_B1, I_tilde_B1, dot_gamma);
[T_ITB2, w_B2_B2_w_I, Jac_B2, dot_Jac_B2] = RecursiveFK(T_ITB1, T_B1_T_B2, Jac_F(1:3, :)*dot_gamma, Jac_B1, dot_Jac_B1, r_B1_B1_r_B2, I_hat_B2, I_tilde_B2, dot_gamma);
[T_ITB3, w_B3_B3_w_I, Jac_B3, dot_Jac_B3] = RecursiveFK(T_ITB2, T_B2_T_B3, Jac_F(1:3, :)*dot_gamma, Jac_B2, dot_Jac_B2, r_B2_B2_r_B3, I_hat_B3, I_tilde_B3, dot_gamma);

[T_ITC1, w_C1_C1_w_I, Jac_C1, dot_Jac_C1] = RecursiveFK(T_ITF,  T_F_T_C1,  Jac_F(1:3, :)*dot_gamma, Jac_F,  dot_Jac_F,  r_F_F_r_C1,   I_hat_C1, I_tilde_C1, dot_gamma);
[T_ITC2, w_C2_C2_w_I, Jac_C2, dot_Jac_C2] = RecursiveFK(T_ITC1, T_C1_T_C2, Jac_F(1:3, :)*dot_gamma, Jac_C1, dot_Jac_C1, r_C1_C1_r_C2, I_hat_C2, I_tilde_C2, dot_gamma);
[T_ITC3, w_C3_C3_w_I, Jac_C3, dot_Jac_C3] = RecursiveFK(T_ITC2, T_C2_T_C3, Jac_F(1:3, :)*dot_gamma, Jac_C2, dot_Jac_C2, r_C2_C2_r_C3, I_hat_C3, I_tilde_C3, dot_gamma);

[T_ITD1, w_D1_D1_w_I, Jac_D1, dot_Jac_D1] = RecursiveFK(T_ITF,  T_F_T_D1,  Jac_F(1:3, :)*dot_gamma, Jac_F,  dot_Jac_F,  r_F_F_r_D1,   I_hat_D1, I_tilde_D1, dot_gamma);
[T_ITD2, w_D2_D2_w_I, Jac_D2, dot_Jac_D2] = RecursiveFK(T_ITD1, T_D1_T_D2, Jac_F(1:3, :)*dot_gamma, Jac_D1, dot_Jac_D1, r_D1_D1_r_D2, I_hat_D2, I_tilde_D2, dot_gamma);
[T_ITD3, w_D3_D3_w_I, Jac_D3, dot_Jac_D3] = RecursiveFK(T_ITD2, T_D2_T_D3, Jac_F(1:3, :)*dot_gamma, Jac_D2, dot_Jac_D2, r_D2_D2_r_D3, I_hat_D3, I_tilde_D3, dot_gamma);

%% 
 
H =  Jac_F' * [    J_FFJ,      skew(G_FFG)*T_ITF';      -T_ITF*skew(G_FFG),  m_F*eye(3)] * Jac_F + ...
    Jac_A1' * [J_A1_A1_J, skew(G_A1_A1_G)*T_ITA1'; -T_ITA1*skew(G_A1_A1_G), m_A1*eye(3)] * Jac_A1 +...
    Jac_A2' * [J_A2_A2_J, skew(G_A2_A2_G)*T_ITA2'; -T_ITA2*skew(G_A2_A2_G), m_A2*eye(3)] * Jac_A2 +...
    Jac_A3' * [J_A3_A3_J, skew(G_A3_A3_G)*T_ITA3'; -T_ITA3*skew(G_A3_A3_G), m_A3*eye(3)] * Jac_A3 +...
    Jac_B1' * [J_B1_B1_J, skew(G_B1_B1_G)*T_ITB1'; -T_ITB1*skew(G_B1_B1_G), m_B1*eye(3)] * Jac_B1 +...
    Jac_B2' * [J_B2_B2_J, skew(G_B2_B2_G)*T_ITB2'; -T_ITB2*skew(G_B2_B2_G), m_B2*eye(3)] * Jac_B2 +...
    Jac_B3' * [J_B3_B3_J, skew(G_B3_B3_G)*T_ITB3'; -T_ITB3*skew(G_B3_B3_G), m_B3*eye(3)] * Jac_B3 +...
    Jac_C1' * [J_C1_C1_J, skew(G_C1_C1_G)*T_ITC1'; -T_ITC1*skew(G_C1_C1_G), m_C1*eye(3)] * Jac_C1 +...
    Jac_C2' * [J_C2_C2_J, skew(G_C2_C2_G)*T_ITC2'; -T_ITC2*skew(G_C2_C2_G), m_C2*eye(3)] * Jac_C2 +...
    Jac_C3' * [J_C3_C3_J, skew(G_C3_C3_G)*T_ITC3'; -T_ITC3*skew(G_C3_C3_G), m_C3*eye(3)] * Jac_C3 +...
    Jac_D1' * [J_D1_D1_J, skew(G_D1_D1_G)*T_ITD1'; -T_ITD1*skew(G_D1_D1_G), m_D1*eye(3)] * Jac_D1 +...
    Jac_D2' * [J_D2_D2_J, skew(G_D2_D2_G)*T_ITD2'; -T_ITD2*skew(G_D2_D2_G), m_D2*eye(3)] * Jac_D2 +...
    Jac_D3' * [J_D3_D3_J, skew(G_D3_D3_G)*T_ITD3'; -T_ITD3*skew(G_D3_D3_G), m_D3*eye(3)] * Jac_D3;


d = Jac_F' * [J_FFJ, skew(G_FFG)*T_ITF'; -T_ITF*skew(G_FFG), m_C*eye(3)] * dot_Jac_C*dot_gamma + Jac_C'*[cross(w_CCwI, J_FFJ*w_CCwI); T_ITF*cross(w_CCwI, cross(w_CCwI, G_FFG))] +...
    Jac_B' * [J_BBJ, skew(G_BBG)*T_ITB'; -T_ITB*skew(G_BBG), m_B*eye(3)] * dot_Jac_B*dot_gamma + Jac_B'*[cross(w_BBwI, J_BBJ*w_BBwI); T_ITB*cross(w_BBwI, cross(w_BBwI, G_BBG))] +...
    Jac_1' * [J_11J, skew(G_11G)*T_IT1'; -T_IT1*skew(G_11G), m_1*eye(3)] * dot_Jac_1*dot_gamma + Jac_1'*[cross(w_11wI, J_11J*w_11wI); T_IT1*cross(w_11wI, cross(w_11wI, G_11G))] +...
    Jac_2' * [J_22J, skew(G_22G)*T_IT2'; -T_IT2*skew(G_22G), m_2*eye(3)] * dot_Jac_2*dot_gamma + Jac_2'*[cross(w_22wI, J_22J*w_22wI); T_IT2*cross(w_22wI, cross(w_22wI, G_22G))] +...
    Jac_3' * [J_33J, skew(G_33G)*T_IT3'; -T_IT3*skew(G_33G), m_3*eye(3)] * dot_Jac_3*dot_gamma + Jac_3'*[cross(w_33wI, J_33J*w_33wI); T_IT3*cross(w_33wI, cross(w_33wI, G_33G))] +...
    Jac_4' * [J_44J, skew(G_44G)*T_IT4'; -T_IT4*skew(G_44G), m_4*eye(3)] * dot_Jac_4*dot_gamma + Jac_4'*[cross(w_44wI, J_44J*w_44wI); T_IT4*cross(w_44wI, cross(w_44wI, G_44G))] +...
    Jac_5' * [J_55J, skew(G_55G)*T_IT5'; -T_IT5*skew(G_55G), m_5*eye(3)] * dot_Jac_5*dot_gamma + Jac_5'*[cross(w_55wI, J_55J*w_55wI); T_IT5*cross(w_55wI, cross(w_55wI, G_55G))] +...
    Jac_6' * [J_66J, skew(G_66G)*T_IT6'; -T_IT6*skew(G_66G), m_6*eye(3)] * dot_Jac_6*dot_gamma + Jac_6'*[cross(w_66wI, J_66J*w_66wI); T_IT6*cross(w_66wI, cross(w_66wI, G_66G))] +...
    Jac_7' * [J_77J, skew(G_77G)*T_IT7'; -T_IT7*skew(G_77G), m_7*eye(3)] * dot_Jac_7*dot_gamma + Jac_7'*[cross(w_77wI, J_77J*w_77wI); T_IT7*cross(w_77wI, cross(w_77wI, G_77G))] +...
    Jac_S' * [J_SSJ, skew(G_SSG)*T_ITS'; -T_ITS*skew(G_SSG), m_S*eye(3)] * dot_Jac_S*dot_gamma + Jac_S'*[cross(w_SSwI, J_SSJ*w_SSwI); T_ITS*cross(w_SSwI, cross(w_SSwI, G_SSG))];












end