function [dot_b, H, d, B, C, G] = Atlas_NewtonEuler(b, F)
% Returns derivative of state for the Atlas System when
% a current state and Torque-Force matrix is supplied.
%
% Inputs:
%   b: state of Atlas system (m, rad, m/s, rad/s)
%   F: Torque-Force matrix    (Nm, N)
%
% Outputs:
%   dot_b : (m/s, rad/s, m/s/s, rad/s/s)
%   H     : System Mass Matrix
%   d     : vector of Coriolis and Centripetal terms
%   B     : Viscous Friction
%   C     : Coulombic Friction
%   G     : Gravitational forces
%
% Example:
%   [dot_state, H, d, B, C, G] = Atlas_NewtonEuler(state, ForceVec);
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
% Previous Ver #: 1.0 : 10/27/2023
%  Current Ver #: 1.5 : 12/07/2023
% Most Recent Version Notes:
%   updated documentation
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
dot_gamma = b(19:36);


% joint positions
[r_IIrF,...
 r_IIrA1, r_IIrA2, r_IIrA3,...
 r_IIrB1, r_IIrB2, r_IIrB3,... 
 r_IIrC1, r_IIrC2, r_IIrC3,...
 r_IIrD1, r_IIrD2, r_IIrD3,...
 T_ITF,...
 T_ITA1, T_ITA2, T_ITA3,...
 T_ITB1, T_ITB2, T_ITB3,...
 T_ITC1, T_ITC2, T_ITC3,...
 T_ITD1, T_ITD2, T_ITD3,...
 ...
 r_F_F_r_A1,r_A1_A1_r_A2,r_A2_A2_r_A3,...
 r_F_F_r_B1,r_B1_B1_r_B2,r_B2_B2_r_B3,...
 r_F_F_r_C1,r_C1_C1_r_C2,r_C2_C2_r_C3,...
 r_F_F_r_D1,r_D1_D1_r_D2,r_D2_D2_r_D3,...
 ...
 T_F_T_A1, T_A1_T_A2, T_A2_T_A3,...
 T_F_T_B1, T_B1_T_B2, T_B2_T_B3,...
 T_F_T_C1, T_C1_T_C2, T_C2_T_C3,...
 T_F_T_D1, T_D1_T_D2, T_D2_T_D3] = Atlas_FK(gamma);

%r_IIrF = [gamma(1); gamma(2); gamma(3)];

%r_F_F_r_A1 = [0.22966150; 0.06751003; -0.05715000];
%r_A1_A1_r_A2 = [0; 0.02794; -0.1016];
%r_A2_A2_r_A3 = [0; 0.0245197; -0.102362];
%
%r_F_F_r_B1 = [0.22966150; -0.06751003; -0.05715000];
%r_B1_B1_r_B2 = [0; -0.02900998; -0.1016];
%r_B2_B2_r_B3 = [0; -0.0245197; -0.102362];
%
%r_F_F_r_C1 = [-0.23388850; 0.06751003; -0.05715000];
%r_C1_C1_r_C2 = [0; 0.02900998; -0.1016];
%r_C2_C2_r_C3 = [0; 0.0245197; -0.102362];
%
%r_F_F_r_D1 = [-0.23388850; -0.06751003; -0.05715000];
%r_D1_D1_r_D2 = [0; -0.02900998; -0.1016];
%r_D2_D2_r_D3 = [0; -0.0245197; -0.102362];


% mass parameters
m_F = 5.02539836; % (kg) [[PLACEHOLDER]]

m_A1 = 0.10938166; % (kg) [[PLACEHOLDER]]
m_A2 = 0.10938166; % (kg)
m_A3 = 0.07942277; % (kg)

m_B1 = m_A1; % (kg)
m_B2 = m_A2; % (kg)
m_B3 = m_A3; % (kg)

m_C1 = m_A1; % (kg)
m_C2 = m_A2; % (kg)
m_C3 = m_A3; % (kg)

m_D1 = m_A1; % (kg)
m_D2 = m_A2; % (kg)
m_D3 = m_A3; % (kg)

% Vectors of first mass moments
G_FFG = [-0.01021315; 0.00016973; -0.04024986]*m_F;

G_A1_A1_G = [0.00005704; 0.03434738; -0.04261051]*m_A1;
G_A2_A2_G = [0.00002378; -0.01143711; 0.07466341]*m_A2;
G_A3_A3_G = [0;          0;           0]*m_A3;

G_B1_B1_G = [-0.00005704; -0.03434738; -0.04261051]*m_A2;
G_B2_B2_G = [-0.00002378; 0.01143711; -0.07466341]*m_B2;
G_B3_B3_G = G_A3_A3_G;

G_C1_C1_G = G_A1_A1_G;
G_C2_C2_G = G_A2_A2_G;
G_C3_C3_G = G_A3_A3_G;

G_D1_D1_G = G_B1_B1_G;
G_D2_D2_G = G_B2_B2_G;
G_D3_D3_G = G_A3_A3_G;





% Moments of inertia (negative tensor form)
J_FFJ = [0.04666550, 0.00003412, 0.00254499;...
         0.00003412, 0.16206870, 0.00004454;...
         0.00254499, 0.00004454, 0.17621226];

J_A1_A1_J = [ 0.00048848, 0.00000013, 0.00000000;...
	         -0.00000013, 0.00035521, 0.00017756;...
	          0.00000000, 0.00017756, 0.00015144];

J_A2_A2_J = [0.00166047,  0.00000018,  0.00000000;...
         	 0.00000018,  0.00157707, -0.00013426;...
	         0.00000000, -0.00013426,  0.00011571];

J_A3_A3_J = [0.00003211, 0.00000000, 0.00000000;...
	         0.00000000, 0.00005967, 0.00000000;...
	         0.00000000, 0.00000000, 0.00003211];

J_B1_B1_J = [ 0.00048848, -0.00000013,  0.00000000;...
             -0.00000013,  0.00035521, -0.00017756;...
              0.00000000, -0.00017756,  0.00015144];

J_B2_B2_J = [0.00166047, 0.00000018, 0.00000000;...
	         0.00000018, 0.00157707, 0.00013426;...
	         0.00000000, 0.00013426, 0.00011571];

J_B3_B3_J = J_A3_A3_J;
	         
	         

J_C1_C1_J = J_A1_A1_J;
J_C2_C2_J = J_A2_A2_J;
J_C3_C3_J = J_A3_A3_J;

J_D1_D1_J = J_B1_B1_J;
J_D2_D2_J = J_B2_B2_J;
J_D3_D3_J = J_B3_B3_J;



% jacobian for Frame frame
xyz_KDE = [cos(phi)*cos(theta), sin(phi), 0; -cos(theta)*sin(phi),...
    cos(phi), 0; sin(theta), 0, 1];

Jac_F = [zeros(3, 3),     xyz_KDE, zeros(3, 12);...
           eye(3, 3), zeros(3, 3), zeros(3, 12)];
    

dot_Jac_F = zeros(6, size(b, 1)/2);
dot_Jac_F(1, 4) = -sin(psi)*dot_psi*cos(theta) -...
    sin(theta)*dot_theta*cos(psi);
dot_Jac_F(1, 5) = dot_psi*cos(psi);
dot_Jac_F(2, 4) = sin(psi)*sin(theta)*dot_theta -...
    cos(psi)*cos(theta)*dot_psi;
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
I_hat_A1(2, 7) = 1;
I_hat_A2(2, 8) = 1;
I_hat_A3(2, 9) = 1;

I_hat_B1(2, 10) = 1;
I_hat_B2(2, 11) = 1;
I_hat_B3(2, 12) = 1;

I_hat_C1(2, 13) = 1;
I_hat_C2(2, 14) = 1;
I_hat_C3(2, 15) = 1;

I_hat_D1(2, 16) = 1;
I_hat_D2(2, 17) = 1;
I_hat_D3(2, 18) = 1;



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


%% Recursive FK
w_FFwI = [dot_phi; dot_theta; dot_psi];
[T_ITA1, w_A1_A1_w_I, Jac_A1, dot_Jac_A1] = ...
RecursiveFK(T_ITF,  T_F_T_A1,  Jac_F(1:3, :)*dot_gamma, Jac_F,  ...
dot_Jac_F,  r_F_F_r_A1,   I_hat_A1, I_tilde_A1, dot_gamma);

[T_ITA2, w_A2_A2_w_I, Jac_A2, dot_Jac_A2] = ...
RecursiveFK(T_ITA1, T_A1_T_A2, Jac_F(1:3, :)*dot_gamma, Jac_A1, ...
dot_Jac_A1, r_A1_A1_r_A2, I_hat_A2, I_tilde_A2, dot_gamma);

[T_ITA3, w_A3_A3_w_I, Jac_A3, dot_Jac_A3] = ...
RecursiveFK(T_ITA2, T_A2_T_A3, Jac_F(1:3, :)*dot_gamma, Jac_A2, ...
dot_Jac_A2, r_A2_A2_r_A3, I_hat_A3, I_tilde_A3, dot_gamma);


[T_ITB1, w_B1_B1_w_I, Jac_B1, dot_Jac_B1] = ...
RecursiveFK(T_ITF,  T_F_T_B1,  Jac_F(1:3, :)*dot_gamma, Jac_F,...
dot_Jac_F,  r_F_F_r_B1,   I_hat_B1, I_tilde_B1, dot_gamma);

[T_ITB2, w_B2_B2_w_I, Jac_B2, dot_Jac_B2] = ...
RecursiveFK(T_ITB1, T_B1_T_B2, Jac_F(1:3, :)*dot_gamma, Jac_B1,...
dot_Jac_B1, r_B1_B1_r_B2, I_hat_B2, I_tilde_B2, dot_gamma);

[T_ITB3, w_B3_B3_w_I, Jac_B3, dot_Jac_B3] = ...
RecursiveFK(T_ITB2, T_B2_T_B3, Jac_F(1:3, :)*dot_gamma, Jac_B2,...
dot_Jac_B2, r_B2_B2_r_B3, I_hat_B3, I_tilde_B3, dot_gamma);


[T_ITC1, w_C1_C1_w_I, Jac_C1, dot_Jac_C1] = ...
RecursiveFK(T_ITF,  T_F_T_C1,  Jac_F(1:3, :)*dot_gamma, Jac_F,...
dot_Jac_F,  r_F_F_r_C1,   I_hat_C1, I_tilde_C1, dot_gamma);

[T_ITC2, w_C2_C2_w_I, Jac_C2, dot_Jac_C2] = ...
RecursiveFK(T_ITC1, T_C1_T_C2, Jac_F(1:3, :)*dot_gamma, Jac_C1,...
dot_Jac_C1, r_C1_C1_r_C2, I_hat_C2, I_tilde_C2, dot_gamma);

[T_ITC3, w_C3_C3_w_I, Jac_C3, dot_Jac_C3] = ...
RecursiveFK(T_ITC2, T_C2_T_C3, Jac_F(1:3, :)*dot_gamma, Jac_C2,...
dot_Jac_C2, r_C2_C2_r_C3, I_hat_C3, I_tilde_C3, dot_gamma);


[T_ITD1, w_D1_D1_w_I, Jac_D1, dot_Jac_D1] = ...
RecursiveFK(T_ITF,  T_F_T_D1,  Jac_F(1:3, :)*dot_gamma, Jac_F,...
dot_Jac_F,  r_F_F_r_D1,   I_hat_D1, I_tilde_D1, dot_gamma);

[T_ITD2, w_D2_D2_w_I, Jac_D2, dot_Jac_D2] = ...
RecursiveFK(T_ITD1, T_D1_T_D2, Jac_F(1:3, :)*dot_gamma, Jac_D1,...
dot_Jac_D1, r_D1_D1_r_D2, I_hat_D2, I_tilde_D2, dot_gamma);

[T_ITD3, w_D3_D3_w_I, Jac_D3, dot_Jac_D3] = ...
RecursiveFK(T_ITD2, T_D2_T_D3, Jac_F(1:3, :)*dot_gamma, Jac_D2,...
dot_Jac_D2, r_D2_D2_r_D3, I_hat_D3, I_tilde_D3, dot_gamma);


%% Ground Colision Detection
r_IIrA1 = r_IIrF + (T_ITF * r_F_F_r_A1);
r_IIrA2 = r_IIrA1 + (T_ITA1 * r_A1_A1_r_A2);
r_IIrA3 = r_IIrA2 + (T_ITA2 * r_A2_A2_r_A3);

r_IIrB1 = r_IIrF + (T_ITF * r_F_F_r_B1);
r_IIrB2 = r_IIrB1 + (T_ITB1 * r_B1_B1_r_B2);
r_IIrB3 = r_IIrB2 + (T_ITB2 * r_B2_B2_r_B3);

r_IIrC1 = r_IIrF + (T_ITF * r_F_F_r_C1);
r_IIrC2 = r_IIrC1 + (T_ITC1 * r_C1_C1_r_C2);
r_IIrC3 = r_IIrC2 + (T_ITC2 * r_C2_C2_r_C3);

r_IIrD1 = r_IIrF + (T_ITF * r_F_F_r_D1);
r_IIrD2 = r_IIrD1 + (T_ITD1 * r_D1_D1_r_D2);
r_IIrD3 = r_IIrD2 + (T_ITD2 * r_D2_D2_r_D3);


r = 0.03757378; % wheel radius (m)
% vector of z positions of wheels, corrected for wheel radius

% pre-allocate F_ground_penetration values
dot_r_IIrF = [dot_delta_x; dot_delta_y; dot_delta_z];
F_A = 0;
F_B = 0;
F_C = 0;
F_D = 0;



if  r_IIrA3(3)-r < 0
    F_A = ground_penetration_force(dot_r_IIrF, r_IIrA3, r_F_F_r_A1, r_A1_A1_r_A2, r_A2_A2_r_A3, T_ITF, T_ITA1, T_ITA2, T_ITA3, [phi; theta; psi], [0; dot_theta_A1; 0], [0; dot_theta_A2; 0],  r, theta_A3, Jac_A3);
end
 
if r_IIrB3(3)-r < 0
    F_B = ground_penetration_force(dot_r_IIrF, r_IIrB3, r_F_F_r_B1, r_B1_B1_r_B2, r_B2_B2_r_B3, T_ITF, T_ITB1, T_ITB2, T_ITB3, [phi; theta; psi], [0; dot_theta_B1; 0], [0; dot_theta_B2; 0],  r, theta_B3, Jac_B3);
end

if r_IIrC3(3)-r < 0
    F_C = ground_penetration_force(dot_r_IIrF, r_IIrC3, r_F_F_r_C1, r_C1_C1_r_C2, r_C2_C2_r_C3, T_ITF, T_ITC1, T_ITC2, T_ITC3, [phi; theta; psi], [0; dot_theta_C1; 0], [0; dot_theta_C2; 0],  r, theta_C3, Jac_C3);
end

if r_IIrD3(3)-r < 0
    F_D = ground_penetration_force(dot_r_IIrF, r_IIrD3, r_F_F_r_D1, r_D1_D1_r_D2, r_D2_D2_r_D3, T_ITF, T_ITD1, T_ITD2, T_ITD3, [phi; theta; psi], [0; dot_theta_D1; 0], [0; dot_theta_D2; 0],  r, theta_D3, Jac_D3);
end


F_ground_collision = F_A + F_B + F_C + F_D;
    
    
    
 



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


d = Jac_F' * [J_FFJ, skew(G_FFG)*T_ITF';...
    -T_ITF*skew(G_FFG), m_F*eye(3)] * dot_Jac_F*dot_gamma...
    + Jac_F'*[cross(w_FFwI, J_FFJ*w_FFwI);....
    T_ITF*cross(w_FFwI, cross(w_FFwI, G_FFG))] +...
    Jac_A1' * [J_A1_A1_J, skew(G_A1_A1_G)*T_ITA1';...
 -T_ITA1*skew(G_A1_A1_G), m_A1*eye(3)] * dot_Jac_A1*dot_gamma...
 + Jac_A1'*[cross(w_A1_A1_w_I, J_A1_A1_J*w_A1_A1_w_I);...
 T_ITA1*cross(w_A1_A1_w_I, cross(w_A1_A1_w_I, G_A1_A1_G))] +...
    Jac_A2' * [J_A2_A2_J, skew(G_A2_A2_G)*T_ITA2';...
 -T_ITA2*skew(G_A2_A2_G), m_A2*eye(3)] * dot_Jac_A2*dot_gamma...
 + Jac_A2'*[cross(w_A2_A2_w_I, J_A2_A2_J*w_A2_A2_w_I);...
 T_ITA2*cross(w_A2_A2_w_I, cross(w_A2_A2_w_I, G_A2_A2_G))] +...
    Jac_A3' * [J_A3_A3_J, skew(G_A3_A3_G)*T_ITA3';...
 -T_ITA3*skew(G_A3_A3_G), m_A3*eye(3)] * dot_Jac_A3*dot_gamma...
 + Jac_A3'*[cross(w_A3_A3_w_I, J_A3_A3_J*w_A3_A3_w_I);...
 T_ITA3*cross(w_A3_A3_w_I, cross(w_A3_A3_w_I, G_A3_A3_G))] +...
    Jac_B1' * [J_B1_B1_J, skew(G_B1_B1_G)*T_ITB1';...
 -T_ITB1*skew(G_B1_B1_G), m_B1*eye(3)] * dot_Jac_B1*dot_gamma...
 + Jac_B1'*[cross(w_B1_B1_w_I, J_B1_B1_J*w_B1_B1_w_I);...
 T_ITB1*cross(w_B1_B1_w_I, cross(w_B1_B1_w_I, G_B1_B1_G))] +...
    Jac_B2' * [J_B2_B2_J, skew(G_B2_B2_G)*T_ITB2';...
 -T_ITB2*skew(G_B2_B2_G), m_B2*eye(3)] * dot_Jac_B2*dot_gamma...
 + Jac_B2'*[cross(w_B2_B2_w_I, J_B2_B2_J*w_B2_B2_w_I);...
 T_ITB2*cross(w_B2_B2_w_I, cross(w_B2_B2_w_I, G_B2_B2_G))] +...
    Jac_B3' * [J_B3_B3_J, skew(G_B3_B3_G)*T_ITB3';...
 -T_ITB3*skew(G_B3_B3_G), m_B3*eye(3)] * dot_Jac_B3*dot_gamma...
 + Jac_B3'*[cross(w_B3_B3_w_I, J_B3_B3_J*w_B3_B3_w_I);...
 T_ITB3*cross(w_B3_B3_w_I, cross(w_B3_B3_w_I, G_B3_B3_G))] +...
    Jac_C1' * [J_C1_C1_J, skew(G_C1_C1_G)*T_ITC1';...
 -T_ITC1*skew(G_C1_C1_G), m_C1*eye(3)] * dot_Jac_C1*dot_gamma...
 + Jac_C1'*[cross(w_C1_C1_w_I, J_C1_C1_J*w_C1_C1_w_I);...
 T_ITC1*cross(w_C1_C1_w_I, cross(w_C1_C1_w_I, G_C1_C1_G))] +...
    Jac_C2' * [J_C2_C2_J, skew(G_C2_C2_G)*T_ITC2';...
 -T_ITC2*skew(G_C2_C2_G), m_C2*eye(3)] * dot_Jac_C2*dot_gamma...
 + Jac_C2'*[cross(w_C2_C2_w_I, J_C2_C2_J*w_C2_C2_w_I);...
 T_ITC2*cross(w_C2_C2_w_I, cross(w_C2_C2_w_I, G_C2_C2_G))] +...
    Jac_C3' * [J_C3_C3_J, skew(G_C3_C3_G)*T_ITC3';...
 -T_ITC3*skew(G_C3_C3_G), m_C3*eye(3)] * dot_Jac_C3*dot_gamma...
 + Jac_C3'*[cross(w_C3_C3_w_I, J_C3_C3_J*w_C3_C3_w_I);...
 T_ITC3*cross(w_C3_C3_w_I, cross(w_C3_C3_w_I, G_C3_C3_G))] +...
    Jac_D1' * [J_D1_D1_J, skew(G_D1_D1_G)*T_ITD1';...
 -T_ITD1*skew(G_D1_D1_G), m_D1*eye(3)] * dot_Jac_D1*dot_gamma...
 + Jac_D1'*[cross(w_D1_D1_w_I, J_D1_D1_J*w_D1_D1_w_I);...
 T_ITD1*cross(w_D1_D1_w_I, cross(w_D1_D1_w_I, G_D1_D1_G))] +...
    Jac_D2' * [J_D2_D2_J, skew(G_D2_D2_G)*T_ITD2';...
 -T_ITD2*skew(G_D2_D2_G), m_D2*eye(3)] * dot_Jac_D2*dot_gamma...
 + Jac_D2'*[cross(w_D2_D2_w_I, J_D2_D2_J*w_D2_D2_w_I);...
 T_ITD2*cross(w_D2_D2_w_I, cross(w_D2_D2_w_I, G_D2_D2_G))] +...
    Jac_D3' * [J_D3_D3_J, skew(G_D3_D3_G)*T_ITD3';...
 -T_ITD3*skew(G_D3_D3_G), m_D3*eye(3)] * dot_Jac_D3*dot_gamma...
 + Jac_D3'*[cross(w_D3_D3_w_I, J_D3_D3_J*w_D3_D3_w_I);...
 T_ITD3*cross(w_D3_D3_w_I, cross(w_D3_D3_w_I, G_D3_D3_G))];


% define gravitational torque for each link
g = [0; 0; -9.81]; % grav vector, (m/s/s)

% get G vector, using equation 34 and summing across all bodies.
G = Jac_F.' * [cross(G_FFG, T_ITF.'*g); m_F*g] +...
    Jac_A1.' * [cross(G_A1_A1_G, T_ITA1.'*g); m_A1*g] +...
    Jac_A2.' * [cross(G_A2_A2_G, T_ITA2.'*g); m_A2*g] +...
    Jac_A3.' * [cross(G_A3_A3_G, T_ITA3.'*g); m_A3*g] +...
    Jac_B1.' * [cross(G_B1_B1_G, T_ITB1.'*g); m_B1*g] +...
    Jac_B2.' * [cross(G_B2_B2_G, T_ITB2.'*g); m_B2*g] +...
    Jac_B3.' * [cross(G_B3_B3_G, T_ITB3.'*g); m_B3*g] +...
    Jac_C1.' * [cross(G_C1_C1_G, T_ITC1.'*g); m_C1*g] +...
    Jac_C2.' * [cross(G_C2_C2_G, T_ITC2.'*g); m_C2*g] +...
    Jac_C3.' * [cross(G_C3_C3_G, T_ITC3.'*g); m_C3*g] +...
    Jac_D1.' * [cross(G_D1_D1_G, T_ITD1.'*g); m_D1*g] +...
    Jac_D2.' * [cross(G_D2_D2_G, T_ITD2.'*g); m_D2*g] +...
    Jac_D3.' * [cross(G_D3_D3_G, T_ITD3.'*g); m_D3*g];

%% Friction
B = zeros(size(b, 1)/2, 1);
B(1) =  0; % no friction on free orientation angles
B(2) =  0; % no friction on free orientation angles
B(3) =  0; % no friction on free orientation angles
B(4) =  0; % no friction on free orientation angles
B(5) =  0; % no friction on free orientation angles
B(6) =  0; % no friction on free orientation angles
B(7) =  .1; % Leg A angle 1
B(8) =  .1; % Leg A angle 2
B(9) =  .1; % Leg A angle 3
B(10) = .1;  % Leg B angle 1
B(11) = .1;  % Leg B angle 2
B(12) = .1;  % Leg B angle 3
B(13) = .1;  % Leg C angle 1
B(14) = .1;  % Leg C angle 2
B(15) = .1;  % Leg C angle 3
B(16) = .1;  % Leg D angle 1
B(17) = .1;  % Leg D angle 2
B(18) = .1;  % Leg D angle 3
B = diag(B);


C = zeros(size(b, 1)/2, 1);
C(1) = 0; % no friction on free orientation angles
C(2) = 0; % no friction on free orientation angles
C(3) = 0; % no friction on free orientation angles
C(4) = 0; % no friction on free orientation angles
C(5) = 0; % no friction on free orientation angles
C(6) = 0; % no friction on free orientation angles
C(7) = 0; % Leg A angle 1
C(8) = 0; % Leg A angle 2
C(9) = 0; % Leg A angle 3
C(10) = 0;  % Leg B angle 1
C(11) = 0;  % Leg B angle 2
C(12) = 0;  % Leg B angle 3
C(13) = 0;  % Leg C angle 1
C(14) = 0;  % Leg C angle 2
C(15) = 0;  % Leg C angle 3
C(16) = 0;  % Leg D angle 1
C(17) = 0;  % Leg D angle 2
C(18) = 0;  % Leg D angle 3
C = diag(C);




%% calculate derivative of state for output
% ddot_gamma = H \ (F - d + G + F_ground_collision - B*dot_gamma - C*sign(dot_gamma));
ddot_gamma = H \ (F - d + G + F_ground_collision);

dot_b = [dot_gamma; ddot_gamma];




end



