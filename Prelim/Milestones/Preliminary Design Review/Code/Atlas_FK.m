function [r_IIrF,...
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
 T_F_T_D1, T_D1_T_D2, T_D2_T_D3] = Atlas_FK(gamma)
% this function will calculate the forward kinematics of the
% Atlas system, and return many parameters of the system for
% use in other functions and scripts
%
% Inputs:
%   gamma : 18x1 vector of joint angles, each of which is defined below
%
% Outputs:
%   any prefix r_ : position vector
%   any prefix T_ : Direction Cosine Matrix
%
% Example:
%   [~, vec1, ~, vecC] = Atlas_FK(zeros(18, 1));
% 
% Description:
%   using vector addition and direction cosine matrices, this function
%   will calculate the full positinal state of the Atlas sytem and return
%   it as many individual variables which are usable by whichever function
%   or script may need some ammount of forward kinematic calculation.
%
% required m-files:
%   rotx.m
%     for DCM rotation about a given x axis through an angle (radians)
%   roty.m
%     for DCM rotation about a given y axis through an angle (radians)
%   rotz.m
%     for DCM rotation about a given z axis through an angle (radians)
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
% Created: 11/03/2023
% Revised: 11/29/2023
%
% Previous Ver #: 1.0 : 04/18/2023
%  Current Ver #: 1.1 : 04/21/2023
% Most Recent Version Notes:
%   updated function header and made code more readable
%


% joint positions
% translational position of main body
delta_x = gamma(1);
delta_y = gamma(2);
delta_z = gamma(3);
r_IIrF = [delta_x; delta_y; delta_z];


r_F_F_r_A1 = [0.27714149; 0.04295; -0.06477];
r_A1_A1_r_A2 = [0; 0.02794; -0.1016];
r_A2_A2_r_A3 = [0; 0.0357237; -0.127762];

r_F_F_r_B1 = [0.27714149; -0.04295; -0.06477];
r_B1_B1_r_B2 = [0; -0.02794; -0.1016];
r_B2_B2_r_B3 = [0; -0.0357237; -0.127762];

r_F_F_r_C1 = [-0.28165851; 0.04041; -0.06477];
r_C1_C1_r_C2 = [0; 0.02794; -0.1016];
r_C2_C2_r_C3 = [0; 0.0357237; -0.127762];

r_F_F_r_D1 = [-0.28165851; -0.04041; -0.06477];
r_D1_D1_r_D2 = [0; -0.02794; -0.1016];
r_D2_D2_r_D3 = [0; -0.0357237; -0.127762];

% pose angles of main body
phi = gamma(4);   % rotation about the forward (x) axis
theta = gamma(5); % rotation about the y axis
psi = gamma(6);   % rotation about the z axis
% front left leg
theta_A1 = gamma(7);
theta_A2 = gamma(8);
theta_A3 = gamma(9);
% front right leg
theta_B1 = gamma(10);
theta_B2 = gamma(11);
theta_B3 = gamma(12);
% back left leg
theta_C1 = gamma(13);
theta_C2 = gamma(14);
theta_C3 = gamma(15);
% back right leg
theta_D1 = gamma(16);
theta_D2 = gamma(17);
theta_D3 = gamma(18);


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


% transform DCMs back to I frame
T_ITA1 = T_ITF * T_F_T_A1;
T_ITA2 = T_ITA1 * T_A1_T_A2;
T_ITA3 = T_ITA2 * T_A2_T_A3;

T_ITB1 = T_ITF * T_F_T_B1;
T_ITB2 = T_ITB1 * T_B1_T_B2;
T_ITB3 = T_ITB2 * T_B2_T_B3;

T_ITC1 = T_ITF * T_F_T_C1;
T_ITC2 = T_ITC1 * T_C1_T_C2;
T_ITC3 = T_ITC2 * T_C2_T_C3;

T_ITD1 = T_ITF * T_F_T_D1;
T_ITD2 = T_ITD1 * T_D1_T_D2;
T_ITD3 = T_ITD2 * T_D2_T_D3;


% Transform Position vectors into I frame
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
end

