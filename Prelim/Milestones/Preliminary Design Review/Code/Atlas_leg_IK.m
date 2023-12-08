function [theta_1_A, theta_2_A, theta_1_B, theta_2_B] = ...
    Atlas_leg_IK(r_F_F_r_3, r_F_F_r_1, l_1, l_2)
% [short summary and purpose]
%
% Inputs:
%   r_F_F_r_3 : position vector from Frame to wheel with respect to Frame
%   r_F_F_r_1 : position vector from Frame to leg joint 1 with respect to
%               Frame
%   l_1       : length of leg segment 1
%   l_2       : length of leg segment 2
%
% Outputs:
%   theta_1_A : leg angle 1 for solution A
%   theta_2_A : leg angle 2 for solution A
%   theta_1_B : leg angle 1 for solution B
%   theta_2_B : leg angle 2 for solution B
%
% Example:
%   [t_1A, t_2A, t_1B, t_2B] = Atlas_leg_IK([.1;-.1;0], [.1;.1;0], .1, .1);
% 
% Description:
%   this function will return the two solutions for the planar-two-link
%   inverse Kinematics solution. An 'elbow up' and 'elbow down' solution.
%
% required m-files:
%   TwoLink_IK.m
%     for the Planar-Two-Link Inverse kinematic solution.
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
% Previous Ver #: 1.0 : 11/22/2023
%  Current Ver #: 1.2 : 12/02/2023
% Most Recent Version Notes:
%   angles limited from 0 to 2pi, updated format.
%

% vector from shoulder to foot in F frame
r_F_1_r_3 = r_F_F_r_3 - r_F_F_r_1;

% vector from shoulder to foot in Two_Link Solution frame
r_S_1_r_3 = rotz(-pi/2).' * (r_F_1_r_3);

% calculate two-link IK
[theta_1_A_S, theta_2_A_S, theta_1_B_S, theta_2_B_S] = ...
    TwoLink_IK(r_S_1_r_3(2), r_S_1_r_3(3), l_1, l_2);

% re-organize directions to suit Atlas convention
theta_1_A = -1 * theta_1_A_S;
theta_2_A = -1 * theta_2_A_S;

theta_1_B = -1 * theta_1_B_S;
theta_2_B = -1 * theta_2_B_S;


% map all angles from zero to 2pi
theta_1_A = angle(exp(1i * theta_1_A));
theta_2_A = angle(exp(1i * theta_2_A));
theta_1_B = angle(exp(1i * theta_1_B));
theta_2_B = angle(exp(1i * theta_2_B));

end
