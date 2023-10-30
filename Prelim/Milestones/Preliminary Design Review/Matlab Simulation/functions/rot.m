function DCM = rot(axis, angle)
% rot(axis, angle) calculates the Direction Cosine Matrix representation
% from the input Euler axis and angle representation
%
% Inputs:
% axis: 3x1 unit position vector representing axis of rotation (length)
% angle: angle of rotation (radians)
%
% Outputs:
% DCM: 3x3 Direction Cosine Matrix (radians)
%
% Example:
% axis = [1; 0; 0];
% angle = pi/2;
% DCM = rot(axis, angle)
% 
% Description:
% when given an input axis of rotation, and an angle to rotate through,
% this function will return a 3x3 Direction Cosine Matrix equivalent to
% the input axis angle rotation.
% 
% required m-files:
%   None
%
% Subfunctions:
%   None
%
% required MAT-files:
%   None
%
% Author: Ian Adelman
% Email: IanAdelman@outlook.com
% Created: 2022
% Revised: 03-18-2023
% Ver#: 2.0
% Version Notes:
%   updated function header and improved code readability
%


% Normalize axis of rotation vector before calculations
axis = axis / norm(axis);

% define versin(angle) for more compact code
versin = (1-cos(angle));

% Compute axis-angle rotation matrix
axisAngleRotMat = [ (axis(1)^2)*versin + cos(angle), axis(1)*axis(2) *versin - axis(3)*sin(angle), axis(1)*axis(3)*versin+axis(2)*sin(angle);  ...
                    axis(1)*axis(2)*versin + axis(3)*sin(angle), (axis(2))^2*versin + cos(angle), axis(2)*axis(3)*versin - axis(1)*sin(angle); ...
                    axis(3)*axis(1)*versin - axis(2)*sin(angle), axis(2)*axis(3)*versin + axis(1)*sin(angle), (axis(3)^2)*versin + cos(angle)  ];
                    
end
