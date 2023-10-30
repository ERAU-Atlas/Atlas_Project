function dcm = roty(theta)
% roty(angle) computes the DCM equivalent to the input angle,
% the axis is set to the Y axis
%
% Inputs:
% angle: angle to rotate through (radians)
%
% Outputs:
% dcm: 3x3 Direction Cosine Matrix (radians)
%
% Example:
% angle = pi/2;
% dcm = roty(angle);
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
%   updated function header, improved code readability
%

dcm = [cos(theta) , 0, sin(theta)  ;...
       0          , 1, 0           ;...
       -sin(theta), 0, cos(theta) ];

end
