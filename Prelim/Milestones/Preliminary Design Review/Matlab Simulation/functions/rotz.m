function dcm = rotz(psi)
% rotz(angle) computes the DCM equivalent to the input angle,
% the axis is set to the Z axis
%
% Inputs:
% angle: angle to rotate through (radians)
%
% Outputs:
% dcm: 3x3 Direction Cosine Matrix (radians)
%
% Example:
% angle = pi/2;
% dcm = rotz(angle);
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

dcm = [cos(psi) , -sin(psi) , 0  ;...
       sin(psi) , cos(psi)  , 0  ;...
       0       , 0        , 1 ];

end
