function skewMat = skew(r)
% skew(r) returns the skew symmetric matrix for an input 3x1 vector r
%
% Inputs:
% r: 3x1 or 1x3 vector
%
% Outputs:
% skewMat: 3x3 skew symmetrix matrix of input vector r
%
% Example:
% skewedMatrix = functionName(positionVector);
% 
% required m-files:
% % None
% Subfunctions:
% % None
% required MAT-files:
% % None
%
% Author: Ian Adelman
% Email: IanAdelman@outlook.com
% Created: 2022
% Revised: 03-12-2023
% Ver#: 2.0
% Version Notes:
% % upadated function header for improved usability and revised code for better legibility.

skewMat = [   0 , -r(3),  r(2); ...
            r(3),    0 , -r(1); ...
           -r(2),  r(1),    0   ];


end