function [unit_quat] = unitQuat(angle, unitVec)
%unitQuat Computes a unit quaternion from an angle and a unit vector
%
% Syntax:
% [unit_quat] = unitQuat(angle, unitVec)
%
% Inputs:
% angle: angle in radians
% unitVec: unit vector, a 3x1 row vector
%
% Outputs:
% unit_quat: unit quaternion, a 4x1 vector
% [cos(angle/2); sin(angle/2)*unitVec]
%
% Example:
% [unit_quat] = unitQuat(pi/4, [1; 0; 0])
%
% Other m-files required: None
% Subfunctions: None
% MAT-files required: None
%
% Author: Ian Adelman
% Email: IanAdelman@outlook.com
% Created: forgot
% Revised: 02-11-2023

% Ensure that unitVec is a 3x1 row vector
if length(unitVec) ~= 3
error("unit vector must be a row vector of length 3");
end

% Normalize the unit vector
unitVec = unitVec./norm(unitVec);

% Calculate the unit quaternion
unit_quat = [cos(angle/2); sin(angle/2)*unitVec];

end