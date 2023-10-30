function qAB = quatmult(qA, qB)
% quatmult(qA, qB) multiplies two unit quaternions together
% to create a unit quaternion which contains the rotations
% of both unit quaternions
%
% Inputs:
% qA: unit quaternion (radians)
% qB: unit quaternion (radians)
%
% Outputs:
% qAB: combined unit quaternion (radians)
%
% Example:
% q_1 = [1; pi/2; pi/2; pi/2];
% q_2 = [1; pi/2; pi/2; pi/2];
% q_12 = quatmult(q_1, q_2);
%
% required m-files:
% % None
%
% Subfunctions:
% % None
%
% required MAT-files:
% % None
%
% Author: Ian Adelman
% Email: IanAdelman@outlook.com
% Created: 2022
% Revised: 03-18-2023
% Ver#: 2.0
% Version Notes:
% % updated function header, increased code readability
%

qAB = [...
    qA(1) , -qA(2) , -qA(3) , -qA(4) ;...
    qA(2) ,  qA(1) , -qA(4) ,  qA(3) ;...
    qA(3) ,  qA(4) ,  qA(1) , -qA(2) ;...
    qA(4) , -qA(3) ,  qA(2) ,  qA(1) ]...
    ...
    *qB;
end