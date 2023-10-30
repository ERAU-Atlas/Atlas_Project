function dcm = rotq(q)
%rotq - Convert a quaternion to a direction cosine matrix (DCM)
%
%  Author: Ian Adelman
%  Creation Date: 
%  Modification Date: 2023-02-11
%  Version: 2.0
%  Maintainer: Ian Adelman
%  Maintainer Email: Adelmani@my.erau.edu
%
%  Inputs:
%    q - a 4x1 quaternion, where q = [q0; q1; q2; q3]
%
%  Outputs:
%    dcm - a 3x3 direction cosine matrix
%
%  Functionality:
%    This function converts a quaternion to a direction cosine matrix
%    (DCM). The DCM is a 3x3 rotation matrix that describes the orientation
%    of an object in 3D space.
%
%  Notes:
%    The index of the quaternion in Matlab starts at 1, so q0 becomes q(1).
%
%  

% index starts at 1 in matlab so q0 becomes q(1)
dcm = [...
    2*q(1)^2-1+2*q(2)^2,2*q(2)*q(3)-2*q(1)*q(4),2*q(2)*q(4)+2*q(1)*q(3);...
    2*q(2)*q(3)+2*q(1)*q(4),2*q(1)^2-1+2*q(3)^2,2*q(3)*q(4)-2*q(1)*q(2);...
    2*q(2)*q(4)-2*q(1)*q(3),2*q(3)*q(4)+2*q(1)*q(2),2*q(1)^2-1+2*q(4)^2];

end
