function [a_0, a_1, a_2, a_3] = cubicSpline(t_1, t_2, Xt_1, Xt_2, dotXt_1, dotXt_2)
% cubicSpline will output the 4 coefficients of a cubic spline connecting two points
%
% Inputs:
% t_1: time value for the start of the spline     (time)
% t_2: time value for the end of the spline       (time)
% Xt_1: position or position vector for time 1    (position)
% Xt_2: position or position vector for time 2    (position)
% dotXt_1: velocity or velocity vector for time 1 (position/time)
% dotXt_2: velocity or velocity vector for time 1 (position/time)
%
% Outputs:
% a_0: coeffient 1 for cubic spline polinomial equation
% a_1: coeffient 2 for cubic spline polinomial equation
% a_2: coeffient 3 for cubic spline polinomial equation
% a_3: coeffient 4 for cubic spline polinomial equation
% polynomial equation form: a_0 + a_1*t + a_2*t^2 + a_3*t^3
%
% Example:
% [a_0, a_1, a_2, a_3] = cubicSpline(time(1), time(2), pos(1), pos(2), vel(1), vel(2))
% 
% Description:
% % given two values of time, vectors for position of time 1 and time 2,
% % and velocity vectors for time 1 and time 2. This function will output 4
% % scalar coefficients a_0 through a_3. These coefficients create the equation
% % for a cubic polynomial which "smoothly" connects the two given points.
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
% % added better function header with description, and better code comments with nicer formatting.
%

% initialize I matrix for matrix math use
I = eye(length(Xt_1));
% capture length of velocity vector for use in matrix math
n = length(Xt_1);

% if time starts at zero, calculate coeffs this way
if (t_1 == 0)
    % calculate first coefficient
    a_0 = Xt_1;
    % calculate second coefficient
    a_1 = dotXt_1;
    % calculate "A" matrix for b = A\x matrix math
    A = [I*t_2^2, I*t_2^3; 2*I*t_2, 3*I*t_2^2];
    % calculate X vector for matrix math
    Xvec = [Xt_2 - a_0 - a_1*t_2; dotXt_2 - a_1];
    % calculate third and fourth coefficient
    temp = A\Xvec;
    a_2 = temp(1:n);
    a_3 = temp(n+1:2*n);

% if the second time point is zero, calculate coeffs this way
elseif (t_2 == 0)
    % calulate first and second coeffs similar to first block, but using t_2 values instead of t_1
    a_0 = Xt_2;
    a_1 = dotXt_2;

    % calculate A and X vector
    A = [I*t_2^2, I*t_1^3; 2*I*t_1, 3*I*t_1^2];
    Xvec = [Xt_1 - a_0 - a_1*t_1; dotXt_1 - a_1];

    % calculate third and fourth coefficient
    temp = A\Xvec;
    a_2 = temp(1:n);
    a_3 = temp(n+1:2*n);

% if neither t_1 or t_2 is zero, calculate coeffs this way
else
    % set up matricies for matrix math
    Xvec = [Xt_1; Xt_2; dotXt_1; dotXt_2];
    A = [I, I*t_1, I*t_1^2, I*t_1^3; ...
        I, I*t_2, I*t_2^2, I*t_2^3; ...
        0*I, I, 2*I*t_1, 3*I*t_1^2; ...
        0*I, I, 2*I*t_2, 3*I*t_2^2];

    % do matrix math
    sol = A\Xvec;

    % pull each coefficient from solution matrix
    a_0 = sol(1:n);
    a_1 = sol(n + 1:2*n);
    a_2 = sol(2*n + 1:3*n);
    a_3 = sol(3*n + 1:4*n);
end


end