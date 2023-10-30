function [q] = qrotx(angle)
% [short summary and purpose]
%
% Inputs:
% [input01: description (units)]
%
% Outputs:
% [output01: description (units)]
%
% Example:
% [out01, out02] = functionName(in01, in02);
% 
% Description:
% [Extended Summary and description if necessary]
%
% required m-files:
%   [ required m-file 1]
%     [ short explanation/use of required m-file 1]
%
% Subfunctions:
%   [Subfunction01]
%     [ short description of Subfunction01]
%
% required MAT-files:
%   [ required MAT-file 1 ]
%     [ short explanation/use of required MAT-file 1]
%
% Author: Ian Adelman
% Email: IanAdelman@outlook.com
% Created: 04/01/2023
% Revised: revisedDate
% Ver#: 1.0
% Version Notes:
% % [[notes about changes since previous versions or important info]
%

q = [cos(angle/2); sin(angle/2); 0; 0];

end