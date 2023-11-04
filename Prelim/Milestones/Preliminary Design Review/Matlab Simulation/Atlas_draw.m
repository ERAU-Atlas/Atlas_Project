function [] = Atlas_draw(gamma, fig, frameNum)
% [short summary and purpose]
%
% Inputs:
%   [input01: description (units)]
%
% Outputs:
%   [output01: description (units)]
%
% Example:
%   [out01, out02] = functionName(in01, in02);
% 
% Description:
%   [Extended Summary and description if necessary]
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
% Author: [ Author(s) Name ]
% Email: [ Email ]
% Maintainre: [ Maintainer Name ]
% Maintainer Email: [ Email ]
% Created: createdDate
% Revised: revisedDate
%
% Previous Ver #: 1.0 : 04/18/2023
%  Current Ver #: 1.1 : 04/21/2023
% Most Recent Version Notes:
%   [[notes about changes since previous versions or important info]
%


% make STL geometry persistant as to not waste load times when animating
persistent dat
persistent p



% if frameNum is 1, meaning this is the first time this function is
% called, set up the figure, read the STL data, and create the patches
if frameNum < 2
    % set up figure
    fig = gcf;             % create figure obj
    xlabel('x (m)')        % label x axis
    ylabel('y (m)')        % label y axis
    zlabel('z (m)')        % label z axis
    view([1,1,1])          % isometric view
    axis equal             % set axes to be equal for better visuals
 
    lt = camlight('left'); % make light obj
    

    % Read in STL data for all joints
    [dat.JF.verts, dat.JF.faces, dat.JF.n, dat.JF.c, dat.JF.stltitle] = stlread("Atlas_Frame.STL");
    [dat.JA1.verts, dat.JA1.faces, dat.JA1.n, dat.JA1.c, dat.JA1.stltitle] = stlread("Atlas_Leg_1.STL");
    [dat.JA2.verts, dat.JA2.faces, dat.JA2.n, dat.JA2.c, dat.JA2.stltitle] = stlread("Atlas_Leg_2.STL");
    [dat.JA3.verts, dat.JA3.faces, dat.JA3.n, dat.JA3.c, dat.JA3.stltitle] = stlread("2337T41_Rubber Wheel.STL");
    [dat.JB1.verts, dat.JB1.faces, dat.JB1.n, dat.JB1.c, dat.JB1.stltitle] = stlread("Atlas_Leg_1.STL");
    [dat.JB2.verts, dat.JB2.faces, dat.JB2.n, dat.JB2.c, dat.JB2.stltitle] = stlread("Atlas_Leg_2.STL");
    [dat.JB3.verts, dat.JB3.faces, dat.JB3.n, dat.JB3.c, dat.JB3.stltitle] = stlread("2337T41_Rubber Wheel.STL");
    [dat.JC1.verts, dat.JC1.faces, dat.JC1.n, dat.JC1.c, dat.JC1.stltitle] = stlread("Atlas_Leg_1.STL");
    [dat.JC2.verts, dat.JC2.faces, dat.JC2.n, dat.JC2.c, dat.JC2.stltitle] = stlread("Atlas_Leg_2.STL");
    [dat.JC3.verts, dat.JC3.faces, dat.JC3.n, dat.JC3.c, dat.JC3.stltitle] = stlread("2337T41_Rubber Wheel.STL");
    [dat.JD1.verts, dat.JD1.faces, dat.JD1.n, dat.JD1.c, dat.JD1.stltitle] = stlread("Atlas_Leg_1.STL");
    [dat.JD2.verts, dat.JD2.faces, dat.JD2.n, dat.JD2.c, dat.JD2.stltitle] = stlread("Atlas_Leg_2.STL");
    [dat.JD3.verts, dat.JD3.faces, dat.JD3.n, dat.JD3.c, dat.JD3.stltitle] = stlread("2337T41_Rubber Wheel.STL");
end



% joint positions
r_IIrF = zeros(3,1);

r_F_F_r_A1 = [0.22966150; 0.06751003; -0.05715000];
r_A1_A1_r_A2 = [0.00000000; -0.02900997; -0.10160000];
r_A2_A2_r_A3 = [-0.00008990; -0.02451970; -0.10218470];

r_F_F_r_B1 = [0.22966150; -0.06751003; -0.05715000];
r_B1_B1_r_B2 = r_A1_A1_r_A2;
r_B2_B2_r_B3 = r_A2_A2_r_A3;

r_F_F_r_C1 = [-0.23388850; 0.06751003; -0.05715000];
r_C1_C1_r_C2 = r_A1_A1_r_A2;
r_C2_C2_r_C3 = r_A2_A2_r_A3;

r_F_F_r_D1 = [-0.23388850; -0.06751003; -0.05715000];
r_D1_D1_r_D2 = r_A1_A1_r_A2;
r_D2_D2_r_D3 = r_A2_A2_r_A3;



% translational position of main body
delta_x = gamma(1);
delta_y = gamma(2);
delta_z = gamma(3);
% pose angles of main body
phi = gamma(4);   % rotation about the forward (x) axis
theta = gamma(5); % rotation about the y axis
psi = gamma(6);   % rotation about the z axis
% front left leg
theta_A1 = gamma(7);
theta_A2 = gamma(8);
theta_A3 = gamma(9);
% front right leg
theta_B1 = gamma(10);
theta_B2 = gamma(11);
theta_B3 = gamma(12);
% back left leg
theta_C1 = gamma(13);
theta_C2 = gamma(14);
theta_C3 = gamma(15);
% back right leg
theta_D1 = gamma(16);
theta_D2 = gamma(17);
theta_D3 = gamma(18);



T_ITF = rotx(phi)*roty(theta)*rotz(psi);

T_FTA1 = roty(theta_A1);
T_A1_T_A2 = roty(theta_A2);
T_A2_T_A3 = roty(theta_A3);

T_FTB1 = roty(theta_B1);
T_B1_T_B2 = roty(theta_B2);
T_B2_T_B3 = roty(theta_B3);

T_FTC1 = roty(theta_C1);
T_C1_T_C2 = roty(theta_C2);
T_C2_T_C3 = roty(theta_C3);

T_FTD1 = roty(theta_D1);
T_D1_T_D2 = roty(theta_D2);
T_D2_T_D3 = roty(theta_D3);



% transform DCMs back to I frame
T_ITA1 = T_ITF * T_FTA1;
T_ITA2 = T_ITA1 * T_A1_T_A2;
T_ITA3 = T_ITA2 * T_A1_T_A2;

T_ITB1 = T_ITF * T_FTB1;
T_ITB2 = T_ITB1 * T_B1_T_B2;
T_ITB3 = T_ITB2 * T_B1_T_B2;

T_ITC1 = T_ITF * T_FTC1;
T_ITC2 = T_ITC1 * T_C1_T_C2;
T_ITC3 = T_ITC2 * T_C1_T_C2;

T_ITD1 = T_ITF * T_FTD1;
T_ITD2 = T_ITD1 * T_D1_T_D2;
T_ITD3 = T_ITD2 * T_D1_T_D2;


% Transform Position vectors into I frame
r_IIrA1 = r_IIrF + (T_ITF * r_F_F_r_A1);
r_IIrA2 = r_IIrA1 + (T_ITA1 * r_A1_A1_r_A2);
r_IIrA3 = r_IIrA2 + (T_ITA2 * r_A2_A2_r_A3);

r_IIrB1 = r_IIrF + (T_ITF * r_F_F_r_B1);
r_IIrB2 = r_IIrB1 + (T_ITB1 * r_B1_B1_r_B2);
r_IIrB3 = r_IIrB2 + (T_ITB2 * r_B2_B2_r_B3);

r_IIrC1 = r_IIrF + (T_ITF * r_F_F_r_C1);
r_IIrC2 = r_IIrC1 + (T_ITC1 * r_C1_C1_r_C2);
r_IIrC3 = r_IIrC2 + (T_ITC2 * r_C2_C2_r_C3);

r_IIrD1 = r_IIrF + (T_ITF * r_F_F_r_D1);
r_IIrD2 = r_IIrD1 + (T_ITD1 * r_D1_D1_r_D2);
r_IIrD3 = r_IIrD2 + (T_ITD2 * r_D2_D2_r_D3);



% Transform STL coordinates into I frame
dat.JF.vertsInI = r_IIrF + T_ITF*(dat.JF.verts)';
dat.JA1.vertsInI = r_IIrA1 + T_ITA1*(dat.JA1.verts)';
dat.JA2.vertsInI = r_IIrA2 + T_ITA2*(dat.JA2.verts)';
dat.JA3.vertsInI = r_IIrA3 + T_ITA3*(dat.JA3.verts)';
dat.JB1.vertsInI = r_IIrB1 + T_ITB1*(dat.JB1.verts)';
dat.JB2.vertsInI = r_IIrB2 + T_ITB2*(dat.JB2.verts)';
dat.JB3.vertsInI = r_IIrB3 + T_ITB3*(dat.JB3.verts)';
dat.JC1.vertsInI = r_IIrC1 + T_ITC1*(dat.JC1.verts)';
dat.JC2.vertsInI = r_IIrC2 + T_ITC2*(dat.JC2.verts)';
dat.JC3.vertsInI = r_IIrC3 + T_ITC3*(dat.JC3.verts)';
dat.JD1.vertsInI = r_IIrD1 + T_ITD1*(dat.JD1.verts)';
dat.JD2.vertsInI = r_IIrD2 + T_ITD2*(dat.JD2.verts)';
dat.JD3.vertsInI = r_IIrD3 + T_ITD3*(dat.JD3.verts)';



if frameNum < 2
    % Draw each frame as patch object
    p{1}  = patch('Faces', dat.JF.faces,  'Vertices', dat.JF.vertsInI',  'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{2}  = patch('Faces', dat.JA1.faces, 'Vertices', dat.JA1.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{3}  = patch('Faces', dat.JA2.faces, 'Vertices', dat.JA2.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{4}  = patch('Faces', dat.JA3.faces, 'Vertices', dat.JA3.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{5}  = patch('Faces', dat.JB1.faces, 'Vertices', dat.JB1.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{6}  = patch('Faces', dat.JB2.faces, 'Vertices', dat.JB2.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{7}  = patch('Faces', dat.JB3.faces, 'Vertices', dat.JB3.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{8}  = patch('Faces', dat.JC1.faces, 'Vertices', dat.JC1.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{9}  = patch('Faces', dat.JC2.faces, 'Vertices', dat.JC2.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{10} = patch('Faces', dat.JC3.faces, 'Vertices', dat.JC3.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{11} = patch('Faces', dat.JD1.faces, 'Vertices', dat.JD1.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{12} = patch('Faces', dat.JD2.faces, 'Vertices', dat.JD2.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
    p{13} = patch('Faces', dat.JD3.faces, 'Vertices', dat.JD3.vertsInI', 'EdgeColor', 'None', 'FaceColor', [0.792157, 0.819608, 0.933333]);
   


else
    % if simulation if past frame 1, just vertex data instead of drawing
    % from scratch (way faster)
    set(p{1},  'Faces', dat.JF.faces, 'Vertices',  dat.JF.vertsInI');
    set(p{2},  'Faces', dat.JA1.faces, 'Vertices', dat.JA1.vertsInI');
    set(p{3},  'Faces', dat.JA2.faces, 'Vertices', dat.JA2.vertsInI');
    set(p{4},  'Faces', dat.JA3.faces, 'Vertices', dat.JA3.vertsInI');
    set(p{5},  'Faces', dat.JB1.faces, 'Vertices', dat.JB1.vertsInI');
    set(p{6},  'Faces', dat.JB2.faces, 'Vertices', dat.JB2.vertsInI');
    set(p{7},  'Faces', dat.JB3.faces, 'Vertices', dat.JB3.vertsInI');
    set(p{8},  'Faces', dat.JC1.faces, 'Vertices', dat.JC1.vertsInI');
    set(p{9},  'Faces', dat.JC2.faces, 'Vertices', dat.JC2.vertsInI');
    set(p{10}, 'Faces', dat.JC3.faces, 'Vertices', dat.JC3.vertsInI');
    set(p{11}, 'Faces', dat.JD1.faces, 'Vertices', dat.JD1.vertsInI');
    set(p{12}, 'Faces', dat.JD2.faces, 'Vertices', dat.JD2.vertsInI');
    set(p{13}, 'Faces', dat.JD3.faces, 'Vertices', dat.JD3.vertsInI');

end





end