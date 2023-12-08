% This script will simulate the Atlas system for a given time and
% timestep.
%
% Inputs:
%   None
%
% Outputs:
%   Figures
%
% Example:
%   N/A
% 
% Description:
%   when run, this script will use simulate over time, the Atlas system
%   and using a Runge-Kutta loop it will calculate the system state for
%   each instance in time, then plot out the joint state variables across
%   time and animate the robot in 3D ocross the same time domain.
%
% required m-files:
%   Atlas_NewtonEuler.m
%     for dynamics calculations
%
% Subfunctions:
%   None
%
% required MAT-files:
%   None
%
% Author: Ian Adelman
% Email: Adelmani@my.erau.edu
% Maintainer: Ian Adelman
% Maintainer Email: Adelmani@my.erau.edu
% Created: createdDate
% Revised: revisedDate
%
% Previous Ver #: 1.0 : 11/22/2023
%  Current Ver #: 1.5 : 12/07/2023
% Most Recent Version Notes:
%   updated format
%


close all
clear
clc


%% prepare simulation
% simulation time-step:
h = 0.001;
t = 0:h:3;
fps = 30;

% set up state vector
b = zeros(18*2, length(t));
b(1, 1) = 0; % x := x axis location of main body
b(2, 1) = 0; % y := y axis location of main body
b(3, 1) = .35; % z := z axis location of main body
b(4, 1) = 0; % phi
b(5, 1) = 0; % theta
b(6, 1) = 0; % psi
b(7, 1) = 0; % Leg A angle 1
b(8, 1) = 0; % Leg A angle 2
b(9, 1) = 0; % Leg A angle 3
b(10, 1) = 0; % Leg B angle 1
b(11, 1) = 0; % Leg B angle 2
b(12, 1) = 0; % Leg B angle 3
b(13, 1) = 0; % Leg C angle 1
b(14, 1) = 0; % Leg C angle 2
b(15, 1) = 0; % Leg C angle 3
b(16, 1) = 0; % Leg D angle 1
b(17, 1) = 0; % Leg D angle 2
b(18, 1) = 0; % Leg D angle 3

% set derivative initial conditions
b(19, 1) = 0; % x     = x axis velocity of main body
b(20, 1) = 0; % y     = y axis velocity of main body
b(21, 1) = 0; % z     = z axis velocity of main body
b(22, 1) = 0; % phi   = angular velocity about x
b(23, 1) = 0; % theta = angular velocity about y
b(24, 1) = 0; % psi   = angular velocity about z
b(25, 1) = 0; % Leg A angular velocity 1
b(26, 1) = 0; % Leg A angular velocity 2
b(27, 1) = 0; % Leg A angular velocity 3
b(28, 1) = 0; % Leg B angular velocity 1
b(29, 1) = 0; % Leg B angular velocity 2
b(30, 1) = 0; % Leg B angular velocity 3
b(31, 1) = 0; % Leg C angular velocity 1
b(32, 1) = 0; % Leg C angular velocity 2
b(33, 1) = 0; % Leg C angular velocity 3
b(34, 1) = 0; % Leg D angular velocity 1
b(35, 1) = 0; % Leg D angular velocity 2
b(36, 1) = 0; % Leg D angular velocity 3


r = 0.03757378; % wheel radius (m)
width = 0.2420874; % width of wheelbase


% correct initial conditions because dot_x, dot_y, and dot_psi cannot be
% set arbitrarily
b(19, 1) = (r/2)*cos(b(6, 1))*b(27, 1) + (r/2)*cos(b(6, 1))*b(30, 1);
b(20, 1) = (r/2)*sin(b(6, 1))*b(27, 1) + (r/2)*sin(b(6, 1))*b(30, 1);
b(24, 1) = (r/width)*b(27, 1) - (r/width)*b(30, 1);

% do the same for the back wheels
b(19, 1) = (r/2)*cos(b(6, 1))*b(33, 1) + (r/2)*cos(b(6, 1))*b(36, 1);
b(20, 1) = (r/2)*sin(b(6, 1))*b(33, 1) + (r/2)*sin(b(6, 1))*b(36, 1);
b(24, 1) = (r/width)*b(33, 1) - (r/width)*b(36, 1);

% set up joint force vector
F = zeros(size(b, 1)/2 , length(t));
F(1, :) = 0*ones(1, length(t));
F(2, :) = 0*ones(1, length(t));
F(3, :) = 0*ones(1, length(t));
F(4, :) = 0*ones(1, length(t));
F(5, :) = 0*ones(1, length(t));
F(6, :) = 0*ones(1, length(t));

F(7, :) = 0*ones(1, length(t));
F(8, :) = 0*ones(1, length(t));
F(9, :) = 0*ones(1, length(t));

F(10, :) = 0*ones(1, length(t));
F(11, :) = 0*ones(1, length(t));
F(12, :) = 0*ones(1, length(t));

F(13, :) = 0*ones(1, length(t));
F(14, :) = 0*ones(1, length(t));
F(15, :) = 0*ones(1, length(t));

F(16, :) = 0*ones(1, length(t));
F(17, :) = 0*ones(1, length(t));
F(18, :) = 0*ones(1, length(t));


%% Runge-Kutta Integrator
disp("starting Runge-Kutta")
tic
for i = 1 : length(t)-1
    disp(i);
   
    k1 = Atlas_NewtonEuler(b(:,i), F(:,i));
    k2 = Atlas_NewtonEuler(b(:,i) + k1 * (h/2), F(:,i));
    k3 = Atlas_NewtonEuler(b(:,i) + k2 * (h/2), F(:,i));
    k4 = Atlas_NewtonEuler(b(:,i) + k3 * h, F(:,i));
    b(:,i+1) = b(:,i)+h*(k1/6+k2/3+k3/3+k4/6);
    
    % correct for angles greater than 2pi
    % b(7:18,i+1) = angle(exp(1i*b(7:18, i+1)));
    clc
end
toc
disp("end of Runge-Kutta")





%% plot system Pose
% initialize figure
fig1 = figure(1);
fig1.Name = "System Pose";

subplot(6,1,1) % plot X pos
plot(t, b(1, :));
xlabel('t (s)')
ylabel('x (m)')

subplot(6,1,2) % plot y pos
title('abcd')
plot(t, b(2, :));
xlabel('t (s)')
ylabel('y (m)')

subplot(6,1,3) % plot z pos
plot(t, b(3, :));
xlabel('t (s)')
ylabel('z (m)')

subplot(6,1,4) % plot phi angle
plot(t, b(4, :));
xlabel('t (s)')
ylabel('\phi (rad)')

subplot(6,1,5) % plot theta angle
plot(t, b(5, :));
xlabel('t (s)')
ylabel('\theta (rad)')

subplot(6,1,6) % plot psi angle
plot(t, b(6, :));
xlabel('t (s)')
ylabel('\psi (rad)')
exportgraphics(fig1, 'System Pose.png', 'Resolution', '600');

%% Plot leg joint angles

% - - - - - - - - - LEG A
fig2 = figure(2);
fig2.Name = "Atlas Simulated Leg Angles: Leg A";

subplot(2,1,1)
plot(t, b(7, :));
xlabel('t (s)')
ylabel('\theta_{A1} (rad)')

subplot(2,1,2)
plot(t, b(8, :));
xlabel('t (s)')
ylabel('\theta_{A2} (rad)')
exportgraphics(fig2, 'angles leg A.png', 'Resolution', '600');

% - - - - - - - - - LEG B
fig3 = figure(3);
fig3.Name = "Atlas Simulated Leg Angles: Leg B";

subplot(2,1,1)
plot(t, b(10, :));
xlabel('t (s)')
ylabel('\theta_{B1} (rad)')

subplot(2,1,2)
plot(t, b(11, :));
xlabel('t (s)')
ylabel('\theta_{B2} (rad)')
exportgraphics(fig3, 'angles leg B.png', 'Resolution', '600');

% - - - - - - - - - LEG C
fig4 = figure(4);
fig4.Name = "Atlas Simulated Leg Angles: Leg C";

subplot(2,1,1)
plot(t, b(13, :));
xlabel('t (s)')
ylabel('\theta_{C1} (rad)')

subplot(2,1,2)
plot(t, b(14, :));
xlabel('t (s)')
ylabel('\theta_{C2} (rad)')
exportgraphics(fig4, 'angles leg C.png', 'Resolution', '600');

% - - - - - - - - - LEG D
fig5 = figure(5);
fig5.Name = "Atlas Simulated Leg Angles: Leg D";
title("Atlas Simulated Leg Angles: Leg D");

subplot(2,1,1)
plot(t, b(15, :));
xlabel('t (s)')
ylabel('\theta_{D1} (rad)')

subplot(2,1,2)
plot(t, b(16, :));
xlabel('t (s)')
ylabel('\theta_{D2} (rad)')
exportgraphics(fig5, 'angles leg D.png', 'Resolution', '600');

%% Plot system Pose derivative
fig6 = figure(6);
fig6.Name = "Joint Velocities";

subplot(6,1,1) % plot X velocity
plot(t, b(19, :));
xlabel('t (s)')
ylabel('$\dot{x}$ (m/s)', 'Interpreter','latex')

subplot(6,1,2) % plot y velocity
title('abcd')
plot(t, b(20, :));
xlabel('t (s)')
ylabel('$\dot{y}$ (m/s)', 'Interpreter','latex')

subplot(6,1,3) % plot z velocity
plot(t, b(21, :));
xlabel('t (s)')
ylabel('$\dot{z}$ (m/s)', 'Interpreter','latex')

subplot(6,1,4) % plot phi angular velocity
plot(t, b(22, :));
xlabel('t (s)')
ylabel('$\dot\phi$ (rad/s)', 'Interpreter','latex')

subplot(6,1,5) % plot theta angular velocity
plot(t, b(23, :));
xlabel('t (s)')
ylabel('$\dot\theta$ (rad/s)', 'Interpreter','latex')

subplot(6,1,6) % plot psi angular velocity
plot(t, b(24, :));
xlabel('t (s)')
ylabel('$\dot{\psi}$ (rad/s)', 'Interpreter','latex')
exportgraphics(fig6, 'System Pose Deriv.png', 'Resolution', '600');

%% Plot Joint Velocities for each leg
% - - - - - - - - - LEG A
fig7 = figure(7);
fig7.Name = "Atlas Simulated Leg Angle Velocities: Leg A";

subplot(2,1,1)
plot(t, b(25, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{A1}}$ (rad/s)', 'Interpreter','latex')

subplot(2,1,2)
plot(t, b(26, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{A2}}$ (rad/s)', 'Interpreter','latex')
exportgraphics(fig7, 'velocities leg A.png', 'Resolution', '600');

% - - - - - - - - - LEG B
fig8 = figure(8);
fig8.Name = "Atlas Simulated Leg Angle Velocities: Leg B";

subplot(2,1,1)
plot(t, b(28, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{B1}}$ (rad/s)', 'Interpreter','latex')

subplot(2,1,2)
plot(t, b(29, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{B2}}$ (rad/s)', 'Interpreter','latex')
exportgraphics(fig8, 'velocities leg B.png', 'Resolution', '600');

% - - - - - - - - - LEG C
fig9 = figure(9);
fig9.Name = "Atlas Simulated Leg Angle Velocities: Leg C";

subplot(2,1,1)
plot(t, b(31, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{C1}}$ (rad/s)', 'Interpreter','latex')

subplot(2,1,2)
plot(t, b(32, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{C2}}$ (rad/s)', 'Interpreter','latex')
exportgraphics(fig9, 'velocities leg C.png', 'Resolution', '600');

% - - - - - - - - - LEG D
fig10 = figure(10);
fig10.Name = "Atlas Simulated Leg Angle Velocities: Leg D";

subplot(2,1,1)
plot(t, b(34, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{D1}}$ (rad/s)', 'Interpreter','latex')

subplot(2,1,2)
plot(t, b(35, :));
xlabel('t (s)')
ylabel('$\dot{\theta_{D2}}$ (rad/s)', 'Interpreter','latex')
exportgraphics(fig10, 'velocities leg D.png', 'Resolution', '600');

%% Animation
if 1
    % initialize figure
    fig3 = figure();
    fig3.Name = "Animation";

% create a video object 
v = VideoWriter('Atlas Simulation.avi');

set(v,'FrameRate',fps);

% open the video
open(v);

% create and capture each frame
k = 0;
for i=1:round(1/(fps*h)):length(t) %20 frames per second
    k = k+1;
    Atlas_draw(b(:,i), fig3, k, [1,1,1], [-1, 1, -1, 1, -.5, .5], 1)
    
    time = (i-1)*h;
    title(time, "FontSize", 10)
    drawnow
    frame = getframe(gcf); %store the current figure window as a frame
    writeVideo(v,frame); %write that frame to the video
end
close(v); %close the video


end


