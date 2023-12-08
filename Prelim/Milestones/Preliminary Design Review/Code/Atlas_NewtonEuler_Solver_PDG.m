close all
clear
clc

setup();
%% prepare simulation
% simulation time-step:
h = 0.001;
t = 0:h:6;
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
F(3, :) = 10*ones(1, length(t));
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


%% set gains and constants
K_p = 1;
K_D = .01;

%% desired state
d_b = zeros(36, length(t));
d_dot_b = zeros(36, length(t));


%% Runge-Kutta Integrator
disp("starting Runge-Kutta")
tic
for i = 1 : length(t)-1
    disp(i);

    F(:,i) =  K_p*(b(1:18, i) - d_b(1:18,i)) + K_D*(b(19:end, i) - d_b(19:end, i));
    F(1:6, i) = 0; % cannot exert forces directly on the system and its pose

    k1 = Atlas_NewtonEuler(b(:,i), F(:,i));
    k2 = Atlas_NewtonEuler(b(:,i) + k1 * (h/2), F(:,i));
    k3 = Atlas_NewtonEuler(b(:,i) + k2 * (h/2), F(:,i));
    k4 = Atlas_NewtonEuler(b(:,i) + k3 * h, F(:,i));
    b(:,i+1) = b(:,i)+h*(k1/6+k2/3+k3/3+k4/6);
    
    % correct for angles greater than 2pi
    % b(7:end,i+1) = angle(exp(1i*b(7:end, i+1)));
    clc
end
toc
disp("end of Runge-Kutta")





%% Data Plot
% initialize figure
fig1 = figure(1);
fig1.Name = "Atlas Pose: Torque Controlled";


% plot angles
for k = 1:6
    subplot(6, 1, k)
    plot(t, b(k, :))
    xlabel('t (s)')
    str = "\theta_" + num2str(k) + "  (rad)";
    ylabel(str);

    % plot desired configuration
    hold on
    plot(t, d_b(k, :), 'color', 'red', 'LineStyle', '--')
    hold off

end


fig2 = figure(2);
fig2.Name = "Leg A joint angles: Torque Controlled";
for k = 1:2
    subplot(2, 1, k)
    plot(t, b(k+10, :))
    xlabel('t (s)')
    str = "\theta_" + num2str(k) + "  (rad)";
    ylabel(str);

    % plot desired configuration
    hold on
    plot(t, d_b(k+10, :), 'color', 'red', 'LineStyle', '--')
    hold off

end




% 
% % plot angles
% for k = 1:size(b, 1)/2
%     subplot(size(b, 1)/2, 1, k)
%     plot(t, b(k, :))
%     xlabel('t (s)')
%     str = "\theta_" + num2str(k) + "  (rad)";
%     ylabel(str);
% 
% end





% % plot velocities
% fig2 = figure(2);
% fig2.Name = "Joint Velocities: normal";
% 
% for k = 1:size(b, 1)/2
%     subplot(size(b, 1)/2, 1, k)
%     plot(t, b(k+(size(b, 1)/2), :))
%     xlabel('t (s)')
%     str = "\theta_" + num2str(k) + "  (rad/sec)";
%     ylabel(str, 'Interpreter','tex');
% 
% end


%% Animation
if 1
    % initialize figure
    fig3 = figure();
    fig3.Name = "Animation";

%create a video object – this will be stored as VP_6242_robot_Animation.avi
v = VideoWriter('Atlas Simulation.avi');

set(v,'FrameRate',fps);

% open the video
open(v);

% create and capture each frame
k = 0;
for i=1:round(1/(fps*h)):length(t) %20 frames per second
    k = k+1;
    % Atlas_draw(b(:,i), fig3, k, [1,1,1], [-.75, .75, -.75, .75, -2, .75], 1)
    Atlas_draw(b(:,i), fig3, k, [1,1,1], [-1, 1, -1, 1, -.5, .5], 1)
    % diffBot_draw(b(:,i), fig3, k);
    time = (i-1)*h;
    title(time, "FontSize", 10)
    drawnow
    frame = getframe(gcf); %store the current figure window as a frame
    writeVideo(v,frame); %write that frame to the video
end
close(v); %close the video


end


