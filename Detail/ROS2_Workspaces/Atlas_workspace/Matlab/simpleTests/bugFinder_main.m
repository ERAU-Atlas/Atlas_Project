% Clear workspace and command window
clear;
clc;

% Setup
setenv('ROS_DOMAIN_ID','10'); % Set ROS domain ID

% Create main node
bugFinder = ros2node("bugFinder");

% Create publishers
bugNumPublisher = ros2publisher(bugFinder, "/bugs", "std_msgs/Int32MultiArray");
bugIdentifier = ros2publisher(bugFinder, "/bugs", "std_msgs/Int32MultiArray");

% Create subscriber
bugFound = ros2subscriber(bugFinder, "/bugs", "std_msgs/Int32MultiArray", @bugFinder_Callback);

% Initialize message
i = ros2message("std_msgs/Int32MultiArray");
i.data = int32([0, int32(randi([1, 200]))]);

% Main loop
while true
    % Increment bug count and generate new bug identifier
    i.data(1) = i.data(1) + 1;
    i.data(2) = int32(randi([1, 200]));

    % Publish message
    send(bugNumPublisher, i);

    % Pause for 1 second
    pause(1);
end