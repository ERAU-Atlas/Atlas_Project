% Set the ROS domain variable to 101 using setenv
setenv('ROS_DOMAIN_ID', '88');

% Create a ROS 2 node
joystick_Node = ros2node("joystick_subscriber");

% Create a subscriber to listen to the joystick message
joySub = ros2subscriber(joystick_Node, "/joy", "sensor_msgs/Joy", @joyCallback);

% Define the callback function for the joystick message
function joyCallback(msg)
    % Process the joystick message here
    % You can access the joystick axes and buttons using msg.Axes and msg.Buttons respectively
end

