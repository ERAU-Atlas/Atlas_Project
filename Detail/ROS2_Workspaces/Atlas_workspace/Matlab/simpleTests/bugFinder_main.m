clear
clc

%% Setup
setenv('ROS_DOMAIN_ID','10'); % set ros domain ID

bugFinder = ros2node("bugFinder"); % main node


bugNumPublisher = ros2publisher(bugFinder, "/bugs", "std_msgs/Int32MultiArray"); % bug publisher
bugIdentifier = ros2publisher(bugFinder, "/bugs", "std_msgs/Int32MultiArray");

bugFound = ros2subscriber(bugFinder, "/bugs", "std_msgs/Int32MultiArray", @bugFinder_Callback);


%% loop
i = ros2message("std_msgs/Int32MultiArray");
i.data = int32([0, int32(randi([1, 200]))]);

while(1)
    i.data(1) = i.data(1) + 1;
    i.data(2) = int32(randi([1, 200]));
    send(bugNumPublisher, i);



    pause(1);
end