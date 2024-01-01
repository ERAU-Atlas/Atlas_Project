function bugFinder_Callback(message)
    % BUGFINDER_CALLBACK This function is a callback for the ROS2 subscriber "bugFound".
    % It receives a message of type std_msgs/Int32MultiArray from the topic "/bugs".
    % The message data is an array of integers where the first element is the total number of bugs found so far,
    % and the second element is the identifier of the current bug found.
    %
    % Parameters:
    %   message : A ROS2 message of type std_msgs/Int32MultiArray. The 'data' field of the message
    %             is an array of integers where the first element is the total number of bugs found so far,
    %             and the second element is the identifier of the current bug found.
    %
    % Example:
    %   bugFinder_Callback(message)
    %
    % See also: ros2subscriber, ros2message
    
    num = message.data;
    fprintf("bug found: #%3.d, (%d bugs so far)\n", num(2), num(1));
    end