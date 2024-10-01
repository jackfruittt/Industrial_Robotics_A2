% Author: Jackson Russelll 14250803
%Load ROS bag
bag = rosbag('2018-03-20-18-34-46.bag');

% List available topics in the ROS bag
bag.AvailableTopics

jointStateData = select(bag, 'Topic','/joint_states');
jointStateMsg = readMessages(jointStateData, 'DataFormat', 'struct');

numMsg = length(jointStateMsg);
jointAngles = zeros(length(jointStateMsg), 6);

%Extract the Joint positions
for i = 1:numMsg    
    jointAngles(i, :) = jointStateMsg{i}.Position; 
end

r = UR3();

for i = 1:10:size(jointAngles, 1)
    r.model.animate(jointAngles(i, :));
    drawnow();
end