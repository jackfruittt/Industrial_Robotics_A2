% Author: Jackson Russelll 14250803
% Class to animate the gripper
classdef moveGripper
    
    properties
        env
    end
    
    methods
        function gripper = moveGripper(env)
            gripper.env = env; %Initialize environment
        end
        
        %Function to move q1 of both grippers while keeping q2 static
        function [finalQLeft, finalQRight] = manipulateGripper(gripper, qLeftStart, qLeftEnd, qRightStart, qRightEnd, numSteps)
            
            % Initial static angles
            qStaticLeft = -pi/4;
            qStaticRight = pi/4;
            
            % Precompute interpolated joint angles
            qLeftInterpolated = zeros(numSteps, 2);
            qRightInterpolated = zeros(numSteps, 2);
            
            % Interpolate joint angles for q1 while q2 is static
            for i = 1:5:numSteps
                
                alpha = (i - 1) / (numSteps - 1); % Interpolation parameter
                % Interpolated joint angles for moving q1
                qLeftInterpolated(i, :) = [ (1 - alpha) * qLeftStart + alpha * qLeftEnd, qStaticLeft];
                qRightInterpolated(i, :) = [ (1 - alpha) * qRightStart + alpha * qRightEnd, qStaticRight];
            
            end
            
            % Animate gripper
            for i = 1:5:numSteps 
                
                % Update the gripper models with the interpolated joint angles
                gripper.env.LinearUR3Model.GripperModel.gripperLeft.animate(qLeftInterpolated(i, :)); 
                gripper.env.LinearUR3Model.GripperModel.gripperRight.animate(qRightInterpolated(i, :)); 
                drawnow();
            
            end
            
            % Return final joint angles
            finalQLeft = qLeftInterpolated(end, :);
            finalQRight = qRightInterpolated(end, :);
        
        end
    end
end






