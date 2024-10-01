% Author: Jackson Russelll 14250803
% Class containing the functional code to move the LinearUR3 from point A
% to Point B. Contains a custom q interpralaotr and alpha equation to
% accept negative qlim values, without this the manipulator will not move as I
% found jtraj to cause a lot of errors, utlises animate() to move the manipulator

classdef moveRobot
    
    properties
        env 
        steps % Number of interpolation steps
        gripperControl % Instance of the moveGripper class
    end
    
    methods
        
        function manipulator = moveRobot(env, steps)
            
            manipulator.env = env; % Initialize environment
            manipulator.steps = steps; % Set number of interpolation steps
            manipulator.gripperControl = moveGripper(env); % Initialize moveGripper instance
        
        end
        
        % Function below animates the manipulator (7-link manipulator) by a number of steps
        function manipulateRobot(manipulator, T1, T2, numSteps, line, qLeft1Start, qLeft1End, qRight1Start, qRight1End, qGripperLeft, qGripperRight)
            
            % Compute joint angles for the start and end positions
            q1 = manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.ikcon(T1); % Start configuration
            q2 = manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.ikcon(T2); % End configuration
            
            % LSPB interpolation
            s = lspb(0, 1, numSteps); % Trajectory blend between 0 and 1 over numSteps
            qPre = nan(numSteps, 7); % Preallocate for performance
            endEffectorTransforms = zeros(4, 4, numSteps);
            
            % Interpolation and forward kinematics
            for i = 1:numSteps
                
                qPre(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Linear interpolation
                endEffectorTransforms(:, :, i) = manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.fkine(qPre(i, :)).T; % FK for the end-effector
            
            end
            
            % Animate the robot movement
            for i = 1:numSteps 
                
                % Update LinearUR3
                manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.animate(qPre(i, :)); 
                
                if line == true
                    
                    rh = findobj('Tag', manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.name); 
                    ud = rh.UserData; 
                    hold on; 
                    ud.trail = plot(0, 0, '-'); 
                    set(rh, 'UserData', ud);
                
                else
                    
                    rh = findobj('Tag', manipulator.env.LinearUR3Model.LinearUR3WithGripper.model.name); 
                    ud = rh.UserData; 
                    hold on; 
                    ud.trail = plot(0, 0, '-', 'Color', 'none'); 
                    set(rh, 'UserData', ud);
                
                end
                
                % Update the gripper's base to follow the end-effector
                endEffectorTransform = endEffectorTransforms(:, :, i) * troty(3*pi/2);
                manipulator.env.LinearUR3Model.GripperModel.gripperLeft.base = endEffectorTransform;
                manipulator.env.LinearUR3Model.GripperModel.gripperRight.base = endEffectorTransform;
                
                % Animate the gripper
                manipulator.env.LinearUR3Model.GripperModel.gripperLeft.animate(qGripperLeft); 
                manipulator.env.LinearUR3Model.GripperModel.gripperRight.animate(qGripperRight);
                
                x = endEffectorTransform(1, 4);
                y = endEffectorTransform(2, 4);
                z = endEffectorTransform(3, 4);
                fprintf('End Effector Position: X = %.2f, Y = %.2f, Z = %.2f\n', x, y, z);
                
                drawnow();
            end
            
            % After robot movement is done, animate the gripper
            manipulator.gripperControl.manipulateGripper(qLeft1Start, qLeft1End, qRight1Start, qRight1End, numSteps);
        end
    end
end
