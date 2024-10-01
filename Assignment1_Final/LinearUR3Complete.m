% Author: Jackson Russelll 14250803
% The LinearUR3 Complete attaches the robotGripper from robotGripper.m to
% the LinearUR3 from LineaerUR3.m
% This allows me to load a complete model of the robot which can be
% controlled later on

classdef LinearUR3Complete
    
    properties(Access = public)
        
        LinearUR3WithGripper %UR3 robot on a linear rail
        GripperModel %Planar 2-link robot gripper
    
    end

    methods
        function self = LinearUR3Complete()
            
            %Initialize the UR3 and Gripper models
            self.LinearUR3WithGripper = LinearUR3(); %Load the LinearUR3 model
            self.GripperModel = robotGripper(); %Load the Gripper model
            
            %Attach the gripper to the end-effector of the UR3
            self.attachGripper();
            
        end

        function attachGripper(self)
            
            % Get the end-effector transformation of LinearUR3
            p1 = self.LinearUR3WithGripper.model.getpos();    % Get current joint angles
            p2 = self.LinearUR3WithGripper.model.fkine(p1).T; % Get end-effector transform
            rotationMat = troty(3*pi/2) * trotx(pi/2);        % Apply rotation for correct orientation
            ewg = p2 * rotationMat;                            % Final gripper position based on end-effector, ewg = end-effector with gripper
        
            % Set the base of the Gripper to the UR3's end-effector position
            self.GripperModel.gripperLeft.base = ewg;
            self.GripperModel.gripperRight.base = ewg;
        
            % Default joint angles for the grippers
            q = [pi/4 -pi/4];  % Right gripper joint angles
            p = [-pi/4 pi/4];  % Left gripper joint angles
        
            % Plot the grippers with the default joint angles
            self.GripperModel.gripperRight.plot(q); % Plot the right gripper
            hold on;
            self.GripperModel.gripperLeft.plot(p);  % Plot the left gripper
        
            drawnow;

        end
    end
end

