classdef robotGripper < handle
    %% robotGripper Class to define a 2-link planar gripper

    properties(Access = public)
        gripperLeft;  % Left-side Gripper
        gripperRight; % Right-side Gripper
        
    end
    
    methods
        %% Constructor
        function self = robotGripper(baseTr)
            if nargin < 1
                baseTr = eye(4); 
            end
            self.CreateModel(baseTr);
            self.PlotModel();
           
        end

        %% Create the gripper model
        function CreateModel(self, baseTr)
            %Create the UR3 model mounted on a linear rail
            %DH parameters
            %Link([θ      d        a        αlpha      sigma])
            % theta is the joint angle for a revolute joint
            % d is the link offset along z
            % a is the link length (the distance between the z-axis of two joints)
            % alpha is the twist angle 
            % Sigma is the joint type, 0 revolute 1 prismatic
            linkLeft(1) = Link([0 0 0.08 0 0]); 
            linkLeft(2) = Link([0 0 0.08 0 0]);
            
            linkRight(1) = Link([0 0 0.08 0 0]); 
            linkRight(2) = Link([0 0 0.08 0 0]);

            % Joint limits
            linkLeft(1).qlim = [-pi pi];
            linkLeft(2).qlim = [-pi pi];
            linkRight(1).qlim = [-pi pi];
            linkRight(2).qlim = [-pi pi];

            % Create the SerialLink model for both grippers
            self.gripperLeft = SerialLink(linkLeft, 'name', 'LeftGripper');
            self.gripperRight = SerialLink(linkRight, 'name', 'RightGripper');  % Use linkRight here

            % Set base transformation for left and right grippers
            %rotationMatrix = troty(0);
            self.gripperRight.base = baseTr; % Offset to the right
            self.gripperLeft.base = baseTr; % Offset to the left
        end

        %% Plot the gripper models
        function PlotModel(self)
            
            q = [pi/4 -pi/4]; % Initial joint angles
            p = [-pi/4 pi/4];
            
            self.gripperRight.plot(q); % Plot the right gripper in the zero configuration
            hold on;
            self.gripperLeft.plot(p);  % Plot the left gripper in the zero configuration

        end
    end
end