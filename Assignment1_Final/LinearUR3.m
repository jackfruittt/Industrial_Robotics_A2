% Author: Jackson Russelll 14250803
% Modified UR3 class which contains the DH parameters for the linear rail
% LinearUR5 and the UR3 robot and fuses them together

classdef LinearUR3 < RobotBaseClass
    %% LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'plyFiles/LinearUR3';
        volume
    end
    
    methods
%% Define robot Function 
        function self = LinearUR3(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end

            %%Add Table Properties to Shift the bease model of the linearUR3
            tableHeight = 0.5;
            tableTransform = transl(0, 0, tableHeight);
            shiftTr = transl(1, 1, 1);
            rotationAngle = 0;
            rotationMatrix = trotz(rotationAngle);
            self.model.base = tableTransform * rotationMatrix * self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();
            self.GetVolume();
        end

%% Create the robot model
        function CreateModel(self)   
            %Create the UR3 model mounted on a linear rail
            %DH parameters
            %Link([θ      d        a        αlpha      sigma])
            % theta is the joint angle for a revolute joint
            % d is the link offset along z
            % a is the link length (the distance between the z-axis of two joints)
            % alpha is the twist angle 
            % Sigma is the joint type, 0 revolute 1 prismatic
            link(1) = Link([pi     0       0          pi/2  1]); % PRISMATIC Link
            link(2) = Link([0      0.1519  0          pi/2   0]);
            link(3) = Link([0      0       -0.24365   0      0]);
            link(4) = Link([0      0       -0.21325   0      0]);
            link(5) = Link([0      0.11235 0          pi/2   0]);
            link(6) = Link([0      0.08535 0          -pi/2	 0]);
            link(7) = Link([0      0.0819  0          0      0]);
            
            %Incorporate joint limits
            link(1).qlim = [-0.8 0];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-150 125]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-135 135]*pi/180;
            link(7).qlim = [-360 360]*pi/180;
        
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
        

        %% Calculate maximum reach and volume
        function [volume] = GetVolume(self)
            self.volume = ((4/3)*pi*(0.542^3)+(pi*(0.542^2)*0.8))/2 ; %Volume of a capsule
            volume = self.volume;
            disp('Volume data for Linear UR3: ');
            disp(['- The maximum reach of the UR3 without linear rail is ', num2str(0.542), 'm']);
            disp(['- The maximum reach of the UR3 with the linear rail is ', num2str(1.342), 'm'...
                ' along the x-axis and ',num2str(0.542), 'm along the y-axis and z-axis']); 
            disp(['- The workspace volume of the UR3 is: ',num2str(self.volume),'m^3']);
        end
    end
end
