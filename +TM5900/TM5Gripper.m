classdef TM5Gripper < RobotBaseClass
    
    properties(Access = public)
        plyFileNameStem = 'plyFiles/Robotiq2FGripper/TM5900';
    end
    
    methods
        %% Constructor
        function self =  TM5Gripper(baseTr)
            self.CreateModel();

            if nargin < 1
                baseTr = eye(4); 
            end
            self.model.base = baseTr * transl(0, -0.012, 0);
            %self.PlotAndColourRobot();
           
        end
        function CreateModel(self)
            % DH parameters for the two-link gripper
            %Link(         [θ      d        a        αlpha      sigma])
            link(1) = Link([0      0        0        0          0]);
            link(2) = Link([0      0.03     0        0          0]);
            link(3) = Link([0      0.1      0        0          0]);

            

            link(1).qlim = [-pi/6 0];
            link(2).qlim = [-pi/6 pi/6];
            

            % Create SerialLink for the gripper
            self.model = SerialLink(link, 'name', self.name);
            self.model.teach();
        end
    end
end