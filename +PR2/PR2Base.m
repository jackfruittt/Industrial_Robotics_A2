classdef PR2Base < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'plyFiles/PR2PrismaticBase/Base';
    end

    methods
        function self = PR2Base(baseTr)
            self.CreateModel();
            if nargin < 1 
                baseTr = eye(4);
            end

            rotationAdjustment = trotx(pi);
            heightAdjustment = transl(0.35,0,-1.025);

            self.model.base = baseTr * rotationAdjustment * heightAdjustment; 
            self.PlotAndColourRobot();
        end

        function CreateModel(self)
            
            link(1) = Link([0    0    -0.085   pi    1]); % Torso Lift, prismatic
            link(2) = Link([0    0.15    0.03   -pi/2    0]); % Head Pan, No laser tilt
            link(3) = Link([0    0    0    0       0]); % Head Tilt

            % Set qlims for joints
            link(1).qlim = [-0.3 0];  
            link(2).qlim = [-168 168]*pi/180; 
            link(3).qlim = [-30 60]*pi/180;

            self.model =  SerialLink(link, 'name', self.name);
        end
    end
end