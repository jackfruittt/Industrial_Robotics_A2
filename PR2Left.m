%                  theta alpha   a   d      LB  UB
%0 Shoulder Pan    0     -90    0.1 0.0    -130 40
%1 Shoulder Lift   0      90    0.0 0.0     60  170
%2 Upper Arm Roll  0     -90    0.0 0.4    -224 44
%3 Elbow Flex      0      90    0.0 0.0     0   133
%4 Forearm Roll    0     -90    0.0 0.321  -180 180
%5 Wrist Flex      0      90    0.0 0.0     0   130
%6 Wrist Roll      0      0     0.0 0.0    -180 180

classdef PR2Left < RobotBaseClass
    
    properties(Access = public)              
        plyFileNameStem = 'plyFiles/PR2/PR2';  
    end
    
    methods
        function self = PR2Left(baseTr)
            
            self.CreateModel();
            
            if nargin < 1
                baseTr = eye(4);  
            end
            heightAdjustment = transl(0, 0, -1); 
            rotationAdjustment = trotx(pi); 

            self.model.base = baseTr * rotationAdjustment * heightAdjustment; 
            self.PlotAndColourRobot();
        end
        
        function CreateModel(self)
            % Define the DH parameters for the PR2 robot links using Link objects
            %Link(         [θ      d        a        αlpha      sigma])
            link(1) = Link([0      0.0      0.1     -pi/2          0]);
            link(2) = Link([0      0.0      0        pi/2          0]);
            link(3) = Link([0      0.4      0       -pi/2          0]);
            link(4) = Link([0      0.0      0        pi/2          0]); 
            link(5) = Link([0      0.321    0       -pi/2          0]); 
            link(6) = Link([0      0.0      0        pi/2          0]); 
            link(7) = Link([0      0.0      0        0             0]);
            
            link(1).qlim = [deg2rad(-130) deg2rad(40)];
            link(2).qlim = [deg2rad(40) deg2rad(210)];
            link(3).qlim = [deg2rad(-224) deg2rad(44)];
            link(4).qlim = [deg2rad(0) deg2rad(133)];
            link(5).qlim = [deg2rad(-180) deg2rad(180)];
            link(6).qlim = [deg2rad(0) deg2rad(130)];
            link(7).qlim = [deg2rad(-180) deg2rad(180)];

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end

