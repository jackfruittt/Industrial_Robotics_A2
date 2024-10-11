classdef PR2LeftArm < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'plyFiles/PR2LeftNew/PR2';
    end

    methods
        function self = PR2LeftArm(baseTr)
            self.CreateModel();
            if nargin < 1 
                baseTr = eye(4);
            end

            heightAdjustment = transl(-0.1, -0.18, 0.2);

            self.model.base = baseTr * heightAdjustment; 
            self.PlotAndColourRobot();
        end

        function CreateModel(self)

            % DH Parameters taken from
            % http://www.ros.org/wiki/pr2_calibration_estimation (Left
            % Arm)
            %
            % Left Qlims adjusted using pr2_manual and manual testing
            % https://www.clearpathrobotics.com/assets/downloads/pr2/pr2_manual_r321.pdf
            %
            %                  theta alpha   a   d      LB  UB
            %0 Shoulder Pan    0     -90    0.1 0.0    -40  130 
            %1 Shoulder Lift   0      90    0.0 0.0     60  170
            %2 Upper Arm Roll  0     -90    0.0 0.4    -44  224
            %3 Elbow Flex      0      90    0.0 0.0     0   133
            %4 Forearm Roll    0     -90    0.0 0.321  -180 180
            %5 Wrist Flex      0      90    0.0 0.0     0   130
            %6 Wrist Roll      0      0     0.0 0.0    -180 180


            link(1) = Link([0      0.0      0.1     -pi/2          0]);
            link(2) = Link([0      0.0      0        pi/2          0]);
            link(3) = Link([0      0.4      0       -pi/2          0]);
            link(4) = Link([0      0.0      0        pi/2          0]); 
            link(5) = Link([0      0.321    0       -pi/2          0]); 
            link(6) = Link([0      0.0      0        pi/2          0]); 
            link(7) = Link([0      0.0      0        0             0]);
            
            link(1).qlim = [deg2rad(-130) deg2rad(40)];
            link(2).qlim = [deg2rad(30) deg2rad(140)]; %Willow Garage spcifies [-30 80], ROS specifies [60 170], [30 140] seems best for MATLAB and fits within 110-deg range
            link(3).qlim = [deg2rad(-224) deg2rad(44)];
            link(4).qlim = [deg2rad(0) deg2rad(133)];
            link(5).qlim = [deg2rad(-180) deg2rad(180)];
            link(6).qlim = [deg2rad(0) deg2rad(130)];
            link(7).qlim = [deg2rad(-180) deg2rad(180)];

            self.model =  SerialLink(link, 'name', self.name);
        end
    end
end