classdef PR2RightArm < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'plyFiles/PR2RightNew/PR2';
    end

    methods
        function self = PR2RightArm(baseTr)
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
            % http://www.ros.org/wiki/pr2_calibration_estimation (Right
            % Arm)
            %                  theta alpha   a   d      LB  UB
            %0 Shoulder Pan    0     -90    0.1 0.0    -130 40 
            %1 Shoulder Lift   0      90    0.0 0.0     60  170
            %2 Upper Arm Roll  0     -90    0.0 0.4    -224 44
            %3 Elbow Flex      0      90    0.0 0.0     0   133
            %4 Forearm Roll    0     -90    0.0 0.321  -180 180
            %5 Wrist Flex      0      90    0.0 0.0     0   130
            %6 Wrist Roll      0      0     0.0 0.0    -180 180


             links = [
                        Revolute('d', 0.0,   'a', 0.1, 'alpha', -pi/2, 'qlim', [deg2rad(-130) deg2rad(40)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(60) deg2rad(170)], 'offset', pi/2)
                        Revolute('d', 0.4,   'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-224) deg2rad(44)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(133)])
                        Revolute('d', 0.321, 'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-180) deg2rad(180)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(130)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', 0,     'qlim', [deg2rad(-180) deg2rad(180)])
                     ];

            self.model =  SerialLink(links, 'name', self.name);
        end
    end
end