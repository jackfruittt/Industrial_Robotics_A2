classdef PR2LeftArm < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'PR2';
    end

    methods
        function self = PR2LeftArm(baseTr)
            self.CreateModel();
            if nargin < 1 
                baseTr = eye(4);
            end
            self.model.base = baseTr; 
            self.PlotAndColourRobot();
        end

        function CreateModel(self)

            % DH Parameters taken from
            % http://www.ros.org/wiki/pr2_calibration_estimation (Left
            % Arm)
            %
            % Left Qlims adjusted using pr2_manual
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


             links = [
                        Revolute('d', 0.0,   'a', 0.1, 'alpha', -pi/2, 'qlim', [deg2rad(-40) deg2rad(130)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(60) deg2rad(170)], 'offset', pi/2)
                        Revolute('d', 0.4,   'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-44) deg2rad(224)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(133)])
                        Revolute('d', 0.321, 'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-180) deg2rad(180)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(130)])
                        Revolute('d', 0.0,   'a', 0.0, 'alpha', 0,     'qlim', [deg2rad(-180) deg2rad(180)])
                     ];

            self.model =  SerialLink(links, 'name', self.name);
        end
    end
end