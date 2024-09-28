% %                  theta alpha   a   d      LB  UB
% %0 Shoulder Pan    0     -90    0.1 0.0    -130 40
% %1 Shoulder Lift   0      90    0.0 0.0     60  170
% %2 Upper Arm Roll  0     -90    0.0 0.4    -224 44
% %3 Elbow Flex      0      90    0.0 0.0     0   133
% %4 Forearm Roll    0     -90    0.0 0.321  -180 180
% %5 Wrist Flex      0      90    0.0 0.0     0   130
% %6 Wrist Roll      0      0     0.0 0.0    -180 180
% 
% links = [
%         Revolute('d', 0.0,   'a', 0.1, 'alpha', -pi/2, 'qlim', [deg2rad(-130) deg2rad(40)])
%         Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(60) deg2rad(170)], 'offset', pi/2)
%         Revolute('d', 0.4,   'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-224) deg2rad(44)])
%         Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(133)])
%         Revolute('d', 0.321, 'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-180) deg2rad(180)])
%         Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(130)])
%         Revolute('d', 0.0,   'a', 0.0, 'alpha', 0,     'qlim', [deg2rad(-180) deg2rad(180)])
% ];
% 
% left =  SerialLink(links, 'name', 'PR2 LEFT', 'manufacturer', 'Willow Garage');
% right = SerialLink(links, 'name', 'PR2 RIGHT', 'manufacturer', 'Willow Garage');
% 
% left.base = transl(0.064614, 0.25858, 0.119) 
% right.base = transl(0.063534, -0.25966, 0.119) 
% 
% % define the workspace vectors:
% %   qz         zero joint angle configuration
% %   qr         vertical 'READY' configuration
% %   qstretch   arm is stretched out in the X direction
% %   qn         arm is at a nominal non-singular configuration
% %
% qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
% qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
% qs = [0 0 -pi/2 0 0 0 0];
% qn = [0 pi/4 pi/2 0 pi/4  0 0];

classdef PR2 < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'PR2';
    end

    methods
        function self = PR2(baseTr)
            self.CreateModel();
            if nargin < 1 
                baseTr = eye(4);
            end
            
            self.model.base = baseTr; 
            %self.left.base = baseTr * transl(0.064614, 0.25858, 0.119);
            %self.right.base = baseTr * transl(0.063534, -0.25966, 0.119);

            self.PlotAndColourRobot();
          
        end

        function CreateModel(self)

            %                  theta alpha   a   d      LB  UB
            %0 Shoulder Pan    0     -90    0.1 0.0    -130 40
            %1 Shoulder Lift   0      90    0.0 0.0     60  170
            %2 Upper Arm Roll  0     -90    0.0 0.4    -224 44
            %3 Elbow Flex      0      90    0.0 0.0     0   133
            %4 Forearm Roll    0     -90    0.0 0.321  -180 180
            %5 Wrist Flex      0      90    0.0 0.0     0   130
            %6 Wrist Roll      0      0     0.0 0.0    -180 180

             links = [
                    Revolute('d', 0.49,   'a', 0.1, 'alpha', pi/2, 'qlim', [deg2rad(-130) deg2rad(40)])
                    Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(60) deg2rad(170)], 'offset', pi/2)
                    Revolute('d', 0.4,   'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-224) deg2rad(44)])
                    Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(133)])
                    Revolute('d', 0.321, 'a', 0.0, 'alpha', -pi/2, 'qlim', [deg2rad(-180) deg2rad(180)])
                    Revolute('d', 0.0,   'a', 0.0, 'alpha', pi/2,  'qlim', [deg2rad(0) deg2rad(130)])
                    Revolute('d', 0.0,   'a', 0.0, 'alpha', 0,     'qlim', [deg2rad(-180) deg2rad(180)])
            ];

            self.model =  SerialLink(links, 'name', 'PR2 LEFT', 'manufacturer', 'Willow Garage');

            %self.model.left =  SerialLink(self.links, 'name', 'PR2 LEFT', 'manufacturer', 'Willow Garage');
            %self.model.right = SerialLink(self.links, 'name', 'PR2 RIGHT', 'manufacturer', 'Willow Garage');
            
            %self.left.n = numel(self.links);
            %self.right.n = numel(self.links);
            
            % define the workspace vectors:
            %   qz         zero joint angle configuration
            %   qr         vertical 'READY' configuration
            %   qstretch   arm is stretched out in the X direction
            %   qn         arm is at a nominal non-singular configuration
            %
            %qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
            %qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
            %qs = [0 0 -pi/2 0 0 0 0];
            %qn = [0 pi/4 pi/2 0 pi/4  0 0];
        end
    end
end