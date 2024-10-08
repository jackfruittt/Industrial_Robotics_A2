%{%}
%θ — Joint angle (rotation about z-axis)
%d — Joint offset (translation along z-axis)
%a — Link length (translation along x-axis)
%α — Link twist (rotation about x-axis)

classdef PR2RightGripper < RobotBaseClass
    %% robotGripper Class to define a 2-link planar gripper

    properties(Access = public)
        plyFileNameStem = 'plyFiles/PR2GripperRight/Right'; 
        
    end

    methods
        %% Constructor
        function self =  PR2RightGripper(baseTr)
            self.CreateModel();

            if nargin < 1
                baseTr = eye(4); 
            end
            self.model.base = baseTr * transl(0, -0.012, 0);
            self.PlotAndColourRobot();
           
        end
        function CreateModel(self)
            L1 = 67.568 / 1000;   % Link length a in meters
            b = 48.972 / 1000;    % Distance b in meters
            L0 = 34.708 / 1000;   % Distance L0 in meters
            r = 91.5 / 1000;      % Distance r in meters
            c = 16.01/1000;
            theta0 = deg2rad(2.976); % Initial angle in radians
            phi0 = deg2rad(29.987);  % Initial angle in radians

            % DH parameters for the two-link gripper
            %Link(         [θ      d        a        αlpha      sigma])
            link(1) = Link([0      0        r        0           0]);
            link(2) = Link([0      0        0         0          0]);

            link(1).qlim = [-pi/6 0];
            link(2).qlim = [-pi/6 pi/6];
            link(1).offset = deg2rad(-18);
            link(2).offset = deg2rad(18);
            

            % Create SerialLink for the gripper
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end

%{
classdef PR2RightGripper < RobotBaseClass
    %% robotGripper Class to define a 2-link planar gripper

    properties(Access = public)
        plyFileNameStem = 'plyFiles/PR2GripperRight/Right'; 
        
    end

    methods
        %% Constructor
        function self =  PR2RightGripper(baseTr)
            self.CreateModel();

            if nargin < 1
                baseTr = eye(4); 
            end
            self.model.base = baseTr * transl(0, -0.012, 0);
            %self.PlotAndColourRobot();
           
        end
        function CreateModel(self)
            L1 = 67.568 / 1000;   % Link length a in meters
            b = 48.972 / 1000;    % Distance b in meters
            L0 = 34.708 / 1000;   % Distance L0 in meters
            r = 91.5 / 1000;      % Distance r in meters
            c = 16.01/1000;
            theta0 = deg2rad(2.976); % Initial angle in radians
            phi0 = deg2rad(29.987);  % Initial angle in radians
            %hold on;
            % DH parameters for the two-link gripper
            %Link(         [θ      d        a        αlpha      sigma])
            link(1) = Link([0      0        r        0           0]);
            link(2) = Link([0      0        0         0          0]);

            link(1).qlim = [-pi/6 0];
            link(2).qlim = [-pi/6 pi/6];
            link(1).offset = deg2rad(-18);
            link(2).offset = deg2rad(18);
            

            % Create SerialLink for the gripper
            self.model = SerialLink(link, 'name', self.name);
            self.model.teach();
        
            centerPoint = [0,0,0];
            radii = [0.6,0.01,0.01];

            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
            for i = 1:2
                self.model.points{i} = [X(:),Y(:),Z(:)];
                warning off
                self.model.faces{i} = delaunay(self.model.points{i});    
                warning on;
                self.model.plot3d([0 0]);
                axis equal
            end
        end
    end
end
%}