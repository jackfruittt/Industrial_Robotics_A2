classdef robotKnife < RobotBaseClass
    %ROBOTKNIFE A class that creates a model for the robot knife
    %   The knife can be attached to a robot's end effector and moved.
    
    properties(Access = public)
        plyFileNameStem = 'plyFiles/knife/knife';
    end
    
    methods
        %% Constructor
        function self = robotKnife(baseTr)
            if nargin < 1
                baseTr = transl(1, 2, 0);
            end
            
            self.CreateModel();
            self.model.base = baseTr;
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
            % Create a single link for the knife
            link(1) = Link('alpha', 0, 'a', 0, 'd', 0.01, 'offset', 0);
            
            % Create the SerialLink model and assign it a name
            self.model = SerialLink(link, 'name', self.name);
        end
        
        %% AttachToEndEffector
        function attachToEndEffector(self, T)
            % Method to attach knife to a robot end effector
            self.model.base = T * trotx(pi) * troty(-pi/2) * transl(-0.4, 0, 0);
            self.model.animate(0);
        end
    end
end
