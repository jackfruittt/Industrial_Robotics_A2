classdef robotBanana < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'plyFiles/Banana/Banana';
    end

    methods
        function self = robotBanana(baseTr)
            if nargin < 1
                baseTr = eye(4);
            end

            self.CreateModel();
            self.model.base = baseTr;
            self.PlotAndColourRobot();
        end

        function CreateModel(self)
            link(1) = Link([0 0 0 0 0]);
            self.model = SerialLink(link, 'name', self.name);
        end

        function attachToEndEffector(self,endEffectorTr)
            self.model.base = endEffectorTr * trotz(-pi/2) * trotx(pi) * transl(0,0,-0.13);
            self.model.animate(0);
        end
    end
end
