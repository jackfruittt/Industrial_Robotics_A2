classdef PR2
    properties (Access = public)
        base;
        leftArmTr;
        leftArm;
        rightArmTr;
        rightArm;
        qBaseDefault = [0 0 0];
        qLeftArmDefault = [0 0 0 0 0 0 0];
        qRightArmDefault = [0 0 0 0 0 0 0];

    end

    methods
        function self = PR2(baseTr)
            if nargin < 1
                baseTr = eye(4);
            end

            self.base = PR2Base(baseTr);
            self.leftArmTr = self.base.model.base.T * transl(-0.1, 0.18, 0.2);
            self.rightArmTr = self.base.model.base.T * transl(-0.1, -0.18, 0.2);
            self.leftArm = PR2LeftArm(self.leftArmTr);
            self.rightArm = PR2RightArm(self.rightArmTr);
            
            self.base.model.teach(self.qBaseDefault);
            hold on
            self.leftArm.model.teach(self.qLeftArmDefault);
            hold on
            self.rightArm.model.teach(self.qRightArmDefault);
            hold on

        end

        function plotPR2(self)
            
            % self.base.model.teach(self.qBaseDefault);
            % hold on
            % self.leftArm.model.teach(self.qLeftArmDefault);
            % hold off

            % self.base.model.teach(self.qBaseDefault);
            % hold on
            % self.leftArm.model.teach(self.qLeftArmDefault);
            % hold on
            % self.rightArm.model.teach(self.qRightArmDefault);
            % hold off
        end

        function updateArmTr(self)
            self.leftArm.model.base = self.base.model.base.T * transl(-0.1, 0.18, 0.2)
            self.rightArm.model.base = self.base.model.base.T * transl(-0.1, -0.18, 0.2)
        end
    end
end