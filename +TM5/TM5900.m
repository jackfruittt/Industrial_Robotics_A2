classdef TM5900 < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!
    
    properties(Access = public)
        plyFileNameStem = 'plyFiles/TM5900/TM5900';
    end
    
    methods
        %% Constructor
        function self = TM5900(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);
                end
            else % All passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
            
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
            % Link(         [θ      d          a        αlpha      sigma])
            link(1) = Link([0      0.1452     0        -pi/2        0]);        
            link(2) = Link([0      0          0.429    0            0]);      
            link(3) = Link([0      0          0.4115   0            0]);         
            link(4) = Link([0     -0.12230    0        pi/2         0]);        
            link(5) = Link([0      0.106      0        pi/2         0]);        
            link(6) = Link([0      0.1132     0        0            0]);         
            
            link(1).qlim = deg2rad([-180 180]);
            link(2).qlim = deg2rad([-180 180]);
            link(3).qlim = deg2rad([-180 180]);
            link(4).qlim = deg2rad([-180 180]);
            link(5).qlim = deg2rad([-180 180]);
            link(6).qlim = deg2rad([-180 180]);
            
            link(2).offset = -pi/2;
            link(4).offset = pi/2;
            
            % Create the SerialLink model and assign it a name
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end

