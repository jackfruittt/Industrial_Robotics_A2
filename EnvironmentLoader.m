% Author: Jackson Russelll 14250803
% The EnvironmentLoader class is responsible for loading all objects, this
% includes the respective robot model (in our case the LinearUR3 with a
% gripper), the bricks, table and miscellaneous items to make the
% environment as realistic as possible

classdef EnvironmentLoader
    
    properties
        pr2Right
        pr2Left
        gripperl1
        gripperr1
        gripperl2
        gripperr2
        gripperrk
    end
    
    methods
        
        function obj = EnvironmentLoader() % Constructor to initialize the environment
            %
            obj.loadCustomObjects();
            compEnv = 0;
            if compEnv
                obj.pr2Left = PR2Left();
                obj.pr2Right = PR2Right();
                obj.gripperl1 = PR2LeftGripper();
                obj.gripperr1 = PR2RightGripper();
                obj.gripperl2 = PR2LeftGripper();
                obj.gripperr2 = PR2RightGripper();
                obj.gripperrk = PR2RightGripperWithKnife();
                light('Position', [1 1 1], 'Style', 'infinite');
                lighting gouraud;  
                material shiny;   
                camlight('headlight');
                camlight('left');
                
                qz = [0 pi/2 0 0 0 0 0];
                obj.pr2Left.model.animate(qz);
                obj.pr2Right.model.animate(qz);
            else
                obj.pr2Left = PR2Left();
                obj.pr2Right = PR2Right();
                obj.gripperl1 = PR2LeftGripper();
                obj.gripperr1 = PR2RightGripper();
                obj.gripperl2 = PR2LeftGripper();
                obj.gripperr2 = PR2RightGripper();
                light('Position', [1 1 1], 'Style', 'infinite');
                lighting gouraud;  
                material shiny;   
                camlight('headlight');
                camlight('left');
                
                qz = [0 pi/2 0 0 0 0 0];
                tr = obj.pr2Right.model.fkine(qz);
                tr1 = obj.pr2Left.model.fkine(qz);
                obj.pr2Left.model.teach(tr1);
                obj.pr2Right.model.teach(tr);
                obj.pr2Left.model.animate(qz);
                obj.pr2Right.model.animate(qz);
            end
        end

        function loadCustomObjects(obj)
            % % Load table one
            % tableOneRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            % obj.CustomPlaceObject('plyFiles/Scenery/tableBrown2.1x1.4x0.5m.ply', [0, 0, 0], 1, tableOneRotations);
            % 
            % % Load floor texture
            % surf([-4, -4; 4, 4] ...
            %     ,[-2, 2; -2, 2] ...
            %     ,[0.01, 0.01; 0.01, 0.01] ...
            %     ,'CData', imread('images/floor_wood.jpg') ...
            %     ,'FaceColor', 'texturemap');
            kitchenRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/ModifiedKitchen_v11.1.ply',[0.80, 0, -1.1], 1, kitchenRotations)
        end
        
        %Custom PlaceObject function that plots ply files based off the RTB PlaceObject function.
        %This function allows me to additionally scale and rotate
        %the ply files I place
        function CustomPlaceObject(~, filename, position, scaleFactor, rotations)
            
            [f, v, data] = plyread(filename, 'tri'); % f faces v vertices

            %Apply scaling
            vScaled = v * scaleFactor;
            vTransformed = vScaled;

            %Apply rotations
            for i = 1:length(rotations)
                rotationAngle = rotations{i}{1};
                rotationPlane = rotations{i}{2};

                %Create rotation matrix based on axis
                switch rotationPlane
                    case 'XY'
                        R = [cosd(rotationAngle), -sind(rotationAngle), 0; 
                             sind(rotationAngle),  cosd(rotationAngle), 0;
                             0,                    0,                   1];
                    case 'XZ'
                        R = [cosd(rotationAngle),  0, sind(rotationAngle);
                             0,                    1, 0;
                             -sind(rotationAngle), 0, cosd(rotationAngle)];
                    case 'YZ'
                        R = [1, 0,                    0;
                             0, cosd(rotationAngle), -sind(rotationAngle);
                             0, sind(rotationAngle),  cosd(rotationAngle)];
                    otherwise
                        %Case sensitive input, output this error just in case
                        error('Invalid rotation axis. Choose from ''XY'', ''XZ'', or ''YZ''.');
                end

                %Apply the rotation
                vTransformed = (R * vTransformed')';
            end

            %Apply translation
            vTranslated = vTransformed + position;
            
            %PlaceObject
            %Plot the object
            %Extract color data if available
           try
                try
                   vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                catch
                     try
                        vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;        
                     catch
                          vertexColours = [0.5,0.5,0.5];
                     end
                end
                %Plot the object with the original colors
                trisurf(f, vTranslated(:,1), vTranslated(:,2), vTranslated(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
           catch ME_1
                disp(ME_1);
           end
        end

        function setupLighting(~)
                 
            delete(findall(gcf, 'Type', 'light'));
            %camlight('headlight');  % Light that moves with the camera
            %lighting gouraud;  % Options: 'flat', 'gouraud'
            %lighting flat;  % Options: 'flat', 'gouraud'
            %material dull;  % Options: 'shiny', 'dull', 'metal'
        
        end
    end
end

