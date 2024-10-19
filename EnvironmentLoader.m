% Author: Jackson Russelll 14250803
% The EnvironmentLoader class is responsible for loading all objects, this
% includes the respective robot model (in our case the LinearUR3 with a
% gripper), the bricks, table and miscellaneous items to make the
% environment as realistic as possible

classdef EnvironmentLoader
    
    properties
        % Make pr2Prismatic base + Arms
        pr2Base
        pr2LeftArm
        pr2RightArm
        pr2KnifeArm
        %pr2Right
        %pr2Left
        gripperl1
        gripperr1
        gripperl2
        gripperr2
        gripperrk
        
        pr2Right
        pr2Left

        tm5700
        tm5700Gripper

    end
    
    methods
        
        function obj = EnvironmentLoader() % Constructor to initialize the environment
            %
            obj.loadCustomObjects();
            compEnv = 0;
            if compEnv
                % Make tm5900, edit base position if required
                obj.tm5700 = TM5.TM5700(transl(1.0,0.8,0.78));

                % Make tm5900 gripper, same as above
                obj.tm5700Gripper = TM5.TM5Gripper((obj.tm5700.model.fkine([0 0 0 0 0 0]).T));
                
                % Make pr2Prismatic base + Arms
                obj.pr2Base = PR2.PR2Base();
                obj.pr2LeftArm = PR2.PR2LeftArm(obj.pr2Base.model.base.T);
                obj.pr2RightArm = PR2.PR2RightArm(obj.pr2Base.model.base.T);
                obj.pr2KnifeArm = Knife.robotKnife();
                %obj.tm5700 = TM5.TM5900();
                %obj.pr2Left = PR2Left();
                %obj.pr2Right = PR2Right();

                % Left arm is 1, right arm is 2, l and r correspond to
                % fingers of the gripper
                obj.gripperl1 = PR2.PR2LeftGripper();
                obj.gripperr1 = PR2.PR2RightGripper();
                obj.gripperl2 = PR2.PR2LeftGripper();
                obj.gripperr2 = PR2.PR2RightGripper();
                % light('Position', [1 1 1], 'Style', 'infinite');
                % lighting gouraud;  
                % material shiny;   
                % camlight('headlight');
                % camlight('left');
                
                qz = [0 0 0 0 0 0 0];
                obj.pr2LeftArm.model.animate(qz);
                obj.pr2RightArm.model.animate(qz);
                %obj.pr2Left.model.animate(qz);
                %obj.pr2Right.model.animate(qz);
            else
                obj.pr2Base = PR2.PR2Base();
                obj.pr2LeftArm = PR2.PR2LeftArm(obj.pr2Base.model.base.T);
                obj.pr2RightArm = PR2.PR2RightArm(obj.pr2Base.model.base.T);
                obj.tm5700 = TM5.TM5700(transl(1.0,0.8,0.78));
                % light('Position', [1 1 1], 'Style', 'infinite');
                % lighting gouraud;  
                % material shiny;   
                % camlight('headlight');
                % camlight('left');
                
                qz = [0 pi/2 0 0 0 0 0];
                q = [0 0 0 0 0 0];
                obj.pr2LeftArm.model.teach();
                obj.pr2RightArm.model.teach();
                              
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

            % Using surf for now to place images onto the environment
            % X + offset
            % Y + offset
            % Z + offset
            % Move the -ve around to adjust image orientation

            surf([0.01, 0.01; 0.01, 0.01] - 1.6, ... % X coordinates (after rotation, becomes depth)
                [-0.3, 0.3; -0.3, 0.3] + 1.6, ... % Y coordinates (horizontal width remains unchanged)
                [0.3, 0.3; -0.3, -0.3] + 1.5, ... % Z coordinates (shift up by adding 2 to the height)
                'CData', imread('images/FootProtectionSafety.jpg'), ...
                'FaceColor', 'texturemap');

            surf([0.01, 0.01; 0.01, 0.01] -1.6, ... % X coordinates (after rotation, becomes depth)
                [-0.3, 0.3; -0.3, 0.3] + 1.6, ... % Y coordinates (horizontal width remains unchanged)
                [0.3, 0.3; -0.3, -0.3] + 0.8, ... % Z coordinates (shift up by adding 2 to the height)
                'CData', imread('images/HairNetSafety.jpg'), ...
                'FaceColor', 'texturemap');

            surf([0.5, -0.5; 0.5, -0.5] + 1.7, ... % X coordinates (after rotation, becomes depth)
                [0.01, 0.01; 0.01, 0.01] - 2.15, ... % Y coordinates (horizontal width remains unchanged)
                [0.5, 0.5; -0.5, -0.5] + 1, ... % Z coordinates (shift up by adding 2 to the height)
            'CData', imread('images/Storyboard.png'), ...
            'FaceColor', 'texturemap');   

            kitchenRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/ModifiedKitchen_v13.2.ply',[0.2, 0, 0], 1, kitchenRotations)

            bananaRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/Banana.ply',[1.0, 0, 0.82], 1, bananaRotations)

            boardRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/cutting_board.ply',[1.0, 0, 0.8], 1, boardRotations)


            hazardLightRotations = { {0, 'XY'}, {0, 'XZ'}, {-90, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/hazard_light.ply',[1.0, -2.13, 1.5], 0.1, hazardLightRotations)

            fireExtinguisherRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} }; 
            obj.CustomPlaceObject('plyFiles/Scenery/fireExtinguisher.ply',[1.0, -2.05, 0], 1, fireExtinguisherRotations)

            eStopRotations = { {0, 'XY'}, {0, 'XZ'}, {-90, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/emergencyStopButton.ply', [1.0, -2.13, 1], 1, eStopRotations)

            barrier1 = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[2.5, -1.55, 0.2], 0.2, barrier1)
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[2.5, -0.35, 0.2], 0.2, barrier1)
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[2.5, 0.85, 0.2], 0.2, barrier1)

            barrier2 = { {-35, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[2, 1.85, 0.2], 0.2, barrier2)

            barrier3 = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[0.87, 2.18, 0.2], 0.2, barrier3)
            obj.CustomPlaceObject('plyFiles/Scenery/SafetyBarrier.ply',[-0.35, 2.18, 0.2], 0.2, barrier3)

            doorRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/Scenery/door.ply',[0.21, -2.15, 0.15], 0.9, doorRotations)
            



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

