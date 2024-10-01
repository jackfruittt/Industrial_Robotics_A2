% Author: Jackson Russelll 14250803
% The EnvironmentLoader class is responsible for loading all objects, this
% includes the respective robot model (in our case the LinearUR3 with a
% gripper), the bricks, table and miscellaneous items to make the
% environment as realistic as possible

classdef EnvironmentLoader
    
    properties
        LinearUR3Model
        Bricks
        plottedObjects
    end
    
    methods
        
        function obj = EnvironmentLoader() % Constructor to initialize the environment
            %Load Complete UR3 + Gripper
            compEnv = 1;
            if compEnv
                obj.LinearUR3Model = LinearUR3Complete();
            else
            %For transformation handling 
                obj.LinearUR3Model = LinearUR3();
                q = [0 pi/2 pi/2 0 0 0 0]
                tr = obj.LinearUR3Model.model.fkine(q);
                obj.LinearUR3Model.model.teach(tr);
                obj.LinearUR3Model.model.animate(q); 
                obj.PlotWorkspaceVolume();
            end
            
            %Load bricks
            obj = obj.loadBricks();
            obj.displayBricks();
            obj.setupLighting();
            %Load other objects (tables, people, miscellaneous items, etc.)
            obj.loadCustomObjects();
            
        end
        
        function obj = loadBricks(obj)
            % Brick Setup
            brickLength = 0.15;  
            brickWidth = 0.18;  
            tableHeight = 0.5;
            startX = -1;   
            %startY = -0.35;
            startY = 0.35;
            z = tableHeight;
            
            % Define the .ply file
            plyFile = 'plyFiles/HalfSizedRedGreenBrick.ply';
            numBricks = 9;
            bricks = []; 
            
            
            %Double for loop approach for a 3x3 grid alternative
            %{ 
            for i = 1:3  
                for j = 1:3 
                    x = startX + (i - 1) * brickLength; % Calculate X-coordinate
                    y = startY + (j - 1) * brickWidth;  % Calculate Y-coordinate
                    varName = sprintf('b%d%d', i, j)   % Dynamically allocate a variable name
                    
                    % Create Brick object with position, initial transform (identity matrix), and .ply file
                    brickObj = Brick(varName, [x, y, z], eye(4), plyFile);
                    
                    % Store the Brick object in the array
                    bricks = [bricks, brickObj];
                end
            end
            %}
           

            for i = 1:numBricks  
                x = startX + (i - 1) * brickLength;
                y = startY;
                varName = sprintf('b%d', i);

                brickObj = Brick(varName, [x, y, z], eye(4), plyFile);

                bricks = [bricks, brickObj];
            end
            obj.Bricks = bricks;
        end
            
        
        function displayBricks(obj)
            for i = 1:numel(obj.Bricks)
                brick = obj.Bricks(i);
                brick.plotBrick();      %Display the brick
            end
        end


        function bricks = getAllBricks(obj)
            bricks = obj.Bricks; 
        end

        function brick = getBrick(obj, brickName)
            for i = 1:numel(obj.Bricks)
                if strcmp(obj.Bricks(i).Name, brickName)
                    brick = obj.Bricks(i);
                    return;
                end
            end
            error('Brick %s not found.', brickName);
        end

        function PlotWorkspaceVolume(obj)
            
            %Robot Info
            %{ end 1 (-1.342, -0.194, 0.652)
            %{ end 2 (0.542, -0.194, 0.652)
            %{ Height z = 1.194 - 0.652 = radius = 0.542
            radius = 0.542; % Radius of the sphere
            cylinderLength = 0.8; % Length of the half-cylinder

            %Sphere centers
            sphereCenter = [0, 0, 0.652];
            sphere2Center = [-0.8, 0, 0.652];

            %Define range for quarter-sphere
            theta = linspace(0, pi/2, 20); %Azimuthal angle from 0 to pi/2, angle from the z-axis in xy plane
            phi = linspace(0, pi, 20);   %Polar angle from 0 to pi, xy plane from x-axis
            [Theta, Phi] = meshgrid(theta, phi);

            %Convert spherical coordinates to Cartesian coordinates
            XSphere = radius * cos(Theta) .* sin(Phi) + sphereCenter(1);
            YSphere = radius * cos(Phi) + sphereCenter(2);
            ZSphere = radius * sin(Theta) .* sin(Phi) + sphereCenter(3);

            X2Sphere = radius * cos(-Theta) .* sin(-Phi) + sphere2Center(1);
            Y2Sphere = radius * cos(-Phi) + sphere2Center(2);
            Z2Sphere = radius * sin(-Theta) .* sin(-Phi) + sphere2Center(3);

            %Range for the half-cylinder
            thetaCylinder = linspace(0, pi, 20);  %Half-circle (0 to pi)
            xCylinder = linspace(-cylinderLength/2, cylinderLength/2, 20) - 0.4;
            [theta_cylinder, x_cylinder] = meshgrid(thetaCylinder, xCylinder);

            %Convert polar coordinates (Theta, X) to Cartesian coordinates (Y, Z, X)
            yCylinder = radius * cos(theta_cylinder);  
            zCylinder = radius * sin(theta_cylinder) + 0.652;  
            hold on;
            plot3(XSphere(:), YSphere(:), ZSphere(:), 'r.'); % Sphere 1
            plot3(X2Sphere(:), Y2Sphere(:), Z2Sphere(:), 'r.'); % Sphere 2
            plot3(x_cylinder(:), yCylinder(:), zCylinder(:), 'b.');
            drawnow();

        end

        function loadCustomObjects(obj)
            % Load table one
            tableOneRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/tableBrown2.1x1.4x0.5m.ply', [0, 0, 0], 1, tableOneRotations);

            % Load table two
            tableTwoRotations = { {90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/tableBrown2.1x1.4x0.5m.ply', [-1.4, 0, 0], 1, tableTwoRotations);

            % Load lego person
            PersonRotations = { {0, 'XY'}, {0, 'XZ'}, {90, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/lego_person.ply', [0, 2.1, 0.53], 0.15, PersonRotations);
            obj.CustomPlaceObject('plyFiles/lego_person.ply', [-1.3, 2.1, 0.53], 0.15, PersonRotations);

            %Load EStop
            eStopRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/emergencyStopButton.ply', [-1.5, -0.8, 0.5], 0.5, eStopRotations);
            
            %Load Hand
            handRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/hand.ply', [-1.5, -0.8, 0.8], 1, handRotations);

            %Load fire extinguisher
            extinRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/fireExtinguisher.ply', [3.5, -1.75, 0], 1, extinRotations);

            %Load Bookshelf
            bookRotations = { {0, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/bookcaseTwoShelves0.5x0.2x0.5m.ply', [3, 1.6, 0], 2, bookRotations);
            obj.CustomPlaceObject('plyFiles/bookcaseTwoShelves0.5x0.2x0.5m.ply', [3, 1.6, 1], 2, bookRotations);

            %Load chair
            chairRotations = { {0, 'XY'}, {0, 'XZ'}, {90, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/chair.ply', [-3, 1.75, 0], 1.5, chairRotations);
            obj.CustomPlaceObject('plyFiles/chair.ply', [-2.5, 1.75, 0], 1.5, chairRotations);
            obj.CustomPlaceObject('plyFiles/chair.ply', [-2, 1.75, 0], 1.5, chairRotations);

            %Load worker/supervisor
            personRotations = { {-90, 'XY'}, {10, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/personMaleOld.ply', [3, 0, -0.1], 1, personRotations);

            %Load Cones
            coneRotations = { {-90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/traffic_cone.ply', [1, -0.75, 0], 1, coneRotations);
            obj.CustomPlaceObject('plyFiles/traffic_cone.ply', [1, 0.75, 0], 1, coneRotations);
            obj.CustomPlaceObject('plyFiles/traffic_cone.ply', [1, 0, 0], 1, coneRotations);
            
            %Load haxard lights
            lightRotations = { {-90, 'XY'}, {0, 'XZ'}, {0, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/hazard_light.ply', [0.5, -0.9, 0.5], 0.1, lightRotations);
            obj.CustomPlaceObject('plyFiles/hazard_light.ply', [0.5, 0.9, 0.5], 0.1, lightRotations);
            
            %Load warning sign
            signRotations = { {-90, 'XY'}, {0, 'XZ'}, {90, 'YZ'} };
            obj.CustomPlaceObject('plyFiles/warning_sign.ply', [0, -1.1, 0.55], 0.1, signRotations);

            % Load floor texture
            surf([-4, -4; 4, 4] ...
                ,[-2, 2; -2, 2] ...
                ,[0.01, 0.01; 0.01, 0.01] ...
                ,'CData', imread('images/floor_wood.jpg') ...
                ,'FaceColor', 'texturemap');
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
            camlight('headlight');  % Light that moves with the camera
            lighting gouraud;  % Options: 'flat', 'gouraud'
            material dull;  % Options: 'shiny', 'dull', 'metal'
        
        end
    end
end

