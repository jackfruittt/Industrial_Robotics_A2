%Curently to load UI, in MATLAB CL run PR2ControlGUI.openGUI();
classdef PR2ControlGUI
    methods(Static)
        
        function openGUI()
            clf;
            clc;
            grid on;
            hold on;
            axis([-4.5 4.5 -3.5 3.5 0 3.5]);
            view(90, 10);
            
            numJointsR = 7;
            numJointsL = 7;
            
            obj.pr2Left = PR2.PR2Left();
            obj.pr2Right = PR2.PR2Right();
            q = [0 pi/2 0 0 0 0 0];
            obj.pr2Right.model.animate(q);
            obj.pr2Left.model.animate(q);
            
            % Retrieve the joint limits (qlims) for both arms
            qlimR = obj.pr2Right.model.qlim;  
            qlimL = obj.pr2Left.model.qlim;   
            
            % Define colors
            hexColor = '#8F938D';
            rgbColor = hex2rgb(hexColor);
            textColor = hex2rgb('#000000');
            
            % Cube
            centerpnt = [1.2, 0, -0.25];
            side = 1.5;
            plotOptions.plotFaces = true;
            [vertex, faces, faceNormals] = RectangularPrism(centerpnt - side / 2, centerpnt + side / 2, plotOptions);
            axis equal;
            camlight;
            
            % Create sliders for left arm
            sliderPanelL = uipanel('Title', 'Left Arm Joint Control', 'FontSize', 12, ...
                'FontName', 'Arial', ...
                'FontWeight', 'bold', ...
                'FontAngle', 'normal', ...
                'Position', [0 0 0.9 0.20], ...
                'BackgroundColor', rgbColor, ...
                'BorderType', 'none');
            
            for i = 1:numJointsL

                uicontrol('Style', 'text', 'Parent', sliderPanelL, ...
                    'FontSize', 12, ...
                    'FontWeight', 'bold', ...
                    'ForegroundColor', textColor, ...
                    'Backgroundcolor', rgbColor, ...
                    'Position', [50, 25 + (i-1)*30, 100, 30], ...
                    'String', ['Left Link ' num2str(i)]);
                defaultValueL = (qlimL(i, 1) + qlimL(i, 2)) / 2;
                
                obj.leftSliders(i) = uicontrol('Style', 'slider', ...
                    'Parent', sliderPanelL, ...
                    'Min', qlimL(i, 1), 'Max', qlimL(i, 2), ...
                    'Value', defaultValueL, ...
                    'Position', [150, 25 + (i-1)*30, 300, 30], ...
                    'SliderStep', [0.01 0.1]);
                
                addlistener(obj.leftSliders(i), 'Value', 'PostSet', @(src, event) updateRobotLeft(i));
                
                % Display slider value
                obj.leftSliderValueText(i) = uicontrol('Style', 'text', ...
                    'Parent', sliderPanelL, ...
                    'Position', [460, 25 + (i-1)*30, 100, 30], ...
                    'String', num2str(defaultValueL, '%.2f'), ...
                    'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0]);
            end
            
            % Create sliders for right arm
            sliderPanelR = uipanel('Title', 'Right Arm Joint Control', ...
                'FontSize', 12, ...
                'FontName', 'Arial', ...
                'FontWeight', 'bold', ...
                'FontAngle', 'normal', ...
                'Position', [0.40, 0, 0.9, 0.20], ...
                'BackgroundColor', rgbColor, ...
                'BorderType', 'none');
            
            for i = 1:numJointsR
                uicontrol('Style', 'text', 'Parent', sliderPanelR, ...
                    'FontSize', 12, ...
                    'FontWeight', 'bold', ...
                    'ForegroundColor', textColor, ...
                    'Backgroundcolor', rgbColor, ...
                    'Position', [50, 25 + (i-1)*30, 100, 30], ...
                    'String', ['Right Link ' num2str(i)]);
                defaultValueR = (qlimR(i, 1) + qlimR(i, 2)) / 2;
                
                obj.rightSliders(i) = uicontrol('Style', 'slider', ...
                    'Parent', sliderPanelR, ...
                    'Min', qlimR(i, 1), 'Max', qlimR(i, 2), ...
                    'Value', defaultValueR, ...
                    'Position', [150, 25 + (i-1)*30, 300, 30], ...
                    'SliderStep', [0.01 0.1]);
                
                addlistener(obj.rightSliders(i), 'Value', 'PostSet', @(src, event) updateRobotRight(i));
                
                obj.rightSliderValueText(i) = uicontrol('Style', 'text', ...
                    'Parent', sliderPanelR, ...
                    'Position', [460, 25 + (i-1)*30, 100, 30], ...
                    'String', num2str(defaultValueR, '%.2f'), ...
                    'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0]);
            end
            
            translationPanel = uipanel('Title', 'Translation Control', 'FontSize', 12, ...
                'FontSize', 12, ...                  
                'FontName', 'Arial', ...              
                'FontWeight', 'bold', ...               
                'FontAngle', 'normal', ...              
                'Position', [0.8, 0, 0.9, 0.20], ... 
                'BackgroundColor', rgbColor, ...
                'BorderType', 'none');
            
            % Sliders for X, Y, Z translations
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 160, 100, 30], 'String', 'Translate X', 'FontSize', 12);
            obj.translateXSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 140, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 110, 100, 30], 'String', 'Translate Y', 'FontSize', 12);
            obj.translateYSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 90, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 60, 100, 30], 'String', 'Translate Z', 'FontSize', 12);
            obj.translateZSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 40, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            % Listeners for real-time updates
            addlistener(obj.translateXSlider, 'Value', 'PreSet', @(src, event) updateTranslation());
            addlistener(obj.translateYSlider, 'Value', 'PreSet', @(src, event) updateTranslation());
            addlistener(obj.translateZSlider, 'Value', 'PreSet', @(src, event) updateTranslation());
            
            openFeatureGUI();
            
            % Function to open the secondary features window
            function openFeatureGUI()
 
                fig2 = figure('Name', 'PR2 Features', 'NumberTitle', 'off', ...
                    'Position', [100, 100, 600, 300]); 
                
                % Button to extract joint values for the right arm 
                uicontrol('Style', 'pushbutton', 'Parent', fig2, ...
                    'String', 'Extract Q Right', 'FontSize', 12, 'FontWeight', 'bold', ...
                    'Position', [20, 200, 150, 50], 'Callback', @(src, event) extractQRight());
                
                obj.qRightTextBox = uicontrol('Style', 'edit', 'Parent', fig2, ...
                    'Position', [180, 200, 350, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0], 'BackgroundColor', [1 1 1], ...
                    'HorizontalAlignment', 'left');
                
                % Button to extract joint values for the left arm
                uicontrol('Style', 'pushbutton', 'Parent', fig2, ...
                    'String', 'Extract Q Left', 'FontSize', 12, 'FontWeight', 'bold', ...
                    'Position', [20, 130, 150, 50], 'Callback', @(src, event) extractQLeft());
                
                obj.qLeftTextBox = uicontrol('Style', 'edit', 'Parent', fig2, ...
                    'Position', [180, 130, 350, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0], 'BackgroundColor', [1 1 1], ...
                    'HorizontalAlignment', 'left');
            end
            
            % Function to update translation of pr2
            function updateTranslation()

                tx = obj.translateXSlider.Value;
                ty = obj.translateYSlider.Value;
                tz = obj.translateZSlider.Value;
                
                heightAdjustmentRight = transl(0, 0.22, -0.71);
                rotationAdjustmentRight = trotx(pi);
                baseTrRight = transl(tx, ty, tz);
                
                heightAdjustmentLeft = transl(0, -0.22, -0.71);
                rotationAdjustmentLeft = trotx(pi);
                baseTrLeft = transl(tx, ty, tz);
                
                obj.pr2Left.model.base = baseTrLeft * rotationAdjustmentLeft * heightAdjustmentLeft;
                obj.pr2Right.model.base = baseTrRight * rotationAdjustmentRight * heightAdjustmentRight;
                
                obj.pr2Left.model.animate(obj.pr2Left.model.getpos());
                obj.pr2Right.model.animate(obj.pr2Right.model.getpos());
                
                drawnow(); 
                
                checkCollision(obj.pr2Left.model, obj.pr2Left.model.getpos());
                checkCollision(obj.pr2Right.model, obj.pr2Right.model.getpos());
            end
            
            % Update function for right arm with collision checking
            function updateRobotRight(index)
                qRight = zeros(1, numJointsR);
                for j = 1:numJointsR
                    qRight(j) = obj.rightSliders(j).Value;
                end
        
                set(obj.rightSliderValueText(index), 'String', num2str(qRight(index), '%.2f'));
                obj.pr2Right.model.animate(qRight);
                
                checkCollision(obj.pr2Right.model, qRight);
            end
            
            % Update function for left arm with collision checking
            function updateRobotLeft(index)
                qLeft = zeros(1, numJointsL);
                for j = 1:numJointsL
                    qLeft(j) = obj.leftSliders(j).Value;
                end
                set(obj.leftSliderValueText(index), 'String', num2str(qLeft(index), '%.2f'));
                obj.pr2Left.model.animate(qLeft);
                
                checkCollision(obj.pr2Left.model, qLeft);
            end
            
            % Function to extract Right Qmat
            function extractQRight(~)
               
                qRight = zeros(1, numJointsR);
                for j = numJointsR:-1:1
                    qRight(j) = round(obj.rightSliders(8 - j).Value, 2); 
                end
                
                qRightReversed = qRight(end:-1:1);
                
                qRightStr = mat2str(qRightReversed);
                set(obj.qRightTextBox, 'String', qRightStr);
            end
            
            % Function to extract Left Qmat
            function extractQLeft(~)

                qLeft = zeros(1, numJointsL);
                for j = numJointsL:-1:1
                    qLeft(j) = round(obj.leftSliders(8 - j).Value, 2); 
                end
                
                qLeftReversed = qLeft(end:-1:1);
                
                qLeftStr = mat2str(qLeftReversed); 
                set(obj.qLeftTextBox, 'String', qLeftStr);
            end
            
            function checkCollision(robot, qMatrix)
       
                tr = GetLinkPoses(qMatrix, robot);
             
                for i = 1:size(tr, 3) - 1
                    for faceIndex = 1:size(faces, 1)
                        vertOnPlane = vertex(faces(faceIndex, 1), :);
                        [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i + 1)');
                        if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex, :), :))
                            plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                            disp('Collision Detected');
                        end
                    end
                end
            end
            
  
            function [transforms] = GetLinkPoses(q, robot)
                links = robot.links;
                transforms = zeros(4, 4, length(links) + 1);
                transforms(:, :, 1) = robot.base;
                for i = 1:length(links)
                    L = links(i);
                    current_transform = transforms(:, :, i);
                    current_transform = current_transform * trotz(q(i) + L.offset) * ...
                        transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
                    transforms(:, :, i + 1) = current_transform;
                end
            end
            
            % Function to check if a point is inside a triangle
            function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
                u = triangleVerts(2, :) - triangleVerts(1, :);
                v = triangleVerts(3, :) - triangleVerts(1, :);
                
                uu = dot(u, u);
                uv = dot(u, v);
                vv = dot(v, v);
                
                w = intersectP - triangleVerts(1, :);
                wu = dot(w, u);
                wv = dot(w, v);
                
                D = uv * uv - uu * vv;
                
                % Get and test parametric coordinates (s and t)
                s = (uv * wv - vv * wu) / D;
                if (s < 0.0 || s > 1.0)
                    result = 0;
                    return;
                end
                
                t = (uv * wu - uu * wv) / D;
                if (t < 0.0 || (s + t) > 1.0)
                    result = 0;
                    return;
                end
                
                result = 1;
            end
        end
    end
end