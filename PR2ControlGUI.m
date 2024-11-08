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
            
            % Initialize the environment loader and use its arms
            env = EnvironmentLoader();  % This loads the PR2 environment, including arms
            
            % Assign the right and left arms from the environment loade
            obj.pr2BaseTr = env.pr2Base.model;
            obj.pr2Right = env.pr2RightArm.model;
            obj.pr2Left = env.pr2LeftArm.model;
            obj.tm5 = env.tm5700.model;
            obj.tm5GripperL = env.tm5700GripperL.model;
            obj.tm5GripperR = env.tm5700GripperR.model;
            obj.gamepad = env.teensyGamepad;
            
            qb = [0 0 0];
            q = [0 2.3 0 0 0 0 0];
            qTM5 = [0 0 0 0 0 0];
            obj.pr2Right.animate(q);
            obj.pr2Left.animate(q);
            obj.pr2BaseTr.animate(qb);
            obj.tm5.animate(qTM5);
            
            % Retrieve the joint limits (qlims) for both arms
            qlimR = obj.pr2Right.qlim;  
            qlimL = obj.pr2Left.qlim;   
            qLimB = obj.pr2BaseTr.qlim;
            qLimTM5 = obj.tm5.qlim;
            
            % Define colors for the UI
            hexColor = '#8F938D';
            rgbColor = hex2rgb(hexColor);
            textColor = hex2rgb('#000000');
            
            % Cube for visual reference in the environment
            lower = [0.7, -1.3, 0];
            upper = [1.45, 1.3, 0.90];
            plotOptions.plotFaces = true;
            [vertex, faces, faceNormals] = RectangularPrism(lower, upper, plotOptions);
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
            
            % Create translation control panel for base movement
            translationPanel = uipanel('Title', 'Translation Control', 'FontSize', 12, ...
                'FontName', 'Arial', ...
                'FontWeight', 'bold', ...
                'FontAngle', 'normal', ...
                'Position', [0.8, 0, 0.9, 0.20], ...
                'BackgroundColor', rgbColor, ...
                'BorderType', 'none');
            
            % Sliders for X, Y, Z translations with corresponding value displays
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 160, 100, 30], 'String', 'Translate X', 'FontSize', 12);
            obj.translateXSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 140, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            % Create a text box to display the value of X slider
            obj.translateXValueText = uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [120, 140, 50, 30], 'String', '0', 'FontSize', 12, ...
                'BackgroundColor', [1 1 1]);
            
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 110, 100, 30], 'String', 'Translate Y', 'FontSize', 12);
            obj.translateYSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 90, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            % Create a text box to display the value of Y slider
            obj.translateYValueText = uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [120, 90, 50, 30], 'String', '0', 'FontSize', 12, ...
                'BackgroundColor', [1 1 1]);
            
            uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [10, 60, 100, 30], 'String', 'Translate Z', 'FontSize', 12);
            obj.translateZSlider = uicontrol('Style', 'slider', 'Parent', translationPanel, ...
                'Min', -1, 'Max', 1, 'Value', 0, 'Position', [10, 40, 100, 30], ...
                'Sliderstep', [0.01, 0.1]);
            
            % Create a text box to display the value of Z slider
            obj.translateZValueText = uicontrol('Style', 'text', 'Parent', translationPanel, ...
                'Position', [120, 40, 50, 30], 'String', '0', 'FontSize', 12, ...
                'BackgroundColor', [1 1 1]);
            
            % Listeners for real-time updates of translation
            addlistener(obj.translateXSlider, 'Value', 'PostSet', @(src, event) updateTranslation());
            addlistener(obj.translateYSlider, 'Value', 'PostSet', @(src, event) updateTranslation());
            addlistener(obj.translateZSlider, 'Value', 'PostSet', @(src, event) updateTranslation());

            
            
            openFeatureGUI();
            openGamepadGUI();
           

            function openGamepadGUI()
                fig3 = figure('Name','TM5 Gamepad Control GUI', 'NumberTitle', 'off', ...
                    'Position', [100, 100, 600, 350]);

                % Display whether or not gamepad is in use 
                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'Gamepad Active: ', ...
                'Position', [20, 250, 200, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                obj.statusTextBox = uicontrol('Style', 'text', 'Parent', fig3, ...
                'Position', [150, 250, 50, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'B8 - Exit Gamepad Control', ...
                'Position', [230, 280, 400, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');
                
                % Push button to detect selection
                uicontrol('Style', 'pushbutton', 'Parent', fig3, 'String','Gamepad Jacobe Control', ...
                'Position', [20 165 200 50], 'FontSize', 12, 'FontWeight', 'bold', ...
                'Callback', @(src, event) jacobeGamepad());
                
                uicontrol('Style', 'pushbutton', 'Parent', fig3, 'String','Gamepad Jacob0 Control', ...
                'Position', [20 100 200 50], 'FontSize', 12, 'FontWeight', 'bold', ...
                'Callback', @(src, event) jacob0Gamepad());
                
                % Show what mode is selected jacobe or jacob0
                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'Mode: ', ...
                'Position', [230, 120, 200, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                obj.selectedModeTextBox = uicontrol('Style', 'text', 'Parent', fig3, ...
                'Position', [280, 120, 200, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                % Reset Q back to default text
                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'B1 - Go back to starting position', ...
                'Position', [230, 310, 400, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');
                
                % Show Current Q after pressing button 3 
                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'B3 - Extract Q Values and Pose', ...
                'Position', [230, 250, 400, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'TM5 Q Values (1-6):', ...
                'Position', [230, 220, 350, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                obj.qTextBox = uicontrol('Style', 'edit', 'Parent', fig3, ...
                    'Position', [230, 165, 350, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0], 'BackgroundColor', [1 1 1], ...
                    'HorizontalAlignment', 'left');

                % Show End Effector Position and Rotation as a 4x4 
                uicontrol('Style', 'text', 'Parent', fig3, 'String', 'Pose: ', ...
                'Position', [230, 60, 200, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ForegroundColor', [0 0 0], 'BackgroundColor', [0.94 0.94 0.94], ...
                'HorizontalAlignment', 'left');

                obj.poseTextBox = uicontrol('Style', 'edit', 'Parent', fig3, ...
                    'Position', [300, 20, 280, 100], 'FontSize', 12, 'FontWeight', 'bold', ...
                    'ForegroundColor', [0 0 0], 'BackgroundColor', [1 1 1], ...
                    'Max', 2, 'HorizontalAlignment', 'left');

            end

             % For selecting either jacobe or jacob0 for the gamepad
            function jacobeGamepad()
                disp('Selected Control for TM5: Jacobe');
                set(obj.selectedModeTextBox, 'String', 'Jacobe');
                set(obj.statusTextBox, 'String', 'Yes');
                runGamepadControl(1)
            end

            function jacob0Gamepad()
                disp('Selected Control for TM5: Jacob0');
                set(obj.selectedModeTextBox, 'String', 'Jacob0');
                set(obj.statusTextBox, 'String', 'Yes');
                runGamepadControl(0)
            end

            % From gamepadtest, modified for gui implementation
            function runGamepadControl(jacobianSelection)
                % Gripper offset
                tm5GripperOffset = troty(-pi/2) * trotx(-pi/2) * transl(0.01, 0, 0);

                % Values from gamepadtest
                kV = 0.2;
                kW = 1.0;
                dt = 0.1;
                lambda = 0.1;
                stickDriftThreshold = 0.1; % Tested this value, works good
                qDotLim = [-0.5 0.5]; % Define joint speed limits
                
                qTM5 = obj.tm5.getpos();

                while true
                    % read joystick data
                    [axes, buttons, ~] = read(obj.gamepad);

                    % map joystick data to linear and angular vel
                    vx = kV * gamepadRemoveStickDrift(axes(1), stickDriftThreshold);
                    vy = kV * gamepadRemoveStickDrift(axes(2), stickDriftThreshold);
                    vz = kV * (buttons(4) - buttons(2));
                    wx = kW * gamepadRemoveStickDrift(axes(3), stickDriftThreshold);
                    wy = kW * gamepadRemoveStickDrift(axes(4), stickDriftThreshold);
                    wz = kW*(buttons(9) - buttons(7));
                    
                    % Combine to a single velocity vector
                    x = [vx; vy; vz; wx; wy; wz];
                    
                    % Check which button was pressed to select jacobian
                    if (jacobianSelection)
                        J = obj.tm5.jacobe(qTM5);
                    else
                        J = obj.tm5.jacob0(qTM5);
                    end
                    
                    % Calculate inverse Jacobian with predefined lambda
                    invJ = inv((J*J')+lambda^2*eye(6))*J';
                    
                    % Compute q velocities 
                    qDot = invJ * x;
                    
                    % Check if calculated q exceeds qlims
                    for i = 1:length(qTM5)
                       if qTM5(i) + dt * qDot(i) < qLimTM5(i,1)
                            qDot(i) = 0;  % Stop if it exceeds lower bound
                       elseif qTM5(i) + dt * qDot(i) > qLimTM5(i,2)
                            qDot(i) = 0;  % Stop if it exceeds upper bound
                        end
                    end
                    
                    % Reset robot position back to zeros(1,6) (Default)
                    if (buttons(1))
                        qTM5 = zeros(1,6);
                        obj.tm5.animate(qTM5);
                    end
                    
                    % Show current Q
                    if (buttons(3))
                        viewFriendlyQTM5 = sprintf('%.4f %.4f %.4f %.4f %.4f %.4f', qTM5);
                        tr = obj.tm5.fkine(qTM5);
                        % tr as a string
                        trStr = sprintf('%.4f %.4f %.4f %.4f\n%.4f %.4f %.4f %.4f\n%.4f %.4f %.4f %.4f\n%.4f %.4f %.4f %.4f', tr.T');

                        %disp(['Saved Configuration ', num2str(qCount), ': ', viewFriendlyQTM5]);
                        % Update the textbox with the latest configuration
                        set(obj.qTextBox, 'String', viewFriendlyQTM5);
                        set(obj.poseTextBox, 'String', trStr);
                    end
                    
                    % Exit gamepad control
                    if (buttons(8))
                        set(obj.statusTextBox, 'String', 'No');
                        disp('Exiting Gamepad Control ...')
                        break;
                    end
                    
                    % Limit qDot to range of -0.5 to 0.5
                    qDot = max(min(qDot,qDotLim(2)), qDotLim(1));
                    
                    % Update q value for tm5
                    qTM5 = qTM5 + (qDot' * dt);
                    
                    % Animate tm5 and gripper 
                    obj.tm5.animate(qTM5);
                    endEffector = obj.tm5.fkine(qTM5);
                    obj.tm5GripperL.base = endEffector.T * tm5GripperOffset;
                    obj.tm5GripperR.base = endEffector.T * tm5GripperOffset;

                    % Just to have the gripper go with the end effector
                    % No open/close feature for GUI
                    obj.tm5GripperL.animate([0 0]);
                    obj.tm5GripperR.animate([0 0]);
                    drawnow();
                end
            end
            
            function value = gamepadRemoveStickDrift(inputValue, stickDriftThreshold)
                % Create deadzone to prevent stick drift
                if abs(inputValue) < stickDriftThreshold
                    value = 0;
                else
                    value = sign(inputValue);
                end
            end

            
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
            
            % Function to update translation of the robot
            function updateTranslation()
                % Get slider values for translation in x, y, z
                tx = obj.translateXSlider.Value;
                ty = obj.translateYSlider.Value;
                tz = obj.translateZSlider.Value;

                set(obj.translateXValueText, 'String', num2str(tx, '%.2f'));
                set(obj.translateYValueText, 'String', num2str(ty, '%.2f'));
                set(obj.translateZValueText, 'String', num2str(tz, '%.2f'));
            
                % Create translation matrix for the base
                baseTr = transl(tx, ty, tz);
            
                % Adjust for the height and orientation of the PR2 base (if needed)
                heightAdjustment = transl(0, 0, -0.71);  % Adjust the base's height
                rotationAdjustment = trotx(pi);          % Adjust the base's orientation
            
                % Apply translation and rotation to the base of the robot
                newBaseTransform = baseTr * rotationAdjustment * heightAdjustment;
            
                % Assign the new transformation to the robot base
                obj.pr2BaseTr.base = newBaseTransform;
            
                % Get the current joint positions for both arms
                qRight = obj.pr2Right.getpos();
                qLeft = obj.pr2Left.getpos();
            
                % Update the base transformation for the arms without recalculating joint angles
                obj.pr2Right.base = newBaseTransform * transl(0, 0.22, 0.2);  % Right arm offset from base
                obj.pr2Left.base = newBaseTransform * transl(0, -0.22, 0.2);  % Left arm offset from base
            
                % Animate the arms with the existing joint positions (no changes in angles)
                obj.pr2Right.animate(qRight);  % Right arm moves with the base
                obj.pr2Left.animate(qLeft);    % Left arm moves with the base
            
                % Animate the base model (if applicable)
                obj.pr2BaseTr.animate(qb);  % Assuming qb contains the base's joint angles
            
                drawnow();  % Ensure immediate update of the graphics
            end
            
            
            
            
            
            % Update function for right arm with collision checking
            function updateRobotRight(index)
                qRight = zeros(1, numJointsR);
                for j = 1:numJointsR
                    qRight(j) = obj.rightSliders(j).Value;
                end
                set(obj.rightSliderValueText(index), 'String', num2str(qRight(index), '%.2f'));
                obj.pr2Right.animate(qRight);
                checkCollision(obj.pr2Right, qRight);
            end
            
            % Update function for left arm with collision checking
            function updateRobotLeft(index)
                qLeft = zeros(1, numJointsL);
                for j = 1:numJointsL
                    qLeft(j) = obj.leftSliders(j).Value;
                end
                set(obj.leftSliderValueText(index), 'String', num2str(qLeft(index), '%.2f'));
                obj.pr2Left.animate(qLeft);
                checkCollision(obj.pr2Left, qLeft);
            end
            
            % Function to extract Right Q-matrix
            function extractQRight(~)
                qRight = zeros(1, numJointsR);
                for j = numJointsR:-1:1
                    qRight(j) = round(obj.rightSliders(8 - j).Value, 2); 
                end
                qRightReversed = qRight(end:-1:1);
                qRightStr = mat2str(qRightReversed);
                set(obj.qRightTextBox, 'String', qRightStr);
            end
            
            % Function to extract Left Q-matrix
            function extractQLeft(~)
                qLeft = zeros(1, numJointsL);
                for j = numJointsL:-1:1
                    qLeft(j) = round(obj.leftSliders(8 - j).Value, 2); 
                end
                qLeftReversed = qLeft(end:-1:1);
                qLeftStr = mat2str(qLeftReversed); 
                set(obj.qLeftTextBox, 'String', qLeftStr);
            end
            
            % Function to check collisions for a robot given its joint configuration
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
            
            % Function to get the poses of all robot links
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
