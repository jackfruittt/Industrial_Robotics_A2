function PR2ControlGUI()
clf;
clc;
grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);

% Number of joints for the right and left arms
numJointsR = 7;
numJointsL = 7;

% Create a structure to hold the PR2 robot objects and sliders
obj.pr2Left = PR2Left();
obj.pr2Right = PR2Right();

% Retrieve the joint limits (qlims) for both arms
qlimR = obj.pr2Right.model.qlim;  % Right arm joint limits
qlimL = obj.pr2Left.model.qlim;   % Left arm joint limits

hexColor = '#8F938D';
rgbColor = hex2rgb(hexColor);
textColor = hex2rgb('#000000');

% Create a panel for the left arm sliders
sliderPanelL = uipanel('Title', 'Left Arm Joint Control', 'FontSize', 12, ...
    'FontSize', 12, ...                      % Set font size
    'FontName', 'Arial', ...                 % Set font type
    'FontWeight', 'bold', ...                % Set text to bold
    'FontAngle', 'normal', ...               % Set text to normal (can be 'italic' if needed)
    'Position', [0 0 0.9 0.20], ... % 90% of figure width, 20% height, 0% from the left side, 0.05% from the bottom
    'BackgroundColor', rgbColor, ...
    'BorderType', 'none');

% Create sliders for the left arm joints
for i = 1:numJointsL
    % Joint label
    uicontrol('Style', 'text', 'Parent', sliderPanelL, ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'ForegroundColor', textColor, ...
        'Backgroundcolor', rgbColor, ...
        'Position', [50, 25 + (i-1)*30, 100, 30], ...
        'String', ['Left Link ' num2str(i)]); 
    defaultValueL = (qlimL(i, 1) + qlimL(i, 2)) / 2;
    
    % Create slider for left arm joint
    obj.leftSliders(i) = uicontrol('Style', 'slider', ...
        'Parent', sliderPanelL, ...
        'Min', qlimL(i, 1), 'Max', qlimL(i, 2), ...
        'Value', defaultValueL, ...
        'Position', [150, 25 + (i-1)*30, 300, 30], ...
        'SliderStep', [0.01 0.1]); % Set step size
    
    addlistener(obj.leftSliders(i), 'Value', 'PostSet', @(src, event) updateRobotLeft(i));

    % Display current slider value
    obj.leftSliderValueText(i) = uicontrol('Style', 'text', ...
        'Parent', sliderPanelL, ...
        'Position', [460, 25 + (i-1)*30, 100, 30], ...
        'String', num2str(defaultValueL, '%.2f'), ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'ForegroundColor', [0 0 0]); % Black text
end

% Create a panel for the right arm sliders
sliderPanelR = uipanel('Title', 'Right Arm Joint Control', ...
    'FontSize', 12, ...                      % Set font size
    'FontName', 'Arial', ...                 % Set font type
    'FontWeight', 'bold', ...                % Set text to bold
    'FontAngle', 'normal', ...               % Set text to normal (can be 'italic' if needed)
    'Position', [0.40, 0, 0.9, 0.20], ...    % Panel position
    'BackgroundColor', rgbColor, ...         % Background color using RGB value
    'BorderType', 'none');                   % Set the border type


% Create sliders for the right arm joints
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

% Create a panel for joystick (translation control) next to the slider panels
translationPanel = uipanel('Title', 'Translation Control', 'FontSize', 12, ...
    'FontSize', 12, ...                      % Set font size
    'FontName', 'Arial', ...                 % Set font type
    'FontWeight', 'bold', ...                % Set text to bold
    'FontAngle', 'normal', ...               % Set text to normal (can be 'italic' if needed)
    'Position', [0.8, 0, 0.9, 0.20], ... % Positioned next to the slider panels
    'BackgroundColor', rgbColor, ...
    'BorderType', 'none');

% Create sliders for X, Y, Z translations
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

% Add listeners for real-time updates
addlistener(obj.translateXSlider, 'Value', 'PreSet', @(src, event) updateTranslation());
addlistener(obj.translateYSlider, 'Value', 'PreSet', @(src, event) updateTranslation());
addlistener(obj.translateZSlider, 'Value', 'PreSet', @(src, event) updateTranslation());

% Function to update translation of both robots
function updateTranslation()
    % Get current slider values for X, Y, Z
    tx = obj.translateXSlider.Value;
    ty = obj.translateYSlider.Value;
    tz = obj.translateZSlider.Value;
    
    heightAdjustmentRight = transl(0, 0.22, -0.71);
    rotationAdjustmentRight = trotx(pi);
    baseTrRight = transl(tx, ty, tz);
    
    heightAdjustmentLeft = transl(0, -0.22, -0.71);
    rotationAdjustmentLeft = trotx(pi);
    baseTrLeft = transl(tx, ty, tz);
    
    % Apply translation to both robots
    obj.pr2Left.model.base = baseTrLeft * rotationAdjustmentLeft * heightAdjustmentLeft;
    obj.pr2Right.model.base = baseTrRight * rotationAdjustmentRight * heightAdjustmentRight;
    
    % Update robot positions in the environment
    obj.pr2Left.model.animate(obj.pr2Left.model.getpos());
    obj.pr2Right.model.animate(obj.pr2Right.model.getpos());

    drawnow(); % Ensure immediate rendering of updates
end

    % Function to update the right arm position
    function updateRobotRight(index)
        qRight = zeros(1, numJointsR);
        for j = 1:numJointsR
            qRight(j) = obj.rightSliders(j).Value;
        end
        
        % Update the slider value display
        set(obj.rightSliderValueText(index), 'String', num2str(qRight(index), '%.2f'));
        
        obj.pr2Right.model.animate(qRight);
    end

    % Function to update the left arm position
    function updateRobotLeft(index)
        qLeft = zeros(1, numJointsL);
        for j = 1:numJointsL
            qLeft(j) = obj.leftSliders(j).Value;
        end
        
        set(obj.leftSliderValueText(index), 'String', num2str(qLeft(index), '%.2f'));
        
        obj.pr2Left.model.animate(qLeft);
    end

    % Function to convert HEX to RGB
    function rgb = hex2rgb(hex)
        hex = char(hex);
        if hex(1) == '#'
            hex = hex(2:end);
        end
        if numel(hex) ~= 6
            error('Input must be a 6-character hex code.');
        end
        % Convert hex string to RGB values in the range [0, 1]
        rgb = [hex2dec(hex(1:2)), hex2dec(hex(3:4)), hex2dec(hex(5:6))] / 255;
    end
end