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

%Position = [left, bottom, width, height]
sliderPanel = uipanel('Title', 'Joint Control', 'FontSize', 12, ...
    'Position', [0 0.05 0.9 0.20]); % 90% of figure width, 20% height, 0% from the left side, 0.05% from the bottom 
for i = 1:numJointsL
    % Joint label
    uicontrol('Style', 'text', 'Parent', sliderPanel, ...
        'Position', [50, 25 + (i-1)*30, 100, 30], ...
        'String', ['Left Joint ' num2str(i)], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'ForegroundColor', [0 0 1]); % Blue text
    defaultValueL = (qlimL(i, 1) + qlimL(i, 2)) / 2;
    % Create slider for left arm joint with custom colors
    obj.leftSliders(i) = uicontrol('Style', 'slider', ...
        'Parent', sliderPanel, ...
        'Min', qlimL(i, 1), 'Max', qlimL(i, 2), ...
        'Value', defaultValueL, ...
        'Position', [150, 25 + (i-1)*30, 300, 30], ...
        'Callback', @(src, event) updateRobotLeft(i), ...
        'BackgroundColor', [0.8 0.8 0.8], ... % Set background color
        'ForegroundColor', [0.2 0.6 0.2], ... % Set slider color
        'SliderStep', [0.01 0.1]); % Set step size
    
    % Display current slider value
    obj.leftSliderValueText(i) = uicontrol('Style', 'text', ...
        'Parent', sliderPanel, ...
        'Position', [460, 25 + (i-1)*30, 100, 30], ...
        'String', num2str(defaultValueL, '%.2f'), ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'ForegroundColor', [0 0 0]); % Black text
end

sliderPanelR = uipanel('Title', 'Joint Control', 'FontSize', 12, ...
        'Position', [0.35, 0.05, 0.5, 0.20]); t   
for i = 1:numJointsR
    
    uicontrol('Style', 'text', 'Parent', sliderPanelR, ...
        'Position', [50, 25 + (i-1)*30, 100, 30], ...
        'String', ['Right Joint ' num2str(i)], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'ForegroundColor', [0 0 1]); 
    defaultValueR = (qlimR(i, 1) + qlimR(i, 2)) / 2;

    obj.rightSliders(i) = uicontrol('Style', 'slider', ...
        'Parent', sliderPanelR, ...
        'Min', qlimR(i, 1), 'Max', qlimR(i, 2), ...
        'Value', defaultValueR, ...
        'Position', [150, 25 + (i-1)*30, 300, 30], ...
        'Callback', @(src, event) updateRobotRight(i), ...
        'BackgroundColor', [0.8 0.8 0.8], ... 
        'ForegroundColor', [0.2 0.6 0.2], ...
        'SliderStep', [0.01 0.1]); 
    
    obj.rightSliderValueText(i) = uicontrol('Style', 'text', ...
        'Parent', sliderPanelR, ...
        'Position', [460, 25 + (i-1)*30, 100, 30], ...
        'String', num2str(defaultValueR, '%.2f'), ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'ForegroundColor', [0 0 0]); 
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

% Function to animate the left arm
    function animateRobotLeft()
        qLeft = zeros(1, numJointsL);
        for j = 1:numJointsL
            qLeft(j) = obj.leftSliders(j).Value;
        end
        
        % Animate the left robot arm
        obj.pr2Left.model.animate(qLeft);
    end

% Function to animate the right arm
    function animateRobotRight()
        qRight = zeros(1, numJointsR);
        for j = 1:numJointsR
            qRight(j) = obj.rightSliders(j).Value;
        end
        
        % Animate the right robot arm
        obj.pr2Right.model.animate(qRight);
    end
end







