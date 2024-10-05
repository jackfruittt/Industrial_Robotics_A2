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

    % Create a panel for the robot visualization
    %modelPanel = uipanel('Title', 'PR2 Model', 'FontSize', 12, ...
    %    'Position', [0.05 0.2 0.6 0.75]); % 60% of figure width, 75% height
    %
    % Create an axes inside the modelPanel to plot the robot
    %modelAxes = axes('Parent', modelPanel);
    %axis(modelAxes, 'equal');
    %hold(modelAxes, 'on');
    
    % Set the current axes for plotting
    %axes(modelAxes);

    % Plot the initial PR2 models in the axes
    %obj.pr2Right.model.plot(zeros(1, numJointsR));
    %obj.pr2Left.model.plot(zeros(1, numJointsL));

    % Sliders and labels for left arm joints
    for i = 1:numJointsL
        % Joint label
        uicontrol('Style', 'text', 'Position', [50, 400 - (i-1)*50, 100, 30], ...
                  'String', ['Left Joint ' num2str(i)]);
        
        % Slider default value as midpoint ofjoint's qlim
        defaultValueL = (qlimL(i, 1) + qlimL(i, 2)) / 2;
        
        % Create slider for left arm joint
        obj.leftSliders(i) = uicontrol('Style', 'slider', ...
            'Min', qlimL(i,1), 'Max', qlimL(i,2), 'Value', defaultValueL, ...
            'Position', [150, 400 - (i-1)*50, 300, 30], ...
            'Callback', @(src, event) updateRobotLeft(i));
        
        % Display current slider value
        obj.leftSliderValueText(i) = uicontrol('Style', 'text', ...
            'Position', [460, 400 - (i-1)*50, 100, 30], ...
            'String', num2str(defaultValueL, '%.2f'));
    end

    % Create animate button for the left arm
    uicontrol('Style', 'pushbutton', 'String', 'Animate Left', ...
              'Position', [250, 50, 80, 40], 'Callback', @(src, event) animateRobotLeft());

    for i = 1:numJointsR

        uicontrol('Style', 'text', 'Position', [550, 400 - (i-1)*50, 100, 30], ...
                  'String', ['Right Joint ' num2str(i)]);
        
        defaultValueR = (qlimR(i, 1) + qlimR(i, 2)) / 2;
        
        obj.rightSliders(i) = uicontrol('Style', 'slider', ...
            'Min', qlimR(i,1), 'Max', qlimR(i,2), 'Value', defaultValueR, ...
            'Position', [650, 400 - (i-1)*50, 300, 30], ...
            'Callback', @(src, event) updateRobotRight(i));
        
        obj.rightSliderValueText(i) = uicontrol('Style', 'text', ...
            'Position', [960, 400 - (i-1)*50, 100, 30], ...
            'String', num2str(defaultValueR, '%.2f'));
    end

    uicontrol('Style', 'pushbutton', 'String', 'Animate Right', ...
              'Position', [500, 50, 80, 40], 'Callback', @(src, event) animateRobotRight());

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




