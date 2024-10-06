clear; clc; clf; 
hold on; 
axis equal; 


pr2Left = PR2Left(); 
pr2Right = PR2Right(); 

GUIAPP = pr2gui();

% Load the PR2 robots into the GUI
GUIAPP.getRobot(pr2Left, pr2Right); 

% Main loop for animation
while true
    
    [qLeft, qRight] = GUIAPP.updateQvalues(); % Method to get joint values
    % Animate the PR2 models
    pr2Left.model.animate(qLeft); 
    pr2Right.model.animate(qRight); 

    drawnow(); 
end
