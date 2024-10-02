clf;
clc;
grid on;
hold on;
axis([-4 4 -2 2 0 2]);
gripperl1 = PR2LeftGripper();
gripperr1 = PR2RightGripper();
gripperl2 = PR2LeftGripper();
gripperr2 = PR2RightGripper();

pr2Left = PR2Left();
pr2Right = PR2Right();

offset = troty(-pi/2) * transl(0.05, 0, 0);

light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;  
material shiny;   
camlight('headlight');
camlight('left');

% Initial joint configuration for the robot arms
qz = [0 pi/2 0 0 0 0 0];

% Loop over multiple steps to animate
for t = 1:50  % Adjust the number of steps as needed
    % Example of varying joint angles over time
    qz(2) = pi/2 + 0.1 * sin(t * 0.1);  % Modify one of the joints for demonstration

    % Update the robot joints
    pr2Left.model.animate(qz);
    pr2Right.model.animate(qz);
    
    % Update the gripper base positions with the offset
    T_leftEndEffector = pr2Left.model.fkine(qz).T;
    T_rightEndEffector = pr2Right.model.fkine(qz).T;
    
    gripperl1.model.base = T_leftEndEffector  * offset;
    gripperr1.model.base = T_leftEndEffector * offset;
    gripperl2.model.base = T_rightEndEffector * offset;
    gripperr2.model.base = T_rightEndEffector * offset;
    
    % Animate the grippers with neutral joint angles [0 0]
    gripperl1.model.animate([0 0]);
    gripperr1.model.animate([0 0]);
    gripperl2.model.animate([0 0]);
    gripperr2.model.animate([0 0]);
    
    pause(0.1);  % Adjust pause to control the speed of the animation
end

    

