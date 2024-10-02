%{
clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
view(30, 20);  

qzRight = [0 pi/2 0 0 0 0 0]  % Assuming 7-DOF arm
qzLeft = [0 pi/2 0 0 0 0 0]   % Assuming 7-DOF arm

% Set the robot to the initial configuration
env.pr2Right.model.animate(qzRight);
env.pr2Left.model.animate(qzLeft);

% Get the initial end-effector positions using forward kinematics
T_rightEndEffector = env.pr2Right.model.fkine(qzRight).T;  % Transformation matrix of the right end-effector
T_leftEndEffector = env.pr2Left.model.fkine(qzLeft).T;     % Transformation matrix of the left end-effector

% Extract the translation (position) from the transformation matrix
initialPosRight = T_rightEndEffector(1:3, 4);  % Right end-effector position
initialPosLeft = T_leftEndEffector(1:3, 4);    % Left end-effector position

% Display the positions
disp('Initial Right End-Effector Position:');
disp(initialPosRight);

disp('Initial Left End-Effector Position:');
disp(initialPosLeft);
%
numSteps = 50; 
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.5936, -0.8628, 0.6472);

offset = troty(-pi/2) * transl(0.05, 0, 0); 
    
q1 = env.pr2Right.model.ikinem(homePosr);
q2 = env.pr2Right.model.ikinem(Tbr);


qMatrixr = jtraj(q1, q2, numSteps);

velocity = zeros(numSteps,7);
acceleration  = zeros(numSteps,7);
for i = 2:numSteps
    velocity(i,:) = qMatrixr(i,:) - qMatrixr(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

env.pr2Right.model.plot(qMatrixr,'trail','r-')


hold off;
%}

clc;
clf;
hold on;
grid on;
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
leftGripper = PR2LeftGripper();
rightGripper = PR2RightGripper();

function GripperOpen(leftGripper, rightGripper, numSteps)
    % Define the joint l
    qLeftOpen = [deg2rad(18), deg2rad(-18)];  % Fully closed position
    qLeftClose = [0, 0]; 
    qRightOpen = [deg2rad(-18), deg2rad(18)]; 
    qRightClose = [0, 0]; 

    qMatrixLeft = jtraj(qLeftClose, qLeftOpen, numSteps);
    qMatrixRight = jtraj(qRightClose, qRightOpen, numSteps);

    % Animate the gripper opening
    for i = 1:numSteps
        leftGripper.model.animate(qMatrixLeft(i, :));
        rightGripper.model.animate(qMatrixRight(i, :));
        drawnow();
    end
end

function GripperClose(leftGripper, rightGripper, numSteps)
    
    qLeftOpen = [deg2rad(18), deg2rad(-18)]; 
    qLeftClose = [0, 0];
    qRightOpen = [deg2rad(-18), deg2rad(18)];  
    qRightClose = [0, 0];  

    qMatrixLeft = jtraj(qLeftOpen, qLeftClose, numSteps);
    qMatrixRight = jtraj(qRightOpen, qRightClose, numSteps);

    for i = 1:numSteps
        leftGripper.model.animate(qMatrixLeft(i, :));
        rightGripper.model.animate(qMatrixRight(i, :));
        drawnow();
    end
end

GripperOpen(leftGripper, rightGripper, 50);
pause(0.5);
GripperClose(leftGripper, rightGripper, 50);


