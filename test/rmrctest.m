clc; clf;
grid on; hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(120, 40);

eStop = serial('COM3', 'BaudRate', 9600); 
env = EnvironmentLoader();

%gripperLeftState = 'closed';
%gripperRightState = 'closed';
robot = robotControl(env);
view(90, 10); 

deltaTime = 0.05;
steps = 50;
lamda = 0.01;
epsilon = 0.00001;

robot.env.tm5700.model.teach();

pause

T1 = [eye(3), [1.000, 0.565, 1.672]'; zeros(1, 3), 1]
T2 = [eye(3), [1.122, 0.401, 1.034]'; zeros(1, 3), 1]

robot.animateTM5RMRC(T1, T2, steps, deltaTime, lamda, epsilon, eStop)

%{
waypoints = 5;

robot.env.pr2LeftArm.model.teach();
hold on
robot.env.pr2RightArm.model.teach();
hold off

% leftT1 = [eye(3), [0.721, 0.180, 0.825]'; zeros(1, 3), 1]; % start pos
% leftT2 = [eye(3), [-0.1, 1.001, 0.825]'; zeros(1, 3), 1]; % end pos

% rightT1 = [eye(3), [0.721, -0.180, 0.825]'; zeros(1, 3), 1]; % start pos
% rightT2 = [eye(3), [0.529, -0.708, 0.825]'; zeros(1, 3), 1]; % end pos

leftWaypoints = {[eye(3), [0.721, 0.180, 0.825]'; zeros(1, 3), 1] ...
                ,[eye(3), [-0.1, 1.001, 0.825]'; zeros(1, 3), 1] ...
                ,[eye(3), [-0.1, 0.790, 1.335]'; zeros(1, 3), 1] ...
                ,[eye(3), [0.51, 0.180, 1.335]'; zeros(1, 3), 1] ...
                ,[eye(3), [0.721, 0.180, 0.825]'; zeros(1, 3), 1]};

rightWaypoints = {[eye(3), [0.721, -0.180, 0.825]'; zeros(1, 3), 1] ...
                 ,[eye(3), [0.529, -0.708, 0.825]'; zeros(1, 3), 1] ...
                 ,[eye(3), [0.367, -0.572, 1.335]'; zeros(1, 3), 1] ...
                 ,[eye(3), [0.510, -0.180, 1.335]'; zeros(1, 3), 1] ...
                 ,[eye(3), [0.721, -0.180, 0.825]'; zeros(1, 3), 1]};

for i=1:waypoints-1
    leftT1 = leftWaypoints{i};
    leftT2 = leftWaypoints{i+1};
    rightT1 = rightWaypoints{i};
    rightT2 = rightWaypoints{i+1};
    robot.movePR2ArmsRMRC(leftT1, leftT2, rightT1, rightT2, steps, deltaTime, lambda, epsilon, eStop);
end

% robot.movePR2ArmsRMRC(leftT1, leftT2, rightT1, rightT2, steps, deltaTime, lambda, epsilon, eStop);
% robot.movePR2ArmsRMRC(leftT2, leftT1, rightT2, rightT1, steps, deltaTime, lambda, epsilon, eStop);
% robot.movePR2ArmsRMRC(leftT1, leftT2, rightT1, rightT2, steps, deltaTime, lambda, epsilon, eStop);
% robot.movePR2ArmsRMRC(leftT2, leftT1, rightT2, rightT1, steps, deltaTime, lambda, epsilon, eStop);
% robot.movePR2ArmsRMRC(leftT1, leftT2, rightT1, rightT2, steps, deltaTime, lambda, epsilon, eStop);
% robot.movePR2ArmsRMRC(leftT2, leftT1, rightT2, rightT1, steps, deltaTime, lambda, epsilon, eStop);
%}

