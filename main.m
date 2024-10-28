clc; clf; clear;

grid on; hold on;
axis([-1.8 3.5 -2.5 2.5 0 2.5]);

view(90, 10);  
% Load environment
eStop = serialport('COM4', 9600);

banana_h = PlaceObject('plyFiles/Scenery/Banana.ply',[1.0, 0.5, 0.82]);
env = EnvironmentLoader();
laser = pr2Laser();
gripperLeftState = 'closed';
gripperRightState = 'closed';
global TM5GripperState;
TM5GripperState = 'close';

gamePadControl = false;

gripperRightStateKnife = 'closed';
robot = robotControl(env);

dt = 0.05;                    % Time step
steps = 50;                   % Number of steps
lambda = 0.01;                % Damping factor for singularity handling
epsilon = 0.00000001;             % Threshold for detecting singularities

sensor_position = [0.35, 0, 1.35]; 
centerPoint = [1.0, 0, 0.82]; 
% For IBVS Variables - Start
pStar = [262 762 762 262;  % uTL, uTR, uBL, uBR
         762 762 262 262];  % vTL, vTR, vBL, vBR

%     P4    P3   P2    P1
P1 = [0.9 , 1.1, 1.1, 0.9; ... X
      0.4, 0.4, 0.6, 0.6; ... Y
      0.82, 0.82, 0.82, 0.82]; ... Z

fps = 25;
lambda = 0.8;
deltaTime = 0.05;
steps = 50;
epsilon = 0.1;
q0_IBVS = deg2rad([90; 22.5; -105; 0; -90; 0]); % Scanning Pose - Predefined
% For IBVS Variables - End

sensor_position = [0.125, 0, 0.9]; 
centerPoint = [0.85, 0, 0.42]; 
xLength = centerPoint(1) - sensor_position(1);
zLength = sensor_position(3) - centerPoint(3);
radii = [0.08, 0.15, 0.06]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);

numSteps = 30;

% Torso Go Up
qStart = [0 0 0];
startTr = robot.env.pr2Base.model.fkine(qStart);
% Rotate End Effector by 90d around Z
tm5Pos2 = tm5Pos1 * trotz(pi/2);
robot.animateTM5(currentQ,tm5Pos1,tm5Pos2,steps,eStop);

% Open Gripper Here
robot.animateTM5GripperMotion('open',eStop);

tm5Pos3 = [tm5Pos2(1:3,1:3), [tm5Pos2(1,4), tm5Pos2(2,4)-0.08, tm5Pos2(3,4)-0.26]'; zeros(1, 3), 1];

% Go down to banana
robot.animateTM5RMRC(currentQ, tm5Pos2, tm5Pos3, steps, deltaTime, epsilon, eStop); 

% Close Gripper Here
robot.animateTM5GripperMotion('closed',eStop);

% Delete Banana
delete(banana_h)

%Arm positions here 

%Posh
%r = [-0.100 -0.680 1.146]
%l = [-0.100  0.680 1.146]
%%%%%%%%%%%%%%%%%% USING transl
%pr2RightPosH = transl(0.250, -0.680, 1.146);
%pr2leftPosH = transl(0.250, 0.680, 1.146);

%Pos1
%pr2RightPos1 = transl(0.250, -1.001, 0.825);
%pr2leftPos1 = transl(0.250, 1.001, 0.825);

%Pos 2 is Pos 1 with Z adjustment
%pr2RightPos1h = transl(0.250, -1.001, 1.125);
%pr2RightPos2 = transl(0.6, -0.6, 0.87) * troty(pi/2);
%pr2leftPos2 = transl(0.250, 1.001, 1.125);
% Base positions for PR2 arms in the form [eye(3), translation; zeros(1, 3), 1]

%%%%%%%%%%%%%%%%%%%%%% Using matrix form
pr2RightPosH = [eye(3), [0.250; -0.680; 1.146]; zeros(1, 3), 1];
pr2LeftPosH = [eye(3), [0.250; 0.680; 1.146]; zeros(1, 3), 1];

% Position 1 in the form [eye(3), translation; zeros(1, 3), 1]
pr2RightPos1 = [eye(3), [0.250; -1.001; 0.825]; zeros(1, 3), 1];
pr2LeftPos1 = [eye(3), [0.250; 1.001; 0.825]; zeros(1, 3), 1];

% Position 2 (with Z adjustment for right arm) in the form [eye(3), translation; zeros(1, 3), 1]
pr2RightPos1h = [eye(3), [0.250; -1.001; 1.125]; zeros(1, 3), 1];

% Rotation matrix
function Ry = customTroty(theta) 
theta_y = theta;
Ry = [cos(theta_y), 0, sin(theta_y); 
       0, 1, 0;
      -sin(theta_y), 0, cos(theta_y)];
end
% Position 2 with rotation (right arm) in the form [R, translation; zeros(1, 3), 1]
pr2RightPos2t = [eye(3), [0.600; -0.600; 0.870]; zeros(1, 3), 1];
pr2RightPos2 = pr2RightPos2t * troty(pi/2);

% Call functions from robotControl to move pr2 
% Move arm -> base up -> move arm -> base down
robot.animatePR2ArmsAndGrippers(pr2RightPosH, pr2RightPos1, pr2LeftPosH, pr2LeftPos1, numSteps, eStop);
%robot.PR2BothGrippersOpen(numSteps, eStop);
robot.animatePR2Base(startTr, endTr, numSteps, eStop); %Move torso up
% Go up 
tm5Pos4 = [tm5Pos3(1:3,1:3), [tm5Pos3(1,4), tm5Pos3(2,4)+0.08, tm5Pos3(3,4)+0.26]'; zeros(1, 3), 1];
robot.animateTM5WithBananaRMRC(currentQ, tm5Pos3, tm5Pos4, steps, deltaTime, epsilon, eStop); 

tm5Pos5 = [tm5Pos4(1:3,1:3), [tm5Pos4(1,4), tm5Pos4(2,4)-0.6, tm5Pos4(3,4)-0.2]'; zeros(1, 3), 1];
robot.animateTM5WithBananaRMRC(currentQ, tm5Pos4, tm5Pos5, steps, deltaTime, epsilon, eStop);

tm5Pos5Q = robot.env.tm5700.model.getpos(); 
tm5Pos6 = robot.env.tm5700.model.fkine(tm5Pos5Q).T;
tm5Pos7 = tm5Pos6 * trotz(-pi/2);
% zeros(1,6) ~ currentQ
robot.animateTM5WithBanana(currentQ,tm5Pos6,tm5Pos7,steps,eStop);

robot.animateTM5GripperMotion('open',eStop);

robot.env.tm5700Banana.model.base = transl(1.0, 0, 0.82)* troty(pi) * trotz(pi/2) * trotx(pi);
robot.env.tm5700Banana.model.animate(0)
drawnow();

tm5Pos7Q = robot.env.tm5700.model.getpos(); 
tm5Pos8 = robot.env.tm5700.model.fkine(tm5Pos7Q).T;
tm5Pos9 = robot.env.tm5700.model.fkine(q0_IBVS).T;

robot.animateTM5(tm5Pos7Q,tm5Pos8,tm5Pos9,steps,eStop);
% End IBVS Procedure

% Start gamePadControl
%% For controlling robot using HID game controller
if (gamePadControl)
    gamepad = env.teensyGamepad; %#ok<UNRCH>
    kV = 0.2;
    kW = 1.0;
    duration = 300;
    dt = 0.1;
    lambda_gamepad = 0.1;

    % Change this value to swap between control modes
    controlPreference = 0;

    if controlPreference == 1
        disp('Selected Control:  EndEffector')
        savedQ = robot.gamepadEndEffectorFrameControl(gamepad, lambda_gamepad, kV, kW, duration, dt, 'TM5', eStop);
        disp('Control session complete. Saved configurations:');
        disp(savedQ);
    elseif controlPreference == 2
        disp('Selected Control:  WorldFrame')
        savedQ = robot.gamepadWorldFrameControl(gamepad, lambda_gamepad, kV, kW, duration, dt, 'TM5', eStop);
        disp('Control session complete. Saved configurations:');
        disp(savedQ);
    end
end
% End gamePadControl

fprintf('First Coordinate: (%.2f, %.2f, %.2f)\n', firstCoord(1), firstCoord(2), firstCoord(3));
fprintf('Last Coordinate: (%.2f, %.2f, %.2f)\n', lastCoord(1), lastCoord(2), lastCoord(3));
%Create T-matrices for the banana start and end
bananaR = [eye(3), [firstCoord(1); firstCoord(2); firstCoord(3)]; zeros(1, 3), 1];
bananaRTr = bananaR * transl(0, 0, 1);
bananaRTr1 = bananaR * transl(-0.2, 0, 0.3) * troty(pi/2);
bananaL = transl(lastCoord) * transl(-0.2, 0, 0) * troty(pi/2);

pr2RightPos3 = [eye(3), [1.071; -0.180; 1.125]; zeros(1, 3), 1];
pr2RightPos4 = [eye(3), [0.875; -0.180; 1.125]; zeros(1, 3), 1];
%Animating moves the left arm to the left end of the banana but in an undesirable position, use waypoint animations
pr2LeftWayPosStart = env.pr2LeftArm.model.ikcon(pr2LeftPos1);
robot.PR2RightGripperOpen(50, eStop);
robot.animatePR2RightArmsAndGrippers(pr2RightPos1h, pr2RightPos2, numSteps, eStop);
%robot.animatePR2SingleRightJoint(5, 180, numSteps);
%robot.animatePR2SingleRightJoint(6, -90, numSteps);
robot.PR2GrabKnife(numSteps, eStop);
deletePlyObject(fakeKnife);
robot.animateRightPR2ArmsAndGrippersWithKnife(pr2RightPos2, pr2RightPos1h, numSteps, eStop);
robot.animateRightPR2ArmsAndGrippersWithKnife(pr2RightPos1h, bananaRTr, numSteps, eStop);
robot.animateRightPR2ArmsAndGrippersWithKnife(bananaRTr, pr2RightPos3, numSteps, eStop);
%robot.animatePR2RightArmRMRC(pr2RightPos3, pr2RightPos4, 100, dt, lambda, epsilon, eStop);
%robot.animatePR2RightArmRMRC(pr2RightPos4, pr2RightPos3, 100, dt, lambda, epsilon, eStop);
robot.animatepr2RightHybridControl(pr2RightPos3, pr2RightPos4, 100, dt, lambda, epsilon, eStop);
currentQ = env.pr2RightArm.model.getpos();
disp('Current joint values for PR2 right arm:');
disp(currentQ);
currentQDeg = rad2deg(currentQ);
disp(currentQDeg);
%{
   -0.3234    1.0360    0.6692    1.4965   -0.7629    0.8827   -2.5361


  -18.5268   59.3585   38.3439   85.7431  -43.7101   50.5728 -145.3072
%}
  %W1 
q1 =  deg2rad([-90   125   0     0     0   0   0]);
q2 =  deg2rad([-85.6 125   0     0     0   0   0]);
q3 =  deg2rad([-85.6 125   0     8.6   0   0   0]);
q4 =  deg2rad([-80.2 125   0    13.8   0   0   0]);
q5 =  deg2rad([-71.8 125   0    22.2   0   0   0]);
q6 =  deg2rad([-67.2 125   0    35.9   0   0   0]);
q7 =  deg2rad([-26.6 125 -13.9  35.9   0   0   0]);
q8 =  deg2rad([-26.6 125 -53.8  35.9   0   0   0]);
q9 =  deg2rad([-15.1 125 -53.8  74.3   0   0   0]);
q10 = deg2rad([-15.1 137 -53.8  74.3   0   0   0]);
q11 = deg2rad([-15.1 140 -53.8  74.3   0   0   0]);
q12 = deg2rad([-15.1 140 -211   74.3   0   0   0]);
q13 = deg2rad([-15.1 140 -211   74.3 -45   0   0]);
q14 = deg2rad([-15.1 140 -211   74.3 -90   0   0]);
q15 = deg2rad([-15.1 140 -211   74.3 -180  0   0]);
q16 = deg2rad([-15.1 140 -211   74.3 -180  0   0]);
q17 = deg2rad([-15.1 140 -211   74.3 -180 20   0]);
q18 = deg2rad([-15.1 140 -211   74.3 -180 45   0]);
q19 = deg2rad([-15.1 140 -211   74.3 -180 53.3 0]);
q20 = deg2rad([-5.18 140 -180   74.3 -180 53.3 0]);
q21 = deg2rad([-5.18 140 -180   90   -180 53.3 0]);
q22 = deg2rad([-5.18 131 -180   82.1 -180 53.3 0]);
q23 = deg2rad([-2.50  97 -180   75.5 -180 53.3 0]);
q24 = currentQ;
q25 = [-0.32 1.04 0.44 1.49 -0.77 0.88 -2.55];
q26 = [0.17 1.04 0.44 1.49 -0.77 0.88 -2.55];
q27 = [0.17 1.04 -0.02 1.49 -0.77 0.88 -2.55];
q28 = [0.17 1.04 -0.5 1.49 -0.77 0.88 -2.55];
q29 = [0.17 1.04 -1.46 1.49 -0.77 0.88 -2.55];
q30 = [0.17 1.04 -1.46 1.49 -0.77 0.88 -2.55];
q31 = [0.17 2.44 -2.49 1.49 -0.77 1.72 -2.55];
q32 = [0.17 2.44 -2.49 1.49 -2.63 1.72 -2.55];
q33 = [0.17 2.44 -2.49 1.49 -2.63 1.72 -2.55];
q34 = [0.17 2.44 -2.49 1.49 -2.63 1.72 -2.55];
q35 = [0.17 2.44 -2.49 1.49 -2.63 1.72 -2.55];
q36 = [0.17 2.44 -2.49 1.49 -2.63 1.72 -2.55];
q37 = [-0.2 2.44 -3.18 1.49 -2.63 0.98 -2.55];
q38 = [-0.2 2.39 -3.18 1.49 -2.63 0.68 -2.55];
q39 = [-0.13 2.39 -3.18 2.05 -2.63 1.31 -2.55];
q40 = [-0.38 2.39 -3.14 2.05 -2.63 1.31 -2.55];
q41 = [-0.38 2.27 -3.12 1.43 -2.63 1.31 -2.55];
q42 = [-0.38 1.93 -3.2 2.06 -2.63 1.73 -2.55];
q43 = [-0.38 1.93 -3.2 2.06 -2.63 1.73 -2.94];
q44 = [-0.38 1.77 -3.2 1.86 -2.63 1.66 -2.94];
q45 = [-0.38 1.68 -3.2 1.86 -2.63 1.66 -2.94];
q46 = [-0.38 1.9 -3.2 1.86 -2.63 1.65 -2.94];
q47 = [-0.43 1.9 -3.2 1.86 -2.63 1.65 -2.94];
q48 = [-0.43 1.67 -3.2 1.86 -2.63 1.65 -2.94];
q49 = [-0.43 1.9 -3.2 1.86 -2.63 1.65 -2.94];
q50 = [-0.49 1.9 -3.2 1.86 -2.63 1.65 -2.94];
qWpmat = [pr2LeftWayPosStart; q1; q2; q3; q4; q5; q6; q7; q8; q9; q10; q11; q12; q13; q14; q15; q16; q17; q18; q19; q20; q21; q22; q23];
qWpmat1 = [currentQ; q25; q26; q27; q28; q29; q30; q31; q32; q33; q34; q35; q36; q37; q38; q39; q40; q41; q42; q43; q44; q45];
robot.animatePR2RightArmsAndGrippersWithWaypointsKnife(qWpmat1, 150, eStop);
robot.animatePR2LeftArmsAndGrippersWithWaypoints(qWpmat, 150, eStop);
%robot.animatepr2RightHybridControl(pr2RightPos4, pr2RightPos3, 100, dt, lambda, epsilon, eStop);
%robot.animatepr2RightRMRCNullSpace(pr2RightPos3, pr2RightPos4, 100, dt, lambda, epsilon, 0);
%robot.animatepr2RightRMRCNullSpace(pr2RightPos4, pr2RightPos3, 100, dt, lambda, epsilon, 0.4);
%robot.animatePR2SingleRightJoint(1, -10, 30);
%robot.animatePR2SingleRightJoint(6, -45, 30);


%Function to delete plyFiles because not using robotBaseClass
function deletePlyObject(objectHandle)

    if ishandle(objectHandle)
        delete(objectHandle);
    else
        warning('The provided handle is not valid or the object does not exist.');
    end
end