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

robot = robotControl(env);
view(90, 10);  

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
radii = [0.1, 0.2, 0.08]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);

% startPos = transl(2, 0.764, 1.092);
% 
% endPos = transl(1.885, -0.555, 0.207);
% 
% numSteps = 30;
% homePosr = transl(0.821, -0.440, 1);
% Tbr = transl(0.594,0.423, 0.647);
% homePosl = transl(0.821, 0, 1);
% Tbl = transl(0.594,-0.863, 0.647);
% 
% Tb3r = transl(0.594, 0.423, 0.947);
% Tb4r = transl(0.821, -0.440, 1.3);
% 
% Tb3l = transl(0.594,-0.863, 0.947);
% Tb4l = transl(0.821, 0, 1.3);
% 
% % Torso Go Up
% qStart = [0 0 0];
% startTr = robot.env.pr2Base.model.fkine(qStart);
% 
% qEnd = [-0.3 0 0];
% endTr = robot.env.pr2Base.model.fkine(qEnd);
% 
% % Torso Go Down
% qStart1 = [-0.3 0 0];
% qEnd1 = [0 0 0];
% 
% startTr1 = robot.env.pr2Base.model.fkine(qStart1);
% endTr1 = robot.env.pr2Base.model.fkine(qEnd1);
% 
% % Call functions from robotControl to move pr2 
% % Move arm -> base up -> move arm -> base down
% robot.animatePR2ArmsAndGrippers(homePosr, Tbr, homePosl, Tbl, numSteps, eStop);
% robot.animatePR2Base(startTr, endTr, numSteps, eStop);
% robot.animatePR2ArmsAndGrippers(Tb3r, Tb4r, Tb3l, Tb4l, numSteps, eStop);
% robot.animatePR2Base(startTr1, endTr1, numSteps, eStop);
% robot.animateRightPR2ArmsAndGrippersWithKnife(Tb4r, Tbr, numSteps, eStop);

%% Start IBVS Procedure
robot.animateTM5GripperMotion('closed',eStop);
robot.animateTM5IBVS(q0_IBVS, pStar, P1, fps, lambda, eStop);

currentQ = robot.env.tm5700.model.getpos(); 
tm5Pos1 = robot.env.tm5700.model.fkine(currentQ).T;

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

function deletePlyObject(objectHandle)

    if ishandle(objectHandle)
        delete(objectHandle);
    else
        warning('The provided handle is not valid or the object does not exist.');
    end
end