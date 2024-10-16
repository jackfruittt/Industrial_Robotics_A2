clc;
clf;

grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);  
% Load environment
eStop = serial('COM3', 'BaudRate', 9600);  
fakeKnife = PlaceObject("plyFiles/Scenery/knife.ply", [0.89, -0.59, 0.86]);
env = EnvironmentLoader();
laser = pr2Laser();
gripperLeftState = 'closed';
gripperRightState = 'closed';
gripperRightStateKnife = 'closed';
robot = robotControl(env);

sensor_position = [0.35, 0, 1.35]; 
centerPoint = [1.0, 0, 0.82]; 
xLength = centerPoint(1) - sensor_position(1);
zLength = sensor_position(3) - centerPoint(3);
radii = [0.08, 0.15, 0.06]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);

numSteps = 30;


% Torso Go Up
qStart = [0 0 0];
startTr = robot.env.pr2Base.model.fkine(qStart);

qEnd = [-0.3 0 0];
endTr = robot.env.pr2Base.model.fkine(qEnd);

% Torso Go Down
qStart1 = [-0.3 0 0];
qEnd1 = [0 0 0];

startTr1 = robot.env.pr2Base.model.fkine(qStart1);
endTr1 = robot.env.pr2Base.model.fkine(qEnd1);

%Arm positions here 

%Posh
%r = [-0.100 -0.680 1.146]
%l = [-0.100  0.680 1.146]

pr2RightPosH = transl(0.250, -0.680, 1.146);
pr2leftPosH = transl(0.250, 0.680, 1.146);

%Pos1
pr2RightPos1 = transl(0.250, -1.001, 0.825);
pr2leftPos1 = transl(0.250, 1.001, 0.825);

%Pos 2 is Pos 1 with Z adjustment
pr2RightPos1h = transl(0.250, -1.001, 1.125);
pr2RightPos2 = transl(0.6, -0.6, 0.87) * troty(pi/2);
%pr2leftPos2 = transl(0.250, 1.001, 1.125);
%Pos 3 is Pos h with Z adjustment
%pr2RightPos3 = transl(1.071, -0.180, 1.125) * troty(-pi/2);
%pr2leftPos3 = transl(1.071, 0.180, 1.125) * troty(-pi/2);

% Call functions from robotControl to move pr2 
% Move arm -> base up -> move arm -> base down
robot.animatePR2ArmsAndGrippers(pr2RightPosH, pr2RightPos1, pr2leftPosH, pr2leftPos1, numSteps, eStop);
%robot.PR2BothGrippersOpen(numSteps, eStop);
robot.animatePR2Base(startTr, endTr, numSteps, eStop); %Move torso up

%Scan for the banana
%[firstCoord, lastCoord] = laser.laser_scan(sensor_position, laser_rotation, radii, centerPoint);

%fprintf('First Coordinate: (%.2f, %.2f, %.2f)\n', firstCoord(1), firstCoord(2), firstCoord(3));
%fprintf('Last Coordinate: (%.2f, %.2f, %.2f)\n', lastCoord(1), lastCoord(2), lastCoord(3));
%Create T-matrices for the banana start and end
%bananaR = transl(firstCoord);
%bananaL = transl(lastCoord) * transl(-0.2, 0, 0) * troty(pi/2);
%Animating moves the left arm to the left end of the banana but in an undesirable position, use waypoint animations
%{
EXAMPLE FROM TEST

rightStart = transl(0.821, -0.440, 1);
qRightStart = pr2.env.pr2Right.model.ikcon(rightStart);
rightWpOne = deg2rad([89.4 90 0 0 0 0 0]);
rightWpTwo = deg2rad([90 67.2 0 15 0 0 0]);
rightEnd = transl(0.172, -0.793, 0.269);
qRightEnd = pr2.env.pr2Right.model.ikcon(rightEnd);

qWpMat = [qRightStart; rightWpTwo; rightWpTwo; qRightEnd];

pr2.animateRightPR2ArmsAndGrippersWithWaypoints(qWpMat, numSteps, eStop);
%}
pr2LeftWayPosStart = env.pr2LeftArm.model.ikcon(pr2leftPos1);
%W1 
q1 = deg2rad([-90 125 0 0 0 0 0]);
%W2
q2 = deg2rad([-90 125 -180 0 0 0 0]);
%W3
q3 = deg2rad([-90 128 -180 0 180 0 0]);
%W4
q4 = env.pr2LeftArm.model.ikcon(transl(0.882, 0.406, 1.146));
q5 = deg2rad([15.5 129 -168 83.3 -180 48 0]);
q6 = deg2rad([15.5 101 -168 80.9 -180 48 -20]);
qWpmat = [pr2LeftWayPosStart; q1; q2; q3; q4; q5; q6];
%robot.animatePR2LeftArmsAndGrippersWithWaypoints(qWpmat, 100, eStop);
robot.animatePR2RightArmsAndGrippers(pr2RightPos1h, pr2RightPos2, numSteps, eStop);
robot.PR2GrabKnife(numSteps, eStop);
deletePlyObject(fakeKnife);
robot.animateRightPR2ArmsAndGrippersWithKnife(pr2RightPos2, pr2RightPos1h, numSteps, eStop);
%robot.animatePR2SingleRightJoint(1, -10, 30);
%robot.animatePR2SingleRightJoint(6, -45, 30);



function deletePlyObject(objectHandle)

    if ishandle(objectHandle)
        delete(objectHandle);
    else
        warning('The provided handle is not valid or the object does not exist.');
    end
end