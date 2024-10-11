clc;
clf;

grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);  
% Load environment
eStop = serial('COM3', 'BaudRate', 9600);  
env = EnvironmentLoader();
robot = robotControl(env);
laser = pr2Laser();
gripperLeftState = 'closed';
gripperRightState = 'closed';
robot = robotControl(env);
eStop = serial('COM3', 'BaudRate', 9600);
view(90, 10);  

sensor_position = [0.125, 0, 0.9]; 
centerPoint = [0.85, 0, 0.42]; 
xLength = centerPoint(1) - sensor_position(1);
zLength = sensor_position(3) - centerPoint(3);
radii = [0.1, 0.2, 0.08]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);

startPos = transl(2, 0.764, 1.092);

endPos = transl(1.885, -0.555, 0.207);

numSteps = 30;
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594,0.423, 0.647);
homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594,-0.863, 0.647);

Tb3r = transl(0.594, 0.423, 0.947);
Tb4r = transl(0.821, -0.440, 1.3);

Tb3l = transl(0.594,-0.863, 0.947);
Tb4l = transl(0.821, 0, 1.3);

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

% Call functions from robotControl to move pr2 
% Move arm -> base up -> move arm -> base down
robot.animatePR2ArmsAndGrippers(homePosr, Tbr, homePosl, Tbl, numSteps, eStop);
robot.animatePR2Base(startTr, endTr, numSteps, eStop);
robot.animatePR2ArmsAndGrippers(Tb3r, Tb4r, Tb3l, Tb4l, numSteps, eStop);
robot.animatePR2Base(startTr1, endTr1, numSteps, eStop);
robot.animateRightPR2ArmsAndGrippersWithKnife(Tb4r, Tbr, numSteps, eStop);


function deletePlyObject(objectHandle)

    if ishandle(objectHandle)
        delete(objectHandle);
    else
        warning('The provided handle is not valid or the object does not exist.');
    end
end