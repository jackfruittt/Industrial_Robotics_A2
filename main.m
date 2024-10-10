clc;
clf;

grid on;
hold on;
axis([-4 4 -2 2 0 2]);

% Load environment
env = EnvironmentLoader();
robot = robotControl(env);
eStop = serial('COM3', 'BaudRate', 9600);
view(30, 20);  

numSteps = 100; 
% Animate arm + grippers
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

hold off;




                 


