clc;
clf;

grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);  
% Load environment
env = EnvironmentLoader();
pr2 = pr2Control(env);
laser = pr2Laser();
gripperLeftState = 'closed';
gripperRightState = 'closed';

sensor_position = [0.125, 0, 0.9]; 
centerPoint = [0.85, 0, 0.42]; 
radii = [0.1, 0.2, 0.08]; 
laser_rotation = deg2rad(57);

numSteps = 30; 
homePosr = transl(0.821, -0.440, 1);
homePosl = transl(0.821, 0, 1);

Tbr = transl(0.594, -0.863, 0.647);
Tbl = transl(0.594, 0.423, 0.647);

Tb2r = transl(0.431, -0.572, 1.256);
Tb2l = transl(0.252, 0.606, 0.894);

Tb3r = transl(0, -0.937, 1.071);
Tb3l = transl(0, 0.490, 1.083);

Tb4r = transl(0.116, -0.669, 0.334);
Tb4l = transl(0.99, 0.294, 0.286);

%% TO USE PR2 ANIMATION FUNCTIONS
% pr2.animateRightPR2ArmsAndGrippers(RightStartPos, RightEndPos, numSteps);
% pr2.animateLeftPR2ArmsAndGrippers(LeftStartPos, LeftEndPos, numSteps);
% pr2.animatePR2ArmsAndGrippers(RightStartPos, RightEndPos, LeftStartPos, LeftEndPos, numSteps);
% pr2.bothGripperClose(numSteps);
% pr2.LeftGripperOpen(numSteps);
% pr2.LeftGripperClose(numSteps);
% pr2.RightGripperClose(numSteps);
% pr2.RightGripperOpen(numSteps);

%pr2.bothGripperClose(50);
pr2.animatePR2ArmsAndGrippers(homePosr, Tb2r, homePosl, Tb3l, numSteps);

%Scan for the banana
[firstCoord, lastCoord] = laser.laser_scan(sensor_position, laser_rotation, radii, centerPoint);

fprintf('First Coordinate: (%.2f, %.2f, %.2f)\n', firstCoord(1), firstCoord(2), firstCoord(3));
fprintf('Last Coordinate: (%.2f, %.2f, %.2f)\n', lastCoord(1), lastCoord(2), lastCoord(3));
%Create T-matrices for the banana start and end
bananaR = transl(firstCoord);
bananaL = transl(lastCoord);
%pr2.animatePR2ArmsAndGripperswithMotionControl(Tb3r, bananaR, Tb3l, bananaL, numSteps);
%pr2.animatePR2ArmsAndGripperswithMotionControl(bananaR, Tb3r, bananaL, Tb3l, numSteps);
pr2.LeftGripperOpen(50);
pr2.LeftGripperClose(50);
pr2.RightGripperOpen(50);
pr2.RightGripperWithKnifeClose(50);
pr2.animateRightPR2ArmsAndGrippersWithKnife(Tb2r, Tb3r, numSteps);
%pr2.RightGripperClose(50);
%pr2.bothGripperOpen(50);
%pr2.bothGripperClose(50);
pr2.LeftGripperOpen(50);
pr2.animatePR2ArmsAndGrippers(Tb3r, Tbr, Tb3l, Tbl, numSteps);
pr2.animatePR2ArmsAndGrippers(Tbr, Tb2r, Tbl, Tb2l, numSteps);
pr2.animateRightPR2ArmsAndGrippers(Tb2r, Tb3r, numSteps);
pr2.RightGripperOpen(50);
pr2.animateLeftPR2ArmsAndGrippers(Tb2l, Tb3l, numSteps);

%Move to banana
pr2.animatePR2ArmsAndGrippers(Tb3r, bananaR, Tb3l, bananaL, numSteps);
pr2.bothGripperClose(50);




        


