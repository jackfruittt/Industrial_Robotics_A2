% Author: Jackson Russelll 14250803
% Main file of Assignment 1 which is used to run all code necessary to
% complete assignment objectives. 

clf;
clc;
grid on;
axis([-4 4 -2 2 0 2]);
hold on;
profile on;
s = profile('status')

%Load class to plot environment
env = EnvironmentLoader(); 
steps = 100;
%Load classes for robot animation
robotControl = moveRobot(env, steps);
gripperControl = moveGripper(env);

brickLength = 0.15;  
brickWidth = 0.18;  
tableHeight = 0.65;
startX = -1;   
%startY = -0.35;
startY = 0.35; % alternative position
z = tableHeight;
numBricks = 9;

%Define the positions of the Bricks in a 1x9 row along the x-axis
Tstart = cell(1, numBricks);

for i = 1:numBricks
    
    xStart = startX + (i - 1) * brickLength; 
    yStart = startY;  %Keep the Y-coordinate constant for a row arrangement
    Tstart{i} = transl(xStart, yStart, z); %Create transformation matrix

end

%Display all transformation matrices T1 to T9 for the bricks starting positions
for i = 1:numBricks
    
    Tn = Tstart{i};
    sprintf('Transformation matrix Tstart%d:\n', i);
    
    %Display ordinates
    x = Tn(1, 4);
    y = Tn(2, 4);
    z = Tn(3, 4);
    fprintf('Ordinates of Tstart%d: x = %.2f, y = %.2f, z = %.2f\n', i, x, y, z);

end

%Define the end position of each brick as a 3x3 matrix 
endX = -01;   
%endY = 0.45;
endY = -0.45; %Alternative end position
endZ = 0.7;

brickEndLength = 0.25;
brickEndWidth = 0.35;
brickHeight = 0.04;

Tend = cell(3, 3);

for i = 1:3
    for j = 1:3  
        xEnd = endX + (i - 1) * brickLength; 
        zEnd = endZ + (j - 1) * brickHeight;  
        Tend{i, j} = transl(xEnd, endY, zEnd);
    end
end

% Display all transformation matrices T1 to T9
n = 1;
for j = 1:3 
    for i = 1:3 
        % Define Tn
        Tn = Tend{i, j};

        sprintf('Transformation matrix Tend%d:\n', i, j, n);
        xCoord = Tn(1, 4);
        yCoord = Tn(2, 4);
        zCoord = Tn(3, 4);
        fprintf('Ordinates of Tend%d: x = %.2f, y = %.2f, z = %.2f\n', n, x, y, z);
        n = n + 1; 
    
    end
end

%Start and End postions for closing the gripper (Left and Right)
qLeftCloseS = pi/4;   
qLeftCloseE = deg2rad(38);  
qRightCloseS = -pi/4;  
qRightCloseE = deg2rad(-38); 

%Start and End positions for opening the gripper (Left and Right)
qLeftOpenS = deg2rad(38);   
qLeftOpenE = pi/4;  
qRightOpenS = deg2rad(-38);  
qRightOpenE = -pi/4; 

%Gripper open default states
qGripperRightOpened = [pi/4 -pi/4];  
qGripperLeftOpened = [-pi/4 pi/4]; 
qGripperRightClosed = [deg2rad(40) -pi/4]; 
qGripperLeftClosed = [deg2rad(-40) pi/4];   

%Number of steps for the robot animations 
numSteps = 100; 

for i = 1:numBricks
    
    %Home position to minimise collisions
    %Howwever, after doing half, it needs a new pos to continue minimising collisions
    if i > 4
        homePos = transl(-0.7, 0, 1.411) 
    else
        homePos = transl(0, 0, 1.20);
    end
    
    % Access the corresponding Tend matrix
    TendIdx = mod(i-1, 3) + 1; %inputs1-3(0-2) inputs4-6(0-2) inputs7-9(0-2)
    TendJdx = floor((i-1)/3) + 1; %inputs1-3(0) inputs4-6(1) inputs7-9(2)
    TendMatrix = Tend{TendIdx, TendJdx};

    Tb = Tstart{i} * rpy2tr(180, 0, 90, 'deg'); %Use rpy2tr() to rotate about its local axis, alternatively use troty(pi) * trotx(pi/2) for a global approach
    robotControl.manipulateRobot(homePos, Tb, numSteps, false, qLeftCloseS, qLeftCloseE, qRightCloseS, qRightCloseE, qGripperRightOpened, qGripperLeftOpened);
    disp('Wanted position for Brick Tb');
    disp(Tb);

    Tw = TendMatrix * rpy2tr(0, 180, 90, 'deg');
    robotControl.manipulateRobot(Tb, Tw, numSteps, false, qLeftOpenS, qLeftOpenE, qRightOpenS, qRightOpenE, qGripperRightClosed, qGripperLeftClosed);
    disp('Wanted position for Brick Tw');
    disp(Tw);
    
    %Extaract the ordinates of Tw 
    x = Tw(1, 4);
    y = Tw(2, 4);
    z = Tw(3, 4);
    PlaceObject('plyFiles/HalfSizedRedGreenBrick.ply', [x, y, z - 0.2]); %plot a brick at Tw
    
    robotControl.manipulateRobot(Tw, homePos, numSteps, false, qLeftOpenS, qLeftOpenE, qRightOpenS, qRightOpenE, qGripperRightOpened, qGripperLeftOpened);
    disp('Wanted position for Brick homePos');
    disp(homePos);

end

profile viewer