clc;
clf;

grid on;
hold on;
axis([-4.5 4.5 -3.5 3.5 0 3.5]);

view(90, 10);  
% Load environment
eStop = serial('COM3', 'BaudRate', 9600);  
%fopen(eStop);
env = EnvironmentLoader();
knife = PlaceObject('plyFiles/Scenery/knife.ply', [0.7, -0.75, 0.75]);
knifeHand = PlaceObject('plyFiles/Scenery/knife_in_hand.ply', [0.7, -0.75, 0.75]);
knifeVertices = get(knifeHand, 'Vertices');
robot = robotControl(env);
laser = pr2Laser();
gripperLeftState = 'closed';
gripperRightState = 'closed';
robot = robotControl(env);
eStop = serial('COM3', 'BaudRate', 9600);
view(30, 20);  

sensor_position = [0.125, 0, 0.9]; 
centerPoint = [0.85, 0, 0.42]; 
xLength = centerPoint(1) - sensor_position(1);
zLength = sensor_position(3) - centerPoint(3);
radii = [0.1, 0.2, 0.08]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);
% Animate arm + grippers
homePosr = transl(0.821, -0.440, 1);
Tbr = transl(0.594,0.423, 0.647);
homePosl = transl(0.821, 0, 1);
Tbl = transl(0.594,-0.863, 0.647);

numSteps = 30; 





startPos = transl(2, 0.764, 1.092);

endPos = transl(1.885, -0.555, 0.207);

knifePos = transl(0.7, -0.75, 0.75) * troty(pi/2);

robot.animatePR2RightArmsAndGrippers(homePosr, Tb2r, numSteps, eStop);
robot.PR2RightGripperOpen(numSteps, eStop);
robot.animatePR2RightArmsAndGrippers(Tb2r, knifePos, numSteps, eStop);
robot.PR2GrabKnife(numSteps, eStop);

offset = troty(-pi/2) * transl(0.05, 0, 0);
knifeOffset = transl(1.85, 0.85, 0.7) * troty(-pi/2);
deletePlyObject(knife);
function movePR2WithObject(env, knifeHand, knifeVertices, startPos, endPos, numSteps, offset, knifeOffset)

    % Calculate the joint trajectory
    qMatrix = jtraj(env.pr2Right.model.ikcon(startPos), env.pr2Right.model.ikcon(endPos), numSteps);

    % Loop through each step in the trajectory
    for i = 1:numSteps
        % Get the current end effector pose for the right arm
        T_rightEndEffector = env.pr2Right.model.fkine(qMatrix(i, :)).T;
Tb3r = transl(0.594, 0.423, 0.947);
Tb4r = transl(0.821, -0.440, 1.3);

        % Set grippers based on the end effector pose with offset
        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;
Tb3l = transl(0.594,-0.863, 0.947);
Tb4l = transl(0.821, 0, 1.3);

        % Homogenize the knife vertices (convert to 4x1 vector)
        knifeVerticesHomogeneous = [knifeVertices, ones(size(knifeVertices, 1), 1)];
% Torso Go Up
qStart = [0 0 0];
startTr = robot.env.pr2Base.model.fkine(qStart);

        % Apply the end-effector transformation to the knife vertices
        transformedVertices = (knifeOffset * T_rightEndEffector * knifeVerticesHomogeneous')';
qEnd = [-0.3 0 0];
endTr = robot.env.pr2Base.model.fkine(qEnd);

        % Remove the homogenizing column and update the knife object
        set(knifeHand, 'Vertices', transformedVertices(:, 1:3));
% Torso Go Down
qStart1 = [-0.3 0 0];
qEnd1 = [0 0 0];

        % Animate the PR2 right arm
        env.pr2Right.model.animate(qMatrix(i, :));
startTr1 = robot.env.pr2Base.model.fkine(qStart1);
endTr1 = robot.env.pr2Base.model.fkine(qEnd1);

% Call functions from robotControl to move pr2 
% Move arm -> base up -> move arm -> base down
robot.animatePR2ArmsAndGrippers(homePosr, Tbr, homePosl, Tbl, numSteps, eStop);
robot.animatePR2Base(startTr, endTr, numSteps, eStop);
robot.animatePR2ArmsAndGrippers(Tb3r, Tb4r, Tb3l, Tb4l, numSteps, eStop);
robot.animatePR2Base(startTr1, endTr1, numSteps, eStop);

hold off;

        % Render the updated scene
        drawnow();
    end
end

movePR2WithObject(env, knifeHand, knifeVertices, knifePos, Tb3r, numSteps, offset, knifeOffset);

movePR2WithObject(env, knifeHand, knifeVertices, Tb3r, Tb4r, numSteps, offset, knifeOffset);

function deletePlyObject(objectHandle)

    if ishandle(objectHandle)
        delete(objectHandle);
    else
        warning('The provided handle is not valid or the object does not exist.');
    end
end

