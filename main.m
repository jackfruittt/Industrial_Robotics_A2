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

sensor_position = [0.125, 0, 0.9]; 
centerPoint = [0.85, 0, 0.42]; 
xLength = centerPoint(1) - sensor_position(1);
zLength = sensor_position(3) - centerPoint(3);
radii = [0.1, 0.2, 0.08]; 
laser_rotation = atan(xLength/zLength);
disp(laser_rotation);

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

        % Set grippers based on the end effector pose with offset
        env.gripperl2.model.base = T_rightEndEffector * offset;
        env.gripperr2.model.base = T_rightEndEffector * offset;

        % Homogenize the knife vertices (convert to 4x1 vector)
        knifeVerticesHomogeneous = [knifeVertices, ones(size(knifeVertices, 1), 1)];

        % Apply the end-effector transformation to the knife vertices
        transformedVertices = (knifeOffset * T_rightEndEffector * knifeVerticesHomogeneous')';

        % Remove the homogenizing column and update the knife object
        set(knifeHand, 'Vertices', transformedVertices(:, 1:3));

        % Animate the PR2 right arm
        env.pr2Right.model.animate(qMatrix(i, :));

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

