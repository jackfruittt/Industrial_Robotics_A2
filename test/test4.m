clc;
clf;

% Load your TM5900 model
robot = TM5900.TM5900(); % Assuming default parameters

% Define initial configuration
q = zeros(1, 6); % Assuming a 6 DOF robot

% Define obstacles in the environment
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2]; % Set the size of the workspace when drawing the robot
robot.model.plot(q, 'workspace', workspace, 'scale', scale); % Changed from robot.model.plot to robot.plot
centerpnt = [1.5, 0, -0.5];
side = 1.5;
plotOptions.plotFaces = true;
[vertex, faces, faceNormals] = RectangularPrism(centerpnt - side / 2, centerpnt + side / 2, plotOptions); % Assuming CreateRectangularPrism replaces RectangularPrism
axis equal
camlight;

% Initialize transformation matrices
tr = zeros(4, 4, robot.model.n + 1);
tr(:, :, 1) = robot.model.base

% Retrieve the links from the robot model
L = robot.model.links;

% Calculate the transformation matrices for each link
for i = 1:robot.model.n
    tr(:, :, i + 1) = tr(:, :, i) * trotz(q(i) + L(i).offset) * transl(0, 0, L(i).d) * transl(L(i).a, 0, 0) * trotx(L(i).alpha);
end

% Loop through each link and check for collision with each face
for i = 1:size(tr, 3) - 1
    for faceIndex = 1:size(faces, 1)
        vertOnPlane = vertex(faces(faceIndex, 1), :); % Corrected indexing
        [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i + 1)');
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex, :), :))
            plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
            disp('Intersection');
        end
    end
end

% Define a trajectory between two points
q1 = [-pi/4, pi/2, 0, 0, 0, 0];
q2 = [pi/4, pi/2, 0, 0, 0, 0];
steps = 2;

% Determine the number of steps needed for a smooth trajectory
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1, q2, steps)))), 1))
    steps = steps + 1;
end

% Generate the joint trajectory
qMatrix = jtraj(q1, q2, steps);

% Collision detection along the trajectory
result = true(steps, 1);
for i = 1:steps
    result(i) = IsCollision(robot.model, qMatrix(i, :), faces, vertex, faceNormals, false);
    robot.model.animate(qMatrix(i, :)); % Changed from robot.model.animate to robot.animate
end


function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;
for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot - seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)
    links = robot.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.base;
    for i = 1:length(links)
    L = links(1,i);
    current_transform = transforms(:,:, i);
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
    end
end

function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
    % Given a point which is known to be on the same plane as the triangle
    % determine if the point is inside (result == 1) or outside a triangle (result == 0)

    u = triangleVerts(2, :) - triangleVerts(1, :);
    v = triangleVerts(3, :) - triangleVerts(1, :);

    uu = dot(u, u);
    uv = dot(u, v);
    vv = dot(v, v);

    w = intersectP - triangleVerts(1, :);
    wu = dot(w, u);
    wv = dot(w, v);

    D = uv * uv - uu * vv;

    % Get and test parametric coordinates (s and t)
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)        % intersectP is outside the Triangle
        result = 0;
        return;
    end

    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside the Triangle
        result = 0;
        return;
    end

    result = 1;  % intersectP is inside the Triangle
end
