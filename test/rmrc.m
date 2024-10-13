clc;
clf;
% Parameters
% Define start and end transformation matrices
pr2Left = PR2.PR2LeftArm();
T1 = [eye(3), [0.504, -0.180, 0.483]'; zeros(1, 3), 1];  % First pose
T2 = [eye(3), [0.504, -0.180, 0.183]'; zeros(1, 3), 1];  % Second pose

% Parameters
dt = 0.05;                    % Time step
steps = 50;                   % Number of steps
lambda = 0.01;                  % Damping factor for singularity handling
epsilon = 0.0001;                % Threshold to detect singularities

% Initialize joint angles from inverse kinematics for the start pose
qStart = pr2Left.model.ikine(T1);  % Inverse kinematics for T1
qMatrix = zeros(steps, 7);         % Preallocate matrix for joint angles
qMatrix(1, :) = qStart;

% Trajectory between start and end pose
trajectory = ctraj(T1, T2, steps); % Cartesian trajectory from T1 to T2

% RMRC loop
for i = 1:steps-1
    % Current joint angles
    q = qMatrix(i, :);
    
    % Forward kinematics to get the current pose
    T = pr2Left.model.fkine(q);
    
    % Calculate the Cartesian velocity
    v = tr2delta(T, trajectory(:,:,i+1)) / dt;
    %v = tr2delta(T, trajectory(:,:,i+1)) / dt; % 6x1 velocity vector
    
    % Compute the Jacobian at the current joint configuration
    J = pr2Left.model.jacob0(q);
    
    % Damped Least Squares (DLS) to handle singularities
    if abs(det(J*J')) < epsilon
        qdot = (J' / (J*J' + lambda^2 * eye(6))) * v;  % DLS inverse
    else
        qdot = J \ v;  % Regular inverse
    end
    
    % Update the joint angles using Euler integration
    qMatrix(i+1, :) = q + qdot' * dt;
end

% Animate the robot motion
for i = 1:steps
    pr2Left.model.animate(qMatrix(i, :));
    pos = pr2Left.model.fkine(qMatrix2(i, :)).T
    pause(0.05);
end

T3 = [eye(3), [0.604, -0.180, 0.183]'; zeros(1, 3), 1]; 
% Initialize joint angles from inverse kinematics for the start pose
qStart1 = pr2Left.model.ikine(T2);  % Inverse kinematics for T1
qMatrix1 = zeros(steps, 7);         % Preallocate matrix for joint angles
qMatrix1(1, :) = qStart1;

% Trajectory between start and end pose
trajectory1 = ctraj(T2, T3, steps); % Cartesian trajectory from T1 to T2

% RMRC loop
for i = 1:steps-1
    % Current joint angles
    q = qMatrix1(i, :);
    
    % Forward kinematics to get the current pose
    T = pr2Left.model.fkine(q);
    
    % Calculate the Cartesian velocity
    v = tr2delta(T, trajectory1(:,:,i+1)) / dt;
    %v = tr2delta(T, trajectory(:,:,i+1)) / dt; % 6x1 velocity vector
    
    % Compute the Jacobian at the current joint configuration
    J = pr2Left.model.jacob0(q);
    
    % Damped Least Squares (DLS) to handle singularities
    if abs(det(J*J')) < epsilon
        qdot = (J' / (J*J' + lambda^2 * eye(6))) * v;  % DLS inverse
    else
        qdot = J \ v;  % Regular inverse
    end
    
    % Update the joint angles using Euler integration
    qMatrix1(i+1, :) = q + qdot' * dt;
end

% Animate the robot motion
for i = 1:steps
    pr2Left.model.animate(qMatrix1(i, :));
    pos = pr2Left.model.fkine(qMatrix2(i, :)).T
    pause(0.05);
end

qStart2 = pr2Left.model.ikine(T3);  % Inverse kinematics for T1
qMatrix2 = zeros(steps, 7);         % Preallocate matrix for joint angles
qMatrix2(1, :) = qStart2;

trajectory2 = ctraj(T3, T1, steps); % Cartesian trajectory from T1 to T2

% RMRC loop
for i = 1:steps-1
    % Current joint angles
    q = qMatrix2(i, :);
    
    % Forward kinematics to get the current pose
    T = pr2Left.model.fkine(q);
    
    % Calculate the Cartesian velocity
    v = tr2delta(T, trajectory2(:,:,i+1)) / dt;
    %v = tr2delta(T, trajectory(:,:,i+1)) / dt; % 6x1 velocity vector
    
    % Compute the Jacobian at the current joint configuration
    J = pr2Left.model.jacob0(q);
    
    % Damped Least Squares (DLS) to handle singularities
    if abs(det(J*J')) < epsilon
        qdot = (J' / (J*J' + lambda^2 * eye(6))) * v;  % DLS inverse
    else
        qdot = J \ v;  % Regular inverse
    end
    
    % Update the joint angles using Euler integration
    qMatrix2(i+1, :) = q + qdot' * dt;
end

% Animate the robot motion
for i = 1:steps
    pr2Left.model.animate(qMatrix2(i, :));
    pos = pr2Left.model.fkine(qMatrix2(i, :)).T
    pause(0.05);
end


