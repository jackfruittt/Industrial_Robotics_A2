clf;
clc;
%{
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
qStart = pr2Left.model.ikcon(T1);  % Inverse kinematics for T1
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
qStart1 = pr2Left.model.ikcon(T2);  % Inverse kinematics for T1
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

qStart2 = pr2Left.model.ikcon(T3);  % Inverse kinematics for T1
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
%}

%RMRC as a function

% Parameters
hold on;
grid on;
axis([-1 1 -1 1 -1 1]);
pr2Left = PR2.PR2LeftArm();   % Load PR2 Left Arm Robot
pr2Left.model.teach();

dt = 0.05;                    % Time step
steps = 50;                   % Number of steps
lambda = 0.01;                % Damping factor for singularity handling
epsilon = 0.00000001;             % Threshold for detecting singularities

theta = deg2rad(-60);  

% rotation matrix 
Ry = [cos(theta), 0, sin(theta); 
            0,         1, 0; 
            -sin(theta), 0, cos(theta)];

% Define T1 and T2 positions (without rotation)
T1Pos = [eye(3), [0.241, -0.180, -0.198]'; zeros(1, 3), 1];  % Initial position
T2Pos = [eye(3), [0.241, -0.180, -0.427]'; zeros(1, 3), 1];  % Final position

% Apply the 65-degree rotation about the local Y-axis to the orientation part
T1Rot = T1Pos;  % Copy T1
T1Rot(1:3, 1:3) = Ry * T1Pos(1:3, 1:3);  % Apply the rotation to T1

T2Rot = T2Pos;  % Copy T2
T2Rot(1:3, 1:3) = Ry * T2Pos(1:3, 1:3);  % Apply the rotation to T2

% Move from T1 to T2
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T2Rot, T1Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T2Rot, T1Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T2Rot, T1Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T2Rot, T1Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T2Rot, T1Rot, steps, dt, lambda, epsilon);
movePR2Arm(pr2Left, T1Rot, T2Rot, steps, dt, lambda, epsilon);

% Move from T2 to T3
%movePR2Arm(pr2Left, T2, T3, steps, dt, lambda, epsilon);

% Move from T3 back to T1
%movePR2Arm(pr2Left, T3, T1, steps, dt, lambda, epsilon);

function movePR2Arm(pr2Left, Tstart, Tend, steps, dt, lambda, epsilon)
    % movePR2Arm - Function to move PR2 Left Arm between two poses using RMRC
    %
    % Syntax: movePR2Arm(pr2Left, Tstart, Tend, steps, dt, lambda, epsilon)
    %
    % Inputs:
    %   pr2Left - PR2 Left Arm robot model
    %   Tstart  - Starting transformation matrix (4x4)
    %   Tend    - Ending transformation matrix (4x4)
    %   steps   - Number of steps for the motion
    %   dt      - Time step for RMRC
    %   lambda  - Damping factor for singularity handling
    %   epsilon - Threshold for detecting singularities

    % Initialize joint angles from inverse kinematics for the start pose
    qStart = pr2Left.model.ikcon(Tstart);  % Inverse kinematics for Tstart
    qMatrix = zeros(steps, 7);             % Preallocate matrix for joint angles
    qMatrix(1, :) = qStart;

    % Trajectory between start and end pose
    trajectory = ctraj(Tstart, Tend, steps); % Cartesian trajectory from Tstart to Tend

    % RMRC loop
    for i = 1:steps-1
        % Current joint angles
        q = qMatrix(i, :);

        % Forward kinematics to get the current pose
        T = pr2Left.model.fkine(q).T;

        % Calculate the Cartesian velocity
        v = tr2delta(T, trajectory(:,:,i+1)) / dt;

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
        drawnow();
    end
end
