clc;
clf;
set(0, 'DefaultFigureWindowStyle', 'docked');

% 3.1
steps = 50;
leftArm = PR2.PR2LeftArm();

% 3.2
T1 = [eye(3), [0.504, -0.180, 0.483]'; zeros(1, 3), 1];  % First pose
T2 = [eye(3), [0.504, -0.180, 0.183]'; zeros(1, 3), 1];  % Second pose

% 3.3
M = [1 1 1 1 1 1];  % Masking Matrix for full pose (translation + orientation)

% Initial joint angle guess (7 DOF)
q0 = zeros(1, 7);  % 7-element initial guess

% 3.4: Inverse Kinematics (solve for joint angles)
q1 = leftArm.model.ikine(T1, 'q0', q0, 'mask', M);  
q2 = leftArm.model.ikine(T2, 'q0', q0, 'mask', M);  

leftArm.model.plot(q1, 'trail', 'r-');
pause(3)

% 3.5: Trajectory between the two poses
qMatrix = jtraj(q1, q2, steps);
leftArm.model.plot(qMatrix, 'trail', 'r-');

% 3.6: Resolved Motion Rate Control (RMRC)
deltaT = 0.05;  % Discrete time step

% 3.7: Interpolate the pose using trinterp for rotations and translations
x = zeros(6, steps);  % Store the translation and orientation in vector form
s = lspb(0, 1, steps);  % Interpolation scalar
for i = 1:steps
    % Interpolate between the two transformations T1 and T2
    T_interpolated = trinterp(T1, T2, s(i));
    
    % Extract the translational and rotational components
    x(1:3, i) = T_interpolated(1:3, 4);  % Translation (x, y, z)
    R = T_interpolated(1:3, 1:3);  % Rotation matrix
    rpy = tr2rpy(R);  % Convert rotation matrix to roll-pitch-yaw (rpy) angles
    x(4:6, i) = rpy';  % Store the orientation (rpy)
end

% 3.8: Initialize qMatrix
qMatrix = nan(steps, 7);

% 3.9: Set initial joint configuration
qMatrix(1, :) = leftArm.model.ikine(T1, 'q0', q0, 'mask', M);

% 3.10: Perform RMRC to update joint angles
for i = 1:steps - 1
    xdot = (x(:, i + 1) - x(:, i)) / deltaT;  % Velocity at this step
    J = leftArm.model.jacob0(qMatrix(i, :));  % Jacobian at the current configuration
    J = J(1:6, :);  % Use the full 6 rows for translation and rotation (x, y, z, roll, pitch, yaw)
    
    % Solve for joint velocities using RMRC
    qdot = pinv(J) * xdot;  % Pseudoinverse for stability
    
    % Update joint angles
    qMatrix(i + 1, :) = qMatrix(i, :) + deltaT * qdot';
end

% 3.11: Plot the updated trajectory
leftArm.model.plot(qMatrix, 'trail', 'r-');

